//! `tpts ccsds <hex|file> [--crc]` — decode a CCSDS Space Packet stream for
//! ground-ops debugging: APID, sequence count, flags, length, and the
//! optional CRC-16 suffix.

use std::path::Path;
use std::process::ExitCode;

use tpt_t_space_ccsds::defs;
use tpt_t_space_ccsds::packet::PacketView;
use tpt_t_space_ccsds::tc::crc16;

use crate::{CliResult, has_flag, positional};

pub fn run(args: &[String]) -> CliResult {
    let input = positional(args, &[]).ok_or("ccsds needs a hex string or a binary file path")?;
    let bytes = load(input)?;
    let with_crc = has_flag(args, "--crc");

    let mut off = 0usize;
    let mut count = 0usize;
    let mut crc_bad = 0usize;
    while off < bytes.len() {
        let rest = &bytes[off..];
        let view = match PacketView::parse(rest) {
            Ok(v) => v,
            Err(e) => {
                println!("offset {off:#06x}: {e} — stream stops here");
                return Ok(ExitCode::FAILURE);
            }
        };
        let len = view.total_len();
        let crc = if with_crc {
            if len < 2 {
                crc_bad += 1;
                "crc missing".to_owned()
            } else {
                let expect = u16::from_be_bytes([rest[len - 2], rest[len - 1]]);
                if crc16(&rest[..len - 2]) == expect {
                    "crc ok".to_owned()
                } else {
                    crc_bad += 1;
                    "crc BAD".to_owned()
                }
            }
        } else {
            String::new()
        };
        println!(
            "#{count:<3} off={off:#06x} apid={:#05x} seq={:<5} {:?} {:?} len={len} {crc}",
            view.apid, view.sequence_count, view.packet_type, view.sequence_flags,
        );
        if let Some(def) = defs::lookup(view.apid) {
            let shown = defs::decode_each(def, view.payload(), |f| {
                println!("      {} = {:.3} {}", f.name, f.value, f.unit);
            });
            if let Err(e) = shown {
                println!("      {e}");
            }
        }
        off += len;
        count += 1;
    }
    println!("\n{count} packet(s), {} byte(s)", bytes.len());
    if crc_bad > 0 {
        println!("{crc_bad} packet(s) failed the CRC-16 check");
        return Ok(ExitCode::FAILURE);
    }
    Ok(ExitCode::SUCCESS)
}

/// A file path if the argument names an existing file, else hex digits
/// (whitespace, `:` and `0x` prefixes ignored).
pub fn load(input: &str) -> Result<Vec<u8>, String> {
    if Path::new(input).is_file() {
        return std::fs::read(input).map_err(|e| format!("{input}: {e}"));
    }
    let cleaned: String = input
        .replace("0x", "")
        .chars()
        .filter(|c| !c.is_whitespace() && *c != ':')
        .collect();
    if cleaned.len() % 2 != 0 {
        return Err("hex input must have an even number of digits".to_owned());
    }
    (0..cleaned.len())
        .step_by(2)
        .map(|i| {
            u8::from_str_radix(&cleaned[i..i + 2], 16)
                .map_err(|_| format!("`{input}` is neither a file nor hex"))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::load;

    #[test]
    fn hex_input_ignores_spacing_colons_and_prefix() {
        assert_eq!(load("0x01 23:AB").unwrap(), vec![0x01, 0x23, 0xAB]);
    }

    #[test]
    fn odd_hex_is_rejected() {
        assert!(load("abc").is_err());
    }

    #[test]
    fn non_hex_non_file_is_rejected() {
        assert!(load("zz").is_err());
    }
}
