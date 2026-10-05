//! Table-free CRC32 (IEEE 802.3, polynomial `0xEDB88320`) — the integrity
//! primitive shared across the workspace: the radiation memory scrubber's
//! region walk, DTN bundle payloads, and the persistent snapshot store all
//! stamp data with this one CRC so a single implementation is certified once
//! (spec §2 traceability).
//!
//! The lookup table is generated at compile time (`const fn`), so the
//! hot-path routine is a single table load + xor per byte with zero
//! allocation and no runtime initialization.

const fn make_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 {
                0xEDB8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
}

static TABLE: [u32; 256] = make_table();

/// CRC32 (IEEE) of `data`, seeded with `crc` so callers can checksum
/// contiguous regions incrementally (seed `0` for a fresh checksum).
#[must_use]
pub fn crc32(crc: u32, data: &[u8]) -> u32 {
    let mut c = crc ^ 0xFFFF_FFFF;
    for &b in data {
        c = TABLE[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    c ^ 0xFFFF_FFFF
}

/// One-shot CRC32 of a byte slice.
#[must_use]
pub fn crc32_one_shot(data: &[u8]) -> u32 {
    crc32(0, data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_vectors() {
        assert_eq!(crc32_one_shot(b""), 0x0000_0000);
        assert_eq!(crc32_one_shot(b"123456789"), 0xCBF4_3926);
        assert_eq!(
            crc32_one_shot(b"The quick brown fox jumps over the lazy dog"),
            0x414F_A339
        );
    }

    #[test]
    fn incremental_equals_one_shot() {
        let data: &[u8] = b"split-across-two-calls";
        let (a, b) = data.split_at(7);
        let inc = crc32(crc32(0, a), b);
        assert_eq!(inc, crc32_one_shot(data));
    }

    #[test]
    fn detects_single_bit_flips() {
        let data = [0xABu8; 64];
        let base = crc32_one_shot(&data);
        for bit in 0..8 {
            let mut flipped = data;
            flipped[17] ^= 1 << bit;
            assert_ne!(crc32_one_shot(&flipped), base, "bit {bit} undetected");
        }
    }
}
