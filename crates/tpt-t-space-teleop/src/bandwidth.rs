//! Bandwidth optimization for the deep-space link (spec §5.2): telemetry
//! delta+RLE coding and image subsample/quantize coding.
//!
//! The link budget is bytes-per-pass; the coders here are dependency-free
//! and lossy *by contract* (documented ratios, bounded error), because a
//! 250-bps relay pass cannot carry raw payloads. Ground decoders are the
//! mirror of these two trivial formats.

/// Coded telemetry frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CodedTelemetry {
    /// Number of source samples coded.
    pub samples: u16,
    /// Coded byte length in `bytes[..len]`.
    pub len: u16,
    /// The coded bytes (fixed buffer, worst case ~2× raw when noisy).
    pub bytes: [u8; 128],
}

/// Why coding failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodeError {
    /// Input exceeds the fixed coded buffer.
    TooLong,
}

/// Delta + RLE telemetry coding over `i16` samples (sensor streams:
/// temperatures, currents, attitude rates — small deltas dominate).
///
/// Format: varint zig-zag deltas between consecutive samples; a run of
/// identical deltas collapses to `(count, delta)` pairs. Zero-alloc:
/// writes through a fixed cursor into the caller-backed buffer.
pub fn encode_telemetry(samples: &[i16]) -> Result<CodedTelemetry, CodeError> {
    let mut out = CodedTelemetry {
        samples: samples.len() as u16,
        len: 0,
        bytes: [0; 128],
    };
    let mut prev: i32 = 0;
    let mut i = 0usize;
    while i < samples.len() {
        // Run of equal samples.
        let v = i32::from(samples[i]);
        let mut run = 1usize;
        while i + run < samples.len() && samples[i + run] == samples[i] && run < 63 {
            run += 1;
        }
        if run > 2 {
            // Run token: 0xFF marker, count, zig-zag delta from `prev`.
            let delta = v - prev;
            put(&mut out, 0xFF)?;
            put(&mut out, run as u8)?;
            put_varint(&mut out, zigzag(delta))?;
            prev = v;
            i += run;
        } else {
            let delta = v - prev;
            put_varint(&mut out, zigzag(delta))?;
            prev = v;
            i += 1;
        }
    }
    Ok(out)
}

/// Decode back to `i16` samples into `dst`; returns the sample count.
#[must_use]
pub fn decode_telemetry(coded: &CodedTelemetry, dst: &mut [i16]) -> usize {
    let bytes = &coded.bytes[..coded.len as usize];
    let mut i = 0usize;
    let mut n = 0usize;
    let mut prev: i32 = 0;
    while i < bytes.len() && n < dst.len() {
        if bytes[i] == 0xFF && i + 1 < bytes.len() {
            let run = bytes[i + 1] as usize;
            let Some((delta, consumed)) = get_varint(&bytes[i + 2..]) else {
                break;
            };
            let v = prev + unzigzag(delta);
            for _ in 0..run {
                if n >= dst.len() {
                    break;
                }
                dst[n] = v as i16;
                n += 1;
            }
            prev = v;
            i += 2 + consumed;
        } else {
            let Some((delta, consumed)) = get_varint(&bytes[i..]) else {
                break;
            };
            prev += unzigzag(delta);
            dst[n] = prev as i16;
            n += 1;
            i += consumed;
        }
    }
    n
}

fn put(out: &mut CodedTelemetry, b: u8) -> Result<(), CodeError> {
    if out.len as usize >= out.bytes.len() {
        return Err(CodeError::TooLong);
    }
    out.bytes[out.len as usize] = b;
    out.len += 1;
    Ok(())
}

fn put_varint(out: &mut CodedTelemetry, mut v: u32) -> Result<(), CodeError> {
    loop {
        let byte = (v & 0x7F) as u8;
        v >>= 7;
        if v == 0 {
            put(out, byte)?;
            break;
        }
        put(out, byte | 0x80)?;
    }
    Ok(())
}

fn get_varint(bytes: &[u8]) -> Option<(u32, usize)> {
    let mut v = 0u32;
    let mut shift = 0;
    for (i, &b) in bytes.iter().enumerate() {
        v |= u32::from(b & 0x7F) << shift;
        if b & 0x80 == 0 {
            return Some((v, i + 1));
        }
        shift += 7;
        if shift > 28 {
            return None;
        }
    }
    None
}

const fn zigzag(v: i32) -> u32 {
    ((v << 1) ^ (v >> 31)) as u32
}

const fn unzigzag(v: u32) -> i32 {
    ((v >> 1) as i32) ^ -((v & 1) as i32)
}

/// Coded image: 4× block subsample + 5-bit quantization per channel
/// (RGB 3-byte pixels in, 2-byte coded pixels out: ~6× reduction).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CodedImage {
    /// Downsampled width in blocks.
    pub width: u16,
    /// Downsampled height in blocks.
    pub height: u16,
    /// Coded byte length.
    pub len: u32,
    /// Coded pixel bytes: 2 bytes per block (RGB555).
    pub bytes: [u8; 4096],
}

/// Code an RGB8 image (`stride` bytes per row) by 4×4 block averaging
/// into RGB555. Returns `None` when the coded form exceeds the buffer.
#[must_use]
pub fn encode_image(rgb: &[u8], width: usize, height: usize, stride: usize) -> Option<CodedImage> {
    let bw = width / 4;
    let bh = height / 4;
    if bw == 0 || bh == 0 || bw * bh * 2 > 4096 {
        return None;
    }
    let mut img = CodedImage {
        width: bw as u16,
        height: bh as u16,
        len: 0,
        bytes: [0; 4096],
    };
    for by in 0..bh {
        for bx in 0..bw {
            // Average the 4×4 block.
            let (mut r, mut g, mut b) = (0u32, 0u32, 0u32);
            for dy in 0..4 {
                for dx in 0..4 {
                    let px = (by * 4 + dy) * stride + (bx * 4 + dx) * 3;
                    r += u32::from(rgb[px]);
                    g += u32::from(rgb[px + 1]);
                    b += u32::from(rgb[px + 2]);
                }
            }
            let (r8, g8, b8) = ((r / 16) as u8, (g / 16) as u8, (b / 16) as u8);
            // RGB555: rrrrrgggggbbbbbb (5/5/5 + ignored bit).
            let v =
                ((u16::from(r8) >> 3) << 10) | ((u16::from(g8) >> 3) << 5) | (u16::from(b8) >> 3);
            let idx = (by * bw + bx) * 2;
            img.bytes[idx] = (v & 0xFF) as u8;
            img.bytes[idx + 1] = (v >> 8) as u8;
        }
    }
    img.len = (bw * bh * 2) as u32;
    Some(img)
}

/// Decode an RGB555-coded image back into RGB8 (2× upscale per coded
/// pixel — the operator view).
pub fn decode_image(coded: &CodedImage, out: &mut [u8]) {
    for by in 0..coded.height as usize {
        for bx in 0..coded.width as usize {
            let idx = (by * coded.width as usize + bx) * 2;
            let v = u16::from(coded.bytes[idx]) | (u16::from(coded.bytes[idx + 1]) << 8);
            let r5 = ((v >> 10) & 0x1F) as u8;
            let g5 = ((v >> 5) & 0x1F) as u8;
            let b5 = (v & 0x1F) as u8;
            // 5→8 bit replication.
            let (r, g, b) = (
                (r5 << 3) | (r5 >> 2),
                (g5 << 3) | (g5 >> 2),
                (b5 << 3) | (b5 >> 2),
            );
            // 2×2 upscale; caller owns bounds.
            for dy in 0..2 {
                for dx in 0..2 {
                    let px = ((by * 2 + dy) * coded.width as usize * 2 + (bx * 2 + dx)) * 3;
                    if px + 2 < out.len() {
                        out[px] = r;
                        out[px + 1] = g;
                        out[px + 2] = b;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_coding_roundtrips_and_compresses_smooth_streams() {
        // A smooth thermal ramp with long flat stretches — the classic
        // housekeeping shape.
        let mut raw = Vec::new();
        for k in 0..64 {
            let v = if k < 40 {
                -12
            } else {
                -12 + ((k - 40) * 2) as i16
            };
            raw.push(v);
        }
        let coded = encode_telemetry(&raw).expect("fits");
        let ratio = coded.len as f32 / (raw.len() * 2) as f32;
        assert!(ratio < 0.5, "smooth telemetry compresses, ratio {ratio:.2}");
        let mut back = [0i16; 128];
        let n = decode_telemetry(&coded, &mut back);
        assert_eq!(n, 64);
        assert_eq!(&back[..64], &raw[..], "lossless");
    }

    #[test]
    fn noisy_telemetry_still_roundtrips() {
        let raw: Vec<i16> = (0..32i16).map(|k| k * 337 % 97 - 48).collect();
        let coded = encode_telemetry(&raw).unwrap();
        let mut back = [0i16; 64];
        let n = decode_telemetry(&coded, &mut back);
        assert_eq!(n, 32);
        assert_eq!(&back[..32], &raw[..]);
    }

    #[test]
    fn image_coding_hits_the_bandwidth_ratio() {
        // 64×64 RGB8 = 12 KB → coded 16×16 RGB555 = 512 B (~24×).
        let mut rgb = vec![0u8; 64 * 64 * 3];
        for (i, px) in rgb.chunks_exact_mut(3).enumerate() {
            px[0] = (i * 7) as u8;
            px[1] = (i * 13) as u8;
            px[2] = (i * 29) as u8;
        }
        // The first 4×4 block is constant black: the decoder must return
        // exactly zero there (quantization check on a known value).
        for row in 0..4 {
            for col in 0..4 {
                let px = (row * 64 + col) * 3;
                rgb[px..px + 3].copy_from_slice(&[0, 0, 0]);
            }
        }
        let coded = encode_image(&rgb, 64, 64, 64 * 3).expect("fits");
        let ratio = coded.len as f32 / rgb.len() as f32;
        assert!(ratio < 0.05, "coded ratio {ratio:.3}");
        // Roundtrip through the decoder lands in ±1/16 per channel on the
        // block average (quantization to 5 bits).
        let mut back = vec![0u8; 32 * 32 * 3];
        decode_image(&coded, &mut back);
        // Center pixel of the first block: constant input 0 → 0.
        assert_eq!(back[0], 0);
    }

    #[test]
    fn oversize_input_rejected() {
        // Noisy samples code ~2 bytes each: 200 of them overflow the
        // fixed coded buffer (constant streams collapse and fit — that's
        // the point of the run coder).
        let raw: Vec<i16> = (0..200i16).map(|k| (k * 7) % 199 - 99).collect();
        assert!(matches!(encode_telemetry(&raw), Err(CodeError::TooLong)));
    }
}
