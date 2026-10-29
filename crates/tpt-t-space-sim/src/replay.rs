//! Replay files — the one on-disk format shared by the headless sim, the
//! `tpts` CLI, and the visualizer.
//!
//! The format is line-oriented CSV with no external dependencies. The
//! first line is the header `tpt-replay,1`; every following line starts
//! with a record kind:
//!
//! ```text
//! tpt-replay,1
//! orbit,<x_m>,<y_m>,<z_m>                 ECI position, one per sample
//! att,<qw>,<qx>,<qy>,<qz>,<sx>,<sy>,<sz>  body→ECI quaternion, sun in body frame
//! bud,<power_w>,<soc_pct>,<temp_c>        power / battery / thermal budgets
//! terrain,<width>,<height>                DEM dimensions (cells)
//! height,<w*h values, row-major>          one line, after `terrain`
//! planned,<gx>,<gy>                       planned path cell
//! actual,<gx>,<gy>                        actual path cell
//! ```
//!
//! Lines starting with `#` and blank lines are ignored. Floats are written
//! with Rust's shortest round-trip `Display`, so `parse(write(r)) == r`.

use std::fmt;
use std::io::{self, Write};

/// Header line, including the format version.
pub const HEADER: &str = "tpt-replay,1";

/// Attitude sample: body→ECI quaternion and the sun direction in the body
/// frame (unit vector).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    /// Quaternion `[w, x, y, z]`.
    pub q: [f32; 4],
    /// Sun direction, body frame.
    pub sun_body: [f32; 3],
}

/// One budget sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Budget {
    /// Solar harvest, watts.
    pub power_w: f32,
    /// Battery state of charge, percent.
    pub soc_pct: f32,
    /// Battery temperature, degrees Celsius.
    pub temp_c: f32,
}

/// Rover terrain: a height grid plus the planned and actual paths over it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Terrain {
    /// Grid width, cells.
    pub width: usize,
    /// Grid height, cells.
    pub height: usize,
    /// Heights, metres, row-major (`width * height` values).
    pub heights: Vec<f32>,
    /// Planned path, grid cells.
    pub planned: Vec<(usize, usize)>,
    /// Actual path, grid cells.
    pub actual: Vec<(usize, usize)>,
}

/// A complete replay: everything the visualizer panes render.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Replay {
    /// Spacecraft ECI positions, metres.
    pub orbit: Vec<[f64; 3]>,
    /// Attitude samples.
    pub attitude: Vec<Attitude>,
    /// Budget samples.
    pub budgets: Vec<Budget>,
    /// Rover terrain and paths.
    pub terrain: Terrain,
}

/// Why a replay file failed to parse.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReplayError {
    /// The first line is not the expected header.
    Header,
    /// A record kind is unknown or a field is missing or malformed.
    Record {
        /// 1-based line number.
        line: usize,
        /// What was wrong.
        reason: &'static str,
    },
    /// The `height` record does not match `terrain` dimensions.
    TerrainSize,
}

impl fmt::Display for ReplayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Header => write!(f, "missing or unsupported header (expected `{HEADER}`)"),
            Self::Record { line, reason } => write!(f, "line {line}: {reason}"),
            Self::TerrainSize => write!(f, "height record does not match terrain dimensions"),
        }
    }
}

impl std::error::Error for ReplayError {}

impl Replay {
    /// Serialize in the documented format.
    ///
    /// # Errors
    /// Propagates any I/O error from `out`.
    pub fn write_to<W: Write>(&self, out: &mut W) -> io::Result<()> {
        writeln!(out, "{HEADER}")?;
        for p in &self.orbit {
            writeln!(out, "orbit,{},{},{}", p[0], p[1], p[2])?;
        }
        for a in &self.attitude {
            let [w, x, y, z] = a.q;
            let [sx, sy, sz] = a.sun_body;
            writeln!(out, "att,{w},{x},{y},{z},{sx},{sy},{sz}")?;
        }
        for b in &self.budgets {
            writeln!(out, "bud,{},{},{}", b.power_w, b.soc_pct, b.temp_c)?;
        }
        let t = &self.terrain;
        if t.width > 0 && t.height > 0 {
            writeln!(out, "terrain,{},{}", t.width, t.height)?;
            write!(out, "height")?;
            for h in &t.heights {
                write!(out, ",{h}")?;
            }
            writeln!(out)?;
        }
        for &(gx, gy) in &t.planned {
            writeln!(out, "planned,{gx},{gy}")?;
        }
        for &(gx, gy) in &t.actual {
            writeln!(out, "actual,{gx},{gy}")?;
        }
        Ok(())
    }

    /// Serialize to a `String`.
    #[must_use]
    pub fn to_text(&self) -> String {
        let mut buf = Vec::new();
        // Writing into a Vec<u8> cannot fail.
        let _ = self.write_to(&mut buf);
        String::from_utf8(buf).unwrap_or_default()
    }

    /// Parse a replay file.
    ///
    /// # Errors
    /// Returns [`ReplayError`] with the offending line for any malformed
    /// record, a bad header, or a height record that does not match the
    /// terrain dimensions.
    pub fn parse(text: &str) -> Result<Self, ReplayError> {
        let mut lines = text
            .lines()
            .enumerate()
            .map(|(i, l)| (i + 1, l.trim()))
            .filter(|(_, l)| !l.is_empty() && !l.starts_with('#'));
        match lines.next() {
            Some((_, HEADER)) => {}
            _ => return Err(ReplayError::Header),
        }
        let mut r = Self::default();
        for (line, text) in lines {
            let mut f = text.split(',');
            let kind = f.next().unwrap_or_default();
            let err = |reason| ReplayError::Record { line, reason };
            match kind {
                "orbit" => r.orbit.push([
                    num(&mut f, line, "orbit x")?,
                    num(&mut f, line, "orbit y")?,
                    num(&mut f, line, "orbit z")?,
                ]),
                "att" => {
                    let q = [
                        num(&mut f, line, "att qw")?,
                        num(&mut f, line, "att qx")?,
                        num(&mut f, line, "att qy")?,
                        num(&mut f, line, "att qz")?,
                    ];
                    let sun_body = [
                        num(&mut f, line, "att sx")?,
                        num(&mut f, line, "att sy")?,
                        num(&mut f, line, "att sz")?,
                    ];
                    r.attitude.push(Attitude { q, sun_body });
                }
                "bud" => r.budgets.push(Budget {
                    power_w: num(&mut f, line, "bud power")?,
                    soc_pct: num(&mut f, line, "bud soc")?,
                    temp_c: num(&mut f, line, "bud temp")?,
                }),
                "terrain" => {
                    r.terrain.width = num(&mut f, line, "terrain width")?;
                    r.terrain.height = num(&mut f, line, "terrain height")?;
                }
                "height" => {
                    let heights: Result<Vec<f32>, _> = f.map(|s| s.parse::<f32>()).collect();
                    r.terrain.heights = heights.map_err(|_| err("height value"))?;
                    if r.terrain.heights.len() != r.terrain.width * r.terrain.height {
                        return Err(ReplayError::TerrainSize);
                    }
                }
                "planned" => r.terrain.planned.push(cell(&mut f, line, "planned cell")?),
                "actual" => r.terrain.actual.push(cell(&mut f, line, "actual cell")?),
                _ => return Err(err("unknown record kind")),
            }
        }
        Ok(r)
    }
}

/// Parse the next comma-separated field as `T`.
fn num<'a, T: std::str::FromStr, I: Iterator<Item = &'a str>>(
    f: &mut I,
    line: usize,
    reason: &'static str,
) -> Result<T, ReplayError> {
    f.next()
        .and_then(|s| s.trim().parse::<T>().ok())
        .ok_or(ReplayError::Record { line, reason })
}

/// Parse the next two fields as a grid cell.
fn cell<'a, I: Iterator<Item = &'a str>>(
    f: &mut I,
    line: usize,
    reason: &'static str,
) -> Result<(usize, usize), ReplayError> {
    Ok((num(f, line, reason)?, num(f, line, reason)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Replay {
        Replay {
            orbit: vec![[7.0e6, -1.25, 0.5], [6.9e6, 3.0, -0.1]],
            attitude: vec![Attitude {
                q: [0.707_107, 0.0, 0.707_107, 0.0],
                sun_body: [0.0, 0.6, 0.8],
            }],
            budgets: vec![Budget {
                power_w: 212.5,
                soc_pct: 88.25,
                temp_c: -3.5,
            }],
            terrain: Terrain {
                width: 2,
                height: 2,
                heights: vec![0.0, 0.1, -0.2, 1.5],
                planned: vec![(0, 1), (1, 1)],
                actual: vec![(0, 1), (1, 0)],
            },
        }
    }

    #[test]
    fn round_trips_exactly() {
        let r = sample();
        assert_eq!(Replay::parse(&r.to_text()), Ok(r));
    }

    #[test]
    fn ignores_comments_and_blank_lines() {
        let text = "tpt-replay,1\n# a comment\n\nbud,1,2,3\n";
        let r = Replay::parse(text).expect("parses");
        assert_eq!(r.budgets.len(), 1);
    }

    #[test]
    fn rejects_bad_header() {
        assert_eq!(Replay::parse("bud,1,2,3\n"), Err(ReplayError::Header));
        assert_eq!(Replay::parse(""), Err(ReplayError::Header));
    }

    #[test]
    fn reports_line_of_bad_field() {
        let err = Replay::parse("tpt-replay,1\nbud,1,oops,3\n").unwrap_err();
        assert!(matches!(err, ReplayError::Record { line: 2, .. }));
    }

    #[test]
    fn rejects_unknown_record() {
        let err = Replay::parse("tpt-replay,1\nmystery,1\n").unwrap_err();
        assert!(matches!(
            err,
            ReplayError::Record {
                line: 2,
                reason: "unknown record kind"
            }
        ));
    }

    #[test]
    fn rejects_terrain_size_mismatch() {
        let text = "tpt-replay,1\nterrain,3,3\nheight,1,2,3\n";
        assert_eq!(Replay::parse(text), Err(ReplayError::TerrainSize));
    }
}
