//! CCSDS packet-definition table (Phase 20): APID → named, typed, scaled
//! fields, so telemetry decodes into engineering units instead of hex.
//!
//! Multi-byte fields are big-endian, as CCSDS specifies. A field is read at
//! `offset`, interpreted as `kind`, and multiplied by `scale` to give the
//! engineering value in `unit`.
//!
//! The built-in [`DEFINITIONS`] are reference layouts for this workspace's
//! subsystems. A real mission supplies its own interface control document
//! and its own table; the decoder does not depend on these values.

/// Wire type of a field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Unsigned 8-bit.
    U8,
    /// Signed 16-bit.
    I16,
    /// Unsigned 16-bit.
    U16,
    /// IEEE-754 single precision.
    F32,
}

impl Kind {
    /// Bytes this kind occupies on the wire.
    #[must_use]
    pub const fn size(self) -> usize {
        match self {
            Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::F32 => 4,
        }
    }
}

/// One named field inside a packet's payload.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Field {
    /// Field name, as shown in decoded output.
    pub name: &'static str,
    /// Byte offset into the payload.
    pub offset: usize,
    /// Wire type.
    pub kind: Kind,
    /// Multiplier from raw counts to engineering units.
    pub scale: f64,
    /// Engineering unit, or empty for a dimensionless value.
    pub unit: &'static str,
}

/// The layout of one packet type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PacketDef {
    /// Application process identifier this layout applies to.
    pub apid: u16,
    /// Packet name.
    pub name: &'static str,
    /// Fields, in payload order.
    pub fields: &'static [Field],
}

/// A decoded field value.
#[derive(Debug, Clone, PartialEq)]
pub struct Decoded {
    /// Field name.
    pub name: &'static str,
    /// Engineering value (raw counts × scale).
    pub value: f64,
    /// Engineering unit.
    pub unit: &'static str,
}

/// Why a payload could not be decoded against a definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DefError {
    /// The payload ends before the named field.
    Short {
        /// The field that did not fit.
        field: &'static str,
    },
}

impl std::fmt::Display for DefError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Short { field } => write!(f, "payload ends before field `{field}`"),
        }
    }
}

impl std::error::Error for DefError {}

/// Reference layouts for the workspace's subsystems.
pub static DEFINITIONS: &[PacketDef] = &[
    PacketDef {
        apid: 0x101,
        name: "adcs",
        fields: &[
            Field {
                name: "wheel_x",
                offset: 0,
                kind: Kind::I16,
                scale: 1.0,
                unit: "rpm",
            },
            Field {
                name: "wheel_y",
                offset: 2,
                kind: Kind::I16,
                scale: 1.0,
                unit: "rpm",
            },
            Field {
                name: "wheel_z",
                offset: 4,
                kind: Kind::I16,
                scale: 1.0,
                unit: "rpm",
            },
            Field {
                name: "sun_angle",
                offset: 6,
                kind: Kind::U16,
                scale: 0.01,
                unit: "deg",
            },
        ],
    },
    PacketDef {
        apid: 0x102,
        name: "power",
        fields: &[
            Field {
                name: "bus_voltage",
                offset: 0,
                kind: Kind::U16,
                scale: 0.01,
                unit: "V",
            },
            Field {
                name: "bus_current",
                offset: 2,
                kind: Kind::I16,
                scale: 0.001,
                unit: "A",
            },
            Field {
                name: "soc",
                offset: 4,
                kind: Kind::U8,
                scale: 0.5,
                unit: "%",
            },
            Field {
                name: "shedding",
                offset: 5,
                kind: Kind::U8,
                scale: 1.0,
                unit: "",
            },
        ],
    },
    PacketDef {
        apid: 0x103,
        name: "thermal",
        fields: &[
            Field {
                name: "battery",
                offset: 0,
                kind: Kind::I16,
                scale: 0.1,
                unit: "C",
            },
            Field {
                name: "electronics",
                offset: 2,
                kind: Kind::I16,
                scale: 0.1,
                unit: "C",
            },
            Field {
                name: "heaters",
                offset: 4,
                kind: Kind::U8,
                scale: 1.0,
                unit: "",
            },
        ],
    },
    PacketDef {
        apid: 0x104,
        name: "dtn",
        fields: &[
            Field {
                name: "pending_bundles",
                offset: 0,
                kind: Kind::U16,
                scale: 1.0,
                unit: "",
            },
            Field {
                name: "free_sectors",
                offset: 2,
                kind: Kind::U16,
                scale: 1.0,
                unit: "",
            },
            Field {
                name: "link_up",
                offset: 4,
                kind: Kind::U8,
                scale: 1.0,
                unit: "",
            },
        ],
    },
];

/// The layout for `apid`, if the table has one.
#[must_use]
pub fn lookup(apid: u16) -> Option<&'static PacketDef> {
    DEFINITIONS.iter().find(|d| d.apid == apid)
}

/// Decode `payload` against `def`, calling `visit` for each field in payload
/// order. Allocation-free: every field is checked before any is reported.
///
/// # Errors
/// [`DefError::Short`] when the payload ends before a field.
pub fn decode_each(
    def: &PacketDef,
    payload: &[u8],
    mut visit: impl FnMut(Decoded),
) -> Result<(), DefError> {
    for f in def.fields {
        read(f, payload)?;
    }
    for f in def.fields {
        let raw = read(f, payload)?;
        visit(Decoded {
            name: f.name,
            value: raw * f.scale,
            unit: f.unit,
        });
    }
    Ok(())
}

/// The raw value of field `f`, read big-endian from `payload`.
fn read(f: &Field, payload: &[u8]) -> Result<f64, DefError> {
    let short = DefError::Short { field: f.name };
    let end = f.offset + f.kind.size();
    let bytes = payload.get(f.offset..end).ok_or(short)?;
    match (f.kind, bytes) {
        (Kind::U8, [a]) => Ok(f64::from(*a)),
        (Kind::I16, [a, b]) => Ok(f64::from(i16::from_be_bytes([*a, *b]))),
        (Kind::U16, [a, b]) => Ok(f64::from(u16::from_be_bytes([*a, *b]))),
        (Kind::F32, [a, b, c, d]) => Ok(f64::from(f32::from_be_bytes([*a, *b, *c, *d]))),
        _ => Err(short),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_all(def: &PacketDef, payload: &[u8]) -> Result<Vec<Decoded>, DefError> {
        let mut out = Vec::new();
        decode_each(def, payload, |d| out.push(d)).map(|()| out)
    }

    #[test]
    fn definitions_are_ordered_disjoint_and_unique() {
        let mut apids: Vec<u16> = DEFINITIONS.iter().map(|d| d.apid).collect();
        apids.sort_unstable();
        apids.dedup();
        assert_eq!(apids.len(), DEFINITIONS.len(), "duplicate APID");
        for def in DEFINITIONS {
            let mut next = 0;
            for f in def.fields {
                assert!(
                    f.offset >= next,
                    "{}.{} overlaps the previous field",
                    def.name,
                    f.name
                );
                next = f.offset + f.kind.size();
            }
        }
    }

    #[test]
    fn decodes_big_endian_scaled_values() {
        let def = lookup(0x102).expect("power is defined");
        // bus 28.50 V, current -1.250 A, SoC 80 % (raw 160), shedding on.
        let payload = [0x0B, 0x22, 0xFB, 0x1E, 160, 1];
        let out = decode_all(def, &payload).expect("decodes");
        assert!((out[0].value - 28.5).abs() < 1e-9 && out[0].unit == "V");
        assert!((out[1].value + 1.25).abs() < 1e-9);
        assert!((out[2].value - 80.0).abs() < 1e-9);
        assert_eq!(out[3].value, 1.0);
    }

    #[test]
    fn short_payload_names_the_field_that_did_not_fit() {
        let def = lookup(0x101).expect("adcs is defined");
        assert_eq!(
            decode_all(def, &[0, 1, 0, 2]),
            Err(DefError::Short { field: "wheel_z" })
        );
    }

    #[test]
    fn unknown_apid_has_no_layout() {
        assert!(lookup(0x7FF).is_none());
    }

    #[test]
    fn size_matches_kind() {
        assert_eq!(Kind::U8.size(), 1);
        assert_eq!(Kind::I16.size(), 2);
        assert_eq!(Kind::U16.size(), 2);
        assert_eq!(Kind::F32.size(), 4);
    }
}
