//! Content-based format detection.

/// A detected image format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    /// Microsoft uf2.
    Uf2,
    /// Could not be determined.
    Unknown,
}

/// Detect an image format from an optional filename and the first bytes of the stream.
///
/// A recognized filename extension takes precedence over content sniffing; otherwise the format
/// is inferred from the leading bytes (the uf2 magic word).
pub fn detect_format(filename: Option<&str>, first_chunk: &[u8]) -> Format {
    if let Some(name) = filename {
        if let Some(dot) = name.rfind('.') {
            if name[dot + 1..].eq_ignore_ascii_case("uf2") {
                return Format::Uf2;
            }
        }
    }

    if first_chunk.len() >= 4 {
        let magic = u32::from_le_bytes([
            first_chunk[0],
            first_chunk[1],
            first_chunk[2],
            first_chunk[3],
        ]);
        if magic == 0x0A32_4655 {
            return Format::Uf2;
        }
    }

    Format::Unknown
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_by_extension_case_insensitive() {
        assert_eq!(detect_format(Some("app.UF2"), &[]), Format::Uf2);
    }

    #[test]
    fn detect_uf2_by_magic() {
        let chunk = [0x55, 0x46, 0x32, 0x0A, 0x00, 0x00, 0x00, 0x00];
        assert_eq!(detect_format(None, &chunk), Format::Uf2);
    }

    #[test]
    fn detect_unknown_fallback() {
        assert_eq!(
            detect_format(None, &[0x00, 0x01, 0x02, 0x03]),
            Format::Unknown
        );
        assert_eq!(detect_format(Some("x.dat"), &[0xFF; 8]), Format::Unknown);
    }
}
