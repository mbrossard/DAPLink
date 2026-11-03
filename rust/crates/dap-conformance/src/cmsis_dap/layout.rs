//! Request lengths, and the fixed reply lengths, from the CMSIS-DAP command layouts.

use super::ids::*;

/// What a command's layout says about its length.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Length {
    /// This many bytes, the id included.
    Len(usize),
    /// A byte needed to size it is missing.
    Short,
    /// No layout sizes it: a vendor or unassigned id, or a batch inside a batch.
    Unsized,
}

/// A probe's own layout for a command, asked first; `true` inside a batch.
pub type ProbeLayout<'a> = &'a dyn Fn(&[u8], bool) -> Option<Length>;

/// The request length of a command whose layout is fixed, id included.
pub fn fixed_request(id: u8) -> Option<usize> {
    Some(match id {
        INFO => 2,
        HOST_STATUS => 3,
        CONNECT => 2,
        DISCONNECT => 1,
        TRANSFER_CONFIGURE => 6,
        TRANSFER_ABORT => 1,
        WRITE_ABORT => 6,
        DELAY => 3,
        RESET_TARGET => 1,
        SWJ_PINS => 7,
        SWJ_CLOCK => 5,
        SWD_CONFIGURE => 2,
        JTAG_IDCODE => 2,
        SWO_TRANSPORT => 2,
        SWO_MODE => 2,
        SWO_BAUDRATE => 5,
        SWO_CONTROL => 2,
        SWO_STATUS => 1,
        SWO_DATA => 3,
        SWO_EXTENDED_STATUS => 2,
        UART_TRANSPORT => 2,
        UART_CONFIGURE => 6,
        UART_CONTROL => 2,
        UART_STATUS => 1,
        _ => return None,
    })
}

/// The reply length of a command whose reply layout is fixed, id included.
pub fn fixed_reply(id: u8) -> Option<usize> {
    Some(match id {
        HOST_STATUS | CONNECT | DISCONNECT | TRANSFER_CONFIGURE | WRITE_ABORT | DELAY => 2,
        SWJ_PINS | SWJ_CLOCK | SWJ_SEQUENCE | SWD_CONFIGURE | JTAG_CONFIGURE => 2,
        SWO_TRANSPORT | SWO_MODE | SWO_CONTROL | UART_TRANSPORT | UART_CONTROL => 2,
        RESET_TARGET => 3,
        SWO_BAUDRATE => 5,
        SWO_STATUS | UART_CONFIGURE => 6,
        UART_STATUS => 10,
        _ => return None,
    })
}

/// Clock cycles a sequence info byte asks for: bits 5:0, 0 meaning 64.
pub fn sequence_bits(info: u8) -> usize {
    match info & 0x3F {
        0 => 64,
        n => usize::from(n),
    }
}

/// The length of the request starting `report`.
pub fn request(report: &[u8]) -> Length {
    request_with(report, &|_, _| None)
}

/// As [`request()`], taking `probe`'s layout wherever it gives one.
pub fn request_with(report: &[u8], probe: ProbeLayout<'_>) -> Length {
    sized(report, false, probe).unwrap_or(Length::Short)
}

fn sized(r: &[u8], batched: bool, probe: ProbeLayout<'_>) -> Option<Length> {
    let id = *r.first()?;
    if let Some(len) = probe(r, batched) {
        return match len {
            Length::Short => None,
            len => Some(len),
        };
    }
    let at = |i: usize| r.get(i).copied();
    let len = match id {
        TRANSFER => {
            let mut pos = 3;
            for _ in 0..at(2)? {
                let req = at(pos)?;
                let word = req & request::READ == 0 || req & request::VALUE_MATCH != 0;
                pos += 1 + if word { 4 } else { 0 };
            }
            pos
        }
        TRANSFER_BLOCK => {
            let count = usize::from(u16::from_le_bytes([at(2)?, at(3)?]));
            5 + if at(4)? & request::READ == 0 {
                4 * count
            } else {
                0
            }
        }
        SWJ_SEQUENCE => {
            let bits = match at(1)? {
                0 => 256,
                n => usize::from(n),
            };
            2 + bits.div_ceil(8)
        }
        JTAG_SEQUENCE => {
            let mut pos = 2;
            for _ in 0..at(1)? {
                pos += 1 + sequence_bits(at(pos)?).div_ceil(8);
            }
            pos
        }
        SWD_SEQUENCE => {
            let mut pos = 2;
            for _ in 0..at(1)? {
                let info = at(pos)?;
                pos += 1 + if info & 0x80 == 0 {
                    sequence_bits(info).div_ceil(8)
                } else {
                    0
                };
            }
            pos
        }
        JTAG_CONFIGURE => 2 + usize::from(at(1)?),
        UART_TRANSFER => 5 + usize::from(u16::from_le_bytes([at(3)?, at(4)?])),
        QUEUE_COMMANDS | EXECUTE_COMMANDS if batched => return Some(Length::Unsized),
        QUEUE_COMMANDS | EXECUTE_COMMANDS => {
            let mut pos = 2;
            for _ in 0..at(1)? {
                match sized(r.get(pos..)?, true, probe)? {
                    Length::Len(n) => pos += n,
                    other => return Some(other),
                }
            }
            pos
        }
        _ => match fixed_request(id) {
            Some(n) => n,
            None => return Some(Length::Unsized),
        },
    };
    Some(Length::Len(len))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_assigned_id_has_a_layout() {
        for (id, name) in COMMANDS {
            let sized = matches!(request(&[id, 0, 0, 0, 0, 0, 0]), Length::Len(_));
            assert!(sized, "{name}");
        }
        for id in (0x0B..=0x0F).chain(0x24..=0x7D).chain(0x80..=0xFF) {
            assert_eq!(request(&[id, 0, 0, 0, 0]), Length::Unsized, "{id:#04x}");
        }
    }

    #[test]
    fn variable_requests_size_from_their_counts() {
        // A write, a value-match read and a plain read.
        let t = [0x05, 0, 3, 0x00, 1, 2, 3, 4, 0x12, 5, 6, 7, 8, 0x02];
        assert_eq!(request(&t), Length::Len(14));
        assert_eq!(request(&t[..9]), Length::Short);
        assert_eq!(request(&[0x06, 0, 2, 0, 0x00]), Length::Len(13));
        assert_eq!(request(&[0x06, 0, 2, 0, 0x02]), Length::Len(5));
        assert_eq!(request(&[0x12, 0]), Length::Len(34));
        assert_eq!(request(&[0x1D, 2, 8, 0xFF, 0x80 | 32]), Length::Len(5));
        assert_eq!(request(&[0x14, 1, 0x40]), Length::Len(11));
        assert_eq!(request(&[0x14, 2, 0x40]), Length::Short);
        assert_eq!(request(&[0x21, 0x40, 0, 3, 0]), Length::Len(8));
        assert_eq!(request(&[0x21, 0x40, 0, 3]), Length::Short);
    }

    #[test]
    fn a_batch_sums_its_commands_and_cannot_nest() {
        assert_eq!(request(&[0x7F, 2, 0x00, 0x04, 0x03]), Length::Len(5));
        assert_eq!(request(&[0x7E, 1, 0x7F, 0]), Length::Unsized);
        assert_eq!(request(&[0x7F, 1, 0x80]), Length::Unsized);
        assert_eq!(request(&[0x7F, 2, 0x00]), Length::Short);
    }

    #[test]
    fn a_probe_layout_is_asked_first() {
        let probe = |r: &[u8], batched: bool| (r[0] == 0x8C && !batched).then_some(Length::Len(9));
        assert_eq!(request_with(&[0x8C], &probe), Length::Len(9));
        assert_eq!(request_with(&[0x7F, 1, 0x8C], &probe), Length::Unsized);
    }
}
