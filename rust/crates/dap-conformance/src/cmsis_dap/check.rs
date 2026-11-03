//! One exchange checked: the request well formed, the reply what its layout and counts allow.

use super::codec::{Command, Context, Counting, Reply};
use super::ids::*;
use super::status::Status;

/// A fact about an exchange that the CMSIS-DAP layouts or value ranges do not allow.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Finding {
    /// The request ends before its layout does.
    ShortRequest,
    /// A `DAP_Transfer` or `DAP_TransferBlock` request for no transfers.
    ZeroCount,
    /// This transfer item asks for a timestamp with a value match or match mask.
    TimestampWithMatch(usize),
    /// No reply to a command that has one.
    NoReply,
    /// A reply to `DAP_TransferAbort`, which has none.
    ReplyToAbort,
    /// The reply starts with neither the command id nor `0xFF`.
    WrongId(u8),
    /// The reply ends before its layout does.
    Truncated {
        /// Bytes the layout needs.
        expected: usize,
        /// Bytes present.
        got: usize,
    },
    /// The reply runs past its layout.
    Excess {
        /// Bytes the layout allows.
        expected: usize,
        /// Bytes present.
        got: usize,
    },
    /// A response status other than DAP_OK and DAP_ERROR.
    UnknownStatus(u8),
    /// A transfer status byte no status can be.
    MalformedStatus(u8),
    /// More transfers counted than requested.
    CountAbove {
        /// Transfers requested.
        requested: usize,
        /// The reply's count.
        counted: usize,
    },
    /// A value mismatch where the transfer that stopped is no value-match read.
    MismatchWithoutMatch,
    /// `DAP_Connect` answered a port it was not asked for.
    WrongPort {
        /// The port requested.
        requested: u8,
        /// The port answered.
        answered: u8,
    },
    /// A `DAP_Info` value of a length its type does not allow.
    InfoLength {
        /// The info id.
        id: u8,
        /// The `Len` answered.
        len: usize,
    },
    /// A `DAP_Info` string without its NUL terminator.
    UnterminatedString(u8),
}

/// A request and the reply decoded against it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Exchange {
    /// The request.
    pub command: Command,
    /// The reply.
    pub reply: Reply,
}

/// What [`exchange`] established.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Checked {
    /// The decoded exchange, when both sides decode.
    pub exchange: Option<Exchange>,
    /// Every finding, request first.
    pub findings: Vec<Finding>,
}

/// Check `reply`, unpadded, as the answer to the command starting `request`.
pub fn exchange(request: &[u8], reply: &[u8], context: Context) -> Checked {
    let command = match Command::decode(request) {
        Ok(c) => c,
        Err(f) => {
            return Checked {
                exchange: None,
                findings: vec![f],
            }
        }
    };
    let mut findings = request_findings(&command);
    let exchange = match Reply::decode(&command, reply, context) {
        Ok(reply) => {
            findings.extend(reply_findings(&command, &reply, context));
            Some(Exchange { command, reply })
        }
        Err(f) => {
            findings.push(f);
            None
        }
    };
    Checked { exchange, findings }
}

fn request_findings(command: &Command) -> Vec<Finding> {
    let mut out = Vec::new();
    match command {
        Command::Transfer(_, items) => {
            out.extend(items.is_empty().then_some(Finding::ZeroCount));
            for (i, item) in items.iter().enumerate() {
                let matching = request::VALUE_MATCH | request::MATCH_MASK;
                if item.req & request::TIMESTAMP != 0 && item.req & matching != 0 {
                    out.push(Finding::TimestampWithMatch(i));
                }
            }
        }
        Command::TransferBlock(_, 0, ..) => out.push(Finding::ZeroCount),
        _ => {}
    }
    out
}

fn status_byte(b: u8) -> Option<Finding> {
    (b != 0x00 && b != 0xFF).then_some(Finding::UnknownStatus(b))
}

fn transfer_status(status: Status) -> Option<Finding> {
    match status {
        Status::Malformed(b) => Some(Finding::MalformedStatus(b)),
        _ => None,
    }
}

fn info_finding(id: u8, value: &[u8]) -> Option<Finding> {
    let len = value.len();
    let allowed: &[usize] = match id {
        info::VENDOR..=info::FIRMWARE_VERSION => {
            return (value.last().is_some_and(|&b| b != 0))
                .then_some(Finding::UnterminatedString(id))
        }
        info::CAPABILITIES => &[0, 1, 2],
        info::UART_RX_BUFFER | info::UART_TX_BUFFER | info::SWO_BUFFER => &[0, 4],
        info::PACKET_COUNT => &[0, 1],
        info::PACKET_SIZE => &[0, 2],
        _ => return None,
    };
    (!allowed.contains(&len)).then_some(Finding::InfoLength { id, len })
}

fn reply_findings(command: &Command, reply: &Reply, context: Context) -> Vec<Finding> {
    let mut out = Vec::new();
    match (command, reply) {
        (Command::Info(id), Reply::Info(value)) => out.extend(info_finding(*id, value)),
        (Command::Connect(requested), Reply::Byte(answered)) => {
            let (r, a) = (*requested, *answered);
            if a != 0 && a != r && !(r == 0 && a <= 2) {
                out.push(Finding::WrongPort {
                    requested: r,
                    answered: a,
                });
            }
        }
        (Command::SwjPins(..), Reply::Byte(_)) => {}
        (_, Reply::Byte(b) | Reply::ResetTarget(b, _) | Reply::Sequence(b, _)) => {
            out.extend(status_byte(*b));
        }
        (_, Reply::Idcode(b, _)) => out.extend(status_byte(*b)),
        (Command::Transfer(_, items), Reply::Transfer(t)) => {
            let counted = usize::from(t.count);
            if counted > items.len() {
                out.push(Finding::CountAbove {
                    requested: items.len(),
                    counted,
                });
            }
            out.extend(transfer_status(t.status));
            let mismatch = matches!(
                t.status,
                Status::Ack { mismatch: true, .. } | Status::ProtocolError { mismatch: true, .. }
            );
            let stopped = match context.transfer {
                Counting::Arm => items.get(counted),
                Counting::DapRs => counted.checked_sub(1).and_then(|i| items.get(i)),
            };
            let match_read = request::READ | request::VALUE_MATCH;
            if mismatch && !stopped.is_some_and(|i| i.req & match_read == match_read) {
                out.push(Finding::MismatchWithoutMatch);
            }
        }
        (Command::TransferBlock(_, requested, ..), Reply::TransferBlock(b)) => {
            if b.count > *requested {
                out.push(Finding::CountAbove {
                    requested: usize::from(*requested),
                    counted: usize::from(b.count),
                });
            }
            out.extend(transfer_status(b.status));
        }
        _ => {}
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cmsis_dap::codec::{Link, Word};
    use crate::cmsis_dap::status::Ack;

    const ARM: Context = Context {
        link: Link::Swd,
        transfer: Counting::Arm,
        block: Counting::Arm,
    };

    const DAP_RS: Context = Context {
        link: Link::Swd,
        transfer: Counting::DapRs,
        block: Counting::DapRs,
    };

    /// As the probes here count: blocks as Arm's firmware, a failed `DAP_Transfer` as dap-rs does.
    const MIXED: Context = Context {
        block: Counting::Arm,
        ..DAP_RS
    };

    fn findings(request: &[u8], reply: &[u8], context: Context) -> Vec<Finding> {
        exchange(request, reply, context).findings
    }

    fn transfer(request: &[u8], reply: &[u8], context: Context) -> (u8, Status, Vec<Word>) {
        let checked = exchange(request, reply, context);
        assert_eq!(checked.findings, [], "{reply:02x?}");
        match checked.exchange.map(|e| e.reply) {
            Some(Reply::Transfer(t)) => (t.count, t.status, t.words),
            other => panic!("{other:?}"),
        }
    }

    fn data(item: usize, value: u32) -> Word {
        Word {
            item,
            timestamp: false,
            value,
        }
    }

    /// Two DP reads, DPIDR then CTRL/STAT.
    const TWO_DP_READS: [u8; 5] = [0x05, 0, 2, 0x02, 0x06];

    #[test]
    fn no_ack_on_the_second_read_counts_one_on_arm_and_two_on_dap_rs() {
        let arm = [0x05, 1, 0x07, 0x77, 0x14, 0xA0, 0x2B];
        let dap_rs = [0x05, 2, 0x07, 0x77, 0x14, 0xA0, 0x2B];
        let want = |count| (count, Status::transfer(0x07), vec![data(0, 0x2BA0_1477)]);
        assert_eq!(transfer(&TWO_DP_READS, &arm, ARM), want(1));
        assert_eq!(transfer(&TWO_DP_READS, &dap_rs, MIXED), want(2));
        assert_eq!(Status::transfer(0x07).ack(), Some(Ack::NoAck));
        assert_eq!(
            findings(&TWO_DP_READS, &dap_rs, ARM),
            [Finding::Truncated {
                expected: 11,
                got: 7
            }]
        );
        assert_eq!(
            findings(&TWO_DP_READS, &arm, MIXED),
            [Finding::Excess {
                expected: 3,
                got: 7
            }]
        );
    }

    #[test]
    fn a_parity_error_is_0x08_on_arm_and_0x0f_on_dap_rs() {
        let arm = [0x05, 1, 0x08, 1, 2, 3, 4];
        let dap_rs = [0x05, 2, 0x0F, 1, 2, 3, 4];
        let (_, status, words) = transfer(&TWO_DP_READS, &arm, ARM);
        assert_eq!((status.ack(), words), (None, vec![data(0, 0x0403_0201)]));
        let (_, status, _) = transfer(&TWO_DP_READS, &dap_rs, MIXED);
        assert_eq!(
            status,
            Status::ProtocolError {
                raw_ack_bits: 7,
                mismatch: false
            }
        );
    }

    #[test]
    fn a_value_mismatch_stops_at_the_match_read() {
        // A match mask write, then a DP value-match read.
        let request = [0x05, 0, 2, 0x20, 0xFF, 0, 0, 0, 0x12, 0x77, 0, 0, 0];
        let (_, status, words) = transfer(&request, &[0x05, 1, 0x11], ARM);
        assert_eq!(
            (status.ack(), status.is_ok(), words),
            (Some(Ack::Ok), false, vec![])
        );
        transfer(&request, &[0x05, 2, 0x11], MIXED);
        assert_eq!(
            findings(&TWO_DP_READS, &[0x05, 1, 0x11], MIXED),
            [Finding::MismatchWithoutMatch]
        );
    }

    #[test]
    fn a_truncated_reply_is_found() {
        let block = [0x06, 0, 3, 0, 0x0F];
        let mut reply = vec![0x06, 3, 0, 0x01];
        reply.extend([0x11; 12]);
        assert_eq!(findings(&block, &reply, ARM), []);
        assert_eq!(
            findings(&block, &reply[..14], ARM),
            [Finding::Truncated {
                expected: 16,
                got: 14
            }]
        );
        assert_eq!(
            findings(&TWO_DP_READS, &[0x05, 2], ARM),
            [Finding::Truncated {
                expected: 3,
                got: 2
            }]
        );
        assert_eq!(
            findings(&TWO_DP_READS, &[0x05, 2, 0x01, 1, 2, 3, 4, 5, 6], ARM),
            [Finding::Truncated {
                expected: 11,
                got: 9
            }]
        );
        assert_eq!(
            findings(&[0x00, 0x04], &[0x00, 6, b'2', b'.'], ARM),
            [Finding::Truncated {
                expected: 8,
                got: 4
            }]
        );
    }

    #[test]
    fn a_failed_block_counts_the_words_read_on_arm_and_one_more_on_dap_rs() {
        let block = [0x06, 0, 3, 0, 0x0F];
        let arm = [0x06, 1, 0, 0x04, 1, 2, 3, 4];
        let dap_rs = [0x06, 2, 0, 0x04, 1, 2, 3, 4];
        assert_eq!(findings(&block, &arm, MIXED), []);
        assert_eq!(findings(&block, &dap_rs, DAP_RS), []);
        assert_eq!(
            findings(&block, &arm, DAP_RS),
            [Finding::Excess {
                expected: 4,
                got: 8
            }]
        );
        let write = [0x06, 0, 2, 0, 0x0D, 1, 2, 3, 4, 5, 6, 7, 8];
        assert_eq!(findings(&write, &[0x06, 1, 0, 0x02], MIXED), []);
        assert_eq!(
            findings(&write, &[0x06, 3, 0, 0x01], ARM),
            [Finding::CountAbove {
                requested: 2,
                counted: 3
            }]
        );
        assert_eq!(
            findings(&block, &[0x06, 0, 0, 0x11], ARM),
            [Finding::MalformedStatus(0x11)]
        );
    }

    #[test]
    fn a_posted_read_may_lose_its_word_on_arm() {
        // Two DRW reads: the second's post collected the first, or failed doing so.
        let request = [0x05, 0, 2, 0x0F, 0x0F];
        assert_eq!(transfer(&request, &[0x05, 1, 0x02], ARM).2, []);
        assert_eq!(
            transfer(&request, &[0x05, 1, 0x02, 9, 0, 0, 0], ARM).2,
            [data(0, 9)]
        );
        // Only JTAG posts DP reads.
        let dp = [0x05, 1, 0x02];
        assert_eq!(transfer(&TWO_DP_READS, &dp, Context::arm(Link::Jtag)).2, []);
        assert_eq!(
            findings(&TWO_DP_READS, &dp, ARM),
            [Finding::Truncated {
                expected: 7,
                got: 3
            }]
        );
    }

    #[test]
    fn timestamps_precede_their_data() {
        let request = [0x05, 0, 2, 0x82, 0x80 | 0x04, 1, 0, 0, 0];
        let reply = [0x05, 2, 0x01, 1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0];
        let stamp = |item, value| Word {
            item,
            timestamp: true,
            value,
        };
        assert_eq!(
            transfer(&request, &reply, ARM).2,
            [stamp(0, 1), data(0, 2), stamp(1, 3)]
        );
        assert_eq!(
            findings(&[0x05, 0, 1, 0x92, 0, 0, 0, 0], &[0x05, 1, 0x01], ARM),
            [Finding::TimestampWithMatch(0)]
        );
    }

    #[test]
    fn empty_transfers_are_outside_the_count_range() {
        assert_eq!(
            findings(&[0x05, 0, 0], &[0x05, 0, 0], ARM),
            [Finding::ZeroCount]
        );
        assert_eq!(
            findings(&[0x06, 0, 0, 0, 0x0F], &[0x06, 0, 0, 0], ARM),
            [Finding::ZeroCount]
        );
    }

    #[test]
    fn ids_unimplemented_commands_and_abort() {
        assert_eq!(findings(&[0x24], &[0xFF], ARM), []);
        assert_eq!(findings(&[0x24], &[0x24, 1, 2], ARM), []);
        assert_eq!(
            findings(&[0x24], &[0xFF, 0], ARM),
            [Finding::Excess {
                expected: 1,
                got: 2
            }]
        );
        assert_eq!(
            findings(&[0x02, 1], &[0x03, 1], ARM),
            [Finding::WrongId(0x03)]
        );
        assert_eq!(findings(&[0x07], &[], ARM), []);
        assert_eq!(findings(&[0x07], &[0x07], ARM), [Finding::ReplyToAbort]);
        assert_eq!(findings(&[0x03], &[], ARM), [Finding::NoReply]);
        assert_eq!(
            findings(&[0x05, 0, 1], &[0x05, 0, 0], ARM),
            [Finding::ShortRequest]
        );
    }

    #[test]
    fn session_and_sequence_replies() {
        assert_eq!(findings(&[0x02, 0], &[0x02, 1], ARM), []);
        assert_eq!(findings(&[0x02, 2], &[0x02, 0], ARM), []);
        assert_eq!(
            findings(&[0x02, 2], &[0x02, 1], ARM),
            [Finding::WrongPort {
                requested: 2,
                answered: 1
            }]
        );
        assert_eq!(findings(&[0x11, 0, 0, 0, 0], &[0x11, 0xFF], ARM), []);
        assert_eq!(
            findings(&[0x11, 0, 0, 0, 0], &[0x11, 1], ARM),
            [Finding::UnknownStatus(1)]
        );
        assert_eq!(
            findings(&[0x10, 0, 0x80, 0, 0, 0, 0], &[0x10, 0x8B], ARM),
            []
        );
        // An 8-bit output, then a 33-bit input.
        let swd = [0x1D, 2, 8, 0xFF, 0x80 | 33];
        assert_eq!(findings(&swd, &[0x1D, 0, 1, 2, 3, 4, 5], ARM), []);
        assert_eq!(findings(&swd, &[0x1D, 0xFF], ARM), []);
        let jtag = [0x14, 2, 0x80 | 3, 0, 0x02, 0];
        assert_eq!(findings(&jtag, &[0x14, 0, 5], ARM), []);
        assert_eq!(
            findings(&[0x16, 0], &[0x16, 0, 0x77, 0x14, 0xA0, 0x4B], ARM),
            []
        );
        assert_eq!(findings(&[0x16, 3], &[0x16, 0xFF], ARM), []);
    }

    #[test]
    fn info_values_have_their_types_lengths() {
        let version = [0x00, 6, b'2', b'.', b'1', b'.', b'1', 0];
        assert_eq!(findings(&[0x00, 0x04], &version, ARM), []);
        assert_eq!(
            findings(&[0x00, 0x04], &[0x00, 5, b'2', b'.', b'1', b'.', b'1'], ARM),
            [Finding::UnterminatedString(0x04)]
        );
        assert_eq!(findings(&[0x00, 0xF0], &[0x00, 2, 0x13, 0x01], ARM), []);
        assert_eq!(findings(&[0x00, 0xFF], &[0x00, 2, 0x00, 0x02], ARM), []);
        assert_eq!(
            findings(&[0x00, 0xFF], &[0x00, 1, 0x40], ARM),
            [Finding::InfoLength { id: 0xFF, len: 1 }]
        );
        assert_eq!(findings(&[0x00, 0x42], &[0x00, 0], ARM), []);
    }

    #[test]
    fn swo_and_uart_replies_size_from_their_counts() {
        assert_eq!(findings(&[0x1C, 4, 0], &[0x1C, 1, 2, 0, 9, 9], ARM), []);
        assert_eq!(
            findings(
                &[0x1E, 0x07],
                &[0x1E, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                ARM
            ),
            []
        );
        assert_eq!(findings(&[0x1E, 0x02], &[0x1E, 0, 0, 0, 0], ARM), []);
        let uart = [0x21, 8, 0, 1, 0, 0x55];
        assert_eq!(findings(&uart, &[0x21, 0, 1, 0, 2, 0, 7, 8], ARM), []);
        assert_eq!(
            findings(&[0x23], &[0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0], ARM),
            []
        );
    }

    #[cfg(feature = "differential")]
    #[test]
    fn every_reference_exchange_checks_clean() {
        use crate::differential::{reference, run};
        use crate::session::{stream, Profile};
        use crate::target::shared;
        let parity = Profile {
            parity: true,
            ..Profile::SWD
        };
        let cases = [
            (Profile::SWD, DAP_RS, false),
            (Profile::SWD, MIXED, true),
            (parity, MIXED, true),
            (Profile::JTAG, Context::arm(Link::Jtag), false),
            (Profile::JTAG_ADIV6, Context::arm(Link::Jtag), false),
            (Profile::JTAG_ABSENT, Context::arm(Link::Jtag), false),
        ];
        for (profile, context, marked) in cases {
            for seed in 0..150 {
                let s = stream(seed, &profile);
                let sim = shared(&s.target);
                let mut dap = reference(&sim);
                if marked {
                    dap.latch.follow(
                        crate::rule::RuleId::CountCompleted,
                        vec![true; s.cmds.len()],
                    );
                }
                let steps = run(&s, &mut dap, &sim);
                for (cmd, step) in s.cmds.iter().zip(&steps[1..]) {
                    let found = findings(&cmd.encode(), &step.response, context);
                    // dap-rs counts an empty SWD block as one transfer.
                    let dap_rs_block = Finding::CountAbove {
                        requested: 0,
                        counted: 1,
                    };
                    let expected: &[Finding] = match cmd.encode()[..] {
                        [0x06, _, 0, 0, ..] if context == DAP_RS => {
                            &[Finding::ZeroCount, dap_rs_block]
                        }
                        [0x05, _, 0, ..] | [0x06, _, 0, 0, ..] => &[Finding::ZeroCount],
                        _ => &[],
                    };
                    assert_eq!(
                        found, expected,
                        "{context:?} seed {seed}: {cmd:?} -> {:02x?}",
                        step.response
                    );
                }
            }
        }
    }
}
