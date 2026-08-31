//! The `V1` control verbs and the ledger line `V1 REPORT` answers with.

use super::{BurstSpec, RxCounters, TxCounters};
use crate::{line, parse_u32};

/// Reply to `V1 START`.
pub const OK_V1_START: &str = "OK V1 START";
/// Reply to `V1 STOP`.
pub const OK_V1_STOPPED: &str = "OK V1 STOPPED";
/// Reply to `V1 RESET`.
pub const OK_V1_RESET: &str = "OK V1 RESET";

/// Buffer a `V1 REPORT` reply needs; the ledger outgrows [`crate::MAX_LINE`].
pub const V1_LINE: usize = 320;

/// Which halves of the run the DUT takes part in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DirMask(u8);

impl DirMask {
    /// The DUT transmits its stream to the host.
    pub const TX: u8 = 1;
    /// The DUT verifies the stream it receives from the host.
    pub const RX: u8 = 2;

    /// Read a mask; 0 arms nothing and is not a run, and no other bits are defined.
    pub const fn new(bits: u8) -> Option<Self> {
        if bits == 0 || bits & !(Self::TX | Self::RX) != 0 {
            None
        } else {
            Some(Self(bits))
        }
    }

    /// The mask as written on the wire.
    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn tx(self) -> bool {
        self.0 & Self::TX != 0
    }

    pub const fn rx(self) -> bool {
        self.0 & Self::RX != 0
    }
}

/// A parsed `V1 START`. Budgets are payload bytes per direction, 0 meaning "until aborted".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct V1Start {
    pub dirs: DirMask,
    pub seed_h2d: u32,
    pub seed_d2h: u32,
    pub interval: u32,
    pub budget_h2d: u32,
    pub budget_d2h: u32,
    /// Shaping for the DUT transmitter.
    pub burst: Option<BurstSpec>,
    /// The DUT transmitter obeys credit grants, starting from 0.
    pub credit: bool,
}

/// A parsed v1 request; a DUT that serves v1 tries [`parse`] before [`crate::parse`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum V1Command {
    /// Arm the run, answering [`OK_V1_START`]; the DUT transmits its whole budget on its own.
    Start(V1Start),
    /// End a run short of its budget: the transmitter closes its stream at the next frame slot.
    Stop,
    /// Report the ledger, answering [`report_response`].
    Report,
    /// Zero the counters and engines, answering [`OK_V1_RESET`].
    Reset,
}

/// The ledger of one DUT: its transmit half, its receive half, and its UART error counters.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct V1Report {
    /// Payload bytes transmitted.
    pub tx: u32,
    /// Frames transmitted.
    pub tx_frames: u32,
    /// Payload bytes received and verified.
    pub verified: u32,
    /// Payload bytes received that differed from the stream.
    pub corrupt: u32,
    /// Payload bytes never received, net.
    pub missing: u32,
    /// Payload bytes received beyond the stream, net.
    pub duplicate: u32,
    /// Payload bytes a resync window could not attribute.
    pub unresolved: u32,
    /// Frames received.
    pub rx_frames: u32,
    /// Resynchronizations entered.
    pub resyncs: u32,
    /// The end-of-stream frame arrived.
    pub eos: bool,
    /// Microseconds from arming the receiver to its last byte.
    pub elapsed_us: u32,
    /// Microseconds from arming the receiver to its first byte.
    pub onset_us: u32,
    /// Longest gap between received bytes, in microseconds.
    pub maxgap_us: u32,
    /// UART framing errors.
    pub ferr: u32,
    /// UART receive overruns.
    pub ovr: u32,
    /// Bytes a full receive ring discarded before software read them.
    pub drop: u32,
    /// Cumulative credit grant the transmitter obeyed.
    pub credit: u32,
}

impl V1Report {
    /// Fold both engine ledgers and the UART counters into the line's `u32` fields, saturating.
    pub fn new(tx: &TxCounters, rx: &RxCounters, ferr: u32, ovr: u32, drop: u32) -> Self {
        Self {
            tx: clamp(tx.payload),
            tx_frames: tx.frames,
            verified: clamp(rx.verified),
            corrupt: clamp(rx.corrupt),
            missing: clamp(rx.missing),
            duplicate: clamp(rx.duplicate),
            unresolved: clamp(rx.unresolved),
            rx_frames: rx.frames,
            resyncs: rx.resyncs,
            eos: rx.eos,
            elapsed_us: rx.elapsed_us,
            onset_us: rx.onset_us,
            maxgap_us: rx.maxgap_us,
            ferr,
            ovr,
            drop,
            credit: clamp(tx.credit),
        }
    }
}

fn clamp(n: u64) -> u32 {
    if n > u64::from(u32::MAX) {
        u32::MAX
    } else {
        n as u32
    }
}

/// Parse one v1 line (without its newline); `None` leaves it to [`crate::parse`].
pub fn parse(line: &str) -> Option<V1Command> {
    let mut tokens = line.trim().split(' ').filter(|t| !t.is_empty());
    if tokens.next()? != "V1" {
        return None;
    }
    let cmd = match tokens.next()? {
        "START" => {
            let mut start = V1Start {
                dirs: DirMask::new(u8::try_from(parse_u32(tokens.next()?)?).ok()?)?,
                seed_h2d: parse_u32(tokens.next()?)?,
                seed_d2h: parse_u32(tokens.next()?)?,
                interval: parse_u32(tokens.next()?)?,
                budget_h2d: parse_u32(tokens.next()?)?,
                budget_d2h: parse_u32(tokens.next()?)?,
                burst: None,
                credit: false,
            };
            while let Some(token) = tokens.next() {
                match token {
                    "BURST" if start.burst.is_none() => {
                        start.burst = Some(BurstSpec {
                            bytes: parse_u32(tokens.next()?)?,
                            idle_ms: parse_u32(tokens.next()?)?,
                        })
                    }
                    "CREDIT" if !start.credit => start.credit = true,
                    _ => return None,
                }
            }
            return Some(V1Command::Start(start));
        }
        "STOP" => V1Command::Stop,
        "REPORT" => V1Command::Report,
        "RESET" => V1Command::Reset,
        _ => return None,
    };
    tokens.next().is_none().then_some(cmd)
}

/// Format the `V1 REPORT` reply into `buf`, newline included.
pub fn report_response<'a>(buf: &'a mut [u8; V1_LINE], report: &V1Report) -> &'a str {
    let V1Report {
        tx,
        tx_frames,
        verified,
        corrupt,
        missing,
        duplicate,
        unresolved,
        rx_frames,
        resyncs,
        eos,
        elapsed_us,
        onset_us,
        maxgap_us,
        ferr,
        ovr,
        drop,
        credit,
    } = *report;
    let eos = u8::from(eos);
    line(
        buf,
        format_args!(
            "OK V1 tx={tx} txf={tx_frames} rxv={verified} rxc={corrupt} rxm={missing} \
             rxd={duplicate} rxu={unresolved} rxf={rx_frames} rsync={resyncs} eos={eos} \
             elapsed_us={elapsed_us} onset_us={onset_us} maxgap_us={maxgap_us} fe={ferr} \
             oe={ovr} credit={credit} drop={drop}"
        ),
    )
}

/// Extract the ledger from a `V1 REPORT` reply, exactly as [`report_response`] wrote it.
pub fn report_reported(reply: &str) -> Option<V1Report> {
    let mut fields = reply.trim().strip_prefix("OK V1 ")?.split(' ');
    let mut take = |name: &str| parse_u32(fields.next()?.strip_prefix(name)?);
    let report = V1Report {
        tx: take("tx=")?,
        tx_frames: take("txf=")?,
        verified: take("rxv=")?,
        corrupt: take("rxc=")?,
        missing: take("rxm=")?,
        duplicate: take("rxd=")?,
        unresolved: take("rxu=")?,
        rx_frames: take("rxf=")?,
        resyncs: take("rsync=")?,
        eos: match take("eos=")? {
            0 => false,
            1 => true,
            _ => return None,
        },
        elapsed_us: take("elapsed_us=")?,
        onset_us: take("onset_us=")?,
        maxgap_us: take("maxgap_us=")?,
        ferr: take("fe=")?,
        ovr: take("oe=")?,
        credit: take("credit=")?,
        // Appended after the field was in service, so an image built before it still parses.
        drop: match fields.next() {
            None => 0,
            Some(field) => parse_u32(field.strip_prefix("drop=")?)?,
        },
    };
    fields.next().is_none().then_some(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MAX_LINE;

    fn start(line: &str) -> V1Start {
        match parse(line) {
            Some(V1Command::Start(start)) => start,
            other => panic!("{line} parsed as {other:?}"),
        }
    }

    const PLAIN: &str = "V1 START 3 1 2 2048 100000 200000";

    #[test]
    fn parses_the_v1_verbs() {
        let plain = start(PLAIN);
        assert_eq!(plain.dirs.bits(), 3);
        assert!(plain.dirs.tx() && plain.dirs.rx());
        assert_eq!(plain.seed_h2d, 1);
        assert_eq!(plain.seed_d2h, 2);
        assert_eq!(plain.interval, 2048);
        assert_eq!(plain.budget_h2d, 100_000);
        assert_eq!(plain.budget_d2h, 200_000);
        assert_eq!(plain.burst, None);
        assert!(!plain.credit);

        let simplex = start("V1 START 1 0 0 64 0 4096");
        assert!(simplex.dirs.tx() && !simplex.dirs.rx());
        // Seed 0 parses, and budget 0 is the run that ends only on abort.
        assert_eq!(simplex.budget_h2d, 0);

        let shaped = start("V1 START 2 1 2 2048 100000 200000 BURST 512 3 CREDIT");
        assert!(!shaped.dirs.tx() && shaped.dirs.rx());
        assert_eq!(
            shaped.burst,
            Some(BurstSpec {
                bytes: 512,
                idle_ms: 3
            })
        );
        assert!(shaped.credit);
        // The two options are independent, in either order.
        assert!(start("V1 START 3 1 2 2048 1 1 CREDIT").credit);
        assert_eq!(
            start("V1 START 3 1 2 2048 1 1 CREDIT BURST 512 3"),
            start("V1 START 3 1 2 2048 1 1 BURST 512 3 CREDIT")
        );

        assert_eq!(parse("V1 STOP"), Some(V1Command::Stop));
        assert_eq!(parse("V1 REPORT"), Some(V1Command::Report));
        assert_eq!(parse("V1 RESET"), Some(V1Command::Reset));
    }

    #[test]
    fn rejects_malformed_lines() {
        assert_eq!(parse(""), None);
        assert_eq!(parse("V1"), None);
        assert_eq!(parse("V1 BOGUS"), None);
        assert_eq!(parse("V1 START"), None);
        assert_eq!(parse("V1 START 3 1 2 2048 100000"), None);
        assert_eq!(parse("V1 START 0 1 2 2048 1 1"), None); // arms nothing
        assert_eq!(parse("V1 START 4 1 2 2048 1 1"), None); // undefined bit
        assert_eq!(parse("V1 START 3 1 2 2048 1 abc"), None);
        assert_eq!(parse("V1 START 3 1 2 2048 1 4294967296"), None);
        assert_eq!(parse("V1 START 3 1 2 2048 1 1 BURST"), None);
        assert_eq!(parse("V1 START 3 1 2 2048 1 1 BURST 512"), None);
        assert_eq!(
            parse("V1 START 3 1 2 2048 1 1 BURST 512 3 BURST 512 3"),
            None
        );
        assert_eq!(parse("V1 START 3 1 2 2048 1 1 CREDIT CREDIT"), None);
        assert_eq!(parse("V1 START 3 1 2 2048 1 1 BOGUS"), None);
        assert_eq!(parse("V1 STOP now"), None);
        assert_eq!(parse("V1 REPORT 1"), None);
        assert_eq!(parse("V1 RESET 1"), None);
        // Other verb families are not v1 lines.
        assert_eq!(parse("SWO STREAM 1"), None);
    }

    #[test]
    fn the_parsers_partition_the_verb_space() {
        for line in [
            PLAIN,
            "V1 STOP",
            "V1 REPORT",
            "V1 RESET",
            "PING",
            "SWO STREAM 1",
        ] {
            let families = u8::from(parse(line).is_some()) + u8::from(crate::parse(line).is_some());
            assert!(families <= 1, "{line} parses in more than one verb family");
        }
        // v1 lines fall through the base parser, so a DUT without v1 answers `ERR unimplemented`.
        assert_eq!(crate::parse(PLAIN), None);
        assert_eq!(crate::parse("V1 REPORT"), None);
    }

    #[test]
    fn the_widest_start_line_fits_a_frame() {
        let line = "V1 START 3 4294967295 4294967295 4294967295 4294967295 4294967295 \
                    BURST 4294967295 4294967295 CREDIT";
        assert!(parse(line).is_some());
        // The newline the DUT frames it with has to fit too.
        assert!(line.len() < MAX_LINE);
    }

    // --- reply vectors the tool string-matches against, pinned exactly ---

    #[test]
    fn the_acknowledgements_are_the_literals_the_tool_waits_for() {
        assert_eq!(OK_V1_START, "OK V1 START");
        assert_eq!(OK_V1_STOPPED, "OK V1 STOPPED");
        assert_eq!(OK_V1_RESET, "OK V1 RESET");
    }

    #[test]
    fn report_response_formats_the_whole_ledger() {
        let report = V1Report {
            tx: 200,
            tx_frames: 4,
            verified: 199,
            corrupt: 1,
            missing: 0,
            duplicate: 0,
            unresolved: 0,
            rx_frames: 4,
            resyncs: 0,
            eos: true,
            elapsed_us: 60_000_000,
            onset_us: 120,
            maxgap_us: 900,
            ferr: 0,
            ovr: 0,
            drop: 0,
            credit: 0,
        };
        let mut buf = [0u8; V1_LINE];
        assert_eq!(
            report_response(&mut buf, &report),
            "OK V1 tx=200 txf=4 rxv=199 rxc=1 rxm=0 rxd=0 rxu=0 rxf=4 rsync=0 eos=1 \
             elapsed_us=60000000 onset_us=120 maxgap_us=900 fe=0 oe=0 credit=0 drop=0\n"
        );
        let mut buf = [0u8; V1_LINE];
        assert_eq!(
            report_response(&mut buf, &V1Report::default()),
            "OK V1 tx=0 txf=0 rxv=0 rxc=0 rxm=0 rxd=0 rxu=0 rxf=0 rsync=0 eos=0 elapsed_us=0 \
             onset_us=0 maxgap_us=0 fe=0 oe=0 credit=0 drop=0\n"
        );
    }

    #[test]
    fn report_response_holds_saturated_counters() {
        let saturated = V1Report {
            tx: u32::MAX,
            tx_frames: u32::MAX,
            verified: u32::MAX,
            corrupt: u32::MAX,
            missing: u32::MAX,
            duplicate: u32::MAX,
            unresolved: u32::MAX,
            rx_frames: u32::MAX,
            resyncs: u32::MAX,
            eos: true,
            elapsed_us: u32::MAX,
            onset_us: u32::MAX,
            maxgap_us: u32::MAX,
            ferr: u32::MAX,
            ovr: u32::MAX,
            drop: 0,
            credit: u32::MAX,
        };
        let mut buf = [0u8; V1_LINE];
        let reply = report_response(&mut buf, &saturated);
        assert!(reply.len() <= V1_LINE, "{} bytes", reply.len());
        assert_eq!(report_reported(reply), Some(saturated));
    }

    #[test]
    fn report_reported_reads_back_what_report_response_wrote() {
        let report = V1Report {
            tx: 1_000_000,
            tx_frames: 489,
            verified: 999_872,
            corrupt: 0,
            missing: 1,
            duplicate: 0,
            unresolved: 127,
            rx_frames: 488,
            resyncs: 1,
            eos: true,
            elapsed_us: 3_600_000,
            onset_us: 45,
            maxgap_us: 1_200,
            ferr: 2,
            ovr: 3,
            drop: 0,
            credit: 8192,
        };
        let mut buf = [0u8; V1_LINE];
        let reply = report_response(&mut buf, &report);
        assert_eq!(report_reported(reply), Some(report));
        let mut buf = [0u8; V1_LINE];
        let zero = report_response(&mut buf, &V1Report::default());
        assert_eq!(report_reported(zero), Some(V1Report::default()));
    }

    #[test]
    fn report_reported_rejects_reordered_missing_or_trailing_fields() {
        const GOOD: &str = "OK V1 tx=0 txf=0 rxv=0 rxc=0 rxm=0 rxd=0 rxu=0 rxf=0 rsync=0 eos=0 \
                            elapsed_us=0 onset_us=0 maxgap_us=0 fe=0 oe=0 credit=0";
        assert!(report_reported(GOOD).is_some());
        // Truncated, reordered, extended, and out-of-range fields are all refused.
        assert_eq!(
            report_reported(
                "OK V1 tx=0 txf=0 rxv=0 rxc=0 rxm=0 rxd=0 rxu=0 rxf=0 rsync=0 eos=0 \
                 elapsed_us=0 onset_us=0 maxgap_us=0 fe=0 oe=0"
            ),
            None
        );
        assert_eq!(
            report_reported(
                "OK V1 txf=0 tx=0 rxv=0 rxc=0 rxm=0 rxd=0 rxu=0 rxf=0 rsync=0 eos=0 \
                 elapsed_us=0 onset_us=0 maxgap_us=0 fe=0 oe=0 credit=0"
            ),
            None
        );
        assert_eq!(
            report_reported(
                "OK V1 tx=0 txf=0 rxv=0 rxc=0 rxm=0 rxd=0 rxu=0 rxf=0 rsync=0 eos=0 \
                 elapsed_us=0 onset_us=0 maxgap_us=0 fe=0 oe=0 credit=0 x=1"
            ),
            None
        );
        assert_eq!(
            report_reported(
                "OK V1 tx=0 txf=0 rxv=0 rxc=0 rxm=0 rxd=0 rxu=0 rxf=0 rsync=0 eos=2 \
                 elapsed_us=0 onset_us=0 maxgap_us=0 fe=0 oe=0 credit=0"
            ),
            None
        );
        assert_eq!(report_reported(OK_V1_START), None);
        assert_eq!(
            report_reported("OK STATS rx=1 tx=1 ferr=0 ovr=0 drop=0"),
            None
        );
    }

    #[test]
    fn a_report_folds_both_engine_ledgers_and_saturates_wide_counters() {
        let tx = TxCounters {
            payload: 1 << 33,
            frames: 12,
            credit: 4096,
            complete: true,
        };
        let rx = RxCounters {
            verified: 100,
            corrupt: 1,
            missing: 2,
            duplicate: 3,
            unresolved: 4,
            frames: 5,
            resyncs: 6,
            eos: true,
            eos_total: 107,
            credit: 4096,
            elapsed_us: 7,
            onset_us: 8,
            maxgap_us: 9,
        };
        let report = V1Report::new(&tx, &rx, 1, 2, 3);
        assert_eq!(report.tx, u32::MAX);
        assert_eq!(report.credit, 4096);
        assert_eq!(report.verified, 100);
        assert_eq!(report.unresolved, 4);
        assert_eq!(report.resyncs, 6);
        assert!(report.eos);
        assert_eq!((report.ferr, report.ovr), (1, 2));
    }
}
