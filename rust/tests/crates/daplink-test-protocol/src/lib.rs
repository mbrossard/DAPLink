//! ASCII, newline-framed line protocol over RTT between the host runner and the DUT firmware.

#![no_std]

use core::fmt::Write;

pub mod prng;
pub mod v1;

/// Maximum framed line length in bytes, including the trailing `\n`.
pub const MAX_LINE: usize = 128;

/// Protocol revision reported in `ID`; bump on a wire-format change.
pub const PROTO_VERSION: u8 = 8;

/// Source revision of the DUT image, reported in `ID`: `g<short hash>`, `-dirty`, or `unknown`.
/// [`PROTO_VERSION`] says what the wire is; this says what the build is, and they move apart.
pub const BUILD_REVISION: &str = daplink_core::DAP_FW_REVISION;

/// Fixed head of the payload the DUT emits on ITM stimulus port 0; the emission tag follows it.
/// Shared so the DUT emitter and the host capture scanner agree on the framed byte sequence.
pub const SWO_PAYLOAD_HEAD: [u8; 7] = [0xA5, 0x5A, 0xC3, 0x3C, 0xF0, 0x0F, 0x99];

/// Total bytes per SWO emission: [`SWO_PAYLOAD_HEAD`] plus the one-byte ordinal tag.
pub const SWO_PAYLOAD_LEN: usize = SWO_PAYLOAD_HEAD.len() + 1;

/// Magic word heading every record of the continuous SWO stream (`SWO STREAM 1`).
pub const SWO_STREAM_MAGIC: u32 = 0x3CC3_A55A;

/// Bytes per streamed record: [`SWO_STREAM_MAGIC`] then the sequence number, both u32 LE.
pub const SWO_STREAM_RECORD_LEN: usize = 8;

/// Boot event line prefix; the DUT appends its name.
pub const EVT_BOOT: &str = "EVT BOOT";
/// Reply to `PING`.
pub const OK_PONG: &str = "OK PONG";
/// Reply to `STATS RESET`.
pub const OK_STATS_RESET: &str = "OK STATS RESET";
/// Reply to anything unparseable, and to a request the DUT declines to serve.
pub const ERR_UNIMPLEMENTED: &str = "ERR unimplemented";

/// The DUT's echo-UART counters, all wrapping on overflow.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// Bytes received on the echo UART.
    pub rx: u32,
    /// Bytes transmitted back.
    pub tx: u32,
    /// Framing errors.
    pub ferr: u32,
    /// Receive overruns.
    pub ovr: u32,
    /// Bytes dropped by a full receive ring; 0 on firmwares without a ring.
    pub drop: u32,
    /// Abort waits that expired with the DMA channel still busy; sticky, and `STATS RESET` does
    /// not clear it, so counters taken after it goes nonzero are suspect.
    pub stuck: u32,
}

/// A parsed host-to-target request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    /// Liveness probe; answered with [`OK_PONG`].
    Ping,
    /// Identity query; answered by [`id_response`].
    Id,
    /// Set the in-RAM stamp; a reset zeroes it, which is what a reset test checks.
    Stamp(u32),
    /// Reprogram the DUT's echo UART baud; answered by [`baud_response`], or [`ERR_UNIMPLEMENTED`] if declined.
    Baud(u32),
    /// Highest rate the DUT's policy accepts for [`Command::Baud`]; answered by
    /// [`baud_max_response`].
    BaudMax,
    /// Read the echo-UART counters; answered by [`stats_response`].
    Stats,
    /// Zero the echo-UART counters; answered by [`OK_STATS_RESET`].
    StatsReset,
    /// Emit the tagged SWO payload on ITM stimulus port 0; answered by [`swo_emit_response`].
    SwoEmit(u8),
    /// Reprogram the DUT's TPIU SWO baud; answered by [`swo_cfg_response`], or
    /// [`ERR_UNIMPLEMENTED`] if the DUT rejects the rate.
    SwoCfg(u32),
    /// Start (`true`) or stop (`false`) the continuous sequence-tagged ITM stream; answered by
    /// [`swo_stream_response`].
    SwoStream(bool),
}

/// Parse one line (without its newline); `None` is answered with [`ERR_UNIMPLEMENTED`].
pub fn parse(line: &str) -> Option<Command> {
    let mut tokens = line.trim().split(' ').filter(|t| !t.is_empty());
    let cmd = match tokens.next()? {
        "PING" => Command::Ping,
        "ID" => Command::Id,
        "STAMP" => Command::Stamp(parse_u32(tokens.next()?)?),
        "BAUD" => match tokens.next()? {
            "MAX" => Command::BaudMax,
            rate => Command::Baud(parse_u32(rate)?),
        },
        "STATS" => match tokens.next() {
            None => Command::Stats,
            Some("RESET") => Command::StatsReset,
            Some(_) => return None,
        },
        "SWO" => match tokens.next()? {
            "EMIT" => {
                let tag = parse_u32(tokens.next()?)?;
                if tag > u8::MAX as u32 {
                    return None;
                }
                Command::SwoEmit(tag as u8)
            }
            "CFG" => Command::SwoCfg(parse_u32(tokens.next()?)?),
            "STREAM" => match parse_u32(tokens.next()?)? {
                0 => Command::SwoStream(false),
                1 => Command::SwoStream(true),
                _ => return None,
            },
            _ => return None,
        },
        _ => return None,
    };
    tokens.next().is_none().then_some(cmd)
}

pub(crate) fn parse_u32(s: &str) -> Option<u32> {
    if s.is_empty() {
        return None;
    }
    s.bytes().try_fold(0u32, |acc, b| match b {
        b'0'..=b'9' => acc.checked_mul(10)?.checked_add(u32::from(b - b'0')),
        _ => None,
    })
}

/// Format the `ID` reply into `buf`, newline included.
pub fn id_response<'a>(buf: &'a mut [u8; MAX_LINE], name: &str, stamp: u32) -> &'a str {
    line(
        buf,
        format_args!("OK ID name={name} proto={PROTO_VERSION} stamp={stamp} rev={BUILD_REVISION}"),
    )
}

/// Format the `STAMP` acknowledgement into `buf`, newline included.
pub fn stamp_response(buf: &mut [u8; MAX_LINE], stamp: u32) -> &str {
    line(buf, format_args!("OK STAMP {stamp}"))
}

/// Format the `BAUD` acknowledgement into `buf`, newline included; `realized` is the DUT's actual rate.
pub fn baud_response(buf: &mut [u8; MAX_LINE], realized: u32) -> &str {
    line(buf, format_args!("OK BAUD {realized}"))
}

/// Format the `BAUD MAX` reply into `buf`, newline included.
pub fn baud_max_response(buf: &mut [u8; MAX_LINE], max: u32) -> &str {
    line(buf, format_args!("OK BAUDMAX {max}"))
}

/// Format the `STATS` reply into `buf`, newline included.
pub fn stats_response<'a>(buf: &'a mut [u8; MAX_LINE], stats: &Stats) -> &'a str {
    let Stats {
        rx,
        tx,
        ferr,
        ovr,
        drop,
        stuck,
    } = *stats;
    line(
        buf,
        format_args!("OK STATS rx={rx} tx={tx} ferr={ferr} ovr={ovr} drop={drop} stuck={stuck}"),
    )
}

/// Format the `SWO EMIT` acknowledgement into `buf`, newline included.
pub fn swo_emit_response(buf: &mut [u8; MAX_LINE], tag: u8) -> &str {
    line(buf, format_args!("OK SWO EMIT {tag}"))
}

/// Format the `SWO CFG` acknowledgement into `buf`, newline included; `realized` is the rate the
/// DUT's trace-clock divisor actually produces, which need not be the rate that was asked for.
pub fn swo_cfg_response(buf: &mut [u8; MAX_LINE], realized: u32) -> &str {
    line(buf, format_args!("OK SWO CFG {realized}"))
}

/// Format the `SWO STREAM` acknowledgement into `buf`, newline included.
pub fn swo_stream_response(buf: &mut [u8; MAX_LINE], on: bool) -> &str {
    line(buf, format_args!("OK SWO STREAM {}", u8::from(on)))
}

/// Extract the realized rate from a `BAUD` reply line; `BAUD MAX` replies do not match.
pub fn baud_realized(reply: &str) -> Option<u32> {
    parse_u32(reply.trim().strip_prefix("OK BAUD ")?.trim())
}

/// Extract the realized trace rate from a `SWO CFG` reply line.
pub fn swo_cfg_realized(reply: &str) -> Option<u32> {
    parse_u32(reply.trim().strip_prefix("OK SWO CFG ")?.trim())
}

/// Extract the policy ceiling from a `BAUD MAX` reply line; `BAUD` replies do not match.
pub fn baud_max_reported(reply: &str) -> Option<u32> {
    parse_u32(reply.trim().strip_prefix("OK BAUDMAX ")?.trim())
}

/// Extract the counters from a `STATS` reply line, exactly as [`stats_response`] wrote them.
pub fn stats_reported(reply: &str) -> Option<Stats> {
    let mut fields = reply.trim().strip_prefix("OK STATS ")?.split(' ');
    let mut take = |name: &str| parse_u32(fields.next()?.strip_prefix(name)?);
    let stats = Stats {
        rx: take("rx=")?,
        tx: take("tx=")?,
        ferr: take("ferr=")?,
        ovr: take("ovr=")?,
        drop: take("drop=")?,
        stuck: take("stuck=")?,
    };
    fields.next().is_none().then_some(stats)
}

/// Extract the `rev=<revision>` field from an `ID` reply; `None` for an image built before the
/// field existed.
pub fn id_revision(reply: &str) -> Option<&str> {
    reply
        .split(' ')
        .find_map(|t| t.trim().strip_prefix("rev="))
        .filter(|rev| !rev.is_empty())
}

/// Extract the `stamp=<n>` field from an `ID` reply.
pub fn id_stamp(reply: &str) -> Option<u32> {
    let field = reply
        .split(' ')
        .find_map(|t| t.trim().strip_prefix("stamp="))?;
    parse_u32(field)
}

pub(crate) fn line<'a>(buf: &'a mut [u8], args: core::fmt::Arguments) -> &'a str {
    struct Cursor<'b>(&'b mut [u8], usize);
    impl Write for Cursor<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let end = self.1 + s.len();
            if end > self.0.len() {
                return Err(core::fmt::Error);
            }
            self.0[self.1..end].copy_from_slice(s.as_bytes());
            self.1 = end;
            Ok(())
        }
    }
    let mut cur = Cursor(&mut buf[..], 0);
    let _ = cur.write_fmt(args);
    let _ = cur.write_str("\n");
    let len = cur.1;
    // The cursor only ever writes UTF-8.
    core::str::from_utf8(&buf[..len]).unwrap_or("")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_known_verbs() {
        assert_eq!(parse("PING"), Some(Command::Ping));
        assert_eq!(parse("ID"), Some(Command::Id));
        assert_eq!(parse("STAMP 7"), Some(Command::Stamp(7)));
        assert_eq!(parse("BAUD 115200"), Some(Command::Baud(115_200)));
        // A rate the DUT will decline still parses; the policy check is the DUT's.
        assert_eq!(parse("BAUD 0"), Some(Command::Baud(0)));
        assert_eq!(parse("BAUD MAX"), Some(Command::BaudMax));
        assert_eq!(parse("STATS"), Some(Command::Stats));
        assert_eq!(parse("STATS RESET"), Some(Command::StatsReset));
        assert_eq!(parse("SWO EMIT 5"), Some(Command::SwoEmit(5)));
        assert_eq!(parse("SWO CFG 1000000"), Some(Command::SwoCfg(1_000_000)));
        assert_eq!(parse("SWO STREAM 1"), Some(Command::SwoStream(true)));
        assert_eq!(parse("SWO STREAM 0"), Some(Command::SwoStream(false)));
    }

    #[test]
    fn rejects_malformed_lines() {
        assert_eq!(parse(""), None);
        assert_eq!(parse("STAMP"), None);
        assert_eq!(parse("BAUD"), None);
        assert_eq!(parse("BAUD abc"), None);
        assert_eq!(parse("BAUD MAX 1"), None);
        assert_eq!(parse("BAUD MAXIMUM"), None);
        assert_eq!(parse("STATS RESET now"), None);
        assert_eq!(parse("STATS BOGUS"), None);
        assert_eq!(parse("SWO"), None);
        assert_eq!(parse("SWO EMIT 256"), None); // out of u8 range
        assert_eq!(parse("SWO BOGUS 1"), None);
        assert_eq!(parse("SWO CFG"), None);
        assert_eq!(parse("SWO CFG abc"), None);
        assert_eq!(parse("SWO STREAM"), None);
        assert_eq!(parse("SWO STREAM 2"), None); // only 0/1 are defined
        assert_eq!(parse("SWO STREAM on"), None);
        assert_eq!(parse("SWO STREAM 1 extra"), None);
        assert_eq!(parse("PING extra"), None);
    }

    #[test]
    fn swo_cfg_response_formats_baud() {
        let mut buf = [0u8; MAX_LINE];
        assert_eq!(
            swo_cfg_response(&mut buf, 2_000_000),
            "OK SWO CFG 2000000\n"
        );
    }

    #[test]
    fn swo_cfg_realized_reads_back_what_swo_cfg_response_wrote() {
        let mut buf = [0u8; MAX_LINE];
        let reply = swo_cfg_response(&mut buf, 115_226);
        assert_eq!(swo_cfg_realized(reply), Some(115_226));
        assert_eq!(swo_cfg_realized("OK SWO EMIT 3"), None);
    }

    #[test]
    fn swo_stream_response_formats_both_states() {
        let mut buf = [0u8; MAX_LINE];
        assert_eq!(swo_stream_response(&mut buf, true), "OK SWO STREAM 1\n");
        let mut buf = [0u8; MAX_LINE];
        assert_eq!(swo_stream_response(&mut buf, false), "OK SWO STREAM 0\n");
    }

    // --- reply vectors the harness string-matches against, pinned exactly ---

    #[test]
    fn ok_pong_is_the_literal_the_rtt_case_waits_for() {
        // rtt.rs's PING exchange looks for this exact substring in the up-channel accumulator.
        assert_eq!(OK_PONG, "OK PONG");
    }

    #[test]
    fn evt_boot_is_the_literal_the_boot_marker_scans_for() {
        // rtt.rs/nreset.rs/uf2.rs all wait for this substring after a reset.
        assert_eq!(EVT_BOOT, "EVT BOOT");
    }

    #[test]
    fn id_response_carries_the_ok_id_prefix_and_stamp_field() {
        let mut buf = [0u8; MAX_LINE];
        let reply = id_response(&mut buf, "rp2040", 42);
        // nreset.rs matches "OK ID" as a prefix substring, then parses the stamp back out.
        assert!(
            reply.starts_with("OK ID name=rp2040 proto=8 stamp=42 rev="),
            "PROTO_VERSION changed; update this vector (and PROTO_VERSION consumers) together: {reply}"
        );
        assert!(reply.ends_with('\n'));
    }

    #[test]
    fn id_stamp_round_trips_through_id_response() {
        let mut buf = [0u8; MAX_LINE];
        let reply = id_response(&mut buf, "rp2040", 42);
        assert_eq!(id_stamp(reply), Some(42));
    }

    #[test]
    fn id_revision_round_trips_through_id_response() {
        let mut buf = [0u8; MAX_LINE];
        let reply = id_response(&mut buf, "rp2040", 42);
        assert_eq!(id_revision(reply), Some(BUILD_REVISION));
    }

    /// An image built before the field existed still parses; the revision is simply unknown.
    #[test]
    fn id_revision_is_absent_from_a_stampless_reply() {
        assert_eq!(id_revision("OK ID name=rp2040 proto=7 stamp=42"), None);
        assert_eq!(id_stamp("OK ID name=rp2040 proto=7 stamp=42"), Some(42));
    }

    /// The stamp says what the build is; `proto` says what the wire is, and neither substitutes.
    #[test]
    fn build_revision_is_not_the_protocol_version() {
        assert!(!BUILD_REVISION.is_empty());
        assert!(BUILD_REVISION.starts_with('g') || BUILD_REVISION == "unknown");
    }

    /// The widest `ID` line the longest DUT name can produce still fits a frame.
    #[test]
    fn id_response_fits_a_frame_at_its_widest() {
        let mut buf = [0u8; MAX_LINE];
        let reply = id_response(&mut buf, "test-stm32f401", u32::MAX);
        assert!(reply.len() <= MAX_LINE, "{} bytes", reply.len());
        assert_eq!(id_revision(reply), Some(BUILD_REVISION));
    }

    #[test]
    fn stamp_response_matches_the_nreset_case_exchange() {
        let mut buf = [0u8; MAX_LINE];
        // nreset.rs exchanges `STAMP 42\n` and matches this exact string back.
        assert_eq!(stamp_response(&mut buf, 42), "OK STAMP 42\n");
    }

    #[test]
    fn baud_response_matches_the_vcom_case_marker() {
        let mut buf = [0u8; MAX_LINE];
        // vcom.rs matches the "OK BAUD" prefix, then parses the realized rate back out.
        assert_eq!(baud_response(&mut buf, 115_200), "OK BAUD 115200\n");
        let mut buf = [0u8; MAX_LINE];
        // The realized rate is the DUT's, not an echo of the request.
        assert_eq!(baud_response(&mut buf, 921_649), "OK BAUD 921649\n");
    }

    #[test]
    fn baud_realized_reads_back_what_baud_response_wrote() {
        let mut buf = [0u8; MAX_LINE];
        let reply = baud_response(&mut buf, 921_649);
        assert_eq!(baud_realized(reply), Some(921_649));
        // The `BAUD MAX` reply is a different verb and must not be read as a realized rate.
        let mut buf = [0u8; MAX_LINE];
        let max = baud_max_response(&mut buf, 2_625_000);
        assert_eq!(baud_realized(max), None);
        assert_eq!(baud_realized("OK BAUD"), None);
    }

    #[test]
    fn baud_max_response_formats_the_policy_ceiling() {
        let mut buf = [0u8; MAX_LINE];
        assert_eq!(
            baud_max_response(&mut buf, 2_625_000),
            "OK BAUDMAX 2625000\n"
        );
    }

    #[test]
    fn baud_max_reported_reads_back_what_baud_max_response_wrote() {
        let mut buf = [0u8; MAX_LINE];
        let reply = baud_max_response(&mut buf, 2_625_000);
        assert_eq!(baud_max_reported(reply), Some(2_625_000));
        // A `BAUD` reply is a different verb and must not be read as a ceiling.
        let mut buf = [0u8; MAX_LINE];
        let baud = baud_response(&mut buf, 115_200);
        assert_eq!(baud_max_reported(baud), None);
        assert_eq!(baud_max_reported("OK BAUDMAX"), None);
    }

    #[test]
    fn stats_reported_reads_back_what_stats_response_wrote() {
        let stats = Stats {
            rx: 4096,
            tx: 4095,
            ferr: 1,
            ovr: 2,
            drop: 3,
            stuck: 4,
        };
        let mut buf = [0u8; MAX_LINE];
        let reply = stats_response(&mut buf, &stats);
        assert_eq!(stats_reported(reply), Some(stats));
        let mut buf = [0u8; MAX_LINE];
        let zero = stats_response(&mut buf, &Stats::default());
        assert_eq!(stats_reported(zero), Some(Stats::default()));
    }

    #[test]
    fn stats_reported_rejects_reordered_missing_or_trailing_fields() {
        assert_eq!(
            stats_reported("OK STATS tx=1 rx=1 ferr=0 ovr=0 drop=0 stuck=0"),
            None
        );
        assert_eq!(
            stats_reported("OK STATS rx=1 tx=1 ferr=0 ovr=0 drop=0"),
            None
        );
        assert_eq!(
            stats_reported("OK STATS rx=1 tx=1 ferr=0 ovr=0 drop=0 stuck=0 x=1"),
            None
        );
        assert_eq!(stats_reported("OK STATS RESET"), None);
        assert_eq!(stats_reported("OK BAUD 9600"), None);
    }

    #[test]
    fn stats_response_formats_all_six_counters() {
        let mut buf = [0u8; MAX_LINE];
        let stats = Stats {
            rx: 4096,
            tx: 4096,
            ferr: 1,
            ovr: 2,
            drop: 3,
            stuck: 4,
        };
        assert_eq!(
            stats_response(&mut buf, &stats),
            "OK STATS rx=4096 tx=4096 ferr=1 ovr=2 drop=3 stuck=4\n"
        );
        let mut buf = [0u8; MAX_LINE];
        assert_eq!(
            stats_response(&mut buf, &Stats::default()),
            "OK STATS rx=0 tx=0 ferr=0 ovr=0 drop=0 stuck=0\n"
        );
    }

    #[test]
    fn stats_response_holds_wrapped_counters() {
        let mut buf = [0u8; MAX_LINE];
        let stats = Stats {
            rx: u32::MAX,
            tx: u32::MAX,
            ferr: u32::MAX,
            ovr: u32::MAX,
            drop: u32::MAX,
            stuck: u32::MAX,
        };
        // The widest line the verb can produce still fits a frame.
        let reply = stats_response(&mut buf, &stats);
        assert_eq!(
            reply,
            "OK STATS rx=4294967295 tx=4294967295 ferr=4294967295 ovr=4294967295 \
             drop=4294967295 stuck=4294967295\n"
        );
        assert!(reply.len() <= MAX_LINE);
    }

    #[test]
    fn ok_stats_reset_is_the_literal_answering_stats_reset() {
        assert_eq!(OK_STATS_RESET, "OK STATS RESET");
    }

    #[test]
    fn err_unimplemented_is_the_literal_answering_a_declined_baud() {
        // A DUT that will not program the requested rate answers with this, as `SWO CFG` does.
        assert_eq!(ERR_UNIMPLEMENTED, "ERR unimplemented");
    }

    #[test]
    fn swo_emit_response_matches_the_swo_case_marker() {
        let mut buf = [0u8; MAX_LINE];
        // swo.rs formats `format!("OK SWO EMIT {tag}")` and matches it against this reply.
        assert_eq!(swo_emit_response(&mut buf, 5), "OK SWO EMIT 5\n");
    }
}
