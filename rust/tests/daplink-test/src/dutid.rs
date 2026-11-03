//! DUT build identity: one `ID` exchange over RTT, read once per run for the header and record.

use std::time::Duration;

use daplink_test_protocol as proto;
use probe_rs::probe::list::Lister;
use probe_rs::Permissions;

use crate::util::chain;
use crate::Ctx;

/// Bound on the RTT attach, so a DUT with no image costs a moment.
const ATTACH_DEADLINE: Duration = Duration::from_secs(2);

/// What the DUT reported about itself; every field is optional.
#[derive(Debug, Default, PartialEq, Eq)]
pub(crate) struct DutId {
    pub(crate) name: Option<String>,
    pub(crate) proto: Option<u32>,
    pub(crate) revision: Option<String>,
}

impl DutId {
    /// The header line's rendering of the identity.
    pub(crate) fn describe(&self) -> String {
        let name = self.name.as_deref().unwrap_or("?");
        let proto = self
            .proto
            .map_or_else(|| "?".to_string(), |p| p.to_string());
        let rev = self.revision.as_deref().unwrap_or("unreported");
        format!("{name} proto={proto} rev={rev}")
    }
}

/// Parse an `ID` reply out of the accumulated RTT text, taking the last line that is one.
pub(crate) fn parse(text: &str) -> Option<DutId> {
    text.lines()
        .rev()
        .find(|l| l.contains("OK ID"))
        .map(|line| DutId {
            name: proto::id_name(line).map(str::to_string),
            proto: proto::id_proto(line),
            revision: proto::id_revision(line).map(str::to_string),
        })
}

/// The `stamp=` field of the last `ID` reply in `text`.
pub(crate) fn last_stamp(text: &str) -> Option<u32> {
    text.lines().rev().find_map(proto::id_stamp)
}

/// Ask the DUT who it is over its own session; any failure records the identity as absent.
pub(crate) fn read(cx: &Ctx) -> Result<DutId, String> {
    let probe = Lister::new().open(cx.selector.clone()).map_err(chain)?;
    let mut session = probe
        .attach(&cx.chip, Permissions::default())
        .map_err(chain)?;
    let mut core = session.core(0).map_err(chain)?;
    let mut rtt = crate::rtt::attach_within(&mut core, cx.dut_elf.as_deref(), ATTACH_DEADLINE)?;
    let seen = crate::rtt::exchange(&mut rtt, &mut core, b"ID\n", "OK ID")?;
    parse(&seen).ok_or_else(|| format!("no ID reply in {seen:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reply the current DUT images write parses whole.
    #[test]
    fn a_current_reply_yields_every_field() {
        let id = parse("EVT BOOT stm32f401\nOK ID name=stm32f401 proto=8 stamp=0 rev=gabc123\n")
            .expect("an ID line is present");
        assert_eq!(id.name.as_deref(), Some("stm32f401"));
        assert_eq!(id.proto, Some(8));
        assert_eq!(id.revision.as_deref(), Some("gabc123"));
        assert_eq!(id.describe(), "stm32f401 proto=8 rev=gabc123");
    }

    /// An image predating `rev=` parses with a null revision.
    #[test]
    fn an_older_reply_leaves_the_revision_absent() {
        let id = parse("OK ID name=rp2040 proto=7 stamp=42").expect("an ID line is present");
        assert_eq!(id.revision, None);
        assert_eq!(id.describe(), "rp2040 proto=7 rev=unreported");
    }

    /// The last reply wins: a run's accumulated text carries every earlier exchange too.
    #[test]
    fn the_last_id_line_wins() {
        let id = parse("OK ID name=a proto=1 rev=g1\nOK ID name=b proto=2 rev=g2\n")
            .expect("an ID line is present");
        assert_eq!(id.name.as_deref(), Some("b"));
    }

    /// A DUT that never answered is absent, not a default-valued identity.
    #[test]
    fn text_without_a_reply_parses_to_nothing() {
        assert_eq!(parse("ERR unimplemented\n"), None);
        assert_eq!(parse(""), None);
    }
}
