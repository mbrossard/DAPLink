//! Preparation and counter brackets around one measurement or a same-rate batch.

use crate::counters::{Conservation, VcomCounters};
use anyhow::Result;

pub(super) trait Window {
    type Output;
    fn prepare(&mut self) -> Result<()>;
    fn baseline(&mut self) -> Result<Option<VcomCounters>>;
    fn measure(&mut self) -> Result<Self::Output>;
    fn release(&mut self, failed: bool);
    fn after(&mut self) -> Result<Option<VcomCounters>>;
}

pub(super) fn run<W: Window>(window: &mut W) -> (Result<W::Output>, Result<Option<Conservation>>) {
    let mut before = None;
    let outcome = (|| {
        window.prepare()?;
        before = window.baseline()?;
        window.measure()
    })();
    window.release(outcome.is_err());
    let after = window.after();
    let conservation = after.map(|after| {
        before
            .zip(after)
            .map(|(before, after)| Conservation { before, after })
    });
    (outcome, conservation)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Trace {
        events: Vec<&'static str>,
        fail: Option<&'static str>,
    }
    impl Trace {
        fn event(&mut self, name: &'static str) -> Result<()> {
            self.events.push(name);
            anyhow::ensure!(self.fail != Some(name), name);
            Ok(())
        }
    }
    impl Window for Trace {
        type Output = ();
        fn prepare(&mut self) -> Result<()> {
            self.event("prepare")
        }
        fn baseline(&mut self) -> Result<Option<VcomCounters>> {
            self.event("baseline")?;
            Ok(Some(VcomCounters {
                uart_rx: 100,
                ..VcomCounters::default()
            }))
        }
        fn measure(&mut self) -> Result<()> {
            self.event("arm")?;
            self.event("stream")
        }
        fn release(&mut self, _: bool) {
            self.events.push("release");
        }
        fn after(&mut self) -> Result<Option<VcomCounters>> {
            self.event("after")?;
            Ok(Some(VcomCounters {
                uart_rx: 140,
                ..VcomCounters::default()
            }))
        }
    }

    #[test]
    fn preparation_never_enters_the_counter_bracket() {
        let mut trace = Trace {
            events: Vec::new(),
            fail: None,
        };
        assert!(run(&mut trace).0.is_ok());
        assert_eq!(
            trace.events,
            ["prepare", "baseline", "arm", "stream", "release", "after"]
        );
    }

    #[test]
    fn failures_release_the_session_and_never_report_success() {
        for (fail, expected) in [
            ("prepare", &["prepare", "release", "after"][..]),
            ("baseline", &["prepare", "baseline", "release", "after"][..]),
            (
                "arm",
                &["prepare", "baseline", "arm", "release", "after"][..],
            ),
            (
                "stream",
                &["prepare", "baseline", "arm", "stream", "release", "after"][..],
            ),
            (
                "after",
                &["prepare", "baseline", "arm", "stream", "release", "after"][..],
            ),
        ] {
            let mut trace = Trace {
                events: Vec::new(),
                fail: Some(fail),
            };
            let (result, conservation) = run(&mut trace);
            assert!(result.is_err() || conservation.is_err());
            if matches!(fail, "prepare" | "baseline") {
                assert!(conservation.unwrap().is_none());
            }
            assert_eq!(trace.events, expected);
        }
    }

    #[test]
    fn repeated_windows_prepare_once_each_and_do_not_reuse_a_baseline() {
        let mut trace = Trace {
            events: Vec::new(),
            fail: None,
        };
        for _ in 0..2 {
            let (outcome, conservation) = run(&mut trace);
            outcome.unwrap();
            let counters = conservation.unwrap().unwrap();
            assert_eq!(counters.before.uart_rx, 100);
            assert_eq!(counters.after.uart_rx, 140);
        }
        assert_eq!(
            trace.events,
            [
                "prepare", "baseline", "arm", "stream", "release", "after", "prepare", "baseline",
                "arm", "stream", "release", "after",
            ]
        );
    }

    #[test]
    fn a_final_counter_error_cannot_be_mistaken_for_unsupported_counters() {
        let mut trace = Trace {
            events: Vec::new(),
            fail: Some("after"),
        };
        let (outcome, conservation) = run(&mut trace);
        assert!(outcome.is_ok());
        assert!(conservation.is_err());
    }

    #[test]
    fn a_failed_stream_still_closes_the_partial_counter_bracket() {
        let mut trace = Trace {
            events: Vec::new(),
            fail: Some("stream"),
        };
        let (outcome, conservation) = run(&mut trace);
        assert_eq!(outcome.unwrap_err().to_string(), "stream");
        assert_eq!(conservation.unwrap().unwrap().delta().uart_rx, 40);
        assert_eq!(
            &trace.events[trace.events.len() - 2..],
            ["release", "after"]
        );
    }
}
