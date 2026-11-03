//! A DAP read loop run beside a streaming case; it counts reads and times nothing.

use probe_rs::{Core, MemoryInterface, Session};

use crate::util::chain;

/// Word reads issued alongside a stream, and how many of them the target answered.
pub(crate) struct DapLoad {
    addr: u64,
    issued: u64,
    answered: u64,
    first_error: Option<String>,
}

impl DapLoad {
    pub(crate) fn new(addr: u64) -> Self {
        DapLoad {
            addr,
            issued: 0,
            answered: 0,
            first_error: None,
        }
    }

    /// One read on an already-borrowed core.
    pub(crate) fn tick(&mut self, core: &mut Core) {
        self.issued += 1;
        match core.read_word_32(self.addr) {
            Ok(_) => self.answered += 1,
            Err(e) => {
                if self.first_error.is_none() {
                    self.first_error = Some(chain(e));
                }
            }
        }
    }

    /// One read through a session, for a caller that holds no core of its own.
    pub(crate) fn tick_session(&mut self, session: &mut Session) {
        match session.core(0) {
            Ok(mut core) => self.tick(&mut core),
            Err(e) => {
                self.issued += 1;
                if self.first_error.is_none() {
                    self.first_error = Some(chain(e));
                }
            }
        }
    }

    /// Whether more reads would help: the floor is not met and nothing has gone wrong yet.
    pub(crate) fn needs_more(&self, min_reads: u64) -> bool {
        self.first_error.is_none() && self.answered < min_reads
    }

    /// Every issued read was answered, and enough were issued to count as loaded.
    pub(crate) fn progress(&self, min_reads: u64) -> Result<(), String> {
        if let Some(e) = &self.first_error {
            return Err(format!(
                "the DAP read loop failed after {} of {} reads: {e}",
                self.answered, self.issued
            ));
        }
        if self.answered < min_reads {
            return Err(format!(
                "the DAP read loop made {} reads, below the {min_reads} that make this a \
                 loaded run rather than an idle one",
                self.answered
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A load that never ran does not pass.
    #[test]
    fn an_idle_loop_does_not_count_as_loaded() {
        let load = DapLoad::new(0);
        assert!(load.progress(1).is_err());
        assert!(load.progress(0).is_ok());
    }

    #[test]
    fn a_reported_error_fails_however_many_reads_landed() {
        let mut load = DapLoad::new(0);
        load.issued = 100;
        load.answered = 99;
        load.first_error = Some("NoAck".into());
        let err = load
            .progress(1)
            .expect_err("an errored loop is not progress");
        assert!(err.contains("NoAck"), "{err}");
    }

    /// `needs_more` must stop asking on an error, or a caller topping the loop up would spin.
    #[test]
    fn needs_more_stops_at_the_floor_and_on_an_error() {
        let mut load = DapLoad::new(0);
        assert!(load.needs_more(4));
        load.answered = 4;
        assert!(!load.needs_more(4));
        load.answered = 0;
        load.first_error = Some("NoAck".into());
        assert!(!load.needs_more(4));
    }

    #[test]
    fn enough_answered_reads_pass() {
        let mut load = DapLoad::new(0);
        load.issued = 32;
        load.answered = 32;
        assert!(load.progress(16).is_ok());
    }
}
