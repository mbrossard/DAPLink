//! The deadline-bound waits of the running case, drained into its record.

use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::run_record::WaitRecord;

static WAITS: Mutex<Vec<WaitRecord>> = Mutex::new(Vec::new());

/// Note one wait against its budget.
pub(crate) fn note(name: &'static str, budget: Duration, used: Duration) {
    fold(
        &mut WAITS.lock().unwrap_or_else(|e| e.into_inner()),
        name,
        budget,
        used,
    );
}

/// Run a wait and note how long it took, whatever it returned.
pub(crate) fn timed<T>(name: &'static str, budget: Duration, wait: impl FnOnce() -> T) -> T {
    let started = Instant::now();
    let outcome = wait();
    note(name, budget, started.elapsed());
    outcome
}

/// What the running case has noted, leaving none for the next.
pub(crate) fn take() -> Vec<WaitRecord> {
    std::mem::take(&mut WAITS.lock().unwrap_or_else(|e| e.into_inner()))
}

/// One name folds to its worst.
fn fold(waits: &mut Vec<WaitRecord>, name: &'static str, budget: Duration, used: Duration) {
    let used = used.as_millis() as u64;
    match waits.iter_mut().find(|w| w.name == name) {
        Some(wait) => {
            wait.max_ms = wait.max_ms.max(used);
            wait.n += 1;
        }
        None => waits.push(WaitRecord {
            name,
            budget_ms: budget.as_millis() as u64,
            max_ms: used,
            n: 1,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_name_folds_to_its_worst() {
        let ms = Duration::from_millis;
        let mut waits = Vec::new();
        fold(&mut waits, "a", ms(100), ms(7));
        fold(&mut waits, "b", ms(50), ms(1));
        fold(&mut waits, "a", ms(100), ms(40));
        fold(&mut waits, "a", ms(100), ms(9));
        assert_eq!(waits.len(), 2);
        assert_eq!((waits[0].name, waits[0].max_ms, waits[0].n), ("a", 40, 3));
        assert_eq!((waits[1].budget_ms, waits[1].max_ms), (50, 1));
    }
}
