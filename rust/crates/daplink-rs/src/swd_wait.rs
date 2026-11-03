//! CMSIS-DAP's WAIT retry count for the `Swd` overrides: `N` retries make `N + 1` attempts.

use dap_rs::swd;

/// One transfer, then up to `retries` more while it answers WAIT.
#[inline(always)]
pub fn wait_retry<T>(
    retries: usize,
    mut transfer: impl FnMut() -> swd::Result<T>,
) -> swd::Result<T> {
    let mut left = retries;
    loop {
        match transfer() {
            Err(swd::Error::AckWait) if left != 0 => left -= 1,
            r => return r,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn attempts(retries: usize, waits: usize) -> (usize, swd::Result<()>) {
        let mut n = 0;
        let r = wait_retry(retries, || {
            n += 1;
            if n > waits {
                Ok(())
            } else {
                Err(swd::Error::AckWait)
            }
        });
        (n, r)
    }

    #[test]
    fn n_retries_make_n_plus_one_attempts() {
        assert_eq!(attempts(0, 0), (1, Ok(())));
        assert_eq!(attempts(0, 1), (1, Err(swd::Error::AckWait)));
        assert_eq!(attempts(2, 2), (3, Ok(())));
        assert_eq!(attempts(2, 5), (3, Err(swd::Error::AckWait)));
    }

    #[test]
    fn only_wait_is_retried() {
        let mut n = 0;
        let r: swd::Result<()> = wait_retry(5, || {
            n += 1;
            Err(swd::Error::AckFault)
        });
        assert_eq!((n, r), (1, Err(swd::Error::AckFault)));
    }
}
