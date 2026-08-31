//! The PUT's identity on the USB bus: the `vid:pid:serial` selector and presence waits.

use std::time::{Duration, Instant};

use nusb::{DeviceInfo, MaybeFuture};

pub const POLL_INTERVAL: Duration = Duration::from_millis(100);

/// A parsed `vid:pid:serial` PUT selector -- the same syntax `--probe` already uses for probe-rs
/// transports, reused here to name the PUT's own USB identity.
pub struct Selector {
    pub vid: u16,
    pub pid: u16,
    pub serial: String,
}

pub fn parse_selector(s: &str) -> Result<Selector, String> {
    let mut parts = s.splitn(3, ':');
    let (Some(vid), Some(pid), Some(serial)) = (parts.next(), parts.next(), parts.next()) else {
        return Err(format!("`{s}` is not a vid:pid:serial selector"));
    };
    let vid = u16::from_str_radix(vid, 16).map_err(|e| format!("bad vid `{vid}`: {e}"))?;
    // probe-rs writes the interface after the product id (`0d28:0204-0:SERIAL`); ignore it.
    let pid_digits = pid.split('-').next().unwrap_or(pid);
    let pid =
        u16::from_str_radix(pid_digits, 16).map_err(|e| format!("bad pid `{pid_digits}`: {e}"))?;
    Ok(Selector {
        vid,
        pid,
        serial: serial.to_string(),
    })
}

impl Selector {
    fn matches(&self, d: &DeviceInfo) -> bool {
        d.vendor_id() == self.vid
            && d.product_id() == self.pid
            && d.serial_number() == Some(self.serial.as_str())
    }

    pub fn find(&self) -> Option<DeviceInfo> {
        nusb::list_devices().wait().ok()?.find(|d| self.matches(d))
    }

    pub fn present(&self) -> bool {
        self.find().is_some()
    }
}

impl std::fmt::Display for Selector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:04x}:{:04x} serial {}",
            self.vid, self.pid, self.serial
        )
    }
}

/// Wait until the device has been on the bus continuously for `settle`. A probe caught between
/// its reset and its detach is on the bus too, so presence alone does not mean re-enumerated;
/// only presence that holds does.
pub fn wait_settled(sel: &Selector, settle: Duration, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    let mut since = None;
    loop {
        match (sel.present(), since) {
            (true, Some(t)) if Instant::now().duration_since(t) >= settle => return true,
            (true, None) => since = Some(Instant::now()),
            (false, _) => since = None,
            _ => {}
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Wait until the device leaves the bus, which is the host acknowledging a disconnect.
pub fn wait_gone(sel: &Selector, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if !sel.present() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_valid_selector() {
        let sel = parse_selector("0d28:0204:00002B87C0A64E97").unwrap();
        assert_eq!(sel.vid, 0x0d28);
        assert_eq!(sel.pid, 0x0204);
        assert_eq!(sel.serial, "00002B87C0A64E97");
    }

    #[test]
    fn parses_a_selector_carrying_an_interface() {
        let sel = parse_selector("0d28:0204-0:0000500B2EC6BEFB").unwrap();
        assert_eq!(sel.pid, 0x0204);
        assert_eq!(sel.serial, "0000500B2EC6BEFB");
    }

    #[test]
    fn rejects_a_malformed_selector() {
        assert!(parse_selector("0d28:0204").is_err());
        assert!(parse_selector("nope:0204:abc").is_err());
    }

    /// A selector no device answers must time out rather than settle.
    #[test]
    fn an_absent_device_never_settles() {
        let sel = parse_selector("dead:beef:NOSUCHDEVICE").unwrap();
        assert!(!wait_settled(
            &sel,
            Duration::from_millis(10),
            Duration::from_millis(50)
        ));
        assert!(wait_gone(&sel, Duration::from_millis(50)));
    }
}
