//! Making an RP probe re-enumerate after a reflash, through the device's own soft-disconnect.
//!
//! `probe-rs download` plus a reset does not produce a USB re-attach on an RP part: its D+
//! pull-up drops for a few milliseconds at boot, far under the hub's disconnect debounce, so the
//! host keeps serving the *pre-flash* descriptors. An interface the new image composes is then
//! simply absent, and a capability case reads that as firmware that does not compose it.
//! Toggling `SIE_CTRL.PULLUP_EN` over the transport holds the disconnect long enough to be seen.

use std::process::Command;
use std::time::{Duration, Instant};

use crate::registry::Firmware;
use crate::usb::{self, Selector};

/// The RP USB controller's `SIE_CTRL` and its D+ pull-up enable bit.
const SIE_CTRL: u32 = 0x5011_004c;
const PULLUP_EN: u32 = 0x0001_0000;

/// Let the freshly reset core bring its USB block up before the first write; a write that lands
/// before it does reads back as zero and toggles nothing.
const FLASH_SETTLE: Duration = Duration::from_millis(500);
const RETRY_DELAY: Duration = Duration::from_millis(500);
const ATTEMPTS: u32 = 6;
/// The disconnect has to outlast the hub's debounce; the firmware's own remount detach uses
/// 500 ms, and this is comfortably clear of it.
const HOLD: Duration = Duration::from_millis(1500);
const GONE_TIMEOUT: Duration = Duration::from_secs(3);

/// Whether this row's part needs the soft-disconnect. Every RP part hides its boot-time pull-up
/// drop under the debounce the same way.
pub fn applies(chip: &str) -> bool {
    chip.starts_with("RP")
}

fn read_word(fw: &Firmware, via: &str, addr: u32) -> Option<u32> {
    let out = Command::new("probe-rs")
        .args(["read", "b32", "--chip", fw.chip, "--probe", via])
        .arg(format!("{addr:#x}"))
        .arg("1")
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    parse_word(&String::from_utf8_lossy(&out.stdout))
}

/// `probe-rs read b32` prints `<address>: <word> [<word>...]` per line, hex and no `0x`; take
/// the first word after the colon.
fn parse_word(text: &str) -> Option<u32> {
    text.lines()
        .filter_map(|l| l.split_once(':'))
        .find_map(|(_, words)| u32::from_str_radix(words.split_whitespace().next()?, 16).ok())
}

fn write_word(fw: &Firmware, via: &str, addr: u32, value: u32) -> bool {
    Command::new("probe-rs")
        .args(["write", "b32", "--chip", fw.chip, "--probe", via])
        .arg(format!("{addr:#x}"))
        .arg(format!("{value:#x}"))
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Drop `PULLUP_EN`, hold until the host drops the device, then restore it. Returns false when
/// the register could not be driven or the host never saw the device leave -- in which case the
/// descriptors it holds are still the pre-flash ones and nothing downstream may be trusted.
pub fn run(fw: &Firmware, via: &str, sel: &Selector) -> bool {
    println!("re-attaching {sel}: SIE_CTRL soft-disconnect over {via}");
    std::thread::sleep(FLASH_SETTLE);

    for attempt in 1..=ATTEMPTS {
        let Some(sie) = read_word(fw, via, SIE_CTRL) else {
            eprintln!("re-attach attempt {attempt}: could not read SIE_CTRL");
            std::thread::sleep(RETRY_DELAY);
            continue;
        };
        // A zero here is not a connected controller with every bit clear: it is the core still
        // coming up. Writing it would toggle nothing and disconnect nothing.
        if sie == 0 {
            eprintln!(
                "re-attach attempt {attempt}: SIE_CTRL reads zero, the core is still coming up"
            );
            std::thread::sleep(RETRY_DELAY);
            continue;
        }
        // Read-modify-write: every other bit the running firmware set stays as it is.
        if !write_word(fw, via, SIE_CTRL, sie & !PULLUP_EN) {
            eprintln!("re-attach attempt {attempt}: could not write SIE_CTRL");
            std::thread::sleep(RETRY_DELAY);
            continue;
        }
        match read_word(fw, via, SIE_CTRL) {
            Some(back) if back != 0 && back & PULLUP_EN == 0 => {}
            back => {
                eprintln!(
                    "re-attach attempt {attempt}: SIE_CTRL read back as {}, the pull-up did not drop",
                    back.map_or("unreadable".to_string(), |b| format!("{b:#010x}"))
                );
                std::thread::sleep(RETRY_DELAY);
                continue;
            }
        }

        let started = Instant::now();
        let gone = usb::wait_gone(sel, GONE_TIMEOUT);
        if let Some(rest) = HOLD.checked_sub(started.elapsed()) {
            std::thread::sleep(rest);
        }
        if !write_word(fw, via, SIE_CTRL, sie | PULLUP_EN) {
            eprintln!("re-attach: could not restore PULLUP_EN; the PUT is left disconnected");
            return false;
        }
        if !gone {
            eprintln!(
                "re-attach: the host never saw {sel} leave the bus, so it still holds the \
                 pre-flash descriptors"
            );
            return false;
        }
        println!("re-attach: the host dropped the PUT and the pull-up is back on");
        return true;
    }

    eprintln!("re-attach: SIE_CTRL would not toggle after {ATTEMPTS} attempts");
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_rp_parts_need_the_soft_disconnect() {
        assert!(applies("RP2040"));
        assert!(applies("RP235x"));
        assert!(!applies("STM32F103CB"));
    }

    /// probe-rs 0.32 prefixes every line with the address it read, so a parser that takes the
    /// first token takes the address and never a value.
    #[test]
    fn reads_the_first_word_probe_rs_prints() {
        assert_eq!(parse_word("5011004c: 20010000\n"), Some(0x2001_0000));
        assert_eq!(parse_word("5011004c: 00000000 deadbeef\n"), Some(0));
        assert_eq!(parse_word("5011004c:\n"), None);
        assert_eq!(
            parse_word("attached to probe: yes\n5011004c: 20010000\n"),
            Some(0x2001_0000)
        );
        assert_eq!(parse_word("20010000\n"), None);
        assert_eq!(parse_word(""), None);
    }
}
