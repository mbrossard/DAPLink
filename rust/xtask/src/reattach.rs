//! Forces an RP probe to re-enumerate after a reflash; its boot-time D+ drop is under hub debounce.

use std::time::{Duration, Instant};

use crate::install::probe_rs_command;
use crate::registry::Firmware;
use crate::usb::{self, Selector};

const SIE_CTRL: u32 = 0x5011_004c;
const PULLUP_EN: u32 = 0x0001_0000;

/// A write before the fresh core's USB block is up reads back zero and toggles nothing.
const FLASH_SETTLE: Duration = Duration::from_millis(500);
const RETRY_DELAY: Duration = Duration::from_millis(500);
const ATTEMPTS: u32 = 6;
/// Must outlast the hub's disconnect debounce.
const HOLD: Duration = Duration::from_millis(1500);
const GONE_TIMEOUT: Duration = Duration::from_secs(3);

/// Whether this part needs the soft-disconnect; true of every RP part.
pub fn applies(chip: &str) -> bool {
    chip.starts_with("RP")
}

fn read_word(fw: &Firmware, via: &str, addr: u32) -> Option<u32> {
    let out = probe_rs_command()
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

/// First word of a `probe-rs read b32` line (`<address>: <word> ...`).
fn parse_word(text: &str) -> Option<u32> {
    text.lines()
        .filter_map(|l| l.split_once(':'))
        .find_map(|(_, words)| u32::from_str_radix(words.split_whitespace().next()?, 16).ok())
}

fn write_word(fw: &Firmware, via: &str, addr: u32, value: u32) -> bool {
    probe_rs_command()
        .args(["write", "b32", "--chip", fw.chip, "--probe", via])
        .arg(format!("{addr:#x}"))
        .arg(format!("{value:#x}"))
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Best-effort clear of `PULLUP_EN` before the download, so the halt does not leave the host retrying.
pub fn clear_pullup(fw: &Firmware, via: &str) -> bool {
    let Some(sie) = read_word(fw, via, SIE_CTRL) else {
        eprintln!("clear pull-up: could not read SIE_CTRL");
        return false;
    };
    if sie == 0 {
        eprintln!("clear pull-up: SIE_CTRL reads zero, the core is still coming up");
        return false;
    }
    write_word(fw, via, SIE_CTRL, sie & !PULLUP_EN)
}

/// Drop `PULLUP_EN` until the host drops the device, then restore it; false means stale descriptors.
pub fn run(fw: &Firmware, via: &str, sel: &Selector) -> bool {
    println!("re-attaching {sel}: SIE_CTRL soft-disconnect over {via}");
    std::thread::sleep(FLASH_SETTLE);

    for attempt in 1..=ATTEMPTS {
        let Some(sie) = read_word(fw, via, SIE_CTRL) else {
            eprintln!("re-attach attempt {attempt}: could not read SIE_CTRL");
            std::thread::sleep(RETRY_DELAY);
            continue;
        };
        // Zero means the core is still coming up; writing it would toggle nothing.
        if sie == 0 {
            eprintln!(
                "re-attach attempt {attempt}: SIE_CTRL reads zero, the core is still coming up"
            );
            std::thread::sleep(RETRY_DELAY);
            continue;
        }
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

    /// Each line is prefixed with the address read, which a first-token parser would take.
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
