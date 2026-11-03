//! Open-drain continuity checks through independent PUT and DUT RTT sessions.

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use daplink_test_protocol as proto;
use probe_rs::probe::list::Lister;
use probe_rs::probe::{DebugProbeSelector, WireProtocol};
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::{Permissions, Session};

use crate::run_record::{self, CaseRecord, DutIdRecord, RunMeta};
use crate::target::Target;
use crate::util::chain;

const EXCHANGE: Duration = Duration::from_millis(200);
// A complete stimulus observation must finish before either endpoint's one-second expiry.
const PHASE: Duration = Duration::from_millis(600);
const POLL: Duration = Duration::from_millis(2);
/// The one row `identify` accepts on the PUT.
const PUT_ROW: &str = "firmware/stlinkv2-conntest";
const PUT_PINS: &[(&str, &str)] = &[("PA7", "IO"), ("PA6", "I"), ("PA10", "I")];
const DUT_PINS: &[(&str, &str)] = &[("PA15", "I"), ("PB3", "IO")];

pub(crate) struct Options<'a> {
    pub probe: &'a str,
    pub via: Option<&'a str>,
    pub probe_elf: Option<&'a str>,
    pub dut_elf: Option<&'a str>,
    pub target: &'static Target,
    pub profile: Option<&'a str>,
    pub cases: &'a [String],
    pub firmware_row: Option<&'a str>,
    pub series: Option<&'a str>,
    pub record: Vec<PathBuf>,
}

pub(crate) fn run(options: Options<'_>) -> ExitCode {
    match run_inner(options) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(error) => {
            eprintln!("scan incomplete: {error}");
            ExitCode::from(2)
        }
    }
}

fn run_inner(options: Options<'_>) -> Result<bool, String> {
    if options.cases != ["scan"]
        || options.target.name != "stm32f401"
        || options.profile != Some("stlinkv2-conntest")
    {
        return Err(
            "scan must be the sole case, with --target stm32f401 --profile stlinkv2-conntest"
                .into(),
        );
    }
    let firmware_row = firmware_row::resolve(Some(PUT_ROW), options.firmware_row);
    for warning in &firmware_row.warnings {
        println!("WARNING: {warning}");
    }
    let via = options.via.ok_or("scan requires --via")?;
    let put_elf = options.probe_elf.ok_or("scan requires --probe-elf")?;
    let dut_elf = options.dut_elf.ok_or("scan requires --dut-elf")?;
    let put_address = crate::rtt::rtt_symbol(put_elf)?;
    let dut_address = crate::rtt::rtt_symbol(dut_elf)?;
    let selector = selector(options.probe)?;
    let via_selector = selector_for_via(via, &selector)?;
    let interrupted = Arc::new(AtomicBool::new(false));
    let signal = interrupted.clone();
    ctrlc::set_handler(move || signal.store(true, Ordering::Relaxed)).map_err(chain)?;
    let started = Instant::now();
    let mut meta = RunMeta {
        run_id: run_record::run_id(),
        mode: "list",
        series: options.series.map(str::to_string),
        vid: selector.vendor_id,
        pid: selector.product_id,
        serial: selector.serial_number.clone(),
        firmware_row_source: firmware_row.source.key(),
        firmware_row: firmware_row.row,
        firmware_version: None,
        harness_version: daplink_core::DAP_FW_VERSION,
        target: options.target.name,
        chip: options.target.chip.into(),
        dut_elf: Some(dut_elf.into()),
        dut_uf2: None,
        dut: None,
    };
    let mut observations = vec![format!("profile=stlinkv2-conntest via={via} probe_elf={put_elf}; expected edges are wiring hypotheses")];
    let result = (|| {
        // Attach and resume the PUT before opening the debug link it serves.
        let mut put = Endpoint::open(
            via_selector,
            "STM32F103C8",
            put_address,
            interrupted.clone(),
        )?;
        let put_id = put.identify("stlinkv2-conntest", PUT_PINS)?;
        meta.firmware_version = put_id.revision.clone();
        observations.push(format!("PUT {}", put_id.describe()));
        let mut dut = Endpoint::open(
            selector,
            options.target.chip,
            dut_address,
            interrupted.clone(),
        )?;
        let result = (|| {
            let dut_id = dut.identify("test-stm32f401-conntest", DUT_PINS)?;
            observations.push(format!("DUT {}", dut_id.describe()));
            meta.dut = Some(DutIdRecord {
                name: dut_id.name,
                proto: dut_id.proto,
                revision: dut_id.revision,
            });
            scan_pair(&mut put, &mut dut, &mut observations)
        })();
        let cleanup = release_pair(&mut put, &mut dut);
        match (result, cleanup) {
            (Ok(value), Ok(())) => Ok(value),
            (Err(error), Ok(())) => Err(error),
            (result, Err(error)) => Err(format!(
                "{}; cleanup: {error}",
                result.err().unwrap_or_else(|| "scan completed".into())
            )),
        }
    })();
    if let Err(error) = &result {
        observations.push(format!("INCOMPLETE: {error}"));
    }
    for observation in &observations {
        println!("scan: {observation}");
    }
    let detail = observations.join("\n");
    for path in options.record {
        run_record::append(
            &path,
            &meta,
            &CaseRecord {
                case: "scan",
                outcome: match &result {
                    Ok(true) => "ok",
                    Ok(false) => "failed",
                    Err(_) => "fatal",
                },
                detail: Some(&detail),
                duration_ms: started.elapsed().as_millis() as u64,
                load1: None,
                waits: Vec::new(),
                speed: None,
            },
        )
        .map_err(|e| format!("record {}: {e}", path.display()))?;
        println!("scan: record {}", path.display());
    }
    result
}

fn selector(text: &str) -> Result<DebugProbeSelector, String> {
    let selector: DebugProbeSelector = text.parse().map_err(chain)?;
    if selector.serial_number.as_deref().is_none_or(str::is_empty) {
        return Err("scan requires explicit probe serials".into());
    }
    Ok(selector)
}

fn selector_for_via(text: &str, put: &DebugProbeSelector) -> Result<DebugProbeSelector, String> {
    let via = selector(text)?;
    if via.vendor_id == put.vendor_id
        && via.product_id == put.product_id
        && via.serial_number == put.serial_number
    {
        return Err("--via and --probe must identify different probes".into());
    }
    Ok(via)
}

struct Endpoint {
    session: Session,
    rtt: Rtt,
    interrupted: Arc<AtomicBool>,
    identified: bool,
}

impl Endpoint {
    fn open(
        selector: DebugProbeSelector,
        chip: &str,
        address: u64,
        interrupted: Arc<AtomicBool>,
    ) -> Result<Self, String> {
        let mut probe = Lister::new().open(selector).map_err(chain)?;
        probe.select_protocol(WireProtocol::Swd).map_err(chain)?;
        probe.set_speed(1000).map_err(chain)?;
        let mut session = probe.attach(chip, Permissions::default()).map_err(chain)?;
        let mut core = session.core(0).map_err(chain)?;
        if core.core_halted().map_err(chain)? {
            core.run().map_err(chain)?;
        }
        let deadline = Instant::now() + Duration::from_secs(3);
        let rtt = loop {
            if interrupted.load(Ordering::Relaxed) {
                return Err("interrupted".into());
            }
            if let Ok(mut rtt) = Rtt::attach_region(&mut core, &ScanRegion::Exact(address)) {
                if rtt.up_channel(0).is_some() && rtt.down_channel(0).is_some() {
                    break rtt;
                }
            }
            if Instant::now() >= deadline {
                return Err(format!("{chip}: RTT attach deadline exceeded"));
            }
            std::thread::sleep(Duration::from_millis(20));
        };
        drop(core);
        let mut endpoint = Self {
            session,
            rtt,
            interrupted,
            identified: false,
        };
        endpoint.drain()?;
        Ok(endpoint)
    }

    fn drain(&mut self) -> Result<(), String> {
        let deadline = Instant::now() + Duration::from_millis(300);
        let mut quiet_since = Instant::now();
        let mut total = 0;
        let mut core = self.session.core(0).map_err(chain)?;
        let channel = self.rtt.up_channel(0).ok_or("missing RTT up channel")?;
        loop {
            if self.interrupted.load(Ordering::Relaxed) {
                return Err("interrupted".into());
            }
            let mut bytes = [0; 256];
            let n = channel.read(&mut core, &mut bytes).map_err(chain)?;
            total += n;
            if n > 0 {
                quiet_since = Instant::now();
            }
            if quiet_since.elapsed() >= Duration::from_millis(20) {
                return Ok(());
            }
            if total > 4096 || Instant::now() >= deadline {
                return Err("RTT boot output does not quiesce".into());
            }
            std::thread::sleep(POLL);
        }
    }

    fn exchange(
        &mut self,
        command: &str,
        deadline: Instant,
        cleanup: bool,
    ) -> Result<String, String> {
        let deadline = deadline.min(Instant::now() + EXCHANGE);
        check_window(self.interrupted.load(Ordering::Relaxed), cleanup, deadline)?;
        let mut core = self.session.core(0).map_err(chain)?;
        let line = format!("{command}\n");
        let written = self
            .rtt
            .down_channel(0)
            .ok_or("missing RTT down channel")?
            .write(&mut core, line.as_bytes())
            .map_err(chain)?;
        if written != line.len() {
            return Err(format!(
                "partial RTT command write: {written}/{}",
                line.len()
            ));
        }
        let channel = self.rtt.up_channel(0).ok_or("missing RTT up channel")?;
        let mut frame = Vec::new();
        loop {
            check_window(self.interrupted.load(Ordering::Relaxed), cleanup, deadline)?;
            let mut bytes = [0; 256];
            let n = channel.read(&mut core, &mut bytes).map_err(chain)?;
            check_window(self.interrupted.load(Ordering::Relaxed), cleanup, deadline)
                .map_err(|e| format!("{command}: {e}"))?;
            if let Some(reply) = frame_line(&mut frame, &bytes[..n])? {
                if reply.starts_with("ERR ") {
                    return Err(format!("{command}: {reply}"));
                }
                return Ok(reply);
            }
            std::thread::sleep(POLL);
        }
    }

    fn expect(&mut self, command: &str, expected: &str, deadline: Instant) -> Result<(), String> {
        let reply = self.exchange(command, deadline, false)?;
        if reply != expected {
            return Err(format!(
                "{command}: expected {expected:?}, received {reply:?}"
            ));
        }
        Ok(())
    }

    fn identify(
        &mut self,
        name: &str,
        pins: &[(&str, &str)],
    ) -> Result<crate::dutid::DutId, String> {
        let reply = self.exchange("ID", Instant::now() + EXCHANGE, false)?;
        let id = parse_identity(&reply, name)?;
        self.expect(
            "GPIO LIST",
            &format!("OK GPIO COUNT {}", pins.len()),
            Instant::now() + EXCHANGE,
        )?;
        for (index, (pin, direction)) in pins.iter().enumerate() {
            self.expect(
                &format!("GPIO LIST {index}"),
                &format!("OK GPIO PIN {index} {pin} {direction}"),
                Instant::now() + EXCHANGE,
            )?;
        }
        self.identified = true;
        self.release()?;
        Ok(id)
    }

    fn read(&mut self, mask: u64, deadline: Instant) -> Result<u64, String> {
        parse_read(&self.exchange("GPIO READ", deadline, false)?, mask)
    }

    fn release(&mut self) -> Result<(), String> {
        if !self.identified {
            return Ok(());
        }
        let reply = self.exchange("GPIO RELEASE", Instant::now() + EXCHANGE, true)?;
        if reply != "OK GPIO RELEASE" {
            return Err(format!("release: {reply}"));
        }
        Ok(())
    }
}

impl Drop for Endpoint {
    fn drop(&mut self) {
        if let Err(error) = self.release() {
            eprintln!("scan cleanup: {error}; firmware inactivity release remains active");
        }
    }
}

fn frame_line(frame: &mut Vec<u8>, bytes: &[u8]) -> Result<Option<String>, String> {
    if frame.len() + bytes.len() > proto::MAX_LINE {
        return Err("oversized RTT reply".into());
    }
    frame.extend_from_slice(bytes);
    let Some(end) = frame.iter().position(|b| *b == b'\n') else {
        return Ok(None);
    };
    if end + 1 != frame.len() {
        return Err("unsolicited or multiple RTT replies".into());
    }
    let text = std::str::from_utf8(&frame[..end]).map_err(chain)?;
    if !text.is_ascii() {
        return Err("non-ASCII RTT reply".into());
    }
    Ok(Some(text.strip_suffix('\r').unwrap_or(text).to_owned()))
}

fn parse_identity(reply: &str, name: &str) -> Result<crate::dutid::DutId, String> {
    let fields: Vec<_> = reply.split_ascii_whitespace().collect();
    if fields.len() != 6
        || fields[..2] != ["OK", "ID"]
        || fields[2] != format!("name={name}")
        || fields[3] != format!("proto={}", proto::PROTO_VERSION)
        || fields[4]
            .strip_prefix("stamp=")
            .and_then(|v| v.parse::<u32>().ok())
            .is_none()
        || fields[5].strip_prefix("rev=").is_none_or(str::is_empty)
    {
        return Err(format!("incompatible scan endpoint ID: {reply:?}"));
    }
    Ok(crate::dutid::DutId {
        name: Some(name.into()),
        proto: Some(proto::PROTO_VERSION.into()),
        revision: fields[5].strip_prefix("rev=").map(str::to_owned),
    })
}

fn parse_read(reply: &str, mask: u64) -> Result<u64, String> {
    let words: Vec<_> = reply.split_ascii_whitespace().collect();
    if words.len() != 4 || words[..3] != ["OK", "GPIO", "READ"] {
        return Err(format!("invalid GPIO READ reply: {reply:?}"));
    }
    let value = u64::from_str_radix(words[3], 16).map_err(chain)?;
    if value & !mask != 0 {
        return Err(format!("GPIO READ reports unknown pins: {value:X}"));
    }
    Ok(value)
}

fn release_pair(put: &mut Endpoint, dut: &mut Endpoint) -> Result<(), String> {
    release_both(|| put.release(), || dut.release())
}

fn release_both(
    put: impl FnOnce() -> Result<(), String>,
    dut: impl FnOnce() -> Result<(), String>,
) -> Result<(), String> {
    let put_result = put();
    let dut_result = dut();
    match (put_result, dut_result) {
        (Ok(()), Ok(())) => Ok(()),
        (a, b) => Err(format!("PUT={a:?} DUT={b:?}")),
    }
}

fn check_window(interrupted: bool, cleanup: bool, deadline: Instant) -> Result<(), String> {
    if interrupted && !cleanup {
        return Err("interrupted; observation inconclusive".into());
    }
    if Instant::now() >= deadline {
        return Err("deadline exceeded; observation inconclusive".into());
    }
    Ok(())
}

fn scan_pair(
    put: &mut Endpoint,
    dut: &mut Endpoint,
    log: &mut Vec<String>,
) -> Result<bool, String> {
    let forward = scan_direction(put, dut, "PUT PA7 -> DUT", 1, DUT_PINS, 1, log)?;
    let backward = scan_direction(dut, put, "DUT PB3 -> PUT", 2, PUT_PINS, 6, log)?;
    Ok(forward && backward)
}

fn scan_direction(
    driver: &mut Endpoint,
    reader: &mut Endpoint,
    name: &str,
    drive: u64,
    pins: &[(&str, &str)],
    expected: u64,
    log: &mut Vec<String>,
) -> Result<bool, String> {
    let mask = (1 << pins.len()) - 1;
    let mut samples = Vec::new();
    for repeat in 0..3 {
        let deadline = Instant::now() + PHASE;
        driver.expect("GPIO CONFIG_INPUT UP", "OK GPIO CONFIG_INPUT", deadline)?;
        reader.expect("GPIO CONFIG_INPUT UP", "OK GPIO CONFIG_INPUT", deadline)?;
        let baseline = reader.read(mask, deadline)?;
        log.push(format!(
            "{name} repeat={} baseline={baseline:X}",
            repeat + 1
        ));
        let mut levels = [0; 2];
        for (index, level) in [0, drive].into_iter().enumerate() {
            let deadline = Instant::now() + PHASE;
            reader.expect("GPIO CONFIG_INPUT UP", "OK GPIO CONFIG_INPUT", deadline)?;
            driver.expect(
                &format!("GPIO DRIVE {drive:X} {level:X}"),
                "OK GPIO DRIVE",
                deadline,
            )?;
            std::thread::sleep(Duration::from_millis(5));
            levels[index] = reader.read(mask, deadline)?;
            log.push(format!(
                "{name} repeat={} drive={level:X} receiver={:X}",
                repeat + 1,
                levels[index]
            ));
        }
        samples.push((baseline, levels[0], levels[1]));
    }
    driver.release()?;
    reader.release()?;
    let verdict = classify(&samples, mask, expected);
    for (index, (pin, _)) in pins.iter().enumerate() {
        let bit = 1 << index;
        let status = if verdict.followers & bit != 0 {
            if expected & bit != 0 {
                "CONFIRMED"
            } else {
                "UNEXPECTED FOLLOWER"
            }
        } else if expected & bit != 0 {
            "MISSING"
        } else {
            "NO EDGE"
        };
        log.push(format!("{name} {pin}: {status}"));
    }
    log.push(format!("{name}: baseline/low/released={samples:X?}; followers={:X} unstable={:X} baseline_low={:X}", verdict.followers, verdict.unstable, verdict.baseline_low));
    Ok(verdict.clean)
}

#[derive(Debug, PartialEq, Eq)]
struct Verdict {
    followers: u64,
    unstable: u64,
    baseline_low: u64,
    clean: bool,
}

fn classify(samples: &[(u64, u64, u64)], mask: u64, expected: u64) -> Verdict {
    let mut followers = mask;
    let mut baseline_low = 0;
    let mut changes = 0;
    let mut release_low = 0;
    for &(baseline, low, released) in samples {
        let following = !low & released & mask;
        followers &= following;
        changes |= (low ^ released) & mask;
        baseline_low |= !baseline & mask;
        release_low |= !released & mask;
    }
    let unstable = (changes & !followers) | release_low;
    Verdict {
        followers,
        unstable,
        baseline_low,
        clean: samples.len() == 3 && followers == expected && unstable == 0 && baseline_low == 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_scan_put_is_a_registry_row() {
        assert!(firmware_row::resolve(None, Some(PUT_ROW))
            .warnings
            .is_empty());
    }

    #[test]
    fn continuity_requires_every_transition_and_no_extra_follower() {
        assert!(classify(&[(7, 1, 7); 3], 7, 6).clean);
        assert!(!classify(&[(7, 3, 7); 3], 7, 6).clean);
        assert!(!classify(&[(7, 0, 7); 3], 7, 6).clean);
        assert!(!classify(&[(7, 7, 7); 3], 7, 6).clean);
        assert!(!classify(&[(1, 1, 1); 3], 7, 6).clean);
        let intermittent = classify(&[(7, 1, 7), (7, 3, 7), (7, 1, 7)], 7, 6);
        assert_eq!(intermittent.followers, 4);
        assert_eq!(intermittent.unstable, 2);
        assert!(!intermittent.clean);
        assert!(!classify(&[(7, 1, 7); 2], 7, 6).clean);
    }

    #[test]
    fn wrong_idle_level_and_inverse_followers_are_not_clean() {
        assert!(!classify(&[(5, 1, 7); 3], 7, 6).clean);
        assert!(!classify(&[(7, 1, 6); 3], 7, 6).clean);
        assert!(!classify(&[(7, 0, 6); 3], 7, 6).clean);
    }

    #[test]
    fn replies_are_bounded_complete_single_lines() {
        let mut frame = Vec::new();
        assert_eq!(frame_line(&mut frame, b"OK GP").unwrap(), None);
        assert_eq!(
            frame_line(&mut frame, b"IO READ 2\n").unwrap(),
            Some("OK GPIO READ 2".into())
        );
        assert!(frame_line(&mut Vec::new(), b"OK GPIO READ 2\nOK GPIO READ 2\n").is_err());
        assert!(frame_line(&mut Vec::new(), &[b'x'; proto::MAX_LINE + 1]).is_err());
        assert!(frame_line(&mut Vec::new(), b"\xff\n").is_err());
    }

    #[test]
    fn read_and_identity_parsing_reject_substring_and_foreign_endpoint_matches() {
        for bad in [
            "note OK GPIO READ 2",
            "OK GPIO READ 2 extra",
            "OK GPIO READ 8",
            "OK GPIO READ nope",
        ] {
            assert!(parse_read(bad, 7).is_err());
        }
        assert_eq!(parse_read("OK GPIO READ 6", 7).unwrap(), 6);
        let id = format!(
            "OK ID name=stlinkv2-conntest proto={} stamp=0 rev=g123",
            proto::PROTO_VERSION
        );
        assert!(parse_identity(&id, "stlinkv2-conntest").is_ok());
        assert!(parse_identity(&format!("note {id}"), "stlinkv2-conntest").is_err());
        assert!(parse_identity(&id, "test-stm32f401-conntest").is_err());
        assert!(parse_identity(&id.replace("stamp=0", "stamp=nope"), "stlinkv2-conntest").is_err());
    }

    #[test]
    fn scan_requires_distinct_explicit_serials() {
        assert!(selector("0d28:0204").is_err());
        let put = selector("0d28:0204:ABC").unwrap();
        assert!(selector_for_via("0d28:0204:ABC", &put).is_err());
        assert!(selector_for_via("0d28:0204:DEF", &put).is_ok());
    }

    #[test]
    fn interruption_permits_cleanup_but_never_relaxes_its_deadline() {
        let deadline = Instant::now() + Duration::from_secs(1);
        assert!(check_window(true, false, deadline).is_err());
        assert!(check_window(true, true, deadline).is_ok());
        assert!(check_window(true, true, Instant::now()).is_err());
        assert!(check_window(false, false, Instant::now()).is_err());
    }

    #[test]
    fn cleanup_attempts_both_endpoints_after_either_failure() {
        let mut called = [false; 2];
        let (put, dut) = called.split_at_mut(1);
        let error = release_both(
            || {
                put[0] = true;
                Err("lost PUT".into())
            },
            || {
                dut[0] = true;
                Ok(())
            },
        )
        .unwrap_err();
        assert_eq!(called, [true, true]);
        assert!(error.contains("lost PUT"));
        assert!(release_both(|| Ok(()), || Err("lost DUT".into())).is_err());
    }
}
