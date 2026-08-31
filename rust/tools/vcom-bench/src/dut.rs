//! DUT control over RTT: the test firmware's `BAUD`, `STATS` and `V1` verbs, and DAP load.

use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use daplink_test_protocol as proto;
use probe_rs::probe::list::Lister;
use probe_rs::probe::DebugProbeSelector;
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::{MemoryInterface, Permissions, Session};

use crate::budget::{DapAchieved, DapProfile, DAP_READ_BYTES};

const EXCHANGE_TIMEOUT: Duration = Duration::from_secs(3);
const POLL: Duration = Duration::from_millis(5);
const DAP_READ_WORDS: usize = DAP_READ_BYTES as usize / 4;
const ATTACH_TIMEOUT: Duration = Duration::from_secs(15);
const RESET_SETTLE: Duration = Duration::from_millis(250);
const RESET_ATTEMPTS: usize = 3;

const TARGETS: &[(&str, &str)] = &[
    ("stm32f401", "STM32F401CEUx"),
    ("rp2040", "RP2040"),
    ("rp2350", "RP235x"),
];

pub fn chip_for(target: &str) -> Result<&'static str> {
    TARGETS
        .iter()
        .find(|(name, _)| *name == target)
        .map(|(_, chip)| *chip)
        .ok_or_else(|| {
            let known: Vec<&str> = TARGETS.iter().map(|(n, _)| *n).collect();
            anyhow!("unknown target '{target}'; known: {}", known.join(", "))
        })
}

/// An attached control session: probe-rs session plus the DUT's RTT channels.
pub struct Dut {
    session: Session,
    rtt: Rtt,
    /// Where the control block was found, so a reset can find it again.
    region: ScanRegion,
}

impl Dut {
    /// Find the RTT control block at the ELF's `_SEGGER_RTT` symbol if `dut_elf` is given, by RAM
    /// scan otherwise.
    pub fn open(selector: &str, chip: &str, dut_elf: Option<&str>) -> Result<Dut> {
        let selector = DebugProbeSelector::from_str(selector)
            .with_context(|| format!("parse probe selector '{selector}'"))?;
        let probe = Lister::new()
            .open(selector)
            .context("open the probe for DUT control")?;
        let mut session = probe
            .attach(chip, Permissions::default())
            .context("attach to the DUT through the probe")?;

        let region = dut_elf
            .and_then(rtt_symbol)
            .map_or(ScanRegion::default(), ScanRegion::Exact);
        let rtt = attach_rtt(&mut session, &region)?;
        Ok(Dut {
            session,
            rtt,
            region,
        })
    }

    /// An over-ceiling rung can wedge the DUT's UART receive path hard enough that only a reset
    /// clears it.
    pub fn reset(&mut self) -> Result<()> {
        self.session
            .core(0)
            .context("reach core 0")?
            .reset()
            .context("reset the DUT")?;
        // A channel that doesn't answer yet is a stale attach worth retrying, not a dead DUT.
        let mut last = None;
        for _ in 0..RESET_ATTEMPTS {
            std::thread::sleep(RESET_SETTLE);
            self.rtt = attach_rtt(&mut self.session, &self.region)?;
            match self.ping() {
                Ok(()) => return Ok(()),
                Err(e) => last = Some(e),
            }
        }
        Err(last.unwrap_or_else(|| anyhow!("no attempt was made")))
            .context("the DUT did not answer PING after a reset")
    }

    pub fn ping(&mut self) -> Result<()> {
        self.exchange("PING\n", |text| text.contains(proto::OK_PONG).then_some(()))
    }

    /// Drains earlier replies first; [`Dut::await_reply`] collects the answer, so tty traffic can
    /// run concurrently.
    fn send(&mut self, line: &str) -> Result<()> {
        let mut core = self.session.core(0).context("reach core 0")?;
        let mut buf = [0u8; 256];
        {
            let up = self
                .rtt
                .up_channel(0)
                .ok_or_else(|| anyhow!("no RTT up channel"))?;
            while up.read(&mut core, &mut buf).context("RTT drain read")? > 0 {}
        }
        let down = self
            .rtt
            .down_channel(0)
            .ok_or_else(|| anyhow!("no RTT down channel"))?;
        down.write(&mut core, line.as_bytes())
            .context("RTT down-channel write")?;
        Ok(())
    }

    /// A partial trailing line is never parsed, since a truncated number could parse as a wrong
    /// one.
    fn await_reply<T>(
        &mut self,
        what: &str,
        parse: impl Fn(&str) -> Option<T>,
        deadline: Instant,
    ) -> Result<T> {
        let mut core = self.session.core(0).context("reach core 0")?;
        let up = self
            .rtt
            .up_channel(0)
            .ok_or_else(|| anyhow!("no RTT up channel"))?;
        let mut buf = [0u8; 256];
        let mut seen = Vec::new();
        loop {
            let n = up
                .read(&mut core, &mut buf)
                .context("RTT up-channel read")?;
            seen.extend_from_slice(&buf[..n]);
            let text = String::from_utf8_lossy(&seen);
            if let Some(end) = text.rfind('\n') {
                if let Some(value) = parse(&text[..=end]) {
                    return Ok(value);
                }
            }
            if Instant::now() >= deadline {
                bail!("no reply to {what:?} in time (saw {text:?})");
            }
            std::thread::sleep(POLL);
        }
    }

    fn exchange<T>(&mut self, line: &str, parse: impl Fn(&str) -> Option<T>) -> Result<T> {
        self.send(line)?;
        let deadline = Instant::now() + EXCHANGE_TIMEOUT;
        self.await_reply(line, parse, deadline)
    }

    pub fn baud_max(&mut self) -> Result<u32> {
        self.exchange("BAUD MAX\n", |text| {
            text.lines().rev().find_map(proto::baud_max_reported)
        })
    }

    /// `Some(realized)` on success, `None` when the DUT declines the rate.
    pub fn set_baud(&mut self, rate: u32) -> Result<Option<u32>> {
        self.exchange(&format!("BAUD {rate}\n"), |text| {
            if let Some(realized) = text.lines().rev().find_map(proto::baud_realized) {
                return Some(Some(realized));
            }
            text.contains(proto::ERR_UNIMPLEMENTED).then_some(None)
        })
    }

    pub fn stats(&mut self) -> Result<proto::Stats> {
        self.exchange("STATS\n", |text| {
            text.lines().rev().find_map(proto::stats_reported)
        })
    }

    pub fn stats_reset(&mut self) -> Result<()> {
        self.exchange("STATS RESET\n", |text| {
            text.contains(proto::OK_STATS_RESET).then_some(())
        })
    }

    /// Arms a run; the DUT suspends its echo and transmits and verifies on its own until its
    /// budget, with no further control traffic.
    pub fn v1_start(&mut self, line: &str) -> Result<()> {
        let armed = self.exchange(line, |text| {
            if text.contains(proto::v1::OK_V1_START) {
                return Some(true);
            }
            text.contains(proto::ERR_UNIMPLEMENTED).then_some(false)
        })?;
        if !armed {
            bail!("the DUT declined {:?}", line.trim());
        }
        Ok(())
    }

    /// A run that meets its budget ends by itself; this is only the timeout-abort path.
    pub fn v1_stop(&mut self) -> Result<()> {
        self.exchange("V1 STOP\n", |text| {
            text.contains(proto::v1::OK_V1_STOPPED).then_some(())
        })
    }

    pub fn v1_report(&mut self) -> Result<proto::v1::V1Report> {
        self.exchange("V1 REPORT\n", |text| {
            text.lines().rev().find_map(proto::v1::report_reported)
        })
    }

    pub fn v1_reset(&mut self) -> Result<()> {
        self.exchange("V1 RESET\n", |text| {
            text.contains(proto::v1::OK_V1_RESET).then_some(())
        })
    }

    /// Reads land on the RTT control block, which the DUT keeps alive in RAM for the whole run.
    pub fn dap_load(&mut self, profile: DapProfile, stop: &AtomicBool) -> Result<DapAchieved> {
        let ptr = self.rtt.ptr();
        let mut core = self.session.core(0).context("reach core 0")?;
        let mut words = [0u32; DAP_READ_WORDS];
        let started = Instant::now();
        let mut commands = 0u64;
        while !stop.load(Ordering::Relaxed) {
            let at = Instant::now();
            core.read_32(ptr, &mut words).context("DAP profile read")?;
            commands += 1;
            if let Some(gap) = profile.gap() {
                if let Some(wait) = (at + gap).checked_duration_since(Instant::now()) {
                    std::thread::sleep(wait);
                }
            }
        }
        Ok(DapAchieved {
            profile,
            commands,
            bytes: commands * DAP_READ_BYTES,
            secs: started.elapsed().as_secs_f64(),
        })
    }

    pub fn dap_command(&mut self) -> Result<()> {
        let ptr = self.rtt.ptr();
        let mut words = [0u32; DAP_READ_WORDS];
        self.session
            .core(0)
            .context("reach core 0")?
            .read_32(ptr, &mut words)
            .context("representative DAP read")
    }
}

fn attach_rtt(session: &mut Session, region: &ScanRegion) -> Result<Rtt> {
    let mut core = session.core(0).context("reach core 0")?;
    let deadline = Instant::now() + ATTACH_TIMEOUT;
    loop {
        match Rtt::attach_region(&mut core, region) {
            Ok(rtt) => return Ok(rtt),
            Err(e) if Instant::now() >= deadline => {
                return Err(anyhow!(e)).context("RTT attach failed");
            }
            Err(_) => std::thread::sleep(Duration::from_millis(100)),
        }
    }
}

fn rtt_symbol(elf: &str) -> Option<u64> {
    use object::{Object, ObjectSymbol};
    let bytes = std::fs::read(elf).ok()?;
    let file = object::File::parse(&*bytes).ok()?;
    file.symbols()
        .find(|s| s.name() == Ok("_SEGGER_RTT"))
        .map(|s| s.address())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chip_lookup_knows_the_supported_targets_and_rejects_strangers() {
        assert_eq!(chip_for("stm32f401").unwrap(), "STM32F401CEUx");
        assert!(chip_for("nope").is_err());
    }

    #[test]
    fn rtt_symbol_is_none_for_bytes_that_are_not_an_object_file() {
        assert_eq!(rtt_symbol("/nonexistent/vcom-bench-symbol.elf"), None);
    }
}
