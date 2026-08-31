//! DUT control over probe-rs and RTT: trace-pin routing and the emitting firmware's stream.

use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use daplink_test_protocol as proto;
use probe_rs::probe::list::Lister;
use probe_rs::probe::DebugProbeSelector;
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::{Core, MemoryInterface, Permissions, Session};

/// Bound on one RTT request/response exchange.
const EXCHANGE_TIMEOUT: Duration = Duration::from_secs(3);
/// Bound on finding the RTT control block after an attach.
const RTT_ATTACH_TIMEOUT: Duration = Duration::from_secs(15);
/// Interval between RTT up-channel reads.
const POLL_INTERVAL: Duration = Duration::from_millis(20);
/// Exchanges tried before a reset DUT is called dead.
const RESET_ATTEMPTS: u32 = 5;
/// Settling time between a reset and the exchange that checks it took.
const RESET_SETTLE: Duration = Duration::from_millis(200);

/// An attached emitting target.
pub struct Dut {
    session: Session,
    elf: Option<String>,
}

impl Dut {
    /// Attach to `chip` through the probe named by `selector`, leaving the core running.
    pub fn attach(selector: &str, chip: &str, elf: Option<&str>) -> Result<Dut> {
        let selector: DebugProbeSelector = selector
            .parse()
            .with_context(|| format!("invalid probe selector '{selector}'"))?;
        let probe = Lister::new()
            .open(selector)
            .context("opening the probe for DUT setup failed")?;
        let mut session = probe
            .attach(chip, Permissions::default())
            .with_context(|| format!("attaching to {chip} failed"))?;
        {
            let mut core = session.core(0).context("reaching core 0 failed")?;
            if core.core_halted()? {
                core.run().context("resuming the DUT failed")?;
            }
        }
        Ok(Dut {
            session,
            elf: elf.map(str::to_owned),
        })
    }

    /// Put the DUT back to its boot state. A tier measured against whatever the previous
    /// instrument left running is not a measurement of this one.
    pub fn reset(&mut self) -> Result<()> {
        self.session
            .core(0)
            .context("reaching core 0")?
            .reset()
            .context("resetting the DUT")?;
        // A control block that does not answer yet is a stale attach worth retrying, not a dead DUT.
        let mut last = None;
        for _ in 0..RESET_ATTEMPTS {
            std::thread::sleep(RESET_SETTLE);
            match self.exchange("PING\n", proto::OK_PONG) {
                Ok(()) => return Ok(()),
                Err(e) => last = Some(e),
            }
        }
        Err(last.unwrap_or_else(|| anyhow!("no attempt was made")))
            .context("the DUT did not answer PING after a reset")
    }

    /// Assert asynchronous SWO routing on the debug-domain register `reg`: an attach rewrites
    /// debugger-domain state, so the DUT's own boot-time write cannot be relied on.
    pub fn route_trace(&mut self, reg: u64) -> Result<()> {
        let mut core = self.session.core(0)?;
        let cr = core
            .read_word_32(reg)
            .context("reading the trace route failed")?;
        core.write_word_32(reg, (cr & !(0b11 << 6)) | (1 << 5))
            .context("routing the trace pin failed")?;
        Ok(())
    }

    /// Reprogram the DUT's TPIU baud. The reply carries the rate the divisor realizes, which for a
    /// rate the trace clock does not divide exactly is not the rate that was asked for.
    pub fn set_swo_baud(&mut self, baud: u32) -> Result<()> {
        self.exchange(&format!("SWO CFG {baud}\n"), "OK SWO CFG")
            .with_context(|| format!("DUT refused a {baud} baud trace rate"))
    }

    /// Start or stop the DUT's continuous sequence-tagged stream.
    pub fn set_stream(&mut self, on: bool) -> Result<()> {
        let n = u8::from(on);
        self.exchange(&format!("SWO STREAM {n}\n"), &format!("OK SWO STREAM {n}"))
            .with_context(|| format!("DUT did not acknowledge SWO STREAM {n}"))
    }

    /// Send one protocol line and wait for its marker in the up channel.
    fn exchange(&mut self, line: &str, marker: &str) -> Result<()> {
        let elf = self.elf.clone();
        let mut core = self.session.core(0)?;
        let mut rtt = attach_rtt(&mut core, elf.as_deref())?;
        let down = rtt
            .down_channel(0)
            .ok_or_else(|| anyhow!("no RTT down channel"))?;
        down.write(&mut core, line.as_bytes())?;

        let channel = rtt
            .up_channel(0)
            .ok_or_else(|| anyhow!("no RTT up channel"))?;
        let mut seen = Vec::new();
        let deadline = Instant::now() + EXCHANGE_TIMEOUT;
        while Instant::now() < deadline {
            let mut buf = [0u8; 256];
            let n = channel.read(&mut core, &mut buf)?;
            seen.extend_from_slice(&buf[..n]);
            let text = String::from_utf8_lossy(&seen);
            if text.contains(marker) {
                return Ok(());
            }
            if text.contains(proto::ERR_UNIMPLEMENTED) {
                bail!("DUT answered `{}`", proto::ERR_UNIMPLEMENTED);
            }
            std::thread::sleep(POLL_INTERVAL);
        }
        bail!(
            "no `{marker}` from the DUT within {EXCHANGE_TIMEOUT:?} (saw {:?})",
            String::from_utf8_lossy(&seen)
        )
    }
}

/// Attach RTT at the ELF's `_SEGGER_RTT` symbol when a DUT image is at hand; a full target-RAM
/// scan takes seconds on a slow wire.
fn attach_rtt(core: &mut Core, elf: Option<&str>) -> Result<Rtt> {
    let region = elf
        .and_then(rtt_symbol)
        .map_or(ScanRegion::default(), ScanRegion::Exact);
    let deadline = Instant::now() + RTT_ATTACH_TIMEOUT;
    loop {
        match Rtt::attach_region(core, &region) {
            Ok(rtt) => return Ok(rtt),
            Err(e) if Instant::now() < deadline => {
                let _ = e;
                std::thread::sleep(Duration::from_millis(100));
            }
            Err(e) => return Err(anyhow!("RTT attach failed: {e}")),
        }
    }
}

/// The `_SEGGER_RTT` control-block address in `elf`, if it can be read.
fn rtt_symbol(elf: &str) -> Option<u64> {
    use object::{Object, ObjectSymbol};
    let bytes = std::fs::read(elf).ok()?;
    let file = object::File::parse(&*bytes).ok()?;
    file.symbols()
        .find(|s| s.name() == Ok("_SEGGER_RTT"))
        .map(|s| s.address())
}
