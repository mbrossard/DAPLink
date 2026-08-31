//! The vcom case: echo a full-packet payload through the probe's USB serial port, at each baud.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use daplink_test_protocol as proto;

use crate::{rtt, util, Ctx, Failure};

/// Bauds exercised, low to high; the top one is the DUT clock's own stretch.
const BAUDS: &[u32] = &[9_600, 115_200, 921_600];

/// One full CDC packet, the overrun-relevant size.
const PAYLOAD_LEN: usize = 64;

/// How far the DUT's realized baud may sit from the requested rate.
const BAUD_TOLERANCE_PCT: u32 = 2;

/// Run each baud in turn: a mismatch or timeout is a failure; a lost probe/session is fatal.
pub(crate) fn run(cx: &mut Ctx, _full_run: bool) -> Result<(), Failure> {
    let name = port_name(cx).map_err(Failure::Failed)?;
    let dut_elf = cx.dut_elf.clone();
    let session = cx
        .session
        .as_mut()
        .ok_or_else(|| Failure::Fatal("vcom needs a successful connect".into()))?;
    let mut core = session
        .core(0)
        .map_err(|e| Failure::Fatal(util::chain(e)))?;

    // The RTT control channels are independent of the echo baud, so one attach serves all.
    let mut rtt = rtt::attach(&mut core, dut_elf.as_deref()).map_err(Failure::Fatal)?;

    // Fixed seed: the payload is synthetic but reproducible run to run.
    let payload = util::Rng::new(0x5641_434f_4d31).payload(PAYLOAD_LEN);
    for &baud in BAUDS {
        let seen = rtt::exchange(
            &mut rtt,
            &mut core,
            format!("BAUD {baud}\n").as_bytes(),
            "OK BAUD",
        )
        .map_err(|e| Failure::Failed(format!("set DUT baud {baud}: {e}")))?;
        let realized = seen
            .lines()
            .rev()
            .find_map(proto::baud_realized)
            .ok_or_else(|| {
                Failure::Failed(format!("no realized baud in the DUT reply to BAUD {baud}"))
            })?;
        if u64::from(realized.abs_diff(baud)) * 100
            > u64::from(baud) * u64::from(BAUD_TOLERANCE_PCT)
        {
            return Err(Failure::Failed(format!(
                "DUT realized {realized} for BAUD {baud}, outside the {BAUD_TOLERANCE_PCT}% budget"
            )));
        }

        let mut port = serialport::new(&name, baud)
            .timeout(Duration::from_millis(500))
            .open()
            .map_err(|e| Failure::Failed(format!("open {name} at {baud}: {e}")))?;

        // Let both ends settle at the new baud, then drain boot noise and stale echo.
        std::thread::sleep(Duration::from_millis(100));
        let mut sink = [0u8; 256];
        while matches!(port.read(&mut sink), Ok(n) if n > 0) {}

        port.write_all(&payload)
            .map_err(|e| Failure::Failed(format!("write at {baud}: {e}")))?;

        let mut got = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut buf = [0u8; PAYLOAD_LEN];
        while got.len() < payload.len() && Instant::now() < deadline {
            match port.read(&mut buf) {
                Ok(n) => got.extend_from_slice(&buf[..n]),
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(Failure::Failed(format!("read at {baud}: {e}"))),
            }
        }
        if got != payload {
            return Err(Failure::Failed(format!(
                "echo mismatch at {baud}: sent {}, got {}",
                util::hex(&payload),
                util::hex(&got)
            )));
        }
    }
    Ok(())
}

/// The serial port whose USB VID/PID/serial match the probe under test.
fn port_name(cx: &Ctx) -> Result<String, String> {
    let ports = serialport::available_ports().map_err(|e| e.to_string())?;
    ports
        .into_iter()
        .find_map(|p| match p.port_type {
            serialport::SerialPortType::UsbPort(info)
                if info.vid == cx.selector.vendor_id
                    && info.pid == cx.selector.product_id
                    && info.serial_number == cx.selector.serial_number =>
            {
                Some(p.port_name)
            }
            _ => None,
        })
        .ok_or_else(|| "no VCOM serial port for the probe".into())
}
