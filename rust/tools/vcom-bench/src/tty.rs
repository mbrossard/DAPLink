//! The probe's USB serial port: line-coding reprogramming, the v1 host halves, and the echo.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use dap_bench::transport::ProbeSelector;
use daplink_test_protocol::v1::{Encoder, RxCounters, TxCounters, Verifier};
use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, ControlOut, ControlType, Direction, In, Out, Recipient};
use nusb::{Endpoint, Interface, MaybeFuture};
use serialport::SerialPort;

use crate::stream::{Clock, Plan};

/// Forces a genuine line-coding change even if the probe was already left at the target rate.
const SCRUB_BAUD: u32 = 9_600;
/// The scrub rate used when the target rate is the scrub rate itself.
const SCRUB_BAUD_ALT: u32 = 115_200;
const BAUD_REQUEST_GAP: Duration = Duration::from_millis(20);

const READ_TIMEOUT: Duration = Duration::from_millis(50);
const DRAIN_QUIET: Duration = Duration::from_millis(150);
const DRAIN_MAX: Duration = Duration::from_secs(2);
/// Added to every echo-read deadline, over the line-rate transfer time.
const READ_GRACE: Duration = Duration::from_millis(500);
const STREAM_POLL: Duration = Duration::from_millis(10);
const WRITE_CHUNK: usize = 4096;
const READ_CHUNK: usize = 8192;
const IDLE_POLL: Duration = Duration::from_millis(1);
const WRITE_TIMEOUT: Duration = Duration::from_secs(10);
const USB_TIMEOUT: Duration = Duration::from_secs(1);
const CDC_COMM_CLASS: u8 = 0x02;
const CDC_ACM_SUBCLASS: u8 = 0x02;
const CDC_DATA_CLASS: u8 = 0x0a;
const SET_LINE_CODING: u8 = 0x20;

pub struct ClaimedCdc {
    out: Endpoint<Bulk, Out>,
    in_: Endpoint<Bulk, In>,
    comm: Interface,
    _data: Interface,
    _restore: DriverRestore,
}

struct DriverRestore {
    device: nusb::Device,
    comm: u8,
    data: u8,
}

impl Drop for DriverRestore {
    fn drop(&mut self) {
        let _ = self.device.attach_kernel_driver(self.comm);
        let _ = self.device.attach_kernel_driver(self.data);
    }
}

impl ClaimedCdc {
    pub fn open(selector: &str) -> Result<Self> {
        let selector: ProbeSelector = selector.parse()?;
        let serial = selector
            .serial
            .as_deref()
            .context("claimed CDC requires a serial-qualified probe selector")?;
        let devices: Vec<_> = nusb::list_devices()
            .wait()
            .context("enumerate USB devices for CDC")?
            .filter(|device| {
                device.vendor_id() == selector.vendor_id
                    && device.product_id() == selector.product_id
                    && device.serial_number() == Some(serial)
            })
            .collect();
        let [info] = devices.as_slice() else {
            bail!("selector {selector} matched {} USB devices", devices.len());
        };
        let comm_number = unique_interface(info, CDC_COMM_CLASS, Some(CDC_ACM_SUBCLASS))?;
        let data_number = unique_interface(info, CDC_DATA_CLASS, None)?;
        let device = info.open().wait().context("open CDC device")?;
        let restore = DriverRestore {
            device: device.clone(),
            comm: comm_number,
            data: data_number,
        };
        let comm = device
            .detach_and_claim_interface(comm_number)
            .wait()
            .context("detach and claim CDC communication interface")?;
        let data = device
            .detach_and_claim_interface(data_number)
            .wait()
            .context("detach and claim CDC data interface")?;
        let descriptor = data.descriptor().context("CDC data interface descriptor")?;
        let mut out_address = None;
        let mut in_address = None;
        for endpoint in descriptor.endpoints() {
            if endpoint.transfer_type() != TransferType::Bulk {
                continue;
            }
            match endpoint.direction() {
                Direction::Out => out_address = Some(endpoint.address()),
                Direction::In => in_address = Some(endpoint.address()),
            }
        }
        let out = data.endpoint::<Bulk, Out>(out_address.context("CDC bulk OUT endpoint")?)?;
        let in_ = data.endpoint::<Bulk, In>(in_address.context("CDC bulk IN endpoint")?)?;
        Ok(Self {
            out,
            in_,
            comm,
            _data: data,
            _restore: restore,
        })
    }

    pub fn scrub_baud(target: u32) -> u32 {
        Tty::scrub_baud(target)
    }

    pub fn set_baud(&self, baud: u32) -> Result<()> {
        let data = line_coding(baud);
        self.comm
            .control_out(
                ControlOut {
                    control_type: ControlType::Class,
                    recipient: Recipient::Interface,
                    request: SET_LINE_CODING,
                    value: 0,
                    index: u16::from(self.comm.interface_number()),
                    data: &data,
                },
                USB_TIMEOUT,
            )
            .wait()
            .context("CDC SET_LINE_CODING")
    }

    pub fn echo(&mut self, payload: &[u8]) -> Result<Ledger> {
        let started = Instant::now();
        let deadline = started + USB_TIMEOUT;
        let read_len = usb_read_len(payload.len(), self.in_.max_packet_size());
        self.in_.submit(Buffer::new(read_len));
        if let Err(error) = self
            .out
            .transfer_blocking(payload.to_vec().into(), USB_TIMEOUT)
            .into_result()
        {
            cancel_pending_in(&mut self.in_);
            return Err(error).context("CDC bulk write");
        }
        let mut got = Vec::with_capacity(payload.len());
        loop {
            let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
                cancel_pending_in(&mut self.in_);
                break;
            };
            let Some(completion) = self.in_.wait_next_complete(remaining) else {
                cancel_pending_in(&mut self.in_);
                break;
            };
            let fragment = completion.into_result().context("CDC bulk read")?;
            if append_echo_fragment(&mut got, &fragment, payload.len()) {
                break;
            }
            self.in_.submit(Buffer::new(read_len));
        }
        Ok(Ledger::new(
            payload.len() as u64,
            got.len() as u64,
            divergence(payload, &got),
            started.elapsed(),
        ))
    }
}

fn cancel_pending_in(endpoint: &mut Endpoint<Bulk, In>) {
    endpoint.cancel_all();
    if endpoint.pending() != 0 {
        let _ = endpoint.wait_next_complete(Duration::from_secs(1));
    }
}

fn append_echo_fragment(got: &mut Vec<u8>, fragment: &[u8], wanted: usize) -> bool {
    got.extend_from_slice(fragment);
    got.len() >= wanted
}

fn unique_interface(info: &nusb::DeviceInfo, class: u8, subclass: Option<u8>) -> Result<u8> {
    let matches: Vec<_> = info
        .interfaces()
        .filter(|interface| {
            interface.class() == class
                && subclass.is_none_or(|subclass| interface.subclass() == subclass)
        })
        .map(|interface| interface.interface_number())
        .collect();
    let [number] = matches.as_slice() else {
        bail!(
            "expected one USB interface class {class:#04x}, found {}",
            matches.len()
        );
    };
    Ok(*number)
}

fn line_coding(baud: u32) -> [u8; 7] {
    let mut data = [0; 7];
    data[..4].copy_from_slice(&baud.to_le_bytes());
    data[6] = 8;
    data
}

fn usb_read_len(payload_len: usize, max_packet_size: usize) -> usize {
    payload_len.div_ceil(max_packet_size) * max_packet_size
}
#[derive(Debug, Clone)]
pub struct Ledger {
    pub written: u64,
    pub read: u64,
    /// Whether every byte read matched the expected sequence, in order, to the full length.
    pub byte_exact: bool,
    /// Offset of the first mismatching or missing byte, when not byte-exact.
    pub first_divergence: Option<u64>,
    /// First write to last byte read (or last write when nothing came back).
    pub elapsed: Duration,
    pub throughput_bps: f64,
}

impl Ledger {
    fn new(written: u64, read: u64, first_divergence: Option<u64>, elapsed: Duration) -> Self {
        let byte_exact = first_divergence.is_none() && read == written && written > 0;
        let throughput_bps = if elapsed.as_secs_f64() > 0.0 {
            read as f64 / elapsed.as_secs_f64()
        } else {
            0.0
        };
        Ledger {
            written,
            read,
            byte_exact,
            first_divergence,
            elapsed,
            throughput_bps,
        }
    }
}

pub fn find_port(vid: u16, pid: u16, serial: &str) -> Result<String> {
    if let Some(port) = find_port_once(vid, pid, serial)? {
        return Ok(port);
    }
    attach_cdc_drivers(vid, pid, serial)?;
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if let Some(port) = find_port_once(vid, pid, serial)? {
            return Ok(port);
        }
        if Instant::now() >= deadline {
            bail!(
                "CDC tty did not return after attaching drivers for {vid:04x}:{pid:04x}:{serial}"
            );
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn find_port_once(vid: u16, pid: u16, serial: &str) -> Result<Option<String>> {
    let ports = serialport::available_ports().context("serial port enumeration failed")?;
    Ok(ports.into_iter().find_map(|p| match p.port_type {
        serialport::SerialPortType::UsbPort(info)
            if info.vid == vid
                && info.pid == pid
                && info.serial_number.as_deref() == Some(serial) =>
        {
            Some(p.port_name)
        }
        _ => None,
    }))
}

fn attach_cdc_drivers(vid: u16, pid: u16, serial: &str) -> Result<()> {
    let devices: Vec<_> = nusb::list_devices()
        .wait()
        .context("enumerate USB devices for CDC recovery")?
        .filter(|device| {
            device.vendor_id() == vid
                && device.product_id() == pid
                && device.serial_number() == Some(serial)
        })
        .collect();
    let [info] = devices.as_slice() else {
        bail!(
            "{vid:04x}:{pid:04x}:{serial} matched {} USB devices for CDC recovery",
            devices.len()
        );
    };
    let comm = unique_interface(info, CDC_COMM_CLASS, Some(CDC_ACM_SUBCLASS))?;
    let data = unique_interface(info, CDC_DATA_CLASS, None)?;
    let device = info.open().wait().context("open CDC recovery device")?;
    let _ = device.attach_kernel_driver(comm);
    let _ = device.attach_kernel_driver(data);
    Ok(())
}

/// An open probe serial port at a forced-fresh baud.
pub struct Tty {
    port: Box<dyn SerialPort>,
}

impl Tty {
    pub fn scrub_baud(target: u32) -> u32 {
        if target == SCRUB_BAUD {
            SCRUB_BAUD_ALT
        } else {
            SCRUB_BAUD
        }
    }

    pub fn open(name: &str, baud: u32) -> Result<Tty> {
        let scrub = Self::scrub_baud(baud);
        let mut port = serialport::new(name, scrub)
            .timeout(READ_TIMEOUT)
            .open()
            .with_context(|| format!("open {name}"))?;
        port.set_baud_rate(baud)
            .with_context(|| format!("set {name} to {baud} baud"))?;
        Ok(Tty { port })
    }

    pub fn set_baud(&mut self, baud: u32) -> Result<()> {
        let scrub = Self::scrub_baud(baud);
        self.port.set_baud_rate(scrub)?;
        std::thread::sleep(BAUD_REQUEST_GAP);
        self.port.set_baud_rate(baud)?;
        Ok(())
    }

    pub fn set_open(&mut self, open: bool) -> Result<()> {
        self.port
            .write_data_terminal_ready(open)
            .context("set CDC DTR")
    }

    /// Write the whole payload before application reads begin, then require the echo to recover.
    pub fn echo_after_reader_pause(
        &mut self,
        payload: &[u8],
        pause: Duration,
        baud: u32,
    ) -> Result<Ledger> {
        let start = Instant::now();
        self.port.set_timeout(WRITE_TIMEOUT)?;
        self.port
            .write_all(payload)
            .context("paused-reader write")?;
        self.port.set_timeout(READ_TIMEOUT)?;
        std::thread::sleep(pause);
        let deadline = Instant::now() + echo_time(payload.len() as u64, baud) + READ_GRACE;
        let mut got = Vec::with_capacity(payload.len());
        let mut last_read = Instant::now();
        let mut buf = [0u8; 4096];
        while got.len() < payload.len() && Instant::now() < deadline {
            match self.port.read(&mut buf) {
                Ok(n) if n > 0 => {
                    got.extend_from_slice(&buf[..n]);
                    last_read = Instant::now();
                }
                Ok(_) => {}
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(e).context("paused-reader read"),
            }
        }
        Ok(Ledger::new(
            payload.len() as u64,
            got.len() as u64,
            divergence(payload, &got),
            last_read.duration_since(start),
        ))
    }

    /// Read and discard until the line is quiet, so transition garbage never enters a ledger.
    pub fn drain(&mut self) -> Result<()> {
        let start = Instant::now();
        let mut quiet_since = Instant::now();
        let mut sink = [0u8; 512];
        while start.elapsed() < DRAIN_MAX {
            match self.port.read(&mut sink) {
                Ok(n) if n > 0 => quiet_since = Instant::now(),
                Ok(_) => {}
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(e).context("drain read"),
            }
            if quiet_since.elapsed() >= DRAIN_QUIET {
                return Ok(());
            }
        }
        bail!("the line never went quiet in {DRAIN_MAX:?}; something is streaming")
    }

    /// Paced echo: write one chunk, require its echo back before the next chunk goes out.
    pub fn echo_paced(&mut self, payload: &[u8], chunk: usize, baud: u32) -> Result<Ledger> {
        let start = Instant::now();
        let mut got: Vec<u8> = Vec::with_capacity(payload.len());
        let mut last_read = start;
        'chunks: for part in payload.chunks(chunk.max(1)) {
            self.port.write_all(part).context("paced write")?;
            let deadline = Instant::now() + echo_time(part.len() as u64, baud) + READ_GRACE;
            let want = got.len() + part.len();
            let mut buf = [0u8; 4096];
            while got.len() < want {
                match self.port.read(&mut buf) {
                    Ok(n) if n > 0 => {
                        got.extend_from_slice(&buf[..n]);
                        last_read = Instant::now();
                    }
                    Ok(_) => {}
                    Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(e) => return Err(e).context("paced read"),
                }
                if Instant::now() > deadline {
                    break 'chunks;
                }
            }
        }
        let elapsed = last_read.duration_since(start);
        Ok(Ledger::new(
            payload.len() as u64,
            got.len() as u64,
            divergence(payload, &got),
            elapsed,
        ))
    }

    /// A reader thread collects the echo concurrently, so a full kernel buffer can't deadlock the
    /// writer.
    pub fn echo_batched(&mut self, payload: &[u8], chunk: usize, baud: u32) -> Result<Ledger> {
        let mut reader = self
            .port
            .try_clone()
            .context("clone port for the echo reader")?;
        let want = payload.len();
        let deadline = echo_time(want as u64, baud) + READ_GRACE;
        let handle = std::thread::spawn(move || -> std::io::Result<(Vec<u8>, Instant)> {
            let start = Instant::now();
            let mut got = Vec::with_capacity(want);
            let mut last_read = start;
            let mut buf = [0u8; 4096];
            while got.len() < want && start.elapsed() < deadline {
                match reader.read(&mut buf) {
                    Ok(n) if n > 0 => {
                        got.extend_from_slice(&buf[..n]);
                        last_read = Instant::now();
                    }
                    Ok(_) => {}
                    Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(e) => return Err(e),
                }
            }
            Ok((got, last_read))
        });

        // A write can legitimately block for the transfer time; a short timeout would misreport
        // that as dead.
        self.port.set_timeout(deadline)?;
        let start = Instant::now();
        let mut write_err = None;
        for part in payload.chunks(chunk.max(1)) {
            if let Err(e) = self.port.write_all(part) {
                write_err = Some(e);
                break;
            }
        }
        self.port.set_timeout(READ_TIMEOUT)?;
        let (got, last_read) = handle
            .join()
            .map_err(|_| anyhow!("echo reader thread panicked"))?
            .context("batched read")?;
        if let Some(e) = write_err {
            return Err(e).context("batched write");
        }
        let elapsed = last_read.duration_since(start.min(last_read));
        Ok(Ledger::new(
            payload.len() as u64,
            got.len() as u64,
            divergence(payload, &got),
            elapsed,
        ))
    }

    /// Must return before the DUT is told to start, since every byte of the stream must fall inside
    /// the measured window.
    pub fn v1_reader(&mut self, verifier: Verifier, clock: Clock) -> Result<Reader> {
        let mut port = self
            .port
            .try_clone()
            .context("clone port for the v1 reader")?;
        port.set_timeout(STREAM_POLL)?;
        let stop = Arc::new(AtomicBool::new(false));
        let done = Arc::new(AtomicBool::new(false));
        let verified = Arc::new(AtomicU64::new(0));
        let ledger = Arc::new(Mutex::new(verifier.counters()));
        let handle = {
            let (stop, done, verified) = (stop.clone(), done.clone(), verified.clone());
            let ledger = ledger.clone();
            std::thread::spawn(move || -> Result<RxCounters, String> {
                let mut verifier = verifier;
                let mut buf = [0u8; READ_CHUNK];
                while !stop.load(Ordering::Relaxed) {
                    match port.read(&mut buf) {
                        Ok(n) if n > 0 => {
                            verifier.feed(&buf[..n], clock.now_us());
                            let counters = verifier.counters();
                            verified.store(counters.verified, Ordering::Relaxed);
                            // Published for the window's checkpoints, which only read it.
                            *ledger.lock().unwrap_or_else(|e| e.into_inner()) = counters;
                            done.store(verifier.complete(), Ordering::Relaxed);
                        }
                        Ok(_) => {}
                        Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                        Err(e) => return Err(e.to_string()),
                    }
                }
                Ok(verifier.counters())
            })
        };
        Ok(Reader {
            stop,
            done,
            verified,
            ledger,
            handle: Some(handle),
        })
    }

    /// Call only once the DUT has acknowledged `V1 START`, so no byte reaches an unarmed verifier.
    pub fn v1_writer(
        &mut self,
        encoder: Encoder,
        pace_bps: f64,
        clock: Clock,
        credit: Option<(Plan, Arc<AtomicU64>)>,
    ) -> Result<Writer> {
        let mut port = self
            .port
            .try_clone()
            .context("clone port for the v1 writer")?;
        // A bridge draining slower than offered backpressures the write; a short timeout would call
        // it dead.
        port.set_timeout(WRITE_TIMEOUT)?;
        let stop = Arc::new(AtomicBool::new(false));
        let close = Arc::new(AtomicBool::new(false));
        let done = Arc::new(AtomicBool::new(false));
        let granted = Arc::new(AtomicU64::new(0));
        let ledger = Arc::new(Mutex::new(encoder.counters()));
        let handle = {
            let (stop, close, done) = (stop.clone(), close.clone(), done.clone());
            let (granted, ledger) = (granted.clone(), ledger.clone());
            std::thread::spawn(move || -> Result<TxCounters, String> {
                let mut encoder = encoder;
                let mut buf = [0u8; WRITE_CHUNK];
                let started = Instant::now();
                let mut written = 0u64;
                while !stop.load(Ordering::Relaxed) {
                    if close.load(Ordering::Relaxed) {
                        encoder.close();
                    }
                    if let Some((plan, verified)) = &credit {
                        let grant = plan.grant_for(verified.load(Ordering::Relaxed));
                        encoder.queue_credit(grant);
                        granted.store(grant, Ordering::Relaxed);
                    }
                    let n = encoder.fill(&mut buf, clock.now_us());
                    if n == 0 {
                        if encoder.complete() {
                            done.store(true, Ordering::Relaxed);
                            break;
                        }
                        // A burst gap or an exhausted grant, not the end of the run.
                        std::thread::sleep(IDLE_POLL);
                        continue;
                    }
                    port.write_all(&buf[..n]).map_err(|e| e.to_string())?;
                    // Published for the window's checkpoints, which only read it.
                    *ledger.lock().unwrap_or_else(|e| e.into_inner()) = encoder.counters();
                    written += n as u64;
                    // Token bucket against the run's own start, so a write that fell behind catches
                    // up.
                    let due = started + Duration::from_secs_f64(written as f64 / pace_bps.max(1.0));
                    if let Some(wait) = due.checked_duration_since(Instant::now()) {
                        std::thread::sleep(wait);
                    }
                }
                Ok(encoder.counters())
            })
        };
        Ok(Writer {
            stop,
            close,
            done,
            granted,
            ledger,
            handle: Some(handle),
        })
    }
}

pub struct Reader {
    stop: Arc<AtomicBool>,
    done: Arc<AtomicBool>,
    verified: Arc<AtomicU64>,
    ledger: Arc<Mutex<RxCounters>>,
    /// Stopped and joined on drop, so an early return can't leave a thread reading behind.
    handle: Option<JoinHandle<Result<RxCounters, String>>>,
}

impl Drop for Reader {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Reader {
    /// The end-of-stream frame has arrived.
    pub fn done(&self) -> bool {
        self.done.load(Ordering::Relaxed)
    }

    /// Payload bytes verified so far, which is what the credit policy grants against.
    pub fn verified(&self) -> Arc<AtomicU64> {
        self.verified.clone()
    }

    /// As of the last read, not the moment of the call.
    pub fn snapshot(&self) -> RxCounters {
        *self.ledger.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn finish(mut self) -> Result<RxCounters> {
        self.stop.store(true, Ordering::Relaxed);
        self.handle
            .take()
            .context("the v1 reader was already finished")?
            .join()
            .map_err(|_| anyhow!("the v1 reader panicked"))?
            .map_err(|e| anyhow!("v1 read: {e}"))
    }
}

pub struct Writer {
    stop: Arc<AtomicBool>,
    close: Arc<AtomicBool>,
    done: Arc<AtomicBool>,
    granted: Arc<AtomicU64>,
    ledger: Arc<Mutex<TxCounters>>,
    /// Stopped and joined on drop, so an early return can't leave a thread writing behind.
    handle: Option<JoinHandle<Result<TxCounters, String>>>,
}

impl Drop for Writer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Writer {
    /// The whole budget, end-of-stream frame included, has left the encoder.
    pub fn done(&self) -> bool {
        self.done.load(Ordering::Relaxed)
    }

    /// End the stream after the span in flight, so the far end still gets an end-of-stream frame.
    pub fn close(&self) {
        self.close.store(true, Ordering::Relaxed);
    }

    /// The highest cumulative grant the writer announced.
    pub fn granted(&self) -> u64 {
        self.granted.load(Ordering::Relaxed)
    }

    /// As of the last write, not the moment of the call.
    pub fn snapshot(&self) -> TxCounters {
        *self.ledger.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn finish(mut self) -> Result<TxCounters> {
        self.stop.store(true, Ordering::Relaxed);
        self.handle
            .take()
            .context("the v1 writer was already finished")?
            .join()
            .map_err(|_| anyhow!("the v1 writer panicked"))?
            .map_err(|e| anyhow!("v1 write: {e}"))
    }
}

/// Line-rate transfer time for `bytes` at `baud`, 10 bits per byte.
pub fn transfer_time(bytes: u64, baud: u32) -> Duration {
    Duration::from_secs_f64(bytes as f64 * 10.0 / f64::from(baud.max(1)))
}

fn echo_time(bytes: u64, baud: u32) -> Duration {
    transfer_time(bytes.saturating_mul(2), baud)
}

/// Offset of the first byte where `got` stops matching `want`, `None` on a full match.
fn divergence(want: &[u8], got: &[u8]) -> Option<u64> {
    if let Some(i) = want.iter().zip(got.iter()).position(|(w, g)| w != g) {
        return Some(i as u64);
    }
    (got.len() != want.len()).then(|| want.len().min(got.len()) as u64)
}

/// The shape-controlled test payload: a rolling counter, so a displaced byte is visible.
pub fn pattern(len: usize) -> Vec<u8> {
    (0..len).map(|i| i as u8).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divergence_finds_the_first_bad_or_missing_byte() {
        assert_eq!(divergence(&[1, 2, 3], &[1, 2, 3]), None);
        assert_eq!(divergence(&[1, 2, 3], &[1, 9, 3]), Some(1));
        assert_eq!(divergence(&[1, 2, 3], &[1, 2]), Some(2));
        assert_eq!(divergence(&[1, 2, 3], &[1, 2, 3, 4]), Some(3));
        assert_eq!(divergence(&[], &[]), None);
    }

    #[test]
    fn transfer_time_is_ten_bits_per_byte() {
        assert_eq!(transfer_time(9_600, 96_000), Duration::from_secs(1));
        assert_eq!(transfer_time(0, 9_600), Duration::ZERO);
    }

    #[test]
    fn echo_time_allows_a_full_round_trip() {
        assert_eq!(echo_time(4_800, 96_000), Duration::from_secs(1));
        assert_eq!(echo_time(0, 9_600), Duration::ZERO);
    }

    #[test]
    fn scrub_baud_always_differs_from_the_target() {
        assert_eq!(Tty::scrub_baud(115_200), 9_600);
        assert_eq!(Tty::scrub_baud(9_600), 115_200);
    }

    #[test]
    fn claimed_cdc_line_coding_is_8n1_little_endian() {
        assert_eq!(line_coding(921_600), [0x00, 0x10, 0x0e, 0x00, 0, 0, 8]);
    }

    #[test]
    fn claimed_cdc_reads_use_whole_endpoint_packets() {
        assert_eq!(usb_read_len(64, 64), 64);
        assert_eq!(usb_read_len(64, 512), 512);
        assert_eq!(usb_read_len(513, 512), 1024);
    }

    #[test]
    fn claimed_cdc_accumulates_one_byte_and_short_completions() {
        let payload = pattern(64);
        let mut got = Vec::new();
        for byte in &payload[..32] {
            assert!(!append_echo_fragment(&mut got, &[*byte], payload.len()));
        }
        assert!(append_echo_fragment(
            &mut got,
            &payload[32..],
            payload.len()
        ));
        assert_eq!(divergence(&payload, &got), None);
    }

    #[test]
    fn claimed_cdc_timeout_keeps_the_partial_ledger() {
        let payload = pattern(64);
        let mut got = Vec::new();
        assert!(!append_echo_fragment(
            &mut got,
            &payload[..7],
            payload.len()
        ));
        let ledger = Ledger::new(
            payload.len() as u64,
            got.len() as u64,
            divergence(&payload, &got),
            USB_TIMEOUT,
        );
        assert_eq!(ledger.read, 7);
        assert_eq!(ledger.first_divergence, Some(7));
        assert!(!ledger.byte_exact);
    }

    #[test]
    fn claimed_cdc_preserves_completion_overshoot() {
        let payload = pattern(64);
        let mut fragment = payload.clone();
        fragment.push(0xaa);
        let mut got = Vec::new();
        assert!(append_echo_fragment(&mut got, &fragment, payload.len()));
        assert_eq!(got.len(), 65);
        assert_eq!(divergence(&payload, &got), Some(64));
    }

    #[test]
    fn a_ledger_is_byte_exact_only_on_a_full_clean_round_trip() {
        let full = Ledger::new(64, 64, None, Duration::from_millis(10));
        assert!(full.byte_exact);
        let short = Ledger::new(64, 63, Some(63), Duration::from_millis(10));
        assert!(!short.byte_exact);
        let empty = Ledger::new(0, 0, None, Duration::from_millis(10));
        assert!(!empty.byte_exact, "an empty run proves nothing");
    }
}
