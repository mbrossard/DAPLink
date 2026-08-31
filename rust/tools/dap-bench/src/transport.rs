//! USB transport: CMSIS-DAP v2 bulk discovery, and serial or pipelined command submission.

use std::collections::VecDeque;
use std::fmt;
use std::str::FromStr;
use std::time::Duration;

use anyhow::{anyhow, bail, Context, Result};
use nusb::descriptors::language_id::US_ENGLISH;
use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, Direction, In, Out};
use nusb::{Endpoint, Interface, MaybeFuture};

/// Interface string substring that identifies a CMSIS-DAP v2 bulk interface.
const CMSIS_DAP_SIGNATURE: &str = "CMSIS-DAP";

/// `bInterfaceClass` for a vendor-specific interface (what CMSIS-DAP v2 advertises).
const VENDOR_SPECIFIC_CLASS: u8 = 0xFF;

/// Timeout for the control transfers used to read string descriptors during discovery.
const STRING_DESCRIPTOR_TIMEOUT: Duration = Duration::from_millis(200);

/// Response read size, before rounding up to a multiple of the endpoint's max packet size.
const RESPONSE_BUFFER_HINT: usize = 4096;

/// The one-byte reply a CMSIS-DAP engine sends for a command it will not run.
pub const INVALID_COMMAND: u8 = 0xFF;

/// Environment variable selecting the transport trace level.
pub const TRACE_ENV: &str = "DAPBENCH_TRACE";

/// Environment variable suppressing the claim-time boundary flush, so a hardware repro can tell
/// a leftover response apart from the cancelled read that removes it.
pub const NO_CLAIM_FLUSH_ENV: &str = "DAPBENCH_NO_CLAIM_FLUSH";

/// Environment variable naming the probe for callers that take no `--probe` flag.
pub const PROBE_ENV: &str = "DAPBENCH_PROBE";

/// Packets a boundary flush discards before it gives up and reports the endpoint dirty.
const BOUNDARY_FLUSH_MAX: usize = 16;

/// How long a boundary flush waits for a stale response before calling the endpoint quiet.
const BOUNDARY_FLUSH_TIMEOUT: Duration = Duration::from_millis(30);

/// Transfer records kept for the on-error window dump.
const HISTORY_LEN: usize = 64;

/// Records replayed by the on-error window dump.
const DUMP_LEN: usize = 24;

/// Bytes of a buffer shown in a trace line.
const TRACE_HEAD_BYTES: usize = 8;

/// Identity of a discovered CMSIS-DAP v2 bulk interface.
#[derive(Clone)]
pub struct ProbeInfo {
    /// USB vendor id.
    pub vendor_id: u16,
    /// USB product id.
    pub product_id: u16,
    /// USB serial-number string, when the device exposes one.
    pub serial: Option<String>,
    /// USB product string, when the device exposes one.
    pub product: Option<String>,
    /// `bInterfaceNumber` of the matched interface.
    pub interface_number: u8,
    /// Bulk OUT endpoint address (command direction).
    pub out_endpoint: u8,
    /// Bulk IN endpoint address (response direction).
    pub in_endpoint: u8,
    /// Max packet size shared by the OUT/IN endpoint pair.
    pub max_packet_size: usize,

    /// The `nusb` device this was enumerated from, so [`Probe::open`] can reopen it directly.
    device_info: nusb::DeviceInfo,
}

/// Enumerate every CMSIS-DAP v2 bulk interface currently attached to the host.
pub fn enumerate() -> Vec<ProbeInfo> {
    let devices = match nusb::list_devices().wait() {
        Ok(devices) => devices,
        Err(err) => {
            eprintln!("warning: USB device enumeration failed: {err}");
            return Vec::new();
        }
    };

    devices.filter_map(probe_from_device).collect()
}

/// A device's `ProbeInfo` if its default alt-setting exposes a CMSIS-DAP v2 bulk interface.
fn probe_from_device(device_info: nusb::DeviceInfo) -> Option<ProbeInfo> {
    let device = device_info.open().wait().ok()?;
    let config = device.active_configuration().ok()?;

    let languages: Vec<u16> = device
        .get_string_descriptor_supported_languages(STRING_DESCRIPTOR_TIMEOUT)
        .wait()
        .map(Iterator::collect)
        .unwrap_or_default();
    let language = languages.first().copied().unwrap_or(US_ENGLISH);

    for group in config.interfaces() {
        let alt = group.first_alt_setting();
        if alt.class() != VENDOR_SPECIFIC_CLASS {
            continue;
        }
        let Some(string_index) = alt.string_index() else {
            continue;
        };
        let Ok(string) = device
            .get_string_descriptor(string_index, language, STRING_DESCRIPTOR_TIMEOUT)
            .wait()
        else {
            continue;
        };
        if !string.contains(CMSIS_DAP_SIGNATURE) {
            continue;
        }

        let mut out_ep = None;
        let mut in_ep = None;
        for ep in alt.endpoints() {
            if ep.transfer_type() != TransferType::Bulk {
                continue;
            }
            match ep.direction() {
                Direction::Out if out_ep.is_none() => out_ep = Some(ep),
                Direction::In if in_ep.is_none() => in_ep = Some(ep),
                _ => {}
            }
        }
        let (Some(out), Some(in_)) = (out_ep, in_ep) else {
            continue;
        };

        return Some(ProbeInfo {
            vendor_id: device_info.vendor_id(),
            product_id: device_info.product_id(),
            serial: device_info.serial_number().map(str::to_owned),
            product: device_info.product_string().map(str::to_owned),
            interface_number: alt.interface_number(),
            out_endpoint: out.address(),
            in_endpoint: in_.address(),
            max_packet_size: out.max_packet_size().min(in_.max_packet_size()),
            device_info: device_info.clone(),
        });
    }

    None
}

/// A parsed `--probe` selector (`VID:PID[:serial]`, or probe-rs's `VID:PID-N[:serial]`).
pub struct ProbeSelector {
    /// USB vendor id to match.
    pub vendor_id: u16,
    /// USB product id to match.
    pub product_id: u16,
    /// Serial number to match, if given.
    pub serial: Option<String>,
}

impl FromStr for ProbeSelector {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        let mut parts = s.splitn(3, ':');
        let vendor_id = parts
            .next()
            .filter(|s| !s.is_empty())
            .ok_or_else(|| anyhow!("probe selector '{s}' is missing a vendor id"))?;
        let product_id = parts.next().filter(|s| !s.is_empty()).ok_or_else(|| {
            anyhow!("probe selector '{s}' is missing a product id (expected VID:PID[:SERIAL])")
        })?;
        let serial = parts.next().map(str::to_owned);

        let product_id_hex = product_id.split('-').next().unwrap_or(product_id);

        let vendor_id = u16::from_str_radix(vendor_id, 16).with_context(|| {
            format!("invalid hex vendor id '{vendor_id}' in probe selector '{s}'")
        })?;
        let product_id = u16::from_str_radix(product_id_hex, 16).with_context(|| {
            format!("invalid hex product id '{product_id}' in probe selector '{s}'")
        })?;

        Ok(ProbeSelector {
            vendor_id,
            product_id,
            serial,
        })
    }
}

impl fmt::Display for ProbeSelector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.serial {
            Some(serial) => write!(f, "{:04x}:{:04x}:{serial}", self.vendor_id, self.product_id),
            None => write!(f, "{:04x}:{:04x}", self.vendor_id, self.product_id),
        }
    }
}

/// Filter `probes` down to the single probe matching `selector`.
pub fn select<'a>(probes: &'a [ProbeInfo], selector: &ProbeSelector) -> Result<&'a ProbeInfo> {
    let matches: Vec<&ProbeInfo> = probes
        .iter()
        .filter(|p| p.vendor_id == selector.vendor_id && p.product_id == selector.product_id)
        .filter(|p| match &selector.serial {
            Some(serial) => p.serial.as_deref() == Some(serial.as_str()),
            None => true,
        })
        .collect();

    match matches.as_slice() {
        [] => Err(anyhow!("no CMSIS-DAP probe matches selector '{selector}'")),
        [single] => Ok(single),
        many => Err(anyhow!(
            "selector '{selector}' matches {} probes; add a serial to disambiguate:\n{}",
            many.len(),
            list_candidates(many)
        )),
    }
}

/// One indexed line per probe, for an error that has to say which ones it could not choose
/// between.
fn list_candidates(probes: &[&ProbeInfo]) -> String {
    probes
        .iter()
        .enumerate()
        .map(|(i, p)| {
            format!(
                "  [{i}] {:04x}:{:04x} serial={}",
                p.vendor_id,
                p.product_id,
                p.serial.as_deref().unwrap_or("<none>")
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// The probe `spec` names, or the only one attached. Several probes and no `spec` is an error,
/// `chooser` naming how this caller is told which one: a caller with no way to say which rig it
/// meant must not attach to another lane's.
fn select_sole<'a>(
    probes: &'a [ProbeInfo],
    spec: Option<&str>,
    chooser: &str,
) -> Result<&'a ProbeInfo> {
    if let Some(spec) = spec {
        return select(probes, &spec.parse()?);
    }
    match probes {
        [] => Err(anyhow!("no CMSIS-DAP v2 probe found")),
        [single] => Ok(single),
        many => {
            let refs: Vec<&ProbeInfo> = many.iter().collect();
            Err(anyhow!(
                "{} CMSIS-DAP probes are attached; {chooser} to choose one:\n{}",
                many.len(),
                list_candidates(&refs)
            ))
        }
    }
}

/// [`select_sole`] for a command-line tool, which says which probe it meant with `--probe`.
pub fn select_cli<'a>(probes: &'a [ProbeInfo], spec: Option<&str>) -> Result<&'a ProbeInfo> {
    select_sole(probes, spec, "pass --probe VID:PID[:SERIAL]")
}

/// [`select_sole`] for a caller that can take no command-line selector.
pub fn select_only<'a>(probes: &'a [ProbeInfo], spec: Option<&str>) -> Result<&'a ProbeInfo> {
    select_sole(probes, spec, &format!("set {PROBE_ENV}=VID:PID[:SERIAL]"))
}

/// [`select_only`] against [`PROBE_ENV`], for a test that can take no command-line selector.
pub fn select_from_env(probes: &[ProbeInfo]) -> Result<&ProbeInfo> {
    let spec = std::env::var(PROBE_ENV).ok();
    let spec = spec.as_deref().map(str::trim).filter(|s| !s.is_empty());
    select_only(probes, spec)
}

/// How much of the request/response window the transport narrates to stderr.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum TraceLevel {
    /// No trace output; the transport behaves exactly as it always has.
    #[default]
    Off,
    /// Claim/flush/cancel boundaries, plus a dump of the last transfers when a batch fails.
    Boundary,
    /// Every request and response, with its batch, position, command id and length.
    Full,
}

impl TraceLevel {
    /// Parse [`TRACE_ENV`]; anything unrecognized but non-empty still yields boundary evidence.
    pub fn from_env_value(value: Option<&str>) -> TraceLevel {
        match value.map(str::trim) {
            None | Some("") | Some("0") | Some("off") | Some("no") | Some("false") => {
                TraceLevel::Off
            }
            Some("2") | Some("full") | Some("all") => TraceLevel::Full,
            Some(_) => TraceLevel::Boundary,
        }
    }

    /// Read [`TRACE_ENV`] from the process environment.
    pub fn from_env() -> TraceLevel {
        TraceLevel::from_env_value(std::env::var(TRACE_ENV).ok().as_deref())
    }
}

/// Which submission path produced a [`XferRecord`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum XferKind {
    /// One [`Probe::transfer`] / [`Probe::transfer_expect`] round trip.
    Serial,
    /// One position of a [`Probe::submit_pipelined`] batch.
    Pipelined,
    /// A packet discarded by a boundary flush: a response this handle never asked for.
    Stale,
}

impl fmt::Display for XferKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            XferKind::Serial => f.write_str("serial"),
            XferKind::Pipelined => f.write_str("pipe"),
            XferKind::Stale => f.write_str("stale"),
        }
    }
}

/// One request/response pair as the transport saw it, kept so a failure can show its window.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XferRecord {
    /// Which submission path produced it.
    pub kind: XferKind,
    /// Pipelined batch number, monotonic over the handle's life (`0` for serial and stale).
    pub batch: u64,
    /// Position within the batch (`0` for serial and stale).
    pub pos: usize,
    /// Command id byte of the request, absent for a stale packet nobody asked for.
    pub req_id: Option<u8>,
    /// Request length in bytes.
    pub req_len: usize,
    /// Command id byte echoed by the response, absent when the response was empty.
    pub resp_id: Option<u8>,
    /// Response length in bytes.
    pub resp_len: usize,
    /// The response's leading bytes, for reading a desync off the trace directly.
    pub resp_head: Vec<u8>,
}

impl fmt::Display for XferRecord {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let id = |v: Option<u8>| match v {
            Some(v) => format!("0x{v:02X}"),
            None => "----".to_string(),
        };
        let head: Vec<String> = self.resp_head.iter().map(|b| format!("{b:02X}")).collect();
        write!(
            f,
            "{:<6} batch {:>6} pos {:>3}  req {} ({:>4}B)  resp {} ({:>4}B)  [{}]",
            self.kind.to_string(),
            self.batch,
            self.pos,
            id(self.req_id),
            self.req_len,
            id(self.resp_id),
            self.resp_len,
            head.join(" ")
        )
    }
}

/// A response whose echoed command id does not belong to the request at its position.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Misalignment {
    /// Position in the batch where the mismatch was seen.
    pub pos: usize,
    /// Command id of the request submitted at that position.
    pub expected: u8,
    /// Command id the response actually carried.
    pub found: u8,
    /// Length of the offending response.
    pub resp_len: usize,
    /// The response was the engine's one-byte invalid-command reply.
    pub invalid_command: bool,
    /// Signed offset to the nearest position whose request id matches `found`; `None` when no
    /// request in the batch carries it.
    pub shift: Option<isize>,
}

impl fmt::Display for Misalignment {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "response at batch position {} echoes command id 0x{:02X}, but position {} sent \
             0x{:02X} ({} bytes)",
            self.pos, self.found, self.pos, self.expected, self.resp_len
        )?;
        if self.invalid_command {
            write!(f, "; 0xFF is the probe's invalid-command reply")?;
        }
        match self.shift {
            Some(n) => write!(
                f,
                "; it matches the request {n} position(s) away (FIFO shift)"
            ),
            None => write!(f, "; no request in this batch carries that id"),
        }
    }
}

/// The command-id byte a CMSIS-DAP request or response buffer leads with.
pub fn command_id(buf: &[u8]) -> Option<u8> {
    buf.first().copied()
}

/// Check the response at `pos` against the request submitted there. Every CMSIS-DAP response
/// echoes its request's command id in byte 0, bar the one-byte `0xFF` invalid-command reply.
pub fn check_pair(requests: &[Vec<u8>], pos: usize, response: &[u8]) -> Option<Misalignment> {
    let expected = command_id(requests.get(pos)?)?;
    let found = command_id(response)?;
    if found == expected {
        return None;
    }
    Some(Misalignment {
        pos,
        expected,
        found,
        resp_len: response.len(),
        invalid_command: found == INVALID_COMMAND,
        shift: shift_hint(requests, pos, found),
    })
}

/// Signed distance from `pos` to the nearest request carrying `found` as its command id.
pub fn shift_hint(requests: &[Vec<u8>], pos: usize, found: u8) -> Option<isize> {
    (1..=requests.len() as isize)
        .flat_map(|d| [-d, d])
        .find(|&d| {
            usize::try_from(pos as isize + d)
                .ok()
                .and_then(|i| requests.get(i))
                .and_then(|r| command_id(r))
                == Some(found)
        })
}

/// An open CMSIS-DAP v2 bulk probe, ready for strict-serial command submission.
pub struct Probe {
    info: ProbeInfo,
    // An `Interface` is released only once its clones and endpoints are all dropped.
    _interface: Interface,
    out: Endpoint<Bulk, Out>,
    in_: Endpoint<Bulk, In>,
    trace: TraceLevel,
    batch: u64,
    history: VecDeque<XferRecord>,
    /// A response may be stranded on the probe; the next command flushes before it is sent.
    desynced: bool,
    /// Packets the claim-time flush discarded; non-zero means a previous handle left the wire
    /// dirty.
    stale_on_claim: usize,
}

impl Probe {
    /// Open the device behind `info`, claiming its CMSIS-DAP v2 interface for exclusive use.
    pub fn open(info: ProbeInfo) -> Result<Probe> {
        let device = info
            .device_info
            .open()
            .wait()
            .context("failed to open probe device")?;
        let interface = device
            .detach_and_claim_interface(info.interface_number)
            .wait()
            .with_context(|| format!("failed to claim interface {}", info.interface_number))?;

        let out = interface
            .endpoint::<Bulk, Out>(info.out_endpoint)
            .context("failed to open bulk OUT endpoint")?;
        let in_ = interface
            .endpoint::<Bulk, In>(info.in_endpoint)
            .context("failed to open bulk IN endpoint")?;

        let mut probe = Probe {
            info,
            _interface: interface,
            out,
            in_,
            trace: TraceLevel::from_env(),
            batch: 0,
            history: VecDeque::with_capacity(HISTORY_LEN),
            desynced: false,
            stale_on_claim: 0,
        };

        // A fresh claim is not a fresh wire: a leftover response would shift every match.
        if !env_flag(NO_CLAIM_FLUSH_ENV) {
            probe.stale_on_claim = probe.flush_boundary();
        }
        if probe.trace >= TraceLevel::Boundary {
            eprintln!(
                "trace: claim iface {} ep_out 0x{:02X} ep_in 0x{:02X} mps {} stale_on_claim {}",
                probe.info.interface_number,
                probe.info.out_endpoint,
                probe.info.in_endpoint,
                probe.info.max_packet_size,
                probe.stale_on_claim,
            );
        }
        Ok(probe)
    }

    /// The identity this probe was opened against.
    pub fn info(&self) -> &ProbeInfo {
        &self.info
    }

    /// The trace level this handle read from [`TRACE_ENV`] when it was opened.
    pub fn trace_level(&self) -> TraceLevel {
        self.trace
    }

    /// Packets the claim-time boundary flush discarded.
    pub fn stale_on_claim(&self) -> usize {
        self.stale_on_claim
    }

    /// Whether a response may still be stranded on the probe from a failed exchange.
    pub fn is_desynced(&self) -> bool {
        self.desynced
    }

    /// Record a response that did not belong to its request; decode-level mismatches are
    /// invisible to the transport, and the next command must flush the stranded response.
    pub fn mark_desynced(&mut self) {
        self.desynced = true;
    }

    /// Drain any stranded response and clear the flag, returning the packets discarded.
    pub fn resynchronize(&mut self) -> Result<usize> {
        self.ensure_idle()?;
        let discarded = self.flush_boundary();
        self.desynced = false;
        if self.trace >= TraceLevel::Boundary {
            eprintln!("trace: resynchronize discarded {discarded} packet(s)");
        }
        Ok(discarded)
    }

    /// The transfer records the handle still remembers, oldest first.
    pub fn history(&self) -> impl Iterator<Item = &XferRecord> {
        self.history.iter()
    }

    /// Discard packets until the IN endpoint goes quiet; a cancelled read is the success case,
    /// and anything that does arrive answered a command this handle never sent.
    fn flush_boundary(&mut self) -> usize {
        let mps = self.info.max_packet_size.max(1);
        let mut discarded = 0;
        for _ in 0..BOUNDARY_FLUSH_MAX {
            let completion = self
                .in_
                .transfer_blocking(Buffer::new(mps), BOUNDARY_FLUSH_TIMEOUT);
            if completion.status.is_err() {
                break;
            }
            let stale = completion.buffer.into_vec();
            discarded += 1;
            let record = XferRecord {
                kind: XferKind::Stale,
                batch: self.batch,
                pos: 0,
                req_id: None,
                req_len: 0,
                resp_id: command_id(&stale),
                resp_len: stale.len(),
                resp_head: head(&stale),
            };
            if self.trace >= TraceLevel::Boundary {
                eprintln!("trace: {record}");
            }
            self.push_history(record);
        }
        discarded
    }

    /// Bring both endpoints idle: `nusb`'s `transfer_blocking` panics if anything is pending, so
    /// an incomplete drain must become an error here.
    fn ensure_idle(&mut self) -> Result<()> {
        if self.out.pending() == 0 && self.in_.pending() == 0 {
            return Ok(());
        }
        if !self.cancel_and_drain() {
            bail!(
                "probe endpoints did not go idle: {} OUT and {} IN transfer(s) still pending \
                 after cancellation",
                self.out.pending(),
                self.in_.pending()
            );
        }
        Ok(())
    }

    /// The prologue every submission shares: idle endpoints, nothing stale on the wire.
    fn prepare(&mut self) -> Result<()> {
        self.ensure_idle()?;
        if self.desynced {
            let discarded = self.flush_boundary();
            self.desynced = false;
            if self.trace >= TraceLevel::Boundary {
                eprintln!(
                    "trace: pre-command flush after a failed exchange discarded {discarded} \
                     packet(s)"
                );
            }
        }
        Ok(())
    }

    /// Remember one transfer for the on-error window dump.
    fn push_history(&mut self, record: XferRecord) {
        if self.history.len() == HISTORY_LEN {
            self.history.pop_front();
        }
        self.history.push_back(record);
    }

    /// Narrate one transfer and remember it.
    fn record(&mut self, record: XferRecord) {
        if self.trace >= TraceLevel::Full {
            eprintln!("trace: {record}");
        }
        self.push_history(record);
    }

    /// Replay the tail of the transfer history, so a failure names the window it happened in.
    fn dump_window(&self, reason: &str) {
        if self.trace < TraceLevel::Boundary {
            return;
        }
        eprintln!("trace: --- transfer window before {reason} ---");
        let skip = self.history.len().saturating_sub(DUMP_LEN);
        for record in self.history.iter().skip(skip) {
            eprintln!("trace: {record}");
        }
        eprintln!("trace: --- end window ---");
    }

    /// Submit one command and read back one response. The IN read length must be a multiple of
    /// the max packet size, and completes on the short packet that ends the response.
    pub fn transfer(&mut self, out: &[u8], timeout: Duration) -> Result<Vec<u8>> {
        let read_len = round_up(RESPONSE_BUFFER_HINT, self.info.max_packet_size.max(1));
        self.serial(out, read_len, timeout)
    }

    /// Submit one command and read back a known-length response, so a full-capacity response --
    /// an exact packet multiple -- does not stall waiting for a short packet.
    pub fn transfer_expect(
        &mut self,
        out: &[u8],
        expected_len: usize,
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        if out.is_empty() {
            bail!("command buffer must not be empty");
        }
        if expected_len == 0 {
            bail!("expected response length must not be zero");
        }
        let read_len = round_up(expected_len, self.info.max_packet_size.max(1));
        self.serial(out, read_len, timeout)
    }

    /// One strict-serial exchange: OUT the command, IN exactly `read_len` bytes. A failure after
    /// delivery strands the response, so the handle is marked desynchronized.
    fn serial(&mut self, out: &[u8], read_len: usize, timeout: Duration) -> Result<Vec<u8>> {
        if out.is_empty() {
            bail!("command buffer must not be empty");
        }
        self.prepare()?;

        let out_completion = self.out.transfer_blocking(Buffer::from(out), timeout);
        if let Err(e) = out_completion.status {
            // The command may have reached the probe: assume it did.
            self.desynced = true;
            return Err(anyhow!(e)).context("bulk OUT transfer failed");
        }

        let in_completion = self.in_.transfer_blocking(Buffer::new(read_len), timeout);
        let response = match in_completion.into_result() {
            Ok(buffer) => buffer.into_vec(),
            Err(e) => {
                self.desynced = true;
                self.dump_window("a failed serial bulk IN transfer");
                return Err(anyhow!(e)).context("bulk IN transfer failed");
            }
        };

        self.record(XferRecord {
            kind: XferKind::Serial,
            batch: self.batch,
            pos: 0,
            req_id: command_id(out),
            req_len: out.len(),
            resp_id: command_id(&response),
            resp_len: response.len(),
            resp_head: head(&response),
        });
        Ok(response)
    }

    /// Submit `requests` with up to `depth` pairs outstanding; CMSIS-DAP answers queued commands
    /// FIFO, so position is the only match. `depth` must not exceed `DAP_PACKET_COUNT`.
    pub fn submit_pipelined(
        &mut self,
        requests: &[Vec<u8>],
        expected_lens: &[usize],
        depth: usize,
        timeout: Duration,
    ) -> Result<Vec<Vec<u8>>> {
        if requests.len() != expected_lens.len() {
            bail!(
                "requests ({}) and expected_lens ({}) must have the same length",
                requests.len(),
                expected_lens.len()
            );
        }
        if depth == 0 {
            bail!("pipeline depth must be at least 1");
        }
        if requests.iter().any(Vec::is_empty) {
            bail!("command buffers must not be empty");
        }
        if expected_lens.contains(&0) {
            bail!("expected response lengths must not be zero");
        }

        self.prepare()?;
        self.batch += 1;
        if self.trace >= TraceLevel::Full {
            let ids: Vec<String> = requests
                .iter()
                .zip(expected_lens)
                .map(|(r, len)| {
                    format!(
                        "{}:{}->{len}",
                        command_id(r).map_or("--".to_string(), |v| format!("{v:02X}")),
                        r.len()
                    )
                })
                .collect();
            eprintln!(
                "trace: batch {} depth {depth} {} request(s) [{}]",
                self.batch,
                requests.len(),
                ids.join(" ")
            );
        }

        let result = self.pipelined_inner(requests, expected_lens, depth, timeout);
        if result.is_err() {
            // Cancelling an IN does not un-send the response its OUT already earned.
            self.desynced = true;
            self.cancel_and_drain();
            self.dump_window("a failed pipelined batch");
        }
        result
    }

    /// [`Probe::submit_pipelined`]'s submit/reap loop, split out so its caller owns the cleanup.
    fn pipelined_inner(
        &mut self,
        requests: &[Vec<u8>],
        expected_lens: &[usize],
        depth: usize,
        timeout: Duration,
    ) -> Result<Vec<Vec<u8>>> {
        let mps = self.info.max_packet_size.max(1);
        let total = requests.len();
        let mut responses = Vec::with_capacity(total);
        let mut submitted = 0;

        let submit_pair =
            |out: &mut Endpoint<Bulk, Out>, in_: &mut Endpoint<Bulk, In>, i: usize| {
                // IN first, so a fast response always finds a receive buffer pending.
                in_.submit(Buffer::new(round_up(expected_lens[i], mps)));
                out.submit(Buffer::from(requests[i].as_slice()));
            };

        while submitted < total && submitted < depth {
            submit_pair(&mut self.out, &mut self.in_, submitted);
            submitted += 1;
        }

        while responses.len() < total {
            // OUT[i] always completes before response i, so this never waits on the wrong end.
            let out_completion = self
                .out
                .wait_next_complete(timeout)
                .ok_or_else(|| anyhow!("bulk OUT transfer timed out"))?;
            out_completion
                .status
                .context("pipelined bulk OUT transfer failed")?;

            let in_completion = self
                .in_
                .wait_next_complete(timeout)
                .ok_or_else(|| anyhow!("bulk IN transfer timed out"))?;
            in_completion
                .status
                .context("pipelined bulk IN transfer failed")?;

            let pos = responses.len();
            let response = in_completion.buffer.into_vec();
            self.record(XferRecord {
                kind: XferKind::Pipelined,
                batch: self.batch,
                pos,
                req_id: command_id(&requests[pos]),
                req_len: requests[pos].len(),
                resp_id: command_id(&response),
                resp_len: response.len(),
                resp_head: head(&response),
            });
            // The echoed command id is the cheapest proof the streams are still paired.
            if let Some(m) = check_pair(requests, pos, &response) {
                bail!("pipelined response/request mismatch: {m}");
            }
            responses.push(response);

            if submitted < total {
                submit_pair(&mut self.out, &mut self.in_, submitted);
                submitted += 1;
            }
        }

        Ok(responses)
    }

    /// Cancel and reap every outstanding transfer, reporting whether both endpoints reached
    /// idle; a transfer left pending would turn the next serial command into a `nusb` panic.
    fn cancel_and_drain(&mut self) -> bool {
        const DRAIN_TIMEOUT: Duration = Duration::from_secs(1);
        /// Cancellation rounds before the endpoint is declared stuck.
        const DRAIN_ROUNDS: usize = 3;

        let (out_before, in_before) = (self.out.pending(), self.in_.pending());
        for _ in 0..DRAIN_ROUNDS {
            self.out.cancel_all();
            self.in_.cancel_all();
            while self.out.pending() > 0 && self.out.wait_next_complete(DRAIN_TIMEOUT).is_some() {}
            while self.in_.pending() > 0 && self.in_.wait_next_complete(DRAIN_TIMEOUT).is_some() {}
            if self.out.pending() == 0 && self.in_.pending() == 0 {
                break;
            }
        }
        let idle = self.out.pending() == 0 && self.in_.pending() == 0;
        if self.trace >= TraceLevel::Boundary {
            eprintln!(
                "trace: cancel_and_drain cancelled {out_before} OUT / {in_before} IN; idle {idle}"
            );
        }
        idle
    }
}

/// Round `len` up to the next multiple of `granularity`.
fn round_up(len: usize, granularity: usize) -> usize {
    if granularity == 0 {
        return len;
    }
    len.div_ceil(granularity) * granularity
}

/// Whether an environment variable is set to something other than an explicit "no".
fn env_flag(name: &str) -> bool {
    matches!(
        std::env::var(name).ok().as_deref().map(str::trim),
        Some(v) if !matches!(v, "" | "0" | "off" | "no" | "false")
    )
}

/// The leading bytes of a buffer, as a trace line shows them.
fn head(buf: &[u8]) -> Vec<u8> {
    buf[..buf.len().min(TRACE_HEAD_BYTES)].to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reqs(ids: &[&[u8]]) -> Vec<Vec<u8>> {
        ids.iter().map(|r| r.to_vec()).collect()
    }

    #[test]
    fn trace_level_reads_off_unless_the_variable_asks_for_more() {
        for off in ["", "0", "off", "no", "false", "  "] {
            assert_eq!(TraceLevel::from_env_value(Some(off)), TraceLevel::Off);
        }
        assert_eq!(TraceLevel::from_env_value(None), TraceLevel::Off);
        for full in ["2", "full", "all"] {
            assert_eq!(TraceLevel::from_env_value(Some(full)), TraceLevel::Full);
        }
        // Anything else still yields boundary evidence rather than silently nothing.
        for boundary in ["1", "on", "boundary", "yes", "please"] {
            assert_eq!(
                TraceLevel::from_env_value(Some(boundary)),
                TraceLevel::Boundary
            );
        }
        assert!(TraceLevel::Full > TraceLevel::Boundary);
        assert!(TraceLevel::Boundary > TraceLevel::Off);
    }

    #[test]
    fn an_aligned_batch_reports_no_mismatch() {
        let requests = reqs(&[&[0x05, 0, 1], &[0x06, 0, 127, 0, 2], &[0x1C, 0xFC, 0x01]]);
        assert_eq!(check_pair(&requests, 0, &[0x05, 1, 0x01]), None);
        assert_eq!(check_pair(&requests, 2, &[0x1C, 0x01, 0, 0]), None);
    }

    #[test]
    fn a_fifo_that_slipped_one_response_behind_is_named_as_such() {
        let requests = reqs(&[&[0x05, 0, 1], &[0x06, 0, 127, 0, 2], &[0x1C, 0xFC, 0x01]]);
        // Position 2 asked for SWO data and got the block read's answer.
        let m = check_pair(&requests, 2, &[0x06, 127, 0, 0x01]).expect("mismatch");
        assert_eq!(m.pos, 2);
        assert_eq!(m.expected, 0x1C);
        assert_eq!(m.found, 0x06);
        assert_eq!(m.shift, Some(-1));
        assert!(!m.invalid_command);
        assert!(m.to_string().contains("0x1C"));
    }

    #[test]
    fn the_invalid_command_reply_is_reported_as_itself() {
        let requests = reqs(&[&[0x1C, 0xFC, 0x01]]);
        let m = check_pair(&requests, 0, &[INVALID_COMMAND]).expect("mismatch");
        assert!(m.invalid_command);
        assert_eq!(m.shift, None, "no request in the batch carries 0xFF");
        assert_eq!(m.resp_len, 1);
        let text = m.to_string();
        assert!(text.contains("invalid-command reply"), "{text}");
        assert!(text.contains("no request in this batch"), "{text}");
    }

    #[test]
    fn shift_hint_prefers_the_nearest_match_and_reports_its_direction() {
        let requests = reqs(&[&[0x1C], &[0x05], &[0x1C], &[0x06]]);
        assert_eq!(shift_hint(&requests, 1, 0x1C), Some(-1));
        assert_eq!(shift_hint(&requests, 0, 0x06), Some(3));
        assert_eq!(shift_hint(&requests, 3, 0x05), Some(-2));
        assert_eq!(shift_hint(&requests, 0, 0xAB), None);
    }

    #[test]
    fn an_empty_response_or_a_missing_request_yields_no_verdict() {
        let requests = reqs(&[&[0x1C, 0, 0]]);
        assert_eq!(check_pair(&requests, 0, &[]), None);
        assert_eq!(check_pair(&requests, 9, &[0x1C]), None);
        assert_eq!(command_id(&[]), None);
        assert_eq!(command_id(&[0x1C, 0xFC]), Some(0x1C));
    }

    #[test]
    fn a_record_prints_its_position_ids_and_lengths() {
        let record = XferRecord {
            kind: XferKind::Pipelined,
            batch: 12,
            pos: 7,
            req_id: Some(0x1C),
            req_len: 3,
            resp_id: Some(0xFF),
            resp_len: 1,
            resp_head: vec![0xFF],
        };
        let text = record.to_string();
        for needle in ["pipe", "12", "7", "0x1C", "0xFF", "FF"] {
            assert!(text.contains(needle), "{text} is missing {needle}");
        }
        let stale = XferRecord {
            kind: XferKind::Stale,
            req_id: None,
            resp_id: None,
            resp_head: Vec::new(),
            ..record
        };
        assert!(stale.to_string().contains("----"));
    }

    #[test]
    fn head_shows_at_most_the_trace_prefix() {
        assert_eq!(head(&[1, 2, 3]), vec![1, 2, 3]);
        assert_eq!(head(&[7u8; 40]).len(), TRACE_HEAD_BYTES);
        assert!(head(&[]).is_empty());
    }

    #[test]
    fn round_up_pads_to_whole_packets() {
        assert_eq!(round_up(3, 512), 512);
        assert_eq!(round_up(512, 512), 512);
        assert_eq!(round_up(513, 512), 1024);
        assert_eq!(round_up(0, 512), 0);
        assert_eq!(round_up(7, 0), 7);
    }

    #[test]
    fn select_only_reports_an_empty_bus_rather_than_choosing() {
        let err = select_only(&[], None).err().expect("no probes attached");
        assert!(err.to_string().contains("no CMSIS-DAP v2 probe found"));
    }

    #[test]
    fn select_only_rejects_a_malformed_spec_before_matching() {
        let err = select_only(&[], Some("0d28:zzzz"))
            .err()
            .expect("malformed");
        assert!(err.to_string().contains("invalid hex product id"));
    }

    #[test]
    fn select_cli_refuses_an_empty_bus_the_way_select_only_does() {
        let err = select_cli(&[], None).err().expect("no probes attached");
        assert!(err.to_string().contains("no CMSIS-DAP v2 probe found"));
        let err = select_cli(&[], Some("0d28:zzzz")).err().expect("malformed");
        assert!(err.to_string().contains("invalid hex product id"));
    }

    #[test]
    fn probe_selector_parses_vid_pid() {
        let sel: ProbeSelector = "0d28:0204".parse().unwrap();
        assert_eq!(sel.vendor_id, 0x0d28);
        assert_eq!(sel.product_id, 0x0204);
        assert_eq!(sel.serial, None);
    }

    #[test]
    fn probe_selector_parses_vid_pid_serial() {
        let sel: ProbeSelector = "0d28:0204:SERIAL123".parse().unwrap();
        assert_eq!(sel.vendor_id, 0x0d28);
        assert_eq!(sel.product_id, 0x0204);
        assert_eq!(sel.serial.as_deref(), Some("SERIAL123"));
    }

    #[test]
    fn probe_selector_strips_probe_rs_interface_index_with_serial() {
        let sel: ProbeSelector = "0d28:0204-0:SERIAL123".parse().unwrap();
        assert_eq!(sel.vendor_id, 0x0d28);
        assert_eq!(sel.product_id, 0x0204);
        assert_eq!(sel.serial.as_deref(), Some("SERIAL123"));
    }

    #[test]
    fn probe_selector_strips_probe_rs_interface_index_without_serial() {
        let sel: ProbeSelector = "0d28:0204-1".parse().unwrap();
        assert_eq!(sel.vendor_id, 0x0d28);
        assert_eq!(sel.product_id, 0x0204);
        assert_eq!(sel.serial, None);
    }

    #[test]
    fn probe_selector_rejects_a_malformed_product_id() {
        let result = "0d28:zzzz".parse::<ProbeSelector>();
        let err = result.err().expect("malformed product id should not parse");
        assert!(err.to_string().contains("invalid hex product id"));
    }
}
