//! The halt case: endpoint halt, halt clear and data toggle on every bulk endpoint, then a USB reset.

use std::time::{Duration, Instant};

use nusb::descriptors::TransferType;
use nusb::transfer::{
    Buffer, Bulk, ControlIn, ControlOut, ControlType, Direction, EndpointDirection, In, Out,
    Recipient, TransferError,
};
use nusb::{Device, Endpoint, Interface, MaybeFuture};

use crate::{msc_wire, Ctx, Failure};

/// Every request here is a register poke on the device; the embassy-stm32 0.6 clear never ends.
const DEADLINE: Duration = Duration::from_secs(1);
/// How long a read waits for bytes that should not be there.
const DRAIN: Duration = Duration::from_millis(50);

const CLASS_VENDOR: u8 = 0xFF;
const CLASS_CDC_COMM: u8 = 0x02;
const CLASS_CDC_DATA: u8 = 0x0A;
const GET_STATUS: u8 = 0;
const CLEAR_FEATURE: u8 = 1;
const SET_FEATURE: u8 = 3;
const ENDPOINT_HALT: u16 = 0;
const BULK_ONLY_RESET: u8 = 0xFF;
const DAP_INFO: u8 = 0x00;
const INFO_PACKET_COUNT: u8 = 0xFE;
const INFO_PACKET_SIZE: u8 = 0xFF;
const INQUIRY: [u8; 6] = [0x12, 0, 0, 0, 36, 0];
const TEST_UNIT_READY: [u8; 6] = [0; 6];

#[derive(Clone, Copy, PartialEq, Debug)]
enum Function {
    Dap,
    Cdc,
    Msc,
}

/// A step's failure; `Wedged` means the device stopped answering control requests.
enum Fault {
    Failed(String),
    Wedged(String),
}

type Step = Result<(), Fault>;

/// A sub-case run against one endpoint of a pair.
type SubCase = fn(&Device, &mut Pair, bool) -> Step;

fn failed<T>(e: impl Into<String>) -> Result<T, Fault> {
    Err(Fault::Failed(e.into()))
}

/// A refused control request is a finding; one that never completes means EP0 is gone.
fn control_fault(what: &str, e: TransferError) -> Fault {
    match e {
        TransferError::Stall => Fault::Failed(format!("{what} was refused")),
        e => Fault::Wedged(format!("{what}: {e}")),
    }
}

/// One function's claimed bulk pair.
struct Pair {
    function: Function,
    ifnum: u8,
    _interface: Interface,
    out: Endpoint<Bulk, Out>,
    inp: Endpoint<Bulk, In>,
    tag: u32,
}

impl Pair {
    fn write(&mut self, data: &[u8]) -> Result<(), TransferError> {
        self.out
            .transfer_blocking(Buffer::from(data.to_vec()), DEADLINE)
            .into_result()
            .map(drop)
    }

    fn read(&mut self, len: usize, timeout: Duration) -> Result<Vec<u8>, TransferError> {
        let len = len.next_multiple_of(self.inp.max_packet_size());
        self.inp
            .transfer_blocking(Buffer::new(len), timeout)
            .into_result()
            .map(|b| b.into_vec())
    }

    /// Read until the IN endpoint goes quiet, so a stale reply cannot answer the next request.
    fn drain(&mut self) {
        while self.read(1, DRAIN).is_ok() {}
    }

    fn info(&mut self, id: u8) -> Result<Vec<u8>, String> {
        self.write(&[DAP_INFO, id])
            .map_err(|e| format!("DAP_Info write: {e}"))?;
        self.read(1, DEADLINE)
            .map_err(|e| format!("DAP_Info reply: {e}"))
    }

    /// One BOT command with an optional data-IN phase, returning its CSW status.
    fn scsi(&mut self, cb: &[u8], data_in: u32) -> Result<u8, String> {
        self.tag = self.tag.wrapping_add(1);
        let cbw = msc_wire::build_cbw(self.tag, data_in, data_in > 0, cb);
        self.write(&cbw).map_err(|e| format!("CBW write: {e}"))?;
        if data_in > 0 {
            self.read(data_in as usize, DEADLINE)
                .map_err(|e| format!("data-in: {e}"))?;
        }
        let csw = self
            .read(msc_wire::CSW_LEN, DEADLINE)
            .map_err(|e| format!("CSW read: {e}"))?;
        msc_wire::parse_csw(&csw, self.tag).map(|(status, _)| status)
    }

    /// Traffic over the pair: an even packet count each way, so `extra` flips the next clear's parity.
    fn traffic(&mut self) -> Step {
        match self.function {
            Function::Dap => {
                let w = |e: TransferError| Fault::Failed(format!("DAP_Info write: {e}"));
                self.write(&[DAP_INFO, INFO_PACKET_SIZE]).map_err(w)?;
                self.write(&[DAP_INFO, INFO_PACKET_COUNT]).map_err(w)?;
                let first = self.read(1, DEADLINE);
                let second = self.read(1, DEADLINE);
                match (first, second) {
                    (Ok(a), Ok(b)) => info_pair_in_order(&a, &b).or_else(failed),
                    (a, b) => failed(format!("DAP_Info replies: {:?}, {:?}", a.err(), b.err())),
                }
            }
            // OUT only: the IN side carries only what the target sends. One byte a packet, since
            // the toggle counts packets and each byte reaches the target and may echo back.
            Function::Cdc => {
                for _ in 0..2 {
                    self.write(&[0x55])
                        .or_else(|e| failed(format!("CDC write: {e}")))?;
                }
                Ok(())
            }
            Function::Msc => {
                for _ in 0..2 {
                    match self.scsi(&INQUIRY, 36) {
                        Ok(msc_wire::CSW_PASSED) => {}
                        Ok(status) => return failed(format!("INQUIRY status {status}")),
                        Err(e) => return failed(format!("INQUIRY: {e}")),
                    }
                }
                Ok(())
            }
        }
    }

    /// One packet each way the function uses.
    fn extra(&mut self) -> Step {
        let result = match self.function {
            Function::Dap => self.info(INFO_PACKET_COUNT).map(drop),
            Function::Cdc => self.write(&[0x55]).map_err(|e| e.to_string()),
            Function::Msc => self.scsi(&TEST_UNIT_READY, 0).map(drop),
        };
        result.or_else(|e| failed(format!("parity exchange: {e}")))
    }

    fn address(&self, dir_in: bool) -> u8 {
        if dir_in {
            self.inp.endpoint_address()
        } else {
            self.out.endpoint_address()
        }
    }

    fn clear(&mut self, dir_in: bool) -> Step {
        if dir_in {
            clear(&mut self.inp)
        } else {
            clear(&mut self.out)
        }
    }

    /// The one transfer a halted endpoint must refuse; a harmless request if it gets through.
    fn probe_halted(&mut self, dir_in: bool) -> Step {
        let outcome = match (dir_in, self.function) {
            (true, _) => self.read(1, DEADLINE).map(drop),
            (false, Function::Dap) => self.write(&[DAP_INFO, INFO_PACKET_COUNT]),
            (false, Function::Cdc) => self.write(&[0x55]),
            (false, Function::Msc) => {
                self.write(&msc_wire::build_cbw(self.tag, 0, false, &TEST_UNIT_READY))
            }
        };
        match outcome {
            Err(TransferError::Stall) => Ok(()),
            Ok(()) => failed("a transfer went through the halt"),
            Err(e) => failed(format!("the halted endpoint answered {e}, not a STALL")),
        }
    }
}

/// The two `DAP_Info` replies `traffic` asks for, once each and in order.
fn info_pair_in_order(first: &[u8], second: &[u8]) -> Result<(), String> {
    let size = first.len() >= 4 && first[..2] == [DAP_INFO, 2];
    let count = second.len() >= 3 && second[..2] == [DAP_INFO, 1];
    if size && count {
        Ok(())
    } else {
        Err(format!(
            "DAP_Info replies {first:02x?}, {second:02x?}: a reply was lost or reordered"
        ))
    }
}

fn clear<D: EndpointDirection>(ep: &mut Endpoint<Bulk, D>) -> Step {
    let started = Instant::now();
    let result = ep.clear_halt().wait();
    let took = started.elapsed();
    match result {
        Ok(()) if took <= DEADLINE => Ok(()),
        Ok(()) => Err(Fault::Wedged(format!("the halt clear took {took:?}"))),
        Err(e) => Err(Fault::Wedged(format!(
            "the halt clear failed after {took:?}: {e}"
        ))),
    }
}

fn endpoint_request(device: &Device, request: u8, index: u8) -> Result<(), TransferError> {
    let setup = ControlOut {
        control_type: ControlType::Standard,
        recipient: Recipient::Endpoint,
        request,
        value: ENDPOINT_HALT,
        index: index.into(),
        data: &[],
    };
    device.control_out(setup, DEADLINE).wait()
}

fn status(device: &Device, recipient: Recipient, index: u8) -> Result<u16, Fault> {
    let setup = ControlIn {
        control_type: ControlType::Standard,
        recipient,
        request: GET_STATUS,
        value: 0,
        index: index.into(),
        length: 2,
    };
    match device.control_in(setup, DEADLINE).wait() {
        Ok(v) if v.len() == 2 => Ok(u16::from_le_bytes([v[0], v[1]])),
        Ok(v) => failed(format!("GET_STATUS answered {v:02x?}")),
        Err(e) => Err(control_fault(&format!("GET_STATUS({index:#04x})"), e)),
    }
}

fn expect_status(device: &Device, ep: u8, halted: bool) -> Step {
    match status(device, Recipient::Endpoint, ep)? {
        s if s & 1 == u16::from(halted) => Ok(()),
        s => failed(format!(
            "GET_STATUS reads {s:#06x}, halted should be {halted}"
        )),
    }
}

/// A: a clear on an endpoint that is not halted, then traffic.
fn clear_unhalted(device: &Device, pair: &mut Pair, dir_in: bool) -> Step {
    pair.clear(dir_in)?;
    expect_status(device, pair.address(dir_in), false)?;
    pair.traffic()
}

/// B: a host halt must hold until its clear, then traffic.
fn halt_and_clear(device: &Device, pair: &mut Pair, dir_in: bool) -> Step {
    let ep = pair.address(dir_in);
    endpoint_request(device, SET_FEATURE, ep).map_err(|e| control_fault("SET_FEATURE", e))?;
    let held = expect_status(device, ep, true).and_then(|()| pair.probe_halted(dir_in));
    pair.clear(dir_in)?;
    held?;
    expect_status(device, ep, false)?;
    pair.traffic()
}

/// B': a reply armed before a halt is delivered or dropped, never duplicated.
fn halt_with_a_reply_armed(device: &Device, pair: &mut Pair) -> Result<String, Fault> {
    pair.write(&[DAP_INFO, INFO_PACKET_COUNT])
        .or_else(|e| failed(format!("DAP_Info write: {e}")))?;
    std::thread::sleep(DRAIN);
    endpoint_request(device, SET_FEATURE, pair.address(true))
        .map_err(|e| control_fault("SET_FEATURE", e))?;
    let held = pair.probe_halted(true);
    pair.clear(true)?;
    held?;
    pair.write(&[DAP_INFO, INFO_PACKET_SIZE])
        .or_else(|e| failed(format!("DAP_Info write: {e}")))?;
    let mut replies = Vec::new();
    while let Ok(reply) = pair.read(1, DEADLINE) {
        replies.push(reply);
    }
    armed_reply_fate(&replies).or_else(failed)
}

/// A': a clear on an unhalted IN with a reply armed is delivered once or dropped, never duplicated.
fn clear_with_a_reply_armed(pair: &mut Pair) -> Result<String, Fault> {
    pair.write(&[DAP_INFO, INFO_PACKET_COUNT])
        .or_else(|e| failed(format!("DAP_Info write: {e}")))?;
    std::thread::sleep(DRAIN);
    pair.clear(true)?;
    pair.write(&[DAP_INFO, INFO_PACKET_SIZE])
        .or_else(|e| failed(format!("DAP_Info write: {e}")))?;
    let mut replies = Vec::new();
    while let Ok(reply) = pair.read(1, DEADLINE) {
        replies.push(reply);
    }
    armed_reply_fate(&replies).or_else(failed)
}

/// Run an armed-reply sub-case at both toggle parities and grade the pair: a clear zeroes the toggle.
fn at_both_parities(
    pair: &mut Pair,
    must_drop: bool,
    mut sub: impl FnMut(&mut Pair) -> Result<String, Fault>,
) -> Result<String, Fault> {
    pair.clear(true)?;
    let even = sub(pair)?;
    pair.drain();
    pair.clear(true)?;
    pair.extra()?;
    let odd = sub(pair)?;
    pair.drain();
    armed_verdict([even, odd], must_drop).or_else(failed)
}

/// The fates must agree, and a reply armed before a host halt must have been dropped.
fn armed_verdict(fates: [String; 2], must_drop: bool) -> Result<String, String> {
    let [even, odd] = fates;
    if even != odd {
        return Err(format!("at even parity {even}; at odd parity {odd}"));
    }
    if must_drop && !even.ends_with("dropped") {
        return Err(format!(
            "{even}: a reply armed before a host halt must be dropped"
        ));
    }
    Ok(even)
}

/// Whether the reply armed before the halt arrived, from what was read after the clear.
fn armed_reply_fate(replies: &[Vec<u8>]) -> Result<String, String> {
    let is = |r: &Vec<u8>, len: u8| r.len() >= 2 && r[..2] == [DAP_INFO, len];
    let armed = replies.iter().filter(|r| is(r, 1)).count();
    let fresh = replies.iter().filter(|r| is(r, 2)).count();
    match (armed, fresh, replies.len()) {
        (0, 1, 1) => Ok("the armed reply was dropped".into()),
        (1, 1, 2) if is(&replies[0], 1) => Ok("the armed reply was delivered".into()),
        _ => Err(format!("replies after the clear: {replies:02x?}")),
    }
}

/// C: an invalid CBW, then Reset Recovery must bring TEST UNIT READY back.
fn invalid_cbw(device: &Device, pair: &mut Pair) -> Result<String, Fault> {
    pair.write(&[0u8; msc_wire::CBW_LEN - 1])
        .or_else(|e| failed(format!("short CBW write: {e}")))?;
    let note = match pair.read(msc_wire::CSW_LEN, DEADLINE) {
        Err(TransferError::Stall) => {
            pair.clear(true)?;
            match pair.read(msc_wire::CSW_LEN, DEADLINE) {
                Err(TransferError::Stall) => "stalled, and held across a bare clear",
                _ => "stalled, but a bare clear released it",
            }
        }
        Err(TransferError::Cancelled) => "met with silence, not a stall",
        Ok(_) => "answered with a CSW",
        Err(e) => return failed(format!("after a short CBW: {e}")),
    };
    let reset = ControlOut {
        control_type: ControlType::Class,
        recipient: Recipient::Interface,
        request: BULK_ONLY_RESET,
        value: 0,
        index: pair.ifnum.into(),
        data: &[],
    };
    device
        .control_out(reset, DEADLINE)
        .wait()
        .map_err(|e| control_fault("Bulk-Only Mass Storage Reset", e))?;
    pair.clear(true)?;
    pair.clear(false)?;
    pair.scsi(&TEST_UNIT_READY, 0)
        .map(drop)
        .or_else(|e| failed(format!("TEST UNIT READY after Reset Recovery: {e}")))?;
    Ok(format!("an invalid CBW was {note}"))
}

/// D: CLEAR_FEATURE and GET_STATUS to the control pipe and to absent twins answer either way, and EP0 lives on.
fn foreign_clears(device: &Device, present: &[u8]) -> Vec<String> {
    let mut findings = Vec::new();
    for ep in [0x00, 0x80].into_iter().chain(absent_twins(present)) {
        match endpoint_request(device, CLEAR_FEATURE, ep) {
            Ok(()) | Err(TransferError::Stall) => {}
            Err(e) => findings.push(format!("CLEAR_FEATURE({ep:#04x}): {e}")),
        }
        if let Err(Fault::Wedged(e)) = status(device, Recipient::Endpoint, ep) {
            findings.push(format!("GET_STATUS({ep:#04x}): {e}"));
        }
        if let Err(Fault::Wedged(e) | Fault::Failed(e)) = status(device, Recipient::Device, 0) {
            findings.push(format!("after CLEAR_FEATURE({ep:#04x}): {e}"));
            break;
        }
    }
    findings
}

/// Every present endpoint's missing twin: an absent address Linux forwards, in either direction.
fn absent_twins(present: &[u8]) -> Vec<u8> {
    let mut twins: Vec<u8> = present
        .iter()
        .map(|&ep| ep ^ 0x80)
        .filter(|twin| !present.contains(twin))
        .collect();
    twins.sort_unstable();
    twins.dedup();
    twins
}

/// What the case claimed: the bulk pairs, the CDC comm interfaces and their endpoints.
struct Claimed {
    pairs: Vec<Pair>,
    _comm: Vec<Interface>,
    comm_endpoints: Vec<u8>,
    /// Every endpoint in the configuration, for D.
    present: Vec<u8>,
}

/// Claim every interface the case drives, detaching the host's drivers.
fn claim(device: &Device) -> Result<Claimed, String> {
    let config = device.active_configuration().map_err(|e| e.to_string())?;
    let detach = |ifnum: u8| {
        device
            .detach_and_claim_interface(ifnum)
            .wait()
            .map_err(|e| format!("claiming interface {ifnum}: {e}"))
    };
    let mut claimed = Claimed {
        pairs: Vec::new(),
        _comm: Vec::new(),
        comm_endpoints: Vec::new(),
        present: Vec::new(),
    };
    for group in config.interfaces() {
        let alt = group.first_alt_setting();
        claimed
            .present
            .extend(alt.endpoints().map(|ep| ep.address()));
        if alt.class() == CLASS_CDC_COMM {
            claimed._comm.push(detach(alt.interface_number())?);
            claimed
                .comm_endpoints
                .extend(alt.endpoints().map(|ep| ep.address()));
            continue;
        }
        let function = match (alt.class(), alt.subclass(), alt.protocol()) {
            (CLASS_VENDOR, _, _) => Function::Dap,
            (CLASS_CDC_DATA, _, _) => Function::Cdc,
            // macOS storage owns the interface, as it does for the msc case.
            (c, s, p) if msc_wire::is_msc_bot_class(c, s, p) && !cfg!(target_os = "macos") => {
                Function::Msc
            }
            _ => continue,
        };
        let bulk = |dir| {
            alt.endpoints()
                .find(|ep| ep.transfer_type() == TransferType::Bulk && ep.direction() == dir)
                .map(|ep| ep.address())
        };
        let (Some(out), Some(inp)) = (bulk(Direction::Out), bulk(Direction::In)) else {
            continue;
        };
        let ifnum = alt.interface_number();
        let interface = detach(ifnum)?;
        claimed.pairs.push(Pair {
            function,
            ifnum,
            out: interface.endpoint(out).map_err(|e| e.to_string())?,
            inp: interface.endpoint(inp).map_err(|e| e.to_string())?,
            _interface: interface,
            tag: 0x4841_0000,
        });
    }
    Ok(claimed)
}

/// Run every sub-case, stopping at the first sign the device no longer answers.
fn check(device: &Device) -> Result<Vec<String>, String> {
    let mut claimed = claim(device)?;
    let mut findings = Vec::new();
    let mut notes = Vec::new();
    for pair in &mut claimed.pairs {
        pair.drain();
        let name = format!("{:?}", pair.function);
        for dir_in in [false, true] {
            let ep = pair.address(dir_in);
            let subs: [(&str, SubCase); 2] = [("A", clear_unhalted), ("B", halt_and_clear)];
            for (sub, run) in subs {
                for shifted in [false, true] {
                    let step = if shifted { pair.extra() } else { Ok(()) };
                    match step.and_then(|()| run(device, pair, dir_in)) {
                        Ok(()) => {}
                        Err(Fault::Failed(e)) => {
                            findings
                                .push(format!("{name} {ep:#04x} {sub}{}: {e}", parity(shifted)));
                            pair.drain();
                        }
                        Err(Fault::Wedged(e)) => {
                            findings
                                .push(format!("{name} {ep:#04x} {sub}{}: {e}", parity(shifted)));
                            return Ok(findings);
                        }
                    }
                }
            }
        }
        let special: Vec<(&str, Result<String, Fault>)> = match pair.function {
            Function::Dap => vec![
                (
                    "B'",
                    at_both_parities(pair, true, |p| halt_with_a_reply_armed(device, p)),
                ),
                (
                    "A'",
                    at_both_parities(pair, false, clear_with_a_reply_armed),
                ),
            ],
            Function::Msc => vec![("C", invalid_cbw(device, pair))],
            // The target's echo of this traffic is not the next case's.
            Function::Cdc => {
                pair.drain();
                Vec::new()
            }
        };
        for (sub, outcome) in special {
            match outcome {
                Ok(note) => notes.push(format!("{name} {sub}: {note}")),
                Err(Fault::Failed(e)) => findings.push(format!("{name} {sub}: {e}")),
                Err(Fault::Wedged(e)) => {
                    findings.push(format!("{name} {sub}: {e}"));
                    return Ok(findings);
                }
            }
        }
    }
    for &ep in &claimed.comm_endpoints {
        match expect_status(device, ep, false) {
            Ok(()) => {}
            Err(Fault::Failed(e) | Fault::Wedged(e)) => {
                findings.push(format!("Cdc {ep:#04x}: {e}"))
            }
        }
    }
    findings.extend(foreign_clears(device, &claimed.present));
    for note in notes {
        println!("halt: {note}");
    }
    Ok(findings)
}

fn parity(shifted: bool) -> &'static str {
    if shifted {
        " shifted"
    } else {
        ""
    }
}

/// Drive every halt sub-case, then reset the device so a wedged stack cannot leak into later cases.
pub(crate) fn run(cx: &mut Ctx, full_run: bool) -> Result<(), Failure> {
    let device = crate::dap::open_device(&cx.selector).map_err(Failure::Fatal)?;
    let outcome = check(&device);
    drop(device);
    let recovered = crate::reattach::run(cx, full_run);
    let mut findings = outcome.map_err(Failure::Failed)?;
    if let Err(Failure::Failed(e) | Failure::Fatal(e)) = recovered {
        findings.push(format!("recovery: {e}"));
    }
    if findings.is_empty() {
        Ok(())
    } else {
        Err(Failure::Failed(findings.join("; ")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn info_replies_must_arrive_once_and_in_order() {
        let size = [DAP_INFO, 2, 0x40, 0x00];
        let count = [DAP_INFO, 1, 0x04];
        assert!(info_pair_in_order(&size, &count).is_ok());
        assert!(info_pair_in_order(&count, &size).is_err());
        assert!(info_pair_in_order(&count, &count).is_err());
    }

    #[test]
    fn an_armed_reply_is_delivered_or_dropped_but_never_doubled() {
        let armed = vec![DAP_INFO, 1, 0x04];
        let fresh = vec![DAP_INFO, 2, 0x40, 0x00];
        assert!(armed_reply_fate(std::slice::from_ref(&fresh))
            .unwrap()
            .contains("dropped"));
        let both = [armed.clone(), fresh.clone()];
        assert!(armed_reply_fate(&both).unwrap().contains("delivered"));
        assert!(armed_reply_fate(&[armed.clone(), armed.clone(), fresh.clone()]).is_err());
        assert!(armed_reply_fate(&[fresh, armed]).is_err());
        assert!(armed_reply_fate(&[]).is_err());
    }

    #[test]
    fn every_missing_twin_is_probed_in_both_directions() {
        assert_eq!(
            absent_twins(&[0x01, 0x82, 0x03, 0x83, 0x84]),
            [0x02, 0x04, 0x81]
        );
    }

    /// Both parities must agree; a halt drops what was armed, a clear may deliver it once.
    #[test]
    fn an_armed_reply_is_graded_across_parities() {
        let dropped = "the armed reply was dropped".to_string();
        let delivered = "the armed reply was delivered".to_string();
        assert!(armed_verdict([dropped.clone(), dropped.clone()], true).is_ok());
        assert!(armed_verdict([dropped.clone(), delivered.clone()], true).is_err());
        assert!(armed_verdict([delivered.clone(), delivered.clone()], true).is_err());
        assert!(armed_verdict([delivered.clone(), delivered.clone()], false).is_ok());
        assert!(armed_verdict([dropped.clone(), dropped], false).is_ok());
        assert!(armed_verdict([delivered.clone(), "x".into()], false).is_err());
    }
}
