//! Polled USB mass-storage class (Bulk-Only Transport) over the virtual volume.

use daplink_rs::daplink_core::msc_proto::{
    build_csw, clamp_reply_len, dispatch, is_test_unit_ready, parse_cbw, reconcile_cbw,
    should_clear_sense, BotVerdict, ScsiAction, Sense, CSW_FAILED, CSW_LEN, CSW_PASSED,
    SENSE_LUN_NOT_SUPPORTED, SENSE_MEDIUM_CHANGED, SENSE_NONE,
};
use daplink_rs::daplink_core::{
    MSC_BULK_ONLY_RESET, MSC_GET_MAX_LUN, MSC_INTERFACE_CLASS, MSC_SECTOR_SIZE,
};
use daplink_vfs::{DirectIo, SectorIo, SectorPoll, TransferSink, Vfs, VfsObserver};
use usb_device::class_prelude::*;
use usb_device::control::{Recipient, RequestType};

/// Upper bound on `step` iterations per pump, so an unexpected state cannot spin forever.
const PUMP_BOUND: usize = 1024;

/// The bulk endpoint pair as the machine sees it: two directions plus stall control.
trait BulkPort {
    /// Read one OUT packet; `Err` means none is pending and the caller retries next poll.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()>;
    /// Write one IN packet; `Err` means the endpoint is busy and the caller retries next poll.
    fn write(&mut self, data: &[u8]) -> Result<usize, ()>;
    /// Stall the device-to-host endpoint.
    fn stall_in(&mut self);
    /// Stall the host-to-device endpoint.
    fn stall_out(&mut self);
}

/// A [`BulkPort`] over the class's real endpoints.
struct EndpointPort<'e, 'a, B: UsbBus> {
    ep_in: &'e EndpointIn<'a, B>,
    ep_out: &'e EndpointOut<'a, B>,
}

impl<B: UsbBus> BulkPort for EndpointPort<'_, '_, B> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        self.ep_out.read(buf).map_err(|_| ())
    }
    fn write(&mut self, data: &[u8]) -> Result<usize, ()> {
        self.ep_in.write(data).map_err(|_| ())
    }
    fn stall_in(&mut self) {
        self.ep_in.stall();
    }
    fn stall_out(&mut self) {
        self.ep_out.stall();
    }
}

/// Where the transport currently is within a command.
#[derive(Clone, Copy)]
enum Phase {
    /// Waiting for the next Command Block Wrapper.
    WaitCbw,
    /// Sending a fixed reply payload (INQUIRY, READ CAPACITY, ...).
    ReplyIn {
        /// Reply bytes.
        buf: [u8; 36],
        /// Valid length, already clamped to the CBW data length.
        len: usize,
        /// Bytes already sent.
        sent: usize,
    },
    /// Streaming read sectors to the host.
    DataIn {
        /// Current logical block.
        lba: u32,
        /// Sectors still to send.
        remaining: u32,
        /// Byte offset within the current sector.
        offset: usize,
        /// Whether the current sector is loaded into the scratch buffer.
        loaded: bool,
    },
    /// Receiving write sectors from the host.
    DataOut {
        /// Current logical block.
        lba: u32,
        /// Sectors still to receive.
        remaining: u32,
        /// Byte offset within the current sector.
        offset: usize,
    },
    /// Sending the Command Status Wrapper.
    SendCsw {
        /// The CSW bytes.
        csw: [u8; CSW_LEN],
        /// Bytes already sent.
        sent: usize,
    },
}

/// The resumable CBW-to-CSW machine, held apart from the endpoints; `EP` is the packet size.
struct Machine<const EP: usize = 64> {
    phase: Phase,
    sense: Sense,
    tag: u32,
    data_len: u32,
    transferred: u32,
    sector: [u8; MSC_SECTOR_SIZE],
    /// A media-changed remount awaiting its UNIT ATTENTION report; survives a BOT reset.
    ua_pending: bool,
    /// The data-phase level the block device already believes; only edges are published.
    data_phase: bool,
}

impl<const EP: usize> Machine<EP> {
    /// `WaitCbw` lands its packet in [`Machine::sector`], so the scratch must hold one.
    const EP_FITS_SECTOR: () = assert!(EP <= MSC_SECTOR_SIZE);

    fn new() -> Self {
        let () = Self::EP_FITS_SECTOR;
        Self {
            phase: Phase::WaitCbw,
            sense: SENSE_NONE,
            tag: 0,
            data_len: 0,
            transferred: 0,
            sector: [0u8; MSC_SECTOR_SIZE],
            ua_pending: false,
            data_phase: false,
        }
    }

    /// Publish a data-phase edge, so the volume holds a media change until the payload has moved.
    fn set_data_phase<I: SectorIo + ?Sized>(&mut self, io: &mut I, open: bool) {
        if self.data_phase != open {
            self.data_phase = open;
            io.set_data_phase(open);
        }
    }

    /// Return to the idle state after a completed command or a host-issued BOT reset.
    fn reset(&mut self) {
        self.phase = Phase::WaitCbw;
        self.transferred = 0;
    }

    /// Move to the CSW phase, with the residue of advertised against transferred bytes.
    fn finish(&mut self, status: u8) {
        let residue = self.data_len.saturating_sub(self.transferred);
        self.phase = Phase::SendCsw {
            csw: build_csw(self.tag, residue, status),
            sent: 0,
        };
    }

    /// Drive until progress stops on a busy endpoint or a pending sector.
    fn pump<P: BulkPort + ?Sized, I: SectorIo + ?Sized>(&mut self, port: &mut P, io: &mut I) {
        for _ in 0..PUMP_BOUND {
            if !self.step(port, io) {
                break;
            }
        }
    }

    /// Advance one packet or phase transition, reporting whether progress was made.
    fn step<P: BulkPort + ?Sized, I: SectorIo + ?Sized>(
        &mut self,
        port: &mut P,
        io: &mut I,
    ) -> bool {
        match self.phase {
            Phase::WaitCbw => {
                // Between commands no phase is open, and this is what closes one a reset tore down.
                self.set_data_phase(io, false);
                if io.media_changed() {
                    self.ua_pending = true;
                }
                // A desynchronised host delivers up to `EP` bytes, and a short buffer can panic.
                let n = match port.read(&mut self.sector[..]) {
                    Ok(n) => n,
                    Err(()) => return false,
                };
                let Some(cbw) = parse_cbw(&self.sector[..n]) else {
                    // A malformed CBW is fatal: stall both pipes and wait for a BOT reset.
                    #[cfg(feature = "defmt")]
                    defmt::warn!(
                        "msc: {=usize}-byte packet is not a CBW; stalling both pipes",
                        n
                    );
                    port.stall_in();
                    port.stall_out();
                    return true;
                };
                self.tag = cbw.tag;
                self.data_len = cbw.data_len;
                self.transferred = 0;
                // Single LUN (GET MAX LUN reports 0): any other addresses a unit that is absent.
                if cbw.lun != 0 {
                    self.sense = SENSE_LUN_NOT_SUPPORTED;
                    self.stall_declared(port, cbw.dir_in);
                    self.finish(CSW_FAILED);
                    return true;
                }
                // Only a TEST UNIT READY declaring no data phase may consume the latch.
                if self.ua_pending && is_test_unit_ready(cbw.cb[0]) && self.data_len == 0 {
                    self.ua_pending = false;
                    self.sense = SENSE_MEDIUM_CHANGED;
                    self.finish(CSW_FAILED);
                    return true;
                }
                let action = dispatch(&cbw.cb, io.media_ready(), self.sense);
                // Reconcile before any data phase runs, or a disagreement desynchronises the stream.
                if let BotVerdict::Reject {
                    status,
                    sense,
                    stall_dir_in,
                } = reconcile_cbw(cbw.dir_in, self.data_len, &action)
                {
                    self.sense = sense;
                    match stall_dir_in {
                        Some(true) => port.stall_in(),
                        Some(false) => port.stall_out(),
                        None => {}
                    }
                    self.finish(status);
                    return true;
                }
                match action {
                    ScsiAction::Reply { data, len } => {
                        let len = clamp_reply_len(len, self.data_len);
                        // Cleared before the phase is queued, so a torn reply loses the condition.
                        if should_clear_sense(cbw.cb[0]) {
                            self.sense = SENSE_NONE;
                        }
                        self.phase = Phase::ReplyIn {
                            buf: data,
                            len,
                            sent: 0,
                        };
                    }
                    ScsiAction::Ok => self.finish(CSW_PASSED),
                    ScsiAction::ReadSectors { lba, count } => {
                        self.phase = Phase::DataIn {
                            lba,
                            remaining: count,
                            offset: 0,
                            loaded: false,
                        };
                    }
                    ScsiAction::WriteSectors { lba, count } => {
                        // The transport now owes the whole payload: the medium may not change.
                        self.set_data_phase(io, true);
                        self.phase = Phase::DataOut {
                            lba,
                            remaining: count,
                            offset: 0,
                        };
                    }
                    ScsiAction::Fail(s) => {
                        self.sense = s;
                        self.stall_declared(port, cbw.dir_in);
                        self.finish(CSW_FAILED);
                    }
                }
                true
            }
            Phase::ReplyIn { buf, len, sent } => {
                if sent >= len {
                    self.transferred = len as u32;
                    // A clamped-to-nothing reply leaves an IN read that would take the CSW for data.
                    if len == 0 && self.data_len > 0 {
                        port.stall_in();
                    }
                    self.finish(CSW_PASSED);
                    return true;
                }
                let end = core::cmp::min(sent + EP, len);
                match port.write(&buf[sent..end]) {
                    Ok(_) => {
                        self.phase = Phase::ReplyIn {
                            buf,
                            len,
                            sent: end,
                        };
                        true
                    }
                    Err(()) => false,
                }
            }
            Phase::DataIn {
                lba,
                remaining,
                offset,
                loaded,
            } => {
                if remaining == 0 {
                    self.finish(CSW_PASSED);
                    return true;
                }
                if !loaded {
                    return match io.read(lba, &mut self.sector) {
                        SectorPoll::Pending => false,
                        SectorPoll::Ready => {
                            self.phase = Phase::DataIn {
                                lba,
                                remaining,
                                offset,
                                loaded: true,
                            };
                            true
                        }
                    };
                }
                let end = core::cmp::min(offset + EP, MSC_SECTOR_SIZE);
                match port.write(&self.sector[offset..end]) {
                    Ok(_) => {
                        self.transferred += (end - offset) as u32;
                        self.phase = if end >= MSC_SECTOR_SIZE {
                            Phase::DataIn {
                                lba: lba + 1,
                                remaining: remaining - 1,
                                offset: 0,
                                loaded: false,
                            }
                        } else {
                            Phase::DataIn {
                                lba,
                                remaining,
                                offset: end,
                                loaded: true,
                            }
                        };
                        true
                    }
                    Err(()) => false,
                }
            }
            Phase::DataOut {
                lba,
                remaining,
                offset,
            } => {
                if remaining == 0 {
                    // Released before the CSW, so a detach armed mid-command lands on this boundary.
                    self.set_data_phase(io, false);
                    self.finish(CSW_PASSED);
                    return true;
                }
                if offset >= MSC_SECTOR_SIZE {
                    return match io.write(lba, &self.sector) {
                        // Leaving the next OUT packet unread is the flow control (the EP NAKs).
                        SectorPoll::Pending => false,
                        SectorPoll::Ready => {
                            self.phase = Phase::DataOut {
                                lba: lba + 1,
                                remaining: remaining - 1,
                                offset: 0,
                            };
                            true
                        }
                    };
                }
                match port.read(&mut self.sector[offset..MSC_SECTOR_SIZE]) {
                    Ok(r) => {
                        self.transferred += r as u32;
                        self.phase = Phase::DataOut {
                            lba,
                            remaining,
                            offset: offset + r,
                        };
                        true
                    }
                    Err(()) => false,
                }
            }
            Phase::SendCsw { csw, sent } => {
                if sent >= CSW_LEN {
                    self.reset();
                    return true;
                }
                match port.write(&csw[sent..CSW_LEN]) {
                    Ok(_) => {
                        self.phase = Phase::SendCsw { csw, sent: CSW_LEN };
                        true
                    }
                    Err(()) => false,
                }
            }
        }
    }

    /// Stall the direction a rejected command declared, so the host's transfer ends at once.
    fn stall_declared<P: BulkPort + ?Sized>(&self, port: &mut P, dir_in: bool) {
        if self.data_len > 0 {
            #[cfg(feature = "defmt")]
            defmt::warn!(
                "msc: command rejected, stalling declared dir_in={=bool}",
                dir_in
            );
            if dir_in {
                port.stall_in();
            } else {
                port.stall_out();
            }
        }
    }
}

/// One SCSI-transparent LUN over Bulk-Only Transport; `EP` is the bulk packet size.
pub struct MscClass<'a, B: UsbBus, const EP: usize = 64> {
    iface: InterfaceNumber,
    ep_in: EndpointIn<'a, B>,
    ep_out: EndpointOut<'a, B>,
    machine: Machine<EP>,
}

impl<'a, B: UsbBus, const EP: usize> MscClass<'a, B, EP> {
    /// Allocate the interface and the two bulk endpoints from `alloc`.
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            iface: alloc.interface(),
            ep_in: alloc.bulk(EP as u16),
            ep_out: alloc.bulk(EP as u16),
            machine: Machine::new(),
        }
    }

    /// Advance the transport over `vfs` as far as the endpoints and the volume allow.
    pub fn service<S: TransferSink, O: VfsObserver>(&mut self, vfs: &mut Vfs<'_, S, O>) {
        let mut io = DirectIo::new(vfs);
        // Disjoint field borrows: the port takes the endpoints, the machine its own state.
        let mut port = EndpointPort {
            ep_in: &self.ep_in,
            ep_out: &self.ep_out,
        };
        self.machine.pump(&mut port, &mut io);
    }
}

impl<B: UsbBus, const EP: usize> UsbClass<B> for MscClass<'_, B, EP> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        let (class, subclass, protocol) = MSC_INTERFACE_CLASS;
        writer.interface(self.iface, class, subclass, protocol)?;
        writer.endpoint(&self.ep_in)?;
        writer.endpoint(&self.ep_out)
    }

    fn reset(&mut self) {
        self.machine.reset();
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        if self.is_for_us(xfer.request(), MSC_GET_MAX_LUN) {
            // Single logical unit: the maximum LUN index is 0.
            xfer.accept_with(&[0]).ok();
        }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        if self.is_for_us(xfer.request(), MSC_BULK_ONLY_RESET) {
            self.machine.reset();
            xfer.accept().ok();
        }
    }
}

impl<B: UsbBus, const EP: usize> MscClass<'_, B, EP> {
    /// Whether `req` is class request `code` addressed to this interface.
    fn is_for_us(&self, req: &usb_device::control::Request, code: u8) -> bool {
        req.request_type == RequestType::Class
            && req.recipient == Recipient::Interface
            && req.index == u8::from(self.iface) as u16
            && req.request == code
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use daplink_rs::daplink_core::msc_bot_cases::BOT_CASES;
    use daplink_rs::daplink_core::msc_proto::{CBW_LEN, CBW_SIGNATURE, CSW_SIGNATURE};
    use daplink_rs::daplink_core::{SCSI_PRODUCT_ID, SCSI_VENDOR_ID};
    use std::collections::{BTreeMap, VecDeque};
    use std::vec::Vec;

    /// The bulk packet size the tests drive the machine at.
    const EP_CHUNK: usize = 64;

    /// A mock [`BulkPort`]: OUT packets queue in `input`, IN writes land in `output`.
    struct MockPort {
        input: VecDeque<Vec<u8>>,
        output: Vec<u8>,
        /// Remaining IN writes before the endpoint reports busy, for resumability tests.
        write_budget: usize,
        stalled_in: bool,
        stalled_out: bool,
    }

    impl MockPort {
        fn new() -> Self {
            Self {
                input: VecDeque::new(),
                output: Vec::new(),
                write_budget: usize::MAX,
                stalled_in: false,
                stalled_out: false,
            }
        }
        fn push_out(&mut self, pkt: &[u8]) {
            self.input.push_back(pkt.to_vec());
        }
    }

    impl BulkPort for MockPort {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
            let pkt = self.input.pop_front().ok_or(())?;
            // As unforgiving as the strictest bus driver, which panics rather than truncating.
            assert!(
                pkt.len() <= buf.len(),
                "read buffer ({} bytes) is shorter than the packet ({} bytes)",
                buf.len(),
                pkt.len()
            );
            buf[..pkt.len()].copy_from_slice(&pkt);
            Ok(pkt.len())
        }
        fn write(&mut self, data: &[u8]) -> Result<usize, ()> {
            if self.write_budget == 0 {
                return Err(());
            }
            self.write_budget -= 1;
            self.output.extend_from_slice(data);
            Ok(data.len())
        }
        fn stall_in(&mut self) {
            self.stalled_in = true;
        }
        fn stall_out(&mut self) {
            self.stalled_out = true;
        }
    }

    /// A trivial in-memory block device over the [`SectorIo`] seam.
    struct RamDisk {
        sectors: BTreeMap<u32, [u8; MSC_SECTOR_SIZE]>,
        ready: bool,
        changed: bool,
    }

    impl RamDisk {
        fn new(ready: bool) -> Self {
            Self {
                sectors: BTreeMap::new(),
                ready,
                changed: false,
            }
        }
    }

    impl SectorIo for RamDisk {
        fn media_ready(&self) -> bool {
            self.ready
        }
        fn read(&mut self, lba: u32, buf: &mut [u8; MSC_SECTOR_SIZE]) -> SectorPoll {
            *buf = self
                .sectors
                .get(&lba)
                .copied()
                .unwrap_or([0u8; MSC_SECTOR_SIZE]);
            SectorPoll::Ready
        }
        fn write(&mut self, lba: u32, buf: &[u8; MSC_SECTOR_SIZE]) -> SectorPoll {
            self.sectors.insert(lba, *buf);
            SectorPoll::Ready
        }
        fn media_changed(&mut self) -> bool {
            core::mem::replace(&mut self.changed, false)
        }
    }

    /// A block device that reports `Pending` a fixed number of times before each write completes.
    struct PendingDisk {
        sectors: BTreeMap<u32, [u8; MSC_SECTOR_SIZE]>,
        write_pending: u32,
    }

    impl SectorIo for PendingDisk {
        fn media_ready(&self) -> bool {
            true
        }
        fn read(&mut self, _lba: u32, _buf: &mut [u8; MSC_SECTOR_SIZE]) -> SectorPoll {
            SectorPoll::Ready
        }
        fn write(&mut self, lba: u32, buf: &[u8; MSC_SECTOR_SIZE]) -> SectorPoll {
            if self.write_pending > 0 {
                self.write_pending -= 1;
                return SectorPoll::Pending;
            }
            self.sectors.insert(lba, *buf);
            SectorPoll::Ready
        }
    }

    /// Build the 31 CBW bytes.
    fn cbw(tag: u32, data_len: u32, dir_in: bool, cb: &[u8]) -> [u8; CBW_LEN] {
        let mut pkt = [0u8; CBW_LEN];
        pkt[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
        pkt[4..8].copy_from_slice(&tag.to_le_bytes());
        pkt[8..12].copy_from_slice(&data_len.to_le_bytes());
        pkt[12] = if dir_in { 0x80 } else { 0x00 };
        pkt[14] = cb.len() as u8;
        pkt[15..15 + cb.len()].copy_from_slice(cb);
        pkt
    }

    /// Split a CSW into `(tag, residue, status)`.
    fn parse_csw(b: &[u8]) -> (u32, u32, u8) {
        assert_eq!(b.len(), CSW_LEN, "CSW length");
        assert_eq!(
            u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            CSW_SIGNATURE,
            "CSW signature"
        );
        (
            u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
            u32::from_le_bytes([b[8], b[9], b[10], b[11]]),
            b[12],
        )
    }

    /// Run one command to completion, splitting the captured IN stream into `(data, csw)`.
    fn run_cmd<const EP: usize>(
        m: &mut Machine<EP>,
        io: &mut RamDisk,
        cbw_pkt: &[u8],
        out_data: &[&[u8]],
    ) -> (Vec<u8>, (u32, u32, u8)) {
        let mut port = MockPort::new();
        port.push_out(cbw_pkt);
        for d in out_data {
            port.push_out(d);
        }
        m.pump(&mut port, io);
        let total = port.output.len();
        assert!(total >= CSW_LEN, "at least a CSW must have been sent");
        let data = port.output[..total - CSW_LEN].to_vec();
        let csw = parse_csw(&port.output[total - CSW_LEN..]);
        (data, csw)
    }

    #[test]
    fn inquiry_carries_identity() {
        let mut m: Machine = Machine::new();
        let mut io = RamDisk::new(true);
        let cb = [0x12u8, 0x00, 0x00, 0x00, 36, 0x00];
        let (data, csw) = run_cmd(&mut m, &mut io, &cbw(0x11, 36, true, &cb), &[]);
        assert_eq!(data.len(), 36);
        assert_eq!(&data[8..16], SCSI_VENDOR_ID);
        assert_eq!(&data[16..32], SCSI_PRODUCT_ID);
        assert_eq!(csw, (0x11, 0, CSW_PASSED));
    }

    #[test]
    fn read10_round_trips_a_sector() {
        let mut m: Machine = Machine::new();
        let mut io = RamDisk::new(true);
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        for (i, b) in sector.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }
        io.sectors.insert(7, sector);
        let cb = [0x28u8, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x01, 0x00];
        let (data, csw) = run_cmd(
            &mut m,
            &mut io,
            &cbw(0x22, MSC_SECTOR_SIZE as u32, true, &cb),
            &[],
        );
        assert_eq!(data, sector.to_vec());
        assert_eq!(csw, (0x22, 0, CSW_PASSED));
    }

    #[test]
    fn write10_round_trips_a_sector() {
        let mut m: Machine = Machine::new();
        let mut io = RamDisk::new(true);
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        for (i, b) in sector.iter_mut().enumerate() {
            *b = (0xFF - (i % 251)) as u8;
        }
        let chunks: Vec<&[u8]> = sector.chunks(EP_CHUNK).collect();
        let cb = [0x2Au8, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x01, 0x00];
        let (data, csw) = run_cmd(
            &mut m,
            &mut io,
            &cbw(0x33, MSC_SECTOR_SIZE as u32, false, &cb),
            &chunks,
        );
        assert!(data.is_empty(), "a write has no device-to-host data");
        assert_eq!(csw, (0x33, 0, CSW_PASSED));
        assert_eq!(io.sectors.get(&9), Some(&sector));
    }

    /// Drive one machine at packet size `EP` through the SCSI probe sequence macOS's block
    /// driver issues while mounting, returning each command's `(data, csw)`.
    fn macos_mount_probe<const EP: usize>() -> Vec<(Vec<u8>, (u32, u32, u8))> {
        let mut m: Machine<EP> = Machine::new();
        let mut io = RamDisk::new(true);
        let mut runs = Vec::new();
        // INQUIRY, 36-byte allocation.
        let inq = [0x12u8, 0x00, 0x00, 0x00, 36, 0x00];
        runs.push(run_cmd(&mut m, &mut io, &cbw(1, 36, true, &inq), &[]));
        // TEST UNIT READY, no data phase.
        let tur = [0x00u8, 0, 0, 0, 0, 0];
        runs.push(run_cmd(&mut m, &mut io, &cbw(2, 0, false, &tur), &[]));
        // MODE SENSE(6), all pages, 192-byte allocation: the write-protect probe.
        let ms = [0x1Au8, 0x00, 0x3F, 0x00, 0xC0, 0x00];
        runs.push(run_cmd(&mut m, &mut io, &cbw(3, 0xC0, true, &ms), &[]));
        // READ CAPACITY(10), 8 bytes.
        let rc = [0x25u8, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        runs.push(run_cmd(&mut m, &mut io, &cbw(4, 8, true, &rc), &[]));
        // READ(10), one sector at LBA 0.
        let rd = [0x28u8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00];
        runs.push(run_cmd(
            &mut m,
            &mut io,
            &cbw(5, MSC_SECTOR_SIZE as u32, true, &rd),
            &[],
        ));
        runs
    }

    #[test]
    fn macos_mount_probe_completes_at_the_high_speed_packet_size() {
        // The 512-byte machine must answer the mount probe exactly as the 64-byte one does.
        let hs = macos_mount_probe::<512>();
        let (ms_data, ms_csw) = &hs[2];
        assert_eq!(ms_data, &[3u8, 0, 0, 0], "MODE SENSE data phase");
        assert_eq!(*ms_csw, (3, 0xC0 - 4, CSW_PASSED), "MODE SENSE CSW");
        assert_eq!(
            hs,
            macos_mount_probe::<64>(),
            "the packet size changes nothing on the wire"
        );
    }

    /// The shared Bulk-Only case table at packet size `EP`; only the stall column is local.
    fn bot_case_table_at<const EP: usize>() {
        for case in BOT_CASES {
            let name = case.name;
            let mut io = RamDisk::new(true);
            let mut m: Machine<EP> = Machine::new();
            m.ua_pending = case.ua_before;
            let mut port = MockPort::new();
            port.push_out(&case.cbw);
            // The fixtures chunk at 64; a wider endpoint accepts short packets, so this is legal.
            for _ in 0..case.out_packets {
                port.push_out(&[0u8; EP_CHUNK]);
            }
            m.pump(&mut port, &mut io);

            assert_eq!(
                port.output.len(),
                case.data_len + CSW_LEN,
                "{name}: device-to-host byte count"
            );
            assert_eq!(
                parse_csw(&port.output[case.data_len..]),
                (case.tag(), case.residue, case.status),
                "{name}: CSW"
            );
            assert_eq!(m.sense, case.sense, "{name}: latched sense");
            assert_eq!(
                (port.stalled_in, port.stalled_out),
                case.stalled_pair(),
                "{name}: stalled endpoints"
            );
            assert_eq!(
                m.ua_pending, case.ua_after,
                "{name}: pending unit attention"
            );
            assert!(port.input.is_empty(), "{name}: host packets all consumed");
        }
    }

    #[test]
    fn bot_case_table_is_answered_as_the_shared_expectations_say() {
        bot_case_table_at::<EP_CHUNK>();
    }

    #[test]
    fn bot_case_table_holds_at_the_high_speed_packet_size() {
        bot_case_table_at::<512>();
    }

    #[test]
    fn nonzero_lun_is_rejected() {
        let mut m: Machine = Machine::new();
        let mut io = RamDisk::new(true);
        let mut c = cbw(0x51, 0, false, &[0x00u8, 0, 0, 0, 0, 0]);
        c[13] = 0x01;
        let (data, (tag, _residue, status)) = run_cmd(&mut m, &mut io, &c, &[]);
        assert_eq!((tag, status), (0x51, CSW_FAILED), "a LUN 1 command fails");
        assert!(data.is_empty(), "no data phase for a rejected LUN");
    }

    #[test]
    fn malformed_cbw_stalls_both_endpoints() {
        let mut m: Machine = Machine::new();
        let mut io = RamDisk::new(true);
        let mut port = MockPort::new();
        port.push_out(&[0xA5u8; CBW_LEN]);
        m.pump(&mut port, &mut io);
        assert!(port.output.is_empty(), "no CSW answers a malformed CBW");
        assert!((port.stalled_in, port.stalled_out) == (true, true));
    }

    #[test]
    fn a_transfer_resumes_across_pumps() {
        // A busy IN endpoint mid-reply must leave the command resumable, not restart or drop it.
        let mut m: Machine = Machine::new();
        let mut io = RamDisk::new(true);
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        sector[0] = 0x5A;
        io.sectors.insert(3, sector);
        let cb = [0x28u8, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x01, 0x00];
        let mut port = MockPort::new();
        port.push_out(&cbw(0x44, MSC_SECTOR_SIZE as u32, true, &cb));
        port.write_budget = 3;
        m.pump(&mut port, &mut io);
        assert_eq!(
            port.output.len(),
            3 * EP_CHUNK,
            "the budget bounded the pump"
        );

        port.write_budget = usize::MAX;
        m.pump(&mut port, &mut io);
        assert_eq!(port.output.len(), MSC_SECTOR_SIZE + CSW_LEN);
        assert_eq!(port.output[..MSC_SECTOR_SIZE], sector);
        assert_eq!(
            parse_csw(&port.output[MSC_SECTOR_SIZE..]),
            (0x44, 0, CSW_PASSED)
        );
    }

    #[test]
    fn a_pending_write_holds_off_the_next_packet() {
        // A queued sector must not be consumed while the previous one is still draining.
        let mut io = PendingDisk {
            sectors: BTreeMap::new(),
            write_pending: 2,
        };
        let mut m: Machine = Machine::new();
        let cb = [0x2Au8, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x02, 0x00];
        let mut port = MockPort::new();
        port.push_out(&cbw(0x77, 2 * MSC_SECTOR_SIZE as u32, false, &cb));
        for _ in 0..(MSC_SECTOR_SIZE / EP_CHUNK) {
            port.push_out(&[0x11u8; EP_CHUNK]);
        }
        port.push_out(&[0x22u8; EP_CHUNK]);

        m.pump(&mut port, &mut io);
        assert!(!io.sectors.contains_key(&9), "the sector is still draining");
        assert_eq!(port.input.len(), 1, "the next packet stayed unread");

        for _ in 1..(MSC_SECTOR_SIZE / EP_CHUNK) {
            port.push_out(&[0x22u8; EP_CHUNK]);
        }
        for _ in 0..PUMP_BOUND {
            m.pump(&mut port, &mut io);
            if io.sectors.contains_key(&10) {
                break;
            }
        }
        assert_eq!(io.sectors.get(&9), Some(&[0x11u8; MSC_SECTOR_SIZE]));
        assert_eq!(io.sectors.get(&10), Some(&[0x22u8; MSC_SECTOR_SIZE]));
        assert_eq!(
            parse_csw(&port.output[port.output.len() - CSW_LEN..]),
            (0x77, 0, CSW_PASSED)
        );
    }

    /// One thing a block device saw, in order.
    #[derive(PartialEq, Eq, Debug)]
    enum Ev {
        /// The transport opened a host command's data phase.
        Open,
        /// The transport closed it.
        Close,
        /// A sector was written at this LBA.
        Wrote(u32),
    }

    /// A block device that records the data-phase bracket interleaved with its writes.
    struct PhaseLog {
        events: Vec<Ev>,
    }
    impl SectorIo for PhaseLog {
        fn media_ready(&self) -> bool {
            true
        }
        fn read(&mut self, _lba: u32, _buf: &mut [u8; MSC_SECTOR_SIZE]) -> SectorPoll {
            SectorPoll::Ready
        }
        fn write(&mut self, lba: u32, _buf: &[u8; MSC_SECTOR_SIZE]) -> SectorPoll {
            self.events.push(Ev::Wrote(lba));
            SectorPoll::Ready
        }
        fn set_data_phase(&mut self, open: bool) {
            self.events.push(if open { Ev::Open } else { Ev::Close });
        }
    }

    #[test]
    fn write10_brackets_its_data_phase_around_every_sector() {
        // Media state may only change on a command boundary, so the phase brackets every sector.
        let mut io = PhaseLog { events: Vec::new() };
        let mut m: Machine = Machine::new();
        let cb = [0x2Au8, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x02, 0x00];
        let mut port = MockPort::new();
        port.push_out(&cbw(0x66, 2 * MSC_SECTOR_SIZE as u32, false, &cb));
        for pkt in [0xA5u8; 2 * MSC_SECTOR_SIZE].chunks(EP_CHUNK) {
            port.push_out(pkt);
        }
        m.pump(&mut port, &mut io);
        assert_eq!(
            io.events,
            std::vec![Ev::Open, Ev::Wrote(7), Ev::Wrote(8), Ev::Close]
        );
    }

    #[test]
    fn a_data_phase_torn_down_mid_flight_is_released_on_the_next_cbw() {
        // An abandoned WRITE(10) leaves the phase open; the return to `WaitCbw` releases it.
        let mut io = PhaseLog { events: Vec::new() };
        let mut m: Machine = Machine::new();
        let cb = [0x2Au8, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x02, 0x00];
        let mut port = MockPort::new();
        port.push_out(&cbw(0x67, 2 * MSC_SECTOR_SIZE as u32, false, &cb));
        m.pump(&mut port, &mut io);
        assert_eq!(io.events, std::vec![Ev::Open], "no payload arrived");
        // Bulk-Only reset, then an idle poll with nothing scripted.
        m.reset();
        m.pump(&mut port, &mut io);
        assert_eq!(io.events, std::vec![Ev::Open, Ev::Close]);
    }

    /// Build a root-directory entry for a dropped file.
    fn dir_entry(name: &[u8; 11], attr: u8, first_cluster: u16, size: u32) -> [u8; 32] {
        let mut e = [0u8; 32];
        e[0..11].copy_from_slice(name);
        e[11] = attr;
        e[26..28].copy_from_slice(&first_cluster.to_le_bytes());
        e[28..32].copy_from_slice(&size.to_le_bytes());
        e
    }

    /// One well-formed 512-byte UF2 block, the smallest thing that starts and finishes a transfer.
    fn uf2_block() -> [u8; MSC_SECTOR_SIZE] {
        let mut b = [0u8; MSC_SECTOR_SIZE];
        b[0..4].copy_from_slice(&0x0A32_4655u32.to_le_bytes()); // MAGIC_START0
        b[4..8].copy_from_slice(&0x9E5D_5157u32.to_le_bytes()); // MAGIC_START1
        b[12..16].copy_from_slice(&0x0800_0000u32.to_le_bytes()); // target addr
        b[16..20].copy_from_slice(&4u32.to_le_bytes()); // payload size
        b[508..512].copy_from_slice(&0x0AB1_6F30u32.to_le_bytes()); // MAGIC_END
        b
    }

    #[test]
    fn completion_mid_write10_does_not_detach_until_the_phase_ends() {
        use daplink_vfs::{MediaState, NoObserver, NullSink, Vfs, VfsConfig, VfsFile};

        // A drop completing mid-WRITE(10) must not take the medium from the running command.
        let mut files: [&mut dyn VfsFile; 0] = [];
        let mut vfs: Vfs<'_, _, _> =
            Vfs::new(VfsConfig::default(), &mut files[..], NullSink, NoObserver);
        // Bind a 512-byte `.UF2` at cluster 2 (LBA 133).
        let mut root = [0u8; MSC_SECTOR_SIZE];
        root[0..32].copy_from_slice(&dir_entry(b"APP     UF2", 0x20, 2, MSC_SECTOR_SIZE as u32));
        vfs.write_sector(131, &root);

        // Two sectors at LBA 133: the declared size is met on the first, slack follows.
        let cb = [0x2Au8, 0x00, 0x00, 0x00, 0x00, 0x85, 0x00, 0x00, 0x02, 0x00];
        let mut m: Machine = Machine::new();
        let mut port = MockPort::new();
        port.push_out(&cbw(0x77, 2 * MSC_SECTOR_SIZE as u32, false, &cb));
        for pkt in uf2_block().chunks(EP_CHUNK) {
            port.push_out(pkt);
        }
        {
            let mut io = DirectIo::new(&mut vfs);
            m.pump(&mut port, &mut io);
        }
        assert_eq!(
            vfs.tick(1),
            MediaState::Ready,
            "the medium must survive the command it is inside"
        );

        // The last sector arrives: the phase closes and the detach lands on that boundary.
        for pkt in [0u8; MSC_SECTOR_SIZE].chunks(EP_CHUNK) {
            port.push_out(pkt);
        }
        {
            let mut io = DirectIo::new(&mut vfs);
            m.pump(&mut port, &mut io);
        }
        assert_eq!(vfs.media(), MediaState::Detached);
        // The whole declared payload was accepted, so the command passes with no residue.
        assert_eq!(
            parse_csw(&port.output),
            (0x77, 0, CSW_PASSED),
            "the command completes cleanly"
        );
        // And the remount the drop earned is deferred, not lost.
        assert_eq!(vfs.tick(daplink_vfs::MSC_RECONNECT_MS), MediaState::Ready);
        assert!(vfs.take_media_changed());
    }

    #[test]
    fn unit_attention_survives_bot_reset() {
        // UNIT ATTENTION persists across a Bulk-Only Transport reset: a BOT reset is not a remount.
        let mut io = RamDisk::new(true);
        io.changed = true; // a remount just latched a media-changed edge
        let mut m: Machine = Machine::new();

        // An INQUIRY latches the media-changed edge without consuming it.
        let inq = [0x12u8, 0x00, 0x00, 0x00, 36, 0x00];
        let (_, (_, _, status)) = run_cmd(&mut m, &mut io, &cbw(1, 36, true, &inq), &[]);
        assert_eq!(status, CSW_PASSED);
        assert!(m.ua_pending, "the latch survives a passing command");

        // A mid-session BOT reset must not clear the pending attention.
        m.reset();
        assert!(m.ua_pending, "a BOT reset does not clear UNIT ATTENTION");
        let tur = [0x00u8];
        let (_, (_, _, status)) = run_cmd(&mut m, &mut io, &cbw(2, 0, false, &tur), &[]);
        assert_eq!(
            status, CSW_FAILED,
            "UNIT ATTENTION is not cleared by a BOT reset"
        );

        // Drain the medium-changed sense so the transport returns to a clean idle.
        let rs = [0x03u8, 0x00, 0x00, 0x00, 18, 0x00];
        let (sense, (_, _, status)) = run_cmd(&mut m, &mut io, &cbw(3, 18, true, &rs), &[]);
        assert_eq!(status, CSW_PASSED);
        assert_eq!(sense[2], 0x06, "medium-changed sense key");

        // A subsequent TEST UNIT READY passes normally (no attention still pending).
        let (_, (_, _, status)) = run_cmd(&mut m, &mut io, &cbw(4, 0, false, &tur), &[]);
        assert_eq!(status, CSW_PASSED);
    }
}
