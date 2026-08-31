//! The CMSIS-DAP vendor flash commands: a sequential image stream decoded straight into a
//! [`TransferSink`], with the command ids and framing of the C firmware's vendor surface.

use daplink_core::vendor::VendorHandler;
use daplink_core::FlashPhase;
use daplink_decoder::{Decoder, Format};

use crate::decoder_set::DecoderSet;
use crate::transfer::{SinkCtx, TransferOutcome, TransferSink};

/// `ID_DAP_MSD_Open` (`ID_DAP_Vendor10`): open the image stream.
pub const ID_DAP_MSD_OPEN: u8 = 0x8A;
/// `ID_DAP_MSD_Close` (`ID_DAP_Vendor11`): finish the image stream.
pub const ID_DAP_MSD_CLOSE: u8 = 0x8B;
/// `ID_DAP_MSD_Write` (`ID_DAP_Vendor12`): one chunk of the image stream.
pub const ID_DAP_MSD_WRITE: u8 = 0x8C;
/// `ID_DAP_SelectEraseMode` (`ID_DAP_Vendor13`): chip erase vs sector erase.
pub const ID_DAP_SELECT_ERASE_MODE: u8 = 0x8D;

/// `STREAM_TYPE_BIN`: raw binary, no in-tree decoder.
pub const STREAM_TYPE_BIN: u8 = 0;
/// `STREAM_TYPE_HEX`: Intel hex, no in-tree decoder.
pub const STREAM_TYPE_HEX: u8 = 1;
/// uf2, at the slot after the C firmware's `STREAM_TYPE_HEX`.
pub const STREAM_TYPE_UF2: u8 = 2;

/// Response status bytes, numbered exactly as the C firmware's `error_t` enumeration.
pub mod status {
    /// `ERROR_SUCCESS`.
    pub const SUCCESS: u8 = 0;
    /// `ERROR_INTERNAL`: the command is illegal in the current stream state, or names no decoder.
    pub const INTERNAL: u8 = 2;
    /// `ERROR_ERROR_DURING_TRANSFER`: the stream's content was rejected.
    pub const DURING_TRANSFER: u8 = 3;
    /// `ERROR_FILE_BOUNDS`.
    pub const FILE_BOUNDS: u8 = 5;
    /// `ERROR_INIT`.
    pub const INIT: u8 = 11;
    /// `ERROR_UNINIT`.
    pub const UNINIT: u8 = 12;
    /// `ERROR_ERASE_SECTOR`.
    pub const ERASE_SECTOR: u8 = 15;
    /// `ERROR_WRITE`.
    pub const WRITE: u8 = 17;
    /// `ERROR_WRITE_VERIFY`.
    pub const WRITE_VERIFY: u8 = 18;
    /// `ERROR_FD_INCOMPATIBLE_IMAGE`.
    pub const INCOMPATIBLE_IMAGE: u8 = 29;

    /// `DAP_OK`, the erase-mode reply.
    pub const DAP_OK: u8 = 0x00;
    /// `DAP_ERROR`, returned where the C firmware cannot fail but this one can.
    pub const DAP_ERROR: u8 = 0xFF;
}

/// Map a latched flash phase onto the nearest C `error_t` code.
fn phase_status(phase: FlashPhase) -> u8 {
    match phase {
        FlashPhase::NoTarget | FlashPhase::Init => status::INIT,
        FlashPhase::Identity | FlashPhase::Family | FlashPhase::Content => {
            status::INCOMPATIBLE_IMAGE
        }
        FlashPhase::Range => status::FILE_BOUNDS,
        FlashPhase::Erase => status::ERASE_SECTOR,
        FlashPhase::Program => status::WRITE,
        FlashPhase::Verify => status::WRITE_VERIFY,
        FlashPhase::Uninit => status::UNINIT,
    }
}

/// The stream type a host may open, mapped to the format it names.
fn stream_format(stream_type: u8) -> Option<Format> {
    match stream_type {
        STREAM_TYPE_UF2 => Some(Format::Uf2),
        _ => None,
    }
}

/// Scratchpad bytes reserved for a record/block straddling two chunks; matches the transfer engine.
const SCRATCH: usize = 1024;

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Closed,
    Open,
    Failed,
}

/// The vendor flash-stream protocol state, over the accepted-format set `D`. A vendor stream is
/// sequential, so chunks reach the decoder as they arrive and never need a write-order queue.
pub struct VendorFlashStream<D: DecoderSet> {
    state: State,
    fmt: Format,
    decoder: Option<D::Running>,
    scratch: [u8; SCRATCH],
    /// Whether the leading bytes still have to be checked against the format the host opened.
    check_head: bool,
    /// Leading stream bytes retained until that check can run; detection needs four.
    head: [u8; 4],
    head_len: usize,
}

impl<D: DecoderSet> Default for VendorFlashStream<D> {
    fn default() -> Self {
        Self::new()
    }
}

impl<D: DecoderSet> VendorFlashStream<D> {
    /// A closed stream.
    pub const fn new() -> Self {
        Self {
            state: State::Closed,
            fmt: Format::Unknown,
            decoder: None,
            scratch: [0u8; SCRATCH],
            check_head: true,
            head: [0u8; 4],
            head_len: 0,
        }
    }

    /// Whether an image stream is currently open (including one that has already failed).
    pub fn is_open(&self) -> bool {
        self.state != State::Closed
    }

    /// Handle one vendor command against `sink`; commands outside the flash set are declined.
    pub fn handle<S: TransferSink>(
        &mut self,
        report: &[u8],
        response: &mut [u8],
        sink: &mut S,
    ) -> Option<usize> {
        // Every reply in this set is the echoed id plus one status byte.
        if response.len() < 2 {
            return None;
        }
        let status = match *report.first()? {
            ID_DAP_MSD_OPEN => self.open(report.get(1).copied().unwrap_or(u8::MAX), sink),
            ID_DAP_MSD_CLOSE => self.close(sink),
            ID_DAP_MSD_WRITE => {
                let len = report.get(1).copied().unwrap_or(0) as usize;
                match report.get(2..2 + len) {
                    Some(data) => self.write(data, sink),
                    // A packet claiming more payload than it carries is malformed, not content.
                    None => status::INTERNAL,
                }
            }
            ID_DAP_SELECT_ERASE_MODE => {
                // Byte 1: 0x00 selects chip erase, nonzero selects sector erase.
                let sector = report.get(1).copied().unwrap_or(0) != 0;
                if sink.set_sector_erase(sector) {
                    status::DAP_OK
                } else {
                    status::DAP_ERROR
                }
            }
            _ => return None,
        };
        response[1] = status;
        Some(2)
    }

    fn open<S: TransferSink>(&mut self, stream_type: u8, sink: &mut S) -> u8 {
        // A stream must not be open already, and must name a format this firmware composed.
        if self.state != State::Closed {
            return status::INTERNAL;
        }
        let Some(fmt) = stream_format(stream_type).filter(|f| D::supports(*f)) else {
            return status::INTERNAL;
        };
        let Some(decoder) = D::make(fmt) else {
            return status::INTERNAL;
        };
        self.decoder = Some(decoder);
        self.fmt = fmt;
        self.check_head = true;
        self.head_len = 0;
        self.state = State::Open;
        // `sniffed` is false: the host named the format in the open request.
        sink.begin(fmt, false);
        status::SUCCESS
    }

    fn write<S: TransferSink>(&mut self, data: &[u8], sink: &mut S) -> u8 {
        if self.state != State::Open {
            return status::INTERNAL;
        }
        if self.check_head {
            // Detection needs four bytes, which a small chunk may not carry on its own.
            let take = (self.head.len() - self.head_len).min(data.len());
            self.head[self.head_len..self.head_len + take].copy_from_slice(&data[..take]);
            self.head_len += take;
            if self.head_len == self.head.len() {
                self.check_head = false;
                // The bytes must actually be the format the host opened.
                if D::detect(&self.head) != Some(self.fmt) {
                    self.state = State::Failed;
                    return status::DURING_TRANSFER;
                }
            }
        }
        let Self {
            decoder, scratch, ..
        } = self;
        let Some(decoder) = decoder.as_mut() else {
            self.state = State::Failed;
            return status::INTERNAL;
        };
        let decoded = {
            let mut ctx = SinkCtx { sink };
            decoder.input(data, scratch, &mut ctx)
        };
        if decoded.is_err() {
            self.state = State::Failed;
            return status::DURING_TRANSFER;
        }
        // The sink latches a flash failure without failing the fragment; surface it on this reply.
        if let Some(phase) = sink.latched_phase() {
            self.state = State::Failed;
            return phase_status(phase);
        }
        status::SUCCESS
    }

    fn close<S: TransferSink>(&mut self, sink: &mut S) -> u8 {
        // A stream must not be closed already.
        if self.state == State::Closed {
            return status::INTERNAL;
        }
        let mut status = status::SUCCESS;
        if self.state == State::Failed {
            status = status::DURING_TRANSFER;
        } else {
            let Self {
                decoder, scratch, ..
            } = self;
            if let Some(decoder) = decoder.as_mut() {
                let mut ctx = SinkCtx { sink };
                if decoder.finish(scratch, &mut ctx).is_err() {
                    status = status::DURING_TRANSFER;
                }
            }
        }
        let outcome = if status == status::SUCCESS {
            TransferOutcome::Done
        } else {
            TransferOutcome::DecodeError
        };
        sink.end(outcome);
        // A phase latched by the sink names the failure more precisely than the decode verdict.
        if let Some(phase) = sink.latched_phase() {
            status = phase_status(phase);
        }
        self.decoder = None;
        self.fmt = Format::Unknown;
        self.check_head = true;
        self.head_len = 0;
        self.state = State::Closed;
        status
    }
}

/// A [`VendorHandler`] owning the sink it streams into, for a firmware whose vendor surface is
/// that sink's only writer.
pub struct VendorFlash<D: DecoderSet, S: TransferSink> {
    stream: VendorFlashStream<D>,
    sink: S,
}

impl<D: DecoderSet, S: TransferSink> VendorFlash<D, S> {
    /// Wrap `sink` with a closed vendor stream.
    pub const fn new(sink: S) -> Self {
        Self {
            stream: VendorFlashStream::new(),
            sink,
        }
    }

    /// Borrow the sink.
    pub fn sink(&self) -> &S {
        &self.sink
    }

    /// Mutably borrow the sink.
    pub fn sink_mut(&mut self) -> &mut S {
        &mut self.sink
    }

    /// Borrow the protocol state.
    pub fn stream(&self) -> &VendorFlashStream<D> {
        &self.stream
    }
}

impl<D: DecoderSet, S: TransferSink> VendorHandler for VendorFlash<D, S> {
    fn handle(&mut self, report: &[u8], response: &mut [u8]) -> Option<usize> {
        self.stream.handle(report, response, &mut self.sink)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder_set::AllDecoders;
    use crate::flash_sink::FlashSink;
    use daplink_core::{FlashBackend, NoVendor};
    use daplink_decoder::encode::bin_to_uf2;
    use std::format;
    use std::string::String;
    use std::vec::Vec;

    const BASE: u32 = 0x0800_0000;
    const FAMILY: u32 = 0xE48B_FF56;

    struct MockBackend {
        ranges: &'static [(u32, u32)],
        calls: Vec<String>,
        fail_program: bool,
        sector_erase: Option<bool>,
        programmed: Vec<(u32, Vec<u8>)>,
    }

    impl MockBackend {
        fn new() -> Self {
            Self {
                ranges: &[(BASE, BASE + 0x1_0000)],
                calls: Vec::new(),
                fail_program: false,
                sector_erase: None,
                programmed: Vec::new(),
            }
        }
    }

    impl FlashBackend for MockBackend {
        fn begin(&mut self) -> Result<(), FlashPhase> {
            self.calls.push(String::from("begin"));
            Ok(())
        }
        fn program(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashPhase> {
            if self.fail_program {
                return Err(FlashPhase::Program);
            }
            self.programmed.push((addr, data.to_vec()));
            Ok(())
        }
        fn finish(&mut self, success: bool) -> Result<(), FlashPhase> {
            self.calls.push(format!("finish:{success}"));
            Ok(())
        }
        fn finish_with_reset(&mut self, success: bool, _reset: bool) -> Result<(), FlashPhase> {
            self.calls.push(format!("finish:{success}"));
            Ok(())
        }
        fn set_sector_erase(&mut self, sector: bool) -> Result<(), FlashPhase> {
            self.sector_erase = Some(sector);
            Ok(())
        }
        fn flash_ranges(&self) -> &[(u32, u32)] {
            self.ranges
        }
    }

    type Handler = VendorFlash<AllDecoders, FlashSink<MockBackend>>;

    fn handler() -> Handler {
        VendorFlash::new(FlashSink::new(MockBackend::new()))
    }

    /// Run one command and return `(response length, status byte)`.
    fn cmd(h: &mut Handler, report: &[u8]) -> (usize, u8) {
        let mut resp = [0u8; 64];
        let len = h
            .handle(report, &mut resp)
            .expect("the flash set owns this id");
        (len, resp[1])
    }

    fn open(h: &mut Handler) -> u8 {
        cmd(h, &[ID_DAP_MSD_OPEN, STREAM_TYPE_UF2]).1
    }

    fn write(h: &mut Handler, data: &[u8]) -> u8 {
        let mut report = Vec::with_capacity(2 + data.len());
        report.push(ID_DAP_MSD_WRITE);
        report.push(data.len() as u8);
        report.extend_from_slice(data);
        cmd(h, &report).1
    }

    fn close(h: &mut Handler) -> u8 {
        cmd(h, &[ID_DAP_MSD_CLOSE]).1
    }

    /// Feed `image` through the open stream in `chunk`-sized pieces, returning the first failure.
    fn stream_in(h: &mut Handler, image: &[u8], chunk: usize) -> u8 {
        for piece in image.chunks(chunk) {
            let s = write(h, piece);
            if s != status::SUCCESS {
                return s;
            }
        }
        status::SUCCESS
    }

    #[test]
    fn open_write_close_flashes_the_image() {
        let bin: Vec<u8> = (0..600u32).map(|i| i as u8).collect();
        let uf2 = bin_to_uf2(&bin, BASE, FAMILY);
        let mut h = handler();

        assert_eq!(open(&mut h), status::SUCCESS);
        assert_eq!(stream_in(&mut h, &uf2, 60), status::SUCCESS);
        assert_eq!(close(&mut h), status::SUCCESS);

        let be = h.sink().backend();
        assert_eq!(
            be.calls,
            [String::from("begin"), String::from("finish:true")]
        );
        // Three 256-byte payload blocks, each landing at its own address.
        let addrs: Vec<u32> = be.programmed.iter().map(|(a, _)| *a).collect();
        assert_eq!(addrs, [BASE, BASE + 256, BASE + 512]);
        assert_eq!(&be.programmed[0].1[..], &bin[..256]);
    }

    /// frozen: these command ids and status bytes are the wire contract host tools hard-code, so
    /// a renumber must fail here rather than at a host integration test.
    #[test]
    fn vendor_flash_ids_and_status_bytes_are_pinned() {
        assert_eq!(ID_DAP_MSD_OPEN, 0x8A);
        assert_eq!(ID_DAP_MSD_CLOSE, 0x8B);
        assert_eq!(ID_DAP_MSD_WRITE, 0x8C);
        assert_eq!(ID_DAP_SELECT_ERASE_MODE, 0x8D);

        assert_eq!(status::SUCCESS, 0);
        assert_eq!(status::INTERNAL, 2);
        assert_eq!(status::DURING_TRANSFER, 3);
        assert_eq!(status::FILE_BOUNDS, 5);
        assert_eq!(status::INIT, 11);
        assert_eq!(status::UNINIT, 12);
        assert_eq!(status::ERASE_SECTOR, 15);
        assert_eq!(status::WRITE, 17);
        assert_eq!(status::WRITE_VERIFY, 18);
        assert_eq!(status::INCOMPATIBLE_IMAGE, 29);
        assert_eq!(status::DAP_OK, 0x00);
        assert_eq!(status::DAP_ERROR, 0xFF);

        // (request bytes, expected reply status byte), run against the real handler.
        let cases: &[(&[u8], u8)] = &[
            (&[ID_DAP_MSD_OPEN, STREAM_TYPE_UF2], status::SUCCESS),
            (&[ID_DAP_MSD_OPEN, STREAM_TYPE_BIN], status::INTERNAL),
            (&[ID_DAP_MSD_CLOSE], status::INTERNAL), // closing an already-closed stream
            (&[ID_DAP_SELECT_ERASE_MODE, 0x00], status::DAP_OK),
            (&[ID_DAP_SELECT_ERASE_MODE, 0x01], status::DAP_OK),
        ];
        for &(report, expect) in cases {
            let mut h = handler();
            let (len, byte) = cmd(&mut h, report);
            assert_eq!(len, 2, "{report:?}");
            assert_eq!(byte, expect, "{report:?}");
        }
    }

    /// Every reply in the set is exactly two bytes: the echoed id plus a status.
    #[test]
    fn replies_are_two_bytes() {
        let mut h = handler();
        assert_eq!(cmd(&mut h, &[ID_DAP_MSD_OPEN, STREAM_TYPE_UF2]).0, 2);
        assert_eq!(cmd(&mut h, &[ID_DAP_MSD_WRITE, 0]).0, 2);
        assert_eq!(cmd(&mut h, &[ID_DAP_MSD_CLOSE]).0, 2);
        assert_eq!(cmd(&mut h, &[ID_DAP_SELECT_ERASE_MODE, 0]).0, 2);
    }

    /// Chunks that straddle uf2 block edges reassemble through the decoder's scratchpad.
    #[test]
    fn chunk_boundaries_straddle_block_edges() {
        let bin: Vec<u8> = (0..1024u32).map(|i| (i * 3) as u8).collect();
        let uf2 = bin_to_uf2(&bin, BASE, FAMILY);
        for chunk in [1usize, 7, 63, 200, 255] {
            let mut h = handler();
            assert_eq!(open(&mut h), status::SUCCESS);
            assert_eq!(stream_in(&mut h, &uf2, chunk), status::SUCCESS, "{chunk}");
            assert_eq!(close(&mut h), status::SUCCESS, "{chunk}");
            let be = h.sink().backend();
            let flat: Vec<u8> = be.programmed.iter().flat_map(|(_, d)| d.clone()).collect();
            assert_eq!(&flat[..bin.len()], &bin[..], "chunk {chunk}");
        }
    }

    /// A stream stopping mid-block leaves a remainder the close must reject, not flash.
    #[test]
    fn close_with_remainder_reports_transfer_error() {
        let bin: Vec<u8> = (0..512u32).map(|i| i as u8).collect();
        let uf2 = bin_to_uf2(&bin, BASE, FAMILY);
        let mut h = handler();
        assert_eq!(open(&mut h), status::SUCCESS);
        // 700 bytes: one whole block plus 188 bytes of the next.
        assert_eq!(stream_in(&mut h, &uf2[..700], 100), status::SUCCESS);
        assert_eq!(close(&mut h), status::DURING_TRANSFER);
        let be = h.sink().backend();
        assert_eq!(be.programmed.len(), 1, "only the whole block reached flash");
        assert_eq!(be.calls.last().map(String::as_str), Some("finish:false"));
    }

    /// An uncomposed stream type is refused at open, as is content of another format.
    #[test]
    fn unknown_formats_are_rejected() {
        let mut h = handler();
        assert_eq!(
            cmd(&mut h, &[ID_DAP_MSD_OPEN, STREAM_TYPE_BIN]).1,
            status::INTERNAL
        );
        assert_eq!(
            cmd(&mut h, &[ID_DAP_MSD_OPEN, STREAM_TYPE_HEX]).1,
            status::INTERNAL
        );
        assert_eq!(cmd(&mut h, &[ID_DAP_MSD_OPEN, 99]).1, status::INTERNAL);

        assert_eq!(open(&mut h), status::SUCCESS);
        assert_eq!(write(&mut h, &[0xA5; 64]), status::DURING_TRANSFER);
        assert_eq!(close(&mut h), status::DURING_TRANSFER);
        assert!(h.sink().backend().programmed.is_empty());
    }

    #[test]
    fn erase_mode_toggles_the_backend() {
        let mut h = handler();
        assert_eq!(
            cmd(&mut h, &[ID_DAP_SELECT_ERASE_MODE, 0x00]).1,
            status::DAP_OK
        );
        assert_eq!(h.sink().backend().sector_erase, Some(false));
        assert_eq!(
            cmd(&mut h, &[ID_DAP_SELECT_ERASE_MODE, 0x01]).1,
            status::DAP_OK
        );
        assert_eq!(h.sink().backend().sector_erase, Some(true));
    }

    /// A sink whose backend has no erase-mode control answers `DAP_ERROR` instead of silently OK.
    #[test]
    fn erase_mode_without_backend_support_reports_dap_error() {
        struct Bare(&'static [(u32, u32)]);
        impl FlashBackend for Bare {
            fn begin(&mut self) -> Result<(), FlashPhase> {
                Ok(())
            }
            fn program(&mut self, _addr: u32, _data: &[u8]) -> Result<(), FlashPhase> {
                Ok(())
            }
            fn finish(&mut self, _success: bool) -> Result<(), FlashPhase> {
                Ok(())
            }
            fn finish_with_reset(&mut self, _s: bool, _r: bool) -> Result<(), FlashPhase> {
                Ok(())
            }
            fn flash_ranges(&self) -> &[(u32, u32)] {
                self.0
            }
        }
        let mut h: VendorFlash<AllDecoders, FlashSink<Bare>> =
            VendorFlash::new(FlashSink::new(Bare(&[(BASE, BASE + 0x1_0000)])));
        let mut resp = [0u8; 64];
        h.handle(&[ID_DAP_SELECT_ERASE_MODE, 0x01], &mut resp);
        assert_eq!(resp[1], status::DAP_ERROR);
    }

    /// State rules mirror the C stream: no reopen while open, no close while closed.
    #[test]
    fn state_rules_match_the_c_stream() {
        let mut h = handler();
        assert_eq!(close(&mut h), status::INTERNAL);
        assert_eq!(open(&mut h), status::SUCCESS);
        assert_eq!(open(&mut h), status::INTERNAL);
        assert_eq!(close(&mut h), status::SUCCESS);
        assert_eq!(write(&mut h, &[0u8; 4]), status::INTERNAL);
    }

    /// A failed stream is recovered by closing it: the next open flashes cleanly.
    #[test]
    fn abort_then_reopen_flashes() {
        let bin: Vec<u8> = (0..256u32).map(|i| i as u8).collect();
        let uf2 = bin_to_uf2(&bin, BASE, FAMILY);
        let mut h = handler();

        assert_eq!(open(&mut h), status::SUCCESS);
        assert_eq!(write(&mut h, &[0x00; 32]), status::DURING_TRANSFER);
        // Further writes on a failed stream are refused until it is closed.
        assert_eq!(write(&mut h, &uf2[..64]), status::INTERNAL);
        assert_eq!(close(&mut h), status::DURING_TRANSFER);

        assert_eq!(open(&mut h), status::SUCCESS);
        assert_eq!(stream_in(&mut h, &uf2, 128), status::SUCCESS);
        assert_eq!(close(&mut h), status::SUCCESS);
        assert_eq!(h.sink().backend().programmed.len(), 1);
    }

    /// A flash failure latched mid-stream surfaces on the write that caused it, not only at close.
    #[test]
    fn flash_failure_surfaces_on_the_write() {
        let bin: Vec<u8> = (0..512u32).map(|i| i as u8).collect();
        let uf2 = bin_to_uf2(&bin, BASE, FAMILY);
        let mut h = VendorFlash::new(FlashSink::new(MockBackend {
            fail_program: true,
            ..MockBackend::new()
        }));
        assert_eq!(open(&mut h), status::SUCCESS);
        assert_eq!(stream_in(&mut h, &uf2, 200), status::WRITE);
        assert_eq!(close(&mut h), status::WRITE);
    }

    /// An out-of-range image is reported as a bounds failure, and never reaches the backend.
    #[test]
    fn out_of_range_image_reports_file_bounds() {
        let uf2 = bin_to_uf2(&[0xAA; 16], 0x2000_0000, FAMILY);
        let mut h = handler();
        assert_eq!(open(&mut h), status::SUCCESS);
        assert_eq!(stream_in(&mut h, &uf2, 200), status::FILE_BOUNDS);
        assert!(h.sink().backend().programmed.is_empty());
    }

    /// Ids outside the flash set are declined; the no-vendor handler declines everything.
    #[test]
    fn foreign_ids_and_no_vendor_fall_through() {
        let mut h = handler();
        let mut resp = [0u8; 64];
        for id in [0x80u8, 0x89, 0x8E, 0x9F] {
            assert_eq!(h.handle(&[id, 0x00], &mut resp), None, "id {id:#x}");
        }
        // `NoVendor` compiles the routing branch out entirely.
        const _: () = assert!(!NoVendor::HANDLES);
        assert_eq!(NoVendor.handle(&[ID_DAP_MSD_OPEN], &mut resp), None);
    }
}
