//! Runtime-neutral SWO trace capture: a dap-rs `Swo` slot over a byte source and a capture backend.

use dap_rs::swo::{Swo, SwoControl, SwoMode, SwoStatus, SwoSupport, SwoTransport};

use crate::ring::ByteConsumer;

/// Where [`RingSwo`] takes captured bytes from: a [`ByteConsumer`] or a platform DMA ring.
pub trait SwoSource {
    /// Bytes buffered right now.
    fn len(&self) -> usize;
    /// `true` if no bytes are buffered.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Bytes the source can hold; reported to the host as the SWO buffer size.
    fn capacity(&self) -> usize;
    /// Copy out up to `buf.len()` bytes, returning the count. Never blocks.
    fn drain(&mut self, buf: &mut [u8]) -> usize;
    /// Discard everything buffered and clear the overrun flag.
    fn clear(&mut self);
    /// Read and clear the overrun flag.
    fn take_overrun(&mut self) -> bool;
}

impl<const N: usize> SwoSource for ByteConsumer<'_, N> {
    fn len(&self) -> usize {
        ByteConsumer::len(self)
    }

    fn capacity(&self) -> usize {
        // The ring keeps one slot empty to tell full from empty, so it buffers N - 1 bytes.
        N - 1
    }

    fn drain(&mut self, buf: &mut [u8]) -> usize {
        ByteConsumer::drain(self, buf)
    }

    fn clear(&mut self) {
        ByteConsumer::clear(self)
    }

    fn take_overrun(&mut self) -> bool {
        ByteConsumer::take_overrun(self)
    }
}

/// Per-platform SWO capture: baud programming and capture arm/stop.
pub trait SwoBackend {
    /// Reprogram the capture baud, returning the realized baud (0 = failed); `baud` is never 0.
    fn set_baudrate(&mut self, baud: u32) -> u32;
    /// Halt capture and drop every byte in flight, leaving it stopped until [`arm`](SwoBackend::arm).
    fn prepare(&mut self);
    /// Start capture, after [`prepare`](SwoBackend::prepare) and a source clear.
    fn arm(&mut self);
    /// Halt capture, leaving whatever the source already holds readable.
    fn stop(&mut self);
}

/// dap-rs [`Swo`] slot over a ring-fed backend: UART-only, polled through `DAP_SWO_Data`.
pub struct RingSwo<HW: SwoBackend, SRC: SwoSource> {
    hw: HW,
    src: SRC,
    transport: SwoTransport,
    active: bool,
    /// Set once a nonzero baud is programmed; capture cannot start until then.
    baud_set: bool,
}

impl<HW: SwoBackend, SRC: SwoSource> RingSwo<HW, SRC> {
    /// Pair a capture mechanism with the source it fills; starts stopped.
    pub fn new(hw: HW, src: SRC) -> Self {
        Self {
            hw,
            src,
            transport: SwoTransport::None,
            active: false,
            baud_set: false,
        }
    }

    /// The byte source, for a platform that wires something onto it after bring-up.
    pub fn source_mut(&mut self) -> &mut SRC {
        &mut self.src
    }
}

impl<HW: SwoBackend, SRC: SwoSource> Swo for RingSwo<HW, SRC> {
    fn set_transport(&mut self, transport: SwoTransport) -> bool {
        match transport {
            // No trace endpoint: reject USB-endpoint streaming.
            SwoTransport::USBEndpoint => false,
            other => {
                self.transport = other;
                true
            }
        }
    }

    fn set_mode(&mut self, mode: SwoMode) -> bool {
        // UART is the supported capture mode; Off is a valid no-op; Manchester is unsupported.
        matches!(mode, SwoMode::UART | SwoMode::Off)
    }

    fn set_baudrate(&mut self, baudrate: u32) -> u32 {
        let realized = if baudrate == 0 {
            0
        } else {
            self.hw.set_baudrate(baudrate)
        };
        self.baud_set = realized != 0;
        // A rejected rate leaves capture stopped, so the host never sees it running unconfigured.
        if !self.baud_set && self.active {
            self.hw.stop();
            self.active = false;
        }
        realized
    }

    fn set_control(&mut self, control: SwoControl) -> bool {
        match control {
            SwoControl::Start => {
                // Refuse to arm an unconfigured UART: the host sets the baud before starting.
                if !self.baud_set {
                    return false;
                }
                self.hw.prepare();
                self.src.clear();
                self.hw.arm();
                self.active = true;
            }
            SwoControl::Stop => {
                self.hw.stop();
                self.active = false;
            }
        }
        true
    }

    fn polling_data(&mut self, buf: &mut [u8]) -> u32 {
        self.src.drain(buf) as u32
    }

    fn streaming_data(&mut self) {}

    fn is_active(&self) -> bool {
        self.active
    }

    fn bytes_available(&self) -> u32 {
        self.src.len() as u32
    }

    fn buffer_size(&self) -> u32 {
        self.src.capacity() as u32
    }

    fn support(&self) -> SwoSupport {
        SwoSupport {
            uart: true,
            manchester: false,
            streaming: false,
        }
    }

    fn status(&mut self) -> SwoStatus {
        SwoStatus {
            active: self.active,
            trace_error: false,
            trace_overrun: self.src.take_overrun(),
            bytes_available: self.src.len() as u32,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::ring::ByteRing;
    use std::vec::Vec;

    /// Call-log mock backend for the [`RingSwo`] choreography.
    struct MockHw {
        log: std::vec::Vec<&'static str>,
    }
    impl SwoBackend for MockHw {
        fn set_baudrate(&mut self, baud: u32) -> u32 {
            self.log.push("baud");
            baud / 2
        }
        fn prepare(&mut self) {
            self.log.push("prepare");
        }
        fn arm(&mut self) {
            self.log.push("arm");
        }
        fn stop(&mut self) {
            self.log.push("stop");
        }
    }

    #[test]
    fn start_order_is_prepare_clear_arm_after_baud() {
        let mut ring: ByteRing<8> = ByteRing::new();
        let (mut p, cons) = ring.split();
        p.push(0xAA); // stale bytes from a previous session
        p.push(0xBB);
        let mut swo = RingSwo::new(MockHw { log: Vec::new() }, cons);
        assert!(
            !swo.set_control(SwoControl::Start),
            "cannot arm before baud"
        );
        assert_eq!(
            swo.set_baudrate(1_000_000),
            500_000,
            "realized value returned"
        );
        assert!(swo.set_control(SwoControl::Start));
        assert_eq!(swo.hw.log, ["baud", "prepare", "arm"]);
        assert_eq!(
            swo.bytes_available(),
            0,
            "ring cleared between prepare and arm"
        );
        assert!(swo.is_active());
        assert!(swo.set_control(SwoControl::Stop));
        assert!(!swo.is_active());
        assert_eq!(swo.hw.log.last(), Some(&"stop"));
    }

    #[test]
    fn polling_drains_in_order_and_status_reports_overrun_once() {
        let mut ring: ByteRing<8> = ByteRing::new();
        let (mut p, cons) = ring.split();
        for b in 0..9 {
            p.push(b); // the last two overrun the 7-byte ring
        }
        let mut swo = RingSwo::new(MockHw { log: Vec::new() }, cons);
        assert_eq!(swo.buffer_size(), 7);
        assert_eq!(swo.bytes_available(), 7);
        let mut buf = [0u8; 4];
        assert_eq!(swo.polling_data(&mut buf), 4);
        assert_eq!(buf, [0, 1, 2, 3]);
        assert_eq!(swo.bytes_available(), 3);
        let status = swo.status();
        assert!(status.trace_overrun);
        assert!(!status.active && !status.trace_error);
        assert_eq!(status.bytes_available, 3);
        assert!(!swo.status().trace_overrun, "cleared by the read");
        let mut rest = [0u8; 8];
        assert_eq!(swo.polling_data(&mut rest), 3);
        assert_eq!(rest[..3], [4, 5, 6]);
        assert_eq!(swo.polling_data(&mut rest), 0);
        assert_eq!(swo.bytes_available(), 0);
    }

    #[test]
    fn rejected_baud_stops_capture() {
        for rejected in [1, 0] {
            let mut ring: ByteRing<8> = ByteRing::new();
            let (_p, cons) = ring.split();
            let mut swo = RingSwo::new(MockHw { log: Vec::new() }, cons);
            assert_eq!(swo.set_baudrate(1_000_000), 500_000);
            assert!(swo.set_control(SwoControl::Start));
            assert_eq!(swo.set_baudrate(rejected), 0);
            assert!(!swo.is_active() && !swo.status().active, "baud {rejected}");
            assert_eq!(swo.hw.log.last(), Some(&"stop"), "baud {rejected}");
            assert!(!swo.set_control(SwoControl::Start), "no rate configured");
        }
    }

    #[test]
    fn baud_zero_rejected_and_usb_transport_refused() {
        let mut ring: ByteRing<8> = ByteRing::new();
        let (_p, cons) = ring.split();
        let mut swo = RingSwo::new(MockHw { log: Vec::new() }, cons);
        assert_eq!(swo.set_baudrate(0), 0);
        assert!(swo.hw.log.is_empty(), "hw untouched for baud 0");
        assert!(!swo.set_transport(SwoTransport::USBEndpoint));
        assert!(swo.set_transport(SwoTransport::DAPCommand));
        assert!(swo.set_mode(SwoMode::UART));
        assert!(!swo.set_mode(SwoMode::Manchester));
        assert!(swo.support().uart);
    }
}
