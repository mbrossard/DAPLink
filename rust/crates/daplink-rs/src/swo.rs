//! Runtime-neutral SWO trace capture: a dap-rs `Swo` slot over a byte source and a capture backend.

use dap_rs::swo::{Swo, SwoControl, SwoMode, SwoStatus, SwoSupport, SwoTransport};

use crate::ring::ByteConsumer;

/// Where [`RingSwo`] takes captured bytes from: a [`ByteConsumer`] for an interrupt-fed capture,
/// or a platform's DMA ring for a controller-fed one.
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

/// Per-platform SWO capture mechanism: baud programming and capture arm/stop. The session
/// choreography and dap-rs surface live in [`RingSwo`]; a backend supplies only the raw peripheral.
pub trait SwoBackend {
    /// Reprogram the capture baud; returns the realized baud (0 = failed). Called only with
    /// `baud > 0` -- [`RingSwo`] handles the zero guard.
    fn set_baudrate(&mut self, baud: u32) -> u32;
    /// Halt capture and drop every byte in flight (hardware FIFO included), so nothing that
    /// predates the session survives it. Leaves capture stopped until [`arm`](SwoBackend::arm).
    fn prepare(&mut self);
    /// Start capture. Called after [`prepare`](SwoBackend::prepare) and a source clear, so a
    /// backend may re-base its transfer here.
    fn arm(&mut self);
    /// Halt capture, leaving whatever the source already holds readable.
    fn stop(&mut self);
}

/// dap-rs [`Swo`] slot over a ring-fed capture backend: UART-only, non-streaming, polled through
/// `DAP_SWO_Data`. Wraps an [`SwoBackend`] mechanism plus the [`SwoSource`] it fills.
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
        if baudrate == 0 {
            return 0;
        }
        let realized = self.hw.set_baudrate(baudrate);
        self.baud_set = realized != 0;
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
