//! CMSIS-DAP v2 probe for original ST-Link/V2 hardware on RTIC: SWD+JTAG and SWO, no VCOM.

#![no_std]
#![no_main]

// Panic policy: halt, so the probe stays inspectable over its own SWD port.
use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use daplink_rs::daplink_core::{
        product_string, DapConfig, DapQueue, DAP_FW_VERSION, DAP_PACKET_SIZE,
    };
    use daplink_rs::{BitbangDap, NoBootloader, RebootVendor, StatusLed};
    use daplink_stm32f1_hal::{probe_parts_stlinkv2, DynCycleDelay, IoPin, LedPin, Usart1Swo};
    use daplink_usb_device::DapUsbNoVcom;
    use static_cell::ConstStaticCell;
    use stm32f1xx_hal::usb::{Peripheral, UsbBus};

    /// The USB-FS bus this firmware's device is built on.
    type Bus = UsbBus<Peripheral>;

    /// The PA9 activity LED's concrete `DapLeds` type: active-low (the pin sinks to light it).
    type StatusLedType = StatusLed<LedPin>;

    /// The reboot-to-bootloader vendor command, honestly declined: the F103's ROM bootloader is
    /// USART-only, so there is no software-reachable USB update path to reboot into.
    type Vendor = RebootVendor<NoBootloader>;

    /// The bit-bang SWD+JTAG engine, with USART1-RX SWO capture in its SWO slot.
    type Engine = BitbangDap<'static, IoPin, DynCycleDelay, Usart1Swo, Vendor, StatusLedType>;

    /// Core cycles of fixed bit-cell work subtracted from every requested half-period delay;
    /// swept for this pin path at 72 MHz, so re-sweep after an `opt-level` change.
    const SWD_FIXED_WORK_CYCLES: u32 = 8;

    /// Floor on the residual spin, in core cycles; zero lets a saturated request run flat out.
    const MIN_HALF_PERIOD_CYCLES: u32 = 0;

    /// Pipelined CMSIS-DAP command-queue depth, also the advertised `DAP_PACKET_COUNT`.
    const DAP_QUEUE_DEPTH: usize = 4;

    /// Per-slot byte capacity of the command queue: one full-speed DAP v2 bulk packet.
    const DAP_QUEUE_SLOT: usize = DAP_PACKET_SIZE as usize;

    /// This image's command queue: the depth the host is told about and the storage behind it.
    type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

    /// The `DAP_Info(MaxPacketCount)` this image advertises; fed to the engine below.
    const DAP_PACKET_COUNT: u8 = 4;

    // Advertised count, ring depth and slot size are one fact; widening one alone fails the build.
    const _: () = {
        assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
        assert!(Queue::new().packet_capacity() == DAP_PACKET_SIZE as usize);
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb: DapUsbNoVcom<'static, Bus>,
        dap: Engine,
        /// Caller-owned command queue, filled and drained by the USB task in the same pass.
        dap_queue: &'static mut Queue,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let delay = DynCycleDelay::with_fixed_work(
            daplink_stm32f1_hal::clocks::CPU_HZ,
            SWD_FIXED_WORK_CYCLES,
            MIN_HALF_PERIOD_CYCLES,
        );
        let parts = probe_parts_stlinkv2(cx.device, delay);
        let cfg = DapConfig::daplink(parts.serial).with_product(product_string!("stlinkv2"));

        let led = StatusLed::new_active_low(parts.led);
        let dap: Engine = BitbangDap::with_vendor(
            parts.deps,
            led,
            delay,
            parts.swo,
            DAP_FW_VERSION,
            cfg.max_packet_size,
            RebootVendor::new(NoBootloader),
        )
        .with_packet_count(DAP_PACKET_COUNT);

        let usb = DapUsbNoVcom::new(parts.usb_alloc, &cfg);

        // Caller-owned queue storage; `DAP_PACKET_COUNT` above names the same depth.
        static DAP_QUEUE: ConstStaticCell<Queue> = ConstStaticCell::new(Queue::new());
        let dap_queue = DAP_QUEUE.take();

        (
            Shared {},
            Local {
                usb,
                dap,
                dap_queue,
            },
        )
    }

    /// Idle: everything is serviced from the USB interrupt, SWO capture included.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    /// USB interrupt: pump the pipelined CMSIS-DAP queue.
    #[task(binds = USB_LP_CAN_RX0, local = [usb, dap, dap_queue])]
    fn on_usb(cx: on_usb::Context) {
        let usb = cx.local.usb;
        let dap = cx.local.dap;
        let dap_queue = &mut **cx.local.dap_queue;
        if usb.poll() {
            if usb.session_ended() {
                // Host gone: drop the dead session's queue and release the wire.
                dap_queue.reset();
                dap.suspend();
            }

            // Skip the OUT endpoint while the ring is full, so it NAKs the host into pacing itself.
            let mut buf = [0u8; DAP_QUEUE_SLOT];
            if !dap_queue.requests_full() {
                if let Some(n) = usb.read_command(&mut buf) {
                    if dap_queue.submit(&buf[..n]).is_err() {
                        debug_assert!(false, "request refused by a checked-non-full ring");
                    }
                }
            }

            // Run queued commands oldest first, never dequeuing with nowhere to put the response.
            let mut resp = [0u8; DAP_QUEUE_SLOT];
            let mut executed = 0;
            while executed < DAP_QUEUE_DEPTH {
                if dap_queue.responses_full() {
                    break;
                }
                let Some(n) = dap_queue.take_request(&mut buf) else {
                    break;
                };
                let len = dap.process_command(&buf[..n], &mut resp);
                if len > 0 && dap_queue.post_response(&resp[..len]).is_err() {
                    debug_assert!(false, "response ring full after a reserved slot");
                }
                executed += 1;
            }

            // Send whatever is ready, stopping at the first busy-endpoint refusal.
            while let Some(n) = dap_queue.peek_response(&mut resp) {
                if !usb.write_response(&resp[..n]) {
                    break;
                }
                dap_queue.discard_response();
            }
        }
    }
}
