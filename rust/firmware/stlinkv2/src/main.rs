//! CMSIS-DAP v2 probe for original ST-Link/V2 hardware on RTIC: SWD+JTAG and SWO, no VCOM.

#![no_std]
#![no_main]

// Panic policy: halt, so the probe stays inspectable over its own SWD port.
use panic_halt as _;

#[cfg(feature = "accel")]
mod spi_swd;

#[cfg(all(feature = "accel", feature = "conntest"))]
compile_error!("accel and conntest reserve incompatible probe-pad compositions");

#[cfg(feature = "conntest")]
mod conntest;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use cortex_m::peripheral::NVIC;
    use daplink_rs::daplink_core::{
        product_string, DapConfig, DapQueue, EndpointSize, PacketSize, DAP_FW_VERSION,
        DAP_PACKET_SIZE,
    };
    #[cfg(all(not(feature = "conntest"), not(feature = "accel")))]
    use daplink_rs::BitbangDap as SelectedDap;
    use daplink_rs::{NoBootloader, RebootVendor, StatusLed};
    #[cfg(feature = "conntest")]
    use daplink_rs::{NoSwo as SelectedSwo, SwdOnlyDap as SelectedDap};
    #[cfg(feature = "conntest")]
    use daplink_stm32f1_hal::probe_parts_stlinkv2_swd as probe_parts_stlinkv2;
    #[cfg(not(feature = "conntest"))]
    use daplink_stm32f1_hal::{probe_parts_stlinkv2, Usart1Swo as SelectedSwo};
    use daplink_stm32f1_hal::{DynCycleDelay, IoPin, LedPin};
    #[cfg(feature = "accel")]
    use daplink_stm32f1_spi_swd::SpiSwdJtagDap as SelectedDap;
    use daplink_usb_device::{dap_pass, DapPassHooks, DapUsbNoVcom};
    use static_cell::ConstStaticCell;
    use stm32f1xx_hal::pac::Interrupt;
    use stm32f1xx_hal::usb::{Peripheral, UsbBus};

    type Bus = UsbBus<Peripheral>;

    /// The PA9 activity LED, active-low.
    type StatusLedType = StatusLed<LedPin>;

    /// The USART-only ROM bootloader offers no USB update path.
    type Vendor = RebootVendor<NoBootloader>;

    #[cfg(not(feature = "accel"))]
    type Engine = SelectedDap<'static, IoPin, DynCycleDelay, SelectedSwo, Vendor, StatusLedType>;

    #[cfg(feature = "accel")]
    type Engine = SelectedDap<
        'static,
        crate::spi_swd::Board,
        IoPin,
        DynCycleDelay,
        crate::spi_swd::F1JtagPacing,
        SelectedSwo,
        Vendor,
        StatusLedType,
    >;

    #[cfg(feature = "conntest")]
    type Scanner = crate::conntest::Control;
    #[cfg(not(feature = "conntest"))]
    type Scanner = ();

    /// No fixed work: a write's ACK-to-turnaround cell is two halves of bare store and spin.
    const SWD_FIXED_WORK_CYCLES: u32 = 0;

    /// Floor on the residual spin, in core cycles; zero runs flat out.
    const MIN_HALF_PERIOD_CYCLES: u32 = 0;

    /// Command-queue depth, also the advertised `DAP_PACKET_COUNT`.
    const DAP_QUEUE_DEPTH: usize = 8;

    /// Maximum commands executed in one USB interrupt activation.
    const DAP_COMMAND_BUDGET: usize = 1;

    const DAP_QUEUE_SLOT: usize = DAP_PACKET_SIZE as usize;

    type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

    const DAP_PACKET: PacketSize = daplink_usb_device::single_transfer_packet(
        Queue::packet_size::<DAP_PACKET_SIZE>(),
        DAP_ENDPOINT,
    );

    const DAP_ENDPOINT: EndpointSize = Queue::endpoint_size::<DAP_PACKET_SIZE>();

    const DAP_PACKET_COUNT: u8 = DAP_QUEUE_DEPTH as u8;

    // Advertised count, ring depth and slot size are one fact.
    const _: () = {
        assert!(DAP_QUEUE_DEPTH <= u8::MAX as usize);
        assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
        assert!(Queue::new().packet_capacity() == DAP_PACKET_SIZE as usize);
        assert!(DAP_COMMAND_BUDGET > 0 && DAP_COMMAND_BUDGET <= DAP_QUEUE_DEPTH);
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb: DapUsbNoVcom<'static, Bus>,
        dap: Engine,
        dap_queue: &'static mut Queue,
        scanner: Scanner,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let delay = DynCycleDelay::with_fixed_work(
            daplink_stm32f1_hal::clocks::CPU_HZ,
            SWD_FIXED_WORK_CYCLES,
            MIN_HALF_PERIOD_CYCLES,
        );
        let parts = probe_parts_stlinkv2(cx.device, delay);
        #[cfg(not(feature = "accel"))]
        let deps = parts.deps;
        #[cfg(feature = "accel")]
        let deps = crate::spi_swd::accel_deps(delay);
        #[cfg(not(feature = "conntest"))]
        let product = product_string!("stlinkv2");
        #[cfg(feature = "conntest")]
        let product = product_string!("stlinkv2-conntest");
        let cfg = DapConfig::daplink(parts.serial, DAP_ENDPOINT).with_product(product);
        let identity = daplink_rs::daplink_core::Identity {
            vendor: cfg.manufacturer,
            product: cfg.product,
            serial: cfg.serial,
            board_vendor: "daplink-rs",
            board_name: "stlinkv2",
        };

        let led = StatusLed::new_active_low(parts.led);
        let dap: Engine = SelectedDap::with_vendor(
            deps,
            led,
            delay,
            parts.swo,
            DAP_FW_VERSION,
            DAP_PACKET,
            RebootVendor::new(NoBootloader),
        )
        .with_packet_count(DAP_PACKET_COUNT)
        .with_identity(identity);

        let usb = DapUsbNoVcom::new(parts.usb_alloc, &cfg)
            .with_endpoint_repair(&daplink_stm32f1_hal::ENDPOINT_REPAIR);

        static DAP_QUEUE: ConstStaticCell<Queue> = ConstStaticCell::new(Queue::new());
        let dap_queue = DAP_QUEUE.take();
        #[cfg(feature = "conntest")]
        let scanner = crate::conntest::Control::new(cx.core.DCB, cx.core.DWT);
        #[cfg(not(feature = "conntest"))]
        let scanner = ();

        (
            Shared {},
            Local {
                usb,
                dap,
                dap_queue,
                scanner,
            },
        )
    }

    #[idle(local = [scanner])]
    fn idle(cx: idle::Context) -> ! {
        let _ = &cx.local.scanner;
        loop {
            #[cfg(feature = "conntest")]
            cx.local.scanner.poll();
            #[cfg(not(feature = "conntest"))]
            cortex_m::asm::wfi();
        }
    }

    struct Pass<'a> {
        dap: &'a mut Engine,
    }

    impl DapPassHooks for Pass<'_> {
        // Keep the engine outside the USB pass to fit the 64 KiB flash.
        #[inline(never)]
        fn execute(&mut self, request: &[u8], response: &mut [u8]) -> usize {
            self.dap.process_command(request, response)
        }

        fn session_reset(&mut self) {
            self.dap.suspend();
        }
    }

    #[task(binds = USB_LP_CAN_RX0, local = [usb, dap, dap_queue])]
    fn on_usb(cx: on_usb::Context) {
        let usb = cx.local.usb;
        let dap_queue = &mut **cx.local.dap_queue;
        let polled = usb.poll();
        let mut hooks = Pass { dap: cx.local.dap };
        let dap_work_remains = dap_pass(dap_queue, usb, &mut hooks, polled, DAP_COMMAND_BUDGET);
        if dap_work_remains {
            NVIC::pend(Interrupt::USB_LP_CAN_RX0);
        }
    }
}
