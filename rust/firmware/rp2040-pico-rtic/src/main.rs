//! CMSIS-DAP v2 probe for the Raspberry Pi Pico (RP2040) on RTIC.

#![no_std]
#![no_main]

#[cfg(not(feature = "overclock-200mhz"))]
const CLOCK: daplink_rp_hal::ClockProfile = daplink_rp_hal::ClockProfile::DEFAULT;
#[cfg(feature = "overclock-200mhz")]
const CLOCK: daplink_rp_hal::ClockProfile = daplink_rp_hal::ClockProfile::MHZ_200;
const CPU_HZ: u32 = CLOCK.cpu_hz();
#[cfg(feature = "accel")]
const _: daplink_rp_pio::PioClock = daplink_rp_pio::PioClock::expect(CPU_HZ);
// A flash build latches the fault into `RESET_STAMP` and resets; every other build halts.
#[cfg(all(feature = "flash", not(feature = "defmt")))]
daplink_vfs::fault_reset_handlers!();
#[cfg(all(not(feature = "flash"), not(feature = "defmt")))]
use panic_halt as _;
// `defmt` halts in every build, so the RTT-attached probe sees the panic message.
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

// rp2040-hal leaves __pre_init unclaimed, so the plain (non-wrapping) form defines it.
daplink_rs::ram_text_pre_init!();

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[cfg(feature = "flash")]
static ELAPSED_MS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

#[cfg(feature = "flash")]
daplink_vfs::retained_stamp!();

#[cfg(feature = "flash")]
#[cortex_m_rt::exception]
fn SysTick() {
    use core::sync::atomic::Ordering;
    // Sole writer, so a plain load/store increment is race-free.
    let next = ELAPSED_MS.load(Ordering::Relaxed).wrapping_add(1);
    ELAPSED_MS.store(next, Ordering::Relaxed);
    cortex_m::peripheral::NVIC::pend(rp2040_hal::pac::Interrupt::USBCTRL_IRQ);
}

#[rtic::app(device = rp2040_hal::pac)]
mod app {
    use cortex_m::peripheral::NVIC;
    #[cfg(not(feature = "accel"))]
    use daplink_rp_hal::IoPin;
    use daplink_rp_hal::UsbBus;
    use daplink_rp_hal::{
        status_led, swo_drain, vcom_account_delivery, vcom_account_discard, vcom_account_drop,
        vcom_account_host_traffic, vcom_baud_state, vcom_counters, vcom_enable_tx_interrupt,
        vcom_irq_service, vcom_request_baud, StatusLedPin, SwoProducer, Uart0Swo, VcomIrq, VcomUsb,
    };
    use daplink_rs::daplink_core::{
        product_string, DapConfig, DapQueue, EndpointSize, PacketSize, DAP_PACKET_SIZE,
        DEFAULT_VCOM_BAUD,
    };
    use daplink_rs::{Activity, StatusLed, VcomBaudVendor, VcomStatsVendor};
    #[cfg(not(feature = "flash"))]
    use daplink_usb_device::{dap_pass, DapPassHooks, DapUsb};
    #[cfg(feature = "flash")]
    use daplink_usb_device::{dap_pass, DapPassHooks, DapUsbMsc as DapUsb};
    use rp2040_hal::pac::Interrupt;
    use static_cell::ConstStaticCell;
    type CycleDelay = daplink_rp_hal::CycleDelay<{ crate::CPU_HZ }>;

    #[cfg(not(feature = "flash"))]
    use daplink_rs::daplink_core::DAP_FW_VERSION;

    #[cfg(feature = "carrier-leds")]
    use daplink_rp_hal::{carrier_leds, CarrierBankLed};
    #[cfg(feature = "carrier-leds")]
    use rp2040_hal::gpio::bank0::{Gpio10, Gpio11, Gpio12, Gpio14};

    // `--no-default-features` bit-bangs; default `accel` adds PIO1 complete-transfer SWD and JTAG.
    #[cfg(not(feature = "accel"))]
    use daplink_rp_hal::probe_parts_pico;
    use daplink_rp_hal::RpUsbBoot;
    #[cfg(all(not(feature = "accel"), not(feature = "flash")))]
    use daplink_rs::BitbangDap;
    use daplink_rs::RebootVendor;

    #[cfg(feature = "accel")]
    use daplink_rp_hal::probe_parts_pico_accel;
    #[cfg(all(feature = "accel", not(feature = "flash")))]
    use daplink_rp_pio::PioTransferDap;
    #[cfg(feature = "accel")]
    use daplink_rp_pio::{
        pio_transfer_deps, PioJtagPins, PioSm, PioSwdPins, RpPioTransferEngine, RpPioTransferFence,
    };
    #[cfg(all(feature = "accel", feature = "flash"))]
    use daplink_rp_pio::{PioTransferDeps, PioTransferJtag, PioTransferSwd};
    #[cfg(any(feature = "accel", feature = "flash"))]
    use daplink_rs::ScanChainStorage;

    #[cfg(feature = "flash")]
    use cortex_m::peripheral::syst::SystClkSource;
    #[cfg(feature = "flash")]
    use daplink_rs::flash_mux::{mux_dap_engine, mux_flash_backend, MuxDapEngine, MuxFlashBackend};
    #[cfg(all(feature = "flash", feature = "accel"))]
    use daplink_rs::FencedFlashBackend;
    #[cfg(feature = "flash")]
    use daplink_rs::SwdMux;
    #[cfg(all(feature = "flash", not(feature = "accel")))]
    use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv};
    #[cfg(feature = "flash-nrf52833")]
    use daplink_targets::targets::nrf52833::NRF52833;
    #[cfg(feature = "flash-nrf52840")]
    use daplink_targets::targets::nrf52840::NRF52840;
    #[cfg(feature = "flash-rp2040")]
    use daplink_targets::targets::rp2040::RP2040;
    #[cfg(feature = "flash-stm32f401")]
    use daplink_targets::targets::stm32f401ce::STM32F401CE;
    #[cfg(feature = "flash")]
    use daplink_vfs::{
        AllDecoders, FlashSink, FlashVolume, StaticCell, VendorFlash, VolumeStorage,
    };

    #[cfg(feature = "flash-rp2040")]
    type PrimaryTarget = RP2040;
    #[cfg(feature = "flash-rp2040")]
    static PRIMARY: &PrimaryTarget = &RP2040;
    #[cfg(all(not(feature = "flash-rp2040"), feature = "flash-stm32f401"))]
    type PrimaryTarget = STM32F401CE;
    #[cfg(all(not(feature = "flash-rp2040"), feature = "flash-stm32f401"))]
    static PRIMARY: &PrimaryTarget = &STM32F401CE;
    #[cfg(all(
        feature = "flash-nrf52840",
        not(any(feature = "flash-rp2040", feature = "flash-stm32f401"))
    ))]
    type PrimaryTarget = NRF52840;
    #[cfg(all(
        feature = "flash-nrf52840",
        not(any(feature = "flash-rp2040", feature = "flash-stm32f401"))
    ))]
    static PRIMARY: &PrimaryTarget = &NRF52840;
    #[cfg(all(
        feature = "flash-nrf52833",
        not(any(
            feature = "flash-rp2040",
            feature = "flash-stm32f401",
            feature = "flash-nrf52840"
        ))
    ))]
    type PrimaryTarget = NRF52833;
    #[cfg(all(
        feature = "flash-nrf52833",
        not(any(
            feature = "flash-rp2040",
            feature = "flash-stm32f401",
            feature = "flash-nrf52840"
        ))
    ))]
    static PRIMARY: &PrimaryTarget = &NRF52833;

    #[cfg(feature = "flash")]
    daplink_targets::target_table! {
        umbrella = "flash",
        count = TARGET_COUNT,
        selector = Selector,
        scratch = SCRATCH,
        table = flash_table,
        buffer = flash_scratch,
        rows = [
            ("flash-rp2040", RP2040, daplink_targets::targets::rp2040::PROGRAM_BUFFER_SIZE),
            ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
            ("flash-nrf52840", NRF52840, daplink_targets::targets::nrf52840::PROGRAM_BUFFER_SIZE),
            ("flash-nrf52833", NRF52833, daplink_targets::targets::nrf52833::PROGRAM_BUFFER_SIZE),
        ],
    }

    /// The onboard status LED: GP25, active-high.
    type StatusLedType = StatusLed<StatusLedPin>;

    /// Carrier heartbeat LED (GP10).
    #[cfg(feature = "carrier-leds")]
    type HeartbeatLed = CarrierBankLed<Gpio10>;
    #[cfg(not(feature = "carrier-leds"))]
    type HeartbeatLed = ();

    /// Carrier DAP-activity LED (GP11).
    #[cfg(feature = "carrier-leds")]
    type DapLed = CarrierBankLed<Gpio11>;
    #[cfg(not(feature = "carrier-leds"))]
    type DapLed = ();

    /// Carrier flash-session LED (GP12).
    #[cfg(feature = "carrier-leds")]
    type FlashLed = CarrierBankLed<Gpio12>;
    #[cfg(not(feature = "carrier-leds"))]
    type FlashLed = ();

    /// Carrier VCOM-traffic LED (GP14).
    #[cfg(feature = "carrier-leds")]
    type VcomLed = CarrierBankLed<Gpio14>;
    #[cfg(not(feature = "carrier-leds"))]
    type VcomLed = ();

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

    const DAP_PACKET_COUNT: u8 = 8;

    // Advertised count, ring depth and slot size are one fact.
    const _: () = {
        assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
        assert!(Queue::new().packet_capacity() == DAP_PACKET_SIZE as usize);
        assert!(DAP_COMMAND_BUDGET > 0 && DAP_COMMAND_BUDGET <= DAP_QUEUE_DEPTH);
    };

    #[cfg(not(feature = "flash"))]
    type RebootOnly = (RebootVendor<RpUsbBoot>, VcomBaudVendor, VcomStatsVendor);

    #[cfg(all(not(feature = "accel"), not(feature = "flash")))]
    type Engine = BitbangDap<'static, IoPin, CycleDelay, Uart0Swo, RebootOnly, StatusLedType>;

    #[cfg(all(not(feature = "flash"), feature = "accel"))]
    type Engine = PioTransferDap<
        'static,
        PioSm,
        RpPioTransferEngine,
        CycleDelay,
        Uart0Swo,
        RebootOnly,
        StatusLedType,
        RpPioTransferFence,
    >;

    #[cfg(all(feature = "flash", not(feature = "accel")))]
    type MuxDeps = LateJtagDeps<IoPin, CycleDelay>;
    #[cfg(all(feature = "flash", not(feature = "accel")))]
    type MuxSwd = LateSwdDrv<IoPin, CycleDelay>;
    #[cfg(all(feature = "flash", not(feature = "accel")))]
    type MuxJtag = LateJtag<IoPin, CycleDelay>;

    #[cfg(all(feature = "flash", feature = "accel"))]
    type MuxDeps = PioTransferDeps<PioSm, RpPioTransferEngine>;
    #[cfg(all(feature = "flash", feature = "accel"))]
    type MuxSwd = PioTransferSwd<PioSm, RpPioTransferEngine>;
    #[cfg(all(feature = "flash", feature = "accel"))]
    type MuxJtag = PioTransferJtag<PioSm, RpPioTransferEngine>;

    #[cfg(feature = "flash")]
    type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
    #[cfg(feature = "flash")]
    type RawBackend =
        MuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, CycleDelay, PrimaryTarget, Selector>;
    #[cfg(all(feature = "flash", not(feature = "accel")))]
    type Backend = RawBackend;
    #[cfg(all(feature = "flash", feature = "accel"))]
    type Backend = FencedFlashBackend<RawBackend, &'static RpPioTransferFence>;
    #[cfg(feature = "flash")]
    type Sink = FlashSink<Backend>;
    #[cfg(feature = "flash")]
    type RebootBusy = &'static Mux;
    /// Reboot, declined with `STATUS_BUSY` during a live flash session.
    #[cfg(feature = "flash")]
    type Vendor = (
        RebootVendor<RpUsbBoot, RebootBusy>,
        VendorFlash<AllDecoders, Sink>,
        VcomBaudVendor,
        VcomStatsVendor,
    );
    #[cfg(all(feature = "flash", not(feature = "accel")))]
    type Engine = MuxDapEngine<
        'static,
        MuxDeps,
        MuxSwd,
        MuxJtag,
        CycleDelay,
        Uart0Swo,
        Vendor,
        StatusLedType,
    >;
    #[cfg(all(feature = "flash", feature = "accel"))]
    type Engine = MuxDapEngine<
        'static,
        MuxDeps,
        MuxSwd,
        MuxJtag,
        CycleDelay,
        Uart0Swo,
        Vendor,
        StatusLedType,
        &'static RpPioTransferFence,
    >;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb: DapUsb<'static, UsbBus>,
        dap: Engine,
        dap_queue: &'static mut Queue,
        vcom_usb: VcomUsb,
        /// The rate last passed to `vcom_request_baud`, which the UART may not have applied yet.
        vcom_baud: u32,
        #[cfg(feature = "defmt")]
        processed: u32,
        swo_producer: SwoProducer,
        vcom_irq: VcomIrq,
        #[cfg(feature = "flash")]
        vfs: &'static mut FlashVolume<Sink>,
        #[cfg(feature = "flash")]
        last_ms: u32,
        #[cfg(feature = "flash")]
        mux: &'static Mux,
        heartbeat_led: HeartbeatLed,
        dap_led: DapLed,
        flash_led: FlashLed,
        vcom_led: VcomLed,
    }

    #[cfg(not(feature = "accel"))]
    macro_rules! probe_deps {
        ($parts:expr) => {
            $parts.deps
        };
    }

    #[cfg(feature = "accel")]
    macro_rules! probe_deps {
        ($parts:expr) => {{
            static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
                ScanChainStorage::new();
            let pins = PioSwdPins {
                swclk: $parts.swclk,
                swdio_out: $parts.swdio,
                swdio_in: None,
                oen: None,
                nreset: Some($parts.nreset),
            };
            let jtag = PioJtagPins {
                tck: $parts.swclk,
                tms: $parts.swdio,
                tdi: $parts.tdi,
                tdo: $parts.tdo,
            };
            // SAFETY: bring-up reserved both PIO blocks and these pads; this is PIO1's sole owner.
            unsafe { pio_transfer_deps(pins, jtag, crate::CPU_HZ, &SCAN) }
        }};
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        #[cfg(feature = "flash")]
        let stamp = unsafe {
            // SAFETY: dedicated NOLOAD SRAM is readable at entry; no DMA or other core accesses it.
            daplink_vfs::stamp::boot_retained(
                core::ptr::addr_of_mut!(crate::__daplink_retained_stamp).cast(),
            )
        };
        #[cfg(all(feature = "flash", not(feature = "defmt")))]
        // SAFETY: only this core runs terminal, non-returning fault handlers.
        unsafe {
            daplink_vfs::stamp::arm_terminal_faults(stamp.fault)
        };

        #[cfg(feature = "defmt")]
        defmt::info!("rp2040-pico-rtic: boot");

        let delay = daplink_rp_hal::CycleDelay::<{ crate::CPU_HZ }>;
        #[cfg(not(feature = "accel"))]
        let parts = probe_parts_pico(cx.device, crate::CLOCK, delay);
        #[cfg(feature = "accel")]
        let parts = probe_parts_pico_accel(cx.device, crate::CLOCK);
        let cfg = DapConfig::daplink(parts.serial, DAP_ENDPOINT)
            .with_product(product_string!("rp2040-pico-rtic"));
        let identity = daplink_rs::daplink_core::Identity {
            vendor: cfg.manufacturer,
            product: cfg.product,
            serial: cfg.serial,
            board_vendor: "daplink-rs",
            board_name: "rp2040-pico-rtic",
        };
        #[cfg(feature = "defmt")]
        defmt::info!("rp2040-pico-rtic: serial {=str}", parts.serial);
        let status = status_led(parts.status_pin);
        #[cfg(feature = "carrier-leds")]
        let carrier = carrier_leds(parts.carrier_pins);
        #[cfg(feature = "carrier-leds")]
        let (heartbeat_led, dap_led, flash_led, vcom_led) =
            (carrier.heartbeat, carrier.dap, carrier.flash, carrier.vcom);
        #[cfg(not(feature = "carrier-leds"))]
        let (heartbeat_led, dap_led, flash_led, vcom_led): (
            HeartbeatLed,
            DapLed,
            FlashLed,
            VcomLed,
        ) = ((), (), (), ());

        #[cfg(all(not(feature = "accel"), not(feature = "flash")))]
        let dap: Engine = BitbangDap::with_vendor(
            probe_deps!(parts),
            status,
            delay,
            parts.swo,
            DAP_FW_VERSION,
            DAP_PACKET,
            (
                RebootVendor::new(RpUsbBoot),
                VcomBaudVendor::new(vcom_baud_state),
                VcomStatsVendor::new(vcom_counters),
            ),
        )
        .with_packet_count(DAP_PACKET_COUNT)
        .with_identity(identity);
        #[cfg(all(feature = "accel", not(feature = "flash")))]
        let dap: Engine = {
            let (deps, fence) = probe_deps!(parts);
            let engine = daplink_rs::DapProbe::with_vendor(
                deps,
                status,
                delay,
                parts.swo,
                DAP_FW_VERSION,
                DAP_PACKET,
                (
                    RebootVendor::new(RpUsbBoot),
                    VcomBaudVendor::new(vcom_baud_state),
                    VcomStatsVendor::new(vcom_counters),
                ),
            );
            engine
                .with_command_fence(fence)
                .with_packet_count(DAP_PACKET_COUNT)
                .with_identity(identity)
        };
        #[cfg(feature = "flash")]
        let (dap, vfs, mux) = {
            #[cfg(not(feature = "accel"))]
            let adapter = probe_deps!(parts);
            #[cfg(feature = "accel")]
            let (adapter, fence) = probe_deps!(parts);
            #[cfg(feature = "accel")]
            let fence: &'static RpPioTransferFence = {
                static FENCE: StaticCell<RpPioTransferFence> = StaticCell::new();
                FENCE.init(fence)
            };

            static MUX: StaticCell<Mux> = StaticCell::new();
            static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
                ScanChainStorage::new();
            static VENDOR_SCRATCH: ConstStaticCell<[u8; SCRATCH]> =
                ConstStaticCell::new([0; SCRATCH]);
            static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

            let mux: &'static Mux = MUX.init(SwdMux::new(adapter));
            let table = flash_table();
            let vendor_backend =
                mux_flash_backend(mux, delay, PRIMARY, table, VENDOR_SCRATCH.take());
            #[cfg(feature = "accel")]
            let vendor_backend = FencedFlashBackend::new(vendor_backend, fence);
            let dap = mux_dap_engine(
                mux,
                delay,
                parts.swo,
                DAP_PACKET,
                &MUX_SCAN,
                (
                    RebootVendor::with_busy_check(RpUsbBoot, mux),
                    VendorFlash::new(FlashSink::new(vendor_backend)),
                    VcomBaudVendor::new(vcom_baud_state),
                    VcomStatsVendor::new(vcom_counters),
                ),
                status,
            );
            #[cfg(feature = "accel")]
            let dap = dap.with_command_fence(fence);
            let dap = dap
                .with_packet_count(DAP_PACKET_COUNT)
                .with_identity(identity);
            let vfs_backend = mux_flash_backend(mux, delay, PRIMARY, table, flash_scratch());
            #[cfg(feature = "accel")]
            let vfs_backend = FencedFlashBackend::new(vfs_backend, fence);
            let vfs = daplink_vfs::assemble_info(
                &VOLUME,
                FlashSink::new(vfs_backend),
                daplink_vfs::DeviceInfo {
                    version: env!("CARGO_PKG_VERSION"),
                    serial: parts.serial,
                    reset_count: stamp.snapshot.reset_count(),
                },
                Some((stamp.snapshot, stamp.status, stamp.steady)),
                "DAPLINK",
            );

            // A 1 ms SysTick is the volume's only wall clock.
            let mut syst = cx.core.SYST;
            syst.set_clock_source(SystClkSource::Core);
            const { assert!(crate::CPU_HZ.is_multiple_of(1000)) };
            syst.set_reload(crate::CPU_HZ / 1000 - 1);
            syst.clear_current();
            syst.enable_interrupt();
            syst.enable_counter();

            (dap, vfs, mux)
        };

        let usb = DapUsb::new(parts.usb_alloc, &cfg);

        static DAP_QUEUE: ConstStaticCell<Queue> = ConstStaticCell::new(Queue::new());
        let dap_queue = DAP_QUEUE.take();

        let (vcom_irq, vcom_usb) = parts.vcom;

        (
            Shared {},
            Local {
                usb,
                dap,
                dap_queue,
                vcom_usb,
                vcom_baud: DEFAULT_VCOM_BAUD,
                #[cfg(feature = "defmt")]
                processed: 0,
                swo_producer: parts.swo_producer,
                vcom_irq,
                #[cfg(feature = "flash")]
                vfs,
                #[cfg(feature = "flash")]
                last_ms: 0,
                #[cfg(feature = "flash")]
                mux,
                heartbeat_led,
                dap_led,
                flash_led,
                vcom_led,
            },
        )
    }

    /// Idle: the ROM reboot entry must run from thread mode.
    #[idle(local = [heartbeat_led])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
            daplink_rp_hal::reboot_watch::<{ crate::CPU_HZ }>();
            cx.local.heartbeat_led.pulse();
        }
    }

    /// UART1 drains receive data before advancing queued transmission.
    #[task(binds = UART1_IRQ, priority = 2, local = [vcom_irq])]
    fn on_uart(cx: on_uart::Context) {
        vcom_irq_service::<{ crate::CPU_HZ }>(cx.local.vcom_irq);
        NVIC::pend(Interrupt::USBCTRL_IRQ);
    }

    /// UART0 SWO capture, masked by `UARTIMSC` until the host starts capture.
    #[task(binds = UART0_IRQ, priority = 2, local = [swo_producer])]
    fn on_swo(cx: on_swo::Context) {
        swo_drain(cx.local.swo_producer);
    }

    struct Pass<'a> {
        dap: &'a mut Engine,
        dap_led: &'a mut DapLed,
        #[cfg(feature = "defmt")]
        processed: &'a mut u32,
    }

    impl DapPassHooks for Pass<'_> {
        fn execute(&mut self, request: &[u8], response: &mut [u8]) -> usize {
            self.dap.process_command(request, response)
        }

        fn session_reset(&mut self) {
            #[cfg(feature = "defmt")]
            {
                defmt::info!("dap session: {=u32} commands processed", *self.processed);
                *self.processed = 0;
            }
            self.dap.suspend();
        }

        fn executed(&mut self, _len: usize) {
            #[cfg(feature = "defmt")]
            {
                *self.processed += 1;
            }
            self.dap_led.pulse();
        }
    }

    /// USB: DAP queue, mass storage and VCOM; sector work runs inline and gates the DAP engine.
    #[task(
        binds = USBCTRL_IRQ,
        local = [
            usb,
            dap,
            dap_queue,
            vcom_usb,
            vcom_baud,
            dap_led,
            flash_led,
            vcom_led,
            #[cfg(feature = "defmt")]
            processed,
            #[cfg(feature = "flash")]
            vfs,
            #[cfg(feature = "flash")]
            last_ms,
            #[cfg(feature = "flash")]
            mux,
        ],
    )]
    fn on_usb(cx: on_usb::Context) {
        let usb = cx.local.usb;
        let dap_queue = &mut **cx.local.dap_queue;
        let vcom = cx.local.vcom_usb;
        let vcom_baud = cx.local.vcom_baud;
        let polled = usb.poll();

        let requested = usb.vcom_line_coding_baud();
        if requested != 0 && requested != *vcom_baud {
            #[cfg(feature = "defmt")]
            defmt::info!(
                "vcom line coding: {=u32}->{=u32} staged={=usize} armed={=bool} ring={=usize}",
                *vcom_baud,
                requested,
                usb.vcom_staged(),
                usb.vcom_in_flight(),
                vcom.rx.len()
            );
            vcom_request_baud(requested);
            *vcom_baud = requested;
        }

        let mut vcom_buf = [0u8; 64];
        // A closed session's backlog is not the next opener's.
        if usb.vcom_take_close_edge() {
            let mut dropped = 0;
            loop {
                let n = vcom.rx.drain(&mut vcom_buf);
                dropped += n;
                if n < vcom_buf.len() {
                    break;
                }
            }
            vcom_account_drop(dropped);
        }
        let room = usb.vcom_write_capacity().min(vcom_buf.len());
        if room > 0 {
            let taken = vcom.rx.drain(&mut vcom_buf[..room]);
            if taken > 0 {
                let sent = usb.vcom_write(&vcom_buf[..taken]);
                vcom_account_delivery(taken, sent);
                cx.local.vcom_led.pulse();
            }
        }
        // Take bytes discarded on a port close or bus reset every pass; nothing else triggers it.
        vcom_account_discard(usb.vcom_take_discarded());

        let room = vcom.tx.remaining_capacity().min(vcom_buf.len());
        if room > 0 {
            let n = usb.vcom_read(&mut vcom_buf[..room]);
            if n > 0 {
                for &byte in &vcom_buf[..n] {
                    // `debug_assert!` drops its argument in release, so the push stays outside it.
                    let pushed = vcom.tx.push(byte);
                    debug_assert!(pushed);
                }
                vcom_account_host_traffic(n);
                vcom_enable_tx_interrupt();
                cx.local.vcom_led.pulse();
            }
        }

        let mut hooks = Pass {
            dap: cx.local.dap,
            dap_led: cx.local.dap_led,
            #[cfg(feature = "defmt")]
            processed: cx.local.processed,
        };
        let dap_work_remains = dap_pass(dap_queue, usb, &mut hooks, polled, DAP_COMMAND_BUDGET);

        #[cfg(feature = "flash")]
        cx.local.flash_led.level(cx.local.mux.is_flashing());

        #[cfg(feature = "flash")]
        {
            use core::sync::atomic::Ordering;
            let now = crate::ELAPSED_MS.load(Ordering::Relaxed);
            let dt = now.wrapping_sub(*cx.local.last_ms);
            *cx.local.last_ms = now;
            let vfs = cx.local.vfs;
            vfs.tick(dt);
            usb.msc_service(vfs);
        }

        if usb.vcom_read_pending() && vcom.tx.remaining_capacity() > 0 {
            NVIC::pend(Interrupt::USBCTRL_IRQ);
        }
        if !vcom.rx.is_empty() && usb.vcom_write_capacity() > 0 {
            NVIC::pend(Interrupt::USBCTRL_IRQ);
        }
        if dap_work_remains {
            NVIC::pend(Interrupt::USBCTRL_IRQ);
        }
    }
}
