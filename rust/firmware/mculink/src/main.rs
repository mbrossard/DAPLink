//! High-speed CMSIS-DAP v2 probe for the NXP MCU-Link (LPC55S69) on RTIC.

#![no_std]
#![no_main]

// A flash build latches the fault into `RESET_STAMP` and resets; every other build halts.
#[cfg(all(feature = "flash", not(feature = "defmt")))]
daplink_vfs::fault_reset_handlers!();
#[cfg(all(not(feature = "flash"), not(feature = "defmt")))]
use panic_halt as _;
// `defmt` halts in every build, so the RTT-attached probe sees the panic message.
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

// One wire engine per image: `bitbang` wins over `accel`; `flash` composes the winner under the mux.
#[cfg(not(any(feature = "accel", feature = "bitbang")))]
compile_error!("enable `accel` (the default wire engine) or `bitbang`");

#[cfg(accel_engine)]
mod fits;
#[cfg(feature = "probe-timing")]
mod probe_timing;
mod vcom;

static ELAPSED_MS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

#[cfg(feature = "flash")]
daplink_vfs::retained_stamp!();

/// 1 ms tick: advance the wall clock and pend the USB task.
#[cortex_m_rt::exception]
fn SysTick() {
    ELAPSED_MS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    cortex_m::peripheral::NVIC::pend(lpc55_hal::raw::Interrupt::USB1);
}

#[rtic::app(device = lpc55_hal::raw)]
mod app {
    use core::sync::atomic::Ordering;
    use cortex_m::peripheral::NVIC;
    use daplink_lpc55_hal::{
        probe_parts, reboot_armed, swo_dma_service_irq, vcom_baud_state, vcom_service_irq,
        vcom_tx_service_irq, Bus, ClockProfile, DynCycleDelay, Fc3Swo, Lpc55Boot, Vcom,
    };

    #[cfg(accel_engine)]
    use crate::fits::AccelPacing;
    #[cfg(accel_engine)]
    use daplink_lpc55_hal::masked_deps;
    #[cfg(not(accel_engine))]
    use daplink_lpc55_hal::IoPin;
    #[cfg(all(accel_engine, not(feature = "flash")))]
    use daplink_lpc55_hal::{MaskedBlock, MaskedSwdDap};
    #[cfg(all(accel_engine, feature = "flash"))]
    use daplink_lpc55_hal::{MaskedDeps, MaskedJtagDrv, MaskedSwdDrv};
    use daplink_rs::daplink_core::{
        product_string, DapQueue, EndpointSize, PacketSize, CDC_HS_PACKET_SIZE, DAP_HS_PACKET_SIZE,
    };
    use daplink_rs::{BootloaderEntry, RebootVendor, VcomBaudVendor, VcomStatsVendor};
    #[cfg(not(feature = "flash"))]
    use daplink_usb_device::{dap_pass, DapPassHooks, DapUsb};
    #[cfg(feature = "flash")]
    use daplink_usb_device::{dap_pass, DapPassHooks, DapUsbMsc as DapUsb};
    use lpc55_hal::raw::Interrupt;
    use static_cell::ConstStaticCell;

    #[cfg(feature = "core-96mhz")]
    const CLOCK: ClockProfile = ClockProfile::MHZ_96;
    #[cfg(not(feature = "core-96mhz"))]
    const CLOCK: ClockProfile = ClockProfile::MHZ_150;
    pub(crate) const CPU_HZ: u32 = CLOCK.cpu_hz();

    #[cfg(not(feature = "flash"))]
    use daplink_rs::daplink_core::DAP_FW_VERSION;
    #[cfg(all(feature = "bitbang", not(feature = "flash")))]
    use daplink_rs::BitbangDap;

    use cortex_m::peripheral::syst::SystClkSource;
    #[cfg(feature = "flash")]
    use daplink_rs::flash_mux::{mux_dap_engine, mux_flash_backend, MuxDapEngine, MuxFlashBackend};
    #[cfg(all(feature = "flash", not(accel_engine)))]
    use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv};
    #[cfg(feature = "flash")]
    use daplink_rs::{ScanChainStorage, SwdMux};
    #[cfg(feature = "flash-nrf52833")]
    use daplink_targets::targets::nrf52833::NRF52833;
    #[cfg(feature = "flash-nrf52840")]
    use daplink_targets::targets::nrf52840::NRF52840;
    #[cfg(feature = "flash-stm32f401")]
    use daplink_targets::targets::stm32f401ce::STM32F401CE;
    #[cfg(feature = "flash")]
    use daplink_vfs::{
        AllDecoders, FlashSink, FlashVolume, StaticCell, VendorFlash, VolumeStorage,
    };

    #[cfg(feature = "flash-stm32f401")]
    type PrimaryTarget = STM32F401CE;
    #[cfg(feature = "flash-stm32f401")]
    static PRIMARY: &PrimaryTarget = &STM32F401CE;
    #[cfg(all(feature = "flash-nrf52840", not(feature = "flash-stm32f401")))]
    type PrimaryTarget = NRF52840;
    #[cfg(all(feature = "flash-nrf52840", not(feature = "flash-stm32f401")))]
    static PRIMARY: &PrimaryTarget = &NRF52840;
    #[cfg(all(
        feature = "flash-nrf52833",
        not(any(feature = "flash-stm32f401", feature = "flash-nrf52840"))
    ))]
    type PrimaryTarget = NRF52833;
    #[cfg(all(
        feature = "flash-nrf52833",
        not(any(feature = "flash-stm32f401", feature = "flash-nrf52840"))
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
            ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
            ("flash-nrf52840", NRF52840, daplink_targets::targets::nrf52840::PROGRAM_BUFFER_SIZE),
            ("flash-nrf52833", NRF52833, daplink_targets::targets::nrf52833::PROGRAM_BUFFER_SIZE),
        ],
    }

    /// Bit-cell core cycles subtracted from the requested half-period.
    const SWD_FIXED_WORK_CYCLES: u32 = 0;

    /// Floor on the residual spin, in core cycles; zero runs flat out.
    const MIN_HALF_PERIOD_CYCLES: u32 = 0;

    /// Command-queue depth: eight 512-byte packets in flight.
    const DAP_QUEUE_DEPTH: usize = 8;

    /// Maximum commands executed in one USB interrupt activation.
    const DAP_COMMAND_BUDGET: usize = 1;

    const DAP_QUEUE_SLOT: usize = DAP_HS_PACKET_SIZE as usize;

    type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

    const DAP_PACKET: PacketSize = daplink_usb_device::single_transfer_packet(
        Queue::packet_size::<DAP_HS_PACKET_SIZE>(),
        DAP_ENDPOINT,
    );

    const DAP_ENDPOINT: EndpointSize = Queue::endpoint_size::<DAP_HS_PACKET_SIZE>();

    const DAP_PACKET_COUNT: u8 = 8;

    // Advertised count, ring depth and slot size are one fact.
    const _: () = {
        assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
        assert!(Queue::new().packet_capacity() == DAP_HS_PACKET_SIZE as usize);
        assert!(DAP_COMMAND_BUDGET > 0 && DAP_COMMAND_BUDGET <= DAP_QUEUE_DEPTH);
    };

    const MSC_EP: usize = DAP_HS_PACKET_SIZE as usize;

    pub(crate) const CDC_EP: usize = CDC_HS_PACKET_SIZE as usize;

    /// VCOM staging: four packets for a host that returns late.
    const CDC_BUF: usize = 4 * CDC_EP;

    #[cfg(feature = "probe-timing")]
    use crate::probe_timing as timing;

    #[cfg(not(feature = "flash"))]
    type Vendor = (RebootVendor<Lpc55Boot>, VcomBaudVendor, VcomStatsVendor);

    #[cfg(all(feature = "bitbang", not(feature = "flash")))]
    type Engine = BitbangDap<'static, IoPin, DynCycleDelay, Fc3Swo, Vendor>;

    #[cfg(all(accel_engine, not(feature = "flash")))]
    type Engine = MaskedSwdDap<'static, AccelPacing, Fc3Swo, Vendor>;

    #[cfg(all(feature = "flash", accel_engine))]
    type MuxDeps = MaskedDeps<AccelPacing>;
    #[cfg(all(feature = "flash", not(accel_engine)))]
    type MuxDeps = LateJtagDeps<IoPin, DynCycleDelay>;
    #[cfg(all(feature = "flash", accel_engine))]
    type MuxSwd = MaskedSwdDrv<AccelPacing>;
    #[cfg(all(feature = "flash", not(accel_engine)))]
    type MuxSwd = LateSwdDrv<IoPin, DynCycleDelay>;
    #[cfg(all(feature = "flash", accel_engine))]
    type MuxJtag = MaskedJtagDrv<AccelPacing>;
    #[cfg(all(feature = "flash", not(accel_engine)))]
    type MuxJtag = LateJtag<IoPin, DynCycleDelay>;
    #[cfg(feature = "flash")]
    type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
    #[cfg(feature = "flash")]
    type Backend =
        MuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, PrimaryTarget, Selector>;
    #[cfg(feature = "flash")]
    type Sink = FlashSink<Backend>;
    #[cfg(feature = "flash")]
    type RebootBusy = &'static Mux;
    /// Reboot, declined with `STATUS_BUSY` during a live flash session.
    #[cfg(feature = "flash")]
    type Vendor = (
        RebootVendor<Lpc55Boot, RebootBusy>,
        VendorFlash<AllDecoders, Sink>,
        VcomBaudVendor,
        VcomStatsVendor,
    );
    #[cfg(feature = "flash")]
    type Engine = MuxDapEngine<'static, MuxDeps, MuxSwd, MuxJtag, DynCycleDelay, Fc3Swo, Vendor>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb: DapUsb<'static, Bus, MSC_EP, CDC_EP, CDC_BUF>,
        dap: Engine,
        #[cfg(feature = "flash")]
        vfs: &'static mut FlashVolume<Sink>,
        #[cfg(feature = "flash")]
        last_ms: u32,
        dap_queue: &'static mut Queue,
        /// A whole SWD `DAP_TransferBlock` from SRAMX; never under the flash mux, which it bypasses.
        #[cfg(all(accel_engine, not(feature = "flash")))]
        block: MaskedBlock<AccelPacing>,
        uart: Vcom,
        /// When VCOM was last serviced; idle ports are serviced once per tick.
        vcom_ms: u32,
        #[cfg(feature = "defmt")]
        processed: u32,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        #[cfg(feature = "probe-timing")]
        timing::start();
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
        defmt::info!("mculink: boot");

        let delay =
            DynCycleDelay::with_fixed_work(CPU_HZ, SWD_FIXED_WORK_CYCLES, MIN_HALF_PERIOD_CYCLES);
        let parts = probe_parts(cx.device, CLOCK, delay, DAP_ENDPOINT);
        let cfg = parts.cfg.with_product(product_string!("mculink"));
        let identity = daplink_rs::daplink_core::Identity {
            vendor: cfg.manufacturer,
            product: cfg.product,
            serial: cfg.serial,
            board_vendor: "daplink-rs",
            board_name: "mculink",
        };
        #[cfg(feature = "defmt")]
        defmt::info!("mculink: serial {=str}", parts.serial);

        #[cfg(all(feature = "bitbang", not(feature = "flash")))]
        let dap: Engine = Engine::with_vendor(
            parts.deps,
            daplink_rs::NoopLeds,
            delay,
            parts.swo,
            DAP_FW_VERSION,
            DAP_PACKET,
            vendors(),
        )
        .with_packet_count(DAP_PACKET_COUNT)
        .with_identity(identity);

        // Copy `.sramx_code` to code-bus SRAM before a transfer runs.
        // SAFETY: linker-provided bounds, and nothing executes from the destination yet.
        #[cfg(accel_engine)]
        unsafe {
            extern "C" {
                static mut __ssramx_code: u32;
                static mut __esramx_code: u32;
                static __sisramx_code: u32;
            }
            let mut dst = core::ptr::addr_of_mut!(__ssramx_code);
            let end = core::ptr::addr_of_mut!(__esramx_code);
            let mut src = core::ptr::addr_of!(__sisramx_code);
            while dst < end {
                dst.write_volatile(src.read_volatile());
                dst = dst.add(1);
                src = src.add(1);
            }
            // The copy completes before any instruction is fetched from SRAMX.
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }

        #[cfg(all(accel_engine, not(feature = "flash")))]
        let dap: Engine = Engine::with_vendor(
            masked_deps(delay, AccelPacing),
            daplink_rs::NoopLeds,
            delay,
            parts.swo,
            DAP_FW_VERSION,
            DAP_PACKET,
            vendors(),
        )
        .with_packet_count(DAP_PACKET_COUNT)
        .with_identity(identity);

        #[cfg(feature = "flash")]
        let (dap, vfs) = {
            static MUX: StaticCell<Mux> = StaticCell::new();
            static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
                ScanChainStorage::new();
            static VENDOR_SCRATCH: ConstStaticCell<[u8; SCRATCH]> =
                ConstStaticCell::new([0; SCRATCH]);
            static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

            #[cfg(accel_engine)]
            let adapter = masked_deps(delay, AccelPacing);
            #[cfg(not(accel_engine))]
            let adapter = parts.deps;
            let mux: &'static Mux = MUX.init(SwdMux::new(adapter));
            let table = flash_table();
            let vendor_backend: Backend =
                mux_flash_backend(mux, delay, PRIMARY, table, VENDOR_SCRATCH.take());
            let dap: Engine = mux_dap_engine(
                mux,
                delay,
                parts.swo,
                DAP_PACKET,
                &MUX_SCAN,
                (
                    RebootVendor::with_busy_check(Lpc55Boot, mux),
                    VendorFlash::new(FlashSink::new(vendor_backend)),
                    VcomBaudVendor::new(vcom_baud_state),
                    VcomStatsVendor::new(crate::vcom::vcom_counters),
                ),
                daplink_rs::NoopLeds,
            )
            .with_packet_count(DAP_PACKET_COUNT)
            .with_identity(identity);
            let vfs_backend: Backend =
                mux_flash_backend(mux, delay, PRIMARY, table, flash_scratch());
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

            (dap, vfs)
        };

        // The 1 ms tick bounds a receive backlog's wait and clocks the flash volume.
        let mut syst = cx.core.SYST;
        syst.set_clock_source(SystClkSource::Core);
        const { assert!(CPU_HZ.is_multiple_of(1000)) };
        syst.set_reload(CPU_HZ / 1000 - 1);
        syst.clear_current();
        syst.enable_interrupt();
        syst.enable_counter();

        // The LPC55 USB1 controller has endpoints 0..6, and the HAL indexes its arrays unchecked.
        let usb = DapUsb::new(parts.usb_alloc, &cfg)
            .with_endpoint_repair(&daplink_usb_device::RegisterCount::<6>);

        // `ConstStaticCell` keeps the ring pair in `.bss` instead of on the init stack.
        static DAP_QUEUE: ConstStaticCell<Queue> = ConstStaticCell::new(Queue::new());
        let dap_queue = DAP_QUEUE.take();

        (
            Shared {},
            Local {
                usb,
                dap,
                #[cfg(feature = "flash")]
                vfs,
                #[cfg(feature = "flash")]
                last_ms: 0,
                dap_queue,
                #[cfg(all(accel_engine, not(feature = "flash")))]
                block: MaskedBlock::new(AccelPacing),
                uart: parts.uart,
                vcom_ms: 0,
                #[cfg(feature = "defmt")]
                processed: 0,
            },
        )
    }

    /// Enter ROM from thread mode after the DAP response drains.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            if reboot_armed() {
                cortex_m::asm::delay(CPU_HZ / 20);
                Lpc55Boot.enter();
            }
        }
    }

    /// VCOM rate changes, receive errors, and the RX FIFO until DMA owns it; above any DAP command.
    #[task(binds = FLEXCOMM0, priority = 2)]
    fn on_uart(_cx: on_uart::Context) {
        #[cfg(feature = "probe-timing")]
        timing::bump(&timing::PROBE_FC0_ENTRIES, 1);
        vcom_service_irq();
        crate::vcom::VCOM_KICK.store(true, Ordering::Relaxed);
        NVIC::pend(Interrupt::USB1);
    }

    /// VCOM TX: re-arm the next staging slot now to keep the line busy; SWO: count completions.
    #[task(binds = DMA0, priority = 2)]
    fn on_dma(_cx: on_dma::Context) {
        #[cfg(feature = "probe-timing")]
        timing::bump(&timing::PROBE_DMA0_ENTRIES, 1);
        vcom_tx_service_irq();
        swo_dma_service_irq();
        // A completed transfer frees ring space, so wake the stalled pass.
        crate::vcom::VCOM_KICK.store(true, Ordering::Relaxed);
        NVIC::pend(Interrupt::USB1);
    }

    struct Pass<'a> {
        dap: &'a mut Engine,
        #[cfg(all(accel_engine, not(feature = "flash")))]
        block: &'a mut MaskedBlock<AccelPacing>,
        #[cfg(feature = "defmt")]
        processed: &'a mut u32,
    }

    impl DapPassHooks for Pass<'_> {
        fn execute(&mut self, request: &[u8], response: &mut [u8]) -> usize {
            #[cfg(all(accel_engine, not(feature = "flash")))]
            if let Some(len) = self.block.process(request, response) {
                return len;
            }
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

        fn requests_full(&mut self) {
            #[cfg(feature = "probe-timing")]
            timing::bump(&timing::PROBE_DAP_FULL, 1);
        }

        fn received(&mut self) {
            #[cfg(feature = "probe-timing")]
            timing::evt(4, 1);
        }

        fn executed(&mut self, _len: usize) {
            #[cfg(feature = "probe-timing")]
            timing::evt(5, (_len >> 2) as u32);
            #[cfg(feature = "defmt")]
            {
                *self.processed += 1;
            }
        }

        fn sent(&mut self, _len: usize) {
            #[cfg(feature = "probe-timing")]
            timing::evt(6, (_len >> 2) as u32);
        }

        fn refused(&mut self, _len: usize) {
            #[cfg(feature = "probe-timing")]
            timing::evt(7, (_len >> 2) as u32);
        }
    }

    #[task(binds = USB1, local = [usb, dap, dap_queue, uart, vcom_ms, #[cfg(all(accel_engine, not(feature = "flash")))] block, #[cfg(feature = "defmt")] processed, #[cfg(feature = "flash")] vfs, #[cfg(feature = "flash")] last_ms])]
    fn on_usb(cx: on_usb::Context) {
        #[cfg(feature = "probe-timing")]
        let entered = timing::now();
        #[cfg(feature = "probe-timing")]
        {
            timing::sample_intstat();
            timing::sample_ep3out();
            timing::evt(1, timing::intstat());
        }
        let usb = cx.local.usb;
        let dap_queue = &mut **cx.local.dap_queue;
        let uart = cx.local.uart;
        // Arm the receive channel only once enumeration is over, or the attach is lost.
        if usb.configured() {
            uart.start_rx_dma();
        }

        let polled = usb.poll();
        #[cfg(feature = "probe-timing")]
        timing::evt(2, polled as u32);

        let _vcom_due = crate::vcom::service(usb, uart, cx.local.vcom_ms);
        #[cfg(feature = "probe-timing")]
        timing::evt(3, _vcom_due as u32);

        let mut hooks = Pass {
            dap: cx.local.dap,
            #[cfg(all(accel_engine, not(feature = "flash")))]
            block: cx.local.block,
            #[cfg(feature = "defmt")]
            processed: cx.local.processed,
        };
        let dap_work_remains = dap_pass(dap_queue, usb, &mut hooks, polled, DAP_COMMAND_BUDGET);
        #[cfg(feature = "probe-timing")]
        timing::evt(8, dap_work_remains as u32);

        #[cfg(feature = "flash")]
        {
            let now = crate::ELAPSED_MS.load(Ordering::Relaxed);
            let dt = now.wrapping_sub(*cx.local.last_ms);
            *cx.local.last_ms = now;
            let vfs = cx.local.vfs;
            vfs.tick(dt);
            usb.msc_service(vfs);
        }

        // Re-pend to drain a landed packet, but never on a full ring: that spins.
        if usb.vcom_read_pending() && uart.write_capacity() != 0 {
            NVIC::pend(Interrupt::USB1);
        }
        if dap_work_remains {
            NVIC::pend(Interrupt::USB1);
        }

        #[cfg(feature = "probe-timing")]
        {
            let cycles = timing::cycles_since(entered);
            timing::bump(&timing::PROBE_USB1_PASSES, 1);
            timing::bump(&timing::PROBE_USB1_TOTAL_US, cycles);
            timing::max(&timing::PROBE_USB1_MAX_US, cycles);
        }
    }

    pub(crate) type Usb = DapUsb<'static, Bus, MSC_EP, CDC_EP, CDC_BUF>;

    /// Reboot and the two VCOM queries, for an engine outside the flash mux.
    #[cfg(not(feature = "flash"))]
    fn vendors() -> Vendor {
        (
            RebootVendor::new(Lpc55Boot),
            VcomBaudVendor::new(vcom_baud_state),
            VcomStatsVendor::new(crate::vcom::vcom_counters),
        )
    }
}
