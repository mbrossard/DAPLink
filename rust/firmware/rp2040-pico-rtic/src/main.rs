//! Minimal CMSIS-DAP v2 debug probe for the Raspberry Pi Pico (RP2040) on RTIC.

#![no_std]
#![no_main]

const _: () = assert!(!cfg!(feature = "overclock-200mhz") || daplink_rp_hal::CPU_HZ == 200_000_000);

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

/// The linker places this 256-byte second-stage bootloader at the start of flash.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// Millisecond wall clock; SysTick is the sole writer and readers take wrap-safe deltas.
#[cfg(feature = "flash")]
static ELAPSED_MS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Reset-surviving probe state (boot counter, last transfer status) in `.uninit` RAM; on this
/// part only a power cycle clears it.
#[cfg(feature = "flash")]
#[link_section = ".uninit.RESET_STAMP"]
static mut RESET_STAMP: core::mem::MaybeUninit<daplink_vfs::ResetStamp> =
    core::mem::MaybeUninit::uninit();

/// 1 ms tick (flash builds only): advance the wall clock and pend the USB task.
#[cfg(feature = "flash")]
#[cortex_m_rt::exception]
fn SysTick() {
    use core::sync::atomic::Ordering;
    // Sole writer, so a plain load/store increment is race-free; thumbv6m has no CAS anyway.
    let next = ELAPSED_MS.load(Ordering::Relaxed).wrapping_add(1);
    ELAPSED_MS.store(next, Ordering::Relaxed);
    cortex_m::peripheral::NVIC::pend(rp2040_hal::pac::Interrupt::USBCTRL_IRQ);
}

#[rtic::app(device = rp2040_hal::pac)]
mod app {
    use cortex_m::peripheral::NVIC;
    use daplink_rp_hal::{
        status_led, swo_drain, vcom_account_delivery, vcom_account_discard,
        vcom_account_host_traffic, vcom_baud_state, vcom_counters, vcom_enable_tx_interrupt,
        vcom_irq_service, vcom_request_baud, CycleDelay, IoPin, StatusLedPin, SwoProducer,
        Uart0Swo, VcomIrq, VcomUsb,
    };
    use daplink_rs::daplink_core::{
        product_string, DapConfig, DapQueue, DAP_PACKET_SIZE, DEFAULT_VCOM_BAUD,
    };
    use daplink_rs::{Activity, StatusLed, VcomBaudVendor, VcomStatsVendor, VendorChain};
    use daplink_usb_device::DapUsb;
    use rp2040_hal::pac::Interrupt;
    use rp2040_hal::usb::UsbBus;
    use static_cell::ConstStaticCell;

    #[cfg(not(feature = "flash"))]
    use daplink_rs::daplink_core::DAP_FW_VERSION;

    #[cfg(feature = "carrier-leds")]
    use daplink_rp_hal::{carrier_leds, CarrierBankLed};
    #[cfg(feature = "carrier-leds")]
    use rp2040_hal::gpio::bank0::{Gpio10, Gpio11, Gpio12, Gpio14};

    #[cfg(not(rp_accel))]
    use daplink_rp_hal::probe_parts_pico;
    use daplink_rp_hal::RpUsbBoot;
    #[cfg(all(not(rp_accel), not(feature = "flash")))]
    use daplink_rs::BitbangDap;
    use daplink_rs::RebootVendor;

    // One wire engine per image: `bitbang` wins where both are on.
    #[cfg(not(any(feature = "accel", feature = "bitbang")))]
    compile_error!("enable `accel` (the default wire engine) or `bitbang`");

    #[cfg(rp_accel)]
    use daplink_rp_hal::probe_parts_pico_accel;
    #[cfg(all(rp_accel, not(feature = "flash")))]
    use daplink_rp_pio::PioSwdJtagDap;
    #[cfg(rp_accel)]
    use daplink_rp_pio::{pio_swd_deps, PioBlock, PioSm, PioSwdPins};
    #[cfg(rp_accel)]
    use daplink_rs::ComposedJtagDeps;
    #[cfg(any(rp_accel, feature = "flash"))]
    use daplink_rs::ScanChainStorage;

    #[cfg(feature = "flash")]
    use cortex_m::peripheral::syst::SystClkSource;
    #[cfg(all(feature = "flash", rp_accel))]
    use daplink_rp_pio::{PioSwdDeps, PioSwdDrv};
    #[cfg(feature = "flash")]
    use daplink_rs::flash_mux::{
        role_build_flash_backend, role_mux_dap_engine_with_vendor, RoleMuxDapEngine,
        RoleMuxFlashBackend,
    };
    #[cfg(feature = "flash")]
    use daplink_rs::SwdMux;
    #[cfg(all(feature = "flash", rp_accel))]
    use daplink_rs::{ComposedJtagDrv, ComposedJtagSwd};
    #[cfg(all(feature = "flash", not(rp_accel)))]
    use daplink_rs::{LateJtag, LateJtagDeps, LateSwdDrv};
    #[cfg(feature = "flash-rp2040")]
    use daplink_targets::targets::rp2040::RP2040;
    #[cfg(feature = "flash-stm32f401")]
    use daplink_targets::targets::stm32f401ce::STM32F401CE;
    #[cfg(feature = "flash")]
    use daplink_vfs::{
        AllDecoders, FlashSink, FlashVolume, StaticCell, VendorFlash, VolumeStorage,
    };

    // The registry the flash spine selects from, derived from the rows below.
    #[cfg(feature = "flash")]
    daplink_targets::target_table! {
        umbrella = "flash",
        count = TARGET_COUNT,
        primary = PrimaryTarget,
        selector = Selector,
        scratch = SCRATCH,
        primary_ref = PRIMARY,
        table = flash_table,
        buffer = flash_scratch,
        rows = [
            ("flash-rp2040", RP2040, daplink_targets::targets::rp2040::PROGRAM_BUFFER_SIZE),
            ("flash-stm32f401", STM32F401CE, daplink_targets::targets::stm32f401ce::PROGRAM_BUFFER_SIZE),
        ],
    }

    /// The onboard status LED's concrete `DapLeds` type: GP25, active-high.
    type StatusLedType = StatusLed<StatusLedPin>;

    /// The carrier-bank heartbeat LED (GP10), pulsed from the idle-loop wake.
    #[cfg(feature = "carrier-leds")]
    type HeartbeatLed = CarrierBankLed<Gpio10>;
    #[cfg(not(feature = "carrier-leds"))]
    type HeartbeatLed = ();

    /// The carrier-bank DAP-activity LED (GP11), pulsed once per command processed.
    #[cfg(feature = "carrier-leds")]
    type DapLed = CarrierBankLed<Gpio11>;
    #[cfg(not(feature = "carrier-leds"))]
    type DapLed = ();

    /// The carrier-bank flash-session LED (GP12), lit while the flash mux holds the wire.
    #[cfg(feature = "carrier-leds")]
    type FlashLed = CarrierBankLed<Gpio12>;
    #[cfg(not(feature = "carrier-leds"))]
    type FlashLed = ();

    /// The carrier-bank VCOM-traffic LED (GP14), pulsed once per byte chunk bridged either way.
    #[cfg(feature = "carrier-leds")]
    type VcomLed = CarrierBankLed<Gpio14>;
    #[cfg(not(feature = "carrier-leds"))]
    type VcomLed = ();

    /// Pipelined CMSIS-DAP command-queue depth, also the advertised `DAP_PACKET_COUNT`.
    const DAP_QUEUE_DEPTH: usize = 8;

    /// Per-slot byte capacity of the command queue: one full-speed DAP v2 bulk packet.
    const DAP_QUEUE_SLOT: usize = DAP_PACKET_SIZE as usize;

    /// This image's command queue: the depth the host is told about and the storage behind it.
    type Queue = DapQueue<DAP_QUEUE_DEPTH, DAP_QUEUE_SLOT>;

    /// The `DAP_Info(MaxPacketCount)` this image advertises; fed to the engine below.
    const DAP_PACKET_COUNT: u8 = 8;

    // Advertised count, ring depth and slot size are one fact; widening one alone fails the build.
    const _: () = {
        assert!(Queue::new().depth() == DAP_PACKET_COUNT as usize);
        assert!(Queue::new().packet_capacity() == DAP_PACKET_SIZE as usize);
    };

    /// The reboot-to-bootloader vendor handler; a non-flash build has no session to gate it behind.
    #[cfg(not(feature = "flash"))]
    type RebootOnly =
        VendorChain<RebootVendor<RpUsbBoot>, VendorChain<VcomBaudVendor, VcomStatsVendor>>;

    /// The bit-bang SWD+JTAG engine: SWO capture, status LED, reboot in the vendor slot.
    #[cfg(all(not(rp_accel), not(feature = "flash")))]
    type Engine = BitbangDap<'static, IoPin, CycleDelay, Uart0Swo, RebootOnly, StatusLedType>;

    /// The accelerated engine: PIO SWD with bit-bang JTAG over the pad loan, same slots as above.
    #[cfg(all(rp_accel, not(feature = "flash")))]
    type Engine =
        PioSwdJtagDap<'static, PioSm, IoPin, CycleDelay, Uart0Swo, RebootOnly, StatusLedType>;

    /// The dap-rs dependency role of the one adapter the flash build arbitrates.
    #[cfg(all(feature = "flash", not(rp_accel)))]
    type MuxDeps = LateJtagDeps<IoPin, CycleDelay>;
    /// The SWD driver role of [`MuxDeps`].
    #[cfg(all(feature = "flash", not(rp_accel)))]
    type MuxSwd = LateSwdDrv<IoPin, CycleDelay>;
    /// The JTAG driver role of [`MuxDeps`].
    #[cfg(all(feature = "flash", not(rp_accel)))]
    type MuxJtag = LateJtag<IoPin, CycleDelay>;

    /// The accelerated spelling of [`MuxDeps`]: PIO SWD with bit-bang JTAG composed on.
    #[cfg(all(feature = "flash", rp_accel))]
    type MuxDeps = ComposedJtagDeps<PioSwdDeps<PioSm>, IoPin, CycleDelay>;
    /// The SWD driver role of [`MuxDeps`].
    #[cfg(all(feature = "flash", rp_accel))]
    type MuxSwd = ComposedJtagSwd<PioSwdDrv<PioSm>, IoPin, CycleDelay>;
    /// The JTAG driver role of [`MuxDeps`].
    #[cfg(all(feature = "flash", rp_accel))]
    type MuxJtag = ComposedJtagDrv<PioSwdDeps<PioSm>, IoPin, CycleDelay>;

    /// The flash-wins arbiter owning the one physical adapter.
    #[cfg(feature = "flash")]
    type Mux = SwdMux<MuxDeps, MuxSwd, MuxJtag>;
    /// The flash-algorithm runtime over an ungated flash-side handle on the mux.
    #[cfg(feature = "flash")]
    type Backend =
        RoleMuxFlashBackend<MuxDeps, MuxSwd, MuxJtag, CycleDelay, PrimaryTarget, Selector>;
    /// The image-fragment sink one flash front-end owns.
    #[cfg(feature = "flash")]
    type Sink = FlashSink<Backend>;
    /// The vendor slot's flash-busy gate: the mux's own live-session flag.
    #[cfg(feature = "flash")]
    type RebootBusy = &'static Mux;
    /// The reboot command, declined with `STATUS_BUSY` while [`RebootBusy`] reports a live flash
    /// session, chained ahead of the vendor flash-stream commands.
    #[cfg(feature = "flash")]
    type Vendor = VendorChain<
        VendorChain<RebootVendor<RpUsbBoot, RebootBusy>, VendorFlash<AllDecoders, Sink>>,
        VendorChain<VcomBaudVendor, VcomStatsVendor>,
    >;
    /// The flash build's engine: gated by the mux, with the vendor flash commands composed in.
    #[cfg(feature = "flash")]
    type Engine = RoleMuxDapEngine<
        'static,
        MuxDeps,
        MuxSwd,
        MuxJtag,
        CycleDelay,
        Uart0Swo,
        Vendor,
        StatusLedType,
    >;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb: DapUsb<'static, UsbBus>,
        dap: Engine,
        /// Caller-owned command queue, filled and drained by the USB task in the same pass.
        dap_queue: &'static mut Queue,
        vcom_usb: VcomUsb,
        /// Last baud pushed to the UART divisor; the VCOM bridge reprograms only on a change.
        vcom_baud: u32,
        /// Commands run since the last connection edge, logged once at `session_ended()`.
        #[cfg(feature = "defmt")]
        processed: u32,
        swo_producer: SwoProducer,
        vcom_irq: VcomIrq,
        /// The virtual mass-storage volume, streaming a dropped image through the flash-wins mux.
        #[cfg(feature = "flash")]
        vfs: &'static mut FlashVolume<Sink>,
        /// The previous [`ELAPSED_MS`](crate::ELAPSED_MS) reading, for a wrap-safe delta.
        #[cfg(feature = "flash")]
        last_ms: u32,
        /// The mux, polled by `on_usb` for [`FlashLed`]'s session state.
        #[cfg(feature = "flash")]
        mux: &'static Mux,
        heartbeat_led: HeartbeatLed,
        dap_led: DapLed,
        flash_led: FlashLed,
        vcom_led: VcomLed,
    }

    /// The one physical adapter's dependency value, inlined at its single use site in each build.
    #[cfg(not(rp_accel))]
    macro_rules! probe_deps {
        ($parts:expr) => {
            $parts.deps
        };
    }

    /// The accelerated spelling of [`probe_deps`]: PIO0 on SWCLK/SWDIO, bit-bang JTAG over the loan.
    #[cfg(rp_accel)]
    macro_rules! probe_deps {
        ($parts:expr, $delay:expr) => {{
            static SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
                ScanChainStorage::new();
            let pins = PioSwdPins {
                swclk: $parts.swclk,
                swdio_out: $parts.swdio,
                // One bidirectional wire and no level shifter, so no separate readback or OE pad.
                swdio_in: None,
                oen: None,
                nreset: Some($parts.nreset),
            };
            // SAFETY: the bring-up above hands over PIO0 and these pads, and this runs once.
            let deps = unsafe { pio_swd_deps(PioBlock::Pio0, pins, daplink_rp_hal::CPU_HZ, &SCAN) };
            ComposedJtagDeps::new(deps, $parts.jtag_lines, $delay)
        }};
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Boot the stamp first: it arms the fault latch, and everything below can panic.
        // SAFETY: `RESET_STAMP` is only touched here, once, before anything concurrent runs.
        #[cfg(feature = "flash")]
        let stamp = daplink_vfs::ResetStamp::boot_in_place(unsafe {
            &mut *core::ptr::addr_of_mut!(crate::RESET_STAMP)
        });

        #[cfg(feature = "defmt")]
        defmt::info!("rp2040-pico-rtic: boot");

        let delay = CycleDelay;
        #[cfg(not(rp_accel))]
        let parts = probe_parts_pico(cx.device, delay);
        #[cfg(rp_accel)]
        let parts = probe_parts_pico_accel(cx.device);
        let cfg = DapConfig::daplink(parts.serial).with_product(product_string!("rp2040"));
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

        #[cfg(all(not(rp_accel), not(feature = "flash")))]
        let dap: Engine = BitbangDap::with_vendor(
            probe_deps!(parts),
            status,
            delay,
            parts.swo,
            DAP_FW_VERSION,
            cfg.max_packet_size,
            VendorChain::new(
                RebootVendor::new(RpUsbBoot),
                VendorChain::new(
                    VcomBaudVendor::new(vcom_baud_state),
                    VcomStatsVendor::new(vcom_counters),
                ),
            ),
        )
        .with_packet_count(DAP_PACKET_COUNT);
        #[cfg(all(rp_accel, not(feature = "flash")))]
        let dap: Engine = Engine::with_vendor(
            probe_deps!(parts, delay),
            status,
            delay,
            parts.swo,
            DAP_FW_VERSION,
            cfg.max_packet_size,
            VendorChain::new(
                RebootVendor::new(RpUsbBoot),
                VendorChain::new(
                    VcomBaudVendor::new(vcom_baud_state),
                    VcomStatsVendor::new(vcom_counters),
                ),
            ),
        )
        .with_packet_count(DAP_PACKET_COUNT);
        #[cfg(feature = "flash")]
        let (dap, vfs, mux) = {
            #[cfg(not(rp_accel))]
            let adapter = probe_deps!(parts);
            #[cfg(rp_accel)]
            let adapter = probe_deps!(parts, delay);

            static MUX: StaticCell<Mux> = StaticCell::new();
            static MUX_SCAN: ScanChainStorage<{ daplink_rs::daplink_core::SCAN_CHAIN_SLOTS }> =
                ScanChainStorage::new();
            // Two front-ends, two backends; the mux grants one session and refuses the other.
            static VENDOR_SCRATCH: ConstStaticCell<[u8; SCRATCH]> =
                ConstStaticCell::new([0; SCRATCH]);
            static VOLUME: VolumeStorage<Sink> = VolumeStorage::new();

            let mux: &'static Mux = MUX.init(SwdMux::new(adapter));
            let table = flash_table();
            let vendor_backend: Backend =
                role_build_flash_backend(mux, delay, PRIMARY, table, VENDOR_SCRATCH.take());
            let dap = role_mux_dap_engine_with_vendor(
                mux,
                delay,
                parts.swo,
                cfg.max_packet_size,
                &MUX_SCAN,
                VendorChain::new(
                    VendorChain::new(
                        RebootVendor::with_busy_check(RpUsbBoot, mux),
                        VendorFlash::new(FlashSink::new(vendor_backend)),
                    ),
                    VendorChain::new(
                        VcomBaudVendor::new(vcom_baud_state),
                        VcomStatsVendor::new(vcom_counters),
                    ),
                ),
                status,
            )
            .with_packet_count(DAP_PACKET_COUNT);
            let vfs_backend: Backend =
                role_build_flash_backend(mux, delay, PRIMARY, table, flash_scratch());
            let vfs = daplink_vfs::assemble_info(
                &VOLUME,
                FlashSink::new(vfs_backend),
                daplink_vfs::DeviceInfo {
                    version: env!("CARGO_PKG_VERSION"),
                    serial: parts.serial,
                    reset_count: stamp.reset_count(),
                },
                Some(stamp),
                "DAPLINK",
            );

            // A 1 ms SysTick is the volume's only wall clock, host traffic or not.
            let mut syst = cx.core.SYST;
            syst.set_clock_source(SystClkSource::Core);
            // One interrupt per millisecond, expressible only at a whole number of kHz.
            const { assert!(daplink_rp_hal::CPU_HZ % 1000 == 0) };
            syst.set_reload(daplink_rp_hal::CPU_HZ / 1000 - 1);
            syst.clear_current();
            syst.enable_interrupt();
            syst.enable_counter();

            (dap, vfs, mux)
        };

        let usb = DapUsb::new(parts.usb_alloc, &cfg);

        // Caller-owned queue storage; `DAP_PACKET_COUNT` above names the same depth.
        static DAP_QUEUE: ConstStaticCell<Queue> = ConstStaticCell::new(Queue::new());
        let dap_queue = DAP_QUEUE.take();

        // Composition finished, so this boot is not a reset loop: forgive the fault streak.
        #[cfg(feature = "flash")]
        daplink_vfs::stamp::clear_fault_streak();
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

    /// Idle: the ROM reboot entry must run from thread mode, so it is checked after every wake,
    /// which is also this firmware's only heartbeat tick.
    #[idle(local = [heartbeat_led])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
            daplink_rp_hal::reboot_watch();
            cx.local.heartbeat_led.pulse();
        }
    }

    /// UART1 drains receive data before advancing queued transmission.
    #[task(binds = UART1_IRQ, priority = 2, local = [vcom_irq])]
    fn on_uart(cx: on_uart::Context) {
        vcom_irq_service(cx.local.vcom_irq);
        NVIC::pend(Interrupt::USBCTRL_IRQ);
    }

    /// UART0 SWO capture, masked by `UARTIMSC` until the host starts capture.
    #[task(binds = UART0_IRQ, priority = 2, local = [swo_producer])]
    fn on_swo(cx: on_swo::Context) {
        swo_drain(cx.local.swo_producer);
    }

    /// USB: pump the DAP queue, advance mass storage, bridge the VCOM port. Sector work runs
    /// inline, so a long flash program holds the mux and gates the DAP engine.
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
        let dap = cx.local.dap;
        let dap_queue = &mut **cx.local.dap_queue;
        let vcom = cx.local.vcom_usb;
        let vcom_baud = cx.local.vcom_baud;
        let polled = usb.poll();

        let requested = usb.vcom_line_coding_baud();
        if requested != 0 && requested != *vcom_baud {
            vcom_request_baud(requested);
            *vcom_baud = requested;
        }

        let mut vcom_buf = [0u8; 64];
        let room = usb.vcom_write_capacity().min(vcom_buf.len());
        if room > 0 {
            let taken = vcom.rx.drain(&mut vcom_buf[..room]);
            if taken > 0 {
                let sent = usb.vcom_write(&vcom_buf[..taken]);
                vcom_account_delivery(taken, sent);
                cx.local.vcom_led.pulse();
            }
        }
        // Staged bytes `cdc_in` already credited that the poll above threw away on a port close
        // or a bus reset; taken every pass, since a closed port sends nothing more to trigger one.
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

        if polled {
            if usb.session_ended() {
                // Host gone: drop the dead session's queue and release the wire.
                #[cfg(feature = "defmt")]
                {
                    defmt::info!(
                        "dap session: {=u32} commands processed",
                        *cx.local.processed
                    );
                    *cx.local.processed = 0;
                }
                dap_queue.reset();
                dap.suspend();
            }

            // Skip the OUT endpoint while the ring is full, so it NAKs the host into pacing itself.
            let mut buf = [0u8; DAP_QUEUE_SLOT];
            if !dap_queue.requests_full() {
                if let Some(n) = usb.read_command(&mut buf) {
                    if dap_queue.submit(&buf[..n]).is_err() {
                        debug_assert!(false, "request refused by a checked-non-full ring");
                        #[cfg(feature = "defmt")]
                        defmt::warn!("dap: request refused, command dropped");
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
                #[cfg(feature = "defmt")]
                {
                    *cx.local.processed += 1;
                }
                cx.local.dap_led.pulse();
                if len > 0 && dap_queue.post_response(&resp[..len]).is_err() {
                    debug_assert!(false, "response ring full after a reserved slot");
                    #[cfg(feature = "defmt")]
                    defmt::warn!("dap: response ring full, response dropped");
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

        #[cfg(feature = "flash")]
        cx.local.flash_led.level(cx.local.mux.is_flashing());

        // Advance the volume's timers by the elapsed milliseconds, then move one bulk packet.
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
    }
}
