//! RP2350 Pico 2 device-under-test image: flash pattern, LED, UART0 echo (GP0 TX / GP1 RX), RTT responder.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use daplink_rp_dma_uart as uartdma;
use daplink_test_protocol as proto;
use daplink_test_protocol::rp_dut::{Dut, Hardware, TxBuffers};
use daplink_test_protocol::v1;
use embassy_rp::clocks::CoreVoltage;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::TreqSel;
use panic_halt as _;
use rtt_target::{rtt_init, ChannelMode};

const NAME: &str = "test-rp2350";

/// Known data for flash-verification cases.
#[used]
static PATTERN: [u32; 256] = {
    let mut a = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        a[i] = (i as u32).wrapping_mul(0x0101_0101) ^ 0xA5A5_5A5A;
        i += 1;
    }
    a
};

/// Boot baud of the echo UART; the `BAUD` verb reprograms it.
const ECHO_BAUD: u32 = 115_200;

/// clk_peri feeds the PL011; `embassy_rp::init`'s default config leaves it at clk_sys.
const CLK_PERI_HZ: u32 = 150_000_000;

/// Highest rate `BAUD` accepts.
const BAUD_MAX: u32 = pl011_baud::max_baud(CLK_PERI_HZ);

/// GPIO function-select value that routes a pin to its UART peripheral.
const FUNCSEL_UART: u8 = 2;

/// Request 1.15 V: at the 1.10 V reset value the 150 MHz PLL intermittently misses lock.
fn clock_config() -> embassy_rp::config::Config {
    let mut config = embassy_rp::config::Config::default();
    config.clocks.core_voltage = CoreVoltage::V1_15;
    config.clocks.usb_clk = None;
    config
}

/// Bring UART0 up on GP0 (TX) / GP1 (RX), 8N1 with FIFOs, straight on the PL011 registers.
fn init_echo_uart(baud: u32) {
    pac::IO_BANK0
        .gpio(0)
        .ctrl()
        .write(|w| w.set_funcsel(FUNCSEL_UART));
    pac::IO_BANK0
        .gpio(1)
        .ctrl()
        .write(|w| w.set_funcsel(FUNCSEL_UART));
    // RP2350 pads reset isolated and input-disabled; the pull-downs fight the far end's TX.
    pac::PADS_BANK0.gpio(0).modify(|w| {
        w.set_iso(false);
        w.set_ie(true);
        w.set_pde(false);
    });
    pac::PADS_BANK0.gpio(1).modify(|w| {
        w.set_iso(false);
        w.set_ie(true);
        w.set_pde(false);
        w.set_pue(true);
    });
    pac::UART0.uartlcr_h().write(|w| {
        w.set_wlen(0b11); // 8 data bits
        w.set_fen(true); // FIFOs enabled
    });
    let _ = set_echo_baud(baud);
}

/// Reprogram UART0's divisor and return the rate it realizes, or `None` for an unreachable one.
fn set_echo_baud(baud: u32) -> Option<u32> {
    let div = pl011_baud::div_x64(CLK_PERI_HZ, baud)?;
    let (ibrd, fbrd) = pl011_baud::split(div);
    let uart = pac::UART0;
    uart.uartcr().modify(|w| w.set_uarten(false));
    uart.uartibrd().write(|w| w.set_baud_divint(ibrd));
    uart.uartfbrd().write(|w| w.set_baud_divfrac(fbrd));
    uart.uartlcr_h().modify(|_| {});
    uart.uartcr().modify(|w| w.set_uarten(true));
    // The line stopped and restarted underneath the ring, so its cursor re-bases from empty.
    uart0_dma_arm().ok()?;
    Some(pl011_baud::realized(CLK_PERI_HZ, div))
}

/// Entries one receive pass takes before yielding to the responder.
const RX_BATCH: usize = 32;

/// UART0 over DMA channels 0 (receive) and 1 (transmit).
static RX_RING: uartdma::RxRing = uartdma::RxRing::new();
// SAFETY: this single-threaded image reserves UART0 and DMA channels 0/1 exclusively.
static mut UART_DMA: uartdma::UartDma =
    unsafe { uartdma::UartDma::new(pac::UART0, 0, 1, TreqSel::UART0_RX, TreqSel::UART0_TX) };
static mut TX_BUFFERS: TxBuffers = TxBuffers::new();

/// Arm the receive ring. Called after every divisor change, which is the only safe moment.
fn uart0_dma_arm() -> Result<(), uartdma::AbortTimeout> {
    // SAFETY: single-threaded, and the ring is a static that outlives every transfer.
    unsafe { (*core::ptr::addr_of_mut!(UART_DMA)).arm(&RX_RING) }
}

/// # Safety
/// No other access to UART_DMA may overlap the returned reference.
unsafe fn uart_dma() -> &'static mut uartdma::UartDma {
    // SAFETY: the caller excludes every other reference for this borrow.
    unsafe { &mut *core::ptr::addr_of_mut!(UART_DMA) }
}

/// Free-running 1 MHz timer for the v1 engines, which expect a u32 that wraps.
fn now_us() -> u32 {
    pac::TIMER0.timerawl().read()
}

/// Echo up to one batch of the receive ring, one byte at a time.
fn uart0_drain(dut: &mut Dut<'_>) {
    let uart = pac::UART0;
    // SAFETY: this synchronous pass neither calls the responder nor rearms the UART.
    let dma = unsafe { uart_dma() };
    for _ in 0..dma.rx_resync() {
        dut.record_overruns(1);
    }
    for _ in 0..RX_BATCH {
        let Some(entry) = dma.rx_next() else {
            break;
        };
        dut.record_rx(entry);
        // Echoed a byte at a time by the CPU: this path is latency-bound, not throughput-bound.
        while uart.uartfr().read().txff() {}
        uart.uartdr().write(|w| w.set_data(entry as u8));
        dut.record_tx(1);
    }
}

/// One pass of an armed v1 run: the ring into the verifier, the encoder into the transmit channel.
fn uart0_stream(dut: &mut Dut<'static>) {
    let now = now_us();
    // SAFETY: this synchronous pass neither calls the responder nor rearms the UART.
    let dma = unsafe { uart_dma() };

    dut.sync_credit();

    // One counter read and one cursor fold per pass; lapped entries are charged, not re-read.
    let mut batch = [0u16; RX_BATCH];
    let (taken, lapped) = dma.rx_read(&mut batch);
    dut.record_overruns(lapped);
    let mut bytes = [0u8; RX_BATCH];
    for (index, &entry) in batch[..taken].iter().enumerate() {
        dut.record_rx(entry);
        bytes[index] = entry as u8;
    }
    dut.feed(&bytes[..taken], now);

    // One buffer stays staged behind the one in flight, so the wire never idles at a boundary.
    if !dma.tx_busy() {
        dut.retire_tx();
    }
    if let Some(bytes) = dut.take_pending_tx() {
        // SAFETY: idle channel; static source stays initialized and unwritten until completion or successful abort.
        unsafe { dma.tx_start(bytes.as_ptr(), bytes.len()) };
    }
    dut.fill_tx(now);
}

struct Rp2350Hardware;

impl Hardware for Rp2350Hardware {
    fn now_us(&self) -> u32 {
        now_us()
    }

    fn set_baud(&mut self, baud: u32) -> Option<u32> {
        set_echo_baud(baud)
    }

    fn tx_abort(&mut self) -> bool {
        // SAFETY: responder dispatch does not overlap either streaming pass.
        unsafe { uart_dma() }.tx_abort().is_ok()
    }

    fn rx_abort_pending(&self) -> bool {
        // SAFETY: responder dispatch does not overlap either streaming pass.
        unsafe { uart_dma() }.rx_abort_pending()
    }

    fn abort_stuck(&self) -> u32 {
        // SAFETY: responder dispatch does not overlap either streaming pass.
        unsafe { uart_dma() }.abort_timeouts()
    }
}

#[entry]
fn main() -> ! {
    // Reset inherited USB state before changing clocks; this image never serves USB.
    pac::RESETS.reset().modify(|w| w.set_usbctrl(true));
    let p = embassy_rp::init(clock_config());
    // Clock initialization rewrites RESETS, so reassert the unused controller's reset.
    pac::RESETS.reset().modify(|w| w.set_usbctrl(true));
    pac::CLOCKS.clk_usb_ctrl().modify(|w| w.set_enable(false));

    let channels = rtt_init! {
        up: {
            0: {
                // NoBlockSkip never blocks, needed since a v1 window stops the host reading RTT.
                size: 1024,
                mode: ChannelMode::NoBlockSkip,
                name: "up"
            }
        }
        down: {
            0: {
                size: 64,
                name: "down"
            }
        }
    };
    let mut up = channels.up.0;
    let mut down = channels.down.0;
    up.write(proto::EVT_BOOT.as_bytes());
    up.write(b" ");
    up.write(NAME.as_bytes());
    up.write(b"\n");

    let mut led = Output::new(p.PIN_25, Level::Low);

    init_echo_uart(ECHO_BAUD);
    let _ = uart0_dma_arm();

    let mut line = proto::CommandLine::new();
    let mut tick = 0u32;
    let mut hardware = Rp2350Hardware;
    // SAFETY: taken once; static slot storage outlives all DMA activity and is never accessed separately.
    let tx = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUFFERS) };
    let mut dut = Dut::new(NAME, BAUD_MAX, tx);
    let mut reply = [0; v1::V1_LINE];
    loop {
        // Serviced every pass so the receive ring does not back up at line rate.
        if dut.abort_pending() || hardware.rx_abort_pending() {
            // Keep RTT available for an explicit abort retry without servicing retained buffers.
        } else if dut.armed() {
            uart0_stream(&mut dut);
        } else {
            uart0_drain(&mut dut);
        }

        let mut buf = [0u8; 16];
        let n = down.read(&mut buf);
        for &b in &buf[..n] {
            if let Some(command) = line.push(b) {
                up.write(dut.respond(&mut hardware, command, &mut reply));
            }
        }

        tick = tick.wrapping_add(1);
        if tick & 0x3_FFFF == 0 {
            led.toggle();
        }
    }
}
