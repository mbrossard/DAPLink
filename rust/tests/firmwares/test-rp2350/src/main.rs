//! RP2350 Pico 2 device-under-test image: flash pattern, LED, UART0 echo (GP0 TX / GP1 RX), RTT responder.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m_rt::entry;
use daplink_rp_dma_uart as uartdma;
use daplink_test_protocol as proto;
use daplink_test_protocol::v1::{self, Direction, Encoder, V1Command, V1Report, V1Start, Verifier};
use embassy_rp::clocks::CoreVoltage;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::TreqSel;
use panic_halt as _;
use rtt_target::{rtt_init, ChannelMode, UpChannel};

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

/// Ordinary zeroed static: a reset clears it, which is exactly what the reset test checks.
static STAMP: AtomicU32 = AtomicU32::new(0);

/// Boot baud of the echo UART; the `BAUD` verb reprograms it.
const ECHO_BAUD: u32 = 115_200;

/// clk_peri feeds the PL011; `embassy_rp::init`'s default config leaves it at clk_sys.
const CLK_PERI_HZ: u32 = 150_000_000;

/// Highest rate `BAUD` accepts.
const BAUD_MAX: u32 = pl011_baud::max_baud(CLK_PERI_HZ);

/// GPIO function-select value that routes a pin to its UART peripheral.
const FUNCSEL_UART: u8 = 2;

/// Echo-UART counters, bumped in the drain and reported by `STATS`.
static RX: AtomicU32 = AtomicU32::new(0);
static TX: AtomicU32 = AtomicU32::new(0);
static FERR: AtomicU32 = AtomicU32::new(0);
static OVR: AtomicU32 = AtomicU32::new(0);

/// Request 1.15 V, the documented core rail above 133 MHz: at the 1.10 V reset value the
/// 150 MHz PLL intermittently misses lock.
fn clock_config() -> embassy_rp::config::Config {
    let mut config = embassy_rp::config::Config::default();
    config.clocks.core_voltage = CoreVoltage::V1_15;
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
    uart0_dma_arm();
    Some(pl011_baud::realized(CLK_PERI_HZ, div))
}

/// Bump a counter; drain and responder both run in `main`, so plain load/store is safe here.
fn bump(counter: &AtomicU32) {
    bump_by(counter, 1);
}

/// Add `n` to a counter in one load/store pair.
fn bump_by(counter: &AtomicU32, n: u32) {
    let v = counter.load(Ordering::Relaxed).wrapping_add(n);
    counter.store(v, Ordering::Relaxed);
}

/// Bytes the verifier is fed at a time.
const V1_CHUNK: usize = 32;

/// Entries one receive pass takes before yielding to the responder.
const RX_BATCH: usize = 32;

/// Bytes per transmit transfer; two of them alternate so one fills while the other is on the wire.
const TX_CHUNK: usize = 256;

/// `tx_inflight` when the transmit channel owns neither buffer.
const NO_BUFFER: usize = 2;

/// UARTDR error bits, which ride the data because the ring stores the register verbatim.
/// Counted, never dropped: a corrupt byte must reach the verifier as a mismatch, not a gap.
const DR_FE: u16 = 1 << 8;
const DR_PE: u16 = 1 << 9;
const DR_BE: u16 = 1 << 10;
const DR_OE: u16 = 1 << 11;

/// UART0 over DMA channels 0 (receive) and 1 (transmit).
static mut RX_RING: uartdma::RxRing = uartdma::RxRing::new();
static mut UART_DMA: uartdma::UartDma =
    uartdma::UartDma::new(pac::UART0, 0, 1, TreqSel::UART0_RX, TreqSel::UART0_TX);

/// Arm the receive ring. Called after every divisor change, which is the only safe moment.
fn uart0_dma_arm() {
    // SAFETY: single-threaded, and the ring is a static that outlives every transfer.
    unsafe {
        let ring = &mut *core::ptr::addr_of_mut!(RX_RING);
        (*core::ptr::addr_of_mut!(UART_DMA)).arm(ring);
    }
}

fn uart_dma() -> &'static mut uartdma::UartDma {
    // SAFETY: single-threaded; nothing else holds a reference.
    unsafe { &mut *core::ptr::addr_of_mut!(UART_DMA) }
}

/// Free-running 1 MHz timer for the v1 engines, which expect a u32 that wraps.
fn now_us() -> u32 {
    pac::TIMER0.timerawl().read()
}

/// State for one v1 measurement run.
struct V1Run {
    /// The main loop pumps the stream instead of echoing.
    armed: bool,
    /// Grants ride the host's stream, so the transmitter's window follows the verifier.
    credit: bool,
    encoder: Option<Encoder>,
    verifier: Option<Verifier>,
    /// Two transmit buffers: the controller owns one while the encoder fills the other.
    tx: [[u8; TX_CHUNK]; 2],
    tx_len: [usize; 2],
    /// The buffer the controller owns, or 2 when it owns neither.
    tx_inflight: usize,
}

impl V1Run {
    const fn new() -> Self {
        Self {
            armed: false,
            credit: false,
            encoder: None,
            verifier: None,
            tx: [[0; TX_CHUNK]; 2],
            tx_len: [0; 2],
            tx_inflight: NO_BUFFER,
        }
    }

    /// Build the engines the mask asks for; the verifier's clock starts here, before the first byte.
    fn start(&mut self, start: &V1Start) {
        uart_dma().tx_abort();
        *self = Self {
            credit: start.credit,
            ..Self::new()
        };
        if start.dirs.tx() {
            let mut encoder = Encoder::new(
                Direction::DutToHost,
                start.seed_d2h,
                start.interval,
                v1::budget(start.budget_d2h),
            );
            if let Some(burst) = start.burst {
                encoder = encoder.with_burst(burst);
            }
            if start.credit {
                encoder = encoder.with_credit();
            }
            self.encoder = Some(encoder);
        }
        if start.dirs.rx() {
            let mut verifier = Verifier::new(
                Direction::HostToDut,
                start.seed_h2d,
                start.interval,
                v1::budget(start.budget_h2d),
            );
            if start.credit {
                verifier = verifier.with_credit();
            }
            verifier.arm(now_us());
            self.verifier = Some(verifier);
        }
        self.armed = true;
    }

    /// End the stream short of its budget, leaving the run armed so the close reaches the wire.
    fn close(&mut self) {
        if let Some(encoder) = self.encoder.as_mut() {
            encoder.close();
        }
    }

    /// Zero both engines and leave the run idle, so the echo comes back.
    fn reset(&mut self) {
        if let Some(encoder) = self.encoder.as_mut() {
            encoder.reset();
        }
        if let Some(verifier) = self.verifier.as_mut() {
            verifier.reset();
        }
        uart_dma().tx_abort();
        self.armed = false;
        self.tx_len = [0; 2];
        self.tx_inflight = NO_BUFFER;
    }

    fn feed(&mut self, bytes: &[u8], now_us: u32) {
        if let Some(verifier) = self.verifier.as_mut() {
            verifier.feed(bytes, now_us);
        }
    }

    /// The run's ledger, both halves and the UART error counters.
    fn report(&self) -> V1Report {
        let tx = self.encoder.as_ref().map(Encoder::counters);
        let rx = self.verifier.as_ref().map(Verifier::counters);
        V1Report::new(
            &tx.unwrap_or_default(),
            &rx.unwrap_or_default(),
            FERR.load(Ordering::Relaxed),
            OVR.load(Ordering::Relaxed),
            // Ring-lap loss is charged to `OVR`, so there is no separate drop site.
            0,
        )
    }
}

/// Echo up to one batch of the receive ring, one byte at a time.
fn uart0_drain() {
    let uart = pac::UART0;
    let dma = uart_dma();
    for _ in 0..dma.rx_resync() {
        bump(&OVR);
    }
    for _ in 0..RX_BATCH {
        let Some(entry) = dma.rx_next() else {
            break;
        };
        // The ring stores `UARTDR` verbatim, so bits 8..11 belong to this entry.
        bump(&RX);
        if entry & (DR_FE | DR_PE | DR_BE) != 0 {
            bump(&FERR);
        }
        if entry & DR_OE != 0 {
            bump(&OVR);
        }
        // Echoed a byte at a time by the CPU: this path is latency-bound, not throughput-bound.
        while uart.uartfr().read().txff() {}
        uart.uartdr().write(|w| w.set_data(entry as u8));
        bump(&TX);
    }
}

/// One pass of an armed v1 run: the ring into the verifier, the encoder into the transmit channel.
fn uart0_stream(run: &mut V1Run) {
    let now = now_us();
    let dma = uart_dma();

    if run.credit {
        let grant = run.verifier.as_ref().map_or(0, Verifier::credit);
        if let Some(encoder) = run.encoder.as_mut() {
            encoder.set_grant(grant);
        }
    }

    // One counter read and one cursor fold per pass; lapped entries are charged, not re-read.
    let mut batch = [0u16; RX_BATCH];
    let (taken, lapped) = dma.rx_read(&mut batch);
    bump_by(&OVR, lapped);
    let mut chunk = [0u8; V1_CHUNK];
    let mut len = 0;
    for &entry in &batch[..taken] {
        if entry & (DR_FE | DR_PE | DR_BE) != 0 {
            bump(&FERR);
        }
        if entry & DR_OE != 0 {
            bump(&OVR);
        }
        chunk[len] = entry as u8;
        len += 1;
        if len == chunk.len() {
            run.feed(&chunk, now);
            len = 0;
        }
    }
    run.feed(&chunk[..len], now);
    bump_by(&RX, taken as u32);

    // One buffer stays staged behind the one in flight, so the wire never idles at a boundary.
    if run.tx_inflight != NO_BUFFER && !dma.tx_busy() {
        // Counted as the transfer retires, so `tx` never leads the wire by a whole launch.
        bump_by(&TX, run.tx_len[run.tx_inflight] as u32);
        run.tx_len[run.tx_inflight] = 0;
        run.tx_inflight = NO_BUFFER;
    }
    if run.tx_inflight == NO_BUFFER {
        for slot in 0..2 {
            if run.tx_len[slot] != 0 {
                // SAFETY: the channel is idle and this buffer is not refilled until it retires.
                unsafe { dma.tx_start(run.tx[slot].as_ptr(), run.tx_len[slot]) };
                run.tx_inflight = slot;
                break;
            }
        }
    }
    if let Some(encoder) = run.encoder.as_mut() {
        for slot in 0..2 {
            if slot != run.tx_inflight && run.tx_len[slot] == 0 {
                run.tx_len[slot] = encoder.fill(&mut run.tx[slot], now);
            }
        }
    }
}

fn stats() -> proto::Stats {
    proto::Stats {
        rx: RX.load(Ordering::Relaxed),
        tx: TX.load(Ordering::Relaxed),
        ferr: FERR.load(Ordering::Relaxed),
        ovr: OVR.load(Ordering::Relaxed),
        // Ring-lap loss is charged to `OVR`, so there is no separate drop site.
        drop: 0,
        stuck: uart_dma().abort_stuck() as u32,
    }
}

fn stats_reset() {
    for counter in [&RX, &TX, &FERR, &OVR] {
        counter.store(0, Ordering::Relaxed);
    }
}

fn write_line(up: &mut UpChannel, text: &str) {
    up.write(text.as_bytes());
    up.write(b"\n");
}

fn respond(up: &mut UpChannel, run: &mut V1Run, request: &[u8]) {
    let mut buf = [0u8; proto::MAX_LINE];
    let request = core::str::from_utf8(request).unwrap_or("");
    // The v1 verbs are outside `proto::Command`, so their parser runs first.
    if let Some(command) = v1::parse(request) {
        return v1_respond(up, run, command);
    }
    let reply = match proto::parse(request) {
        Some(proto::Command::Ping) => return write_line(up, proto::OK_PONG),
        Some(proto::Command::Id) => {
            proto::id_response(&mut buf, NAME, STAMP.load(Ordering::Relaxed))
        }
        Some(proto::Command::Stamp(n)) => {
            STAMP.store(n, Ordering::Relaxed);
            proto::stamp_response(&mut buf, n)
        }
        Some(proto::Command::Baud(n)) => match set_echo_baud(n) {
            Some(realized) => proto::baud_response(&mut buf, realized),
            None => return write_line(up, proto::ERR_UNIMPLEMENTED),
        },
        Some(proto::Command::BaudMax) => proto::baud_max_response(&mut buf, BAUD_MAX),
        Some(proto::Command::Stats) => proto::stats_response(&mut buf, &stats()),
        Some(proto::Command::StatsReset) => {
            stats_reset();
            return write_line(up, proto::OK_STATS_RESET);
        }
        // The RP2350 routes no SWO signal to a package pin, so the SWO verbs are unimplemented.
        Some(
            proto::Command::SwoEmit(_) | proto::Command::SwoCfg(_) | proto::Command::SwoStream(_),
        ) => return write_line(up, proto::ERR_UNIMPLEMENTED),
        None => return write_line(up, proto::ERR_UNIMPLEMENTED),
    };
    up.write(reply.as_bytes());
}

fn v1_respond(up: &mut UpChannel, run: &mut V1Run, command: V1Command) {
    match command {
        V1Command::Start(start) => {
            // The window owns the error counters, so the run starts them clean.
            stats_reset();
            run.start(&start);
            write_line(up, v1::OK_V1_START);
        }
        // Closes the stream on the next frame slot so the receiver still gets an end-of-stream frame.
        V1Command::Stop => {
            run.close();
            write_line(up, v1::OK_V1_STOPPED);
        }
        V1Command::Report => {
            // The ledger outgrows `proto::MAX_LINE`.
            let mut buf = [0u8; v1::V1_LINE];
            up.write(v1::report_response(&mut buf, &run.report()).as_bytes());
        }
        V1Command::Reset => {
            run.reset();
            stats_reset();
            write_line(up, v1::OK_V1_RESET);
        }
    }
}

#[entry]
fn main() -> ! {
    // Brings the crystal, PLLs and peripheral clocks up; the UART divisors depend on it.
    let p = embassy_rp::init(clock_config());

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

    // GP25 is the Pico 2's onboard LED.
    let mut led = Output::new(p.PIN_25, Level::Low);

    init_echo_uart(ECHO_BAUD);
    uart0_dma_arm();

    let mut line = [0u8; proto::MAX_LINE];
    let mut len = 0usize;
    let mut tick = 0u32;
    let mut run = V1Run::new();
    loop {
        // Serviced every pass so the receive ring does not back up at line rate.
        if run.armed {
            uart0_stream(&mut run);
        } else {
            uart0_drain();
        }

        let mut buf = [0u8; 16];
        let n = down.read(&mut buf);
        for &b in &buf[..n] {
            if b == b'\n' {
                respond(&mut up, &mut run, &line[..len]);
                len = 0;
            } else if len < line.len() {
                line[len] = b;
                len += 1;
            } else {
                // Overlong line: drop it and resync at the next newline.
                len = 0;
            }
        }

        tick = tick.wrapping_add(1);
        if tick & 0x3_FFFF == 0 {
            led.toggle();
        }
    }
}
