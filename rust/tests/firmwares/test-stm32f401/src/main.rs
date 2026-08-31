//! Device-under-test image for the STM32F401 rig: flash data, LED, echo UART, v1, RTT responder.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::{DCB, ITM, TPIU};
use cortex_m_rt::entry;
use daplink_test_protocol::{
    self as proto,
    v1::{self, Direction, Encoder, V1Command, V1Report, V1Start, Verifier},
};
use panic_halt as _;
use rtt_target::{rtt_init, ChannelMode, UpChannel};
use stm32_usart_brr::{usart_baud_realized, usart_brr};

const NAME: &str = "test-stm32f401";

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

/// Echo-UART counters reported by `STATS`, all wrapping on overflow.
static ECHO_RX: AtomicU32 = AtomicU32::new(0);
static ECHO_TX: AtomicU32 = AtomicU32::new(0);
static ECHO_FERR: AtomicU32 = AtomicU32::new(0);
static ECHO_OVR: AtomicU32 = AtomicU32::new(0);
static ECHO_DROP: AtomicU32 = AtomicU32::new(0);
/// Transmit-DMA abort waits that expired with the stream still busy; sticky, as `STATS` reports.
static TX_DMA_STUCK: AtomicU32 = AtomicU32::new(0);

const RCC_CR: *mut u32 = 0x4002_3800 as *mut u32;
const RCC_PLLCFGR: *mut u32 = 0x4002_3804 as *mut u32;
const RCC_CFGR: *mut u32 = 0x4002_3808 as *mut u32;
const FLASH_ACR: *mut u32 = 0x4002_3C00 as *mut u32;
const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32;
const RCC_APB1ENR: *mut u32 = 0x4002_3840 as *mut u32;
const GPIOA_MODER: *mut u32 = 0x4002_0000 as *mut u32;
const GPIOA_AFRL: *mut u32 = 0x4002_0020 as *mut u32;
const GPIOC_MODER: *mut u32 = 0x4002_0800 as *mut u32;
const GPIOC_ODR: *mut u32 = 0x4002_0814 as *mut u32;
const USART2_SR: *mut u32 = 0x4000_4400 as *mut u32;
const USART2_DR: *mut u32 = 0x4000_4404 as *mut u32;
const USART2_BRR: *mut u32 = 0x4000_4408 as *mut u32;
const USART2_CR1: *mut u32 = 0x4000_440C as *mut u32;
const USART2_CR3: *mut u32 = 0x4000_4414 as *mut u32;
/// USART2_RX is DMA1 stream 5 channel 4; the stream block sits at 0x10 + 0x18 * 5 from the base.
const DMA1_HIFCR: *mut u32 = 0x4002_600C as *mut u32;
const DMA1_S5CR: *mut u32 = 0x4002_6088 as *mut u32;
const DMA1_S5NDTR: *mut u32 = 0x4002_608C as *mut u32;
const DMA1_S5PAR: *mut u32 = 0x4002_6090 as *mut u32;
const DMA1_S5M0AR: *mut u32 = 0x4002_6094 as *mut u32;
const DMA1_HISR: *const u32 = 0x4002_6004 as *const u32;
const DMA1_S6CR: *mut u32 = 0x4002_60A0 as *mut u32;
const DMA1_S6NDTR: *mut u32 = 0x4002_60A4 as *mut u32;
const DMA1_S6PAR: *mut u32 = 0x4002_60A8 as *mut u32;
const DMA1_S6M0AR: *mut u32 = 0x4002_60AC as *mut u32;

/// Stop the transmit stream and drop what it had left: its ring dies with the run. A wedged
/// stream expires the wait and is reported through `STATS stuck` rather than hanging the loop.
unsafe fn tx_dma_stop() {
    DMA1_S6CR.write_volatile(0);
    let mut spins = 0u32;
    while DMA1_S6CR.read_volatile() & 1 != 0 {
        spins += 1;
        if spins >= DMA_ABORT_SPIN_LIMIT {
            TX_DMA_STUCK.fetch_add(1, Ordering::Relaxed);
            break;
        }
    }
    DMA1_HIFCR.write_volatile(0b11_1101 << 16); // stream 6's flags
}
const TIM2_CR1: *mut u32 = 0x4000_0000 as *mut u32;
const TIM2_EGR: *mut u32 = 0x4000_0014 as *mut u32;
const TIM2_CNT: *mut u32 = 0x4000_0024 as *mut u32;
const TIM2_PSC: *mut u32 = 0x4000_0028 as *mut u32;
const TIM2_ARR: *mut u32 = 0x4000_002C as *mut u32;
/// DBGMCU_CR: bit 5 TRACE_IOEN, bits [7:6] TRACE_MODE (00 = asynchronous SWO).
const DBGMCU_CR: *mut u32 = 0xE004_2004 as *mut u32;

/// Core clock (HCLK) after [`clock_init`]: HSE 25 MHz -> PLL /25 x336 /4.
const CPU_HZ: u32 = 84_000_000;
/// APB1 clock feeding USART2 (HCLK 84 MHz / 2), the reference for every echo baud divisor.
const APB1_HZ: u32 = CPU_HZ / 2;
/// Clock the APB1 timers see: the APB1 prescaler is not 1, so TIM2 runs at twice PCLK1.
const APB1_TIMER_HZ: u32 = APB1_HZ * 2;
/// Highest echo baud this image programs: BRR divisor 1.0 at oversampling 8 (RM0368 §19.3.4).
const ECHO_BAUD_MAX: u32 = APB1_HZ / 8;
/// Lowest echo baud this image programs: any less and the rounded divisor overflows 16-bit BRR.
const ECHO_BAUD_MIN: u32 = APB1_HZ / 0xFFFF + 1;

/// Trace clock feeding the TPIU async prescaler: HCLK, this part having no TRACECLKIN.
const TRACE_CLK_HZ: u32 = CPU_HZ;
/// Boot-time SWO/ITM baud (84 MHz / 84); `SWO CFG` reprograms the rate afterward.
const SWO_BAUD: u32 = 1_000_000;
/// Boot-time TPIU async prescaler divisor for [`SWO_BAUD`].
const SWO_BAUD_DIVISOR: u32 = TRACE_CLK_HZ / SWO_BAUD;
/// Widest async prescaler divisor: `TPIU_ACPR.PRESCALER` is 13 bits, so `divisor - 1` fits 0x1FFF.
const SWO_DIVISOR_MAX: u32 = 0x2000;
/// Highest SWO baud this image programs: prescaler divisor 1.
const SWO_BAUD_MAX: u32 = TRACE_CLK_HZ;
/// Lowest SWO baud this image programs: any less and the divisor overflows the 13-bit prescaler.
const SWO_BAUD_MIN: u32 = TRACE_CLK_HZ / SWO_DIVISOR_MAX + 1;
const _: () = assert!(SWO_BAUD_DIVISOR <= SWO_DIVISOR_MAX);
/// Bound on the per-byte ITM FIFO-ready poll, so a stalled trace sink cannot hang the loop.
const FIFO_READY_SPIN_LIMIT: u32 = 1_000_000;
/// Bound on a DMA abort wait, so a stream that never lets go cannot hang the loop either.
const DMA_ABORT_SPIN_LIMIT: u32 = 100_000;

/// Receive ring the DMA controller fills; it holds milliseconds of line at the fastest rate.
const RX_RING: usize = 2048;

/// Bytes one pump pass moves in each direction before the loop goes back to servicing RTT.
const V1_PASS: usize = 64;
/// Staging ring between the encoder and the single-byte transmit register; a power of two.
const V1_TX_RING: usize = 512;
/// Bytes the encoder generates in one pass, which bounds how long the ring goes unserviced.
const V1_TX_SLICE: usize = 64;
/// Bytes one verifier pass copies out of the receive ring at a time; divides [`RX_RING`].
const V1_RX_BATCH: usize = 128;
const _: () = assert!(RX_RING.is_multiple_of(V1_RX_BATCH));

/// ITM/SWO trace over PB3 (shared with JTDO), which comes out of reset in its AF0 debug function
/// and so needs no pin setup beyond the trace enable below.
struct SwoOut {
    dcb: DCB,
    tpiu: TPIU,
    itm: ITM,
}

impl SwoOut {
    /// Enable trace and bring up the TPIU (async NRZ/SWO) and ITM (stimulus port 0) once at boot.
    fn configure(&mut self) {
        // DEMCR.TRCENA: required for ITM/TPIU/DWT to function.
        self.dcb.enable_trace();
        // Route the trace pin and select asynchronous (SWO) mode.
        unsafe {
            let cr = DBGMCU_CR.read_volatile();
            DBGMCU_CR.write_volatile((cr & !(0b11 << 6)) | (1 << 5));
        }
        // SAFETY: this image owns the DCB/TPIU/ITM tokens, so no write here aliases.
        unsafe {
            self.tpiu.sppr.write(2); // asynchronous NRZ (UART) encoding
            self.tpiu.acpr.write(SWO_BAUD_DIVISOR - 1);
            // No ETM in this design: disable the formatter so ITM packets are emitted unframed.
            self.tpiu.ffcr.modify(|ffcr| ffcr & !(1 << 1));
            self.itm.lar.write(0xC5AC_CE55); // CoreSight lock-access unlock
            self.itm.ter[0].modify(|ter| ter | 1); // enable stimulus port 0
            self.itm.tcr.modify(|tcr| tcr | 1); // ITMENA
        }
    }

    /// Rewrite the TPIU async prescaler for a new SWO baud, returning the rate the rounded
    /// divisor realizes; `None` outside [`SWO_BAUD_MIN`]..=[`SWO_BAUD_MAX`], where the prescaler
    /// would truncate and the reported rate would not be the rate on the wire.
    fn set_baud(&mut self, baud: u32) -> Option<u32> {
        if !(SWO_BAUD_MIN..=SWO_BAUD_MAX).contains(&baud) {
            return None;
        }
        let divisor = (TRACE_CLK_HZ + baud / 2) / baud;
        // SAFETY: as in `configure`, this image owns the TPIU token exclusively.
        unsafe {
            self.tpiu.acpr.write(divisor - 1);
        }
        Some(TRACE_CLK_HZ / divisor)
    }

    /// Write each byte to ITM stimulus port 0, which frames it as `0x01` then the byte.
    fn emit(&mut self, bytes: &[u8]) {
        for &b in bytes {
            if !self.wait_fifo() {
                return;
            }
            self.itm.stim[0].write_u8(b);
        }
    }

    /// Write one word to ITM stimulus port 0, framed as `0x03` then the word, LSB first.
    fn emit_word(&mut self, word: u32) -> bool {
        if !self.wait_fifo() {
            return false;
        }
        self.itm.stim[0].write_u32(word);
        true
    }

    /// Spin until the stimulus FIFO accepts a write, bounded by [`FIFO_READY_SPIN_LIMIT`].
    fn wait_fifo(&mut self) -> bool {
        let mut spins = 0u32;
        while !self.itm.stim[0].is_fifo_ready() {
            spins += 1;
            if spins >= FIFO_READY_SPIN_LIMIT {
                return false;
            }
        }
        true
    }
}

/// One record of the continuous stream: the shared magic word then the sequence number. A refused
/// magic word takes the sequence word with it, so a stalled sink costs a gap, not a resync.
fn emit_record(swo: &mut SwoOut, seq: u32) {
    if swo.emit_word(proto::SWO_STREAM_MAGIC) {
        swo.emit_word(seq);
    }
}

/// USART2 on PA2 (TX) / PA3 (RX), 115200 8N1 off APB1 = 42 MHz (HCLK 84 MHz / 2).
unsafe fn usart2_init() {
    RCC_AHB1ENR.write_volatile(RCC_AHB1ENR.read_volatile() | 1); // GPIOAEN
    RCC_APB1ENR.write_volatile(RCC_APB1ENR.read_volatile() | (1 << 17)); // USART2EN
    let moder = GPIOA_MODER.read_volatile() & !(0b1111 << 4);
    GPIOA_MODER.write_volatile(moder | (0b10 << 4) | (0b10 << 6)); // PA2/PA3 alternate function
    let afrl = GPIOA_AFRL.read_volatile() & !(0xFF << 8);
    GPIOA_AFRL.write_volatile(afrl | (7 << 8) | (7 << 12)); // AF7 = USART2
    let (_, brr) = usart_brr(APB1_HZ, 115_200); // oversampling 16, APB1 = 42 MHz
    USART2_BRR.write_volatile(brr);
    USART2_CR1.write_volatile((1 << 13) | (1 << 3) | (1 << 2)); // UE | TE | RE
}

/// Reprogram USART2 to `baud`, which the caller must have held to
/// [`ECHO_BAUD_MIN`]..=[`ECHO_BAUD_MAX`].
unsafe fn usart2_set_baud(baud: u32) -> u32 {
    let (over8, brr) = usart_brr(APB1_HZ, baud);
    let cr1 = USART2_CR1.read_volatile();
    USART2_CR1.write_volatile(cr1 & !(1 << 13)); // clear UE
    USART2_BRR.write_volatile(brr);
    let cr1 = (cr1 & !(1 << 15)) | if over8 { 1 << 15 } else { 0 }; // OVER8
    USART2_CR1.write_volatile(cr1 | (1 << 13)); // restore, UE set
    usart_baud_realized(APB1_HZ, over8, brr)
}

/// TIM2 free-running at 1 MHz; its 32-bit counter wraps, as the engines' arithmetic expects.
unsafe fn timer_init() {
    RCC_APB1ENR.write_volatile(RCC_APB1ENR.read_volatile() | 1); // TIM2EN
    TIM2_PSC.write_volatile(APB1_TIMER_HZ / 1_000_000 - 1);
    TIM2_ARR.write_volatile(u32::MAX);
    TIM2_EGR.write_volatile(1); // UG: load the prescaler
    TIM2_CR1.write_volatile(1); // CEN
}

fn micros() -> u32 {
    unsafe { TIM2_CNT.read_volatile() }
}

/// Snapshot of the echo counters.
fn echo_stats() -> proto::Stats {
    proto::Stats {
        rx: ECHO_RX.load(Ordering::Relaxed),
        tx: ECHO_TX.load(Ordering::Relaxed),
        ferr: ECHO_FERR.load(Ordering::Relaxed),
        ovr: ECHO_OVR.load(Ordering::Relaxed),
        drop: ECHO_DROP.load(Ordering::Relaxed),
        // The echo path has no DMA to abort; the v1 transmit path does.
        stuck: TX_DMA_STUCK.load(Ordering::Relaxed),
    }
}

fn echo_stats_reset() {
    ECHO_RX.store(0, Ordering::Relaxed);
    ECHO_TX.store(0, Ordering::Relaxed);
    ECHO_FERR.store(0, Ordering::Relaxed);
    ECHO_OVR.store(0, Ordering::Relaxed);
    ECHO_DROP.store(0, Ordering::Relaxed);
}

/// Bring SYSCLK up from HSE 25 MHz through the PLL to 84 MHz, before any peripheral setup.
unsafe fn clock_init() {
    FLASH_ACR.write_volatile(2 | (1 << 8) | (1 << 9) | (1 << 10)); // 2 wait states, PRFTEN|ICEN|DCEN
    RCC_CR.write_volatile(RCC_CR.read_volatile() | (1 << 16)); // HSEON
    while RCC_CR.read_volatile() & (1 << 17) == 0 {} // wait HSERDY
    RCC_PLLCFGR.write_volatile(25 | (336 << 6) | (0b01 << 16) | (1 << 22) | (7 << 24)); // HSE/25*336/4, PLLQ=7
    let cfgr = (RCC_CFGR.read_volatile() & !0xFCF0) | (0b100 << 10); // AHB /1, APB1 /2, APB2 /1
    RCC_CFGR.write_volatile(cfgr);
    RCC_CR.write_volatile(RCC_CR.read_volatile() | (1 << 24)); // PLLON
    while RCC_CR.read_volatile() & (1 << 25) == 0 {} // wait PLLRDY
    RCC_CFGR.write_volatile((RCC_CFGR.read_volatile() & !0b11) | 0b10); // SW = PLL
    while (RCC_CFGR.read_volatile() >> 2) & 0b11 != 0b10 {} // wait SWS = PLL
}

/// The DMA receive ring and how much software has taken: the controller writes unasked, so loss
/// is visible only as a write past what was still unread.
struct Rx {
    buf: [u8; RX_RING],
    /// Bytes the controller has written, accumulated across its wraps.
    written: u64,
    /// Bytes handed to a consumer.
    read: u64,
    /// Where the controller stood at the last poll, as an offset into the ring.
    at: usize,
    /// Error flags the last poll saw, so an episode is counted once and not every pass.
    err: u32,
}

impl Rx {
    const fn new() -> Self {
        Rx {
            buf: [0; RX_RING],
            written: 0,
            read: 0,
            at: 0,
            err: 0,
        }
    }

    /// Arm the stream over this ring; the buffer must not move afterward.
    unsafe fn start(&mut self) {
        RCC_AHB1ENR.write_volatile(RCC_AHB1ENR.read_volatile() | (1 << 21)); // DMA1EN
        DMA1_S5CR.write_volatile(0);
        while DMA1_S5CR.read_volatile() & 1 != 0 {}
        DMA1_HIFCR.write_volatile(0b11_1101 << 6); // stream 5's flags
        DMA1_S5PAR.write_volatile(USART2_DR as u32);
        DMA1_S5M0AR.write_volatile(self.buf.as_ptr() as u32);
        DMA1_S5NDTR.write_volatile(RX_RING as u32);
        // Channel 4, memory incrementing, circular, peripheral to memory, byte wide, high priority.
        DMA1_S5CR.write_volatile((0b100 << 25) | (0b10 << 16) | (1 << 10) | (1 << 8) | 1);
        USART2_CR3.write_volatile(USART2_CR3.read_volatile() | (1 << 6) | (1 << 7)); // DMAR, DMAT
        self.written = 0;
        self.read = 0;
        self.at = 0;
        self.err = 0;
    }

    /// Take the controller's position, and charge anything it wrote over that was still unread.
    fn poll(&mut self) {
        // This read arms the clear the controller's next DR read completes; edges are episodes.
        let sr = unsafe { USART2_SR.read_volatile() } & ((1 << 1) | (1 << 3));
        let fresh = sr & !self.err;
        self.err = sr;
        if fresh & (1 << 1) != 0 {
            ECHO_FERR.fetch_add(1, Ordering::Relaxed);
        }
        if fresh & (1 << 3) != 0 {
            ECHO_OVR.fetch_add(1, Ordering::Relaxed);
        }

        let pos = RX_RING - unsafe { DMA1_S5NDTR.read_volatile() } as usize;
        self.written += ((pos + RX_RING - self.at) % RX_RING) as u64;
        self.at = pos;
        let unread = self.written - self.read;
        if unread > RX_RING as u64 {
            ECHO_DROP.fetch_add((unread - RX_RING as u64) as u32, Ordering::Relaxed);
            self.read = self.written - RX_RING as u64;
        }
    }

    /// Copy the next unread run into `out`, stopping at the ring's seam; zero when none waits.
    fn read(&self, out: &mut [u8]) -> usize {
        let unread = (self.written - self.read) as usize;
        let at = (self.read % RX_RING as u64) as usize;
        let n = unread.min(RX_RING - at).min(out.len());
        for (i, slot) in out[..n].iter_mut().enumerate() {
            // SAFETY: in bounds by the clamp; volatile because the controller's write is not.
            *slot = unsafe { core::ptr::read_volatile(self.buf.as_ptr().add(at + i)) };
        }
        n
    }

    fn consume(&mut self, n: usize) {
        self.read += n as u64;
        ECHO_RX.fetch_add(n as u32, Ordering::Relaxed);
    }

    /// Drop what is buffered without counting it as loss: a rate change invalidates it.
    fn resync(&mut self) {
        self.poll();
        self.read = self.written;
    }
}

/// Echo what the controller received, one byte per free transmitter, so a backlog drains at
/// line rate rather than holding the responder loop.
unsafe fn usart2_echo(rx: &mut Rx) {
    rx.poll();
    let mut byte = [0u8; 1];
    for _ in 0..V1_PASS {
        if USART2_SR.read_volatile() & (1 << 7) == 0 {
            break; // TXE clear: the transmitter is still busy
        }
        if rx.read(&mut byte) == 0 {
            break;
        }
        USART2_DR.write_volatile(u32::from(byte[0]));
        rx.consume(1);
        ECHO_TX.fetch_add(1, Ordering::Relaxed);
    }
}

/// An armed v1 run: transmitter, verifier, and the staging ring to the transmit register.
struct Run {
    /// Cleared by `V1 RESET`, which returns the UART to the echo path.
    active: bool,
    credit: bool,
    enc: Option<Encoder>,
    ver: Option<Verifier>,
    tx: [u8; V1_TX_RING],
    /// Free-running ring cursors; their difference is what is staged and not yet transmitted.
    tx_head: usize,
    tx_tail: usize,
    /// Bytes the transmit stream is carrying right now; zero when it is idle.
    tx_dma: usize,
}

impl Run {
    /// Build the two engines a `V1 START` asks for; the DUT transmits `d2h` and verifies `h2d`.
    fn new(start: &V1Start) -> Self {
        let enc = start.dirs.tx().then(|| {
            let mut enc = Encoder::new(
                Direction::DutToHost,
                start.seed_d2h,
                start.interval,
                v1::budget(start.budget_d2h),
            );
            if let Some(burst) = start.burst {
                enc = enc.with_burst(burst);
            }
            if start.credit {
                enc = enc.with_credit();
            }
            enc
        });
        let ver = start.dirs.rx().then(|| {
            let ver = Verifier::new(
                Direction::HostToDut,
                start.seed_h2d,
                start.interval,
                v1::budget(start.budget_h2d),
            );
            if start.credit {
                ver.with_credit()
            } else {
                ver
            }
        });
        Self {
            active: true,
            credit: start.credit,
            enc,
            ver,
            tx: [0; V1_TX_RING],
            tx_head: 0,
            tx_tail: 0,
            tx_dma: 0,
        }
    }

    /// One bounded pass over both directions; never blocks so the caller keeps polling RTT.
    fn service(&mut self, rx: &mut Rx) {
        if self.credit {
            if let (Some(enc), Some(ver)) = (self.enc.as_mut(), self.ver.as_ref()) {
                enc.set_grant(ver.credit());
            }
        }
        self.pump_rx(rx);
        let mut dry = false;
        for _ in 0..V1_PASS {
            let mut moved = unsafe { self.pump_tx() };
            if !dry && V1_TX_RING - self.staged() >= V1_TX_SLICE {
                if self.stage() {
                    moved = true;
                } else {
                    dry = true; // out of budget, credit, or inside a burst gap
                }
            }
            if !moved {
                break;
            }
        }
    }

    /// Verify everything received; each run carries one timestamp, so a reported gap is between
    /// passes, not bytes.
    fn pump_rx(&mut self, rx: &mut Rx) {
        rx.poll();
        let now = micros();
        let mut batch = [0u8; V1_RX_BATCH];
        for _ in 0..RX_RING / V1_RX_BATCH {
            let n = rx.read(&mut batch);
            if n == 0 {
                break;
            }
            if let Some(ver) = self.ver.as_mut() {
                ver.feed(&batch[..n], now);
            }
            rx.consume(n);
        }
    }

    /// Bytes generated and not yet handed to the transmit register.
    fn staged(&self) -> usize {
        self.tx_tail.wrapping_sub(self.tx_head)
    }

    /// Generate one slice into the ring; false means the encoder has nothing to give right now.
    fn stage(&mut self) -> bool {
        let at = self.tx_tail % V1_TX_RING;
        let room = (V1_TX_RING - self.staged())
            .min(V1_TX_RING - at)
            .min(V1_TX_SLICE);
        let Some(enc) = self.enc.as_mut() else {
            return false;
        };
        let n = enc.fill(&mut self.tx[at..at + room], micros());
        self.tx_tail = self.tx_tail.wrapping_add(n);
        n > 0
    }

    /// Retire a finished transfer, then arm the next contiguous run of staged bytes. One shot,
    /// not circular: a ring the encoder falls behind would otherwise be sent twice.
    unsafe fn pump_tx(&mut self) -> bool {
        if self.tx_dma != 0 {
            if DMA1_HISR.read_volatile() & (1 << 21) == 0 {
                return false; // TCIF6 clear: still transmitting
            }
            DMA1_HIFCR.write_volatile(0b11_1101 << 16);
            self.tx_head = self.tx_head.wrapping_add(self.tx_dma);
            ECHO_TX.fetch_add(self.tx_dma as u32, Ordering::Relaxed);
            self.tx_dma = 0;
        }
        let staged = self.staged();
        if staged == 0 {
            return false;
        }
        let at = self.tx_head % V1_TX_RING;
        let n = staged.min(V1_TX_RING - at);
        tx_dma_stop();
        DMA1_S6PAR.write_volatile(USART2_DR as u32);
        DMA1_S6M0AR.write_volatile(self.tx.as_ptr().add(at) as u32);
        DMA1_S6NDTR.write_volatile(n as u32);
        // Channel 4, memory incrementing, memory to peripheral, byte wide, high priority.
        DMA1_S6CR.write_volatile((0b100 << 25) | (0b10 << 16) | (1 << 10) | (0b01 << 6) | 1);
        self.tx_dma = n;
        true
    }

    /// The run's ledger; a half the mask left unarmed reports zeroes.
    fn report(&self, ferr: u32, ovr: u32, drop: u32) -> V1Report {
        let tx = self.enc.as_ref().map(Encoder::counters).unwrap_or_default();
        let rx = self
            .ver
            .as_ref()
            .map(Verifier::counters)
            .unwrap_or_default();
        V1Report::new(&tx, &rx, ferr, ovr, drop)
    }

    /// End the stream short of its budget, leaving the run active so the close reaches the wire.
    fn close(&mut self) {
        if let Some(enc) = self.enc.as_mut() {
            enc.close();
        }
    }

    /// Zero both engines, keeping the configuration `V1 START` chose.
    fn reset(&mut self) {
        if let Some(enc) = self.enc.as_mut() {
            enc.reset();
        }
        if let Some(ver) = self.ver.as_mut() {
            ver.reset();
        }
        unsafe { tx_dma_stop() };
        self.tx_dma = 0;
        self.tx_head = 0;
        self.tx_tail = 0;
    }
}

fn decline(up: &mut UpChannel) {
    up.write(proto::ERR_UNIMPLEMENTED.as_bytes());
    up.write(b"\n");
}

/// Serve one v1 verb; nothing here waits on the wire, since a run is pumped from the main loop.
fn v1_request(up: &mut UpChannel, run: &mut Option<Run>, rx: &mut Rx, cmd: V1Command) {
    match cmd {
        V1Command::Start(start) => {
            // The old ledger, and whatever the ring caught before the run, belong to the last run.
            echo_stats_reset();
            rx.resync();
            // Nothing may still be reading the outgoing run's ring once this one replaces it.
            unsafe { tx_dma_stop() };
            let mut armed = Run::new(&start);
            if let Some(ver) = armed.ver.as_mut() {
                ver.arm(micros());
            }
            *run = Some(armed);
            up.write(v1::OK_V1_START.as_bytes());
            up.write(b"\n");
        }
        V1Command::Stop => {
            // Closes at the next frame slot, so the receiver still gets an end-of-stream frame.
            if let Some(armed) = run {
                armed.close();
            }
            up.write(v1::OK_V1_STOPPED.as_bytes());
            up.write(b"\n");
        }
        V1Command::Report => {
            let stats = echo_stats();
            let report = match run.as_ref() {
                Some(armed) => armed.report(stats.ferr, stats.ovr, stats.drop),
                None => V1Report {
                    ferr: stats.ferr,
                    ovr: stats.ovr,
                    drop: stats.drop,
                    ..V1Report::default()
                },
            };
            // The ledger outgrows the buffer every other reply uses.
            let mut buf = [0u8; v1::V1_LINE];
            up.write(v1::report_response(&mut buf, &report).as_bytes());
        }
        V1Command::Reset => {
            echo_stats_reset();
            rx.resync();
            if let Some(armed) = run {
                armed.active = false;
                armed.reset();
            }
            up.write(v1::OK_V1_RESET.as_bytes());
            up.write(b"\n");
        }
    }
}

fn respond(
    up: &mut UpChannel,
    swo: &mut SwoOut,
    streaming: &mut bool,
    run: &mut Option<Run>,
    rx: &mut Rx,
    request: &[u8],
) {
    let line = core::str::from_utf8(request).ok();
    // The two parsers partition the verb space; the base one leaves `V1` lines alone.
    if let Some(cmd) = line.and_then(v1::parse) {
        return v1_request(up, run, rx, cmd);
    }
    let mut buf = [0u8; proto::MAX_LINE];
    let reply = match line.and_then(proto::parse) {
        Some(proto::Command::Ping) => {
            up.write(proto::OK_PONG.as_bytes());
            up.write(b"\n");
            return;
        }
        Some(proto::Command::Id) => {
            proto::id_response(&mut buf, NAME, STAMP.load(Ordering::Relaxed))
        }
        Some(proto::Command::Stamp(n)) => {
            STAMP.store(n, Ordering::Relaxed);
            proto::stamp_response(&mut buf, n)
        }
        Some(proto::Command::Baud(n)) => {
            if !(ECHO_BAUD_MIN..=ECHO_BAUD_MAX).contains(&n) {
                // Outside the divisor's reach: decline and leave the UART running as it was.
                return decline(up);
            }
            let realized = unsafe { usart2_set_baud(n) };
            rx.resync();
            proto::baud_response(&mut buf, realized)
        }
        Some(proto::Command::BaudMax) => proto::baud_max_response(&mut buf, ECHO_BAUD_MAX),
        Some(proto::Command::Stats) => proto::stats_response(&mut buf, &echo_stats()),
        Some(proto::Command::StatsReset) => {
            // Whatever the ring caught while the line settled is nobody's data.
            echo_stats_reset();
            rx.resync();
            up.write(proto::OK_STATS_RESET.as_bytes());
            up.write(b"\n");
            return;
        }
        Some(proto::Command::SwoEmit(tag)) => {
            let mut payload = [0u8; proto::SWO_PAYLOAD_LEN];
            payload[..proto::SWO_PAYLOAD_HEAD.len()].copy_from_slice(&proto::SWO_PAYLOAD_HEAD);
            payload[proto::SWO_PAYLOAD_LEN - 1] = tag;
            swo.emit(&payload);
            proto::swo_emit_response(&mut buf, tag)
        }
        Some(proto::Command::SwoStream(on)) => {
            *streaming = on;
            proto::swo_stream_response(&mut buf, on)
        }
        Some(proto::Command::SwoCfg(baud)) => match swo.set_baud(baud) {
            Some(realized) => proto::swo_cfg_response(&mut buf, realized),
            None => return decline(up),
        },
        None => return decline(up),
    };
    up.write(reply.as_bytes());
}

#[entry]
fn main() -> ! {
    unsafe { clock_init() }; // HSE 25 MHz -> 84 MHz SYSCLK, before any peripheral setup

    // NoBlockSkip keeps a write during a measurement window from blocking on an unpolled channel.
    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
                mode: ChannelMode::NoBlockSkip,
                name: "up"
            }
        }
        down: {
            0: {
                size: 128,
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

    // PC13 (the board's active-low LED) as a push-pull output; GPIOB clocked for the PB3 trace pad.
    unsafe {
        RCC_AHB1ENR.write_volatile(RCC_AHB1ENR.read_volatile() | (1 << 1) | (1 << 2)); // GPIOBEN | GPIOCEN
        GPIOC_MODER.write_volatile((GPIOC_MODER.read_volatile() & !(0b11 << 26)) | (0b01 << 26));
        usart2_init();
        timer_init();
    }

    // Core peripherals (DCB/TPIU/ITM) drive SWO trace out on PB3; bring the trace path up once.
    let cp = cortex_m::Peripherals::take().expect("core peripherals taken once");
    let mut swo = SwoOut {
        dcb: cp.DCB,
        tpiu: cp.TPIU,
        itm: cp.ITM,
    };
    swo.configure();

    let mut line = [0u8; proto::MAX_LINE];
    let mut len = 0usize;
    let mut tick = 0u32;
    let mut streaming = false;
    let mut seq = 0u32;
    let mut run: Option<Run> = None;
    // Boxed nowhere and never moved after this: the controller writes into it by address.
    let mut rx = Rx::new();
    unsafe { rx.start() };
    loop {
        // The controller keeps the line drained; each pass takes what it has written so far.
        match &mut run {
            Some(armed) if armed.active => armed.service(&mut rx),
            _ => unsafe { usart2_echo(&mut rx) },
        }

        let mut buf = [0u8; 32];
        let n = down.read(&mut buf);
        for &b in &buf[..n] {
            if b == b'\n' {
                respond(
                    &mut up,
                    &mut swo,
                    &mut streaming,
                    &mut run,
                    &mut rx,
                    &line[..len],
                );
                len = 0;
            } else if len < line.len() {
                line[len] = b;
                len += 1;
            } else {
                // Overlong line: drop it and resync at the next newline.
                len = 0;
            }
        }

        // One record per pass: the ITM FIFO's back-pressure paces the stream to the SWO rate.
        if streaming {
            emit_record(&mut swo, seq);
            seq = seq.wrapping_add(1);
        }

        tick = tick.wrapping_add(1);
        if tick & 0x3_FFFF == 0 {
            unsafe { GPIOC_ODR.write_volatile(GPIOC_ODR.read_volatile() ^ (1 << 13)) };
        }
    }
}
