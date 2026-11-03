//! RTT connectivity endpoint with SWD retained and trace disabled.

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
use daplink_test_protocol::gpio::{Pin, Pins, Pull, Scanner};
use panic_halt as _;
use rtt_target::{rtt_init, ChannelMode};

const NAME: &str = "test-stm32f401-conntest";
const CPU_HZ: u32 = 16_000_000;
const RCC_CR: *mut u32 = 0x4002_3800 as *mut u32;
const RCC_CFGR: *mut u32 = 0x4002_3808 as *mut u32;
const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32;
const DBGMCU_CR: *mut u32 = 0xE004_2004 as *mut u32;
const GPIOA: usize = 0x4002_0000;
const GPIOB: usize = 0x4002_0400;
const MODER: usize = 0;
const OTYPER: usize = 4;
const PUPDR: usize = 0x0C;
const IDR: usize = 0x10;
const BSRR: usize = 0x18;

fn modify(register: *mut u32, mask: u32, value: u32) {
    // Only this loop configures GPIO and clock registers; there are no interrupt handlers.
    unsafe { register.write_volatile((register.read_volatile() & !mask) | value) };
}

fn field(base: usize, offset: usize, pin: u32, value: u32) {
    modify(
        (base + offset) as *mut u32,
        3 << (pin * 2),
        value << (pin * 2),
    );
}

fn input(base: usize, pin: u32, pull: Pull) {
    field(base, MODER, pin, 0);
    let pull = match pull {
        Pull::None => 0,
        Pull::Up => 1,
        Pull::Down => 2,
    };
    field(base, PUPDR, pin, pull);
}

struct ScanPins;

impl Pins for ScanPins {
    fn list(&self) -> &'static [Pin] {
        &[
            Pin {
                name: "PA15",
                drive: false,
            },
            Pin {
                name: "PB3",
                drive: true,
            },
        ]
    }

    fn input(&mut self, pull: Pull) {
        input(GPIOA, 15, pull);
        input(GPIOB, 3, pull);
    }

    fn drive(&mut self, outputs: u64, levels: u64) {
        input(GPIOA, 15, Pull::Up);
        input(GPIOB, 3, Pull::Up);
        if outputs & 2 != 0 {
            // Set open drain and its latch while the pad is still an input.
            modify((GPIOB + OTYPER) as *mut u32, 1 << 3, 1 << 3);
            let latch = if levels & 2 != 0 { 1 << 3 } else { 1 << 19 };
            unsafe { ((GPIOB + BSRR) as *mut u32).write_volatile(latch) };
            field(GPIOB, MODER, 3, 1);
        }
    }

    fn read(&mut self) -> u64 {
        unsafe {
            let a = ((GPIOA + IDR) as *const u32).read_volatile();
            let b = ((GPIOB + IDR) as *const u32).read_volatile();
            u64::from(((a >> 15) & 1) | (((b >> 3) & 1) << 1))
        }
    }
}

#[entry]
fn main() -> ! {
    modify(RCC_CR, 1, 1);
    while unsafe { RCC_CR.read_volatile() } & 2 == 0 {}
    modify(RCC_CFGR, 0xF3, 0);
    while unsafe { RCC_CFGR.read_volatile() } & 0xC != 0 {}
    modify(RCC_AHB1ENR, 3, 3);
    let _ = unsafe { RCC_AHB1ENR.read_volatile() };
    modify(DBGMCU_CR, 1 << 5, 0);
    let mut cp = cortex_m::Peripherals::take().expect("core peripherals taken once");
    cp.DCB.enable_trace();
    cp.DWT.set_cycle_count(0);
    cp.DWT.enable_cycle_counter();
    let mut scanner = Scanner::new(ScanPins, NAME, CPU_HZ);
    let channels = rtt_init! {
        up: { 0: { size: 512, mode: ChannelMode::NoBlockSkip, name: "up" } }
        down: { 0: { size: 128, name: "down" } }
    };
    let mut up = channels.up.0;
    let mut down = channels.down.0;
    up.write(b"EVT BOOT test-stm32f401-conntest\n");
    let mut reply = [0u8; 128];
    let mut byte = [0u8; 1];
    loop {
        let now = DWT::cycle_count();
        scanner.expire(now);
        if down.read(&mut byte) != 0 {
            if let Some(n) = scanner.feed(byte[0], now, &mut reply) {
                up.write(&reply[..n]);
            }
        }
    }
}
