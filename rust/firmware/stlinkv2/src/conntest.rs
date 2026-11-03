//! RTT-controlled GPIO scanning alongside the USB SWD service.

use cortex_m::peripheral::{DCB, DWT};
use daplink_test_protocol::gpio::{Pin, Pins, Pull, Scanner};
use daplink_test_protocol::MAX_LINE;
use rtt_target::{rtt_init, ChannelMode, DownChannel, UpChannel};
use stm32f1xx_hal::pac;

struct Lines;
const PADS: [u8; 3] = [7, 6, 10];

impl Lines {
    fn configure(index: usize, mode: u32, high: bool) {
        // USB may update adjacent GPIO fields; keep each RMW indivisible.
        cortex_m::interrupt::free(|_| unsafe {
            let gpio = &*pac::GPIOA::ptr();
            let pin = PADS[index];
            let shift = (pin % 8) * 4;
            let set_mode = |value| {
                if pin < 8 {
                    gpio.crl()
                        .modify(|r, w| w.bits((r.bits() & !(15 << shift)) | (value << shift)));
                } else {
                    gpio.crh()
                        .modify(|r, w| w.bits((r.bits() & !(15 << shift)) | (value << shift)));
                }
            };
            // Release any output before changing its pull latch.
            if mode & 3 == 0 {
                set_mode(4);
            }
            gpio.bsrr()
                .write(|w| w.bits(1 << if high { pin } else { pin + 16 }));
            set_mode(mode);
        });
    }
}

impl Pins for Lines {
    fn list(&self) -> &'static [Pin] {
        &[
            Pin {
                name: "PA7",
                drive: true,
            },
            Pin {
                name: "PA6",
                drive: false,
            },
            Pin {
                name: "PA10",
                drive: false,
            },
        ]
    }
    fn input(&mut self, pull: Pull) {
        for i in 0..PADS.len() {
            Self::configure(i, if pull == Pull::None { 4 } else { 8 }, pull == Pull::Up);
        }
    }
    fn drive(&mut self, outputs: u64, levels: u64) {
        self.input(Pull::Up);
        // F1 output mode has no pull-up; release as an input so a buffer input cannot float.
        if outputs & 1 != 0 && levels & 1 == 0 {
            Self::configure(0, 0b0110, false);
        }
    }
    fn read(&mut self) -> u64 {
        let bits = unsafe { (&*pac::GPIOA::ptr()).idr().read().bits() };
        PADS.iter()
            .enumerate()
            .fold(0, |mask, (i, p)| mask | (u64::from((bits >> p) & 1) << i))
    }
}

pub struct Control {
    scan: Scanner<Lines>,
    up: UpChannel,
    down: DownChannel,
}
impl Control {
    pub fn new(mut dcb: DCB, mut dwt: DWT) -> Self {
        dcb.enable_trace();
        dwt.enable_cycle_counter();
        let channels = rtt_init! {
            up: { 0: { size: 512, mode: ChannelMode::NoBlockSkip, name: "scan" } }
            down: { 0: { size: 128, name: "scan" } }
        };
        Self {
            scan: Scanner::new(Lines, "stlinkv2-conntest", 72_000_000),
            up: channels.up.0,
            down: channels.down.0,
        }
    }
    pub fn poll(&mut self) {
        let now = DWT::cycle_count();
        self.scan.expire(now);
        let mut input = [0; 32];
        let mut reply = [0; MAX_LINE];
        let n = self.down.read(&mut input);
        for &byte in &input[..n] {
            if let Some(len) = self.scan.feed(byte, now, &mut reply) {
                self.up.write(&reply[..len]);
            }
        }
    }
}
