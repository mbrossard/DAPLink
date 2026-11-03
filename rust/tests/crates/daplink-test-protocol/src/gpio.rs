//! Bounded RTT GPIO scan commands and an inactivity release guard.

use crate::{id_response, try_line, MAX_LINE, OK_PONG};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pull {
    None,
    Up,
    Down,
}

pub struct Pin {
    pub name: &'static str,
    pub drive: bool,
}

/// Implementations preserve unrelated pads and drive only open-drain low/release.
pub trait Pins {
    fn list(&self) -> &'static [Pin];
    fn input(&mut self, pull: Pull);
    fn drive(&mut self, outputs: u64, levels: u64);
    fn read(&mut self) -> u64;
}

#[derive(Debug, PartialEq, Eq)]
pub enum Command {
    List(Option<usize>),
    Input(Pull),
    Drive(u64, u64),
    Read,
    Release,
}

pub fn parse(line: &str) -> Option<Command> {
    let mut words = line.split_ascii_whitespace();
    if words.next()? != "GPIO" {
        return None;
    }
    let command = match words.next()? {
        "LIST" => Command::List(words.next().map(str::parse).transpose().ok()?),
        "CONFIG_INPUT" => Command::Input(match words.next()? {
            "NONE" => Pull::None,
            "UP" => Pull::Up,
            "DOWN" => Pull::Down,
            _ => return None,
        }),
        "DRIVE" => Command::Drive(
            u64::from_str_radix(words.next()?, 16).ok()?,
            u64::from_str_radix(words.next()?, 16).ok()?,
        ),
        "READ" => Command::Read,
        "RELEASE" => Command::Release,
        _ => return None,
    };
    words.next().is_none().then_some(command)
}

pub struct Scanner<P> {
    pins: P,
    name: &'static str,
    last: u32,
    timeout: u32,
    active: bool,
    line: [u8; MAX_LINE],
    len: usize,
    overflow: bool,
}

impl<P: Pins> Scanner<P> {
    /// `timeout` and `now` use the same wrapping tick counter; poll at least once per wrap.
    /// # Panics
    /// Panics if the candidate list exceeds 64 pins or the timeout is zero.
    pub fn new(mut pins: P, name: &'static str, timeout: u32) -> Self {
        assert!(pins.list().len() <= 64 && timeout != 0);
        pins.input(Pull::None);
        Self {
            pins,
            name,
            last: 0,
            timeout,
            active: false,
            line: [0; MAX_LINE],
            len: 0,
            overflow: false,
        }
    }

    pub fn expire(&mut self, now: u32) {
        if self.active && now.wrapping_sub(self.last) >= self.timeout {
            self.pins.input(Pull::None);
            self.active = false;
        }
    }

    /// One bounded line in, one newline-terminated reply out; oversized lines are discarded whole.
    pub fn feed(&mut self, byte: u8, now: u32, output: &mut [u8; MAX_LINE]) -> Option<usize> {
        self.expire(now);
        if byte != b'\n' {
            if self.len < self.line.len() && !self.overflow {
                self.line[self.len] = byte;
                self.len += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }
        let mut line = [0u8; MAX_LINE];
        line[..self.len].copy_from_slice(&self.line[..self.len]);
        let text = core::str::from_utf8(&line[..self.len]).ok();
        let overflow = self.overflow;
        self.len = 0;
        self.overflow = false;
        Some(self.handle(if overflow { None } else { text }, now, output))
    }

    fn handle(&mut self, line: Option<&str>, now: u32, output: &mut [u8; MAX_LINE]) -> usize {
        let line = line.unwrap_or("").trim();
        let reply = if line == "ID" {
            Some(id_response(output, self.name, 0))
        } else if line == "PING" {
            try_line(output, format_args!("{OK_PONG}"))
        } else {
            let command = parse(line);
            let pins = self.pins.list();
            let mask = if pins.len() < 64 {
                (1u64 << pins.len()) - 1
            } else {
                u64::MAX
            };
            let allowed = pins
                .iter()
                .enumerate()
                .fold(0, |a, (i, p)| a | ((p.drive as u64) << i));
            let valid = match command {
                Some(Command::List(Some(i))) => i < pins.len(),
                Some(Command::Drive(out, level)) => {
                    out & !allowed == 0 && (out | level) & !mask == 0 && level & !out == 0
                }
                Some(_) => true,
                None => false,
            };
            if !valid {
                try_line(output, format_args!("ERR gpio"))
            } else {
                self.last = now;
                match command.expect("validated command") {
                    Command::List(None) => {
                        try_line(output, format_args!("OK GPIO COUNT {}", pins.len()))
                    }
                    Command::List(Some(i)) => try_line(
                        output,
                        format_args!(
                            "OK GPIO PIN {i} {} {}",
                            pins[i].name,
                            if pins[i].drive { "IO" } else { "I" }
                        ),
                    ),
                    Command::Input(pull) => {
                        self.pins.input(pull);
                        self.active = true;
                        try_line(output, format_args!("OK GPIO CONFIG_INPUT"))
                    }
                    Command::Drive(out, level) => {
                        self.pins.drive(out, level);
                        self.active = true;
                        try_line(output, format_args!("OK GPIO DRIVE"))
                    }
                    Command::Read => {
                        try_line(output, format_args!("OK GPIO READ {:X}", self.pins.read()))
                    }
                    Command::Release => {
                        self.pins.input(Pull::None);
                        self.active = false;
                        try_line(output, format_args!("OK GPIO RELEASE"))
                    }
                }
            }
        };
        match reply.map(str::len) {
            Some(len) => len,
            None => {
                let error = b"ERR reply\n";
                output[..error.len()].copy_from_slice(error);
                error.len()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    struct Fake {
        outputs: u64,
        pull: Pull,
    }
    impl Pins for Fake {
        fn list(&self) -> &'static [Pin] {
            &[
                Pin {
                    name: "TX",
                    drive: true,
                },
                Pin {
                    name: "RX",
                    drive: false,
                },
            ]
        }
        fn input(&mut self, p: Pull) {
            self.outputs = 0;
            self.pull = p;
        }
        fn drive(&mut self, out: u64, _: u64) {
            self.outputs = out;
        }
        fn read(&mut self) -> u64 {
            2
        }
    }
    fn send(s: &mut Scanner<Fake>, line: &str, tick: u32) -> std::string::String {
        let mut buf = [0; MAX_LINE];
        let mut n = 0;
        for b in line.bytes().chain(*b"\n") {
            if let Some(len) = s.feed(b, tick, &mut buf) {
                n = len;
            }
        }
        std::string::String::from_utf8(buf[..n].to_vec()).unwrap()
    }
    #[test]
    fn rejects_without_touching_active_pin() {
        let mut s = Scanner::new(
            Fake {
                outputs: 0,
                pull: Pull::None,
            },
            "test",
            100,
        );
        assert_eq!(send(&mut s, "GPIO DRIVE 1 0", 0), "OK GPIO DRIVE\n");
        for bad in [
            "GPIO DRIVE 2 0",
            "GPIO DRIVE 4 0",
            "GPIO DRIVE 1 2",
            "GPIO READ extra",
            "GPIO LIST 9",
        ] {
            assert_eq!(send(&mut s, bad, 1), "ERR gpio\n");
            assert_eq!(s.pins.outputs, 1);
        }
        s.expire(100);
        assert_eq!(s.pins.outputs, 0);
    }
    #[test]
    fn wrap_timeout_and_overflow() {
        let mut s = Scanner::new(
            Fake {
                outputs: 0,
                pull: Pull::None,
            },
            "test",
            100,
        );
        send(&mut s, "GPIO DRIVE 1 0", u32::MAX - 50);
        s.expire(48);
        assert_eq!(s.pins.outputs, 1);
        s.expire(49);
        assert_eq!(s.pins.outputs, 0);
        assert_eq!(send(&mut s, &"X".repeat(129), 50), "ERR gpio\n");
        assert_eq!(send(&mut s, "GPIO READ", 51), "OK GPIO READ 2\n");
        assert_eq!(
            send(&mut s, "GPIO CONFIG_INPUT UP", 52),
            "OK GPIO CONFIG_INPUT\n"
        );
        assert_eq!(s.pins.pull, Pull::Up);
    }
}
