//! SWD words on the simulated pads, framed as ADIv5 B4.2 frames them.

use dap_rs::swd;

use crate::target::{Line, Shared};

/// A word driver on the simulated pads, through the SPI block or through the GPIOs.
pub struct SimWords {
    sim: Shared,
    spi: bool,
}

impl SimWords {
    /// Words through the GPIOs, on SWCLK as the engine left it.
    pub fn gpio(sim: &Shared) -> Self {
        Self {
            sim: sim.clone(),
            spi: false,
        }
    }

    /// Words through the SPI block, which takes both shared pads first.
    pub fn spi(sim: &Shared) -> Self {
        let mut s = sim.borrow_mut();
        s.set_peripheral(Line::Swclk, true);
        s.set_peripheral(Line::Swdio, true);
        drop(s);
        Self {
            sim: sim.clone(),
            spi: true,
        }
    }

    /// One clock cell: drive SWDIO or release it, sample in the low half.
    fn cell(&mut self, drive: Option<bool>) -> bool {
        let mut s = self.sim.borrow_mut();
        if self.spi {
            return s.spi_cell(drive);
        }
        match drive {
            Some(bit) => {
                s.set_latch(Line::Swdio, bit);
                s.set_output(Line::Swdio, true);
            }
            None => s.set_output(Line::Swdio, false),
        }
        s.set_latch(Line::Swclk, false);
        let bit = s.level(Line::Swdio);
        s.set_latch(Line::Swclk, true);
        bit
    }

    fn request(&mut self, req: u8) -> u8 {
        for i in 0..8 {
            self.cell(Some(req >> i & 1 != 0));
        }
        self.cell(None);
        (0..3).fold(0, |ack, i| ack | u8::from(self.cell(None)) << i)
    }

    /// One read transaction; a bad parity still clocks the trailing turnaround.
    pub fn read(&mut self, req: u8) -> swd::Result<u32> {
        let ack = self.request(req);
        if let Err(e) = swd::Ack::try_ok(ack) {
            self.cell(None);
            return Err(e);
        }
        let data = (0..32).fold(0, |v, i| v | u32::from(self.cell(None)) << i);
        let parity = self.cell(None);
        self.cell(None);
        if parity != (data.count_ones() & 1 != 0) {
            return Err(swd::Error::BadParity);
        }
        Ok(data)
    }

    /// Idle cells with SWDIO driven low, as a block clocks them after an OK ACK.
    pub fn idle(&mut self, cycles: u8) {
        for _ in 0..cycles {
            self.cell(Some(false));
        }
    }

    /// One write transaction; a no-ACK is `NoAck`, as on a read.
    pub fn write(&mut self, req: u8, data: u32) -> swd::Result<()> {
        let ack = self.request(req);
        self.cell(None);
        swd::Ack::try_ok(ack)?;
        for i in 0..32 {
            self.cell(Some(data >> i & 1 != 0));
        }
        self.cell(Some(data.count_ones() & 1 != 0));
        Ok(())
    }
}
