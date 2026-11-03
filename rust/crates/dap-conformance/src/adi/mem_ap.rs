//! A MEM-AP over a sparse memory whose top page faults.

use std::collections::BTreeMap;

use super::regs::{BASE, BD0, CSW, DRW, IDR, TAR};

/// Memory at or above this address faults.
pub const FAULT_BASE: u32 = 0xFFFF_F000;

/// CSW, TAR and the memory behind them; untouched words are a hash of their address.
#[derive(Clone, Debug, Default)]
pub struct MemAp {
    csw: u32,
    tar: u32,
    mem: BTreeMap<u32, u32>,
    seed: u64,
}

fn splitmix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^ (x >> 31)
}

impl MemAp {
    /// A MEM-AP whose untouched memory `seed` decides.
    pub fn new(seed: u64) -> Self {
        Self {
            seed,
            ..Self::default()
        }
    }

    /// One register access at byte offset `addr`: the value, and whether the memory access faulted.
    pub fn access(&mut self, read: bool, addr: u8, data: u32) -> (u32, bool) {
        match addr {
            CSW => {
                if !read {
                    self.csw = data & 0x0F00_0030 | 2;
                }
                (self.csw | 0x40, false)
            }
            TAR => {
                if !read {
                    self.tar = data;
                }
                (self.tar, false)
            }
            DRW => {
                let tar = self.tar;
                let v = self.mem(read, tar, data);
                // Single increment wraps within the 1 KiB page.
                if v.is_some() && self.csw >> 4 & 3 == 1 {
                    self.tar = tar & !0x3FF | tar.wrapping_add(4) & 0x3FF;
                }
                (v.unwrap_or(0), v.is_none())
            }
            _ if addr & 0xF0 == BD0 => {
                let at = self.tar & !0xF | u32::from(addr & 0xC);
                let v = self.mem(read, at, data);
                (v.unwrap_or(0), v.is_none())
            }
            BASE => (0xE00F_F003, false),
            IDR => (0x2477_0011, false),
            _ => (0, false),
        }
    }

    fn mem(&mut self, read: bool, at: u32, data: u32) -> Option<u32> {
        let at = at & !3;
        if at >= FAULT_BASE {
            return None;
        }
        if read {
            let seed = self.seed;
            Some(
                *self
                    .mem
                    .entry(at)
                    .or_insert_with(|| splitmix(seed ^ u64::from(at)) as u32),
            )
        } else {
            self.mem.insert(at, data);
            Some(data)
        }
    }
}
