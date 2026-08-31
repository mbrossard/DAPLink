//! The aggregate USB service budget and DAP load axis, in packet slots per millisecond.

use std::time::Duration;

/// Bytes in the packet slot the budget is counted in, which is one full-speed bulk packet.
pub const SLOT: f64 = 64.0;

/// Bulk transactions a full-speed 1 ms frame can theoretically carry.
const FS_SLOTS_PER_MS: f64 = 19.0;

/// Bulk transactions a high-speed millisecond can theoretically carry: 13 per microframe, eight
/// microframes.
const HS_SLOTS_PER_MS: f64 = 104.0;

/// Bulk max packet size that marks a high-speed endpoint.
const HS_PACKET: usize = 512;

/// Share of the theoretical budget a bridge actually serves: 13 of a full-speed frame's 19 slots.
const DERATE: f64 = 13.0 / 19.0;

/// Round trips per second a depth-1 stress profile offers, before the bus answers.
const STRESS_CPS: f64 = 1_000.0;

const REPRESENTATIVE_CPS: f64 = 100.0;

pub const DAP_READ_BYTES: u64 = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BusBudget {
    pub packet: usize,
    pub bytes_per_ms: f64,
}

impl BusBudget {
    pub fn for_packet(packet: usize) -> Self {
        let (slots, size) = if packet >= HS_PACKET {
            (HS_SLOTS_PER_MS, HS_PACKET as f64)
        } else {
            (FS_SLOTS_PER_MS, SLOT)
        };
        BusBudget {
            packet,
            bytes_per_ms: slots * size * DERATE,
        }
    }

    pub fn high_speed(&self) -> bool {
        self.packet >= HS_PACKET
    }

    pub fn slots_per_ms(&self) -> f64 {
        self.bytes_per_ms / SLOT
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Demand {
    /// Host to DUT, payload and marker frames together.
    pub h2d_bps: f64,
    /// DUT to host, likewise.
    pub d2h_bps: f64,
    pub dap_bps: f64,
}

impl Demand {
    pub fn slots_per_ms(&self) -> f64 {
        (self.h2d_bps + self.d2h_bps + self.dap_bps) / 1000.0 / SLOT
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Clamp {
    /// Offered load, 64-byte slots per millisecond.
    pub demand: f64,
    /// Derated budget, same units.
    pub budget: f64,
    pub fits: bool,
    pub high_speed: bool,
}

impl Clamp {
    pub fn load(&self) -> f64 {
        if self.budget > 0.0 {
            self.demand / self.budget
        } else {
            f64::INFINITY
        }
    }
}

pub fn clamp(demand: &Demand, budget: &BusBudget) -> Clamp {
    let high_speed = budget.high_speed();
    let (demand, budget) = (demand.slots_per_ms(), budget.slots_per_ms());
    Clamp {
        demand,
        budget,
        fits: demand <= budget,
        high_speed,
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, clap::ValueEnum)]
pub enum DapProfile {
    /// No DAP traffic at all in the window, the control channel included.
    #[default]
    None,
    /// One 64-byte RAM read every 10 ms, one command outstanding.
    Representative,
    /// Back-to-back 64-byte RAM reads, one command outstanding.
    Stress,
}

impl DapProfile {
    pub fn key(self) -> &'static str {
        match self {
            DapProfile::None => "none",
            DapProfile::Representative => "representative",
            DapProfile::Stress => "stress",
        }
    }

    pub fn gap(self) -> Option<Duration> {
        match self {
            DapProfile::None => None,
            DapProfile::Representative => Some(Duration::from_millis(10)),
            DapProfile::Stress => None,
        }
    }

    pub fn nominal_cps(self) -> f64 {
        match self {
            DapProfile::None => 0.0,
            DapProfile::Representative => REPRESENTATIVE_CPS,
            DapProfile::Stress => STRESS_CPS,
        }
    }

    /// Bus bytes per second the profile offers: one command and one response packet per read.
    pub fn nominal_bps(self) -> f64 {
        self.nominal_cps() * 2.0 * SLOT
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DapAchieved {
    pub profile: DapProfile,
    pub commands: u64,
    pub bytes: u64,
    pub secs: f64,
}

impl DapAchieved {
    pub fn cps(&self) -> f64 {
        if self.secs > 0.0 {
            self.commands as f64 / self.secs
        } else {
            0.0
        }
    }

    pub fn bps(&self) -> f64 {
        if self.secs > 0.0 {
            self.bytes as f64 / self.secs
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The wire byte rate of one direction at `baud`, markers included.
    fn wire(baud: u32) -> f64 {
        f64::from(baud) / 10.0
    }

    #[test]
    fn a_full_speed_probe_gets_thirteen_of_its_nineteen_slots() {
        let fs = BusBudget::for_packet(64);
        assert!(!fs.high_speed());
        assert!((fs.slots_per_ms() - 13.0).abs() < 1e-9);
        assert!((fs.bytes_per_ms - 832.0).abs() < 1e-9);
    }

    #[test]
    fn a_high_speed_probe_gets_its_own_budget_from_the_same_derating() {
        let hs = BusBudget::for_packet(512);
        assert!(hs.high_speed());
        assert!((hs.bytes_per_ms - 104.0 * 512.0 * 13.0 / 19.0).abs() < 1e-6);
        assert!(hs.slots_per_ms() > 500.0);
    }

    #[test]
    fn the_f401_symmetric_point_costs_the_slots_the_design_predicts() {
        let demand = Demand {
            h2d_bps: wire(2_625_000),
            d2h_bps: wire(2_625_000),
            dap_bps: 0.0,
        };
        assert!((demand.slots_per_ms() - 8.203).abs() < 0.01);
        let out = clamp(&demand, &BusBudget::for_packet(64));
        assert!(out.fits);
        assert!(out.load() < 0.7);
    }

    #[test]
    fn the_symmetric_points_the_synthesis_voided_are_refused_up_front() {
        let fs = BusBudget::for_packet(64);
        for baud in [6_000_000, 9_375_000] {
            let demand = Demand {
                h2d_bps: wire(baud),
                d2h_bps: wire(baud),
                dap_bps: 0.0,
            };
            assert!(
                !clamp(&demand, &fs).fits,
                "{baud} symmetric should be refused"
            );
        }
        let one_way = Demand {
            h2d_bps: 0.0,
            d2h_bps: wire(6_000_000),
            dap_bps: 0.0,
        };
        assert!(clamp(&one_way, &fs).fits);
    }

    #[test]
    fn the_dap_profile_moves_the_decision_at_the_margin() {
        let fs = BusBudget::for_packet(64);
        let baud = 3_900_000;
        let point = |profile: DapProfile| Demand {
            h2d_bps: wire(baud),
            d2h_bps: wire(baud),
            dap_bps: profile.nominal_bps(),
        };
        assert!(clamp(&point(DapProfile::None), &fs).fits);
        assert!(clamp(&point(DapProfile::Representative), &fs).fits);
        assert!(!clamp(&point(DapProfile::Stress), &fs).fits);
        assert!(clamp(&point(DapProfile::Stress), &BusBudget::for_packet(512)).fits);
    }

    #[test]
    fn the_profiles_are_exactly_the_three_the_design_names() {
        assert_eq!(DapProfile::default(), DapProfile::None);
        assert_eq!(DapProfile::None.nominal_bps(), 0.0);
        assert_eq!(
            DapProfile::Representative.gap(),
            Some(Duration::from_millis(10))
        );
        assert_eq!(DapProfile::Stress.gap(), None);
        assert!(DapProfile::Stress.nominal_cps() > DapProfile::Representative.nominal_cps());
    }

    #[test]
    fn an_achieved_rate_is_zero_until_the_thread_has_run() {
        let idle = DapAchieved::default();
        assert_eq!((idle.cps(), idle.bps()), (0.0, 0.0));
        let ran = DapAchieved {
            profile: DapProfile::Stress,
            commands: 3_000,
            bytes: 3_000 * DAP_READ_BYTES,
            secs: 3.0,
        };
        assert!((ran.cps() - 1_000.0).abs() < 1e-9);
        assert!((ran.bps() - 64_000.0).abs() < 1e-9);
    }
}
