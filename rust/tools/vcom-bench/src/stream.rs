//! The v1 stream's host side: the run plan, its per-direction ledgers, and their criteria.

use std::time::Instant;

use daplink_test_protocol::v1::{
    self, BurstSpec, DirMask, RxCounters, TxCounters, V1Report, FRAME_LEN,
};

pub use daplink_test_protocol::v1::{Direction, DEFAULT_INTERVAL};

use crate::budget::{DapAchieved, Demand};

/// Both directions, in report order.
pub const DIRECTIONS: [Direction; 2] = [Direction::HostToDut, Direction::DutToHost];

/// Goodput a direction claiming the line rate must hold, as a fraction of the payload budget.
pub const PASS_FRAC: f64 = 0.99;

/// Untimed warmup before a measured window, seconds.
pub const WARMUP_SECS: f64 = 5.0;
/// Seconds between checkpoints inside a measured window.
pub const CHECKPOINT_SECS: f64 = 5.0;
/// Delivered fraction a checkpoint must hold before the window is abandoned, for a direction
/// claiming the line rate.
pub const CHECKPOINT_FLOOR: f64 = 0.90;
/// Shortest window a discovery rung is measured over, seconds.
pub const DISCOVERY_SECS: f64 = 30.0;
/// Window one qualification run covers, seconds.
pub const QUALIFY_SECS: f64 = 60.0;
/// Qualification runs a ceiling needs.
pub const QUALIFY_RUNS: u32 = 3;
/// Window the soak covers, seconds.
pub const SOAK_SECS: f64 = 600.0;
/// Outstanding grant the host keeps ahead of its verifier in credit mode, in bytes.
pub const CREDIT_WINDOW: u64 = 4096;
/// Fraction of the line rate the host leg is paced to in a return-priority run.
pub const RETURN_PRIORITY_FRAC: f64 = 0.25;

pub fn label(dir: Direction) -> &'static str {
    match dir {
        Direction::HostToDut => "host -> DUT",
        Direction::DutToHost => "DUT -> host",
    }
}

pub fn key(dir: Direction) -> &'static str {
    match dir {
        Direction::HostToDut => "host_to_dut",
        Direction::DutToHost => "dut_to_host",
    }
}

/// The host's microsecond clock, shared by every engine in one run so their timestamps agree.
#[derive(Clone, Copy, Debug)]
pub struct Clock {
    start: Instant,
}

impl Clock {
    pub fn new() -> Self {
        Clock {
            start: Instant::now(),
        }
    }

    /// Microseconds since the clock started, wrapping the way the engines expect.
    pub fn now_us(&self) -> u32 {
        self.start.elapsed().as_micros() as u32
    }
}

/// A run's two streams, named by the direction that carries them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Seeds {
    pub h2d: u32,
    pub d2h: u32,
}

impl Default for Seeds {
    fn default() -> Self {
        Seeds {
            h2d: 0x5EED_0001,
            d2h: 0x5EED_0002,
        }
    }
}

/// What each end transmits in a run, and how the result may be read.
#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum Shape {
    /// Both ends transmit their whole budget at line rate: the acceptance test.
    Duplex,
    /// Both ends transmit, the DUT only as far as the host's cumulative grants reach: claims no
    /// line rate.
    Credit,
    /// Both ends transmit, the host leg paced to a fraction so the return leg keeps the bridge.
    ReturnPriority,
    /// Host to DUT alone; a diagnostic.
    Out,
    /// DUT to host alone; a diagnostic.
    Back,
}

impl Shape {
    /// The host transmits, so the DUT verifies.
    pub fn host_tx(self) -> bool {
        self != Shape::Back
    }

    /// The DUT transmits, so the host verifies.
    pub fn dut_tx(self) -> bool {
        self != Shape::Out
    }

    /// The `V1 START` direction mask: bit 0 arms the DUT transmitter, bit 1 its verifier.
    pub fn dir_mask(self) -> u8 {
        let mut bits = 0;
        if self.dut_tx() {
            bits |= DirMask::TX;
        }
        if self.host_tx() {
            bits |= DirMask::RX;
        }
        bits
    }

    /// Whether the shape claims `dir` runs at line rate, which is what the pass bar judges.
    pub fn claims_line_rate(self, dir: Direction) -> bool {
        match (self, dir) {
            (Shape::Credit, _) => false,
            (Shape::ReturnPriority, Direction::HostToDut) => false,
            (_, Direction::HostToDut) => self.host_tx(),
            (_, Direction::DutToHost) => self.dut_tx(),
        }
    }

    pub fn key(self) -> &'static str {
        match self {
            Shape::Duplex => "duplex",
            Shape::Credit => "credit",
            Shape::ReturnPriority => "return-priority",
            Shape::Out => "out",
            Shape::Back => "back",
        }
    }
}

/// One measured window's whole configuration, fixed before the run starts.
#[derive(Clone, Copy, Debug)]
pub struct Plan {
    pub shape: Shape,
    pub seeds: Seeds,
    /// Payload bytes between marker frames.
    pub interval: u32,
    /// The nominal window, which is what goodput is denominated over.
    pub secs: f64,
    /// The rate each end reported programming; a direction's budget derives from its own
    /// transmitter's clock, the only one that can pace it.
    pub dut_baud: u32,
    pub probe_baud: u32,
    /// Fraction of the line rate the host transmitter is paced to.
    pub host_frac: f64,
    /// Shaping for the transmitters it is applied to.
    pub burst: Option<BurstSpec>,
    pub burst_host: bool,
    pub burst_dut: bool,
    /// Bytes the host keeps granted ahead of its verifier in credit mode.
    pub credit_window: u64,
}

impl Plan {
    /// Wire bytes per second the direction's transmitter runs at, ten bits to the byte.
    pub fn line_bps(&self, dir: Direction) -> f64 {
        let baud = match dir {
            Direction::HostToDut => self.probe_baud,
            Direction::DutToHost => self.dut_baud,
        };
        f64::from(baud) / 10.0
    }

    /// Share of the wire the payload gets; the marker frames are the instrument's own cost.
    pub fn payload_fraction(&self) -> f64 {
        let interval = f64::from(self.interval.max(FRAME_LEN as u32));
        interval / (interval + FRAME_LEN as f64)
    }

    /// The rating baseline: payload bytes per second this direction at line rate can deliver.
    pub fn payload_budget_bps(&self, dir: Direction) -> f64 {
        self.line_bps(dir) * self.payload_fraction()
    }

    /// Goodput a direction claiming the line rate must reach.
    pub fn pass_bar_bps(&self, dir: Direction) -> f64 {
        self.payload_budget_bps(dir) * PASS_FRAC
    }

    /// Payload bytes a transmitter paced to `frac` of its line rate emits over the window.
    fn budget_at(&self, dir: Direction, frac: f64) -> u32 {
        let bytes = self.secs * self.payload_budget_bps(dir) * frac;
        if bytes >= f64::from(u32::MAX) {
            u32::MAX
        } else {
            bytes.max(0.0).round() as u32
        }
    }

    /// The host transmitter's payload budget, 0 when it does not transmit.
    pub fn budget_h2d(&self) -> u32 {
        if self.shape.host_tx() {
            self.budget_at(Direction::HostToDut, self.host_frac)
        } else {
            0
        }
    }

    /// The DUT transmitter's payload budget, 0 when it does not transmit.
    pub fn budget_d2h(&self) -> u32 {
        if self.shape.dut_tx() {
            self.budget_at(Direction::DutToHost, 1.0)
        } else {
            0
        }
    }

    /// Wire bytes per second the host transmitter is paced to.
    pub fn host_pace_bps(&self) -> f64 {
        self.line_bps(Direction::HostToDut) * self.host_frac
    }

    /// What the run offers the bus, both directions' wire traffic plus the DAP profile's.
    pub fn demand(&self, dap_bps: f64) -> Demand {
        Demand {
            h2d_bps: if self.shape.host_tx() {
                self.host_pace_bps()
            } else {
                0.0
            },
            d2h_bps: if self.shape.dut_tx() {
                self.line_bps(Direction::DutToHost)
            } else {
                0.0
            },
            dap_bps,
        }
    }

    /// The same operating point with the host leg paced, for when a symmetric point does not fit
    /// the bus budget.
    pub fn as_return_priority(&self) -> Plan {
        Plan {
            shape: Shape::ReturnPriority,
            host_frac: RETURN_PRIORITY_FRAC,
            ..*self
        }
    }

    /// The `V1 START` line that arms the DUT for this plan.
    pub fn start_line(&self) -> String {
        let mut line = format!(
            "V1 START {} {} {} {} {} {}",
            self.shape.dir_mask(),
            self.seeds.h2d,
            self.seeds.d2h,
            self.interval,
            self.budget_h2d(),
            self.budget_d2h(),
        );
        if let (Some(burst), true) = (self.burst, self.burst_dut && self.shape.dut_tx()) {
            line.push_str(&format!(" BURST {} {}", burst.bytes, burst.idle_ms));
        }
        if self.shape == Shape::Credit {
            line.push_str(" CREDIT");
        }
        line.push('\n');
        line
    }

    /// The host's transmitter, or `None` when the shape leaves the host silent.
    pub fn host_encoder(&self) -> Option<v1::Encoder> {
        if !self.shape.host_tx() {
            return None;
        }
        let mut encoder = v1::Encoder::new(
            Direction::HostToDut,
            self.seeds.h2d,
            self.interval,
            u64::from(self.budget_h2d()),
        );
        if let (Some(burst), true) = (self.burst, self.burst_host) {
            encoder = encoder.with_burst(burst);
        }
        Some(encoder)
    }

    /// The host's receiver, unarmed, or `None` when the DUT does not transmit.
    pub fn host_verifier(&self) -> Option<v1::Verifier> {
        self.shape.dut_tx().then(|| {
            v1::Verifier::new(
                Direction::DutToHost,
                self.seeds.d2h,
                self.interval,
                u64::from(self.budget_d2h()),
            )
        })
    }

    /// The grant the host announces once its verifier has taken `verified` payload bytes.
    pub fn grant_for(&self, verified: u64) -> u64 {
        verified.saturating_add(self.credit_window)
    }
}

/// One direction's share of one checkpoint interval.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Slice {
    /// Payload bytes the interval carried: delivered for the return leg, offered for the host leg.
    pub payload: u64,
    /// Those bytes as a fraction of the interval's payload budget.
    pub frac: f64,
    pub corrupt: u64,
    pub missing: u64,
    pub duplicate: u64,
    pub unresolved: u64,
    pub resyncs: u32,
}

impl Slice {
    /// Payload bytes the interval failed to account for cleanly.
    pub fn lost(&self) -> u64 {
        self.corrupt + self.missing + self.duplicate + self.unresolved
    }
}

/// One read of the running host-side ledger; it reads counters the engines already keep, never
/// pausing the stream.
#[derive(Clone, Copy, Debug)]
pub struct Checkpoint {
    /// Seconds from arming to this read.
    pub at_secs: f64,
    pub h2d: Option<Slice>,
    pub d2h: Option<Slice>,
}

impl Checkpoint {
    pub fn slice(&self, dir: Direction) -> Option<&Slice> {
        match dir {
            Direction::HostToDut => self.h2d.as_ref(),
            Direction::DutToHost => self.d2h.as_ref(),
        }
    }
}

/// What the shape of one direction's checkpoint series says the run is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trend {
    /// Every checkpoint at the budget.
    Holding,
    /// A steady slight shortfall with no trend: a fixed per-packet cost, not a defect.
    Shortfall,
    /// High and then falling away: progressive starvation, a backlog filling.
    Declining,
    /// Low from the first checkpoint and flat: a leg pinned by a scheduling bound.
    Structural,
    /// A cliff to nothing: a wedge, or a verifier that lost the stream.
    Cliff,
    /// None of the known shapes; the series is the evidence and the class is open.
    Mixed,
    /// Too few checkpoints to say anything.
    Unsampled,
}

impl Trend {
    pub fn key(self) -> &'static str {
        match self {
            Trend::Holding => "holding",
            Trend::Shortfall => "shortfall",
            Trend::Declining => "declining",
            Trend::Structural => "structural",
            Trend::Cliff => "cliff",
            Trend::Mixed => "mixed",
            Trend::Unsampled => "unsampled",
        }
    }

    /// What the shape reads as, for the report.
    pub fn reads_as(self) -> &'static str {
        match self {
            Trend::Holding => "at the budget throughout",
            Trend::Shortfall => "a fixed per-packet cost, not a bug",
            Trend::Declining => "progressive starvation -- a backlog filling",
            Trend::Structural => "structural: a leg pinned by a scheduling bound",
            Trend::Cliff => "a wedge or a verifier desync",
            Trend::Mixed => "no single known shape; read the series",
            Trend::Unsampled => "no checkpoint fell inside the window",
        }
    }
}

/// Every checkpoint one window took, in order.
#[derive(Clone, Debug, Default)]
pub struct Series {
    pub period: f64,
    pub points: Vec<Checkpoint>,
}

impl Series {
    /// One direction's delivered fractions, in order.
    pub fn fractions(&self, dir: Direction) -> Vec<f64> {
        self.points
            .iter()
            .filter_map(|p| p.slice(dir).map(|s| s.frac))
            .collect()
    }

    pub fn trend(&self, dir: Direction) -> Trend {
        classify(&self.fractions(dir))
    }
}

/// Name the shape of a sequence of per-checkpoint delivered fractions.
pub fn classify(f: &[f64]) -> Trend {
    let Some(&first) = f.first() else {
        return Trend::Unsampled;
    };
    let last = f[f.len() - 1];
    let max = f.iter().copied().fold(f64::MIN, f64::max);
    let min = f.iter().copied().fold(f64::MAX, f64::min);
    let step = f.windows(2).map(|w| w[0] - w[1]).fold(0.0_f64, f64::max);
    if min >= PASS_FRAC {
        return Trend::Holding;
    }
    if min <= 0.25 && step >= 0.5 {
        return Trend::Cliff;
    }
    if max < CHECKPOINT_FLOOR && max - min <= 0.05 {
        return Trend::Structural;
    }
    if first >= CHECKPOINT_FLOOR && first - last >= 0.05 {
        return Trend::Declining;
    }
    if min >= CHECKPOINT_FLOOR && max - min <= 0.03 {
        return Trend::Shortfall;
    }
    Trend::Mixed
}

/// The ones that fall strictly inside the window; the closing interval is judged by the run's own
/// ledgers instead.
pub fn checkpoints_inside(secs: f64, period: f64) -> usize {
    if period <= 0.0 || secs <= 0.0 {
        return 0;
    }
    ((secs / period).ceil() as usize).saturating_sub(1)
}

/// Why a window was cut short, and how long it had held.
#[derive(Clone, Debug)]
pub struct Abandon {
    /// Seconds from arming to the failing checkpoint: the run's time to failure.
    pub at_secs: f64,
    /// Which checkpoint failed, counting from 1.
    pub checkpoint: usize,
    pub reason: String,
}

/// Takes the window's checkpoints and decides when the window is not worth finishing.
pub struct Sampler {
    plan: Plan,
    period: f64,
    floor: f64,
    /// Elapsed seconds the next checkpoint is due at.
    due: f64,
    /// Elapsed seconds the last one was taken at, which is where its interval starts.
    last_at: f64,
    taken: usize,
    prev_h2d: u64,
    prev_d2h: RxCounters,
    series: Series,
}

impl Sampler {
    pub fn new(plan: &Plan, period: f64, floor: f64) -> Self {
        Sampler {
            plan: *plan,
            period,
            floor,
            due: period,
            last_at: 0.0,
            taken: 0,
            prev_h2d: 0,
            prev_d2h: RxCounters::default(),
            series: Series {
                period,
                points: Vec::new(),
            },
        }
    }

    /// Reads the running ledgers if a checkpoint is due at `elapsed`, and says whether the window
    /// is worth finishing.
    pub fn sample(
        &mut self,
        elapsed: f64,
        tx: Option<TxCounters>,
        rx: Option<RxCounters>,
    ) -> Option<Abandon> {
        if self.period <= 0.0 || elapsed < self.due || self.due >= self.plan.secs {
            return None;
        }
        let span = elapsed - self.last_at;
        self.last_at = elapsed;
        // A poll that arrives a period late skips the checkpoints it missed, rather than firing one
        // per poll after.
        while self.due <= elapsed {
            self.due += self.period;
        }
        self.taken += 1;
        let h2d = tx.map(|tx| {
            let payload = tx.payload.saturating_sub(self.prev_h2d);
            self.prev_h2d = tx.payload;
            Slice {
                payload,
                frac: self.frac(
                    Direction::HostToDut,
                    payload,
                    span,
                    self.share(Direction::HostToDut),
                ),
                ..Slice::default()
            }
        });
        let d2h = rx.map(|rx| {
            let base = self.prev_d2h;
            self.prev_d2h = rx;
            let payload = rx.verified.saturating_sub(base.verified);
            Slice {
                payload,
                frac: self.frac(Direction::DutToHost, payload, span, 1.0),
                corrupt: rx.corrupt.saturating_sub(base.corrupt),
                missing: rx.missing.saturating_sub(base.missing),
                duplicate: rx.duplicate.saturating_sub(base.duplicate),
                unresolved: rx.unresolved.saturating_sub(base.unresolved),
                resyncs: rx.resyncs.saturating_sub(base.resyncs),
            }
        });
        self.series.points.push(Checkpoint {
            at_secs: elapsed,
            h2d,
            d2h,
        });
        let total = |dir| match dir {
            Direction::HostToDut => tx.map(|tx| tx.payload),
            Direction::DutToHost => rx.map(|rx| rx.verified),
        };
        self.judge(elapsed, total)
    }

    /// Payload bytes over a span as a fraction of what the budget offers in that span.
    fn frac(&self, dir: Direction, payload: u64, span: f64, share: f64) -> f64 {
        let budget = self.plan.payload_budget_bps(dir) * share * span;
        if budget > 0.0 {
            payload as f64 / budget
        } else {
            0.0
        }
    }

    /// The first checkpoint carries the stream's onset, so only loss condemns it.
    fn judge(&self, elapsed: f64, total: impl Fn(Direction) -> Option<u64>) -> Option<Abandon> {
        let point = self.series.points.last()?;
        let abandon = |reason: String| {
            Some(Abandon {
                at_secs: elapsed,
                checkpoint: self.taken,
                reason,
            })
        };
        for dir in DIRECTIONS {
            let Some(slice) = point.slice(dir) else {
                continue;
            };
            if slice.lost() > 0 {
                return abandon(format!(
                    "{}: {} corrupt, {} missing, {} duplicate, {} unresolved bytes in this \
                     checkpoint",
                    label(dir),
                    slice.corrupt,
                    slice.missing,
                    slice.duplicate,
                    slice.unresolved,
                ));
            }
            if !self.plan.shape.claims_line_rate(dir) || self.taken < 2 || slice.frac >= self.floor
            {
                continue;
            }
            let share = self.share(dir);
            let sofar = self.frac(dir, total(dir).unwrap_or(0), elapsed, share);
            if sofar < self.floor {
                return abandon(format!(
                    "{}: {:.1}% of the payload budget in this checkpoint, {:.1}% since the run \
                     started",
                    label(dir),
                    slice.frac * 100.0,
                    sofar * 100.0,
                ));
            }
        }
        None
    }

    /// The share of the payload budget a direction's transmitter is paced to.
    fn share(&self, dir: Direction) -> f64 {
        match dir {
            Direction::HostToDut => self.plan.host_frac,
            Direction::DutToHost => 1.0,
        }
    }

    pub fn finish(self) -> Series {
        self.series
    }
}

/// One direction's ledger: what its transmitter emitted and what its receiver made of it.
#[derive(Clone, Copy, Debug)]
pub struct Leg {
    pub dir: Direction,
    /// The run claims this direction ran at line rate, so goodput meets the pass bar.
    pub claimed: bool,
    pub tx_payload: u64,
    pub tx_frames: u32,
    pub rx: RxCounters,
    /// Payload total the receiver read out of the end-of-stream frame, where it reports one.
    pub rx_eos_total: Option<u64>,
}

impl Leg {
    /// Over the nominal window or the time the receiver was actually taking bytes, whichever is
    /// longer.
    pub fn goodput_bps(&self, secs: f64) -> f64 {
        let took = secs.max(f64::from(self.rx.elapsed_us) / 1e6);
        if took > 0.0 {
            self.rx.verified as f64 / took
        } else {
            0.0
        }
    }

    /// Goodput over `secs` as a fraction of the payload budget.
    pub fn budget_frac(&self, plan: &Plan, secs: f64) -> f64 {
        let budget = plan.payload_budget_bps(self.dir);
        if budget > 0.0 {
            self.goodput_bps(secs) / budget
        } else {
            0.0
        }
    }

    /// Every byte the transmitter sent is accounted for at the receiver, and both ends agree on the
    /// total.
    pub fn reconciles(&self) -> bool {
        self.rx.eos
            && self.rx.accounted() == self.tx_payload
            && self
                .rx_eos_total
                .is_none_or(|total| total == self.tx_payload)
    }
}

/// The probe's own VCOM byte ledger; every field is a free-running counter, differenced by
/// wrapping subtraction. The high-water marks only ever ratchet up, so their difference is the
/// growth inside a bracket, not a count.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VcomCounters {
    /// Bytes the bridge took off the UART.
    pub uart_rx: u32,
    /// Bytes the CDC IN endpoint accepted for the host; a probe that stages toward the endpoint
    /// credits them at the stream and takes the credit back when a stage is discarded, so the
    /// field is exact once the stage drains.
    pub cdc_in: u32,
    /// Bytes taken off the UART that the host would not take, a discarded stage included.
    pub host_dropped: u32,
    /// Bytes dropped because the probe's receive ring was full.
    pub ring_full: u32,
    /// Error-flagged reception, in the unit the `units` word declares.
    pub frame_err: u32,
    /// Receive overrun, in the unit the `units` word declares.
    pub overrun: u32,
    /// Bytes the probe accepted from the CDC OUT endpoint.
    pub cdc_out: u32,
    /// Bytes the probe's transmit path put on the wire.
    pub uart_tx: u32,
    /// Receive-DMA faults the probe recovered from.
    pub rx_dma_err: u32,
    /// Counter coverage; `None` denotes a legacy response with unknown coverage.
    pub valid: Option<u32>,
    /// Two bits per slot, evidence quality per sourced counter; `None` before the v2 reply.
    pub grade: Option<u32>,
    /// One bit per slot, set where the counter is episodes rather than bytes; `None` before v2.
    pub units: Option<u32>,
}

/// The six receive-ledger slots every reply carries, and the identity three inside them.
pub const VALID_RX_LEDGER: u32 = 0b11_1111;
const VALID_IDENTITY: u32 = 0b000111;
pub const VALID_RX_DMA_ERR: u32 = 1 << SLOT_RX_DMA_ERR;

/// Slot indices into the mask words, in wire order.
pub const SLOT_RING_FULL: usize = 3;
pub const SLOT_FRAME_ERR: usize = 4;
pub const SLOT_OVERRUN: usize = 5;
pub const SLOT_CDC_OUT: usize = 6;
pub const SLOT_UART_TX: usize = 7;
pub const SLOT_RX_DMA_ERR: usize = 8;

pub const GRADE_LOWER_BOUND: u32 = 1;
pub const GRADE_EXACT: u32 = 2;

/// One slot's grade out of the two-bit-per-slot word; `None` on a reply without the word.
pub fn slot_grade(grade: Option<u32>, slot: usize) -> Option<u32> {
    grade.map(|g| (g >> (2 * slot)) & 0b11)
}

/// Whether a slot's counter is episodes rather than bytes; a reply without the word means bytes.
pub fn slot_episodes(units: Option<u32>, slot: usize) -> bool {
    units.is_some_and(|u| u & (1 << slot) != 0)
}

/// Whether the seam behind this reply sources the slot at all. A legacy reply carries no coverage
/// word, so its slots are taken at face value; a slot with its bit clear reads zero by contract
/// and means "I cannot tell you", never "nothing happened".
pub fn slot_sourced(valid: Option<u32>, slot: usize) -> bool {
    valid.is_none_or(|mask| mask & (1 << slot) != 0)
}

/// One counter rendered against its own coverage, grade and unit: `unsourced` where the seam does
/// not source the slot, `>=` where the seam grades it a lower bound, and an episode count labeled
/// so it is never read as bytes.
pub fn render_counter(c: &VcomCounters, value: u32, slot: usize) -> String {
    if !slot_sourced(c.valid, slot) {
        return "unsourced".into();
    }
    let bound = match slot_grade(c.grade, slot) {
        Some(GRADE_LOWER_BOUND) => ">=",
        _ => "",
    };
    let unit = if slot_episodes(c.units, slot) {
        " episodes"
    } else {
        ""
    };
    format!("{bound}{value}{unit}")
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConservationClass {
    CompleteClean,
    PartialClean,
    ObservedLoss,
    IdentityFailed,
    /// A DMA channel was reinitialised under live traffic, so the bracket's counters are not
    /// evidence of anything.
    DmaFaulted,
}

impl ConservationClass {
    pub fn key(self) -> &'static str {
        match self {
            Self::CompleteClean => "complete-clean",
            Self::PartialClean => "partial-clean",
            Self::ObservedLoss => "observed-loss",
            Self::IdentityFailed => "identity-failed",
            Self::DmaFaulted => "dma-faulted",
        }
    }
}

impl VcomCounters {
    pub fn since(self, base: Self) -> Self {
        Self {
            uart_rx: self.uart_rx.wrapping_sub(base.uart_rx),
            cdc_in: self.cdc_in.wrapping_sub(base.cdc_in),
            host_dropped: self.host_dropped.wrapping_sub(base.host_dropped),
            ring_full: self.ring_full.wrapping_sub(base.ring_full),
            frame_err: self.frame_err.wrapping_sub(base.frame_err),
            overrun: self.overrun.wrapping_sub(base.overrun),
            cdc_out: self.cdc_out.wrapping_sub(base.cdc_out),
            uart_tx: self.uart_tx.wrapping_sub(base.uart_tx),
            rx_dma_err: self.rx_dma_err.wrapping_sub(base.rx_dma_err),
            valid: match (self.valid, base.valid) {
                (Some(a), Some(b)) => Some(a & b),
                _ => None,
            },
            // Coverage words are constants per firmware, so the closing sample's carry over.
            grade: base.grade.and(self.grade),
            units: base.units.and(self.units),
        }
    }

    /// Every byte the bridge took off the UART is accounted for on the host side or charged to a
    /// loss site.
    pub fn identity_holds(self) -> bool {
        self.uart_rx == self.cdc_in.wrapping_add(self.host_dropped)
    }

    /// The probe charged a loss to a named site.
    pub fn lost_any(self) -> bool {
        self.host_dropped != 0 || self.ring_full != 0 || self.frame_err != 0 || self.overrun != 0
    }

    /// The sourced DMA-error counter moved: a channel was reinitialised under live traffic.
    pub fn dma_faulted(self) -> bool {
        let sourced = |bit| self.valid.is_none_or(|mask: u32| mask & bit != 0);
        self.rx_dma_err != 0 && sourced(VALID_RX_DMA_ERR)
    }
}

/// The probe's ledger sampled either side of a verb's windows; the bracket spans every window the
/// verb ran.
#[derive(Clone, Copy, Debug)]
pub struct Conservation {
    pub before: VcomCounters,
    pub after: VcomCounters,
}

impl Conservation {
    /// What the bracketed windows put through the bridge.
    pub fn delta(&self) -> VcomCounters {
        self.after.since(self.before)
    }

    /// The identity holds at the closing sample, and nothing inside the bracket was lost.
    pub fn clean(&self) -> bool {
        matches!(
            self.classification(),
            ConservationClass::CompleteClean | ConservationClass::PartialClean
        )
    }

    pub fn classification(&self) -> ConservationClass {
        let delta = self.delta();
        let valid = delta.valid;
        // First: a recovered fault means nothing else in the bracket is evidence.
        if delta.dma_faulted() {
            return ConservationClass::DmaFaulted;
        }
        if valid.is_some_and(|mask| mask & VALID_IDENTITY == VALID_IDENTITY)
            && !delta.identity_holds()
        {
            return ConservationClass::IdentityFailed;
        }
        let loss = [
            delta.host_dropped,
            delta.ring_full,
            delta.frame_err,
            delta.overrun,
        ];
        let loss_bits = [1 << 2, 1 << 3, 1 << 4, 1 << 5];
        if loss
            .into_iter()
            .zip(loss_bits)
            .any(|(n, bit)| n != 0 && valid.is_none_or(|mask| mask & bit != 0))
        {
            return ConservationClass::ObservedLoss;
        }
        // Complete needs more than coverage: a lower-bound counter at zero proves nothing, so
        // every receive-ledger slot must also grade exact. A v1 reply cannot say, so it stays
        // partial.
        let exact = delta
            .grade
            .is_some_and(|g| (0..6).all(|slot| (g >> (2 * slot)) & 0b11 == GRADE_EXACT));
        if valid.is_some_and(|mask| mask & VALID_RX_LEDGER == VALID_RX_LEDGER) && exact {
            ConservationClass::CompleteClean
        } else {
            ConservationClass::PartialClean
        }
    }
}

/// Both ends' credit ledger for a credit-windowed run.
#[derive(Clone, Copy, Debug, Default)]
pub struct Credit {
    /// Highest cumulative grant the host announced.
    pub granted: u64,
    /// Highest cumulative grant the DUT transmitter obeyed.
    pub obeyed: u64,
}

/// One finished window: both directions' ledgers and everything needed to judge them.
#[derive(Clone, Debug)]
pub struct Run {
    pub plan: Plan,
    pub legs: Vec<Leg>,
    /// UART framing errors the DUT counted.
    pub ferr: u32,
    /// UART receive overruns the DUT counted.
    pub ovr: u32,
    /// Bytes the DUT's receive ring discarded before its software read them.
    pub drop: u32,
    /// Abort waits the DUT's DMA driver gave up on with the channel still busy; sticky across
    /// `STATS RESET`, so a nonzero read taints every counter taken since it latched.
    pub stuck: u32,
    /// Host wall time from arming to the end of the drain.
    pub measured_secs: f64,
    /// The streams did not end even after the host closed them.
    pub timed_out: bool,
    /// The host closed the streams rather than both transmitters meeting their budgets; normal for
    /// a shape that claims no line rate.
    pub closed: bool,
    pub abandoned: Option<Abandon>,
    pub series: Series,
    pub dap: DapAchieved,
    pub credit: Option<Credit>,
    /// The probe's own ledger over the bracket, where its firmware serves the counter query.
    pub conservation: Option<Conservation>,
}

impl Run {
    pub fn leg(&self, dir: Direction) -> Option<&Leg> {
        self.legs.iter().find(|leg| leg.dir == dir)
    }

    /// Its whole window, or the checkpoint it was abandoned at; every rate is denominated over
    /// this.
    pub fn sustained_secs(&self) -> f64 {
        match &self.abandoned {
            Some(a) => a.at_secs,
            None => self.plan.secs,
        }
    }

    /// One direction's goodput as a fraction of its payload budget, over the seconds the run
    /// actually held.
    pub fn budget_frac(&self, dir: Direction) -> f64 {
        self.leg(dir).map_or(0.0, |leg| {
            leg.budget_frac(&self.plan, self.sustained_secs())
        })
    }

    /// One direction's goodput over the seconds the run actually held.
    pub fn goodput_bps(&self, dir: Direction) -> f64 {
        self.leg(dir)
            .map_or(0.0, |leg| leg.goodput_bps(self.sustained_secs()))
    }

    /// The run's verdict, with a reason for every criterion it failed.
    pub fn verdict(&self) -> Verdict {
        let mut reasons = Vec::new();
        let mut notes = Vec::new();
        if self.legs.is_empty() {
            reasons.push("the run armed no direction".to_string());
        }
        if let Some(a) = &self.abandoned {
            reasons.push(format!(
                "abandoned at checkpoint {} ({:.1}s): {}",
                a.checkpoint, a.at_secs, a.reason
            ));
        }
        if self.timed_out {
            reasons.push("the streams did not end even after the host closed them".to_string());
        }
        if self.ferr != 0 || self.ovr != 0 || self.drop != 0 {
            reasons.push(format!(
                "the DUT UART counted {} framing errors, {} overruns and {} ring-dropped bytes",
                self.ferr, self.ovr, self.drop
            ));
        }
        if self.stuck != 0 {
            reasons.push(format!(
                "the DUT latched {} stuck DMA aborts: a channel was left running, so every \
                 counter since is suspect",
                self.stuck
            ));
        }
        for leg in &self.legs {
            let name = label(leg.dir);
            let rx = &leg.rx;
            // The goodput bar never judges an unclaimed leg, so the close is its only witness.
            if self.closed && !leg.claimed {
                notes.push(format!(
                    "{name}: claims no line rate, and the host closed the run short of the \
                     offered budget"
                ));
            }
            if !rx.eos {
                reasons.push(format!("{name}: no end-of-stream frame arrived"));
            }
            if !rx.clean() {
                reasons.push(format!(
                    "{name}: {} corrupt, {} missing, {} duplicate, {} unresolved bytes",
                    rx.corrupt, rx.missing, rx.duplicate, rx.unresolved
                ));
            }
            if !leg.reconciles() {
                reasons.push(format!(
                    "{name}: the ledgers do not reconcile ({} sent, {} accounted)",
                    leg.tx_payload,
                    rx.accounted()
                ));
            }
            if leg.claimed {
                let goodput = leg.goodput_bps(self.sustained_secs());
                if goodput < self.plan.pass_bar_bps(leg.dir) {
                    reasons.push(format!(
                        "{name}: {goodput:.0} B/s is {:.1}% of the {:.0} B/s payload budget",
                        self.budget_frac(leg.dir) * 100.0,
                        self.plan.payload_budget_bps(leg.dir),
                    ));
                }
            }
        }
        if let Some(c) = self.conservation {
            let after = c.after;
            if !after.identity_holds() {
                reasons.push(format!(
                    "probe: the counter ledger does not reconcile ({} off the UART, {} to the \
                     host, {} dropped)",
                    after.uart_rx, after.cdc_in, after.host_dropped
                ));
            }
            let d = c.delta();
            if d.lost_any() {
                reasons.push(format!(
                    "probe: {} bytes dropped by the host leg, {} to a full receive ring, {} \
                     error-flagged frames, {} overruns",
                    d.host_dropped, d.ring_full, d.frame_err, d.overrun
                ));
            }
            // The disposition `stuck` has: a recovered fault taints every counter around it.
            if d.dma_faulted() {
                reasons.push(format!(
                    "probe: recovered {} receive-DMA fault(s) mid-window: a channel was \
                     reinitialised under live traffic, so the window's counters are not evidence",
                    d.rx_dma_err
                ));
            }
        }
        Verdict {
            pass: reasons.is_empty(),
            reasons,
            notes,
        }
    }

    /// A rung's grade. `Qualified` relaxes only the goodput floor and the probe's loss counters;
    /// loss no counter saw is never qualified, because nothing bounds it.
    pub fn grade(&self) -> Grade {
        if self.verdict().pass {
            return Grade::Clean;
        }
        let structural = self.legs.is_empty()
            || self.abandoned.is_some()
            || self.timed_out
            || self.stuck != 0
            || self
                .legs
                .iter()
                .any(|leg| !leg.rx.eos || !leg.rx.clean() || !leg.reconciles())
            || self
                .conservation
                .is_some_and(|c| !c.after.identity_holds() || c.delta().dma_faulted());
        if structural {
            return Grade::Fail;
        }
        let carried = self
            .legs
            .iter()
            .filter(|leg| leg.claimed)
            .all(|leg| self.budget_frac(leg.dir) >= QUALIFIED_FRAC);
        if carried {
            Grade::Qualified
        } else {
            Grade::Fail
        }
    }
}

/// Fraction of the payload budget a rung must carry to count as `Grade::Qualified`.
pub const QUALIFIED_FRAC: f64 = 0.999;

/// How well a rung held, in the two senses a ceiling is asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Grade {
    Fail,
    /// Carried at least [`QUALIFIED_FRAC`] of budget; any loss was counted, never silent.
    Qualified,
    /// Byte-exact with every probe counter still at zero.
    Clean,
}

/// A run's outcome and why it came out that way.
#[derive(Clone, Debug)]
pub struct Verdict {
    pub pass: bool,
    /// One line per failed criterion; empty on a pass.
    pub reasons: Vec<String>,
    /// Facts no criterion judges, named so they are visible on a pass.
    pub notes: Vec<String>,
}

/// The DUT's receive ledger, as the report line carries it.
pub fn dut_rx(report: &V1Report) -> RxCounters {
    RxCounters {
        verified: u64::from(report.verified),
        corrupt: u64::from(report.corrupt),
        missing: u64::from(report.missing),
        duplicate: u64::from(report.duplicate),
        unresolved: u64::from(report.unresolved),
        frames: report.rx_frames,
        resyncs: report.resyncs,
        eos: report.eos,
        // The report carries the flag, not the total the end-of-stream frame held.
        eos_total: 0,
        credit: u64::from(report.credit),
        elapsed_us: report.elapsed_us,
        onset_us: report.onset_us,
        maxgap_us: report.maxgap_us,
    }
}

/// Fold the host engines' counters and the DUT's ledger into the run's per-direction legs.
pub fn legs(
    plan: &Plan,
    host_tx: Option<TxCounters>,
    host_rx: Option<RxCounters>,
    dut: &V1Report,
) -> Vec<Leg> {
    let mut legs = Vec::new();
    if let Some(tx) = host_tx {
        legs.push(Leg {
            dir: Direction::HostToDut,
            claimed: plan.shape.claims_line_rate(Direction::HostToDut),
            tx_payload: tx.payload,
            tx_frames: tx.frames,
            rx: dut_rx(dut),
            rx_eos_total: None,
        });
    }
    if let Some(rx) = host_rx {
        legs.push(Leg {
            dir: Direction::DutToHost,
            claimed: plan.shape.claims_line_rate(Direction::DutToHost),
            tx_payload: u64::from(dut.tx),
            tx_frames: dut.tx_frames,
            rx,
            rx_eos_total: Some(rx.eos_total),
        });
    }
    legs
}

#[cfg(test)]
mod tests {
    use super::*;
    use daplink_test_protocol::v1::{Encoder, Verifier};

    const BAUD: u32 = 2_625_000;

    fn plan(shape: Shape, secs: f64) -> Plan {
        Plan {
            shape,
            seeds: Seeds::default(),
            interval: DEFAULT_INTERVAL,
            secs,
            dut_baud: BAUD,
            probe_baud: BAUD,
            host_frac: 1.0,
            burst: None,
            burst_host: false,
            burst_dut: false,
            credit_window: CREDIT_WINDOW,
        }
    }

    // --- budget arithmetic ---

    #[test]
    fn the_payload_fraction_is_the_interval_over_the_interval_plus_a_frame() {
        let p = plan(Shape::Duplex, 30.0);
        assert!((p.payload_fraction() - 2048.0 / 2072.0).abs() < 1e-12);
        assert!((p.payload_fraction() - 0.98842).abs() < 1e-5);
        let dense = Plan { interval: 64, ..p };
        assert!((dense.payload_fraction() - 64.0 / 88.0).abs() < 1e-12);
        // An interval below one frame is raised to a frame, as the engines do.
        let floor = Plan { interval: 1, ..p };
        assert!((floor.payload_fraction() - 0.5).abs() < 1e-12);
    }

    #[test]
    fn the_budget_is_the_window_at_the_payload_rate() {
        let p = plan(Shape::Duplex, 30.0);
        assert!((p.line_bps(Direction::DutToHost) - 262_500.0).abs() < 1e-9);
        let expected = (30.0_f64 * 262_500.0 * 2048.0 / 2072.0).round() as u32;
        assert_eq!(p.budget_h2d(), expected);
        assert_eq!(p.budget_d2h(), expected);
        assert!(
            (p.pass_bar_bps(Direction::DutToHost)
                - p.payload_budget_bps(Direction::DutToHost) * 0.99)
                .abs()
                < 1e-9
        );
    }

    #[test]
    fn each_direction_is_rated_against_its_own_transmitter() {
        // Ends whose divisors land apart: rating a leg against the other's clock fails it.
        let p = Plan {
            dut_baud: 933_333,
            probe_baud: 922_509,
            ..plan(Shape::Duplex, 30.0)
        };
        assert!(
            p.payload_budget_bps(Direction::HostToDut) < p.payload_budget_bps(Direction::DutToHost)
        );
        // The host leg at its own line rate clears its bar, and would miss the DUT's.
        let flat_out = p.payload_budget_bps(Direction::HostToDut);
        assert!(flat_out >= p.pass_bar_bps(Direction::HostToDut));
        assert!(flat_out < p.pass_bar_bps(Direction::DutToHost));
        assert!(p.budget_h2d() < p.budget_d2h());
    }

    #[test]
    fn a_shape_budgets_only_the_directions_it_arms() {
        let out = plan(Shape::Out, 30.0);
        assert_eq!(out.shape.dir_mask(), DirMask::RX);
        assert_eq!(out.budget_d2h(), 0);
        assert!(out.budget_h2d() > 0);
        assert!(out.host_verifier().is_none());

        let back = plan(Shape::Back, 30.0);
        assert_eq!(back.shape.dir_mask(), DirMask::TX);
        assert_eq!(back.budget_h2d(), 0);
        assert!(back.host_encoder().is_none());

        let duplex = plan(Shape::Duplex, 30.0);
        assert_eq!(duplex.shape.dir_mask(), DirMask::TX | DirMask::RX);
    }

    #[test]
    fn a_return_priority_run_paces_the_host_leg_and_claims_only_the_return() {
        let p = Plan {
            host_frac: RETURN_PRIORITY_FRAC,
            ..plan(Shape::ReturnPriority, 30.0)
        };
        assert_eq!(p.budget_h2d(), p.budget_d2h() / 4);
        assert!((p.host_pace_bps() - p.line_bps(Direction::DutToHost) / 4.0).abs() < 1e-9);
        assert!(!p.shape.claims_line_rate(Direction::HostToDut));
        assert!(p.shape.claims_line_rate(Direction::DutToHost));
        let demand = p.demand(0.0);
        assert!((demand.h2d_bps - p.line_bps(Direction::DutToHost) / 4.0).abs() < 1e-9);
        assert!((demand.d2h_bps - p.line_bps(Direction::DutToHost)).abs() < 1e-9);
    }

    #[test]
    fn the_paced_alternative_is_only_worth_offering_where_it_fits() {
        let fs = crate::budget::BusBudget::for_packet(64);
        let over = |baud| {
            let p = Plan {
                dut_baud: baud,
                probe_baud: baud,
                ..plan(Shape::Duplex, 30.0)
            };
            assert!(!crate::budget::clamp(&p.demand(0.0), &fs).fits);
            crate::budget::clamp(&p.as_return_priority().demand(0.0), &fs).fits
        };
        assert!(over(6_000_000));
        assert!(!over(9_375_000));
    }

    #[test]
    fn credit_mode_claims_no_line_rate_and_grants_ahead_of_its_verifier() {
        let p = plan(Shape::Credit, 30.0);
        assert!(!p.shape.claims_line_rate(Direction::HostToDut));
        assert!(!p.shape.claims_line_rate(Direction::DutToHost));
        assert_eq!(p.grant_for(0), CREDIT_WINDOW);
        assert_eq!(p.grant_for(100_000), 100_000 + CREDIT_WINDOW);
    }

    #[test]
    fn the_start_line_is_the_plan_in_the_duts_frame() {
        let p = plan(Shape::Duplex, 30.0);
        let budget = p.budget_h2d();
        assert_eq!(
            p.start_line(),
            format!("V1 START 3 1592590337 1592590338 2048 {budget} {budget}\n")
        );
        let shaped = Plan {
            burst: Some(BurstSpec {
                bytes: 512,
                idle_ms: 3,
            }),
            burst_dut: true,
            ..plan(Shape::Credit, 30.0)
        };
        let line = shaped.start_line();
        assert!(line.contains(" BURST 512 3 CREDIT\n"), "{line}");
        // A burst meant only for the host leg never reaches the DUT.
        let host_only = Plan {
            burst_dut: false,
            burst_host: true,
            ..shaped
        };
        assert!(!host_only.start_line().contains("BURST"));
        // Every line the tool emits is one the DUT's parser accepts.
        for p in [p, shaped, host_only] {
            assert!(
                v1::parse(p.start_line().trim()).is_some(),
                "{}",
                p.start_line()
            );
        }
    }

    // --- ledger reconciliation and verdicts ---

    /// A DUT report of a clean run: `tx` payload bytes emitted, `rx` verified.
    fn clean_report(tx: u32, rx: u32, interval: u32) -> V1Report {
        let (frames, rx_frames) = (tx / interval + 1, rx / interval + 1);
        V1Report {
            tx,
            tx_frames: frames,
            verified: rx,
            corrupt: 0,
            missing: 0,
            duplicate: 0,
            unresolved: 0,
            rx_frames,
            resyncs: 0,
            eos: true,
            elapsed_us: 30_000_000,
            onset_us: 120,
            maxgap_us: 900,
            ferr: 0,
            ovr: 0,
            drop: 0,
            credit: 0,
        }
    }

    /// A host receive ledger of the same clean run.
    fn clean_rx(bytes: u64, frames: u32) -> RxCounters {
        RxCounters {
            verified: bytes,
            corrupt: 0,
            missing: 0,
            duplicate: 0,
            unresolved: 0,
            frames,
            resyncs: 0,
            eos: true,
            eos_total: bytes,
            credit: 0,
            elapsed_us: 30_000_000,
            onset_us: 90,
            maxgap_us: 1_100,
        }
    }

    /// A run in which each transmitter delivered its whole budget: a passing rung.
    fn clean_run(plan: Plan) -> Run {
        let (out, back) = (plan.budget_h2d(), plan.budget_d2h());
        let report = clean_report(back, out, plan.interval);
        let tx = TxCounters {
            payload: u64::from(out),
            frames: out / plan.interval + 1,
            credit: 0,
            complete: true,
        };
        let rx = clean_rx(u64::from(back), back / plan.interval + 1);
        Run {
            legs: legs(&plan, Some(tx), Some(rx), &report),
            plan,
            ferr: 0,
            ovr: 0,
            drop: 0,
            stuck: 0,
            measured_secs: plan.secs + 1.0,
            timed_out: false,
            closed: false,
            abandoned: None,
            series: Series::default(),
            dap: DapAchieved::default(),
            credit: None,
            conservation: None,
        }
    }

    #[test]
    fn a_clean_full_rate_run_passes_on_every_criterion() {
        let run = clean_run(plan(Shape::Duplex, 30.0));
        let verdict = run.verdict();
        assert!(verdict.pass, "{:?}", verdict.reasons);
        for dir in DIRECTIONS {
            let leg = run.leg(dir).expect("both legs");
            assert!(leg.reconciles());
            assert!((run.budget_frac(dir) - 1.0).abs() < 1e-3);
        }
    }

    /// A probe ledger that took `uart_rx` bytes off the UART and delivered all of them.
    fn ledger(uart_rx: u32) -> VcomCounters {
        VcomCounters {
            uart_rx,
            cdc_in: uart_rx,
            ..VcomCounters::default()
        }
    }

    #[test]
    fn free_running_counters_are_differenced_by_wrapping() {
        let before = ledger(u32::MAX - 100);
        let after = ledger(923);
        assert_eq!(after.since(before).uart_rx, 1024);
    }

    /// A grade word claiming exact on every receive-ledger slot.
    const EXACT_LEDGER: u32 = 0b10_10_10_10_10_10;

    #[test]
    fn conservation_classifies_full_partial_loss_and_identity_failure() {
        let classify = |after: VcomCounters| {
            Conservation {
                before: VcomCounters {
                    valid: after.valid,
                    grade: after.grade,
                    units: after.units,
                    ..VcomCounters::default()
                },
                after,
            }
            .classification()
        };
        assert_eq!(
            classify(VcomCounters {
                valid: Some(VALID_RX_LEDGER),
                grade: Some(EXACT_LEDGER),
                units: Some(0),
                ..ledger(10)
            }),
            ConservationClass::CompleteClean
        );
        // Coverage without exactness proves nothing was seen, not that nothing was lost.
        assert_eq!(
            classify(VcomCounters {
                valid: Some(VALID_RX_LEDGER),
                ..ledger(10)
            }),
            ConservationClass::PartialClean
        );
        assert_eq!(
            classify(VcomCounters {
                valid: Some(VALID_RX_LEDGER),
                grade: Some(EXACT_LEDGER & !(0b11 << 6) | (GRADE_LOWER_BOUND << 6)),
                units: Some(0),
                ..ledger(10)
            }),
            ConservationClass::PartialClean
        );
        assert_eq!(classify(ledger(10)), ConservationClass::PartialClean);
        assert_eq!(
            classify(VcomCounters {
                cdc_in: 9,
                host_dropped: 1,
                ..ledger(10)
            }),
            ConservationClass::ObservedLoss
        );
        assert_eq!(
            classify(VcomCounters {
                cdc_in: 9,
                valid: Some(VALID_IDENTITY),
                ..ledger(10)
            }),
            ConservationClass::IdentityFailed
        );
        assert_eq!(
            classify(VcomCounters {
                cdc_in: 9,
                ..ledger(10)
            }),
            ConservationClass::PartialClean
        );
        // A recovered DMA fault outranks everything else the bracket seems to say.
        assert_eq!(
            classify(VcomCounters {
                rx_dma_err: 1,
                valid: Some(VALID_RX_LEDGER | VALID_RX_DMA_ERR),
                grade: Some(EXACT_LEDGER),
                units: Some(0),
                ..ledger(10)
            }),
            ConservationClass::DmaFaulted
        );
        // An unsourced DMA-error slot cannot fault a bracket.
        assert_eq!(
            classify(VcomCounters {
                rx_dma_err: 3,
                valid: Some(VALID_RX_LEDGER),
                ..ledger(10)
            }),
            ConservationClass::PartialClean
        );
    }

    #[test]
    fn a_lower_bound_counter_renders_as_a_bound_and_an_unsourced_one_says_so() {
        let c = VcomCounters {
            valid: Some(VALID_RX_LEDGER),
            grade: Some(
                (GRADE_LOWER_BOUND << (2 * SLOT_RING_FULL)) | (GRADE_EXACT << (2 * SLOT_OVERRUN)),
            ),
            units: Some(1 << SLOT_OVERRUN),
            ..VcomCounters::default()
        };
        assert_eq!(render_counter(&c, 7, SLOT_RING_FULL), ">=7");
        assert_eq!(render_counter(&c, 3, SLOT_OVERRUN), "3 episodes");
        // Outside the coverage word the slot reads zero by contract, which is not a measurement.
        assert_eq!(render_counter(&c, 0, SLOT_UART_TX), "unsourced");
    }

    #[test]
    fn a_reconciling_probe_ledger_leaves_a_clean_run_passing() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        run.conservation = Some(Conservation {
            before: ledger(4096),
            after: ledger(1_000_000),
        });
        let verdict = run.verdict();
        assert!(verdict.pass, "{:?}", verdict.reasons);
    }

    #[test]
    fn a_probe_ledger_that_does_not_reconcile_fails_an_otherwise_clean_run() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        run.conservation = Some(Conservation {
            before: ledger(4096),
            after: VcomCounters {
                uart_rx: 1_000_000,
                cdc_in: 999_000,
                ..VcomCounters::default()
            },
        });
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(verdict.reasons.iter().any(|r| r.contains("probe:")));
    }

    #[test]
    fn a_loss_charged_to_a_named_site_fails_the_run_and_names_it() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        run.conservation = Some(Conservation {
            before: ledger(4096),
            after: VcomCounters {
                ring_full: 318,
                ..ledger(1_000_000)
            },
        });
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(verdict
            .reasons
            .iter()
            .any(|r| r.contains("318 to a full receive ring")));
    }

    #[test]
    fn a_loss_before_the_bracket_does_not_fail_the_windows_inside_it() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        run.conservation = Some(Conservation {
            before: VcomCounters {
                ring_full: 318,
                ..ledger(4096)
            },
            after: VcomCounters {
                ring_full: 318,
                ..ledger(1_000_000)
            },
        });
        let verdict = run.verdict();
        assert!(verdict.pass, "{:?}", verdict.reasons);
    }

    #[test]
    fn a_missing_end_of_stream_frame_fails_the_run_rather_than_shortening_it() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        let leg = run
            .legs
            .iter_mut()
            .find(|leg| leg.dir == Direction::DutToHost)
            .expect("the return leg");
        leg.rx.eos = false;
        leg.rx.eos_total = 0;
        leg.rx_eos_total = Some(0);
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(verdict.reasons.iter().any(|r| r.contains("end-of-stream")));
        assert!(verdict.reasons.iter().any(|r| r.contains("reconcile")));
    }

    #[test]
    fn an_unresolved_byte_fails_the_run_even_though_the_ledger_reconciles() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        let leg = &mut run.legs[0];
        leg.rx.verified -= 128;
        leg.rx.unresolved += 128;
        assert!(leg.reconciles(), "the conservation law still holds");
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(verdict.reasons.iter().any(|r| r.contains("unresolved")));
    }

    #[test]
    fn a_lost_byte_breaks_reconciliation_when_the_receiver_never_accounts_for_it() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        run.legs[0].rx.verified -= 1;
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(verdict.reasons.iter().any(|r| r.contains("reconcile")));
    }

    #[test]
    fn a_recovered_dma_fault_fails_an_otherwise_perfect_run() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        run.conservation = Some(Conservation {
            before: ledger(4096),
            after: VcomCounters {
                rx_dma_err: 1,
                ..ledger(1_000_000)
            },
        });
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(
            verdict.reasons.iter().any(|r| r.contains("reinitialised")),
            "{:?}",
            verdict.reasons
        );
        assert_eq!(
            run.grade(),
            Grade::Fail,
            "a fault is structural, never qualified"
        );
    }

    #[test]
    fn a_stuck_dma_abort_fails_an_otherwise_perfect_run() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        assert!(run.verdict().pass);
        run.stuck = 1;
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(
            verdict.reasons.iter().any(|r| r.contains("stuck")),
            "{:?}",
            verdict.reasons
        );
        // The latch means loss no counter bounds, so the rung can never be Qualified either.
        assert_eq!(run.grade(), Grade::Fail);
    }

    #[test]
    fn a_uart_error_counter_fails_an_otherwise_perfect_run() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        run.ovr = 1;
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(verdict.reasons.iter().any(|r| r.contains("overruns")));
    }

    #[test]
    fn goodput_below_the_bar_is_reported_as_its_own_number() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        let short = run.legs[0].rx.verified / 10;
        run.legs[0].rx.verified -= short;
        run.legs[0].rx.unresolved += short;
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(
            verdict.reasons.iter().any(|r| r.contains("90.0%")),
            "{:?}",
            verdict.reasons
        );
    }

    #[test]
    fn a_deliberately_slowed_leg_is_not_judged_against_the_line_rate() {
        let p = Plan {
            host_frac: RETURN_PRIORITY_FRAC,
            ..plan(Shape::ReturnPriority, 30.0)
        };
        let run = clean_run(p);
        // The host leg delivers a quarter of the budget by construction and still passes.
        let host = run.leg(Direction::HostToDut).expect("the host leg");
        assert!(!host.claimed);
        assert!((run.budget_frac(Direction::HostToDut) - 0.25).abs() < 1e-3);
        let verdict = run.verdict();
        assert!(verdict.pass, "{:?}", verdict.reasons);
    }

    #[test]
    fn a_timed_out_run_fails_whatever_its_ledgers_say() {
        let mut run = clean_run(plan(Shape::Duplex, 30.0));
        run.timed_out = true;
        assert!(!run.verdict().pass);
    }

    // --- closing a run the host ended, rather than dropping it ---

    /// A run the host closed short of its budget: each transmitter emitted `frac` of it.
    fn closed_run(plan: Plan, frac: f64) -> Run {
        let out = (f64::from(plan.budget_h2d()) * frac) as u32;
        let back = (f64::from(plan.budget_d2h()) * frac) as u32;
        let report = clean_report(back, out, plan.interval);
        let tx = TxCounters {
            payload: u64::from(out),
            frames: out / plan.interval + 1,
            credit: 0,
            complete: true,
        };
        let rx = clean_rx(u64::from(back), back / plan.interval + 1);
        let took = (plan.secs * frac * 1e6) as u32;
        let mut legs = legs(&plan, Some(tx), Some(rx), &report);
        for leg in &mut legs {
            leg.rx.elapsed_us = took;
        }
        Run {
            legs,
            plan,
            ferr: 0,
            ovr: 0,
            drop: 0,
            stuck: 0,
            measured_secs: plan.secs + 1.0,
            timed_out: false,
            closed: true,
            abandoned: None,
            series: Series::default(),
            dap: DapAchieved::default(),
            credit: Some(Credit {
                granted: u64::from(back) + CREDIT_WINDOW,
                obeyed: u64::from(back),
            }),
            conservation: None,
        }
    }

    #[test]
    fn a_credit_run_the_host_closed_is_judged_on_its_ledgers_alone() {
        // The shape is budgeted at a line rate it never claims, so the host always closes it.
        let run = closed_run(plan(Shape::Credit, 30.0), 0.4);
        assert!(run.closed);
        for dir in DIRECTIONS {
            let leg = run.leg(dir).expect("both legs");
            assert!(leg.reconciles());
            assert!(!leg.claimed);
        }
        let verdict = run.verdict();
        assert!(verdict.pass, "{:?}", verdict.reasons);
        // The close is still named, one note per unclaimed leg, so it is visible on the pass.
        assert_eq!(verdict.notes.len(), 2, "{:?}", verdict.notes);
        assert!(verdict
            .notes
            .iter()
            .all(|n| n.contains("the host closed the run")));
    }

    #[test]
    fn a_duplex_run_the_host_closed_fails_on_the_goodput_it_actually_delivered() {
        let run = closed_run(plan(Shape::Duplex, 30.0), 0.4);
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(
            verdict.reasons.iter().any(|r| r.contains("40.0%")),
            "{:?}",
            verdict.reasons
        );
        // Both legs claim line rate, so the goodput bar judges the close and no note doubles it.
        assert!(verdict.notes.is_empty(), "{:?}", verdict.notes);
    }

    #[test]
    fn a_closed_return_priority_run_names_its_unclaimed_host_leg() {
        let p = Plan {
            host_frac: RETURN_PRIORITY_FRAC,
            ..plan(Shape::ReturnPriority, 30.0)
        };
        let mut run = clean_run(p);
        assert!(run.verdict().notes.is_empty());
        run.closed = true;
        let verdict = run.verdict();
        assert!(verdict.pass, "{:?}", verdict.reasons);
        assert_eq!(verdict.notes.len(), 1, "{:?}", verdict.notes);
        assert!(verdict.notes[0].starts_with("host -> DUT"));
    }

    #[test]
    fn a_budget_delivered_inside_the_drain_margin_is_not_a_line_rate() {
        // A budget that arrives a second late must not be scored as line rate just because the
        // window was short.
        let p = plan(Shape::Duplex, 10.0);
        let mut run = clean_run(p);
        for leg in &mut run.legs {
            leg.rx.elapsed_us = 11_000_000;
        }
        assert!((run.budget_frac(Direction::DutToHost) - 10.0 / 11.0).abs() < 1e-3);
        let verdict = run.verdict();
        assert!(!verdict.pass, "{:?}", verdict.reasons);
        assert!(verdict.reasons.iter().any(|r| r.contains("90.9%")));
    }

    #[test]
    fn an_abandoned_run_reports_its_time_to_failure_and_the_rate_it_held() {
        let p = plan(Shape::Duplex, 60.0);
        let mut run = closed_run(p, 7.0 / 60.0);
        run.abandoned = Some(Abandon {
            at_secs: 7.0,
            checkpoint: 2,
            reason: "DUT -> host: 41216 missing bytes in this checkpoint".to_string(),
        });
        assert!((run.sustained_secs() - 7.0).abs() < 1e-9);
        // Denominated over the seconds it held, the run was at the budget until it died.
        for dir in DIRECTIONS {
            assert!((run.budget_frac(dir) - 1.0).abs() < 1e-2, "{dir:?}");
        }
        let verdict = run.verdict();
        assert!(!verdict.pass);
        assert!(verdict
            .reasons
            .iter()
            .any(|r| r.contains("abandoned at checkpoint 2 (7.0s)")));
    }

    // --- checkpoints: reads of the running ledger, and the shapes they make ---

    /// A host transmit ledger after `payload` bytes.
    fn tx_at(payload: u64) -> TxCounters {
        TxCounters {
            payload,
            frames: 0,
            credit: 0,
            complete: false,
        }
    }

    /// A host receive ledger after `verified` bytes, all of them clean.
    fn rx_at(verified: u64) -> RxCounters {
        RxCounters {
            verified,
            ..RxCounters::default()
        }
    }

    /// Drive a sampler over `secs` seconds, with both directions delivering the per-second fraction
    /// `rate` gives.
    fn sweep(
        plan: &Plan,
        secs: f64,
        rate: impl Fn(f64) -> f64,
        loss: impl Fn(f64) -> u64,
    ) -> (Series, Option<Abandon>) {
        let mut sampler = Sampler::new(plan, CHECKPOINT_SECS, CHECKPOINT_FLOOR);
        let (mut payload, mut lost) = (0.0_f64, 0u64);
        let (mut t, step) = (0.0_f64, 0.02_f64);
        let mut abandon = None;
        while t < secs && abandon.is_none() {
            t += step;
            payload += plan.payload_budget_bps(Direction::DutToHost) * step * rate(t);
            lost += loss(t);
            abandon = sampler.sample(
                t,
                Some(tx_at(payload as u64)),
                Some(RxCounters {
                    missing: lost,
                    ..rx_at(payload as u64)
                }),
            );
        }
        (sampler.finish(), abandon)
    }

    #[test]
    fn checkpoints_fall_inside_the_window_and_measure_what_it_carried() {
        let p = plan(Shape::Duplex, 60.0);
        let (series, abandon) = sweep(&p, 60.0, |_| 1.0, |_| 0);
        assert!(abandon.is_none());
        assert_eq!(series.points.len(), 11);
        assert_eq!(checkpoints_inside(60.0, CHECKPOINT_SECS), 11);
        assert_eq!(checkpoints_inside(10.0, CHECKPOINT_SECS), 1);
        assert_eq!(checkpoints_inside(2.0, CHECKPOINT_SECS), 0);
        assert_eq!(checkpoints_inside(60.0, 0.0), 0);
        assert!((series.points[0].at_secs - 5.0).abs() < 0.1);
        assert!((series.points[10].at_secs - 55.0).abs() < 0.1);
        for point in &series.points {
            let d2h = point.d2h.expect("the return leg");
            assert!((d2h.frac - 1.0).abs() < 0.02, "{d2h:?}");
            assert_eq!(d2h.lost(), 0);
        }
        assert_eq!(series.trend(Direction::DutToHost), Trend::Holding);
        assert_eq!(series.trend(Direction::HostToDut), Trend::Holding);
    }

    #[test]
    fn a_late_poll_skips_the_checkpoints_it_missed_rather_than_bunching_them() {
        let p = plan(Shape::Duplex, 60.0);
        let mut sampler = Sampler::new(&p, CHECKPOINT_SECS, CHECKPOINT_FLOOR);
        let deliver = |secs: f64| (p.payload_budget_bps(Direction::DutToHost) * secs) as u64;
        // One poll lands three periods late, and the next one arrives on the period after it.
        assert!(sampler
            .sample(17.0, Some(tx_at(deliver(17.0))), Some(rx_at(deliver(17.0))))
            .is_none());
        assert!(sampler
            .sample(
                17.02,
                Some(tx_at(deliver(17.02))),
                Some(rx_at(deliver(17.02)))
            )
            .is_none());
        assert!(sampler
            .sample(20.0, Some(tx_at(deliver(20.0))), Some(rx_at(deliver(20.0))))
            .is_none());
        let series = sampler.finish();
        assert_eq!(series.points.len(), 2);
        assert!((series.points[1].at_secs - 20.0).abs() < 1e-9);
    }

    #[test]
    fn a_window_is_abandoned_at_the_first_checkpoint_that_loses_a_byte() {
        let p = plan(Shape::Duplex, 60.0);
        let (series, abandon) = sweep(&p, 60.0, |_| 1.0, |t| u64::from(t > 7.0 && t < 7.1));
        let abandon = abandon.expect("the loss ends the window");
        assert!(abandon.at_secs > 7.0 && abandon.at_secs < 11.0);
        assert_eq!(abandon.checkpoint, 2);
        assert!(abandon.reason.contains("missing"), "{}", abandon.reason);
        assert_eq!(series.points.len(), 2);
        assert!(series.points[1].d2h.expect("the return leg").missing > 0);
        assert_eq!(series.points[0].d2h.expect("the return leg").lost(), 0);
    }

    #[test]
    fn one_stalled_interval_does_not_end_a_window_the_run_makes_up_for() {
        // A host thread descheduled for half a second, then a token bucket catches it back up.
        let p = plan(Shape::Duplex, 60.0);
        let (series, abandon) = sweep(
            &p,
            60.0,
            |t| match t {
                t if (20.0..20.5).contains(&t) => 0.0,
                t if (20.5..21.0).contains(&t) => 2.0,
                _ => 1.0,
            },
            |_| 0,
        );
        assert!(
            abandon.is_none(),
            "{:?}",
            series.fractions(Direction::DutToHost)
        );
        // A leg that stays down, on the other hand, fails both measures and ends the window.
        let (_, abandon) = sweep(&p, 60.0, |t| f64::from(u8::from(t < 20.0)), |_| 0);
        let abandon = abandon.expect("a leg that stops delivering ends the window");
        assert!(abandon.at_secs < 40.0, "{abandon:?}");
        assert!(abandon.reason.contains("since the run started"));
    }

    #[test]
    fn the_onset_checkpoint_does_not_condemn_a_window_but_the_next_one_does() {
        let p = plan(Shape::Duplex, 60.0);
        // A leg that is slow to start and then holds runs the whole window.
        let (_, abandon) = sweep(&p, 60.0, |t| if t < 3.0 { 0.0 } else { 1.0 }, |_| 0);
        assert!(abandon.is_none());
        // A leg pinned low from the start is abandoned at the second checkpoint, not the first.
        let (series, abandon) = sweep(&p, 60.0, |_| 0.22, |_| 0);
        let abandon = abandon.expect("a pinned leg ends the window");
        assert_eq!(abandon.checkpoint, 2);
        assert!(abandon.at_secs < 11.0);
        assert_eq!(series.trend(Direction::DutToHost), Trend::Structural);
    }

    #[test]
    fn a_steady_slight_shortfall_runs_the_whole_window_and_is_reported_as_one() {
        // A fixed per-packet cost is not a defect to cut a window short for, though it still fails
        // the rung.
        let p = plan(Shape::Duplex, 60.0);
        let (series, abandon) = sweep(&p, 60.0, |_| 0.976, |_| 0);
        assert!(abandon.is_none());
        assert_eq!(series.trend(Direction::DutToHost), Trend::Shortfall);
    }

    #[test]
    fn a_shape_that_claims_no_line_rate_is_not_abandoned_for_missing_one() {
        // A credit run delivers whatever the grants allow; only loss can condemn it.
        let p = plan(Shape::Credit, 60.0);
        let (series, abandon) = sweep(&p, 60.0, |_| 0.3, |_| 0);
        assert!(abandon.is_none());
        assert_eq!(series.points.len(), 11);
        let (_, abandon) = sweep(&p, 60.0, |_| 0.3, |t| u64::from(t > 12.0 && t < 12.1));
        assert!(abandon.is_some());
    }

    #[test]
    fn classify_names_every_series_shape_it_recognizes() {
        // A receive backlog filling between 2 and 15 s.
        assert_eq!(classify(&[1.0, 0.98, 0.85, 0.60, 0.40]), Trend::Declining);
        // A leg pinned at 22% by a scheduling bound, from the first checkpoint.
        assert_eq!(classify(&[0.22, 0.22, 0.23, 0.22]), Trend::Structural);
        // A full-speed CDC re-arm costing a steady 2.4%.
        assert_eq!(classify(&[0.976, 0.976, 0.975, 0.977]), Trend::Shortfall);
        // A receive path that wedged partway through.
        assert_eq!(classify(&[1.0, 1.0, 0.0]), Trend::Cliff);
        // And the two ends of the scale.
        assert_eq!(classify(&[1.0, 0.999, 1.0]), Trend::Holding);
        assert_eq!(classify(&[]), Trend::Unsampled);
        // A single low checkpoint reads as pinned, not as a decay nothing has been seen of yet.
        assert_eq!(classify(&[0.22]), Trend::Structural);
        // Neither steady nor a trend: the class stays open rather than being guessed at.
        assert_eq!(classify(&[0.5, 0.95, 0.6, 0.93]), Trend::Mixed);
    }

    // --- the engines, through an in-process pipe ---

    /// Run `plan`'s host transmitter into a verifier of the same stream, dropping the byte at
    /// `drop` if given.
    fn pipe(plan: &Plan, drop: Option<usize>) -> (TxCounters, RxCounters) {
        let mut encoder = plan.host_encoder().expect("a transmitting shape");
        let mut verifier = v1::Verifier::new(
            Direction::HostToDut,
            plan.seeds.h2d,
            plan.interval,
            u64::from(plan.budget_h2d()),
        );
        verifier.arm(0);
        let mut buf = [0u8; 512];
        let mut at = 0usize;
        let mut now = 0u32;
        while !encoder.complete() {
            let n = encoder.fill(&mut buf, now);
            assert!(n > 0, "the encoder stalled");
            now += 1_000;
            for &byte in &buf[..n] {
                if Some(at) != drop {
                    verifier.feed(&[byte], now);
                }
                at += 1;
            }
        }
        (encoder.counters(), verifier.counters())
    }

    #[test]
    fn a_clean_pipe_reconciles_and_verifies_the_whole_budget() {
        let plan = plan(Shape::Duplex, 0.02);
        let (tx, rx) = pipe(&plan, None);
        assert_eq!(tx.payload, u64::from(plan.budget_h2d()));
        assert_eq!(rx.verified, tx.payload);
        assert!(rx.reconciles() && rx.clean());
        let leg = Leg {
            dir: Direction::HostToDut,
            claimed: true,
            tx_payload: tx.payload,
            tx_frames: tx.frames,
            rx,
            rx_eos_total: Some(rx.eos_total),
        };
        assert!(leg.reconciles());
        // Denominated over the nominal window, the pipe delivers the whole payload budget.
        assert!((leg.budget_frac(&plan, plan.secs) - 1.0).abs() < 1e-3);
    }

    #[test]
    fn a_lossy_pipe_reconciles_into_missing_and_unresolved_bytes() {
        let plan = plan(Shape::Duplex, 0.02);
        let (tx, rx) = pipe(&plan, Some(100));
        assert_eq!(rx.missing, 1);
        assert!(rx.unresolved > 0);
        assert!(!rx.clean());
        let leg = Leg {
            dir: Direction::HostToDut,
            claimed: true,
            tx_payload: tx.payload,
            tx_frames: tx.frames,
            rx,
            rx_eos_total: Some(rx.eos_total),
        };
        assert!(leg.reconciles(), "the conservation law survives the loss");
        assert!(leg.budget_frac(&plan, plan.secs) < 1.0);
    }

    #[test]
    fn a_credit_limited_pipe_moves_exactly_what_the_host_granted() {
        let plan = plan(Shape::Credit, 0.02);
        let budget = u64::from(plan.budget_d2h());
        // The DUT's transmitter, credit-limited, against the host's verifier granting ahead.
        let mut encoder =
            Encoder::new(Direction::DutToHost, plan.seeds.d2h, plan.interval, budget).with_credit();
        let mut verifier =
            Verifier::new(Direction::DutToHost, plan.seeds.d2h, plan.interval, budget);
        verifier.arm(0);
        let mut buf = [0u8; 512];
        let mut now = 0u32;
        let mut granted = 0u64;
        while !encoder.complete() {
            let grant = plan.grant_for(verifier.counters().verified);
            assert!(grant >= granted, "grants are cumulative");
            granted = grant;
            encoder.set_grant(grant);
            let n = encoder.fill(&mut buf, now);
            now += 1_000;
            if n == 0 {
                // Out of credit with nothing verified yet would deadlock the run.
                assert!(
                    verifier.counters().verified < encoder.counters().payload,
                    "the transmitter stalled inside its grant"
                );
                continue;
            }
            verifier.feed(&buf[..n], now);
            assert!(encoder.counters().payload <= granted);
        }
        let rx = verifier.counters();
        assert_eq!(rx.verified, budget);
        assert!(rx.reconciles() && rx.clean());
        assert_eq!(encoder.counters().credit, granted);
    }
}
