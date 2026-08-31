//! Transport-agnostic Bulk-Only Transport plumbing: the CBW-to-CSW data phase over an abstract
//! bulk pair, plus the millisecond clock the class loop feeds the volume.

use core::future::poll_fn;
use core::task::Poll;

use daplink_core::msc_proto::{
    build_csw, clamp_reply_len, dispatch, is_test_unit_ready, reconcile_cbw, should_clear_sense,
    BotVerdict, Cbw, ScsiAction, Sense, CSW_FAILED, CSW_PASSED, SENSE_LUN_NOT_SUPPORTED,
    SENSE_MEDIUM_CHANGED, SENSE_NONE,
};
use daplink_core::MSC_SECTOR_SIZE;

use crate::msc_io::{SectorIo, SectorPoll};

/// Async write half of the bulk pair; a transport blanket-implements it over its IN endpoint.
// The futures are awaited in place by the single MSC task, so no auto-trait bound is wanted.
#[allow(async_fn_in_trait)]
pub trait BulkIn {
    /// Write one bulk IN packet, mapping any transport error to `Err(())`.
    async fn write(&mut self, data: &[u8]) -> Result<(), ()>;
}

/// Async read half of the bulk pair; a transport blanket-implements it over its OUT endpoint.
#[allow(async_fn_in_trait)]
pub trait BulkOut {
    /// Read one bulk OUT packet, returning the byte count or `Err(())` on a transport error.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()>;
}

/// Yield to the executor once, so a pending sector poll does not starve the other tasks.
async fn yield_once() {
    let mut yielded = false;
    poll_fn(|cx| {
        if yielded {
            Poll::Ready(())
        } else {
            yielded = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
    .await
}

/// Millisecond wall clock over a monotonic tick counter, scaling by `NUM`/`DEN` ms per tick and
/// carrying the sub-millisecond remainder, so sub-millisecond readings still advance it.
#[derive(Default)]
pub struct MsClock {
    last_ticks: u64,
    frac: u32,
}

impl MsClock {
    /// Start the clock at `now_ticks`.
    pub const fn new(now_ticks: u64) -> Self {
        Self {
            last_ticks: now_ticks,
            frac: 0,
        }
    }

    /// Advance towards `now_ticks`, returning the whole milliseconds consumed.
    pub fn advance_to<const NUM: u32, const DEN: u32>(&mut self, now_ticks: u64) -> u32 {
        // The cap keeps every step inside 32 bits; a longer gap drains over later readings.
        let max_delta = ((u32::MAX - (DEN - 1)) / NUM) as u64;
        let delta = now_ticks.saturating_sub(self.last_ticks).min(max_delta);
        self.last_ticks += delta;
        let scaled = self.frac + delta as u32 * NUM;
        let ms = scaled / DEN;
        self.frac = scaled % DEN;
        ms
    }
}

/// Run one CBW-to-CSW cycle over the bulk pair, for a bulk packet size of `EP`. Returns without a
/// CSW only when the host tears the connection down, and ends an empty data-IN phase with a ZLP.
#[allow(clippy::too_many_arguments)]
pub async fn run_command<IN: BulkIn, OUT: BulkOut, IO: SectorIo, const EP: usize>(
    ep_in: &mut IN,
    ep_out: &mut OUT,
    io: &mut IO,
    sector: &mut [u8; MSC_SECTOR_SIZE],
    sense: &mut Sense,
    ua_pending: &mut bool,
    cbw: &Cbw,
) {
    // Single-LUN device (GET MAX LUN reports 0): any other LUN targets a unit that does not exist.
    if cbw.lun != 0 {
        *sense = SENSE_LUN_NOT_SUPPORTED;
        if cbw.dir_in && cbw.data_len > 0 && ep_in.write(&[]).await.is_err() {
            return;
        }
        let _ = ep_in
            .write(&build_csw(cbw.tag, cbw.data_len, CSW_FAILED))
            .await;
        return;
    }
    // One declaring a data phase is a BOT case 4/9 violation, and must reach the reject below.
    if *ua_pending && is_test_unit_ready(cbw.cb[0]) && cbw.data_len == 0 {
        *ua_pending = false;
        *sense = SENSE_MEDIUM_CHANGED;
        let _ = ep_in
            .write(&build_csw(cbw.tag, cbw.data_len, CSW_FAILED))
            .await;
        return;
    }
    let action = dispatch(&cbw.cb, io.media_ready(), *sense);
    // Before either endpoint: a direction mismatch would wedge this task awaiting the host.
    if let BotVerdict::Reject {
        status,
        sense: reject_sense,
        stall_dir_in: _,
    } = reconcile_cbw(cbw.dir_in, cbw.data_len, &action)
    {
        *sense = reject_sense;
        if cbw.dir_in && cbw.data_len > 0 && ep_in.write(&[]).await.is_err() {
            return;
        }
        let _ = ep_in.write(&build_csw(cbw.tag, cbw.data_len, status)).await;
        return;
    }
    let mut transferred: u32 = 0;
    let status = match action {
        ScsiAction::Reply { data, len } => {
            // Never send more than the host's declared data-phase length, so the residue stays exact.
            let len = clamp_reply_len(len, cbw.data_len);
            // Otherwise the host's outstanding IN read consumes the CSW as reply bytes.
            if len == 0 && cbw.data_len > 0 && ep_in.write(&[]).await.is_err() {
                return;
            }
            for chunk in data[..len].chunks(EP) {
                if ep_in.write(chunk).await.is_err() {
                    return;
                }
                transferred += chunk.len() as u32;
            }
            // Cleared only once the reply is on the wire, so a torn reply stays latched.
            if should_clear_sense(cbw.cb[0]) {
                *sense = SENSE_NONE;
            }
            CSW_PASSED
        }
        ScsiAction::ReadSectors { lba, count } => {
            for i in 0..count {
                while io.read(lba + i, sector) != SectorPoll::Ready {
                    yield_once().await;
                }
                for chunk in sector.chunks(EP) {
                    if ep_in.write(chunk).await.is_err() {
                        return;
                    }
                    transferred += chunk.len() as u32;
                }
            }
            CSW_PASSED
        }
        ScsiAction::WriteSectors { lba, count } => {
            // The transport now owes the whole payload: the medium may not change.
            io.set_data_phase(true);
            for i in 0..count {
                let mut off = 0;
                while off < MSC_SECTOR_SIZE {
                    // A short OUT packet pushes `off` off the grid, and `off + EP` would overrun.
                    let end = core::cmp::min(off + EP, MSC_SECTOR_SIZE);
                    match ep_out.read(&mut sector[off..end]).await {
                        Ok(n) => {
                            off += n;
                            transferred += n as u32;
                        }
                        Err(()) => {
                            io.set_data_phase(false);
                            return;
                        }
                    }
                }
                while io.write(lba + i, sector) != SectorPoll::Ready {
                    yield_once().await;
                }
            }
            // Closed before the CSW, so a detach armed mid-command lands on this boundary.
            io.set_data_phase(false);
            CSW_PASSED
        }
        ScsiAction::Ok => CSW_PASSED,
        ScsiAction::Fail(s) => {
            *sense = s;
            // An unknown opcode reaches here un-rejected, so it needs the same terminator.
            if cbw.dir_in && cbw.data_len > 0 && ep_in.write(&[]).await.is_err() {
                return;
            }
            CSW_FAILED
        }
    };

    let residue = cbw.data_len.saturating_sub(transferred);
    let _ = ep_in.write(&build_csw(cbw.tag, residue, status)).await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Waker};
    use daplink_core::msc_bot_cases::{BotCase, BOT_CASES};
    use daplink_core::msc_proto::{CSW_LEN, CSW_PHASE_ERROR, CSW_SIGNATURE, SENSE_LBA_RANGE};
    use daplink_core::{SCSI_PRODUCT_ID, SCSI_VENDOR_ID, VFS_TOTAL_SECTORS};
    use std::collections::BTreeMap;
    use std::vec::Vec;

    /// Poll a future to completion; `yield_once` re-arms itself, so a noop waker suffices.
    fn block_on<F: Future>(f: F) -> F::Output {
        let mut cx = Context::from_waker(Waker::noop());
        let mut f = pin!(f);
        loop {
            if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// Captures every byte written to the bulk IN endpoint.
    struct MockIn {
        written: Vec<u8>,
        /// Length of each write in order: a ZLP is only observable as packet framing.
        packets: Vec<usize>,
    }
    impl MockIn {
        fn new() -> Self {
            Self {
                written: Vec::new(),
                packets: Vec::new(),
            }
        }
        /// The 13-byte CSW is always the tail of the written stream.
        fn csw(&self) -> (u32, u32, u8) {
            let n = self.written.len();
            let csw = &self.written[n - CSW_LEN..];
            let sig = u32::from_le_bytes([csw[0], csw[1], csw[2], csw[3]]);
            assert_eq!(sig, CSW_SIGNATURE);
            let tag = u32::from_le_bytes([csw[4], csw[5], csw[6], csw[7]]);
            let residue = u32::from_le_bytes([csw[8], csw[9], csw[10], csw[11]]);
            (tag, residue, csw[12])
        }
        /// The data-phase bytes preceding the CSW.
        fn data(&self) -> &[u8] {
            &self.written[..self.written.len() - CSW_LEN]
        }
    }
    impl BulkIn for MockIn {
        async fn write(&mut self, data: &[u8]) -> Result<(), ()> {
            self.written.extend_from_slice(data);
            self.packets.push(data.len());
            Ok(())
        }
    }

    /// Feeds queued host-to-device packets in order.
    struct MockOut {
        packets: Vec<Vec<u8>>,
        idx: usize,
    }
    impl MockOut {
        fn new(packets: Vec<Vec<u8>>) -> Self {
            Self { packets, idx: 0 }
        }
        fn empty() -> Self {
            Self::new(Vec::new())
        }
    }
    impl BulkOut for MockOut {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
            let p = &self.packets[self.idx];
            self.idx += 1;
            // The real driver returns `BufferOverflow`, which the adapter maps to `Err(())`.
            if p.len() > buf.len() {
                return Err(());
            }
            buf[..p.len()].copy_from_slice(p);
            Ok(p.len())
        }
    }

    /// A trivial always-ready block device over a sparse map of sectors.
    struct RamDisk {
        sectors: BTreeMap<u32, [u8; 512]>,
        ready: bool,
    }
    impl RamDisk {
        fn new() -> Self {
            Self {
                sectors: BTreeMap::new(),
                ready: true,
            }
        }
    }
    impl SectorIo for RamDisk {
        fn media_ready(&self) -> bool {
            self.ready
        }
        fn read(&mut self, lba: u32, buf: &mut [u8; 512]) -> SectorPoll {
            *buf = *self.sectors.get(&lba).unwrap_or(&[0u8; 512]);
            SectorPoll::Ready
        }
        fn write(&mut self, lba: u32, buf: &[u8; 512]) -> SectorPoll {
            self.sectors.insert(lba, *buf);
            SectorPoll::Ready
        }
    }

    fn cb(op: u8, rest: &[u8]) -> [u8; 16] {
        let mut cb = [0u8; 16];
        cb[0] = op;
        cb[1..1 + rest.len()].copy_from_slice(rest);
        cb
    }

    fn cbw(tag: u32, data_len: u32, dir_in: bool, cb: [u8; 16]) -> Cbw {
        Cbw {
            tag,
            data_len,
            dir_in,
            lun: 0,
            cb,
            cb_len: 16,
        }
    }

    /// CDB tail (cb[1..]) for READ(10)/WRITE(10): lba at cb[2..6], count at cb[7..9].
    fn rw10(lba: u32, count: u16) -> [u8; 9] {
        let mut rest = [0u8; 9];
        rest[1..5].copy_from_slice(&lba.to_be_bytes());
        rest[6..8].copy_from_slice(&count.to_be_bytes());
        rest
    }

    fn run(disk: &mut RamDisk, out: &mut MockOut, sense: &mut Sense, c: &Cbw) -> MockIn {
        let mut ua = false;
        run_ua(disk, out, sense, &mut ua, c)
    }

    fn run_ua(
        disk: &mut RamDisk,
        out: &mut MockOut,
        sense: &mut Sense,
        ua_pending: &mut bool,
        c: &Cbw,
    ) -> MockIn {
        let mut ep_in = MockIn::new();
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        block_on(run_command::<_, _, _, 64>(
            &mut ep_in,
            out,
            disk,
            &mut sector,
            sense,
            ua_pending,
            c,
        ));
        ep_in
    }

    #[test]
    fn ms_clock_reports_whole_milliseconds() {
        // 1_000_000: 1000 ticks per ms, so 2.5 ms reports 2 ms and keeps the remainder.
        let mut clock = MsClock::new(0);
        assert_eq!(clock.advance_to::<1, 1000>(2500), 2);
        assert_eq!(clock.frac, 500);
        // The carried remainder joins the next 600 ticks to complete the third millisecond.
        assert_eq!(clock.advance_to::<1, 1000>(3100), 1);
        assert_eq!(clock.frac, 100);

        // 32_768: a millisecond is 32.768 ticks, so 82 ticks is 2.5 ms and reports the same.
        let mut clock = MsClock::new(0);
        assert_eq!(clock.advance_to::<125, 4096>(82), 2);
        assert_eq!(clock.advance_to::<125, 4096>(102), 1);
    }

    #[test]
    fn ms_clock_millisecond_boundary_at_a_32_768_tick_rate() {
        // 32 ticks is 976.6 us and must report nothing; the 33rd crosses 1 ms and must report it.
        let mut clock = MsClock::new(0);
        assert_eq!(clock.advance_to::<125, 4096>(32), 0);
        assert_eq!(clock.advance_to::<125, 4096>(33), 1);
        // Nothing is consumed twice: 32.768 * 2 = 65.536 ticks, so the second ms lands at 66.
        assert_eq!(clock.advance_to::<125, 4096>(65), 0);
        assert_eq!(clock.advance_to::<125, 4096>(66), 1);
    }

    #[test]
    fn ms_clock_counts_real_milliseconds_at_a_32_768_tick_rate() {
        // A truncated 32 ticks per millisecond would report 1_024_000, 2.3% long.
        let mut clock = MsClock::new(0);
        assert_eq!(clock.advance_to::<125, 4096>(32_768_000), 1_000_000);
        assert_eq!(clock.frac, 0);
    }

    #[test]
    fn ms_clock_sub_ms_storm_still_advances() {
        // A clock that reset its baseline on every reading would report zero forever.
        let mut clock = MsClock::new(0);
        let mut total = 0u32;
        for now in (0..=10_000).step_by(400) {
            total += clock.advance_to::<1, 1000>(now);
        }
        assert_eq!(total, 10);

        let mut clock = MsClock::new(0);
        let mut total = 0u32;
        for now in (0..=327_680).step_by(10) {
            total += clock.advance_to::<125, 4096>(now);
        }
        assert_eq!(total, 10_000);
    }

    #[test]
    fn ms_clock_zero_delta_advances_nothing() {
        let mut clock = MsClock {
            last_ticks: 7_000,
            frac: 3,
        };
        assert_eq!(clock.advance_to::<1, 1000>(7_000), 0);
        assert_eq!(clock.last_ticks, 7_000);
        assert_eq!(clock.frac, 3, "a zero delta touches no state");

        // A stale baseline (a reading behind the last one) never underflows either.
        let mut clock = MsClock::new(5_000);
        assert_eq!(clock.advance_to::<1, 1000>(1_000), 0);
        assert_eq!(clock.last_ticks, 5_000);
    }

    #[test]
    fn ms_clock_caps_a_huge_delta_and_carries_the_rest() {
        // A delta past the cap is consumed only to it, so later readings drain the rest.
        let mut clock = MsClock::new(0);
        assert_eq!(clock.advance_to::<125, 4096>(u64::MAX), 1_048_574);
        assert_eq!(clock.last_ticks, 34_359_705);
        assert_eq!(clock.frac, 4_021);
        assert_eq!(clock.advance_to::<125, 4096>(u64::MAX), 1_048_575);
        assert_eq!(clock.last_ticks, 2 * 34_359_705);
        assert_eq!(clock.frac, 3_946);

        let mut clock = MsClock::new(0);
        assert_eq!(clock.advance_to::<1, 1000>(u64::MAX), 4_294_966);
        assert_eq!(clock.last_ticks, 4_294_966_296);
        assert_eq!(clock.frac, 296);
    }

    /// Check after every reading that the milliseconds reported so far are exactly
    /// `floor(ticks * 1000 / hz)`, computed in `u128`.
    fn matches_oracle<const NUM: u32, const DEN: u32>(hz: u64, steps: usize) {
        let mut clock = MsClock::new(0);
        let mut now = 0u64;
        let mut reported = 0u64;
        let mut seed = 0x2545_F491_4F6C_DD1Du64;
        for _ in 0..steps {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            now += (seed >> 33) % 5_000;
            reported += u64::from(clock.advance_to::<NUM, DEN>(now));
            assert_eq!(
                u128::from(reported),
                u128::from(now) * 1000 / u128::from(hz),
                "reported total diverged from the oracle at {now} ticks (hz {hz})"
            );
            assert!(clock.frac < DEN, "the carried remainder must stay reduced");
            assert_eq!(clock.last_ticks, now, "an uncapped delta is fully consumed");
        }
    }

    #[test]
    fn ms_clock_matches_a_u128_oracle_at_both_shipping_tick_rates() {
        matches_oracle::<1, 1000>(1_000_000, 20_000);
        matches_oracle::<125, 4096>(32_768, 20_000);
    }

    #[test]
    fn read10_round_trip() {
        let mut disk = RamDisk::new();
        let mut expect = [0u8; 512];
        for (i, b) in expect.iter_mut().enumerate() {
            *b = (i % 251) as u8;
        }
        disk.sectors.insert(100, expect);
        let c = cbw(0xAA, 512, true, cb(0x28, &rw10(100, 1)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data(), &expect[..]);
        assert_eq!(ep_in.csw(), (0xAA, 0, CSW_PASSED));
    }

    #[test]
    fn write10_round_trip() {
        let mut disk = RamDisk::new();
        let mut payload = [0u8; 512];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = (i % 97) as u8;
        }
        let packets: Vec<Vec<u8>> = payload.chunks(64).map(|c| c.to_vec()).collect();
        let c = cbw(0xBB, 512, false, cb(0x2A, &rw10(50, 1)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::new(packets), &mut sense, &c);
        assert_eq!(disk.sectors.get(&50), Some(&payload));
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.csw(), (0xBB, 0, CSW_PASSED));
    }

    #[test]
    fn write10_short_packet_no_panic() {
        // A non-compliant host splits the 512-byte data phase into misaligned OUT packets.
        let mut disk = RamDisk::new();
        let mut payload = [0u8; 512];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = (i % 131) as u8;
        }
        let mut packets: Vec<Vec<u8>> = Vec::new();
        let mut off = 0usize;
        for &size in &[40usize, 64, 64, 64, 64, 64, 64, 64, 24] {
            packets.push(payload[off..off + size].to_vec());
            off += size;
        }
        assert_eq!(off, 512);

        let c = cbw(0xCC, 512, false, cb(0x2A, &rw10(70, 1)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::new(packets), &mut sense, &c);
        assert_eq!(disk.sectors.get(&70), Some(&payload));
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.csw(), (0xCC, 0, CSW_PASSED));
    }

    #[test]
    fn write10_packet_overrunning_the_clamped_tail_aborts() {
        // Off the grid the final window is under a packet, so a full one must abandon, not commit.
        let mut disk = RamDisk::new();
        let mut payload = [0u8; 512];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = (i % 131) as u8;
        }
        let mut packets: Vec<Vec<u8>> = Vec::new();
        let mut off = 0usize;
        for &size in &[40usize, 64, 64, 64, 64, 64, 64, 64] {
            packets.push(payload[off..off + size].to_vec());
            off += size;
        }
        assert_eq!(off, 488, "the tail window must be shorter than a packet");
        packets.push([0xA5u8; 64].to_vec());

        let c = cbw(0xCD, 512, false, cb(0x2A, &rw10(71, 1)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::new(packets), &mut sense, &c);
        assert!(
            !disk.sectors.contains_key(&71),
            "a torn data phase must not commit a sector"
        );
        assert!(
            ep_in.written.is_empty(),
            "an abandoned command sends no data and no CSW"
        );
    }

    #[test]
    fn nonzero_lun_is_rejected() {
        let mut disk = RamDisk::new();
        let mut c = cbw(0x51, 0, false, cb(0x00, &[])); // TEST UNIT READY
        c.lun = 1;
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0, "no data phase for a rejected LUN");
        assert_eq!(ep_in.csw(), (0x51, 0, CSW_FAILED));
        // SPC: the unit is missing, not the operation code, so 5/25/00 and not 5/20/00.
        assert_eq!((sense.key, sense.asc, sense.ascq), (0x05, 0x25, 0x00));
    }

    #[test]
    fn reply_clamped_to_cbw_data_len() {
        // A CDB allocation length past the CBW's data length truncates, with a zero residue.
        let mut disk = RamDisk::new();
        let c = cbw(0x30, 16, true, cb(0x12, &[0x00, 0x00, 0x00, 36]));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 16);
        assert_eq!(ep_in.csw(), (0x30, 0, CSW_PASSED));
    }

    #[test]
    fn zero_allocation_length_reply_ends_with_a_zero_length_packet() {
        // Case 4 (Hi > Dn) through a legal command: an INQUIRY with a zero allocation length.
        let mut disk = RamDisk::new();
        let mut sense = SENSE_NONE;
        let c = cbw(0xC1, 36, true, cb(0x12, &[]));
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(
            ep_in.packets.as_slice(),
            &[0, CSW_LEN],
            "a zero-length packet precedes the CSW"
        );
        assert_eq!(ep_in.csw(), (0xC1, 36, CSW_PASSED));

        // A nonzero allocation length is untouched: its packet ends the host's read by itself.
        let c = cbw(0xC2, 36, true, cb(0x12, &[0x00, 0x00, 0x00, 36]));
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.packets.as_slice(), &[36, CSW_LEN]);
        assert_eq!(ep_in.csw(), (0xC2, 0, CSW_PASSED));
    }

    #[test]
    fn nonzero_lun_with_data_in_phase_ends_with_a_zero_length_packet() {
        let mut disk = RamDisk::new();
        let mut c = cbw(0x52, 36, true, cb(0x12, &[0x00, 0x00, 0x00, 36]));
        c.lun = 1;
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.packets.as_slice(), &[0, CSW_LEN]);
        assert_eq!(ep_in.csw(), (0x52, 36, CSW_FAILED));
    }

    #[test]
    fn read10_length_mismatch_reject_ends_with_a_zero_length_packet() {
        let mut disk = RamDisk::new();
        let c = cbw(0x53, 512, true, cb(0x28, &rw10(60, 2)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.packets.as_slice(), &[0, CSW_LEN]);
        assert_eq!(ep_in.csw(), (0x53, 512, CSW_FAILED));
    }

    #[test]
    fn unit_attention_phase_violation_ends_with_a_zero_length_packet() {
        // The pending attention must survive a TEST UNIT READY that declares a data phase.
        let mut disk = RamDisk::new();
        let mut sense = SENSE_NONE;
        let mut ua = true;
        let c = cbw(0x54, 36, true, cb(0x00, &[]));
        let ep_in = run_ua(&mut disk, &mut MockOut::empty(), &mut sense, &mut ua, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.packets.as_slice(), &[0, CSW_LEN]);
        assert_eq!(ep_in.csw(), (0x54, 36, CSW_FAILED));
        assert!(
            ua,
            "the pending attention is not consumed by a rejected CBW"
        );
    }

    #[test]
    fn inquiry_identity() {
        let mut disk = RamDisk::new();
        let c = cbw(0x01, 36, true, cb(0x12, &[0x00, 0x00, 0x00, 36]));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        let data = ep_in.data();
        assert_eq!(data.len(), 36);
        assert_eq!(&data[8..16], SCSI_VENDOR_ID);
        assert_eq!(&data[16..32], SCSI_PRODUCT_ID);
        assert_eq!(ep_in.csw(), (0x01, 0, CSW_PASSED));
    }

    #[test]
    fn read_capacity() {
        let mut disk = RamDisk::new();
        let c = cbw(0x02, 8, true, cb(0x25, &[]));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        let data = ep_in.data();
        assert_eq!(data.len(), 8);
        assert_eq!(&data[0..4], &(VFS_TOTAL_SECTORS - 1).to_be_bytes());
        assert_eq!(&data[4..8], &512u32.to_be_bytes());
        assert_eq!(ep_in.csw(), (0x02, 0, CSW_PASSED));
    }

    #[test]
    fn test_unit_ready_ready_and_not() {
        let mut disk = RamDisk::new();
        let c = cbw(0x03, 0, false, cb(0x00, &[]));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.csw(), (0x03, 0, CSW_PASSED));

        disk.ready = false;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.csw(), (0x03, 0, CSW_FAILED));
    }

    #[test]
    fn sense_latch_and_request_sense_readback_then_clear() {
        let mut disk = RamDisk::new();
        let mut sense = SENSE_NONE;

        // An out-of-range READ(10) fails and latches the LBA-range sense.
        let bad = cbw(0x10, 512, true, cb(0x28, &rw10(VFS_TOTAL_SECTORS, 1)));
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &bad);
        assert_eq!(ep_in.csw(), (0x10, 512, CSW_FAILED));
        assert_eq!(sense, SENSE_LBA_RANGE);

        // REQUEST SENSE reports the latched key/asc/ascq...
        let rs = cbw(0x11, 18, true, cb(0x03, &[0x00, 0x00, 0x00, 18]));
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &rs);
        let data = ep_in.data();
        assert_eq!(data[2], SENSE_LBA_RANGE.key);
        assert_eq!(data[12], SENSE_LBA_RANGE.asc);
        assert_eq!(data[13], SENSE_LBA_RANGE.ascq);
        assert_eq!(ep_in.csw(), (0x11, 0, CSW_PASSED));
        // ...and clears it.
        assert_eq!(sense, SENSE_NONE);

        // A second REQUEST SENSE now reports "no sense".
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &rs);
        assert_eq!(ep_in.data()[2], SENSE_NONE.key);
    }

    #[test]
    fn unknown_opcode_fails_with_full_residue() {
        let mut disk = RamDisk::new();
        let c = cbw(0x20, 36, true, cb(0xFD, &[]));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.csw(), (0x20, 36, CSW_FAILED));
    }

    #[test]
    fn unknown_opcode_with_data_in_phase_ends_with_a_zero_length_packet() {
        let mut disk = RamDisk::new();
        let c = cbw(0x23, 36, true, cb(0xFD, &[]));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.packets.as_slice(), &[0, CSW_LEN]);
        assert_eq!(ep_in.csw(), (0x23, 36, CSW_FAILED));
    }

    #[test]
    fn media_changed_first_test_unit_ready_raises_unit_attention() {
        let mut disk = RamDisk::new();
        let mut sense = SENSE_NONE;
        let mut ua = true; // a remount just latched a media-changed edge

        let tur = cbw(0x40, 0, false, cb(0x00, &[]));
        let ep_in = run_ua(&mut disk, &mut MockOut::empty(), &mut sense, &mut ua, &tur);
        assert_eq!(ep_in.csw(), (0x40, 0, CSW_FAILED));
        assert_eq!(sense, SENSE_MEDIUM_CHANGED);
        assert!(!ua, "the pending unit attention is consumed");

        let rs = cbw(0x41, 18, true, cb(0x03, &[0x00, 0x00, 0x00, 18]));
        let ep_in = run_ua(&mut disk, &mut MockOut::empty(), &mut sense, &mut ua, &rs);
        let data = ep_in.data();
        assert_eq!(data[2], SENSE_MEDIUM_CHANGED.key);
        assert_eq!(data[12], SENSE_MEDIUM_CHANGED.asc);
        assert_eq!(data[13], SENSE_MEDIUM_CHANGED.ascq);
        assert_eq!(sense, SENSE_NONE);

        let ep_in = run_ua(&mut disk, &mut MockOut::empty(), &mut sense, &mut ua, &tur);
        assert_eq!(ep_in.csw(), (0x40, 0, CSW_PASSED));
    }

    #[test]
    fn phase_malformed_tur_keeps_unit_attention() {
        // Spending it on a command failing for another reason would hide the republish.
        let mut disk = RamDisk::new();
        let mut sense = SENSE_NONE;
        let mut ua = true;

        // Case 4 (Hi > Dn): a device-to-host data phase declared.
        let c = cbw(0x70, 36, true, cb(0x00, &[]));
        let ep_in = run_ua(&mut disk, &mut MockOut::empty(), &mut sense, &mut ua, &c);
        assert_eq!(ep_in.data().len(), 0, "no data phase for the reject");
        assert_eq!(ep_in.csw(), (0x70, 36, CSW_FAILED));
        assert!(ua, "the pending attention is not consumed");

        // Case 9 (Ho > Dn): the empty MockOut proves no host packet is consumed first.
        let c = cbw(0x71, 512, false, cb(0x00, &[]));
        let ep_in = run_ua(&mut disk, &mut MockOut::empty(), &mut sense, &mut ua, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.csw(), (0x71, 512, CSW_FAILED));
        assert!(ua, "the pending attention is still not consumed");

        let tur = cbw(0x72, 0, false, cb(0x00, &[]));
        let ep_in = run_ua(&mut disk, &mut MockOut::empty(), &mut sense, &mut ua, &tur);
        assert_eq!(ep_in.csw(), (0x72, 0, CSW_FAILED));
        assert_eq!(sense, SENSE_MEDIUM_CHANGED);
        assert!(!ua, "a phase-clean TUR does consume it");
    }

    #[test]
    fn write10_data_len_mismatch_rejected() {
        // With no OUT packets supplied, a wedging implementation would block or panic.
        let mut disk = RamDisk::new();
        let c = cbw(0xDD, 512, false, cb(0x2A, &rw10(60, 2)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert!(
            !disk.sectors.contains_key(&60),
            "no sector is written on a rejected mismatch"
        );
        assert_eq!(ep_in.data().len(), 0, "no data phase on rejection");
        assert_eq!(ep_in.csw(), (0xDD, 512, CSW_FAILED));
    }

    #[test]
    fn read10_data_len_mismatch_rejected() {
        let mut disk = RamDisk::new();
        disk.sectors.insert(60, [7u8; 512]);
        let c = cbw(0xEE, 512, true, cb(0x28, &rw10(60, 2)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0, "no data phase on rejection");
        assert_eq!(ep_in.csw(), (0xEE, 512, CSW_FAILED));
    }

    #[test]
    fn read10_wrong_direction_is_phase_error() {
        // Case 10 (Ho <> Di): completing against an empty MockOut is the no-wedge assertion.
        let mut disk = RamDisk::new();
        disk.sectors.insert(5, [0x11u8; 512]);
        let c = cbw(0x61, 512, false, cb(0x28, &rw10(5, 1)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0, "no data written before the CSW");
        assert_eq!(ep_in.csw(), (0x61, 512, CSW_PHASE_ERROR));
    }

    #[test]
    fn write10_wrong_direction_is_phase_error() {
        // Case 8 (Hi <> Do).
        let mut disk = RamDisk::new();
        let c = cbw(0x62, 512, true, cb(0x2A, &rw10(5, 1)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert!(!disk.sectors.contains_key(&5), "no sector is written");
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.csw(), (0x62, 512, CSW_PHASE_ERROR));
    }

    /// The IN packet lengths a case must produce: a ZLP where a stalling transport would stall
    /// IN, then the data-phase chunks, then the CSW.
    fn want_packets(case: &BotCase) -> Vec<usize> {
        let mut want = Vec::new();
        if case.ends_with_zlp() {
            want.push(0);
        }
        let mut left = case.data_len;
        while left > 0 {
            let n = core::cmp::min(left, 64);
            want.push(n);
            left -= n;
        }
        want.push(CSW_LEN);
        want
    }

    #[test]
    fn bot_case_table_is_answered_as_the_shared_expectations_say() {
        // This transport cannot stall, so a stall-IN row ends the phase with a ZLP instead.
        for case in BOT_CASES {
            let name = case.name;
            let mut disk = RamDisk::new();
            let mut sense = SENSE_NONE;
            let mut ua = case.ua_before;
            let parsed = daplink_core::msc_proto::parse_cbw(&case.cbw).expect(name);
            let packets: Vec<Vec<u8>> = (0..case.out_packets).map(|_| [0u8; 64].to_vec()).collect();
            let mut out = MockOut::new(packets);
            let ep_in = run_ua(&mut disk, &mut out, &mut sense, &mut ua, &parsed);

            assert_eq!(ep_in.data().len(), case.data_len, "{name}: data bytes");
            assert_eq!(
                ep_in.csw(),
                (case.tag(), case.residue, case.status),
                "{name}: CSW"
            );
            assert_eq!(sense, case.sense, "{name}: latched sense");
            assert_eq!(ua, case.ua_after, "{name}: pending unit attention");
            assert_eq!(
                ep_in.packets,
                want_packets(case),
                "{name}: IN packet framing"
            );
            assert_eq!(out.idx, case.out_packets, "{name}: host packets consumed");
        }
    }

    #[test]
    fn tur_with_data_len_in_fails_with_full_residue() {
        let mut disk = RamDisk::new();
        let c = cbw(0x63, 36, true, cb(0x00, &[]));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0, "no data phase for the reject");
        assert_eq!(ep_in.csw(), (0x63, 36, CSW_FAILED));
    }

    #[test]
    fn tur_with_data_len_out_fails_with_full_residue() {
        let mut disk = RamDisk::new();
        let c = cbw(0x64, 512, false, cb(0x00, &[]));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.csw(), (0x64, 512, CSW_FAILED));
    }

    #[test]
    fn out_of_range_read_fails_with_full_residue() {
        let mut disk = RamDisk::new();
        let c = cbw(0x21, 1024, true, cb(0x28, &rw10(VFS_TOTAL_SECTORS - 1, 4)));
        let mut sense = SENSE_NONE;
        let ep_in = run(&mut disk, &mut MockOut::empty(), &mut sense, &c);
        assert_eq!(ep_in.data().len(), 0);
        assert_eq!(ep_in.csw(), (0x21, 1024, CSW_FAILED));
    }

    /// A block device that parks a fixed number of times, recording every `set_data_phase`.
    struct PendingDisk {
        sectors: BTreeMap<u32, [u8; 512]>,
        read_pending: u32,
        write_pending: u32,
        phase_events: Vec<bool>,
    }
    impl PendingDisk {
        fn new(read_pending: u32, write_pending: u32) -> Self {
            Self {
                sectors: BTreeMap::new(),
                read_pending,
                write_pending,
                phase_events: Vec::new(),
            }
        }
    }
    impl SectorIo for PendingDisk {
        fn media_ready(&self) -> bool {
            true
        }
        fn read(&mut self, lba: u32, buf: &mut [u8; 512]) -> SectorPoll {
            if self.read_pending > 0 {
                self.read_pending -= 1;
                return SectorPoll::Pending;
            }
            *buf = *self.sectors.get(&lba).unwrap_or(&[0u8; 512]);
            SectorPoll::Ready
        }
        fn write(&mut self, lba: u32, buf: &[u8; 512]) -> SectorPoll {
            if self.write_pending > 0 {
                self.write_pending -= 1;
                return SectorPoll::Pending;
            }
            self.sectors.insert(lba, *buf);
            SectorPoll::Ready
        }
        fn set_data_phase(&mut self, open: bool) {
            self.phase_events.push(open);
        }
    }

    // No outstanding-read marker across polls: `run_command`'s `yield_once` loop retries inline.

    #[test]
    fn read10_resumes_after_a_pending_sector_poll() {
        let mut disk = PendingDisk::new(3, 0);
        let mut expect = [0u8; 512];
        for (i, b) in expect.iter_mut().enumerate() {
            *b = (i % 199) as u8;
        }
        disk.sectors.insert(9, expect);
        let c = cbw(0x80, 512, true, cb(0x28, &rw10(9, 1)));
        let mut sense = SENSE_NONE;
        let mut ea_in = MockOut::empty();
        let mut ep_in = MockIn::new();
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        let mut ua = false;
        block_on(run_command::<_, _, _, 64>(
            &mut ep_in,
            &mut ea_in,
            &mut disk,
            &mut sector,
            &mut sense,
            &mut ua,
            &c,
        ));
        assert_eq!(ep_in.data(), &expect[..]);
        assert_eq!(ep_in.csw(), (0x80, 0, CSW_PASSED));
    }

    #[test]
    fn write10_resumes_after_a_pending_sector_poll() {
        let mut disk = PendingDisk::new(0, 2);
        let mut payload = [0u8; 512];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = (i % 173) as u8;
        }
        let packets: Vec<Vec<u8>> = payload.chunks(64).map(|c| c.to_vec()).collect();
        let c = cbw(0x81, 512, false, cb(0x2A, &rw10(11, 1)));
        let mut sense = SENSE_NONE;
        let mut ep_out = MockOut::new(packets);
        let mut ep_in = MockIn::new();
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        let mut ua = false;
        block_on(run_command::<_, _, _, 64>(
            &mut ep_in,
            &mut ep_out,
            &mut disk,
            &mut sector,
            &mut sense,
            &mut ua,
            &c,
        ));
        assert_eq!(disk.sectors.get(&11), Some(&payload));
        assert_eq!(ep_in.csw(), (0x81, 0, CSW_PASSED));
        assert_eq!(
            disk.phase_events,
            [true, false],
            "a parked sector poll must not repeat the bracket"
        );
    }

    #[test]
    fn write_sectors_brackets_the_data_phase() {
        let mut disk = PendingDisk::new(0, 0);
        let payload = [0u8; 512];
        let packets: Vec<Vec<u8>> = payload.chunks(64).map(|c| c.to_vec()).collect();
        let c = cbw(0x82, 512, false, cb(0x2A, &rw10(20, 1)));
        let mut sense = SENSE_NONE;
        let mut ep_out = MockOut::new(packets);
        let mut ep_in = MockIn::new();
        let mut sector = [0u8; MSC_SECTOR_SIZE];
        let mut ua = false;
        block_on(run_command::<_, _, _, 64>(
            &mut ep_in,
            &mut ep_out,
            &mut disk,
            &mut sector,
            &mut sense,
            &mut ua,
            &c,
        ));
        assert_eq!(ep_in.csw(), (0x82, 0, CSW_PASSED));
        assert_eq!(
            disk.phase_events,
            [true, false],
            "the phase opens at WriteSectors dispatch and closes before the CSW"
        );
    }
}
