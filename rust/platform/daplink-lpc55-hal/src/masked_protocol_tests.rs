use super::*;
extern crate std;
use std::cell::RefCell;
use std::vec;
use std::vec::Vec;

use dap_differential::corpus::transfer_block as corpus;

std::thread_local! {
    static SCRIPT: RefCell<(Vec<corpus::Step>, usize, usize)> = const { RefCell::new((Vec::new(), 0, 0)) };
    /// A simulated target the words run on in place of the script, when one is installed.
    pub(super) static LIVE: RefCell<Option<dap_differential::SimWords>> = const { RefCell::new(None) };
}

pub(super) fn mask_taken() {
    SCRIPT.with(|s| s.borrow_mut().2 += 1);
}

fn next(request: u8, write: Option<u32>) -> swd::Result<u32> {
    SCRIPT.with(|s| {
        let mut s = s.borrow_mut();
        let step = s.0[s.1];
        s.1 += 1;
        assert_eq!((request, write), (step.request, step.write));
        match step.reply {
            corpus::Reply::Word(word) => Ok(word),
            corpus::Reply::Wait => Err(swd::Error::AckWait),
            corpus::Reply::Fault => Err(swd::Error::AckFault),
            corpus::Reply::NoAck => Err(swd::Error::NoAck),
            corpus::Reply::Parity => Err(swd::Error::BadParity),
        }
    })
}

pub(super) fn read(request: u8) -> swd::Result<u32> {
    let live = LIVE.with(|l| l.borrow_mut().as_mut().map(|w| w.read(request)));
    live.unwrap_or_else(|| next(request, None))
}
pub(super) fn write(request: u8, word: u32) -> swd::Result<()> {
    let live = LIVE.with(|l| l.borrow_mut().as_mut().map(|w| w.write(request, word)));
    live.unwrap_or_else(|| next(request, Some(word)).map(|_| ()))
}

#[derive(Clone, Copy)]
pub(super) struct P;
impl JtagPacing for P {
    const FAST_HALF_CYCLES: u32 = 0;
    fn pace_low(&self) {}
    fn pace_high(&self) {}
}
impl MaskedPacing for P {
    const SWD_FAST_HALF_CYCLES: u32 = 0;
    const JTAG_FAST_MIN_HZ: u32 = 0;
    const JTAG_SEQUENCE_FLOOR_CYCLES: u32 = 1;
    const JTAG_LOW_FLOOR_PS: u32 = 0;
    const JTAG_HIGH_FLOOR_PS: u32 = 0;
    const JTAG_PERIOD_FLOOR_PS: u32 = 0;
    const JTAG_CONTROL_FIXED_CYCLES: (u32, u32) = (0, 0);
    const JTAG_ACK_FIXED_CYCLES: (u32, u32) = (0, 0);
    const JTAG_READ_FIXED_CYCLES: (u32, u32) = (0, 0);
    const JTAG_WRITE_FIXED_CYCLES: (u32, u32) = (0, 0);
    const JTAG_IR_FIXED_CYCLES: (u32, u32) = (0, 0);
    fn swd_control_low(&self) {}
    fn swd_control_high(&self) {}
    fn swd_read_low(&self) {}
    fn swd_read_high(&self) {}
    fn swd_write_low(&self) {}
    fn swd_write_high(&self) {}
}

pub(super) fn serial() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    let guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    SWD_ACTIVE.store(false, Ordering::Relaxed);
    SCRIPT.with(|s| *s.borrow_mut() = (Vec::new(), 0, 0));
    guard
}

#[test]
fn block_protocol_corpus_runs_the_local_loops() {
    let _serial = serial();
    for half in [0, 30] {
        for plan in &SWD_PLAN {
            plan.store(half, Ordering::Relaxed);
        }
        corpus::check_blocks(|report, retries, steps| {
            SCRIPT.with(|s| *s.borrow_mut() = (steps.to_vec(), 0, 0));
            let mut block = MaskedBlock::new(P);
            let mut response =
                vec![0xa5; 4 + 4 * u16::from_le_bytes([report[2], report[3]]) as usize];
            // No idle cycles, which would leave the block to dap-rs.
            assert_eq!(
                block.process(
                    &[4, 0, retries as u8, (retries >> 8) as u8, 7, 0],
                    &mut response
                ),
                None
            );
            SWD_ACTIVE.store(true, Ordering::Relaxed);
            let len = block.process(report, &mut response).unwrap();
            SCRIPT.with(|s| assert_eq!(s.borrow().1, steps.len()));
            assert!(response[len..].iter().all(|b| *b == 0xa5));
            response.truncate(len);
            response
        });
    }
}

#[test]
fn single_posted_read_retries_and_retires_only_rdbuff() {
    let _serial = serial();
    for half in [0, 30] {
        for plan in &SWD_PLAN {
            plan.store(half, Ordering::Relaxed);
        }
        for (reply, status) in [
            (corpus::Reply::Word(0x11223344), 1),
            (corpus::Reply::Wait, 2),
            (corpus::Reply::Fault, 4),
            (corpus::Reply::NoAck, 7),
            (corpus::Reply::Parity, 15),
        ] {
            let steps = vec![
                corpus::Step {
                    request: 0x9f,
                    write: None,
                    reply: corpus::Reply::Word(0xdeadbeef),
                },
                corpus::Step {
                    request: 0xbd,
                    write: None,
                    reply: corpus::Reply::Wait,
                },
                corpus::Step {
                    request: 0xbd,
                    write: None,
                    reply,
                },
            ];
            SCRIPT.with(|s| *s.borrow_mut() = (steps, 0, 0));
            SWD_ACTIVE.store(true, Ordering::Relaxed);
            let mut block = MaskedBlock::new(P);
            block.wait_attempts = 2;
            let mut response = [0xa5; 8];
            let len = block.process(&[6, 0, 1, 0, 0x0f], &mut response);
            assert_eq!(&response[..4], &[6, u8::from(status == 1), 0, status]);
            if status == 1 {
                assert_eq!(len, Some(8));
                assert_eq!(&response[4..], &0x11223344u32.to_le_bytes());
            } else {
                assert_eq!(len, Some(4));
                assert_eq!(&response[4..], &[0xa5; 4]);
            }
            SCRIPT.with(|s| assert_eq!(s.borrow().1, 3));
        }
    }
}

#[test]
fn packet_observation_does_not_own_the_lpc55_port() {
    let _serial = serial();
    let mut block = MaskedBlock::new(P);
    let mut response = [0xa5; 8];
    for active in [false, true] {
        SWD_ACTIVE.store(active, Ordering::Relaxed);
        for report in [
            &[2, 0][..],
            &[2, 1],
            &[2, 2],
            &[2, 255],
            &[2],
            &[3],
            &[5, 0, 1, 6],
        ] {
            assert_eq!(block.process(report, &mut response), None);
            assert_eq!(SWD_ACTIVE.load(Ordering::Relaxed), active);
            assert_eq!(response, [0xa5; 8]);
        }
    }
}

#[test]
fn malformed_blocks_fall_back_without_response_or_word_writes() {
    let _serial = serial();
    SWD_ACTIVE.store(true, Ordering::Relaxed);
    let mut block = MaskedBlock::new(P);
    for report in [
        &[][..],
        &[6],
        &[6, 0, 1],
        &[6, 0, 1, 0],
        &[6, 0, 0, 0, 2],
        &[6, 0, 1, 0, 0],
        &[6, 0, 1, 0, 0, 1, 2, 3],
    ] {
        let mut response = [0xa5; 8];
        assert_eq!(block.process(report, &mut response), None);
        assert_eq!(response, [0xa5; 8]);
    }
    for capacity in 0..8 {
        let mut response = [0xa5; 8];
        assert_eq!(
            block.process(&[6, 0, 1, 0, 2], &mut response[..capacity]),
            None
        );
        assert_eq!(response, [0xa5; 8]);
    }
    SCRIPT.with(|s| assert_eq!(s.borrow().1, 0));
}

#[test]
fn retry_configuration_is_little_endian_and_requires_the_full_report() {
    let _serial = serial();
    let mut block = MaskedBlock::new(P);
    assert_eq!(block.wait_attempts, 6);
    let mut response = [0xa5; 8];
    for retry in [0, 0x1234, 65535] {
        let report = [4, 9, retry as u8, (retry >> 8) as u8, 0xff, 0xff];
        assert_eq!(block.process(&report, &mut response), None);
        assert_eq!((block.wait_attempts, block.idle_cycles), (retry + 1, 9));
        for len in 0..6 {
            block.process(&[4, 0, 1, 0, 0, 0][..len], &mut response);
            assert_eq!((block.wait_attempts, block.idle_cycles), (retry + 1, 9));
        }
    }
    assert_eq!(response, [0xa5; 8]);
}

#[test]
fn idle_cycles_leave_blocks_to_dap_rs() {
    let _serial = serial();
    SWD_ACTIVE.store(true, Ordering::Relaxed);
    let mut block = MaskedBlock::new(P);
    let mut response = [0xa5; 8];
    assert_eq!(block.process(&[4, 2, 0, 0, 0, 0], &mut response), None);
    assert_eq!(block.process(&[6, 0, 1, 0, 2], &mut response), None);
    assert_eq!(response, [0xa5; 8]);
    SCRIPT.with(|s| assert_eq!(s.borrow().2, 0));
    let steps = vec![corpus::Step {
        request: 0xa5,
        write: None,
        reply: corpus::Reply::Word(7),
    }];
    SCRIPT.with(|s| *s.borrow_mut() = (steps, 0, 0));
    assert_eq!(block.process(&[4, 0, 0, 0, 0, 0], &mut response), None);
    assert_eq!(block.process(&[6, 0, 1, 0, 2], &mut response), Some(8));
    assert_eq!(response, [6, 1, 0, 1, 7, 0, 0, 0]);
}

#[test]
fn malformed_payloads_keep_the_mask_acquisition_boundary() {
    let _serial = serial();
    SWD_ACTIVE.store(true, Ordering::Relaxed);
    let mut block = MaskedBlock::new(P);
    for (report, capacity, takes) in [
        (&[6, 0, 1, 0][..], 8, 0),
        (&[6, 0, 0, 0, 2][..], 8, 0),
        (&[6, 0, 1, 0, 2][..], 3, 0),
        (&[6, 0, 1, 0, 2][..], 7, 1),
        (&[6, 0, 1, 0, 0, 1, 2, 3][..], 8, 1),
    ] {
        SCRIPT.with(|s| *s.borrow_mut() = (Vec::new(), 0, 0));
        let mut response = [0xa5; 8];
        assert_eq!(block.process(report, &mut response[..capacity]), None);
        assert_eq!(response, [0xa5; 8]);
        SCRIPT.with(|s| assert_eq!(s.borrow().2, takes));
    }
}
