//! The block against the literal corpus and malformed reports, over a recorded wire.

extern crate std;

use std::vec;
use std::vec::Vec;

use dap_differential::corpus::transfer_block as corpus;
use daplink_rs::dap_rs::driver::bitbang::SwdAdapter;

use super::tests::*;
use super::*;

fn corpus_script(steps: &[corpus::Step]) -> Vec<bool> {
    let mut script = Vec::new();
    for step in steps {
        script.extend(match step.reply {
            corpus::Reply::Word(word) if step.write.is_none() => {
                read_script(1, word, word.count_ones() % 2 != 0)
            }
            corpus::Reply::Word(_) => ack_script(1),
            corpus::Reply::Wait => ack_script(2),
            corpus::Reply::Fault => ack_script(4),
            corpus::Reply::NoAck => ack_script(7),
            corpus::Reply::Parity => read_script(1, 0, true),
        });
    }
    script
}

/// dap-rs's answer with one WAIT retry more than `retries`, its attempts CMSIS-DAP's count.
fn pinned_block(report: &[u8], retries: u16, script: &[bool]) -> Vec<u8> {
    let retries = retries + 1;
    use daplink_rs::dap_rs::dap::Dap;
    use daplink_rs::dap_rs::driver::bitbang::JtagAdapter;
    let trace = Trace::new(script);
    let mut dap: Dap<
        '_,
        _,
        _,
        _,
        JtagAdapter<TracePin, MockDelay>,
        SwdAdapter<TracePin, MockDelay>,
        _,
    > = Dap::new(
        adapter(&trace),
        daplink_rs::NoopLeds,
        MockDelay,
        daplink_rs::NoSwo,
        "test",
        512,
    );
    let mut response = vec![0xa5; 4 + 4 * u16::from_le_bytes([report[2], report[3]]) as usize];
    assert_eq!(dap.process_command(&[2, 1], &mut response), 2);
    assert_eq!(&response[..2], &[2, 1]);
    dap.process_command(
        &[4, 9, retries as u8, (retries >> 8) as u8, 7, 0],
        &mut response,
    );
    response.fill(0xa5);
    let len = dap.process_command(report, &mut response);
    assert_eq!(trace.borrow().next, script.len());
    assert!(response[len..].iter().all(|b| *b == 0xa5));
    response.truncate(len);
    response
}

#[test]
fn literal_block_corpus_matches_the_pinned_command_handler() {
    corpus::check_blocks(|report, retries, steps| {
        let script = corpus_script(steps);
        let mut expected = (retries < u16::MAX).then(|| pinned_block(report, retries, &script));
        // dap-rs reports a no-ACK write as 0x0F; the block, as the bit-bang engine, as 0x07.
        let no_ack_write =
            |s: &corpus::Step| s.write.is_some() && matches!(s.reply, corpus::Reply::NoAck);
        if let Some(expected) = expected.as_mut().filter(|_| steps.iter().any(no_ack_write)) {
            if expected[3] == 0x0F {
                expected[3] = 0x07;
            }
        }
        // dap-rs counts the transfer that failed; the block counts what completed, as Arm does.
        if let Some(expected) = expected.as_mut().filter(|e| e[3] != 1) {
            let done = if report[4] & 2 != 0 {
                (expected.len() - 4) / 4
            } else {
                usize::from(u16::from_le_bytes([expected[1], expected[2]])) - 1
            };
            expected[1..3].copy_from_slice(&(done as u16).to_le_bytes());
        }
        let trace = Trace::new(&script);
        let mut block: H5Block<P> = H5Block::new(250_000_000);
        let mut response = vec![0xa5; 4 + 4 * u16::from_le_bytes([report[2], report[3]]) as usize];
        block.observe(&[4, 9, retries as u8, (retries >> 8) as u8, 7, 0], &[4, 0]);
        let len = block
            .transfer_block(report, &mut response, || MockWire(trace.clone()))
            .unwrap();
        let t = trace.borrow();
        assert_eq!(t.next, script.len());
        let mut pos = 0;
        for step in steps {
            for bit in 0..8 {
                assert_eq!(
                    t.cells[pos + bit].driven,
                    Some(step.request & (1 << bit) != 0)
                );
            }
            if let (Some(word), corpus::Reply::Word(_)) = (step.write, step.reply) {
                for bit in 0..32 {
                    assert_eq!(t.cells[pos + 13 + bit].driven, Some(word & (1 << bit) != 0));
                }
            }
            pos += match step.reply {
                corpus::Reply::Word(_) | corpus::Reply::Parity => {
                    // The configured nine idle cycles after an OK ACK, a bad parity included.
                    let idle = &t.cells[pos + 46..pos + 55];
                    assert!(idle.iter().all(|c| c.driven == Some(false)));
                    55
                }
                _ => 13,
            };
        }
        assert_eq!(t.cells.len(), pos);
        assert!(response[len..].iter().all(|b| *b == 0xa5));
        response.truncate(len);
        if let Some(expected) = expected {
            assert_eq!(response, expected);
        }
        response
    });
}

#[test]
fn malformed_blocks_never_resolve_the_h5_wire() {
    let block: H5Block<P> = H5Block::new(250_000_000);
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
        assert_eq!(
            block.transfer_block::<MockWire>(report, &mut response, || panic!(
                "fallback resolved GPIO"
            )),
            None
        );
        assert_eq!(response, [0xa5; 8]);
    }
    for capacity in 0..8 {
        let mut response = [0xa5; 8];
        assert_eq!(
            block.transfer_block::<MockWire>(
                &[6, 0, 1, 0, 2],
                &mut response[..capacity],
                || panic!("short response resolved GPIO")
            ),
            None
        );
        assert_eq!(response, [0xa5; 8]);
    }
}

#[test]
fn h5_port_observation_follows_dap_rs_connect_answers() {
    let mut block: H5Block<P> = H5Block::new(250_000_000);
    for (report, response, held) in [
        (&[2, 0][..], &[2, 1][..], true),
        (&[2, 2][..], &[2, 2][..], false),
        (&[2, 1][..], &[2, 1][..], true),
        (&[2, 255][..], &[2, 0][..], true),
        (&[2][..], &[0xff][..], true),
        (&[3][..], &[3, 0][..], false),
    ] {
        answered(&mut block, report, response);
        assert_eq!(block.dap.swd_held(), held);
    }
}

#[test]
fn all_truncated_retry_configurations_leave_the_h5_budget_unchanged() {
    let mut block: H5Block<P> = H5Block::new(250_000_000);
    for retry in [0, 0x1234, 65535] {
        answered(
            &mut block,
            &[4, 9, retry as u8, (retry >> 8) as u8, 0xff, 0xff],
            &[4, 0],
        );
        assert_eq!((block.wait_attempts, block.idle_cycles), (retry + 1, 9));
        for len in 0..6 {
            answered(&mut block, &[4, 0, 1, 0, 0, 0][..len], &[4, 0]);
            assert_eq!((block.wait_attempts, block.idle_cycles), (retry + 1, 9));
        }
    }
}

#[test]
fn skipped_observations_leave_the_port_and_retry_mirror_stale() {
    use daplink_rs::dap_rs::dap::Dap;
    use daplink_rs::dap_rs::driver::bitbang::JtagAdapter;
    let trace = Trace::new(&read_script(1, 0x11223344, false));
    let mut dap: Dap<
        '_,
        _,
        _,
        _,
        JtagAdapter<TracePin, MockDelay>,
        SwdAdapter<TracePin, MockDelay>,
        _,
    > = Dap::new(
        adapter(&trace),
        daplink_rs::NoopLeds,
        MockDelay,
        daplink_rs::NoSwo,
        "test",
        512,
    );
    let mut block: H5Block<P> = H5Block::new(250_000_000);
    let mut response = [0xa5; 8];
    assert_eq!(block.process(&[2, 1], &mut response), None);
    dap.process_command(&[2, 1], &mut response);
    block.observe(&[2, 1], &response[..2]);
    assert_eq!(&response[..2], &[2, 1]);

    // Simulate a caller bypassing every intercept observation while the bus is unavailable.
    dap.process_command(&[4, 0, 0, 0, 0, 0], &mut response);
    dap.process_command(&[2, 2], &mut response);
    assert_eq!(&response[..2], &[2, 2]);
    assert_eq!(block.wait_attempts, 6);
    assert!(block.dap.swd_held());

    // The retained intercept predicate admits SWD after the engine selected JTAG.
    let handled = if block.dap.swd_held() {
        block.transfer_block(&[6, 0, 1, 0, 6], &mut response, || MockWire(trace.clone()))
    } else {
        None
    };
    assert_eq!(handled, Some(8));
    assert_eq!(response, [6, 1, 0, 1, 0x44, 0x33, 0x22, 0x11]);
}
