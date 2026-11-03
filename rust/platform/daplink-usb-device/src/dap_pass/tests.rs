use super::*;
use std::vec::Vec;

/// A scripted transport: reads, write verdicts and session-end verdicts are consumed in order.
#[derive(Default)]
struct MockUsb {
    reads: Vec<Vec<u8>>,
    write_ok: Vec<bool>,
    ended: Vec<bool>,
    written: Vec<Vec<u8>>,
    read_calls: usize,
    ended_calls: usize,
}

impl DapTransport for MockUsb {
    fn session_ended(&mut self) -> bool {
        self.ended_calls += 1;
        if self.ended.is_empty() {
            false
        } else {
            self.ended.remove(0)
        }
    }

    fn read_command(&mut self, slot: &mut [u8]) -> Option<usize> {
        self.read_calls += 1;
        if self.reads.is_empty() {
            return None;
        }
        let packet = self.reads.remove(0);
        slot[..packet.len()].copy_from_slice(&packet);
        Some(packet.len())
    }

    fn write_response(&mut self, resp: &[u8]) -> bool {
        let ok = if self.write_ok.is_empty() {
            true
        } else {
            self.write_ok.remove(0)
        };
        if ok {
            self.written.push(resp.to_vec());
        }
        ok
    }
}

/// A recording hook set; `execute` answers a request with its first byte doubled.
#[derive(Default)]
struct Hooks {
    executes: Vec<Vec<u8>>,
    events: Vec<&'static str>,
}

impl DapPassHooks for Hooks {
    fn execute(&mut self, req: &[u8], resp: &mut [u8]) -> usize {
        self.executes.push(req.to_vec());
        resp[0] = req[0];
        resp[1] = req[0].wrapping_mul(2);
        2
    }

    fn session_reset(&mut self) {
        self.events.push("reset");
    }

    fn requests_full(&mut self) {
        self.events.push("full");
    }

    fn received(&mut self) {
        self.events.push("received");
    }

    fn batch_start(&mut self) {
        self.events.push("start");
    }

    fn batch_end(&mut self) {
        self.events.push("end");
    }

    fn executed(&mut self, _len: usize) {
        self.events.push("executed");
    }

    fn sent(&mut self, _len: usize) {
        self.events.push("sent");
    }

    fn refused(&mut self, _len: usize) {
        self.events.push("refused");
    }
}

type Queue = DapQueue<2, 64>;

fn queued(queue: &mut Queue, packets: &[&[u8]]) {
    for packet in packets {
        queue.submit(packet).expect("test queue accepts a packet");
    }
}

#[test]
fn executes_and_sends_in_submit_order() {
    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11], &[0x22]]);
    let mut usb = MockUsb::default();
    let mut hooks = Hooks::default();

    let work_remains = dap_pass(&mut queue, &mut usb, &mut hooks, false, 2);

    assert!(!work_remains);
    assert_eq!(hooks.executes, [[0x11], [0x22]]);
    assert_eq!(usb.written, [[0x11, 0x22], [0x22, 0x44]]);
    assert_eq!(
        hooks.events,
        ["start", "executed", "executed", "end", "sent", "sent"]
    );
}

#[test]
fn a_refused_write_keeps_the_response_for_the_next_pass() {
    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11], &[0x22]]);
    let mut usb = MockUsb {
        write_ok: std::vec![false],
        ..MockUsb::default()
    };
    let mut hooks = Hooks::default();

    dap_pass(&mut queue, &mut usb, &mut hooks, false, 2);

    assert!(usb.written.is_empty());
    assert_eq!(hooks.events.last(), Some(&"refused"));
    assert_eq!(queue.pending_response(), Some(&[0x11, 0x22][..]));

    let mut hooks = Hooks::default();
    dap_pass(&mut queue, &mut usb, &mut hooks, false, 2);

    assert_eq!(usb.written, [[0x11, 0x22], [0x22, 0x44]]);
    assert!(hooks.executes.is_empty());
}

#[test]
fn work_remains_only_when_the_budget_stopped_the_batch() {
    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11], &[0x22]]);
    let mut usb = MockUsb::default();
    let mut hooks = Hooks::default();
    assert!(dap_pass(&mut queue, &mut usb, &mut hooks, false, 1));

    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11], &[0x22]]);
    let mut hooks = Hooks::default();
    assert!(!dap_pass(&mut queue, &mut usb, &mut hooks, false, 2));
}

#[test]
fn a_full_response_ring_blocks_the_batch() {
    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11], &[0x22]]);
    let mut usb = MockUsb {
        write_ok: std::vec![false],
        ..MockUsb::default()
    };
    let mut hooks = Hooks::default();
    dap_pass(&mut queue, &mut usb, &mut hooks, false, 2);
    assert_eq!(hooks.executes.len(), 2);

    queued(&mut queue, &[&[0x33]]);
    usb.write_ok = std::vec![false];
    let mut hooks = Hooks::default();
    dap_pass(&mut queue, &mut usb, &mut hooks, false, 2);

    assert!(hooks.executes.is_empty());
    assert_eq!(hooks.events, ["start", "end", "refused"]);
}

#[test]
fn a_dead_session_resets_the_queue_once() {
    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11]]);
    let mut usb = MockUsb {
        ended: std::vec![true],
        ..MockUsb::default()
    };
    let mut hooks = Hooks::default();

    dap_pass(&mut queue, &mut usb, &mut hooks, true, 2);

    assert!(!queue.command_ready());
    assert_eq!(hooks.events.iter().filter(|e| **e == "reset").count(), 1);

    let mut hooks = Hooks::default();
    dap_pass(&mut queue, &mut usb, &mut hooks, true, 2);
    assert!(!hooks.events.contains(&"reset"));
}

#[test]
fn an_unpolled_pass_checks_the_session_without_reading_the_endpoint() {
    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11]]);
    let mut usb = MockUsb {
        reads: std::vec![std::vec![0x99]],
        ..MockUsb::default()
    };
    let mut hooks = Hooks::default();

    dap_pass(&mut queue, &mut usb, &mut hooks, false, 2);

    assert_eq!(usb.read_calls, 0);
    assert_eq!(usb.ended_calls, 1);
    assert_eq!(hooks.executes, [[0x11]]);
    assert_eq!(usb.written, [[0x11, 0x22]]);
}

#[test]
fn an_unpolled_reset_retires_requests_and_responses_before_execution() {
    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11], &[0x22]]);
    let mut usb = MockUsb {
        write_ok: std::vec![false],
        ..MockUsb::default()
    };
    dap_pass(&mut queue, &mut usb, &mut Hooks::default(), false, 1);
    assert!(queue.command_ready());
    assert_eq!(queue.pending_response(), Some(&[0x11, 0x22][..]));

    usb.ended = std::vec![true, false];
    usb.reads = std::vec![std::vec![0x33]];
    let mut hooks = Hooks::default();
    assert!(!dap_pass(&mut queue, &mut usb, &mut hooks, false, 2));
    assert_eq!(hooks.events, ["reset", "start", "end"]);
    assert!(hooks.executes.is_empty());
    assert!(usb.written.is_empty());
    assert_eq!(usb.read_calls, 0);
    assert!(!queue.command_ready());
    assert!(queue.pending_response().is_none());

    let mut hooks = Hooks::default();
    dap_pass(&mut queue, &mut usb, &mut hooks, true, 2);
    assert!(!hooks.events.contains(&"reset"));
    assert_eq!(hooks.executes, [[0x33]]);
    assert_eq!(usb.written, [[0x33, 0x66]]);
}

#[test]
fn a_polled_pass_takes_one_packet() {
    let mut queue = Queue::new();
    let mut usb = MockUsb {
        reads: std::vec![std::vec![0x11]],
        ..MockUsb::default()
    };
    let mut hooks = Hooks::default();

    dap_pass(&mut queue, &mut usb, &mut hooks, true, 2);

    assert_eq!(hooks.events[0], "received");
    assert_eq!(usb.written, [[0x11, 0x22]]);
}

#[test]
fn a_full_request_ring_skips_the_endpoint() {
    let mut queue = Queue::new();
    queued(&mut queue, &[&[0x11], &[0x22]]);
    let mut usb = MockUsb {
        reads: std::vec![std::vec![0x33]],
        ..MockUsb::default()
    };
    let mut hooks = Hooks::default();

    dap_pass(&mut queue, &mut usb, &mut hooks, true, 0);

    assert_eq!(usb.read_calls, 0);
    assert_eq!(hooks.events[0], "full");
}
