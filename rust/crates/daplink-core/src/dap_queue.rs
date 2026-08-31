//! Fixed-capacity request/response ring pair for CMSIS-DAP command pipelining.

use core::sync::atomic::{AtomicBool, Ordering};

/// CMSIS-DAP `DAP_TransferAbort` command id.
pub const DAP_TRANSFER_ABORT: u8 = 0x07;

/// Why a request or response operation could not use a queue slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueueError {
    /// The ring already holds its capacity of `N` packets, so the host must wait.
    Full,
    /// The packet is larger than the per-slot capacity `S`.
    Oversized,
}

/// The outcome of [`DapQueue::submit`] or [`DapQueue::receive_with`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Submit {
    /// The packet was appended to the request ring.
    Enqueued,
    /// The packet was a `DAP_TransferAbort`: the abort flag was set and it was not enqueued.
    AbortIntercepted,
    /// The packet was zero-length and was dropped without enqueuing.
    Dropped,
}

/// One strict-FIFO ring of up to `N` packets, each up to `S` bytes.
struct Ring<const N: usize, const S: usize> {
    data: [[u8; S]; N],
    len: [usize; N],
    /// Index of the oldest occupied slot (the next to pop).
    head: usize,
    /// Number of occupied slots.
    count: usize,
}

impl<const N: usize, const S: usize> Ring<N, S> {
    const fn new() -> Self {
        Self {
            data: [[0u8; S]; N],
            len: [0usize; N],
            head: 0,
            count: 0,
        }
    }

    fn push(&mut self, pkt: &[u8]) -> Result<(), QueueError> {
        if pkt.len() > S {
            return Err(QueueError::Oversized);
        }
        if self.count == N {
            return Err(QueueError::Full);
        }
        let idx = (self.head + self.count) % N;
        self.data[idx][..pkt.len()].copy_from_slice(pkt);
        self.len[idx] = pkt.len();
        self.count += 1;
        Ok(())
    }

    fn front(&self) -> Option<&[u8]> {
        (self.count != 0).then(|| &self.data[self.head][..self.len[self.head]])
    }

    /// Pop the oldest packet into `out`, returning its stored length (may exceed `out.len()`).
    fn pop_into(&mut self, out: &mut [u8]) -> Option<usize> {
        if self.count == 0 {
            return None;
        }
        let idx = self.head;
        let n = self.len[idx];
        let copied = core::cmp::min(n, out.len());
        out[..copied].copy_from_slice(&self.data[idx][..copied]);
        self.head = (self.head + 1) % N;
        self.count -= 1;
        Some(n)
    }

    /// Copy the oldest packet into `out` without removing it (see [`drop_front`](Self::drop_front)).
    fn peek_into(&self, out: &mut [u8]) -> Option<usize> {
        if self.count == 0 {
            return None;
        }
        let idx = self.head;
        let n = self.len[idx];
        let copied = core::cmp::min(n, out.len());
        out[..copied].copy_from_slice(&self.data[idx][..copied]);
        Some(n)
    }

    /// Drop the oldest packet, returning whether one was removed.
    fn drop_front(&mut self) -> bool {
        if self.count == 0 {
            return false;
        }
        self.head = (self.head + 1) % N;
        self.count -= 1;
        true
    }

    /// Discard every packet. O(1): only occupancy bookkeeping is touched.
    fn clear(&mut self) {
        self.head = 0;
        self.count = 0;
    }
}

/// A request/response ring pair for pipelined CMSIS-DAP command processing. `N` is the queue depth
/// (advertised `DAP_PACKET_COUNT`) and `S` the per-packet byte capacity.
pub struct DapQueue<const N: usize, const S: usize> {
    req: Ring<N, S>,
    resp: Ring<N, S>,
    abort: AtomicBool,
    /// Bumped only by [`reset`](Self::reset); see [`generation`](Self::generation).
    generation: u32,
}

impl<const N: usize, const S: usize> DapQueue<N, S> {
    /// An empty queue. `const`, so it can back a caller-owned `static` cell.
    pub const fn new() -> Self {
        Self {
            req: Ring::new(),
            resp: Ring::new(),
            abort: AtomicBool::new(false),
            generation: 0,
        }
    }

    /// The queue depth `N` (the maximum number of in-flight requests / advertised packet count).
    pub const fn depth(&self) -> usize {
        N
    }

    /// The per-packet byte capacity `S`.
    pub const fn packet_capacity(&self) -> usize {
        S
    }

    /// Offer a received OUT packet to the request ring; intercepts [`DAP_TRANSFER_ABORT`] instead
    /// of enqueuing it.
    pub fn submit(&mut self, packet: &[u8]) -> Result<Submit, QueueError> {
        if packet.is_empty() {
            return Ok(Submit::Dropped);
        }
        if packet.first() == Some(&DAP_TRANSFER_ABORT) {
            self.abort.store(true, Ordering::Release);
            return Ok(Submit::AbortIntercepted);
        }
        self.req.push(packet)?;
        Ok(Submit::Enqueued)
    }

    /// Receive directly into the next request slot; `None` means no packet was available.
    pub fn receive_with<F>(&mut self, receive: F) -> Result<Option<Submit>, QueueError>
    where
        F: FnOnce(&mut [u8]) -> Option<usize>,
    {
        if self.req.count == N {
            return Err(QueueError::Full);
        }
        let idx = (self.req.head + self.req.count) % N;
        let Some(n) = receive(&mut self.req.data[idx]) else {
            return Ok(None);
        };
        if n > S {
            return Err(QueueError::Oversized);
        }
        if n == 0 {
            return Ok(Some(Submit::Dropped));
        }
        if self.req.data[idx][0] == DAP_TRANSFER_ABORT {
            self.abort.store(true, Ordering::Release);
            return Ok(Some(Submit::AbortIntercepted));
        }
        self.req.len[idx] = n;
        self.req.count += 1;
        Ok(Some(Submit::Enqueued))
    }

    /// Dequeue the oldest pending request into `out`, returning its length, or `None` if empty.
    pub fn take_request(&mut self, out: &mut [u8]) -> Option<usize> {
        self.req.pop_into(out)
    }

    /// Execute the oldest request directly from its slot into the next response slot.
    pub fn execute_next_with<F>(&mut self, execute: F) -> Result<Option<usize>, QueueError>
    where
        F: FnOnce(&[u8], &mut [u8]) -> usize,
    {
        if self.req.count == 0 {
            return Ok(None);
        }
        if self.resp.count == N {
            return Err(QueueError::Full);
        }
        let req_idx = self.req.head;
        let resp_idx = (self.resp.head + self.resp.count) % N;
        let response_len = execute(
            &self.req.data[req_idx][..self.req.len[req_idx]],
            &mut self.resp.data[resp_idx],
        );
        self.req.drop_front();
        if response_len > S {
            return Err(QueueError::Oversized);
        }
        if response_len != 0 {
            self.resp.len[resp_idx] = response_len;
            self.resp.count += 1;
        }
        Ok(Some(response_len))
    }

    /// Number of requests waiting to be executed.
    pub fn requests_pending(&self) -> usize {
        self.req.count
    }

    /// Whether the request ring is at capacity.
    pub fn requests_full(&self) -> bool {
        self.req.count == N
    }

    /// Append an executed command's response, preserving execution order; an empty `packet`
    /// enqueues nothing.
    pub fn post_response(&mut self, packet: &[u8]) -> Result<(), QueueError> {
        if packet.is_empty() {
            return Ok(());
        }
        self.resp.push(packet)
    }

    /// Dequeue the oldest pending response into `out`, returning its length, or `None` if empty.
    pub fn take_response(&mut self, out: &mut [u8]) -> Option<usize> {
        self.resp.pop_into(out)
    }

    /// Number of responses waiting to be written to the host.
    pub fn responses_pending(&self) -> usize {
        self.resp.count
    }

    /// Whether the response ring is at capacity (`N` responses buffered).
    pub fn responses_full(&self) -> bool {
        self.resp.count == N
    }

    /// Copy the oldest pending response into `out` without dequeuing it; pairs with
    /// [`discard_response`](Self::discard_response) for a send that may need retrying.
    pub fn peek_response(&self, out: &mut [u8]) -> Option<usize> {
        self.resp.peek_into(out)
    }

    /// Borrow the oldest pending response in place until the caller accepts or defers it.
    pub fn pending_response(&self) -> Option<&[u8]> {
        self.resp.front()
    }

    /// Discard the oldest pending response after it has been sent, returning whether one was
    /// removed.
    pub fn discard_response(&mut self) -> bool {
        self.resp.drop_front()
    }

    /// [`discard_response`](Self::discard_response), but only for the `generation` the peek came
    /// from; a [`reset`](Self::reset) since then removes nothing. An awaiting send must use this.
    pub fn discard_response_for(&mut self, generation: u32) -> bool {
        generation == self.generation && self.resp.drop_front()
    }

    /// Whether a `DAP_TransferAbort` has been seen since the flag was last cleared.
    pub fn abort_requested(&self) -> bool {
        self.abort.load(Ordering::Acquire)
    }

    /// Clear the abort flag; the command engine does this at transfer entry.
    pub fn clear_abort(&self) {
        self.abort.store(false, Ordering::Release);
    }

    /// The shared abort flag, for the command engine's wait-retry loops to poll.
    pub fn abort_flag(&self) -> &AtomicBool {
        &self.abort
    }

    /// Discard every queued request and response, clear the abort flag, and bump the
    /// [`generation`](Self::generation) counter; call once when a session dies.
    pub fn reset(&mut self) {
        self.req.clear();
        self.resp.clear();
        self.abort.store(false, Ordering::Release);
        self.generation = self.generation.wrapping_add(1);
    }

    /// The session generation, bumped only by [`reset`](Self::reset). A sender snapshots it with a
    /// peek and hands it to `discard_response_for`, so a session that died mid-send retires nothing.
    pub fn generation(&self) -> u32 {
        self.generation
    }
}

impl<const N: usize, const S: usize> Default for DapQueue<N, S> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A packet of `len` bytes whose first byte is `first` and rest ascending, so a round trip is
    /// verifiable and never accidentally looks like an abort (unless `first` is `0x07`).
    fn pkt(first: u8, len: usize) -> [u8; 64] {
        let mut b = [0u8; 64];
        b[0] = first;
        for (i, x) in b.iter_mut().enumerate().take(len).skip(1) {
            *x = i as u8;
        }
        b
    }

    #[test]
    fn submit_and_take_preserve_fifo_order() {
        let mut q: DapQueue<4, 64> = DapQueue::new();
        assert_eq!(q.depth(), 4);
        assert_eq!(q.packet_capacity(), 64);

        for id in [0x10u8, 0x11, 0x12] {
            assert_eq!(q.submit(&pkt(id, 8)[..8]), Ok(Submit::Enqueued));
        }
        assert_eq!(q.requests_pending(), 3);

        // Requests come back oldest-first.
        for id in [0x10u8, 0x11, 0x12] {
            let mut out = [0u8; 64];
            let n = q.take_request(&mut out).expect("request pending");
            assert_eq!(n, 8);
            assert_eq!(out[0], id);
        }
        assert_eq!(q.take_request(&mut [0u8; 64]), None);
        assert_eq!(q.requests_pending(), 0);
    }

    #[test]
    fn responses_drain_in_execution_order() {
        let mut q: DapQueue<3, 64> = DapQueue::new();
        for id in [0xA0u8, 0xA1, 0xA2] {
            q.post_response(&pkt(id, 5)[..5]).unwrap();
        }
        assert_eq!(q.responses_pending(), 3);
        for id in [0xA0u8, 0xA1, 0xA2] {
            let mut out = [0u8; 64];
            let n = q.take_response(&mut out).expect("response pending");
            assert_eq!(n, 5);
            assert_eq!(out[0], id);
        }
        assert_eq!(q.take_response(&mut [0u8; 64]), None);
    }

    #[test]
    fn full_request_ring_backpressures() {
        let mut q: DapQueue<2, 64> = DapQueue::new();
        assert_eq!(q.submit(&pkt(1, 4)[..4]), Ok(Submit::Enqueued));
        assert_eq!(q.submit(&pkt(2, 4)[..4]), Ok(Submit::Enqueued));
        assert!(q.requests_full());
        // Third submit is refused, not silently dropped.
        assert_eq!(q.submit(&pkt(3, 4)[..4]), Err(QueueError::Full));
        // Draining one slot makes room again, and the ring wraps correctly.
        let mut out = [0u8; 64];
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 1);
        assert_eq!(q.submit(&pkt(3, 4)[..4]), Ok(Submit::Enqueued));
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 2);
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 3);
    }

    #[test]
    fn oversized_packet_rejected() {
        let mut q: DapQueue<2, 8> = DapQueue::new();
        assert_eq!(q.submit(&[0u8; 9]), Err(QueueError::Oversized));
        assert_eq!(q.post_response(&[0u8; 9]), Err(QueueError::Oversized));
        assert_eq!(q.requests_pending(), 0);
        assert_eq!(q.responses_pending(), 0);
    }

    #[test]
    fn transfer_abort_is_peeked_not_enqueued() {
        let mut q: DapQueue<4, 64> = DapQueue::new();
        assert!(!q.abort_requested());

        // A normal command enqueues.
        assert_eq!(q.submit(&pkt(0x05, 4)[..4]), Ok(Submit::Enqueued));

        // A DAP_TransferAbort sets the flag and is dropped without consuming a slot.
        let abort = [DAP_TRANSFER_ABORT];
        assert_eq!(q.submit(&abort), Ok(Submit::AbortIntercepted));
        assert!(q.abort_requested());
        assert!(q.abort_flag().load(Ordering::Acquire));
        assert_eq!(
            q.requests_pending(),
            1,
            "abort must not occupy a request slot"
        );

        // The engine clears it at transfer entry.
        q.clear_abort();
        assert!(!q.abort_requested());

        // The earlier real command is still queued and intact.
        let mut out = [0u8; 64];
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 0x05);
    }

    #[test]
    fn draining_into_empty_slice_pops_without_copying() {
        // Popping into an empty slice must still advance the ring: the slots matter, not the bytes.
        let mut q: DapQueue<3, 64> = DapQueue::new();
        for id in [0x20u8, 0x21, 0x22] {
            q.submit(&pkt(id, 8)[..8]).unwrap();
            q.post_response(&pkt(id, 8)[..8]).unwrap();
        }
        assert_eq!(q.requests_pending(), 3);
        assert_eq!(q.responses_pending(), 3);

        let mut drained = 0;
        while q.take_request(&mut []).is_some() {
            drained += 1;
        }
        assert_eq!(drained, 3, "every request slot is emptied");
        while q.take_response(&mut []).is_some() {}
        assert_eq!(q.requests_pending(), 0);
        assert_eq!(q.responses_pending(), 0);

        // The drained ring is fully reusable (head/count wrapped cleanly).
        assert_eq!(q.submit(&pkt(0x30, 4)[..4]), Ok(Submit::Enqueued));
        let mut out = [0u8; 64];
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 0x30);
    }

    #[test]
    fn empty_response_enqueues_nothing() {
        let mut q: DapQueue<2, 64> = DapQueue::new();
        q.post_response(&[]).unwrap();
        assert_eq!(q.responses_pending(), 0);
        assert_eq!(q.take_response(&mut [0u8; 64]), None);
    }

    #[test]
    fn peek_response_is_non_consuming_and_discard_commits() {
        // A refused IN write must leave the peeked response queued and in order.
        let mut q: DapQueue<3, 64> = DapQueue::new();
        for id in [0xB0u8, 0xB1, 0xB2] {
            q.post_response(&pkt(id, 6)[..6]).unwrap();
        }
        assert!(q.responses_full());

        // Peeking the same response repeatedly never advances the ring.
        let mut out = [0u8; 64];
        for _ in 0..3 {
            assert_eq!(q.peek_response(&mut out), Some(6));
            assert_eq!(out[0], 0xB0);
            assert_eq!(q.responses_pending(), 3);
        }

        // Discard commits the peeked packet; the next peek sees the following one, in order.
        assert!(q.discard_response());
        assert_eq!(q.responses_pending(), 2);
        assert!(!q.responses_full());
        assert_eq!(q.peek_response(&mut out), Some(6));
        assert_eq!(out[0], 0xB1);

        // Drain the rest and confirm both empties report correctly.
        assert!(q.discard_response());
        assert!(q.discard_response());
        assert_eq!(q.peek_response(&mut out), None);
        assert!(!q.discard_response());
        assert_eq!(q.responses_pending(), 0);
    }

    #[test]
    fn reset_drains_rings_clears_abort_and_bumps_generation() {
        let mut q: DapQueue<3, 64> = DapQueue::new();
        assert_eq!(q.generation(), 0);

        // Populate every kind of state a dying session can leave behind.
        q.submit(&pkt(0x50, 8)[..8]).unwrap();
        q.submit(&pkt(0x51, 8)[..8]).unwrap();
        q.post_response(&pkt(0x52, 8)[..8]).unwrap();
        q.submit(&[DAP_TRANSFER_ABORT]).unwrap();
        assert!(q.abort_requested());

        q.reset();

        assert_eq!(q.requests_pending(), 0, "requests drained");
        assert_eq!(q.responses_pending(), 0, "responses drained");
        assert!(!q.abort_requested(), "stale abort cleared");
        assert_eq!(q.generation(), 1, "generation bumped once per reset");
        assert_eq!(q.take_request(&mut [0u8; 64]), None);
        assert_eq!(q.peek_response(&mut [0u8; 64]), None);

        // Each further reset bumps again, even on an already-empty queue.
        q.reset();
        assert_eq!(q.generation(), 2);
    }

    #[test]
    fn queue_is_fully_reusable_after_reset() {
        let mut q: DapQueue<2, 64> = DapQueue::new();
        // Leave the rings mid-wrap so the reset must recover from a nonzero head.
        q.submit(&pkt(0x60, 4)[..4]).unwrap();
        q.submit(&pkt(0x61, 4)[..4]).unwrap();
        let mut out = [0u8; 64];
        q.take_request(&mut out).unwrap();
        q.post_response(&pkt(0x62, 4)[..4]).unwrap();

        q.reset();

        // A fresh session runs through the full request/response cycle, in order.
        assert_eq!(q.submit(&pkt(0x70, 4)[..4]), Ok(Submit::Enqueued));
        assert_eq!(q.submit(&pkt(0x71, 4)[..4]), Ok(Submit::Enqueued));
        assert!(q.requests_full());
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 0x70);
        q.post_response(&pkt(0x72, 4)[..4]).unwrap();
        assert_eq!(q.peek_response(&mut out), Some(4));
        assert_eq!(out[0], 0x72);
        assert!(q.discard_response());
    }

    #[test]
    fn generation_guard_detects_a_reset_across_an_unlocked_execution() {
        // Snapshot the generation with the take, execute on the copy, post only if it still matches.
        let mut q: DapQueue<2, 64> = DapQueue::new();
        q.submit(&pkt(0x40, 6)[..6]).unwrap();

        let mut req = [0u8; 64];
        let n = q.take_request(&mut req).expect("request pending");
        let gen = q.generation();

        // The session dies while the command executes on the copy.
        q.reset();

        // The executor's re-check fails, so the stale response is dropped by the caller.
        assert_ne!(q.generation(), gen, "reset changed the generation");
        assert_eq!(n, 6);
        assert_eq!(q.responses_pending(), 0);
    }

    #[test]
    fn epoch_scoped_retire_commits_within_one_session() {
        // The happy path: peek, "send", retire with the generation the peek was taken under.
        let mut q: DapQueue<3, 64> = DapQueue::new();
        for id in [0xC0u8, 0xC1] {
            q.post_response(&pkt(id, 6)[..6]).unwrap();
        }
        let mut out = [0u8; 64];
        for id in [0xC0u8, 0xC1] {
            let gen = q.generation();
            assert_eq!(q.peek_response(&mut out), Some(6));
            assert_eq!(out[0], id);
            assert!(q.discard_response_for(gen), "live generation retires");
        }
        assert_eq!(q.responses_pending(), 0);
        // An empty ring retires nothing even at the live generation.
        assert!(!q.discard_response_for(q.generation()));
    }

    #[test]
    fn epoch_scoped_retire_spares_the_next_session_after_a_reset() {
        // An unscoped retire would eat the new session's first response and shift every later one.
        let mut q: DapQueue<3, 64> = DapQueue::new();
        q.post_response(&pkt(0xD0, 6)[..6]).unwrap();
        let stale = q.generation();
        let mut out = [0u8; 64];
        assert_eq!(q.peek_response(&mut out), Some(6));

        q.reset();
        q.post_response(&pkt(0xD1, 4)[..4]).unwrap();

        assert!(
            !q.discard_response_for(stale),
            "a retire from the dead session must not touch the live ring"
        );
        assert_eq!(q.responses_pending(), 1, "the new response survives");
        assert_eq!(q.peek_response(&mut out), Some(4));
        assert_eq!(out[0], 0xD1);

        // And the live generation still retires normally.
        assert!(q.discard_response_for(q.generation()));
        assert_eq!(q.responses_pending(), 0);
    }

    #[test]
    fn epoch_scoped_retire_is_idempotent_for_a_stale_generation() {
        // The guard is on the generation, not on occupancy, so a stale retire repeats harmlessly.
        let mut q: DapQueue<3, 64> = DapQueue::new();
        q.post_response(&pkt(0xE0, 4)[..4]).unwrap();
        let stale = q.generation();
        q.reset();
        for id in [0xE1u8, 0xE2, 0xE3] {
            q.post_response(&pkt(id, 4)[..4]).unwrap();
        }
        for _ in 0..5 {
            assert!(!q.discard_response_for(stale));
            assert_eq!(q.responses_pending(), 3);
        }
        // Still strictly in order for the live session.
        let mut out = [0u8; 64];
        for id in [0xE1u8, 0xE2, 0xE3] {
            assert_eq!(q.peek_response(&mut out), Some(4));
            assert_eq!(out[0], id);
            assert!(q.discard_response_for(q.generation()));
        }
    }

    #[test]
    fn every_reset_invalidates_an_outstanding_retire() {
        // Two resets in a row must not let a doubly-stale generation match again.
        let mut q: DapQueue<2, 64> = DapQueue::new();
        let first = q.generation();
        q.reset();
        let second = q.generation();
        q.reset();
        q.post_response(&pkt(0xF0, 4)[..4]).unwrap();
        assert!(!q.discard_response_for(first));
        assert!(!q.discard_response_for(second));
        assert_eq!(q.responses_pending(), 1);
        assert!(q.discard_response_for(q.generation()));
    }

    #[test]
    fn empty_packet_is_dropped_and_is_not_an_abort() {
        let mut q: DapQueue<2, 64> = DapQueue::new();
        assert_eq!(q.submit(&[]), Ok(Submit::Dropped));
        // A ZLP must neither arm the abort flag nor cost a request slot.
        assert!(!q.abort_requested());
        assert_eq!(q.requests_pending(), 0);
    }

    #[test]
    fn in_place_receive_preserves_framing_abort_and_backpressure() {
        let mut q: DapQueue<2, 8> = DapQueue::new();
        assert_eq!(q.receive_with(|_| Some(0)), Ok(Some(Submit::Dropped)));
        assert_eq!(q.receive_with(|_| Some(9)), Err(QueueError::Oversized));
        assert_eq!(q.requests_pending(), 0);
        assert_eq!(
            q.receive_with(|slot| {
                slot[..3].copy_from_slice(&[0x21, 1, 2]);
                Some(3)
            }),
            Ok(Some(Submit::Enqueued))
        );
        assert_eq!(
            q.receive_with(|slot| {
                slot[0] = DAP_TRANSFER_ABORT;
                Some(1)
            }),
            Ok(Some(Submit::AbortIntercepted))
        );
        assert!(q.abort_requested());
        assert_eq!(q.requests_pending(), 1);
        assert_eq!(q.receive_with(|_| None), Ok(None));

        q.receive_with(|slot| {
            slot[..2].copy_from_slice(&[0x22, 3]);
            Some(2)
        })
        .unwrap();
        let mut called = false;
        assert_eq!(
            q.receive_with(|_| {
                called = true;
                None
            }),
            Err(QueueError::Full)
        );
        assert!(!called, "a full ring must not touch the endpoint");
    }

    #[test]
    fn in_place_execute_and_send_preserve_fifo_order_across_wrap() {
        let mut q: DapQueue<2, 8> = DapQueue::new();
        for id in [0x31u8, 0x32] {
            q.receive_with(|slot| {
                slot[..2].copy_from_slice(&[id, 7]);
                Some(2)
            })
            .unwrap();
        }

        for id in [0x31u8, 0x32] {
            assert_eq!(
                q.execute_next_with(|request, response| {
                    assert_eq!(request, &[id, 7]);
                    response[..2].copy_from_slice(&[id + 0x40, 9]);
                    2
                }),
                Ok(Some(2))
            );
        }
        assert_eq!(q.execute_next_with(|_, _| 0), Ok(None));

        for id in [0x71u8, 0x72] {
            assert_eq!(q.pending_response(), Some(&[id, 9][..]));
            assert!(q.discard_response());
        }
        assert_eq!(q.pending_response(), None);

        q.receive_with(|slot| {
            slot[0] = 0x33;
            Some(1)
        })
        .unwrap();
        assert_eq!(q.execute_next_with(|_, _| 0), Ok(Some(0)));
        assert_eq!(q.responses_pending(), 0);

        q.receive_with(|slot| {
            slot[0] = 0x34;
            Some(1)
        })
        .unwrap();
        assert_eq!(q.execute_next_with(|_, _| 9), Err(QueueError::Oversized));
        assert_eq!(q.requests_pending(), 0);
        assert_eq!(q.responses_pending(), 0);
    }
}
