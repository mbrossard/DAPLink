//! Fixed-capacity request/response ring pair for CMSIS-DAP command pipelining.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

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
    /// The packet was a `DAP_TransferAbort`: consumed without a reply and without a slot.
    AbortIntercepted,
    /// The packet was zero-length and was dropped without enqueuing.
    Dropped,
}

/// SPSC ring of `N` packets of `S` bytes; each method belongs to exactly one side.
struct Ring<const N: usize, const S: usize> {
    data: [UnsafeCell<[u8; S]>; N],
    len: [UnsafeCell<usize>; N],
    /// Next slot to pop, in `0..2 * N`; written only by the consumer.
    head: AtomicUsize,
    /// Next slot to fill, in `0..2 * N`; written only by the producer.
    tail: AtomicUsize,
}

impl<const N: usize, const S: usize> Ring<N, S> {
    const fn new() -> Self {
        Self {
            data: [const { UnsafeCell::new([0u8; S]) }; N],
            len: [const { UnsafeCell::new(0usize) }; N],
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    /// Cursors run over `0..2 * N` so a full ring is distinguishable from an empty one at any `N`.
    fn advance(cursor: usize) -> usize {
        if cursor + 1 == 2 * N {
            0
        } else {
            cursor + 1
        }
    }

    /// The storage slot a cursor names.
    fn slot(cursor: usize) -> usize {
        if cursor >= N {
            cursor - N
        } else {
            cursor
        }
    }

    fn occupancy(head: usize, tail: usize) -> usize {
        if tail >= head {
            tail - head
        } else {
            tail + 2 * N - head
        }
    }

    fn count(&self) -> usize {
        Self::occupancy(
            self.head.load(Ordering::Acquire),
            self.tail.load(Ordering::Acquire),
        )
    }

    /// Producer: the free slot, or `None` when full; unseen until [`publish`](Self::publish).
    // The producer's own slot, and every caller holds a `&mut` handle, so only one is ever live.
    #[allow(clippy::mut_from_ref)]
    fn staging(&self) -> Option<&mut [u8; S]> {
        let tail = self.tail.load(Ordering::Relaxed);
        if Self::occupancy(self.head.load(Ordering::Acquire), tail) == N {
            return None;
        }
        // SAFETY: the producer owns the slot at `tail` until it publishes it.
        Some(unsafe { &mut *self.data[Self::slot(tail)].get() })
    }

    /// Producer: hand the staged slot's `len` bytes to the consumer.
    fn publish(&self, len: usize) {
        let tail = self.tail.load(Ordering::Relaxed);
        // SAFETY: still the producer's slot; the `Release` store below hands it over.
        unsafe { *self.len[Self::slot(tail)].get() = len };
        self.tail.store(Self::advance(tail), Ordering::Release);
    }

    /// Producer: copy `pkt` into the free slot and publish it.
    fn push(&self, pkt: &[u8]) -> Result<(), QueueError> {
        if pkt.len() > S {
            return Err(QueueError::Oversized);
        }
        let Some(slot) = self.staging() else {
            return Err(QueueError::Full);
        };
        slot[..pkt.len()].copy_from_slice(pkt);
        self.publish(pkt.len());
        Ok(())
    }

    /// Consumer: the oldest published packet, still in its slot.
    fn front(&self) -> Option<&[u8]> {
        let head = self.head.load(Ordering::Relaxed);
        if head == self.tail.load(Ordering::Acquire) {
            return None;
        }
        let slot = Self::slot(head);
        // SAFETY: a published slot is the consumer's, and the `Acquire` load above ordered its bytes.
        unsafe {
            let bytes: &[u8; S] = &*self.data[slot].get();
            Some(&bytes[..*self.len[slot].get()])
        }
    }

    /// Consumer: release the oldest packet's slot back to the producer.
    fn pop(&self) -> bool {
        let head = self.head.load(Ordering::Relaxed);
        if head == self.tail.load(Ordering::Acquire) {
            return false;
        }
        self.head.store(Self::advance(head), Ordering::Release);
        true
    }

    /// Discard every packet; the caller guarantees neither side is running.
    fn clear(&self) {
        self.head.store(0, Ordering::Relaxed);
        self.tail.store(0, Ordering::Relaxed);
    }
}

/// Request/response ring pair of depth `N` and packet size `S`; see [`split`](Self::split).
pub struct DapQueue<const N: usize, const S: usize> {
    req: Ring<N, S>,
    resp: Ring<N, S>,
    /// Bumped only by [`reset`](DapQueue::reset); see [`generation`](DapQueue::generation).
    generation: AtomicU32,
}

// SAFETY: split handles uniquely own ring ends; split reset requires quiescent ends and slot borrows.
unsafe impl<const N: usize, const S: usize> Sync for DapQueue<N, S> {}

impl<const N: usize, const S: usize> DapQueue<N, S> {
    /// An empty queue. `const`, so it can back a caller-owned `static` cell.
    pub const fn new() -> Self {
        Self {
            req: Ring::new(),
            resp: Ring::new(),
            generation: AtomicU32::new(0),
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

    /// `P` as a packet size to advertise; the build fails if a slot holds fewer than `P` bytes.
    pub const fn packet_size<const P: u16>() -> PacketSize {
        const { assert!(P as usize <= S, "DAP packet size exceeds the queue slot") };
        PacketSize(P)
    }

    /// `E` as the DAP bulk endpoint size; the build fails if a slot holds fewer than `E` bytes.
    pub const fn endpoint_size<const E: u16>() -> EndpointSize {
        const { assert!(E as usize <= S, "DAP endpoint size exceeds the queue slot") };
        EndpointSize(E)
    }

    /// One handle per side (reader, executor, writer); the `&mut self` borrow keeps them unique.
    pub fn split(
        &mut self,
    ) -> (
        RequestProducer<'_, N, S>,
        CommandExecutor<'_, N, S>,
        ResponseConsumer<'_, N, S>,
    ) {
        let queue: &Self = self;
        (
            RequestProducer { queue },
            CommandExecutor { queue },
            ResponseConsumer { queue },
        )
    }

    /// Enqueue a received OUT packet; [`DAP_TRANSFER_ABORT`] is consumed, never enqueued or answered.
    pub fn submit(&mut self, packet: &[u8]) -> Result<Submit, QueueError> {
        submit(&self.req, packet)
    }

    /// Receive directly into the next request slot; `None` means no packet was available.
    pub fn receive_with<F>(&mut self, receive: F) -> Result<Option<Submit>, QueueError>
    where
        F: FnOnce(&mut [u8]) -> Option<usize>,
    {
        receive_with(&self.req, receive)
    }

    #[cfg(test)]
    fn take_request(&mut self, out: &mut [u8]) -> Option<usize> {
        take_request(&self.req, out)
    }

    /// Execute the oldest request directly from its slot into the next response slot.
    pub fn execute_next_with<F>(&mut self, execute: F) -> Result<Option<usize>, QueueError>
    where
        F: FnOnce(&[u8], &mut [u8]) -> usize,
    {
        execute_next_with(&self.req, &self.resp, execute)
    }

    #[cfg(test)]
    fn requests_pending(&self) -> usize {
        self.req.count()
    }

    /// Whether the request ring is at capacity.
    pub fn requests_full(&self) -> bool {
        self.req.count() == N
    }

    /// Append an executed command's response in order; an empty `packet` enqueues nothing.
    pub fn post_response(&mut self, packet: &[u8]) -> Result<(), QueueError> {
        post_response(&self.resp, packet)
    }

    #[cfg(test)]
    fn responses_pending(&self) -> usize {
        self.resp.count()
    }

    /// Whether the response ring is at capacity (`N` responses buffered).
    pub fn responses_full(&self) -> bool {
        self.resp.count() == N
    }

    /// Whether one queued request can execute without overflowing the response ring.
    pub fn command_ready(&self) -> bool {
        self.req.count() != 0 && self.resp.count() != N
    }

    /// Copy the oldest response into `out` without dequeuing it; returns its full length, even past `out`.
    pub fn peek_response(&self, out: &mut [u8]) -> Option<usize> {
        peek_response(&self.resp, out)
    }

    /// Borrow the oldest pending response in place until the caller accepts or defers it.
    pub fn pending_response(&self) -> Option<&[u8]> {
        self.resp.front()
    }

    /// Discard the oldest response once sent; returns whether one was removed.
    pub fn discard_response(&mut self) -> bool {
        self.resp.pop()
    }

    /// [`discard_response`](Self::discard_response) scoped to the peek's `generation`.
    pub fn discard_response_for(&mut self, generation: u32) -> bool {
        generation == self.generation() && self.resp.pop()
    }

    /// Discard every queued packet and bump the [`generation`](Self::generation); call when a session dies.
    pub fn reset(&mut self) {
        reset(self);
    }

    /// The session generation, bumped only by [`reset`](Self::reset).
    pub fn generation(&self) -> u32 {
        self.generation.load(Ordering::Acquire)
    }
}

impl<const N: usize, const S: usize> Default for DapQueue<N, S> {
    fn default() -> Self {
        Self::new()
    }
}

fn submit<const N: usize, const S: usize>(
    req: &Ring<N, S>,
    packet: &[u8],
) -> Result<Submit, QueueError> {
    if packet.is_empty() {
        return Ok(Submit::Dropped);
    }
    if packet.first() == Some(&DAP_TRANSFER_ABORT) {
        return Ok(Submit::AbortIntercepted);
    }
    req.push(packet)?;
    Ok(Submit::Enqueued)
}

fn receive_with<const N: usize, const S: usize, F>(
    req: &Ring<N, S>,
    receive: F,
) -> Result<Option<Submit>, QueueError>
where
    F: FnOnce(&mut [u8]) -> Option<usize>,
{
    let Some(slot) = req.staging() else {
        return Err(QueueError::Full);
    };
    let Some(n) = receive(&mut slot[..]) else {
        return Ok(None);
    };
    if n > S {
        return Err(QueueError::Oversized);
    }
    if n == 0 {
        return Ok(Some(Submit::Dropped));
    }
    if slot[0] == DAP_TRANSFER_ABORT {
        return Ok(Some(Submit::AbortIntercepted));
    }
    req.publish(n);
    Ok(Some(Submit::Enqueued))
}

/// Publish `len` bytes staged in the free request slot, framed as in [`submit`].
fn commit<const N: usize, const S: usize>(
    req: &Ring<N, S>,
    len: usize,
) -> Result<Submit, QueueError> {
    if len > S {
        return Err(QueueError::Oversized);
    }
    if len == 0 {
        return Ok(Submit::Dropped);
    }
    let slot = req
        .staging()
        .expect("a staged slot cannot fill behind its producer");
    if slot[0] == DAP_TRANSFER_ABORT {
        return Ok(Submit::AbortIntercepted);
    }
    req.publish(len);
    Ok(Submit::Enqueued)
}

#[cfg(test)]
fn take_request<const N: usize, const S: usize>(req: &Ring<N, S>, out: &mut [u8]) -> Option<usize> {
    let n = copy_front(req, out)?;
    req.pop();
    Some(n)
}

fn execute_next_with<const N: usize, const S: usize, F>(
    req: &Ring<N, S>,
    resp: &Ring<N, S>,
    execute: F,
) -> Result<Option<usize>, QueueError>
where
    F: FnOnce(&[u8], &mut [u8]) -> usize,
{
    if req.count() == 0 {
        return Ok(None);
    }
    let Some(response) = resp.staging() else {
        return Err(QueueError::Full);
    };
    let request = req.front().expect("the request ring is not empty");
    let response_len = execute(request, &mut response[..]);
    req.pop();
    if response_len > S {
        return Err(QueueError::Oversized);
    }
    if response_len != 0 {
        resp.publish(response_len);
    }
    Ok(Some(response_len))
}

fn post_response<const N: usize, const S: usize>(
    resp: &Ring<N, S>,
    packet: &[u8],
) -> Result<(), QueueError> {
    if packet.is_empty() {
        return Ok(());
    }
    resp.push(packet)
}

fn peek_response<const N: usize, const S: usize>(
    resp: &Ring<N, S>,
    out: &mut [u8],
) -> Option<usize> {
    copy_front(resp, out)
}

/// Copy the oldest packet into `out`, returning its stored length (which may exceed `out.len()`).
fn copy_front<const N: usize, const S: usize>(ring: &Ring<N, S>, out: &mut [u8]) -> Option<usize> {
    let packet = ring.front()?;
    let copied = core::cmp::min(packet.len(), out.len());
    out[..copied].copy_from_slice(&packet[..copied]);
    Some(packet.len())
}

fn reset<const N: usize, const S: usize>(queue: &DapQueue<N, S>) {
    queue.req.clear();
    queue.resp.clear();
    queue
        .generation
        .store(queue.generation().wrapping_add(1), Ordering::Release);
}

/// The reader's handle: the only side that fills request slots.
pub struct RequestProducer<'a, const N: usize, const S: usize> {
    queue: &'a DapQueue<N, S>,
}

impl<const N: usize, const S: usize> RequestProducer<'_, N, S> {
    /// Enqueue a received OUT packet; [`DAP_TRANSFER_ABORT`] is consumed, never enqueued or answered.
    pub fn submit(&mut self, packet: &[u8]) -> Result<Submit, QueueError> {
        submit(&self.queue.req, packet)
    }

    /// Receive directly into the next request slot; `None` means no packet was available.
    pub fn receive_with<F>(&mut self, receive: F) -> Result<Option<Submit>, QueueError>
    where
        F: FnOnce(&mut [u8]) -> Option<usize>,
    {
        receive_with(&self.queue.req, receive)
    }

    /// The free request slot for an endpoint to read into, or `None` when full.
    pub fn staging(&mut self) -> Option<&mut [u8]> {
        self.queue.req.staging().map(|slot| &mut slot[..])
    }

    /// Publish `len` bytes of the staged slot; see [`submit`](Self::submit) for the framing rules.
    pub fn commit(&mut self, len: usize) -> Result<Submit, QueueError> {
        commit(&self.queue.req, len)
    }

    /// Whether the request ring is at capacity.
    pub fn requests_full(&self) -> bool {
        self.queue.req.count() == N
    }

    /// The session generation; see [`DapQueue::generation`].
    pub fn generation(&self) -> u32 {
        self.queue.generation()
    }
}

/// The executor's handle: the only side that drains requests and fills response slots.
pub struct CommandExecutor<'a, const N: usize, const S: usize> {
    queue: &'a DapQueue<N, S>,
}

impl<const N: usize, const S: usize> CommandExecutor<'_, N, S> {
    /// Execute the oldest request directly from its slot into the next response slot.
    pub fn execute_next_with<F>(&mut self, execute: F) -> Result<Option<usize>, QueueError>
    where
        F: FnOnce(&[u8], &mut [u8]) -> usize,
    {
        execute_next_with(&self.queue.req, &self.queue.resp, execute)
    }

    #[cfg(test)]
    fn take_request(&mut self, out: &mut [u8]) -> Option<usize> {
        take_request(&self.queue.req, out)
    }

    /// Append an executed command's response in order; an empty `packet` enqueues nothing.
    pub fn post_response(&mut self, packet: &[u8]) -> Result<(), QueueError> {
        post_response(&self.queue.resp, packet)
    }

    /// Whether one queued request can execute without overflowing the response ring.
    pub fn command_ready(&self) -> bool {
        self.queue.req.count() != 0 && self.queue.resp.count() != N
    }

    /// Whether the request ring is at capacity.
    pub fn requests_full(&self) -> bool {
        self.queue.req.count() == N
    }

    /// Whether the response ring is at capacity (`N` responses buffered).
    pub fn responses_full(&self) -> bool {
        self.queue.resp.count() == N
    }

    /// Discard both rings and bump the generation.
    ///
    /// # Safety
    ///
    /// No other handle may touch either ring or hold a slot borrow until this returns.
    pub unsafe fn reset(&mut self) {
        reset(self.queue);
    }

    /// The session generation; see [`DapQueue::generation`].
    pub fn generation(&self) -> u32 {
        self.queue.generation()
    }
}

/// The writer's handle: the only side that drains response slots.
pub struct ResponseConsumer<'a, const N: usize, const S: usize> {
    queue: &'a DapQueue<N, S>,
}

impl<const N: usize, const S: usize> ResponseConsumer<'_, N, S> {
    /// Copy the oldest response into `out` without dequeuing it; returns its full length, even past `out`.
    pub fn peek_response(&self, out: &mut [u8]) -> Option<usize> {
        peek_response(&self.queue.resp, out)
    }

    /// Borrow the oldest pending response in place until the caller accepts or defers it.
    pub fn pending_response(&self) -> Option<&[u8]> {
        self.queue.resp.front()
    }

    /// Discard the oldest pending response after it has been sent.
    pub fn discard_response(&mut self) -> bool {
        self.queue.resp.pop()
    }

    /// [`discard_response`](Self::discard_response) scoped to the peek's `generation`.
    pub fn discard_response_for(&mut self, generation: u32) -> bool {
        generation == self.queue.generation() && self.queue.resp.pop()
    }

    /// Whether the response ring is at capacity (`N` responses buffered).
    pub fn responses_full(&self) -> bool {
        self.queue.resp.count() == N
    }

    /// The session generation; see [`DapQueue::generation`].
    pub fn generation(&self) -> u32 {
        self.queue.generation()
    }
}

/// A DAP packet size that a queue slot holds; built only by [`DapQueue::packet_size`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PacketSize(u16);

impl PacketSize {
    /// The size in bytes.
    pub const fn get(self) -> u16 {
        self.0
    }
}

/// A DAP bulk endpoint size that a queue slot holds; built only by [`DapQueue::endpoint_size`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EndpointSize(u16);

impl EndpointSize {
    /// The size in bytes.
    pub const fn get(self) -> u16 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `len`-byte packet starting at `first` and ascending.
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
            let n = q.peek_response(&mut out).expect("response pending");
            assert_eq!(n, 5);
            assert_eq!(out[0], id);
            assert!(q.discard_response());
        }
        assert_eq!(q.peek_response(&mut [0u8; 64]), None);
    }

    #[test]
    fn full_request_ring_backpressures() {
        let mut q: DapQueue<2, 64> = DapQueue::new();
        assert_eq!(q.submit(&pkt(1, 4)[..4]), Ok(Submit::Enqueued));
        assert_eq!(q.submit(&pkt(2, 4)[..4]), Ok(Submit::Enqueued));
        assert!(q.requests_full());
        assert_eq!(q.submit(&pkt(3, 4)[..4]), Err(QueueError::Full));
        let mut out = [0u8; 64];
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 1);
        assert_eq!(q.submit(&pkt(3, 4)[..4]), Ok(Submit::Enqueued));
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 2);
        assert_eq!(q.take_request(&mut out).unwrap(), 4);
        assert_eq!(out[0], 3);
    }

    /// Filling the ring to `N` and draining it back holds at any depth.
    fn depth_fills_and_drains<const N: usize>() {
        let mut q: DapQueue<N, 64> = DapQueue::new();
        // Offset clear of DAP_TRANSFER_ABORT (0x07), which submit() intercepts instead of enqueuing.
        for id in 0x10..0x10 + N as u8 {
            assert_eq!(q.submit(&pkt(id, 4)[..4]), Ok(Submit::Enqueued));
        }
        assert!(q.requests_full());
        assert_eq!(q.submit(&pkt(0xEE, 4)[..4]), Err(QueueError::Full));
        let mut out = [0u8; 64];
        for id in 0x10..0x10 + N as u8 {
            assert_eq!(q.take_request(&mut out).unwrap(), 4);
            assert_eq!(out[0], id);
        }
        assert_eq!(q.take_request(&mut out), None);
    }

    #[test]
    fn depth_four_fills_and_drains() {
        depth_fills_and_drains::<4>();
    }

    #[test]
    fn depth_eight_fills_and_drains() {
        depth_fills_and_drains::<8>();
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
    fn transfer_abort_is_consumed_without_a_slot_or_a_response() {
        let mut q: DapQueue<4, 64> = DapQueue::new();

        assert_eq!(q.submit(&pkt(0x05, 4)[..4]), Ok(Submit::Enqueued));
        assert_eq!(
            q.submit(&[DAP_TRANSFER_ABORT]),
            Ok(Submit::AbortIntercepted)
        );
        assert_eq!(q.submit(&pkt(0x06, 4)[..4]), Ok(Submit::Enqueued));
        assert_eq!(
            q.requests_pending(),
            2,
            "abort must not occupy a request slot"
        );

        // The abort never reaches execution, so it can never produce a response packet.
        for id in [0x05u8, 0x06] {
            assert_eq!(
                q.execute_next_with(|request, response| {
                    assert_eq!(request[0], id);
                    response[0] = id + 0x40;
                    1
                }),
                Ok(Some(1))
            );
        }
        assert_eq!(q.execute_next_with(|_, _| 0), Ok(None));

        for id in [0x45u8, 0x46] {
            assert_eq!(q.pending_response(), Some(&[id][..]));
            assert!(q.discard_response());
        }
        assert_eq!(q.pending_response(), None);
    }

    #[test]
    fn a_trailing_abort_never_stalls_a_full_request_ring() {
        // The abort takes no slot, so it is accepted even when every request slot is occupied.
        let mut q: DapQueue<2, 64> = DapQueue::new();
        q.submit(&pkt(0x11, 4)[..4]).unwrap();
        q.submit(&pkt(0x12, 4)[..4]).unwrap();
        assert!(q.requests_full());
        assert_eq!(
            q.submit(&[DAP_TRANSFER_ABORT]),
            Ok(Submit::AbortIntercepted)
        );
        assert_eq!(q.requests_pending(), 2);
        let mut out = [0u8; 64];
        for id in [0x11u8, 0x12] {
            assert_eq!(q.take_request(&mut out).unwrap(), 4);
            assert_eq!(out[0], id);
        }
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
        while q.peek_response(&mut []).is_some() {
            assert!(q.discard_response());
        }
        assert_eq!(q.requests_pending(), 0);
        assert_eq!(q.responses_pending(), 0);

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
        assert_eq!(q.peek_response(&mut [0u8; 64]), None);
    }

    #[test]
    fn peek_response_is_non_consuming_and_discard_commits() {
        // A refused IN write must leave the peeked response queued and in order.
        let mut q: DapQueue<3, 64> = DapQueue::new();
        for id in [0xB0u8, 0xB1, 0xB2] {
            q.post_response(&pkt(id, 6)[..6]).unwrap();
        }
        assert!(q.responses_full());

        let mut out = [0u8; 64];
        for _ in 0..3 {
            assert_eq!(q.peek_response(&mut out), Some(6));
            assert_eq!(out[0], 0xB0);
            assert_eq!(q.responses_pending(), 3);
        }

        assert!(q.discard_response());
        assert_eq!(q.responses_pending(), 2);
        assert!(!q.responses_full());
        assert_eq!(q.peek_response(&mut out), Some(6));
        assert_eq!(out[0], 0xB1);

        assert!(q.discard_response());
        assert!(q.discard_response());
        assert_eq!(q.peek_response(&mut out), None);
        assert!(!q.discard_response());
        assert_eq!(q.responses_pending(), 0);
    }

    #[test]
    fn reset_drains_rings_and_bumps_generation() {
        let mut q: DapQueue<3, 64> = DapQueue::new();
        assert_eq!(q.generation(), 0);

        q.submit(&pkt(0x50, 8)[..8]).unwrap();
        q.submit(&pkt(0x51, 8)[..8]).unwrap();
        q.post_response(&pkt(0x52, 8)[..8]).unwrap();

        q.reset();

        assert_eq!(q.requests_pending(), 0, "requests drained");
        assert_eq!(q.responses_pending(), 0, "responses drained");
        assert_eq!(q.generation(), 1, "generation bumped once per reset");
        assert_eq!(q.take_request(&mut [0u8; 64]), None);
        assert_eq!(q.peek_response(&mut [0u8; 64]), None);

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

        q.reset();

        assert_ne!(q.generation(), gen, "reset changed the generation");
        assert_eq!(n, 6);
        assert_eq!(q.responses_pending(), 0);
    }

    #[test]
    fn epoch_scoped_retire_commits_within_one_session() {
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
        assert_eq!(q.requests_pending(), 0);
    }

    #[test]
    fn in_place_receive_preserves_framing_interception_and_backpressure() {
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
        assert_eq!(
            q.requests_pending(),
            1,
            "abort must not occupy a request slot"
        );
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

    fn budgeted_pass<const N: usize>(
        q: &mut DapQueue<N, 8>,
        budget: usize,
        sent: &mut [u8],
        sent_count: &mut usize,
    ) -> bool {
        let mut executed = 0;
        while executed < budget && q.command_ready() {
            q.execute_next_with(|request, response| {
                response[0] = request[0] + 0x40;
                1
            })
            .unwrap();
            executed += 1;
        }
        while let Some(response) = q.pending_response() {
            sent[*sent_count] = response[0];
            *sent_count += 1;
            q.discard_response();
        }
        executed == budget && q.command_ready()
    }

    #[test]
    fn one_and_two_command_budgets_make_ordered_progress() {
        for budget in [1, 2] {
            let mut q: DapQueue<4, 8> = DapQueue::new();
            for command in 0x10..0x14 {
                q.submit(&[command]).unwrap();
            }
            let mut sent = [0; 4];
            let mut sent_count = 0;
            let mut passes = 0;
            loop {
                passes += 1;
                if !budgeted_pass(&mut q, budget, &mut sent, &mut sent_count) {
                    break;
                }
            }
            assert_eq!(passes, 4 / budget);
            assert_eq!(sent, [0x50, 0x51, 0x52, 0x53]);
            assert_eq!(sent_count, 4);
            assert_eq!(q.requests_pending(), 0);
            assert_eq!(q.responses_pending(), 0);
        }
    }

    #[test]
    fn a_stalled_response_writer_is_not_self_pended() {
        let mut q: DapQueue<2, 8> = DapQueue::new();
        q.submit(&[0x10]).unwrap();
        q.post_response(&[0x90]).unwrap();
        q.post_response(&[0x91]).unwrap();
        assert!(!q.command_ready(), "a full response ring blocks execution");

        let mut sent = [0; 3];
        let mut sent_count = 0;
        assert!(!budgeted_pass(&mut q, 1, &mut sent, &mut sent_count));
        assert_eq!(q.requests_pending(), 1);
        assert_eq!(&sent[..sent_count], [0x90, 0x91]);
        assert!(q.command_ready(), "retiring responses restores progress");
        assert!(!budgeted_pass(&mut q, 1, &mut sent, &mut sent_count));
        assert_eq!(sent, [0x90, 0x91, 0x50]);
    }

    #[test]
    fn a_split_writer_retires_only_its_own_generation() {
        let mut q: DapQueue<3, 64> = DapQueue::new();
        q.post_response(&pkt(0xD0, 6)[..6]).unwrap();
        let stale = q.generation();
        q.reset();
        q.post_response(&pkt(0xD1, 4)[..4]).unwrap();
        let (_, _, mut writer) = q.split();
        assert!(!writer.discard_response_for(stale));
        let mut out = [0u8; 64];
        assert_eq!(writer.peek_response(&mut out), Some(4));
        assert_eq!(out[0], 0xD1);
    }

    #[test]
    fn split_handles_run_a_command_end_to_end() {
        let mut q: DapQueue<4, 64> = DapQueue::new();
        let (mut reader, mut executor, mut writer) = q.split();

        assert_eq!(reader.submit(&pkt(0x11, 4)[..4]), Ok(Submit::Enqueued));
        assert_eq!(reader.submit(&pkt(0x12, 4)[..4]), Ok(Submit::Enqueued));
        // The abort takes no slot on the producer side either.
        assert_eq!(
            reader.submit(&[DAP_TRANSFER_ABORT]),
            Ok(Submit::AbortIntercepted)
        );
        assert!(executor.command_ready());

        for id in [0x11u8, 0x12] {
            assert_eq!(
                executor.execute_next_with(|request, response| {
                    assert_eq!(request[0], id);
                    response[0] = id + 0x40;
                    1
                }),
                Ok(Some(1))
            );
        }
        assert_eq!(executor.execute_next_with(|_, _| 0), Ok(None));

        let mut out = [0u8; 64];
        for id in [0x51u8, 0x52] {
            let generation = writer.generation();
            assert_eq!(writer.peek_response(&mut out), Some(1));
            assert_eq!(out[0], id);
            assert!(writer.discard_response_for(generation));
        }
        assert_eq!(writer.pending_response(), None);
    }

    #[test]
    fn split_handles_backpressure_and_reset_across_a_session() {
        let mut q: DapQueue<2, 64> = DapQueue::new();
        let (mut reader, mut executor, mut writer) = q.split();

        reader.submit(&pkt(0x21, 4)[..4]).unwrap();
        reader.submit(&pkt(0x22, 4)[..4]).unwrap();
        assert!(reader.requests_full());
        assert_eq!(reader.submit(&pkt(0x23, 4)[..4]), Err(QueueError::Full));

        executor
            .execute_next_with(|_, response| {
                response[0] = 0x61;
                1
            })
            .unwrap();
        executor
            .execute_next_with(|_, response| {
                response[0] = 0x62;
                1
            })
            .unwrap();
        assert!(executor.responses_full());
        let stale = writer.generation();
        assert_eq!(writer.peek_response(&mut [0u8; 64]), Some(1));

        // SAFETY: neither other handle is running or borrowing a slot.
        unsafe { executor.reset() };
        assert!(!reader.requests_full());
        assert!(!executor.command_ready());
        assert!(
            !writer.discard_response_for(stale),
            "the dead session retires nothing"
        );
        assert_eq!(writer.pending_response(), None);

        reader.submit(&pkt(0x24, 4)[..4]).unwrap();
        assert_eq!(
            executor.execute_next_with(|_, response| {
                response[0] = 0x64;
                1
            }),
            Ok(Some(1))
        );
        assert_eq!(writer.pending_response(), Some(&[0x64][..]));
        assert!(writer.discard_response());
    }

    #[test]
    fn split_receive_with_fills_the_slot_in_place() {
        let mut q: DapQueue<2, 8> = DapQueue::new();
        let (mut reader, mut executor, _writer) = q.split();
        assert_eq!(
            reader.receive_with(|slot| {
                slot[..2].copy_from_slice(&[0x31, 5]);
                Some(2)
            }),
            Ok(Some(Submit::Enqueued))
        );
        assert_eq!(reader.receive_with(|_| None), Ok(None));
        let mut out = [0u8; 8];
        assert_eq!(executor.take_request(&mut out), Some(2));
        assert_eq!(&out[..2], &[0x31, 5]);
        assert_eq!(executor.take_request(&mut out), None);
    }
}
