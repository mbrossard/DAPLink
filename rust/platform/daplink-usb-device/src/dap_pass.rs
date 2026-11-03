//! The polled CMSIS-DAP pass: one receive, a bounded execute batch, and a drain.

use daplink_core::DapQueue;

#[cfg(test)]
mod tests;

/// The endpoint side of a pass: what the DAP bulk pair has to offer.
pub trait DapTransport {
    /// Whether the host session ended since the last call.
    fn session_ended(&mut self) -> bool;
    /// Read a landed command packet into `slot`, or `None` when none is pending.
    fn read_command(&mut self, slot: &mut [u8]) -> Option<usize>;
    /// Write one response packet, returning `false` when the endpoint refused it.
    fn write_response(&mut self, resp: &[u8]) -> bool;
}

/// The firmware side of a pass: command execution, and the instrumentation points around it.
pub trait DapPassHooks {
    /// Execute one request into the response slot, returning the response length.
    fn execute(&mut self, req: &[u8], resp: &mut [u8]) -> usize;
    /// Retire whatever the dead session owned; the queue is already reset.
    fn session_reset(&mut self);
    /// The OUT endpoint was skipped because the request ring is full.
    fn requests_full(&mut self) {}
    /// A command packet landed in the request ring.
    fn received(&mut self) {}
    /// The execute batch is about to start.
    fn batch_start(&mut self) {}
    /// The execute batch is over, whatever ended it.
    fn batch_end(&mut self) {}
    /// One command executed, producing `len` response bytes.
    fn executed(&mut self, len: usize) {
        let _ = len;
    }
    /// The IN endpoint accepted a response of `len` bytes; host receipt is not established.
    fn sent(&mut self, len: usize) {
        let _ = len;
    }
    /// The endpoint refused a response of `len` bytes, ending the drain.
    fn refused(&mut self, len: usize) {
        let _ = len;
    }
}

/// Run one pass over `queue`; returns whether the batch stopped on `budget` with work queued,
/// which the caller answers by re-pending its own task.
pub fn dap_pass<const N: usize, const S: usize, T: DapTransport, H: DapPassHooks>(
    queue: &mut DapQueue<N, S>,
    usb: &mut T,
    hooks: &mut H,
    polled: bool,
    budget: usize,
) -> bool {
    // USB reset may make poll return false; retire the session before executing queued work.
    if usb.session_ended() {
        queue.reset();
        hooks.session_reset();
    }

    if polled {
        // Skip the OUT endpoint while the ring is full, so it NAKs the host into pacing itself.
        if queue.requests_full() {
            hooks.requests_full();
        } else if queue
            .receive_with(|slot| usb.read_command(slot))
            .expect("a checked-non-full request slot accepts one endpoint packet")
            .is_some()
        {
            hooks.received();
        }
    }

    // Run a bounded command batch even when this pass was software-pended.
    hooks.batch_start();
    let mut executed = 0;
    while executed < budget {
        if queue.responses_full() {
            break;
        }
        match queue.execute_next_with(|request, response| hooks.execute(request, response)) {
            Ok(Some(len)) => {
                hooks.executed(len);
                executed += 1;
            }
            Ok(None) => break,
            Err(_) => {
                debug_assert!(false, "reserved request and response slots fit one packet");
                break;
            }
        }
    }
    hooks.batch_end();

    // Send whatever is ready, stopping at the first busy-endpoint refusal.
    while let Some(response) = queue.pending_response() {
        if !usb.write_response(response) {
            hooks.refused(response.len());
            break;
        }
        hooks.sent(response.len());
        queue.discard_response();
    }
    executed == budget && queue.command_ready()
}
