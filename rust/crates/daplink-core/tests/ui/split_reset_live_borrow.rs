#![allow(dead_code)]

#[path = "../../src/dap_queue.rs"]
mod dap_queue;

fn reset_with_a_live_response() {
    let mut queue = dap_queue::DapQueue::<1, 8>::new();
    let (_reader, mut executor, writer) = queue.split();
    executor.post_response(&[1]).unwrap();
    let borrowed = writer.pending_response().unwrap();
    executor.reset();
    executor.post_response(&[2]).unwrap();
    assert_eq!(borrowed[0], 1);
}
