//! `DAP_TransferBlock` vectors on scripted SWD answers: Arm's counts and posting, dap-rs's `0x0F`.

/// What the target answers one SWD transfer.
#[derive(Clone, Copy, Debug)]
pub enum Reply {
    /// OK, with this read data.
    Word(u32),
    /// WAIT.
    Wait,
    /// FAULT.
    Fault,
    /// No ACK.
    NoAck,
    /// OK, read data with bad parity.
    Parity,
}

/// One SWD transfer the block must make, and the target's answer.
#[derive(Clone, Copy, Debug)]
pub struct Step {
    /// The SWD request header.
    pub request: u8,
    /// The word written, for a write.
    pub write: Option<u32>,
    /// The answer.
    pub reply: Reply,
}

fn read(request: u8, reply: Reply) -> Step {
    Step {
        request,
        write: None,
        reply,
    }
}

/// Run every vector through `run(report, wait_retries, script)`, which answers the report.
pub fn check_blocks(mut run: impl FnMut(&[u8], u16, &[Step]) -> Vec<u8>) {
    let ap = [0x06, 0, 3, 0, 0x0f];
    let dp = [0x06, 0, 3, 0, 0x06];
    let ap_steps = [
        read(0x9f, Reply::Word(0xdeadbeef)),
        read(0x9f, Reply::Word(0x11223344)),
        read(0x9f, Reply::Word(0x55667788)),
        read(0xbd, Reply::Word(0x99aabbcc)),
    ];
    let dp_steps = [
        read(0x8d, Reply::Word(0x11223344)),
        read(0x8d, Reply::Word(0x55667788)),
        read(0x8d, Reply::Word(0x99aabbcc)),
    ];
    let complete = [
        6, 3, 0, 1, 0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0xcc, 0xbb, 0xaa, 0x99,
    ];
    assert_eq!(run(&ap, 5, &ap_steps), complete);
    assert_eq!(run(&dp, 5, &dp_steps), complete);
    for high in 0..16 {
        let flagged = [6, 0xff, 3, 0, 0x0f | high << 4];
        assert_eq!(run(&flagged, 5, &ap_steps), complete);
    }

    let partials: [&[u8]; 4] = [
        &[6, 0, 0, 0],
        &[6, 0, 0, 0],
        &[6, 1, 0, 0, 0x44, 0x33, 0x22, 0x11],
        &[6, 2, 0, 0, 0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55],
    ];
    for (reply, status) in [
        (Reply::Wait, 2),
        (Reply::Fault, 4),
        (Reply::NoAck, 7),
        (Reply::Parity, 15),
    ] {
        for at in 0..4 {
            let mut script = ap_steps[..at].to_vec();
            let mut failure = ap_steps[at];
            failure.reply = reply;
            script.extend(core::iter::repeat_n(
                failure,
                if status == 2 { 4 } else { 1 },
            ));
            let mut expected = partials[at].to_vec();
            expected[3] = status;
            assert_eq!(
                run(&ap, 3, &script),
                expected,
                "AP transaction {at}: {reply:?}"
            );
        }
        for at in 0..3 {
            let mut script = dp_steps[..at].to_vec();
            let mut failure = dp_steps[at];
            failure.reply = reply;
            script.extend(core::iter::repeat_n(
                failure,
                if status == 2 { 4 } else { 1 },
            ));
            let mut expected = partials[at + 1].to_vec();
            expected[3] = status;
            assert_eq!(
                run(&dp, 3, &script),
                expected,
                "DP transaction {at}: {reply:?}"
            );
        }
    }

    let write = [
        6, 0, 3, 0, 8, 0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0xcc, 0xbb, 0xaa, 0x99,
    ];
    let writes = [0x11223344, 0x55667788, 0x99aabbcc].map(|word| Step {
        request: 0xb1,
        write: Some(word),
        reply: Reply::Word(0),
    });
    assert_eq!(run(&write, 5, &writes), [6, 3, 0, 1]);
    for (reply, status) in [(Reply::Wait, 2), (Reply::Fault, 4), (Reply::NoAck, 7)] {
        for (at, field) in [(0, 0), (1, 1), (2, 2)] {
            let mut script = writes[..at].to_vec();
            let mut failure = writes[at];
            failure.reply = reply;
            script.extend(core::iter::repeat_n(
                failure,
                if status == 2 { 4 } else { 1 },
            ));
            assert_eq!(run(&write, 3, &script), [6, field, 0, status]);
        }
    }
    for (report, first) in [
        (&ap[..], ap_steps[0]),
        (&dp, dp_steps[0]),
        (&write, writes[0]),
    ] {
        let wait = Step {
            reply: Reply::Wait,
            ..first
        };
        assert_eq!(run(report, 0, &[wait]), [6, 0, 0, 2]);
    }
    let one = [6, 0, 1, 0, 6];
    assert_eq!(
        run(&one, 0, &[read(0x8d, Reply::Word(0x11223344))]),
        [6, 1, 0, 1, 0x44, 0x33, 0x22, 0x11]
    );
    let mut script = vec![read(0x8d, Reply::Wait); 65535];
    script.push(read(0x8d, Reply::Word(0x11223344)));
    assert_eq!(
        run(&one, 65535, &script),
        [6, 1, 0, 1, 0x44, 0x33, 0x22, 0x11]
    );
    script[65535].reply = Reply::Wait;
    assert_eq!(run(&one, 65535, &script), [6, 0, 0, 2]);
    assert_eq!(
        run(&[6, 0, 255, 255, 6], 0, &[read(0x8d, Reply::Wait)]),
        [6, 0, 0, 2]
    );
    let script = vec![read(0x8d, Reply::Word(0)); 256];
    let response = run(&[6, 0, 0, 1, 6], 5, &script);
    assert_eq!(&response[..4], &[6, 0, 1, 1]);
    assert_eq!(response.len(), 1028);
    assert!(response[4..].iter().all(|b| *b == 0));
}
