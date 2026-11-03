use daplink_flash::{Error, FlashInterface, FlashManager, StepPoll, SyscallRun};
use std::{cell::RefCell, rc::Rc};

type Log = Rc<RefCell<Vec<&'static str>>>;

struct RecordingFlash {
    log: Log,
    fail: Option<&'static str>,
    pending: bool,
    lose_geometry_after_erase: bool,
}

impl RecordingFlash {
    fn event(&mut self, event: &'static str) -> Result<(), Error> {
        self.log.borrow_mut().push(event);
        if self.fail == Some(event) {
            Err(Error::InvalidState)
        } else {
            Ok(())
        }
    }
}

impl FlashInterface for RecordingFlash {
    fn init(&mut self) -> Result<(), Error> {
        self.event("init")
    }
    fn uninit(&mut self) -> Result<(), Error> {
        self.event("uninit")
    }
    fn set_post_flash_reset(&mut self, reset: bool) {
        self.log
            .borrow_mut()
            .push(if reset { "reset:on" } else { "reset:off" });
    }
    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        assert_eq!((addr, data), (0x1000, &[1, 2, 0xff, 0xff][..]));
        self.event("program:blocking")
    }
    fn verify(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        assert_eq!((addr, data), (0x1000, &[1, 2, 0xff, 0xff][..]));
        self.event("verify:blocking")
    }
    fn erase_sector(&mut self, addr: u32) -> Result<(), Error> {
        assert_eq!(addr, 0x1000);
        self.event("erase:blocking")
    }
    fn erase_chip(&mut self) -> Result<(), Error> {
        self.event("chip:blocking")
    }
    fn program_page_min_size(&self, _: u32) -> u32 {
        4
    }
    fn erase_sector_size(&self, _: u32) -> Option<u32> {
        if self.lose_geometry_after_erase {
            self.log.borrow_mut().push("geometry:size");
            if self.log.borrow().contains(&"erase:blocking") {
                return None;
            }
        }
        Some(16)
    }
    fn erase_sector_base(&self, _: u32) -> Option<u32> {
        Some(0x1000)
    }
    fn supports_chip_erase(&self) -> bool {
        true
    }
    fn supports_sector_erase(&self) -> bool {
        true
    }
    fn erase_sector_begin(&mut self, addr: u32) -> Result<SyscallRun, Error> {
        assert_eq!(addr, 0x1000);
        self.event("erase:start")?;
        self.pending = true;
        Ok(SyscallRun::done(0))
    }
    fn program_page_begin(&mut self, addr: u32, data: &[u8]) -> Result<SyscallRun, Error> {
        assert_eq!((addr, data), (0x1000, &[1, 2, 0xff, 0xff][..]));
        self.event("program:start")?;
        self.pending = true;
        Ok(SyscallRun::done(0))
    }
    fn verify_begin(&mut self, _: u32, _: &[u8]) -> Result<SyscallRun, Error> {
        self.event("verify:start")?;
        self.pending = true;
        Ok(SyscallRun::done(0))
    }
    fn advance_syscall(&mut self, _: &mut SyscallRun, _: u64) -> StepPoll {
        if self.pending {
            self.pending = false;
            self.log.borrow_mut().push("poll:pending");
            StepPoll::Pending { wake_at_us: 17 }
        } else if self.event("poll:done").is_err() {
            StepPoll::Err(Error::InvalidState)
        } else {
            StepPoll::Done
        }
    }
    fn finish_syscall(&mut self, _: SyscallRun) -> Result<(), Error> {
        self.event("status")
    }
    fn delay_us(&mut self, _: u32) {
        panic!("manager blocking calls use device blocking methods")
    }
}

#[test]
fn blocking_dispatch_and_cleanup_are_literal() {
    for (fail, expected) in [
        (
            None,
            &[
                "init",
                "reset:off",
                "erase:blocking",
                "program:blocking",
                "verify:blocking",
                "uninit",
            ][..],
        ),
        (
            Some("erase:blocking"),
            &["init", "reset:off", "erase:blocking", "uninit"][..],
        ),
        (
            Some("program:blocking"),
            &[
                "init",
                "reset:off",
                "erase:blocking",
                "program:blocking",
                "uninit",
            ][..],
        ),
        (
            Some("verify:blocking"),
            &[
                "init",
                "reset:off",
                "erase:blocking",
                "program:blocking",
                "verify:blocking",
                "uninit",
            ][..],
        ),
        (
            Some("uninit"),
            &[
                "init",
                "reset:off",
                "erase:blocking",
                "program:blocking",
                "verify:blocking",
                "uninit",
            ][..],
        ),
    ] {
        let log = Log::default();
        let flash = RecordingFlash {
            log: log.clone(),
            fail,
            pending: false,
            lose_geometry_after_erase: false,
        };
        let mut buf = [0; 4];
        let mut manager = FlashManager::new(flash, &mut buf).unwrap();
        manager.set_verify(true);
        manager.write(0x1000, &[1, 2]).unwrap();
        let (result, _) = manager.finish(false);
        assert_eq!(result.is_err(), fail.is_some());
        assert_eq!(&*log.borrow(), expected);
        println!("blocking {fail:?}: {:?}", log.borrow());
    }
}

#[test]
fn resumable_dispatch_pending_status_and_abort_are_literal() {
    let log = Log::default();
    let flash = RecordingFlash {
        log: log.clone(),
        fail: None,
        pending: false,
        lose_geometry_after_erase: false,
    };
    let mut buf = [0; 4];
    let mut manager = FlashManager::new(flash, &mut buf).unwrap();
    manager.set_verify(true);
    manager.prepare().unwrap();
    assert_eq!(manager.stage(0x1000, &[1, 2]), Ok(2));
    let mut run = manager.flush_begin().unwrap();
    assert_eq!(
        manager.advance_flush(&mut run, 0),
        StepPoll::Pending { wake_at_us: 17 }
    );
    assert_eq!(
        manager.advance_flush(&mut run, 17),
        StepPoll::Pending { wake_at_us: 17 }
    );
    assert_eq!(
        manager.advance_flush(&mut run, 17),
        StepPoll::Pending { wake_at_us: 17 }
    );
    assert_eq!(
        manager.advance_flush(&mut run, 17),
        StepPoll::Pending { wake_at_us: 17 }
    );
    assert_eq!(
        manager.advance_flush(&mut run, 17),
        StepPoll::Pending { wake_at_us: 17 }
    );
    assert_eq!(manager.advance_flush(&mut run, 17), StepPoll::Done);
    manager.abort();
    assert_eq!(
        &*log.borrow(),
        &[
            "init",
            "erase:start",
            "poll:pending",
            "poll:done",
            "status",
            "program:start",
            "poll:pending",
            "poll:done",
            "status",
            "verify:start",
            "poll:pending",
            "poll:done",
            "status",
            "uninit",
        ]
    );
    println!("resumable: {:?}", log.borrow());
}

#[test]
fn resumable_start_poll_and_status_errors_stop_before_program() {
    for (fail, expected) in [
        ("erase:start", &["init", "erase:start", "uninit"][..]),
        (
            "poll:done",
            &["init", "erase:start", "poll:pending", "poll:done", "uninit"][..],
        ),
        (
            "status",
            &[
                "init",
                "erase:start",
                "poll:pending",
                "poll:done",
                "status",
                "uninit",
            ][..],
        ),
    ] {
        let log = Log::default();
        let flash = RecordingFlash {
            log: log.clone(),
            fail: Some(fail),
            pending: false,
            lose_geometry_after_erase: false,
        };
        let mut buf = [0; 4];
        let mut manager = FlashManager::new(flash, &mut buf).unwrap();
        manager.prepare().unwrap();
        manager.stage(0x1000, &[1, 2]).unwrap();
        match manager.flush_begin() {
            Err(error) => assert_eq!(error, Error::InvalidState),
            Ok(mut run) => {
                assert_eq!(
                    manager.advance_flush(&mut run, 0),
                    StepPoll::Pending { wake_at_us: 17 }
                );
                assert_eq!(
                    manager.advance_flush(&mut run, 17),
                    StepPoll::Err(Error::InvalidState)
                );
            }
        }
        manager.abort();
        assert_eq!(&*log.borrow(), expected);
        println!("resumable {fail}: {:?}", log.borrow());
    }
}

#[test]
fn blocking_flush_uses_geometry_captured_before_erase() {
    let log = Log::default();
    let flash = RecordingFlash {
        log: log.clone(),
        fail: None,
        pending: false,
        lose_geometry_after_erase: true,
    };
    let mut buf = [0; 4];
    let mut manager = FlashManager::new(flash, &mut buf).unwrap();
    manager.write(0x1000, &[1, 2]).unwrap();
    let (result, _) = manager.finish(false);
    println!(
        "mutable-geometry: result={result:?}, events={:?}",
        log.borrow()
    );
    assert_eq!(result, Ok(()));
    assert_eq!(
        &*log.borrow(),
        &[
            "init",
            "reset:off",
            "geometry:size",
            "erase:blocking",
            "program:blocking",
            "uninit",
        ]
    );
}
