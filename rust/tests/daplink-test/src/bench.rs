//! Bench-side defmt capture: a second probe streams the probe under test's decoded log to a file.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use probe_rs::probe::list::Lister;
use probe_rs::probe::DebugProbeSelector;
use probe_rs::{Permissions, Session};

use crate::rtt::attach_up_only;
use crate::util::chain;

/// RTT attach deadline: covers the control block's republish window, not a boot.
const ATTACH_DEADLINE: Duration = Duration::from_secs(5);
/// Poll interval of the capture thread between empty RTT reads.
const POLL: Duration = Duration::from_millis(10);
/// After a stop request, how long the capture thread keeps draining buffered RTT data.
const DRAIN_AFTER_STOP: Duration = Duration::from_millis(500);

/// A running capture: the transport-probe thread plus its stop signal.
pub struct DefmtCapture {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    log_path: PathBuf,
}

/// Default defmt log destination, derived from the probe ELF's file stem.
fn default_log_path(elf_path: &Path) -> PathBuf {
    let stem = elf_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("probe");
    PathBuf::from(format!("{stem}.defmt.log"))
}

/// Open `via` as a second probe-rs session and stream `elf_path`'s decoded defmt frames to a log
/// file from a background thread; `Ok(None)` when the ELF carries no defmt table.
pub fn start(via: &str, elf_path: &Path, chip: &str) -> Result<Option<DefmtCapture>, String> {
    let elf = std::fs::read(elf_path)
        .map_err(|e| format!("cannot read probe ELF `{}`: {e}", elf_path.display()))?;
    let Some(table) = defmt_decoder::Table::parse(&elf)
        .map_err(|e| format!("parsing defmt table from `{}`: {e}", elf_path.display()))?
    else {
        println!(
            "bench: `{}` has no defmt table (a default build carries no logging); skipping the \
             defmt capture. To capture, rebuild with the defmt feature, e.g. `cargo xtask build \
             <firmware> --no-default-features --features defmt`",
            elf_path.display()
        );
        return Ok(None);
    };

    let selector: DebugProbeSelector = via
        .parse()
        .map_err(|e| format!("invalid --via `{via}`: {e}"))?;
    let probe = Lister::new()
        .open(selector)
        .map_err(|e| format!("opening transport probe {via}: {}", chain(e)))?;
    let session = probe
        .attach(chip, Permissions::default())
        .map_err(|e| format!("attaching to {chip} via {via}: {}", chain(e)))?;

    let log_path = default_log_path(elf_path);
    let file = std::fs::File::create(&log_path)
        .map_err(|e| format!("create {}: {e}", log_path.display()))?;

    // The capture thread re-derives the RTT control-block address from the path itself.
    println!("bench: defmt -> {}", log_path.display());
    let stop = Arc::new(AtomicBool::new(false));
    let thread = {
        let stop = stop.clone();
        let elf_path = elf_path.to_path_buf();
        std::thread::spawn(move || capture(session, elf_path, table, file, stop))
    };
    Ok(Some(DefmtCapture {
        stop,
        thread: Some(thread),
        log_path,
    }))
}

impl DefmtCapture {
    /// Stop the capture thread, drain and flush the log, and drop the transport session.
    pub fn finish(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        println!("bench: defmt log written to {}", self.log_path.display());
    }
}

/// Capture-thread body: stream up-channel 0 through the defmt decoder into `file`, flushing per
/// read so an interrupted run still leaves a usable log, until `stop` is set.
fn capture(
    mut session: Session,
    elf_path: PathBuf,
    table: defmt_decoder::Table,
    mut file: std::fs::File,
    stop: Arc<AtomicBool>,
) {
    let mut core = match session.core(0) {
        Ok(c) => c,
        Err(e) => {
            eprintln!(
                "bench: defmt capture aborted (core open failed: {})",
                chain(e)
            );
            return;
        }
    };
    let elf_path_str = elf_path.to_string_lossy().into_owned();
    let mut rtt = match attach_up_only(&mut core, Some(elf_path_str.as_str()), ATTACH_DEADLINE) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("bench: defmt capture aborted ({e})");
            return;
        }
    };
    let mut decoder = table.new_stream_decoder();
    let mut buf = [0u8; 1024];
    let mut stop_seen: Option<Instant> = None;
    loop {
        let n = match rtt.up_channel(0) {
            Some(up) => up.read(&mut core, &mut buf).unwrap_or(0),
            None => 0,
        };
        if n > 0 {
            decoder.received(&buf[..n]);
            loop {
                match decoder.decode() {
                    Ok(frame) => {
                        let _ = writeln!(file, "{}", frame.display(false));
                    }
                    Err(defmt_decoder::DecodeError::UnexpectedEof) => break,
                    Err(defmt_decoder::DecodeError::Malformed)
                        if table.encoding().can_recover() =>
                    {
                        // Recoverable framing: skip the malformed frame, keep decoding.
                    }
                    Err(defmt_decoder::DecodeError::Malformed) => {
                        let _ =
                            writeln!(file, "<malformed defmt frame; stream may have lost data>");
                        break;
                    }
                }
            }
            let _ = file.flush();
        }
        if stop.load(Ordering::Relaxed) {
            let since = *stop_seen.get_or_insert_with(Instant::now);
            // Drain whatever the firmware already buffered, then stop.
            if n == 0 || since.elapsed() > DRAIN_AFTER_STOP {
                break;
            }
        }
        if n == 0 {
            std::thread::sleep(POLL);
        }
    }
    let _ = file.flush();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_log_path_derives_from_the_elf_stem() {
        assert_eq!(
            default_log_path(Path::new("target/defmt/rp2040-pico-embassy")),
            Path::new("rp2040-pico-embassy.defmt.log")
        );
    }

    #[test]
    fn default_log_path_falls_back_without_a_readable_stem() {
        assert_eq!(
            default_log_path(Path::new("/")),
            Path::new("probe.defmt.log")
        );
    }
}
