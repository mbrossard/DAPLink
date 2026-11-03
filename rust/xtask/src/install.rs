//! Building and installing one registry row: build, UF2, flash, and the PUT's return to the bus.

use std::ffi::{OsStr, OsString};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

use crate::registry::{Bootloader, Firmware};
use crate::{bootsel, elf, reattach, revision, root, usb};

pub(crate) fn build(fw: &Firmware) -> bool {
    println!("building {} for {}", fw.name, fw.target);
    let mut cmd = Command::new("cargo");
    cmd.args(["build", "--release", "--locked", "--target", fw.target]);
    // Absolute: the build runs in the crate directory, and output belongs under `rust/target/`.
    cmd.arg("--target-dir").arg(root().join(fw.target_dir()));
    if fw.no_default_features {
        cmd.arg("--no-default-features");
    }
    if !fw.features.is_empty() {
        cmd.args(["--features", &fw.features.join(",")]);
    }
    revision::stamp(&mut cmd, &root());
    revision::stamp_row(&mut cmd, fw.name);
    let ok = cmd
        .current_dir(root().join(fw.path))
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    ok && emit_uf2(fw)
}

/// Write the uf2 image beside the built ELF for rows that name a uf2 family.
fn emit_uf2(fw: &Firmware) -> bool {
    let Some(family) = fw.uf2_family else {
        return true;
    };
    let elf_path = root().join(fw.elf());
    let elf_bytes = match std::fs::read(&elf_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{}: could not read {}: {e}", fw.name, elf_path.display());
            return false;
        }
    };
    let (base, image) = match elf::flash_image(&elf_bytes) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("{}: could not extract flash image: {e}", fw.name);
            return false;
        }
    };
    let uf2 = daplink_decoder::encode::bin_to_uf2(&image, base, family);
    let uf2_path = PathBuf::from(format!("{}.uf2", elf_path.display()));
    if let Err(e) = std::fs::write(&uf2_path, &uf2) {
        eprintln!("{}: could not write {}: {e}", fw.name, uf2_path.display());
        return false;
    }
    println!(
        "wrote {} ({} bytes, {} blocks)",
        uf2_path.display(),
        uf2.len(),
        uf2.len() / 512
    );
    true
}

/// `probe-rs`, or the binary `DAPLINK_PROBE_RS` names.
pub(crate) fn probe_rs_command() -> Command {
    Command::new(probe_rs_program(std::env::var_os("DAPLINK_PROBE_RS")))
}

fn probe_rs_program(chosen: Option<OsString>) -> OsString {
    chosen
        .filter(|path| !path.is_empty())
        .unwrap_or_else(|| "probe-rs".into())
}

fn probe_rs(args: &[impl AsRef<OsStr>]) -> bool {
    probe_rs_command()
        .args(args)
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

const ZERO_CHUNK_WORDS: u32 = 64;

/// Zero RAM through `probe-rs write` calls, for targets whose `download` refuses a RAM base.
fn zero_via_write_loop(fw: &Firmware, probe: &str, addr: u32, size: u32) -> bool {
    let base = ["write", "b32", "--chip", fw.chip, "--probe", probe].map(String::from);
    let whole_words = size / 4;
    let mut word = 0;
    while word < whole_words {
        let n = ZERO_CHUNK_WORDS.min(whole_words - word);
        let mut args = base.to_vec();
        args.push(format!("{:#x}", addr + word * 4));
        args.extend(std::iter::repeat_n("0".to_string(), n as usize));
        if !probe_rs(&args) {
            return false;
        }
        word += n;
    }
    let tail = size % 4;
    if tail > 0 {
        let mut args: Vec<String> = ["write", "b8", "--chip", fw.chip, "--probe", probe]
            .map(String::from)
            .to_vec();
        args.push(format!("{:#x}", addr + whole_words * 4));
        args.extend(std::iter::repeat_n("0".to_string(), tail as usize));
        if !probe_rs(&args) {
            return false;
        }
    }
    true
}

/// Zero the ELF's `.uninit` region so reset-surviving RAM boots clean.
fn zero_uninit(fw: &Firmware, elf: &Path, probe: &str) -> bool {
    let bytes = match std::fs::read(elf) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{}: could not read {}: {e}", fw.name, elf.display());
            return false;
        }
    };
    let (addr, size) = match elf::uninit_range(&bytes) {
        Ok(Some(range)) => range,
        Ok(None) => return true,
        Err(e) => {
            eprintln!("{}: could not read .uninit section: {e}", fw.name);
            return false;
        }
    };
    println!("zeroing .uninit @{addr:#010x}+{size:#x}");

    let zeros_path =
        std::env::temp_dir().join(format!("xtask-uninit-zero-{}.bin", std::process::id()));
    if let Err(e) = std::fs::write(&zeros_path, vec![0u8; size as usize]) {
        eprintln!("{}: could not write zero-fill scratch file: {e}", fw.name);
        return false;
    }
    let downloaded = probe_rs(&[
        "download",
        "--binary-format",
        "bin",
        "--base-address",
        &format!("{addr:#x}"),
        "--chip",
        fw.chip,
        "--probe",
        probe,
        zeros_path.to_str().unwrap_or_default(),
    ]);
    let _ = std::fs::remove_file(&zeros_path);

    downloaded || zero_via_write_loop(fw, probe, addr, size)
}

/// Flash a built row, zero its `.uninit` RAM and reset; RP rows clear the pull-up first.
fn flash(fw: &Firmware, probe: &str) -> bool {
    let elf = root().join(fw.elf());
    println!("flashing {} ({})", fw.name, fw.chip);
    if reattach::applies(fw.chip) && !reattach::clear_pullup(fw, probe) {
        eprintln!("{}: could not clear PULLUP_EN before the download", fw.name);
    }
    probe_rs(&[
        "download",
        "--preverify",
        "--chip",
        fw.chip,
        "--probe",
        probe,
        elf.to_str().unwrap_or_default(),
    ]) && zero_uninit(fw, &elf, probe)
        && probe_rs(&["reset", "--chip", fw.chip, "--probe", probe])
}

/// How a row's image reaches the board, carrying what that way needs.
#[derive(Clone, Copy)]
pub(crate) enum Route<'a> {
    /// Through a transport probe, by its probe-rs selector.
    Probe(&'a str),
    /// Onto the PUT's own BOOTSEL volume, optionally at a given mount path.
    Bootsel(&'a usb::Selector, Option<&'a Path>),
}

/// The row's route from the options; the error names the missing flag.
pub(crate) fn route<'a>(
    fw: &Firmware,
    via: Option<&'a str>,
    put: Option<&'a usb::Selector>,
    bootsel_mount: Option<&'a Path>,
) -> Result<Route<'a>, String> {
    match (fw.bootloader, via, put) {
        (Bootloader::Probe, Some(via), _) => Ok(Route::Probe(via)),
        (Bootloader::Probe, None, Some(put)) if fw.rom_installable() => {
            Ok(Route::Bootsel(put, bootsel_mount))
        }
        (Bootloader::Probe, None, _) => Err(format!(
            "firmware `{}` flashes over a probe transport; pass --via <sel>",
            fw.name
        )),
        (Bootloader::BootselUf2, _, Some(put)) => Ok(Route::Bootsel(put, bootsel_mount)),
        (Bootloader::BootselUf2, _, None) => Err(format!(
            "firmware `{}` flashes through its own BOOTSEL volume; pass --put <vid:pid:serial>",
            fw.name
        )),
    }
}

/// Flash a built row by its route, then wait for a PUT to re-enumerate as itself.
pub(crate) fn install(fw: &Firmware, route: Route, put: Option<&usb::Selector>) -> bool {
    let flashed = match route {
        Route::Probe(via) => flash(fw, via),
        Route::Bootsel(put, mount) => {
            let elf = root().join(fw.elf());
            let uf2 = PathBuf::from(format!("{}.uf2", elf.display()));
            bootsel::flash_bootsel(&uf2, put, mount)
        }
    };
    if !flashed {
        return false;
    }
    let Some(put) = put else {
        return true;
    };
    // Without a re-attach the host keeps the pre-flash descriptors.
    if let Route::Probe(via) = route {
        if reattach::applies(fw.chip) && !reattach::run(fw, via, put) {
            return false;
        }
    }
    wait_for_put(put)
}

/// The flashed board's own USB selector; required for a PUT, `None` for a DUT image.
pub(crate) fn put_selector(
    fw: &Firmware,
    spec: Option<&str>,
) -> Result<Option<usb::Selector>, String> {
    match (spec, fw.profile) {
        (Some(spec), _) => usb::parse_selector(spec).map(Some),
        (None, None) => Ok(None),
        (None, Some(_)) => Err(format!(
            "firmware `{}` is a probe under test; pass --put <vid:pid:serial> so the reflash \
             can be followed on the bus",
            fw.name
        )),
    }
}

/// How long the reflashed PUT must hold its place on the bus before the harness may open it.
const PUT_SETTLE: Duration = Duration::from_millis(750);

const PUT_REENUM_TIMEOUT: Duration = Duration::from_secs(20);

/// Wait for the reflashed PUT to be back on the bus under its own serial.
fn wait_for_put(sel: &usb::Selector) -> bool {
    println!("waiting for the PUT to settle on the bus: {sel}");
    if usb::wait_settled(sel, PUT_SETTLE, PUT_REENUM_TIMEOUT) {
        return true;
    }
    eprintln!("PUT {sel} did not settle on the bus within {PUT_REENUM_TIMEOUT:?}");
    false
}

#[cfg(test)]
mod tests {
    use super::{probe_rs_program, put_selector, route, Route};
    use crate::find_firmware;
    use crate::registry;
    use daplink_decoder::{detect_format, Decoder, DecoderContext, Error, Format, Uf2Decoder};

    /// Reassembles decoded fragments into a flat image starting at `base`.
    struct Rebuild {
        base: u32,
        image: Vec<u8>,
        family: Option<u32>,
    }

    impl DecoderContext for Rebuild {
        fn write_fragment(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
            let off = (addr - self.base) as usize;
            if self.image.len() < off + data.len() {
                self.image.resize(off + data.len(), 0);
            }
            self.image[off..off + data.len()].copy_from_slice(data);
            Ok(())
        }

        fn set_uf2_family(&mut self, family: Option<u32>) -> bool {
            self.family = family;
            true
        }
    }

    /// `emit_uf2` output detects as uf2 and decodes back to the exact image, base and family.
    #[test]
    fn uf2_round_trips_through_daplink_decoder() {
        let base = 0x1000_0000u32;
        let bin: Vec<u8> = (0..2500u32).map(|i| (i * 31) as u8).collect();
        let family = registry::UF2_FAMILY_RP2040;

        let uf2 = daplink_decoder::encode::bin_to_uf2(&bin, base, family);
        assert_eq!(uf2.len() % 512, 0);
        assert_eq!(uf2.len() / 512, bin.len().div_ceil(256));

        assert_eq!(detect_format(None, &uf2), Format::Uf2);
        assert_eq!(detect_format(Some("test-rp2040.uf2"), &[]), Format::Uf2);

        let mut dec = Uf2Decoder::new(None);
        let mut sp = [0u8; 512];
        let mut ctx = Rebuild {
            base,
            image: Vec::new(),
            family: None,
        };
        dec.input(&uf2, &mut sp, &mut ctx).unwrap();
        dec.finish(&mut sp, &mut ctx).unwrap();

        assert_eq!(ctx.family, Some(family));
        assert!(ctx.image.len() >= bin.len());
        assert_eq!(&ctx.image[..bin.len()], &bin[..]);
    }

    /// Rows sharing a build directory would evict each other's output.
    #[test]
    fn every_row_builds_in_its_own_directory() {
        let mut seen = std::collections::BTreeSet::new();
        for fw in registry::FIRMWARE {
            assert!(
                seen.insert(fw.target_dir()),
                "{} shares a target dir",
                fw.name
            );
        }
    }

    /// A PUT reflash that cannot be followed on the bus is refused; a DUT image has nothing to follow.
    #[test]
    fn only_a_put_row_demands_its_own_selector() {
        let put = find_firmware("firmware/rp2040-pico-rtic").expect("a registered PUT row");
        let dut = find_firmware("tests/firmwares/test-rp2040").expect("a registered DUT row");
        assert!(put_selector(put, None).is_err());
        assert!(put_selector(dut, None).expect("a DUT needs none").is_none());
        assert!(put_selector(put, Some("0d28:0204:SERIAL"))
            .expect("a full selector parses")
            .is_some());
        // probe-rs accepts a serial-less selector; the bus wait cannot use one.
        assert!(put_selector(put, Some("0d28:0204")).is_err());
    }

    /// A probe row needs a transport, or for an RP probe its PUT; a BOOTSEL row needs its PUT.
    #[test]
    fn each_route_demands_what_it_flashes_through() {
        let probe = find_firmware("firmware/rp2040-pico-rtic").expect("a probe-flashed row");
        let bootsel = find_firmware("firmware/debugprobe").expect("a BOOTSEL row");
        let put = crate::usb::parse_selector("2e8a:000c:E6614103E7").unwrap();
        assert!(matches!(
            route(probe, Some("0d28:0204:X"), None, None),
            Ok(Route::Probe("0d28:0204:X"))
        ));
        let no_uf2 = find_firmware("firmware/rp2350-pico2-embassy-bitbang").expect("a row");
        let err = route(no_uf2, None, Some(&put), None).err().unwrap();
        assert!(err.contains("--via"), "{err}");
        for name in ["firmware/rp2040-pico-rtic", "firmware/rp2350-pico2-embassy"] {
            let rp = find_firmware(name).expect("an RP probe row");
            assert!(matches!(
                route(rp, None, Some(&put), None),
                Ok(Route::Bootsel(_, None))
            ));
        }
        let dut = find_firmware("tests/firmwares/test-rp2350").expect("a DUT row");
        assert!(route(dut, None, Some(&put), None).is_err());
        assert!(matches!(
            route(bootsel, None, Some(&put), None),
            Ok(Route::Bootsel(_, None))
        ));
        let err = route(bootsel, Some("0d28:0204:X"), None, None)
            .err()
            .unwrap();
        assert!(err.contains("--put"), "{err}");
    }

    #[test]
    fn the_probe_rs_program_defaults_unless_a_binary_is_named() {
        assert_eq!(probe_rs_program(None), "probe-rs");
        assert_eq!(probe_rs_program(Some("".into())), "probe-rs");
        assert_eq!(probe_rs_program(Some("/x/probe-rs".into())), "/x/probe-rs");
    }
}
