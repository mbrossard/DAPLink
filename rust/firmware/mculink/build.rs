//! Place `memory.x` on the linker search path and add the link script.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=sramx.x");
    println!("cargo:rerun-if-changed=dmamem.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");

    // The DMA block is pinned ahead of `.data` so no unrelated static can move it.
    File::create(out.join("dmamem.x"))
        .unwrap()
        .write_all(include_bytes!("dmamem.x"))
        .unwrap();
    println!("cargo:rustc-link-arg-bins=-Tdmamem.x");

    // The accel engine's SRAMX placement rides as its own `-T` script; memory.x cannot carry it.
    if env::var_os("CARGO_FEATURE_ACCEL").is_some()
        && env::var_os("CARGO_FEATURE_BITBANG").is_none()
        && env::var_os("CARGO_FEATURE_FLASH").is_none()
    {
        File::create(out.join("sramx.x"))
            .unwrap()
            .write_all(include_bytes!("sramx.x"))
            .unwrap();
        println!("cargo:rustc-link-arg-bins=-Tsramx.x");
    }

    // defmt-rtt's linker script only exists to link against when the feature pulls the crate in.
    if env::var_os("CARGO_FEATURE_DEFMT").is_some() {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }

    // The default 150 MHz build links NXP's power library for `POWER_SetVoltageForFreq`.
    if env::var_os("CARGO_FEATURE_CORE_96MHZ").is_none() {
        let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
        let lib_dir = manifest.join("../../../source/hic_hal/nxp/lpc55xx/LPC55S69/gcc");
        let lib_file = lib_dir.join("libpower_hardabi.a");
        assert!(
            lib_file.is_file(),
            "the LPC55S69 power library is missing at {}; the 150 MHz build needs \
             POWER_SetVoltageForFreq from it",
            lib_file.display()
        );
        println!("cargo:rustc-link-search=native={}", lib_dir.display());
        println!("cargo:rustc-link-lib=static=power_hardabi");
        println!("cargo:rerun-if-changed={}", lib_file.display());
    }
}
