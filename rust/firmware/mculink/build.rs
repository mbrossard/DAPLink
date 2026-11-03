//! Link scripts, the `accel_engine` cfg, the alignment check and the 150 MHz power library.

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

    File::create(out.join("dmamem.x"))
        .unwrap()
        .write_all(include_bytes!("dmamem.x"))
        .unwrap();
    println!("cargo:rustc-link-arg-bins=-Tdmamem.x");

    // One wire engine per image: `bitbang` wins over `accel`.
    println!("cargo::rustc-check-cfg=cfg(accel_engine)");
    let accel_engine = env::var_os("CARGO_FEATURE_ACCEL").is_some()
        && env::var_os("CARGO_FEATURE_BITBANG").is_none();
    if accel_engine {
        println!("cargo:rustc-cfg=accel_engine");
        File::create(out.join("sramx.x"))
            .unwrap()
            .write_all(include_bytes!("sramx.x"))
            .unwrap();
        println!("cargo:rustc-link-arg-bins=-Tsramx.x");
    }

    // A `RUSTFLAGS` in the environment replaces the config's flags; LLVM takes the last value.
    let rustflags = env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    let align = rustflags
        .rsplit_once("-align-all-functions=")
        .map(|(_, rest)| rest.split(|c: char| !c.is_ascii_digit()).next().unwrap());
    assert!(
        align == Some("2"),
        "build without RUSTFLAGS, or make `-C llvm-args=-align-all-functions=2` its last \
         alignment flag: the paced bodies' timing depends on it"
    );

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
