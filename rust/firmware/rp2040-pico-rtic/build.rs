//! Place `memory.x` on the linker search path and add the link script.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    println!("cargo:rustc-check-cfg=cfg(rp_accel)");
    if env::var_os("CARGO_FEATURE_ACCEL").is_some()
        && env::var_os("CARGO_FEATURE_BITBANG").is_none()
    {
        println!("cargo:rustc-cfg=rp_accel");
    }

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    File::create(out.join("ram-text.x"))
        .unwrap()
        .write_all(include_bytes!("ram-text.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=ram-text.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    // Whole image runs from SRAM: pull .text there before link.x places the rest.
    println!("cargo:rustc-link-arg-bins=-Tram-text.x");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");

    // defmt-rtt's linker script only exists to link against when the feature pulls the crate in.
    if env::var_os("CARGO_FEATURE_DEFMT").is_some() {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }
}
