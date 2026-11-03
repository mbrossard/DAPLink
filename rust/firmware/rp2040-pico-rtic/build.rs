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

    if env::var_os("CARGO_FEATURE_DEFMT").is_some() {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }
}
