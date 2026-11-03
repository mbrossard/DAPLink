//! Add the linker scripts; embassy-stm32's `memory-x` feature supplies `memory.x`.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("ramcode.x"), include_bytes!("ramcode.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=ramcode.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tramcode.x");

    if env::var_os("CARGO_FEATURE_DEFMT").is_some() {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
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
}
