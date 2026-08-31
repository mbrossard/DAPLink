//! Add the linker scripts; embassy-stm32's `memory-x` feature supplies `memory.x`.

fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");

    // defmt-rtt's linker script only exists to link against when the feature pulls the crate in.
    if std::env::var_os("CARGO_FEATURE_DEFMT").is_some() {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }
}
