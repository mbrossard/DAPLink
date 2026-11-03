//! Flags a Cortex-M7 build: the only M7 signal stable Rust offers is `-C target-cpu`.

fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rustc-check-cfg=cfg(daplink_cortex_m7)");
    let flags = std::env::var("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    let mut flags = flags.split('\x1f');
    while let Some(flag) = flags.next() {
        let cpu = match flag {
            "-C" | "--codegen" => flags.next(),
            _ => flag.strip_prefix("-C").or(flag.strip_prefix("--codegen=")),
        };
        if cpu.and_then(|c| c.strip_prefix("target-cpu=")) == Some("cortex-m7") {
            println!("cargo::rustc-cfg=daplink_cortex_m7");
        }
    }
}
