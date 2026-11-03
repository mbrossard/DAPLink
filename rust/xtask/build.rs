#[path = "src/sources.rs"]
mod sources;

use std::fmt::Write;

/// Embed the sources by name; the crate includes them itself, so they are the ones it compiled.
fn main() {
    let dir = std::env::var_os("CARGO_MANIFEST_DIR").expect("set by cargo");
    let out = std::env::var_os("OUT_DIR").expect("set by cargo");
    println!("cargo::rerun-if-changed=src");
    let mut list = String::from("&[\n");
    for file in sources::files(dir.as_ref()).expect("xtask sources are readable") {
        writeln!(
            list,
            "({file:?}, include_bytes!(concat!(env!(\"CARGO_MANIFEST_DIR\"), \"/{file}\"))),"
        )
        .unwrap();
    }
    list.push(']');
    std::fs::write(std::path::Path::new(&out).join("sources.rs"), list)
        .expect("OUT_DIR is writable");
}
