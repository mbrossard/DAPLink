#[test]
fn split_reset_requires_an_unsafe_call() {
    let fixture = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/ui/split_reset_live_borrow.rs");
    let output =
        std::process::Command::new(std::env::var_os("RUSTC").unwrap_or_else(|| "rustc".into()))
            .args([
                "--edition=2021",
                "--crate-type=lib",
                "--emit=metadata",
                "-o",
                "-",
            ])
            .arg(fixture)
            .output()
            .expect("rustc must be available to check the API");
    let errors = String::from_utf8_lossy(&output.stderr);
    assert!(!output.status.success(), "safe split reset compiled");
    assert!(errors.contains("E0133"), "unexpected rejection: {errors}");
}
