use std::{env, fs, process::Command};

#[test]
fn retained_capabilities_reject_safe_aliases_and_terminal_calls() {
    let executable = env::current_exe().unwrap();
    let deps = executable.parent().unwrap();
    let library = fs::read_dir(deps)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| {
            path.file_name()
                .unwrap()
                .to_string_lossy()
                .starts_with("libdaplink_vfs-")
                && path.extension().is_some_and(|ext| ext == "rlib")
        })
        .max_by_key(|path| fs::metadata(path).unwrap().modified().unwrap())
        .expect("cargo built daplink-vfs");
    let scratch = deps.join(format!("retained-ownership-{}", std::process::id()));
    fs::create_dir_all(&scratch).unwrap();
    for (name, source, diagnostic) in [
        (
            "live_token",
            "let mut owner = InitializedStamp::zeroed(); let mut first = owner.boot(); let _second = owner.boot(); first.status.store(b\"live\", false);",
            "E0499",
        ),
        (
            "duplicate_writer",
            "let mut owner = InitializedStamp::zeroed(); let first = owner.boot(); let _second = first.status.clone();",
            "E0599",
        ),
        (
            "returning_terminal",
            "let _ = daplink_vfs::stamp::terminal_panic(None);",
            "E0133",
        ),
        (
            "uninitialized_admission",
            "let mut slot = core::mem::MaybeUninit::<[u32; 34]>::uninit(); let _ = daplink_vfs::stamp::boot_retained(slot.as_mut_ptr().cast());",
            "E0133",
        ),
        (
            "other_core_fault_writer",
            "let owner = Box::leak(Box::new(InitializedStamp::zeroed())); let boot = owner.boot(); std::thread::spawn(move || drop(boot.fault));",
            "E0277",
        ),
    ] {
        let input = scratch.join(format!("{name}.rs"));
        fs::write(&input, format!("use daplink_vfs::stamp::InitializedStamp; fn main() {{ {source} }}")).unwrap();
        let result = Command::new(env::var_os("RUSTC").unwrap_or_else(|| "rustc".into()))
            .arg("--edition=2021")
            .arg("--crate-name").arg(name)
            .arg("--emit=metadata")
            .arg("--extern").arg(format!("daplink_vfs={}", library.display()))
            .arg("-L").arg(format!("dependency={}", deps.display()))
            .arg("--out-dir").arg(&scratch)
            .arg(&input)
            .output().unwrap();
        let stderr = String::from_utf8(result.stderr).unwrap();
        assert!(!result.status.success(), "{name} unexpectedly compiled");
        assert!(stderr.contains(diagnostic), "{name}: {stderr}");
    }
}
