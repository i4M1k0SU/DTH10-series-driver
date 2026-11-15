//! Builds this project's own firmware (`../dth30-fw`) for the chip's MIPS
//! core and leaves its raw image in `OUT_DIR/dth30-fw.bin` for
//! `Firmware::own`.
//!
//! The firmware is a separate package with its own target description; it
//! is built here with a nested `cargo build`. Rust ships no `core` for
//! that CPU, so `core` is built from source (the `rust-src` component,
//! listed in `rust-toolchain.toml`), which a stable toolchain only does
//! with `RUSTC_BOOTSTRAP=1`: set here, for the nested build alone.
//!
//! With `DTH30_NO_OWN_FIRMWARE` set in the environment the firmware is
//! not built and `Firmware::own` gives `None`; the programs then look for
//! a firmware file (`Firmware::select`). This is for a Rust installation
//! without `rust-src`.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Where the image is linked and loaded (kseg0 of SDRAM physical 1 MB).
const BASE: u32 = 0x8010_0000;
/// The mark at image +0x100 ("DTH3").
const MAGIC: u32 = 0x3348_5444;
const SKIP_ENV: &str = "DTH30_NO_OWN_FIRMWARE";

fn main() {
    println!("cargo::rerun-if-env-changed={SKIP_ENV}");
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    let image = out.join("dth30-fw.bin");
    if env::var_os(SKIP_ENV).is_some() {
        fs::write(&image, []).expect("write the empty image");
        return;
    }
    let fw = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR")).join("../dth30-fw");
    for f in ["src", "Cargo.toml", "Cargo.lock", "link.x", "mipsel-xcode2.json", ".cargo/config.toml"] {
        println!("cargo::rerun-if-changed={}", fw.join(f).display());
    }

    // The nested build's files have long names. Under OUT_DIR they can pass
    // the 260-character limit of Windows paths, so they go next to cargo's
    // `build` directory instead (OUT_DIR is `<profile dir>/build/.../out`).
    let target_dir = out
        .ancestors()
        .find(|p| p.file_name().is_some_and(|n| n == "build"))
        .and_then(Path::parent)
        .map_or_else(|| out.join("fw"), |profile| profile.join("dth30-fw"));
    let mut cmd = Command::new(env::var_os("CARGO").unwrap_or_else(|| "cargo".into()));
    cmd.current_dir(&fw).args(["build", "--release", "--target-dir"]).arg(&target_dir);
    // What cargo set up for this build script and the outer build must not
    // leak into the build for another target.
    for (key, _) in env::vars_os() {
        let k = key.to_string_lossy();
        let cargo_internal =
            ["CARGO_CFG_", "CARGO_FEATURE_", "CARGO_PKG_", "CARGO_ENCODED_", "CARGO_PROFILE_", "CARGO_BUILD_"]
                .iter()
                .any(|p| k.starts_with(p));
        let other = [
            "RUSTFLAGS",
            "RUSTC",
            "RUSTDOC",
            "RUSTC_LINKER",
            "RUSTC_WRAPPER",
            "RUSTC_WORKSPACE_WRAPPER",
            "CLIPPY_ARGS",
            "CARGO_TARGET_DIR",
            "CARGO_MANIFEST_DIR",
            "CARGO_MANIFEST_PATH",
            "CARGO_PRIMARY_PACKAGE",
            "TARGET",
            "HOST",
            "PROFILE",
            "OPT_LEVEL",
            "DEBUG",
            "OUT_DIR",
        ]
        .contains(&&*k);
        if cargo_internal || other {
            cmd.env_remove(&key);
        }
    }
    cmd.env("RUSTC_BOOTSTRAP", "1");
    let status = cmd.status().unwrap_or_else(|e| panic!("could not run cargo for the firmware: {e}"));
    if !status.success() {
        panic!(
            "building the firmware in {} failed ({status}). It needs the Rust source component \
             (`rustup component add rust-src`); set {SKIP_ENV}=1 to build without the firmware.",
            fw.display()
        );
    }

    let elf = target_dir.join("mipsel-xcode2/release/dth30-fw");
    let raw = flatten(&fs::read(&elf).unwrap_or_else(|e| panic!("{}: {e}", elf.display())), &elf);
    fs::write(&image, raw).expect("write the image");
}

/// The loadable segments of the firmware's ELF file as one image starting
/// at [`BASE`].
fn flatten(elf: &[u8], path: &Path) -> Vec<u8> {
    let u16_at = |o: usize| u16::from_le_bytes(elf[o..o + 2].try_into().unwrap()) as usize;
    let u32_at = |o: usize| u32::from_le_bytes(elf[o..o + 4].try_into().unwrap());
    // ELF32, little endian.
    assert!(elf.len() > 0x34 && elf[..6] == [0x7F, b'E', b'L', b'F', 1, 1], "{}: not an ELF32 file", path.display());
    let (phoff, phentsize, phnum) = (u32_at(0x1C) as usize, u16_at(0x2A), u16_at(0x2C));
    let mut image = Vec::new();
    for i in 0..phnum {
        let ph = phoff + i * phentsize;
        let (kind, offset, addr, filesz) =
            (u32_at(ph), u32_at(ph + 4) as usize, u32_at(ph + 8), u32_at(ph + 16) as usize);
        // PT_LOAD with contents.
        if kind != 1 || filesz == 0 {
            continue;
        }
        let at = addr.checked_sub(BASE).unwrap_or_else(|| panic!("{}: segment below the image base", path.display()))
            as usize;
        if image.len() < at + filesz {
            image.resize(at + filesz, 0);
        }
        image[at..at + filesz].copy_from_slice(&elf[offset..offset + filesz]);
    }
    assert!(
        image.len() >= 0x200 && image[0x100..0x104] == MAGIC.to_le_bytes(),
        "{}: the firmware's mark is missing",
        path.display()
    );
    image
}
