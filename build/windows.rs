//! Windows VERSIONINFO shared by the executables and the BonDriver DLL.

use std::env;

const AUTHOR: &str = "i4M1k0SU";

// VERSIONINFO values (winver.h), which no crate of the build provides.
/// `VFT_APP`: the file is an application.
const VFT_APP: u64 = 1;
/// `VFT_DLL`: the file is a DLL.
const VFT_DLL: u64 = 2;
/// `VS_FF_PRERELEASE`: the file is a development version.
const VS_FF_PRERELEASE: u64 = 2;

pub fn compile(filename: &str, dll: bool) {
    println!("cargo:rerun-if-changed=../../build/windows.rs");
    if env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("windows") {
        return;
    }

    let product = env::var("CARGO_PKG_NAME").expect("package name");
    let version = env::var("CARGO_PKG_VERSION").expect("package version");
    let year = jiff::Timestamp::now().to_zoned_datetime(jiff::tz::TimeZone::UTC).year();
    let mut numeric = 0u64;
    for component in ["MAJOR", "MINOR", "PATCH"] {
        let value: u16 = env::var(format!("CARGO_PKG_VERSION_{component}"))
            .expect("version component")
            .parse()
            .expect("Windows version components must fit in 0..=65535");
        numeric = (numeric << 16) | u64::from(value);
    }
    numeric <<= 16;

    let mut resource = winresource::WindowsResource::new();
    resource
        .set("CompanyName", AUTHOR)
        .set("ProductName", &product)
        .set("LegalCopyright", &format!("Copyright (c) {year} {AUTHOR}"))
        .set("FileDescription", &env::var("CARGO_PKG_DESCRIPTION").expect("package description"))
        .set("FileVersion", &version)
        .set("ProductVersion", &version)
        .set("InternalName", filename.rsplit_once('.').map_or(filename, |(name, _)| name))
        .set("OriginalFilename", filename)
        .set_version_info(winresource::VersionInfo::FILEVERSION, numeric)
        .set_version_info(winresource::VersionInfo::PRODUCTVERSION, numeric)
        .set_version_info(winresource::VersionInfo::FILETYPE, if dll { VFT_DLL } else { VFT_APP });
    if !env::var("CARGO_PKG_VERSION_PRE").expect("prerelease component").is_empty() {
        resource.set_version_info(winresource::VersionInfo::FILEFLAGS, VS_FF_PRERELEASE);
    }
    resource.compile().expect("compile Windows version resource (Windows SDK / rc.exe required)");
}
