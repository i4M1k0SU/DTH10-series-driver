//! With the `rtti-check` feature, build the C++ side of the RTTI test
//! (`tests/rtti_check.cpp`) with the host's MSVC so that `cargo test
//! --features rtti-check` can run a real `dynamic_cast` on our object. The
//! feature is off by default: a plain build needs no C++ compiler.

#[path = "../../build/windows.rs"]
mod windows;

fn main() {
    windows::compile("BonDriver_DTH.dll", true);
    println!("cargo:rerun-if-changed=tests/rtti_check.cpp");
    if std::env::var_os("CARGO_FEATURE_RTTI_CHECK").is_some() && std::env::var("CARGO_CFG_WINDOWS").is_ok() {
        cc::Build::new().cpp(true).file("tests/rtti_check.cpp").compile("dth10_rtti_check");
    }
}
