#[path = "../../build/windows.rs"]
mod windows;

fn main() {
    windows::compile("dth30-diag.exe", false);
}
