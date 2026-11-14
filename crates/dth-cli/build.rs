#[path = "../../build/windows.rs"]
mod windows;

fn main() {
    windows::compile("dth.exe", false);
}
