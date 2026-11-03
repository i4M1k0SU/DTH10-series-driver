#[path = "../../build/windows.rs"]
mod windows;

fn main() {
    windows::compile("dth10.exe", false);
}
