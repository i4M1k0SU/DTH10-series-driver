//! Tuner session logic for the BUFFALO DTH10 series of tuners, shared by the TVTest
//! BonDriver (`dth10-bondriver`) and the stand-alone `dth10` command:
//!
//! * [`channels`] – tuning systems, channel tables (UHF / BS-IF / CS-IF) and channel-name parsing
//! * [`config`]   – [`TunerConfig`], the settings both front ends map their options to
//! * [`psi`]      – PAT/PMT collector for the device-side descrambling mode
//! * [`tuner`]    – [`Tuner`]: USB open, kit init, tuning, the TS reader thread with CEPM decryption, [`TsQueue`]
//! * [`proto`]    – the request/reply lines between `dth10 serve` and its clients (`dth10 stream`, the BonDriver)
//! * [`ipc`]      – the socket those lines travel on (Unix socket on Linux, loopback TCP everywhere)
//!
//! The crate is platform independent (it only depends on `dth10-core` and the
//! `nusb` transport of `dth10-usb`).

pub mod channels;
pub mod config;
pub mod ipc;
pub mod proto;
pub mod psi;
pub mod tuner;

pub use channels::{Channel, System};
pub use config::{DeviceProgram, StreamModeCfg, TunerConfig};
pub use tuner::{Aligner, TsQueue, Tuner};
