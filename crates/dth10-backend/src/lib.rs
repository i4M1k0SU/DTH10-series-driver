//! The BUFFALO DTH10 series behind [`dth_common`]'s [`dth_common::Backend`]
//! and [`dth_common::Session`]:
//!
//! * [`config`]  – [`TunerConfig`], the series' own settings
//! * [`psi`]     – PAT/PMT collector for the device-side descrambling mode
//! * [`tuner`]   – [`Tuner`]: USB open, kit init, tuning, the TS reader thread with CEPM decryption
//! * [`backend`] – [`Dth10Backend`] (finding and opening units) and [`KeyStore`] (the application credentials)
//!
//! The crate is platform independent (it depends on `dth10-core` and the
//! `nusb` transport of `dth10-usb`).

pub mod backend;
pub mod config;
pub mod psi;
pub mod tuner;

pub use backend::{Dth10Backend, KeyStore, app_keys_path, state_file_for};
pub use config::TunerConfig;
pub use tuner::{Aligner, Tuner};
