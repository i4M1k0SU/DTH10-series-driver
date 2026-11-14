//! Parts shared by the BUFFALO DTH10-series (`dth10-*`) and DT-H30/U2-family
//! (`dth30-*`) crates that belong to neither family:
//!
//! * [`error`]    – [`Error`] / [`Result`], used by both families' protocol layers
//! * [`LockExt`]  – mutex locking that survives a poisoned lock
//! * [`channels`] – tuning systems, channel tables (UHF / BS-IF / CS-IF) and channel-name parsing
//! * [`stream`]   – the stream settings (raw / device mode, the program to descramble)
//! * [`queue`]    – [`queue::TsQueue`], where a tuner's TS goes
//! * [`session`]  – [`session::Session`], what is asked of an opened unit
//! * [`backend`]  – [`backend::Backend`], one family of devices
//!
//! It depends on nothing but `std`: the chip crates of both families use it.

pub mod backend;
pub mod channels;
pub mod error;
pub mod queue;
pub mod session;
pub mod stream;

pub use backend::{Backend, Request, Unit};
pub use channels::{Channel, System, default_tuner};
pub use error::{Error, Result};
pub use queue::TsQueue;
pub use session::Session;
pub use stream::{DeviceProgram, StreamModeCfg};

/// `Mutex::lock` that carries on when a thread panicked while holding the
/// lock: what these crates keep behind a mutex (queues, handshake sets,
/// connection slots) stays usable, and a poisoned lock would otherwise take
/// the whole tuner down with the one thread.
pub trait LockExt<T> {
    fn locked(&self) -> std::sync::MutexGuard<'_, T>;
}

impl<T> LockExt<T> for std::sync::Mutex<T> {
    fn locked(&self) -> std::sync::MutexGuard<'_, T> {
        self.lock().unwrap_or_else(|e| e.into_inner())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[test]
    fn a_poisoned_lock_stays_usable() {
        let m = Arc::new(Mutex::new(1));
        let m2 = m.clone();
        let _ = std::thread::spawn(move || {
            let _g = m2.lock().unwrap();
            panic!("poison");
        })
        .join();
        assert!(m.is_poisoned());
        *m.locked() += 1;
        assert_eq!(*m.locked(), 2);
    }
}
