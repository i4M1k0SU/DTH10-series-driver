//! [`Session`]: what `dth10 serve` asks of a unit, whichever family it is
//! ([`crate::Tuner`] for the DTH10 series, [`crate::dth30::Dth30Tuner`] for
//! the DT-H30/U2 family).

use std::sync::Arc;

use dth10_core::Result;
use dth10_core::cepm::AppKeys;

use crate::{Channel, DeviceProgram, StreamModeCfg, System, TsQueue, Tuner};

/// One opened unit: tuners 0 (terrestrial) and 1 (satellite, or a second
/// terrestrial tuner), each streaming into its own [`TsQueue`].
pub trait Session: Send {
    /// Where one tuner's TS goes.
    fn set_queue(&mut self, tuner: usize, queue: TsQueue);
    /// The application credentials for the link decryption (units that
    /// need none ignore them).
    fn with_app_keys(&mut self, _keys: Arc<AppKeys>) {}
    /// Whether streaming `system` in this mode needs the application
    /// credentials.
    fn needs_app_keys(&self, raw: bool, system: System) -> bool;
    /// Whether tuner #1 is a second terrestrial tuner.
    fn has_second_terrestrial(&self) -> bool;
    /// Whether the two tuners stream one at a time (a tune on one stops the
    /// other's stream), as on the DT-H50/U2W.
    fn exclusive_tuners(&self) -> bool {
        false
    }
    fn set_stream_mode(&mut self, mode: StreamModeCfg);
    fn set_device_program(&mut self, program: DeviceProgram);
    /// Antenna supply for BS/CS (units without one ignore it).
    fn set_lnb(&mut self, _volt: u32) {}
    fn set_attenuator(&mut self, _on: bool) {}
    fn set_booster(&mut self, _on: bool) {}
    fn set_lock_wait_ms(&mut self, system: System, ms: u64);
    /// Tune tuner `tuner` to `ch` and start streaming into its queue.
    fn set_channel_on(&mut self, tuner: usize, ch: &Channel) -> Result<()>;
    /// C/N in dB of one tuner (0.0 when unavailable).
    fn signal_level_of(&mut self, tuner: usize) -> f32;
    /// Stop one tuner's stream.
    fn release(&mut self, tuner: usize);
    /// One tuner's stream gave itself up (the device is gone).
    fn stream_failed(&self, tuner: usize) -> bool;
    /// The C/N of every tuner tuned to a channel (about once a second while
    /// streaming).
    fn poll_status(&mut self) -> [Option<f32>; 2];
    /// Stop everything and put the device to rest.
    fn close(&mut self);
}

impl Session for Tuner {
    fn set_queue(&mut self, tuner: usize, queue: TsQueue) {
        Tuner::set_queue(self, tuner, queue)
    }
    fn with_app_keys(&mut self, keys: Arc<AppKeys>) {
        Tuner::with_app_keys(self, keys)
    }
    /// Device mode and BS/CS decrypt the link; a raw terrestrial stream
    /// does not.
    fn needs_app_keys(&self, raw: bool, system: System) -> bool {
        !raw || system == System::IsdbS
    }
    fn has_second_terrestrial(&self) -> bool {
        Tuner::has_second_terrestrial(self)
    }
    fn set_stream_mode(&mut self, mode: StreamModeCfg) {
        Tuner::set_stream_mode(self, mode)
    }
    fn set_device_program(&mut self, program: DeviceProgram) {
        Tuner::set_device_program(self, program)
    }
    fn set_lnb(&mut self, volt: u32) {
        Tuner::set_lnb(self, volt)
    }
    fn set_attenuator(&mut self, on: bool) {
        Tuner::set_attenuator(self, on)
    }
    fn set_booster(&mut self, on: bool) {
        Tuner::set_booster(self, on)
    }
    fn set_lock_wait_ms(&mut self, system: System, ms: u64) {
        Tuner::set_lock_wait_ms(self, system, ms)
    }
    fn set_channel_on(&mut self, tuner: usize, ch: &Channel) -> Result<()> {
        Tuner::set_channel_on(self, tuner, ch)
    }
    fn signal_level_of(&mut self, tuner: usize) -> f32 {
        Tuner::signal_level_of(self, tuner)
    }
    fn release(&mut self, tuner: usize) {
        Tuner::release(self, tuner)
    }
    fn stream_failed(&self, tuner: usize) -> bool {
        Tuner::stream_failed(self, tuner)
    }
    fn poll_status(&mut self) -> [Option<f32>; 2] {
        Tuner::poll_status(self)
    }
    fn close(&mut self) {
        Tuner::close(self)
    }
}
