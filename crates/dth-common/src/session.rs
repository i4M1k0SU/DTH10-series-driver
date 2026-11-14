//! [`Session`]: what the daemon and the command ask of an opened unit,
//! whichever family it is (`dth10-backend`, `dth30-backend`).

use crate::Result;
use crate::channels::{Channel, System};
use crate::queue::TsQueue;
use crate::stream::{DeviceProgram, StreamModeCfg};

/// One opened unit: tuners 0 (terrestrial) and 1 (satellite, or a second
/// terrestrial tuner), each streaming into its own [`TsQueue`].
pub trait Session: Send {
    /// Where one tuner's TS goes.
    fn set_queue(&mut self, tuner: usize, queue: TsQueue);
    /// Whether tuner #1 is a second terrestrial tuner.
    fn has_second_terrestrial(&self) -> bool;
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
