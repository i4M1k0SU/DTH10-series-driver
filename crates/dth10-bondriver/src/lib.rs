//! BonDriver (IBonDriver / IBonDriver2) DLL for the BUFFALO DTH10 series of
//! USB TV tuners (verified on a DT-H70/U2).
//!
//! The C++ interface TVTest expects (`IBonDriver2.h`):
//!
//! ```cpp
//! class IBonDriver {
//!   virtual const BOOL  OpenTuner(void);
//!   virtual void        CloseTuner(void);
//!   virtual const BOOL  SetChannel(const BYTE bCh);
//!   virtual const float GetSignalLevel(void);
//!   virtual const DWORD WaitTsStream(const DWORD dwTimeOut);
//!   virtual const DWORD GetReadyCount(void);
//!   virtual const BOOL  GetTsStream(BYTE *pDst, DWORD *pdwSize, DWORD *pdwRemain);
//!   virtual const BOOL  GetTsStream(BYTE **ppDst, DWORD *pdwSize, DWORD *pdwRemain);
//!   virtual void        PurgeTsStream(void);
//!   virtual void        Release(void);
//! };
//! class IBonDriver2 : public IBonDriver {
//!   virtual LPCTSTR     GetTunerName(void);
//!   virtual const BOOL  IsTunerOpening(void);
//!   virtual LPCTSTR     EnumTuningSpace(const DWORD dwSpace);
//!   virtual LPCTSTR     EnumChannelName(const DWORD dwSpace, const DWORD dwChannel);
//!   virtual const BOOL  SetChannel(const DWORD dwSpace, const DWORD dwChannel);
//!   virtual const DWORD GetCurSpace(void);
//!   virtual const DWORD GetCurChannel(void);
//! };
//! extern "C" __declspec(dllexport) IBonDriver *CreateBonDriver(void);
//! ```
//!
//! MSVC lays out *adjacent overloads* in reverse declaration order in the
//! vtable, so the `BYTE **` variant of `GetTsStream` comes before the
//! `BYTE *` variant. Member functions use `thiscall` on x86 and the standard
//! x64 calling convention otherwise.
//!
//! The DLL never opens the device itself: the tuner is held by the server
//! process `dth10.exe serve` in the DLL's folder ([`client`]), which lets
//! several DLL instances (two TVTest windows, GR and BS/CS) share one unit
//! and brings the stream back by itself after a crash or an unplug.
//!
//! Every entry point runs under [`guard`]: a panic inside the DLL must not
//! unwind into TVTest (Rust aborts the process when a panic reaches an
//! `extern` boundary), so it is caught, logged, and the call fails.

#![cfg(windows)]
#![allow(non_snake_case)]

mod channels;
mod client;
mod config;
mod rtti;

use std::ffi::c_void;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Mutex, Once};

use config::Config;
use dth10_core::LockExt;
use dth10_tuner::TsQueue;

#[cfg(target_arch = "x86")]
macro_rules! bon_abi {
    ($($t:tt)*) => { unsafe extern "thiscall" $($t)* };
}
#[cfg(not(target_arch = "x86"))]
macro_rules! bon_abi {
    ($($t:tt)*) => { unsafe extern "system" $($t)* };
}

type Bool = i32;
type Dword = u32;
type Pcwstr = *const u16;

#[repr(C)]
struct VTable {
    open_tuner: bon_abi!(fn(*mut Obj) -> Bool),
    close_tuner: bon_abi!(fn(*mut Obj)),
    set_channel_byte: bon_abi!(fn(*mut Obj, u8) -> Bool),
    get_signal_level: bon_abi!(fn(*mut Obj) -> f32),
    wait_ts_stream: bon_abi!(fn(*mut Obj, Dword) -> Dword),
    get_ready_count: bon_abi!(fn(*mut Obj) -> Dword),
    get_ts_stream_pp: bon_abi!(fn(*mut Obj, *mut *mut u8, *mut Dword, *mut Dword) -> Bool),
    get_ts_stream_p: bon_abi!(fn(*mut Obj, *mut u8, *mut Dword, *mut Dword) -> Bool),
    purge_ts_stream: bon_abi!(fn(*mut Obj)),
    release: bon_abi!(fn(*mut Obj)),
    // IBonDriver2
    get_tuner_name: bon_abi!(fn(*mut Obj) -> Pcwstr),
    is_tuner_opening: bon_abi!(fn(*mut Obj) -> Bool),
    enum_tuning_space: bon_abi!(fn(*mut Obj, Dword) -> Pcwstr),
    enum_channel_name: bon_abi!(fn(*mut Obj, Dword, Dword) -> Pcwstr),
    set_channel2: bon_abi!(fn(*mut Obj, Dword, Dword) -> Bool),
    get_cur_space: bon_abi!(fn(*mut Obj) -> Dword),
    get_cur_channel: bon_abi!(fn(*mut Obj) -> Dword),
}

/// The vtable preceded by the MSVC RTTI Complete Object Locator pointer
/// (`vptr[-1]`), see [`rtti`].
#[repr(C)]
struct VTableWithRtti {
    col: *const c_void,
    vt: VTable,
}
unsafe impl Sync for VTableWithRtti {}

static VTABLE: VTableWithRtti = VTableWithRtti {
    col: rtti::col_ptr(),
    vt: VTable {
        open_tuner,
        close_tuner,
        set_channel_byte,
        get_signal_level,
        wait_ts_stream,
        get_ready_count,
        get_ts_stream_pp,
        get_ts_stream_p,
        purge_ts_stream,
        release,
        get_tuner_name,
        is_tuner_opening,
        enum_tuning_space,
        enum_channel_name,
        set_channel2,
        get_cur_space,
        get_cur_channel,
    },
};

struct State {
    /// The client of the server (the tuner itself lives in `dth10.exe serve`,
    /// see [`client`]); `None` until OpenTuner.
    backend: Option<client::Client>,
    queue: TsQueue,
    cur_space: Dword,
    cur_channel: Dword,
    /// Buffer handed out by the `BYTE **` variant of GetTsStream; must stay
    /// valid until the next call, as TVTest expects.
    last_chunk: Vec<u8>,
    /// Backing storage for the strings returned to TVTest.
    strings: channels::StringCache,
    /// Settings from `<dll>.ini`.
    config: Config,
}

#[repr(C)]
pub struct Obj {
    vtbl: &'static VTable,
    state: Mutex<State>,
}

/// Run an entry point's body with panics caught. A panic that reached the
/// `extern` boundary would abort TVTest; here it is logged (the panic hook
/// writes the message and location first) and the call answers `on_panic`:
/// FALSE, 0, 0.0 or a null string, which TVTest treats as a failed call. The
/// state mutex may be poisoned afterwards; [`LockExt::locked`] goes on
/// regardless, so later calls still work with whatever state was left.
fn guard<R>(name: &str, on_panic: R, f: impl FnOnce() -> R) -> R {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(r) => r,
        Err(_) => {
            log::error!("{name}: a panic in the DLL was caught at the boundary; the call fails");
            on_panic
        }
    }
}

/// Log panics through `log` (TVTest has no console to show stderr), then
/// let the previous hook have them too. Installed once per process.
fn install_panic_hook() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            log::error!("panic in BonDriver_DTH10: {info}");
            previous(info);
        }));
    });
}

/// The entry point's body under [`guard`], with the object's state locked.
fn with_state<R>(this: *mut Obj, name: &str, on_panic: R, f: impl FnOnce(&mut State) -> R) -> R {
    guard(name, on_panic, || {
        // SAFETY: `this` is the object we handed out from CreateBonDriver.
        let obj = unsafe { &*this };
        let mut guard = obj.state.locked();
        f(&mut guard)
    })
}

/// Entry point looked up by TVTest with `GetProcAddress`.
#[unsafe(no_mangle)]
pub extern "C" fn CreateBonDriver() -> *mut c_void {
    guard("CreateBonDriver", std::ptr::null_mut(), || {
        let config = Config::load(CreateBonDriver as *const ());
        init_logging(&config);
        install_panic_hook();
        log::info!(
            "BonDriver_DTH10 {} ini {:?} mode {:?} spaces {:?}",
            env!("CARGO_PKG_VERSION"),
            config.ini_path,
            config.tuner.stream_mode,
            config.spaces
        );
        let obj = Box::new(Obj {
            vtbl: &VTABLE.vt,
            state: Mutex::new(State {
                backend: None,
                queue: TsQueue::new(),
                cur_space: 0,
                cur_channel: 0,
                last_chunk: Vec::new(),
                strings: channels::StringCache::default(),
                config,
            }),
        });
        Box::into_raw(obj) as *mut c_void
    })
}

/// `[Log] Level` / `[Log] File` from the ini; `RUST_LOG` overrides the level.
fn init_logging(cfg: &Config) {
    let mut b = env_logger::Builder::new();
    b.parse_filters(&cfg.log_level);
    if let Ok(env) = std::env::var("RUST_LOG") {
        b.parse_filters(&env);
    }
    if !cfg.log_file.is_empty()
        && let Ok(f) = std::fs::OpenOptions::new().create(true).append(true).open(&cfg.log_file)
    {
        b.target(env_logger::Target::Pipe(Box::new(f)));
    }
    let _ = b.try_init();
}

bon_abi!(
    fn open_tuner(this: *mut Obj) -> Bool {
        with_state(this, "OpenTuner", 0, |st| {
            if st.backend.is_some() {
                return 1;
            }
            match client::Client::open(&st.config, st.queue.clone()) {
                Ok(b) => {
                    st.backend = Some(b);
                    1
                }
                Err(e) => {
                    log::error!("OpenTuner failed: {e}");
                    0
                }
            }
        })
    }
);

bon_abi!(
    fn close_tuner(this: *mut Obj) {
        with_state(this, "CloseTuner", (), |st| {
            if let Some(mut b) = st.backend.take() {
                b.close();
            }
            st.queue.purge();
        })
    }
);

bon_abi!(
    fn set_channel_byte(this: *mut Obj, ch: u8) -> Bool {
        // Legacy API: channel index within space 0 (terrestrial).
        // SAFETY: same object pointer, same contract.
        unsafe { set_channel2(this, 0, ch as Dword) }
    }
);

bon_abi!(
    fn get_signal_level(this: *mut Obj) -> f32 {
        with_state(this, "GetSignalLevel", 0.0, |st| st.backend.as_mut().map_or(0.0, |b| b.signal_level()))
    }
);

bon_abi!(
    fn wait_ts_stream(this: *mut Obj, timeout_ms: Dword) -> Dword {
        guard("WaitTsStream", 0, || {
            let queue = with_state(this, "WaitTsStream", None, |st| Some(st.queue.clone()));
            // Do not hold the state lock while waiting.
            match queue {
                Some(q) if q.wait(timeout_ms) => 1,
                _ => 0,
            }
        })
    }
);

bon_abi!(
    fn get_ready_count(this: *mut Obj) -> Dword {
        with_state(this, "GetReadyCount", 0, |st| st.queue.len() as Dword)
    }
);

bon_abi!(
    fn get_ts_stream_pp(this: *mut Obj, pp_dst: *mut *mut u8, p_size: *mut Dword, p_remain: *mut Dword) -> Bool {
        with_state(this, "GetTsStream", 0, |st| {
            let Some(chunk) = st.queue.pop() else {
                // SAFETY: pointers supplied by TVTest.
                unsafe {
                    if !p_size.is_null() {
                        *p_size = 0;
                    }
                    if !p_remain.is_null() {
                        *p_remain = 0;
                    }
                }
                return 0;
            };
            st.last_chunk = chunk;
            // SAFETY: pointers supplied by TVTest; `last_chunk` outlives the call.
            unsafe {
                if !pp_dst.is_null() {
                    *pp_dst = st.last_chunk.as_mut_ptr();
                }
                if !p_size.is_null() {
                    *p_size = st.last_chunk.len() as Dword;
                }
                if !p_remain.is_null() {
                    *p_remain = st.queue.len() as Dword;
                }
            }
            1
        })
    }
);

bon_abi!(
    fn get_ts_stream_p(this: *mut Obj, p_dst: *mut u8, p_size: *mut Dword, p_remain: *mut Dword) -> Bool {
        guard("GetTsStream", 0, || {
            let mut tmp: *mut u8 = std::ptr::null_mut();
            let mut size: Dword = 0;
            // SAFETY: same object pointer, same contract.
            let r = unsafe { get_ts_stream_pp(this, &mut tmp, &mut size, p_remain) };
            if r != 0 && !p_dst.is_null() && !tmp.is_null() {
                // SAFETY: TVTest guarantees `p_dst` holds at least `*p_size` bytes
                // (it sizes it from GetTsStream's contract of 188-byte multiples).
                unsafe { std::ptr::copy_nonoverlapping(tmp, p_dst, size as usize) };
            }
            // SAFETY: pointer supplied by TVTest.
            unsafe {
                if !p_size.is_null() {
                    *p_size = size;
                }
            }
            r
        })
    }
);

bon_abi!(
    fn purge_ts_stream(this: *mut Obj) {
        with_state(this, "PurgeTsStream", (), |st| st.queue.purge())
    }
);

bon_abi!(
    fn release(this: *mut Obj) {
        // SAFETY: `this` came from Box::into_raw in CreateBonDriver.
        unsafe { close_tuner(this) };
        // The box is taken back exactly once: a panic while dropping it (the
        // tuner's close, a thread join) is caught here and nothing touches
        // `this` again, so there is no double free.
        guard("Release", (), || drop(unsafe { Box::from_raw(this) }));
    }
);

bon_abi!(
    fn get_tuner_name(this: *mut Obj) -> Pcwstr {
        with_state(this, "GetTunerName", std::ptr::null(), |st| st.strings.wide("BUFFALO DTH10 series (dth10-rs)"))
    }
);

bon_abi!(
    fn is_tuner_opening(this: *mut Obj) -> Bool {
        with_state(this, "IsTunerOpening", 0, |st| st.backend.is_some() as Bool)
    }
);

/// The tuning space behind TVTest's space index (`[Device] Spaces` may hide some).
fn real_space(st: &State, space: Dword) -> Option<u32> {
    st.config.spaces.get(space as usize).copied()
}

bon_abi!(
    fn enum_tuning_space(this: *mut Obj, space: Dword) -> Pcwstr {
        with_state(this, "EnumTuningSpace", std::ptr::null(), |st| {
            match real_space(st, space).and_then(channels::space_name) {
                Some(n) => st.strings.wide(n),
                None => std::ptr::null(),
            }
        })
    }
);

bon_abi!(
    fn enum_channel_name(this: *mut Obj, space: Dword, ch: Dword) -> Pcwstr {
        with_state(this, "EnumChannelName", std::ptr::null(), |st| {
            match real_space(st, space).and_then(|s| channels::channel(s, ch)) {
                Some(c) => st.strings.wide(&c.name),
                None => std::ptr::null(),
            }
        })
    }
);

bon_abi!(
    fn set_channel2(this: *mut Obj, space: Dword, ch: Dword) -> Bool {
        with_state(this, "SetChannel", 0, |st| {
            let Some(c) = real_space(st, space).and_then(|s| channels::channel(s, ch)) else { return 0 };
            let Some(b) = st.backend.as_mut() else { return 0 };
            match b.set_channel(&c) {
                Ok(()) => {
                    st.cur_space = space;
                    st.cur_channel = ch;
                    st.queue.purge();
                    1
                }
                Err(e) => {
                    log::error!("SetChannel({space},{ch}) failed: {e}");
                    0
                }
            }
        })
    }
);

bon_abi!(
    fn get_cur_space(this: *mut Obj) -> Dword {
        with_state(this, "GetCurSpace", 0, |st| st.cur_space)
    }
);

bon_abi!(
    fn get_cur_channel(this: *mut Obj) -> Dword {
        with_state(this, "GetCurChannel", 0, |st| st.cur_channel)
    }
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_panic_is_caught_and_the_call_fails() {
        assert_eq!(guard("test", 7, || 1), 1);
        assert_eq!(guard("test", 7, || -> i32 { panic!("boom") }), 7);
        assert!(guard("test", std::ptr::null::<u16>(), || -> *const u16 { panic!("boom") }).is_null());
    }

    /// The object TVTest gets: every vtable call works on it, and a panic in
    /// one call (here provoked through the state lock) does not unwind out.
    #[test]
    fn object_survives_calls_and_a_panic() {
        let p = CreateBonDriver() as *mut Obj;
        assert!(!p.is_null());
        // SAFETY: a live object from CreateBonDriver.
        unsafe {
            assert_eq!(is_tuner_opening(p), 0);
            assert!(!get_tuner_name(p).is_null());
            assert!(!enum_tuning_space(p, 0).is_null());
            assert!(enum_tuning_space(p, 99).is_null());
            assert_eq!(get_ready_count(p), 0);
            assert_eq!(wait_ts_stream(p, 0), 0);
            assert_eq!(set_channel2(p, 0, 0), 0, "no tuner open: SetChannel fails");
            // a panic inside with_state is caught and answers on_panic
            assert_eq!(with_state(p, "test", 5, |_| -> i32 { panic!("inside") }), 5);
            // the state is still usable afterwards (poisoned lock tolerated)
            assert_eq!(get_cur_space(p), 0);
            release(p);
        }
    }

    /// `cargo test --features rtti-check`: the MSVC runtime's own
    /// `dynamic_cast<IBonDriver2*>` on our hand-made RTTI, and the IBonDriver2
    /// slots, from C++ (tests/rtti_check.cpp).
    #[cfg(feature = "rtti-check")]
    #[test]
    fn msvc_dynamic_cast_finds_ibondriver2() {
        unsafe extern "C" {
            fn dth10_rtti_check() -> i32;
        }
        // SAFETY: a plain C function of ours, built by build.rs.
        let failed_step = unsafe { dth10_rtti_check() };
        assert_eq!(failed_step, 0, "step {failed_step} of the C++ check failed (2 = dynamic_cast returned null)");
    }
}
