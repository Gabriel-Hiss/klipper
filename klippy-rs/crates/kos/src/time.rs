//! Clock reads and sleeps, all through glibc so the vclock shim can
//! interpose them.  Port of `chelper.get_monotonic` (`klippy/chelper/pyhelper.c`)
//! and the `time.sleep` path used by `reactor._sys_pause`
//! (`klippy/reactor.py`).

use crate::cconv;
use std::os::raw::c_long;

/// `timespec` as returned by `clock_gettime`.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timespec {
    /// Whole seconds.
    pub tv_sec: libc::time_t,
    /// Nanoseconds within the second.
    pub tv_nsec: libc::c_long,
}

/// `CLOCK_REALTIME`.
pub const CLOCK_REALTIME: libc::c_int = libc::CLOCK_REALTIME;
/// `CLOCK_MONOTONIC`.
pub const CLOCK_MONOTONIC: libc::c_int = libc::CLOCK_MONOTONIC;
/// `CLOCK_MONOTONIC_RAW` (used by the reference `get_monotonic`).
pub const CLOCK_MONOTONIC_RAW: libc::c_int = libc::CLOCK_MONOTONIC_RAW;

// SAFETY: glibc clock/nanosleep; no other contract.
// SAFETY: see below.
unsafe extern "C" {
    fn clock_gettime(clockid: libc::c_int, tp: *mut Timespec) -> libc::c_int;
    fn nanosleep(req: *const Timespec, rem: *mut Timespec) -> libc::c_int;
}

/// Read `clockid` exactly like the C helper: `ts.tv_sec + ts.tv_nsec * 1e-9`.
///
/// On error the reference logs to stderr and returns `0.`; this
/// returns `0.` without logging (documented divergence, unobservable
/// in the parity harness where the clock never fails).
pub fn clock_gettime_f64(clockid: libc::c_int) -> f64 {
    let mut ts = Timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: `ts` is valid out storage; `clockid` is a glibc clock.
    let ret = unsafe { clock_gettime(clockid, &mut ts as *mut Timespec) };
    if ret != 0 {
        return 0.;
    }
    // Exact order of `pyhelper.c`: (double)sec + (double)nsec * .000000001
    ts.tv_sec as f64 + (ts.tv_nsec as f64) * 0.000000001
}

/// Virtual monotonic time. Port of `chelper.get_monotonic`:
/// `CLOCK_MONOTONIC_RAW`, `sec + nsec * .000000001` as `f64`.
pub fn monotonic() -> f64 {
    clock_gettime_f64(CLOCK_MONOTONIC_RAW)
}

/// Wall-clock time (`CLOCK_REALTIME`). Under the vclock shim this is
/// `epoch + monotonic`.
pub fn wall_time() -> f64 {
    clock_gettime_f64(CLOCK_REALTIME)
}

/// Port of the C `fill_time(double)` helper: `time_t t = time`,
/// then `{t, (time - t) * 1000000000.}`.
///
/// The conversions go through [`cconv`] so they share the documented
/// out-of-range behaviour of the reference build.
pub fn timespec_from_f64(time: f64) -> Timespec {
    let sec = cconv::f64_to_i64(time);
    let frac = time - (sec as f64);
    Timespec {
        tv_sec: sec,
        tv_nsec: cconv::f64_to_i64(frac * 1_000_000_000.) as c_long,
    }
}

/// Inverse of [`timespec_from_f64`] for positive times, same formula
/// as [`monotonic`].
pub fn f64_from_timespec(ts: &Timespec) -> f64 {
    ts.tv_sec as f64 + (ts.tv_nsec as f64) * 0.000000001
}

/// Sleep `secs` seconds via `nanosleep`, restarting after `EINTR`
/// with the remaining time (mirrors PEP 475 retry of `time.sleep`).
/// Non-positive delays return immediately.
pub fn sleep(secs: f64) {
    if secs <= 0. || secs.is_nan() {
        return;
    }
    let mut req = timespec_from_f64(secs);
    loop {
        let mut rem = Timespec { tv_sec: 0, tv_nsec: 0 };
        // SAFETY: `req`/`rem` are valid storage.
        let ret = unsafe { nanosleep(&req as *const Timespec, &mut rem as *mut Timespec) };
        if ret == 0 {
            return;
        }
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        if errno == libc::EINTR {
            req = rem;
            continue;
        }
        // Any other error (e.g. EINVAL for a negative remainder) ends
        // the sleep, like the Python path which never requests it.
        return;
    }
}

/// Sleep until `waketime` on the monotonic clock. Port of
/// `reactor._sys_pause`: `delay = waketime - monotonic()`, sleep only
/// when `delay > 0`, then return `monotonic()`.
pub fn sleep_until(waketime: f64) -> f64 {
    let delay = waketime - monotonic();
    if delay > 0. {
        sleep(delay);
    }
    monotonic()
}

/// Absolute-expiry helper for condvar timeouts: `monotonic + secs`.
pub fn monotonic_plus(secs: f64) -> f64 {
    monotonic() + secs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timespec_roundtrip() {
        let cases = [0., 1.5, 123.000000001, 123.000000999, -0.5];
        for &t in &cases {
            let ts = timespec_from_f64(t);
            assert_eq!(ts.tv_sec as f64 + (ts.tv_nsec as f64) * 1e-9, t, "t={t}");
        }
        assert_eq!(timespec_from_f64(2.5), Timespec { tv_sec: 2, tv_nsec: 500_000_000 });
    }

    #[test]
    fn monotonic_is_increasing() {
        let a = monotonic();
        let b = monotonic();
        assert!(b >= a);
    }

    #[test]
    fn sleep_nonpositive_returns() {
        sleep(0.);
        sleep(-1.);
    }
}
