//! Clock reads, sleeps and local-time conversion, all through glibc so the
//! vclock shim can interpose them.  Port of `chelper.get_monotonic`
//! (`klippy/chelper/pyhelper.c`), the `time.sleep` path used by
//! `reactor._sys_pause` (`klippy/reactor.py`), and the CPython
//! `time.localtime` / `time.strftime` / `time.asctime` conversions
//! (`localtime_r` / `strftime` in the C locale, honouring `TZ`).

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
    #[link_name = "tzset"]
    fn raw_tzset();
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
    // `time_t t = time`: a 64-bit conversion on x86_64, a 32-bit one on the
    // KE (MIPS32 glibc 2.29, 32-bit `time_t`).
    let sec: libc::time_t = if std::mem::size_of::<libc::time_t>() == 8 {
        cconv::f64_to_i64(time) as libc::time_t
    } else {
        cconv::f64_to_i32(time) as libc::time_t
    };
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

/// Broken-down time (`struct tm`), as filled by [`localtime`].
pub type Tm = libc::tm;

/// `time.tzset()`: re-read `TZ`. `localtime_r` reads `TZ` only on its
/// first call, like a CPython process reads it once at `time` import.
pub fn tzset() {
    // SAFETY: glibc tzset has no arguments; it updates libc globals.
    unsafe { raw_tzset() }
}

/// `time.localtime(t)`: `t` is floored to whole seconds
/// (`_PyTime_ObjectToTime_t(.., _PyTime_ROUND_FLOOR)`) and converted by
/// glibc `localtime_r` in the `TZ` zone. `None` where CPython raises
/// (non-finite or out-of-range `t`, `localtime_r` failure).
pub fn localtime(t: f64) -> Option<Tm> {
    let floor = t.floor();
    // CPython's `time_t` range check; `time_t` is 32-bit on the KE target.
    // A 64-bit `time_t::MAX` rounds up to 2^63, which saturates in the
    // cast below and then fails in `localtime_r` like in CPython.
    if !floor.is_finite() || floor < libc::time_t::MIN as f64 || floor > libc::time_t::MAX as f64 {
        return None;
    }
    let secs = floor as libc::time_t;
    // SAFETY: an all-zero `tm` is valid out storage (`tm_zone` NULL).
    let mut tm: Tm = unsafe { std::mem::zeroed() };
    // SAFETY: `secs` and `tm` are valid for the duration of the call.
    let got = unsafe { libc::localtime_r(&secs, &mut tm) };
    if got.is_null() { None } else { Some(tm) }
}

/// `time.strftime(format, tm)`: glibc `strftime` in the C locale. Like
/// CPython the buffer starts at 1024 bytes and doubles while the result
/// is empty, up to `256 * len(format)`.
pub fn strftime(format: &str, tm: &Tm) -> String {
    let Ok(cformat) = std::ffi::CString::new(format) else {
        // CPython raises `ValueError: embedded null character`; every
        // caller passes a literal format.
        panic!("strftime format {format:?} contains a NUL byte");
    };
    let mut size = 1024usize;
    loop {
        let mut buf = vec![0u8; size];
        // SAFETY: `buf` holds `size` bytes, `cformat` is NUL-terminated
        // and `tm` is a valid `struct tm`.
        let n = unsafe { libc::strftime(buf.as_mut_ptr().cast(), size, cformat.as_ptr(), tm) };
        if n > 0 || size >= 256 * format.len() {
            buf.truncate(n);
            return String::from_utf8_lossy(&buf).into_owned();
        }
        size += size;
    }
}

/// `time.asctime(tm)`: CPython's own `_asctime`
/// (`"%s %s%3d %.2d:%.2d:%.2d %d"`), not glibc `asctime`.
pub fn asctime(tm: &Tm) -> String {
    const WEEKDAYS: [&str; 7] = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"];
    const MONTHS: [&str; 12] =
        ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
    format!(
        "{} {}{:3} {:02}:{:02}:{:02} {}",
        WEEKDAYS[tm.tm_wday as usize],
        MONTHS[tm.tm_mon as usize],
        tm.tm_mday,
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec,
        1900 + i64::from(tm.tm_year)
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timespec_truncates_like_fill_time() {
        // C `fill_time` truncates both casts toward zero: 999 ns stored
        // as 998.99... becomes 998, and -0.5 keeps a negative nsec.
        let cases = [(0., 0, 0), (1.5, 1, 500_000_000), (123.000000999, 123, 998), (-0.5, 0, -500_000_000)];
        for (t, sec, nsec) in cases {
            assert_eq!(timespec_from_f64(t), Timespec { tv_sec: sec, tv_nsec: nsec }, "t={t}");
        }
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
