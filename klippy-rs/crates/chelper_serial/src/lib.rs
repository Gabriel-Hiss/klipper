//! Port of the klippy/chelper serial code.
//!
//! - [`msgblock`]: `klippy/chelper/msgblock.c` / `msgblock.h` — CRC16-CCITT,
//!   message check/decode, VLQ encode/decode, clock estimates and the
//!   shared `queue_message` / `clock_estimate` types.
//! - [`pollreactor`]: `klippy/chelper/pollreactor.c` / `pollreactor.h` — the
//!   `poll(2)` + timer dispatch loop the serial background thread runs.
//! - [`serialqueue`]: `klippy/chelper/serialqueue.c` / `serialqueue.h` —
//!   command queues, upcoming/ready scheduling by `req_clock` /
//!   `min_clock`, block building (sequence numbers, CRC), retransmit with
//!   RFC6298 RTT estimation, the receive window, fast readers,
//!   notifications, the stats string, debug-file mode and the background
//!   thread. All blocking primitives (threads, mutexes, condvars, `poll`,
//!   clock reads) go through `kos` so the virtual-clock shim in
//!   `parity/vclock` interposes them exactly like the C code.
//!
//! The `errorf` / `report_errno` helpers of `pyhelper.c` are reproduced
//! here as a process-wide logging callback (see
//! [`set_logging_callback`]), matching the C global
//! `python_logging_callback`.

pub mod msgblock;
pub mod pollreactor;
pub mod serialqueue;

pub use msgblock::{clock_from_clock32, clock_from_time, clock_to_time, ClockEstimate,
                   QueueMessage, MESSAGE_DEST, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_PAYLOAD_MAX,
                   MESSAGE_SEQ_MASK, MESSAGE_SYNC};
pub use pollreactor::{PR_NEVER, PR_NOW, PollReactor};
pub use serialqueue::{alloc_commandqueue, free_commandqueue, FastReaderCb, PullQueueMessage,
                      SerialQueue, BACKGROUND_PRIORITY_CLOCK, MAX_CLOCK};

use std::sync::atomic::{AtomicUsize, Ordering};

/// Installed logging callback behind an atomic word; the boxed closure is
/// leaked for the process lifetime (the C installs a function pointer
/// into the same global once, through `set_python_logging_callback`).
static LOG_CALLBACK: AtomicUsize = AtomicUsize::new(0);

/// Port of `pyhelper.c set_python_logging_callback`: install the sink
/// for [`errorf`] / [`report_errno`] output. The callback receives the
/// fully formatted message (the C callback receives the same string;
/// the default C sink appends "\n" to stderr).
pub fn set_logging_callback<F>(f: F)
where
    F: Fn(&str) + Send + Sync + 'static,
{
    let boxed = Box::new(f);
    let raw = Box::into_raw(boxed) as usize;
    LOG_CALLBACK.store(raw, Ordering::SeqCst);
}

/// Port of the C `errorf` logger: format output is produced by callers
/// (to match the `printf` formats of the C call sites); this truncates
/// at 511 bytes (the C `buf[512]` in `errorf`), then invokes the
/// installed callback, or writes `msg + "\n"` to stderr (the C
/// `default_logger`).
pub(crate) fn errorf(msg: &str) {
    let truncated = truncate_errorf(msg);
    let raw = LOG_CALLBACK.load(Ordering::SeqCst);
    if raw != 0 {
        // SAFETY: the box is never freed (leaked on install) and is
        // Send+Sync; it stays valid for the process lifetime.
        let cb = unsafe { &*(raw as *const Box<dyn Fn(&str) + Send + Sync>) };
        cb(&truncated);
        return;
    }
    let mut out = truncated.into_bytes();
    out.push(b'\n');
    let _ = kos::fd::write_all(2, &out);
}

fn truncate_errorf(msg: &str) -> String {
    let bytes = msg.as_bytes();
    if bytes.len() <= 511 {
        return msg.to_string();
    }
    // C truncates by running `vsnprintf` into `char buf[512]` (byte
    // truncation, then `buf[511] = '\0'`); a UTF-8 value is decoded
    // the same way a C string truncation would be observed.
    String::from_utf8_lossy(&bytes[..511]).into_owned()
}

/// Port of the C `report_errno`:
/// `errorf("Got error %d in %s: (%d)%s", rc, where, errno, strerror(errno))`.
/// `rc` is the `read`/`write`/`poll` return value of the failing call
/// (typically `-1`); `errno` is the captured OS error number.
pub(crate) fn report_errno(where_: &str, rc: i32, errno_num: i32) {
    let strerror_text = strerror_text(errno_num);
    errorf(&format!("Got error {rc} in {where_}: ({errno_num}){strerror_text}"));
}

/// `strerror(3)` through glibc. `kos::fd` has no errno-to-string
/// wrapper; this formatting-only call needs no virtual-clock
/// interposition (it performs no syscall), so it is kept here and
/// reported to the integration owner as a suggested `kos` addition
/// (`kos::fd::strerror`).
fn strerror_text(errno_num: i32) -> String {
    use std::os::raw::c_int;
    unsafe extern "C" {
        fn strerror(errnum: c_int) -> *mut std::os::raw::c_char;
    }
    // SAFETY: glibc strerror is thread-safe and returns a static
    // string (lw() may return NULL for unknown error numbers).
    let ptr = unsafe { strerror(errno_num) };
    if ptr.is_null() {
        return String::new();
    }
    // SAFETY: `ptr` points at a NUL-terminated static string.
    let cstr = unsafe { std::ffi::CStr::from_ptr(ptr) };
    match cstr.to_str() {
        Ok(s) => s.to_string(),
        // The C `errorf` passes the raw `strerror` bytes to the
        // callback (`%s`); lossy decoding only differs for
        // non-UTF-8 errno strings, which do not occur in glibc.
        Err(_) => cstr.to_string_lossy().into_owned(),
    }
}

pub mod trdispatch;
