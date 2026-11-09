//! Threads over `pthread_create` / `pthread_join`.
//!
//! Port of the C side (`pyhelper.c set_thread_name` via
//! `prctl(PR_SET_NAME)`) and the `threading` usage in
//! `serialhdl.py` / `queuelogger.py`.  All thread start, join and
//! wake go through here so the vclock shim sees the same thread
//! structure.

use std::os::raw::{c_int, c_ulong, c_void};

/// `PR_SET_NAME` for `prctl`.
pub const PR_SET_NAME: c_int = libc::PR_SET_NAME;

/// `PR_GET_NAME` for `prctl`.
pub const PR_GET_NAME: c_int = libc::PR_GET_NAME;

// SAFETY: glibc pthread/prctl; valid pointers only.
unsafe extern "C" {
    fn pthread_create(
        tid: *mut *mut c_void,
        attr: *const c_void,
        start: unsafe extern "C" fn(*mut c_void) -> *mut c_void,
        arg: *mut c_void,
    ) -> c_int;
    fn pthread_join(tid: *mut c_void, ret: *mut *mut c_void) -> c_int;
    fn pthread_detach(tid: *mut c_void) -> c_int;
    // glibc `int prctl(int option, ...)`; the kernel reads the extra
    // arguments as `unsigned long` (32-bit on MIPS32 o32).
    fn prctl(option: c_int, ...) -> c_int;
}

struct Spawned {
    f: Option<Box<dyn FnOnce() + Send>>,
}

unsafe extern "C" fn trampoline(arg: *mut c_void) -> *mut c_void {
    // The pthread ABI cannot unwind. Reactor greenlets transfer their
    // panics to run(); other worker failures terminate only that thread.
    let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // SAFETY: `arg` is a `Box<Spawned>` created in `spawn`.
        let mut spawned = unsafe { Box::from_raw(arg as *mut Spawned) };
        let f = spawned.f.take().expect("spawned closure present");
        f();
    }));
    std::ptr::null_mut()
}

/// Handle to a thread created by [`spawn`]/[`spawn_named`].
///
/// Ownership of the closure crosses the FFI boundary; the box is
/// released by the trampoline.  Dropping the handle without [`join`]
/// detaches the thread (equivalent to Python leaving a thread
/// running at exit).
pub struct JoinHandle {
    tid: usize,
}
impl JoinHandle {
    fn tid_ptr(&self) -> *mut c_void {
        self.tid as *mut c_void
    }

}

impl JoinHandle {
    /// Block until the thread finishes.
    pub fn join(self) {
        let mut ret: *mut c_void = std::ptr::null_mut();
        // SAFETY: valid thread handle.
        unsafe {
            pthread_join(self.tid_ptr(), &mut ret as *mut *mut c_void);
        }
        std::mem::forget(self);
    }
    /// Keep running without waiting; used at process shutdown.
    pub fn detach(self) {
        // SAFETY: valid thread handle.
        unsafe {
            pthread_detach(self.tid_ptr());
        }
        std::mem::forget(self);
    }
}

impl Drop for JoinHandle {
    fn drop(&mut self) {
        // SAFETY: valid handle; detach is idempotent via forget.
        unsafe {
            pthread_detach(self.tid_ptr());
        }
    }
}

// `JoinHandle` is moved across threads in reactor finalize (the
// handle lives in shared reactor state).
// SAFETY: the join/detach handle is only used once.
unsafe impl Send for JoinHandle {}

/// Spawn a thread running `f`.
pub fn spawn<F: FnOnce() + Send + 'static>(f: F) -> JoinHandle {
    let mut tid: *mut c_void = std::ptr::null_mut();
    let boxed: *mut Spawned = Box::into_raw(Box::new(Spawned { f: Some(Box::new(f)) }));
    // SAFETY: `trampoline` handles the box; default attr.
    let rc = unsafe { pthread_create(&mut tid as *mut *mut c_void, std::ptr::null(), trampoline, boxed as *mut c_void) };
    assert_eq!(rc, 0, "pthread_create failed: {rc}");
    JoinHandle { tid: tid as usize }
}

/// Spawn a thread and give it a kernel name via
/// `prctl(PR_SET_NAME)`, like `pyhelper.c set_thread_name`
/// (buffer `char name[16]`, so at most 15 bytes + NUL).
pub fn spawn_named<F: FnOnce() + Send + 'static>(name: &str, f: F) -> JoinHandle {
    let owned = name.to_owned();
    spawn(move || {
        set_thread_name(&owned);
        f();
    })
}

/// `prctl(PR_SET_NAME, name)`. Names longer than 15 bytes are
/// truncated, matching the C `char name[16]` buffer. Returns the
/// raw `prctl` result (`0` on success).
pub fn set_thread_name(name: &str) -> c_int {
    let mut buf = [0u8; 16];
    let bytes = name.as_bytes();
    let n = bytes.len().min(15);
    buf[..n].copy_from_slice(&bytes[..n]);
    let ptr = buf.as_ptr() as c_ulong;
    let zero: c_ulong = 0;
    // SAFETY: `buf` outlives the call and holds a NUL-terminated name.
    unsafe { prctl(PR_SET_NAME, ptr, zero, zero, zero) }
}

/// Read the current thread's kernel name (`PR_GET_NAME`).
/// Returns an empty string on failure.
pub fn current_thread_name() -> String {
    let mut buf = [0u8; 16];
    let ptr = buf.as_mut_ptr() as c_ulong;
    let zero: c_ulong = 0;
    // SAFETY: `buf` is valid 16-byte storage.
    let rc = unsafe { prctl(PR_GET_NAME, ptr, zero, zero, zero) };
    if rc != 0 {
        return String::new();
    }
    let len = buf.iter().position(|&b| b == 0).unwrap_or(16);
    String::from_utf8_lossy(&buf[..len]).into_owned()
}

/// pthread TLS used by `reactor` greenlets to know their greenlet id.
pub mod tls {
    use std::os::raw::{c_int, c_void};

    use crate::sync::Once;

    // SAFETY: glibc pthread key functions.
    unsafe extern "C" {
        fn pthread_key_create(key: *mut u32, destructor: *mut c_void) -> c_int;
        fn pthread_setspecific(key: u32, value: *const c_void) -> c_int;
        fn pthread_getspecific(key: u32) -> *mut c_void;
    }

    static KEY: Once = Once::const_new();
    static mut RAW_KEY: u32 = 0;

    /// Create the TLS key once (idempotent).
    fn key() -> u32 {
        KEY.call_once(|| {
            let mut k: u32 = 0;
            // SAFETY: single-threaded init under `Once`.
            unsafe {
                pthread_key_create(&mut k as *mut u32, std::ptr::null_mut());
                RAW_KEY = k;
            }
        });
        // SAFETY: initialised by `call_once` above.
        unsafe { RAW_KEY }
    }

    /// Current thread's greenlet id. `None` outside reactor greenlets.
    pub fn get() -> Option<usize> {
        // SAFETY: key is created; NULL means unset.
        let v = unsafe { pthread_getspecific(key()) };
        if v.is_null() {
            None
        } else {
            // `set` stores `id + 1` so that id 0 is distinct from NULL.
            Some(v as usize - 1)
        }
    }

    /// Set the current thread's greenlet id.
    pub fn set(id: usize) {
        // SAFETY: key is created.
        unsafe {
            pthread_setspecific(key(), (id + 1) as *const c_void);
        }
    }

    /// Clear the current thread's greenlet id.
    pub fn clear() {
        // SAFETY: key is created.
        unsafe {
            pthread_setspecific(key(), std::ptr::null());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CString;
    use crate::sync::{Mutex, Condvar};

    #[test]
    fn spawn_and_join() {
        let shared = std::sync::Arc::new(Mutex::new(Vec::new()));
        let s2 = std::sync::Arc::clone(&shared);
        let cv = std::sync::Arc::new(Condvar::new());
        let cv2 = std::sync::Arc::clone(&cv);
        let h = spawn(move || {
            s2.lock().push(41);
            cv2.notify_all();
        });
        h.join();
        assert_eq!(*shared.lock(), vec![41]);
    }

    #[test]
    fn thread_name_roundtrip() {
        let name = CString::new("kos-test-thread").unwrap();
        let n = name.to_string_lossy().into_owned();
        let h = spawn(move || {
            set_thread_name(&n);
            assert_eq!(current_thread_name(), n);
        });
        h.join();
    }

    #[test]
    fn tls_roundtrip() {
        tls::set(7);
        assert_eq!(tls::get(), Some(7));
        tls::clear();
        assert_eq!(tls::get(), None);
    }
}
