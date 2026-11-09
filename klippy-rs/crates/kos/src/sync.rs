//! Synchronization over pthread/sem, mirroring the `threading` and
//! `queue` usage in the reference host.
//!
//! `Condvar` uses a CLOCK_MONOTONIC condattr so timeouts agree with
//! [`crate::time::monotonic`] under the vclock shim.

use std::collections::VecDeque;
use std::ops::{Deref, DerefMut};
use std::os::raw::{c_int, c_void};
use std::sync::Arc;

use crate::time;

// SAFETY: glibc pthread/sem; see each use.
unsafe extern "C" {
    fn pthread_mutex_init(m: *mut c_void, attr: *const c_void) -> c_int;
    fn pthread_mutex_destroy(m: *mut c_void) -> c_int;
    fn pthread_mutex_lock(m: *mut c_void) -> c_int;
    fn pthread_mutex_unlock(m: *mut c_void) -> c_int;
    fn pthread_mutex_trylock(m: *mut c_void) -> c_int;
    fn pthread_condattr_init(a: *mut c_void) -> c_int;
    fn pthread_condattr_setclock(a: *mut c_void, id: c_int) -> c_int;
    fn pthread_condattr_destroy(a: *mut c_void) -> c_int;
    fn pthread_cond_init(c: *mut c_void, attr: *const c_void) -> c_int;
    fn pthread_cond_destroy(c: *mut c_void) -> c_int;
    fn pthread_cond_signal(c: *mut c_void) -> c_int;
    fn pthread_cond_broadcast(c: *mut c_void) -> c_int;
    fn pthread_cond_wait(c: *mut c_void, m: *mut c_void) -> c_int;
    fn pthread_cond_timedwait(
        c: *mut c_void,
        m: *mut c_void,
        abs: *const time::Timespec,
    ) -> c_int;
    fn sem_init(s: *mut c_void, pshared: c_int, value: u32) -> c_int;
    fn sem_destroy(s: *mut c_void) -> c_int;
    fn sem_wait(s: *mut c_void) -> c_int;
    fn sem_trywait(s: *mut c_void) -> c_int;
    fn sem_post(s: *mut c_void) -> c_int;
    fn sem_getvalue(s: *mut c_void, out: *mut c_int) -> c_int;
}

#[repr(C, align(8))]
struct RawMutex {
    _priv: [u8; 40],
}

#[repr(C, align(8))]
struct RawCond {
    _priv: [u8; 48],
}

#[repr(C, align(8))]
struct RawSem {
    _priv: [u8; 32],
}

fn check(rc: c_int, what: &str) {
    assert_eq!(rc, 0, "{what} failed: {rc}");
}

/// RAII guard for [`Mutex`]; unlocks on drop like `threading.Lock`.
/// The guard is `!Send` (a locked mutex must not move threads).
pub struct MutexGuard<'a, T> {
    parent: &'a Mutex<T>,
    _not_send: *mut (),
}

impl<'a, T> Deref for MutexGuard<'a, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: lock held and mutex alive for `'a`.
        unsafe { &*self.parent.data.get() }
    }
}
impl<'a, T> DerefMut for MutexGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: `&mut self` guarantees exclusive access.
        unsafe { &mut *self.parent.data.get() }
    }
}
impl<'a, T> Drop for MutexGuard<'a, T> {
    fn drop(&mut self) {
        // SAFETY: lock held, parent alive.
        unsafe {
            check(pthread_mutex_unlock(self.parent.raw_mut()), "unlock");
        }
    }
}

/// Mutual exclusion over `pthread_mutex_*` (non-recursive). Equivalent
/// to `threading.Lock` guarded with the reactor. Also the building
/// block for [`Channel`] and [`crate::thread::tls`].
pub struct Mutex<T> {
    raw: RawMutex,
    data: std::cell::UnsafeCell<T>,
}

// SAFETY: pthread mutex + UnsafeCell gives the same contract as
// `std::sync::Mutex` when `T: Send`.
unsafe impl<T: Send> Send for Mutex<T> {}
unsafe impl<T: Send> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    /// Create a new unlocked mutex.
    pub fn new(data: T) -> Self {
        let m = Mutex { raw: RawMutex { _priv: [0; 40] }, data: std::cell::UnsafeCell::new(data) };
        // SAFETY: valid storage, default attr.
        unsafe {
            check(pthread_mutex_init(m.raw_mut(), std::ptr::null()), "init mutex");
        }
        m
    }
    fn raw_mut(&self) -> *mut c_void {
        // `self` is alive; the raw storage is address-stable via `&self`.
        (&self.raw as *const RawMutex) as *mut c_void
    }
    /// Lock, blocking. Spurious absence of wakeups is impossible here.
    pub fn lock(&self) -> MutexGuard<'_, T> {
        // SAFETY: valid mutex.
        unsafe {
            check(pthread_mutex_lock(self.raw_mut()), "lock mutex");
        }
        MutexGuard { parent: self, _not_send: std::ptr::null_mut() }
    }
    /// Try to lock without blocking.
    pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
        let rc = unsafe { pthread_mutex_trylock(self.raw_mut()) };
        if rc == 0 {
            Some(MutexGuard { parent: self, _not_send: std::ptr::null_mut() })
        } else {
            None
        }
    }
}

impl<T> Drop for Mutex<T> {
    fn drop(&mut self) {
        // SAFETY: unshared by `&mut self`.
        unsafe {
            pthread_mutex_destroy((&mut self.raw as *mut RawMutex) as *mut c_void);
        }
    }
}

/// Condition variable over `pthread_cond_*` with a CLOCK_MONOTONIC
/// condattr, so [`wait_timeout`] measures the same clock the shim
/// interposes. Waiters MUST re-check their predicate (spurious
/// wakeups are part of the contract).
///
/// The `pthread_cond_t` lives in its own allocation and is initialised
/// there: POSIX leaves a moved condvar undefined, and the virtual-clock
/// shim records the condattr clock per condvar address.
pub struct Condvar {
    raw: Box<std::cell::UnsafeCell<RawCond>>,
}

// SAFETY: condvar shared between threads.
unsafe impl Send for Condvar {}
unsafe impl Sync for Condvar {}

impl Condvar {
    /// Create a new condvar with CLOCK_MONOTONIC timeouts.
    pub fn new() -> Self {
        let cb = Condvar { raw: Box::new(std::cell::UnsafeCell::new(RawCond { _priv: [0; 48] })) };
        // SAFETY: default attr with monotonic clock; the condvar is
        // initialised at its final heap address.
        unsafe {
            let mut attr = [0u8; 56];
            check(pthread_condattr_init(attr.as_mut_ptr() as *mut c_void), "condattr init");
            check(
                pthread_condattr_setclock(attr.as_mut_ptr() as *mut c_void, time::CLOCK_MONOTONIC),
                "condattr setclock",
            );
            check(pthread_cond_init(cb.raw_mut(), attr.as_ptr() as *const c_void), "cond init");
            pthread_condattr_destroy(attr.as_mut_ptr() as *mut c_void);
        }
        cb
    }
    fn raw_mut(&self) -> *mut c_void {
        self.raw.get() as *mut c_void
    }
    /// Block until signalled. Returns the guard; re-check predicate.
    pub fn wait<'a, T>(&self, guard: MutexGuard<'a, T>) -> MutexGuard<'a, T> {
        // SAFETY: guard holds the lock; cond/mutex are both valid.
        unsafe {
            let m = guard.parent.raw_mut();
            check(pthread_cond_wait(self.raw_mut(), m), "cond wait");
        }
        guard
    }
    /// Block until signalled or `secs` seconds elapse. Returns the guard;
    /// the caller re-checks its predicate. A non-positive or NaN `secs`
    /// returns at once.
    pub fn wait_timeout<'a, T>(&self, guard: MutexGuard<'a, T>, secs: f64) -> MutexGuard<'a, T> {
        if secs.is_nan() || secs <= 0. {
            return guard;
        }
        // The deadline is absolute on the condattr clock. `time::monotonic()`
        // reads `CLOCK_MONOTONIC_RAW`, which `pthread_condattr_setclock`
        // rejects, so the deadline must come from `CLOCK_MONOTONIC` itself.
        let abs = time::timespec_from_f64(time::clock_gettime_f64(time::CLOCK_MONOTONIC) + secs);
        // SAFETY: valid objects; `abs` outlives the call.
        let rc = unsafe {
            let m = guard.parent.raw_mut();
            pthread_cond_timedwait(self.raw_mut(), m, &abs as *const time::Timespec)
        };
        if rc != libc::ETIMEDOUT {
            check(rc, "cond timedwait");
        }
        guard
    }
    /// Wake one waiter.
    pub fn notify_one(&self) {
        // SAFETY: valid condvar.
        unsafe {
            pthread_cond_signal(self.raw_mut());
        }
    }
    /// Wake all waiters.
    pub fn notify_all(&self) {
        // SAFETY: valid condvar.
        unsafe {
            pthread_cond_broadcast(self.raw_mut());
        }
    }
}

impl Default for Condvar {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Condvar {
    fn drop(&mut self) {
        // SAFETY: unshared by `&mut self`.
        unsafe {
            pthread_cond_destroy(self.raw_mut());
        }
    }
}

/// Counting semaphore over `sem_*`. A value-1 semaphore is a
/// `threading.Lock` (see [`Lock`]).
pub struct Semaphore {
    raw: RawSem,
}

// SAFETY: sem is thread-safe.
unsafe impl Send for Semaphore {}
unsafe impl Sync for Semaphore {}

impl Semaphore {
    /// Create with `initial` (must be >= 0).
    pub fn new(initial: u32) -> Self {
        let mut s = Semaphore { raw: RawSem { _priv: [0; 32] } };
        // SAFETY: valid storage, process-local.
        unsafe {
            check(sem_init((&mut s.raw as *mut RawSem) as *mut c_void, 0, initial), "sem_init");
        }
        s
    }
    fn raw_mut(&self) -> *mut c_void {
        (&self.raw as *const RawSem) as *mut c_void
    }
    /// Block until the count is positive, then decrement.
    pub fn acquire(&self) {
        loop {
            // SAFETY: valid sem.
            let rc = unsafe { sem_wait(self.raw_mut()) };
            if rc != 0 {
                let e = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if e == libc::EINTR {
                    continue;
                }
                check(rc, "sem_wait");
            }
            return;
        }
    }
    /// Decrement without blocking; `false` if not available.
    pub fn try_acquire(&self) -> bool {
        // SAFETY: valid sem.
        let rc = unsafe { sem_trywait(self.raw_mut()) };
        rc == 0
    }
    /// Unlock / increment.
    pub fn release(&self) {
        // SAFETY: valid sem.
        unsafe {
            sem_post(self.raw_mut());
        }
    }
    /// Current count.
    pub fn value(&self) -> i32 {
        let mut v: c_int = 0;
        // SAFETY: valid sem.
        unsafe {
            sem_getvalue(self.raw_mut(), &mut v as *mut c_int);
        }
        v
    }
}

impl Drop for Semaphore {
    fn drop(&mut self) {
        // SAFETY: unshared by `&mut self`.
        unsafe {
            sem_destroy((&mut self.raw as *mut RawSem) as *mut c_void);
        }
    }
}

/// Binary semaphore: a `threading.Lock` over `sem_*`.
pub struct Lock {
    sem: Semaphore,
}
impl Lock {
    /// Create an unlocked lock.
    pub fn new() -> Self {
        Lock { sem: Semaphore::new(1) }
    }
    /// Acquire (blocking).
    pub fn acquire(&self) {
        self.sem.acquire();
    }
    /// Try to acquire without blocking.
    pub fn try_acquire(&self) -> bool {
        self.sem.try_acquire()
    }
    /// Release.
    pub fn release(&self) {
        self.sem.release();
    }
    /// `true` when unlocked (like `Lock.locked()`).
    pub fn locked(&self) -> bool {
        self.sem.value() == 0
    }
}
impl Default for Lock {
    fn default() -> Self {
        Self::new()
    }
}

/// One-shot `Once` over `pthread_once`-free Rust init; mirrors
/// `functools.lru_cache` / import-once needs in the host.
pub struct Once {
    state: std::sync::atomic::AtomicUsize,
}

impl Once {
    /// Const constructor.
    pub const fn const_new() -> Self {
        Once { state: std::sync::atomic::AtomicUsize::new(0) }
    }
}
impl Once {
    /// Create uninitialised.
    pub fn new() -> Self {
        Once { state: std::sync::atomic::AtomicUsize::new(0) }
    }
    /// Run `f` once (other callers wait while it runs).
    pub fn call_once<F: FnOnce()>(&self, f: F) {
        use std::sync::atomic::Ordering;
        if self.state.load(Ordering::Acquire) == 2 {
            return;
        }
        // Serialise via CAS loop with a spin-wait on the running
        // state; the critical section is short and never sleeps
        // (no blocking while holding it), so this is deadlock-free.
        loop {
            match self.state.compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => {
                    f();
                    self.state.store(2, Ordering::Release);
                    return;
                }
                Err(2) => return,
                Err(_) => std::hint::spin_loop(),
            }
        }
    }
}
impl Default for Once {
    fn default() -> Self {
        Self::new()
    }
}

struct ChannelInner<T> {
    mu: Mutex<(VecDeque<T>, bool)>,
    cv: Condvar,
}

/// `queue.Queue`-like channel (unbounded, FIFO) over kos sync.
/// `put` never blocks (like `queue.Queue()` unbounded), `get`
/// blocks, `get_timeout` returns `None` on timeout (like
/// `queue.Empty`).
#[allow(dead_code)]
pub struct Channel<T> {
    tx: Arc<ChannelInner<T>>,
}

/// Sender half of a [`Channel`]; clonable.
#[derive(Clone)]
pub struct Sender<T> {
    inner: Arc<ChannelInner<T>>,
}

/// Receiver half of a [`Channel`].
pub struct Receiver<T> {
    inner: Arc<ChannelInner<T>>,
}

/// Create an unbounded [`Channel`] (like `queue.Queue()`).
pub fn channel<T>() -> (Sender<T>, Receiver<T>) {
    let inner = Arc::new(ChannelInner { mu: Mutex::new((VecDeque::new(), false)), cv: Condvar::new() });
    (Sender { inner: Arc::clone(&inner) }, Receiver { inner })
}

impl<T> Sender<T> {
    /// Put `v` at the tail; never blocks.
    pub fn put(&self, v: T) {
        let mut q = self.inner.mu.lock();
        q.0.push_back(v);
        drop(q);
        self.inner.cv.notify_all();
    }
    /// Alias for [`put`] (matches the name used by queuelogger).
    pub fn send(&self, v: T) {
        self.put(v);
    }
    /// Number of queued items (`qsize()`).
    pub fn qsize(&self) -> usize {
        self.inner.mu.lock().0.len()
    }
    /// `true` when empty (`empty()`).
    pub fn empty(&self) -> bool {
        self.inner.mu.lock().0.is_empty()
    }
}

impl<T> Receiver<T> {
    /// Take the head, blocking. Returns `None` when closed and empty.
    pub fn get(&self) -> Option<T> {
        let mut q = self.inner.mu.lock();
        loop {
            if let Some(v) = q.0.pop_front() {
                let v = v;
                drop(q);
                return Some(v);
            }
            if q.1 {
                return None;
            }
            q = self.inner.cv.wait(q);
        }
    }
    /// Take the head, blocking up to `secs` monotonic seconds.
    /// `None` on timeout or closed-and-empty.
    pub fn get_timeout(&self, secs: f64) -> Option<T> {
        let mut q = self.inner.mu.lock();
        let deadline = time::monotonic() + secs;
        loop {
            if let Some(v) = q.0.pop_front() {
                let v = v;
                drop(q);
                return Some(v);
            }
            if q.1 {
                return None;
            }
            let now = time::monotonic();
            let left = deadline - now;
            if left <= 0. {
                return None;
            }
            q = self.inner.cv.wait_timeout(q, left);
        }
    }
    /// Non-blocking take.
    pub fn get_nowait(&self) -> Option<T> {
        let mut q = self.inner.mu.lock();
        q.0.pop_front()
    }
    /// Close the channel; blocked getters return `None`.
    pub fn close(&self) {
        let mut guard = self.inner.mu.lock();
        guard.1 = true;
        drop(guard);
        self.inner.cv.notify_all();
    }
    /// Alias for [`close`].
    pub fn shutdown(&self) {
        self.close();
    }
    /// Number of queued items.
    pub fn qsize(&self) -> usize {
        self.inner.mu.lock().0.len()
    }
    /// `true` when empty.
    pub fn empty(&self) -> bool {
        self.inner.mu.lock().0.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::thread::spawn;

    #[test]
    fn mutex_basic() {
        let m = Mutex::new(40);
        *m.lock() += 2;
        assert_eq!(*m.lock(), 42);
    }

    #[test]
    fn condvar_wake() {
        let shared = Arc::new((Mutex::new(0), Condvar::new()));
        let s2 = Arc::clone(&shared);
        let h = spawn(move || {
            let (mu, cv) = &*s2;
            let mut v = mu.lock();
            *v = 1;
            drop(v);
            cv.notify_all();
        });
        let (mu, cv) = &*shared;
        let mut v = mu.lock();
        while *v == 0 {
            v = cv.wait(v);
        }
        assert_eq!(*v, 1);
        h.join();
    }

    /// A timed wait with no notifier sleeps until its monotonic deadline.
    /// A condvar moved after `pthread_cond_init` lost its monotonic clock
    /// under the virtual-clock shim and returned at once.
    #[test]
    fn condvar_timed_wait_reaches_deadline() {
        let shared = Arc::new((Mutex::new(()), Condvar::new()));
        let (mu, cv) = &*shared;
        let start = crate::time::monotonic();
        let guard = mu.lock();
        let _guard = cv.wait_timeout(guard, 0.25);
        let elapsed = crate::time::monotonic() - start;
        assert!(elapsed >= 0.25, "timed wait returned after {elapsed} s");
    }

    #[test]
    fn semaphore_and_lock() {
        let l = Lock::new();
        assert!(!l.locked());
        assert!(l.try_acquire());
        assert!(l.locked());
        assert!(!l.try_acquire());
        l.release();
        assert!(!l.locked());
    }

    #[test]
    fn channel_fifo() {
        let (sx, rx) = channel::<i32>();
        assert!(rx.empty());
        sx.put(1);
        sx.put(2);
        assert_eq!(rx.qsize(), 2);
        assert_eq!(rx.get(), Some(1));
        assert_eq!(rx.get_nowait(), Some(2));
        assert_eq!(rx.get_nowait(), None);
        assert_eq!(rx.get_timeout(0.001), None);
    }
}
