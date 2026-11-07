//! OS layer interposable by the virtual clock.
//!
//! The only crate allowed to touch time, blocking and threads (see
//! `klippy-rs/ARCHITECTURE.md` platform rules).  Everything goes through
//! glibc symbols via `libc` so the vclock shim can interpose them; never
//! std futex primitives, `std::thread` or `Instant`.
//!
//! Modules:
//! - [`time`]: `monotonic` (port of `chelper.get_monotonic` /
//!   `reactor.monotonic`), wall time, sleeps.  Virtual monotonic time
//!   starts at 0; `CLOCK_REALTIME = epoch + monotonic` under the shim.
//! - [`thread`]: spawn and join over `pthread_create` / `pthread_join`,
//!   names via `prctl(PR_SET_NAME)` like `pyhelper.c set_thread_name`,
//!   plus pthread TLS (`[`thread::tls`]`) used by `reactor` greenlets
//!   to learn their greenlet id.
//! - [`sync`]: [`sync::Mutex`], [`sync::Condvar`] (CLOCK_MONOTONIC
//!   condattr), [`sync::Semaphore`] / [`sync::Lock`] over `sem_*`
//!   (the `threading.Lock` / semaphore equivalent), and a
//!   `queue.Queue`-like [`sync::Channel`].
//! - [`fd`]: nonblocking helpers, poll, epoll, pipes, raw read/write.
//! - [`pty`]: port of `util.create_pty`.
//! - [`termios`]: termios helpers used by `serialhdl` / `util`.
//! - [`socket`]: unix sockets.
//! - [`libm`]: `extern "C"` declarations into the system libm through
//!   `kos::libm`, never Rust intrinsics, so results match glibc 2.29
//!   bit for bit.  (`sqrt` may use `f64::sqrt`, correctly rounded.)
//! - [`cconv`]: C cast helpers reproducing the reference C behaviour
//!   for double to int64/uint64/int32/uint32 including out-of-range and
//!   negative inputs on x86_64 SSE2 (cvttsd2si semantics).

#![forbid(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]

pub mod cconv;
pub mod fd;
pub mod libm;
pub mod pty;
pub mod socket;
pub mod sync;
pub mod termios;
pub mod thread;
pub mod time;

/// Errors returned by the raw fd helpers.
pub type IoResult<T> = std::io::Result<T>;


