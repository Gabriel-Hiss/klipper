//! File descriptors: nonblocking helpers (port of `util.set_nonblock`),
//! poll, epoll, pipes and raw read/write. All through glibc so the
//! vclock shim can interpose them.

use std::os::raw::{c_int, c_void};

use crate::IoResult;

// SAFETY: glibc fd/poll/epoll; see each use.
unsafe extern "C" {
    fn fcntl(fd: c_int, cmd: c_int, ...) -> c_int;
    #[link_name = "read"]
    fn raw_read(fd: c_int, buf: *mut c_void, count: usize) -> isize;
    #[link_name = "write"]
    fn raw_write(fd: c_int, buf: *const c_void, count: usize) -> isize;
    #[link_name = "close"]
    fn closeFd(fd: c_int) -> c_int;
    #[link_name = "pipe"]
    fn raw_pipe(fds: *mut c_int) -> c_int;
    #[link_name = "poll"]
    fn raw_poll(fds: *mut PollFd, nfds: libc::nfds_t, timeout: c_int) -> c_int;
    fn epoll_create1(flags: c_int) -> c_int;
    fn epoll_ctl(epfd: c_int, op: c_int, fd: c_int, ev: *const EpollEvent) -> c_int;
    fn epoll_wait(epfd: c_int, evs: *mut EpollEvent, maxev: c_int, timeout: c_int) -> c_int;
}

/// `struct pollfd`.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PollFd {
    /// File descriptor.
    pub fd: c_int,
    /// Requested events.
    pub events: i16,
    /// Returned events.
    pub revents: i16,
}

impl PollFd {
    /// Create for `fd` requesting `events`.
    pub fn new(fd: i32, events: i16) -> Self {
        PollFd { fd, events, revents: 0 }
    }
}

/// `POLLIN`.
pub const POLLIN: i16 = 0x1;
/// `POLLOUT`.
pub const POLLOUT: i16 = 0x4;
/// `POLLHUP`.
pub const POLLHUP: i16 = 0x10;
/// `POLLERR`.
pub const POLLERR: i16 = 0x8;


/// `EPOLLIN`.
pub const EPOLLIN: u32 = 1;
/// `EPOLLOUT`.
pub const EPOLLOUT: u32 = 4;
/// `EPOLLHUP`.
pub const EPOLLHUP: u32 = 16;
/// `EPOLLRDH`.
pub const EPOLLRDH: u32 = 8192;

/// `struct epoll_event` with the target's layout (packed 12 bytes on
/// x86_64, 16 bytes with `data` at offset 8 on MIPS32 o32). The fd is
/// stored in `u64` (`epoll_data_t`).
pub type EpollEvent = libc::epoll_event;

/// Setting an fd non-blocking. Port of `util.set_nonblock`:
/// `fcntl(F_SETFL, fcntl(F_GETFL) | O_NONBLOCK)`.
pub fn set_nonblock(fd: i32) -> IoResult<()> {
    // SAFETY: valid fd or error.
    let flags = unsafe { fcntl(fd, libc::F_GETFL) };
    if flags < 0 {
        return Err(std::io::Error::last_os_error());
    }
    let rc = unsafe { fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) };
    if rc < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// Clear `O_NONBLOCK` (inverse of [`set_nonblock`]).
pub fn clear_nonblock(fd: i32) -> IoResult<()> {
    // SAFETY: valid fd or error.
    let flags = unsafe { fcntl(fd, libc::F_GETFL) };
    if flags < 0 {
        return Err(std::io::Error::last_os_error());
    }
    let rc = unsafe { fcntl(fd, libc::F_SETFL, flags & !libc::O_NONBLOCK) };
    if rc < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// Current `O_NONBLOCK` status.
pub fn is_nonblock(fd: i32) -> IoResult<bool> {
    // SAFETY: valid fd or error.
    let flags = unsafe { fcntl(fd, libc::F_GETFL) };
    if flags < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(flags & libc::O_NONBLOCK != 0)
    }
}

/// Raw `read(2)`; `Err` on failure (callers decide on `EINTR`).
pub fn read(fd: i32, buf: &mut [u8]) -> IoResult<usize> {
    // SAFETY: `buf` valid for `len`.
    let rc = unsafe { raw_read(fd, buf.as_mut_ptr() as *mut c_void, buf.len()) };
    if rc < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(rc as usize)
    }
}

/// Raw `write(2)`; `Err` on failure.
pub fn write(fd: i32, buf: &[u8]) -> IoResult<usize> {
    // SAFETY: `buf` valid for `len`.
    let rc = unsafe { raw_write(fd, buf.as_ptr() as *const c_void, buf.len()) };
    if rc < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(rc as usize)
    }
}

/// Write all of `buf`, looping on short writes (`EIO` and partial
/// writes follow the short-write path). Used for the self-pipe.
pub fn write_all(fd: i32, buf: &[u8]) -> IoResult<()> {
    let mut done = 0;
    while done < buf.len() {
        let n = write(fd, &buf[done..])?;
        if n == 0 {
            return Err(std::io::Error::from(std::io::ErrorKind::WriteZero));
        }
        done += n;
    }
    Ok(())
}

/// Close `fd`, ignoring errors (like Python `os.close` in cleanup).
pub fn close(fd: i32) {
    // SAFETY: fd may be invalid; error ignored.
    unsafe {
        closeFd(fd);
    }
}

/// Create an unnamed pipe. Returns `(read, write)` fds. The caller
/// sets nonblocking flags (the reactor self-pipe sets both).
pub fn pipe() -> IoResult<(i32, i32)> {
    let mut fds = [0i32; 2];
    // SAFETY: `fds` valid out storage.
    let rc = unsafe { raw_pipe(fds.as_mut_ptr() as *mut c_int) };
    if rc < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok((fds[0], fds[1]))
    }
}

/// `poll(2)`, one call. `timeout_ms` is milliseconds (negative waits
/// forever). Returns the number of ready fds with `revents` filled in;
/// an interrupted call returns the `EINTR` error, which the C
/// `pollreactor_run` reports and exits on.
pub fn poll(fds: &mut [PollFd], timeout_ms: i32) -> IoResult<usize> {
    // `poll(NULL, 0, ms)` is the sleep form used with no fds.
    let ptr = if fds.is_empty() { std::ptr::null_mut() } else { fds.as_mut_ptr() };
    // SAFETY: `ptr` is valid for `fds.len()` entries (or NULL with 0).
    let rc = unsafe { raw_poll(ptr, fds.len() as libc::nfds_t, timeout_ms) };
    if rc < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(rc as usize)
    }
}

/// CPython `select.poll().poll(timeout_ms)` (PEP 475): [`poll`] retried
/// after `EINTR` with the timeout recomputed from a `CLOCK_MONOTONIC`
/// deadline (ceiling milliseconds); an expired deadline returns `0`.
pub fn poll_retry(fds: &mut [PollFd], timeout_ms: i32) -> IoResult<usize> {
    let now = || crate::time::clock_gettime_f64(crate::time::CLOCK_MONOTONIC);
    let deadline = (timeout_ms >= 0).then(|| now() + f64::from(timeout_ms) / 1000.);
    let mut ms = timeout_ms;
    loop {
        match poll(fds, ms) {
            Err(e) if e.raw_os_error() == Some(libc::EINTR) => {
                if let Some(deadline) = deadline {
                    let remaining = deadline - now();
                    if remaining < 0. {
                        return Ok(0);
                    }
                    ms = crate::cconv::f64_to_i32((remaining * 1000.).ceil());
                }
            }
            result => return result,
        }
    }
}

/// `strerror(errnum)` from glibc (the text C `report_errno` prints).
/// Unknown numbers give glibc's `"Unknown error N"`.
pub fn strerror(errnum: i32) -> String {
    // SAFETY: glibc `strerror` returns a NUL-terminated string that stays
    // valid until the next `strerror` call in this thread; it is copied
    // before returning.
    let ptr = unsafe { libc::strerror(errnum) };
    if ptr.is_null() {
        return String::new();
    }
    // SAFETY: `ptr` is non-NULL and NUL-terminated.
    unsafe { std::ffi::CStr::from_ptr(ptr) }.to_string_lossy().into_owned()
}

/// Epoll instance. Port of the operations `EPollReactor` uses
/// (`select.epoll`): create, register/modify/unregister, wait.
pub struct Epoll {
    fd: i32,
}

impl Epoll {
    /// Create a new epoll instance (`EPOLL_CLOEXEC`).
    pub fn new() -> IoResult<Self> {
        // SAFETY: glibc epoll_create1.
        let fd = unsafe { epoll_create1(libc::EPOLL_CLOEXEC) };
        if fd < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(Epoll { fd })
        }
    }
    /// The epoll fd.
    pub fn fd(&self) -> i32 {
        self.fd
    }
    /// Register `fd` with `events`.
    pub fn register(&self, fd: i32, events: u32) -> IoResult<()> {
        let ev = EpollEvent { events, u64: fd as u64 };
        // SAFETY: `ev` valid.
        let rc = unsafe { epoll_ctl(self.fd, libc::EPOLL_CTL_ADD, fd, &ev as *const EpollEvent) };
        if rc < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
    /// Modify an existing registration.
    pub fn modify(&self, fd: i32, events: u32) -> IoResult<()> {
        let ev = EpollEvent { events, u64: fd as u64 };
        // SAFETY: `ev` valid.
        let rc = unsafe { epoll_ctl(self.fd, libc::EPOLL_CTL_MOD, fd, &ev as *const EpollEvent) };
        if rc < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
    /// Remove a registration.
    pub fn unregister(&self, fd: i32) -> IoResult<()> {
        // SAFETY: glibc epoll_ctl.
        let rc = unsafe { epoll_ctl(self.fd, libc::EPOLL_CTL_DEL, fd, std::ptr::null()) };
        if rc < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
    /// Wait for events; fills `events` and returns the count.
    /// `timeout_ms` negative waits forever (`epoll_wait(-1)`).
    pub fn wait(&self, events: &mut [EpollEvent], timeout_ms: i32) -> IoResult<usize> {
        if events.is_empty() {
            // Sleep form of epoll_wait.
            // SAFETY: null list only for sleep.
            let rc = unsafe { epoll_wait(self.fd, std::ptr::null_mut(), 0, timeout_ms) };
            if rc < 0 {
                return Err(std::io::Error::last_os_error());
            }
            return Ok(rc as usize);
        }
        loop {
            // SAFETY: `events` valid.
            let rc = unsafe {
                epoll_wait(self.fd, events.as_mut_ptr(), events.len() as c_int, timeout_ms)
            };
            if rc < 0 {
                let e = std::io::Error::last_os_error();
                if e.raw_os_error() == Some(libc::EINTR) {
                    continue;
                }
                return Err(e);
            }
            return Ok(rc as usize);
        }
    }
}

impl Drop for Epoll {
    fn drop(&mut self) {
        // SAFETY: owned fd.
        unsafe {
            closeFd(self.fd);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn poll_timeout() {
        let mut fds = [PollFd::new(-1, POLLIN)];
        // -1 is ignored by poll (only counted for events); use timeout.
        let n = poll(&mut fds, 1).unwrap();
        assert_eq!(n, 0);
    }

    #[test]
    fn pipe_roundtrip() {
        let (r, w) = pipe().unwrap();
        set_nonblock(r).unwrap();
        set_nonblock(w).unwrap();
        write(w, b"hi").unwrap();
        let mut buf = [0; 8];
        assert_eq!(read(r, &mut buf).unwrap(), 2);
        assert_eq!(&buf[..2], b"hi");
        assert!(is_nonblock(r).unwrap());
        close(r);
        close(w);
    }

    #[test]
    fn epoll_empty_wait() {
        let ep = Epoll::new().unwrap();
        let mut evs = [EpollEvent { events: 0, u64: 0 }];
        assert_eq!(ep.wait(&mut evs, 1).unwrap(), 0);
    }
}
