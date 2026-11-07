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
    fn raw_poll(fds: *mut PollFd, nfds: u64, timeout: c_int) -> c_int;
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

/// `struct epoll_event`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct EpollEvent {
    /// Events.
    pub events: u32,
    /// Associated fd.
    pub fd: c_int,
    _pad: [u8; 4],
}

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

/// `poll(2)`. `timeout_ms` is milliseconds (negative waits
/// forever, like `poll(-1)`); restarts on `EINTR` like PEP 475.
/// Returns the number of ready fds; `revents` is filled in.
pub fn poll(fds: &mut [PollFd], timeout_ms: i32) -> IoResult<usize> {
    if fds.is_empty() {
        // `poll(NULL, 0, ms)` sleeps; model with the same call.
        // SAFETY: null list with 0 entries is used only for sleep.
        let rc = unsafe { raw_poll(std::ptr::null_mut(), 0, timeout_ms) };
        if rc < 0 {
            return Err(std::io::Error::last_os_error());
        }
        return Ok(rc as usize);
    }
    loop {
        // SAFETY: `fds` valid for `len`.
        let rc = unsafe { raw_poll(fds.as_mut_ptr(), fds.len() as u64, timeout_ms) };
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
        let ev = EpollEvent { events, fd, _pad: [0; 4] };
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
        let ev = EpollEvent { events, fd, _pad: [0; 4] };
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
        let mut evs = [EpollEvent { events: 0, fd: 0, _pad: [0; 4] }];
        assert_eq!(ep.wait(&mut evs, 1).unwrap(), 0);
    }
}
