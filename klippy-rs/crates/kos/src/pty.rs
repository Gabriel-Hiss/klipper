//! Pseudo-terminal port of `util.create_pty`. Through glibc
//! `posix_openpt` / `grantpt` / `unlockpt` / `ptsname_r` (the
//! equivalent of `pty.openpty()`).

use std::os::raw::{c_char, c_int};

use crate::{fd, termios, IoResult};

// SAFETY: glibc pty helpers.
unsafe extern "C" {
    fn posix_openpt(oflag: c_int) -> c_int;
    fn grantpt(fd: c_int) -> c_int;
    fn unlockpt(fd: c_int) -> c_int;
    fn ptsname_r(fd: c_int, buf: *mut c_char, buflen: usize) -> c_int;
    fn chmod(path: *const c_char, mode: libc::mode_t) -> c_int;
    fn unlink(path: *const c_char) -> c_int;
    fn symlink(target: *const c_char, linkpath: *const c_char) -> c_int;
}

fn cpath(path: &str) -> std::ffi::CString {
    std::ffi::CString::new(path).expect("no interior NUL")
}

/// Port of `pty.openpty()`: returns `(master, slave)` fds.
pub fn openpty() -> IoResult<(i32, i32)> {
    // SAFETY: `posix_openpt` with O_RDWR|O_NOCTTY.
    let master = unsafe { posix_openpt(libc::O_RDWR | libc::O_NOCTTY) };
    if master < 0 {
        return Err(std::io::Error::last_os_error());
    }
    // SAFETY: valid master fd.
    let rc = unsafe { grantpt(master) };
    if rc != 0 {
        let e = std::io::Error::last_os_error();
        fd::close(master);
        return Err(e);
    }
    // SAFETY: valid master fd.
    unsafe {
        unlockpt(master);
    }
    let mut buf = [0i8; 256];
    // SAFETY: `buf` valid out storage.
    let rc = unsafe { ptsname_r(master, buf.as_mut_ptr() as *mut c_char, buf.len()) };
    if rc != 0 {
        fd::close(master);
        return Err(std::io::Error::from_raw_os_error(rc));
    }
    let slave_name = crate::cconv::bytes_to_cstring(&buf);
    // SAFETY: NUL-terminated slave name.
    let slave = unsafe { libc::open(slave_name.as_ptr() as *const c_char, libc::O_RDWR | libc::O_NOCTTY) };
    if slave < 0 {
        let e = std::io::Error::last_os_error();
        fd::close(master);
        return Err(e);
    }
    Ok((master, slave))
}

/// Port of `util.create_pty(ptyname)`:
/// - unlink old symlink (ignore errors)
/// - `chmod 0o660` the slave
/// - symlink slave -> `ptyname`
/// - set master nonblocking
/// - ECHO off on the master with `TCSADRAIN`
///
/// Returns the master fd. The slave fd is closed (like the Python
/// local going out of scope) — the slave device remains openable
/// through the symlink, which is how g-code input connects.
pub fn create_pty(ptyname: &str) -> IoResult<i32> {
    let (master, slave) = openpty()?;
    let link = cpath(ptyname);
    // SAFETY: `link` is a valid path.
    unsafe {
        unlink(link.as_ptr() as *const c_char);
    }
    let mut buf = [0i8; 256];
    // SAFETY: valid slave fd.
    let rc = unsafe { ptsname_r(slave, buf.as_mut_ptr() as *mut c_char, buf.len()) };
    if rc != 0 {
        fd::close(master);
        fd::close(slave);
        return Err(std::io::Error::from_raw_os_error(rc));
    }
    let slave_name = crate::cconv::bytes_to_cstring(&buf);
    // SAFETY: valid slave name.
    unsafe {
        chmod(slave_name.as_ptr() as *const c_char, 0o660);
    }
    // SAFETY: both pointers valid paths.
    let rc = unsafe { symlink(slave_name.as_ptr() as *const c_char, link.as_ptr() as *const c_char) };
    if rc != 0 {
        let e = std::io::Error::last_os_error();
        fd::close(master);
        fd::close(slave);
        return Err(e);
    }
    fd::close(slave);
    fd::set_nonblock(master)?;
    termios::set_echo(master, false);
    Ok(master)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_pty_roundtrip() {
        let path = format!("/tmp/klippy-p{}-{}.pty", std::process::id(), crate::time::monotonic() as i64);
        let master = create_pty(&path).unwrap();
        let slave = unsafe { libc::open(cpath(&path).as_ptr() as *const c_char, libc::O_RDWR | libc::O_NOCTTY) };
        assert!(slave >= 0);
        unsafe { libc::close(slave) };
        fd::close(master);
        let link = cpath(&path);
        unsafe { libc::unlink(link.as_ptr() as *const c_char) };
    }
}
