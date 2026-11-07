//! Unix sockets (port of the `socket.AF_UNIX` use in `webhooks.py`
//! and `serialhdl.py` pipe transport). Through glibc `socket(2)`.

use std::os::raw::{c_char, c_int, c_void};

use crate::{fd, IoResult};

// SAFETY: glibc socket/address helpers.
unsafe extern "C" {
    #[link_name = "socket"]
    fn raw_socket(domain: c_int, ty: c_int, protocol: c_int) -> c_int;
    #[link_name = "bind"]
    fn raw_bind(fd: c_int, addr: *const c_void, len: u32) -> c_int;
    #[link_name = "listen"]
    fn raw_listen(fd: c_int, backlog: c_int) -> c_int;
    #[link_name = "connect"]
    fn raw_connect(fd: c_int, addr: *const c_void, len: u32) -> c_int;
    #[link_name = "accept"]
    fn raw_accept(fd: c_int, addr: *mut c_void, len: *mut u32) -> c_int;
    #[link_name = "shutdown"]
    fn raw_shutdown(fd: c_int, how: c_int) -> c_int;
    fn getaddrinfo(host: *const c_char, service: *const c_char, hints: *const c_void, res: *mut *mut c_void) -> c_int;
    fn freeaddrinfo(res: *mut c_void);
}

/// `AF_UNIX`.
pub const AF_UNIX: c_int = 1;
/// `AF_INET`.
pub const AF_INET: c_int = 2;
/// `SOCK_STREAM`.
pub const SOCK_STREAM: c_int = 1;
/// `SOCK_DGRAM`.
pub const SOCK_DGRAM: c_int = 2;
/// `SHUT_RD`.
pub const SHUT_RD: c_int = 0;
/// `SHUT_WR`.
pub const SHUT_WR: c_int = 1;
/// `SHUT_RDWR`.
pub const SHUT_RDWR: c_int = 2;

/// A bound unix address (path-limited to 107 bytes like sockaddr_un).
#[repr(C)]
pub struct UnixAddr {
    family: u16,
    path: [u8; 108],
}

/// `sockaddr_un` for a path, or `None` if the path is too long.
pub fn unix_addr(path: &str) -> Option<UnixAddr> {
    if path.len() >= 100 {
        return None;
    }
    let mut a = UnixAddr { family: AF_UNIX as u16, path: [0; 108] };
    a.path[..path.len()].copy_from_slice(path.as_bytes());
    Some(a)
}

/// Create a socket (`domain`, `ty`, protocol 0).
pub fn socket(domain: c_int, ty: c_int) -> IoResult<i32> {
    let fd = unsafe { raw_socket(domain, ty, 0) };
    if fd < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(fd)
    }
}

/// Bind a unix socket at `path` (like `socket.bind` on `AF_UNIX`).
/// The caller is responsible for unlinking a stale socket file.
pub fn unix_bind(path: &str) -> IoResult<i32> {
    let addr = unix_addr(path).ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidInput, "unix path too long"))?;
    let fd = unsafe { raw_socket(AF_UNIX, SOCK_STREAM, 0) };
    if fd < 0 {
        return Err(std::io::Error::last_os_error());
    }
    // SAFETY: `addr` valid for its length.
    let rc = unsafe { raw_bind(fd, &addr as *const UnixAddr as *const c_void, std::mem::size_of::<UnixAddr>() as u32) };
    if rc < 0 {
        let e = std::io::Error::last_os_error();
        fd::close(fd);
        return Err(e);
    }
    Ok(fd)
}

/// Connect a unix stream socket to `path` (like `serialhdl` pipe connect).
pub fn unix_connect(path: &str) -> IoResult<i32> {
    let addr = unix_addr(path).ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidInput, "unix path too long"))?;
    let fd = unsafe { raw_socket(AF_UNIX, SOCK_STREAM, 0) };
    if fd < 0 {
        return Err(std::io::Error::last_os_error());
    }
    // SAFETY: `addr` valid.
    let rc = unsafe { raw_connect(fd, &addr as *const UnixAddr as *const c_void, std::mem::size_of::<UnixAddr>() as u32) };
    if rc < 0 {
        let e = std::io::Error::last_os_error();
        fd::close(fd);
        return Err(e);
    }
    Ok(fd)
}

/// `listen(fd, backlog)`.
pub fn listen(fd: i32, backlog: c_int) -> IoResult<()> {
    // SAFETY: valid socket or error.
    let rc = unsafe { raw_listen(fd, backlog) };
    if rc < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// `accept(fd)` (no address out).
pub fn accept(fd: i32) -> IoResult<i32> {
    // SAFETY: valid listening socket or error.
    let nfd = unsafe { raw_accept(fd, std::ptr::null_mut(), std::ptr::null_mut()) };
    if nfd < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(nfd)
    }
}

/// `shutdown(fd, how)` (`SHUT_*`).
pub fn shutdown(fd: i32, how: c_int) -> IoResult<()> {
    // SAFETY: valid socket or error.
    let rc = unsafe { raw_shutdown(fd, how) };
    if rc < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// Resolve `host:service` via `getaddrinfo` and return the first
/// sockaddr bytes to connect with. Used by the webhooks server for
/// the API socket when configured as a hostname.
#[allow(dead_code)]
pub struct ResolvedAddr {
    ptr: *mut c_void,
    len: u32,
}

/// Resolve and connect to `host:port` (first result).
pub fn resolve_connect(host: &str, port: &str) -> IoResult<i32> {
    let host = std::ffi::CString::new(host).unwrap_or_default();
    let port = std::ffi::CString::new(port).unwrap_or_default();
    let mut res: *mut c_void = std::ptr::null_mut();
    // SAFETY: valid strings; `res` is out storage.
    let rc = unsafe { getaddrinfo(host.as_ptr() as *const c_char, port.as_ptr() as *const c_char, std::ptr::null(), &mut res as *mut *mut c_void) };
    if rc != 0 {
        return Err(std::io::Error::from_raw_os_error(-rc));
    }
    // First result: sockaddr_family(2) + sockaddr_len(4...) — read
    // family and length from the struct head. glibc addrinfo head:
    // ai_flags..sa_family offset 24, ai_addrlen at 16.
    // SAFETY: `res` is a valid addrinfo.
    let (family, alen) = unsafe {
        let ai = res as *const u8;
        let alen = std::ptr::read_unaligned(ai.add(16) as *const u32);
        let family = std::ptr::read_unaligned(ai.add(24) as *const u16);
        (family as c_int, alen)
    };
    let _addr: *const c_void = unsafe { (res as *const u8).add(32) as *const c_void };
    let fd = unsafe { raw_socket(family, SOCK_STREAM, 0) };
    if fd < 0 {
        unsafe { freeaddrinfo(res) };
        return Err(std::io::Error::last_os_error());
    }
    // SAFETY: `addr` points into the addrinfo allocation.
    let rc = unsafe { raw_connect(fd, _addr, alen) };
    // SAFETY: `res` from getaddrinfo.
    unsafe { freeaddrinfo(res) };
    if rc < 0 {
        let e = std::io::Error::last_os_error();
        fd::close(fd);
        return Err(e);
    }
    fd::set_nonblock(fd).ok();
    Ok(fd)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unix_stream_roundtrip() {
        let path = format!("/tmp/klippy-sock-{}.sock", std::process::id());
        let _ = std::fs::remove_file(&path);
        let server = unix_bind(&path).unwrap();
        listen(server, 5).unwrap();
        fd::set_nonblock(server).unwrap();
        let client = unix_connect(&path).unwrap();
        let accepted = accept(server).unwrap();
        fd::write(client, b"ping").unwrap();
        let mut buf = [0; 8];
        assert_eq!(fd::read(accepted, &mut buf).unwrap(), 4);
        assert_eq!(&buf[..4], b"ping");
        fd::close(client);
        fd::close(accepted);
        fd::close(server);
        let _ = std::fs::remove_file(&path);
    }
}
