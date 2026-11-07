//! Termios helpers used by `serialhdl` / `util` (`clear_hupcl`,
//! ECHO handling in `create_pty`). Through glibc `tcgetattr` /
//! `tcsetattr` / `tcflush`.

use std::os::raw::c_int;

/// `struct termios` (glibc layout via `libc`).
pub type Termios = libc::termios;

/// `HUPCL`.
pub const HUPCL: u32 = libc::HUPCL;
/// `ECHO`.
pub const ECHO: u32 = libc::ECHO;
/// `TCSADRAIN`.
pub const TCSADRAIN: c_int = libc::TCSADRAIN;
/// `TCSANOW`.
pub const TCSANOW: c_int = libc::TCSANOW;
/// `TCSAFLUSH` (drain + flush).
pub const TCSAFLUSH: c_int = libc::TCSAFLUSH;

/// `tcgetattr(fd, &t)`.
pub fn tcgetattr(fd: i32) -> std::io::Result<Termios> {
    let mut t: Termios = unsafe { std::mem::zeroed() };
    // SAFETY: `t` valid out storage; `fd` is a tty or error.
    let rc = unsafe { libc::tcgetattr(fd, &mut t) };
    if rc != 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(t)
    }
}

/// `tcsetattr(fd, action, &t)`.
pub fn tcsetattr(fd: i32, action: c_int, t: &Termios) -> std::io::Result<()> {
    // SAFETY: `t` valid.
    let rc = unsafe { libc::tcsetattr(fd, action, t) };
    if rc != 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// `tcflush(fd, queue)`.
pub fn tcflush(fd: i32, queue: c_int) -> std::io::Result<()> {
    // SAFETY: `fd` is a tty or error.
    let rc = unsafe { libc::tcflush(fd, queue) };
    if rc != 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// Port of `util.clear_hupcl`: clear `HUPCL` in `c_cflag` with
/// `TCSADRAIN`, ignoring `termios.error` (like the Python `try`).
pub fn clear_hupcl(fd: i32) {
    if let Ok(mut t) = tcgetattr(fd) {
        t.c_cflag &= !(HUPCL as libc::tcflag_t);
        let _ = tcsetattr(fd, TCSADRAIN, &t);
    }
}

/// Atomically toggle `ECHO` in `c_lflag` with `TCSADRAIN`, ignoring
/// errors (port of the `create_pty` ECHO-off step).
pub fn set_echo(fd: i32, on: bool) {
    if let Ok(mut t) = tcgetattr(fd) {
        if on {
            t.c_lflag |= ECHO as libc::tcflag_t;
        } else {
            t.c_lflag &= !(ECHO as libc::tcflag_t);
        }
        let _ = tcsetattr(fd, TCSADRAIN, &t);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore]
    fn tcgetattr_bad_fd() {
        assert!(tcgetattr(-1).is_err());
    }
}
