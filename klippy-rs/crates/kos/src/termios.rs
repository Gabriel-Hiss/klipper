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
/// `TCOFLUSH` (`tcflush` queue selector: flush written, untransmitted data).
pub const TCOFLUSH: c_int = libc::TCOFLUSH;

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

/// Configure pyserial's timeout=0, 8N1 UART mode without flow control.
pub fn configure_uart(fd: i32, baud: i32) -> std::io::Result<()> {
    let mut t = tcgetattr(fd)?;
    t.c_cflag |= libc::CLOCAL | libc::CREAD;
    t.c_lflag &= !(libc::ICANON | libc::ECHO | libc::ECHOE | libc::ECHOK
        | libc::ECHONL | libc::ISIG | libc::IEXTEN | libc::ECHOCTL | libc::ECHOKE);
    t.c_oflag &= !(libc::OPOST | libc::ONLCR | libc::OCRNL);
    t.c_iflag &= !(libc::INLCR | libc::IGNCR | libc::ICRNL | libc::IGNBRK
        | libc::IUCLC | libc::PARMRK | libc::INPCK | libc::ISTRIP
        | libc::IXON | libc::IXOFF | libc::IXANY);
    t.c_cflag &= !(libc::CSIZE | libc::CSTOPB | libc::PARENB
        | libc::PARODD | libc::CMSPAR | libc::CRTSCTS);
    t.c_cflag |= libc::CS8;
    t.c_cc[libc::VMIN] = 0;
    t.c_cc[libc::VTIME] = 0;
    let speed = match baud {
        0 => Some(libc::B0), 50 => Some(libc::B50), 75 => Some(libc::B75),
        110 => Some(libc::B110), 134 => Some(libc::B134), 150 => Some(libc::B150),
        200 => Some(libc::B200), 300 => Some(libc::B300), 600 => Some(libc::B600),
        1200 => Some(libc::B1200), 1800 => Some(libc::B1800), 2400 => Some(libc::B2400),
        4800 => Some(libc::B4800), 9600 => Some(libc::B9600), 19200 => Some(libc::B19200),
        38400 => Some(libc::B38400), 57600 => Some(libc::B57600),
        115200 => Some(libc::B115200), 230400 => Some(libc::B230400),
        460800 => Some(libc::B460800), 500000 => Some(libc::B500000),
        576000 => Some(libc::B576000), 921600 => Some(libc::B921600),
        1000000 => Some(libc::B1000000), 1152000 => Some(libc::B1152000),
        1500000 => Some(libc::B1500000), 2000000 => Some(libc::B2000000),
        2500000 => Some(libc::B2500000), 3000000 => Some(libc::B3000000),
        3500000 => Some(libc::B3500000), 4000000 => Some(libc::B4000000),
        n if n < 0 => return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput, "Invalid baud rate")),
        _ => None,
    };
    // SAFETY: both calls receive a valid termios and a supported speed.
    unsafe {
        libc::cfsetispeed(&mut t, speed.unwrap_or(libc::B38400));
        libc::cfsetospeed(&mut t, speed.unwrap_or(libc::B38400));
    }
    tcsetattr(fd, TCSANOW, &t)?;
    if speed.is_none() {
        let mut t2: libc::termios2 = unsafe { std::mem::zeroed() };
        // SAFETY: libc supplies the target-specific termios2 layout and ioctl codes.
        if unsafe { libc::ioctl(fd, libc::TCGETS2, &mut t2) } < 0 {
            return Err(std::io::Error::last_os_error());
        }
        t2.c_cflag = (t2.c_cflag & !libc::CBAUD) | libc::BOTHER;
        t2.c_ispeed = baud as libc::speed_t;
        t2.c_ospeed = baud as libc::speed_t;
        // SAFETY: t2 was returned by TCGETS2 and only its speed fields changed.
        if unsafe { libc::ioctl(fd, libc::TCSETS2, &t2) } < 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(())
}

/// Set RTS or DTR using pyserial's TIOCMBIS/TIOCMBIC operations.
pub fn set_modem_line(fd: i32, rts: bool, asserted: bool) -> std::io::Result<()> {
    let bit: c_int = if rts { libc::TIOCM_RTS } else { libc::TIOCM_DTR };
    let request = if asserted { libc::TIOCMBIS } else { libc::TIOCMBIC };
    // SAFETY: ioctl reads one initialized integer.
    if unsafe { libc::ioctl(fd, request, &bit) } < 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

/// Initial pyserial open tolerates devices, including PTYs, without modem lines.
pub fn initialize_modem_lines(fd: i32, rts: bool, dtr: bool) -> std::io::Result<()> {
    let result = set_modem_line(fd, false, dtr).and_then(|()| set_modem_line(fd, true, rts));
    match result {
        Err(e) if matches!(e.raw_os_error(), Some(libc::EINVAL | libc::ENOTTY)) => Ok(()),
        result => result,
    }
}

/// Discard unread input, as pyserial does immediately after opening.
pub fn flush_input(fd: i32) -> std::io::Result<()> {
    tcflush(fd, libc::TCIFLUSH)
}

/// Encoded input/output baud constants returned by tcgetattr.
pub fn speeds(t: &Termios) -> (u32, u32) {
    // SAFETY: both functions read a valid termios.
    unsafe { (libc::cfgetispeed(t), libc::cfgetospeed(t)) }
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
