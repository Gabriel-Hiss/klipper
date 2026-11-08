//! Port of `klippy/chelper/msgblock.c` / `msgblock.h`: CRC16-CCITT,
//! message check/decode, VLQ encode/decode, clock estimates.

use kos::cconv;

/// `MESSAGE_MIN` from `msgblock.h`.
pub const MESSAGE_MIN: usize = 5;
/// `MESSAGE_MAX` from `msgblock.h`.
pub const MESSAGE_MAX: usize = 64;
/// `MESSAGE_HEADER_SIZE` from `msgblock.h`.
pub const MESSAGE_HEADER_SIZE: usize = 2;
/// `MESSAGE_TRAILER_SIZE` from `msgblock.h`.
pub const MESSAGE_TRAILER_SIZE: usize = 3;
/// `MESSAGE_POS_LEN` from `msgblock.h`.
pub const MESSAGE_POS_LEN: usize = 0;
/// `MESSAGE_POS_SEQ` from `msgblock.h`.
pub const MESSAGE_POS_SEQ: usize = 1;
/// `MESSAGE_TRAILER_CRC` from `msgblock.h`.
pub const MESSAGE_TRAILER_CRC: usize = 3;
/// `MESSAGE_TRAILER_SYNC` from `msgblock.h`.
pub const MESSAGE_TRAILER_SYNC: usize = 1;
/// `MESSAGE_PAYLOAD_MAX` from `msgblock.h` (`MESSAGE_MAX - MESSAGE_MIN`).
pub const MESSAGE_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;
/// `MESSAGE_SEQ_MASK` from `msgblock.h`.
pub const MESSAGE_SEQ_MASK: u64 = 0x0f;
/// `MESSAGE_DEST` from `msgblock.h`.
pub const MESSAGE_DEST: u8 = 0x10;
/// `MESSAGE_SYNC` from `msgblock.h`.
pub const MESSAGE_SYNC: u8 = 0x7E;

/// `struct queue_message` from `msgblock.h`.
///
/// The C struct carries a union of `(min_clock, req_clock)` (on a
/// command queue) and `(sent_time, receive_time)` (in sent/receive
/// queues). The port keeps both sets of fields; union vs. plain
/// fields is unobservable outside the struct itself, and keeping both
/// avoids reproducing the C overlap semantics unintentionally.
#[derive(Debug, Clone)]
pub struct QueueMessage {
    /// Message bytes (`msg[MESSAGE_MAX]`).
    pub msg: [u8; MESSAGE_MAX],
    /// Message length in bytes, or 0 for notification placeholders.
    pub len: i32,
    /// Earliest mcu clock at which this message may be transmitted.
    pub min_clock: u64,
    /// Requested mcu clock (priority); repurposed as the send sequence
    /// number while the message sits on the notify queue.
    pub req_clock: u64,
    /// Time the payload was (last) received by the mcu estimate.
    pub sent_time: f64,
    /// Time the message arrived locally (`get_monotonic()` based).
    pub receive_time: f64,
    /// Nonzero while the message waits for an ack notification.
    pub notify_id: u64,
}

impl QueueMessage {
    /// Port of `message_alloc`: zeroed message.
    pub fn new() -> Self {
        QueueMessage {
            msg: [0; MESSAGE_MAX],
            len: 0,
            min_clock: 0,
            req_clock: 0,
            sent_time: 0.0,
            receive_time: 0.0,
            notify_id: 0,
        }
    }
}

impl Default for QueueMessage {
    fn default() -> Self {
        Self::new()
    }
}

/// `struct clock_estimate` from `msgblock.h`.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClockEstimate {
    /// Clock value at [`ClockEstimate::conv_time`].
    pub conv_clock: u64,
    /// Monotonic time at which the estimate was taken.
    pub conv_time: f64,
    /// Estimated mcu clock frequency (ticks per second).
    pub est_freq: f64,
}

/// Port of the C `msgblock_crc16_ccitt` ("ccitt" variant with the
/// Klipper bit-twiddle, initial value `0xffff`).
pub fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in buf {
        let mut data: u8 = b ^ (crc & 0xff) as u8;
        data ^= data << 4;
        crc = (((data as u16) << 8) | (crc >> 8)) ^ ((data >> 4) as u16) ^ ((data as u16) << 3);
    }
    crc
}

/// Port of the C `msgblock_check`.
///
/// Returns `0` (need more data), a positive message length, or a
/// negative skip count (discard up to and including the next SYNC).
/// `need_sync` is the C `*need_sync` (nonzero while scanning bytes).
pub fn msgblock_check(need_sync: &mut u8, buf: &[u8]) -> i32 {
    let buf_len = buf.len() as i32;
    if buf_len < MESSAGE_MIN as i32 {
        // Need more data
        return 0;
    }
    if *need_sync != 0 {
        return msgblock_check_error(need_sync, buf, buf_len);
    }
    let msglen = buf[MESSAGE_POS_LEN];
    if msglen < MESSAGE_MIN as u8 || msglen > MESSAGE_MAX as u8 {
        return msgblock_check_error(need_sync, buf, buf_len);
    }
    let msgseq = buf[MESSAGE_POS_SEQ];
    if (msgseq & 0xf0) != MESSAGE_DEST {
        return msgblock_check_error(need_sync, buf, buf_len);
    }
    if buf_len < msglen as i32 {
        // Need more data
        return 0;
    }
    if buf[msglen as usize - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
        return msgblock_check_error(need_sync, buf, buf_len);
    }
    let msgcrc = ((buf[msglen as usize - MESSAGE_TRAILER_CRC] as u16) << 8)
        | buf[msglen as usize - MESSAGE_TRAILER_CRC + 1] as u16;
    let crc = crc16_ccitt(&buf[..msglen as usize - MESSAGE_TRAILER_SIZE]);
    if crc != msgcrc {
        return msgblock_check_error(need_sync, buf, buf_len);
    }
    msglen as i32
}

fn msgblock_check_error(need_sync: &mut u8, buf: &[u8], buf_len: i32) -> i32 {
    // Discard bytes until next SYNC found
    let found = buf.iter().position(|&c| c == MESSAGE_SYNC);
    if let Some(pos) = found {
        *need_sync = 0;
        // C: `-(next_sync - buf + 1)` — discard through the SYNC byte.
        return -((pos as i32) + 1);
    }
    *need_sync = 1;
    -buf_len
}

/// Encode one variable-length integer at `buf[p..]`, returning the
/// offset just past the written bytes. Port of the C `encode_int`.
///
/// The C bounds are `sv < 3<<5 && sv >= -(1<<5)` (1 byte), `3<<12` /
/// `-(1<<12)` (2 bytes), `3<<19` / `-(1<<19)` (3 bytes), `3<<26` /
/// `-(1<<26)` (4 bytes), else 5 bytes; `sv` is `(int32_t)v`.
pub fn encode_int(buf: &mut [u8], p: usize, v: u32) -> usize {
    let sv = v as i32;
    let mut q = p;
    if !(-32..96).contains(&sv) {
        if !(-4096..12288).contains(&sv) {
            if !(-524288..1572864).contains(&sv) {
                if !(-67108864..201326592).contains(&sv) {
                    buf[q] = ((v >> 28) as u8) | 0x80;
                    q += 1;
                }
                buf[q] = (((v >> 21) & 0x7f) as u8) | 0x80;
                q += 1;
            }
            buf[q] = (((v >> 14) & 0x7f) as u8) | 0x80;
            q += 1;
        }
        buf[q] = (((v >> 7) & 0x7f) as u8) | 0x80;
        q += 1;
    }
    buf[q] = (v & 0x7f) as u8;
    q + 1
}

/// Parse one VLQ at `msg[p..]`. Returns `(value, next_offset)`, or
/// `None` if the VLQ runs past the message.
///
/// The C `parse_int` reads without a bounds check (an overlong VLQ
/// reads past the message end); the port returns `None` instead of
/// reading out of bounds. All in-range behaviour is identical.
fn parse_int(msg: &[u8], mut p: usize) -> Option<(u32, usize)> {
    let c0 = *msg.get(p)?;
    p += 1;
    let mut v: u32 = (c0 & 0x7f) as u32;
    let mut c = c0;
    if (c & 0x60) == 0x60 {
        // C: `v |= -0x20`, i.e. set the high 27 bits (sign extension
        // of the 5-bit two's-complement head byte).
        v |= 0xffff_ffe0;
    }
    while c & 0x80 != 0 {
        c = *msg.get(p)?;
        p += 1;
        // C: `v = (v<<7) | (c & 0x7f)` on a `uint32_t`; the wrapping
        // shift reproduces the x86_64 machine behaviour for overlong
        // (out-of-range) VLQs.
        v = (v << 7) | ((c & 0x7f) as u32);
    }
    Some((v, p))
}

/// Port of the C `msgblock_decode`: parse the VLQ contents of a
/// complete message (including header/trailer) into `data`.
///
/// Returns `0` on success, `-1` on malformed input.
pub fn msgblock_decode(data: &mut [u32], msg: &[u8]) -> i32 {
    let msg_len = msg.len();
    if msg_len < MESSAGE_HEADER_SIZE + MESSAGE_TRAILER_SIZE {
        return -1;
    }
    let mut p = MESSAGE_HEADER_SIZE;
    let end = msg_len - MESSAGE_TRAILER_SIZE;
    for slot in data.iter_mut() {
        if p >= end {
            return -1;
        }
        match parse_int(msg, p) {
            Some((v, next)) => {
                *slot = v;
                p = next;
            }
            None => return -1,
        }
    }
    if p != end {
        // Invalid message
        return -1;
    }
    0
}

/// Port of the C `message_fill`: allocate a message and copy `data`.
/// The C copies `len` bytes unconditionally (`memcpy(msg, data, len)`);
/// for out-of-range lengths that overflows the fixed 64-byte buffer
/// (undefined behaviour). The port clamps to the in-range behaviour:
/// `len` is kept for `len` in `[0, MESSAGE_MAX]` and clamped to
/// `MESSAGE_MAX` past it so downstream slices never panic.
pub fn message_fill(data: &[u8], len: i32) -> Box<QueueMessage> {
    let mut qm = Box::new(QueueMessage::new());
    if len > 0 {
        let n = (len as usize).min(MESSAGE_MAX).min(data.len());
        qm.msg[..n].copy_from_slice(&data[..n]);
        qm.len = n as i32;
    } else {
        qm.len = 0;
    }
    qm
}

/// Port of the C `message_alloc_and_encode`: allocate a message and
/// fill it with VLQ-encoded integers. If the encoding exceeds
/// `MESSAGE_PAYLOAD_MAX` bytes, logs `Encode error` (via
/// [`crate::errorf`], the C `errorf`) and returns a zero-length
/// message.
pub fn message_alloc_and_encode(data: &[u32]) -> Box<QueueMessage> {
    let mut qm = Box::new(QueueMessage::new());
    let mut p = 0_usize;
    for &v in data {
        p = encode_int(&mut qm.msg, p, v);
        if p > MESSAGE_PAYLOAD_MAX {
            crate::errorf("Encode error");
            qm.len = 0;
            return qm;
        }
    }
    qm.len = p as i32;
    qm
}

/// Port of the C `clock_from_clock32`: extend a 32-bit clock value to
/// its full 64-bit value.
///
/// C: `last_clock + (int32_t)(clock32 - last_clock)`. The subtraction
/// is unsigned 64-bit (wrapping), truncated to its low 32 bits,
/// reinterpreted as `i32`, sign-extended, and added (wrapping).
pub fn clock_from_clock32(last_clock: u64, clock32: u32) -> u64 {
    last_clock.wrapping_add(((clock32 as u64).wrapping_sub(last_clock) as u32) as i32 as i64 as u64)
}

/// Port of the C `clock_to_time`:
/// `conv_time + (int64_t)(clock - conv_clock) / est_freq`.
pub fn clock_to_time(ce: &ClockEstimate, clock: u64) -> f64 {
    ce.conv_time + (clock.wrapping_sub(ce.conv_clock) as i64 as f64) / ce.est_freq
}

/// Port of the C `clock_from_time`:
/// `(int64_t)((time - conv_time) * est_freq + .5) + conv_clock`.
///
/// The floating-point-to-`i64` cast goes through `kos::cconv` so the
/// out-of-range / NaN behaviour matches the C binary.
pub fn clock_from_time(ce: &ClockEstimate, time: f64) -> u64 {
    (cconv::f64_to_i64((time - ce.conv_time) * ce.est_freq + 0.5) as u64).wrapping_add(ce.conv_clock)
}

/// Port of the C `clock_fill`.
pub fn clock_fill(ce: &mut ClockEstimate, est_freq: f64, conv_time: f64, conv_clock: u64) {
    ce.est_freq = est_freq;
    ce.conv_time = conv_time;
    ce.conv_clock = conv_clock;
}
