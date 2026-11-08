//! Port of the `msgblock.c` helpers used by `stepcompress.c`.
//!
//! Only the pieces the step compressor needs are ported here: VLQ integer
//! encoding (`encode_int`) and the payload packing of
//! `message_alloc_and_encode`.  They produce the `msg` bytes of a
//! `queue_message` exactly as the C does.

/// Payload size limit from `msgblock.h`: `MESSAGE_MAX - MESSAGE_MIN`.
pub const MESSAGE_PAYLOAD_MAX: usize = 59;

/// Maximum encoded bytes of one message (`msgblock.h` `MESSAGE_MAX`).
pub const MESSAGE_MAX: usize = 64;

/// Encode one `uint32_t` as a variable length quantity.
///
/// Faithful port of `encode_int` in `msgblock.c`.  The value is
/// reinterpreted as `int32_t` to pick the length (1..5 bytes); the
/// emitted byte groups are all shifted out of the original `v`.
fn encode_int(out: &mut [u8; MESSAGE_MAX], mut p: usize, v: u32) -> usize {
    let sv = v as i32;
    // Number of VLQ bytes: the C goto cascade emits one byte per level
    // below the selected range check, plus the final 7 bits.
    let nbytes = if (-(1 << 5)..(3 << 5)).contains(&sv) {
        1
    } else if (-(1 << 12)..(3 << 12)).contains(&sv) {
        2
    } else if (-(1 << 19)..(3 << 19)).contains(&sv) {
        3
    } else if (-(1 << 26)..(3 << 26)).contains(&sv) {
        4
    } else {
        5
    };
    // Continuation bytes carry 7-bit groups from the top down; each is
    // shifted out of the original `v` exactly like the C labels f1..f3.
    for k in (1..nbytes).rev() {
        out[p] = (((v >> (7 * k)) & 0x7f) | 0x80) as u8;
        p += 1;
    }
    out[p] = (v & 0x7f) as u8;
    p + 1
}

/// Port of `message_alloc_and_encode`: encode `data` into the fixed-size
/// message buffer and return the produced bytes.
///
/// Returns `None` when the C code would take its `fail` path (the encoded
/// payload would pass `msg[MESSAGE_PAYLOAD_MAX]`); callers keep the same
/// fallback behaviour as the C (a zero-length message).
pub fn encode_message(data: &[u32]) -> Option<Vec<u8>> {
    let mut buf = [0u8; MESSAGE_MAX];
    let mut p = 0usize;
    for &v in data {
        p = encode_int(&mut buf, p, v);
        if p > MESSAGE_PAYLOAD_MAX {
            return None;
        }
    }
    Some(buf[..p].to_vec())
}
