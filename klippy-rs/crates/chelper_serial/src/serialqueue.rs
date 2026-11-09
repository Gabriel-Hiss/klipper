//! Port of `klippy/chelper/serialqueue.c` / `serialqueue.h`: serial
//! port command queuing. Command queues, upcoming/ready scheduling by
//! `min_clock` / `req_clock`, block building (sequence numbers, CRC),
//! retransmit with RFC6298 RTT estimation, the receive window, fast
//! readers, notifications, the stats string, debug-file mode, the
//! `[DEBUG-motion]` send-path rings and the background thread.
//!
//! The C intrusive lists (`list.h`) are modelled with `VecDeque`:
//! `list_add_tail` / `list_del` / `list_join_tail` over the notify,
//! sent, ready and upcoming queues map onto push-back / pop-front /
//! drain semantics with the same iteration order.
//!
//! All sleeps, polls, thread starts and locks go through `kos`.

use std::collections::VecDeque;
use std::os::raw::c_void;
use std::sync::Arc;

use kos::cconv;
use kos::fd;
use kos::sync;
use kos::termios;
use kos::thread;
use kos::time;

use crate::msgblock;
use crate::msgblock::{crc16_ccitt, message_fill, ClockEstimate, QueueMessage};
use crate::pollreactor::{PollReactor, PR_NEVER, PR_NOW};
use pycompat::text::format_f_fixed;

// ---- serialqueue.h ---------------------------------------------------

/// `MAX_CLOCK` from `serialqueue.h`.
pub const MAX_CLOCK: u64 = 0x7fffffffffffffff;
/// `BACKGROUND_PRIORITY_CLOCK` from `serialqueue.h`.
pub const BACKGROUND_PRIORITY_CLOCK: u64 = 0x7fffffff00000000;

// ---- file-local constants (serialqueue.c) ----------------------------

/// `SQPF_SERIAL`.
const SQPF_SERIAL: usize = 0;
/// `SQPF_PIPE`.
const SQPF_PIPE: usize = 1;
/// `SQPF_NUM`.
const SQPF_NUM: usize = 2;

/// `SQPT_RETRANSMIT`.
const SQPT_RETRANSMIT: usize = 0;
/// `SQPT_COMMAND`.
const SQPT_COMMAND: usize = 1;
/// `SQPT_NUM`.
const SQPT_NUM: usize = 2;

/// `SQT_UART` (`'u'`).
pub const SQT_UART: u8 = b'u';
/// `SQT_CAN` (`'c'`).
pub const SQT_CAN: u8 = b'c';
/// `SQT_DEBUGFILE` (`'f'`).
pub const SQT_DEBUGFILE: u8 = b'f';

/// `MIN_RTO`.
const MIN_RTO: f64 = 0.025;
/// `MAX_RTO`.
const MAX_RTO: f64 = 5.000;
/// `MAX_PENDING_BLOCKS`.
const MAX_PENDING_BLOCKS: u64 = 12;
/// `MIN_REQTIME_DELTA`.
const MIN_REQTIME_DELTA: f64 = 0.100;
/// `MIN_BACKGROUND_DELTA`.
const MIN_BACKGROUND_DELTA: f64 = 0.005;

const DEBUG_QUEUE_SENT: usize = 100;
const DEBUG_QUEUE_RECEIVE: usize = 100;

const CANBUS_PACKET_BITS: u32 = (1 + 11 + 3 + 4) + (16 + 2 + 7 + 3);
const CANBUS_IFS_BITS: u32 = 4;

/// glibc `tcflush` queue selectors (`<termios.h>`); `kos::termios`
/// exports only the `tcsetattr` actions, so `TCOFLUSH` is spelled
/// out here and reported as a suggested `kos` addition.
const TCOFLUSH: i32 = 1;

// Dispatch decision reasons (serialqueue.c `sq_debug_*` block).
const SQDBG_SEND: u8 = 0;
const SQDBG_NOWORK: u8 = 1;
const SQDBG_MINCLOCK: u8 = 2;
const SQDBG_REQWAIT: u8 = 3;
const SQDBG_ACKWAIT: u8 = 4;
const SQDBG_WINDOW: u8 = 5;
const SQDBG_RACE: u8 = 6;
const SQDBG_PENDING: u8 = 7;
const SQDBG_NOCLOCK: u8 = 8;
const SQDBG_WRITE: u8 = 9;
const SQDBG_RETRANSMIT: u8 = 10;
const SQDBG_EXIT: u8 = 11;
const SQDBG_REASONS: usize = 12;

const SQDBG_REASON_NAMES: [&str; SQDBG_REASONS] = [
    "send", "nowork", "minclock", "reqwait", "ackwait", "window", "race", "pending", "noclock",
    "write", "retransmit", "exit",
];

#[allow(dead_code)]
const SQDBG_NOTABLE_MASK: u32 = (1 << SQDBG_ACKWAIT)
    | (1 << SQDBG_WINDOW)
    | (1 << SQDBG_NOCLOCK)
    | (1 << SQDBG_WRITE)
    | (1 << SQDBG_RETRANSMIT)
    | (1 << SQDBG_EXIT);

const SQDBGF_STALLED: u8 = 0x01;
const SQDBGF_READY_EMPTY: u8 = 0x02;
const SQDBGF_MUSTWAKE: u8 = 0x04;
const SQDBGF_NOCLOCK: u8 = 0x08;
const SQDBGF_CAN: u8 = 0x10;
const SQDBGF_WRITE_FAIL: u8 = 0x20;
const SQDBGF_NAK: u8 = 0x40;

const SQDBG_FAR: f64 = 1000000.;
const DEBUG_SEND_EVENTS: usize = 64;
const DEBUG_NOTABLE_EVENTS: usize = 16;

// ---- debug rings ------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct SqDebugEvent {
    eventtime: f64,
    last_eventtime: f64,
    sched: f64,
    waketime: f64,
    idletime: f64,
    ack_clock: u64,
    want_clock: u64,
    send_seq: u64,
    receive_seq: u64,
    ready_bytes: u32,
    upcoming_bytes: u32,
    need_ack_bytes: u32,
    pending: i32,
    result: i32,
    repeat: u16,
    reason: u8,
    flags: u8,
}

impl Default for SqDebugEvent {
    fn default() -> Self {
        SqDebugEvent {
            eventtime: 0.0,
            last_eventtime: 0.0,
            sched: 0.0,
            waketime: 0.0,
            idletime: 0.0,
            ack_clock: 0,
            want_clock: 0,
            send_seq: 0,
            receive_seq: 0,
            ready_bytes: 0,
            upcoming_bytes: 0,
            need_ack_bytes: 0,
            pending: 0,
            result: 0,
            repeat: 0,
            reason: 0,
            flags: 0,
        }
    }
}

#[derive(Debug, Clone)]
struct SqDebug {
    ring: Vec<SqDebugEvent>,
    notable: Vec<SqDebugEvent>,
    pos: u32,
    count: u32,
    notable_pos: u32,
    notable_count: u32,
    total: u32,
    reason_count: [u32; SQDBG_REASONS],
}

impl Default for SqDebug {
    fn default() -> Self {
        SqDebug {
            ring: vec![SqDebugEvent::default(); DEBUG_SEND_EVENTS],
            notable: vec![SqDebugEvent::default(); DEBUG_NOTABLE_EVENTS],
            pos: 0,
            count: 0,
            notable_pos: 0,
            notable_count: 0,
            total: 0,
            reason_count: [0; SQDBG_REASONS],
        }
    }
}

/// Append an event to a ring, coalescing a run of identical events
/// into the previous entry (port of `sq_debug_push`).
fn sq_debug_push(
    ring: &mut [SqDebugEvent],
    pos: &mut u32,
    count: &mut u32,
    ev: SqDebugEvent,
) {
    let size = ring.len() as u32;
    let last_idx = ((*pos + size - 1) % size) as usize;
    let last = ring[last_idx];
    // Sends coalesce per command_event pass (send_seq always moves);
    // wait reasons coalesce for as long as the sequence numbers stand
    // still.
    let same = *count != 0
        && last.reason == ev.reason
        && if ev.reason == SQDBG_SEND {
            last.eventtime == ev.eventtime
        } else {
            last.send_seq == ev.send_seq && last.receive_seq == ev.receive_seq
        };
    if same {
        let first = last.eventtime;
        let repeat = if last.repeat < 0xffff { last.repeat + 1 } else { 0xffff };
        let result = if ev.reason == SQDBG_SEND { last.result + ev.result } else { ev.result };
        let mut coalesced = ev;
        coalesced.eventtime = first;
        coalesced.repeat = repeat;
        coalesced.result = result;
        ring[last_idx] = coalesced;
        return;
    }
    ring[*pos as usize] = ev;
    *pos = (*pos + 1) % size;
    if *count < size {
        *count += 1;
    }
}

// ---- state ------------------------------------------------------------

/// Fast reader callback: `func(fr, eventtime, data, len)`. `ctx` is
/// the caller's fastreader object pointer (the C `fr`), `data` the
/// received message buffer (the C `sq->input_buf`) and `len` its
/// length.
pub type FastReaderCb = extern "C" fn(*mut c_void, f64, *mut u8, i32);

#[derive(Clone)]
struct FastReaderEntry {
    func: FastReaderCb,
    prefix: Vec<u8>,
    ctx: usize,
}

/// Port of `struct command_queue` (`struct message_sub_queue ready,
/// upcoming`). The C splits locking by queue (upcoming under the
/// transmit lock, ready under `sq->lock`); the port holds one mutex
/// per queue pair. Both lock orders (`sq -> tx -> cq` and
/// `tx -> cq`) are preserved, so the observable migration semantics
/// are identical.
pub struct CommandQueue {
    queues: sync::Mutex<CommandQueueQueues>,
}

struct CommandQueueQueues {
    ready: VecDeque<Box<QueueMessage>>,
    upcoming: VecDeque<Box<QueueMessage>>,
}

impl CommandQueue {
    /// Expose the pending (`upcoming`) queue for inspection: a clone of
    /// every message not yet moved to the ready queue, oldest first. The
    /// C parity tests read the same list by walking
    /// `cq->upcoming.msg_queue`.
    pub fn upcoming_messages(&self) -> Vec<QueueMessage> {
        self.queues.lock().upcoming.iter().map(|qm| (**qm).clone()).collect()
    }
}

/// State guarded by `sq->lock` (`struct serialqueue` in the C).
struct QueueState {
    receive_window: i32,
    bittime_adjust: f64,
    idle_time: f64,
    ce: ClockEstimate,
    last_receive_sent_time: f64,
    send_seq: u64,
    receive_seq: u64,
    ignore_nak_seq: u64,
    last_ack_seq: u64,
    retransmit_seq: u64,
    rtt_sample_seq: u64,
    sent_queue: VecDeque<Box<QueueMessage>>,
    srtt: f64,
    rttvar: f64,
    rto: f64,
    ready_queues: VecDeque<Arc<CommandQueue>>,
    ready_bytes: i32,
    need_ack_bytes: i32,
    last_ack_bytes: i32,
    notify_queue: VecDeque<Box<QueueMessage>>,
    last_write_fail_time: f64,
    fast_readers: Vec<FastReaderEntry>,
    old_sent: VecDeque<Box<QueueMessage>>,
    dbg: SqDebug,
    bytes_write: u32,
    bytes_read: u32,
    bytes_retransmit: u32,
    bytes_invalid: u32,
}

/// State guarded by `transmit_requests.lock`.
struct TransmitState {
    upcoming_queues: VecDeque<Arc<CommandQueue>>,
    upcoming_bytes: i32,
    need_kick_clock: u64,
    min_release_clock: u64,
}

/// State guarded by `receiver.lock`.
struct ReceiverState {
    queue: VecDeque<Box<QueueMessage>>,
    old_receive: VecDeque<Box<QueueMessage>>,
    waiting: bool,
}

/// Input buffer state. In the C the input buffer is only touched by
/// the background thread; the port takes a short uncontended mutex so
/// the buffer can be shared as `&Inner` without `UnsafeCell`. The
/// lock is never held across a blocking call or a callback.
struct InputState {
    input_buf: [u8; 4096],
    input_pos: usize,
    need_sync: u8,
}

/// Shared inner state (the C `struct serialqueue` minus the
/// pollreactor, which the port owns separately).
pub(crate) struct Inner {
    name: [u8; 16],
    serial_fd: i32,
    serial_fd_type: u8,
    client_id: i32,
    pipe_rfd: i32,
    pipe_wfd: i32,
    input: sync::Mutex<InputState>,
    receiver: sync::Mutex<ReceiverState>,
    receiver_cond: sync::Condvar,
    transmit: sync::Mutex<TransmitState>,
    queue: sync::Mutex<QueueState>,
    dispatch_lock: sync::Mutex<()>,
}

/// Message returned by [`SerialQueue::pull`] /
/// [`SerialQueue::extract_old`] (the C `struct pull_queue_message`).
#[derive(Debug, Clone, Copy)]
pub struct PullQueueMessage {
    /// Message bytes.
    pub msg: [u8; msgblock::MESSAGE_MAX],
    /// Message length in bytes (-1 is the C "no message" sentinel on
    /// pull exit; the port returns `None` there instead).
    pub len: i32,
    /// Time the payload was (last) received by the mcu estimate.
    pub sent_time: f64,
    /// Time the message arrived locally (`get_monotonic()` based).
    pub receive_time: f64,
    /// Notification id (for acks that complete a `send_wait_ack`).
    pub notify_id: u64,
}

// ---- helpers -----------------------------------------------------------

/// Port of `calculate_bittime` (`DIV_ROUND_UP(bytes, 8)` is the C
/// `compiler.h` macro).
fn calculate_bittime(inner: &Inner, q: &QueueState, bytes: u32) -> f64 {
    if inner.serial_fd_type == SQT_CAN {
        let pkts = bytes.wrapping_add(7) / 8;
        let bits =
            bytes.wrapping_mul(8).wrapping_add(pkts.wrapping_mul(CANBUS_PACKET_BITS));
        q.bittime_adjust * (bits.wrapping_sub(CANBUS_IFS_BITS) as f64)
    } else {
        q.bittime_adjust * (bytes as f64)
    }
}

/// Debug queue with 100 pre-allocated zeroed messages, as after the C
/// `debug_queue_alloc` at `serialqueue_alloc` time.
fn debug_queue() -> VecDeque<Box<QueueMessage>> {
    let mut v = VecDeque::new();
    for _ in 0..DEBUG_QUEUE_SENT {
        v.push_back(Box::new(QueueMessage::new()));
    }
    v
}

/// Port of `debug_queue_add` (`_debug_queue_add`): append at the tail,
/// evict and free the head (the port's `Box` drop is the free).
fn debug_queue_add(queue: &mut VecDeque<Box<QueueMessage>>, qm: Box<QueueMessage>) {
    queue.push_back(qm);
    let _evicted = queue.pop_front();
}

/// Record a send-path decision (port of `sq_debug_note`). Only called
/// from the background thread with `sq->lock` held.
#[allow(clippy::too_many_arguments)]
fn sq_debug_note_mut(
    inner: &Inner,
    pr: &PollReactor<Arc<Inner>>,
    q: &mut QueueState,
    reason: u8,
    flags: u8,
    eventtime: f64,
    waketime: f64,
    idletime: f64,
    ack_clock: u64,
    want_clock: u64,
    pending: i32,
    result: i32,
) {
    let upcoming_bytes = inner.transmit.lock().upcoming_bytes;
    let dbg = &mut q.dbg;
    dbg.total = dbg.total.wrapping_add(1);
    dbg.reason_count[reason as usize] = dbg.reason_count[reason as usize].wrapping_add(1);
    let ev = SqDebugEvent {
        eventtime,
        last_eventtime: eventtime,
        sched: pr.get_timer(SQPT_COMMAND),
        waketime,
        idletime,
        ack_clock,
        want_clock,
        send_seq: q.send_seq,
        receive_seq: q.receive_seq,
        ready_bytes: q.ready_bytes as u32,
        upcoming_bytes: upcoming_bytes as u32,
        need_ack_bytes: q.need_ack_bytes as u32,
        pending,
        result,
        repeat: 1,
        reason,
        flags,
    };
    sq_debug_push(&mut dbg.ring, &mut dbg.pos, &mut dbg.count, ev);
    if (1u32 << reason) & SQDBG_NOTABLE_MASK != 0 {
        sq_debug_push(&mut dbg.notable, &mut dbg.notable_pos, &mut dbg.notable_count, ev);
    }
}

// ---- receiver / kick ----------------------------------------------------

/// Add messages and wake up the receiver thread if it is waiting
/// (port of `receive_append_wake`; `msgs` is `list_join_tail`ed onto
/// the receiver queue).
fn receive_append_wake(inner: &Inner, msgs: VecDeque<Box<QueueMessage>>) {
    let dokick = {
        let mut recv = inner.receiver.lock();
        for m in msgs {
            recv.queue.push_back(m);
        }
        if recv.waiting {
            recv.waiting = false;
            true
        } else {
            false
        }
    };
    if dokick {
        inner.receiver_cond.notify_one();
    }
}

/// Write to the internal pipe to wake the background thread if in poll
/// (port of `kick_bg_thread`).
fn kick_bg_thread(inner: &Inner) {
    if fd::write(inner.pipe_wfd, b".").is_err() {
        crate::report_errno("pipe write", -1, last_errno());
    }
}

fn last_errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

// ---- receive path ---------------------------------------------------------

/// Update internal state when the receive sequence increases (port
/// of `update_receive_seq`).
fn update_receive_seq(
    inner: &Inner,
    pr: &PollReactor<Arc<Inner>>,
    q: &mut QueueState,
    eventtime: f64,
    rseq: u64,
) {
    // Remove from sent queue
    let mut sent_seq = q.receive_seq;
    loop {
        if q.sent_queue.is_empty() {
            // Got an ack for a message not sent; must be connection init
            q.send_seq = rseq;
            q.last_receive_sent_time = 0.;
            break;
        }
        let sent = q.sent_queue.pop_front().unwrap();
        let sent_len = sent.len;
        let sent_receive_time = sent.receive_time;
        q.need_ack_bytes -= sent.len;
        debug_queue_add(&mut q.old_sent, sent);
        sent_seq = sent_seq.wrapping_add(1);
        if rseq == sent_seq {
            // Found sent message corresponding with the received sequence
            q.last_receive_sent_time = sent_receive_time;
            q.last_ack_bytes = sent_len;
            break;
        }
    }
    q.receive_seq = rseq;
    pr.update_timer(SQPT_COMMAND, PR_NOW);

    // Update retransmit info
    if q.rtt_sample_seq != 0 && rseq > q.rtt_sample_seq && q.last_receive_sent_time != 0. {
        // RFC6298 rtt calculations
        let delta = eventtime - q.last_receive_sent_time;
        if q.srtt == 0. {
            q.rttvar = delta / 2.0;
            q.srtt = delta * 10.0; // use a higher start default
        } else {
            q.rttvar = (3.0 * q.rttvar + kos::libm::fabs(q.srtt - delta)) / 4.0;
            q.srtt = (7.0 * q.srtt + delta) / 8.0;
        }
        let mut rttvar4 = q.rttvar * 4.0;
        if rttvar4 < 0.001 {
            rttvar4 = 0.001;
        }
        // C: `if (rto < MIN_RTO) rto = MIN_RTO; else if (rto > MAX_RTO)
        // rto = MAX_RTO;` — `clamp` matches, including NaN (both
        // comparisons are false for NaN, and `clamp` passes NaN
        // through).
        q.rto = (q.srtt + rttvar4).clamp(MIN_RTO, MAX_RTO);
        q.rtt_sample_seq = 0;
    }
    if q.sent_queue.is_empty() {
        pr.update_timer(SQPT_RETRANSMIT, PR_NEVER);
    } else {
        let sent_len = q.sent_queue.front().map(|m| m.len).unwrap_or(0);
        let nr = eventtime + q.rto + calculate_bittime(inner, q, sent_len as u32);
        pr.update_timer(SQPT_RETRANSMIT, nr);
    }
}

/// Process a well-formed input message (port of `handle_message`).
fn handle_message(inner: &Inner, pr: &PollReactor<Arc<Inner>>, eventtime: f64, msg: &[u8]) {
    let len = msg.len() as i32;
    // Fast readers run last, after the main lock is released; find the
    // matching entry inside the lock, then dispatch below (the C
    // returns from `handle_message` after the callback).
    let matched_fast_reader = {
        let mut q = inner.queue.lock();

        // Calculate receive sequence number
        let rseq_delta =
            (msg[msgblock::MESSAGE_POS_SEQ] as u64).wrapping_sub(q.receive_seq) & 0x0f;
        let rseq = q.receive_seq.wrapping_add(rseq_delta);
        if rseq != q.receive_seq {
            // New sequence number
            if rseq > q.send_seq && q.receive_seq != 1 {
                // An ack for a message not sent?  Out of order message?
                q.bytes_invalid = q.bytes_invalid.wrapping_add(len as u32);
                return;
            }
            update_receive_seq(inner, pr, &mut q, eventtime, rseq);
        }
        q.bytes_read = q.bytes_read.wrapping_add(len as u32);

        // Check for pending messages on notify_queue
        let wake_seq = rseq.wrapping_sub(1).wrapping_sub(if len > msgblock::MESSAGE_MIN as i32 { 1 } else { 0 });
        let mut received: VecDeque<Box<QueueMessage>> = VecDeque::new();
        while let Some(qm) = q.notify_queue.pop_front() {
            let notify_msg_sent_seq = qm.req_clock;
            if notify_msg_sent_seq > wake_seq {
                q.notify_queue.push_front(qm);
                break;
            }
            let mut qm = qm;
            qm.len = 0;
            qm.sent_time = q.last_receive_sent_time;
            qm.receive_time = eventtime;
            received.push_back(qm);
        }

        // Process message
        if len == msgblock::MESSAGE_MIN as i32 {
            // Ack/nak message
            if q.last_ack_seq < rseq {
                q.last_ack_seq = rseq;
            } else if rseq > q.ignore_nak_seq && !q.sent_queue.is_empty() {
                // Duplicate Ack is a Nak - do fast retransmit
                pr.update_timer(SQPT_RETRANSMIT, PR_NOW);
            }
        } else {
            // Data message - add to receive queue
            let mut qm = message_fill(msg, len);
            qm.sent_time =
                if rseq > q.retransmit_seq { q.last_receive_sent_time } else { 0. };
            // Must be time post read()
            qm.receive_time = time::monotonic();
            let bittime = calculate_bittime(inner, &q, len as u32);
            qm.receive_time -= bittime;
            received.push_back(qm);
        }

        if !received.is_empty() {
            receive_append_wake(inner, received);
        }

        // Check fast readers
        let mut found = None;
        for fr in q.fast_readers.iter() {
            let prefix_len = fr.prefix.len() as i32;
            if len < prefix_len + msgblock::MESSAGE_MIN as i32 {
                continue;
            }
            let plen = prefix_len as usize;
            if msg[msgblock::MESSAGE_HEADER_SIZE..msgblock::MESSAGE_HEADER_SIZE + plen]
                != fr.prefix[..]
            {
                continue;
            }
            // Protect dispatch before releasing sq->lock, so removal
            // cannot return while this callback is pending.
            found = Some((fr.func, fr.ctx, inner.dispatch_lock.lock()));
            break;
        }
        found
    }; // `sq->lock` released here (the C releases it before dispatching)

    if let Some((func, ctx, _dispatch)) = matched_fast_reader {
        let mut data = [0; msgblock::MESSAGE_MAX];
        data[..msg.len()].copy_from_slice(msg);
        // SAFETY: removal waits on the dispatch lock before freeing ctx.
        func(ctx as *mut c_void, eventtime, data.as_mut_ptr(), len);
    }
}

/// Callback for input activity on the serial fd (port of
/// `input_event`).
fn input_event(inner: &Inner, pr: &PollReactor<Arc<Inner>>, eventtime: f64) {
    if inner.serial_fd_type == SQT_CAN {
        let mut cf = [0u8; 16];
        match fd::read(inner.serial_fd, &mut cf) {
            Ok(n) if n > 0 => {
                let can_id = u32::from_le_bytes([cf[0], cf[1], cf[2], cf[3]]);
                if can_id != (inner.client_id as u32).wrapping_add(1) {
                    return;
                }
                let can_dlc = cf[4] as usize;
                let mut st = inner.input.lock();
                let pos = st.input_pos;
                let avail = st.input_buf.len().saturating_sub(pos);
                let n = can_dlc.min(avail);
                st.input_buf[pos..pos + n].copy_from_slice(&cf[8..8 + n]);
                st.input_pos += n;
            }
            Ok(_) => {
                crate::report_errno("can read", 0, last_errno());
                pr.do_exit();
                return;
            }
            Err(_) => {
                crate::report_errno("can read", -1, last_errno());
                pr.do_exit();
                return;
            }
        }
    } else {
        // Single read of whatever the nonblocking fd has (the C reads
        // straight into `input_buf + input_pos`). Map the result
        // onto the C `ret` (`> 0` bytes, `0` for EOF, `< 0` for
        // error) so the EOF/error branches behave identically.
        let ret = {
            let mut st = inner.input.lock();
            let end = st.input_pos;
            let spare = &mut st.input_buf[end..];
            match fd::read(inner.serial_fd, spare) {
                Ok(n) if n > 0 => {
                    st.input_pos += n;
                    n as i32
                }
                Ok(_) => 0,
                Err(_) => -1,
            }
        };
        if ret <= 0 {
            if ret < 0 {
                crate::report_errno("read", -1, last_errno());
            } else {
                crate::errorf("Got EOF when reading from device");
            }
            pr.do_exit();
            return;
        }
    }
    loop {
        let action = {
            let mut st = inner.input.lock();
            let pos = st.input_pos;
            let mut ns = st.need_sync;
            let len = msgblock::msgblock_check(&mut ns, &st.input_buf[..pos]);
            st.need_sync = ns;
            if len == 0 {
                // Need more data
                return;
            }
            if len > 0 {
                let l = len as usize;
                let mut msg = [0; msgblock::MESSAGE_MAX];
                msg[..l].copy_from_slice(&st.input_buf[..l]);
                st.input_buf.copy_within(l..pos, 0);
                st.input_pos = pos - l;
                Ok((msg, l))
            } else {
                // Skip bad data at beginning of input
                let skip = (-len) as usize;
                st.input_buf.copy_within(skip..pos, 0);
                st.input_pos = pos - skip;
                Err(skip)
            }
        };
        match action {
            Ok((msg, len)) => handle_message(inner, pr, eventtime, &msg[..len]),
            Err(skip) => {
                let mut q = inner.queue.lock();
                q.bytes_invalid = q.bytes_invalid.wrapping_add(skip as u32);
            }
        }
    }
}

/// Callback for input activity on the pipe fd (port of `kick_event`).
fn kick_event(inner: &Inner, pr: &PollReactor<Arc<Inner>>, _eventtime: f64) {
    let mut dummy = [0u8; 4096];
    if fd::read(inner.pipe_rfd, &mut dummy).is_err() {
        crate::report_errno("pipe read", -1, last_errno());
    }
    pr.update_timer(SQPT_COMMAND, PR_NOW);
}

// ---- transmit path -------------------------------------------------------

/// OS write of data to be sent to the mcu (port of `do_write`; callers
/// hold `sq->lock`).
fn do_write(
    inner: &Inner,
    pr: &PollReactor<Arc<Inner>>,
    q: &mut QueueState,
    buf: &[u8],
    eventtime: f64,
) {
    if inner.serial_fd_type != SQT_CAN {
        let buflen = buf.len() as i32;
        match fd::write(inner.serial_fd, buf) {
            Ok(n) => {
                // Only failures and short writes are recorded; a full
                // write is already implied by the preceding "send"
                // entries.
                if n != buf.len() {
                    sq_debug_note_mut(
                        inner,
                        pr,
                        q,
                        SQDBG_WRITE,
                        SQDBGF_WRITE_FAIL,
                        eventtime,
                        PR_NOW,
                        0.,
                        0,
                        0,
                        buflen,
                        n as i32,
                    );
                }
            }
            Err(_) => {
                crate::report_errno("write", -1, last_errno());
                sq_debug_note_mut(
                    inner,
                    pr,
                    q,
                    SQDBG_WRITE,
                    SQDBGF_WRITE_FAIL,
                    eventtime,
                    PR_NOW,
                    0.,
                    0,
                    0,
                    buflen,
                    -1,
                );
            }
        }
        return;
    }
    // Write to CAN fd
    let mut rest = buf;
    while !rest.is_empty() {
        let size = if rest.len() > 8 { 8 } else { rest.len() };
        let mut cf = [0u8; 16];
        cf[0..4].copy_from_slice(&(inner.client_id as u32).to_le_bytes());
        cf[4] = size as u8;
        cf[8..8 + size].copy_from_slice(&rest[..size]);
        match fd::write(inner.serial_fd, &cf) {
            Ok(_) => {
                q.last_write_fail_time = 0.0;
                rest = &rest[size..];
            }
            Err(_) => {
                crate::report_errno("can write", -1, last_errno());
                let curtime = time::monotonic();
                let buflen = rest.len() as i32;
                sq_debug_note_mut(
                    inner,
                    pr,
                    q,
                    SQDBG_WRITE,
                    SQDBGF_WRITE_FAIL | SQDBGF_CAN,
                    eventtime,
                    curtime,
                    0.,
                    0,
                    0,
                    buflen,
                    -1,
                );
                if q.last_write_fail_time == 0. {
                    q.last_write_fail_time = curtime;
                } else if curtime > q.last_write_fail_time + 10.0 {
                    crate::errorf("Halting reads due to CAN write errors.");
                    sq_debug_note_mut(
                        inner,
                        pr,
                        q,
                        SQDBG_EXIT,
                        SQDBGF_WRITE_FAIL | SQDBGF_CAN,
                        eventtime,
                        curtime,
                        q.last_write_fail_time,
                        0,
                        0,
                        buflen,
                        -1,
                    );
                    pr.do_exit();
                }
                return;
            }
        }
    }
}

/// Callback timer for when a retransmit should be done (port of
/// `retransmit_event`; returns the next waketime).
fn retransmit_event(inner: &Inner, pr: &PollReactor<Arc<Inner>>, eventtime: f64) -> f64 {
    if inner.serial_fd_type == SQT_UART && termios::tcflush(inner.serial_fd, TCOFLUSH).is_err() {
        crate::report_errno("tcflush", -1, last_errno());
    }

    let mut q = inner.queue.lock();

    // Retransmit all pending messages
    let mut buf = [0u8; msgblock::MESSAGE_MAX * 12 + 1];
    let mut buflen = 0_usize;
    buf[buflen] = msgblock::MESSAGE_SYNC;
    buflen += 1;
    let mut first_buflen = 0_usize;
    for qm in q.sent_queue.iter() {
        let l = qm.len as usize;
        buf[buflen..buflen + l].copy_from_slice(&qm.msg[..l]);
        buflen += l;
        if first_buflen == 0 {
            first_buflen = l + 1;
        }
    }
    do_write(inner, pr, &mut q, &buf[..buflen], eventtime);
    q.bytes_retransmit = q.bytes_retransmit.wrapping_add(buflen as u32);

    // Update rto
    let mut rflags = 0_u8;
    if pr.get_timer(SQPT_RETRANSMIT) == PR_NOW {
        // Retransmit due to nak
        rflags = SQDBGF_NAK;
        q.ignore_nak_seq = q.receive_seq;
        if q.receive_seq < q.retransmit_seq {
            // Second nak for this retransmit - don't allow third
            q.ignore_nak_seq = q.retransmit_seq;
        }
    } else {
        // Retransmit due to timeout
        q.rto *= 2.0;
        if q.rto > MAX_RTO {
            q.rto = MAX_RTO;
        }
        q.ignore_nak_seq = q.send_seq;
    }
    q.retransmit_seq = q.send_seq;
    q.rtt_sample_seq = 0;
    q.idle_time = eventtime + calculate_bittime(inner, &q, buflen as u32);
    let waketime = eventtime + q.rto + calculate_bittime(inner, &q, first_buflen as u32);
    let idle_time = q.idle_time;
    sq_debug_note_mut(
        inner,
        pr,
        &mut q,
        SQDBG_RETRANSMIT,
        rflags,
        eventtime,
        waketime,
        idle_time,
        0,
        0,
        first_buflen as i32,
        buflen as i32,
    );
    waketime
}

/// Construct a block of data to be sent to the serial port (port of
/// `build_and_send_command`; `buf` is the scratch space starting at
/// the current `pending` offset, `pending` is `buflen` so far).
fn build_and_send_command(
    inner: &Inner,
    pr: &PollReactor<Arc<Inner>>,
    q: &mut QueueState,
    buf: &mut [u8],
    pending: i32,
    eventtime: f64,
) -> i32 {
    let mut len = msgblock::MESSAGE_HEADER_SIZE as i32;
    let mut msgs = 0_i32;
    let mut first_req_clock = 0_u64;
    while q.ready_bytes != 0 {
        // Find highest priority message (message with lowest req_clock)
        let mut min_clock = MAX_CLOCK;
        let mut best: Option<usize> = None;
        for (i, cq) in q.ready_queues.iter().enumerate() {
            let queues = cq.queues.lock();
            if let Some(m) = queues.ready.front() {
                if m.req_clock < min_clock {
                    min_clock = m.req_clock;
                    best = Some(i);
                }
            }
        }
        let best = match best {
            Some(b) => b,
            None => break,
        };
        let cq = Arc::clone(&q.ready_queues[best]);
        let qm_len = cq.queues.lock().ready.front().map(|m| m.len).unwrap_or(0);
        // Append message to outgoing command
        if len + qm_len > (msgblock::MESSAGE_MAX - msgblock::MESSAGE_TRAILER_SIZE) as i32 {
            break;
        }
        if msgs == 0 {
            first_req_clock = min_clock;
        }
        msgs += 1;
        let qm = {
            let mut queues = cq.queues.lock();
            queues.ready.pop_front().unwrap()
        };
        if cq.queues.lock().ready.is_empty() {
            q.ready_queues.remove(best);
        }
        let l = qm.len as usize;
        buf[len as usize..len as usize + l].copy_from_slice(&qm.msg[..l]);
        len += qm.len;
        q.ready_bytes -= qm.len;
        if qm.notify_id != 0 {
            // Message requires notification - add to notify list
            let mut qm = qm;
            qm.req_clock = q.send_seq;
            q.notify_queue.push_back(qm);
        }
        // else: freed (the Box drops here)
    }

    // Fill header / trailer
    len += msgblock::MESSAGE_TRAILER_SIZE as i32;
    buf[msgblock::MESSAGE_POS_LEN] = len as u8;
    buf[msgblock::MESSAGE_POS_SEQ] =
        msgblock::MESSAGE_DEST | ((q.send_seq & msgblock::MESSAGE_SEQ_MASK) as u8);
    let crc = crc16_ccitt(&buf[..(len - msgblock::MESSAGE_TRAILER_SIZE as i32) as usize]);
    buf[len as usize - msgblock::MESSAGE_TRAILER_CRC] = (crc >> 8) as u8;
    buf[len as usize - msgblock::MESSAGE_TRAILER_CRC + 1] = (crc & 0xff) as u8;
    buf[len as usize - msgblock::MESSAGE_TRAILER_SYNC] = msgblock::MESSAGE_SYNC;

    // Store message block
    let mut idletime = if eventtime > q.idle_time { eventtime } else { q.idle_time };
    let bittime = calculate_bittime(inner, q, (pending + len) as u32);
    idletime += bittime;
    let mut out = QueueMessage::new();
    out.msg[..len as usize].copy_from_slice(&buf[..len as usize]);
    out.len = len;
    out.sent_time = eventtime;
    out.receive_time = idletime;
    if q.sent_queue.is_empty() {
        pr.update_timer(SQPT_RETRANSMIT, idletime + q.rto);
    }
    if q.rtt_sample_seq == 0 {
        q.rtt_sample_seq = q.send_seq;
    }
    // Recorded before send_seq/need_ack_bytes are updated, so send_seq
    // is this block's own sequence number.
    sq_debug_note_mut(
        inner,
        pr,
        q,
        SQDBG_SEND,
        0,
        eventtime,
        PR_NOW,
        idletime,
        0,
        first_req_clock,
        pending,
        len,
    );
    q.send_seq = q.send_seq.wrapping_add(1);
    q.need_ack_bytes += len;
    q.sent_queue.push_back(Box::new(out));
    len
}

/// Move messages from upcoming queues to ready queues (port of
/// `check_upcoming_queues`; the caller holds `sq->lock`).
fn check_upcoming_queues(inner: &Inner, q: &mut QueueState, ack_clock: u64) -> u64 {
    let mut tx = inner.transmit.lock();
    tx.need_kick_clock = 0;
    let min_release_clock = tx.min_release_clock;
    if ack_clock < min_release_clock {
        return min_release_clock;
    }

    let mut min_stalled_clock = MAX_CLOCK;
    let mut i = 0_usize;
    while i < tx.upcoming_queues.len() {
        let cq = Arc::clone(&tx.upcoming_queues[i]);
        let not_in_ready_queues = cq.queues.lock().ready.is_empty();
        // Move messages from the upcoming.msg_queue to the ready.msg_queue
        {
            let mut queues = cq.queues.lock();
            while let Some(m) = queues.upcoming.front() {
                let next_min = m.min_clock;
                if ack_clock < next_min {
                    if next_min < min_stalled_clock {
                        min_stalled_clock = next_min;
                    }
                    break;
                }
                let m = queues.upcoming.pop_front().unwrap();
                tx.upcoming_bytes -= m.len;
                q.ready_bytes += m.len;
                queues.ready.push_back(m);
            }
        }
        // Remove cq from the list if it is now empty
        if cq.queues.lock().upcoming.is_empty() {
            tx.upcoming_queues.remove(i);
        } else {
            i += 1;
        }
        // Add to ready queues
        if not_in_ready_queues && !cq.queues.lock().ready.is_empty() {
            q.ready_queues.push_back(cq);
        }
    }
    tx.min_release_clock = min_stalled_clock;
    min_stalled_clock
}

/// Set the next transmit queue need_kick_clock (port of
/// `update_need_kick_clock`).
fn update_need_kick_clock(inner: &Inner, wantclock: u64) -> i32 {
    let mut tx = inner.transmit.lock();
    if wantclock > tx.min_release_clock {
        return -1;
    }
    tx.need_kick_clock = wantclock;
    0
}

/// Determine if ready to send commands (or the amount of time to sleep
/// if not) (port of `check_send_command`).
fn check_send_command(
    inner: &Inner,
    pr: &PollReactor<Arc<Inner>>,
    q: &mut QueueState,
    pending: i32,
    eventtime: f64,
) -> f64 {
    // Check for upcoming messages now ready
    let mut idletime = if eventtime > q.idle_time { eventtime } else { q.idle_time };
    let bittime = calculate_bittime(inner, q, (pending + msgblock::MESSAGE_MIN as i32) as u32);
    idletime += bittime;
    let ack_clock = msgblock::clock_from_time(&q.ce, idletime);
    let min_stalled_clock = check_upcoming_queues(inner, q, ack_clock);

    // Check if valid to send messages
    if q.send_seq.wrapping_sub(q.receive_seq) >= MAX_PENDING_BLOCKS && q.receive_seq != u64::MAX {
        // Need an ack before more messages can be sent
        sq_debug_note_mut(
            inner,
            pr,
            q,
            SQDBG_ACKWAIT,
            0,
            eventtime,
            eventtime + 0.250,
            idletime,
            ack_clock,
            min_stalled_clock,
            pending,
            0,
        );
        return eventtime + 0.250;
    }
    if q.send_seq > q.receive_seq && q.receive_window != 0 {
        let mut need_ack_bytes = q.need_ack_bytes + msgblock::MESSAGE_MAX as i32;
        if q.last_ack_seq < q.receive_seq {
            need_ack_bytes += q.last_ack_bytes;
        }
        if need_ack_bytes > q.receive_window {
            // Wait for ack from past messages before sending next message
            sq_debug_note_mut(
                inner,
                pr,
                q,
                SQDBG_WINDOW,
                0,
                eventtime,
                eventtime + 0.250,
                idletime,
                ack_clock,
                min_stalled_clock,
                pending,
                need_ack_bytes,
            );
            return eventtime + 0.250;
        }
    }

    // Check if a block is fully ready to send
    if q.ready_bytes >= msgblock::MESSAGE_PAYLOAD_MAX as i32 {
        return PR_NOW;
    }
    if q.ce.est_freq == 0. {
        // Clock unknown during initial startup - recheck on each add
        if q.ready_bytes != 0 {
            return PR_NOW;
        }
        let mustwake = update_need_kick_clock(inner, 1);
        sq_debug_note_mut(
            inner,
            pr,
            q,
            SQDBG_NOCLOCK,
            SQDBGF_NOCLOCK
                | SQDBGF_READY_EMPTY
                | if mustwake != 0 { SQDBGF_MUSTWAKE } else { 0 },
            eventtime,
            if mustwake != 0 { eventtime } else { PR_NEVER },
            idletime,
            ack_clock,
            0,
            pending,
            mustwake,
        );
        if mustwake != 0 {
            return eventtime;
        }
        return PR_NEVER;
    }

    // Check if it is still needed to send messages from the ready_queues
    let mut min_ready_clock = MAX_CLOCK;
    for cq in q.ready_queues.iter() {
        let queues = cq.queues.lock();
        if let Some(m) = queues.ready.front() {
            // Update min_ready_clock
            let mut req_clock = m.req_clock;
            let bgtime = if pending != 0 { idletime } else { q.idle_time };
            let bgoffset = MIN_REQTIME_DELTA + MIN_BACKGROUND_DELTA;
            if req_clock == BACKGROUND_PRIORITY_CLOCK {
                req_clock = msgblock::clock_from_time(&q.ce, bgtime + bgoffset);
            }
            if req_clock < min_ready_clock {
                min_ready_clock = req_clock;
            }
        }
    }
    let reqclock_delta = cconv::f64_to_u64(MIN_REQTIME_DELTA * q.ce.est_freq);
    if min_ready_clock <= ack_clock.wrapping_add(reqclock_delta) {
        return PR_NOW;
    }

    // Determine next wakeup time
    if pending != 0 {
        // Caller wont sleep anyway - just return
        sq_debug_note_mut(
            inner,
            pr,
            q,
            SQDBG_PENDING,
            0,
            eventtime,
            eventtime,
            idletime,
            ack_clock,
            min_ready_clock,
            pending,
            0,
        );
        return eventtime;
    }
    let mut wantclock = min_ready_clock.wrapping_sub(reqclock_delta);
    // Classify the wait: nothing queued at all, gated by a message
    // min_clock, or waiting for a req_clock to enter the send window
    let mut reason = SQDBG_REQWAIT;
    let mut flags = 0_u8;
    if min_ready_clock == MAX_CLOCK {
        reason = SQDBG_NOWORK;
        flags = SQDBGF_READY_EMPTY;
    }
    if min_stalled_clock < wantclock {
        wantclock = min_stalled_clock;
        reason = SQDBG_MINCLOCK;
        flags |= SQDBGF_STALLED;
    }
    let mustwake = update_need_kick_clock(inner, wantclock);
    if mustwake != 0 {
        // Raced with add of new command - avoid sleeping
        sq_debug_note_mut(
            inner,
            pr,
            q,
            SQDBG_RACE,
            flags | SQDBGF_MUSTWAKE,
            eventtime,
            eventtime,
            idletime,
            ack_clock,
            wantclock,
            pending,
            0,
        );
        return eventtime;
    }
    let waketime = idletime + ((wantclock.wrapping_sub(ack_clock) as f64) / q.ce.est_freq);
    sq_debug_note_mut(
        inner,
        pr,
        q,
        reason,
        flags,
        eventtime,
        waketime,
        idletime,
        ack_clock,
        wantclock,
        pending,
        0,
    );
    waketime
}

/// Callback timer to send data to the serial port (port of
/// `command_event`; returns the next waketime).
fn command_event(inner: &Inner, pr: &PollReactor<Arc<Inner>>, eventtime: f64) -> f64 {
    let mut q = inner.queue.lock();
    let mut buf = [0u8; msgblock::MESSAGE_MAX * MAX_PENDING_BLOCKS as usize];
    let mut buflen = 0_i32;
    let mut waketime: f64;
    loop {
        waketime = check_send_command(inner, pr, &mut q, buflen, eventtime);
        if waketime != PR_NOW {
            break;
        }
        let sent = build_and_send_command(
            inner,
            pr,
            &mut q,
            &mut buf[buflen as usize..],
            buflen,
            eventtime,
        );
        buflen += sent;
        if buflen + msgblock::MESSAGE_MAX as i32 > buf.len() as i32 {
            break;
        }
    }
    if buflen != 0 {
        // Write message blocks
        do_write(inner, pr, &mut q, &buf[..buflen as usize], eventtime);
        q.bytes_write = q.bytes_write.wrapping_add(buflen as u32);
        let idletime = if eventtime > q.idle_time { eventtime } else { q.idle_time };
        let bittime = calculate_bittime(inner, &q, buflen as u32);
        q.idle_time = idletime + bittime;
        waketime = PR_NOW;
    }
    waketime
}

/// Main background thread (port of `background_thread`).
fn background_thread(inner: Arc<Inner>, pr: Arc<PollReactor<Arc<Inner>>>) {
    pr.run(&inner);

    // Wake any waiting receivers
    receive_append_wake(&inner, VecDeque::new());
}

// ---- public API ------------------------------------------------------------

/// Port of `struct serialqueue` (see `serialqueue.h` for the C
/// declarations).
pub struct SerialQueue {
    inner: Arc<Inner>,
    pr: Arc<PollReactor<Arc<Inner>>>,
    bg: sync::Mutex<Option<thread::JoinHandle>>,
}

impl SerialQueue {
    /// Port of `serialqueue_alloc`. Returns `None` if the internal
    /// pipe cannot be created (the C `goto fail`).
    pub fn alloc(serial_fd: i32, serial_fd_type: u8, client_id: i32, name: &[u8]) -> Option<SerialQueue> {
        let (pipe_rfd, pipe_wfd) = match fd::pipe() {
            Ok(p) => p,
            Err(_) => {
                crate::report_errno("init", -1, last_errno());
                return None;
            }
        };

        // `strncpy(sq->name, name, 16); sq->name[15] = '\0';`
        let mut name_buf = [0u8; 16];
        let n = name.len().min(16);
        name_buf[..n].copy_from_slice(&name[..n]);
        name_buf[15] = 0;

        let inner = Arc::new(Inner {
            name: name_buf,
            serial_fd,
            serial_fd_type,
            client_id,
            pipe_rfd,
            pipe_wfd,
            input: sync::Mutex::new(InputState {
                input_buf: [0; 4096],
                input_pos: 0,
                need_sync: 0,
            }),
            receiver: sync::Mutex::new(ReceiverState {
                queue: VecDeque::new(),
                old_receive: debug_queue(),
                waiting: false,
            }),
            receiver_cond: sync::Condvar::new(),
            transmit: sync::Mutex::new(TransmitState {
                upcoming_queues: VecDeque::new(),
                upcoming_bytes: 0,
                need_kick_clock: MAX_CLOCK,
                min_release_clock: MAX_CLOCK,
            }),
            queue: sync::Mutex::new(QueueState {
                receive_window: 0,
                bittime_adjust: 0.0,
                idle_time: 0.0,
                ce: ClockEstimate { conv_clock: 0, conv_time: 0.0, est_freq: 0.0 },
                last_receive_sent_time: 0.0,
                send_seq: 1,
                receive_seq: if serial_fd_type == SQT_DEBUGFILE { u64::MAX } else { 1 },
                ignore_nak_seq: 0,
                last_ack_seq: 0,
                retransmit_seq: 0,
                rtt_sample_seq: 0,
                sent_queue: VecDeque::new(),
                srtt: 0.0,
                rttvar: 0.0,
                rto: if serial_fd_type == SQT_DEBUGFILE { PR_NEVER } else { MIN_RTO },
                ready_queues: VecDeque::new(),
                ready_bytes: 0,
                need_ack_bytes: 0,
                last_ack_bytes: 0,
                notify_queue: VecDeque::new(),
                last_write_fail_time: 0.0,
                fast_readers: Vec::new(),
                old_sent: debug_queue(),
                dbg: SqDebug::default(),
                bytes_write: 0,
                bytes_read: 0,
                bytes_retransmit: 0,
                bytes_invalid: 0,
            }),
            dispatch_lock: sync::Mutex::new(()),
        });

        let pr = Arc::new(PollReactor::<Arc<Inner>>::new(SQPF_NUM, SQPT_NUM));
        {
            let prc = Arc::downgrade(&pr);
            pr.add_fd(
                SQPF_SERIAL,
                serial_fd,
                serial_fd_type == SQT_DEBUGFILE,
                Box::new(move |c: &Arc<Inner>, t| input_event(c, &prc.upgrade().expect("running reactor"), t)),
            );
            let prc = Arc::downgrade(&pr);
            pr.add_fd(
                SQPF_PIPE,
                pipe_rfd,
                false,
                Box::new(move |c: &Arc<Inner>, t| {
                    kick_event(c, &prc.upgrade().expect("running reactor"), t);
                }),
            );
            let prc = Arc::downgrade(&pr);
            pr.add_timer(SQPT_RETRANSMIT, Box::new(move |c: &Arc<Inner>, t| retransmit_event(c, &prc.upgrade().expect("running reactor"), t)));
            let prc = Arc::downgrade(&pr);
            pr.add_timer(SQPT_COMMAND, Box::new(move |c: &Arc<Inner>, t| command_event(c, &prc.upgrade().expect("running reactor"), t)));
        }
        crate::pollreactor::fd_set_non_blocking(serial_fd);
        crate::pollreactor::fd_set_non_blocking(pipe_rfd);
        crate::pollreactor::fd_set_non_blocking(pipe_wfd);

        // Debugging: init banner (the C `errorf` in serialqueue_alloc).
        // `event_bytes`/`debug_bytes` are the C `sizeof` values for
        // `struct sq_debug_event` (96) / `struct sq_debug` (7752,
        // including its 4 bytes of tail padding).
        let name_str = String::from_utf8_lossy(name_until_nul(&name_buf)).into_owned();
        crate::errorf(&format!(
            "[DEBUG-motion] version=motion-v4 serial={name_str} init type={} client_id={client_id} ring=64 notable=16 event_bytes=96 debug_bytes=7752 enabled=1",
            serial_fd_type as char
        ));

        let thread_name = name_str.clone();
        let inner2 = Arc::clone(&inner);
        let pr2 = Arc::clone(&pr);
        let handle = thread::spawn_named(&thread_name, move || background_thread(inner2, pr2));

        Some(SerialQueue { inner, pr, bg: sync::Mutex::new(Some(handle)) })
    }

    /// Port of `serialqueue_exit`: stop the background thread and
    /// wait for it.
    pub fn exit(&self) {
        let handle = self.bg.lock().take();
        if let Some(handle) = handle {
            self.pr.do_exit();
            kick_bg_thread(&self.inner);
            handle.join();
        }
    }

    /// Port of `serialqueue_free`: exit if needed, then drop every
    /// internal queue.
    pub fn free(self) {
        drop(self);
    }

    /// Register a low-latency message handler (port of
    /// `serialqueue_add_fastreader`).
    ///
    /// # Safety
    /// The callback and `ctx` must stay valid while registered (the
    /// C contract: the fastreader object outlives its registration).
    pub unsafe fn add_fastreader(&self, func: FastReaderCb, prefix: &[u8], ctx: *mut c_void) {
        let mut q = self.inner.queue.lock();
        q.fast_readers.push(FastReaderEntry { func, prefix: prefix.to_vec(), ctx: ctx as usize });
    }

    /// Remove a previously registered low-latency message handler
    /// (port of `serialqueue_rm_fastreader`).
    ///
    /// # Safety
    /// Calls `add_fastreader`-matched pairs only; see there.
    pub unsafe fn rm_fastreader(&self, func: FastReaderCb, ctx: *mut c_void) {
        let ctx_usize = ctx as usize;
        {
            let mut q = self.inner.queue.lock();
            q.fast_readers.retain(|fr| !(fr.func as usize == func as usize && fr.ctx == ctx_usize));
        }
        let _dispatch = self.inner.dispatch_lock.lock();
    }

    /// Add a batch of messages to the given command queue (port of
    /// `serialqueue_send_batch`). Takes ownership of `msgs` (the C
    /// `list_head *msgs` is consumed).
    pub fn send_batch(&self, cq: &Arc<CommandQueue>, mut msgs: Vec<Box<QueueMessage>>) {
        // Make sure min_clock is set in list and calculate total bytes
        let mut len = 0_i32;
        for qm in msgs.iter_mut() {
            if qm.min_clock.wrapping_add(3 << 29) < qm.req_clock
                && qm.req_clock != BACKGROUND_PRIORITY_CLOCK
            {
                // Avoid mcu clock comparison 31-bit overflow issues
                qm.min_clock = qm.req_clock.wrapping_sub(3 << 29);
            }
            len += qm.len;
        }
        if len == 0 {
            return;
        }
        let min_clock = msgs[0].min_clock;

        // Add list to cq->upcoming_queue
        let mut mustwake = false;
        {
            let mut tx = self.inner.transmit.lock();
            let mut queues = cq.queues.lock();
            if queues.upcoming.is_empty() {
                tx.upcoming_queues.push_back(Arc::clone(cq));
                if min_clock < tx.min_release_clock {
                    tx.min_release_clock = min_clock;
                }
                if min_clock < tx.need_kick_clock {
                    tx.need_kick_clock = 0;
                    mustwake = true;
                }
            }
            for m in msgs.drain(..) {
                queues.upcoming.push_back(m);
            }
            tx.upcoming_bytes += len;
        }

        // Wake the background thread if necessary
        if mustwake {
            kick_bg_thread(&self.inner);
        }
    }

    /// Helper to send a single message (port of `serialqueue_send_one`).
    pub fn send_one(&self, cq: &Arc<CommandQueue>, qm: Box<QueueMessage>) {
        self.send_batch(cq, vec![qm]);
    }

    /// Schedule the transmission of a message at a given time and
    /// priority (port of `serialqueue_send`).
    pub fn send(
        &self,
        cq: &Arc<CommandQueue>,
        msg: &[u8],
        min_clock: u64,
        req_clock: u64,
        notify_id: u64,
    ) {
        let mut qm = QueueMessage::new();
        let n = msg.len().min(msgblock::MESSAGE_MAX);
        qm.msg[..n].copy_from_slice(&msg[..n]);
        qm.len = n as i32;
        qm.min_clock = min_clock;
        qm.req_clock = req_clock;
        qm.notify_id = notify_id;
        self.send_one(cq, Box::new(qm));
    }

    /// Return a message read from the serial port, or wait for one if
    /// none available. Returns `None` if the background thread
    /// exited with no message (the C `pqm->len = -1`).
    pub fn pull(&self) -> Option<PullQueueMessage> {
        let mut recv = self.inner.receiver.lock();
        // Wait for message to be available
        loop {
            if let Some(qm) = recv.queue.pop_front() {
                // Copy message
                let pqm = PullQueueMessage {
                    msg: qm.msg,
                    len: qm.len,
                    sent_time: qm.sent_time,
                    receive_time: qm.receive_time,
                    notify_id: qm.notify_id,
                };
                if qm.len != 0 {
                    debug_queue_add(&mut recv.old_receive, qm);
                }
                // else: `message_free(qm)` (the Box drops here)
                return Some(pqm);
            }
            if self.pr.is_exit() {
                return None;
            }
            recv.waiting = true;
            recv = self.inner.receiver_cond.wait(recv);
        }
    }

    /// Port of `serialqueue_set_wire_frequency`.
    pub fn set_wire_frequency(&self, frequency: f64) {
        let mut q = self.inner.queue.lock();
        if self.inner.serial_fd_type == SQT_CAN {
            q.bittime_adjust = 1. / frequency;
        } else {
            // An 8N1 serial line is 10 bits per byte (1 start, 8 data, 1 stop)
            q.bittime_adjust = 10. / frequency;
        }
    }

    /// Port of `serialqueue_set_receive_window`.
    pub fn set_receive_window(&self, receive_window: i32) {
        self.inner.queue.lock().receive_window = receive_window;
    }

    /// Port of `serialqueue_set_clock_est`.
    pub fn set_clock_est(&self, est_freq: f64, conv_time: f64, conv_clock: u64) {
        msgblock::clock_fill(&mut self.inner.queue.lock().ce, est_freq, conv_time, conv_clock);
    }

    /// Port of `serialqueue_get_clock_est`.
    pub fn get_clock_est(&self) -> ClockEstimate {
        self.inner.queue.lock().ce
    }

    /// Port of `serialqueue_get_stats`: writes the `key=value` stats
    /// string into `buf` with `snprintf` semantics (truncated at
    /// `buf.len() - 1`, NUL-terminated when `buf` is non-empty).
    pub fn get_stats(&self, buf: &mut [u8]) {
        // Snapshot under sq->lock then transmit lock (the C holds
        // both while copying), then format outside the locks.
        let (bytes_write, bytes_read, bytes_retransmit, bytes_invalid, send_seq, receive_seq,
             retransmit_seq, srtt, rttvar, rto, ready_bytes, upcoming_bytes) = {
            let q = self.inner.queue.lock();
            let tx = self.inner.transmit.lock();
            (
                q.bytes_write,
                q.bytes_read,
                q.bytes_retransmit,
                q.bytes_invalid,
                q.send_seq,
                q.receive_seq,
                q.retransmit_seq,
                q.srtt,
                q.rttvar,
                q.rto,
                q.ready_bytes,
                tx.upcoming_bytes,
            )
        };
        // C: `send_seq=%u` etc. with `(int)` casts; `%u` prints the
        // low 32 bits, so render the `u32` truncation directly.
        let s = format!(
            "bytes_write={} bytes_read={} bytes_retransmit={} bytes_invalid={} send_seq={} receive_seq={} retransmit_seq={} srtt={} rttvar={} rto={} ready_bytes={} upcoming_bytes={}",
            bytes_write,
            bytes_read,
            bytes_retransmit,
            bytes_invalid,
            send_seq as u32,
            receive_seq as u32,
            retransmit_seq as u32,
            format_f_fixed(srtt, 3, false),
            format_f_fixed(rttvar, 3, false),
            format_f_fixed(rto, 3, false),
            ready_bytes as u32,
            upcoming_bytes as u32,
        );
        if buf.is_empty() {
            return;
        }
        let bytes = s.as_bytes();
        let n = bytes.len().min(buf.len() - 1);
        buf[..n].copy_from_slice(&bytes[..n]);
        buf[n] = 0;
    }

    /// Extract old messages stored in the debug queues (port of
    /// `serialqueue_extract_old`). With `sentq` set, dumps the
    /// send-path rings via the logging callback (the C
    /// `sq_debug_dump`, invoked from the same path).
    pub fn extract_old(&self, sentq: bool, max: usize) -> Vec<PullQueueMessage> {
        let count = if sentq { DEBUG_QUEUE_SENT } else { DEBUG_QUEUE_RECEIVE };
        let mut current = {
            let mut v = VecDeque::new();
            for _ in 0..count {
                v.push_back(Box::new(QueueMessage::new()));
            }
            v
        };

        // Atomically replace existing debug list with a new zero'd list.
        if sentq {
            {
                let mut q = self.inner.queue.lock();
                std::mem::swap(&mut q.old_sent, &mut current);
            }
            // Error and shutdown dumps reach here via `serialhdl`
            // `dump_debug()`. Rendered after releasing `sq->lock`
            // because `errorf` enters the Python logging callback.
            sq_debug_dump(&self.inner, &self.pr);
        } else {
            let mut recv = self.inner.receiver.lock();
            std::mem::swap(&mut recv.old_receive, &mut current);
        }

        // Walk the debug list
        let mut out = Vec::new();
        while let Some(qm) = current.pop_front() {
            if qm.len != 0 && out.len() < max {
                out.push(PullQueueMessage {
                    msg: qm.msg,
                    len: qm.len,
                    sent_time: qm.sent_time,
                    receive_time: qm.receive_time,
                    notify_id: 0,
                });
            }
        }
        out
    }
}

impl Drop for SerialQueue {
    fn drop(&mut self) {
        self.exit();
        {
            let mut q = self.inner.queue.lock();
            q.sent_queue.clear();
            q.notify_queue.clear();
            q.old_sent.clear();
            for cq in q.ready_queues.iter() {
                cq.queues.lock().ready.clear();
            }
            q.ready_queues.clear();
        }
        {
            let mut recv = self.inner.receiver.lock();
            recv.queue.clear();
            recv.old_receive.clear();
        }
        {
            let mut tx = self.inner.transmit.lock();
            for cq in tx.upcoming_queues.iter() {
                cq.queues.lock().upcoming.clear();
            }
            tx.upcoming_queues.clear();
        }
        // The C closes both ends of the wake-up pipe after freeing the reactor.
        fd::close(self.inner.pipe_rfd);
        fd::close(self.inner.pipe_wfd);
    }
}

/// Port of `serialqueue_alloc_commandqueue`.
pub fn alloc_commandqueue() -> Arc<CommandQueue> {
    Arc::new(CommandQueue {
        queues: sync::Mutex::new(CommandQueueQueues {
            ready: VecDeque::new(),
            upcoming: VecDeque::new(),
        }),
    })
}

/// Port of `serialqueue_free_commandqueue`.
pub fn free_commandqueue(cq: Arc<CommandQueue>) {
    let queues = cq.queues.lock();
    if !queues.ready.is_empty() || !queues.upcoming.is_empty() {
        crate::errorf("Memory leak! Can't free non-empty commandqueue");
        // The C `return`s before `free(cq)` here (a leak); the port
        // forgets the allocation the same way.
        drop(queues);
        std::mem::forget(cq);
        return;
    }
    drop(queues);
    // `cq` is freed by the Arc drop here (the C `free(cq)`).
}

fn name_until_nul(name: &[u8]) -> &[u8] {
    let end = name.iter().position(|&b| b == 0).unwrap_or(name.len());
    &name[..end]
}

// ---- dump path ---------------------------------------------------------------

fn f6(v: f64) -> String {
    format_f_fixed(v, 6, false)
}

fn f3(v: f64) -> String {
    format_f_fixed(v, 3, false)
}

/// Render one bounded ring in chronological order (port of
/// `sq_debug_dump_ring`).
fn sq_debug_dump_ring(
    name: &str,
    label: &str,
    ring: &[SqDebugEvent],
    pos: u32,
    count: u32,
) {
    let size = ring.len() as u32;
    let mut i = 0_u32;
    while i < count {
        let d = &ring[((pos + size - count + i) % size) as usize];
        // sched/wake kind: 'n'=PR_NOW, 'v'=never (PR_NEVER, or a sleep
        // so far out that it will only end via a kick), 't'=absolute
        // time. late is how long after the scheduled wake the
        // decision ran.
        let late = d.eventtime - d.sched;
        let in_ = d.waketime - d.last_eventtime;
        let sk = if d.sched <= PR_NOW { 'n' } else if late < -SQDBG_FAR { 'v' } else { 't' };
        let wk = if d.waketime <= PR_NOW { 'n' } else if in_ > SQDBG_FAR { 'v' } else { 't' };
        let reason = if (d.reason as usize) < SQDBG_REASONS {
            SQDBG_REASON_NAMES[d.reason as usize]
        } else {
            "?"
        };
        crate::errorf(&format!(
            "[DEBUG-motion] serial={name} {label}[{i}] r={reason} rep={} t={} dt={} sched={sk} late={} wake={wk} in={} idle={} ack_clock={} want_clock={} seq={}/{} ready={} upcoming={} need_ack={} pending={} result={} flags=0x{:02x}",
            d.repeat as u32,
            f6(d.eventtime),
            f6(d.last_eventtime - d.eventtime),
            f6(if sk == 't' { late } else { 0. }),
            f6(if wk == 't' { in_ } else { 0. }),
            f6(d.idletime),
            d.ack_clock,
            d.want_clock,
            d.send_seq,
            d.receive_seq,
            d.ready_bytes,
            d.upcoming_bytes,
            d.need_ack_bytes,
            d.pending,
            d.result,
            d.flags,
        ));
        i += 1;
    }
}

/// Dump the bounded send-path rings (port of `sq_debug_dump`). The
/// snapshot is taken under the locks and rendered afterwards.
fn sq_debug_dump(inner: &Inner, pr: &PollReactor<Arc<Inner>>) {
    let name = String::from_utf8_lossy(name_until_nul(&inner.name)).into_owned();
    let serial_fd_type = inner.serial_fd_type;
    let now = time::monotonic();
    // Snapshot under sq->lock then transmit lock.
    struct Snap {
        send_seq: u64,
        receive_seq: u64,
        last_ack_seq: u64,
        retransmit_seq: u64,
        ignore_nak_seq: u64,
        ready_bytes: i32,
        upcoming_bytes: i32,
        need_ack_bytes: i32,
        last_ack_bytes: i32,
        receive_window: i32,
        bytes_write: u32,
        bytes_read: u32,
        bytes_retransmit: u32,
        bytes_invalid: u32,
        idle_time: f64,
        srtt: f64,
        rttvar: f64,
        rto: f64,
        last_write_fail_time: f64,
        last_receive_sent_time: f64,
        ce: ClockEstimate,
        need_kick_clock: u64,
        min_release_clock: u64,
    }
    let (snap, dbg, cmd_wake, rtm_wake, is_exit) = {
        let q = inner.queue.lock();
        let tx = inner.transmit.lock();
        let snap = Snap {
            send_seq: q.send_seq,
            receive_seq: q.receive_seq,
            last_ack_seq: q.last_ack_seq,
            retransmit_seq: q.retransmit_seq,
            ignore_nak_seq: q.ignore_nak_seq,
            ready_bytes: q.ready_bytes,
            upcoming_bytes: tx.upcoming_bytes,
            need_ack_bytes: q.need_ack_bytes,
            last_ack_bytes: q.last_ack_bytes,
            receive_window: q.receive_window,
            bytes_write: q.bytes_write,
            bytes_read: q.bytes_read,
            bytes_retransmit: q.bytes_retransmit,
            bytes_invalid: q.bytes_invalid,
            idle_time: q.idle_time,
            srtt: q.srtt,
            rttvar: q.rttvar,
            rto: q.rto,
            last_write_fail_time: q.last_write_fail_time,
            last_receive_sent_time: q.last_receive_sent_time,
            ce: q.ce,
            need_kick_clock: tx.need_kick_clock,
            min_release_clock: tx.min_release_clock,
        };
        let dbg = q.dbg.clone();
        (snap, dbg, pr.get_timer(SQPT_COMMAND), pr.get_timer(SQPT_RETRANSMIT), pr.is_exit())
    };

    // Each ring keeps only its most recent entries; older runs are
    // overwritten. The reason counters below are cumulative and
    // therefore survive overwrite.
    crate::errorf(&format!(
        "[DEBUG-motion] version=motion-v4 serial={name} dump now={} type={} exit={} events={} ring={}/64 notable={}/16 coalesced_runs=1 older_runs_overwritten=1",
        f6(now),
        serial_fd_type as char,
        if is_exit { 1 } else { 0 },
        dbg.total,
        dbg.count,
        dbg.notable_count,
    ));
    crate::errorf(&format!(
        "[DEBUG-motion] serial={name} state send_seq={} receive_seq={} last_ack_seq={} retransmit_seq={} ignore_nak_seq={} ready_bytes={} upcoming_bytes={} need_ack_bytes={} last_ack_bytes={} receive_window={} bytes_write={} bytes_read={} bytes_retransmit={} bytes_invalid={}",
        snap.send_seq,
        snap.receive_seq,
        snap.last_ack_seq,
        snap.retransmit_seq,
        snap.ignore_nak_seq,
        snap.ready_bytes,
        snap.upcoming_bytes,
        snap.need_ack_bytes,
        snap.last_ack_bytes,
        snap.receive_window,
        snap.bytes_write,
        snap.bytes_read,
        snap.bytes_retransmit,
        snap.bytes_invalid,
    ));
    // cmd_late > 0 means the command timer is overdue right now
    let cmd_late = {
        let raw = now - cmd_wake;
        if (-SQDBG_FAR..=SQDBG_FAR).contains(&raw) { raw } else { 0. }
    };
    let clock_time = if now > snap.idle_time { now } else { snap.idle_time };
    let clock_now = if snap.ce.est_freq != 0. {
        msgblock::clock_from_time(&snap.ce, clock_time)
    } else {
        0
    };
    crate::errorf(&format!(
        "[DEBUG-motion] serial={name} timing idle_time={} srtt={} rttvar={} rto={} last_write_fail={} last_recv_sent={} cmd_wake={} cmd_late={} rtm_wake={} est_freq={} conv_time={} conv_clock={} clock_now={} need_kick_clock={} min_release_clock={}",
        f6(snap.idle_time),
        f6(snap.srtt),
        f6(snap.rttvar),
        f6(snap.rto),
        f6(snap.last_write_fail_time),
        f6(snap.last_receive_sent_time),
        f6(cmd_wake),
        f6(cmd_late),
        f6(rtm_wake),
        f3(snap.ce.est_freq),
        f6(snap.ce.conv_time),
        snap.ce.conv_clock,
        clock_now,
        snap.need_kick_clock,
        snap.min_release_clock,
    ));
    crate::errorf(&format!(
        "[DEBUG-motion] serial={name} counts send={} nowork={} minclock={} reqwait={} ackwait={} window={} race={} pending={} noclock={} write={} retransmit={} exit={}",
        dbg.reason_count[SQDBG_SEND as usize],
        dbg.reason_count[SQDBG_NOWORK as usize],
        dbg.reason_count[SQDBG_MINCLOCK as usize],
        dbg.reason_count[SQDBG_REQWAIT as usize],
        dbg.reason_count[SQDBG_ACKWAIT as usize],
        dbg.reason_count[SQDBG_WINDOW as usize],
        dbg.reason_count[SQDBG_RACE as usize],
        dbg.reason_count[SQDBG_PENDING as usize],
        dbg.reason_count[SQDBG_NOCLOCK as usize],
        dbg.reason_count[SQDBG_WRITE as usize],
        dbg.reason_count[SQDBG_RETRANSMIT as usize],
        dbg.reason_count[SQDBG_EXIT as usize],
    ));
    sq_debug_dump_ring(&name, "notable", &dbg.notable, dbg.notable_pos, dbg.notable_count);
    sq_debug_dump_ring(&name, "ring", &dbg.ring, dbg.pos, dbg.count);
}
