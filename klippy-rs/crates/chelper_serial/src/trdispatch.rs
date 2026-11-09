//! Port of `klippy/chelper/trdispatch.c`: trigger sync ("trsync")
//! message dispatch across MCUs during homing.
//!
//! One [`Trdispatch`] groups the MCUs taking part in a homing move.
//! Each MCU gets a [`TrdispatchMcu`] (the C `struct trdispatch_mcu`),
//! registered as a fast reader on that MCU's
//! [`SerialQueue`](crate::serialqueue::SerialQueue) for `trsync_state`
//! responses. When an MCU reports a trigger (or a timeout), the
//! trigger fans out to every MCU; while all MCUs keep reporting
//! `can_trigger`, the timeout is extended on each MCU from the minimum
//! acknowledged time, exactly as the C does.
//!
//! Locking mirrors the C: one mutex guards the shared state (the C
//! `td->lock`, covering `is_active`, `can_trigger`, `dispatch_reason`
//! and every MCU's clocks and clock estimates). The fast-reader
//! callback and [`TrdispatchMcu::setup`] read the clock estimate under
//! that lock, and the sends happen under it too, matching the C lock
//! order (`td->lock` before the serial locks taken by
//! [`SerialQueue::send_one`](crate::serialqueue::SerialQueue::send_one)).

use std::ffi::c_void;
use std::sync::Arc;

use kos::sync;

use crate::msgblock::{
    clock_from_clock32, clock_from_time, clock_to_time, message_alloc_and_encode, ClockEstimate,
};
use crate::serialqueue::{CommandQueue, SerialQueue};
use crate::PR_NEVER;

/// Per-MCU clocks guarded by the dispatch lock (the C fields after
/// `// Remaining fields protected by trdispatch lock`).
struct McuState {
    last_status_clock: u64,
    expire_clock: u64,
    expire_ticks: u64,
    min_extend_ticks: u64,
    ce: ClockEstimate,
}

/// Fast-reader callback context (the C `container_of(fr, struct
/// trdispatch_mcu, fr)` plus `tdm->td`). Lives in the group entry so
/// the callback can find the dispatch group, the MCU slot and the
/// expected oid without taking the lock first, exactly like the C
/// checks `fields[1] != tdm->trsync_oid` before locking. The group
/// pointer is type-erased so the entry stays `Send` (every mutation
/// goes through the dispatch lock).
struct McuHeader {
    td: *const c_void,
    index: usize,
    trsync_oid: u32,
}

/// One group entry (the C `struct trdispatch_mcu`).
struct McuEntry {
    sq: Arc<SerialQueue>,
    cq: Arc<CommandQueue>,
    trsync_oid: u32,
    set_timeout_msgtag: u32,
    trigger_msgtag: u32,
    prefix: Vec<u8>,
    state: McuState,
    header: Box<McuHeader>,
}

/// Shared state guarded by the dispatch lock (the C `struct
/// trdispatch` fields under `td->lock`). Slots stay `None` once freed
/// so live [`TrdispatchMcu`] indices never shift.
struct Shared {
    is_active: bool,
    can_trigger: bool,
    dispatch_reason: u32,
    mcus: Vec<Option<McuEntry>>,
}

/// Port of `struct trdispatch`. Always used behind an [`Arc`] (see
/// [`Trdispatch::mcu_alloc`]); [`TrdispatchMcu`] keeps one, so the
/// group outlives its registrations.
pub struct Trdispatch {
    shared: sync::Mutex<Shared>,
}

// SAFETY: every field behind the dispatch lock is plain data, except
// the entry headers' back pointers (`*const c_void`, an erased
// `*const Trdispatch`) and the queue `Arc`s (both `Send + Sync`). The
// back pointer only names the group that owns the entry; the group
// outlives its registrations (each handle holds an `Arc`), and header
// removal unregisters first while `rm_fastreader` waits for in-flight
// callbacks, so the pointer is never dereferenced after free.
unsafe impl Send for Trdispatch {}
unsafe impl Sync for Trdispatch {}

impl Trdispatch {
    /// Port of `trdispatch_alloc`.
    pub fn new() -> Self {
        Trdispatch {
            shared: sync::Mutex::new(Shared {
                is_active: false,
                can_trigger: false,
                dispatch_reason: 0,
                mcus: Vec::new(),
            }),
        }
    }

    /// Port of `trdispatch_mcu_alloc`: register an MCU with the group
    /// and precompute the `trsync_state` fast-reader prefix (the C
    /// `message_alloc_and_encode` of `{state_msgtag, trsync_oid}`).
    ///
    /// Takes `&Arc<Self>` so the callback header can point at a stable
    /// group address.
    pub fn mcu_alloc(
        self: &Arc<Self>,
        sq: Arc<SerialQueue>,
        cq: Arc<CommandQueue>,
        trsync_oid: u32,
        set_timeout_msgtag: u32,
        trigger_msgtag: u32,
        state_msgtag: u32,
    ) -> TrdispatchMcu {
        let dummy = message_alloc_and_encode(&[state_msgtag, trsync_oid]);
        let prefix = dummy.msg[..dummy.len as usize].to_vec();
        let mut shared = self.shared.lock();
        let index = match shared.mcus.iter().position(|e| e.is_none()) {
            Some(i) => i,
            None => {
                shared.mcus.push(None);
                shared.mcus.len() - 1
            }
        };
        shared.mcus[index] = Some(McuEntry {
            sq,
            cq,
            trsync_oid,
            set_timeout_msgtag,
            trigger_msgtag,
            prefix,
            state: McuState {
                last_status_clock: 0,
                expire_clock: 0,
                expire_ticks: 0,
                min_extend_ticks: 0,
                ce: ClockEstimate::default(),
            },
            header: Box::new(McuHeader {
                td: Arc::as_ptr(self) as *const c_void,
                index,
                trsync_oid,
            }),
        });
        TrdispatchMcu {
            td: Arc::clone(self),
            index: Some(index),
        }
    }

    /// Port of `trdispatch_start`: arm the dispatch and register the
    /// `trsync_state` fast reader on every MCU. No-op when already
    /// active or when no MCU is registered.
    pub fn start(&self, dispatch_reason: u32) {
        let regs: Vec<(Arc<SerialQueue>, Vec<u8>, *mut c_void)> = {
            let mut shared = self.shared.lock();
            if shared.is_active || !shared.mcus.iter().any(|e| e.is_some()) {
                return;
            }
            shared.dispatch_reason = dispatch_reason;
            shared.is_active = true;
            shared.can_trigger = true;
            shared
                .mcus
                .iter()
                .flatten()
                .map(|m| {
                    (
                        Arc::clone(&m.sq),
                        m.prefix.clone(),
                        (&*m.header) as *const McuHeader as *mut c_void,
                    )
                })
                .collect()
        };
        // The C registers after releasing `td->lock`.
        for (sq, prefix, ctx) in &regs {
            // SAFETY: `ctx` is the entry header, alive until the entry
            // is removed, which first unregisters (see `stop` and
            // `TrdispatchMcu::remove`); the callback and prefix
            // outlive the registration.
            unsafe { sq.add_fastreader(handle_trsync_state, prefix, *ctx) };
        }
    }

    /// Port of `trdispatch_stop`: disarm and unregister every fast
    /// reader. No-op when not active.
    pub fn stop(&self) {
        let regs: Vec<(Arc<SerialQueue>, *mut c_void)> = {
            let mut shared = self.shared.lock();
            if !shared.is_active {
                return;
            }
            shared.is_active = false;
            shared
                .mcus
                .iter()
                .flatten()
                .map(|m| {
                    (
                        Arc::clone(&m.sq),
                        (&*m.header) as *const McuHeader as *mut c_void,
                    )
                })
                .collect()
        };
        for (sq, ctx) in &regs {
            // SAFETY: matched with the `start` registration above;
            // `rm_fastreader` waits for in-flight callbacks.
            unsafe { sq.rm_fastreader(handle_trsync_state, *ctx) };
        }
    }
}

impl Default for Trdispatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Port of `struct trdispatch_mcu`: the caller's handle to one MCU
/// registration. Release with [`TrdispatchMcu::free`]; dropping
/// without it removes the registration the same way (unregistering
/// first when active), so a leaked handle can never leave a dangling
/// fast-reader context behind.
pub struct TrdispatchMcu {
    td: Arc<Trdispatch>,
    index: Option<usize>,
}

// SAFETY: plain index plus an `Arc` to a `Send + Sync` group; all
// mutation goes through the dispatch lock.
unsafe impl Send for TrdispatchMcu {}

impl TrdispatchMcu {
    /// Port of `trdispatch_mcu_setup`: tune the clocks for a trigger
    /// test and snapshot the MCU's clock estimate.
    pub fn setup(
        &self,
        last_status_clock: u64,
        expire_clock: u64,
        expire_ticks: u64,
        min_extend_ticks: u64,
    ) {
        let index = self.index.expect("setup on a freed TrdispatchMcu");
        let mut shared = self.td.shared.lock();
        if let Some(entry) = shared.mcus.get_mut(index).and_then(|e| e.as_mut()) {
            // The C snapshots the estimate under `td->lock`.
            let ce = entry.sq.get_clock_est();
            entry.state.last_status_clock = last_status_clock;
            entry.state.expire_clock = expire_clock;
            entry.state.expire_ticks = expire_ticks;
            entry.state.min_extend_ticks = min_extend_ticks;
            entry.state.ce = ce;
        }
    }

    /// Release the MCU registration. The dispatch should be stopped
    /// first (as the C caller does); when still active this
    /// unregisters the reader before releasing the entry.
    pub fn free(mut self) {
        self.remove();
        std::mem::forget(self);
    }

    fn remove(&mut self) {
        if let Some(index) = self.index.take() {
            let (entry, active) = {
                let mut shared = self.td.shared.lock();
                let entry = shared.mcus.get_mut(index).and_then(|e| e.take());
                let active = shared.is_active;
                (entry, active)
            };
            if let Some(entry) = entry {
                if active {
                    // SAFETY: the entry was registered by `start` and
                    // is now out of the group, so no new callback can
                    // find it; `rm_fastreader` waits for in-flight
                    // ones before the header is freed below.
                    unsafe {
                        entry.sq.rm_fastreader(
                            handle_trsync_state,
                            (&*entry.header) as *const McuHeader as *mut c_void,
                        );
                    }
                }
            }
        }
    }
}

impl Drop for TrdispatchMcu {
    fn drop(&mut self) {
        self.remove();
    }
}

/// Port of `handle_trsync_state`: the serialqueue fast-reader callback
/// for `trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u`.
extern "C" fn handle_trsync_state(ctx: *mut c_void, eventtime: f64, data: *mut u8, len: i32) {
    // SAFETY: `ctx` is an entry header, alive while registered (removal
    // unregisters first and `rm_fastreader` waits for in-flight
    // callbacks).
    let (td, index, want_oid) = unsafe {
        let header = ctx as *const McuHeader;
        ((*header).td as *const Trdispatch, (*header).index, (*header).trsync_oid)
    };
    // SAFETY: the group outlives its registrations (each handle holds
    // an `Arc`), and the message buffer is the serialqueue input of
    // length `len`.
    let msg: &[u8] = unsafe {
        std::slice::from_raw_parts(
            data as *const u8,
            usize::try_from(len).unwrap_or(0),
        )
    };

    // Parse: trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u
    let mut fields = [0u32; 5];
    if crate::msgblock::msgblock_decode(&mut fields, msg) != 0 || fields[1] != want_oid {
        return;
    }
    let can_trigger = fields[2];
    let clock = fields[4];

    // SAFETY: as above, the group is alive while the reader runs.
    let td: &Trdispatch = unsafe { &*td };
    let mut shared = td.shared.lock();
    if !shared.can_trigger {
        return;
    }
    // The freed-while-registered case cannot happen (removal
    // unregisters first and `rm_fastreader` waits for in-flight
    // callbacks), but validate the oid anyway: the slot may have been
    // removed and reused between registration and dispatch.
    if shared
        .mcus
        .get(index)
        .and_then(|e| e.as_ref())
        .is_none_or(|e| e.trsync_oid != want_oid)
    {
        return;
    }

    if can_trigger == 0 {
        // MCU reports trigger or timeout: propagate to all MCUs.
        shared.can_trigger = false;
        let reason = shared.dispatch_reason;
        // The C holds `td->lock` across the sends; `send_one` takes the
        // serial locks in the same order there as here.
        for m in shared.mcus.iter().flatten() {
            let qm =
                message_alloc_and_encode(&[m.trigger_msgtag, m.trsync_oid, reason]);
            m.sq.send_one(&m.cq, qm);
        }
        return;
    }

    // MCU is still working: update its last status clock. The C reads
    // the clock estimate under `td->lock`.
    let ce = shared.mcus[index].as_ref().map(|m| m.sq.get_clock_est());
    if let (Some(ce), Some(entry)) = (ce, shared.mcus[index].as_mut()) {
        entry.state.ce = ce;
        entry.state.last_status_clock =
            clock_from_clock32(clock_from_time(&ce, eventtime), clock);
    }

    // Determine minimum acknowledged time among all MCUs. The C keeps
    // the minimum entry pointer (`min_tdm`); the port keeps its slot
    // index (freed slots stay `None` and are skipped).
    let mut min_time = PR_NEVER;
    let mut next_min_time = PR_NEVER;
    let mut min_slot: Option<usize> = None;
    for (slot, m) in shared.mcus.iter().enumerate().filter_map(|(i, e)| e.as_ref().map(|m| (i, m))) {
        let status_time = clock_to_time(&m.state.ce, m.state.last_status_clock);
        if status_time < next_min_time {
            next_min_time = status_time;
            if status_time < min_time {
                next_min_time = min_time;
                min_time = status_time;
                min_slot = Some(slot);
            }
        }
    }
    if next_min_time == PR_NEVER {
        next_min_time = min_time;
    }

    // Send trsync_set_timeout messages to other MCUs (if needed).
    for (slot, m) in shared.mcus.iter_mut().enumerate().filter_map(|(i, e)| e.as_mut().map(|m| (i, m))) {
        let status_time = if Some(slot) == min_slot {
            next_min_time
        } else {
            min_time
        };
        let expire =
            clock_from_time(&m.state.ce, status_time).wrapping_add(m.state.expire_ticks);
        // The C `(int64_t)(expire - m->expire_clock) >=
        // m->min_extend_ticks` compares in `uint64_t` (usual arithmetic
        // conversions), so the round trip through `int64_t` is the
        // identity on the bits.
        if expire.wrapping_sub(m.state.expire_clock) >= m.state.min_extend_ticks {
            m.state.expire_clock = expire;
            let mut qm = message_alloc_and_encode(&[
                m.set_timeout_msgtag,
                m.trsync_oid,
                expire as u32,
            ]);
            qm.req_clock = expire;
            m.sq.send_one(&m.cq, qm);
        }
    }
}
