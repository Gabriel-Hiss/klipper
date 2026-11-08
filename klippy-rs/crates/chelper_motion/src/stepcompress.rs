//! Port of the `stepcompress` unit (`klippy/chelper/stepcompress.c`).
//!
//! The compressor turns a series of scheduled step times into a short list
//! of `queue_step` / `set_next_step_dir` MCU messages (VLQ-encoded, see
//! [`msgblock`]).  Every integer operation that can overflow in the C is
//! done with `wrapping_*` at the reference width, and every float-to-int
//! conversion goes through [`kos::cconv`] so the results match the
//! reference binary bit for bit (see `parity/maps/map-motion.md` section 5,
//! F5-F10).
//!
//! Error handling mirrors the C: failing paths return [`ERROR_RET`] and
//! leave the not-yet-consumed queue untouched.  The fork's `[DEBUG-motion]`
//! rings and `errorf` logging are not reproduced; they do not affect the
//! emitted messages, the history or the return codes.

mod msgblock;

use std::collections::VecDeque;

use kos::cconv;

/// Return code for the error paths (`ERROR_RET` in `stepcompress.h`).
pub const ERROR_RET: i32 = -989898989;

/// Maximum clock delta between queued messages (`CLOCK_DIFF_MAX`).
pub const CLOCK_DIFF_MAX: u64 = 3 << 28;

/// Step+dir+step filter window in seconds (`SDS_FILTER_TIME`).
const SDS_FILTER_TIME: f64 = 0.000750;
/// Quadratic deviation coefficient (`QUADRATIC_DEV`).
const QUADRATIC_DEV: u32 = 11;
/// Initial queue allocation (`QUEUE_START_SIZE`).
const QUEUE_START_SIZE: usize = 1024;

/// One history entry, matching `struct pull_history_steps` in
/// `stepcompress.h` (and the cffi layout read by `stepper.py`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PullHistorySteps {
    pub first_clock: u64,
    pub last_clock: u64,
    pub start_position: i64,
    pub step_count: i32,
    pub interval: i32,
    pub add: i32,
}
/// One queued MCU message, matching the fields of `struct queue_message`
/// (`msgblock.h`) that the step compressor fills: the encoded `msg` bytes,
/// `len`, `req_clock` and `min_clock`.
#[derive(Clone, Debug, PartialEq)]
pub struct QueuedMessage {
    /// Encoded payload bytes (`msg[0..len]`).
    pub bytes: Vec<u8>,
    /// Clock requested for the message (`req_clock`).
    pub req_clock: u64,
    /// Earliest clock the message may be sent (`min_clock`).
    pub min_clock: u64,
}

/// One `queue_step` command before serialization (`struct step_move`).
#[derive(Clone, Copy, Debug)]
struct StepMove {
    interval: u32,
    count: u16,
    add: i16,
}

/// Internal history entry (`struct history_steps`).
#[derive(Clone, Copy, Debug)]
struct HistorySteps {
    first_clock: u64,
    last_clock: u64,
    start_position: i64,
    step_count: i32,
    interval: i32,
    add: i32,
}

/// Min/max acceptable point times (`struct points`).
#[derive(Clone, Copy, Debug)]
struct Points {
    minp: i32,
    maxp: i32,
}

/// A step compressor shared with its syncemitter background thread.
pub type SharedStepcompress = std::sync::Arc<kos::sync::Mutex<Stepcompress>>;

/// The step compressor state (`struct stepcompress`).
pub struct Stepcompress {
    // Staged step clocks: `queue[queue_pos..queue_next]` holds
    // `clock32` values (low 32 bits only, like the C); `queue.len()` is
    // the allocated capacity (`queue_end - queue`).
    queue: Vec<u32>,
    queue_pos: usize,
    queue_next: usize,
    // Internal tracking
    max_error: u32,
    mcu_time_offset: f64,
    mcu_freq: f64,
    last_step_print_time: f64,
    // Message generation
    last_step_clock: u64,
    msgs: Vec<QueuedMessage>,
    oid: u32,
    queue_step_msgtag: i32,
    set_next_step_dir_msgtag: i32,
    sdir: i32,
    invert_sdir: i32,
    // Step+dir+step filter
    next_step_clock: u64,
    next_step_dir: i32,
    // History tracking (newest first, like `list_add_head`)
    last_position: i64,
    history: VecDeque<HistorySteps>,
}

impl Default for Stepcompress {
    fn default() -> Self {
        Self::new()
    }
}

impl Stepcompress {
    /// Allocate a new compressor, zeroed like the C `malloc` + `memset`
    /// except `sdir`, which the C initializes to -1.
    pub fn new() -> Self {
        Stepcompress {
            queue: Vec::new(),
            queue_pos: 0,
            queue_next: 0,
            max_error: 0,
            mcu_time_offset: 0.0,
            mcu_freq: 0.0,
            last_step_print_time: 0.0,
            last_step_clock: 0,
            msgs: Vec::new(),
            oid: 0,
            queue_step_msgtag: 0,
            set_next_step_dir_msgtag: 0,
            sdir: -1,
            invert_sdir: 0,
            next_step_clock: 0,
            next_step_dir: 0,
            last_position: 0,
            history: VecDeque::new(),
        }
    }

    /// Fill message id information (`stepcompress_fill`).
    pub fn fill(
        &mut self,
        oid: u32,
        max_error: u32,
        queue_step_msgtag: i32,
        set_next_step_dir_msgtag: i32,
    ) {
        self.oid = oid;
        self.max_error = max_error;
        self.queue_step_msgtag = queue_step_msgtag;
        self.set_next_step_dir_msgtag = set_next_step_dir_msgtag;
    }

    /// Set the inverted stepper direction flag
    /// (`stepcompress_set_invert_sdir`).
    pub fn set_invert_sdir(&mut self, invert_sdir: u32) {
        let invert_sdir = i32::from(invert_sdir != 0);
        if invert_sdir != self.invert_sdir {
            self.invert_sdir = invert_sdir;
            if self.sdir >= 0 {
                self.sdir ^= 1;
            }
        }
    }

    /// Current MCU clock estimate (`stepcompress_get_oid`).
    pub fn get_oid(&self) -> u32 {
        self.oid
    }

    /// Pending step direction (`stepcompress_get_step_dir`).
    pub fn get_step_dir(&self) -> i32 {
        self.next_step_dir
    }

    /// Drain the queued messages (the port's `msg_queue` list, in order).
    pub fn drain_messages(&mut self) -> Vec<QueuedMessage> {
        std::mem::take(&mut self.msgs)
    }

    /// Set the conversion rate of `print_time` to mcu clock
    /// (`stepcompress_set_time`).
    pub fn set_time(&mut self, time_offset: f64, mcu_freq: f64) {
        self.mcu_time_offset = time_offset;
        self.mcu_freq = mcu_freq;
        self.calc_last_step_print_time();
    }

    /// Expire the history older than the given clock
    /// (`stepcompress_history_expire`).
    pub fn history_expire(&mut self, end_clock: u64) {
        while let Some(hs) = self.history.back() {
            if hs.last_clock > end_clock {
                break;
            }
            self.history.pop_back();
        }
    }

    /// Add next step time (`stepcompress_append`).
    pub fn append(&mut self, sdir: i32, print_time: f64, step_time: f64) -> i32 {
        // Calculate step clock
        let offset = print_time - self.last_step_print_time;
        let rel_sc = (step_time + offset) * self.mcu_freq;
        let step_clock = self
            .last_step_clock
            .wrapping_add(cconv::f64_to_u64(rel_sc));
        // Flush previous pending step (if any)
        if self.next_step_clock != 0 {
            if sdir != self.next_step_dir {
                let diff = step_clock
                    .wrapping_sub(self.next_step_clock) as i64 as f64;
                if diff < SDS_FILTER_TIME * self.mcu_freq {
                    // Rollback last step to avoid rapid step+dir+step
                    self.next_step_clock = 0;
                    self.next_step_dir = sdir;
                    return 0;
                }
            }
            let ret = self.queue_append();
            if ret != 0 {
                return ret;
            }
        }
        // Store this step as the next pending step
        self.next_step_clock = step_clock;
        self.next_step_dir = sdir;
        0
    }

    /// Commit next pending step (`stepcompress_commit`).
    pub fn commit(&mut self) -> i32 {
        if self.next_step_clock != 0 {
            return self.queue_append();
        }
        0
    }

    /// Flush pending steps (`stepcompress_flush`).
    pub fn flush(&mut self, move_clock: u64) -> i32 {
        if self.next_step_clock != 0 && move_clock >= self.next_step_clock {
            let ret = self.queue_append();
            if ret != 0 {
                return ret;
            }
        }
        self.queue_flush(move_clock)
    }

    /// Reset the internal state (`stepcompress_reset`).
    pub fn reset(&mut self, last_step_clock: u64) -> i32 {
        let ret = self.flush(u64::MAX);
        if ret != 0 {
            return ret;
        }
        self.last_step_clock = last_step_clock;
        self.sdir = -1;
        self.calc_last_step_print_time();
        0
    }

    /// Set `last_position` (`stepcompress_set_last_position`).
    pub fn set_last_position(&mut self, clock: u64, last_position: i64) -> i32 {
        let ret = self.flush(u64::MAX);
        if ret != 0 {
            return ret;
        }
        self.last_position = last_position;

        // Add a marker to the history list
        self.history.push_front(HistorySteps {
            first_clock: clock,
            last_clock: clock,
            start_position: last_position,
            step_count: 0,
            interval: 0,
            add: 0,
        });
        0
    }

    /// Search history of moves to find a past position at a given clock
    /// (`stepcompress_find_past_position`).
    pub fn find_past_position(&self, clock: u64) -> i64 {
        let mut last_position = self.last_position;
        for hs in &self.history {
            if clock < hs.first_clock {
                last_position = hs.start_position;
                continue;
            }
            if clock >= hs.last_clock {
                return hs.start_position.wrapping_add(hs.step_count as i64);
            }
            let interval = hs.interval;
            let add = hs.add;
            let ticks = (clock.wrapping_sub(hs.first_clock) as i32)
                .wrapping_add(interval);
            let offset: i32 = if add == 0 {
                // `interval` is zero only for entries the C code can never
                // reach here (markers have first == last; count == 1 moves
                // have first == last; a failing check_line keeps other
                // count == 1 moves out of the history).  The C would
                // divide by zero, which is undefined; use 0 defensively.
                if interval == 0 {
                    0
                } else {
                    ticks.wrapping_div(interval)
                }
            } else {
                // Solve for "count" using quadratic formula
                let a = 0.5 * add as f64;
                let b = interval as f64 - 0.5 * add as f64;
                let c = -(ticks as f64);
                cconv::f64_to_i32(((b * b - 4.0 * a * c).sqrt() - b) / (2.0 * a))
            };
            if hs.step_count < 0 {
                return hs.start_position.wrapping_sub(offset as i64);
            }
            return hs.start_position.wrapping_add(offset as i64);
        }
        last_position
    }

    /// Return history of `queue_step` commands, newest first
    /// (`stepcompress_extract_old`).
    pub fn extract_old(
        &self,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> Vec<PullHistorySteps> {
        let mut res = Vec::new();
        for hs in &self.history {
            if start_clock >= hs.last_clock || res.len() as i32 >= max {
                break;
            }
            if end_clock <= hs.first_clock {
                continue;
            }
            res.push(PullHistorySteps {
                first_clock: hs.first_clock,
                last_clock: hs.last_clock,
                start_position: hs.start_position,
                step_count: hs.step_count,
                interval: hs.interval,
                add: hs.add,
            });
        }
        res
    }

    // -- internals ---------------------------------------------------

    /// Determine the "print time" of the last_step_clock.
    fn calc_last_step_print_time(&mut self) {
        let lsc = self.last_step_clock as f64;
        self.last_step_print_time = self.mcu_time_offset + (lsc - 0.5) / self.mcu_freq;
    }

    /// Given a requested step time, return the minimum and maximum
    /// acceptable times (`minmax_point`); `i` is relative to the queue pos.
    fn minmax_point(&self, i: usize) -> Points {
        let lsc = self.last_step_clock as u32;
        let point = self.queue[self.queue_pos + i].wrapping_sub(lsc);
        let prevpoint = if i > 0 {
            self.queue[self.queue_pos + i - 1].wrapping_sub(lsc)
        } else {
            0
        };
        let mut max_error = point.wrapping_sub(prevpoint) / 2;
        if max_error > self.max_error {
            max_error = self.max_error;
        }
        Points {
            minp: point.wrapping_sub(max_error) as i32,
            maxp: point as i32,
        }
    }

    /// Find a `StepMove` that covers a series of step times
    /// (`compress_bisect_add`).
    fn compress_bisect_add(&self) -> StepMove {
        let qlast = self.queue_next.min(self.queue_pos + 65535);
        let point = self.minmax_point(0);
        let mut outer_mininterval = point.minp;
        let mut outer_maxinterval = point.maxp;
        let mut add: i32 = 0;
        let mut minadd: i32 = -0x8000;
        let mut maxadd: i32 = 0x7fff;
        let mut bestinterval: i32 = 0;
        let mut bestcount: i32 = 1;
        let mut bestadd: i32 = 1;
        let mut bestreach: i32 = i32::MIN;
        let mut zerointerval: i32 = 0;
        let mut zerocount: i32 = 0;
        let mut nextpoint: Points;
        let mut nextmininterval: i32;
        let mut nextmaxinterval: i32;
        let mut interval: i32;
        let mut nextcount: i32;

        loop {
            // Find longest valid sequence with the given 'add'
            nextmininterval = outer_mininterval;
            nextmaxinterval = outer_maxinterval;
            interval = nextmaxinterval;
            nextcount = 1;
            loop {
                nextcount = nextcount.wrapping_add(1);
                if self.queue_pos + (nextcount as usize) > qlast {
                    let count = nextcount.wrapping_sub(1);
                    return StepMove {
                        interval: interval as u32,
                        count: count as u16,
                        add: add as i16,
                    };
                }
                nextpoint = self.minmax_point(nextcount as usize - 1);
                let nextaddfactor = nextcount.wrapping_mul(nextcount - 1) / 2;
                let c = add.wrapping_mul(nextaddfactor);
                if nextmininterval.wrapping_mul(nextcount)
                    < nextpoint.minp.wrapping_sub(c)
                {
                    nextmininterval =
                        idiv_up(nextpoint.minp.wrapping_sub(c), nextcount);
                }
                if nextmaxinterval.wrapping_mul(nextcount)
                    > nextpoint.maxp.wrapping_sub(c)
                {
                    nextmaxinterval =
                        idiv_down(nextpoint.maxp.wrapping_sub(c), nextcount);
                }
                if nextmininterval > nextmaxinterval {
                    break;
                }
                interval = nextmaxinterval;
            }

            // Check if this is the best sequence found so far
            let count = nextcount.wrapping_sub(1);
            let addfactor = count.wrapping_mul(count - 1) / 2;
            let reach = add
                .wrapping_mul(addfactor)
                .wrapping_add(interval.wrapping_mul(count));
            if reach > bestreach || (reach == bestreach && interval > bestinterval) {
                bestinterval = interval;
                bestcount = count;
                bestadd = add;
                bestreach = reach;
                if add == 0 {
                    zerointerval = interval;
                    zerocount = count;
                }
                if count > 0x200 {
                    // No 'add' will improve sequence; avoid integer overflow
                    break;
                }
            }

            // Check if a greater or lesser add could extend the sequence
            let nextaddfactor = nextcount.wrapping_mul(nextcount - 1) / 2;
            let nextreach = add
                .wrapping_mul(nextaddfactor)
                .wrapping_add(interval.wrapping_mul(nextcount));
            if nextreach < nextpoint.minp {
                minadd = add.wrapping_add(1);
                outer_maxinterval = nextmaxinterval;
            } else {
                maxadd = add.wrapping_sub(1);
                outer_mininterval = nextmininterval;
            }

            // The maximum valid deviation between two quadratic sequences
            // can be calculated and used to further limit the add range.
            if count > 1 {
                let errdelta = (self.max_error.wrapping_mul(QUADRATIC_DEV)
                    / (count.wrapping_mul(count) as u32)) as i32;
                if minadd < add.wrapping_sub(errdelta) {
                    minadd = add.wrapping_sub(errdelta);
                }
                if maxadd > add.wrapping_add(errdelta) {
                    maxadd = add.wrapping_add(errdelta);
                }
            }

            // See if next point would further limit the add range
            let mut c = outer_maxinterval.wrapping_mul(nextcount);
            if minadd.wrapping_mul(nextaddfactor) < nextpoint.minp.wrapping_sub(c) {
                minadd = idiv_up(nextpoint.minp.wrapping_sub(c), nextaddfactor);
            }
            c = outer_mininterval.wrapping_mul(nextcount);
            if maxadd.wrapping_mul(nextaddfactor) > nextpoint.maxp.wrapping_sub(c) {
                maxadd = idiv_down(nextpoint.maxp.wrapping_sub(c), nextaddfactor);
            }

            // Bisect valid add range and try again with new 'add'
            if minadd > maxadd {
                break;
            }
            add = maxadd.wrapping_sub(maxadd.wrapping_sub(minadd) / 4);
        }
        if zerocount + zerocount / 16 >= bestcount {
            // Prefer add=0 if it's similar to the best found sequence
            return StepMove {
                interval: zerointerval as u32,
                count: zerocount as u16,
                add: 0,
            };
        }
        StepMove {
            interval: bestinterval as u32,
            count: bestcount as u16,
            add: bestadd as i16,
        }
    }

    /// Verify that a given `StepMove` matches the actual step times
    /// (`check_line`).
    fn check_line(&self, mov: &StepMove) -> i32 {
        if mov.count == 0
            || (mov.interval == 0 && mov.add == 0 && mov.count > 1)
            || mov.interval >= 0x80000000
        {
            return ERROR_RET;
        }
        let mut p: u32 = 0;
        let mut interval = mov.interval;
        let mut i: u16 = 0;
        while i < mov.count {
            let point = self.minmax_point(i as usize);
            p = p.wrapping_add(interval);
            // In C, `p` (uint32_t) is compared against the int32_t bounds
            // after the usual unsigned conversion.
            if p < point.minp as u32 || p > point.maxp as u32 {
                return ERROR_RET;
            }
            if interval >= 0x80000000 {
                return ERROR_RET;
            }
            interval = interval.wrapping_add(mov.add as u32);
            i = i.wrapping_add(1);
        }
        0
    }

    /// Helper to create a `queue_step` command from a `StepMove`
    /// (`add_move`).
    fn add_move(&mut self, first_clock: u64, mov: &StepMove) {
        let addfactor = (mov.count as i32).wrapping_mul(mov.count as i32 - 1) / 2;
        let ticks = ((mov.add as i32).wrapping_mul(addfactor) as u32)
            .wrapping_add(mov.interval.wrapping_mul(mov.count as u32 - 1));
        let last_clock = first_clock.wrapping_add(ticks as u64);

        // Create and queue a queue_step command
        let msg = [
            self.queue_step_msgtag as u32,
            self.oid,
            mov.interval,
            mov.count as u32,
            mov.add as u32,
        ];
        let mut qm = QueuedMessage {
            bytes: msgblock::encode_message(&msg).unwrap_or_default(),
            req_clock: self.last_step_clock,
            min_clock: self.last_step_clock,
        };
        if mov.count == 1
            && first_clock
                >= self
                    .last_step_clock
                    .wrapping_add(CLOCK_DIFF_MAX)
        {
            qm.req_clock = first_clock;
        }
        self.msgs.push(qm);
        self.last_step_clock = last_clock;

        // Create and store move in history tracking
        let step_count = if self.sdir != 0 {
            mov.count as i32
        } else {
            -(mov.count as i32)
        };
        self.history.push_front(HistorySteps {
            first_clock,
            last_clock,
            start_position: self.last_position,
            step_count,
            interval: mov.interval as i32,
            add: mov.add as i32,
        });
        self.last_position = self.last_position.wrapping_add(step_count as i64);
    }

    /// Convert previously scheduled steps into commands for the mcu
    /// (`queue_flush`).
    fn queue_flush(&mut self, move_clock: u64) -> i32 {
        if self.queue_pos >= self.queue_next {
            return 0;
        }
        while self.last_step_clock < move_clock {
            let mov = self.compress_bisect_add();
            // The target is useful when a partial flush detects an older bad step.
            let ret = self.check_line(&mov);
            if ret != 0 {
                return ret;
            }
            let first_clock = self
                .last_step_clock
                .wrapping_add(mov.interval as u64);
            self.add_move(first_clock, &mov);
            if self.queue_pos + mov.count as usize >= self.queue_next {
                self.queue_pos = 0;
                self.queue_next = 0;
                break;
            }
            self.queue_pos += mov.count as usize;
        }
        self.calc_last_step_print_time();
        0
    }

    /// Generate a `queue_step` for a step far in the future from the last
    /// step (`stepcompress_flush_far`).
    fn flush_far(&mut self, abs_step_clock: u64) -> i32 {
        let mov = StepMove {
            interval: (abs_step_clock.wrapping_sub(self.last_step_clock)) as u32,
            count: 1,
            add: 0,
        };
        self.add_move(abs_step_clock, &mov);
        self.calc_last_step_print_time();
        0
    }

    /// Send the `set_next_step_dir` command (`set_next_step_dir`).
    fn set_next_step_dir(&mut self, sdir: i32) -> i32 {
        if self.sdir == sdir {
            return 0;
        }
        let ret = self.queue_flush(u64::MAX);
        if ret != 0 {
            return ret;
        }
        self.sdir = sdir;
        let msg = [
            self.set_next_step_dir_msgtag as u32,
            self.oid,
            (sdir ^ self.invert_sdir) as u32,
        ];
        self.msgs.push(QueuedMessage {
            bytes: msgblock::encode_message(&msg).unwrap_or_default(),
            req_clock: self.last_step_clock,
            min_clock: 0,
        });
        0
    }

    /// Slow path for `queue_append` - handle next step far in future
    /// (`queue_append_far`).
    fn queue_append_far(&mut self) -> i32 {
        let step_clock = self.next_step_clock;
        self.next_step_clock = 0;
        let ret = self.queue_flush(
            step_clock
                .wrapping_sub(CLOCK_DIFF_MAX)
                .wrapping_add(1),
        );
        if ret != 0 {
            return ret;
        }
        if step_clock
            >= self
                .last_step_clock
                .wrapping_add(CLOCK_DIFF_MAX)
        {
            return self.flush_far(step_clock);
        }
        self.store_queued(step_clock);
        0
    }

    /// Slow path for `queue_append` - expand the internal queue storage
    /// (`queue_append_extend`).
    fn queue_append_extend(&mut self) -> i32 {
        if self.queue_next - self.queue_pos > 65535 + 2000 {
            // No point in keeping more than 64K steps in memory
            let flush = self.queue[self.queue_next - 65535]
                .wrapping_sub(self.last_step_clock as u32) as u64;
            let ret = self.queue_flush(self.last_step_clock.wrapping_add(flush));
            if ret != 0 {
                return ret;
            }
        }

        if self.queue_next >= self.queue.len() {
            // Make room in the queue
            let in_use = self.queue_next - self.queue_pos;
            if self.queue_pos > 0 {
                // Shuffle the internal queue to avoid having to allocate more ram
                self.queue.copy_within(self.queue_pos..self.queue_next, 0);
            } else {
                // Expand the internal queue of step times
                let mut alloc = self.queue.len();
                if alloc == 0 {
                    alloc = QUEUE_START_SIZE;
                }
                while in_use >= alloc {
                    alloc *= 2;
                }
                self.queue.resize(alloc, 0);
            }
            self.queue_pos = 0;
            self.queue_next = in_use;
        }

        self.store_queued(self.next_step_clock);
        0
    }

    /// Store `next_step_clock` into the staged queue (all call sites).
    fn store_queued(&mut self, step_clock: u64) {
        if self.queue_next >= self.queue.len() {
            // Only reachable on the queue_append_far slow path, where the
            // C writes without a capacity check; grow instead of
            // corrupting the heap.
            let mut alloc = self.queue.len().max(QUEUE_START_SIZE);
            while self.queue_next >= alloc {
                alloc *= 2;
            }
            self.queue.resize(alloc, 0);
        }
        self.queue[self.queue_next] = step_clock as u32;
        self.queue_next += 1;
        self.next_step_clock = 0;
    }

    /// Add a step time to the queue (`queue_append`).
    fn queue_append(&mut self) -> i32 {
        if self.next_step_dir != self.sdir {
            let ret = self.set_next_step_dir(self.next_step_dir);
            if ret != 0 {
                return ret;
            }
        }
        if self.next_step_clock
            >= self
                .last_step_clock
                .wrapping_add(CLOCK_DIFF_MAX)
        {
            return self.queue_append_far();
        }
        if self.queue_next >= self.queue.len() {
            return self.queue_append_extend();
        }
        self.store_queued(self.next_step_clock);
        0
    }
}

/// `idiv_up`: `(n + d - 1) / d` for non-negative `n`, else `n / d`.
/// Division truncates toward zero on both sides; `d` is always positive at
/// the call sites.
fn idiv_up(n: i32, d: i32) -> i32 {
    if n >= 0 {
        n.wrapping_add(d).wrapping_sub(1) / d
    } else {
        n / d
    }
}

/// `idiv_down`: `n / d` for non-negative `n`, else `(n - d + 1) / d`.
fn idiv_down(n: i32, d: i32) -> i32 {
    if n >= 0 {
        n / d
    } else {
        n.wrapping_sub(d).wrapping_add(1) / d
    }
}
