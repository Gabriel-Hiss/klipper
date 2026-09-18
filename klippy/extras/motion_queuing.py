# Helper code for low-level motion queuing and flushing
#
# Copyright (C) 2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections, logging
import chelper

BGFLUSH_LOW_TIME = 0.200
BGFLUSH_HIGH_TIME = 0.400
BGFLUSH_SG_LOW_TIME = 1.000
BGFLUSH_SG_HIGH_TIME = 1.250
BGFLUSH_EXTRA_TIME = 0.250

MOVE_HISTORY_EXPIRE = 30.
MIN_KIN_TIME = 0.100
STEPCOMPRESS_FLUSH_TIME = 0.050
SDS_CHECK_TIME = 0.001 # step+dir+step filter in stepcompress.c

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.100

# [DEBUG-motion] v4 bounded host production timing instrumentation
DEBUG_RING_SIZE = 32
DEBUG_SLOW_RING_SIZE = 16
DEBUG_WAKE_RING_SIZE = 48
DEBUG_SLOW_FLUSH_TIME = 0.010
DEBUG_LATE_WAKE_TIME = 0.020
DEBUG_DUMP_ENTRIES = 10

def debug_callback_name(cb):
    # Resolve a flush callback name; only used on the reporting path
    if cb is None:
        return "-"
    name = getattr(cb, '__qualname__', None)
    if name is None:
        name = getattr(cb, '__name__', None) or type(cb).__name__
    obj = getattr(cb, '__self__', None)
    if obj is not None:
        oname = getattr(obj, '_name', None)
        if oname:
            name = "%s(%s)" % (name, oname)
    return name[:48]

class DebugFlushBatch:
    # Phase timing for one _advance_flush_time() batch (host side only)
    __slots__ = ('seq', 'start', 'flush_time', 'step_gen_time',
                 'prev_flush_time', 'prev_step_gen_time', 'cb_time',
                 'gen_time', 'fin_time', 'total_time', 'stage',
                 'slow_cb_time', 'slow_cb_name', 'cb_count', 'trapq_count')
    def __init__(self, seq, start, flush_time, step_gen_time,
                 prev_flush_time, prev_step_gen_time):
        self.seq = seq
        self.start = start
        self.flush_time = flush_time
        self.step_gen_time = step_gen_time
        self.prev_flush_time = prev_flush_time
        self.prev_step_gen_time = prev_step_gen_time
        self.cb_time = self.gen_time = self.fin_time = self.total_time = 0.
        self.slow_cb_time = 0.
        self.stage = "callbacks"
        self.slow_cb_name = "-"
        self.cb_count = self.trapq_count = 0
    def desc(self):
        return ("seq=%d at=%.6f total=%.6f cb=%.6f gen=%.6f fin=%.6f"
                " stage=%s flush=%.9f step_gen=%.9f prev_flush=%.9f"
                " prev_step_gen=%.9f cbs=%d slow_cb=%.6f slow_cb_name=%s"
                " trapqs=%d"
                % (self.seq, self.start, self.total_time, self.cb_time,
                   self.gen_time, self.fin_time, self.stage, self.flush_time,
                   self.step_gen_time, self.prev_flush_time,
                   self.prev_step_gen_time, self.cb_count, self.slow_cb_time,
                   self.slow_cb_name, self.trapq_count))

class PrinterMotionQueuing:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        # C trapq tracking
        self.trapqs = []
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        # C steppersync tracking
        self.steppersyncmgr = ffi_main.gc(ffi_lib.steppersyncmgr_alloc(),
                                          ffi_lib.steppersyncmgr_free)
        self.syncemitters = []
        self.syncemitter_to_name = {}
        self.steppersyncs = []
        self.steppersyncmgr_gen_steps = ffi_lib.steppersyncmgr_gen_steps
        # History expiration
        self.clear_history_time = 0.
        # Flush notification callbacks
        self.flush_callbacks = []
        # Kinematic step generation scan window time tracking
        self.kin_flush_delay = SDS_CHECK_TIME
        # MCU tracking
        self.all_mcus = [m for n, m in printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.can_pause = True
        if self.mcu.is_fileoutput():
            self.can_pause = False
        # Flush tracking
        flush_handler = self._flush_handler
        if not self.can_pause:
            flush_handler = self._flush_handler_debug
        self.flush_timer = self.reactor.register_timer(flush_handler)
        self.do_kick_flush_timer = True
        self.last_flush_time = self.last_step_gen_time = 0.
        self.need_flush_time = self.need_step_gen_time = 0.
        self.debug_flush_time = self.debug_step_gen_time = 0.
        self.debug_flush_stage = "idle"
        # [DEBUG-motion] v4 bounded production timing rings
        self.debug_now = self.reactor.monotonic
        self.debug_batch_seq = self.debug_batch_count = 0
        self.debug_batch_pending = None
        self.debug_failed_batch = self.debug_worst_batch = None
        self.debug_batches = collections.deque([], DEBUG_RING_SIZE)
        self.debug_slow_batches = collections.deque([], DEBUG_SLOW_RING_SIZE)
        self.debug_wake_events = collections.deque([], DEBUG_WAKE_RING_SIZE)
        self.debug_batch_totals = [0., 0., 0., 0.]
        self.debug_expect_wake = self.debug_worst_wake_late = 0.
        self.debug_wake_count = self.debug_late_wake_count = 0
        # "Drip" timing (for homing and probing moves)
        self.drip_start_times = []
        # Register handlers
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        printer.register_event_handler("klippy:analyze_shutdown",
                                       self._debug_shutdown)
        logging.info("[DEBUG-motion] version=host-v4 motion_queuing timing"
                     " rings batches=%d slow=%d wakes=%d", DEBUG_RING_SIZE,
                     DEBUG_SLOW_RING_SIZE, DEBUG_WAKE_RING_SIZE)
    # C trapq tracking
    def allocate_trapq(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapqs.append(trapq)
        return trapq
    def wipe_trapq(self, trapq):
        # Expire any remaining movement in the trapq (force to history list)
        self.trapq_finalize_moves(trapq, self.reactor.NEVER, 0.)
    def lookup_trapq_append(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.trapq_append
    # C steppersync tracking
    def _lookup_steppersync(self, mcu):
        for ss_mcu, ss in self.steppersyncs:
            if ss_mcu is mcu:
                return ss
        ffi_main, ffi_lib = chelper.get_ffi()
        ss = ffi_lib.steppersyncmgr_alloc_steppersync(self.steppersyncmgr)
        self.steppersyncs.append((mcu, ss))
        return ss
    def allocate_syncemitter(self, mcu, name, alloc_stepcompress=True):
        sname = name.split()[-1]
        thread_name = sname.encode("utf-8")[:15]
        ss = self._lookup_steppersync(mcu)
        ffi_main, ffi_lib = chelper.get_ffi()
        se = ffi_lib.steppersync_alloc_syncemitter(ss, thread_name,
                                                   alloc_stepcompress)
        self.syncemitter_to_name[se] = name
        self.syncemitters.append(se)
        return se
    def setup_mcu_movequeue(self, mcu, serialqueue, move_count):
        # Setup steppersync object for the mcu's main movequeue
        ffi_main, ffi_lib = chelper.get_ffi()
        ss = self._lookup_steppersync(mcu)
        ffi_lib.steppersync_setup_movequeue(ss, serialqueue, move_count)
        mcu_freq = float(mcu.seconds_to_clock(1.))
        ffi_lib.steppersync_set_time(ss, 0., mcu_freq)
    def stats(self, eventtime):
        # Globally calibrate mcu clocks (and step generation clocks)
        sync_time = self.last_step_gen_time
        ffi_main, ffi_lib = chelper.get_ffi()
        for mcu, ss in self.steppersyncs:
            offset, freq = mcu.calibrate_clock(sync_time, eventtime)
            ffi_lib.steppersync_set_time(ss, offset, freq)
        # Calculate history expiration
        est_print_time = self.mcu.estimated_print_time(eventtime)
        self.clear_history_time = max(0., est_print_time - MOVE_HISTORY_EXPIRE)
        return False, ""
    # Flush notification callbacks
    def register_flush_callback(self, callback, can_add_trapq=False):
        if can_add_trapq:
            self.flush_callbacks = [callback] + self.flush_callbacks
        else:
            self.flush_callbacks = self.flush_callbacks + [callback]
    def unregister_flush_callback(self, callback):
        if callback in self.flush_callbacks:
            fcbs = list(self.flush_callbacks)
            fcbs.remove(callback)
            self.flush_callbacks = fcbs
    # Kinematic step generation scan window time tracking
    def get_kin_flush_delay(self):
        return self.kin_flush_delay
    def check_step_generation_scan_windows(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        kin_flush_delay = SDS_CHECK_TIME
        for se in self.syncemitters:
            sk = ffi_lib.syncemitter_get_stepper_kinematics(se)
            if sk == ffi_main.NULL:
                continue
            trapq = ffi_lib.itersolve_get_trapq(sk)
            if trapq == ffi_main.NULL:
                continue
            pre_active = ffi_lib.itersolve_get_gen_steps_pre_active(sk)
            post_active = ffi_lib.itersolve_get_gen_steps_post_active(sk)
            kin_flush_delay = max(kin_flush_delay, pre_active, post_active)
        self.kin_flush_delay = kin_flush_delay
    # Flush tracking
    def _handle_shutdown(self):
        self.can_pause = False
    def _debug_shutdown(self, msg, details):
        # Read Python bookkeeping only; do not flush or query hardware here.
        logging.error(
            "[DEBUG-motion] motion flush stage=%s attempted_flush=%.9f"
            " attempted_step_gen=%.9f last_flush=%.9f last_step_gen=%.9f"
            " need_flush=%.9f need_step_gen=%.9f clear_history=%.9f"
            " kin_flush_delay=%.9f can_pause=%s kick_timer=%s"
            " drip_count=%d trapqs=%d syncemitters=%d steppersyncs=%d"
            " flush_callbacks=%d",
            self.debug_flush_stage, self.debug_flush_time,
            self.debug_step_gen_time, self.last_flush_time,
            self.last_step_gen_time, self.need_flush_time,
            self.need_step_gen_time, self.clear_history_time,
            self.kin_flush_delay, self.can_pause, self.do_kick_flush_timer,
            len(self.drip_start_times), len(self.trapqs),
            len(self.syncemitters), len(self.steppersyncs),
            len(self.flush_callbacks))
        try:
            self._debug_dump_timing()
        except Exception:
            # Diagnostics must never replace the original shutdown reason.
            logging.exception("[DEBUG-motion] Unable to dump motion timing")
    def _debug_dump_timing(self):
        # Bounded dump of host production timing (v4)
        tot = self.debug_batch_totals
        logging.error(
            "[DEBUG-motion] version=host-v4 motion timing batches=%d"
            " cb_total=%.6f gen_total=%.6f fin_total=%.6f time_total=%.6f"
            " wakes=%d late_wakes=%d worst_late_wake=%.6f expect_wake=%.6f"
            " pending=%s", self.debug_batch_count, tot[0], tot[1], tot[2],
            tot[3], self.debug_wake_count, self.debug_late_wake_count,
            self.debug_worst_wake_late, self.debug_expect_wake,
            self.debug_batch_pending is not None)
        for label, batch in (("pending", self.debug_batch_pending),
                             ("failed", self.debug_failed_batch),
                             ("worst", self.debug_worst_batch)):
            if batch is not None:
                logging.error("[DEBUG-motion] motion batch %s %s", label,
                              batch.desc())
        for batch in list(self.debug_slow_batches)[-DEBUG_DUMP_ENTRIES:]:
            logging.error("[DEBUG-motion] motion batch slow %s", batch.desc())
        for batch in list(self.debug_batches)[-DEBUG_DUMP_ENTRIES:]:
            logging.error("[DEBUG-motion] motion batch recent %s",
                          batch.desc())
        for ev in list(self.debug_wake_events)[-DEBUG_DUMP_ENTRIES:]:
            logging.error(
                "[DEBUG-motion] motion wake at=%.6f late=%.6f dur=%.6f"
                " batches=%d ret_in=%.6f last_flush=%.9f last_step_gen=%.9f"
                " need_flush=%.9f need_step_gen=%.9f", *ev)
        for line in self.reactor.get_timing_debug_report(DEBUG_DUMP_ENTRIES):
            logging.error("%s", line)
    def _debug_note_batch(self, batch):
        self.debug_batch_count += 1
        tot = self.debug_batch_totals
        tot[0] += batch.cb_time
        tot[1] += batch.gen_time
        tot[2] += batch.fin_time
        tot[3] += batch.total_time
        self.debug_batches.append(batch)
        worst = self.debug_worst_batch
        if worst is None or batch.total_time > worst.total_time:
            self.debug_worst_batch = batch
        if batch.stage != "idle":
            # Batch did not complete - keep the failure timing
            self.debug_failed_batch = batch
            self.debug_slow_batches.append(batch)
        elif batch.total_time > DEBUG_SLOW_FLUSH_TIME:
            self.debug_slow_batches.append(batch)
    def _advance_flush_time(self, want_flush_time, want_step_gen_time=0.):
        flush_time = max(want_flush_time, self.last_flush_time,
                         want_step_gen_time - STEPCOMPRESS_FLUSH_TIME)
        step_gen_time = max(want_step_gen_time, self.last_step_gen_time,
                            flush_time)
        self.debug_flush_time = flush_time
        self.debug_step_gen_time = step_gen_time
        self.debug_flush_stage = "callbacks"
        # [DEBUG-motion] v4 - time each phase of the batch; the record stays
        # reachable via debug_batch_pending until the batch completes.
        now = self.debug_now
        self.debug_batch_seq += 1
        t_start = now()
        batch = DebugFlushBatch(self.debug_batch_seq, t_start, flush_time,
                                step_gen_time, self.last_flush_time,
                                self.last_step_gen_time)
        self.debug_batch_pending = batch
        try:
            # Invoke flush callbacks (if any)
            t_phase = t_start
            slow_cb = None
            with self.reactor.assert_no_pause():
                for cb in self.flush_callbacks:
                    cb(flush_time, step_gen_time)
                    t_cb = now()
                    cb_time = t_cb - t_phase
                    t_phase = t_cb
                    if cb_time > batch.slow_cb_time:
                        batch.slow_cb_time = cb_time
                        slow_cb = cb
            batch.cb_count = len(self.flush_callbacks)
            batch.cb_time = t_phase - t_start
            if slow_cb is not None:
                batch.slow_cb_name = debug_callback_name(slow_cb)
            # Determine maximum history to keep
            trapq_free_time = step_gen_time - self.kin_flush_delay
            clear_history_time = self.clear_history_time
            if not self.can_pause:
                clear_history_time = max(
                    0., trapq_free_time - MOVE_HISTORY_EXPIRE)
            # Generate stepper movement and transmit
            self.debug_flush_stage = "generate_steps"
            batch.stage = "generate_steps"
            ret = self.steppersyncmgr_gen_steps(
                self.steppersyncmgr, flush_time, step_gen_time,
                clear_history_time)
            t_gen = now()
            batch.gen_time = t_gen - t_phase
            if ret:
                msg = "Internal error in stepcompress"
                queue_name = self.syncemitter_to_name.get(ret)
                err_details = {
                    "queue_name": queue_name,
                    "flush_time": flush_time, "step_gen_time": step_gen_time,
                    "last_flush_time": self.last_flush_time,
                    "last_step_gen_time": self.last_step_gen_time,
                    "clear_history_time": clear_history_time }
                self.printer.invoke_shutdown(msg, err_details)
                raise self.mcu.error(msg)
            self.last_flush_time = flush_time
            self.last_step_gen_time = step_gen_time
            # Move processed trapq entries to history, and expire old history
            self.debug_flush_stage = "finalize_trapqs"
            batch.stage = "finalize_trapqs"
            for trapq in self.trapqs:
                self.trapq_finalize_moves(trapq, trapq_free_time,
                                          clear_history_time)
            batch.trapq_count = len(self.trapqs)
            batch.fin_time = now() - t_gen
            batch.stage = "idle"
            self.debug_flush_stage = "idle"
        finally:
            batch.total_time = now() - t_start
            self.debug_batch_pending = None
            self._debug_note_batch(batch)
    def _await_flush_time(self, want_flush_time):
        while 1:
            if self.last_flush_time >= want_flush_time or not self.can_pause:
                return
            systime = self.reactor.monotonic()
            est_print_time = self.mcu.estimated_print_time(systime)
            wait = want_flush_time - BGFLUSH_HIGH_TIME - est_print_time
            if wait <= 0.:
                return
            self.reactor.pause(systime + min(1., wait))
    def flush_all_steps(self):
        flush_time = self.need_step_gen_time
        self._await_flush_time(flush_time)
        self._advance_flush_time(flush_time)
    def calc_step_gen_restart(self, est_print_time):
        kin_time = max(est_print_time + MIN_KIN_TIME, self.last_step_gen_time)
        return kin_time + self.kin_flush_delay
    def _flush_handler(self, eventtime):
        # [DEBUG-motion] v4 - bounded wakeup accounting for the background
        # flush timer.  The flush policy itself is _flush_handler_body().
        now = self.debug_now
        t_enter = now()
        expect = self.debug_expect_wake
        wake_late = 0.
        if expect:
            wake_late = eventtime - expect
        seq = self.debug_batch_seq
        ret = self.reactor.NEVER
        try:
            ret = self._flush_handler_body(eventtime)
            return ret
        finally:
            self._debug_note_wake(eventtime, now() - t_enter, wake_late,
                                  self.debug_batch_seq - seq, ret)
    def _debug_note_wake(self, eventtime, duration, wake_late, nbatch, ret):
        self.debug_wake_count += 1
        if wake_late > DEBUG_LATE_WAKE_TIME:
            self.debug_late_wake_count += 1
            if wake_late > self.debug_worst_wake_late:
                self.debug_worst_wake_late = wake_late
        if ret >= self.reactor.NEVER:
            self.debug_expect_wake = 0.
            ret_in = -1.
        else:
            self.debug_expect_wake = ret
            ret_in = ret - eventtime
        self.debug_wake_events.append(
            (eventtime, wake_late, duration, nbatch, ret_in,
             self.last_flush_time, self.last_step_gen_time,
             self.need_flush_time, self.need_step_gen_time))
    def _flush_handler_body(self, eventtime):
        try:
            est_print_time = self.mcu.estimated_print_time(eventtime)
            aggr_sg_time = self.need_step_gen_time - 2.*self.kin_flush_delay
            if self.last_step_gen_time < aggr_sg_time:
                # Actively stepping - want more aggressive flushing
                want_sg_time = est_print_time + BGFLUSH_SG_HIGH_TIME
                batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME
                next_batch_time = self.last_step_gen_time + batch_time
                if next_batch_time > est_print_time:
                    # Improve run-to-run reproducibility by batching from last
                    if next_batch_time > want_sg_time + 0.005:
                        # Delay flushing until next wakeup
                        next_batch_time = self.last_step_gen_time
                    want_sg_time = next_batch_time
                want_sg_time = min(want_sg_time, aggr_sg_time)
                # Flush motion queues (if needed)
                if want_sg_time > self.last_step_gen_time:
                    self._advance_flush_time(0., want_sg_time)
            else:
                # Not stepping (or only step remnants) - use relaxed flushing
                want_flush_time = est_print_time + BGFLUSH_HIGH_TIME
                max_flush_time = self.need_flush_time + BGFLUSH_EXTRA_TIME
                want_flush_time = min(want_flush_time, max_flush_time)
                # Flush motion queues (if needed)
                if want_flush_time > self.last_flush_time:
                    self._advance_flush_time(want_flush_time)
            # Reschedule timer
            aggr_sg_time = self.need_step_gen_time - 2.*self.kin_flush_delay
            if self.last_step_gen_time < aggr_sg_time:
                waketime = self.last_step_gen_time - BGFLUSH_SG_LOW_TIME
            else:
                self.do_kick_flush_timer = True
                max_flush_time = self.need_flush_time + BGFLUSH_EXTRA_TIME
                if self.last_flush_time >= max_flush_time:
                    return self.reactor.NEVER
                waketime = self.last_flush_time - BGFLUSH_LOW_TIME
            return eventtime + waketime - est_print_time
        except:
            logging.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
        return self.reactor.NEVER
    def _flush_handler_debug(self, eventtime):
        # Use custom flushing code when in batch output mode
        try:
            faux_time = self.need_flush_time - 1.5
            batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME
            flush_count = 0
            while self.last_step_gen_time < faux_time:
                target = self.last_step_gen_time + batch_time
                if flush_count > 100. and faux_time > target:
                    target += int((faux_time-target) / batch_time) * batch_time
                self._advance_flush_time(0., target)
                flush_count += 1
            if flush_count:
                return self.reactor.NOW
            self._advance_flush_time(self.need_flush_time + BGFLUSH_EXTRA_TIME)
            self.do_kick_flush_timer = True
            return self.reactor.NEVER
        except:
            logging.exception("Exception in flush_handler_debug")
            self.printer.invoke_shutdown("Exception in flush_handler_debug")
        return self.reactor.NEVER
    def note_mcu_movequeue_activity(self, mq_time, is_step_gen=True):
        if is_step_gen:
            mq_time += self.kin_flush_delay
            self.need_step_gen_time = max(self.need_step_gen_time, mq_time)
        self.need_flush_time = max(self.need_flush_time, mq_time)
        if self.do_kick_flush_timer:
            self.do_kick_flush_timer = False
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
    # "Drip" timing (for homing and probing moves)
    def drip_update_time(self, start_time, end_time, drip_completion):
        self.drip_start_times.append(start_time)
        self._await_flush_time(start_time)
        # Disable background flushing from timer
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False
        self._advance_flush_time(start_time - SDS_CHECK_TIME, start_time)
        # Flush in segments until drip_completion signal
        flush_time = start_time
        while flush_time < end_time:
            if drip_completion.test():
                break
            curtime = self.reactor.monotonic()
            est_print_time = self.mcu.estimated_print_time(curtime)
            wait_time = flush_time - est_print_time - DRIP_TIME
            if wait_time > 0. and self.can_pause:
                # Pause before sending more steps
                drip_completion.wait(curtime + wait_time)
                continue
            flush_time = min(flush_time + DRIP_SEGMENT_TIME, end_time)
            self.note_mcu_movequeue_activity(flush_time)
            self._advance_flush_time(flush_time - SDS_CHECK_TIME, flush_time)
        # Restore background flushing
        self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
        self._advance_flush_time(flush_time + self.kin_flush_delay)
        self.drip_start_times.remove(start_time)
    def check_drip_timing(self):
        if not self.drip_start_times:
            return None
        return min(self.drip_start_times)

def load_config(config):
    return PrinterMotionQueuing(config)
