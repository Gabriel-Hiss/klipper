#!/usr/bin/env python3
"""Generic Klipper MCU simulator (parity/mcusim/mcusim.py).

Self-contained, stdlib only, Python 3.8 compatible. Runs under the
virtual clock (time.monotonic() is virtualized by parity/vclock when
LD_PRELOAD is active); MCU clock = monotonic * CLOCK_FREQ.

CLI:
    python3 parity/mcusim/mcusim.py --dict DICT --name NAME --pty PATH
        [--script SCRIPT.json] [--trace FILE] [--capture FILE]

Creates a pty and symlinks it at PATH (like the Linux MCU firmware
src/linux/console.c), serves the identify data block, implements the
block transport of src/command.c / klippy/chelper/msgblock.c, and
mirrors firmware semantics of src/*.c, with every timer as an event at
an absolute clock. See SCRIPT.md for the script format and timing model.
Trace: one JSON line per received command and sent response. Capture:
the raw bytes the host wrote.
"""
import argparse
import json
import math
import os
import pty
import select
import signal
import sys
import termios
import time
import tty
import zlib

# Wire constants (klippy/msgproto.py, src/command.h)
MESSAGE_MIN = 5
MESSAGE_MAX = 64
MESSAGE_HEADER_SIZE = 2
MESSAGE_TRAILER_SIZE = 3
MESSAGE_POS_LEN = 0
MESSAGE_POS_SEQ = 1
MESSAGE_TRAILER_CRC = 3
MESSAGE_TRAILER_SYNC = 1
MESSAGE_SEQ_MASK = 0x0F
MESSAGE_DEST = 0x10
MESSAGE_SYNC = 0x7E

NON_SHUTDOWN_ALLOWED = frozenset(
    ["get_clock", "get_uptime", "get_config", "identify",
     "emergency_stop", "clear_shutdown", "stats", "debug_nop", "debug_ping"])

MASK32 = 0xFFFFFFFF
# src/stepper.c POSITION_BIAS
POSITION_BIAS = 0x40000000
# BLTouch control pulse widths in seconds (klippy/extras/bltouch.py
# Commands). The sim decodes queue_digital_out on_ticks on the scripted
# control pin back to these commands.
BLTOUCH_COMMANDS = {
    "pin_down": 0.000650, "touch_mode": 0.001165, "pin_up": 0.001475,
    "self_test": 0.001780, "reset": 0.002190,
    "set_5V_output_mode": 0.001988, "set_OD_output_mode": 0.002091,
    "output_mode_store": 0.001884,
}
BLTOUCH_PULSE_TOLERANCE = 0.000050

def crc16_ccitt(buf):
    crc = 0xFFFF
    for data in buf:
        data ^= crc & 0xFF
        data ^= (data & 0x0F) << 4
        crc = ((data << 8) | (crc >> 8)) ^ (data >> 4) ^ (data << 3)
        crc &= 0xFFFF
    return crc


def encode_int(v):
    out = []
    v = int(v)
    if v >= 0x3000000 or v < -0x4000000:
        out.append(((v >> 28) & 0x7F) | 0x80)
        out.append(((v >> 21) & 0x7F) | 0x80)
        out.append(((v >> 14) & 0x7F) | 0x80)
        out.append(((v >> 7) & 0x7F) | 0x80)
        out.append(v & 0x7F)
    elif v >= 0x18000 or v < -0x80000:
        out.append(((v >> 21) & 0x7F) | 0x80)
        out.append(((v >> 14) & 0x7F) | 0x80)
        out.append(((v >> 7) & 0x7F) | 0x80)
        out.append(v & 0x7F)
    elif v >= 0x3000 or v < -0x1000:
        out.append(((v >> 14) & 0x7F) | 0x80)
        out.append(((v >> 7) & 0x7F) | 0x80)
        out.append(v & 0x7F)
    elif v >= 0x60 or v < -0x20:
        out.append(((v >> 7) & 0x7F) | 0x80)
        out.append(v & 0x7F)
    else:
        out.append(v & 0x7F)
    return out


def decode_int(data, pos):
    c = data[pos]
    pos += 1
    v = c & 0x7F
    if (c & 0x60) == 0x60:
        v |= -0x20
    while c & 0x80:
        c = data[pos]
        pos += 1
        v = (v << 7) | (c & 0x7F)
    return v, pos


def encode_msgid(msgid):
    msgid = int(msgid)
    if msgid >= 0x80:
        return [(msgid >> 7) | 0x80, msgid & 0x7F]
    return [msgid & 0x7F]


def decode_msgid(data, pos):
    v = data[pos]
    pos += 1
    if v & 0x80:
        v = ((v & 0x7F) << 7) | data[pos]
        pos += 1
    return v, pos


def parse_format(fmt):
    """Return list of (type, name) from a dictionary format string."""
    parts = fmt.split()
    out = []
    for part in parts[1:]:
        if "=" not in part:
            continue
        name, typ = part.split("=", 1)
        out.append((typ, name))
    return out


INT_TYPES = ("%u", "%i", "%hu", "%hi", "%c")


def is_int_type(typ):
    return typ in INT_TYPES


def signed_type(typ):
    return typ in ("%i", "%hi")


def encode_params(fmt, values):
    out = []
    for typ, _name in parse_format(fmt):
        if is_int_type(typ):
            out += encode_int(values.pop(0))
        elif typ in ("%s", "%.*s", "%*s"):
            blob = values.pop(0)
            if isinstance(blob, str):
                blob = blob.encode()
            out.append(len(blob))
            out.extend(blob)
        else:
            raise ValueError("unknown param type %s" % (typ,))
    return out


def decode_params(fmt, data, pos):
    params = {}
    for typ, name in parse_format(fmt):
        if is_int_type(typ):
            v, pos = decode_int(data, pos)
            if not signed_type(typ):
                v = v & 0xFFFFFFFF
            params[name] = v
        elif typ in ("%s", "%.*s", "%*s"):
            ln = data[pos]
            pos += 1
            params[name] = bytes(bytearray(data[pos:pos + ln]))
            pos += ln
        else:
            raise ValueError("unknown param type %s" % (typ,))
    return params, pos


def format_command(fmt, params):
    parts = [fmt.split()[0]]
    for typ, name in parse_format(fmt):
        if name in params:
            v = params[name]
            if isinstance(v, bytes):
                v = v.hex()
            parts.append("%s=%s" % (name, v))
    return " ".join(parts)


class Sim:
    def __init__(self, dict_bytes, name, script, trace=None):
        self.name = name
        self.raw_dict = dict_bytes
        self.d = json.loads(dict_bytes.decode())
        self.cfg = self.d.get("config", {})
        self.freq = int(self.cfg.get("CLOCK_FREQ", 16000000))
        self.commands = self.d.get("commands", {})       # format -> msgid
        self.responses = self.d.get("responses", {})     # format -> msgid
        self.output = self.d.get("output", {})
        self.enumerations = self.d.get("enumerations", {})
        self.id_by_format = dict(self.commands)
        self.id_by_format.update(self.responses)
        self.id_by_format.update(self.output)
        self.fmt_by_id = {v: k for k, v in self.id_by_format.items()}
        self.fmt_by_name = {}
        for fmt in self.id_by_format:
            self.fmt_by_name[fmt.split()[0]] = fmt
        self.script = script or {}
        self.trace_f = open(trace, "w") if trace else None
        self.cap_f = None
        self.fd = None
        # Single wire counter exactly like src/command.c next_sequence:
        # init MESSAGE_DEST, frames every tx with it, advances only when
        # a valid host block arrives. The host re-bases its send_seq off
        # the first nak ("connection init" in serialqueue.c), which is
        # what aligns both sides after boot.
        self.next_sequence = MESSAGE_DEST
        self.inbuf = bytearray()
        self.shutdown = False
        self.shutdown_reason = 0
        self.is_config = 0
        self.config_crc = 0
        start = self.script.get("start_state", {})
        self.is_config = int(start.get("is_config", 0))
        self.config_crc = int(start.get("crc", 0))
        self.oid_count = 0
        self.steppers = {}
        self.endstops = {}
        self.trsyncs = {}
        self.adcs = {}
        self.gpios = {}        # digital_out oid -> dict
        self.spi = {}
        self.i2c = {}
        self.tmcuart = {}
        self.buttons = {}
        self.neopixels = {}
        self.bulk = {}
        self.debug_vals = {}
        self.pin_values = {}   # output pin number -> level (set_digital_out)
        self.stop_links = {}   # trsync_oid -> list of stepper oids
        self.clock0 = time.monotonic()
        self.thermal = {}      # adc oid -> thermal model state
        self.pwm_duty = {}     # digital_out oid -> duty 0..1
        self.pwm_pin = {}      # digital_out oid -> pin
        self.last_thermal_t = 0.0  # virtual seconds since sim start
        self.seq_counters = {}  # bulk sensor oid -> next sequence
        self.spi_reply_idx = {}
        self.tmcuart_idx = {}
        self.i2c_idx = {}
        self.forced_shutdown_at = self.script.get("forced_shutdown_at")
        self.forced_done = False
        # Input pin model (see SCRIPT.md): static levels, positional
        # endstop contacts and the BLTouch probe.
        self.pin_levels = self.script.get("pins", {})
        self.endstop_rules = self.script.get("endstop_triggers", {})
        self.bltouch = None
        bl = self.script.get("bltouch")
        if bl is not None:
            steps = [int(n) for n in bl["contact_steps"]]
            if not steps:
                raise ValueError("bltouch.contact_steps must not be empty")
            self.bltouch = {"sensor_pin": bl["sensor_pin"],
                            "control_pin": bl["control_pin"],
                            "contact_steps": steps, "events": [],
                            "descents": 0}

    def write_all(self, data):
        if self.fd is None:
            raise AttributeError("sim fd not set; assign sim.fd before sending")
        payload = bytes(data)
        self.capture_transport("tx", payload)
        view = memoryview(payload)
        while view:
            n = os.write(self.fd, view)
            view = view[n:]


    # -- clock ------------------------------------------------------
    def now64(self):
        return int(round((time.monotonic() - self.clock0) * self.freq))

    def now32(self):
        return self.now64() & 0xFFFFFFFF

    def clock_to_64(self, clk32):
        # Interpret 32-bit clock as nearest to current 64-bit clock.
        now = self.now64()
        now32 = now & 0xFFFFFFFF
        diff = (int(clk32) & 0xFFFFFFFF) - now32
        if diff >= (1 << 31):
            diff -= (1 << 32)
        elif diff < -(1 << 31):
            diff += (1 << 32)
        return now + diff

    def virt_t(self):
        return time.monotonic() - self.clock0

    # -- trace ------------------------------------------------------
    def trace(self, direction, text):
        if self.trace_f:
            self.trace_f.write(json.dumps(
                {"t": round(self.virt_t(), 6), "clock": self.now32(),
                 "mcu": self.name, "dir": direction, "msg": text}) + "\n")

    def capture_transport(self, direction, data):
        # Raw transport bytes for the harness mcu/<name>.bin artifact.
        # Only host-to-sim ("rx") is captured: like batch -o files, the
        # artifact holds the command stream parsedump decodes, not the
        # interleaved responses.
        if direction != "rx":
            return
        if self.cap_f is not None and data:
            self.cap_f.write(bytes(data))
            try:
                self.cap_f.flush()
            except Exception:
                pass

    # -- send -------------------------------------------------------
    def send_msg(self, fmt, values):
        values = list(values)
        msgid = self.id_by_format[fmt]
        payload = encode_msgid(msgid) + encode_params(fmt, values)
        self.send_block(payload)
        if self.trace_f:
            parts = [fmt.split()[0]]
            for (typ, name), v in zip(parse_format(fmt), values):
                if isinstance(v, bytes):
                    v = v.hex()
                parts.append("%s=%s" % (name, v))
            self.trace("tx", " ".join(parts))

    def send_block(self, payload):
        if len(payload) + MESSAGE_MIN > MESSAGE_MAX:
            raise ValueError("message too large")
        msglen = MESSAGE_MIN + len(payload)
        out = [msglen, self.next_sequence] + list(payload)
        crc = crc16_ccitt(out)
        out.append((crc >> 8) & 0xFF)
        out.append(crc & 0xFF)
        out.append(MESSAGE_SYNC)
        # Firmware command_add_frame does not advance the counter; only
        # command_find_block does on valid receipt.
        self.write_all(bytes(out))

    def send_ack(self):
        self.send_block([])

    def send_by_name(self, name, values):
        fmt = self.fmt_by_name.get(name)
        if fmt is None:
            return
        self.send_msg(fmt, values)

    # -- shutdown ---------------------------------------------------
    def do_shutdown(self, reason_name):
        if self.shutdown:
            return
        self.shutdown = True
        sid = 0
        table = self.enumerations.get("static_string_id", {})
        if isinstance(table, dict):
            sid = int(table.get(reason_name, 0))
            # Range form {name: [val, count]} unsupported; strings only.
            if isinstance(table.get(reason_name), list):
                sid = int(table[reason_name][0])
        self.shutdown_reason = sid
        # Firmware shutdown handlers stop every timer: stepper_shutdown,
        # trsync_shutdown, and no further endstop/ADC/button events.
        for st in self.steppers.values():
            st["queue"] = []
            self.stepper_stop(st)
        for oid in list(self.trsyncs):
            self.trsyncs[oid] = self.trsync_cleared()
        self.stop_links.clear()
        for e in self.endstops.values():
            e["active"] = False
        for a in self.adcs.values():
            a["active"] = False
        for b in self.buttons.values():
            b["send_at"] = None
        self.send_by_name("shutdown", [self.now32(), sid])

    def report_is_shutdown(self):
        self.send_by_name("is_shutdown", [self.shutdown_reason])

    # -- command handlers -------------------------------------------
    def handle(self, cmd_name, params):
        fn = getattr(self, "cmd_" + cmd_name, None)
        if fn is not None:
            try:
                fn(params)
            except Exception as exc:
                # A handler bug must stay visible: trace it and write it
                # to stderr (collected as mcusim-<name>.log).
                self.trace("tx", "error " + cmd_name + ": " + repr(exc))
                sys.stderr.write("mcusim %s: error in %s %r: %r\n"
                                 % (self.name, cmd_name, params, exc))
                sys.stderr.flush()
            return
        # Unsimulated commands: accept and trace, never crash.
        self.trace("rx", "unhandled " + cmd_name)

    def cmd_identify(self, p):
        offset = p["offset"]
        count = p["count"]
        data = zlib.compress(self.raw_dict, 9)
        if offset >= len(data):
            count = 0
        elif offset + count > len(data):
            count = len(data) - offset
        self.send_by_name("identify_response", [offset, data[offset:offset + count] if count else b""])

    def cmd_get_clock(self, p):
        self.send_by_name("clock", [self.now32()])

    def cmd_get_uptime(self, p):
        now = self.now64()
        self.send_by_name("uptime", [(now >> 32) & 0xFFFFFFFF, now & 0xFFFFFFFF])

    def cmd_get_config(self, p):
        move_count = 512 if self.is_config else 0
        # main dict order: is_config,crc,is_shutdown,move_count
        vals = [1 if self.is_config else 0, self.config_crc,
                1 if self.shutdown else 0, move_count]
        self.send_by_name("config", vals)

    def cmd_finalize_config(self, p):
        if self.is_config:
            # Already finalized still accepts crc update? Firmware
            # shuts down on double finalize; host never does this.
            s = "Already finalized"
        else:
            s = None
        self.is_config = 1
        self.config_crc = p.get("crc", 0) & 0xFFFFFFFF
        if s:
            self.do_shutdown(s)

    def cmd_allocate_oids(self, p):
        if self.oid_count:
            self.do_shutdown("oids already allocated")
            return
        self.oid_count = p["count"]

    def cmd_reset(self, p):
        self.do_reset()

    def cmd_config_reset(self, p):
        self.do_reset()

    def do_reset(self):
        if not self.shutdown:
            self.do_shutdown("config_reset only available when shutdown")
            return
        self.shutdown = False
        self.shutdown_reason = 0
        self.is_config = 0
        start = self.script.get("start_state", {})
        self.config_crc = int(start.get("crc", 0)) if start and 0 else 0
        self.oid_count = 0
        self.steppers.clear()
        self.endstops.clear()
        self.trsyncs.clear()
        self.adcs.clear()
        self.gpios.clear()
        self.spi.clear()
        self.i2c.clear()
        self.tmcuart.clear()
        self.buttons.clear()
        self.neopixels.clear()
        self.bulk.clear()
        self.pwm_duty.clear()
        self.stop_links.clear()
        self.send_by_name("starting", [])

    def cmd_emergency_stop(self, p):
        self.do_shutdown("Command request")

    def cmd_clear_shutdown(self, p):
        if not self.shutdown:
            self.do_shutdown("Shutdown cleared when not shutdown")
            return
        self.shutdown = False
        self.shutdown_reason = 0

    def cmd_stats(self, p):
        self.send_by_name("stats", [0, 0, 0])

    def cmd_debug_nop(self, p):
        return

    def cmd_debug_ping(self, p):
        data = p.get("data", b"")
        self.send_by_name("pong", [data])

    def cmd_debug_read(self, p):
        self.send_by_name("debug_result", [self.debug_vals.get(p.get("addr", 0), 0)])

    def cmd_debug_write(self, p):
        self.debug_vals[p.get("addr", 0)] = p.get("val", 0)
        self.send_by_name("debug_result", [p.get("val", 0)])

    # steppers (src/stepper.c). Positions use the firmware's uint32
    # encoding (POSITION_BIAS plus the top-bit reverse flag) so
    # stepper_position replies match the real MCU bit for bit. Steps
    # run at the clocks the host scheduled: reset_step_clock sets the
    # base, a move's first step is `interval` after the previous step,
    # and each later step adds the running interval, which grows by
    # `add` (stepper_load_next / stepper_event_full).
    def stepper_state(self, oid):
        st = self.steppers.get(oid)
        if st is None:
            st = {"queue": [], "count": 0, "interval": 0, "add": 0,
                  "next_step_time": 0,
                  "position": (-POSITION_BIAS) & MASK32,
                  "last_dir": False, "next_dir": False,
                  "need_reset": False, "steps_taken": 0}
            self.steppers[oid] = st
        return st

    def cmd_config_stepper(self, p):
        st = self.stepper_state(p["oid"])
        st["step_pin"] = p["step_pin"]
        st["dir_pin"] = p["dir_pin"]

    def stepper_load_next(self, st):
        if not st["queue"]:
            st["count"] = 0
            return
        m = st["queue"].pop(0)
        pos = ((-st["position"]) & MASK32) if m["dir"] else st["position"]
        st["position"] = (pos + m["count"]) & MASK32
        st["add"] = m["add"]
        st["interval"] = m["interval"] + m["add"]
        # The firmware adds in uint32 (after a homing stop the host
        # sends reset_step_clock clock=0 and an absolute 32-bit first
        # interval), so extend the sum to the 64-bit clock nearest now.
        st["next_step_time"] = self.clock_to_64(
            (st["next_step_time"] + m["interval"]) & MASK32)
        st["count"] = m["count"]

    def cmd_queue_step(self, p):
        st = self.stepper_state(p["oid"])
        if not p["count"]:
            self.do_shutdown("Invalid count parameter")
            return
        move = {"interval": p["interval"], "count": p["count"],
                "add": p["add"], "dir": st["last_dir"] != st["next_dir"]}
        if st["count"]:
            st["last_dir"] = st["next_dir"]
            st["queue"].append(move)
        elif st["need_reset"]:
            # Stopped by a trsync trigger: the firmware frees the move
            # until reset_step_clock clears SF_NEED_RESET.
            self.trace("rx", "queue_step dropped (need reset)")
        else:
            st["last_dir"] = st["next_dir"]
            st["queue"].append(move)
            self.stepper_load_next(st)

    def cmd_set_next_step_dir(self, p):
        self.stepper_state(p["oid"])["next_dir"] = bool(p["dir"])

    def cmd_reset_step_clock(self, p):
        st = self.stepper_state(p["oid"])
        if st["count"]:
            self.do_shutdown("Can't reset time when stepper active")
            return
        st["next_step_time"] = self.clock_to_64(p["clock"])
        st["need_reset"] = False

    def stepper_position_raw(self, st):
        # stepper_get_position(): subtract steps not yet taken, undo the
        # top-bit reverse flag.
        pos = (st["position"] - st["count"]) & MASK32
        if pos & 0x80000000:
            pos = (-pos) & MASK32
        return pos

    def cmd_stepper_get_position(self, p):
        st = self.stepper_state(p["oid"])
        pos = (self.stepper_position_raw(st) - POSITION_BIAS) & MASK32
        if pos & 0x80000000:
            pos -= 1 << 32
        self.send_by_name("stepper_position", [p["oid"], pos])

    def cmd_stepper_stop_on_trigger(self, p):
        self.stepper_state(p["oid"])
        self.stop_links.setdefault(p["trsync_oid"], []).append(p["oid"])

    def stepper_advance(self, st, until):
        # Take every step scheduled at or before clock `until`.
        while st["count"] and st["next_step_time"] <= until:
            st["steps_taken"] += 1
            st["count"] -= 1
            if st["count"]:
                st["next_step_time"] += st["interval"]
                st["interval"] += st["add"]
            else:
                self.stepper_load_next(st)

    def advance_steppers(self, until):
        for st in self.steppers.values():
            self.stepper_advance(st, until)

    def stepper_stop(self, st):
        # stepper_stop(): keep the reached position, drop pending steps,
        # clear the direction flags and latch SF_NEED_RESET.
        st["position"] = (-self.stepper_position_raw(st)) & MASK32
        st["count"] = 0
        st["next_step_time"] = 0
        st["queue"] = []
        st["last_dir"] = st["next_dir"] = False
        st["need_reset"] = True

    def stepper_future_step_time(self, st, n):
        # Clock of the n-th (1-based) step not yet taken, or None while
        # fewer than n steps are queued.
        if n < 1 or not st["count"]:
            return None
        t, interval, add, count = (st["next_step_time"], st["interval"],
                                   st["add"], st["count"])
        moves = iter(st["queue"])
        while True:
            if n <= count:
                k = n - 1
                return t + k * interval + add * k * (k - 1) // 2
            n -= count
            k = count - 1
            last = t + k * interval + add * k * (k - 1) // 2
            m = next(moves, None)
            if m is None:
                return None
            t = last + m["interval"]
            interval = m["interval"] + m["add"]
            add = m["add"]
            count = m["count"]

    # Input pins. Levels are physical (what gpio_in_read returns).
    def static_pin_level(self, pin, pull_up):
        if pin is not None:
            for key in (self.pin_name(pin), str(pin)):
                if key in self.pin_levels:
                    return int(self.pin_levels[key])
        return 1 if pull_up else 0

    def pin_name(self, pin):
        # Reverse the dict pin enumeration, expanding range entries
        # ("PA0": [0, 16] means PA0..PA15 = 0..15, as in
        # msgproto.fill_enumerations).
        table = self.enumerations.get("pin", {})
        if not isinstance(pin, int):
            return str(pin)
        if isinstance(table, dict):
            for name, val in table.items():
                if isinstance(val, (list, tuple)):
                    start, count = int(val[0]), int(val[1])
                    if not start <= pin < start + count:
                        continue
                    root = str(name)
                    while root and root[-1].isdigit():
                        root = root[:-1]
                    base = int(str(name)[len(root):] or 0)
                    return "%s%d" % (root, base + pin - start)
                if val == pin:
                    return name
        return str(pin)

    # endstops (src/endstop.c) and trsync (src/trsync.c). The sim does
    # not poll: the trigger clock of each armed endstop is predicted from
    # the scripted pin model, sampling every rest_ticks and then
    # requiring sample_count-1 more matching samples sample_ticks apart
    # (endstop_event / endstop_oversample_event). run_until() processes
    # that trigger in clock order with step, report and expire events,
    # so linked steppers stop on the exact step the firmware would.
    def cmd_config_endstop(self, p):
        self.endstops[p["oid"]] = {"pin": p["pin"], "pull_up": p["pull_up"],
                                   "homing": False, "active": False,
                                   "nextwake": 0}

    def cmd_endstop_home(self, p):
        e = self.endstops.setdefault(p["oid"], {"pin": None, "pull_up": 0,
                                                "nextwake": 0})
        e["active"] = False
        if not p["sample_count"]:
            e["homing"] = False
            return
        e.update({"homing": True, "active": True, "armed": False,
                  "waketime": self.clock_to_64(p["clock"]),
                  "sample_ticks": p["sample_ticks"],
                  "sample_count": p["sample_count"],
                  "rest_ticks": p["rest_ticks"], "want": p["pin_value"],
                  "trsync_oid": p["trsync_oid"], "reason": p["trigger_reason"],
                  "base": {}, "crossing": {}, "travel": None})

    def endstop_arm(self, e, clock):
        # Runs at the first sample clock (the "arm" event): travel counts
        # the linked steppers' steps after this clock, so a retract move
        # still running when endstop_home arrived does not count toward
        # the next contact.
        e["armed"] = True
        e["base"] = {so: self.stepper_state(so)["steps_taken"]
                     for so in self.stop_links.get(e["trsync_oid"], [])}
        e["travel"] = self.endstop_travel(e, clock)

    def endstop_travel(self, e, clock):
        # Steps of linked-stepper travel after arming until the pin
        # reaches its triggered level, or None when motion never
        # triggers it.
        name = self.pin_name(e["pin"])
        bl = self.bltouch
        if bl is not None and name == bl["sensor_pin"]:
            if not self.bltouch_state(clock)["lowered"]:
                return None
            steps = bl["contact_steps"]
            travel = steps[bl["descents"] % len(steps)]
            bl["descents"] += 1
            self.trace("rx", "bltouch descent %d contact after %d steps"
                       % (bl["descents"], travel))
            return travel
        rule = self.endstop_rules.get(name)
        if rule is not None:
            return int(rule["travel_steps"])
        return None

    def endstop_crossing(self, e):
        # Clock of the step at which a linked stepper has travelled
        # `travel` steps since arming (earliest over linked steppers), or
        # None while unknown. Each stepper's clock is cached once known:
        # later queue_step commands only append steps after it.
        travel = e.get("travel")
        if travel is None or not e.get("homing") or not e.get("armed"):
            return None
        if travel <= 0:
            return e["waketime"]
        best = None
        for so, base in e["base"].items():
            t = e["crossing"].get(so)
            if t is None:
                st = self.steppers[so]
                need = travel - (st["steps_taken"] - base)
                if need <= 0:
                    raise RuntimeError("endstop crossing passed unobserved")
                t = self.stepper_future_step_time(st, need)
                if t is None:
                    continue
                e["crossing"][so] = t
            if best is None or t < best:
                best = t
        return best

    def endstop_level(self, e, clock):
        # Physical level of the endstop pin at `clock`.
        crossing = self.endstop_crossing(e)
        crossed = crossing is not None and clock >= crossing
        name = self.pin_name(e["pin"])
        bl = self.bltouch
        if bl is not None and name == bl["sensor_pin"]:
            state = self.bltouch_state(clock)
            if state["lowered"]:
                return 1 if crossed else 0
            return 1 if state["touch"] else 0
        rule = self.endstop_rules.get(name)
        if rule is not None:
            level = int(rule["triggered_level"])
            return level if crossed else 1 - level
        return self.static_pin_level(e["pin"], e.get("pull_up", 0))

    def endstop_predict(self, e):
        # (trigger clock, nextwake) of an armed endstop given everything
        # known so far, or None if no known future sample triggers it.
        want = e["want"]
        rest = max(1, e["rest_ticks"])
        sample_ticks = e["sample_ticks"]
        count = e["sample_count"]
        points = []
        crossing = self.endstop_crossing(e)
        if crossing is not None:
            points.append(crossing)
        bl = self.bltouch
        if bl is not None and self.pin_name(e["pin"]) == bl["sensor_pin"]:
            points.extend(clock for clock, _cmd in bl["events"])
        points.sort()
        cur = e["waketime"]
        for _guard in range(100000):
            if self.endstop_level(e, cur) == want:
                for j in range(1, count):
                    if self.endstop_level(e, cur + j * sample_ticks) != want:
                        break
                else:
                    return cur + (count - 1) * sample_ticks, cur + rest
                cur += rest
                continue
            later = [pt for pt in points if pt > cur]
            if not later:
                return None
            cur += -(-(later[0] - cur) // rest) * rest
        raise RuntimeError("endstop prediction did not converge")

    def cmd_endstop_query_state(self, p):
        e = self.endstops.get(p["oid"], {"homing": False, "nextwake": 0,
                                         "pin": None})
        self.send_by_name("endstop_state",
                          [p["oid"], 1 if e.get("homing") else 0,
                           e.get("nextwake", 0) & MASK32,
                           self.endstop_level(e, self.now64())])

    def trsync_cleared(self):
        return {"can": False, "report_ticks": 0, "report_time": None,
                "expire": None, "expire_reason": 0, "reason": 0}

    def cmd_config_trsync(self, p):
        self.trsyncs[p["oid"]] = self.trsync_cleared()

    def cmd_trsync_start(self, p):
        oid = p["oid"]
        ts = self.trsync_cleared()
        ts.update({"can": True, "report_ticks": p["report_ticks"],
                   "expire_reason": p["expire_reason"]})
        if p["report_ticks"]:
            ts["report_time"] = self.clock_to_64(p["report_clock"])
        self.trsyncs[oid] = ts
        # trsync_clear() drops every registered stop signal.
        self.stop_links[oid] = []

    def cmd_trsync_set_timeout(self, p):
        ts = self.trsyncs.get(p["oid"])
        if ts is not None and ts["can"]:
            ts["expire"] = self.clock_to_64(p["clock"])

    def trsync_do_trigger(self, oid, reason, clock, report=True):
        ts = self.trsyncs.get(oid)
        if ts is None or not ts["can"]:
            return
        ts["can"] = False
        ts["reason"] = reason
        for so in self.stop_links.pop(oid, []):
            st = self.steppers.get(so)
            if st is not None:
                self.stepper_advance(st, clock)
                self.stepper_stop(st)
        if report:
            # trsync_task reports the trigger with timer_read_time().
            self.send_by_name("trsync_state", [oid, 0, reason, clock & MASK32])

    def cmd_trsync_trigger(self, p):
        oid = p["oid"]
        # ts->flags = 0 cancels the task report of this trigger; the
        # command answers with its own report (clock 0).
        self.trsync_do_trigger(oid, p["reason"], self.now64(), report=False)
        ts = self.trsyncs.setdefault(oid, self.trsync_cleared())
        ts["can"] = False
        ts["report_time"] = None
        ts["expire"] = None
        self.send_by_name("trsync_state", [oid, 0, ts["reason"], 0])

    # ADC (src/adccmds.c). Script entries ("adc", "thermal") are keyed by
    # analog_in oid or by MCU pin name; pin names survive oid
    # renumbering between configs.
    def script_entry(self, section, oid, pin):
        table = self.script.get(section, {})
        if str(oid) in table:
            return table[str(oid)]
        if pin is not None:
            name = self.pin_name(pin)
            if name in table:
                return table[name]
        return None

    def cmd_config_analog_in(self, p):
        self.adcs[p["oid"]] = {"pin": p["pin"], "active": False, "batch": []}

    def adc_script_value(self, cfg, t):
        table = cfg.get("table")
        if table:
            # [[t, value], ...] piecewise linear, clamped.
            prev_t, prev_v = table[0]
            for tt, vv in table[1:]:
                if t >= tt:
                    prev_t, prev_v = tt, vv
                    continue
                if prev_t == tt:
                    return float(prev_v)
                frac = (t - prev_t) / (tt - prev_t)
                return prev_v + frac * (vv - prev_v)
            return float(prev_v)
        mode = cfg.get("mode", "constant")
        if mode != "constant":
            raise ValueError("unknown adc mode %r" % (mode,))
        return float(cfg.get("value", 0))

    def thermal_init(self, oid, cfg):
        if oid not in self.thermal:
            self.thermal[oid] = {
                "value": float(cfg.get("ambient_adc", 0)),
                "ambient": float(cfg.get("ambient_adc", 0)),
                "span": float(cfg.get("span_adc", 1000)),
                "tau": float(cfg.get("tau_seconds", 10.0)),
                "pwm": cfg.get("pwm_oid")}

    def thermal_update(self, now_t):
        dt = now_t - self.last_thermal_t
        self.last_thermal_t = now_t
        if dt <= 0:
            return
        for oid, th in self.thermal.items():
            pwm_oid = th.get("pwm")
            duty = self.pwm_duty.get(pwm_oid, 0.0) if pwm_oid is not None \
                else 0.0
            target = th["ambient"] + duty * th["span"]
            # First order response: dv/dt = (target - v) / tau.
            # Exact per-step integration is v += (target-v)*(1-exp(-dt/tau)).
            frac = 1.0 - math.exp(-dt / th["tau"]) if th["tau"] > 0 else 1.0
            th["value"] += (target - th["value"]) * frac

    def adc_value(self, oid, t):
        pin = self.adcs.get(oid, {}).get("pin")
        cfg = self.script_entry("thermal", oid, pin)
        if cfg is not None:
            self.thermal_init(oid, cfg)
            self.thermal_update(t)
            return self.thermal[oid]["value"]
        cfg = self.script_entry("adc", oid, pin)
        if cfg is not None:
            return self.adc_script_value(cfg, t)
        return 0.0

    def cmd_query_analog_in(self, p):
        oid = p["oid"]
        a = self.adcs.setdefault(oid, {"pin": None})
        a.update({"next_begin": self.clock_to_64(p["clock"]),
                  "sample_ticks": p["sample_ticks"],
                  "sample_count": p["sample_count"],
                  "rest_ticks": p["rest_ticks"],
                  "bytes_per_report": p.get("bytes_per_report", 2),
                  "min_value": p["min_value"], "max_value": p["max_value"],
                  "range_check_count": p["range_check_count"],
                  "invalid": 0, "batch": [],
                  "active": bool(p["sample_count"] and p["rest_ticks"])})
        self.adc_schedule(a)

    def adc_schedule(self, a):
        # The report goes out once the last of sample_count samples has
        # been read (analog_in_event then analog_in_task).
        a["report_at"] = a["next_begin"] \
            + max(0, a["sample_count"] - 1) * a["sample_ticks"]

    def adc_event(self, oid, clock):
        a = self.adcs[oid]
        sample = self.adc_value(oid, clock / float(self.freq))
        summed = min(0xFFFF, max(0, int(round(sample * a["sample_count"]))))
        if a["min_value"] <= summed <= a["max_value"]:
            a["invalid"] = 0
        else:
            a["invalid"] += 1
            if a["invalid"] >= a["range_check_count"]:
                self.do_shutdown("ADC out of range")
                return
        a["next_begin"] += a["rest_ticks"]
        self.adc_schedule(a)
        nxt = a["next_begin"] & MASK32
        if "values=" in self.fmt_by_name.get("analog_in_state", ""):
            a["batch"].append(summed)
            per_msg = max(1, (a["bytes_per_report"] or 2) // 2)
            if len(a["batch"]) >= per_msg:
                packed = b"".join(bytes([v & 0xFF, (v >> 8) & 0xFF])
                                  for v in a["batch"][:per_msg])
                self.send_by_name("analog_in_state", [oid, nxt, packed])
                a["batch"] = []
        else:
            self.send_by_name("analog_in_state", [oid, nxt, summed])

    # GPIO/PWM (gpiocmds.c, pwmcmds.c)
    def cmd_config_digital_out(self, p):
        self.gpios[p["oid"]] = {"pin": p["pin"], "value": p["value"],
                                "default": p["default_value"],
                                "max_duration": p["max_duration"],
                                "cycle_ticks": 0}
        self.pwm_pin[p["oid"]] = p["pin"]

    def cmd_config_pwm_out(self, p):
        oid = p["oid"]
        self.gpios[oid] = {"pin": p["pin"], "value": 0, "default": p.get("default_value", 0),
                           "max_duration": p.get("max_duration", 0),
                           "cycle_ticks": p.get("cycle_ticks", 0)}
        self.pwm_pin[oid] = p["pin"]
        # value is PWM units; duty normalized later.
        self.pwm_duty[oid] = p.get("value", 0) / 32768.0

    def cmd_set_digital_out_pwm_cycle(self, p):
        g = self.gpios.get(p["oid"])
        if g is None:
            return
        g["cycle_ticks"] = p["cycle_ticks"]

    def cmd_queue_digital_out(self, p):
        oid = p["oid"]
        g = self.gpios.setdefault(oid, {"cycle_ticks": 0})
        g["value"] = 1 if p["on_ticks"] else 0
        cycle = g.get("cycle_ticks", 0)
        if cycle:
            self.pwm_duty[oid] = p["on_ticks"] / float(cycle)
        else:
            self.pwm_duty[oid] = 1.0 if p["on_ticks"] else 0.0
        self.bltouch_control(oid, self.clock_to_64(p["clock"]), p["on_ticks"])

    # BLTouch probe (script "bltouch"). Control commands are software
    # PWM pulses on the control pin; the sim decodes each nonzero pulse
    # width at its scheduled clock. The sensor pin reads, at any clock:
    # raised -> 0, raised after touch_mode -> 1 (the connection self
    # test), lowered -> 0 until the linked Z stepper has travelled the
    # descent's contact_steps since endstop_home, then 1.
    def bltouch_control(self, oid, clock, on_ticks):
        bl = self.bltouch
        if bl is None or not on_ticks:
            return
        if self.pin_name(self.gpios[oid].get("pin")) != bl["control_pin"]:
            return
        width = on_ticks / float(self.freq)
        cmd = min(sorted(BLTOUCH_COMMANDS),
                  key=lambda c: abs(BLTOUCH_COMMANDS[c] - width))
        if abs(BLTOUCH_COMMANDS[cmd] - width) > BLTOUCH_PULSE_TOLERANCE:
            raise ValueError("bltouch pulse of %d ticks matches no command"
                             % on_ticks)
        bl["events"].append((clock, cmd))
        bl["events"].sort(key=lambda ev: ev[0])
        self.trace("rx", "bltouch %s at clock %d" % (cmd, clock & MASK32))

    def bltouch_state(self, clock):
        state = {"lowered": False, "touch": False}
        for at, cmd in self.bltouch["events"]:
            if at > clock:
                break
            if cmd == "pin_down":
                state = {"lowered": True, "touch": False}
            elif cmd == "touch_mode":
                state["touch"] = True
            elif cmd in ("pin_up", "reset", "self_test"):
                state = {"lowered": False, "touch": False}
        return state

    # SPI / I2C / tmcuart (spicmds.c, i2ccmds.c, tmcuart.c)
    def cmd_config_spi(self, p):
        self.spi[p["oid"]] = {"pin": p["pin"], "bus": None}

    def cmd_config_spi_without_cs(self, p):
        self.spi[p["oid"]] = {"pin": None, "bus": None}

    def cmd_spi_set_bus(self, p):
        st = self.spi.setdefault(p["oid"], {})
        if p["mode"] > 3 or st.get("bus") is not None:
            self.do_shutdown("Invalid spi config")
            return
        st["bus"] = (p["spi_bus"], p["mode"], p["rate"])

    def script_blob_reply(self, section, key, length):
        table = self.script.get(section, {})
        rows = table.get(key)
        if rows is None:
            return bytes([0] * length)
        if isinstance(rows, (list, tuple)):
            # Sequence of replies: cycle by index stored per key.
            idx = self.script.get("_idx_" + section + key, 0) if False else None
            # Per-sim state kept in self caches dicts.
            cache = getattr(self, section + "_cache", None)
            if cache is None:
                setattr(self, section + "_cache", {})
                cache = getattr(self, section + "_cache")
            n = cache.get(key, 0)
            cache[key] = n + 1
            row = rows[n % len(rows)]
        else:
            row = rows
        if isinstance(row, str):
            return bytes.fromhex(row)
        if isinstance(row, int):
            return bytes([row & 0xFF] * length)
        return bytes(row)

    def cmd_spi_transfer(self, p):
        data = p.get("data", b"")
        reply = self.script_blob_reply("spi_replies", data.hex(), len(data))
        self.send_by_name("spi_transfer_response", [p["oid"], reply])

    def cmd_spi_send(self, p):
        return

    def cmd_config_spi_shutdown(self, p):
        return

    def cmd_config_i2c(self, p):
        self.i2c[p["oid"]] = {}

    def cmd_i2c_set_bus(self, p):
        st = self.i2c.setdefault(p["oid"], {})
        st["bus"] = (p["i2c_bus"], p["rate"], p["address"])

    def cmd_i2c_transfer(self, p):
        write = p.get("write", b"")
        read_len = p.get("read_len", 0)
        reply = self.script_blob_reply("i2c_replies", write.hex(), read_len)
        # Firmware sends i2c_transfer_response when present in dict.
        if "i2c_transfer_response" in self.fmt_by_name:
            self.send_by_name("i2c_transfer_response", [p["oid"], reply])
        elif "i2c_response" in self.fmt_by_name:
            self.send_by_name("i2c_response", [p["oid"], reply])
        elif "i2c_result" in self.fmt_by_name:
            self.send_by_name("i2c_result", [p["oid"], reply])

    def cmd_config_tmcuart(self, p):
        self.tmcuart[p["oid"]] = {"rx": p["rx_pin"], "tx": p["tx_pin"]}

    def cmd_tmcuart_send(self, p):
        write = p.get("write", b"")
        read_len = p.get("read", 0)
        reply = self.script_blob_reply("tmcuart_replies", write.hex(), read_len)
        if reply == bytes([0] * read_len) and len(write) >= 3:
            # No scripted bytes: emulate TMC2208 register traffic. Decode
            # the serial start/stop bit stream back to data bytes; the
            # register index sits in the third byte.
            try:
                bits = 0
                nbits = 0
                for b in bytes(write):
                    bits |= b << nbits
                    nbits += 8
                raw = bytearray()
                for i in range(len(write)):
                    raw.append((bits >> (i * 10 + 1)) & 0xFF)
                if len(raw) >= 3:
                    reg = raw[2] & 0x7F
                    is_write = bool(raw[2] & 0x80)
                    if is_write:
                        self.tmcuart_idx[reg] = self.tmcuart_idx.get(reg, 0) + 1
                        self.tmcuart_idx[0x02] = self.tmcuart_idx.get(0x02, 0) + 1
                    elif read_len == 10:
                        val = self.tmc_reg_value(reg)
                        enc = self.tmc_encode_read_reply(reg, val)
                        if len(enc) == read_len:
                            reply = enc
            except Exception:
                pass
        self.send_by_name("tmcuart_response", [p["oid"], reply])

    def tmc_reg_value(self, reg):
        # Deterministic TMC2208 values: IFCNT counts successful writes so
        # the host write-verify handshake passes; status registers read
        # clean (no overtemp, no short, cs_actual nonzero).
        regs = self.script.get("tmc_registers", {})
        if str(reg) in regs:
            return int(regs[str(reg)])
        n = self.tmcuart_idx.get(reg, 0)
        if reg == 0x02:
            return n & 0xFF
        if reg == 0x6F:
            # DRV_STATUS: cs_actual=8 (motor current flowing), no error
            # or warning flags (otpw/t120/t143/t150/t157 clear) so the
            # host error check passes deterministically.
            return 0x00080000
        if reg == 0x01:
            return 0x00000000
        if reg == 0x00:
            return 0x00000100
        return 0x00000000


    def tmc_encode_read_reply(self, reg, val):
        # Mirror of klippy/extras/tmc_uart.py _encode_write with sync 0x05
        # and address 0xff, then serial start/stop bit expansion.
        msg = bytearray([0x05, 0xFF, reg & 0x7F,
                         (val >> 24) & 0xFF, (val >> 16) & 0xFF,
                         (val >> 8) & 0xFF, val & 0xFF])
        crc = 0
        for b in msg:
            for i in range(8):
                if (crc >> 7) ^ (b & 0x01):
                    crc = (crc << 1) ^ 0x07
                else:
                    crc = (crc << 1)
                crc &= 0xFF
                b >>= 1
        msg.append(crc)
        out = 0
        pos = 0
        for d in msg:
            b = (d << 1) | 0x200
            out |= (b << pos)
            pos += 10
        res = bytearray()
        for i in range((pos + 7) // 8):
            res.append((out >> (i * 8)) & 0xFF)
        return bytes(res)

    # Buttons (src/buttons.c): pins are sampled every rest_ticks; a
    # level that differs from the query's invert mask on two samples
    # is reported, then retransmitted every retransmit_count+1 samples
    # until acknowledged. Scripted pin levels are static, so the only
    # report is the difference seen at buttons_query.
    def button_state(self, oid):
        return self.buttons.setdefault(oid, {"count": 0, "pins": {}, "ack": 0,
                                             "reports": [], "send_at": None})

    def cmd_config_buttons(self, p):
        b = self.button_state(p["oid"])
        b["count"] = p["button_count"]

    def cmd_buttons_add(self, p):
        self.button_state(p["oid"])["pins"][p["pos"]] = (p["pin"], p["pull_up"])

    def cmd_buttons_query(self, p):
        b = self.button_state(p["oid"])
        b.update({"rest_ticks": p["rest_ticks"],
                  "retransmit": p["retransmit_count"], "ack": 0,
                  "reports": [], "send_at": None})
        if not p["rest_ticks"]:
            return
        status = 0
        for pos, (pin, pull_up) in b["pins"].items():
            if self.static_pin_level(pin, pull_up):
                status |= 1 << pos
        if status != (p["invert"] & 0xFF):
            b["reports"] = [status]
            b["send_at"] = self.clock_to_64(p["clock"]) + p["rest_ticks"]

    def buttons_event(self, oid, clock):
        b = self.buttons[oid]
        self.send_by_name("buttons_state", [oid, b["ack"], bytes(b["reports"])])
        b["send_at"] = clock + (b["retransmit"] + 1) * b["rest_ticks"]

    def cmd_buttons_ack(self, p):
        b = self.button_state(p["oid"])
        count = p["count"]
        b["ack"] = (b["ack"] + count) & 0xFF
        if count >= len(b["reports"]):
            b["reports"] = []
            b["send_at"] = None
        else:
            b["reports"] = b["reports"][count:]

    # Neopixel (neopixel.c)
    def cmd_config_neopixel(self, p):
        self.neopixels[p["oid"]] = {"size": p["data_size"]}

    def cmd_neopixel_update(self, p):
        st = self.neopixels.setdefault(p["oid"], {"size": 0, "data": {}})
        st["data"][p["pos"]] = p["data"]

    def cmd_neopixel_send(self, p):
        self.send_by_name("neopixel_result", [p["oid"], 1])

    # Bulk / misc sensors: scripted chunks (sensor_bulk.c et al.)
    def cmd_config_adxl345(self, p):
        self.bulk[p["oid"]] = {"kind": "adxl345", "seq": 0}

    def cmd_query_adxl345(self, p):
        self.respond_bulk(p["oid"], "adxl345")

    def cmd_config_bmi160(self, p):
        self.bulk[p["oid"]] = {"kind": "bmi160", "seq": 0}

    def cmd_query_bmi160(self, p):
        self.respond_bulk(p["oid"], "bmi160")

    def cmd_config_lis2dw(self, p):
        self.bulk[p["oid"]] = {"kind": "lis2dw", "seq": 0}

    def cmd_query_lis2dw(self, p):
        self.respond_bulk(p["oid"], "lis2dw")

    def cmd_config_ldc1612(self, p):
        self.bulk[p["oid"]] = {"kind": "ldc1612", "seq": 0}

    def cmd_config_ldc1612_with_intb(self, p):
        self.bulk[p["oid"]] = {"kind": "ldc1612", "seq": 0}

    def cmd_config_hx71x(self, p):
        self.bulk[p["oid"]] = {"kind": "hx71x", "seq": 0}

    def cmd_config_mpu9250(self, p):
        self.bulk[p["oid"]] = {"kind": "mpu9250", "seq": 0}

    def cmd_config_icm20948(self, p):
        self.bulk[p["oid"]] = {"kind": "icm20948", "seq": 0}

    def cmd_query_thermocouple(self, p):
        cfg = self.script.get("thermocouple", {}).get(str(p["oid"]), {})
        self.send_by_name("thermocouple_result",
                          [p["oid"], self.now32(),
                           int(cfg.get("value", 0)), int(cfg.get("fault", 0))])

    def cmd_query_counter(self, p):
        cfg = self.script.get("counter", {}).get(str(p["oid"]), {})
        self.send_by_name("counter_state",
                          [p["oid"], self.now32(),
                           int(cfg.get("count", 0)), self.now32()])

    def respond_bulk(self, oid, kind):
        status = self.script.get("sensor_bulk", {}).get(str(oid), {})
        st = self.bulk.setdefault(oid, {"kind": kind, "seq": 0})
        chunks = status.get("chunks", [])
        seq = st.get("seq", 0)
        if kind == "adxl345" and "adxl345_status" in self.fmt_by_name:
            nxt = (seq + 1) & 0xFFFF
            self.send_by_name("adxl345_status",
                              [oid, self.now32(), 0, nxt, len(chunks) - seq > 0 and 1 or 0, 0, 0])
            for chunk in chunks[seq:seq + 1]:
                if isinstance(chunk, dict):
                    data = chunk.get("data", "")
                    if isinstance(data, str):
                        data = bytes.fromhex(data)
                    self.send_by_name("adxl345_data", [oid, seq, data])
            st["seq"] = seq + 1
        else:
            # Generic bulk status where the dict provides one.
            name = "sensor_bulk_status"
            if name in self.fmt_by_name:
                self.send_by_name(name, [oid, self.now32(), 0, seq & 0xFFFF, 0, 0])
            st["seq"] = seq + 1

    # Debug/misc passthroughs
    def cmd_query_fancheck(self, p):
        self.send_by_name("fan_status", [p["oid"], 0, 0, 0, 0, 0])

    # -- timer events -------------------------------------------------
    # Every firmware timer the sim models becomes an event with an
    # absolute 64-bit clock. The main loop sleeps (virtual time) until
    # the earliest one and run_until() processes them in clock order.
    def pending_events(self):
        evs = []
        for oid, e in self.endstops.items():
            if not e.get("active"):
                continue
            if not e["armed"]:
                evs.append((e["waketime"], -1, oid, "arm", None))
                continue
            pred = self.endstop_predict(e)
            if pred is not None:
                evs.append((pred[0], 0, oid, "endstop", pred[1]))
        for oid, ts in self.trsyncs.items():
            if ts["can"] and ts["expire"] is not None:
                evs.append((ts["expire"], 1, oid, "expire", None))
            if ts["report_time"] is not None:
                evs.append((ts["report_time"], 2, oid, "report", None))
        for oid, a in self.adcs.items():
            if a.get("active"):
                evs.append((a["report_at"], 3, oid, "adc", None))
        for oid, b in self.buttons.items():
            if b.get("send_at") is not None:
                evs.append((b["send_at"], 4, oid, "buttons", None))
        if self.forced_shutdown_at is not None and not self.forced_done:
            evs.append((int(round(self.forced_shutdown_at * self.freq)),
                        5, 0, "shutdown", None))
        return evs

    def next_event_clock(self):
        evs = self.pending_events()
        return min(evs)[0] if evs else None

    def run_until(self, target):
        while True:
            evs = self.pending_events()
            if not evs:
                break
            ev = min(evs)
            if ev[0] > target:
                break
            self.advance_steppers(ev[0])
            self.process_event(ev)
        self.advance_steppers(target)

    def process_event(self, ev):
        clock, _order, oid, kind, extra = ev
        if kind == "arm":
            self.endstop_arm(self.endstops[oid], clock)
        elif kind == "endstop":
            e = self.endstops[oid]
            e["active"] = False
            e["nextwake"] = extra & MASK32
            self.trsync_do_trigger(e["trsync_oid"], e["reason"], clock)
        elif kind == "expire":
            ts = self.trsyncs[oid]
            ts["expire"] = None
            self.trsync_do_trigger(oid, ts["expire_reason"], clock)
        elif kind == "report":
            ts = self.trsyncs[oid]
            self.send_by_name("trsync_state",
                              [oid, 1 if ts["can"] else 0, ts["reason"],
                               clock & MASK32])
            ts["report_time"] += ts["report_ticks"]
        elif kind == "adc":
            self.adc_event(oid, clock)
        elif kind == "buttons":
            self.buttons_event(oid, clock)
        elif kind == "shutdown":
            self.forced_done = True
            self.do_shutdown("Command request")


def open_pty_symlink(path):
    master, slave = pty.openpty()
    # Raw line discipline, like src/linux/console.c memset-zeroed
    # termios: the pty shares one termios struct, so a canonical slave
    # would echo the sim's own frames back into its read path (ack
    # storm) and mangle binary with ONLCR/ICANON. tty.setraw clears
    # ECHO, ICANON, ISIG, OPOST and input translations on both ends.
    try:
        tty.setraw(master)
    except termios.error:
        pass
    slave_name = os.ttyname(slave)
    if path and os.path.abspath(path) != os.path.abspath(slave_name):
        try:
            if os.path.islink(path) or os.path.exists(path):
                os.unlink(path)
        except OSError:
            pass
        try:
            os.symlink(slave_name, path)
        except OSError:
            pass
    return master, slave, slave_name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dict", required=True)
    ap.add_argument("--name", default="mcu")
    ap.add_argument("--pty", default=None)
    ap.add_argument("--script", default=None)
    ap.add_argument("--trace", default=None)
    ap.add_argument("--capture", default=None)
    args = ap.parse_args()
    with open(args.dict, "rb") as f:
        dict_bytes = f.read()
    script = {}
    if args.script:
        with open(args.script) as f:
            script = json.load(f)
    master, slave, slave_name = open_pty_symlink(args.pty) if args.pty \
        else (None, None, None)
    # When no pty requested, use stdin/stdout for tests.
    sim = Sim(dict_bytes, args.name, script, args.trace)
    if args.capture:
        sim.cap_f = open(args.capture, "wb")

    def on_signal(signum, frame):
        # Flush and leave at once: raising SystemExit here can land in
        # interpreter shutdown and print "Exception ignored" noise.
        for f in (sim.trace_f, sim.cap_f):
            if f is not None:
                f.flush()
        os._exit(0)

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    fd = master if master is not None else sys.stdin.buffer.fileno()
    sim.fd = fd
    evf = sim.trace_f
    sim.send_by_name("starting", [])

    alive = True
    while alive:
        # Sleep (virtual time) until the next firmware timer event, but
        # wake at least every 0.5 s. The timeout is rounded up to whole
        # microseconds so the wake never lands before the event clock.
        timeout = 0.5
        nxt = sim.next_event_clock()
        if nxt is not None:
            ticks = max(0, nxt - sim.now64())
            timeout = min(timeout, math.ceil(ticks * 1e6 / sim.freq) / 1e6)
        try:
            r, _, _ = select.select([fd], [], [], timeout)
        except (OSError, select.error):
            break
        if r:
            try:
                data = os.read(fd, 4096)
            except OSError:
                break
            if not data:
                break
            if slave is not None:
                # The host has the slave open now. Drop our own slave fd
                # so the read above fails (EIO) once the host exits: the
                # sim then stops at that virtual time instead of running
                # on until the harness terminates it in wall time.
                os.close(slave)
                slave = None
            sim.capture_transport("rx", data)
            sim.inbuf.extend(data)
        # Timer events due before these commands arrived run first.
        sim.run_until(sim.now64())
        # Process all complete blocks in the buffer.
        while True:
            parsed = sim_extract(sim)
            if parsed is None:
                break
            action, items, pop = parsed
            del sim.inbuf[:pop]
            if action == "block":
                for cmd in items:
                    cmd_name, params = cmd
                    if sim.shutdown and cmd_name not in NON_SHUTDOWN_ALLOWED:
                        sim.report_is_shutdown()
                        continue
                    sim.trace("rx", format_command(sim.fmt_by_name.get(cmd_name, cmd_name), params))
                    sim.handle(cmd_name, params)
                sim.send_ack()
            elif action == "nak":
                sim.send_ack()
        sim.run_until(sim.now64())
    if evf:
        try:
            evf.flush()
        except Exception:
            pass
    return 0


def sim_extract(sim):
    buf = sim.inbuf
    if not buf:
        return None
    # Skip leading sync bytes; resync otherwise.
    if buf[0] == MESSAGE_SYNC:
        return ("skip", [], 1)
    if buf[0] == MESSAGE_SYNC and False:
        pass
    if len(buf) < MESSAGE_MIN:
        return None
    msglen = buf[0]
    if msglen < MESSAGE_MIN or msglen > MESSAGE_MAX:
        return sim_resync(sim)
    seq = buf[1]
    if (seq & ~MESSAGE_SEQ_MASK) != MESSAGE_DEST:
        return sim_resync(sim)
    if len(buf) < msglen:
        return None
    if buf[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC:
        return sim_resync(sim)
    msgcrc = (buf[msglen - MESSAGE_TRAILER_CRC] << 8) | buf[msglen - MESSAGE_TRAILER_CRC + 1]
    crc = crc16_ccitt(bytes(buf[:msglen - MESSAGE_TRAILER_SIZE]))
    if crc != msgcrc:
        return sim_resync(sim)
    # Valid CRC: decode payload commands.
    payload = bytes(buf[MESSAGE_HEADER_SIZE:msglen - MESSAGE_TRAILER_SIZE])
    cmds = []
    pos = 0
    try:
        while pos < len(payload):
            msgid, pos = decode_msgid(payload, pos)
            fmt = sim.fmt_by_id.get(msgid)
            if fmt is None:
                # Unknown command id: treat as invalid -> nak.
                return ("nak", [], msglen)
            params, pos = decode_params(fmt, payload, pos)
            name = fmt.split()[0]
            cmds.append((name, params))
    except Exception:
        return ("nak", [], msglen)
    # Sequence check like command_find_block: mismatch -> nak, which the
    # host uses to re-base its send_seq ("connection init"). Match
    # advances the shared counter, exactly like the firmware.
    if seq != sim.next_sequence:
        return ("nak", [], msglen)
    sim.next_sequence = ((seq + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST
    return ("block", cmds, msglen)


def sim_resync(sim):
    buf = sim.inbuf
    try:
        idx = bytes(buf).index(MESSAGE_SYNC)
    except ValueError:
        return ("nak", [], len(buf))
    # Discard up to and including sync marker (command.c error path).
    return ("nak", [], idx + 1)


if __name__ == "__main__":
    sys.exit(main())
