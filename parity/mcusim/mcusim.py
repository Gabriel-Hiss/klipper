#!/usr/bin/env python3
"""Generic Klipper MCU simulator (parity/mcusim/mcusim.py).

Self-contained, stdlib only, Python 3.8 compatible. Runs under the
virtual clock (time.monotonic() is virtualized by parity/vclock when
LD_PRELOAD is active); MCU clock = monotonic * CLOCK_FREQ.

CLI:
    python3 parity/mcusim/mcusim.py --dict DICT --name NAME --pty PATH
        [--script SCRIPT.json] [--trace FILE]

Creates a pty and symlinks it at PATH (like the Linux MCU firmware
src/linux/console.c), serves the identify data block, implements the
block transport of src/command.c / klippy/chelper/msgblock.c, and
mirrors firmware semantics of src/*.c. See SCRIPT.md for the script
format. Trace: one JSON line per received command and sent response.
"""
import argparse
import json
import math
import os
import pty
import select
import signal
import sys
import time
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
        self.next_seq = MESSAGE_DEST
        self.recv_next_seq = MESSAGE_DEST
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
        self.pin_values = {}   # pin number -> 0/1 (input pins)
        self.stepper_pos = {}  # oid -> position (int with bias)
        self.stop_links = {}   # trsync_oid -> list of stepper oids
        self.clock0 = time.monotonic()
        self.thermal = {}      # adc oid -> thermal model state
        self.pwm_duty = {}     # digital_out oid -> duty 0..1
        self.pwm_pin = {}      # digital_out oid -> pin
        self.last_thermal_t = time.monotonic()
        self.seq_counters = {}  # bulk sensor oid -> next sequence
        self.spi_reply_idx = {}
        self.tmcuart_idx = {}
        self.i2c_idx = {}
        self.forced_shutdown_at = self.script.get("forced_shutdown_at")
        self.forced_done = False

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

    # -- send -------------------------------------------------------
    def send_msg(self, fmt, values):
        values = list(values)
        msgid = self.id_by_format[fmt]
        payload = encode_msgid(msgid) + encode_params(fmt, values)
        self.send_block(payload)
        if self.trace_f:
            p = self.fmt_by_name.get(fmt.split()[0], fmt)
            self.trace("tx", p + " " + fmt)

    def send_block(self, payload):
        if len(payload) + MESSAGE_MIN > MESSAGE_MAX:
            raise ValueError("message too large")
        msglen = MESSAGE_MIN + len(payload)
        out = [msglen, self.next_seq] + list(payload)
        crc = crc16_ccitt(out)
        out.append((crc >> 8) & 0xFF)
        out.append(crc & 0xFF)
        out.append(MESSAGE_SYNC)
        self.next_seq = ((self.next_seq + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST
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
        self.send_by_name("shutdown", [self.now32(), sid])

    def report_is_shutdown(self):
        self.send_by_name("is_shutdown", [self.shutdown_reason])

    # -- command handlers -------------------------------------------
    def handle(self, cmd_name, params):
        fn = getattr(self, "cmd_" + cmd_name, None)
        if fn is not None:
            try:
                fn(params)
                return
            except Exception as exc:
                self.trace("tx", "error " + cmd_name + ": " + repr(exc))
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

    # steppers (stepper.c)
    def stepper_state(self, oid):
        st = self.steppers.get(oid)
        if st is None:
            st = {"queue": [], "cur": None, "pos": -0x40000000,
                  "next_dir": False, "last_dir": False,
                  "stop_trsync": None, "start_pos": -0x40000000}
            self.steppers[oid] = st
        return st

    def cmd_config_stepper(self, p):
        st = self.stepper_state(p["oid"])
        st["step_pin"] = p["step_pin"]
        st["dir_pin"] = p["dir_pin"]
        st["step_pulse_ticks"] = p.get("step_pulse_ticks", 0)

    def cmd_queue_step(self, p):
        st = self.stepper_state(p["oid"])
        if not p["count"]:
            self.do_shutdown("Invalid count parameter")
            return
        need_dir = st["next_dir"] != st["last_dir"]
        st["last_dir"] = st["next_dir"]
        st["queue"].append({"interval": p["interval"], "count": p["count"],
                            "add": p["add"], "dir": need_dir})
        if st["cur"] is None:
            self.start_next_move(st)

    def start_next_move(self, st):
        if not st["queue"]:
            st["cur"] = None
            return
        m = st["queue"].pop(0)
        if m["dir"]:
            st["pos"] = -st["pos"] if st["pos"] else (-st["pos"] or 0)
            st["pos"] = -st["pos"] if st["pos"] >= 0 else (st["pos"] + 0)
            # Firmware: position = -position + count logic; tracked as
            # signed steps from start_pos. Normalize to bias space:
            st["pos"] = -(st["pos"] - (-0x40000000)) - 0x40000000
            if st["pos"] == 0:
                st["pos"] = -0x40000000
            st["sign"] = -st.get("sign", 1)
        else:
            st.setdefault("sign", 1)
        st["cur"] = {"left": m["count"], "interval": m["interval"],
                     "add": m["add"], "next": self.now64(),
                     "sign": st.get("sign", 1)}

    def cmd_set_next_step_dir(self, p):
        st = self.stepper_state(p["oid"])
        st["next_dir"] = bool(p["dir"])

    def cmd_reset_step_clock(self, p):
        st = self.stepper_state(p["oid"])
        if st["cur"] is not None:
            self.do_shutdown("Can't reset time when stepper active")
            return
        # Reset clock baseline; position unchanged.
        st["clock_base"] = p["clock"]

    def cmd_stepper_get_position(self, p):
        st = self.stepper_state(p["oid"])
        self.advance_steppers()
        pos = st["pos"] & 0xFFFFFFFF
        if pos >= (1 << 31):
            pos -= (1 << 32)
        self.send_by_name("stepper_position", [p["oid"], pos])

    def cmd_stepper_stop_on_trigger(self, p):
        st = self.stepper_state(p["oid"])
        st["stop_trsync"] = p["trsync_oid"]
        self.stop_links.setdefault(p["trsync_oid"], []).append(p["oid"])

    def advance_steppers(self):
        now = self.now64()
        for st in self.steppers.values():
            while st["cur"] is not None and st["cur"]["next"] <= now:
                cur = st["cur"]
                cur["left"] -= 1
                st["pos"] += cur["sign"]
                nxt = cur["next"] + cur["interval"]
                cur["interval"] += cur["add"]
                cur["next"] = nxt
                if cur["left"] <= 0:
                    self.start_next_move(st)
                    if st["cur"] is None:
                        break
                else:
                    break

    def stepper_steps_since(self, oid, base):
        st = self.steppers.get(oid)
        if st is None:
            return 0
        return st["pos"] - base

    # endstops / trsync (endstop.c, trsync.c)
    def cmd_config_endstop(self, p):
        self.endstops[p["oid"]] = {"pin": p["pin"], "pull_up": p["pull_up"],
                                   "homing": False, "nextwake": 0}

    def cmd_endstop_home(self, p):
        if not p["sample_count"]:
            e = self.endstops.get(p["oid"], {})
            e["homing"] = False
            return
        e = self.endstops.setdefault(p["oid"], {"pin": 0, "pull_up": 0})
        oid = p["oid"]
        tr = self.script.get("endstop_triggers", {}).get(str(oid), {})
        e.update({"homing": True, "clock": self.clock_to_64(p["clock"]),
                  "sample_ticks": p["sample_ticks"],
                  "sample_count": p["sample_count"],
                  "rest_ticks": p["rest_ticks"],
                  "pin_value": p["pin_value"],
                  "trsync_oid": p["trsync_oid"],
                  "trigger_reason": p["trigger_reason"],
                  "trigger_count": p["sample_count"],
                  "mode": "sample",
                  "next": self.clock_to_64(p["clock"])})
        # Record stepper baseline for positional trigger scripts.
        so = tr.get("stepper_oid")
        if so is not None:
            st = self.steppers.get(so)
            e["base_pos"] = st["pos"] if st else 0
            e["at_steps"] = tr.get("at_steps", 0)

    def cmd_endstop_query_state(self, p):
        e = self.endstops.get(p["oid"], {"homing": False, "nextwake": 0, "pin": 0})
        pin = self.pin_value(e)
        self.send_by_name("endstop_state",
                          [p["oid"], 1 if e.get("homing") else 0,
                           e.get("nextwake", 0) & 0xFFFFFFFF, pin])

    def pin_value(self, e):
        pin = e.get("pin", 0)
        scripted = self.script.get("endstop_pin_values", {})
        key = str(pin)
        if key in scripted:
            return int(scripted[key])
        # Positional trigger: pin asserts once stepper passes at_steps.
        tr = None
        for oid_s, cfg in self.script.get("endstop_triggers", {}).items():
            if cfg.get("pin") == pin:
                tr = cfg
                break
        if tr and "stepper_oid" in tr:
            st = self.steppers.get(tr["stepper_oid"])
            base = e.get("base_pos", None)
            if st is not None and base is not None:
                if st["pos"] - base >= tr.get("at_steps", 0):
                    return int(tr.get("pin_value", 1))
                return 1 - int(tr.get("pin_value", 1))
        return self.pin_values.get(pin, 0)

    def tick_endstops(self):
        now = self.now64()
        for oid, e in list(self.endstops.items()):
            if not e.get("homing"):
                continue
            if now < e.get("next", 0):
                continue
            val = self.pin_value(e)
            want = e.get("pin_value", 1)
            if e.get("mode") == "sample":
                if val != want:
                    e["next"] = now + e["rest_ticks"]
                    continue
                e["mode"] = "oversample"
                e["trigger_count"] = e["sample_count"]
                e["next"] = now
                e["nextwake"] = e["next"] & 0xFFFFFFFF
            # oversample mode
            if val != want:
                e["mode"] = "sample"
                e["nextwake"] = e["next"] & 0xFFFFFFFF
                nxt = e["next"] + e["rest_ticks"]
                e["next"] = nxt
                continue
            cnt = e.get("trigger_count", 1) - 1
            if cnt <= 0:
                e["homing"] = False
                self.do_trigger(e.get("trsync_oid", 0),
                                e.get("trigger_reason", 1))
            else:
                e["trigger_count"] = cnt
                e["next"] = e["next"] + e["sample_ticks"]

    def cmd_config_trsync(self, p):
        self.trsyncs[p["oid"]] = {"can": False, "report_ticks": 0,
                                  "next_report": 0, "expire": None,
                                  "expire_reason": 0, "reason": 0,
                                  "pending": False}

    def do_trigger(self, ts_oid, reason):
        ts = self.trsyncs.get(ts_oid)
        if ts is None or not ts.get("can"):
            return
        ts["can"] = False
        ts["reason"] = reason
        ts["pending"] = True
        ts["expire"] = None
        # Stop linked steppers (stepper.c stop on trsync trigger).
        for so in self.stop_links.get(ts_oid, []):
            st = self.steppers.get(so)
            if st is not None:
                st["queue"].clear()
                st["cur"] = None

    def cmd_trsync_start(self, p):
        ts = self.trsyncs.setdefault(p["oid"], {"can": False})
        ts.update({"can": True, "next_report": self.clock_to_64(p["report_clock"]),
                   "report_ticks": p["report_ticks"],
                   "expire_reason": p["expire_reason"], "reason": 0,
                   "pending": False, "expire": None})
        if ts["report_ticks"]:
            ts["next_report"] = self.clock_to_64(p["report_clock"])

    def cmd_trsync_set_timeout(self, p):
        ts = self.trsyncs.get(p["oid"])
        if ts and ts.get("can"):
            ts["expire"] = self.clock_to_64(p["clock"])

    def cmd_trsync_trigger(self, p):
        oid = p["oid"]
        self.do_trigger(oid, p["reason"])
        ts = self.trsyncs.get(oid, {"reason": p["reason"]})
        ts["can"] = False
        ts["next_report"] = 0
        ts["expire"] = None
        self.send_by_name("trsync_state", [oid, 0, ts.get("reason", p["reason"]), 0])
        ts["pending"] = False

    def tick_trsync(self):
        now = self.now64()
        for oid, ts in list(self.trsyncs.items()):
            if ts.get("expire") is not None and ts.get("can") and now >= ts["expire"]:
                self.do_trigger(oid, ts.get("expire_reason", 1))
            if ts.get("pending"):
                ts["pending"] = False
                self.send_by_name("trsync_state",
                                  [oid, 1 if ts.get("can") else 0,
                                   ts.get("reason", 0), self.now32()])
            elif ts.get("report_ticks") and ts.get("next_report") and now >= ts["next_report"]:
                ts["next_report"] += ts["report_ticks"]
                self.send_by_name("trsync_state",
                                  [oid, 1 if ts.get("can") else 0,
                                   ts.get("reason", 0), self.now32()])

    # ADC / thermal (adccmds.c); thermal coefficients documented in SCRIPT.md
    def cmd_config_analog_in(self, p):
        self.adcs[p["oid"]] = {"pin": p["pin"], "reports": [], "batch": []}

    def adc_script_value(self, oid, t):
        cfg = self.script.get("adc", {}).get(str(oid), {})
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
        if mode == "constant":
            return float(cfg.get("value", 0))
        return 0.0

    def thermal_init(self, oid):
        cfg = self.script.get("thermal", {}).get(str(oid))
        if cfg is None:
            return
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
        if str(oid) in self.script.get("thermal", {}):
            self.thermal_init(oid)
            self.thermal_update(t)
            return self.thermal[oid]["value"]
        if str(oid) in self.script.get("adc", {}):
            return self.adc_script_value(oid, t)
        return 0.0

    def cmd_query_analog_in(self, p):
        oid = p["oid"]
        st = self.adcs.setdefault(oid, {"pin": 0})
        fmt_cmd = self.fmt_by_name.get("query_analog_in", "")
        has_bpr = fmt_cmd and "bytes_per_report" in fmt_cmd
        st.update({"clock64": self.clock_to_64(p["clock"]),
                   "sample_ticks": p["sample_ticks"],
                   "sample_count": p["sample_count"],
                   "rest_ticks": p["rest_ticks"],
                   "bytes_per_report": p.get("bytes_per_report", 2) if has_bpr else 2,
                   "min_value": p["min_value"], "max_value": p["max_value"],
                   "range_check_count": p["range_check_count"],
                   "invalid": 0, "batch": [], "active": True,
                   "next_begin": self.clock_to_64(p["clock"])})
        if not p["sample_count"]:
            st["active"] = False

    def tick_adc(self):
        now = self.now64()
        t = self.virt_t()
        for oid, st in list(self.adcs.items()):
            if not st.get("active"):
                continue
            if now < st.get("next_begin", now):
                continue
            # One report boundary: sample_count samples averaged (sum).
            sample = self.adc_value(oid, t)
            summed = int(round(sample * st["sample_count"]))
            if summed < 0:
                summed = 0
            if summed > 0xFFFF:
                summed = 0xFFFF
            if st["min_value"] <= summed <= st["max_value"]:
                st["invalid"] = 0
            else:
                st["invalid"] += 1
                if st["invalid"] >= st["range_check_count"]:
                    st["invalid"] = 0
                    self.do_shutdown("ADC out of range")
            st["next_begin"] += st["rest_ticks"]
            fmt_resp = self.fmt_by_name.get("analog_in_state", "")
            if "values=" in fmt_resp:
                st["batch"].append(summed)
                per_msg = max(1, (st.get("bytes_per_report", 2) or 2) // 2)
                if len(st["batch"]) >= per_msg:
                    packed = b""
                    for v in st["batch"][:per_msg]:
                        packed += bytes([v & 0xFF, (v >> 8) & 0xFF])
                    nxt = st["next_begin"] & 0xFFFFFFFF
                    self.send_by_name("analog_in_state", [oid, nxt, packed])
                    st["batch"] = []
            else:
                nxt = st["next_begin"] & 0xFFFFFFFF
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
        g = self.gpios.setdefault(p["oid"], {"cycle_ticks": 0})
        g["value"] = 1 if p["on_ticks"] else 0
        oid = p["oid"]
        cycle = g.get("cycle_ticks", 0) or 1
        self.pwm_duty[oid] = (p["on_ticks"] / float(cycle)) if cycle else 1.0

    def cmd_update_digital_out(self, p):
        g = self.gpios.setdefault(p["oid"], {"cycle_ticks": 0})
        g["value"] = p["value"]

    def cmd_set_digital_out(self, p):
        pin, val = p["pin"], p["value"]
        self.pin_values[pin] = val
        for oid, g in self.gpios.items():
            if g.get("pin") == pin:
                g["value"] = val
                self.pwm_duty[oid] = 1.0 if val else 0.0

    def cmd_set_pwm_out(self, p):
        oid = p["oid"]
        if oid in self.gpios:
            self.gpios[oid]["value"] = p.get("value", 0)
            self.pwm_duty[oid] = p.get("value", 0) / 32768.0

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
        self.send_by_name("tmcuart_response", [p["oid"], reply])

    # Buttons (buttons.c)
    def cmd_config_buttons(self, p):
        self.buttons[p["oid"]] = {"count": p["button_count"], "ack": 0,
                                  "pins": {}}

    def cmd_buttons_add(self, p):
        b = self.buttons.setdefault(p["oid"], {"count": 0, "ack": 0, "pins": {}})
        b["pins"][p["pos"]] = (p["pin"], p["pull_up"])

    def cmd_buttons_query(self, p):
        b = self.buttons.setdefault(p["oid"], {"count": 0, "ack": 0, "pins": {}})
        b["next"] = self.clock_to_64(p["clock"])
        b["rest_ticks"] = p["rest_ticks"]
        b["retransmit"] = p["retransmit_count"]
        b["invert"] = p["invert"]
        self.emit_buttons(p["oid"])

    def emit_buttons(self, oid):
        b = self.buttons.get(oid, {"count": 0, "ack": 0, "pins": {}})
        cfg = self.script.get("buttons", {}).get(str(oid), {})
        state = bytes(int(cfg.get(str(i), 0)) for i in range(b.get("count", 0)))
        self.send_by_name("buttons_state", [oid, b.get("ack", 0), state])

    def cmd_buttons_ack(self, p):
        b = self.buttons.setdefault(p["oid"], {"count": 0, "ack": 0})
        b["ack"] = p["count"]

    def tick_buttons(self):
        now = self.now64()
        for oid, b in self.buttons.items():
            if "next" in b and now >= b["next"]:
                self.emit_buttons(oid)
                b["next"] = now + b.get("rest_ticks", 0)

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

    # -- forced shutdown events / future script triggers ------------
    def tick_script(self):
        if self.forced_shutdown_at is not None and not self.forced_done \
                and self.virt_t() >= self.forced_shutdown_at:
            self.forced_done = True
            self.do_shutdown("Command request")

    def tick_all(self):
        self.tick_script()
        self.advance_steppers()
        self.tick_endstops()
        self.tick_trsync()
        self.tick_adc()
        self.tick_buttons()


def open_pty_symlink(path):
    master, slave = pty.openpty()
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

    def on_signal(signum, frame):
        try:
            if master is not None:
                os.close(master)
        except OSError:
            pass
        sys.exit(0)

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    fd = master if master is not None else sys.stdin.buffer.fileno()
    evf = sim.trace_f
    sim.send_by_name("starting", [])
    alive = True
    while alive:
        # Compute select timeout to next periodic event (cap 0.5s).
        timeout = 0.5
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
            sim.inbuf.extend(data)
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
                    sim.trace("rx", cmd_name)
                    sim.handle(cmd_name, params)
                sim.send_ack()
            elif action == "nak":
                sim.send_ack()
            else:
                break
        sim.tick_all()
        if master is None and not r and not sim.inbuf:
            pass
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
    # Sequence check like command_find_block: mismatch -> nak.
    if seq != sim.recv_next_seq:
        return ("nak", [], msglen)
    sim.recv_next_seq = ((seq + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST
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
