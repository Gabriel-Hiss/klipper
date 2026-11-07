# Python unit recorder.
#
# A sys.setprofile tracer configured by unit id to (module, qualified
# names). Records calls with serializable args and results, and
# identities for host objects. Covers the pure units: msgproto, gcode
# line parsing, pins, mathutil, shaper_defs, thermistor, configfile
# parsing, and gcode_macro template rendering.
#
# Enabled by KLIPPY_RECORD_DIR plus KLIPPY_RECORD_UNITS (comma
# separated unit ids). The sitecustomize hook installs the tracer
# before the scenario runs; record.py sets both variables.
import logging
import sys
import threading

import recorder
import units


class _LogCapture(logging.Handler):
    def __init__(self):
        super().__init__()
        self.lines = []

    def emit(self, record):
        try:
            self.lines.append(self.format(record))
        except Exception:
            pass


class UnitTracer:
    """sys.setprofile tracer for a fixed set of units."""

    def __init__(self, unit_ids):
        self.unit_ids = tuple(unit_ids)
        self._targets = {}  # (module, qualname) -> unit id
        self._modules = set()
        for unit_id in self.unit_ids:
            cfg = units.UNIT_MAP[unit_id]
            module = cfg["module"]
            self._modules.add(module)
            for name in cfg["names"]:
                self._targets[(module, name)] = unit_id
        self._stack = []
        self._local = threading.local()
        self._ids = recorder.GLOBAL_IDS
        self._writer = None

    def _thread_stack(self):
        stack = getattr(self._local, "stack", None)
        if stack is None:
            stack = []
            self._local.stack = stack
        return stack

    def _matches(self, frame):
        module = frame.f_globals.get("__name__", "")
        if module not in self._modules:
            return None, None
        code = frame.f_code
        qualname = getattr(code, "co_qualname", None)
        if qualname is None:
            qualname = code.co_name
        unit = self._targets.get((module, qualname))
        if unit is not None:
            return unit, qualname
        # Methods are defined in the class namespace; the qualname
        # from setprofile already includes the class on 3.11+, but on
        # 3.8 (the pad) it is just the function name. Fall back to a
        # name-only match within the module.
        for (mod, name), uid in self._targets.items():
            if mod == module and name == qualname:
                return uid, qualname
        for (mod, name), uid in self._targets.items():
            if mod == module and "." not in name and name == code.co_name:
                return uid, name
        return None, None

    def _profile(self, frame, event, arg):
        if event not in ("call", "return", "exception"):
            return
        match = self._matches(frame)
        if match[0] is None:
            return
        unit, qualname = match
        stack = self._thread_stack()
        if event == "call":
            self._on_call(unit, qualname, frame)
        else:
            self._on_return(unit, frame, event, arg)

    def _on_call(self, unit, qualname, frame):
        writer = recorder.get_writer()
        if writer is None:
            return
        args = self._snapshot_args(frame)
        object_id = None
        code = frame.f_code
        if code.co_argcount > 0 and code.co_varnames:
            first = code.co_varnames[0]
            if first == "self":
                try:
                    object_id = self._ids.stable_id(
                        frame.f_locals["self"])
                except KeyError:
                    object_id = None
        stack = self._thread_stack()
        log_capture = None
        if not stack:
            # Only the outermost traced call captures log lines, so a
            # line is recorded once, on the outer call.
            log_capture = _LogCapture()
            log_capture.setFormatter(logging.Formatter("%(message)s"))
            logging.getLogger().addHandler(log_capture)
        entry = {
            "unit": unit,
            "function": qualname,
            "args": args,
            "object_id": object_id,
            "_log_capture": log_capture,
            "_frame": frame,
            "_writer": writer,
        }
        stack.append(entry)
        self._writer = writer

    def _on_return(self, unit, frame, event, arg):
        stack = self._thread_stack()
        if not stack:
            return
        entry = stack.pop()
        writer = entry.get("_writer") or recorder.get_writer()
        log_capture = entry.pop("_log_capture", None)
        args = entry.get("args", [])
        if log_capture is not None:
            logging.getLogger().removeHandler(log_capture)
        mutated = self._mutated_args(entry, frame)
        side = {
            "mutated_args": mutated,
            "mcu_messages": [],
            "log_lines": list(log_capture.lines) if log_capture else [],
        }
        record = {
            "v": 1,
            "unit": entry.get("unit", unit),
            "function": entry.get("function", ""),
            "args": args,
            "side_effects": side,
            "object_id": entry.get("object_id"),
            "kind": "python",
        }
        if event == "exception":
            exc_type, exc_value = arg[0], arg[1]
            record["exc"] = {
                "type": exc_type.__name__,
                "message": str(exc_value),
            }
        else:
            record["ret"] = recorder.encode_value_safe(arg, self._ids)
        if writer is not None:
            writer.write(record["unit"], record)

    def _snapshot_args(self, frame):
        code = frame.f_code
        varnames = code.co_varnames
        args = []
        for i in range(code.co_argcount):
            name = varnames[i]
            try:
                value = frame.f_locals[name]
            except KeyError:
                continue
            args.append(recorder.encode_value_safe(value, self._ids))
        return args

    def _mutated_args(self, entry, frame):
        """Report list/dict arguments mutated during the call.

        The entry kept the encoded arguments from call time; a deep
        re-encoding that differs means the value changed. We only
        compare mutable containers by re-encoding the current frame
        locals for the same positions.
        """
        code = frame.f_code
        varnames = code.co_varnames
        before = entry.get("args", [])
        mutated = []
        for i in range(min(code.co_argcount, len(before))):
            name = varnames[i]
            try:
                value = frame.f_locals[name]
            except KeyError:
                continue
            if not isinstance(value, (list, dict)):
                continue
            after = recorder.encode_value_safe(value, self._ids)
            if after != before[i]:
                mutated.append([i, after])
        return mutated

    def install(self):
        if not self.unit_ids:
            return
        sys.setprofile(self._profile)
        threading.setprofile(self._profile)

    def uninstall(self):
        sys.setprofile(None)
        threading.setprofile(None)


_TRACER = None


def get_tracer():
    global _TRACER
    if _TRACER is not None:
        return _TRACER
    raw = os_environ_units()
    if not raw:
        return None
    _TRACER = UnitTracer(raw)
    return _TRACER


def os_environ_units():
    import os
    raw = os.environ.get("KLIPPY_RECORD_UNITS", "")
    return [part.strip() for part in raw.split(",") if part.strip()]


def install_from_environ():
    tracer = get_tracer()
    if tracer is not None:
        tracer.install()
    return tracer
