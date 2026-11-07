# C boundary recorder.
#
# A wrapper around the cffi `lib` from `klippy/chelper/__init__.py
# get_ffi()`. Logs every call and return, including out-parameters
# and pulled structs, to the unit files defined in FORMAT.md.
#
# Usage:
#     import cffi_recorder
#     ffi, lib = cffi_recorder.activate()  # wrapped get_ffi()
#
# Or wrap an already-obtained lib:
#     lib = cffi_recorder.wrap_lib(ffi, real_lib)
#
# The wrapper records args, the return value, mutated out-parameters
# (buffers snapshotted before the call and diffed after), and the
# pulled structs of `*_extract_old` calls as field maps.
import logging
import os

import recorder
import units

# Struct field lists mirroring the cffi defs in klippy/chelper/__init__.py.
STRUCT_FIELDS = {
    "struct pull_move": (
        "print_time", "move_t", "start_v", "accel",
        "start_x", "start_y", "start_z",
        "x_r", "y_r", "z_r",
    ),
    "struct pull_history_steps": (
        "first_clock", "last_clock", "start_position",
        "step_count", "interval", "add",
    ),
    "struct pull_queue_message": (
        "len", "sent_time", "receive_time", "notify_id",
    ),
}

# Map C factory/call names to their unit ids, mirroring units.py.
_LIB_TO_UNIT = {}
for _unit, _cfg in units.UNIT_MAP.items():
    if _cfg["kind"] in ("c_boundary", "both"):
        for _name in _cfg["names"]:
            _LIB_TO_UNIT[_name] = _unit

# Functions whose C buffers act as out-parameters filled by the call.
_OUT_PARAM_FUNCS = {
    "trapq_extract_old",
    "stepcompress_extract_old",
    "serialqueue_extract_old",
    "serialqueue_get_stats",
}

# Functions whose calls emit MCU messages (raw bytes go on the wire).
_MCU_EMIT_FUNCS = {
    "serialqueue_send",
}


class _LogCapture(logging.Handler):
    def __init__(self):
        super().__init__()
        self.lines = []

    def emit(self, record):
        try:
            self.lines.append(self.format(record))
        except Exception:
            pass


class CffiLibWrapper:
    """Proxy for the cffi `lib` that records every call."""

    def __init__(self, ffi, real_lib):
        object.__setattr__(self, "_ffi", ffi)
        object.__setattr__(self, "_lib", real_lib)
        object.__setattr__(self, "_ids", recorder.GLOBAL_IDS)

    def __getattr__(self, name):
        ffi = object.__getattribute__(self, "_ffi")
        real_lib = object.__getattribute__(self, "_lib")
        real = getattr(real_lib, name)
        if not callable(real):
            return real
        wrapper = RecordingLibWrapper(self, name, real)
        return wrapper

    def __dir__(self):
        return dir(object.__getattribute__(self, "_lib"))


class RecordingLibWrapper:
    """One recorded cffi function."""

    def __init__(self, parent, name, real):
        self.parent = parent
        self.name = name
        self.real = real
        self.ffi = parent._ffi
        self.unit = _LIB_TO_UNIT.get(name)

    def __call__(self, *args, **kwargs):
        writer = recorder.get_writer()
        if writer is None or self.unit is None or _disabled():
            return self.real(*args, **kwargs)
        encoded_args = [encode_cffi_value(self.ffi, arg, self.parent._ids)
                        for arg in args]
        before = self._snapshot_out_params(args)
        log_capture = _LogCapture()
        log_capture.setFormatter(logging.Formatter("%(message)s"))
        logging.getLogger().addHandler(log_capture)
        try:
            result = self.real(*args, **kwargs)
            exc = None
        except Exception as e:
            result = None
            exc = e
        finally:
            logging.getLogger().removeHandler(log_capture)
        mutated = self._diff_out_params(before, args)
        mcu_messages = self._mcu_messages(args, result)
        record = recorder.base_record(self.unit, self.name, "c_boundary")
        record["args"] = encoded_args
        record["side_effects"] = {
            "mutated_args": mutated,
            "mcu_messages": mcu_messages,
            "log_lines": list(log_capture.lines),
        }
        if exc is not None:
            record["exc"] = {"type": type(exc).__name__, "message": str(exc)}
        else:
            record["ret"] = encode_cffi_value(self.ffi, result,
                                              self.parent._ids)
        writer.write(self.unit, record)
        if exc is not None:
            raise exc
        return result

    def _snapshot_out_params(self, args):
        if self.name not in _OUT_PARAM_FUNCS:
            return {}
        snapshot = {}
        for i, arg in enumerate(args):
            cdata = _as_cdata(self.ffi, arg)
            if cdata is None:
                continue
            snapshot[i] = _dump_buffer(self.ffi, cdata)
        return snapshot

    def _diff_out_params(self, before, args):
        mutated = []
        for i, old in sorted(before.items()):
            cdata = _as_cdata(self.ffi, args[i])
            if cdata is None:
                continue
            new = _dump_buffer(self.ffi, cdata)
            if new != old:
                mutated.append([i, encode_cffi_value(self.ffi, new,
                                                     self.parent._ids)])
        return mutated

    def _mcu_messages(self, args, result):
        if self.name not in _MCU_EMIT_FUNCS:
            return []
        out = []
        for arg in args:
            cdata = _as_cdata(self.ffi, arg)
            if cdata is None:
                continue
            try:
                raw = bytes(self.ffi.buffer(cdata))
            except Exception:
                continue
            if raw:
                out.append(raw.hex())
        return out


def _disabled():
    return os.environ.get("KLIPPY_RECORD_C", "") == "0"


def _as_cdata(ffi, value):
    try:
        return ffi.typeof(value)
    except Exception:
        return None


def _dump_buffer(ffi, cdata):
    """Snapshot the readable bytes of a cdata buffer, or None."""
    try:
        return bytes(ffi.buffer(cdata))
    except Exception:
        return None


def encode_cffi_value(ffi, value, ids):
    """Encode a cffi value per FORMAT.md.

    Structs become field maps, pointers become stable ids, byte
    buffers become hex, and floats (via addressable doubles) stay
    lossless.
    """
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        if recorder.INT_MIN <= value <= recorder.INT_MAX:
            return value
        return {"$int": str(value)}
    if isinstance(value, float):
        return recorder.encode_float(value)
    if isinstance(value, str):
        return value
    if isinstance(value, (bytes, bytearray)):
        return {"$bytes": bytes(value).hex()}
    if isinstance(value, (list, tuple)):
        return [encode_cffi_value(ffi, item, ids) for item in value]
    # cdata objects: dispatch on their cffi type.
    try:
        ctype = ffi.typeof(value)
    except Exception:
        return {"$oid": ids.stable_id(value)}
    ctype_str = str(ctype)
    if ctype_str in STRUCT_FIELDS:
        return _encode_struct(ffi, value, ctype_str, ids)
    if ctype_str.startswith("struct ") and ctype_str[7:] in STRUCT_FIELDS:
        return _encode_struct(ffi, value, ctype_str, ids)
    # char buffers and strings.
    try:
        raw = bytes(ffi.buffer(value))
        if ctype_str.startswith("char") or "char *" in ctype_str:
            return {"$bytes": raw.hex()}
    except Exception:
        pass
    # Opaque pointers and objects keep their identity.
    try:
        addr = int(ffi.cast("intptr_t", value))
        return {"$opaque": {"type": ctype_str, "addr": hex(addr)}}
    except Exception:
        return {"$oid": ids.stable_id(value)}


def _encode_struct(ffi, value, ctype_str, ids):
    fields = STRUCT_FIELDS[ctype_str]
    out = []
    for name in fields:
        try:
            field = getattr(value, name)
        except Exception:
            field = None
        out.append([name, encode_cffi_value(ffi, field, ids)])
    return {"$struct": {"type": ctype_str, "fields": out}}


def wrap_lib(ffi, real_lib):
    """Return a recording proxy for a cffi lib."""
    return CffiLibWrapper(ffi, real_lib)


def activate():
    """Patch klippy.chelper.get_ffi so callers get the wrapped lib.

    When KLIPPY_RECORD_C_HELPER names a recording c_helper.so, the
    wrapper also points the build check at it, so the recording
    library is used without editing the frozen klippy/ tree.
    """
    import klippy.chelper as chelper

    if hasattr(activate, "_done"):
        return chelper.get_ffi()
    activate._done = True
    real_get_ffi = chelper.get_ffi

    def recording_get_ffi():
        ffi, lib = real_get_ffi()
        return ffi, wrap_lib(ffi, lib)

    helper = os.environ.get("KLIPPY_RECORD_C_HELPER")
    if helper and os.path.isfile(helper):
        real_check_build = chelper.check_build_c_library
        chelper.check_build_c_library = lambda: helper
        activate._real_check_build = real_check_build
    chelper.get_ffi = recording_get_ffi
    return recording_get_ffi()
