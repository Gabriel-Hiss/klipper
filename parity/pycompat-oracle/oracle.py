#!/usr/bin/env python3
"""CPython oracle for pycompat differential tests.

Reads JSON-lines requests on stdin, one ``{"op": ..., "args": {...}}`` per
line, and writes one JSON response per line: ``{"ok": true, "result": ...}``
on success or ``{"ok": false, "kind": "ValueError", "msg": "..."}`` on
failure, where ``kind`` is the exception class name and ``msg`` is
``str(e)``.

Run with the reference interpreter (Python 3.8):

    PYCOMPAT_PYTHON=python3.8 python3 -m pytest ...
    python3.8 parity/pycompat-oracle/oracle.py < requests.jsonl > responses.jsonl

Floats travel as hex bit patterns (``bits``) so that ``-0.0``, subnormals and
``nan``/``inf`` survive JSON exactly. Ints travel as decimal strings.
Byte strings travel as hex. ``PyValue`` payloads use tagged objects:
``{"t": "none"}``, ``{"t": "bool", "v": true}``,
``{"t": "int", "v": "123"}``, ``{"t": "float", "bits": "3ff0000000000000"}``,
``{"t": "str", "v": "..."}``, ``{"t": "bytes", "hex": "6162"}``,
``{"t": "list", "items": [...]}``, ``{"t": "tuple", "items": [...]}``,
``{"t": "dict", "items": [[k, v], ...]}``, ``{"t": "host", "v": 0}``.

The ``sum`` op sums with plain left-to-right addition, matching CPython 3.8
(Neumaier summation only arrived in 3.12).
"""
import json
import struct
import sys


def f64_from_bits(bits):
    return struct.unpack("<d", struct.pack("<Q", int(bits, 16)))[0]


def f64_to_bits(x):
    return "%016x" % struct.unpack("<Q", struct.pack("<d", x))[0]


def decode_value(v):
    t = v.get("t")
    if t == "none":
        return None
    if t == "bool":
        return bool(v["v"])
    if t == "int":
        return int(v["v"], 10)
    if t == "float":
        return f64_from_bits(v["bits"])
    if t == "str":
        return v["v"]
    if t == "bytes":
        return bytes.fromhex(v["hex"])
    if t == "list":
        return [decode_value(x) for x in v["items"]]
    if t == "tuple":
        return tuple(decode_value(x) for x in v["items"])
    if t == "dict":
        return {decode_value(k): decode_value(val) for k, val in v["items"]}
    if t == "host":
        return Host(v["v"])
    raise ValueError("unknown tag %r" % (t,))


class Host:
    """Opaque host object: repr only, always truthy."""

    def __init__(self, h):
        self.h = h

    def __repr__(self):
        return "<host object at %#x>" % self.h

    __str__ = __repr__

    def __bool__(self):
        return True


def encode_value(v):
    if v is None:
        return {"t": "none"}
    if isinstance(v, bool):
        return {"t": "bool", "v": v}
    if isinstance(v, int):
        return {"t": "int", "v": str(v)}
    if isinstance(v, float):
        return {"t": "float", "bits": f64_to_bits(v)}
    if isinstance(v, str):
        return {"t": "str", "v": v}
    if isinstance(v, bytes):
        return {"t": "bytes", "hex": v.hex()}
    if isinstance(v, list):
        return {"t": "list", "items": [encode_value(x) for x in v]}
    if isinstance(v, tuple):
        return {"t": "tuple", "items": [encode_value(x) for x in v]}
    if isinstance(v, dict):
        return {"t": "dict", "items": [[encode_value(k), encode_value(val)] for k, val in v.items()]}
    if isinstance(v, Host):
        return {"t": "host", "v": v.h}
    raise TypeError("cannot encode %r" % type(v))


def exc(e):
    return {"ok": False, "kind": type(e).__name__, "msg": str(e)}


def handle(op, args):
    # --- float text ---
    if op == "float_repr":
        return {"ok": True, "result": repr(f64_from_bits(args["bits"]))}
    if op == "float_str":
        return {"ok": True, "result": str(f64_from_bits(args["bits"]))}
    if op == "str_repr":
        return {"ok": True, "result": repr(args["s"])}
    if op == "bytes_repr":
        return {"ok": True, "result": repr(bytes.fromhex(args["hex"]))}
    if op == "float_py_hash":
        return {"ok": True, "result": hash(f64_from_bits(args["bits"]))}
    # --- percent formatting: args use scalar tags ---
    def pct_val(d):
        return decode_value(d)

    if op == "percent_format":
        fmt = args["fmt"]
        vals = tuple(pct_val(d) for d in args["args"])
        # CPython: singleton tuple vs flat args handled by the caller.
        try:
            return {"ok": True, "result": fmt % vals}
        except Exception as e:
            return exc(e)
    if op == "percent_format_map":
        fmt = args["fmt"]
        m = {k: pct_val(v) for k, v in args["map"]}
        try:
            return {"ok": True, "result": fmt % m}
        except Exception as e:
            return exc(e)
    if op == "format_value":
        v = pct_val(args["value"])
        try:
            return {"ok": True, "result": format(v, args["spec"])}
        except Exception as e:
            return exc(e)
    if op == "str_format":
        tmpl = args["template"]
        pos = tuple(pct_val(d) for d in args["args"])
        kw = {k: pct_val(v) for k, v in args["kwargs"]}
        try:
            return {"ok": True, "result": tmpl.format(*pos, **kw)}
        except Exception as e:
            return exc(e)
    # --- rounding ---
    if op == "round_py":
        try:
            return {"ok": True, "result": str(round(f64_from_bits(args["bits"])))}
        except Exception as e:
            return exc(e)
    if op == "round_py_nd":
        try:
            x = f64_from_bits(args["bits"])
            n = args["ndigits"]
            r = round(x, n)
            if isinstance(r, int):
                return {"ok": True, "result": str(r)}
            return {"ok": True, "result": f64_to_bits(r)}
        except Exception as e:
            return exc(e)
    if op == "trunc_float_to_bigint":
        try:
            return {"ok": True, "result": str(int(f64_from_bits(args["bits"])))}
        except Exception as e:
            return exc(e)
    if op == "float_from_string":
        try:
            return {"ok": True, "result": f64_to_bits(float(args["s"]))}
        except Exception as e:
            return exc(e)
    if op == "int_from_string":
        try:
            s = args["s"]
            b = args["base"]
            return {"ok": True, "result": str(int(s, b))}
        except Exception as e:
            return exc(e)
    # --- int arithmetic (decimal strings) ---
    if op == "int_floor_div":
        try:
            return {"ok": True, "result": str(int(args["a"], 10) // int(args["b"], 10))}
        except Exception as e:
            return exc(e)
    if op == "int_mod":
        try:
            return {"ok": True, "result": str(int(args["a"], 10) % int(args["b"], 10))}
        except Exception as e:
            return exc(e)
    if op == "int_divmod":
        try:
            q, r = divmod(int(args["a"], 10), int(args["b"], 10))
            return {"ok": True, "result": [str(q), str(r)]}
        except Exception as e:
            return exc(e)
    if op == "int_pow":
        try:
            return {"ok": True, "result": str(pow(int(args["a"], 10), args["e"]))}
        except Exception as e:
            return exc(e)
    if op == "int_cmp":
        a = int(args["a"], 10)
        b = int(args["b"], 10)
        return {"ok": True, "result": (a > b) - (a < b)}
    if op == "int_is_odd":
        return {"ok": True, "result": bool(int(args["a"], 10) % 2)}
    if op == "int_py_hash":
        return {"ok": True, "result": hash(int(args["a"], 10))}
    # --- float arithmetic ---
    if op == "float_floor_div":
        try:
            return {"ok": True, "result": f64_to_bits(f64_from_bits(args["a"]) // f64_from_bits(args["b"]))}
        except Exception as e:
            return exc(e)
    if op == "float_mod":
        try:
            return {"ok": True, "result": f64_to_bits(f64_from_bits(args["a"]) % f64_from_bits(args["b"]))}
        except Exception as e:
            return exc(e)
    if op == "float_true_div":
        try:
            return {"ok": True, "result": f64_to_bits(f64_from_bits(args["a"]) / f64_from_bits(args["b"]))}
        except Exception as e:
            return exc(e)
    if op == "float_pow":
        try:
            return {"ok": True, "result": f64_to_bits(f64_from_bits(args["a"]) ** f64_from_bits(args["b"]))}
        except Exception as e:
            return exc(e)
    if op == "cmp_bigint_float":
        # Returns -1/0/1 for bigint - float (nan guarded by caller).
        a = int(args["a"], 10)
        b = f64_from_bits(args["bits"])
        return {"ok": True, "result": (a > b) - (a < b)}
    if op == "py_sum_floats":
        xs = [f64_from_bits(b) for b in args["bits_list"]]
        total = 0
        for v in xs:
            total = total + v
        return {"ok": True, "result": f64_to_bits(total if isinstance(total, float) else 0.0)}
    # --- PyValue ---
    if op == "py_repr":
        return {"ok": True, "result": repr(decode_value(args["value"]))}
    if op == "py_str":
        return {"ok": True, "result": str(decode_value(args["value"]))}
    if op == "py_truthy":
        return {"ok": True, "result": bool(decode_value(args["value"]))}
    if op == "py_equal":
        return {"ok": True, "result": decode_value(args["a"]) == decode_value(args["b"])}
    if op == "py_compare":
        try:
            a = decode_value(args["a"])
            b = decode_value(args["b"])
            return {"ok": True, "result": (a > b) - (a < b)}
        except Exception as e:
            return exc(e)
    if op == "py_rich_compare":
        try:
            a = decode_value(args["a"])
            b = decode_value(args["b"])
            opr = args["op"]
            r = {"==": a == b, "!=": a != b, "<": a < b, "<=": a <= b, ">": a > b, ">=": a >= b}[opr]
            return {"ok": True, "result": bool(r)}
        except Exception as e:
            return exc(e)
    if op == "py_hash":
        try:
            v = decode_value(args["value"])
            return {"ok": True, "result": hash(v)}
        except Exception as e:
            return exc(e)
    if op == "type_name":
        return {"ok": True, "result": type(decode_value(args["value"])).__name__}
    # --- JSON ---
    if op == "json_dumps":
        opts = args.get("opts", {})
        try:
            s = json.dumps(
                decode_value(args["value"]),
                separators=tuple(opts.get("separators", (", ", ": "))),
                ensure_ascii=opts.get("ensure_ascii", True),
                allow_nan=opts.get("allow_nan", True),
            )
            return {"ok": True, "result": s}
        except Exception as e:
            return exc(e)
    if op == "json_loads":
        try:
            v = json.loads(args["s"])
            return {"ok": True, "result": encode_value(v)}
        except Exception as e:
            return exc(e)
    return exc(ValueError("unknown op %r" % (op,)))


def main():
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            resp = handle(req["op"], req.get("args", {}))
        except Exception as e:
            resp = exc(e)
        sys.stdout.write(json.dumps(resp) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
