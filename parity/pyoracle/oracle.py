#!/usr/bin/env python3
# Live reference oracle: evaluates Python against the frozen klippy/ sources and answers in the
# recording format (parity/record/FORMAT.md), so Rust tests can compare a port with the
# reference on generated inputs.
#
# Protocol: one JSON object per input line, one recording line per answer.
#   {"op": "exec", "code": "..."}   run statements in the shared namespace; ret is null
#   {"op": "eval", "expr": "..."}   evaluate an expression; ret is the encoded value
# Exceptions come back as {"exc": {"type": ..., "message": ...}} like recorded calls.
# The namespace starts with klippy/ and klippy/extras/ importable as in a klippy run.
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "klippy"))
sys.path.insert(0, os.path.join(REPO, "parity", "record"))

import recorder  # noqa: E402


def main():
    namespace = {"__name__": "pyoracle"}
    ids = recorder.IdTable() if hasattr(recorder, "IdTable") else None
    out = sys.stdout
    for seq, line in enumerate(sys.stdin, 1):
        line = line.strip()
        if not line:
            continue
        req = json.loads(line)
        rec = {"v": 1, "unit": "pyoracle", "scenario": "live", "seq": seq,
               "function": req.get("op", ""), "args": [], "object_id": None,
               "kind": "python",
               "side_effects": {"mutated_args": [], "mcu_messages": [], "log_lines": []}}
        try:
            if req["op"] == "exec":
                exec(req["code"], namespace)
                result = None
            elif req["op"] == "eval":
                result = eval(req["expr"], namespace)
            else:
                raise ValueError("unknown op %r" % (req["op"],))
            rec["ret"] = recorder.encode_value_safe(result, ids)
        except Exception as e:
            rec["exc"] = {"type": type(e).__name__, "message": str(e)}
        out.write(json.dumps(rec, separators=(",", ":")) + "\n")
        out.flush()


if __name__ == "__main__":
    main()
