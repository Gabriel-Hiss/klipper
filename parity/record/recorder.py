# Shared serialization for per-unit recordings.
#
# Implements the value encoding in FORMAT.md: lossless floats (hex
# float plus repr), lowercase hex bytes, C struct field maps, and
# stable ids for opaque objects and host objects.
import json
import os
import threading

INT_MIN = -(2 ** 63)
INT_MAX = 2 ** 63 - 1


class IdTable:
    """Assigns stable ids to opaque and host objects.

    Ids are `<ClassName>#<n>`, counting up from 1 per process. The
    table keeps strong references to every object it ids, so an id is
    never reused within a process, not even after garbage collection.
    """

    def __init__(self):
        self._by_key = {}
        self._counter = 0
        self._lock = threading.Lock()

    def stable_id(self, obj):
        key = (id(obj), type(obj).__name__)
        with self._lock:
            entry = self._by_key.get(key)
            if entry is not None and entry[0] is obj:
                return entry[1]
            self._counter += 1
            label = "%s#%d" % (type(obj).__name__, self._counter)
            self._by_key[key] = (obj, label)
            return label


class EncodeError(Exception):
    pass


def encode_float(x):
    return {"$float": {"hex": x.hex(), "repr": repr(x)}}


def encode_value(value, ids):
    """Encode a Python value per FORMAT.md.

    Raises EncodeError for values the recorder cannot represent, so
    the tracer can record a type placeholder instead of failing.
    """
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        if INT_MIN <= value <= INT_MAX:
            return value
        return {"$int": str(value)}
    if isinstance(value, float):
        return encode_float(value)
    if isinstance(value, str):
        return value
    if isinstance(value, (bytes, bytearray)):
        return {"$bytes": bytes(value).hex()}
    if isinstance(value, tuple):
        return {"$tuple": [encode_value(item, ids) for item in value]}
    if isinstance(value, list):
        return [encode_value(item, ids) for item in value]
    if isinstance(value, dict):
        return {"$dict": [[encode_value(k, ids), encode_value(v, ids)]
                          for k, v in value.items()]}
    # Host objects and anything else keep their identity.
    raise EncodeError("unencodable: %s" % type(value).__name__)


def encode_value_safe(value, ids):
    """Like encode_value, but falls back to a type placeholder."""
    try:
        return encode_value(value, ids)
    except (EncodeError, RecursionError):
        return {"$oid": ids.stable_id(value)}


class RecordWriter:
    """Appends one JSON object per line, one file per (unit, scenario)."""

    def __init__(self, root, scenario):
        self.root = root
        self.scenario = scenario
        self._files = {}
        self._counters = {}
        self._lock = threading.Lock()

    def path_for(self, unit):
        return os.path.join(self.root, unit,
                            "%s.jsonl" % self.scenario)

    def write(self, unit, record):
        record = dict(record)
        record["scenario"] = self.scenario
        with self._lock:
            self._counters[unit] = self._counters.get(unit, 0) + 1
            record["seq"] = self._counters[unit]
            path = self.path_for(unit)
            slot = self._files.get(unit)
            if slot is None or slot[0] != path:
                if slot is not None:
                    slot[1].close()
                directory = os.path.dirname(path)
                if not os.path.isdir(directory):
                    os.makedirs(directory, exist_ok=True)
                slot = (path, open(path, "a", encoding="utf-8"))
                self._files[unit] = slot
            slot[1].write(json.dumps(record, separators=(",", ":"),
                                     ensure_ascii=False))
            slot[1].write("\n")
            slot[1].flush()

    def close(self):
        with self._lock:
            for path, file in list(self._files.values()):
                if not file.closed:
                    file.close()
            self._files.clear()


GLOBAL_IDS = IdTable()


def get_writer():
    """Return the process-wide RecordWriter, or None when disabled.

    Enabled by KLIPPY_RECORD_DIR; KLIPPY_RECORD_SCENARIO names the
    scenario. The writer is rooted at
    `KLIPPY_RECORD_DIR/<unit>/<scenario>.jsonl`.
    """
    root = os.environ.get("KLIPPY_RECORD_DIR")
    if not root:
        return None
    scenario = os.environ.get("KLIPPY_RECORD_SCENARIO", "default")
    writer = getattr(get_writer, "_writer", None)
    if writer is None or writer.root != root or writer.scenario != scenario:
        writer = RecordWriter(root, scenario)
        get_writer._writer = writer
    return writer


def base_record(unit, function, kind, object_id=None):
    return {
        "v": 1,
        "unit": unit,
        "function": function,
        "args": [],
        "side_effects": {"mutated_args": [], "mcu_messages": [],
                         "log_lines": []},
        "object_id": object_id,
        "kind": kind,
    }
