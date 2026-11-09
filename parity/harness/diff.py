#!/usr/bin/env python3
"""Compare two harness artifact dirs after normalization.

Applies parity/normalize/table.json to both sides, then compares every text
artifact. MCU raw streams are compared by their message payloads with the
framing removed (serialqueue groups commands into frames by background-thread
timing, so two runs of the same Python host frame identical commands
differently); on a payload difference the first divergence reports the decoded
message index, the oid when present, and the raw byte offset of the diverging
block (computed with klippy/msgproto when the dictionary is available).
"""
import argparse
import difflib
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
NORMALIZE_DIR = os.path.join(os.path.dirname(HERE), "normalize")
KLIPPY_DIR = os.path.join(REPO_ROOT, "klippy")
if KLIPPY_DIR not in sys.path:
    sys.path.insert(0, KLIPPY_DIR)
sys.path.insert(0, NORMALIZE_DIR)
try:
    import normalize as _normalize
except ImportError:
    _normalize = None


def _load_rules(table_path):
    if _normalize is not None:
        return _normalize.load_table(table_path)
    return []


def _apply_debug_motion_flag(text, relpath, debug_motion):
    if relpath.endswith(".log") or relpath in ("log", "stdout.txt", "stderr.txt"):
        if not debug_motion:
            lines = []
            for line in text.splitlines(True):
                if "[DEBUG-motion]" in line:
                    continue
                lines.append(line)
            return "".join(lines)
    return text


def _norm_text(path, relpath, rules, debug_motion):
    with open(path, "rb") as f:
        raw = f.read()
    text = raw.decode("utf-8", "replace")
    if _normalize is not None:
        text, _applied = _normalize.normalize_text(text, relpath, rules)
    return _apply_debug_motion_flag(text, relpath, debug_motion)


def _manifest_exit(artifact_dir):
    path = os.path.join(artifact_dir, "manifest.json")
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return str(json.load(f).get("exit_code"))
    except Exception:
        return None


def _collect_relpaths(artifact_dir):
    out = set()
    for root, dirnames, files in os.walk(artifact_dir):
        dirnames.sort()
        for name in sorted(files):
            if name == "manifest.json":
                continue
            full = os.path.join(root, name)
            rel = os.path.relpath(full, artifact_dir).replace(os.sep, "/")
            out.add(rel)
    return sorted(out)


_OID_RE = re.compile(r"oid=(\d+)")
_BLOCK_RE = re.compile(r"^seq:\s*[0-9a-fA-F]+")


def _mcu_byte_offset(artifact_dir, relpath, line_index):
    """Map a decoded-line index to a raw byte offset using msgproto."""
    base = os.path.basename(relpath)
    if not base.startswith("mcu") or not relpath.endswith(".txt"):
        return None
    mcu_dir = os.path.join(artifact_dir, "mcu")
    bin_path = os.path.join(mcu_dir, base[:-4] + ".bin")
    dict_path = os.path.join(mcu_dir, base[:-4] + ".dict")
    if not (os.path.exists(bin_path) and os.path.exists(dict_path)):
        return None
    try:
        import msgproto
    except Exception:
        return None
    try:
        with open(dict_path, "rb") as f:
            dictionary = f.read()
        mp = msgproto.MessageParser()
        mp.process_identify(dictionary, decompress=False)
        with open(bin_path, "rb") as f:
            data = bytearray(f.read())
        idx = 0
        offset = 0
        while True:
            l = mp.check_packet(data)
            if l == 0 or l < 0:
                break
            msgs = mp.dump(data[:l])
            body = msgs[1:]
            for body_line in body:
                if idx == line_index:
                    return offset
                idx += 1
            offset += l
            data = data[l:]
            if not data:
                break
    except Exception:
        return None
    return None


def _first_divergence(left_text, right_text):
    left_lines = left_text.splitlines()
    right_lines = right_text.splitlines()
    max_len = max(len(left_lines), len(right_lines))
    for i in range(max_len):
        left_line = left_lines[i] if i < len(left_lines) else "<EOF>"
        right_line = right_lines[i] if i < len(right_lines) else "<EOF>"
        if left_line != right_line:
            return i, left_line, right_line, left_lines, right_lines
    return None, None, None, left_lines, right_lines


def _frame_payloads(data):
    """Concatenated message payloads of a raw MCU stream, without the frame
    header (length, sequence) and trailer (crc, sync). None when the stream
    does not split into whole valid frames."""
    try:
        import msgproto
    except Exception:
        return None
    mp = msgproto.MessageParser()
    data = bytearray(data)
    out = bytearray()
    while data:
        length = mp.check_packet(data)
        if length <= 0:
            return None
        out += data[msgproto.MESSAGE_HEADER_SIZE:length - msgproto.MESSAGE_TRAILER_SIZE]
        data = data[length:]
    return bytes(out)


def _is_mcu_relpath(relpath):
    return relpath.startswith("mcu/") and relpath.endswith(".txt")


def _compare_file(left_dir, right_dir, relpath, rules, debug_motion):
    left_path = os.path.join(left_dir, *relpath.split("/"))
    right_path = os.path.join(right_dir, *relpath.split("/"))
    l_is_bin = relpath.endswith(".bin")
    if l_is_bin:
        with open(left_path, "rb") as f:
            left_raw = f.read()
        with open(right_path, "rb") as f:
            right_raw = f.read()
        if left_raw == right_raw:
            return []
        left_payload = _frame_payloads(left_raw)
        if left_payload is not None and left_payload == _frame_payloads(right_raw):
            return []
        idx, left_dec, right_dec, l_lines, r_lines = _decoded_divergence(
            left_dir, right_dir, relpath)
        offset = None
        if idx is not None:
            offset = _mcu_byte_offset(left_dir, relpath, idx)
        oid = _extract_oid(left_dec)
        return [
            ("DIVERGE %s (raw bytes, %d vs %d bytes)" % (relpath, len(left_raw), len(right_raw)),
             _context("first decoded divergence", idx, left_dec, right_dec, l_lines, r_lines,
                      oid=oid, byte_offset=offset)),
        ]
    left_text = _norm_text(left_path, relpath, rules, debug_motion)
    right_text = _norm_text(right_path, relpath, rules, debug_motion)
    if left_text == right_text:
        return []
    idx, left_line, right_line, l_lines, r_lines = _first_divergence(left_text, right_text)
    oid = _extract_oid(left_line) if left_line else None
    offset = _mcu_byte_offset(left_dir, relpath, idx) if _is_mcu_relpath(relpath) and idx is not None else None
    return [
        ("DIVERGE %s" % relpath,
         _context("first divergence: line %d" % (idx + 1) if idx is not None else "first divergence",
                  idx, left_line, right_line, l_lines, r_lines, oid=oid, byte_offset=offset)),
    ]


def _extract_oid(line):
    if not line:
        return None
    match = _OID_RE.search(line)
    return match.group(1) if match else None


def _context(title, idx, left_line, right_line, l_lines, r_lines, oid=None, byte_offset=None):
    buf = ["  %s" % title]
    if oid is not None:
        buf.append("  oid=%s" % oid)
    if byte_offset is not None:
        buf.append("  raw byte offset=%d" % byte_offset)
    buf.append("  left : %r" % left_line)
    buf.append("  right: %r" % right_line)
    if idx is not None:
        buf.append("--- left context ---")
        for i in range(max(0, idx - 3), min(len(l_lines), idx + 4)):
            buf.append("%6d%s %s" % (i + 1, "*" if i == idx else " ", l_lines[i][:200]))
        buf.append("--- right context ---")
        for i in range(max(0, idx - 3), min(len(r_lines), idx + 4)):
            buf.append("%6d%s %s" % (i + 1, "*" if i == idx else " ", r_lines[i][:200]))
    return "\n".join(buf)


def _decoded_divergence(left_dir, right_dir, relpath):
    base = os.path.basename(relpath)
    txt_name = base[:-4] + ".txt"
    for side, root in (("left", left_dir), ("right", right_dir)):
        txt_path = os.path.join(root, "mcu", txt_name)
        if not os.path.exists(txt_path):
            return None, None, None, [], []
    with open(os.path.join(left_dir, "mcu", txt_name), "rb") as f:
        left_text = f.read().decode("utf-8", "replace")
    with open(os.path.join(right_dir, "mcu", txt_name), "rb") as f:
        right_text = f.read().decode("utf-8", "replace")
    return _first_divergence(left_text, right_text)


def compare(left_dir, right_dir, table_path, debug_motion=True):
    rules = _load_rules(table_path)
    left_files = set(_collect_relpaths(left_dir))
    right_files = set(_collect_relpaths(right_dir))
    all_files = left_files | right_files
    results = []
    for relpath in sorted(all_files):
        if relpath not in left_files:
            results.append(("ONLY-RIGHT %s" % relpath,))
            continue
        if relpath not in right_files:
            results.append(("ONLY-LEFT %s" % relpath,))
            continue
        results.extend(_compare_file(left_dir, right_dir, relpath, rules, debug_motion))
    left_exit = _manifest_exit(left_dir)
    right_exit = _manifest_exit(right_dir)
    if left_exit != right_exit:
        results.append(("DIVERGE exit_code left=%s right=%s" % (left_exit, right_exit),))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left", help="left artifact dir")
    parser.add_argument("right", help="right artifact dir")
    parser.add_argument("--table", default=os.path.join(NORMALIZE_DIR, "table.json"))
    parser.add_argument("--debug-motion", action="store_true", default=True,
                          help="compare [DEBUG-motion] lines (default)")
    parser.add_argument("--no-debug-motion", dest="debug_motion", action="store_false",
                          help="mask whole [DEBUG-motion] lines on both sides")
    args = parser.parse_args()
    results = compare(args.left, args.right, args.table, args.debug_motion)
    if not results:
        print("IDENTICAL (after normalization, %d artifacts)" % len(_collect_relpaths(args.left)))
        return 0
    for block in results:
        print("\n".join(block))
        print("---")
    return 1


if __name__ == "__main__":
    sys.exit(main())
