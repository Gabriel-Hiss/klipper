#!/usr/bin/env python3
"""Apply the frozen normalizer table to text artifacts.

A rule matches by artifact glob (fnmatch against the root-relative path) and
its regular expression is applied in table order. The table is frozen: a rule
is added only when a value is proven irreproducible between the Python oracle
and the Rust port, and every rule carries its justification.
"""
import argparse
import fnmatch
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TABLE = os.path.join(HERE, "table.json")


def load_table(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    rules = data.get("rules", [])
    compiled = []
    for rule in rules:
        artifact = rule.get("artifact", "*")
        globs = [g.strip() for g in artifact.split(",") if g.strip()]
        pattern = rule["pattern"]
        compiled.append(
            {
                "id": rule["id"],
                "globs": globs,
                "regex": re.compile(pattern),
                "replacement": rule.get("replacement", ""),
            }
        )
    return compiled


def _matches(relpath, globs):
    base = os.path.basename(relpath)
    for glob in globs:
        if fnmatch.fnmatch(relpath, glob) or fnmatch.fnmatch(base, glob):
            return True
    return False


def normalize_text(text, relpath, rules):
    if not isinstance(text, str):
        text = text.decode("utf-8", "replace")
    applied = []
    for rule in rules:
        if _matches(relpath, rule["globs"]):
            new, count = rule["regex"].subn(rule["replacement"], text)
            if count:
                applied.append((rule["id"], count))
                text = new
    return text, applied


def is_text_artifact(relpath):
    lower = relpath.lower()
    if lower.endswith(".bin") or lower.endswith(".dict"):
        return False
    return True


def normalize_dir(src_dir, rules):
    """Yield (relpath, normalized_text) for every text file under src_dir."""
    for root, _dirs, files in os.walk(src_dir):
        for name in sorted(files):
            path = os.path.join(root, name)
            relpath = os.path.relpath(path, src_dir).replace(os.sep, "/")
            if not is_text_artifact(relpath):
                continue
            with open(path, "rb") as f:
                raw = f.read()
            text = raw.decode("utf-8", "replace")
            normalized, _applied = normalize_text(text, relpath, rules)
            yield relpath, normalized


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="input file (text)")
    parser.add_argument("output", nargs="?", help="output file (default: stdout)")
    parser.add_argument("--table", default=DEFAULT_TABLE)
    parser.add_argument("--artifact", default=None,
                          help="artifact name used for glob matching (default: input basename)")
    args = parser.parse_args()
    rules = load_table(args.table)
    relpath = args.artifact or os.path.basename(args.input)
    with open(args.input, "rb") as f:
        raw = f.read()
    text = raw.decode("utf-8", "replace")
    normalized, _applied = normalize_text(text, relpath, rules)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(normalized)
    else:
        sys.stdout.write(normalized)
    return 0


if __name__ == "__main__":
    sys.exit(main())
