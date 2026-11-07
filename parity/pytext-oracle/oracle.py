# Oracle for pytext: emits expected results as JSON into
# tests/fixtures/pytext/. Stdlib only, CPython 3.8 semantics.
#
# Regenerate with:
#   python3 parity/pytext-oracle/oracle.py
#
# The Rust unit tests in klippy-rs/crates/pytext/tests/ read the fixtures
# when present and fall back to embedded expectations otherwise.

import configparser
import io
import json
import os
import shlex
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "tests", "fixtures", "pytext")


def klippy_parser(data, source):
    parser = configparser.RawConfigParser(
        strict=False, inline_comment_prefixes=(';', '#'))
    parser.read_file(io.StringIO(data), source)
    return parser


def parser_snapshot(parser):
    sections = []
    for name in parser.sections():
        items = [[k, v] for k, v in parser.items(name)]
        sections.append({"name": name, "items": items})
    return {
        "sections": sections,
        "defaults": [[k, v] for k, v in parser.defaults().items()],
        "written": parser.write(io.StringIO()) and _write(parser),
    }


def _write(parser):
    sfile = io.StringIO()
    parser.write(sfile)
    return sfile.getvalue()


def parse_cfg(path):
    with open(path, "r") as f:
        data = f.read()
    data = data.replace("\r\n", "\n")
    parser = klippy_parser(data, path)
    return parser_snapshot(parser)


def shlex_cases():
    cases = [
        "M104 S200",
        'G1 X10 Y20 Z5 F1500',
        'SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.8',
        'M117 Hello World',
        'M117 "quoted text"',
        "M117 'single quoted'",
        "FOO BAR='a b' ; comment",
        "FOO BAR='a b' # comment",
        "FOO a=\\'b",
    ]
    results = []
    for text in cases:
        try:
            lex = shlex.shlex(text, posix=True)
            lex.whitespace_split = True
            lex.commenters = '#;'
            tokens = list(lex)
            results.append({"input": text, "tokens": tokens, "error": None})
        except ValueError as e:
            results.append({"input": text, "tokens": [], "error": str(e)})
    return results


def main():
    os.makedirs(OUT, exist_ok=True)
    cfg_files = []
    for pattern in [
        os.path.join(ROOT, "test", "klippy", "*.cfg"),
        os.path.join(ROOT, "config", "*.cfg"),
        os.path.join(ROOT, "config", "ender-3-v3-ke", "*.cfg"),
    ]:
        import glob
        cfg_files.extend(sorted(glob.glob(pattern)))
    fixtures = {"config": {}, "shlex": shlex_cases()}
    for path in cfg_files:
        try:
            fixtures["config"][os.path.relpath(path, ROOT)] = parse_cfg(path)
        except Exception as e:
            fixtures["config"][os.path.relpath(path, ROOT)] = {"error": str(e)}
    dest = os.path.join(OUT, "fixtures.json")
    with open(dest, "w") as f:
        json.dump(fixtures, f, indent=2, sort_keys=True)
        f.write("\n")
    print("wrote %s with %d config files" % (dest, len(fixtures["config"])))


if __name__ == "__main__":
    main()
