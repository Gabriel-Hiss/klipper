#!/usr/bin/env python3
# Driver: record a scenario with every recorder enabled.
#
#   python3 parity/record/record.py --scenario DIR --units a,b \
#       --out tests/fixtures/recordings/
#
# Runs the scenario through parity/harness/run.py --impl python with
# the recorders enabled by environment variables and a sitecustomize
# hook. Recordings land at <out>/<unit_id>/<scenario>.jsonl, where
# <scenario> is the scenario directory basename.
#
# Environment passed to the harness child:
#   KLIPPY_RECORD_DIR        --out above (absolute path)
#   KLIPPY_RECORD_UNITS      comma separated unit ids
#   KLIPPY_RECORD_SCENARIO   scenario basename
#   KLIPPY_RECORD_HOME       this directory (parity/record, absolute)
#   KLIPPY_RECORD_C_HELPER   optional recording c_helper.so path
#   PYTHONPATH               hook dir (containing sitecustomize.py)
#                            prepended to any existing PYTHONPATH
#
# Options:
#   --scenario DIR   scenario directory (required)
#   --units a,b      comma separated unit ids (required; "all" for all)
#   --out DIR        recordings root (default tests/fixtures/recordings
#                    under the repository root)
#   --c-helper PATH  recording c_helper.so from c_internal/build.sh
#   --impl NAME      harness impl (default python)
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))

import units as units_module


def parse_units(raw, default="all"):
    raw = (raw or default).strip()
    if raw == "all":
        return sorted(units_module.UNIT_MAP.keys())
    out = [part.strip() for part in raw.split(",") if part.strip()]
    for unit in out:
        if unit not in units_module.UNIT_MAP:
            raise ValueError("unknown unit: %s" % unit)
    return out


def resolve_out(raw):
    if not raw:
        raw = os.path.join("tests", "fixtures", "recordings")
    if os.path.isabs(raw):
        return raw
    candidate = os.path.join(os.getcwd(), raw)
    if os.path.isdir(os.path.dirname(candidate)):
        return candidate
    return os.path.join(REPO_ROOT, raw)


def make_hook_dir():
    hook = tempfile.mkdtemp(prefix="record-hook-")
    shutil.copy(os.path.join(HERE, "sitecustomize.py"),
                os.path.join(hook, "sitecustomize.py"))
    return hook


def main():
    parser = argparse.ArgumentParser(
        description="record a scenario with every recorder enabled")
    parser.add_argument("--scenario", required=True,
                        help="scenario directory")
    parser.add_argument("--units", required=True,
                        help="comma separated unit ids, or 'all'")
    parser.add_argument("--out", default=None,
                        help="recordings root "
                        "(default tests/fixtures/recordings)")
    parser.add_argument("--c-helper", default=None,
                        help="recording c_helper.so path")
    parser.add_argument("--impl", default="python",
                        help="harness impl (default python)")
    args = parser.parse_args()

    scenario = os.path.abspath(args.scenario)
    if not os.path.isdir(scenario):
        parser.error("scenario directory not found: %s" % scenario)
    scenario_name = os.path.basename(scenario.rstrip(os.sep))
    out_dir = os.path.abspath(resolve_out(args.out))
    unit_ids = parse_units(args.units)

    hook_dir = make_hook_dir()
    env = dict(os.environ)
    env["KLIPPY_RECORD_DIR"] = out_dir
    env["KLIPPY_RECORD_UNITS"] = ",".join(unit_ids)
    env["KLIPPY_RECORD_SCENARIO"] = scenario_name
    env["KLIPPY_RECORD_HOME"] = HERE
    if args.c_helper:
        env["KLIPPY_RECORD_C_HELPER"] = os.path.abspath(args.c_helper)
    pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = hook_dir + (
        os.pathsep + pythonpath if pythonpath else "")

    harness = os.path.join(REPO_ROOT, "parity", "harness", "run.py")
    if not os.path.isfile(harness):
        sys.stderr.write(
            "error: harness not found at %s\n"
            "the harness (parity/harness/run.py) is built by the "
            "harness task; the recorder driver cannot run without "
            "it.\n" % harness)
        return 2
    harness_out = tempfile.mkdtemp(prefix="record-harness-out-")
    cmd = [sys.executable, harness, "--impl", args.impl,
           "--scenario", scenario, "--out", harness_out]
    sys.stderr.write("recording %s (units %s) ...\n"
                     % (scenario_name, ",".join(unit_ids)))
    result = subprocess.call(cmd, env=env, cwd=REPO_ROOT)
    if result != 0:
        sys.stderr.write(
            "harness exited with %d; recordings may be incomplete "
            "under %s\n" % (result, out_dir))
    for unit in unit_ids:
        path = os.path.join(out_dir, unit, scenario_name + ".jsonl")
        mark = "OK " if os.path.isfile(path) else "MISS"
        sys.stderr.write("%s %s\n" % (mark, path))
    return result


if __name__ == "__main__":
    sys.exit(main())
