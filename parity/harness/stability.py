#!/usr/bin/env python3
"""Run one scenario N times for one impl and report artifact stability.

Passes when every run's normalized artifacts are identical (and exit codes
match). Each run uses run.py; the report is written to <out>/stability.json
and printed. A failure prints the first divergence between run 1 and the
failing run, exactly like diff.py.
"""
import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
NORMALIZE_DIR = os.path.join(os.path.dirname(HERE), "normalize")
if NORMALIZE_DIR not in sys.path:
    sys.path.insert(0, NORMALIZE_DIR)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
try:
    import normalize as _normalize
except ImportError:
    _normalize = None
try:
    import diff as _diff
except ImportError:
    _diff = None


def _run_one(run_py, impl, scenario, out_dir, extra):
    if os.path.exists(out_dir):
        import shutil

        shutil.rmtree(out_dir)
    cmd = [sys.executable, run_py, "--impl", impl,
           "--scenario", scenario, "--out", out_dir] + extra
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True, cwd=REPO_ROOT)
    return proc.returncode, proc.stdout + proc.stderr


def compare_all(base_dir, other_dir, table_path, debug_motion=True):
    if _diff is None:
        return [("diff module unavailable",)]
    return _diff.compare(base_dir, other_dir, table_path, debug_motion)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--impl", choices=["python", "rust"], required=True)
    parser.add_argument("--rust-bin", default=None)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--out", required=True,
                        help="output dir for run-001.. artifacts plus stability.json")
    parser.add_argument("--table", default=os.path.join(NORMALIZE_DIR, "table.json"))
    parser.add_argument("--vclockd", default=None)
    parser.add_argument("--mcusim", default=None)
    parser.add_argument("--epoch", type=int, default=1700000000)
    parser.add_argument("--python", default=None)
    parser.add_argument("--no-debug-motion", action="store_true",
                        help="mask [DEBUG-motion] lines when comparing")
    args = parser.parse_args()
    run_py = os.path.join(HERE, "run.py")
    os.makedirs(args.out, exist_ok=True)
    extra = []
    if args.rust_bin:
        extra += ["--rust-bin", args.rust_bin]
    if args.vclockd:
        extra += ["--vclockd", args.vclockd]
    if args.mcusim:
        extra += ["--mcusim", args.mcusim]
    extra += ["--epoch", str(args.epoch)]
    if args.python:
        extra += ["--python", args.python]
    debug_motion = not args.no_debug_motion
    run_dirs = []
    run_results = []
    for i in range(1, args.runs + 1):
        out_dir = os.path.join(args.out, "run-%03d" % i)
        returncode, output = _run_one(run_py, args.impl, args.scenario, out_dir, extra)
        run_dirs.append(out_dir)
        run_results.append({"run": i, "returncode": returncode, "output_tail": output[-2000:]})
        sys.stderr.write("stability: run %d/%d returncode=%d\n" % (i, args.runs, returncode))
    stable = True
    divergences = []
    base = run_dirs[0]
    for i, other in enumerate(run_dirs[1:], start=2):
        diffs = compare_all(base, other, args.table, debug_motion)
        if diffs:
            stable = False
            divergences.append({"run": i, "diffs": ["\n".join(b) for b in diffs]})
    report = {
        "scenario": args.scenario,
        "impl": args.impl,
        "runs": args.runs,
        "stable": stable,
        "run_results": run_results,
        "divergences": divergences,
    }
    report_path = os.path.join(args.out, "stability.json")
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
        f.write("\n")
    if stable:
        print("STABLE: %d runs identical (after normalization)" % args.runs)
        return 0
    print("UNSTABLE: %d/%d runs diverged" % (len(divergences), args.runs))
    for div in divergences[:1]:
        print("run %d vs run 1:" % div["run"])
        for block in div["diffs"]:
            print(block)
            print("---")
    return 1


if __name__ == "__main__":
    sys.exit(main())
