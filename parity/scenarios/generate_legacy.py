#!/usr/bin/env python3
"""Convert test/klippy/*.test into harness scenarios.

Reproduces the multi-config semantics of scripts/test_klippy.py: the first
CONFIG is held; when a second CONFIG appears the first is launched, and each
later CONFIG launches immediately with the dictionary and G-code current at
that point. One scenario directory per CONFIG is written under
parity/scenarios/legacy/<slug>/ plus an index.json. The generator is
reproducible: sorted file order, sorted dict handling, deterministic slugs.

Dictionaries are referenced as tests/fixtures/dict/<name> using the exact
names the .test files use (for example atmega2560.dict). Those files are
expected to be populated from the CI data-dict artifact (see map-parity 4.3);
missing dictionaries are a scenario setup error at run time, not a generator
error, so the legacy corpus converts before the dicts land.
"""
import argparse
import json
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LEGACY_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "legacy")
TEST_DIR = os.path.join(REPO_ROOT, "test", "klippy")
FIXTURE_DICT_DIR = "tests/fixtures/dict"


def slug_config(config_path):
    base = os.path.basename(config_path)
    stem = os.path.splitext(base)[0]
    stem = re.sub(r"[^a-zA-Z0-9]+", "-", stem).strip("-").lower()
    return stem or "config"


def repo_relative(test_file, config_path):
    """CONFIG paths in .test files are relative to the .test file dir."""
    test_dir = os.path.dirname(test_file)
    abs_path = os.path.normpath(os.path.join(test_dir, *config_path.split("/")))
    return os.path.relpath(abs_path, REPO_ROOT).replace(os.sep, "/")


def gcode_relative(test_file, gcode_path):
    test_dir = os.path.dirname(test_file)
    abs_path = os.path.normpath(os.path.join(test_dir, *gcode_path.split("/")))
    return os.path.relpath(abs_path, REPO_ROOT).replace(os.sep, "/")


def parse_test_file(test_file):
    """Yield (config_rel, dict_list, gcode_rel, inline_lines, should_fail)."""
    config_fname = gcode_fname = dict_fnames = None
    should_fail = False
    multi_tests = False
    gcode_lines = []
    results = []

    def emit():
        results.append((config_fname, list(dict_fnames) if dict_fnames else None,
                        gcode_fname, list(gcode_lines), should_fail))

    with open(test_file, "r", encoding="utf-8") as f:
        for line in f:
            cpos = line.find("#")
            if cpos >= 0:
                line = line[:cpos]
            parts = line.strip().split()
            if not parts:
                continue
            if parts[0] == "CONFIG":
                if config_fname is not None:
                    if not multi_tests:
                        multi_tests = True
                        if gcode_fname is not None and gcode_lines:
                            raise ValueError(
                                "Can't specify both a gcode file and gcode commands in %s" % test_file)
                        emit()
                    config_fname = repo_relative(test_file, parts[1])
                    # Later CONFIG launches immediately (test_klippy.py).
                    if gcode_fname is not None and gcode_lines:
                        raise ValueError(
                            "Can't specify both a gcode file and gcode commands in %s" % test_file)
                    emit()
                else:
                    config_fname = repo_relative(test_file, parts[1])
            elif parts[0] == "DICTIONARY":
                if len(parts) < 2:
                    raise ValueError("DICTIONARY without file in %s" % test_file)
                dict_fnames = ["%s/%s" % (FIXTURE_DICT_DIR, parts[1])]
                for mcu_dict in parts[2:]:
                    mcu, fname = mcu_dict.split("=", 1)
                    dict_fnames.append("%s=%s/%s" % (
                        mcu.strip(), FIXTURE_DICT_DIR, fname.strip()))
            elif parts[0] == "GCODE":
                gcode_fname = gcode_relative(test_file, parts[1])
            elif parts[0] == "SHOULD_FAIL":
                should_fail = True
            else:
                gcode_lines.append(line.strip())
    if not multi_tests:
        if config_fname is None:
            raise ValueError("config file not specified in %s" % test_file)
        if dict_fnames is None:
            raise ValueError("data dictionary file not specified in %s" % test_file)
        if gcode_fname is not None and gcode_lines:
            raise ValueError("Can't specify both a gcode file and gcode commands in %s" % test_file)
        emit()
    return results


def dicts_to_mcus(dict_list):
    """Convert test_klippy -d list into mcus entries."""
    mcus = []
    for item in dict_list:
        if "=" in item:
            name, path = item.split("=", 1)
            mcus.append({"name": name, "dict": path})
        else:
            mcus.append({"name": "mcu", "dict": item})
    if not mcus:
        mcus = [{"name": "mcu", "dict": "%s/atmega2560.dict" % FIXTURE_DICT_DIR}]
    return mcus


def write_scenario(slug, config_rel, dict_list, gcode_rel, inline_lines, should_fail,
                   index_entries):
    scenario_dir = os.path.join(LEGACY_OUT, slug)
    os.makedirs(scenario_dir, exist_ok=True)
    if gcode_rel is not None:
        gcode_spec = {"source": "file", "path": gcode_rel}
    else:
        gcode_path = os.path.join(scenario_dir, "input.gcode")
        with open(gcode_path, "w", encoding="utf-8") as f:
            f.write("\n".join(inline_lines + [""]))
        gcode_spec = {"source": "file", "path": "legacy/%s/input.gcode" % slug}
    scenario = {
        "name": slug,
        "mode": "batch",
        "config": {"main": config_rel, "files": [], "mounts": []},
        "mcus": dicts_to_mcus(dict_list),
        "gcode": gcode_spec,
        "webhooks": {"script": [], "expect": {}},
        "expect": {"exit": 255 if should_fail else 0, "signals": []},
        "collect": [],
        "run": {"log": "file", "verbose": True, "debug_motion": True},
    }
    scenario_path = os.path.join(scenario_dir, "scenario.json")
    with open(scenario_path, "w", encoding="utf-8") as f:
        json.dump(scenario, f, indent=2, sort_keys=True)
        f.write("\n")
    index_entries.append({"slug": slug, "scenario": "legacy/%s/scenario.json" % slug,
                          "config": config_rel, "should_fail": should_fail})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test-dir", default=TEST_DIR)
    parser.add_argument("--out", default=LEGACY_OUT)
    parser.add_argument("--check", action="store_true",
                        help="verify generated scenarios parse and exit 0 without writing")
    args = parser.parse_args()
    test_files = sorted(
        os.path.join(args.test_dir, name)
        for name in os.listdir(args.test_dir) if name.endswith(".test"))
    index_entries = []
    count = 0
    for test_file in test_files:
        stem = os.path.splitext(os.path.basename(test_file))[0]
        cases = parse_test_file(test_file)
        for i, (config_rel, dict_list, gcode_rel, inline_lines, should_fail) in enumerate(cases, start=1):
            slug = "%s-%03d-%s" % (stem.lower(), i, slug_config(config_rel))
            if args.check:
                index_entries.append({"slug": slug, "config": config_rel,
                                      "should_fail": should_fail})
            else:
                write_scenario(slug, config_rel, dict_list, gcode_rel,
                               inline_lines, should_fail, index_entries)
            count += 1
    if not args.check:
        # Deterministic index.
        index_entries.sort(key=lambda e: e["slug"])
        index_path = os.path.join(LEGACY_OUT, "index.json")
        with open(index_path, "w", encoding="utf-8") as f:
            json.dump({"count": len(index_entries), "scenarios": index_entries},
                      f, indent=2, sort_keys=True)
            f.write("\n")
    print("generated %d scenarios from %d test files (check=%s)" % (
        count, len(test_files), args.check))
    return 0


if __name__ == "__main__":
    sys.exit(main())
