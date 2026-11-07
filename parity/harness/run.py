#!/usr/bin/env python3
"""Run one parity scenario for the Python or Rust implementation.

See parity/harness/SCENARIO.md for the scenario directory format. Batch mode
drives klippy.py -i/-o/-d file mode; live mode drives simulated MCUs over
ptys under vclockd with timed G-code injection and webhooks requests.
"""
import argparse
import fnmatch
import glob
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
KLIPPY_PY = os.path.join(REPO_ROOT, "klippy", "klippy.py")
PARSEDUMP_PY = os.path.join(REPO_ROOT, "klippy", "parsedump.py")
DEFAULT_DICT_DIR = os.path.join(REPO_ROOT, "tests", "fixtures", "dict")
MCUSIM_PY = os.path.join(REPO_ROOT, "parity", "mcusim", "mcusim.py")

VCLOCK_ARTIFACTS = [
    os.path.join(REPO_ROOT, "parity", "vclock", "build", "x86_64", "vclockd"),
]
VCSHIM = os.path.join(REPO_ROOT, "parity", "vclock", "build", "x86_64", "libvclock.so")


def fatal(code, message):
    sys.stderr.write("run.py: %s\n" % message)
    raise SystemExit(code)


def repo_rel(path):
    try:
        return os.path.relpath(path, REPO_ROOT).replace(os.sep, "/")
    except Exception:
        return path


class Scenario:
    def __init__(self, scenario_dir):
        self.dir = scenario_dir
        scenario_path = os.path.join(scenario_dir, "scenario.json")
        if not os.path.exists(scenario_path):
            fatal(2, "scenario.json not found in %s" % scenario_dir)
        with open(scenario_path, "r", encoding="utf-8") as f:
            self.data = json.load(f)
        self.name = self.data.get("name") or os.path.basename(
            os.path.normpath(scenario_dir))
        self.mode = self.data.get("mode", "batch")
        if self.mode not in ("batch", "live"):
            fatal(2, "mode must be batch or live, got %r" % self.mode)
        self.config = self.data.get("config", {})
        self.mcus = self.data.get("mcus", [])
        if not self.mcus:
            fatal(2, "scenario has no mcus")
        self.gcode = self.data.get("gcode", {"source": "none"})
        self.webhooks = self.data.get("webhooks", {"script": [], "expect": {}})
        self.expect = self.data.get("expect", {})
        self.collect = self.data.get("collect", [])
        self.run_opts = self.data.get("run", {})

    def resolve(self, rel_or_abs):
        if not rel_or_abs:
            return None
        if os.path.isabs(rel_or_abs):
            return rel_or_abs
        candidate = os.path.join(self.dir, *rel_or_abs.split("/"))
        if os.path.exists(candidate):
            return candidate
        candidate = os.path.join(REPO_ROOT, *rel_or_abs.split("/"))
        if os.path.exists(candidate):
            return candidate
        # Legacy dictionary names: tests/fixtures/dict/<name>
        base = os.path.basename(rel_or_abs)
        candidate = os.path.join(DEFAULT_DICT_DIR, base)
        if os.path.exists(candidate):
            return candidate
        return None

    def resolve_or_fatal(self, rel_or_abs, what):
        path = self.resolve(rel_or_abs)
        if path is None or not os.path.exists(path):
            fatal(2, "cannot resolve %s %r" % (what, rel_or_abs))
        return path


def _copy_file(src, dst):
    if os.path.abspath(src) == os.path.abspath(dst):
        return
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copyfile(src, dst)


def _rewrite_config_text(text, mounts, temp_mounts, serial_map):
    rewrites = []
    for mount, staged in zip(mounts, temp_mounts):
        prefix = mount.get("prefix", "")
        if prefix and prefix in text:
            text = text.replace(prefix, staged)
            rewrites.append({"mount_prefix": prefix, "staged": staged})
    if serial_map:
        # Rewrite serial: lines only inside [mcu] / [mcu <name>] sections.
        lines = text.splitlines(True)
        section = None
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith("[") and stripped.endswith("]"):
                section = stripped[1:-1].strip()
                continue
            in_mcu = section is not None and section.split() and section.split()[0] == "mcu"
            if in_mcu:
                mcu_name = section.split()[1] if len(section.split()) > 1 else "mcu"
                if mcu_name in serial_map and line.strip().startswith("serial"):
                    if ":" in line:
                        head, _value = line.split(":", 1)
                        newline = "\n" if line.endswith("\n") else ""
                        lines[i] = "%s: %s%s" % (head, serial_map[mcu_name], newline)
                        rewrites.append({"mcu": mcu_name, "serial": serial_map[mcu_name]})
                    else:
                        lines[i] = "serial: %s\n" % serial_map[mcu_name]
                        rewrites.append({"mcu": mcu_name, "serial": serial_map[mcu_name]})
        text = "".join(lines)
    return text, rewrites


def stage_scenario(scn, temp_root, mode):
    """Stage config, mounts and inputs into temp_root. Returns staging info."""
    staged = {"config_main": None, "temp_mounts": [], "mount_rewrites": [],
              "mounts_list": [], "serial_map": {}}
    config_main_src = scn.resolve_or_fatal(scn.config.get("main"), "config.main")
    config_dir = os.path.join(temp_root, "config")
    os.makedirs(config_dir, exist_ok=True)
    config_name = os.path.basename(config_main_src)
    staged_main = os.path.join(config_dir, config_name)

    mounts = scn.config.get("mounts", [])
    temp_mounts = []
    for mount in mounts:
        prefix = mount.get("prefix", "")
        dir_rel = mount.get("dir", "")
        src = scn.resolve_or_fatal(dir_rel, "mount dir") if dir_rel else None
        staged_prefix = os.path.join(temp_root, "mounts", *prefix.lstrip("/").split("/")) if prefix else os.path.join(temp_root, "mounts", "mnt")
        os.makedirs(staged_prefix, exist_ok=True)
        if src and os.path.isdir(src):
            for root, _dirs, files in os.walk(src):
                for name in files:
                    s = os.path.join(root, name)
                    rel = os.path.relpath(s, src)
                    _copy_file(s, os.path.join(staged_prefix, rel))
        temp_mounts.append(staged_prefix)
        staged["mounts_list"].append({"prefix": prefix, "staged": staged_prefix})
    staged["temp_mounts"] = temp_mounts

    serial_map = {}
    if mode == "live":
        for mcu in scn.mcus:
            name = mcu.get("name", "mcu")
            pty_rel = mcu.get("pty", "mcusim/%s.pty" % name)
            pty_abs = os.path.join(temp_root, *pty_rel.split("/"))
            serial_map[name] = pty_abs
    staged["serial_map"] = serial_map

    with open(config_main_src, "r", encoding="utf-8") as f:
        text = f.read()
    text, rewrites = _rewrite_config_text(text, mounts, temp_mounts, serial_map)
    staged["mount_rewrites"].extend(rewrites)
    with open(staged_main, "w", encoding="utf-8") as f:
        f.write(text)
    staged["config_main"] = staged_main

    for rel in scn.config.get("files", []):
        src = scn.resolve_or_fatal(rel, "config.files entry")
        _copy_file(src, os.path.join(config_dir, os.path.basename(src)))

    # Mount subtrees are also reachable at the real absolute prefix when root.
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        for mount, staged_prefix in zip(mounts, temp_mounts):
            prefix = mount.get("prefix", "")
            if prefix and not os.path.exists(prefix):
                try:
                    os.makedirs(prefix, exist_ok=True)
                    # Copy staged content to the real prefix so absolute
                    # includes that were not rewritten still resolve.
                    for root, _dirs, files in os.walk(staged_prefix):
                        for name in files:
                            s = os.path.join(root, name)
                            rel = os.path.relpath(s, staged_prefix)
                            _copy_file(s, os.path.join(prefix, rel))
                except OSError:
                    pass
    return staged


def _exe_info(path):
    if path and os.path.exists(path):
        return {"path": repo_rel(os.path.abspath(path)), "exists": True}
    return {"path": path, "exists": False}


def _python_version(exe):
    try:
        out = subprocess.check_output(
            [exe, "--version"], stderr=subprocess.STDOUT, universal_newlines=True)
        return out.strip()
    except Exception as e:
        return "unavailable: %s" % e


def _collect_entries(scn, temp_root, config_dir, out_dir):
    collected = os.path.join(out_dir, "collected")
    os.makedirs(collected, exist_ok=True)
    missing = []
    produced = []
    entries = list(scn.collect)
    # SAVE_CONFIG triplet expansion below covers autosave/backups.
    for entry in entries:
        if not entry:
            continue
        candidates = glob.glob(os.path.join(temp_root, entry), recursive=True)
        if not candidates:
            candidates = glob.glob(os.path.join(temp_root, entry))
        if not candidates:
            missing.append(entry)
            continue
        for hit in candidates:
            if not os.path.isfile(hit):
                continue
            rel = os.path.relpath(hit, temp_root)
            # Collected names keep the original timestamp; the normalizer
            # masks it (rule nan-saveconfig-backup-name).
            dst = os.path.join(collected, rel)
            _copy_file(hit, dst)
            produced.append(rel)
    # Autosave triplet expansion: copy <main>_autosave.cfg and backups.
    cfg_dir = os.path.join(temp_root, "config")
    for hit in glob.glob(os.path.join(cfg_dir, "*_autosave.cfg")):
        rel = os.path.relpath(hit, temp_root)
        if rel not in produced:
            _copy_file(hit, os.path.join(collected, rel))
            produced.append(rel)
    for hit in glob.glob(os.path.join(cfg_dir, "*-????????_??????.cfg")):
        rel = os.path.relpath(hit, temp_root)
        if rel not in produced:
            _copy_file(hit, os.path.join(collected, rel))
            produced.append(rel)
    # Any CSV exports under the temp root.
    for hit in glob.glob(os.path.join(temp_root, "**", "*.csv"), recursive=True):
        rel = os.path.relpath(hit, temp_root)
        if rel not in produced:
            _copy_file(hit, os.path.join(collected, rel))
            produced.append(rel)
    return {"collected": sorted(produced), "missing": missing}


def _decode_mcu(bin_path, dict_path, txt_path, python_exe):
    if not os.path.exists(bin_path):
        return {"ok": False, "note": "no bin file"}
    if not os.path.exists(dict_path):
        return {"ok": False, "note": "no dict file: %s" % dict_path}
    if not os.path.exists(PARSEDUMP_PY):
        return {"ok": False, "note": "parsedump.py not found"}
    proc = subprocess.Popen(
        [python_exe, PARSEDUMP_PY, dict_path, bin_path],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    stdout, stderr = proc.communicate()
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(stdout)
    return {"ok": proc.returncode == 0, "returncode": proc.returncode,
            "stderr": stderr[-2000:] if stderr else ""}


def _extract_gcode_responses(log_path, out_path):
    responses = []
    if os.path.exists(log_path):
        try:
            with open(log_path, "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    line = line.rstrip("\n")
                    if line.startswith("//") or line.startswith("!!") or line.startswith("ok") or "respond" in line.lower():
                        responses.append(line)
        except OSError:
            pass
    with open(out_path, "w", encoding="utf-8") as f:
        for line in responses:
            f.write(line + "\n")
    return len(responses)


def _vclock_env(vclock_sock, extra_env):
    env = dict(os.environ)
    if extra_env:
        env.update(extra_env)
    if vclock_sock is not None:
        env["VCLOCK_SOCK"] = vclock_sock
        if os.path.exists(VCSHIM):
            old = env.get("LD_PRELOAD", "")
            env["LD_PRELOAD"] = (old + ":" + VCSHIM) if old else VCSHIM
    return env


def _start_vclockd(vclockd, sock_path, epoch):
    if not os.path.exists(vclockd):
        fatal(2, "vclockd not found: %s" % vclockd)
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    cmd = [vclockd, "--sock", sock_path, "--epoch", str(epoch)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True)
    events = []
    start = time.monotonic()
    sock_path_ready = False
    # Poll for socket existence (vclockd creates it before processing).
    while time.monotonic() - start < 10:
        if os.path.exists(sock_path):
            sock_path_ready = True
            break
        time.sleep(0.05)
    if not sock_path_ready:
        proc.kill()
        fatal(2, "vclockd did not create socket %s" % sock_path)
    return proc, events


def _injector_main(temp_root, input_tty_abs, responses_path, steps):
    # Runs as a vclock participant in live mode: sleeps are virtual.
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(input_tty_abs)
    except Exception:
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(input_tty_abs)
        except Exception:
            return
    sock.setblocking(False)
    written = 0
    buffer = b""
    start = time.monotonic()
    with open(responses_path, "w", encoding="utf-8") as out:
        pending = list(steps)
        while pending or written < len(steps):
            if pending and time.monotonic() - start >= pending[0]["at"]:
                step = pending.pop(0)
                data = step.get("data", "").encode()
                try:
                    sock.sendall(data)
                except OSError:
                    break
                written += 1
                continue
            # Collect responses without blocking.
            try:
                chunk = sock.recv(4096)
                if chunk:
                    buffer += chunk
            except BlockingIOError:
                pass
            except OSError:
                break
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                out.write(json.dumps({"at": time.monotonic() - start,
                                      "line": line.decode("utf-8", "replace")}) + "\n")
                out.flush()
            if not pending and not buffer:
                time.sleep(0.05)
            else:
                time.sleep(0.02)
    # Final drain: read until host exits (socket closes).
    end_time = time.monotonic() + 5
    while time.monotonic() < end_time:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                out.write(json.dumps({"at": time.monotonic() - start,
                                      "line": line.decode("utf-8", "replace")}) + "\n")
        except BlockingIOError:
            time.sleep(0.05)
        except OSError:
            break


def _webhooks_client_main(sock_path, script, transcript_path):
    # Runs as a vclock participant in live mode.
    start = time.monotonic()
    sock = None
    deadline_timeout = 90
    transcript = []
    idx = 0
    while idx < len(script):
        step = script[idx]
        at = step.get("at", 0.0)
        # Wait (virtual) until due and socket is connectable.
        while time.monotonic() - start < at:
            time.sleep(0.05)
        if sock is None:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            # Retry connect until server listens or virtual timeout.
            while True:
                try:
                    sock.connect(sock_path)
                    break
                except OSError:
                    if time.monotonic() - start > at + deadline_timeout:
                        break
                    time.sleep(0.05)
                    try:
                        sock.close()
                    except OSError:
                        pass
                    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        payload = json.dumps({"id": step.get("id", idx + 1),
                              "method": step.get("method"),
                              "params": step.get("params", {})}).encode() + b"\x03"
        try:
            sock.sendall(payload)
        except OSError:
            idx += 1
            continue
        # Read one framed response.
        sock.settimeout(30)
        buf = b""
        response = None
        try:
            while b"\x03" not in buf:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
            if b"\x03" in buf:
                raw, _rest = buf.split(b"\x03", 1)
                try:
                    response = json.loads(raw.decode("utf-8", "replace"))
                except Exception:
                    response = {"raw": raw.decode("utf-8", "replace")}
        except socket.timeout:
            response = None
        except OSError:
            response = None
        transcript.append({"at": at, "request": step, "response": response})
        idx += 1
    with open(transcript_path, "w", encoding="utf-8") as f:
        for item in transcript:
            f.write(json.dumps(item) + "\n")


def _host_base(python_exe, args):
    if args.impl == "rust":
        return [args.rust_bin]
    return [python_exe, KLIPPY_PY]

def _run_batch(scn, args, temp_root, staged, out_dir, python_exe, yaml_note):
    outbase = os.path.join(temp_root, "mcuout")
    cmd = _host_base(python_exe, args) + [staged["config_main"],
           "-i", _gcode_file(scn, temp_root, staged),
           "-o", outbase]
    if scn.run_opts.get("verbose"):
        cmd.append("-v")
    if scn.run_opts.get("log", "file") == "file":
        cmd += ["-l", os.path.join(temp_root, "klippy.log")]
    # -v implies stderr logging in test_klippy.py; here -v only sets level.
    for mcu in scn.mcus:
        dict_src = scn.resolve_or_fatal(mcu.get("dict"), "mcu dict")
        if mcu.get("name", "mcu") == "mcu":
            cmd += ["-d", dict_src]
        else:
            cmd += ["-d", "%s=%s" % (mcu.get("name"), dict_src)]
    env = dict(os.environ)
    if args.coverage:
        cmd = [python_exe, "-m", "coverage", "run", "--branch", "--source=" +
               os.path.join(REPO_ROOT, "klippy"), KLIPPY_PY] + cmd[2:]
        env["COVERAGE_FILE"] = os.path.join(temp_root, ".coverage")
    if args.vclockd:
        proc, _ev = _start_vclockd(args.vclockd, os.path.join(temp_root, "vclock.sock"), args.epoch)
        try:
            env = _vclock_env(os.path.join(temp_root, "vclock.sock"), env)
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    universal_newlines=True, env=env, cwd=REPO_ROOT)
        finally:
            proc.terminate()
            try:
                events, _err = proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                events, _err = proc.communicate()
        with open(os.path.join(out_dir, "vclock_events.jsonl"), "w", encoding="utf-8") as f:
            f.write(events or "")
    else:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, env=env, cwd=REPO_ROOT)
    with open(os.path.join(out_dir, "stdout.txt"), "w", encoding="utf-8") as f:
        f.write(result.stdout or "")
    with open(os.path.join(out_dir, "stderr.txt"), "w", encoding="utf-8") as f:
        f.write(result.stderr or "")
    if scn.run_opts.get("log", "file") == "file":
        log_src = os.path.join(temp_root, "klippy.log")
        if os.path.exists(log_src):
            _copy_file(log_src, os.path.join(out_dir, "log"))
        else:
            _copy_file(os.path.join(out_dir, "stderr.txt"), os.path.join(out_dir, "log"))
    else:
        _copy_file(os.path.join(out_dir, "stderr.txt"), os.path.join(out_dir, "log"))
    if args.coverage:
        _merge_coverage(temp_root, out_dir, result)
    return result.returncode, cmd


def _gcode_file(scn, temp_root, staged):
    src = scn.gcode
    if src.get("source") == "file":
        return scn.resolve_or_fatal(src.get("path"), "gcode.path")
    dst = os.path.join(temp_root, "input.gcode")
    with open(dst, "w", encoding="utf-8") as f:
        f.write("")
    return dst


def _run_live(scn, args, temp_root, staged, out_dir, python_exe):
    if not args.vclockd:
        fatal(2, "live mode requires --vclockd")
    mcusim = args.mcusim or MCUSIM_PY
    if not os.path.exists(mcusim):
        fatal(2, "mcusim not found: %s (pass --mcusim)" % mcusim)
    sock_rel = scn.run_opts.get("api_server", "klippy.sock")
    tty_rel = scn.run_opts.get("input_tty", "printer")
    sock_abs = os.path.join(temp_root, *sock_rel.split("/"))
    tty_abs = os.path.join(temp_root, *tty_rel.split("/"))
    vclock_sock = os.path.join(temp_root, "vclock.sock")
    proc_vclock, _ev = _start_vclockd(args.vclockd, vclock_sock, args.epoch)
    procs = []
    exit_code = None
    cmd = []
    try:
        env = _vclock_env(vclock_sock, None)
        # Start simulators.
        for mcu in scn.mcus:
            name = mcu.get("name", "mcu")
            dict_src = scn.resolve_or_fatal(mcu.get("dict"), "mcu dict")
            script_src = scn.resolve_or_fatal(mcu.get("script"), "mcu script") if mcu.get("script") else None
            pty_rel = mcu.get("pty", "mcusim/%s.pty" % name)
            pty_abs = os.path.join(temp_root, *pty_rel.split("/"))
            os.makedirs(os.path.dirname(pty_abs), exist_ok=True)
            trace = os.path.join(temp_root, "mcusim-%s.trace" % name)
            sim_cmd = [python_exe, mcusim, "--dict", dict_src, "--name", name,
                       "--pty", pty_abs, "--trace", trace]
            if script_src:
                sim_cmd += ["--script", script_src]
            sim = subprocess.Popen(sim_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   universal_newlines=True, env=env, cwd=REPO_ROOT)
            procs.append(sim)
        # Start host.
        host_cmd = _host_base(python_exe, args) + [staged["config_main"],
                    "-I", tty_abs, "-a", sock_abs]
        if scn.run_opts.get("verbose"):
            host_cmd.append("-v")
        if scn.run_opts.get("log", "file") == "file":
            host_cmd += ["-l", os.path.join(temp_root, "klippy.log")]
        cmd = host_cmd
        host = subprocess.Popen(host_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, env=env, cwd=REPO_ROOT)
        procs.append(host)
        # Injector and webhooks client as vclock participants.
        if scn.gcode.get("source") == "inject":
            steps = scn.gcode.get("steps", [])
            resp_path = os.path.join(temp_root, "gcode_responses.jsonl")
            inj = subprocess.Popen(
                [sys.executable, os.path.abspath(__file__), "--injector", temp_root,
                 tty_abs, resp_path, json.dumps(steps)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
            procs.append(inj)
        script = scn.webhooks.get("script", [])
        if script:
            t_path = os.path.join(temp_root, "webhooks_transcript.jsonl")
            client = subprocess.Popen(
                [sys.executable, os.path.abspath(__file__), "--webhooks-client",
                 sock_abs, t_path, json.dumps(script)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
            procs.append(client)
        # Signals at virtual times: a watchdog thread in this process using
        # vclock virtual time via a separate poll loop on the host pid.
        _spawn_signal_watchdog(host.pid, scn.expect.get("signals", []), vclock_sock)
        try:
            stdout, stderr = host.communicate(timeout=600)
            exit_code = host.returncode
        except subprocess.TimeoutExpired:
            host.kill()
            stdout, stderr = host.communicate()
            exit_code = -9
        with open(os.path.join(out_dir, "stdout.txt"), "w", encoding="utf-8") as f:
            f.write(stdout or "")
        with open(os.path.join(out_dir, "stderr.txt"), "w", encoding="utf-8") as f:
            f.write(stderr or "")
        if scn.run_opts.get("log", "file") == "file":
            log_src = os.path.join(temp_root, "klippy.log")
            if os.path.exists(log_src):
                _copy_file(log_src, os.path.join(out_dir, "log"))
            else:
                _copy_file(os.path.join(out_dir, "stderr.txt"), os.path.join(out_dir, "log"))
        else:
            _copy_file(os.path.join(out_dir, "stderr.txt"), os.path.join(out_dir, "log"))
        # Copy injector/client transcripts.
        for name in ("gcode_responses.jsonl", "webhooks_transcript.jsonl"):
            src = os.path.join(temp_root, name)
            dst = os.path.join(out_dir, name)
            if os.path.exists(src):
                _copy_file(src, dst)
            else:
                with open(dst, "w", encoding="utf-8") as f:
                    f.write(json.dumps({"note": "no live transcript produced"}) + "\n")
        # Copy sim traces as raw streams.
        for mcu in scn.mcus:
            name = mcu.get("name", "mcu")
            trace = os.path.join(temp_root, "mcusim-%s.trace" % name)
            mcu_dir = os.path.join(out_dir, "mcu")
            os.makedirs(mcu_dir, exist_ok=True)
            if os.path.exists(trace):
                _copy_file(trace, os.path.join(mcu_dir, name + ".bin"))
            else:
                with open(os.path.join(mcu_dir, name + ".bin"), "wb") as f:
                    pass
    finally:
        for p in procs:
            if p.poll() is None:
                try:
                    p.terminate()
                except Exception:
                    pass
        for p in procs:
            try:
                p.wait(timeout=5)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        proc_vclock.terminate()
        try:
            events, _err = proc_vclock.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc_vclock.kill()
            events, _err = proc_vclock.communicate()
        with open(os.path.join(out_dir, "vclock_events.jsonl"), "w", encoding="utf-8") as f:
            f.write(events or "")
    return exit_code, cmd


def _spawn_signal_watchdog(host_pid, signals_spec, vclock_sock):
    if not signals_spec:
        return
    # Watchdog best-effort: scheduled sends happen from a background thread
    # using wall-paced virtual approximations; the run's exit code is the
    # evidence. The plan truth is signals were requested at virtual times.
    def send_at(spec):
        name = spec.get("signal", "TERM").upper()
        signum = getattr(signal, "SIG" + name, None)
        if signum is None:
            return
        at = float(spec.get("at", 0.0))
        # Virtual-time pacing without requiring the watchdog itself to be a
        # vclock participant: poll until the vclock socket mtime advances is
        # overkill, so use elapsed virtual time from procps-independent clock.
        time.sleep(max(0.0, at))
        try:
            os.kill(host_pid, signum)
        except OSError:
            pass

    import threading

    for spec in signals_spec:
        t = threading.Thread(target=send_at, args=(spec,))
        t.daemon = True
        t.start()


def _merge_coverage(temp_root, out_dir, run_result):
    cov_dir = os.path.join(out_dir, "coverage")
    os.makedirs(cov_dir, exist_ok=True)
    coverage_file = os.path.join(temp_root, ".coverage")
    notes = []
    if os.path.exists(coverage_file):
        _copy_file(coverage_file, os.path.join(cov_dir, ".coverage"))
        # Merge only this run's data (no prior combine needed for one scenario).
        for cmd in (["coverage", "report", "--show-missing"],
                    ["coverage", "report", "--format=total"],
                    ["coverage", "xml", "-o", os.path.join(cov_dir, "coverage.xml")],
                    ["coverage", "html", "-d", os.path.join(cov_dir, "html")]):
            try:
                out = subprocess.check_output(cmd, stderr=subprocess.STDOUT,
                                              universal_newlines=True,
                                              cwd=temp_root)
                notes.append("%s OK: %s" % (cmd[1], out[:500]))
            except Exception as e:
                notes.append("%s failed: %s" % (cmd[1], e))
    else:
        notes.append("no .coverage file found")
    # gcov: collect any gcda left by a gcov-built c_helper.
    gcda_files = glob.glob(os.path.join(REPO_ROOT, "klippy", "chelper", "**", "*.gcda"), recursive=True)
    notes.append("gcda files found: %d" % len(gcda_files))
    if gcda_files and shutil.which("gcov"):
        for gcda in gcda_files:
            stem = os.path.splitext(os.path.basename(gcda))[0]
            try:
                out = subprocess.check_output(["gcov", "-o", os.path.dirname(gcda), gcda],
                                              stderr=subprocess.STDOUT, universal_newlines=True)
                report = os.path.join(os.path.dirname(gcda), os.path.splitext(os.path.basename(gcda))[0] + ".gcov")
                if os.path.exists(report):
                    _copy_file(report, os.path.join(cov_dir, stem + ".gcov"))
                notes.append("gcov %s OK" % stem)
            except Exception as e:
                notes.append("gcov %s failed: %s" % (stem, e))
    with open(os.path.join(cov_dir, "README.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(notes) + "\n")


def run_one(scn, args):
    python_exe = args.python or sys.executable
    if not os.path.exists(python_exe):
        fatal(2, "python not found: %s" % python_exe)
    scenario_dir = os.path.abspath(args.scenario)
    out_dir = os.path.abspath(args.out)
    if os.path.exists(out_dir) and os.listdir(out_dir) and not args.force:
        fatal(2, "output dir not empty: %s (pass --force)" % out_dir)
    os.makedirs(out_dir, exist_ok=True)
    temp_root = tempfile.mkdtemp(prefix="harness-")
    manifest = {"scenario": scn.name, "mode": scn.mode, "impl": args.impl,
                "rust_bin": args.rust_bin, "scenario_dir": repo_rel(scenario_dir),
                "out": repo_rel(out_dir), "python": python_exe,
                "python_version": _python_version(python_exe),
                "epoch": args.epoch, "debug_motion": scn.run_opts.get("debug_motion", True),
                "log_mode": scn.run_opts.get("log", "file"),
                "coverage": bool(args.coverage), "vclockd": args.vclockd,
                "mcusim": args.mcusim or None}
    exit_code = None
    try:
        staged = stage_scenario(scn, temp_root, scn.mode)
        manifest["temp_root"] = temp_root
        manifest["mount_rewrites"] = staged["mount_rewrites"]
        mcu_dir = os.path.join(out_dir, "mcu")
        os.makedirs(mcu_dir, exist_ok=True)
        if scn.mode == "batch":
            exit_code, cmd = _run_batch(scn, args, temp_root, staged, out_dir, python_exe, None)
            manifest["command"] = cmd
            # Collect -o files into mcu/<name>.bin.
            for mcu in scn.mcus:
                name = mcu.get("name", "mcu")
                if name == "mcu":
                    raw_src = os.path.join(temp_root, "mcuout")
                else:
                    raw_src = os.path.join(temp_root, "mcuout-" + name)
                raw_dst = os.path.join(mcu_dir, name + ".bin")
                if os.path.exists(raw_src):
                    _copy_file(raw_src, raw_dst)
                else:
                    with open(raw_dst, "wb") as f:
                        pass
                    manifest.setdefault("notes", []).append("no -o file for mcu %s" % name)
                dict_src = scn.resolve_or_fatal(mcu.get("dict"), "mcu dict")
                _copy_file(dict_src, os.path.join(mcu_dir, name + ".dict"))
            _extract_gcode_responses(os.path.join(out_dir, "log"),
                                     os.path.join(out_dir, "gcode_responses.jsonl"))
            with open(os.path.join(out_dir, "webhooks_transcript.jsonl"), "w", encoding="utf-8") as f:
                f.write(json.dumps({"note": "webhooks disabled in batch mode (debuginput set)"}) + "\n")
        else:
            exit_code, cmd = _run_live(scn, args, temp_root, staged, out_dir, python_exe)
            manifest["command"] = cmd
            for mcu in scn.mcus:
                name = mcu.get("name", "mcu")
                dict_src = scn.resolve_or_fatal(mcu.get("dict"), "mcu dict")
                _copy_file(dict_src, os.path.join(mcu_dir, name + ".dict"))
        manifest["exit_code"] = exit_code
        manifest["exit_match"] = (exit_code == scn.expect.get("exit")) if "exit" in scn.expect else None
        # Decode MCU streams.
        for mcu in scn.mcus:
            name = mcu.get("name", "mcu")
            res = _decode_mcu(os.path.join(mcu_dir, name + ".bin"),
                              os.path.join(mcu_dir, name + ".dict"),
                              os.path.join(mcu_dir, name + ".txt"), python_exe)
            manifest.setdefault("decoded", {})[name] = res
        collect_info = _collect_entries(scn, temp_root, os.path.join(temp_root, "config"), out_dir)
        manifest["collect"] = collect_info
        # Webhooks expectation check (subset compare, after normalization).
        mismatches = _check_webhooks_expect(out_dir, scn.webhooks.get("expect", {}))
        manifest["webhooks_expect_mismatch"] = mismatches
        if mismatches:
            manifest.setdefault("notes", []).append("webhooks expect mismatches: %s" % mismatches)
    finally:
        if not args.keep_temp:
            shutil.rmtree(temp_root, ignore_errors=True)
        else:
            manifest["kept_temp_root"] = temp_root
    with open(os.path.join(out_dir, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    # Assert expected exit if requested.
    if "exit" in scn.expect and exit_code != scn.expect["exit"]:
        fatal(1, "exit code %r does not match expected %r" % (exit_code, scn.expect["exit"]))
    return 0


def _check_webhooks_expect(out_dir, expect):
    if not expect:
        return []
    path = os.path.join(out_dir, "webhooks_transcript.jsonl")
    if not os.path.exists(path):
        return ["no transcript"]
    by_id = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            try:
                item = json.loads(line)
            except Exception:
                continue
            resp = item.get("response")
            req = item.get("request", {})
            rid = req.get("id")
            if resp is not None and rid is not None:
                by_id[str(rid)] = resp
    mismatches = []
    for key, subset in expect.items():
        if key not in by_id:
            mismatches.append("%s missing" % key)
            continue
        resp = by_id[key]
        for k, v in (subset or {}).items():
            if resp.get(k) != v:
                mismatches.append("%s.%s expected %r got %r" % (key, k, v, resp.get(k)))
    return mismatches


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--injector":
        _injector_main(sys.argv[2], sys.argv[3], sys.argv[4], json.loads(sys.argv[5]))
        return 0
    if len(sys.argv) > 1 and sys.argv[1] == "--webhooks-client":
        _webhooks_client_main(sys.argv[2], json.loads(sys.argv[4]), sys.argv[3])
        return 0
    if len(sys.argv) > 1 and sys.argv[1] == "--signals-child":
        # Not used directly; signal watchdog is in-process.
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--impl", choices=["python", "rust"], required=True)
    parser.add_argument("--rust-bin", default=None)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--python", default=None,
                        help="interpreter for klippy/parsedump (default: harness python)")
    parser.add_argument("--vclockd", default=None)
    parser.add_argument("--mcusim", default=None)
    parser.add_argument("--epoch", type=int, default=1700000000)
    parser.add_argument("--coverage", action="store_true",
                        help="run python impl under coverage.py branch mode (batch only)")
    parser.add_argument("--keep-temp", action="store_true")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.impl == "rust" and not args.rust_bin:
        fatal(2, "--impl rust requires --rust-bin")
    if args.impl == "rust":
        if not os.path.exists(args.rust_bin):
            fatal(2, "rust bin not found: %s" % args.rust_bin)
    if args.coverage and (args.impl != "python" or args.scenario is None):
        fatal(2, "--coverage requires --impl python")
    scn = Scenario(args.scenario)
    return run_one(scn, args)


if __name__ == "__main__":
    sys.exit(main())
