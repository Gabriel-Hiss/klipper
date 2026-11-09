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
import select
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
        candidate = os.path.join(REPO_ROOT, "parity", "scenarios", *rel_or_abs.split("/"))
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
        # Rewrite serial: lines inside [mcu] / [mcu <name>] sections to
        # the staged pty of that MCU. Both uart and pipe transports take
        # a pty slave path (the Linux MCU firmware opens a pty too, see
        # src/linux/console.c), so kind never changes the rewrite.
        # Sections without a scenario MCU entry (no sim behind them) keep
        # their serial line: rewriting them would point the host at a
        # path nothing serves and fail the run at connect time.
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


def _start_vclockd(vclockd, sock_path, epoch, expect=None):
    """Start vclockd. Without `expect` its stdout is text for
    communicate(); with `expect` (live mode) it is an unbuffered byte
    pipe that _VclockEvents reads while participants register."""
    if not os.path.exists(vclockd):
        fatal(2, "vclockd not found: %s" % vclockd)
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    cmd = [vclockd, "--sock", sock_path, "--epoch", str(epoch)]
    if expect is not None:
        cmd += ["--expect", str(expect)]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                bufsize=0)
    else:
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


class _VclockEvents:
    """Reads vclockd's JSON event lines while the live participants start.

    Live runs start vclockd with --expect N: no participant runs and
    virtual time stays at 0 until all N processes have registered. The
    harness launches them one at a time and waits for each `register`
    event, so registration order (vclockd's round-robin order) and the
    virtual time every participant starts at are the same on every run
    and for both implementations, however long each process takes to
    start in wall time."""

    def __init__(self, proc):
        self.proc = proc
        self.data = b""
        self.registered = set()
        self.names = {}

    def _parse(self):
        for line in self.data.split(b"\n"):
            try:
                ev = json.loads(line.decode("utf-8", "replace"))
            except ValueError:
                continue
            if ev.get("event") == "register":
                self.registered.add(ev.get("pid"))

    def wait_register(self, child, what, log_path, timeout=60.0):
        self.names[child.pid] = what
        deadline = time.monotonic() + timeout
        fd = self.proc.stdout.fileno()
        while child.pid not in self.registered:
            if child.poll() is not None:
                fatal(2, "%s exited (%r) before registering with vclockd: %s"
                      % (what, child.returncode, _tail(log_path)))
            if self.proc.poll() is not None:
                fatal(2, "vclockd exited while waiting for %s: %s"
                      % (what, self.data.decode("utf-8", "replace")[-2000:]))
            if time.monotonic() > deadline:
                fatal(2, "%s (pid %d) never registered with vclockd; it must load "
                      "libvclock.so through LD_PRELOAD (dynamically linked)"
                      % (what, child.pid))
            r, _, _ = select.select([fd], [], [], 0.1)
            if r:
                chunk = os.read(fd, 65536)
                if chunk:
                    self.data += chunk
                    self._parse()

    def finish(self):
        """All event lines, with every pid replaced by the participant
        name ("host", "mcusim mcu", ...). Processes the participants fork
        are named "child 1", "child 2", ... in order of first appearance.
        Register lines keep vclockd's order (fixed by the launch
        protocol); exit lines are sorted by name, because after run.end_at
        the order in which vclockd notices the processes end depends on
        wall-clock teardown. Pids differ per run; the names do not."""
        rest, _err = self.proc.communicate(timeout=5)
        text = (self.data + (rest or b"")).decode("utf-8", "replace")
        names = dict(self.names)
        children = [0]

        def name(pid):
            if pid not in names:
                children[0] += 1
                names[pid] = "child %d" % children[0]
            return names[pid]

        out = []
        exits = []
        for line in text.splitlines():
            try:
                ev = json.loads(line)
            except ValueError:
                out.append(line)
                continue
            if "pid" in ev:
                ev["pid"] = name(ev["pid"])
            for thread in ev.get("threads", []):
                if "pid" in thread:
                    thread["pid"] = name(thread["pid"])
            if ev.get("event") == "exit":
                exits.append(ev)
                continue
            out.append(json.dumps(ev, separators=(",", ":")))
        exit_lines = [json.dumps(ev, separators=(",", ":"))
                      for ev in sorted(exits, key=lambda ev: str(ev["pid"]))]
        done = [i for i, line in enumerate(out) if line == '{"event":"done"}']
        at = done[0] if done else len(out)
        out[at:at] = exit_lines
        return "".join(line + "\n" for line in out)


def _tail(path, size=2000):
    if not path:
        return ""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()[-size:]
    except OSError:
        return ""


def _injector_main(temp_root, input_tty_abs, responses_path, steps):
    # Runs as a vclock participant: time.monotonic() is virtual seconds
    # since vclockd started the clock, which is the scenario timeline
    # (`at`). The input tty is the pty slave klippy.py -I creates
    # (util.create_pty); every participant starts at virtual 0, so the
    # host may not have created it yet on the first try.
    first_at = steps[0]["at"] if steps else 0.0
    while True:
        try:
            fd = os.open(input_tty_abs, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            break
        except OSError as e:
            if time.monotonic() > first_at:
                sys.stderr.write("injector: input tty %s missing at the first step"
                                 " (virtual %.3f): %s\n" % (input_tty_abs, first_at, e))
                return 1
            time.sleep(0.05)
    pending = list(steps)
    buffer = b""
    with open(responses_path, "w", encoding="utf-8") as out:
        def emit(raw):
            # create_pty only clears ECHO, so ONLCR turns host "\n" into
            # "\r\n": strip the "\r" so transcripts compare on content.
            out.write(json.dumps({"at": time.monotonic(),
                                  "line": raw.decode("utf-8", "replace").rstrip("\r")}) + "\n")
            out.flush()
        while True:
            if pending and time.monotonic() >= pending[0]["at"]:
                os.write(fd, pending.pop(0).get("data", "").encode())
                continue
            timeout = max(0.0, pending[0]["at"] - time.monotonic()) if pending else None
            r, _, _ = select.select([fd], [], [], timeout)
            if not r:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                # EIO: the host exited and closed the pty master.
                break
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                emit(line)
        if buffer:
            emit(buffer)
    os.close(fd)
    return 0



def _webhooks_client_main(sock_path, script, transcript_path):
    # Runs as a vclock participant in live mode. Request `at` times are
    # absolute virtual seconds since process start: compare
    # time.monotonic() directly, like the injector.
    sock = None
    deadline_timeout = 90
    transcript = []
    idx = 0
    while idx < len(script):
        step = script[idx]
        at = step.get("at", 0.0)
        # Wait (virtual) until due and socket is connectable.
        if time.monotonic() < at:
            time.sleep(at - time.monotonic())
        if sock is None:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            # Retry connect until server listens or virtual timeout.
            while True:
                try:
                    sock.connect(sock_path)
                    break
                except OSError:
                    if time.monotonic() > at + deadline_timeout:
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
    end_at = scn.run_opts.get("end_at")
    if end_at is None:
        fatal(2, "live scenario needs run.end_at (see SCENARIO.md)")
    sock_rel = scn.run_opts.get("api_server", "klippy.sock")
    tty_rel = scn.run_opts.get("input_tty", "printer")
    sock_abs = os.path.join(temp_root, *sock_rel.split("/"))
    tty_abs = os.path.join(temp_root, *tty_rel.split("/"))
    inject = scn.gcode.get("source") == "inject"
    script = scn.webhooks.get("script", [])
    # Signals and the end condition share one participant.
    signal_specs = list(scn.expect.get("signals", []))
    signal_specs.append({"at": float(end_at), "signal": "TERM"})
    participants = len(scn.mcus) + 1 + (1 if inject else 0) + (1 if script else 0) + 1
    vclock_sock = os.path.join(temp_root, "vclock.sock")
    proc_vclock, _ev = _start_vclockd(args.vclockd, vclock_sock, args.epoch,
                                      expect=participants)
    clock = _VclockEvents(proc_vclock)
    procs = []
    helper_logs = {}
    exit_code = None
    cmd = []
    wall_timeout = False
    env = _vclock_env(vclock_sock, None)

    def launch(what, argv, log_name, **kw):
        # Helper stdio goes to a temp file collected into the artifacts,
        # so a crashed helper leaves its traceback behind.
        log_path = os.path.join(temp_root, log_name)
        with open(log_path, "w") as log_f:
            child = subprocess.Popen(argv, stdout=log_f, stderr=subprocess.STDOUT,
                                     env=env, **kw)
        procs.append(child)
        helper_logs[log_name] = log_path
        clock.wait_register(child, what, log_path)
        return child

    try:
        for mcu in scn.mcus:
            name = mcu.get("name", "mcu")
            dict_src = scn.resolve_or_fatal(mcu.get("dict"), "mcu dict")
            script_src = scn.resolve_or_fatal(mcu.get("script"), "mcu script")
            pty_abs = staged["serial_map"][name]
            os.makedirs(os.path.dirname(pty_abs), exist_ok=True)
            launch("mcusim %s" % name,
                   [python_exe, mcusim, "--dict", dict_src, "--name", name,
                    "--pty", pty_abs, "--script", script_src,
                    "--trace", os.path.join(temp_root, "mcusim-%s.trace" % name),
                    "--capture", os.path.join(temp_root, "mcusim-%s.bin" % name)],
                   "mcusim-%s.log" % name, cwd=REPO_ROOT)
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
        clock.wait_register(host, "host", None)
        if inject:
            launch("gcode injector",
                   [sys.executable, os.path.abspath(__file__), "--injector", temp_root,
                    tty_abs, os.path.join(temp_root, "gcode_responses.jsonl"),
                    json.dumps(scn.gcode.get("steps", []))],
                   "injector.log")
        if script:
            launch("webhooks client",
                   [sys.executable, os.path.abspath(__file__), "--webhooks-client",
                    sock_abs, os.path.join(temp_root, "webhooks_transcript.jsonl"),
                    json.dumps(script)],
                   "webhooks-client.log")
        # The last registration starts the virtual clock.
        launch("signal timer",
               [sys.executable, os.path.abspath(__file__), "--signals-child",
                str(host.pid), json.dumps(signal_specs)],
               "signals.log")
        try:
            stdout, stderr = host.communicate(timeout=args.wall_timeout)
            exit_code = host.returncode
        except subprocess.TimeoutExpired:
            # Safety net only: run.end_at ends every live run in virtual
            # time; reaching this means the virtual clock stalled.
            wall_timeout = True
            host.kill()
            stdout, stderr = host.communicate()
            exit_code = host.returncode
        with open(os.path.join(out_dir, "stdout.txt"), "w", encoding="utf-8") as f:
            f.write(stdout or "")
        with open(os.path.join(out_dir, "stderr.txt"), "w", encoding="utf-8") as f:
            f.write(stderr or "")
    finally:
        # After the host exits every other participant ends by itself in
        # virtual time (the sims and the injector see the host close its
        # ptys, the client and the signal timer run out of steps), so
        # wait for that before terminating anything: a SIGTERM racing a
        # normal exit makes the helper logs differ between runs. Then
        # terminate leftovers one at a time, in launch order.
        grace = time.monotonic() + 10.0
        for p in procs:
            try:
                p.wait(timeout=max(0.0, grace - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        for p in procs:
            if p.poll() is None:
                p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        proc_vclock.terminate()
        try:
            events = clock.finish()
        except subprocess.TimeoutExpired:
            proc_vclock.kill()
            events = clock.finish()
        with open(os.path.join(out_dir, "vclock_events.jsonl"), "w", encoding="utf-8") as f:
            f.write(events)
    if scn.run_opts.get("log", "file") == "file":
        log_src = os.path.join(temp_root, "klippy.log")
        if os.path.exists(log_src):
            _copy_file(log_src, os.path.join(out_dir, "log"))
        else:
            _copy_file(os.path.join(out_dir, "stderr.txt"), os.path.join(out_dir, "log"))
    else:
        _copy_file(os.path.join(out_dir, "stderr.txt"), os.path.join(out_dir, "log"))
    # Injector/client transcripts and every helper's stdio log.
    for name in ("gcode_responses.jsonl", "webhooks_transcript.jsonl"):
        src = os.path.join(temp_root, name)
        dst = os.path.join(out_dir, name)
        if os.path.exists(src):
            _copy_file(src, dst)
        else:
            with open(dst, "w", encoding="utf-8") as f:
                f.write(json.dumps({"note": "no live transcript produced"}) + "\n")
    for name, src in helper_logs.items():
        _copy_file(src, os.path.join(out_dir, name))
    # The sims' JSON traces (decoded commands, responses, probe events).
    for mcu in scn.mcus:
        trace = os.path.join(temp_root, "mcusim-%s.trace" % mcu.get("name", "mcu"))
        if os.path.exists(trace):
            _copy_file(trace, os.path.join(out_dir, os.path.basename(trace)))
    # mcu/<name>.bin is every byte the host wrote to that sim (the sim's
    # --capture), the live counterpart of the batch -o file.
    for mcu in scn.mcus:
        name = mcu.get("name", "mcu")
        capture = os.path.join(temp_root, "mcusim-%s.bin" % name)
        dst = os.path.join(out_dir, "mcu", name + ".bin")
        if os.path.exists(capture):
            _copy_file(capture, dst)
        else:
            with open(dst, "wb"):
                pass
    return exit_code, cmd, {"participants": participants, "end_at": float(end_at),
                            "signals": signal_specs, "wall_timeout": wall_timeout}


def _signals_child_main(host_pid, spec_json):
    # vclock participant: each `at` is an absolute virtual timestamp on
    # the scenario timeline, identical for both implementations.
    specs = sorted(json.loads(spec_json), key=lambda s: float(s.get("at", 0.0)))
    for spec in specs:
        at = float(spec.get("at", 0.0))
        if time.monotonic() < at:
            time.sleep(at - time.monotonic())
        name = spec.get("signal", "TERM").upper()
        signum = getattr(signal, "SIG" + name)
        try:
            os.kill(host_pid, signum)
        except ProcessLookupError:
            # The host already exited (an earlier signal or a crash).
            return


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
            exit_code, cmd, live = _run_live(scn, args, temp_root, staged, out_dir, python_exe)
            manifest["command"] = cmd
            manifest["live"] = live
            if live["wall_timeout"]:
                manifest.setdefault("notes", []).append(
                    "host killed after --wall-timeout %ss wall time before run.end_at"
                    % args.wall_timeout)
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
        return _injector_main(sys.argv[2], sys.argv[3], sys.argv[4], json.loads(sys.argv[5]))
    if len(sys.argv) > 1 and sys.argv[1] == "--webhooks-client":
        _webhooks_client_main(sys.argv[2], json.loads(sys.argv[4]), sys.argv[3])
        return 0
    if len(sys.argv) > 1 and sys.argv[1] == "--signals-child":
        _signals_child_main(int(sys.argv[2]), sys.argv[3])
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
    parser.add_argument("--wall-timeout", type=float, default=600.0,
                        help="live mode: wall-clock seconds before the host is killed"
                             " (safety net; run.end_at ends live runs in virtual time)")
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
