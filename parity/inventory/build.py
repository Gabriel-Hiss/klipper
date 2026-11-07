#!/usr/bin/env python3
"""Generate parity/inventory/units.json from the klippy source tree.

Stdlib only. Python sources are split with `ast`; C sources with a
deliberate, brace-matching splitter; `.cfg` data files are chunked by line
range. Every covered file maps to exactly one contiguous span of units.
"""
import ast
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "units.json"

# --- crate / family assignment -------------------------------------------
# Families come from parity/maps/map-extras-{a,b,c}.md. Each extras file
# stem maps to exactly one family; unmapped stems land in `misc`.
FAMILY = {}
def _fam(fam, stems):
    for s in stems:
        FAMILY[s] = fam

_fam("bus_transport", """bus bulk_sensor buttons board_pins duplicate_pin_override
canbus_ids canbus_stats error_mcu query_adc query_endstops""".split())
_fam("digipots_dac", """ad5206 dac084S085 mcp4018 mcp4451 mcp4728""".split())
_fam("adc_temperature", """adc_temperature adc_scaled ads1x1x ads1220 ads131m0x
ds18b20 aht10 bme280 thermistor temperature_sensor temperature_combined
temperature_host temperature_mcu temperature_probe temperature_fan
spi_temperature sht3x verify_heater pid_calibrate z_thermal_adjust lm75 htu21d""".split())
_fam("accelerometers_angle", """adxl345 bmi160 angle icm20948 mpu9250 lis2dw lis3dh""".split())
_fam("probes_leveling", """bltouch bed_mesh bed_tilt bed_screws axis_twist_compensation
delta_calibrate endstop_phase probe probe_eddy_current trigger_analog smart_effector
z_tilt quad_gantry_level screws_tilt_adjust safe_z_home skew_correction tuning_tower""".split())
_fam("displays_ui", """display display_status st7920 uc1701 hd44780 hd44780_spi
font8x14 menu menu_keys aip31068_spi""".split())
_fam("leds", """dotstar led neopixel pca9533 pca9632""".split())
_fam("fans", """controller_fan fan fan_generic heater_fan""".split())
_fam("macros_templates", """delayed_gcode gcode_macro gcode_button homing_override
gcode_arcs firmware_retraction""".split())
_fam("print_job", """exclude_object virtual_sdcard sdcard_loop print_stats
pause_resume statistics""".split())
_fam("motion_extras", """extruder_stepper motion_queuing input_shaper shaper_defs
gcode_move homing force_move manual_stepper motion_report stepper_enable
idle_timeout garbage_collection""".split())
_fam("pins_outputs", """output_pin multi_pin pwm_tool pwm_cycle_time static_pwm_clock
static_digital_output servo pulse_counter samd_sercom sx1509 replicape""".split())
_fam("heaters_temperature", """heaters heater_bed heater_generic homing_heaters""".split())
_fam("filament_sensors", """filament_switch_sensor filament_motion_sensor
hall_filament_width_sensor tsl1401cl_filament_width_sensor""".split())
_fam("load_cells_probing", """hx71x load_cell load_cell_probe ldc1612 manual_probe""".split())
_fam("tmc_drivers", """tmc tmc_uart tmc2130 tmc2208 tmc2209 tmc2240 tmc2660 tmc5160""".split())
_fam("resonance_analysis", """resonance_tester shaper_calibrate""".split())
_fam("host_state", """respond save_variables""".split())
_fam("misc", """palette2""".split())
FAMILY["__init__"] = "misc"

CHELPER_SERIAL = {"msgblock", "serialqueue", "pollreactor", "trdispatch", "pyhelper"}
CHELPER_MOTION = {"trapq", "itersolve", "stepcompress", "steppersync",
                  "kin_cartesian", "kin_corexy", "kin_corexz", "kin_delta",
                  "kin_deltesian", "kin_extruder", "kin_generic", "kin_idex",
                  "kin_polar", "kin_rotary_delta", "kin_shaper", "kin_winch"}

# Block-level indivisible units: (relative path, symbol) -> reason.
INDIVISIBLE = {
    ("klippy/reactor.py", "SelectReactor"): "reactor dispatch/greenlets: single greenlet-baton state machine, splitting breaks timer/fd/completion ordering",
    ("klippy/configfile.py", "ConfigFileReader"): "configfile parse/autosave: parse pass shares the mutable config tree with autosave",
    ("klippy/configfile.py", "ConfigAutoSave"): "configfile parse/autosave: autosave rewrites the tree built by the parse pass",
    ("klippy/configfile.py", "PrinterConfig"): "configfile parse/autosave: registry over the shared config tree and autosave callbacks",
    ("klippy/mcu.py", "MCUConnectHelper"): "mcu config/connect state machine: CRC/restart/identify handshake is one state machine",
    ("klippy/gcode.py", "GCodeDispatch"): "gcode dispatch: mux and direct dispatch state is shared, command tables are mutated together",
    ("klippy/extras/gcode_macro.py", "PrinterGCodeMacro"): "gcode_macro Jinja environment: one shared jinja2.Environment ('{%','%}', '{','}') instance and load_template",
    ("klippy/chelper/stepcompress.c", "*"): "stepcompress compression core: the bisect buffer, step chains and flush history share one mutable ring state",
    ("klippy/chelper/serialqueue.c", "*"): "serialqueue core: batching, retransmission and the receive window share one queue state",
    ("klippy/chelper/itersolve.c", "*"): "itersolve solver: the fixed-point solve loop is a single indivisible numeric kernel",
}
WHOLE_FILE_INDIVISIBLE = {k for k, v in INDIVISIBLE.items() if v == "*"}

# Verification heuristics.
UNVERIFIABLE_FILES = {
    "klippy/console.py": "interactive MCU debug console (tty input and eval); no non-interactive scenario exercises it",
    "klippy/parsedump.py": "offline .dict + serial dump decoder; reads host dump files, never runs on the printer",
    "klippy/queuelogger.py": "background log-file writer with host filesystem rotation and thread timing",
    "klippy/util.py": "host process/OS identity (cpu info, /proc, git version, pty) with no simulated counterpart",
}
KE_ONLY_HINTS = [
    ("display", "real LCD timing and controller waveforms are not modeled by the MCU simulator"),
    ("load_cell", "strain/amplifier settling needs real hardware signal timing"),
    ("ldc1612", "eddy-current sensor response needs real coil timing"),
    ("tsl1401cl", "line-array filament measurement needs real optical hardware"),
    ("resonance_tester", "requires real vibration input from the physical machine"),
    ("shaper_calibrate", "requires real accelerometer vibration data"),
    ("bed_mesh", "requires real bed surface flatness measurement"),
    ("angle", "angle calibration solves for the real machine's belt/encoder behaviour"),
    ("delta_calibrate", "requires real probe measurement of the physical machine"),
    ("axis_twist_compensation", "requires real Z-motion measurement"),
    ("z_tilt", "requires real probing of the physical gantry"),
    ("quad_gantry_level", "requires real gantry tramming on the physical machine"),
    ("screws_tilt_adjust", "requires real bed-screw probing"),
    ("probe", "physical probe actuation and contact is not modeled by the simulator"),
    ("bltouch", "physical probe actuation and contact is not modeled by the simulator"),
    ("manual_probe", "requires a human touching the nozzle to the bed"),
    ("trigger_analog", "Trigger-filtered sensor data needs real chip waveforms"),
    ("smart_effector", "requires a real daughter board and its timing"),
    ("kin_polar", "KE uses cartesian kinematics only; other geometries need real machine verification"),
    ("kin_rotary_delta", "KE uses cartesian kinematics only; other geometries need real machine verification"),
    ("kin_winch", "KE does not use winch kinematics; needs real machine verification"),
    ("hybrid_", "hybrid kinematics need real machine verification; KE is cartesian"),
]

def verify(path, stem):
    if path in UNVERIFIABLE_FILES:
        return {"class": "unverifiable", "reason": UNVERIFIABLE_FILES[path]}
    for hint, reason in KE_ONLY_HINTS:
        if hint in stem or hint in path:
            return {"class": "ke-only", "reason": reason}
    return {"class": "sim",
            "reason": "verifiable with MCU simulator plus virtual clock (config, gcode, MCU messages and status are serializable)"}

def recordable(path, text):
    """I/O serializable enough for per-unit JSONL recordings."""
    if path in ("klippy/chelper/serialqueue.c", "klippy/chelper/steppersync.c",
                "klippy/chelper/pollreactor.c", "klippy/reactor.py"):
        return False
    if any(tok in text for tok in ("threading", "greenlet", "pty", "socket",
                                   "subprocess", "faulthandler", "/proc/",
                                   "os.cpu", "pthread_")):
        return False
    return True

def difficulty(key, lines):
    high = key in {
        "klippy/reactor.py", "klippy/mcu.py", "klippy/configfile.py",
        "klippy/extras/gcode_macro.py", "klippy/extras/bed_mesh.py",
        "klippy/extras/probe_eddy_current.py", "klippy/webhooks.py",
        "klippy/gcode.py", "klippy/chelper/stepcompress.c",
        "klippy/chelper/serialqueue.c", "klippy/chelper/itersolve.c",
    }
    if high or lines > 350:
        return "high"
    if lines > 120:
        return "medium"
    return "low"

def layer_for(crate, path, stem):
    if crate == "msgproto":
        return 1
    if crate == "chelper_serial":
        return 2
    if crate == "chelper_motion":
        return 3
    if crate == "kinematics":
        return 3
    if crate.startswith("extras_"):
        return 4
    # core crate
    if path == "klippy/parsedump.py":
        return 1
    if path in ("klippy/util.py", "klippy/queuelogger.py", "klippy/mathutil.py"):
        return 0
    if path in ("klippy/webhooks.py", "klippy/klippy.py", "klippy/console.py"):
        return 5
    return 2

def crate_module(rel):
    parts = rel.split("/")
    stem = parts[-1].rsplit(".", 1)[0]
    ext = parts[-1].rsplit(".", 1)[-1]
    if rel.startswith("klippy/chelper/"):
        if stem in CHELPER_SERIAL or rel == "klippy/chelper/__init__.py":
            return "chelper_serial", stem
        if stem in CHELPER_MOTION:
            return "chelper_motion", stem
        return "chelper_serial", stem
    if rel.startswith("klippy/kinematics/"):
        return "kinematics", stem
    if rel.startswith("klippy/extras/"):
        fam = FAMILY.get(stem, "misc")
        if parts[-2] == "display" and stem in ("__init__",):
            fam = "displays_ui"
        mod = ("extras.display." + stem) if parts[-2] == "display" else ("extras." + stem)
        return "extras_" + fam, mod
    if stem == "msgproto":
        return "msgproto", "msgproto"
    if rel.startswith("klippy/"):
        return "core", stem
    return "core", stem

def module_path(rel):
    """Dotted source-module path used as the unit-id prefix."""
    if rel.startswith("klippy/chelper/"):
        return "chelper." + rel.split("/")[-1].rsplit(".", 1)[0]
    if rel.startswith("klippy/kinematics/"):
        return "kinematics." + rel.split("/")[-1].rsplit(".", 1)[0]
    if rel.startswith("klippy/extras/"):
        stem = rel.split("/")[-1].rsplit(".", 1)[0]
        if "/display/" in rel:
            return "extras.display." + stem
        return "extras." + stem
    if rel.startswith("klippy/"):
        return rel.split("/")[-1].rsplit(".", 1)[0]
    return rel.split("/")[-1].rsplit(".", 1)[0]

# --- contract extraction -------------------------------------------------
def extract_contracts(lines, start, end):
    gcode, config, status, endpoints, mcu, logs = [], [], [], [], [], []
    seen = set()
    def add(lst, val):
        if val not in seen:
            seen.add(val)
            lst.append(val)
    for i in range(start - 1, min(end, len(lines))):
        text = lines[i]
        for m in re.finditer(r'''register(?:_mux)?_command\([^)]*?['"]([A-Za-z0-9_]+)['"]''', text):
            add(gcode, m.group(1))
        for m in re.finditer(r'''\.get(?:float|int|boolean|choice|list|floatlist|intlist|lists|list|prefix_options|prefix_sections)?\(\s*['"]([A-Za-z0-9_]+)['"]''', text):
            add(config, m.group(1))
        for m in re.finditer(r'''add_endpoint\(['"]([^'"]+)['"]''', text):
            add(endpoints, m.group(1))
        for m in re.finditer(r'''logging\.\w+\(([^)]*?)['"]([^'"]*%[^'"]*)['"]''', text):
            add(logs, m.group(2)[:80])
        for m in re.finditer(r'''['"]([a-z_]+ [a-z_]+=[^'"]*)['"]''', text):
            if "oid=" in m.group(1) or "response=" in m.group(1) or "data=" in m.group(1):
                add(mcu, m.group(1)[:80])
        if "get_status" in text:
            for m in re.finditer(r'''['"](\w+)['"]\s*:''', text):
                if m.group(1) not in ("self",):
                    add(status, m.group(1))
        if re.match(r"\s*def cmd_(\w+)", text):
            m = re.match(r"\s*def cmd_(\w+)", text)
            add(gcode, m.group(1).upper())
    cap = 40
    return {"gcode": gcode[:cap], "config": config[:cap],
            "status": status[:cap], "endpoints": endpoints[:cap],
            "mcu": mcu[:cap], "logs": logs[:cap]}

# --- C splitting ---------------------------------------------------------
C_FUNC_RE = re.compile(r"^[\w\s]+?\b([A-Za-z_]\w*)\s*\([^;{]*\)\s*(?:/\*.*\*/\s*)?\{$")

def c_functions(text):
    lines = text.splitlines()
    depths, depth = [], 0
    for line in lines:
        depths.append(depth)
        for ch in line:
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
    funcs = []
    for idx, line in enumerate(lines):
        s = line.strip()
        if not s or s.startswith(("#", "/", "*")):
            continue
        m = C_FUNC_RE.match(s)
        if not m:
            continue
        name = m.group(1)
        if name in ("if", "for", "while", "switch", "return"):
            continue
        if depths[idx] != 0:
            continue
        # function body ends at the first line after idx whose scan returns depth to 0
        d, end, j = 0, None, idx
        while j < len(lines):
            for ch in lines[j]:
                if ch == "{":
                    d += 1
                elif ch == "}":
                    d -= 1
            if j > idx and d == 0:
                end = j + 1
                break
            j += 1
        if end is None:
            end = len(lines)
        funcs.append((name, idx + 1, end))
    return funcs

def c_deps(rel, text, lines):
    stems = set()
    for line in lines:
        m = re.search(r'#include\s+"([^"]+)"', line)
        if m:
            stems.add(m.group(1).rsplit(".", 1)[0])
    return sorted("chelper." + s for s in stems if s)

# --- Python deps ---------------------------------------------------------
def py_deps(rel, tree):
    deps = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                if a.name == "klippy" or a.name.startswith("klippy.") or a.name.startswith("extras."):
                    deps.add(a.name)
                elif a.name.startswith("chelper"):
                    deps.add("chelper")
        elif isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            if mod.startswith("klippy") and mod != "klippy":
                deps.add(mod)
            elif mod.startswith("extras"):
                deps.add(mod)
            elif mod == "klippy":
                deps.add("klippy")
            if node.level:
                deps.add("chelper" if "chelper" in (ROOT / rel).as_posix() else mod)
    out = []
    for d in sorted(deps):
        if d.startswith("klippy."):
            out.append(d[len("klippy."):])
        elif d.startswith("klippy"):
            out.append("klippy")
        elif d.startswith("extras."):
            out.append(d)
        elif d == "chelper":
            out.append("chelper")
        else:
            out.append(d)
    return sorted(set(out))

# --- splitting -----------------------------------------------------------
def camel(name):
    parts = re.split(r"_+", name.lstrip("_"))
    s = "".join(p[:1].upper() + p[1:] for p in parts if p)
    return s or "Module"

def node_span(node):
    starts = [getattr(node, "lineno", 1)]
    for d in getattr(node, "decorator_list", []) or []:
        starts.append(getattr(d, "lineno", 99))
    return min(starts), node.end_lineno

def class_fields(cls_node):
    fields = set()
    for sub in ast.walk(cls_node):
        if isinstance(sub, ast.Attribute) and isinstance(sub.value, ast.Name) and sub.value.id == "self":
            fields.add(sub.attr)
    return sorted(fields)

def python_blocks(tree, lines, target, rel):
    blocks = []
    for node in tree.body:
        start, end = node_span(node)
        div = None
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if (rel, node.name) in INDIVISIBLE:
                div = INDIVISIBLE[(rel, node.name)]
            blocks.append({"name": camel(node.name), "start": start, "end": end,
                           "solo": bool(div), "raw": node.name, "kind": "function",
                           "indiv": div})
        elif isinstance(node, ast.ClassDef):
            if (rel, node.name) in INDIVISIBLE:
                blocks.append({"name": node.name, "start": start, "end": node.end_lineno,
                               "solo": True, "raw": node.name, "kind": "class",
                               "fields": class_fields(node), "class": node.name,
                               "indiv": INDIVISIBLE[(rel, node.name)]})
                continue
            span = (node.end_lineno - start + 1)
            methods = [n for n in node.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
            if span > target * 1.5 and len(methods) > 1:
                groups, cur, curlines = [], [], 0
                for m in methods:
                    ms, me = node_span(m)
                    mlines = me - ms + 1
                    if cur and curlines + mlines > target:
                        groups.append(cur)
                        cur, curlines = [], 0
                    cur.append((m.name, ms, me))
                    curlines += mlines
                if cur:
                    groups.append(cur)
                fields = class_fields(node)
                for gi, g in enumerate(groups):
                    gs = g[0][1] if gi else start
                    ge = g[-1][2]
                    blocks.append({"name": node.name + "_" + camel(g[0][0]) + ("Group" if len(groups) > 1 else ""),
                                     "start": gs, "end": ge, "solo": True,
                                     "raw": node.name, "kind": "class-group",
                                     "fields": fields, "class": node.name})
            else:
                blocks.append({"name": node.name, "start": start, "end": node.end_lineno,
                               "solo": False, "raw": node.name, "kind": "class",
                               "fields": class_fields(node), "class": node.name})
        else:
            # module header / stray top-level statements
            blocks.append({"name": "ModuleHeader", "start": start, "end": end,
                           "solo": False, "raw": "_module", "kind": "module"})
    blocks.sort(key=lambda b: b["start"])
    return blocks

def pack_blocks(blocks, lines_total, target):
    chunks, cur, curlines = [], [], 0
    for b in blocks:
        size = b["end"] - b["start"] + 1
        if b.get("solo"):
            if cur:
                chunks.append(cur)
                cur, curlines = [], 0
            chunks.append([b])
            continue
        if cur and curlines + size > target and curlines >= 30:
            chunks.append(cur)
            cur, curlines = [], 0
        cur.append(b)
        curlines += size
    if cur:
        chunks.append(cur)
    units = []
    for ci, chunk in enumerate(chunks):
        start = 1 if ci == 0 else chunk[0]["start"]
        end = (chunks[ci + 1][0]["start"] - 1) if ci + 1 < len(chunks) else lines_total
        name = chunk[0]["name"]
        kind = chunk[0]["kind"]
        units.append({"name": name, "start": start, "end": end, "kind": kind,
                      "fields": chunk[0].get("fields"), "class": chunk[0].get("class"),
                      "indiv": chunk[0].get("indiv")})
    prev_end = 0
    for u in units:
        u["start"] = prev_end + 1
        prev_end = u["end"]
    if units and prev_end != lines_total:
        units[-1]["end"] = lines_total
    return units

def chunk_cfg(lines, target):
    units = []
    start = 1
    n = len(lines)
    while start <= n:
        end = min(start + target - 1, n)
        units.append({"name": "CfgSection", "start": start, "end": end,
                      "kind": "data", "fields": None})
        start = end + 1
    return units

def main():
    files = []
    files += sorted(p.relative_to(ROOT).as_posix() for p in (ROOT / "klippy").glob("*.py"))
    files += sorted(p.relative_to(ROOT).as_posix() for p in (ROOT / "klippy/kinematics").glob("*.py"))
    files += sorted(p.relative_to(ROOT).as_posix() for p in (ROOT / "klippy/extras").rglob("*.py"))
    files += sorted(p.relative_to(ROOT).as_posix() for p in (ROOT / "klippy/extras").rglob("*.cfg"))
    files += sorted(p.relative_to(ROOT).as_posix() for p in (ROOT / "klippy/chelper").glob("*.c"))
    files += sorted(p.relative_to(ROOT).as_posix() for p in (ROOT / "klippy/chelper").glob("*.h"))

    # target size from total lines so the count lands near 1000
    total_lines = 0
    texts = {}
    for rel in files:
        t = (ROOT / rel).read_text(encoding="utf-8", errors="replace")
        texts[rel] = t
        total_lines += t.count("\n") + 1
    target = max(42, min(60, round(total_lines / 870)))

    units, seen_ids = [], {}
    for rel in files:
        text = texts[rel]
        lines = text.splitlines()
        nlines = len(lines) if len(lines) >= text.count("\n") else text.count("\n") + 1
        crate, mod = crate_module(rel)
        kind = "tool" if rel in ("klippy/console.py", "klippy/parsedump.py") else (
            "c" if rel.endswith((".c", ".h")) else ("tool" if rel.endswith(".cfg") else "py"))
        stem = rel.split("/")[-1].rsplit(".", 1)[0]
        layer = layer_for(crate, rel, stem)
        ver = verify(rel, stem)
        rec = recordable(rel, text)
        diff = difficulty(rel, nlines)
        whole_indiv = (rel, "*") in INDIVISIBLE
        if whole_indiv:
            deps = c_deps(rel, text, lines) if rel.endswith((".c", ".h")) else []
            units.append({
                "id": module_path(rel) + "." + camel(stem),
                "source": {"path": rel, "start": 1, "end": nlines},
                "kind": kind, "lines": nlines, "crate": crate, "module": mod,
                "deps": deps,
                "contracts": extract_contracts(lines, 1, nlines),
                "verification": ver, "recordable": rec, "difficulty": diff,
                "layer": layer, "indivisible": True, "reason": INDIVISIBLE[(rel, "*")],
            })
            continue

        if rel.endswith((".c", ".h")):
            deps = c_deps(rel, text, lines)
            if rel.endswith(".h"):
                blocks = [{"name": camel(stem) + "Decls", "start": 1, "end": nlines,
                           "solo": True, "kind": "c-decls", "fields": None}]
                units_for_file = blocks
            else:
                funcs = c_functions(text)
                if not funcs:
                    units_for_file = chunk_cfg(lines, target)
                    for u in units_for_file:
                        u["kind"] = "c-decls"
                else:
                    units_for_file = []
                    # group small functions, keep large ones solo
                    cur, curlines = [], 0
                    for fn, fs, fe in funcs:
                        size = fe - fs + 1
                        if size > target + 20:
                            if cur:
                                units_for_file.append({"name": camel(cur[0][0]), "start": cur[0][1], "end": cur[-1][2], "kind": "c-func", "fields": None})
                                cur, curlines = [], 0
                            units_for_file.append({"name": camel(fn), "start": fs, "end": fe, "kind": "c-func", "fields": None})
                            continue
                        if cur and curlines + size > target and curlines >= 30:
                            units_for_file.append({"name": camel(cur[0][0]), "start": cur[0][1], "end": cur[-1][2], "kind": "c-func", "fields": None})
                            cur, curlines = [], 0
                        cur.append((fn, fs, fe))
                        curlines += size
                    if cur:
                        units_for_file.append({"name": camel(cur[0][0]), "start": cur[0][1], "end": cur[-1][2], "kind": "c-func", "fields": None})
            # make spans contiguous over the file
            prev = 0
            for u in units_for_file:
                u["start"] = prev + 1
                prev = u["end"]
            if units_for_file:
                units_for_file[-1]["end"] = nlines
            for u in units_for_file:
                sid = module_path(rel) + "." + u["name"]
                n = seen_ids.get(sid, 0)
                seen_ids[sid] = n + 1
                if n:
                    sid += f"_{n}"
                elines = u["end"] - u["start"] + 1
                units.append({
                    "id": sid,
                    "source": {"path": rel, "start": u["start"], "end": u["end"]},
                    "kind": kind, "lines": elines, "crate": crate, "module": mod,
                    "deps": deps,
                    "contracts": extract_contracts(lines, u["start"], u["end"]),
                    "verification": ver, "recordable": rec, "difficulty": diff,
                    "layer": layer,
                })
            continue

        if rel.endswith(".cfg"):
            for u in chunk_cfg(lines, target):
                sid = module_path(rel) + ".CfgSection_" + str(u["start"])
                units.append({
                    "id": sid,
                    "source": {"path": rel, "start": u["start"], "end": u["end"]},
                    "kind": "tool", "lines": u["end"] - u["start"] + 1,
                    "crate": crate, "module": mod, "deps": [],
                    "contracts": extract_contracts(lines, u["start"], u["end"]),
                    "verification": ver, "recordable": rec, "difficulty": diff,
                    "layer": layer,
                })
            continue

        # Python files
        try:
            tree = ast.parse(text)
        except SyntaxError:
            units.append({
                "id": module_path(rel) + ".Unparsed",
                "source": {"path": rel, "start": 1, "end": nlines},
                "kind": kind, "lines": nlines, "crate": crate, "module": mod,
                "deps": [], "contracts": extract_contracts(lines, 1, nlines),
                "verification": ver, "recordable": rec, "difficulty": diff,
                "layer": layer,
            })
            continue
        deps = py_deps(rel, tree)
        blocks = python_blocks(tree, lines, target, rel)
        packed = pack_blocks(blocks, nlines, target)
        for u in packed:
            sid = module_path(rel) + "." + u["name"]
            n = seen_ids.get(sid, 0)
            seen_ids[sid] = n + 1
            if n:
                sid += f"_{n}"
            elines = u["end"] - u["start"] + 1
            unit = {
                "id": sid,
                "source": {"path": rel, "start": u["start"], "end": u["end"]},
                "kind": kind, "lines": elines, "crate": crate, "module": mod,
                "deps": deps,
                "contracts": extract_contracts(lines, u["start"], u["end"]),
                "verification": ver, "recordable": rec, "difficulty": diff,
                "layer": layer,
            }
            if u.get("kind") == "class-group":
                unit["fields"] = u.get("fields")
                unit["class"] = u.get("class")
            if u.get("indiv"):
                unit["indivisible"] = True
                unit["reason"] = u["indiv"]
            units.append(unit)

    # sibling groups of split classes share their field list
    from collections import defaultdict
    sib = defaultdict(list)
    for u in units:
        if u.get("class"):
            sib[(u["source"]["path"], u["class"])].append(u["id"])
    for u in units:
        if u.get("class") and len(sib[(u["source"]["path"], u["class"])]) > 1:
            u["shared_with"] = sib[(u["source"]["path"], u["class"])]

    OUT.write_text(json.dumps({"units": units}, indent=2) + "\n", encoding="utf-8")
    print(f"units={len(units)} target_size={target} total_lines={total_lines}")
    from collections import Counter
    print("per crate:", dict(Counter(u["crate"] for u in units)))
    print("per layer:", dict(Counter(u["layer"] for u in units)))
    print("per verify:", dict(Counter(u["verification"]["class"] for u in units)))
    print("recordable:", dict(Counter(u["recordable"] for u in units)))
    print("indivisible:", [u["id"] for u in units if u.get("indivisible")])


if __name__ == "__main__":
    sys.exit(main())
