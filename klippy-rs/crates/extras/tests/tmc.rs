//! Parity tests for `extras::tmc` (port of `klippy/extras/tmc.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). A scenario is a list of textual
//! steps interpreted twice: by a Python interpreter driving the real
//! `extras.tmc` (with the real `configfile.ConfigWrapper`,
//! `gcode.GCodeCommand`, `stepper.parse_step_distance`,
//! `extras.stepper_enable` and `extras.bulk_sensor`) and by a Rust
//! interpreter driving the port. Both sides record the same trace: every call
//! the code under test makes on its collaborators (MCU register reads and
//! writes, G-code responses, shutdowns, pin and heater registrations),
//! every returned value and every raised exception (class and message),
//! followed by the captured log lines. The two traces must be identical.
//!
//! The scenarios cover `FieldHelper`, `TMCErrorCheck`, `TMCStallguardDump`,
//! `TMCCommandHelper` (`SET_TMC_FIELD`, `INIT_TMC`, `SET_TMC_CURRENT`,
//! `DUMP_TMC`, phase tracking, enable/disable handling, connect/identify,
//! `get_status`), `TMCVirtualPinHelper` and the configuration helpers
//! (`TMCWaveTableHelper`, `TMCMicrostepHelper`, `TMCtstepHelper`,
//! `TMCStealthchopHelper`, `TMCVcoolthrsHelper`, `TMCVhighHelper`).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so expected values come from the oracle alone.

use extras::force_move::ForceMoveApi;
use extras::heaters::{Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureSensorApi};
use extras::stepper_enable::{PrinterStepperEnable, PrinterStepperEnableApi};
use extras::tmc::{
    module_factory, tmc_microstep_helper, tmc_stealthchop_helper, tmc_tstep_helper,
    tmc_vcoolthrs_helper, tmc_vhigh_helper, tmc_wave_table_helper, ConfigFieldDefault,
    FieldFormatter, FieldHelper, McuTmcApi, ReadTranslate, RegisterRaw, TMCCommandHelper,
    TMCErrorCheck, TMCStallguardDump, TMCVirtualPinHelper, TmcCurrentHelperApi,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueryWrapper, CommandQueue,
    CommandWrapper, ConfigCallback, DigitalOutPin, DigitalOutPinApi, Endstop, EndstopApi,
    LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinResolver,
    PinsApi, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi, EV_KLIPPY_CONNECT, EV_KLIPPY_MCU_IDENTIFY,
    EV_STEPPER_SET_DIR_INVERTED, EV_STEPPER_SYNC_MCU_POSITION, EV_HOMING_MOVE_BEGIN,
    EV_HOMING_MOVE_END,
};
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, Rail, RailOpts, Stepper, StepperFactory, StepperFactoryApi,
    StepperKinematics, Trapq,
};
use klippy_api::toolhead::{
    ExtraAxis, HomingMoveApi, HomingTargetApi, Kinematics, LookaheadCallback, PrinterExtruder,
    ToolHeadApi,
};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use klippy_api::{log, EventArg, GCell, ObjectRef};
use pycompat::text::{float_from_string, float_repr, float_str, int_from_string, str_repr};
use pycompat::PyValue;
use reactor::{Completion, Reactor, NEVER};
use recording::Value;
use std::sync::{Arc, LazyLock};

// ---------------------------------------------------------------------------
// Python side
// ---------------------------------------------------------------------------

const SETUP: &str = r##"
import configparser
import logging
import gcode as gcode_mod
import configfile
import extras.tmc as tmc
import extras.stepper_enable as stepper_enable

CommandError = gcode_mod.CommandError

PY_LOGS = []
class _LogCapture(logging.Handler):
    def emit(self, record):
        PY_LOGS.append("%s:%s" % (logging.getLevelName(record.levelno), record.getMessage()))
logging.getLogger().addHandler(_LogCapture())
logging.getLogger().setLevel(logging.INFO)

T = []
def tr(s):
    T.append(s)

class FakeMutex:
    def __enter__(self):
        return self
    def __exit__(self, *args):
        return False

class FakeTimer:
    def __init__(self, cb, waketime):
        self.cb = cb
        self.waketime = waketime

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.callbacks = []
        self.timers = []
    def monotonic(self):
        return 100.
    def pause(self, waketime):
        return waketime
    def mutex(self):
        return FakeMutex()
    def register_timer(self, cb, waketime=NOW):
        t = FakeTimer(cb, waketime)
        self.timers.append(t)
        return t
    def unregister_timer(self, timer):
        timer.waketime = self.NEVER
        self.timers.remove(timer)
    def register_callback(self, cb, waketime=NOW):
        self.callbacks.append(cb)
    def run_callbacks(self):
        cbs = self.callbacks
        self.callbacks = []
        for cb in cbs:
            cb(0.)

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self.reactor = FakeReactor()
    def get_reactor(self):
        return self.reactor
    def get_start_args(self):
        return {}
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is not configfile.sentinel:
            return default
        raise self.config_error("Unknown config object '%s'" % (name,))
    def load_object(self, config, section, default=configfile.sentinel):
        return self.lookup_object(section, default)
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        res = []
        for cb in list(self.handlers.get(event, [])):
            res.append(cb(*params))
        return res
    def invoke_shutdown(self, msg, details={}):
        tr('invoke_shutdown(%r)' % (msg,))

class FakeGCode:
    def __init__(self):
        self.mux = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        tr('register_command(%s)' % (cmd,))
    def register_mux_command(self, cmd, key, value, func, desc=None):
        tr('register_mux_command(%s, %s, %s, %r)' % (cmd, key, value, desc))
        self.mux[(cmd, value)] = func
    def get_mutex(self):
        return FakeMutex()
    def respond_info(self, msg, log=True):
        tr('respond_info(%r)' % (msg,))
    def respond_raw(self, msg):
        tr('respond_raw(%r)' % (msg,))

class FakeToolHead:
    def get_last_move_time(self):
        tr('get_last_move_time')
        return 10.5
    def wait_moves(self):
        tr('wait_moves')

class FakeStepper:
    def __init__(self, name):
        self._name = name
        self.active_callbacks = []
        self.dir_inv = False
        self.mcu_pos = 0
        self.step_dist = 0.0125
        self.both_edge = False
    def get_name(self, short=False):
        return self._name
    def get_dir_inverted(self):
        return (self.dir_inv, self.dir_inv)
    def get_mcu_position(self, cmd_pos=None):
        return self.mcu_pos
    def mcu_to_commanded_position(self, mcu_pos):
        return (mcu_pos + 3) * self.step_dist
    def get_step_dist(self):
        return self.step_dist
    def setup_default_pulse_duration(self, pulse_duration, step_both_edge):
        tr('setup_default_pulse_duration(%r, %r)' % (pulse_duration, step_both_edge))
    def get_pulse_duration(self):
        return (0.000002, self.both_edge)
    def add_active_callback(self, cb):
        self.active_callbacks.append(cb)
    def fire_active(self, print_time):
        cbs = self.active_callbacks
        self.active_callbacks = []
        for cb in cbs:
            cb(print_time)

class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime * 2. + 0.25

class FakeMcuTmc:
    def __init__(self, fields, regs, freq):
        self.fields = fields
        self.name_to_reg = regs
        self.tmc_frequency = freq
        self.script = {}
        self.setfail = {}
        self.mcu = FakeMcu()
    def _item(self, name):
        seq = self.script.get(name, [0])
        if len(seq) > 1:
            return seq.pop(0)
        return seq[0]
    def get_fields(self):
        return self.fields
    def get_register(self, name):
        tr('get_register(%s)' % (name,))
        it = self._item(name)
        if isinstance(it, str):
            raise CommandError(it)
        if isinstance(it, tuple):
            return it[0]
        return it
    def get_register_raw(self, name):
        tr('get_register_raw(%s)' % (name,))
        it = self._item(name)
        if isinstance(it, str):
            raise CommandError(it)
        if isinstance(it, int):
            it = (it, 0.5, None)
        d = {'data': it[0], '#receive_time': it[1]}
        if it[2] is not None:
            d['spi_status'] = it[2]
        return d
    def set_register(self, name, val, print_time=None):
        tr('set_register(%s, %d, %r)' % (name, val, print_time))
        if name in self.setfail:
            raise CommandError(self.setfail[name])
    def get_tmc_frequency(self):
        return self.tmc_frequency
    def get_mcu(self):
        return self.mcu

class FakeCur:
    def __init__(self, hold_none):
        self.hold_none = hold_none
        if hold_none:
            self.cur = [0.8, None, None, 2.0]
        else:
            self.cur = [0.8, 0.5, 0.5, 2.0]
        self.fail = None
    def get_current(self):
        return tuple(self.cur)
    def set_current(self, run_current, hold_current, print_time):
        tr('set_current(%r, %r, %r)' % (run_current, hold_current, print_time))
        if self.fail is not None:
            raise CommandError(self.fail)
        if self.hold_none:
            self.cur = [run_current, None, None, 2.0]
        else:
            self.cur = [run_current, hold_current, hold_current, 2.0]

class FakeHeaters:
    def register_monitor(self, config):
        tr('register_monitor(%s)' % (config.get_name(),))

class FakeWebhooks:
    def register_mux_endpoint(self, path, key, value, callback):
        tr('register_mux_endpoint(%s, %s, %s)' % (path, key, value))

class FakeForceMove:
    def lookup_stepper(self, name):
        tr('lookup_stepper(%s)' % (name,))
        if name in STEPPERS:
            return STEPPERS[name]
        raise configparser.Error("Unknown stepper %s" % (name,))

class FakePin:
    def __init__(self, label):
        self.label = label
    def setup_max_duration(self, duration):
        tr('%s.setup_max_duration(%r)' % (self.label, duration))
    def set_digital(self, print_time, value):
        tr('%s.set_digital(%r, %d)' % (self.label, print_time, value))

class FakeChip:
    def __init__(self):
        self.count = 0
    def setup_pin(self, pin_type, pin_params):
        self.count += 1
        label = 'pin%d' % (self.count,)
        tr('chip.setup_pin(%s, %s:%s)' % (pin_type, pin_params['chip_name'], pin_params['pin']))
        return FakePin(label)

class FakeEndstop:
    pass

PinsError = type('error', (Exception,), {})

class FakePins:
    error = PinsError
    def __init__(self, chip):
        self.chip = chip
        self.active = {}
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        desc = pin_desc.strip()
        invert = 0
        if can_invert and desc.startswith('!'):
            invert = 1
            desc = desc[1:]
        if ':' not in desc:
            chip_name, pin = 'mcu', desc
        else:
            chip_name, pin = [s.strip() for s in desc.split(':', 1)]
        share_name = '%s:%s' % (chip_name, pin)
        if share_name in self.active:
            share_params = self.active[share_name]
            if share_type is None or share_type != share_params['share_type']:
                raise configparser.Error("pin %s used multiple times in config" % (pin,))
            return share_params
        params = {'chip': self.chip, 'chip_name': chip_name, 'pin': pin,
                  'invert': invert, 'pullup': 0, 'share_type': share_type}
        self.active[share_name] = params
        return params
    def setup_pin(self, pin_type, pin_desc):
        tr('pins.setup_pin(%s, %s)' % (pin_type, pin_desc))
        return ENDSTOP
    def register_chip(self, name, chip):
        tr('register_chip(%s)' % (name,))

class FakeHMove:
    def __init__(self, endstops):
        self.endstops = endstops
    def get_mcu_endstops(self):
        return self.endstops

FORMATTERS = {
    "mres": (lambda v: "%d(%dusteps)" % (v, 0x100 >> v)),
    "otpw": (lambda v: "1(OvertempWarning!)" if v else ""),
    "i_scale_analog": (lambda v: "1(ExtVREF)" if v else ""),
}

def translate(reg_name, val):
    return (reg_name.lower() if reg_name == "GSTAT" else reg_name, val + 1)

def setup_world(fields, signed, regs, freq, reads, hold_none, cfgtext, driver_name):
    global T, PRINTER, GCODE, TOOLHEAD, PINS, CHIP, ENDSTOP, STEPPERS, STEPPER_X
    global STEPPER_Y, SE, FP, CFG, STEPPER_CFG, FIELDS, MCU_TMC, CUR, OBJ, READS
    T = []
    del PY_LOGS[:]
    OBJ = {}
    READS = reads
    FP = configparser.RawConfigParser(strict=False)
    FP.read_string(cfgtext)
    PRINTER = FakePrinter()
    GCODE = FakeGCode()
    TOOLHEAD = FakeToolHead()
    CHIP = FakeChip()
    PINS = FakePins(CHIP)
    ENDSTOP = FakeEndstop()
    STEPPER_X = FakeStepper('stepper_x')
    STEPPER_Y = FakeStepper('stepper_y')
    STEPPERS = {'stepper_x': STEPPER_X, 'stepper_z': STEPPER_Y}
    PRINTER.add_object('gcode', GCODE)
    PRINTER.add_object('toolhead', TOOLHEAD)
    PRINTER.add_object('pins', PINS)
    PRINTER.add_object('heaters', FakeHeaters())
    PRINTER.add_object('webhooks', FakeWebhooks())
    PRINTER.add_object('force_move', FakeForceMove())
    root = configfile.ConfigWrapper(PRINTER, FP, {}, 'stepper_x')
    SE = stepper_enable.PrinterStepperEnable(root)
    PRINTER.add_object('stepper_enable', SE)
    STEPPER_CFG = root.getsection('stepper_x')
    SE.register_stepper(STEPPER_CFG, STEPPER_X)
    CFG = configfile.ConfigWrapper(PRINTER, FP, {}, driver_name)
    FIELDS = tmc.FieldHelper(fields, signed, FORMATTERS)
    MCU_TMC = FakeMcuTmc(FIELDS, regs, freq)
    CUR = FakeCur(hold_none)
    del T[:]
    del PY_LOGS[:]

def pint(t):
    return int(t, 0)

def popt(a, i):
    if len(a) <= i or a[i] == '-':
        return None
    return int(a[i], 0)

def popts(a, i):
    if len(a) <= i or a[i] == '-':
        return None
    return a[i]

def parse_item(t):
    if t.startswith('!'):
        return t[1:].replace('_', ' ')
    if t.startswith('raw:'):
        parts = t.split(':')
        spi = int(parts[3], 0) if len(parts) > 3 else None
        return (int(parts[1], 0), float(parts[2]), spi)
    return int(t, 0)

def pfloat(t):
    if t == 'None':
        return None
    return float(t)

def run_step(line):
    tr('> ' + line)
    a = line.split()
    op = a[0]
    try:
        step(op, a, line)
    except Exception as e:
        tr('EXC %s: %s' % (type(e).__name__, e))

def step(op, a, line):
    if op == 'fh_get':
        tr('ret %r' % (FIELDS.get_field(a[1], popt(a, 2), popts(a, 3)),))
    elif op == 'fh_set':
        tr('ret %r' % (FIELDS.set_field(a[1], pint(a[2]), popt(a, 3), popts(a, 4)),))
    elif op == 'fh_cfg':
        d = {'T': True, 'F': False}.get(a[2])
        if d is None:
            d = pint(a[2])
        tr('ret %r' % (FIELDS.set_config_field(CFG, a[1], d),))
    elif op == 'fh_pretty':
        tr('ret %r' % (FIELDS.pretty_format(a[1], pint(a[2])),))
    elif op == 'fh_regfields':
        tr('ret %r' % (FIELDS.get_reg_fields(a[1], pint(a[2])),))
    elif op == 'fh_lookup':
        tr('ret %r' % (FIELDS.lookup_register(a[1]),))
    elif op == 'fh_regs':
        tr('ret %r' % (dict(FIELDS.registers),))
    elif op == 'mcu':
        if a[1] == 'script':
            MCU_TMC.script[a[2]] = [parse_item(t) for t in a[3:]]
        elif a[1] == 'setfail':
            MCU_TMC.setfail[a[2]] = ' '.join(a[3:])
        elif a[1] == 'clearfail':
            del MCU_TMC.setfail[a[2]]
    elif op == 'cur':
        if a[1] == 'set':
            CUR.cur = [pfloat(a[2]), pfloat(a[3]), pfloat(a[4]), pfloat(a[5])]
        elif a[1] == 'fail':
            CUR.fail = ' '.join(a[2:])
        elif a[1] == 'ok':
            CUR.fail = None
    elif op == 'stepper':
        st = STEPPER_X if a[1] == 'X' else STEPPER_Y
        if a[2] == 'pos':
            st.mcu_pos = pint(a[3])
        elif a[2] == 'dir':
            st.dir_inv = bool(pint(a[3]))
        elif a[2] == 'edge':
            st.both_edge = bool(pint(a[3]))
        elif a[2] == 'dist':
            st.step_dist = float(a[3])
    elif op == 'new':
        if a[1] == 'echeck':
            OBJ['echeck'] = tmc.TMCErrorCheck(CFG, MCU_TMC)
        elif a[1] == 'record':
            OBJ['record'] = tmc.TMCStallguardDump(CFG, MCU_TMC)
        elif a[1] == 'cmd':
            OBJ['cmd'] = tmc.TMCCommandHelper(CFG, MCU_TMC, CUR)
            OBJ['echeck'] = OBJ['cmd'].echeck_helper
            OBJ['record'] = OBJ['cmd'].record_helper
        elif a[1] == 'vpin':
            OBJ['vpin'] = tmc.TMCVirtualPinHelper(CFG, MCU_TMC)
        tr('new ok')
    elif op == 'echeck':
        ec = OBJ['echeck']
        if a[1] == 'query':
            info = ec.drv_status_reg_info if a[2] == 'drv' else ec.gstat_reg_info
            try:
                tr('ret %r' % (ec._query_register(info, len(a) > 3),))
            finally:
                tr('last %d' % (info[0],))
        elif a[1] == 'start':
            tr('ret %r' % (ec.start_checks(),))
        elif a[1] == 'stop':
            ec.stop_checks()
        elif a[1] == 'periodic':
            tr('ret %r' % (ec._do_periodic_check(float(a[2])),))
        elif a[1] == 'status':
            tr('ret %r' % (ec.get_status(),))
        elif a[1] == 'qtemp':
            ec._query_temperature()
    elif op == 'record':
        rec = OBJ['record']
        if a[1] == 'start':
            rec._start()
        elif a[1] == 'stop':
            rec._stop()
        elif a[1] == 'query':
            tr('ret %r' % (rec._query_tmc(float(a[2])),))
        elif a[1] == 'dump':
            tr('ret %r' % (rec._dump(0.),))
    elif op == 'cmd':
        c = OBJ['cmd']
        if a[1] == 'init_regs':
            pt = None if len(a) < 3 else float(a[2])
            c._init_registers(pt)
        elif a[1] == 'phases':
            tr('ret %r' % (c._get_phases(),))
        elif a[1] == 'offset':
            tr('ret %r' % (c.get_phase_offset(),))
        elif a[1] == 'status':
            tr('ret %r' % (c.get_status(),))
        elif a[1] == 'dump_setup':
            c.setup_register_dump(list(READS), translate if len(a) > 2 else None)
    elif op == 'gcode':
        cmd = a[1]
        params = {}
        for t in a[2:]:
            k, v = t.split('=', 1)
            params[k] = v
        func = GCODE.mux.get((cmd, params.get('STEPPER')))
        if func is None:
            tr('NOCMD')
        else:
            gcmd = gcode_mod.GCodeCommand(GCODE, cmd, ' '.join(a[1:]), params, False)
            func(gcmd)
    elif op == 'ev':
        st = STEPPER_X if a[2] == 'X' else STEPPER_Y
        ev = 'stepper:sync_mcu_position' if a[1] == 'sync' else 'stepper:set_dir_inverted'
        PRINTER.send_event(ev, st)
    elif op == 'identify':
        PRINTER.send_event('klippy:mcu_identify')
    elif op == 'connect':
        PRINTER.send_event('klippy:connect')
    elif op == 'fire':
        STEPPER_X.fire_active(float(a[1]))
    elif op == 'motor':
        el = SE.lookup_enable('stepper_x')
        if a[1] == 'on':
            el.motor_enable(float(a[2]))
        else:
            el.motor_disable(float(a[2]))
    elif op == 'runcb':
        PRINTER.reactor.run_callbacks()
    elif op == 'vpin':
        v = OBJ['vpin']
        if a[1] == 'setup':
            params = {'chip': v, 'chip_name': 'tmc_x', 'pin': a[3], 'invert': pint(a[4]),
                      'pullup': pint(a[5]), 'share_type': None}
            r = v.setup_pin(a[2], params)
            tr('ret %s' % ('Endstop' if r is ENDSTOP else repr(r),))
        elif a[1] == 'begin':
            PRINTER.send_event('homing:homing_move_begin',
                               FakeHMove([ENDSTOP] if a[2] == 'yes' else []))
        elif a[1] == 'end':
            PRINTER.send_event('homing:homing_move_end',
                               FakeHMove([ENDSTOP] if a[2] == 'yes' else []))
    elif op == 'helper':
        if a[1] == 'wave':
            tmc.TMCWaveTableHelper(CFG, MCU_TMC)
        elif a[1] == 'micro':
            tmc.TMCMicrostepHelper(CFG, MCU_TMC)
        elif a[1] == 'stealth':
            tmc.TMCStealthchopHelper(CFG, MCU_TMC)
        elif a[1] == 'vcool':
            tmc.TMCVcoolthrsHelper(CFG, MCU_TMC)
        elif a[1] == 'vhigh':
            tmc.TMCVhighHelper(CFG, MCU_TMC)
    elif op == 'tstep':
        ps = STEPPER_X if a[2] == 'st' else None
        cf = CFG if a[2] == 'cfg' else None
        tr('ret %r' % (tmc.TMCtstepHelper(MCU_TMC, float(a[1]), pstepper=ps, config=cf),))
    else:
        raise Exception('unknown step %s' % (op,))

def final_trace():
    return '\n'.join(T + ['--- logs ---'] + PY_LOGS)
"##;

// ---------------------------------------------------------------------------
// Rust-side trace and logs
// ---------------------------------------------------------------------------

static TRACE: LazyLock<GCell<Vec<String>>> = LazyLock::new(|| GCell::new(Vec::new()));
static LOGS: LazyLock<GCell<Vec<String>>> = LazyLock::new(|| GCell::new(Vec::new()));

fn tr(s: impl Into<String>) {
    TRACE.borrow_mut().push(s.into());
}

fn install_log_sink() {
    let _ = log::set_sink(Box::new(|rec| {
        LOGS.borrow_mut().push(format!("{}:{}", rec.level.name(), rec.message));
    }));
}

fn fo(v: Option<f64>) -> String {
    match v {
        Some(v) => float_repr(v),
        None => "None".to_string(),
    }
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn pint(t: &str) -> i64 {
    if let Some(h) = t.strip_prefix("0x") {
        i64::from_str_radix(h, 16).unwrap()
    } else if let Some(h) = t.strip_prefix("-0x") {
        -i64::from_str_radix(h, 16).unwrap()
    } else {
        t.parse().unwrap()
    }
}

fn popt(a: &[&str], i: usize) -> Option<i64> {
    match a.get(i) {
        None => None,
        Some(&"-") => None,
        Some(t) => Some(pint(t)),
    }
}

fn popts<'a>(a: &[&'a str], i: usize) -> Option<&'a str> {
    match a.get(i) {
        None => None,
        Some(&"-") => None,
        Some(t) => Some(*t),
    }
}

fn pfloat(t: &str) -> Option<f64> {
    if t == "None" {
        None
    } else {
        Some(t.parse().unwrap())
    }
}

fn unimpl<T>(what: &str) -> T {
    unimplemented!("{what} is not used by tmc")
}

// ---------------------------------------------------------------------------
// Rust fakes
// ---------------------------------------------------------------------------

struct CfgData {
    sections: Vec<(String, Vec<(String, String)>)>,
}

struct FakeConfig {
    printer: Printer,
    data: Arc<CfgData>,
    section: String,
}

impl FakeConfig {
    fn raw(&self, option: &str) -> Option<String> {
        let lower = option.to_lowercase();
        // configparser (strict=False) keeps the last value of a repeated option.
        let mut found = None;
        for (name, opts) in &self.data.sections {
            if *name == self.section {
                for (k, v) in opts {
                    if k.to_lowercase() == lower {
                        found = Some(v.clone());
                    }
                }
            }
        }
        found
    }

    #[allow(clippy::too_many_arguments)]
    fn wrapper<T: Copy + PartialOrd>(
        &self,
        option: &str,
        default: Option<Option<T>>,
        parse: &dyn Fn(&str) -> Option<T>,
        show: &dyn Fn(T) -> String,
        minval: Option<T>,
        maxval: Option<T>,
        above: Option<T>,
        below: Option<T>,
    ) -> KResult<Option<T>> {
        let raw = match self.raw(option) {
            Some(r) => r,
            None => {
                return match default {
                    Some(d) => Ok(d),
                    None => Err(KlippyError::config(format!(
                        "Option '{}' in section '{}' must be specified",
                        option, self.section
                    ))),
                }
            }
        };
        let v = match parse(&raw) {
            Some(v) => v,
            None => {
                return Err(KlippyError::config(format!(
                    "Unable to parse option '{}' in section '{}'",
                    option, self.section
                )))
            }
        };
        if let Some(m) = minval {
            if v < m {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must have minimum of {}",
                    option,
                    self.section,
                    show(m)
                )));
            }
        }
        if let Some(m) = maxval {
            if v > m {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must have maximum of {}",
                    option,
                    self.section,
                    show(m)
                )));
            }
        }
        if let Some(m) = above {
            if v <= m {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be above {}",
                    option,
                    self.section,
                    show(m)
                )));
            }
        }
        if let Some(m) = below {
            if v >= m {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be below {}",
                    option,
                    self.section,
                    show(m)
                )));
            }
        }
        Ok(Some(v))
    }
}

fn parse_i64(raw: &str) -> Option<i64> {
    int_from_string(raw, 10).ok().and_then(|n| n.0.to_string().parse().ok())
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.raw(option) {
            Some(r) => Ok(Some(r)),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.wrapper(
            option,
            opts.default,
            &parse_i64,
            &|v| v.to_string(),
            opts.minval,
            opts.maxval,
            None,
            None,
        )
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.wrapper(
            option,
            opts.default,
            &|raw| float_from_string(raw).ok(),
            &|v| float_str(v),
            opts.minval,
            opts.maxval,
            opts.above,
            opts.below,
        )
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        self.wrapper(
            option,
            opts.default,
            &|raw| match raw.trim().to_lowercase().as_str() {
                "1" | "yes" | "true" | "on" => Some(true),
                "0" | "no" | "false" | "off" => Some(false),
                _ => None,
            },
            &|v| py_bool(v).to_string(),
            None,
            None,
            None,
            None,
        )
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        assert!(opts.default.is_none(), "the test only reads required choices");
        let PyValue::Dict(map) = choices else { panic!("choices must be a dict") };
        let c = self
            .getint(option, GetIntOpts::default())?
            .expect("required option yields a value");
        let key = PyValue::Int(pycompat::PyInt::from_i64(c));
        match map.get(&key) {
            Some(v) => Ok(v.clone()),
            None => Err(KlippyError::config(format!(
                "Choice '{}' for option '{}' in section '{}' is not a valid choice",
                c, option, self.section
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        unimpl("getlists")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        unimpl("getlist")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        unimpl("getintlist")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        unimpl("getfloatlist")
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            data: self.data.clone(),
            section: section.to_string(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        self.data.sections.iter().any(|(n, _)| n == section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        unimpl("get_prefix_sections")
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        unimpl("get_prefix_options")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {
        unimpl("deprecate")
    }
}

struct PrinterState {
    objects: IndexMap<String, ObjectRef>,
    handlers: IndexMap<String, Vec<EventCallback>>,
}

struct FakePrinter {
    reactor: Reactor,
    state: GCell<PrinterState>,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor,
            state: GCell::new(PrinterState { objects: IndexMap::new(), handlers: IndexMap::new() }),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        unimpl("get_state_message")
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
        unimpl("update_error_msg")
    }
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.state.borrow_mut().objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.state.borrow().objects.get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::config(format!("Unknown config object '{}'", name)))
            }
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        unimpl("lookup_objects")
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        unimpl("run")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        unimpl("set_rollover_info")
    }
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        tr(format!("invoke_shutdown({})", str_repr(msg)));
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        unimpl("invoke_async_shutdown")
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.state.borrow_mut().handlers.entry(event.to_string()).or_default().push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        // Take the callbacks out so a handler may register more handlers.
        let mut callbacks = self.state.borrow_mut().handlers.shift_remove(event).unwrap_or_default();
        let mut result = Ok(Vec::new());
        for cb in callbacks.iter_mut() {
            if let Err(e) = cb(params) {
                result = Err(e);
                break;
            }
        }
        let mut state = self.state.borrow_mut();
        let entry = state.handlers.entry(event.to_string()).or_default();
        callbacks.extend(std::mem::take(entry));
        *entry = callbacks;
        result
    }
    fn request_exit(&self, _result: &str) {
        unimpl("request_exit")
    }
}



type MuxHandler = Arc<kos::sync::Mutex<GCodeCommandHandler>>;
type ActiveCallbacks = GCell<Vec<Box<dyn FnMut(f64) + Send>>>;

#[derive(Default)]
struct FakeGCode {
    mux: GCell<IndexMap<(String, String), MuxHandler>>,
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        tr(format!("register_command({})", cmd));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        let desc = match desc {
            Some(d) => str_repr(d),
            None => "None".to_string(),
        };
        tr(format!(
            "register_mux_command({}, {}, {}, {})",
            cmd,
            key,
            value.unwrap_or("None"),
            desc
        ));
        self.mux.borrow_mut().insert(
            (cmd.to_string(), value.unwrap_or("None").to_string()),
            Arc::new(kos::sync::Mutex::new(func)),
        );
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        unimpl("get_command_help")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        unimpl("get_status")
    }
    fn register_output_handler(&self, _callback: OutputCallback) {
        unimpl("register_output_handler")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        unimpl("request_restart")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        unimpl("run_script")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        unimpl("run_script_from_command")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        unreachable_mutex()
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        unimpl("create_gcode_command")
    }
    fn respond_raw(&self, _msg: &str) {
        unimpl("respond_raw")
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        tr(format!("respond_info({})", str_repr(msg)));
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        unimpl("is_traditional_gcode")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        unimpl("cmd_m112")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        unimpl("process_commands")
    }
}

// `get_mutex` hands out a mutex of the world's reactor; set once per world.
static WORLD_REACTOR: LazyLock<GCell<Option<Reactor>>> = LazyLock::new(|| GCell::new(None));

fn unreachable_mutex() -> reactor::ReactorMutex {
    WORLD_REACTOR.borrow().as_ref().expect("world reactor").mutex(false)
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGCmd {
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCmd {
    fn error(&self, msg: String) -> KlippyError {
        KlippyError::command(format!("Error on '{}': {}", self.commandline, msg))
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(self.error(format!("missing {}", name))),
            },
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                Some(d) => Ok(d),
                None => Err(self.error(format!("missing {}", name))),
            };
        };
        let Some(v) = parse_i64(raw) else {
            return Err(self.error(format!("unable to parse {}", raw)));
        };
        if let Some(m) = opts.minval {
            if v < m {
                return Err(self.error(format!("{} must have minimum of {}", name, m)));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(self.error(format!("{} must have maximum of {}", name, m)));
            }
        }
        Ok(Some(v))
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                Some(d) => Ok(d),
                None => Err(self.error(format!("missing {}", name))),
            };
        };
        let Ok(v) = float_from_string(raw) else {
            return Err(self.error(format!("unable to parse {}", raw)));
        };
        if let Some(m) = opts.minval {
            if v < m {
                return Err(self.error(format!("{} must have minimum of {}", name, float_str(m))));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(self.error(format!("{} must have maximum of {}", name, float_str(m))));
            }
        }
        if let Some(m) = opts.above {
            if v <= m {
                return Err(self.error(format!("{} must be above {}", name, float_str(m))));
            }
        }
        if let Some(m) = opts.below {
            if v >= m {
                return Err(self.error(format!("{} must be below {}", name, float_str(m))));
            }
        }
        Ok(Some(v))
    }
    fn get_command(&self) -> String {
        unimpl("get_command")
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        unimpl("get_raw_command_parameters")
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        tr(format!("respond_info({})", str_repr(msg)));
    }
    fn respond_raw(&self, _msg: &str) {
        unimpl("respond_raw")
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        unimpl("ack")
    }
}

struct FakeFactory;

impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        unimpl("printer_stepper")
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        unimpl("lookup_rail")
    }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        unimpl("lookup_multi_rail")
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        unimpl("generic_rail")
    }
    /// `stepper.parse_step_distance` for a config without `gear_ratio`.
    fn parse_step_distance(
        &self,
        config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        let rotation_dist = config
            .getfloat(
                "rotation_distance",
                GetFloatOpts { default: None, above: Some(0.), ..Default::default() },
            )?
            .expect("required");
        let microsteps = config
            .getint("microsteps", GetIntOpts { default: None, minval: Some(1), ..Default::default() })?
            .expect("required");
        let full_steps = config
            .getint(
                "full_steps_per_rotation",
                GetIntOpts { default: Some(Some(200)), minval: Some(1), ..Default::default() },
            )?
            .expect("has default");
        if full_steps % 4 != 0 {
            return Err(KlippyError::config(format!(
                "full_steps_per_rotation invalid in section '{}'",
                config.get_name()
            )));
        }
        Ok((rotation_dist, (full_steps * microsteps) as f64 * 1.0))
    }
}

struct FakeToolHead;

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        unimpl("get_position")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        unimpl("set_position")
    }
    fn get_kinematics(&self) -> Kinematics {
        unimpl("get_kinematics")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        unimpl("flush_step_generation")
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        tr("get_last_move_time");
        Ok(10.5)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> {
        unimpl("dwell")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        unimpl("drip_move")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        unimpl("move")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        unimpl("manual_move")
    }
    fn wait_moves(&self) -> KResult<()> {
        tr("wait_moves");
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        unimpl("get_status")
    }
    fn get_trapq(&self) -> Trapq {
        unimpl("get_trapq")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        unimpl("get_max_velocity")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        unimpl("set_max_velocities")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        unimpl("register_lookahead_callback")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        unimpl("check_busy")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        unimpl("limit_next_junction_speed")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        unimpl("set_extruder")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        unimpl("get_extruder")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> {
        unimpl("add_extra_axis")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> {
        unimpl("remove_extra_axis")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        unimpl("get_extra_axes")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        unimpl("stats")
    }
    fn stepper_factory(&self) -> StepperFactory {
        Arc::new(FakeFactory)
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct FakeHeaters;

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {
        unimpl("add_sensor_factory")
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        unimpl("setup_heater")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        unimpl("get_all_heaters")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        unimpl("lookup_heater")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        unimpl("setup_sensor")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        unimpl("register_sensor")
    }
    fn register_monitor(&self, config: &Config) {
        tr(format!("register_monitor({})", config.get_name()));
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        unimpl("turn_off_all_heaters")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        unimpl("set_temperature")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

struct FakeWebhooks;

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, _path: &str, _callback: WebHooksCallback) -> KResult<()> {
        unimpl("register_endpoint")
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        _callback: WebHooksCallback,
    ) -> KResult<()> {
        tr(format!("register_mux_endpoint({}, {}, {})", path, key, value));
        Ok(())
    }
    fn call_remote_method(
        &self,
        _method: &str,
        _params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        unimpl("call_remote_method")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        unimpl("get_status")
    }
}

klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

struct FakeForceMove {
    steppers: Vec<(String, Stepper)>,
}

impl ForceMoveApi for FakeForceMove {
    fn lookup_stepper(&self, name: Option<&str>) -> KResult<Stepper> {
        let name = name.unwrap_or("None");
        tr(format!("lookup_stepper({})", name));
        for (n, s) in &self.steppers {
            if n == name {
                return Ok(s.clone());
            }
        }
        Err(KlippyError::config(format!("Unknown stepper {}", name)))
    }
    fn manual_move(&self, _stepper: &Stepper, _dist: f64, _speed: f64, _accel: f64) -> KResult<()> {
        unimpl("manual_move")
    }
}

klippy_api::printer_object!(FakeForceMove; dyn ForceMoveApi);

struct FakeStepper {
    name: String,
    active_callbacks: ActiveCallbacks,
    dir_inv: GCell<bool>,
    mcu_pos: GCell<i64>,
    step_dist: GCell<f64>,
    both_edge: GCell<bool>,
}

impl FakeStepper {
    fn new(name: &str) -> Arc<FakeStepper> {
        Arc::new(FakeStepper {
            name: name.to_string(),
            active_callbacks: GCell::new(Vec::new()),
            dir_inv: GCell::new(false),
            mcu_pos: GCell::new(0),
            step_dist: GCell::new(0.0125),
            both_edge: GCell::new(false),
        })
    }

    fn fire_active(&self, print_time: f64) {
        let cbs = std::mem::take(&mut *self.active_callbacks.borrow_mut());
        for mut cb in cbs {
            cb(print_time);
        }
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        unimpl("get_mcu")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        unimpl("get_oid")
    }
    fn units_in_radians(&self) -> bool {
        unimpl("units_in_radians")
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (Some(0.000002), self.both_edge.get())
    }
    fn setup_default_pulse_duration(&self, pulse_duration: f64, step_both_edge: bool) {
        tr(format!(
            "setup_default_pulse_duration({}, {})",
            float_repr(pulse_duration),
            py_bool(step_both_edge)
        ));
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        unimpl("setup_itersolve")
    }
    fn get_step_dist(&self) -> f64 {
        self.step_dist.get()
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        unimpl("get_rotation_distance")
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {
        unimpl("set_rotation_distance")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        (self.dir_inv.get(), self.dir_inv.get())
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        unimpl("set_dir_inverted")
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        unimpl("calc_position_from_coord")
    }
    fn set_position(&self, _coord: [f64; 3]) {
        unimpl("set_position")
    }
    fn get_commanded_position(&self) -> f64 {
        unimpl("get_commanded_position")
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        self.mcu_pos.get()
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        unimpl("get_past_mcu_position")
    }
    fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64 {
        (mcu_pos + 3) as f64 * self.step_dist.get()
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<PullHistoryStep>, i64) {
        unimpl("dump_steps")
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        unimpl("get_stepper_kinematics")
    }
    fn set_stepper_kinematics(&self, _sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        unimpl("set_stepper_kinematics")
    }
    fn note_homing_end(&self) -> KResult<()> {
        unimpl("note_homing_end")
    }
    fn get_trapq(&self) -> Option<Trapq> {
        unimpl("get_trapq")
    }
    fn set_trapq(&self, _tq: Option<Trapq>) -> Option<Trapq> {
        unimpl("set_trapq")
    }
    fn add_active_callback(&self, cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        self.active_callbacks.borrow_mut().push(cb);
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        unimpl("is_active_axis")
    }
}

klippy_api::printer_object!(FakeStepper; dyn McuStepperApi);

struct FakeDigitalOut {
    label: String,
}

impl DigitalOutPinApi for FakeDigitalOut {
    fn get_mcu(&self) -> Mcu {
        unimpl("get_mcu")
    }
    fn setup_max_duration(&self, max_duration: f64) {
        tr(format!("{}.setup_max_duration({})", self.label, float_repr(max_duration)));
    }
    fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {
        unimpl("setup_start_value")
    }
    fn set_digital(&self, print_time: f64, value: bool) {
        tr(format!("{}.set_digital({}, {})", self.label, float_repr(print_time), i64::from(value)));
    }
}

struct FakeEndstop;

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> Mcu {
        unimpl("get_mcu")
    }
    fn add_stepper(&self, _stepper: Stepper) {
        unimpl("add_stepper")
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        unimpl("get_steppers")
    }
    fn home_start(
        &self,
        _print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> Completion<bool> {
        unimpl("home_start")
    }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        unimpl("home_wait")
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        unimpl("query_endstop")
    }
}

/// The MCU behind every fake pin, doubling as the `chip` of `lookup_pin`.
struct FakeMcu {
    count: GCell<i64>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        unimpl("get_printer")
    }
    fn is_fileoutput(&self) -> bool {
        unimpl("is_fileoutput")
    }
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        assert_eq!(pin_type, "digital_out");
        self.count.set(self.count.get() + 1);
        let label = format!("pin{}", self.count.get());
        tr(format!(
            "chip.setup_pin({}, {}:{})",
            pin_type, pin_params.chip_name, pin_params.pin
        ));
        Ok(Pin::DigitalOut(Arc::new(FakeDigitalOut { label }) as DigitalOutPin))
    }
    fn create_oid(&self) -> KResult<i64> {
        unimpl("create_oid")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        unimpl("register_config_callback")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        unimpl("add_config_cmd")
    }
    fn request_move_queue_slot(&self) {
        unimpl("request_move_queue_slot")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        unimpl("get_query_slot")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        unimpl("seconds_to_clock")
    }
    fn min_schedule_time(&self) -> f64 {
        unimpl("min_schedule_time")
    }
    fn max_nominal_duration(&self) -> f64 {
        unimpl("max_nominal_duration")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        unimpl("lookup_command")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        unimpl("lookup_query_command")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        unimpl("try_lookup_command")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        unimpl("alloc_command_queue")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        unimpl("register_serial_response")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        unimpl("check_valid_response")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        unimpl("get_enumerations")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        unimpl("get_constants")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        unimpl("get_constant_float")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        unimpl("print_time_to_clock")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        unimpl("clock_to_print_time")
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime * 2. + 0.25
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        unimpl("clock32_to_clock64")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        unimpl("calibrate_clock")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        unimpl("get_status")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        unimpl("stats")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        unimpl("create_trigger_dispatch")
    }
}

struct FakePins {
    mcu: Mcu,
    active: GCell<IndexMap<String, PinParams>>,
    endstop: Endstop,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        unimpl("parse_pin")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let mut desc = pin_desc.trim();
        let mut invert = 0;
        if opts.can_invert && desc.starts_with('!') {
            invert = 1;
            desc = &desc[1..];
        }
        let (chip_name, pin) = match desc.split_once(':') {
            None => ("mcu".to_string(), desc.to_string()),
            Some((c, p)) => (c.trim().to_string(), p.trim().to_string()),
        };
        let share_name = format!("{}:{}", chip_name, pin);
        if let Some(share_params) = self.active.borrow().get(&share_name).cloned() {
            if opts.share_type.is_none() || opts.share_type != share_params.share_type {
                return Err(KlippyError::config(format!(
                    "pin {} used multiple times in config",
                    pin
                )));
            }
            return Ok(share_params);
        }
        let params = PinParams {
            chip: self.mcu.clone(),
            chip_name,
            pin,
            invert,
            pullup: 0,
            share_type: opts.share_type,
        };
        self.active.borrow_mut().insert(share_name, params.clone());
        Ok(params)
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        tr(format!("pins.setup_pin({}, {})", pin_type, pin_desc));
        Ok(Pin::Endstop(self.endstop.clone()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        unimpl("reset_pin_sharing")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        unimpl("get_pin_resolver")
    }
    fn register_chip(&self, chip_name: &str, _chip: Chip) -> KResult<()> {
        tr(format!("register_chip({})", chip_name));
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        unimpl("allow_multi_use_pin")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeHMove {
    endstops: Vec<Endstop>,
}

impl HomingMoveApi for FakeHMove {
    fn get_mcu_endstops(&self) -> Vec<Endstop> {
        self.endstops.clone()
    }
    fn check_no_movement(&self) -> Option<String> {
        unimpl("check_no_movement")
    }
}

// ---- mcu_tmc and current helper ----

#[derive(Clone)]
enum Item {
    Val(i64),
    Err(String),
    Raw(i64, f64, Option<i64>),
}

fn parse_item(t: &str) -> Item {
    if let Some(m) = t.strip_prefix('!') {
        Item::Err(m.replace('_', " "))
    } else if let Some(r) = t.strip_prefix("raw:") {
        let parts: Vec<&str> = r.split(':').collect();
        let spi = parts.get(2).map(|s| pint(s));
        Item::Raw(pint(parts[0]), parts[1].parse().unwrap(), spi)
    } else {
        Item::Val(pint(t))
    }
}

struct FakeMcuTmc {
    fields: Arc<FieldHelper>,
    regs: Vec<(String, i64)>,
    freq: Option<f64>,
    script: GCell<IndexMap<String, Vec<Item>>>,
    setfail: GCell<IndexMap<String, String>>,
    mcu: Mcu,
}

impl FakeMcuTmc {
    fn item(&self, name: &str) -> Item {
        let mut script = self.script.borrow_mut();
        match script.get_mut(name) {
            Some(seq) if seq.len() > 1 => seq.remove(0),
            Some(seq) => seq[0].clone(),
            None => Item::Val(0),
        }
    }
}

impl McuTmcApi for FakeMcuTmc {
    fn get_fields(&self) -> Arc<FieldHelper> {
        self.fields.clone()
    }
    fn get_register(&self, reg_name: &str) -> KResult<i64> {
        tr(format!("get_register({})", reg_name));
        match self.item(reg_name) {
            Item::Err(m) => Err(KlippyError::command(m)),
            Item::Val(v) => Ok(v),
            Item::Raw(v, _, _) => Ok(v),
        }
    }
    fn get_register_raw(&self, reg_name: &str) -> KResult<RegisterRaw> {
        tr(format!("get_register_raw({})", reg_name));
        match self.item(reg_name) {
            Item::Err(m) => Err(KlippyError::command(m)),
            Item::Val(v) => Ok(RegisterRaw { data: v, receive_time: 0.5, spi_status: None }),
            Item::Raw(v, t, s) => Ok(RegisterRaw { data: v, receive_time: t, spi_status: s }),
        }
    }
    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        tr(format!("set_register({}, {}, {})", reg_name, val, fo(print_time)));
        if let Some(m) = self.setfail.borrow().get(reg_name) {
            return Err(KlippyError::command(m.clone()));
        }
        Ok(())
    }
    fn get_tmc_frequency(&self) -> Option<f64> {
        self.freq
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn name_to_reg_value(&self, reg_name: &str) -> Option<i64> {
        self.regs.iter().find(|(n, _)| n == reg_name).map(|(_, v)| *v)
    }
}

struct FakeCur {
    hold_none: bool,
    cur: GCell<[Option<f64>; 4]>,
    fail: GCell<Option<String>>,
}

impl TmcCurrentHelperApi for FakeCur {
    fn get_current(&self) -> (f64, Option<f64>, Option<f64>, f64) {
        let c = self.cur.get();
        (c[0].unwrap(), c[1], c[2], c[3].unwrap())
    }
    fn set_current(
        &self,
        run_current: f64,
        hold_current: Option<f64>,
        print_time: f64,
    ) -> KResult<()> {
        tr(format!(
            "set_current({}, {}, {})",
            float_repr(run_current),
            fo(hold_current),
            float_repr(print_time)
        ));
        if let Some(m) = self.fail.cloned() {
            return Err(KlippyError::command(m));
        }
        if self.hold_none {
            self.cur.set([Some(run_current), None, None, Some(2.0)]);
        } else {
            self.cur.set([Some(run_current), hold_current, hold_current, Some(2.0)]);
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Driver profiles (register maps shared with the Python side)
// ---------------------------------------------------------------------------

type FieldTable = Vec<(&'static str, Vec<(&'static str, i64)>)>;

struct Profile {
    prefix: &'static str,
    fields: FieldTable,
    signed: Vec<&'static str>,
    regs: Vec<(&'static str, i64)>,
    freq: Option<f64>,
    reads: Vec<&'static str>,
    hold_none: bool,
}

fn mslut_fields() -> FieldTable {
    let mut t: FieldTable = Vec::new();
    for (i, n) in ["mslut0", "mslut1", "mslut2", "mslut3", "mslut4", "mslut5", "mslut6", "mslut7"]
        .iter()
        .enumerate()
    {
        let reg: &'static str = ["MSLUT0", "MSLUT1", "MSLUT2", "MSLUT3", "MSLUT4", "MSLUT5", "MSLUT6", "MSLUT7"][i];
        t.push((reg, vec![(*n, 0xffffffff)]));
    }
    t.push((
        "MSLUTSEL",
        vec![
            ("w0", 0x3),
            ("w1", 0xc),
            ("w2", 0x30),
            ("w3", 0xc0),
            ("x1", 0xff00),
            ("x2", 0xff0000),
            ("x3", 0xff000000),
        ],
    ));
    t.push(("MSLUTSTART", vec![("start_sin", 0xff), ("start_sin90", 0xff0000)]));
    t
}

fn spi_profile() -> Profile {
    let mut fields: FieldTable = vec![
        (
            "GCONF",
            vec![
                ("i_scale_analog", 0x01),
                ("en_pwm_mode", 0x04),
                ("diag0_stall", 0x8000),
                ("diag1_stall", 0x10000),
            ],
        ),
        ("GSTAT", vec![("reset", 1), ("drv_err", 2), ("uv_cp", 4)]),
        ("IHOLD_IRUN", vec![("ihold", 0x1f), ("irun", 0x1f00), ("iholddelay", 0xf0000)]),
        ("TPWMTHRS", vec![("tpwmthrs", 0xfffff)]),
        ("TCOOLTHRS", vec![("tcoolthrs", 0xfffff)]),
        ("THIGH", vec![("thigh", 0xfffff)]),
    ];
    fields.extend(mslut_fields());
    fields.extend(vec![
        (
            "CHOPCONF",
            vec![
                ("toff", 0xf),
                ("hstrt", 0x70),
                ("mres", 0x0f000000),
                ("intpol", 0x10000000),
                ("dedge", 0x20000000),
            ],
        ),
        ("COOLCONF", vec![("sgt", 0x7f0000), ("sfilt", 0x1000000)]),
        ("MSCNT", vec![("mscnt", 0x3ff)]),
        (
            "DRV_STATUS",
            vec![
                ("sg_result", 0x3ff),
                ("stealth", 0x4000),
                ("fsactive", 0x8000),
                ("cs_actual", 0x1f0000),
                ("stallguard", 0x1000000),
                ("ot", 0x2000000),
                ("otpw", 0x4000000),
                ("s2ga", 0x8000000),
                ("s2gb", 0x10000000),
                ("stst", 0x80000000),
            ],
        ),
    ]);
    Profile {
        prefix: "tmc2130",
        fields,
        signed: vec!["sgt"],
        regs: vec![
            ("GCONF", 0),
            ("GSTAT", 1),
            ("IHOLD_IRUN", 16),
            ("TPWMTHRS", 19),
            ("TCOOLTHRS", 20),
            ("THIGH", 21),
            ("MSLUT0", 96),
            ("MSLUT1", 97),
            ("MSLUT2", 98),
            ("MSLUT3", 99),
            ("MSLUT4", 100),
            ("MSLUT5", 101),
            ("MSLUT6", 102),
            ("MSLUT7", 103),
            ("MSLUTSEL", 104),
            ("MSLUTSTART", 105),
            ("MSCNT", 106),
            ("CHOPCONF", 108),
            ("COOLCONF", 109),
            ("DRV_STATUS", 111),
        ],
        freq: Some(13200000.0),
        reads: vec!["GSTAT", "DRV_STATUS", "MSCNT"],
        hold_none: false,
    }
}

fn uart_profile() -> Profile {
    let fields: FieldTable = vec![
        (
            "GCONF",
            vec![
                ("i_scale_analog", 1),
                ("en_spreadcycle", 4),
                ("pdn_disable", 0x40),
                ("mstep_reg_select", 0x80),
            ],
        ),
        ("GSTAT", vec![("reset", 1), ("drv_err", 2), ("uv_cp", 4)]),
        ("IFCNT", vec![("ifcnt", 0xff)]),
        ("IHOLD_IRUN", vec![("ihold", 0x1f), ("irun", 0x1f00), ("iholddelay", 0xf0000)]),
        ("TPWMTHRS", vec![("tpwmthrs", 0xfffff)]),
        ("TCOOLTHRS", vec![("tcoolthrs", 0xfffff)]),
        ("SGTHRS", vec![("sgthrs", 0xff)]),
        ("MSCNT", vec![("mscnt", 0x3ff)]),
        (
            "CHOPCONF",
            vec![
                ("toff", 0xf),
                ("hstrt", 0x70),
                ("mres", 0x0f000000),
                ("intpol", 0x10000000),
                ("dedge", 0x20000000),
            ],
        ),
        (
            "DRV_STATUS",
            vec![
                ("otpw", 1),
                ("ot", 2),
                ("s2ga", 4),
                ("s2gb", 8),
                ("s2vsa", 16),
                ("s2vsb", 32),
                ("cs_actual", 0x1f0000),
                ("stealth", 0x40000000),
            ],
        ),
        ("SG_RESULT", vec![("sg_result", 0x3ff)]),
    ];
    Profile {
        prefix: "tmc2209",
        fields,
        signed: vec![],
        regs: vec![
            ("GCONF", 0),
            ("GSTAT", 1),
            ("IFCNT", 2),
            ("IHOLD_IRUN", 16),
            ("TPWMTHRS", 19),
            ("TCOOLTHRS", 20),
            ("SGTHRS", 64),
            ("SG_RESULT", 65),
            ("MSCNT", 106),
            ("CHOPCONF", 108),
            ("DRV_STATUS", 111),
        ],
        freq: Some(12000000.0),
        reads: vec!["GSTAT", "IFCNT", "DRV_STATUS", "MSCNT", "SG_RESULT"],
        hold_none: false,
    }
}

fn sg4_profile() -> Profile {
    let mut fields: FieldTable = vec![
        (
            "GCONF",
            vec![("en_pwm_mode", 0x04), ("diag0_stall", 0x8000), ("diag1_stall", 0x10000)],
        ),
        ("GSTAT", vec![("reset", 1), ("drv_err", 2), ("uv_cp", 4)]),
        ("IHOLD_IRUN", vec![("ihold", 0x1f), ("irun", 0x1f00), ("iholddelay", 0xf0000)]),
        ("TPWMTHRS", vec![("tpwmthrs", 0xfffff)]),
        ("TCOOLTHRS", vec![("tcoolthrs", 0xfffff)]),
        ("THIGH", vec![("thigh", 0xfffff)]),
    ];
    fields.extend(mslut_fields());
    fields.extend(vec![
        (
            "CHOPCONF",
            vec![
                ("toff", 0xf),
                ("hstrt", 0x70),
                ("mres", 0x0f000000),
                ("intpol", 0x10000000),
                ("dedge", 0x20000000),
            ],
        ),
        ("MSCNT", vec![("mscnt", 0x3ff)]),
        ("SG4_THRS", vec![("sg4_thrs", 0xff), ("sg4_filt_en", 0x100)]),
        ("ADC_TEMP", vec![("adc_temp", 0x1fff)]),
        (
            "DRV_STATUS",
            vec![
                ("sg_result", 0x3ff),
                ("s2vsa", 0x1000),
                ("s2vsb", 0x2000),
                ("stealth", 0x4000),
                ("fsactive", 0x8000),
                ("cs_actual", 0x1f0000),
                ("stallguard", 0x1000000),
                ("ot", 0x2000000),
                ("otpw", 0x4000000),
                ("s2ga", 0x8000000),
                ("s2gb", 0x10000000),
                ("stst", 0x80000000),
            ],
        ),
    ]);
    Profile {
        prefix: "tmc2240",
        fields,
        signed: vec![],
        regs: vec![
            ("GCONF", 0),
            ("GSTAT", 1),
            ("IHOLD_IRUN", 16),
            ("TPWMTHRS", 19),
            ("TCOOLTHRS", 20),
            ("THIGH", 21),
            ("MSLUT0", 96),
            ("MSLUT1", 97),
            ("MSLUT2", 98),
            ("MSLUT3", 99),
            ("MSLUT4", 100),
            ("MSLUT5", 101),
            ("MSLUT6", 102),
            ("MSLUT7", 103),
            ("MSLUTSEL", 104),
            ("MSLUTSTART", 105),
            ("MSCNT", 106),
            ("CHOPCONF", 108),
            ("DRV_STATUS", 111),
            ("SG4_THRS", 116),
            ("SG4_RESULT", 117),
            ("ADC_TEMP", 81),
        ],
        freq: Some(12500000.0),
        reads: vec!["GSTAT", "DRV_STATUS", "MSCNT", "SG4_RESULT", "ADC_TEMP"],
        hold_none: false,
    }
}

fn c2660_profile() -> Profile {
    Profile {
        prefix: "tmc2660",
        fields: vec![
            ("DRVCTRL", vec![("mres", 0xf), ("dedge", 0x100), ("intpol", 0x200)]),
            ("SGCSCONF", vec![("cs", 0x1f), ("sgt", 0x7f00)]),
            ("READRSP@RDSEL0", vec![("mstep", 0xffc00)]),
            (
                "READRSP@RDSEL2",
                vec![
                    ("sg", 0x1),
                    ("ot", 0x2),
                    ("otpw", 0x4),
                    ("s2ga", 0x8),
                    ("s2gb", 0x10),
                    ("se", 0x7c00),
                ],
            ),
        ],
        signed: vec!["sgt"],
        regs: vec![("DRVCTRL", 0), ("SGCSCONF", 0xd0000)],
        freq: None,
        reads: vec!["READRSP@RDSEL0", "READRSP@RDSEL1", "READRSP@RDSEL2"],
        hold_none: true,
    }
}

fn formatters() -> IndexMap<String, FieldFormatter> {
    let mut m: IndexMap<String, FieldFormatter> = IndexMap::new();
    m.insert("mres".to_string(), Arc::new(|v| format!("{}({}usteps)", v, 0x100i64 >> v)));
    m.insert(
        "otpw".to_string(),
        Arc::new(|v| if v != 0 { "1(OvertempWarning!)".to_string() } else { String::new() }),
    );
    m.insert(
        "i_scale_analog".to_string(),
        Arc::new(|v| if v != 0 { "1(ExtVREF)".to_string() } else { String::new() }),
    );
    m
}

fn translate() -> ReadTranslate {
    Arc::new(|reg_name: &str, val: i64| {
        (if reg_name == "GSTAT" { reg_name.to_lowercase() } else { reg_name.to_string() }, val + 1)
    })
}

fn field_map(table: &FieldTable) -> IndexMap<String, IndexMap<String, i64>> {
    table
        .iter()
        .map(|(reg, fields)| {
            (reg.to_string(), fields.iter().map(|(n, m)| (n.to_string(), *m)).collect())
        })
        .collect()
}

fn py_fields(table: &FieldTable) -> String {
    let regs: Vec<String> = table
        .iter()
        .map(|(reg, fields)| {
            let fs: Vec<String> = fields.iter().map(|(n, m)| format!("\"{}\": {}", n, m)).collect();
            format!("\"{}\": {{{}}}", reg, fs.join(", "))
        })
        .collect();
    format!("{{{}}}", regs.join(", "))
}

fn py_str_list(items: &[&str]) -> String {
    let v: Vec<String> = items.iter().map(|s| format!("\"{}\"", s)).collect();
    format!("[{}]", v.join(", "))
}

fn py_float_opt(v: Option<f64>) -> String {
    match v {
        Some(f) => format!("{:?}", f),
        None => "None".to_string(),
    }
}

// ---------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------

struct Scn {
    name: &'static str,
    profile: Profile,
    driver_name: String,
    driver_opts: Vec<(&'static str, &'static str)>,
    stepper_opts: Vec<(&'static str, &'static str)>,
    stepper_section: bool,
    extra_sections: Vec<(&'static str, Vec<(&'static str, &'static str)>)>,
    steps: Vec<&'static str>,
}

impl Scn {
    fn new(name: &'static str, profile: Profile, steps: Vec<&'static str>) -> Scn {
        let driver_name = format!("{} stepper_x", profile.prefix);
        Scn {
            name,
            profile,
            driver_name,
            driver_opts: Vec::new(),
            stepper_opts: Vec::new(),
            stepper_section: true,
            extra_sections: Vec::new(),
            steps,
        }
    }
    fn driver(mut self, opts: &[(&'static str, &'static str)]) -> Scn {
        self.driver_opts = opts.to_vec();
        self
    }
    fn stepper(mut self, opts: &[(&'static str, &'static str)]) -> Scn {
        self.stepper_opts = opts.to_vec();
        self
    }
}

fn config_sections(scn: &Scn) -> Vec<(String, Vec<(String, String)>)> {
    let mut sections: Vec<(String, Vec<(String, String)>)> = Vec::new();
    if scn.stepper_section {
        let mut opts = vec![
            ("rotation_distance".to_string(), "40".to_string()),
            ("microsteps".to_string(), "16".to_string()),
        ];
        for (k, v) in &scn.stepper_opts {
            opts.push((k.to_string(), v.to_string()));
        }
        sections.push(("stepper_x".to_string(), opts));
    }
    for (name, opts) in &scn.extra_sections {
        sections.push((
            name.to_string(),
            opts.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        ));
    }
    sections.push((
        scn.driver_name.clone(),
        scn.driver_opts.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
    ));
    sections
}

fn config_text(sections: &[(String, Vec<(String, String)>)]) -> String {
    let mut out = String::new();
    for (name, opts) in sections {
        out.push_str(&format!("[{}]\n", name));
        for (k, v) in opts {
            out.push_str(&format!("{}: {}\n", k, v));
        }
    }
    out
}

#[allow(clippy::vec_init_then_push)]
fn scenarios() -> Vec<Scn> {
    let mut v = Vec::new();

    // FieldHelper -----------------------------------------------------------
    v.push(Scn::new(
        "fields_basic",
        spi_profile(),
        vec![
            "fh_lookup toff",
            "fh_lookup nope",
            "fh_regs",
            "fh_get toff",
            "fh_set toff 3",
            "fh_set mres 4",
            "fh_set intpol 1",
            "fh_set toff 0x1f",
            "fh_get toff",
            "fh_get mres",
            "fh_get mres 0x03000000",
            "fh_get toff 0x7 CHOPCONF",
            "fh_get hstrt - GCONF",
            "fh_set toff 9 0xffffffff",
            "fh_set toff 2 - CHOPCONF",
            "fh_set mres -1",
            "fh_regs",
            "fh_get sgt",
            "fh_set sgt -3",
            "fh_get sgt",
            "fh_set sgt 63",
            "fh_get sgt",
            "fh_set sgt 64",
            "fh_get sgt",
            "fh_set sgt -64",
            "fh_get sgt",
            "fh_get sgt 0x7f0000",
            "fh_get nope",
            "fh_set nope 1",
            "fh_get toff 1 BADREG",
            "fh_get nope 0 CHOPCONF",
            "fh_pretty CHOPCONF 0x14000103",
            "fh_pretty CHOPCONF 0x2f00000f",
            "fh_pretty DRV_STATUS 0x4c1f0123",
            "fh_pretty DRV_STATUS 0",
            "fh_pretty GCONF 1",
            "fh_pretty GSTAT -1",
            "fh_pretty NOPE 5",
            "fh_regfields GSTAT 3",
            "fh_regfields CHOPCONF 0x14000103",
            "fh_regfields NOPE 1",
            "fh_regfields COOLCONF 0x7f0000",
            "fh_regs",
        ],
    ));
    v.push(
        Scn::new(
            "fields_config",
            spi_profile(),
            vec![
                "fh_cfg toff 3",
                "fh_cfg hstrt 2",
                "fh_cfg sfilt F",
                "fh_cfg mres 0",
                "fh_cfg ihold 5",
                "fh_cfg irun 31",
                "fh_cfg i_scale_analog T",
                "fh_cfg dedge F",
                "fh_cfg sgt 0",
                "fh_cfg nope 1",
                "fh_cfg intpol T",
                "fh_cfg iholddelay 7",
                "fh_cfg stallguard 1",
                "fh_regs",
            ],
        )
        .driver(&[
            ("driver_TOFF", "7"),
            ("driver_HSTRT", "9"),
            ("driver_SFILT", "maybe"),
            ("driver_MRES", "x"),
            ("driver_IHOLD", "-1"),
            ("driver_DEDGE", "1"),
            ("driver_SGT", "-5"),
            ("driver_intpol", "off"),
            ("driver_IHOLDDELAY", "99"),
        ]),
    );
    v.push(
        Scn::new(
            "fields_config_signed_limits",
            spi_profile(),
            vec!["fh_cfg sgt 1", "fh_set sgt 0", "fh_regs"],
        )
        .driver(&[("driver_SGT", "-65")]),
    );
    v.push(
        Scn::new(
            "fields_config_signed_max",
            spi_profile(),
            vec!["fh_cfg sgt 1", "fh_regs"],
        )
        .driver(&[("driver_SGT", "64")]),
    );
    v.push(
        Scn::new(
            "fields_config_signed_edge",
            spi_profile(),
            vec!["fh_cfg sgt 1", "fh_get sgt", "fh_regs"],
        )
        .driver(&[("driver_SGT", "-64")]),
    );

    // Configuration helpers -----------------------------------------------
    v.push(
        Scn::new(
            "helpers_spi",
            spi_profile(),
            vec![
                "helper micro",
                "helper stealth",
                "helper vcool",
                "helper vhigh",
                "fh_regs",
                "tstep 100 st",
                "tstep 100 cfg",
                "tstep 0 st",
                "tstep -5 cfg",
                "tstep 1e-9 st",
                "tstep 1e-320 st",
                "tstep nan st",
                "tstep inf st",
                "tstep 5 none",
                "tstep 0.5 st",
                "stepper X dist 0.003125",
                "tstep 120 st",
            ],
        )
        .driver(&[
            ("stealthchop_threshold", "120"),
            ("coolstep_threshold", "80"),
            ("high_velocity_threshold", "300"),
            ("interpolate", "false"),
        ])
        .stepper(&[("microsteps", "32")]),
    );
    v.push(Scn::new(
        "helpers_defaults",
        spi_profile(),
        vec!["helper micro", "helper stealth", "helper vcool", "helper vhigh", "fh_regs"],
    ));
    v.push(Scn::new(
        "helpers_uart_defaults",
        uart_profile(),
        vec!["helper micro", "helper stealth", "helper vcool", "fh_regs", "tstep 20 st"],
    ));
    v.push(
        Scn::new(
            "helpers_uart_thresholds",
            uart_profile(),
            vec!["helper micro", "helper stealth", "helper vcool", "fh_regs", "tstep 20 cfg"],
        )
        .driver(&[("stealthchop_threshold", "0"), ("coolstep_threshold", "35.5")])
        .stepper(&[("microsteps", "1")]),
    );
    v.push(
        Scn::new(
            "helpers_bad_microsteps",
            spi_profile(),
            vec!["helper micro", "helper stealth", "fh_regs"],
        )
        .stepper(&[("microsteps", "7")]),
    );
    v.push(
        Scn::new(
            "helpers_threshold_errors",
            spi_profile(),
            vec!["helper stealth", "helper vcool", "helper vhigh", "fh_regs"],
        )
        .driver(&[
            ("stealthchop_threshold", "-1"),
            ("coolstep_threshold", "abc"),
            ("high_velocity_threshold", "-0.5"),
        ]),
    );
    v.push(
        Scn::new(
            "helpers_no_stepper_section",
            spi_profile(),
            vec!["helper micro", "helper stealth", "fh_regs"],
        )
        .driver(&[("stealthchop_threshold", "50")])
        .no_stepper_section(),
    );
    v.push(Scn::new(
        "helpers_step_dist_errors",
        spi_profile(),
        vec!["helper micro", "tstep 10 cfg"],
    )
    .driver(&[])
    .stepper(&[("full_steps_per_rotation", "202")]));
    v.push(
        Scn::new(
            "helpers_wave_defaults",
            spi_profile(),
            vec!["helper wave", "fh_regs"],
        ),
    );
    v.push(
        Scn::new(
            "helpers_wave_overrides",
            sg4_profile(),
            vec!["helper wave", "fh_regs"],
        )
        .driver(&[
            ("driver_MSLUT0", "4294967295"),
            ("driver_W2", "2"),
            ("driver_START_SIN", "7"),
            ("driver_x3", "0"),
        ]),
    );
    v.push(
        Scn::new(
            "helpers_wave_error",
            spi_profile(),
            vec!["helper wave", "fh_regs"],
        )
        .driver(&[("driver_X1", "300")]),
    );
    v.push(
        Scn::new(
            "helpers_wave_range",
            spi_profile(),
            vec!["helper wave", "fh_regs"],
        )
        .driver(&[("driver_W0", "4"), ("driver_MSLUT7", "-1")]),
    );

    // TMCErrorCheck ----------------------------------------------------------
    v.push(Scn::new(
        "echeck_spi",
        spi_profile(),
        vec![
            "new echeck",
            "echeck status",
            "echeck start",
            "echeck status",
            "mcu script DRV_STATUS 0x04000000",
            "echeck query drv",
            "mcu script DRV_STATUS 0",
            "echeck query drv",
            "echeck status",
            "mcu script DRV_STATUS 0x02000000",
            "echeck query drv",
            "mcu script DRV_STATUS 0x02000000 0",
            "echeck query drv",
            "fh_set irun 10",
            "mcu script DRV_STATUS 0",
            "echeck query drv",
            "fh_set ihold 5",
            "echeck query drv",
            "fh_set irun 2",
            "echeck query drv",
            "mcu script DRV_STATUS 0x00040000",
            "echeck query drv",
            "mcu script DRV_STATUS 0x0c1f0000",
            "echeck query drv",
            "echeck status",
            "mcu script GSTAT 4",
            "echeck query gstat",
            "echeck query gstat clear",
            "mcu script GSTAT 0",
            "echeck query gstat clear",
            "echeck stop",
            "echeck status",
            "echeck stop",
        ],
    ));
    v.push(Scn::new(
        "echeck_spi_start_errors",
        spi_profile(),
        vec![
            "new echeck",
            "mcu script DRV_STATUS !some_error",
            "echeck start",
            "echeck status",
            "mcu script DRV_STATUS 0",
            "mcu script GSTAT 2",
            "echeck start",
            "echeck status",
            "mcu script GSTAT 0",
            "echeck start",
            "echeck status",
            "echeck periodic 10.0",
            "echeck periodic 11.5",
            "mcu script DRV_STATUS 0x08000000",
            "echeck periodic 12.5",
            "echeck status",
            "mcu script DRV_STATUS 0",
            "mcu script GSTAT !gstat_broken",
            "echeck periodic 13.25",
            "echeck start",
        ],
    ));
    v.push(Scn::new(
        "echeck_uart",
        uart_profile(),
        vec![
            "new echeck",
            "mcu script GSTAT 1 0",
            "echeck start",
            "echeck status",
            "mcu script GSTAT 4 0",
            "echeck start",
            "mcu script GSTAT 4",
            "echeck start",
            "mcu script GSTAT 0",
            "mcu script DRV_STATUS !Unable_to_read_tmc_uart_x_register_DRV_STATUS 0",
            "echeck query drv",
            "mcu script DRV_STATUS !Unable_to_read_tmc_uart_x_register_DRV_STATUS",
            "echeck query drv",
            "mcu script DRV_STATUS !Unable_to_write_x",
            "echeck query drv",
            "mcu script DRV_STATUS 0x00000020",
            "echeck query drv",
            "mcu script DRV_STATUS 0x00000001",
            "echeck query drv",
            "mcu script DRV_STATUS 0",
            "echeck periodic 20.0",
        ],
    ));
    v.push(Scn::new(
        "echeck_sg4_temperature",
        sg4_profile(),
        vec![
            "new echeck",
            "echeck status",
            "mcu script ADC_TEMP 2200",
            "echeck start",
            "echeck status",
            "echeck periodic 1.0",
            "echeck status",
            "mcu script ADC_TEMP !comm_error",
            "echeck periodic 2.0",
            "echeck status",
            "mcu script ADC_TEMP 1900",
            "echeck qtemp",
            "echeck status",
            "mcu script ADC_TEMP 2038",
            "echeck qtemp",
            "echeck status",
            "mcu script ADC_TEMP 8191",
            "echeck qtemp",
            "echeck status",
            "mcu script DRV_STATUS 0x04000010",
            "echeck periodic 3.0",
            "echeck status",
            "echeck stop",
            "echeck status",
        ],
    ));
    v.push(Scn::new(
        "echeck_2660",
        c2660_profile(),
        vec![
            "new echeck",
            "echeck status",
            "echeck start",
            "echeck status",
            "mcu script READRSP@RDSEL2 0x00000002",
            "echeck query drv",
            "mcu script READRSP@RDSEL2 0x00000004",
            "echeck query drv",
            "mcu script READRSP@RDSEL2 0",
            "echeck query drv",
            "fh_set cs 10",
            "echeck query drv",
            "fh_set cs 2",
            "echeck query drv",
            "mcu script READRSP@RDSEL2 0x00000400",
            "echeck query drv",
            "echeck status",
            "echeck periodic 5.0",
        ],
    ));

    // TMCStallguardDump -----------------------------------------------------
    v.push(Scn::new(
        "record_spi",
        spi_profile(),
        vec![
            "new record",
            "record dump",
            "mcu script DRV_STATUS raw:0x4123:1.5:8",
            "record start",
            "record query 5.0",
            "record query 6.0",
            "record dump",
            "record dump",
            "mcu script DRV_STATUS raw:0x4000:2.0:0",
            "record query 7.0",
            "record dump",
            "mcu script DRV_STATUS !boom",
            "record query 8.0",
            "record dump",
            "record start",
            "record dump",
            "record stop",
            "record dump",
            "record stop",
        ],
    ));
    v.push(Scn::new(
        "record_spi_no_spi_status",
        spi_profile(),
        vec![
            "new record",
            "mcu script DRV_STATUS raw:0x4123:1.5",
            "record start",
            "record query 5.0",
            "record query 5.5",
            "record dump",
            "record stop",
        ],
    ));
    v.push(Scn::new(
        "record_uart_sg_result",
        uart_profile(),
        vec![
            "new record",
            "mcu script DRV_STATUS 5",
            "record start",
            "mcu script SG_RESULT raw:77:2.5 raw:300:3.5",
            "record query 4.0",
            "record query 4.005",
            "record dump",
            "mcu script SG_RESULT !uart_gone",
            "record query 4.01",
            "record dump",
            "record stop",
        ],
    ));
    v.push(Scn::new(
        "record_sg4",
        sg4_profile(),
        vec![
            "new record",
            "mcu script DRV_STATUS raw:0x44105:1.0:3",
            "record start",
            "mcu script SG4_RESULT raw:999:2.0",
            "record query 5.0",
            "mcu script DRV_STATUS raw:0x10105:1.25:3",
            "record query 6.0",
            "record dump",
            "mcu script DRV_STATUS raw:0x44105:1.0:3",
            "mcu script SG4_RESULT !sg4_fail",
            "record query 7.0",
            "record dump",
            "record stop",
        ],
    ));
    v.push(Scn::new(
        "record_unsupported_2660",
        c2660_profile(),
        vec!["new record", "new echeck"],
    ));
    v.push(Scn::new(
        "record_start_error",
        spi_profile(),
        vec![
            "new record",
            "mcu script DRV_STATUS !cannot_start",
            "record start",
            "record dump",
            "record stop",
        ],
    ));

    // TMCCommandHelper ------------------------------------------------------
    v.push(Scn::new(
        "cmd_commands_spi",
        spi_profile(),
        vec![
            "new cmd",
            "fh_regs",
            "gcode INIT_TMC STEPPER=stepper_x",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=toff VALUE=4",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=TOFF VALUE=5",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=nope VALUE=5",
            "gcode SET_TMC_FIELD STEPPER=stepper_x VALUE=5",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=toff",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=toff VALUE=1 VELOCITY=5",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=tcoolthrs VELOCITY=50",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=toff VALUE=abc",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=tcoolthrs VELOCITY=-1",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=sgt VALUE=-7",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=intpol VALUE=0x1",
            "identify",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=tcoolthrs VELOCITY=50",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=thigh VELOCITY=0",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=thigh VELOCITY=0.0001",
            "mcu setfail GCONF broken",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=en_pwm_mode VALUE=1",
            "mcu clearfail GCONF",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x CURRENT=1.2",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x HOLDCURRENT=0.3",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.9 HOLDCURRENT=0.7",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x CURRENT=3",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x CURRENT=-1",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x HOLDCURRENT=0",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x HOLDCURRENT=2.5",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x CURRENT=x",
            "cur fail current_broken",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x CURRENT=1",
            "cur ok",
            "gcode SET_TMC_CURRENT STEPPER=stepper_y CURRENT=1",
            "gcode DUMP_TMC STEPPER=stepper_x",
            "gcode INIT_TMC STEPPER=stepper_x",
            "cmd init_regs",
            "cmd init_regs 3.5",
            "mcu setfail CHOPCONF init_failed",
            "cmd init_regs 4.5",
            "gcode INIT_TMC STEPPER=stepper_x",
            "cmd phases",
            "cmd offset",
            "cmd status",
        ],
    ));
    v.push(Scn::new(
        "cmd_commands_2660",
        c2660_profile(),
        vec![
            "new cmd",
            "fh_regs",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=cs VELOCITY=5",
            "gcode SET_TMC_FIELD STEPPER=stepper_x FIELD=cs VALUE=9",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x CURRENT=1.5",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x HOLDCURRENT=0.5",
            "gcode SET_TMC_CURRENT STEPPER=stepper_x CURRENT=1 HOLDCURRENT=0.4",
            "gcode INIT_TMC STEPPER=stepper_x",
            "cmd status",
        ],
    ));
    v.push(Scn::new(
        "cmd_new_heaters_monitor",
        sg4_profile(),
        vec!["new cmd", "cmd status", "cmd offset"],
    ));
    v.push(
        Scn::new(
            "cmd_new_unknown_stepper_section",
            spi_profile(),
            vec!["new cmd", "identify", "connect"],
        )
        .no_stepper_section(),
    );
    v.push(
        Scn::new(
            "cmd_identify_unknown_stepper",
            spi_profile(),
            vec![
                "new cmd",
                "identify",
                "cmd offset",
                "connect",
                "gcode INIT_TMC STEPPER=stepper_y",
                "gcode INIT_TMC STEPPER=stepper_z",
            ],
        )
        .named("tmc2130 stepper_y")
        .extra(&[("stepper_y", &[("rotation_distance", "40"), ("microsteps", "16")])]),
    );
    v.push(Scn::new(
        "cmd_connect_before_identify",
        spi_profile(),
        vec!["new cmd", "connect", "identify", "connect"],
    ));
    v.push(Scn::new(
        "cmd_phase_tracking_spi",
        spi_profile(),
        vec![
            "new cmd",
            "identify",
            "mcu script MSCNT 100",
            "stepper X pos 7",
            "ev sync X",
            "cmd offset",
            "cmd status",
            "ev sync X",
            "stepper X pos 9",
            "ev sync X",
            "cmd offset",
            "stepper X dir 1",
            "ev sync X",
            "ev dir X",
            "cmd offset",
            "ev sync Y",
            "mcu script MSCNT 0",
            "stepper X pos 0",
            "ev dir X",
            "mcu script MSCNT 1023",
            "ev sync X",
            "mcu script MSCNT 511",
            "stepper X dir 0",
            "stepper X pos 5000",
            "ev sync X",
            "fh_set mres 0",
            "cmd phases",
            "ev sync X",
            "fh_set mres 8",
            "cmd phases",
            "ev sync X",
            "cmd offset",
            "fh_set mres 9",
            "cmd phases",
            "ev sync X",
            "cmd offset",
            "cmd status",
        ],
    ));
    v.push(Scn::new(
        "cmd_phase_errors",
        spi_profile(),
        vec![
            "new cmd",
            "identify",
            "mcu script MSCNT 100",
            "ev sync X",
            "cmd offset",
            "mcu script MSCNT !oops",
            "ev sync X",
            "cmd offset",
            "mcu script MSCNT 100",
            "ev sync X",
            "motor on 1.0",
            "mcu script MSCNT !oops",
            "ev sync X",
            "cmd offset",
            "ev dir X",
            "mcu script MSCNT 12",
            "ev sync X",
            "cmd offset",
        ],
    ));
    v.push(Scn::new(
        "cmd_phase_2660",
        c2660_profile(),
        vec![
            "new cmd",
            "identify",
            "mcu script READRSP@RDSEL0 0xffc00",
            "stepper X pos 3",
            "ev sync X",
            "cmd offset",
            "cmd phases",
            "cmd status",
        ],
    ));
    v.push(Scn::new(
        "cmd_phase_before_identify",
        spi_profile(),
        vec!["new cmd", "mcu script MSCNT 100", "ev sync X", "cmd offset", "cmd status"],
    ));
    v.push(Scn::new(
        "cmd_virtual_enable",
        spi_profile(),
        vec![
            "new cmd",
            "identify",
            "stepper X edge 1",
            "fh_set toff 5",
            "connect",
            "fh_regs",
            "fire 4.0",
            "runcb",
            "fh_regs",
            "cmd status",
            "mcu script MSCNT 321",
            "mcu script DRV_STATUS 0",
            "motor off 6.0",
            "runcb",
            "fh_regs",
            "cmd status",
            "fire 8.0",
            "runcb",
            "cmd status",
            "mcu script DRV_STATUS 0x02000000",
            "motor off 9.0",
            "runcb",
            "fire 10.0",
            "runcb",
            "cmd status",
        ],
    ));
    v.push(Scn::new(
        "cmd_connect_init_failure",
        spi_profile(),
        vec![
            "new cmd",
            "identify",
            "mcu setfail CHOPCONF connect_failed",
            "connect",
            "mcu clearfail CHOPCONF",
            "fire 2.0",
            "runcb",
            "mcu setfail CHOPCONF enable_failed",
            "motor off 3.0",
            "runcb",
            "fire 4.0",
            "runcb",
            "cmd status",
        ],
    ));
    v.push(
        Scn::new(
            "cmd_dedicated_enable",
            spi_profile(),
            vec![
                "new cmd",
                "identify",
                "connect",
                "fh_regs",
                "mcu script MSCNT 640",
                "fire 3.0",
                "runcb",
                "cmd status",
                "motor off 5.0",
                "runcb",
                "cmd status",
                "fire 6.0",
                "runcb",
                "cmd status",
            ],
        )
        .stepper(&[("enable_pin", "PA1")]),
    );
    v.push(Scn::new(
        "cmd_enable_reset_uart",
        uart_profile(),
        vec![
            "new cmd",
            "identify",
            "connect",
            "mcu script MSCNT 100",
            "ev sync X",
            "cmd offset",
            "mcu script GSTAT 1 0",
            "fire 2.0",
            "runcb",
            "cmd offset",
            "cmd status",
            "motor off 4.0",
            "runcb",
            "mcu script GSTAT 0",
            "fire 5.0",
            "runcb",
            "cmd offset",
            "cmd status",
        ],
    ));
    v.push(Scn::new(
        "cmd_enable_phase_error",
        spi_profile(),
        vec![
            "new cmd",
            "identify",
            "connect",
            "mcu script MSCNT !phase_failed",
            "fire 2.0",
            "runcb",
            "cmd offset",
            "cmd status",
        ],
    ));
    v.push(Scn::new(
        "cmd_enable_phase_present",
        spi_profile(),
        vec![
            "new cmd",
            "identify",
            "connect",
            "mcu script MSCNT 50",
            "ev sync X",
            "mcu script MSCNT 70",
            "fire 2.0",
            "runcb",
            "cmd offset",
            "motor off 3.0",
            "runcb",
            "fire 4.0",
            "runcb",
            "cmd offset",
        ],
    ));
    v.push(Scn::new(
        "cmd_dump_tmc",
        spi_profile(),
        vec![
            "new cmd",
            "gcode DUMP_TMC STEPPER=stepper_x",
            "cmd dump_setup",
            "fh_set toff 3",
            "fh_set mscnt 1",
            "mcu script GSTAT 0",
            "mcu script DRV_STATUS 0x0c1f0123",
            "mcu script MSCNT 0x155",
            "gcode DUMP_TMC STEPPER=stepper_x",
            "gcode DUMP_TMC STEPPER=stepper_x REGISTER=gstat",
            "gcode DUMP_TMC STEPPER=stepper_x REGISTER=chopconf",
            "gcode DUMP_TMC STEPPER=stepper_x REGISTER=MSCNT",
            "gcode DUMP_TMC STEPPER=stepper_x REGISTER=nope",
            "mcu script DRV_STATUS !read_failed",
            "gcode DUMP_TMC STEPPER=stepper_x REGISTER=DRV_STATUS",
            "gcode DUMP_TMC STEPPER=stepper_x",
        ],
    ));
    v.push(Scn::new(
        "cmd_dump_tmc_translate",
        uart_profile(),
        vec![
            "new cmd",
            "cmd dump_setup translate",
            "fh_set toff 2",
            "mcu script GSTAT 1",
            "mcu script IFCNT 7",
            "mcu script DRV_STATUS 0x40000001",
            "mcu script MSCNT 12",
            "mcu script SG_RESULT 99",
            "gcode DUMP_TMC STEPPER=stepper_x",
            "gcode DUMP_TMC STEPPER=stepper_x REGISTER=gstat",
            "gcode DUMP_TMC STEPPER=stepper_x REGISTER=ifcnt",
        ],
    ));
    v.push(Scn::new(
        "cmd_dump_tmc_2660",
        c2660_profile(),
        vec![
            "new cmd",
            "cmd dump_setup translate",
            "mcu script READRSP@RDSEL0 0x12345",
            "mcu script READRSP@RDSEL1 0x1",
            "mcu script READRSP@RDSEL2 0x405",
            "gcode DUMP_TMC STEPPER=stepper_x",
            "gcode DUMP_TMC STEPPER=stepper_x REGISTER=drvctrl",
        ],
    ));

    // TMCVirtualPinHelper -----------------------------------------------------
    v.push(
        Scn::new(
            "vpin_diag0",
            spi_profile(),
            vec![
                "new vpin",
                "vpin begin yes",
                "vpin setup digital_out virtual_endstop 0 0",
                "vpin setup endstop other_pin 0 0",
                "vpin setup endstop virtual_endstop 1 0",
                "vpin setup endstop virtual_endstop 0 1",
                "vpin setup endstop virtual_endstop 0 -1",
                "vpin setup endstop virtual_endstop 0 0",
                "vpin begin no",
                "fh_regs",
                "vpin begin yes",
                "fh_regs",
                "vpin end no",
                "vpin end yes",
                "fh_regs",
                "vpin begin yes",
                "fh_set en_pwm_mode 1",
                "fh_set tcoolthrs 77",
                "vpin end yes",
                "fh_regs",
            ],
        )
        .driver(&[("diag0_pin", "^PA1"), ("diag1_pin", "PB2"), ("diag_pin", "PC3")]),
    );
    v.push(
        Scn::new(
            "vpin_diag1",
            spi_profile(),
            vec![
                "new vpin",
                "vpin setup endstop virtual_endstop 0 0",
                "vpin begin yes",
                "vpin end yes",
                "fh_regs",
            ],
        )
        .driver(&[("diag1_pin", "^PB2"), ("diag_pin", "PC3")]),
    );
    v.push(Scn::new(
        "vpin_no_diag_pin",
        spi_profile(),
        vec![
            "new vpin",
            "vpin setup endstop virtual_endstop 0 0",
            "vpin begin yes",
        ],
    ));
    v.push(
        Scn::new(
            "vpin_sg4",
            sg4_profile(),
            vec![
                "new vpin",
                "vpin setup endstop virtual_endstop 0 0",
                "vpin begin yes",
                "vpin end yes",
                "fh_set sg4_thrs 5",
                "vpin begin yes",
                "fh_regs",
                "vpin end yes",
                "fh_regs",
            ],
        )
        .driver(&[("diag0_pin", "PA1")]),
    );
    v.push(
        Scn::new(
            "vpin_uart_diag_pin",
            uart_profile(),
            vec![
                "new vpin",
                "vpin setup endstop virtual_endstop 0 0",
                "vpin begin yes",
                "fh_regs",
                "vpin end yes",
                "fh_regs",
                "fh_set tcoolthrs 9",
                "vpin begin yes",
                "vpin end yes",
                "mcu setfail GCONF broken",
                "vpin begin yes",
                "mcu clearfail GCONF",
                "vpin end yes",
                "fh_regs",
            ],
        )
        .driver(&[("diag_pin", "^PC3"), ("diag0_pin", "PX0")]),
    );
    v.push(
        Scn::new(
            "vpin_duplicate_registration",
            spi_profile(),
            vec![
                "new vpin",
                "vpin setup endstop virtual_endstop 0 0",
                "vpin setup endstop virtual_endstop 0 0",
                "vpin begin yes",
                "vpin end yes",
            ],
        )
        .driver(&[("diag0_pin", "PA1")]),
    );
    v
}

impl Scn {
    fn no_stepper_section(mut self) -> Scn {
        self.stepper_section = false;
        self
    }
    fn named(mut self, name: &str) -> Scn {
        self.driver_name = name.to_string();
        self
    }
    fn extra(mut self, sections: &[(&'static str, &[(&'static str, &'static str)])]) -> Scn {
        self.extra_sections = sections.iter().map(|(n, o)| (*n, o.to_vec())).collect();
        self
    }
}

// ---------------------------------------------------------------------------
// Rust world and step interpreter
// ---------------------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    reactor: Reactor,
    gcode: Arc<FakeGCode>,
    cfg: Config,
    fields: Arc<FieldHelper>,
    mcu_tmc: Arc<FakeMcuTmc>,
    cur: Arc<FakeCur>,
    stepper_x: Arc<FakeStepper>,
    stepper_y: Arc<FakeStepper>,
    se: Arc<PrinterStepperEnable>,
    endstop: Arc<FakeEndstop>,
    reads: Vec<String>,
    echeck: Option<Arc<TMCErrorCheck>>,
    record: Option<Arc<TMCStallguardDump>>,
    cmd: Option<Arc<TMCCommandHelper>>,
    vpin: Option<Arc<TMCVirtualPinHelper>>,
}

fn build_world(scn: &Scn) -> World {
    TRACE.borrow_mut().clear();
    LOGS.borrow_mut().clear();
    let reactor = Reactor::new(false);
    *WORLD_REACTOR.borrow_mut() = Some(reactor.clone());
    let printer = FakePrinter::new(reactor.clone());
    let printer_api: Printer = printer.clone();
    let data = Arc::new(CfgData { sections: config_sections(scn) });
    let root: Config = Arc::new(FakeConfig {
        printer: printer_api.clone(),
        data: data.clone(),
        section: "stepper_x".to_string(),
    });
    let gcode = Arc::new(FakeGCode::default());
    let fake_mcu: Mcu = Arc::new(FakeMcu { count: GCell::new(0) });
    let endstop = Arc::new(FakeEndstop);
    let stepper_x = FakeStepper::new("stepper_x");
    let stepper_y = FakeStepper::new("stepper_y");
    let objects: Vec<(&str, ObjectRef)> = vec![
        ("gcode", gcode.clone() as ObjectRef),
        ("toolhead", Arc::new(FakeToolHead) as ObjectRef),
        (
            "pins",
            Arc::new(FakePins {
                mcu: fake_mcu.clone(),
                active: GCell::new(IndexMap::new()),
                endstop: endstop.clone() as Endstop,
            }) as ObjectRef,
        ),
        ("heaters", Arc::new(FakeHeaters) as ObjectRef),
        ("webhooks", Arc::new(FakeWebhooks) as ObjectRef),
        (
            "force_move",
            Arc::new(FakeForceMove {
                steppers: vec![
                    ("stepper_x".to_string(), stepper_x.clone() as Stepper),
                    ("stepper_z".to_string(), stepper_y.clone() as Stepper),
                ],
            }) as ObjectRef,
        ),
    ];
    for (name, obj) in objects {
        printer.add_object(name, obj).unwrap();
    }
    let se = PrinterStepperEnable::new(&root).unwrap();
    printer.add_object("stepper_enable", se.clone() as ObjectRef).unwrap();
    let stepper_cfg = root.getsection("stepper_x");
    se.register_stepper(&stepper_cfg, &(stepper_x.clone() as Stepper)).unwrap();
    let cfg: Config = Arc::new(FakeConfig {
        printer: printer_api,
        data,
        section: scn.driver_name.clone(),
    });
    let profile = &scn.profile;
    let fields = FieldHelper::new(
        field_map(&profile.fields),
        profile.signed.iter().map(|s| s.to_string()).collect(),
        formatters(),
        None,
    );
    let mcu_tmc = Arc::new(FakeMcuTmc {
        fields: fields.clone(),
        regs: profile.regs.iter().map(|(n, v)| (n.to_string(), *v)).collect(),
        freq: profile.freq,
        script: GCell::new(IndexMap::new()),
        setfail: GCell::new(IndexMap::new()),
        mcu: fake_mcu,
    });
    let cur = Arc::new(FakeCur {
        hold_none: profile.hold_none,
        cur: GCell::new(if profile.hold_none {
            [Some(0.8), None, None, Some(2.0)]
        } else {
            [Some(0.8), Some(0.5), Some(0.5), Some(2.0)]
        }),
        fail: GCell::new(None),
    });
    TRACE.borrow_mut().clear();
    LOGS.borrow_mut().clear();
    World {
        printer,
        reactor,
        gcode,
        cfg,
        fields,
        mcu_tmc,
        cur,
        stepper_x,
        stepper_y,
        se,
        endstop,
        reads: profile.reads.iter().map(|s| s.to_string()).collect(),
        echeck: None,
        record: None,
        cmd: None,
        vpin: None,
    }
}

fn run_reactor_callbacks(w: &World) {
    let r = w.reactor.clone();
    let end_at = w.reactor.monotonic() + 0.02;
    w.reactor.register_timer(
        move |_| {
            r.end();
            NEVER
        },
        end_at,
    );
    w.reactor.run();
}

fn run_step(w: &mut World, line: &str) {
    tr(format!("> {}", line));
    let a: Vec<&str> = line.split_whitespace().collect();
    if let Err(e) = step(w, &a) {
        tr(format!("EXC {}: {}", e.class_name(), e.message()));
    }
}

fn cur_run_stepper(w: &World, which: &str) -> Arc<FakeStepper> {
    if which == "X" {
        w.stepper_x.clone()
    } else {
        w.stepper_y.clone()
    }
}

fn step(w: &mut World, a: &[&str]) -> KResult<()> {
    match a[0] {
        "fh_get" => {
            let v = w.fields.get_field(a[1], popt(a, 2), popts(a, 3))?;
            tr(format!("ret {}", v));
        }
        "fh_set" => {
            let v = w.fields.set_field(a[1], pint(a[2]), popt(a, 3), popts(a, 4))?;
            tr(format!("ret {}", v));
        }
        "fh_cfg" => {
            let d = match a[2] {
                "T" => ConfigFieldDefault::Bool(true),
                "F" => ConfigFieldDefault::Bool(false),
                t => ConfigFieldDefault::Int(pint(t)),
            };
            let v = w.fields.set_config_field(&w.cfg, a[1], d)?;
            tr(format!("ret {}", v));
        }
        "fh_pretty" => {
            tr(format!("ret {}", str_repr(&w.fields.pretty_format(a[1], pint(a[2])))));
        }
        "fh_regfields" => {
            let d: IndexMap<PyValue, PyValue> = w
                .fields
                .get_reg_fields(a[1], pint(a[2]))
                .into_iter()
                .map(|(k, v)| (PyValue::Str(k), PyValue::Int(pycompat::PyInt::from_i64(v))))
                .collect();
            tr(format!("ret {}", PyValue::Dict(d).repr()));
        }
        "fh_lookup" => match w.fields.lookup_register(a[1]) {
            Some(r) => tr(format!("ret {}", str_repr(&r))),
            None => tr("ret None"),
        },
        "fh_regs" => {
            let d: IndexMap<PyValue, PyValue> = w
                .fields
                .registers()
                .into_iter()
                .map(|(k, v)| (PyValue::Str(k), PyValue::Int(pycompat::PyInt::from_i64(v))))
                .collect();
            tr(format!("ret {}", PyValue::Dict(d).repr()));
        }
        "mcu" => match a[1] {
            "script" => {
                let items: Vec<Item> = a[3..].iter().map(|t| parse_item(t)).collect();
                w.mcu_tmc.script.borrow_mut().insert(a[2].to_string(), items);
            }
            "setfail" => {
                w.mcu_tmc.setfail.borrow_mut().insert(a[2].to_string(), a[3..].join(" "));
            }
            "clearfail" => {
                w.mcu_tmc.setfail.borrow_mut().shift_remove(a[2]);
            }
            other => panic!("unknown mcu step {other}"),
        },
        "cur" => match a[1] {
            "set" => w.cur.cur.set([pfloat(a[2]), pfloat(a[3]), pfloat(a[4]), pfloat(a[5])]),
            "fail" => *w.cur.fail.borrow_mut() = Some(a[2..].join(" ")),
            "ok" => *w.cur.fail.borrow_mut() = None,
            other => panic!("unknown cur step {other}"),
        },
        "stepper" => {
            let st = cur_run_stepper(w, a[1]);
            match a[2] {
                "pos" => st.mcu_pos.set(pint(a[3])),
                "dir" => st.dir_inv.set(pint(a[3]) != 0),
                "edge" => st.both_edge.set(pint(a[3]) != 0),
                "dist" => st.step_dist.set(a[3].parse().unwrap()),
                other => panic!("unknown stepper step {other}"),
            }
        }
        "new" => {
            let mcu_tmc: Arc<dyn McuTmcApi> = w.mcu_tmc.clone();
            match a[1] {
                "echeck" => w.echeck = Some(TMCErrorCheck::new(&w.cfg, mcu_tmc)?),
                "record" => w.record = Some(TMCStallguardDump::new(&w.cfg, mcu_tmc)?),
                "cmd" => {
                    let cmd = TMCCommandHelper::new(&w.cfg, mcu_tmc, w.cur.clone())?;
                    w.echeck = Some(cmd.echeck_helper.clone());
                    w.record = Some(cmd.record_helper.clone());
                    w.cmd = Some(cmd);
                }
                "vpin" => w.vpin = Some(TMCVirtualPinHelper::new(&w.cfg, mcu_tmc)?),
                other => panic!("unknown new step {other}"),
            }
            tr("new ok");
        }
        "echeck" => {
            let ec = w.echeck.clone().expect("echeck exists");
            match a[1] {
                "query" => {
                    let info = if a[2] == "drv" {
                        Some(&ec.drv_status_reg_info)
                    } else {
                        ec.gstat_reg_info.as_ref()
                    }
                    .expect("register info");
                    let r = ec._query_register(info, a.len() > 3);
                    let last = info.borrow().last_value;
                    match r {
                        Ok(v) => {
                            tr(format!("ret {}", v));
                            tr(format!("last {}", last));
                        }
                        Err(e) => {
                            tr(format!("last {}", last));
                            return Err(e);
                        }
                    }
                }
                "start" => {
                    let r = ec.start_checks()?;
                    tr(format!("ret {}", py_bool(r)));
                }
                "stop" => ec.stop_checks(),
                "periodic" => {
                    let r = ec._do_periodic_check(a[2].parse().unwrap());
                    tr(format!("ret {}", float_repr(r)));
                }
                "status" => tr(format!("ret {}", ec.get_status(None).repr())),
                "qtemp" => ec._query_temperature()?,
                other => panic!("unknown echeck step {other}"),
            }
        }
        "record" => {
            let rec = w.record.clone().expect("record exists");
            match a[1] {
                "start" => rec._start()?,
                "stop" => rec._stop()?,
                "query" => {
                    let r = rec._query_tmc(a[2].parse().unwrap());
                    tr(format!("ret {}", float_repr(r)));
                }
                "dump" => tr(format!("ret {}", rec._dump(0.)?.repr())),
                other => panic!("unknown record step {other}"),
            }
        }
        "cmd" => {
            let c = w.cmd.clone().expect("cmd exists");
            match a[1] {
                "init_regs" => {
                    let pt = a.get(2).map(|t| t.parse().unwrap());
                    c._init_registers(pt)?;
                }
                "phases" => tr(format!("ret {}", c._get_phases()?)),
                "offset" => {
                    let (off, phases) = c.get_phase_offset();
                    let off = match off {
                        Some(o) => o.to_string(),
                        None => "None".to_string(),
                    };
                    tr(format!("ret ({}, {})", off, phases));
                }
                "status" => tr(format!("ret {}", c.get_status(None).repr())),
                "dump_setup" => {
                    let t = if a.len() > 2 { Some(translate()) } else { None };
                    c.setup_register_dump(w.reads.clone(), t)?;
                }
                other => panic!("unknown cmd step {other}"),
            }
        }
        "gcode" => {
            let cmd = a[1];
            let mut params: IndexMap<String, String> = IndexMap::new();
            for t in &a[2..] {
                let (k, v) = t.split_once('=').unwrap();
                params.insert(k.to_string(), v.to_string());
            }
            let key = (cmd.to_string(), params.get("STEPPER").cloned().unwrap_or("None".to_string()));
            let handler = w.gcode.mux.borrow().get(&key).cloned();
            match handler {
                None => tr("NOCMD"),
                Some(h) => {
                    let gcmd: GCmd =
                        Arc::new(FakeGCmd { commandline: a[1..].join(" "), params });
                    let mut guard = h.lock();
                    (*guard)(&gcmd)?;
                }
            }
        }
        "ev" => {
            let st = cur_run_stepper(w, a[2]);
            let ev = if a[1] == "sync" { EV_STEPPER_SYNC_MCU_POSITION } else { EV_STEPPER_SET_DIR_INVERTED };
            w.printer.send_event(ev, &[EventArg::Object(st as ObjectRef)])?;
        }
        "identify" => {
            w.printer.send_event(EV_KLIPPY_MCU_IDENTIFY, &[])?;
        }
        "connect" => {
            w.printer.send_event(EV_KLIPPY_CONNECT, &[])?;
        }
        "fire" => w.stepper_x.fire_active(a[1].parse().unwrap()),
        "motor" => {
            let el = PrinterStepperEnableApi::lookup_enable(&*w.se, "stepper_x")?;
            let pt: f64 = a[2].parse().unwrap();
            if a[1] == "on" {
                el.motor_enable(pt);
            } else {
                el.motor_disable(pt)?;
            }
        }
        "runcb" => run_reactor_callbacks(w),
        "vpin" => {
            let v = w.vpin.clone().expect("vpin exists");
            match a[1] {
                "setup" => {
                    let params = PinParams {
                        chip: w.mcu_tmc.mcu.clone(),
                        chip_name: "tmc_x".to_string(),
                        pin: a[3].to_string(),
                        invert: pint(a[4]),
                        pullup: pint(a[5]),
                        share_type: None,
                    };
                    let r = klippy_api::mcu::ChipApi::setup_pin(&*v, a[2], params)?;
                    match r {
                        Pin::Endstop(_) => tr("ret Endstop"),
                        _ => panic!("virtual pin returned a non endstop pin"),
                    }
                }
                "begin" | "end" => {
                    let endstops: Vec<Endstop> =
                        if a[2] == "yes" { vec![w.endstop.clone() as Endstop] } else { vec![] };
                    let hm: Arc<dyn HomingMoveApi> = Arc::new(FakeHMove { endstops });
                    let ev = if a[1] == "begin" { EV_HOMING_MOVE_BEGIN } else { EV_HOMING_MOVE_END };
                    w.printer.send_event(ev, &[EventArg::handle(hm)])?;
                }
                other => panic!("unknown vpin step {other}"),
            }
        }
        "helper" => {
            let m: &dyn McuTmcApi = &*w.mcu_tmc;
            match a[1] {
                "wave" => tmc_wave_table_helper(&w.cfg, m)?,
                "micro" => tmc_microstep_helper(&w.cfg, m)?,
                "stealth" => tmc_stealthchop_helper(&w.cfg, m)?,
                "vcool" => tmc_vcoolthrs_helper(&w.cfg, m)?,
                "vhigh" => tmc_vhigh_helper(&w.cfg, m)?,
                other => panic!("unknown helper {other}"),
            }
        }
        "tstep" => {
            let velocity: f64 = a[1].parse().unwrap();
            let stepper: Option<Stepper> =
                if a[2] == "st" { Some(w.stepper_x.clone() as Stepper) } else { None };
            let cfg: Option<&Config> = if a[2] == "cfg" { Some(&w.cfg) } else { None };
            let r = tmc_tstep_helper(&*w.mcu_tmc, velocity, stepper.as_ref(), cfg)?;
            tr(format!("ret {}", r));
        }
        other => panic!("unknown step {other}"),
    }
    Ok(())
}

fn rust_trace(w: &World) -> String {
    let _ = &w.record;
    let mut lines = TRACE.cloned();
    lines.push("--- logs ---".to_string());
    lines.extend(LOGS.cloned());
    lines.join("\n")
}

fn assert_traces(name: &str, py: &str, rs: &str) {
    if py == rs {
        return;
    }
    let pl: Vec<&str> = py.split('\n').collect();
    let rl: Vec<&str> = rs.split('\n').collect();
    let mut i = 0;
    while i < pl.len() && i < rl.len() && pl[i] == rl[i] {
        i += 1;
    }
    let ctx_start = i.saturating_sub(6);
    let mut msg = format!("scenario {name}: traces differ at line {i}\n--- common context ---\n");
    for l in &pl[ctx_start..i] {
        msg.push_str(l);
        msg.push('\n');
    }
    msg.push_str(&format!("--- python (oracle) ---\n{}\n", pl.get(i).copied().unwrap_or("<end>")));
    msg.push_str(&format!("--- rust ---\n{}\n", rl.get(i).copied().unwrap_or("<end>")));
    panic!("{msg}");
}

fn py_literal_steps(steps: &[&str]) -> String {
    let v: Vec<String> = steps.iter().map(|s| format!("{:?}", s)).collect();
    format!("[{}]", v.join(", "))
}

fn run_scenario(py: &mut pyoracle::Oracle, scn: &Scn) {
    let sections = config_sections(scn);
    let text = config_text(&sections);
    let p = &scn.profile;
    let regs = format!(
        "{{{}}}",
        p.regs.iter().map(|(n, v)| format!("\"{}\": {}", n, v)).collect::<Vec<_>>().join(", ")
    );
    let setup = format!(
        "setup_world({fields}, {signed}, {regs}, {freq}, {reads}, {hold}, {text:?}, {driver:?})",
        fields = py_fields(&p.fields),
        signed = py_str_list(&p.signed),
        freq = py_float_opt(p.freq),
        reads = py_str_list(&p.reads),
        hold = py_bool(p.hold_none),
        text = text,
        driver = scn.driver_name,
    );
    py.exec(&setup).unwrap_or_else(|e| panic!("scenario {}: python setup failed: {:?}", scn.name, e));
    py.exec(&format!("for _l in {}:\n    run_step(_l)\n", py_literal_steps(&scn.steps)))
        .unwrap_or_else(|e| panic!("scenario {}: python steps failed: {:?}", scn.name, e));
    let expected = match py.eval("final_trace()") {
        Ok(Value::Str(s)) => s,
        other => panic!("scenario {}: unexpected oracle answer {:?}", scn.name, other),
    };
    let mut w = build_world(scn);
    for line in &scn.steps {
        run_step(&mut w, line);
    }
    let actual = rust_trace(&w);
    assert_traces(scn.name, &expected, &actual);
}

#[test]
fn tmc_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the tmc oracle comparison");
        return;
    };
    install_log_sink();
    py.exec(SETUP).unwrap_or_else(|e| panic!("python setup failed: {:?}", e));
    for scn in scenarios() {
        run_scenario(&mut py, &scn);
    }
}

#[test]
fn tmc_module_has_no_loader() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the tmc oracle comparison");
        return;
    };
    py.exec("import extras.tmc as tmc_mod").unwrap();
    let has = |py: &mut pyoracle::Oracle, name: &str| {
        matches!(
            py.eval(&format!("hasattr(tmc_mod, '{}')", name)),
            Ok(Value::Bool(true))
        )
    };
    let factory = module_factory();
    assert_eq!(has(&mut py, "load_config"), factory.load_config.is_some());
    assert_eq!(has(&mut py, "load_config_prefix"), factory.load_config_prefix.is_some());
    assert_eq!(has(&mut py, "add_early_printer_objects"), factory.add_early_printer_objects.is_some());
    assert_eq!(has(&mut py, "add_printer_objects"), factory.add_printer_objects.is_some());
}
