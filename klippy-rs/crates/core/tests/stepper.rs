//! Parity tests for `klippy_core::stepper` (port of `klippy/stepper.py`).
//!
//! Every expected value comes from the live reference through `pyoracle` (CPython 3.8.2 +
//! the frozen `klippy/`): the same `MCU_stepper` / `GenericPrinterRail` / `PrinterStepper` /
//! `parse_step_distance` call runs in Python and in the port, and the returned values (floats
//! by bits), raised errors (class and message), config reads with their defaults (the real
//! `configfile.ConfigWrapper` access tracking), MCU commands, pin lookups, events, flush
//! callbacks and queued syncemitter messages are compared.
//!
//! Python runs the real `klippy/configfile.py` over a `RawConfigParser`, and the frozen
//! `klippy/stepper.py` over fakes of the printer, MCU, pins, `motion_queuing`,
//! `query_endstops`, `configfile` and the chelper cffi library. The fake chelper
//! (`FakeLib`) stands in for the C code `stepper.py` calls: its `itersolve_*`,
//! `trapq` and `stepcompress` behaviour is checked against the real C library in
//! `fake_chelper_matches_c` (through `chelper_ref`). The port runs the real `chelper_motion`
//! kinematics and trapq behind `klippy_core::motion_handles`.
//!
//! Not observable through the contract, so not compared: the `stepcompress_fill` arguments
//! (the port owns its `Stepcompress`; the Python fill record is compared with the values the
//! Rust fake MCU handed out), `syncemitter_set_stepper_kinematics`, and the
//! `Internal error in stepcompress` paths (the compressor cannot fail without appended steps).

use std::ffi::c_void;
use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetFloatOpts, GetIntOpts, GetListsOpts, GetStrOpts,
    ListParser, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandQueryWrapper, CommandWrapper, EndstopApi,
    LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinsApi,
    QuerySendOpts, SendOpts,
};
use klippy_api::object::{interface, EventArg};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, QueryEndstopsApi, RailApi, RailOpts, Stepper, StepperFactoryApi,
    StepperKinematics, StepperRegistryApi, Trapq,
};
use klippy_api::toolhead::{FlushCallback, MotionQueuingApi, SyncEmitterHandle, TrapqAppend};
use klippy_core::motion_handles::{alloc_stepper_kinematics, kinematics_of, trapq_of, TrapqHandle};
use klippy_core::stepper::{
    module_factory, parse_step_distance, printer_stepper, GenericPrinterRail, McuStepper,
    StepperFactory,
};
use kos::sync::Mutex;
use pycompat::PyValue;
use pyoracle::Oracle;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python side.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, math, configparser, configfile

sentinel = object()

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self.events = []
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=sentinel):
        return self.lookup_object(section, default)
    def add_object(self, name, obj):
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        self.events.append((event, [p.get_name() for p in params]))
        return [cb(*params) for cb in self.handlers.get(event, [])]

class FakeCommand:
    def __init__(self, tag):
        self.tag = tag
    def get_command_tag(self):
        return self.tag

class FakeQueryCommand:
    def __init__(self, responses, log):
        self.responses = responses
        self.log = log
    def send(self, data, params=None):
        self.log.append(list(data))
        return self.responses.pop(0)

class FakeMcu:
    def __init__(self, printer, name, freq, fileoutput, constants):
        self.printer = printer
        self._name = name
        self.freq = freq
        self.fileoutput = fileoutput
        self.constants = dict(constants)
        self._oid = 10
        self.config_cmds = []
        self.config_callbacks = []
        self.query_log = []
        self.query_responses = []
        self.query_lookups = []
        self.tags = {}
        self._tag = 100
    def get_name(self):
        return self._name
    def get_printer(self):
        return self.printer
    def is_fileoutput(self):
        return self.fileoutput
    def create_oid(self):
        self._oid += 1
        return self._oid
    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append((cmd, is_init, on_restart))
    def seconds_to_clock(self, time):
        return int(time * self.freq)
    def get_constants(self):
        return dict(self.constants)
    def lookup_command(self, msg, cq=None):
        if msg not in self.tags:
            self._tag += 1
            self.tags[msg] = self._tag
        return FakeCommand(self.tags[msg])
    def lookup_query_command(self, msg, resp, oid=None, cq=None, is_async=False):
        self.query_lookups.append((msg, resp, oid))
        return FakeQueryCommand(self.query_responses, self.query_log)
    def print_time_to_clock(self, print_time):
        return int(print_time * self.freq)
    def estimated_print_time(self, eventtime):
        return eventtime

class FakeEndstop:
    def __init__(self, pin):
        self.pin = pin
        self.steppers = []
    def add_stepper(self, stepper):
        self.steppers.append(stepper)
    def get_steppers(self):
        return list(self.steppers)

class FakeProbeEndstop(FakeEndstop):
    def __init__(self, pin, position):
        FakeEndstop.__init__(self, pin)
        self.position = position
    def get_position_endstop(self):
        return self.position

class FakePins:
    def __init__(self, mcu, probe):
        self.mcu = mcu
        self.probe = probe
        self.log = []
    def parse_pin(self, desc, can_invert=False, can_pullup=False):
        params = self._parse(desc, can_invert, can_pullup)
        self.log.append(('parse_pin', desc, can_invert, can_pullup))
        return params
    def _parse(self, desc, can_invert, can_pullup):
        desc = desc.strip()
        pullup = invert = 0
        if can_pullup and (desc.startswith('^') or desc.startswith('~')):
            pullup = 1
            if desc.startswith('~'):
                pullup = -1
            desc = desc[1:].strip()
        if can_invert and desc.startswith('!'):
            invert = 1
            desc = desc[1:].strip()
        if ':' not in desc:
            chip_name, pin = 'mcu', desc
        else:
            chip_name, pin = [s.strip() for s in desc.split(':', 1)]
        return {'chip': self.mcu, 'chip_name': chip_name, 'pin': pin,
                'invert': invert, 'pullup': pullup}
    def lookup_pin(self, desc, can_invert=False, can_pullup=False, share_type=None):
        self.log.append(('lookup_pin', desc, can_invert, can_pullup, share_type))
        return self._parse(desc, can_invert, can_pullup)
    def setup_pin(self, pin_type, pin_desc):
        assert pin_type == 'endstop'
        self.log.append(('setup_pin', pin_type, pin_desc))
        if self.probe is not None:
            return FakeProbeEndstop(pin_desc, self.probe)
        return FakeEndstop(pin_desc)

MAX_NULL_MOVE = 1.0

class FakeTrapq:
    # Mirrors trapq_add_move() of trapq.c, including the null move that fills a time gap.
    def __init__(self):
        self.moves = []
    def add_move(self, print_time, move_t, axes_r):
        if self.moves:
            prev_pt, prev_mt = self.moves[-1][0], self.moves[-1][1]
        else:
            prev_pt, prev_mt = -1.0, 0.0
        if prev_pt + prev_mt < print_time:
            if prev_pt <= 0. and print_time > MAX_NULL_MOVE:
                null_pt = print_time - MAX_NULL_MOVE
            else:
                null_pt = prev_pt + prev_mt
            self.moves.append((null_pt, print_time - null_pt, (0., 0., 0.)))
        self.moves.append((print_time, move_t, axes_r))

class FakeMotionQueuing:
    def __init__(self):
        self.syncemitters = []
        self.flush_callbacks = []
        self.registered_flags = []
        self.unregistered = 0
    def allocate_syncemitter(self, mcu, name, alloc_stepcompress=True):
        se = {'name': name, 'sk': None, 'queued_msgs': [],
              'sc': {'history': [], 'last_position': 0, 'invert_sdir': 0, 'fill': None}}
        self.syncemitters.append(se)
        return se
    def register_flush_callback(self, callback, can_add_trapq=False):
        self.registered_flags.append(can_add_trapq)
        if can_add_trapq:
            self.flush_callbacks = [callback] + self.flush_callbacks
        else:
            self.flush_callbacks = self.flush_callbacks + [callback]
    def unregister_flush_callback(self, callback):
        if callback in self.flush_callbacks:
            fcbs = list(self.flush_callbacks)
            fcbs.remove(callback)
            self.flush_callbacks = fcbs
            self.unregistered += 1

class FakeQueryEndstops:
    def __init__(self):
        self.registered = []
    def register_endstop(self, endstop, name):
        self.registered.append((endstop, name))

class FakeConfigfile:
    def __init__(self):
        self.deprecated = []
    def deprecate_mcu_code(self, mcu, feature, msg=None):
        self.deprecated.append((mcu.get_name(), feature))

class FakeRegistrar:
    def __init__(self):
        self.registered = []
    def register_stepper(self, config, stepper):
        self.registered.append((config.get_name(), stepper.get_name()))

# --- chelper fake (checked against the C library in fake_chelper_matches_c) ---
class FakeMain:
    NULL = None
    def gc(self, obj, free):
        return obj
    def new(self, kind, n):
        assert kind == 'struct pull_history_steps[]'
        return [{'first_clock': 0, 'last_clock': 0, 'start_position': 0,
                 'interval': 0, 'step_count': 0, 'add': 0} for _ in range(n)]

def _sk(axes, calc):
    return {'commanded_pos': 0., 'last_flush_time': 0., 'tq': None, 'step_dist': 0.,
            'axes': axes, 'calc': calc}

class FakeLib:
    # stepcompress
    def syncemitter_get_stepcompress(self, se):
        return se['sc']
    def stepcompress_set_invert_sdir(self, sc, inv):
        sc['invert_sdir'] = int(bool(inv))
    def stepcompress_fill(self, sc, oid, maxerr, step_tag, dir_tag):
        sc['fill'] = (oid, maxerr, step_tag, dir_tag)
    def stepcompress_find_past_position(self, sc, clock):
        last_position = sc['last_position']
        for hs in sc['history']:
            if clock < hs['first_clock']:
                last_position = hs['start_position']
                continue
            if clock >= hs['last_clock']:
                return hs['start_position'] + hs['step_count']
            raise AssertionError("only position markers exist without step generation")
        return last_position
    def stepcompress_extract_old(self, sc, data, count, start_clock, end_clock):
        n = 0
        for hs in sc['history']:
            if start_clock >= hs['last_clock'] or n >= count:
                break
            if end_clock <= hs['first_clock']:
                continue
            data[n] = dict(hs)
            n += 1
        return n
    def stepcompress_reset(self, sc, last_step_clock):
        return 0
    def stepcompress_set_last_position(self, sc, clock, last_position):
        sc['last_position'] = last_position
        sc['history'].insert(0, {'first_clock': clock, 'last_clock': clock,
                                 'start_position': last_position, 'step_count': 0,
                                 'interval': 0, 'add': 0})
        return 0
    # syncemitter
    def syncemitter_set_stepper_kinematics(self, se, sk):
        se['sk'] = sk
    def syncemitter_queue_msg(self, se, req_clock, data, length):
        se['queued_msgs'].append((req_clock, tuple(list(data)[:length])))
    # itersolve
    def itersolve_calc_position_from_coord(self, sk, x, y, z):
        return sk['calc'](x, y, z)
    def itersolve_set_position(self, sk, x, y, z):
        sk['commanded_pos'] = sk['calc'](x, y, z)
    def itersolve_get_commanded_pos(self, sk):
        return sk['commanded_pos']
    def itersolve_set_trapq(self, sk, tq, step_dist):
        sk['tq'] = tq
        sk['step_dist'] = step_dist
    def itersolve_is_active_axis(self, sk, axis):
        axis = axis[0]
        if axis < 120 or axis > 122:
            return 0
        return 1 if chr(axis) in sk['axes'] else 0
    def itersolve_check_active(self, sk, flush_time):
        tq = sk['tq']
        if tq is None:
            return 0.
        moves = tq.moves
        i = 0
        while i < len(moves) and sk['last_flush_time'] >= moves[i][0] + moves[i][1]:
            i += 1
        while i < len(moves):
            pt, mt, ax = moves[i]
            if (('x' in sk['axes'] and ax[0] != 0.) or ('y' in sk['axes'] and ax[1] != 0.)
                    or ('z' in sk['axes'] and ax[2] != 0.)):
                return pt
            if flush_time <= pt + mt:
                return 0.
            i += 1
        return 0.
    # kinematics allocators (the linear family)
    def cartesian_stepper_alloc(self, axis):
        a = chr(axis[0])
        return _sk(a, lambda x, y, z: {'x': x, 'y': y, 'z': z}[a])
    def corexy_stepper_alloc(self, typ):
        if typ == b'+':
            return _sk('xy', lambda x, y, z: x + y)
        return _sk('xy', lambda x, y, z: x - y)
    def corexz_stepper_alloc(self, typ):
        if typ == b'+':
            return _sk('xz', lambda x, y, z: x + z)
        return _sk('xz', lambda x, y, z: x - z)
    def generic_cartesian_stepper_alloc(self, ax, ay, az):
        axes = ''.join(c for c, v in zip('xyz', (ax, ay, az)) if v != 0.)
        return _sk(axes, lambda x, y, z: ax * x + ay * y + az * z)
    def free(self, obj):
        pass

_fmain = FakeMain()
_flib = FakeLib()
_fake_chelper = types.ModuleType('chelper')
_fake_chelper.get_ffi = lambda: (_fmain, _flib)
sys.modules['chelper'] = _fake_chelper

import stepper

def build_world(ini, constants, fileoutput, probe):
    global printer, mcu, pins, mq, cfile, qes, fileconfig, access_tracking
    printer = FakePrinter()
    mcu = FakeMcu(printer, 'mcu', 12000000.0, fileoutput, constants)
    pins = FakePins(mcu, probe)
    mq = FakeMotionQueuing()
    cfile = FakeConfigfile()
    qes = FakeQueryEndstops()
    printer.add_object('pins', pins)
    printer.add_object('motion_queuing', mq)
    printer.add_object('configfile', cfile)
    printer.add_object('query_endstops', qes)
    for name in ('stepper_enable', 'force_move', 'motion_report'):
        printer.add_object(name, FakeRegistrar())
    fileconfig = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
    fileconfig.read_string(ini)
    access_tracking = {}

def cfg(section):
    return configfile.ConfigWrapper(printer, fileconfig, access_tracking, section)

def world_state():
    return {
        'tracking': [((k[0], k[1]), v) for k, v in access_tracking.items()],
        'pin_log': list(pins.log),
        'qes': [(e.pin, n) for e, n in qes.registered],
        'registrars': [(n, list(printer.objects[n].registered))
                       for n in ('stepper_enable', 'force_move', 'motion_report')
                       if n in printer.objects],
        'syncemitters': [se['name'] for se in mq.syncemitters],
        'oid': mcu._oid,
        'handlers': [(e, len(h)) for e, h in printer.handlers.items()],
        'callbacks': len(mcu.config_callbacks),
        'deprecated': list(cfile.deprecated),
        'events': list(printer.events),
        'config_cmds': list(mcu.config_cmds),
        'tags': list(mcu.tags.items()),
        'query_lookups': list(mcu.query_lookups),
        'query_log': list(mcu.query_log),
        'flush': (len(mq.flush_callbacks), list(mq.registered_flags), mq.unregistered),
        'queued': [list(se['queued_msgs']) for se in mq.syncemitters],
    }

def dump_steps(s, count, start, end):
    data, n = s.dump_steps(count, start, end)
    return (n, [(k['first_clock'], k['last_clock'], k['start_position'], k['interval'],
                 k['step_count'], k['add']) for k in data[:n]])

def extract(sc, count, start, end):
    data = _fmain.new('struct pull_history_steps[]', 10)
    n = _flib.stepcompress_extract_old(sc, data, count, start, end)
    return (n, [(k['first_clock'], k['last_clock'], k['start_position'], k['interval'],
                 k['step_count'], k['add']) for k in data[:n]])
"#;

const FREQ: f64 = 12_000_000.0;

/// One message queued on a syncemitter: `(req_clock, words)`.
type QueuedMsg = (u64, Vec<u32>);
/// A `build_config_matrix` row: step pin, pulse duration, MCU constants, requested both
/// edges, and whether the build must fail.
type BuildRow<'a> = (&'a str, Option<f64>, Vec<(&'a str, &'a str)>, bool, bool);
type Maker = Box<dyn Fn() -> *mut c_void>;
type Moves = Vec<(f64, f64, [f64; 3])>;
type CaseOptions<'a> = Vec<(&'a str, &'a str)>;
type ActiveScenario<'a> = (&'a str, Moves, Vec<(f64, f64)>);

fn oracle() -> Option<Oracle> {
    match Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("stepper fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping stepper oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary and \
                 run in the oracle image"
            );
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "stepper".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn same_error(label: &str, expected: &recording::Exception, actual: &KlippyError) {
    assert_eq!(expected.class_name, actual.class_name(), "{label}: error class");
    assert_eq!(expected.message, actual.message(), "{label}: error message");
}

/// Both sides must succeed or both must fail with the same class and message.
/// Returns whether they succeeded.
fn compare_unit(label: &str, expected: pyoracle::OracleResult, actual: &KResult<()>) -> bool {
    match (expected, actual) {
        (Ok(_), Ok(())) => true,
        (Err(e), Err(a)) => {
            same_error(label, &e, a);
            false
        }
        (Ok(_), Err(a)) => panic!("{label}: python succeeded, rust failed with {a}"),
        (Err(e), Ok(())) => {
            panic!("{label}: python raised {}: {}, rust succeeded", e.class_name, e.message)
        }
    }
}

fn pyf(x: f64) -> String {
    assert!(x.is_finite());
    format!("{x:?}")
}

fn pyb(b: bool) -> &'static str {
    if b { "True" } else { "False" }
}

fn py_opt_f(x: Option<f64>) -> String {
    x.map_or_else(|| "None".to_string(), pyf)
}

fn vf(x: f64) -> Value {
    Value::float(x)
}

fn vs(x: &str) -> Value {
    Value::str(x)
}

fn data_ptr<T: ?Sized>(a: &Arc<T>) -> *const () {
    Arc::as_ptr(a) as *const ()
}

fn same<T: ?Sized>(a: &Arc<T>, b: &Arc<T>) -> bool {
    data_ptr(a) == data_ptr(b)
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse().expect("small int")),
        PyValue::Float(f) => vf(*f),
        PyValue::Str(s) => vs(s),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_value).collect()),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        other => panic!("unexpected value {}", other.repr()),
    }
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type SharedHandler = Arc<Mutex<EventCallback>>;

struct FakePrinter {
    objects: Mutex<IndexMap<String, klippy_api::ObjectRef>>,
    handlers: Mutex<IndexMap<String, Vec<SharedHandler>>>,
    events: Mutex<Vec<(String, Vec<String>)>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(IndexMap::new()),
            events: Mutex::new(Vec::new()),
        }
    }

    fn add(&self, name: &str, obj: klippy_api::ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }

    fn fire(&self, event: &str) -> KResult<Vec<PyValue>> {
        self.send_event(event, &[])
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        ("Printer is ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: klippy_api::ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<klippy_api::ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::Config(format!("Unknown config object '{name}'")))
            }
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<klippy_api::ObjectRef>)> {
        self.objects.lock().iter().map(|(k, v)| (k.clone(), v.clone())).map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<klippy_api::ObjectRef>> {
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers
            .lock()
            .entry(event.to_string())
            .or_default()
            .push(Arc::new(Mutex::new(callback)));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let names = params
            .iter()
            .map(|p| match p {
                EventArg::Object(obj) => interface::<dyn McuStepperApi>(obj)
                    .map(|s| s.get_name(None))
                    .expect("event payload is a stepper"),
                _ => panic!("unexpected event payload"),
            })
            .collect();
        self.events.lock().push((event.to_string(), names));
        let handlers: Vec<SharedHandler> =
            self.handlers.lock().get(event).cloned().unwrap_or_default();
        let mut out = Vec::new();
        for handler in handlers {
            let mut cb = handler.lock();
            out.push((*cb)(params)?);
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

/// The config file shared by every section wrapper: `section -> option (lower case) -> value`
/// plus the `access_tracking` dict of `configfile.ConfigWrapper`.
struct ConfigData {
    printer: Printer,
    sections: IndexMap<String, IndexMap<String, String>>,
    tracking: Mutex<IndexMap<(String, String), Value>>,
}

/// `configfile.ConfigWrapper` semantics for the getters `stepper.py` uses.
struct FakeConfig {
    section: String,
    data: Arc<ConfigData>,
}

impl FakeConfig {
    fn raw(&self, option: &str) -> Option<String> {
        self.data.sections.get(&self.section)?.get(&option.to_lowercase()).cloned()
    }

    fn track(&self, option: &str, value: Value) {
        self.data
            .tracking
            .lock()
            .insert((self.section.to_lowercase(), option.to_lowercase()), value);
    }

    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.section
        ))
    }

    fn unparsable(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.section
        ))
    }

    fn bound_error(&self, option: &str, what: &str, bound: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must {what} {bound}",
            self.section
        ))
    }

    /// `ConfigWrapper._get_wrapper` for an option that is not in the file.
    fn absent<T: Clone>(
        &self,
        option: &str,
        default: Option<Option<T>>,
        note_valid: bool,
        to_value: impl Fn(&T) -> Value,
    ) -> KResult<Option<T>> {
        match default {
            None => Err(self.missing(option)),
            Some(default) => {
                if note_valid {
                    if let Some(d) = &default {
                        self.track(option, to_value(d));
                    }
                }
                Ok(default)
            }
        }
    }

    /// `lparser` of `ConfigWrapper.getlists`.
    fn lparser(
        &self,
        option: &str,
        value: &str,
        pos: usize,
        opts: &GetListsOpts,
    ) -> KResult<PyValue> {
        let parts: Vec<String> = if value.trim().is_empty() {
            Vec::new()
        } else {
            value.split(opts.seps[pos].as_str()).map(|p| p.trim().to_string()).collect()
        };
        if pos > 0 {
            let mut nested = Vec::new();
            for p in parts.iter().filter(|p| !p.is_empty()) {
                nested.push(self.lparser(option, p, pos - 1, opts)?);
            }
            return Ok(PyValue::Tuple(nested));
        }
        let mut res = Vec::new();
        for p in &parts {
            res.push(match opts.parser {
                ListParser::Float => PyValue::Float(
                    p.parse::<f64>().map_err(|_| self.unparsable(option))?,
                ),
                ListParser::Int => PyValue::Int(
                    p.parse::<i64>().map_err(|_| self.unparsable(option))?.into(),
                ),
                ListParser::Str => PyValue::Str(p.clone()),
            });
        }
        if let Some(count) = opts.count {
            if res.len() != count {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have {count} elements",
                    self.section
                )));
            }
        }
        Ok(PyValue::Tuple(res))
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.data.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        let Some(raw) = self.raw(option) else {
            return self.absent(option, opts.default, opts.note_valid, |v: &String| vs(v));
        };
        if opts.note_valid {
            self.track(option, vs(&raw));
        }
        Ok(Some(raw))
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.raw(option) else {
            return self.absent(option, opts.default, opts.note_valid, |v: &i64| Value::int(*v));
        };
        let v: i64 = raw.trim().parse().map_err(|_| self.unparsable(option))?;
        if opts.note_valid {
            self.track(option, Value::int(v));
        }
        if let Some(min) = opts.minval {
            if v < min {
                return Err(self.bound_error(option, "have minimum of", &min.to_string()));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(self.bound_error(option, "have maximum of", &max.to_string()));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.raw(option) else {
            return self.absent(option, opts.default, opts.note_valid, |v: &f64| vf(*v));
        };
        let v: f64 = raw.trim().parse().map_err(|_| self.unparsable(option))?;
        if opts.note_valid {
            self.track(option, vf(v));
        }
        let fs = pycompat::text::float_str;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(self.bound_error(option, "have minimum of", &fs(min)));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(self.bound_error(option, "have maximum of", &fs(max)));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(self.bound_error(option, "be above", &fs(above)));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(self.bound_error(option, "be below", &fs(below)));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let Some(raw) = self.raw(option) else {
            return self.absent(option, opts.default, opts.note_valid, |v: &bool| Value::Bool(*v));
        };
        let v = match raw.trim().to_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => true,
            "0" | "no" | "false" | "off" => false,
            _ => return Err(self.unparsable(option)),
        };
        if opts.note_valid {
            self.track(option, Value::Bool(v));
        }
        Ok(Some(v))
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: klippy_api::config::GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by stepper.py")
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        let Some(raw) = self.raw(option) else {
            let default = self.absent(option, opts.default.clone(), opts.note_valid, |v: &PyValue| {
                pyvalue_to_value(v)
            })?;
            return Ok(default.unwrap_or(PyValue::None));
        };
        let v = self.lparser(option, &raw, opts.seps.len() - 1, &opts)?;
        if opts.note_valid {
            self.track(option, pyvalue_to_value(&v));
        }
        Ok(v)
    }
    fn getlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetListOpts,
    ) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by stepper.py")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by stepper.py")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by stepper.py")
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig { section: section.to_string(), data: self.data.clone() })
    }
    fn has_section(&self, section: &str) -> bool {
        self.data.sections.contains_key(section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        self.data
            .sections
            .keys()
            .filter(|s| s.starts_with(prefix))
            .map(|s| self.getsection(s))
            .collect()
    }
    fn get_prefix_options(&self, prefix: &str) -> KResult<Vec<String>> {
        Ok(self.data
            .sections
            .get(&self.section)
            .map(|s| s.keys().filter(|o| o.starts_with(prefix)).cloned().collect())
            .unwrap_or_default())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> {
        panic!("deprecate is not used by stepper.py")
    }
}

struct FakeCmd {
    tag: i64,
}

impl klippy_api::mcu::CommandWrapperApi for FakeCmd {
    fn send(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        panic!("stepper.py only reads the command tag")
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        panic!("stepper.py only reads the command tag")
    }
    fn get_command_tag(&self) -> i64 {
        self.tag
    }
}

struct FakeQueryCmd {
    responses: Arc<Mutex<std::collections::VecDeque<PyValue>>>,
    log: Arc<Mutex<Vec<Vec<PyValue>>>>,
}

impl klippy_api::mcu::CommandQueryWrapperApi for FakeQueryCmd {
    fn send(&self, data: Vec<PyValue>, _opts: QuerySendOpts) -> KResult<PyValue> {
        self.log.lock().push(data);
        Ok(self.responses.lock().pop_front().expect("a queued query response"))
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("stepper.py does not use a preface command")
    }
}

struct NoopAsync;

impl klippy_api::mcu::AsyncResponseWrapperApi for NoopAsync {
    fn unregister(&self) {}
}

struct FakeMcu {
    printer: Printer,
    name: String,
    fileoutput: bool,
    constants: IndexMap<String, String>,
    next_oid: Mutex<i64>,
    config_cmds: Mutex<Vec<(String, AddConfigCmdOpts)>>,
    config_callbacks: Mutex<Vec<klippy_api::mcu::ConfigCallback>>,
    query_log: Arc<Mutex<Vec<Vec<PyValue>>>>,
    query_responses: Arc<Mutex<std::collections::VecDeque<PyValue>>>,
    query_lookups: Mutex<Vec<(String, String, Option<i64>)>>,
    tags: Mutex<IndexMap<String, i64>>,
    next_tag: Mutex<i64>,
}

impl FakeMcu {
    fn new(printer: Printer, name: &str, fileoutput: bool, constants: &[(String, String)]) -> FakeMcu {
        FakeMcu {
            printer,
            name: name.to_string(),
            fileoutput,
            constants: constants.iter().cloned().collect(),
            next_oid: Mutex::new(10),
            config_cmds: Mutex::new(Vec::new()),
            config_callbacks: Mutex::new(Vec::new()),
            query_log: Arc::new(Mutex::new(Vec::new())),
            query_responses: Arc::new(Mutex::new(std::collections::VecDeque::new())),
            query_lookups: Mutex::new(Vec::new()),
            tags: Mutex::new(IndexMap::new()),
            next_tag: Mutex::new(100),
        }
    }

    /// Run the registered `_build_config` callbacks like the MCU does at connect time.
    fn run_config_callbacks(&self) -> KResult<()> {
        let mut callbacks = std::mem::take(&mut *self.config_callbacks.lock());
        let mut result = Ok(());
        for cb in callbacks.iter_mut() {
            result = cb();
            if result.is_err() {
                break;
            }
        }
        self.config_callbacks.lock().append(&mut callbacks);
        result
    }

    fn tag_of(&self, msg: &str) -> i64 {
        *self.tags.lock().get(msg).expect("command was looked up")
    }

    fn queue_response(&self, pos: i64, receive_time: f64) {
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("pos".to_string()), PyValue::Int(pos.into()));
        map.insert(PyValue::Str("#receive_time".to_string()), PyValue::Float(receive_time));
        self.query_responses.lock().push_back(PyValue::Dict(map));
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        self.fileoutput
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("stepper.py sets pins up through the pins object")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut oid = self.next_oid.lock();
        *oid += 1;
        Ok(*oid)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.config_callbacks.lock().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.config_cmds.lock().push((cmd.to_string(), opts));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * FREQ) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.100
    }
    fn max_nominal_duration(&self) -> f64 {
        3.0
    }
    fn lookup_command(&self, msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        let mut tags = self.tags.lock();
        if let Some(tag) = tags.get(msgformat) {
            return Ok(Arc::new(FakeCmd { tag: *tag }));
        }
        let mut next = self.next_tag.lock();
        *next += 1;
        tags.insert(msgformat.to_string(), *next);
        Ok(Arc::new(FakeCmd { tag: *next }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.query_lookups.lock().push((msgformat.to_string(), respformat.to_string(), opts.oid));
        Ok(Arc::new(FakeQueryCmd {
            responses: self.query_responses.clone(),
            log: self.query_log.clone(),
        }))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        Arc::new(NoopAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        self.constants.clone()
    }
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        Err(KlippyError::Config(format!("Unknown constant {name}")))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * FREQ) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / FREQ
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 1.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("stepper.py does not create trigger dispatches")
    }
    fn serialqueue(&self) -> Option<Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
}

struct FakeMq {
    names: Mutex<Vec<String>>,
    flush_callbacks: Mutex<Vec<(FlushCallback, bool)>>,
    registered_flags: Mutex<Vec<bool>>,
    unregistered: Mutex<i64>,
    queued: Mutex<Vec<Vec<QueuedMsg>>>,
}

klippy_api::printer_object!(FakeMq; dyn MotionQueuingApi);

/// Fake `SyncEmitterApi`: a real `Stepcompress` (so fill/invert/reset/
/// last-position/find-past/extract behave like the port) plus the
/// kinematics slot and the queued-msg record `stepper.py` compares.
pub struct FakeSyncEmitter {
    sc: Mutex<chelper_motion::stepcompress::Stepcompress>,
    sk: Mutex<Option<StepperKinematics>>,
    index: usize,
    mq: *const FakeMq,
}

// SAFETY: only the test thread touches the fake.
unsafe impl Send for FakeSyncEmitter {}
unsafe impl Sync for FakeSyncEmitter {}

impl klippy_api::toolhead::SyncEmitterApi for FakeSyncEmitter {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn queue_msg(&self, req_clock: u64, data: &[u32]) {
        // SAFETY: the fake outlives the test that uses it.
        let mq = unsafe { &*self.mq };
        mq.queued.lock()[self.index].push((req_clock, data.to_vec()));
    }
    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>) {
        *self.sk.lock() = sk;
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        self.sk.lock().clone()
    }
    fn stepcompress_fill(&self, oid: u32, max_error: u32, queue_step_msgtag: i32, set_next_step_dir_msgtag: i32) {
        self.sc.lock().fill(oid, max_error, queue_step_msgtag, set_next_step_dir_msgtag);
    }
    fn stepcompress_set_invert_sdir(&self, invert: u32) {
        self.sc.lock().set_invert_sdir(invert);
    }
    fn stepcompress_reset(&self, last_step_clock: u64) -> i32 {
        self.sc.lock().reset(last_step_clock)
    }
    fn stepcompress_set_last_position(&self, clock: u64, last_position: i64) -> i32 {
        self.sc.lock().set_last_position(clock, last_position)
    }
    fn stepcompress_find_past_position(&self, clock: u64) -> i64 {
        self.sc.lock().find_past_position(clock)
    }
    fn stepcompress_extract_old(&self, max: i32, start_clock: u64, end_clock: u64) -> Vec<klippy_api::stepper::PullHistoryStep> {
        self.sc
            .lock()
            .extract_old(max, start_clock, end_clock)
            .iter()
            .map(|hs| klippy_api::stepper::PullHistoryStep {
                first_clock: hs.first_clock as i64,
                last_clock: hs.last_clock as i64,
                start_position: hs.start_position,
                interval: i64::from(hs.interval),
                step_count: i64::from(hs.step_count),
                add: i64::from(hs.add),
            })
            .collect()
    }
}

impl FakeMq {
    fn new() -> FakeMq {
        FakeMq {
            names: Mutex::new(Vec::new()),
            flush_callbacks: Mutex::new(Vec::new()),
            registered_flags: Mutex::new(Vec::new()),
            unregistered: Mutex::new(0),
            queued: Mutex::new(Vec::new()),
        }
    }

    /// Call the registered flush callbacks like `motion_queuing` does after a flush.
    fn flush(&self, must_flush_time: f64, max_step_gen_time: f64) {
        let callbacks: Vec<FlushCallback> =
            self.flush_callbacks.lock().iter().map(|(cb, _)| cb.clone()).collect();
        for cb in callbacks {
            cb(must_flush_time, max_step_gen_time).expect("stepper flush callback");
        }
    }
}

impl MotionQueuingApi for FakeMq {
    fn allocate_trapq(&self) -> Trapq {
        TrapqHandle::alloc()
    }
    fn lookup_trapq_append(&self) -> TrapqAppend {
        panic!("stepper.py does not append moves")
    }
    fn wipe_trapq(&self, _trapq: &Trapq) {}
    fn allocate_syncemitter(
        &self,
        _mcu: Mcu,
        name: &str,
        alloc_stepcompress: bool,
    ) -> KResult<SyncEmitterHandle> {
        assert!(alloc_stepcompress, "steppers allocate a stepcompress");
        let mut names = self.names.lock();
        names.push(name.to_string());
        self.queued.lock().push(Vec::new());
        let index = names.len() - 1;
        Ok(Arc::new(FakeSyncEmitter {
            sc: Mutex::new(chelper_motion::stepcompress::Stepcompress::new()),
            sk: Mutex::new(None),
            index,
            mq: self as *const FakeMq,
        }) as SyncEmitterHandle)
    }
    fn queue_syncemitter_msg(&self, syncemitter: &SyncEmitterHandle, req_clock: u64, data: &[u32]) {
        syncemitter.queue_msg(req_clock, data);
    }
    fn setup_mcu_movequeue(&self, _mcu: Mcu, _serialqueue: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>, _move_count: i64) {}
    fn register_flush_callback(&self, callback: FlushCallback, can_add_trapq: bool) {
        self.registered_flags.lock().push(can_add_trapq);
        let mut callbacks = self.flush_callbacks.lock();
        if can_add_trapq {
            callbacks.insert(0, (callback, can_add_trapq));
        } else {
            callbacks.push((callback, can_add_trapq));
        }
    }
    fn unregister_flush_callback(&self, callback: &FlushCallback) {
        let mut callbacks = self.flush_callbacks.lock();
        if let Some(pos) = callbacks.iter().position(|(cb, _)| same(cb, callback)) {
            callbacks.remove(pos);
            *self.unregistered.lock() += 1;
        }
    }
    fn register_flush_callback_named(
        &self,
        callback: FlushCallback,
        can_add_trapq: bool,
        _name: Option<&str>,
        _obj_name: Option<&str>,
    ) {
        self.register_flush_callback(callback, can_add_trapq);
    }
    fn get_kin_flush_delay(&self) -> f64 {
        0.001
    }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> {
        Ok(())
    }
    fn calc_step_gen_restart(&self, est_print_time: f64) -> f64 {
        est_print_time
    }
    fn note_mcu_movequeue_activity(&self, _mq_time: f64, _is_step_gen: bool) {}
    fn drip_update_time(
        &self,
        _start_time: f64,
        _end_time: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_drip_timing(&self) -> Option<f64> {
        None
    }
}

struct FakeEndstop {
    probe: Option<f64>,
    steppers: Mutex<Vec<Stepper>>,
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> klippy_api::KResult<Mcu> {
        panic!("stepper.py does not ask an endstop for its mcu")
    }
    fn add_stepper(&self, stepper: Stepper) {
        self.steppers.lock().push(stepper);
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.lock().clone()
    }
    fn home_start(
        &self,
        _print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> klippy_api::KResult<reactor::Completion<bool>> {
        panic!("stepper.py does not home")
    }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        panic!("stepper.py does not home")
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        panic!("stepper.py does not query endstops")
    }
    fn get_position_endstop(&self) -> Option<f64> {
        self.probe
    }
}

struct FakePins {
    mcu: Mcu,
    probe: Option<f64>,
    log: Mutex<Vec<Value>>,
    created: Mutex<Vec<(String, Arc<FakeEndstop>)>>,
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl FakePins {
    /// `pins.parse_pin` without the log entry.
    fn parse(&self, desc: &str, can_invert: bool, can_pullup: bool) -> PinParams {
        let mut desc = desc.trim();
        let mut pullup = 0;
        let mut invert = 0;
        if can_pullup && (desc.starts_with('^') || desc.starts_with('~')) {
            pullup = if desc.starts_with('~') { -1 } else { 1 };
            desc = desc[1..].trim();
        }
        if can_invert && desc.starts_with('!') {
            invert = 1;
            desc = desc[1..].trim();
        }
        let (chip_name, pin) = match desc.split_once(':') {
            None => ("mcu", desc),
            Some((chip, pin)) => (chip.trim(), pin.trim()),
        };
        PinParams {
            chip: self.mcu.clone(),
            chip_name: chip_name.to_string(),
            pin: pin.to_string(),
            invert,
            pullup,
            share_type: None,
        }
    }

    /// The pin description an endstop was set up with.
    fn desc_of(&self, endstop: &Arc<dyn EndstopApi>) -> String {
        self.created
            .lock()
            .iter()
            .find(|(_, e)| data_ptr(e) == data_ptr(endstop))
            .map(|(desc, _)| desc.clone())
            .expect("endstop from setup_pin")
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams> {
        self.log.lock().push(Value::Tuple(vec![
            vs("parse_pin"),
            vs(pin_desc),
            Value::Bool(opts.can_invert),
            Value::Bool(opts.can_pullup),
        ]));
        Ok(self.parse(pin_desc, opts.can_invert, opts.can_pullup))
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        self.log.lock().push(Value::Tuple(vec![
            vs("lookup_pin"),
            vs(pin_desc),
            Value::Bool(opts.can_invert),
            Value::Bool(opts.can_pullup),
            opts.share_type.as_deref().map_or(Value::Null, vs),
        ]));
        let mut params = self.parse(pin_desc, opts.can_invert, opts.can_pullup);
        params.share_type = opts.share_type;
        Ok(params)
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        assert_eq!(pin_type, "endstop");
        self.log.lock().push(Value::Tuple(vec![vs("setup_pin"), vs(pin_type), vs(pin_desc)]));
        let endstop =
            Arc::new(FakeEndstop { probe: self.probe, steppers: Mutex::new(Vec::new()) });
        self.created.lock().push((pin_desc.to_string(), endstop.clone()));
        Ok(Pin::Endstop(endstop))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("stepper.py does not use pin resolvers")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("stepper.py does not register chips")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("stepper.py does not share pins")
    }
}

struct FakeConfigfile {
    deprecated: Mutex<Vec<(String, String)>>,
}

klippy_api::printer_object!(FakeConfigfile; dyn PrinterConfigApi);

impl PrinterConfigApi for FakeConfigfile {
    fn get_printer(&self) -> Printer {
        panic!("not used")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("not used")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("not used")
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {}
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, mcu: &Mcu, feature: &str, _msg: Option<String>) {
        self.deprecated.lock().push((mcu.get_name(), feature.to_string()));
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) -> KResult<()> { Ok(()) }
    fn remove_section(&self, _section: &str) {}
}

struct FakeRegistrar {
    registered: Mutex<Vec<(String, String)>>,
}

klippy_api::printer_object!(FakeRegistrar; dyn StepperRegistryApi);

impl StepperRegistryApi for FakeRegistrar {
    fn register_stepper(&self, config: &Config, stepper: Stepper) -> KResult<()> {
        self.registered.lock().push((config.get_name(), stepper.get_name(None)));
        Ok(())
    }
}

struct FakeQueryEndstops {
    registered: Mutex<Vec<(Arc<dyn EndstopApi>, String)>>,
}

klippy_api::printer_object!(FakeQueryEndstops; dyn QueryEndstopsApi);

impl QueryEndstopsApi for FakeQueryEndstops {
    fn register_endstop(&self, mcu_endstop: Arc<dyn EndstopApi>, name: &str) {
        self.registered.lock().push((mcu_endstop, name.to_string()));
    }
}

// ---------------------------------------------------------------------------
// World: the same printer on both sides.
// ---------------------------------------------------------------------------

const REGISTRARS: [&str; 3] = ["stepper_enable", "force_move", "motion_report"];

#[derive(Default, Clone)]
struct WorldCfg {
    sections: Vec<(String, Vec<(String, String)>)>,
    constants: Vec<(String, String)>,
    fileoutput: bool,
    probe: Option<f64>,
}

impl WorldCfg {
    fn section(mut self, name: &str, options: &[(&str, &str)]) -> WorldCfg {
        self.sections.push((
            name.to_string(),
            options.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        ));
        self
    }

    fn constants(mut self, constants: &[(&str, &str)]) -> WorldCfg {
        self.constants =
            constants.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect();
        self
    }

    fn fileoutput(mut self) -> WorldCfg {
        self.fileoutput = true;
        self
    }

    fn probe(mut self, position: f64) -> WorldCfg {
        self.probe = Some(position);
        self
    }

    fn ini(&self) -> String {
        let mut text = String::new();
        for (name, options) in &self.sections {
            text.push_str(&format!("[{name}]\n"));
            for (k, v) in options {
                text.push_str(&format!("{k}: {v}\n"));
            }
        }
        text
    }
}

struct World {
    printer: Arc<FakePrinter>,
    mcu: Arc<FakeMcu>,
    pins: Arc<FakePins>,
    mq: Arc<FakeMq>,
    configfile: Arc<FakeConfigfile>,
    qes: Arc<FakeQueryEndstops>,
    registrars: Vec<(&'static str, Arc<FakeRegistrar>)>,
    data: Arc<ConfigData>,
}

fn new_world(py: &mut Oracle, cfg: &WorldCfg) -> World {
    let printer = Arc::new(FakePrinter::new());
    let printer_ref: Printer = printer.clone();
    let mcu = Arc::new(FakeMcu::new(printer_ref.clone(), "mcu", cfg.fileoutput, &cfg.constants));
    let mcu_ref: Mcu = mcu.clone();
    let pins = Arc::new(FakePins {
        mcu: mcu_ref.clone(),
        probe: cfg.probe,
        log: Mutex::new(Vec::new()),
        created: Mutex::new(Vec::new()),
    });
    let mq = Arc::new(FakeMq::new());
    let configfile = Arc::new(FakeConfigfile { deprecated: Mutex::new(Vec::new()) });
    let qes = Arc::new(FakeQueryEndstops { registered: Mutex::new(Vec::new()) });
    printer.add("pins", pins.clone());
    printer.add("motion_queuing", mq.clone());
    printer.add("configfile", configfile.clone());
    printer.add("query_endstops", qes.clone());
    let mut registrars = Vec::new();
    for name in REGISTRARS {
        let registrar = Arc::new(FakeRegistrar { registered: Mutex::new(Vec::new()) });
        printer.add(name, registrar.clone());
        registrars.push((name, registrar));
    }
    let sections = cfg
        .sections
        .iter()
        .map(|(name, options)| {
            (
                name.clone(),
                options.iter().map(|(k, v)| (k.to_lowercase(), v.clone())).collect(),
            )
        })
        .collect();
    let data = Arc::new(ConfigData {
        printer: printer_ref,
        sections,
        tracking: Mutex::new(IndexMap::new()),
    });
    let constants = cfg
        .constants
        .iter()
        .map(|(k, v)| format!("{k:?}: {v:?}"))
        .collect::<Vec<_>>()
        .join(", ");
    py.exec(&format!(
        "build_world({:?}, {{{constants}}}, {}, {})",
        cfg.ini(),
        pyb(cfg.fileoutput),
        py_opt_f(cfg.probe)
    ))
    .expect("python world");
    World { printer, mcu, pins, mq, configfile, qes, registrars, data }
}

impl World {
    fn config(&self, section: &str) -> Config {
        Arc::new(FakeConfig { section: section.to_string(), data: self.data.clone() })
    }

    fn remove_object(&self, name: &str) {
        self.printer.objects.lock().shift_remove(name);
    }

    fn state(&self) -> IndexMap<&'static str, Value> {
        let mut m = IndexMap::new();
        let pair = |a: Value, b: Value| Value::Tuple(vec![a, b]);
        m.insert(
            "tracking",
            Value::List(
                self.data
                    .tracking
                    .lock()
                    .iter()
                    .map(|((s, o), v)| pair(pair(vs(s), vs(o)), v.clone()))
                    .collect(),
            ),
        );
        m.insert("pin_log", Value::List(self.pins.log.lock().clone()));
        m.insert(
            "qes",
            Value::List(
                self.qes
                    .registered
                    .lock()
                    .iter()
                    .map(|(e, n)| pair(vs(&self.pins.desc_of(e)), vs(n)))
                    .collect(),
            ),
        );
        m.insert(
            "registrars",
            Value::List(
                self.registrars
                    .iter()
                    .filter(|(name, _)| self.printer.objects.lock().contains_key(*name))
                    .map(|(name, r)| {
                        pair(
                            vs(name),
                            Value::List(
                                r.registered.lock().iter().map(|(c, s)| pair(vs(c), vs(s))).collect(),
                            ),
                        )
                    })
                    .collect(),
            ),
        );
        m.insert(
            "syncemitters",
            Value::List(self.mq.names.lock().iter().map(|n| vs(n)).collect()),
        );
        m.insert("oid", Value::int(*self.mcu.next_oid.lock()));
        m.insert(
            "handlers",
            Value::List(
                self.printer
                    .handlers
                    .lock()
                    .iter()
                    .map(|(e, h)| pair(vs(e), Value::int(h.len() as i64)))
                    .collect(),
            ),
        );
        m.insert("callbacks", Value::int(self.mcu.config_callbacks.lock().len() as i64));
        m.insert(
            "deprecated",
            Value::List(
                self.configfile.deprecated.lock().iter().map(|(a, b)| pair(vs(a), vs(b))).collect(),
            ),
        );
        m.insert(
            "events",
            Value::List(
                self.printer
                    .events
                    .lock()
                    .iter()
                    .map(|(e, names)| pair(vs(e), Value::List(names.iter().map(|n| vs(n)).collect())))
                    .collect(),
            ),
        );
        m.insert(
            "config_cmds",
            Value::List(
                self.mcu
                    .config_cmds
                    .lock()
                    .iter()
                    .map(|(c, o)| {
                        Value::Tuple(vec![vs(c), Value::Bool(o.is_init), Value::Bool(o.on_restart)])
                    })
                    .collect(),
            ),
        );
        m.insert(
            "tags",
            Value::List(self.mcu.tags.lock().iter().map(|(k, v)| pair(vs(k), Value::int(*v))).collect()),
        );
        m.insert(
            "query_lookups",
            Value::List(
                self.mcu
                    .query_lookups
                    .lock()
                    .iter()
                    .map(|(a, b, oid)| {
                        Value::Tuple(vec![vs(a), vs(b), oid.map_or(Value::Null, Value::int)])
                    })
                    .collect(),
            ),
        );
        m.insert(
            "query_log",
            Value::List(
                self.mcu
                    .query_log
                    .lock()
                    .iter()
                    .map(|row| Value::List(row.iter().map(pyvalue_to_value).collect()))
                    .collect(),
            ),
        );
        m.insert(
            "flush",
            Value::Tuple(vec![
                Value::int(self.mq.flush_callbacks.lock().len() as i64),
                Value::List(self.mq.registered_flags.lock().iter().map(|b| Value::Bool(*b)).collect()),
                Value::int(*self.mq.unregistered.lock()),
            ]),
        );
        m.insert(
            "queued",
            Value::List(
                self.mq
                    .queued
                    .lock()
                    .iter()
                    .map(|se| {
                        Value::List(
                            se.iter()
                                .map(|(req, words)| {
                                    pair(
                                        Value::int(*req as i64),
                                        Value::Tuple(words.iter().map(|w| Value::int(i64::from(*w))).collect()),
                                    )
                                })
                                .collect(),
                        )
                    })
                    .collect(),
            ),
        );
        m
    }
}

/// Compare every observable effect on the fakes, except the aspects in `skip`.
fn compare_state(label: &str, py: &mut Oracle, world: &World, skip: &[&str]) {
    for (aspect, value) in world.state() {
        if skip.contains(&aspect) {
            continue;
        }
        check(&format!("{label}: {aspect}"), py.eval(&format!("world_state()[{aspect:?}]")), value);
    }
}

// ---------------------------------------------------------------------------
// Steppers.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Spec {
    section: &'static str,
    step_pin: &'static str,
    dir_pin: &'static str,
    rotation: f64,
    steps: f64,
    pulse: Option<f64>,
    units: bool,
}

impl Spec {
    fn new() -> Spec {
        Spec {
            section: "stepper_x",
            step_pin: "PB0",
            dir_pin: "!PB1",
            rotation: 40.0,
            steps: 1600.0,
            pulse: None,
            units: false,
        }
    }
}

fn rust_stepper(world: &World, spec: &Spec) -> KResult<Arc<McuStepper>> {
    McuStepper::new(
        &world.config(spec.section),
        world.pins.parse(spec.step_pin, true, false),
        world.pins.parse(spec.dir_pin, true, false),
        spec.rotation,
        spec.steps,
        spec.pulse,
        spec.units,
    )
}

fn py_stepper(py: &mut Oracle, var: &str, spec: &Spec) -> pyoracle::OracleResult {
    py.exec(&format!(
        "{var} = stepper.MCU_stepper(cfg({:?}), pins._parse({:?}, True, False), \
         pins._parse({:?}, True, False), {}, {}, {}, {})",
        spec.section,
        spec.step_pin,
        spec.dir_pin,
        pyf(spec.rotation),
        pyf(spec.steps),
        py_opt_f(spec.pulse),
        pyb(spec.units)
    ))
}

fn stepper_summary(label: &str, py: &mut Oracle, var: &str, rust: &dyn McuStepperApi) {
    let c = |what: &str, expr: String, actual: Value| {
        (format!("{label}: {what}"), expr, actual)
    };
    let (rd, spr) = rust.get_rotation_distance();
    let (inv, orig) = rust.get_dir_inverted();
    let (pulse, both) = rust.get_pulse_duration();
    let rows = vec![
        c("oid", format!("{var}.get_oid()"), Value::int(rust.get_oid())),
        c("name", format!("{var}.get_name()"), vs(&rust.get_name(None))),
        c("name short", format!("{var}.get_name(short=True)"), vs(&rust.get_name(Some(true)))),
        c("name full", format!("{var}.get_name(short=False)"), vs(&rust.get_name(Some(false)))),
        c("step_dist", format!("{var}.get_step_dist()"), vf(rust.get_step_dist())),
        c("rotation", format!("{var}.get_rotation_distance()"), Value::Tuple(vec![vf(rd), vf(spr)])),
        c("units", format!("{var}.units_in_radians()"), Value::Bool(rust.units_in_radians())),
        c(
            "pulse",
            format!("{var}.get_pulse_duration()"),
            Value::Tuple(vec![pulse.map_or(Value::Null, vf), Value::Bool(both)]),
        ),
        c(
            "dir inverted",
            format!("tuple(bool(x) for x in {var}.get_dir_inverted())"),
            Value::Tuple(vec![Value::Bool(inv), Value::Bool(orig)]),
        ),
        c("mcu", format!("{var}.get_mcu().get_name()"), vs(&rust.get_mcu().get_name())),
    ];
    for (label, expr, actual) in rows {
        check(&label, py.eval(&expr), actual);
    }
}

struct Alloc {
    func: &'static str,
    params: Vec<PyValue>,
    py_args: &'static str,
}

fn allocs() -> Vec<Alloc> {
    let byte = |c: u8| vec![PyValue::Bytes(vec![c])];
    vec![
        Alloc { func: "cartesian_stepper_alloc", params: byte(b'x'), py_args: "b'x'" },
        Alloc { func: "cartesian_stepper_alloc", params: byte(b'y'), py_args: "b'y'" },
        Alloc { func: "cartesian_stepper_alloc", params: byte(b'z'), py_args: "b'z'" },
        Alloc { func: "corexy_stepper_alloc", params: byte(b'+'), py_args: "b'+'" },
        Alloc { func: "corexy_stepper_alloc", params: byte(b'-'), py_args: "b'-'" },
        Alloc { func: "corexz_stepper_alloc", params: byte(b'+'), py_args: "b'+'" },
        Alloc { func: "corexz_stepper_alloc", params: byte(b'-'), py_args: "b'-'" },
        Alloc {
            func: "generic_cartesian_stepper_alloc",
            params: vec![PyValue::Float(1.5), PyValue::Float(0.0), PyValue::Float(-0.5)],
            py_args: "1.5, 0.0, -0.5",
        },
    ]
}

fn setup_itersolve_both(py: &mut Oracle, var: &str, rust: &dyn McuStepperApi, alloc: &Alloc) {
    rust.setup_itersolve(alloc.func, &alloc.params);
    py.exec(&format!("{var}.setup_itersolve({:?}, {})", alloc.func, alloc.py_args))
        .expect("py setup_itersolve");
}

fn sk_state(sk: &StepperKinematics) -> (f64, bool, f64) {
    kinematics_of(sk).expect("core handle").with_sk(|sk| (sk.step_dist, sk.tq.is_some(), sk.commanded_pos))
}

fn add_move(tq: &Trapq, print_time: f64, move_t: f64, axes_r: [f64; 3]) {
    trapq_of(tq)
        .expect("core trapq")
        .shared()
        .lock()
        .append(print_time, 0.0, move_t, 0.0, 0.0, 0.0, 0.0, axes_r[0], axes_r[1], axes_r[2], 0.0, 1.0, 0.0);
}

fn py_add_move(py: &mut Oracle, var: &str, print_time: f64, move_t: f64, axes_r: [f64; 3]) {
    py.exec(&format!(
        "{var}.add_move({}, {}, ({}, {}, {}))",
        pyf(print_time),
        pyf(move_t),
        pyf(axes_r[0]),
        pyf(axes_r[1]),
        pyf(axes_r[2])
    ))
    .expect("py add_move");
}

// ---------------------------------------------------------------------------
// Tests: MCU_stepper.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_empty_and_constants() {
    let m = module_factory();
    assert!(
        m.load_config.is_none()
            && m.load_config_prefix.is_none()
            && m.add_early_printer_objects.is_none()
            && m.add_printer_objects.is_none()
    );
    let Some(mut py) = oracle() else { return };
    check(
        "MIN_BOTH_EDGE_DURATION",
        py.eval("stepper.MIN_BOTH_EDGE_DURATION"),
        vf(klippy_core::stepper::MIN_BOTH_EDGE_DURATION),
    );
    check(
        "MIN_OPTIMIZED_BOTH_EDGE_DURATION",
        py.eval("stepper.MIN_OPTIMIZED_BOTH_EDGE_DURATION"),
        vf(klippy_core::stepper::MIN_OPTIMIZED_BOTH_EDGE_DURATION),
    );
    check(
        "MAX_STEPCOMPRESS_ERROR",
        py.eval("stepper.MAX_STEPCOMPRESS_ERROR"),
        vf(klippy_core::stepper::MAX_STEPCOMPRESS_ERROR),
    );
}

#[test]
fn init_identity() {
    let Some(mut py) = oracle() else { return };
    let specs = [
        Spec::new(),
        Spec { section: "stepper_z1", step_pin: "!PD3", dir_pin: "PD4", ..Spec::new() },
        Spec { section: "manual_stepper my_axis", pulse: Some(0.000004), ..Spec::new() },
        Spec { section: "extruder", rotation: 33.5, steps: 3200.0, units: true, ..Spec::new() },
    ];
    for spec in specs {
        let world = new_world(&mut py, &WorldCfg::default().section(spec.section, &[]));
        let rust = rust_stepper(&world, &spec).expect("rust stepper");
        py_stepper(&mut py, "s", &spec).expect("python stepper");
        stepper_summary(spec.section, &mut py, "s", &*rust);
        compare_state(spec.section, &mut py, &world, &["pin_log"]);
    }
}

#[test]
fn init_errors() {
    let Some(mut py) = oracle() else { return };
    let cfg = WorldCfg::default().section("stepper_x", &[]);

    // The dir pin on another MCU: the oid and the config callback are already taken.
    let world = new_world(&mut py, &cfg);
    let other: Mcu = Arc::new(FakeMcu::new(world.printer.clone(), "other", false, &[]));
    let mut dir = world.pins.parse("!PB1", true, false);
    dir.chip = other;
    let err = McuStepper::new(
        &world.config("stepper_x"),
        world.pins.parse("PB0", true, false),
        dir,
        40.0,
        1600.0,
        None,
        false,
    )
    .err()
    .expect("dir pin mismatch");
    py.exec("other_mcu = FakeMcu(printer, 'other', 12000000.0, False, {})").expect("other mcu");
    let expected = py.exec(
        "stepper.MCU_stepper(cfg('stepper_x'), pins._parse('PB0', True, False), \
         {'chip': other_mcu, 'pin': 'PB1', 'invert': 1}, 40.0, 1600.0, None, False)",
    );
    assert!(!compare_unit("dir mismatch", expected, &Err(err)));
    compare_state("dir mismatch", &mut py, &world, &["pin_log"]);

    // steps_per_rotation of zero: ZeroDivisionError before the oid is taken.
    let world = new_world(&mut py, &cfg);
    let spec = Spec { steps: 0.0, ..Spec::new() };
    let err = rust_stepper(&world, &spec).err().expect("zero steps");
    let expected = py_stepper(&mut py, "s", &spec);
    assert!(!compare_unit("zero steps", expected, &Err(err)));
    compare_state("zero steps", &mut py, &world, &["pin_log"]);

    // No motion_queuing object: the oid and callback are taken, the lookup fails.
    let world = new_world(&mut py, &cfg);
    world.remove_object("motion_queuing");
    py.exec("del printer.objects['motion_queuing']").expect("remove python object");
    let err = rust_stepper(&world, &Spec::new()).err().expect("no motion_queuing");
    let expected = py_stepper(&mut py, "s", &Spec::new());
    assert!(!compare_unit("no motion_queuing", expected, &Err(err)));
    compare_state("no motion_queuing", &mut py, &world, &["pin_log"]);
}

#[test]
fn build_config_matrix() {
    let Some(mut py) = oracle() else { return };
    // (step pin, pulse duration, constants, requested both edges, expect a config error)
    let rows: Vec<BuildRow> = vec![
        ("PB0", None, vec![("STEPPER_STEP_BOTH_EDGE", "1")], true, false),
        ("PB0", None, vec![("STEPPER_STEP_BOTH_EDGE", "1")], false, false),
        ("!PB0", Some(0.000001), vec![("STEPPER_STEP_BOTH_EDGE", "1"), ("STEPPER_BOTH_EDGE", "1")], true, false),
        ("PB0", Some(0.0000003), vec![("STEPPER_STEP_BOTH_EDGE", "1")], true, false),
        ("PB0", Some(0.0000002), vec![], true, false),
        ("PB0", Some(0.0000002), vec![("STEPPER_STEP_BOTH_EDGE", "1"), ("STEPPER_OPTIMIZED_UNSTEP", "1")], true, false),
        ("!PB0", Some(0.0000002), vec![("STEPPER_BOTH_EDGE", "1")], true, false),
        ("PB0", Some(0.0000001), vec![("STEPPER_BOTH_EDGE", "1")], true, false),
        ("PB0", Some(0.0000004), vec![("STEPPER_BOTH_EDGE", "1"), ("STEPPER_STEP_BOTH_EDGE", "1")], true, false),
        ("PB0", None, vec![], false, false),
        ("PB0", Some(0.0), vec![("STEPPER_STEP_BOTH_EDGE", "2")], true, false),
        ("PB0", None, vec![("STEPPER_BOTH_EDGE", "x")], true, true),
        ("PB0", None, vec![("STEPPER_STEP_BOTH_EDGE", "1"), ("STEPPER_OPTIMIZED_UNSTEP", "1.5")], true, true),
    ];
    for (i, (step_pin, pulse, constants, req, expect_error)) in rows.iter().enumerate() {
        let label = format!("row {i}");
        let world = new_world(
            &mut py,
            &WorldCfg::default().section("stepper_x", &[]).constants(constants),
        );
        let spec = Spec { step_pin, pulse: *pulse, ..Spec::new() };
        let rust = rust_stepper(&world, &spec).expect("rust stepper");
        py_stepper(&mut py, "s", &spec).expect("python stepper");
        rust.setup_default_pulse_duration(0.000002, *req);
        py.exec(&format!("s.setup_default_pulse_duration(0.000002, {})", pyb(*req)))
            .expect("py setup_default_pulse_duration");
        let rust_result = world.mcu.run_config_callbacks();
        let py_result = py.exec("for cb in list(mcu.config_callbacks): cb()");
        let ok = compare_unit(&label, py_result, &rust_result);
        assert_eq!(ok, !*expect_error, "{label}: expected error state");
        stepper_summary(&label, &mut py, "s", &*rust);
        compare_state(&label, &mut py, &world, &["pin_log"]);
        if ok {
            // `stepcompress_fill` is private to the port; compare the Python record with the
            // values the fake MCU handed out for the same commands.
            let fill = Value::Tuple(vec![
                Value::int(rust.get_oid()),
                Value::int(world.mcu.seconds_to_clock(klippy_core::stepper::MAX_STEPCOMPRESS_ERROR)),
                Value::int(world.mcu.tag_of("queue_step oid=%c interval=%u count=%hu add=%hi")),
                Value::int(world.mcu.tag_of("set_next_step_dir oid=%c dir=%c")),
            ]);
            check(&format!("{label}: fill"), py.eval("mq.syncemitters[-1]['sc']['fill']"), fill);
        }
    }
}

#[test]
fn mcu_position_math() {
    let Some(mut py) = oracle() else { return };
    let coords: [[f64; 3]; 7] = [
        [0.0, 0.0, 0.0],
        [10.25, 3.5, -2.0],
        [100.0, 200.0, 50.0],
        [-1.0, 2.0, 3.0],
        [0.0125, 0.0, 0.0],
        [-0.0125, 0.0, 0.0],
        [0.0375, 0.0, 0.0],
    ];
    for (rotation, steps) in [(40.0, 1600.0), (8.0, 640.0)] {
        for alloc in allocs() {
            let label = format!("{} {} rot {rotation}", alloc.func, alloc.py_args);
            let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]));
            let spec = Spec { rotation, steps, ..Spec::new() };
            let rust = rust_stepper(&world, &spec).expect("rust stepper");
            py_stepper(&mut py, "s", &spec).expect("python stepper");
            setup_itersolve_both(&mut py, "s", &*rust, &alloc);
            for c in coords {
                let label = format!("{label} {c:?}");
                rust.set_position(c);
                py.exec(&format!("s.set_position([{}, {}, {}])", pyf(c[0]), pyf(c[1]), pyf(c[2])))
                    .expect("py set_position");
                check(&format!("{label}: commanded"), py.eval("s.get_commanded_position()"), vf(rust.get_commanded_position()));
                check(&format!("{label}: mcu pos"), py.eval("s.get_mcu_position()"), Value::int(rust.get_mcu_position(None)));
                check(&format!("{label}: mcu pos explicit"), py.eval("s.get_mcu_position(3.5)"), Value::int(rust.get_mcu_position(Some(3.5))));
                check(&format!("{label}: mcu pos negative"), py.eval("s.get_mcu_position(-3.5)"), Value::int(rust.get_mcu_position(Some(-3.5))));
                check(&format!("{label}: to commanded"), py.eval("s.mcu_to_commanded_position(7)"), vf(rust.mcu_to_commanded_position(7)));
                check(&format!("{label}: to commanded negative"), py.eval("s.mcu_to_commanded_position(-7)"), vf(rust.mcu_to_commanded_position(-7)));
                check(
                    &format!("{label}: calc"),
                    py.eval("s.calc_position_from_coord([10.0, 20.0, 30.0])"),
                    vf(rust.calc_position_from_coord([10.0, 20.0, 30.0])),
                );
            }
            // `set_rotation_distance` keeps the MCU position.
            let before = rust.get_mcu_position(None);
            rust.set_rotation_distance(80.0);
            py.exec("s.set_rotation_distance(80.0)").expect("py set_rotation_distance");
            assert_eq!(before, rust.get_mcu_position(None), "{label}: mcu position kept");
            stepper_summary(&label, &mut py, "s", &*rust);
            check(&label, py.eval("s.get_mcu_position()"), Value::int(rust.get_mcu_position(None)));
            check(&label, py.eval("s.mcu_to_commanded_position(7)"), vf(rust.mcu_to_commanded_position(7)));
            let sk = rust.get_stepper_kinematics().expect("sk");
            check(
                &format!("{label}: sk step_dist"),
                py.eval("s.get_stepper_kinematics()['step_dist']"),
                vf(sk_state(&sk).0),
            );
        }
    }
}

#[test]
fn is_active_axis() {
    let Some(mut py) = oracle() else { return };
    let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]));
    let rust = rust_stepper(&world, &Spec::new()).expect("rust stepper");
    py_stepper(&mut py, "s", &Spec::new()).expect("python stepper");
    for alloc in allocs() {
        setup_itersolve_both(&mut py, "s", &*rust, &alloc);
        for axis in ['x', 'y', 'z', 'e', 'X', 'a', '{'] {
            check(
                &format!("{} {} axis {axis}", alloc.func, alloc.py_args),
                py.eval(&format!("bool(s.is_active_axis({axis:?}))")),
                Value::Bool(rust.is_active_axis(axis)),
            );
        }
    }
}

#[test]
fn kinematics_and_trapq() {
    let Some(mut py) = oracle() else { return };
    let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]));
    let rust = rust_stepper(&world, &Spec::new()).expect("rust stepper");
    py_stepper(&mut py, "s", &Spec::new()).expect("python stepper");
    assert!(rust.get_stepper_kinematics().is_none());
    check("sk none", py.eval("s.get_stepper_kinematics()"), Value::Null);
    assert!(rust.get_trapq().is_none());
    check("trapq none", py.eval("s.get_trapq()"), Value::Null);

    let all_allocs = allocs();
    let alloc = &all_allocs[0];
    setup_itersolve_both(&mut py, "s", &*rust, alloc);
    let sk1 = rust.get_stepper_kinematics().expect("sk after setup_itersolve");
    py.exec("sk1 = s.get_stepper_kinematics()").expect("py sk1");
    let (step_dist, has_tq, _) = sk_state(&sk1);
    check("sk step_dist", py.eval("sk1['step_dist']"), vf(step_dist));
    check("sk has trapq", py.eval("sk1['tq'] is not None"), Value::Bool(has_tq));
    assert!(rust.get_trapq().is_none());

    // set_trapq returns the previous trapq and installs the new one in the kinematics.
    let tq1 = TrapqHandle::alloc();
    py.exec("tq1 = FakeTrapq()").expect("py tq1");
    let old = rust.set_trapq(Some(tq1.clone()));
    py.exec("old = s.set_trapq(tq1)").expect("py set_trapq");
    check("first set_trapq returns NULL", py.eval("old"), Value::Null);
    assert!(old.is_none());
    check("installed", py.eval("s.get_trapq() is tq1"), Value::Bool(same(&rust.get_trapq().expect("trapq"), &tq1)));
    check("sk has trapq", py.eval("sk1['tq'] is tq1"), Value::Bool(sk_state(&sk1).1));
    let tq2 = TrapqHandle::alloc();
    py.exec("tq2 = FakeTrapq()").expect("py tq2");
    let old = rust.set_trapq(Some(tq2.clone())).expect("previous trapq");
    py.exec("old = s.set_trapq(tq2)").expect("py set_trapq 2");
    check("second set_trapq returns tq1", py.eval("old is tq1"), Value::Bool(same(&old, &tq1)));
    let old = rust.set_trapq(None).expect("previous trapq");
    py.exec("old = s.set_trapq(None)").expect("py set_trapq None");
    check("set_trapq(None) returns tq2", py.eval("old is tq2"), Value::Bool(same(&old, &tq2)));
    check("trapq cleared", py.eval("s.get_trapq()"), Value::Null);
    assert!(rust.get_trapq().is_none());
    check("sk trapq cleared", py.eval("sk1['tq'] is not None"), Value::Bool(sk_state(&sk1).1));
    rust.set_trapq(Some(tq2.clone()));
    py.exec("s.set_trapq(tq2)").expect("py set_trapq tq2");

    // Replacing the kinematics keeps the MCU position, the trapq and the step distance.
    rust.set_position([12.5, 0.0, 0.0]);
    py.exec("s.set_position([12.5, 0.0, 0.0])").expect("py set_position");
    let before = rust.get_mcu_position(None);
    let sk2 = alloc_stepper_kinematics("cartesian_stepper_alloc", &[PyValue::Bytes(b"y".to_vec())])
        .expect("sk2");
    py.exec("sk2 = _flib.cartesian_stepper_alloc(b'y')").expect("py sk2");
    let old = rust.set_stepper_kinematics(Some(sk2.clone())).expect("previous kinematics");
    py.exec("old = s.set_stepper_kinematics(sk2)").expect("py set_stepper_kinematics");
    check("old kinematics", py.eval("old is sk1"), Value::Bool(same(&old, &sk1)));
    check("new kinematics", py.eval("s.get_stepper_kinematics() is sk2"), Value::Bool(same(&rust.get_stepper_kinematics().expect("sk"), &sk2)));
    assert_eq!(before, rust.get_mcu_position(None));
    check("mcu position kept", py.eval("s.get_mcu_position()"), Value::int(rust.get_mcu_position(None)));
    check("new commanded", py.eval("s.get_commanded_position()"), vf(rust.get_commanded_position()));
    let (step_dist, has_tq, commanded) = sk_state(&sk2);
    check("sk2 step_dist", py.eval("sk2['step_dist']"), vf(step_dist));
    check("sk2 trapq", py.eval("sk2['tq'] is tq2"), Value::Bool(has_tq));
    check("sk2 commanded", py.eval("sk2['commanded_pos']"), vf(commanded));

    // The first setup of a stepper that already has a trapq installs it into the new kinematics.
    rust.set_rotation_distance(80.0);
    py.exec("s.set_rotation_distance(80.0)").expect("py set_rotation_distance");
    check("sk2 step_dist after rotation change", py.eval("sk2['step_dist']"), vf(sk_state(&sk2).0));
    compare_state("kinematics", &mut py, &world, &["pin_log"]);
}

#[test]
fn dir_inverted_events() {
    let Some(mut py) = oracle() else { return };
    for dir_pin in ["!PB1", "PB1"] {
        let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]));
        let spec = Spec { dir_pin, ..Spec::new() };
        let rust = rust_stepper(&world, &spec).expect("rust stepper");
        py_stepper(&mut py, "s", &spec).expect("python stepper");
        for (i, inverted) in [true, true, false, false, true].into_iter().enumerate() {
            let label = format!("{dir_pin} step {i}");
            rust.set_dir_inverted(inverted).expect("set_dir_inverted");
            py.exec(&format!("s.set_dir_inverted({})", pyb(inverted))).expect("py set_dir_inverted");
            let (inv, orig) = rust.get_dir_inverted();
            check(
                &label,
                py.eval("tuple(bool(x) for x in s.get_dir_inverted())"),
                Value::Tuple(vec![Value::Bool(inv), Value::Bool(orig)]),
            );
            compare_state(&label, &mut py, &world, &["pin_log"]);
        }
    }
}

#[test]
fn query_mcu_position_and_history() {
    let Some(mut py) = oracle() else { return };
    let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]));
    let rust = rust_stepper(&world, &Spec::new()).expect("rust stepper");
    py_stepper(&mut py, "s", &Spec::new()).expect("python stepper");
    world.mcu.run_config_callbacks().expect("rust build_config");
    py.exec("for cb in list(mcu.config_callbacks): cb()").expect("py build_config");
    setup_itersolve_both(&mut py, "s", &*rust, &allocs()[0]);

    let replies = [(123, 100.0, true), (-40, 101.5, false), (7, 0.25, false)];
    for (i, (pos, receive_time, inverted)) in replies.into_iter().enumerate() {
        let label = format!("query {i}");
        rust.set_dir_inverted(inverted).expect("set_dir_inverted");
        py.exec(&format!("s.set_dir_inverted({})", pyb(inverted))).expect("py set_dir_inverted");
        world.mcu.queue_response(pos, receive_time);
        py.exec(&format!("mcu.query_responses.append({{'pos': {pos}, '#receive_time': {}}})", pyf(receive_time)))
            .expect("py queue reply");
        // The klippy:connect handler runs `_query_mcu_position`.
        world.printer.fire("klippy:connect").expect("rust connect");
        py.exec("printer.send_event('klippy:connect')").expect("py connect");
        check(&format!("{label}: mcu pos"), py.eval("s.get_mcu_position()"), Value::int(rust.get_mcu_position(None)));
        check(&format!("{label}: commanded"), py.eval("s.get_commanded_position()"), vf(rust.get_commanded_position()));
        for t in [0.0, 0.2, 99.0, 100.0, 100.5, 101.5, 101.6, 200.0] {
            check(
                &format!("{label}: past position {t}"),
                py.eval(&format!("s.get_past_mcu_position({})", pyf(t))),
                Value::int(rust.get_past_mcu_position(t)),
            );
        }
        for (count, start, end) in [
            (10, 0, i64::MAX - 1),
            (1, 0, i64::MAX - 1),
            (0, 0, i64::MAX - 1),
            (10, 1_200_000_000, 1_300_000_000),
            (10, 1_218_000_000, 1_218_000_001),
            (10, 5, 6),
        ] {
            let (steps, n) = rust.dump_steps(count, start, end);
            let rows = Value::Tuple(vec![
                Value::int(n),
                Value::List(
                    steps
                        .iter()
                        .map(|s| {
                            Value::Tuple(vec![
                                Value::int(s.first_clock),
                                Value::int(s.last_clock),
                                Value::int(s.start_position),
                                Value::int(s.interval),
                                Value::int(s.step_count),
                                Value::int(s.add),
                            ])
                        })
                        .collect(),
                ),
            ]);
            check(&format!("{label}: dump {count} {start} {end}"), py.eval(&format!("dump_steps(s, {count}, {start}, {end})")), rows);
        }
        compare_state(&label, &mut py, &world, &["pin_log"]);
    }
}

#[test]
fn file_output_skips_position_query() {
    let Some(mut py) = oracle() else { return };
    let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]).fileoutput());
    let rust = rust_stepper(&world, &Spec::new()).expect("rust stepper");
    py_stepper(&mut py, "s", &Spec::new()).expect("python stepper");
    world.mcu.run_config_callbacks().expect("rust build_config");
    py.exec("for cb in list(mcu.config_callbacks): cb()").expect("py build_config");
    setup_itersolve_both(&mut py, "s", &*rust, &allocs()[0]);
    world.printer.fire("klippy:connect").expect("rust connect");
    py.exec("printer.send_event('klippy:connect')").expect("py connect");
    rust.note_homing_end().expect("rust note_homing_end");
    py.exec("s.note_homing_end()").expect("py note_homing_end");
    compare_state("file output", &mut py, &world, &["pin_log"]);
    assert!(world.mcu.query_log.lock().is_empty());
}

#[test]
fn note_homing_end() {
    let Some(mut py) = oracle() else { return };
    let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]));
    let rust = rust_stepper(&world, &Spec::new()).expect("rust stepper");
    py_stepper(&mut py, "s", &Spec::new()).expect("python stepper");
    world.mcu.run_config_callbacks().expect("rust build_config");
    py.exec("for cb in list(mcu.config_callbacks): cb()").expect("py build_config");
    setup_itersolve_both(&mut py, "s", &*rust, &allocs()[0]);
    rust.set_position([2.5, 0.0, 0.0]);
    py.exec("s.set_position([2.5, 0.0, 0.0])").expect("py set_position");
    for (pos, receive_time) in [(5, 50.0), (-9, 60.25)] {
        world.mcu.queue_response(pos, receive_time);
        py.exec(&format!("mcu.query_responses.append({{'pos': {pos}, '#receive_time': {}}})", pyf(receive_time)))
            .expect("py queue reply");
        rust.note_homing_end().expect("rust note_homing_end");
        py.exec("s.note_homing_end()").expect("py note_homing_end");
        check("mcu pos", py.eval("s.get_mcu_position()"), Value::int(rust.get_mcu_position(None)));
        compare_state(&format!("note_homing_end {pos}"), &mut py, &world, &["pin_log"]);
    }
}

#[test]
fn active_callbacks() {
    let Some(mut py) = oracle() else { return };
    // (stepper axis, moves as (print_time, move_t, axes_r), flush windows)
    let scenarios: Vec<ActiveScenario> = vec![
        ("no moves", vec![], vec![(10.0, 10.0)]),
        ("x active", vec![(0.5, 2.0, [1.0, 0.0, 0.0])], vec![(0.1, 0.1), (3.0, 3.0), (0.5, 0.5)]),
        ("only y moves", vec![(0.5, 2.0, [0.0, 1.0, 0.0])], vec![(0.1, 0.1), (10.0, 10.0)]),
        (
            "y then x",
            vec![(1.0, 1.0, [0.0, 1.0, 0.0]), (2.0, 1.0, [1.0, 0.0, 0.0])],
            vec![(0.5, 0.5), (1.5, 1.5), (2.0, 2.0), (2.5, 2.5), (10.0, 10.0)],
        ),
        (
            "gap before x",
            vec![(0.1, 1.0, [0.0, 1.0, 0.0]), (5.0, 1.0, [1.0, 0.0, 0.0])],
            vec![(2.0, 2.0), (4.9, 4.9), (6.0, 6.0)],
        ),
    ];
    for (name, moves, windows) in scenarios {
        for (i, (must_flush, max_step_gen)) in windows.iter().enumerate() {
            let label = format!("{name} window {i}");
            let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]));
            let rust = rust_stepper(&world, &Spec::new()).expect("rust stepper");
            py_stepper(&mut py, "s", &Spec::new()).expect("python stepper");
            setup_itersolve_both(&mut py, "s", &*rust, &allocs()[0]);
            let tq = TrapqHandle::alloc();
            py.exec("tq = FakeTrapq()").expect("py tq");
            for (print_time, move_t, axes_r) in &moves {
                add_move(&tq, *print_time, *move_t, *axes_r);
                py_add_move(&mut py, "tq", *print_time, *move_t, *axes_r);
            }
            rust.set_trapq(Some(tq));
            py.exec("s.set_trapq(tq)").expect("py set_trapq");

            let got = Arc::new(Mutex::new(Vec::new()));
            for tag in 0..2i64 {
                let got = got.clone();
                rust.add_active_callback(Box::new(move |ret| { got.lock().push((tag, ret)); Ok(()) }))
                    .expect("add_active_callback");
            }
            py.exec(
                "got = []\n\
                 s.add_active_callback(lambda ret: got.append((0, ret)))\n\
                 s.add_active_callback(lambda ret: got.append((1, ret)))",
            )
            .expect("py add_active_callback");
            compare_state(&format!("{label}: registered"), &mut py, &world, &["pin_log"]);

            world.mq.flush(*must_flush, *max_step_gen);
            py.exec(&format!(
                "for cb in list(mq.flush_callbacks): cb({}, {})",
                pyf(*must_flush),
                pyf(*max_step_gen)
            ))
            .expect("py flush");
            let fired = Value::List(
                got.lock().iter().map(|(tag, ret)| Value::Tuple(vec![Value::int(*tag), vf(*ret)])).collect(),
            );
            check(&format!("{label}: callbacks"), py.eval("got"), fired);
            compare_state(&format!("{label}: after flush"), &mut py, &world, &["pin_log"]);

            // Callbacks are consumed; a second registration installs the flush callback again.
            world.mq.flush(*must_flush, *max_step_gen);
            py.exec(&format!(
                "for cb in list(mq.flush_callbacks): cb({}, {})",
                pyf(*must_flush),
                pyf(*max_step_gen)
            ))
            .expect("py second flush");
            rust.add_active_callback(Box::new(|_| Ok(()))).expect("re-register");
            py.exec("s.add_active_callback(lambda ret: None)").expect("py re-register");
            compare_state(&format!("{label}: re-registered"), &mut py, &world, &["pin_log"]);
        }
    }
}

/// `_check_active` calls the callbacks in order; the first one raising
/// escapes the flush callback and the rest never run.
#[test]
fn active_callback_error_propagates() {
    let Some(mut py) = oracle() else { return };
    let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &[]));
    let rust = rust_stepper(&world, &Spec::new()).expect("rust stepper");
    py_stepper(&mut py, "s", &Spec::new()).expect("python stepper");
    setup_itersolve_both(&mut py, "s", &*rust, &allocs()[0]);
    let tq = TrapqHandle::alloc();
    py.exec("tq = FakeTrapq()").expect("py tq");
    add_move(&tq, 0.5, 2.0, [1.0, 0.0, 0.0]);
    py_add_move(&mut py, "tq", 0.5, 2.0, [1.0, 0.0, 0.0]);
    rust.set_trapq(Some(tq));
    py.exec("s.set_trapq(tq)").expect("py set_trapq");
    let got = Arc::new(Mutex::new(Vec::new()));
    rust.add_active_callback(Box::new(|ret| {
        Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: format!("active {}", pycompat::text::float_repr(ret)),
        })
    }))
    .expect("add raising callback");
    let got2 = got.clone();
    rust.add_active_callback(Box::new(move |ret| { got2.lock().push(ret); Ok(()) }))
        .expect("add recording callback");
    py.exec(
        "got = []\n\
         def bad(ret):\n    raise ValueError('active %s' % (ret,))\n\
         s.add_active_callback(bad)\n\
         s.add_active_callback(lambda ret: got.append(ret))",
    )
    .expect("py add_active_callback");
    let expected = py
        .exec("for cb in list(mq.flush_callbacks): cb(3.0, 3.0)")
        .expect_err("py flush raises");
    let callbacks: Vec<FlushCallback> =
        world.mq.flush_callbacks.lock().iter().map(|(cb, _)| cb.clone()).collect();
    let actual = callbacks
        .iter()
        .try_for_each(|cb| cb(3.0, 3.0))
        .expect_err("rust flush raises");
    assert_eq!(actual.class_name(), expected.class_name, "class");
    assert_eq!(actual.message(), expected.message, "message");
    check("later callbacks skipped", py.eval("got"), Value::List(got.lock().iter().map(|r| vf(*r)).collect()));
    compare_state("after raising callback", &mut py, &world, &["pin_log"]);
}

// ---------------------------------------------------------------------------
// Tests: parse_step_distance and PrinterStepper.
// ---------------------------------------------------------------------------

#[test]
fn parse_step_distance_cases() {
    let Some(mut py) = oracle() else { return };
    let base: Vec<(&str, &str)> = vec![("rotation_distance", "40"), ("microsteps", "16")];
    let with = |extra: &[(&'static str, &'static str)]| {
        let mut v = base.clone();
        v.extend_from_slice(extra);
        v
    };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("plain", base.clone()),
        ("gear ratio", with(&[("gear_ratio", "80:20")])),
        ("two stage gear ratio", with(&[("gear_ratio", "80:20, 3:1")])),
        ("comma gear ratio", with(&[("gear_ratio", "80:20,5:2"), ("full_steps_per_rotation", "400")])),
        ("empty gear ratio", with(&[("gear_ratio", "")])),
        ("full steps 200", with(&[("full_steps_per_rotation", "200")])),
        ("full steps invalid", with(&[("full_steps_per_rotation", "201")])),
        ("full steps zero", with(&[("full_steps_per_rotation", "0")])),
        ("full steps text", with(&[("full_steps_per_rotation", "many")])),
        ("gear ratio zero divisor", with(&[("gear_ratio", "1:0")])),
        ("gear ratio odd", with(&[("gear_ratio", "2")])),
        ("gear ratio three", with(&[("gear_ratio", "1:2:3")])),
        ("gear ratio text", with(&[("gear_ratio", "a:1")])),
        ("no rotation distance", vec![("microsteps", "16")]),
        ("rotation distance zero", vec![("rotation_distance", "0"), ("microsteps", "16")]),
        ("rotation distance negative", vec![("rotation_distance", "-4"), ("microsteps", "16")]),
        ("rotation distance text", vec![("rotation_distance", "far"), ("microsteps", "16")]),
        ("no microsteps", vec![("rotation_distance", "40")]),
        ("microsteps zero", vec![("rotation_distance", "40"), ("microsteps", "0")]),
        ("microsteps text", vec![("rotation_distance", "40"), ("microsteps", "16.5")]),
        ("radians", vec![("gear_ratio", "50:1"), ("microsteps", "32")]),
        ("radians rotation too", vec![("rotation_distance", "40"), ("gear_ratio", "50:1"), ("microsteps", "32")]),
    ];
    for (name, options) in cases {
        for units in [None, Some(false), Some(true)] {
            for note_valid in [false, true] {
                let label = format!("{name} units {units:?} note {note_valid}");
                let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &options));
                let rust = parse_step_distance(&world.config("stepper_x"), units, note_valid);
                let units_py = units.map_or("None", pyb);
                let expected = py.eval(&format!(
                    "stepper.parse_step_distance(cfg('stepper_x'), {units_py}, {})",
                    pyb(note_valid)
                ));
                match (expected, rust) {
                    (Ok(e), Ok((rd, spr))) => {
                        check(&label, Ok(e), Value::Tuple(vec![vf(rd), vf(spr)]));
                    }
                    (Err(e), Err(a)) => same_error(&label, &e, &a),
                    (Ok(e), Err(a)) => panic!("{label}: python returned {e:?}, rust failed with {a}"),
                    (Err(e), Ok(v)) => {
                        panic!("{label}: python raised {}: {}, rust returned {v:?}", e.class_name, e.message)
                    }
                }
                compare_state(&label, &mut py, &world, &["pin_log"]);
            }
        }
    }
}

#[test]
fn printer_stepper_wiring() {
    let Some(mut py) = oracle() else { return };
    let full: Vec<(&str, &str)> = vec![
        ("step_pin", "PB0"),
        ("dir_pin", "!PB1"),
        ("rotation_distance", "40"),
        ("microsteps", "16"),
        ("step_pulse_duration", "0.000002"),
    ];
    let without = |drop: &str| -> Vec<(&str, &str)> {
        full.iter().filter(|(k, _)| *k != drop).cloned().collect()
    };
    let cases: Vec<(&str, CaseOptions, bool)> = vec![
        ("full", full.clone(), false),
        ("no pulse", without("step_pulse_duration"), false),
        ("radians", vec![("step_pin", "!PB0"), ("dir_pin", "PB1"), ("gear_ratio", "80:16"), ("microsteps", "16")], true),
        ("radians without gear ratio", without("rotation_distance"), true),
        ("no step_pin", without("step_pin"), false),
        ("no dir_pin", without("dir_pin"), false),
        ("no rotation distance", without("rotation_distance"), false),
        ("no microsteps", without("microsteps"), false),
        (
            "pulse too long",
            full.iter().map(|(k, v)| (*k, if *k == "step_pulse_duration" { "0.002" } else { *v })).collect(),
            false,
        ),
        (
            "pulse negative",
            full.iter().map(|(k, v)| (*k, if *k == "step_pulse_duration" { "-0.1" } else { *v })).collect(),
            false,
        ),
        (
            "pulse max",
            full.iter().map(|(k, v)| (*k, if *k == "step_pulse_duration" { "0.001" } else { *v })).collect(),
            false,
        ),
        ("pins with chip", vec![("step_pin", "mcu:PB0"), ("dir_pin", "mcu:!PB1"), ("rotation_distance", "8"), ("microsteps", "64"), ("full_steps_per_rotation", "400")], false),
    ];
    for (name, options, units) in cases {
        let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &options));
        let rust = printer_stepper(&world.config("stepper_x"), units);
        let expected = py.exec(&format!("ps = stepper.PrinterStepper(cfg('stepper_x'), {})", pyb(units)));
        match (expected, &rust) {
            (Ok(_), Ok(stepper)) => stepper_summary(name, &mut py, "ps", &**stepper),
            (Err(e), Err(a)) => same_error(name, &e, a),
            (Ok(_), Err(a)) => panic!("{name}: python succeeded, rust failed with {a}"),
            (Err(e), Ok(_)) => panic!("{name}: python raised {}: {}", e.class_name, e.message),
        }
        compare_state(name, &mut py, &world, &[]);
    }
    // A missing registrar object stops the registration loop.
    for missing in REGISTRARS {
        let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &full));
        world.remove_object(missing);
        py.exec(&format!("del printer.objects[{missing:?}]")).expect("remove python object");
        let rust = printer_stepper(&world.config("stepper_x"), false);
        let expected = py.exec("ps = stepper.PrinterStepper(cfg('stepper_x'))");
        let ok = compare_unit(&format!("missing {missing}"), expected, &rust.as_ref().map(|_| ()).map_err(Clone::clone));
        assert!(!ok);
        compare_state(&format!("missing {missing}"), &mut py, &world, &[]);
    }
}

// ---------------------------------------------------------------------------
// Tests: GenericPrinterRail, LookupRail, LookupMultiRail.
// ---------------------------------------------------------------------------

fn rail_summary(label: &str, py: &mut Oracle, var: &str, rail: &dyn RailApi, world: &World) {
    let (lo, hi) = rail.get_range();
    let info = rail.get_homing_info();
    let endstops = rail.get_endstops();
    let rows: Vec<(String, String, Value)> = vec![
        (format!("{label}: range"), format!("{var}.get_range()"), Value::Tuple(vec![vf(lo), vf(hi)])),
        (
            format!("{label}: homing info"),
            format!("tuple({var}.get_homing_info())"),
            Value::Tuple(vec![
                vf(info.speed),
                vf(info.position_endstop),
                vf(info.retract_speed),
                vf(info.retract_dist),
                Value::Bool(info.positive_dir),
                vf(info.second_homing_speed),
            ]),
        ),
        (format!("{label}: name"), format!("{var}.get_name()"), vs(&rail.get_name(None))),
        (format!("{label}: name short"), format!("{var}.get_name(short=True)"), vs(&rail.get_name(Some(true)))),
        (format!("{label}: name full"), format!("{var}.get_name(short=False)"), vs(&rail.get_name(Some(false)))),
        (
            format!("{label}: endstops"),
            format!("[(e.pin, n) for e, n in {var}.get_endstops()]"),
            Value::List(
                endstops
                    .iter()
                    .map(|(e, n)| Value::Tuple(vec![vs(&world.pins.desc_of(e)), vs(n)]))
                    .collect(),
            ),
        ),
        (
            format!("{label}: endstop steppers"),
            format!("[[s.get_name() for s in e.get_steppers()] for e, n in {var}.get_endstops()]"),
            Value::List(
                endstops
                    .iter()
                    .map(|(e, _)| Value::List(e.get_steppers().iter().map(|s| vs(&s.get_name(None))).collect()))
                    .collect(),
            ),
        ),
        (
            format!("{label}: steppers"),
            format!("[s.get_name() for s in {var}.get_steppers()]"),
            Value::List(rail.get_steppers().iter().map(|s| vs(&s.get_name(None))).collect()),
        ),
    ];
    for (label, expr, actual) in rows {
        check(&label, py.eval(&expr), actual);
    }
}

struct RailCase {
    name: &'static str,
    options: Vec<(&'static str, &'static str)>,
    opts: RailOpts,
    probe: Option<f64>,
    drop_query_endstops: bool,
}

fn rail_case(name: &'static str, options: &[(&'static str, &'static str)]) -> RailCase {
    RailCase {
        name,
        options: options.to_vec(),
        opts: RailOpts::default(),
        probe: None,
        drop_query_endstops: false,
    }
}

fn rail_options(overrides: &[(&'static str, &'static str)], drop: &[&str]) -> Vec<(&'static str, &'static str)> {
    let mut options: Vec<(&'static str, &'static str)> = vec![
        ("endstop_pin", "PC0"),
        ("position_endstop", "0"),
        ("position_min", "0"),
        ("position_max", "235"),
        ("homing_speed", "50"),
        ("homing_positive_dir", "false"),
    ];
    options.retain(|(k, _)| !drop.contains(k) && !overrides.iter().any(|(ok, _)| ok == k));
    options.extend_from_slice(overrides);
    options
}

fn py_rail_ctor(opts: &RailOpts) -> String {
    format!(
        "stepper.GenericPrinterRail(cfg('stepper_x'), {}, {}, {})",
        pyb(opts.need_position_minmax),
        py_opt_f(opts.default_position_endstop),
        pyb(opts.units_in_radians)
    )
}

#[test]
fn rail_construction() {
    let Some(mut py) = oracle() else { return };
    let mut no_minmax = rail_case("no minmax", &rail_options(&[("position_endstop", "100")], &["position_min", "position_max", "homing_positive_dir"]));
    no_minmax.opts.need_position_minmax = false;
    let mut no_minmax_zero = rail_case("no minmax endstop zero", &rail_options(&[], &["position_min", "position_max", "homing_positive_dir"]));
    no_minmax_zero.opts.need_position_minmax = false;
    let mut default_endstop = rail_case("default endstop", &rail_options(&[], &["position_endstop", "homing_positive_dir"]));
    default_endstop.opts.default_position_endstop = Some(50.0);
    let mut default_ignored = rail_case("default ignored", &rail_options(&[("position_endstop", "10")], &["homing_positive_dir"]));
    default_ignored.opts.default_position_endstop = Some(50.0);
    let mut probe = rail_case("probe endstop", &rail_options(&[], &["position_endstop", "homing_positive_dir"]));
    probe.probe = Some(7.5);
    let mut probe_default = rail_case("probe endstop with default", &rail_options(&[], &["position_endstop"]));
    probe_default.probe = Some(235.0);
    probe_default.opts.default_position_endstop = Some(1.0);
    let mut no_qes = rail_case("no query_endstops", &rail_options(&[], &[]));
    no_qes.drop_query_endstops = true;
    let mut no_qes_no_pin = rail_case("no query_endstops and no endstop_pin", &rail_options(&[], &["endstop_pin"]));
    no_qes_no_pin.drop_query_endstops = true;
    let mut radians = rail_case("radians", &rail_options(&[], &[]));
    radians.opts.units_in_radians = true;

    let cases = vec![
        rail_case("explicit false", &rail_options(&[], &[])),
        rail_case("explicit true", &rail_options(&[("position_endstop", "235"), ("homing_positive_dir", "true")], &[])),
        rail_case("infer low", &rail_options(&[], &["homing_positive_dir"])),
        rail_case("infer high", &rail_options(&[("position_endstop", "235")], &["homing_positive_dir"])),
        rail_case("infer boundary low", &rail_options(&[("position_endstop", "58.75")], &["homing_positive_dir"])),
        rail_case("infer boundary high", &rail_options(&[("position_endstop", "176.25")], &["homing_positive_dir"])),
        rail_case("infer middle", &rail_options(&[("position_endstop", "117")], &["homing_positive_dir"])),
        rail_case("all options", &rail_options(&[("endstop_pin", "^!PC1"), ("homing_speed", "40"), ("second_homing_speed", "10"), ("homing_retract_speed", "20"), ("homing_retract_dist", "3")], &[])),
        rail_case("derived speeds", &rail_options(&[("homing_speed", "7")], &[])),
        rail_case("chip prefix", &rail_options(&[("endstop_pin", "mcu:PC2")], &[])),
        rail_case("true with endstop at min", &rail_options(&[("homing_positive_dir", "true")], &[])),
        rail_case("false with endstop at max", &rail_options(&[("position_endstop", "235")], &[])),
        rail_case("endstop above max", &rail_options(&[("position_endstop", "300")], &["homing_positive_dir"])),
        rail_case("endstop below min", &rail_options(&[("position_endstop", "-1")], &["homing_positive_dir"])),
        rail_case("no endstop_pin", &rail_options(&[], &["endstop_pin"])),
        rail_case("no position_endstop", &rail_options(&[], &["position_endstop"])),
        rail_case("no position_max", &rail_options(&[], &["position_max"])),
        rail_case("max equals min", &rail_options(&[("position_min", "10"), ("position_max", "10"), ("position_endstop", "10")], &[])),
        rail_case("max below min", &rail_options(&[("position_min", "10"), ("position_max", "5")], &[])),
        rail_case("homing_speed zero", &rail_options(&[("homing_speed", "0")], &[])),
        rail_case("second_homing_speed zero", &rail_options(&[("second_homing_speed", "0")], &[])),
        rail_case("retract speed negative", &rail_options(&[("homing_retract_speed", "-1")], &[])),
        rail_case("retract dist negative", &rail_options(&[("homing_retract_dist", "-1")], &[])),
        rail_case("retract dist zero", &rail_options(&[("homing_retract_dist", "0")], &[])),
        rail_case("position_min text", &rail_options(&[("position_min", "abc")], &[])),
        rail_case("homing_positive_dir text", &rail_options(&[("homing_positive_dir", "maybe")], &[])),
        rail_case("homing_positive_dir yes", &rail_options(&[("position_endstop", "235"), ("homing_positive_dir", "Yes")], &[])),
        no_minmax,
        no_minmax_zero,
        default_endstop,
        default_ignored,
        probe,
        probe_default,
        no_qes,
        no_qes_no_pin,
        radians,
    ];
    for case in cases {
        let mut cfg = WorldCfg::default().section("stepper_x", &case.options);
        if let Some(p) = case.probe {
            cfg = cfg.probe(p);
        }
        let world = new_world(&mut py, &cfg);
        if case.drop_query_endstops {
            world.remove_object("query_endstops");
            py.exec("del printer.objects['query_endstops']").expect("remove python object");
        }
        let rust = GenericPrinterRail::new(&world.config("stepper_x"), case.opts);
        let expected = py.exec(&format!("rail = {}", py_rail_ctor(&case.opts)));
        match (expected, &rust) {
            (Ok(_), Ok(rail)) => rail_summary(case.name, &mut py, "rail", &**rail, &world),
            (Err(e), Err(a)) => same_error(case.name, &e, a),
            (Ok(_), Err(a)) => panic!("{}: python succeeded, rust failed with {a}", case.name),
            (Err(e), Ok(_)) => panic!("{}: python raised {}: {}", case.name, e.class_name, e.message),
        }
        compare_state(case.name, &mut py, &world, &[]);
    }
}

#[test]
fn rail_lookup_endstop() {
    let Some(mut py) = oracle() else { return };
    let world = new_world(&mut py, &WorldCfg::default().section("stepper_x", &rail_options(&[], &[])));
    let rust = GenericPrinterRail::new(&world.config("stepper_x"), RailOpts::default()).expect("rust rail");
    py.exec(&format!("rail = {}", py_rail_ctor(&RailOpts::default()))).expect("python rail");
    let lookups: [(Option<&str>, &str); 12] = [
        (Some("PC0"), "other"),
        (Some("mcu:PC0"), "prefixed"),
        (Some(" PC0 "), "padded"),
        (Some("!PC0"), "inverted"),
        (Some("^PC0"), "pullup"),
        (Some("~PC0"), "pulldown"),
        (Some("PC1"), "second"),
        (Some("PC1"), "second again"),
        (Some("!PC1"), "second inverted"),
        (Some("^!PC2"), "third"),
        (Some("^!PC2"), "third again"),
        (None, "none"),
    ];
    for (i, (pin, name)) in lookups.into_iter().enumerate() {
        let label = format!("lookup {i} {pin:?}");
        let py_pin = pin.map_or("None".to_string(), |p| format!("{p:?}"));
        let expected = py.exec(&format!("found = rail.lookup_endstop({py_pin}, {name:?})"));
        let result = rust.lookup_endstop(pin, name);
        match (expected, &result) {
            (Ok(_), Ok(found)) => {
                let index = rust
                    .get_endstops()
                    .iter()
                    .position(|(e, _)| data_ptr(e) == data_ptr(found))
                    .expect("endstop registered") as i64;
                check(
                    &label,
                    py.eval("[i for i, (e, n) in enumerate(rail.get_endstops()) if e is found][0]"),
                    Value::int(index),
                );
            }
            (Err(e), Err(a)) => same_error(&label, &e, a),
            (Ok(_), Err(a)) => panic!("{label}: python succeeded, rust failed with {a}"),
            (Err(e), Ok(_)) => panic!("{label}: python raised {}: {}", e.class_name, e.message),
        }
        rail_summary(&label, &mut py, "rail", &*rust, &world);
        compare_state(&label, &mut py, &world, &[]);
    }
}

fn rail_steppers_config() -> WorldCfg {
    let wiring = |step: &'static str, dir: &'static str| {
        vec![("step_pin", step), ("dir_pin", dir), ("rotation_distance", "40"), ("microsteps", "16")]
    };
    let mut x = wiring("PB0", "!PB1");
    x.extend(rail_options(&[("endstop_pin", "PC0")], &[]));
    let mut x1 = wiring("PB2", "PB3");
    x1.extend_from_slice(&[("endstop_pin", "^PC1"), ("full_steps_per_rotation", "400")]);
    let x2 = wiring("PB4", "PB5");
    WorldCfg::default()
        .section("stepper_x", &x)
        .section("stepper_x1", &x1)
        .section("stepper_x2", &x2)
        .section("stepper_x4", &wiring("PB6", "PB7"))
        .section("stepper_y", &wiring("PD0", "PD1"))
}

#[test]
fn lookup_rail_and_multi_rail() {
    let Some(mut py) = oracle() else { return };
    for multi in [false, true] {
        for with_x1 in [true, false] {
            let name = format!("multi {multi} x1 {with_x1}");
            let mut cfg = rail_steppers_config();
            if !with_x1 {
                cfg.sections.retain(|(n, _)| n.as_str() != "stepper_x1");
            }
            let world = new_world(&mut py, &cfg);
            let (rust_fn, py_fn) = if multi {
                (klippy_core::stepper::lookup_multi_rail as fn(&Config, RailOpts) -> KResult<klippy_api::stepper::Rail>, "LookupMultiRail")
            } else {
                (klippy_core::stepper::lookup_rail as fn(&Config, RailOpts) -> KResult<klippy_api::stepper::Rail>, "LookupRail")
            };
            let rail = rust_fn(&world.config("stepper_x"), RailOpts::default()).expect("rust rail");
            py.exec(&format!("rail = stepper.{py_fn}(cfg('stepper_x'), True, None, False)")).expect("python rail");
            rail_summary(&name, &mut py, "rail", &*rail, &world);
            compare_state(&name, &mut py, &world, &[]);
            let count = rail.get_steppers().len();
            assert_eq!(count, if multi && with_x1 { 3 } else { 1 }, "{name}: stepper count");

            // Delegation to every stepper, and to the first one for the position helpers.
            let all_allocs = allocs();
            let alloc = &all_allocs[3];
            rail.setup_itersolve(alloc.func, &alloc.params);
            py.exec(&format!("rail.setup_itersolve({:?}, {})", alloc.func, alloc.py_args)).expect("py rail setup_itersolve");
            rail.set_position([12.5, 3.25, 0.0]);
            py.exec("rail.set_position([12.5, 3.25, 0.0])").expect("py rail set_position");
            check(
                &format!("{name}: stepper positions"),
                py.eval("[(s.get_commanded_position(), s.get_mcu_position()) for s in rail.get_steppers()]"),
                Value::List(
                    rail.get_steppers()
                        .iter()
                        .map(|s| Value::Tuple(vec![vf(s.get_commanded_position()), Value::int(s.get_mcu_position(None))]))
                        .collect(),
                ),
            );
            check(&format!("{name}: rail commanded"), py.eval("rail.get_commanded_position()"), vf(rail.get_commanded_position()));
            check(
                &format!("{name}: rail calc"),
                py.eval("rail.calc_position_from_coord([1.0, 2.0, 3.0])"),
                vf(rail.calc_position_from_coord([1.0, 2.0, 3.0])),
            );
            let tq = TrapqHandle::alloc();
            py.exec("tq = FakeTrapq()").expect("py tq");
            rail.set_trapq(Some(tq.clone()));
            py.exec("rail.set_trapq(tq)").expect("py rail set_trapq");
            check(
                &format!("{name}: trapq on every stepper"),
                py.eval("[s.get_trapq() is tq for s in rail.get_steppers()]"),
                Value::List(
                    rail.get_steppers()
                        .iter()
                        .map(|s| Value::Bool(s.get_trapq().is_some_and(|t| same(&t, &tq))))
                        .collect(),
                ),
            );
            rail.set_trapq(None);
            py.exec("rail.set_trapq(None)").expect("py rail set_trapq None");
            check(
                &format!("{name}: trapq cleared"),
                py.eval("[s.get_trapq() is None for s in rail.get_steppers()]"),
                Value::List(rail.get_steppers().iter().map(|s| Value::Bool(s.get_trapq().is_none())).collect()),
            );
        }
    }
}

#[test]
fn rail_add_stepper_variants() {
    let Some(mut py) = oracle() else { return };
    // (endstop_pin, endstop_name) for `add_stepper`.
    let variants: [(Option<&str>, Option<&str>); 6] = [
        (None, None),
        (None, Some("ignored")),
        (Some("PC3"), Some("custom")),
        (Some("PC4"), Some("")),
        (Some("PC4"), None),
        (Some("!PC0"), None),
    ];
    for (i, (pin, name)) in variants.into_iter().enumerate() {
        let label = format!("variant {i} {pin:?} {name:?}");
        let world = new_world(&mut py, &rail_steppers_config());
        let rail = GenericPrinterRail::new(&world.config("stepper_x"), RailOpts::default()).expect("rust rail");
        py.exec(&format!("rail = {}", py_rail_ctor(&RailOpts::default()))).expect("python rail");
        let stepper = printer_stepper(&world.config("stepper_y"), false).expect("rust stepper");
        py.exec("st = stepper.PrinterStepper(cfg('stepper_y'))").expect("python stepper");
        let result = rail.add_stepper(stepper, pin, name);
        let py_pin = pin.map_or("None".to_string(), |p| format!("{p:?}"));
        let py_name = name.map_or("None".to_string(), |n| format!("{n:?}"));
        let expected = py.exec(&format!("rail.add_stepper(st, {py_pin}, {py_name})"));
        compare_unit(&label, expected, &result);
        rail_summary(&label, &mut py, "rail", &*rail, &world);
        compare_state(&label, &mut py, &world, &[]);
    }

    // `add_stepper_from_config` uses the rail's units and the section's own endstop_pin.
    for units in [false, true] {
        let mut cfg = rail_steppers_config();
        for (name, options) in cfg.sections.iter_mut() {
            if name.as_str() == "stepper_x" {
                options.retain(|(k, _)| k != "rotation_distance");
                options.push(("gear_ratio".to_string(), "50:1".to_string()));
            }
        }
        let label = format!("from config units {units}");
        let world = new_world(&mut py, &cfg);
        let opts = RailOpts { units_in_radians: units, ..RailOpts::default() };
        let rail = klippy_core::stepper::lookup_rail(&world.config("stepper_x"), opts);
        let expected = py.exec(&format!(
            "rail = stepper.LookupRail(cfg('stepper_x'), True, None, {})",
            pyb(units)
        ));
        let ok = compare_unit(&label, expected, &rail.as_ref().map(|_| ()).map_err(Clone::clone));
        if let Ok(rail) = &rail {
            assert!(ok);
            rail_summary(&label, &mut py, "rail", &**rail, &world);
            check(
                &format!("{label}: units"),
                py.eval("[s.units_in_radians() for s in rail.get_steppers()]"),
                Value::List(rail.get_steppers().iter().map(|s| Value::Bool(s.units_in_radians())).collect()),
            );
        }
        compare_state(&label, &mut py, &world, &[]);
    }
}

#[test]
fn factory_delegates_to_module_functions() {
    let Some(mut py) = oracle() else { return };
    let factory = StepperFactory;
    let world = new_world(&mut py, &rail_steppers_config());

    let stepper = factory.printer_stepper(&world.config("stepper_y"), false).expect("printer_stepper");
    py.exec("st = stepper.PrinterStepper(cfg('stepper_y'), False)").expect("py printer_stepper");
    stepper_summary("factory printer_stepper", &mut py, "st", &*stepper);

    let (rd, spr) = factory
        .parse_step_distance(&world.config("stepper_y"), None, true)
        .expect("parse_step_distance");
    check(
        "factory parse_step_distance",
        py.eval("stepper.parse_step_distance(cfg('stepper_y'), None, True)"),
        Value::Tuple(vec![vf(rd), vf(spr)]),
    );

    let rail = factory.generic_rail(&world.config("stepper_x"), RailOpts::default()).expect("generic_rail");
    py.exec(&format!("rail = {}", py_rail_ctor(&RailOpts::default()))).expect("py generic rail");
    rail_summary("factory generic_rail", &mut py, "rail", &*rail, &world);

    let rail = factory.lookup_rail(&world.config("stepper_x"), RailOpts::default()).expect("lookup_rail");
    py.exec("rail = stepper.LookupRail(cfg('stepper_x'))").expect("py lookup_rail");
    rail_summary("factory lookup_rail", &mut py, "rail", &*rail, &world);

    let rail = factory.lookup_multi_rail(&world.config("stepper_x"), RailOpts::default()).expect("lookup_multi_rail");
    py.exec("rail = stepper.LookupMultiRail(cfg('stepper_x'))").expect("py lookup_multi_rail");
    rail_summary("factory lookup_multi_rail", &mut py, "rail", &*rail, &world);
    compare_state("factory", &mut py, &world, &[]);
}

// ---------------------------------------------------------------------------
// The fake chelper against the real C library.
// ---------------------------------------------------------------------------

#[repr(C)]
struct ListHead {
    next: *mut ListHead,
    prev: *mut ListHead,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct PullHistorySteps {
    first_clock: u64,
    last_clock: u64,
    start_position: i64,
    step_count: i32,
    interval: i32,
    add: i32,
}

unsafe extern "C" {
    fn free(ptr: *mut c_void);
    fn cartesian_stepper_alloc(axis: i8) -> *mut c_void;
    fn corexy_stepper_alloc(kind: i8) -> *mut c_void;
    fn corexz_stepper_alloc(kind: i8) -> *mut c_void;
    fn generic_cartesian_stepper_alloc(a_x: f64, a_y: f64, a_z: f64) -> *mut c_void;
    fn itersolve_calc_position_from_coord(sk: *mut c_void, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_set_position(sk: *mut c_void, x: f64, y: f64, z: f64);
    fn itersolve_get_commanded_pos(sk: *mut c_void) -> f64;
    fn itersolve_is_active_axis(sk: *mut c_void, axis: i8) -> i32;
    fn itersolve_set_trapq(sk: *mut c_void, tq: *mut c_void, step_dist: f64);
    fn itersolve_check_active(sk: *mut c_void, flush_time: f64) -> f64;
    fn trapq_alloc() -> *mut c_void;
    fn trapq_free(tq: *mut c_void);
    #[allow(clippy::too_many_arguments)]
    fn trapq_append(
        tq: *mut c_void,
        print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos_x: f64,
        start_pos_y: f64,
        start_pos_z: f64,
        axes_r_x: f64,
        axes_r_y: f64,
        axes_r_z: f64,
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    );
    fn stepcompress_alloc(msg_queue: *mut ListHead) -> *mut c_void;
    fn stepcompress_free(sc: *mut c_void);
    fn stepcompress_set_last_position(sc: *mut c_void, clock: u64, last_position: i64) -> i32;
    fn stepcompress_find_past_position(sc: *mut c_void, clock: u64) -> i64;
    fn stepcompress_extract_old(
        sc: *mut c_void,
        p: *mut PullHistorySteps,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> i32;
}

#[test]
fn fake_chelper_matches_c() {
    chelper_ref::link();
    let Some(mut py) = oracle() else { return };

    // itersolve: position helpers and active axes of every allocator the tests use.
    let coords: [[f64; 3]; 4] =
        [[0.0, 0.0, 0.0], [10.25, 3.5, -2.0], [100.0, 200.0, 50.0], [-1.0, 2.0, 3.0]];
    let makers: Vec<(&str, &str, Maker)> = vec![
        ("cartesian x", "_flib.cartesian_stepper_alloc(b'x')", Box::new(|| unsafe { cartesian_stepper_alloc(b'x' as i8) })),
        ("cartesian y", "_flib.cartesian_stepper_alloc(b'y')", Box::new(|| unsafe { cartesian_stepper_alloc(b'y' as i8) })),
        ("cartesian z", "_flib.cartesian_stepper_alloc(b'z')", Box::new(|| unsafe { cartesian_stepper_alloc(b'z' as i8) })),
        ("corexy +", "_flib.corexy_stepper_alloc(b'+')", Box::new(|| unsafe { corexy_stepper_alloc(b'+' as i8) })),
        ("corexy -", "_flib.corexy_stepper_alloc(b'-')", Box::new(|| unsafe { corexy_stepper_alloc(b'-' as i8) })),
        ("corexz +", "_flib.corexz_stepper_alloc(b'+')", Box::new(|| unsafe { corexz_stepper_alloc(b'+' as i8) })),
        ("corexz -", "_flib.corexz_stepper_alloc(b'-')", Box::new(|| unsafe { corexz_stepper_alloc(b'-' as i8) })),
        ("generic", "_flib.generic_cartesian_stepper_alloc(1.5, 0.0, -0.5)", Box::new(|| unsafe { generic_cartesian_stepper_alloc(1.5, 0.0, -0.5) })),
    ];
    for (name, py_alloc, make) in &makers {
        py.exec(&format!("fsk = {py_alloc}")).expect("fake alloc");
        let sk = make();
        for c in coords {
            let args = format!("fsk, {}, {}, {}", pyf(c[0]), pyf(c[1]), pyf(c[2]));
            check(
                &format!("{name} calc {c:?}"),
                py.eval(&format!("_flib.itersolve_calc_position_from_coord({args})")),
                vf(unsafe { itersolve_calc_position_from_coord(sk, c[0], c[1], c[2]) }),
            );
            py.exec(&format!("_flib.itersolve_set_position({args})")).expect("fake set_position");
            unsafe { itersolve_set_position(sk, c[0], c[1], c[2]) };
            check(
                &format!("{name} commanded {c:?}"),
                py.eval("_flib.itersolve_get_commanded_pos(fsk)"),
                vf(unsafe { itersolve_get_commanded_pos(sk) }),
            );
        }
        for axis in [b'x', b'y', b'z', b'e', b'w', b'{'] {
            check(
                &format!("{name} axis {}", axis as char),
                py.eval(&format!("_flib.itersolve_is_active_axis(fsk, bytes([{axis}]))")),
                Value::int(i64::from(unsafe { itersolve_is_active_axis(sk, axis as i8) })),
            );
        }
        unsafe { free(sk) };
    }

    // itersolve_check_active over a trapq, including the null moves that fill time gaps.
    let scenarios: Vec<(Moves, Vec<f64>)> = vec![
        (vec![], vec![0.0, 10.0]),
        (vec![(0.5, 2.0, [1.0, 0.0, 0.0])], vec![0.1, 0.5, 3.0]),
        (vec![(0.5, 2.0, [0.0, 1.0, 0.0])], vec![0.1, 0.5, 3.0, 10.0]),
        (vec![(2.0, 2.0, [0.0, 1.0, 0.0])], vec![0.1, 1.5, 3.0]),
        (
            vec![(1.0, 1.0, [0.0, 1.0, 0.0]), (2.0, 1.0, [1.0, 0.0, 0.0])],
            vec![0.5, 1.5, 2.0, 2.5, 10.0],
        ),
        (
            vec![(0.1, 1.0, [0.0, 1.0, 0.0]), (5.0, 1.0, [1.0, 0.0, 0.0]), (6.0, 0.5, [0.5, 0.5, 0.5])],
            vec![2.0, 4.9, 5.0, 6.5, 8.0],
        ),
    ];
    for (i, (moves, flush_times)) in scenarios.iter().enumerate() {
        py.exec("fsk = _flib.cartesian_stepper_alloc(b'x'); ftq = FakeTrapq()").expect("fake sk and trapq");
        let sk = unsafe { cartesian_stepper_alloc(b'x' as i8) };
        let tq = unsafe { trapq_alloc() };
        for (print_time, move_t, axes_r) in moves {
            py_add_move(&mut py, "ftq", *print_time, *move_t, *axes_r);
            unsafe {
                trapq_append(
                    tq, *print_time, 0.0, *move_t, 0.0, 0.0, 0.0, 0.0, axes_r[0], axes_r[1],
                    axes_r[2], 0.0, 1.0, 0.0,
                );
            }
        }
        py.exec("_flib.itersolve_set_trapq(fsk, ftq, 0.025)").expect("fake set_trapq");
        unsafe { itersolve_set_trapq(sk, tq, 0.025) };
        for flush_time in flush_times {
            check(
                &format!("check_active scenario {i} flush {flush_time}"),
                py.eval(&format!("_flib.itersolve_check_active(fsk, {})", pyf(*flush_time))),
                vf(unsafe { itersolve_check_active(sk, *flush_time) }),
            );
        }
        unsafe {
            free(sk);
            trapq_free(tq);
        }
    }

    // stepcompress: position markers, the only history `stepper.py` creates itself.
    let mut msg_queue = Box::new(ListHead { next: std::ptr::null_mut(), prev: std::ptr::null_mut() });
    let head: *mut ListHead = &mut *msg_queue;
    msg_queue.next = head;
    msg_queue.prev = head;
    let sc = unsafe { stepcompress_alloc(head) };
    py.exec("fsc = {'history': [], 'last_position': 0}").expect("fake stepcompress");
    for (clock, position) in [(1000u64, 5i64), (2000, -3), (1500, 7), (1500, 8)] {
        py.exec(&format!("_flib.stepcompress_set_last_position(fsc, {clock}, {position})"))
            .expect("fake set_last_position");
        assert_eq!(0, unsafe { stepcompress_set_last_position(sc, clock, position) });
    }
    let clocks: [u64; 9] = [0, 999, 1000, 1001, 1499, 1500, 1501, 2000, 5000];
    check(
        "find_past_position",
        py.eval(&format!(
            "[_flib.stepcompress_find_past_position(fsc, c) for c in {:?}]",
            clocks.to_vec()
        )),
        Value::List(
            clocks
                .iter()
                .map(|c| Value::int(unsafe { stepcompress_find_past_position(sc, *c) }))
                .collect(),
        ),
    );
    let windows: [(i32, u64, u64); 6] = [
        (10, 0, u64::MAX / 2),
        (2, 0, u64::MAX / 2),
        (0, 0, u64::MAX / 2),
        (10, 1000, 1500),
        (10, 1500, 1501),
        (10, 2000, 3000),
    ];
    for (max, start, end) in windows {
        let mut rows = vec![PullHistorySteps::default(); 10];
        let n = unsafe { stepcompress_extract_old(sc, rows.as_mut_ptr(), max, start, end) };
        let expected = Value::Tuple(vec![
            Value::int(i64::from(n)),
            Value::List(
                rows[..n as usize]
                    .iter()
                    .map(|r| {
                        Value::Tuple(vec![
                            Value::int(r.first_clock as i64),
                            Value::int(r.last_clock as i64),
                            Value::int(r.start_position),
                            Value::int(i64::from(r.interval)),
                            Value::int(i64::from(r.step_count)),
                            Value::int(i64::from(r.add)),
                        ])
                    })
                    .collect(),
            ),
        ]);
        check(
            &format!("extract_old {max} {start} {end}"),
            py.eval(&format!("extract(fsc, {max}, {start}, {end})")),
            expected,
        );
    }
    unsafe { stepcompress_free(sc) };
}
