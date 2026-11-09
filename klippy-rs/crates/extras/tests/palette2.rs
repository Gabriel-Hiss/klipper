//! Parity tests for `extras::palette2` (port of `klippy/extras/palette2.py`).
//!
//! The reference `Palette2` class runs in the live CPython 3.8.2 oracle (`pyoracle`) with Python
//! fakes for pyserial (`sys.modules['serial']`), the printer, config, G-code, toolhead, heater,
//! `virtual_sdcard` and `pause_resume`, and with the real `klippy/reactor.py` `PollReactor`.
//! The port runs through Rust fakes of the contract traits with the same inputs. Both sides run
//! the same list of [`Step`]s and, after every step, produce the step result (return value or
//! exception class and message) and a snapshot of everything observable:
//!
//! - serial connection, timers, `is_printing`/`is_loading`/`is_splicing`/`is_setup_complete`,
//!   `signal_disconnect`, the heartbeat (exact value in direct mode, set or unset in sessions),
//! - `get_status`, serial bytes written, `serial.Serial` arguments, `close()` count,
//! - `respond_info` messages, `run_script` scripts, `create_gcode_command` calls,
//!   `virtual_sdcard.get_file_list` arguments, `pause_resume` calls,
//! - every log record (the monotonic time in `... -> P2 : ...` lines is replaced by `T`).
//!
//! Steps that need the reactor (`PALETTE_CONNECT` waits on `reactor.pause`) run first, inside
//! `reactor.run()` on both sides; the reactor ends right after them and the remaining steps call
//! the handlers and timer callbacks directly, which keeps the timer interleaving out of the
//! comparison. Times that depend on the wall clock (`reactor.monotonic()`) compare as `T`.
//!
//! Skipped with a message when `$KLIPPY_PYTHON` is unset. Run with
//! `cargo test -p extras --test palette2`.

use std::collections::VecDeque;
use std::sync::{Arc, LazyLock, Once};

use extras::heaters::{ControlApi, HeaterApi};
use extras::palette2::{
    module_factory, open_posix_serial, Palette2, SerialException, SerialOpenError, SerialOpener,
    SerialPort,
};
use extras::pause_resume::PauseResumeApi;
use extras::virtual_sdcard::VirtualSDApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GCmdFloatOpts,
    GetIntOpts as GCmdIntOpts, GetOpts as GCmdGetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{StepperFactory, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, ExtraAxisApi, HomingTargetApi, Kinematics, LookaheadCallback, MoveApi,
    PrinterExtruder, PrinterExtruderApi, ToolHeadApi,
};
use klippy_api::{EventArg, KResult, KlippyError, ObjectRef, PrinterObject};
use kos::sync::Mutex;
use pycompat::text::{float_from_string, float_repr, int_from_string, str_repr};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex, NEVER, NOW};
use recording::{Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python side
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys
import types
import logging
import reactor as py_reactor


class Error(Exception):
    pass


class CommandError(Exception):
    pass


class _Sentinel:
    pass


sentinel = _Sentinel()


class SerialException(IOError):
    pass


SERIAL = {}


def reset_serial():
    SERIAL.clear()
    SERIAL.update({'open_fail': False, 'open_value_error': False, 'read_error': False,
                   'write_error': False, 'rx': [], 'written': [], 'opened': [], 'closed': 0})


reset_serial()


class FakeSerial:
    def __init__(self, port, baud, timeout=None, write_timeout=None):
        SERIAL['opened'].append((port, baud, timeout, write_timeout))
        if SERIAL['open_value_error']:
            raise ValueError('Not a valid baudrate: %r' % (baud,))
        if SERIAL['open_fail']:
            raise SerialException('open failed')

    def read(self, size=1):
        if SERIAL['read_error']:
            raise SerialException('read failed')
        if SERIAL['rx']:
            return SERIAL['rx'].pop(0)
        return b''

    def write(self, data):
        if SERIAL['write_error']:
            raise SerialException('write failed')
        SERIAL['written'].append(data)
        return len(data)

    def close(self):
        SERIAL['closed'] += 1


_serial_mod = types.ModuleType('serial')
_serial_mod.SerialException = SerialException
_serial_mod.Serial = FakeSerial
sys.modules['serial'] = _serial_mod

import extras.palette2 as palette2

LOG = []


class _Handler(logging.Handler):
    def emit(self, record):
        LOG.append((record.levelname, record.getMessage()))


logging.getLogger().setLevel(logging.DEBUG)
logging.getLogger().addHandler(_Handler())


def norm_log(msg):
    i = msg.find(' -> P2 : ')
    if i > 0:
        try:
            float(msg[:i])
        except ValueError:
            return msg
        return 'T' + msg[i:]
    return msg


class Ctx:
    def __init__(self):
        self.can_extrude = True
        self.heater_error = False
        self.busy = (0., 0., True)
        self.files = []
        self.files_error = False
        self.sd_calls = []
        self.pauses = 0
        self.resumes = 0
        self.pause_error = False
        self.loaded = []


CTX = Ctx()


class FakeGCmd:
    error = CommandError

    def __init__(self, gcode, command, commandline):
        self._gcode = gcode
        self._command = command
        self._commandline = commandline

    def get_command(self):
        return self._command

    def get_commandline(self):
        return self._commandline

    def respond_info(self, msg, log=True):
        self._gcode.respond_info(msg, log)


class FakeGCode:
    def __init__(self):
        self.registered = []
        self.handlers = {}
        self.infos = []
        self.scripts = []
        self.created = []
        self.run_error = None

    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registered.append([cmd, desc, when_not_ready])
        self.handlers[cmd] = func

    def respond_info(self, msg, log=True):
        self.infos.append([msg, log])

    def run_script(self, script):
        self.scripts.append(script)
        if self.run_error is not None:
            raise self.run_error

    def create_gcode_command(self, command, commandline, params):
        self.created.append([command, commandline, dict(params)])
        return FakeGCmd(self, command, commandline)


class FakeHeater:
    @property
    def can_extrude(self):
        return CTX.can_extrude


class FakeExtruder:
    def get_heater(self):
        if CTX.heater_error:
            raise CommandError('Extruder not configured')
        return FakeHeater()


class FakeToolHead:
    def get_extruder(self):
        return FakeExtruder()

    def check_busy(self, eventtime):
        return CTX.busy


class FakeVSD:
    def get_file_list(self, check_subdirs=False):
        CTX.sd_calls.append(check_subdirs)
        if CTX.files_error:
            raise CommandError('Unable to get file list')
        return list(CTX.files)


class FakePauseResume:
    def send_pause_command(self):
        CTX.pauses += 1
        if CTX.pause_error:
            raise CommandError('pause failed')

    def send_resume_command(self):
        CTX.resumes += 1
        if CTX.pause_error:
            raise CommandError('resume failed')


class FakePrinter:
    config_error = Error
    command_error = CommandError

    def __init__(self, reactor, objects, missing):
        self.reactor = reactor
        self.objects = objects
        self.missing = set(missing)

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise Error("Unknown config object '%s'" % (name,))
        return default

    def load_object(self, config, section, default=sentinel):
        CTX.loaded.append(section)
        if section in self.missing or section not in self.objects:
            raise Error("Unable to load module '%s'" % (section,))
        return self.objects[section]


class FakeConfig:
    error = Error

    def __init__(self, printer, values):
        self.printer = printer
        self.values = dict(values)
        self.reads = []

    def get_printer(self):
        return self.printer

    def _get(self, kind, option, default, parse, minval, maxval):
        self.reads.append([kind, option, 'sentinel' if default is sentinel else repr(default),
                           repr(minval), repr(maxval)])
        if option not in self.values:
            if default is sentinel:
                raise Error("Option '%s' in section 'palette2' must be specified" % (option,))
            return default
        try:
            v = parse(self.values[option])
        except ValueError:
            raise Error("Unable to parse option '%s' in section 'palette2'" % (option,))
        if minval is not None and v < minval:
            raise Error("Option '%s' in section 'palette2' must have minimum of %s"
                        % (option, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section 'palette2' must have maximum of %s"
                        % (option, maxval))
        return v

    def get(self, option, default=sentinel, note_valid=True):
        return self._get('get', option, default, str, None, None)

    def getint(self, option, default=sentinel, minval=None, maxval=None, note_valid=True):
        return self._get('getint', option, default, int, minval, maxval)

    def getfloat(self, option, default=sentinel, minval=None, maxval=None, above=None,
                 below=None, note_valid=True):
        return self._get('getfloat', option, default, float, minval, maxval)


GCODE = None
RX = None
P2 = None
CONFIG = None


def try_build(values, missing):
    global GCODE, RX, P2, CONFIG, CTX
    reset_serial()
    CTX = Ctx()
    RX = py_reactor.PollReactor()
    del LOG[:]
    GCODE = FakeGCode()
    objects = {'gcode': GCODE, 'toolhead': FakeToolHead(), 'virtual_sdcard': FakeVSD(),
               'pause_resume': FakePauseResume(), 'gcode_move': object()}
    printer = FakePrinter(RX, objects, missing)
    CONFIG = FakeConfig(printer, values)
    P2 = None
    try:
        P2 = palette2.load_config(CONFIG)
        result = ['ok']
    except Exception as e:
        result = ['exc', type(e).__name__, str(e)]
    return [result, GCODE.registered, CONFIG.reads, CTX.loaded]


def call(f):
    try:
        v = f()
    except Exception as e:
        return ['exc', type(e).__name__, str(e)]
    return ['ok', v]


def cmd(name, line):
    return GCODE.handlers[name](FakeGCmd(GCODE, name, line))


def setk(name, value):
    if name in SERIAL:
        SERIAL[name] = value
    else:
        setattr(CTX, name, value)


def set_run_error(msg):
    GCODE.run_error = None if msg is None else CommandError(msg)


def norm_t(v, off):
    if v == RX.NEVER:
        return v
    return 'T' if abs(v - (RX.monotonic() + off)) < 5.0 else v


def heartbeat_ago(off):
    SERIAL['rx'] = [b'O99\n']
    return norm_t(P2._run_Read(RX.monotonic() + off), off)


def snap(exact_hb):
    p = P2
    hb = p.heartbeat
    return {
        'serial': p.serial is not None,
        'timers': [p.read_timer is not None, p.write_timer is not None,
                   p.heartbeat_timer is not None],
        'smart': p.smart_load_timer is not None,
        'flags': [p.is_printing, p.is_loading, p.is_splicing, p.is_setup_complete,
                  p.signal_disconnect],
        'heartbeat': hb if exact_hb else (None if hb is None else 'set'),
        'status': p.get_status(0.),
        'written': [d.decode() for d in SERIAL['written']],
        'opened': [[a, b] for (a, b, c, d) in SERIAL['opened']],
        'closed': SERIAL['closed'],
        'infos': [list(i) for i in GCODE.infos],
        'scripts': list(GCODE.scripts),
        'created': [list(c) for c in GCODE.created],
        'sd_calls': list(CTX.sd_calls),
        'pause': [CTX.pauses, CTX.resumes],
        'logs': [[l, norm_log(m)] for (l, m) in LOG],
    }


SCRIPT = []


def run_script(k, exact_hb):
    out = []

    def run_range(lo, hi):
        for i in range(lo, hi):
            out.append([call(SCRIPT[i]), snap(exact_hb)])

    if k > 0:
        def driver(eventtime):
            try:
                run_range(0, k)
            except BaseException as e:
                out.append(['driver-exc', type(e).__name__, str(e)])
            RX.end()
            return RX.NEVER
        RX.register_timer(driver, RX.NOW)
        RX.run()
    run_range(k, len(SCRIPT))
    return out
"#;

fn py_str(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (' '..='~').contains(&c) => out.push(c),
            c => out.push_str(&format!("\\U{:08x}", c as u32)),
        }
    }
    out.push('\'');
    out
}

fn py_bytes(b: &[u8]) -> String {
    let mut out = String::from("b'");
    for byte in b {
        out.push_str(&format!("\\x{:02x}", byte));
    }
    out.push('\'');
    out
}

/// A finite float as a Python float expression.
fn py_f(x: f64) -> String {
    format!("{:?}", x)
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn py_dict(values: &[(&str, &str)]) -> String {
    let items: Vec<String> =
        values.iter().map(|(k, v)| format!("{}: {}", py_str(k), py_str(v))).collect();
    format!("{{{}}}", items.join(", "))
}

fn py_list(items: &[&str]) -> String {
    let items: Vec<String> = items.iter().map(|s| py_str(s)).collect();
    format!("[{}]", items.join(", "))
}

// ---------------------------------------------------------------------------
// Steps
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
enum Knob {
    OpenFail(bool),
    OpenValueError(bool),
    ReadError(bool),
    WriteError(bool),
    CanExtrude(bool),
    HeaterError(bool),
    Busy(f64, f64, bool),
    Files(Vec<(&'static str, i64)>),
    FilesError(bool),
    PauseError(bool),
    RunError(Option<&'static str>),
}

#[derive(Clone, Debug)]
enum Step {
    /// Run the handler registered for the G-code command.
    Cmd(&'static str, String),
    Read(f64),
    Write(f64),
    Smart(f64),
    Heartbeat(f64),
    P2cmd(String),
    Feed(Vec<Vec<u8>>),
    /// Feed `O99` and read it at `monotonic + offset`.
    HeartbeatAgo(f64),
    Knob(Knob),
}

fn cmd(name: &'static str, line: &str) -> Step {
    Step::Cmd(name, line.to_string())
}

fn p2(line: &str) -> Step {
    Step::P2cmd(line.to_string())
}

fn feed(chunks: &[&[u8]]) -> Step {
    Step::Feed(chunks.iter().map(|c| c.to_vec()).collect())
}

fn knob(k: Knob) -> Step {
    Step::Knob(k)
}

/// `count` writes at increasing times starting at `t0`.
fn writes(count: usize, t0: f64) -> Vec<Step> {
    (0..count).map(|i| Step::Write(t0 + i as f64)).collect()
}

fn py_expr(step: &Step) -> String {
    match step {
        Step::Cmd(name, line) => format!("cmd({}, {})", py_str(name), py_str(line)),
        Step::Read(t) => format!("P2._run_Read({})", py_f(*t)),
        Step::Write(t) => format!("P2._run_Write({})", py_f(*t)),
        Step::Smart(t) => format!("P2._run_Smart_Load({})", py_f(*t)),
        Step::Heartbeat(t) => format!("norm_t(P2._run_Heartbeat({}), 5.0)", py_f(*t)),
        Step::P2cmd(line) => format!("P2.p2cmd({})", py_str(line)),
        Step::Feed(chunks) => {
            let items: Vec<String> = chunks.iter().map(|c| py_bytes(c)).collect();
            format!("setk('rx', [{}])", items.join(", "))
        }
        Step::HeartbeatAgo(off) => format!("heartbeat_ago({})", py_f(*off)),
        Step::Knob(k) => match k {
            Knob::OpenFail(b) => format!("setk('open_fail', {})", py_bool(*b)),
            Knob::OpenValueError(b) => format!("setk('open_value_error', {})", py_bool(*b)),
            Knob::ReadError(b) => format!("setk('read_error', {})", py_bool(*b)),
            Knob::WriteError(b) => format!("setk('write_error', {})", py_bool(*b)),
            Knob::CanExtrude(b) => format!("setk('can_extrude', {})", py_bool(*b)),
            Knob::HeaterError(b) => format!("setk('heater_error', {})", py_bool(*b)),
            Knob::Busy(a, b, c) => {
                format!("setk('busy', ({}, {}, {}))", py_f(*a), py_f(*b), py_bool(*c))
            }
            Knob::Files(files) => {
                let items: Vec<String> =
                    files.iter().map(|(n, s)| format!("({}, {})", py_str(n), s)).collect();
                format!("setk('files', [{}])", items.join(", "))
            }
            Knob::FilesError(b) => format!("setk('files_error', {})", py_bool(*b)),
            Knob::PauseError(b) => format!("setk('pause_error', {})", py_bool(*b)),
            Knob::RunError(None) => "set_run_error(None)".to_string(),
            Knob::RunError(Some(msg)) => format!("set_run_error({})", py_str(msg)),
        },
    }
}

// ---------------------------------------------------------------------------
// Rust fakes
// ---------------------------------------------------------------------------

struct SerialState {
    open_fail: bool,
    open_value_error: bool,
    read_error: bool,
    write_error: bool,
    rx: VecDeque<Vec<u8>>,
    written: Vec<Vec<u8>>,
    opened: Vec<(String, i64)>,
    closed: i64,
}

struct Ctx {
    serial: Mutex<SerialState>,
    can_extrude: Mutex<bool>,
    heater_error: Mutex<bool>,
    busy: Mutex<(f64, f64, bool)>,
    files: Mutex<Vec<(String, u64)>>,
    files_error: Mutex<bool>,
    sd_calls: Mutex<Vec<bool>>,
    pauses: Mutex<(i64, i64)>,
    pause_error: Mutex<bool>,
    loaded: Mutex<Vec<String>>,
    reads: Mutex<Vec<Vec<String>>>,
}

impl Ctx {
    fn new() -> Arc<Ctx> {
        Arc::new(Ctx {
            serial: Mutex::new(SerialState {
                open_fail: false,
                open_value_error: false,
                read_error: false,
                write_error: false,
                rx: VecDeque::new(),
                written: Vec::new(),
                opened: Vec::new(),
                closed: 0,
            }),
            can_extrude: Mutex::new(true),
            heater_error: Mutex::new(false),
            busy: Mutex::new((0., 0., true)),
            files: Mutex::new(Vec::new()),
            files_error: Mutex::new(false),
            sd_calls: Mutex::new(Vec::new()),
            pauses: Mutex::new((0, 0)),
            pause_error: Mutex::new(false),
            loaded: Mutex::new(Vec::new()),
            reads: Mutex::new(Vec::new()),
        })
    }
}

struct FakePort {
    ctx: Arc<Ctx>,
}

impl SerialPort for FakePort {
    fn read(&mut self) -> Result<Vec<u8>, SerialException> {
        let mut st = self.ctx.serial.lock();
        if st.read_error {
            return Err(SerialException("read failed".to_string()));
        }
        Ok(st.rx.pop_front().unwrap_or_default())
    }

    fn write(&mut self, data: &[u8]) -> Result<usize, SerialException> {
        let mut st = self.ctx.serial.lock();
        if st.write_error {
            return Err(SerialException("write failed".to_string()));
        }
        st.written.push(data.to_vec());
        Ok(data.len())
    }

    fn close(&mut self) {
        self.ctx.serial.lock().closed += 1;
    }
}

fn fake_opener(ctx: &Arc<Ctx>) -> SerialOpener {
    let ctx = ctx.clone();
    Arc::new(move |port: &str, baud: i64| {
        {
            let mut st = ctx.serial.lock();
            st.opened.push((port.to_string(), baud));
            if st.open_value_error {
                return Err(SerialOpenError::Other(KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: format!("Not a valid baudrate: {}", baud),
                }));
            }
            if st.open_fail {
                return Err(SerialOpenError::Serial(SerialException("open failed".to_string())));
            }
        }
        Ok(Box::new(FakePort { ctx: ctx.clone() }) as Box<dyn SerialPort>)
    })
}

type CreatedCommand = (String, String, IndexMap<String, String>);

struct FakeGCode {
    reactor: Reactor,
    commands: Mutex<Vec<(String, Option<String>, bool)>>,
    handlers: Mutex<Vec<(String, Arc<Mutex<GCodeCommandHandler>>)>>,
    infos: Arc<Mutex<Vec<(String, bool)>>>,
    scripts: Mutex<Vec<String>>,
    created: Mutex<Vec<CreatedCommand>>,
    run_error: Mutex<Option<String>>,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            reactor,
            commands: Mutex::new(Vec::new()),
            handlers: Mutex::new(Vec::new()),
            infos: Arc::new(Mutex::new(Vec::new())),
            scripts: Mutex::new(Vec::new()),
            created: Mutex::new(Vec::new()),
            run_error: Mutex::new(None),
        })
    }

    fn make_gcmd(&self, command: &str, commandline: &str) -> GCmd {
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            infos: self.infos.clone(),
        })
    }

    /// Runs the handler registered for `name`, like `gcode.handlers[name](gcmd)`.
    fn dispatch(&self, name: &str, commandline: &str) -> KResult<()> {
        let handler = self
            .handlers
            .lock()
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, h)| h.clone())
            .unwrap_or_else(|| panic!("no handler registered for {name}"));
        let gcmd = self.make_gcmd(name, commandline);
        let mut guard = handler.lock();
        let f: &mut GCodeCommandHandler = &mut guard;
        f(&gcmd)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.commands.lock().push((cmd.to_string(), opts.desc.clone(), opts.when_not_ready));
        if let Some(handler) = func {
            self.handlers.lock().push((cmd.to_string(), Arc::new(Mutex::new(handler))));
        }
        Ok(None)
    }

    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        panic!("register_mux_command is not used by palette2")
    }

    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }

    fn register_output_handler(&self, _callback: OutputCallback) {}

    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }

    fn run_script(&self, script: &str) -> KResult<()> {
        self.scripts.lock().push(script.to_string());
        match &*self.run_error.lock() {
            Some(msg) => Err(KlippyError::Command(msg.clone())),
            None => Ok(()),
        }
    }

    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.run_script(script)
    }

    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }

    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        self.created.lock().push((command.to_string(), commandline.to_string(), params.clone()));
        self.make_gcmd(command, commandline)
    }

    fn respond_raw(&self, _msg: &str) {}

    fn respond_info(&self, msg: &str, log: bool) {
        self.infos.lock().push((msg.to_string(), log));
    }

    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }

    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }

    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGCmd {
    command: String,
    commandline: String,
    infos: Arc<Mutex<Vec<(String, bool)>>>,
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, _name: &str, _opts: GCmdGetOpts) -> KResult<PyValue> {
        panic!("gcmd.get is not used by palette2")
    }

    fn get_int(&self, _name: &str, _opts: GCmdIntOpts) -> KResult<Option<i64>> {
        panic!("gcmd.get_int is not used by palette2")
    }

    fn get_float(&self, _name: &str, _opts: GCmdFloatOpts) -> KResult<Option<f64>> {
        panic!("gcmd.get_float is not used by palette2")
    }

    fn get_command(&self) -> String {
        self.command.clone()
    }

    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }

    fn get_command_parameters(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }

    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }

    fn respond_info(&self, msg: &str, log: bool) {
        self.infos.lock().push((msg.to_string(), log));
    }

    fn respond_raw(&self, _msg: &str) {}

    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeHeater {
    ctx: Arc<Ctx>,
}

impl HeaterApi for FakeHeater {
    fn get_name(&self) -> String {
        String::new()
    }
    fn get_pwm_delay(&self) -> f64 {
        0.
    }
    fn get_max_power(&self) -> f64 {
        0.
    }
    fn get_smooth_time(&self) -> f64 {
        0.
    }
    fn set_pwm(&self, _read_time: f64, _value: f64) {}
    fn temperature_callback(&self, _read_time: f64, _temp: f64) {}
    fn set_temp(&self, _degrees: f64) -> KResult<()> {
        Ok(())
    }
    fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        (0., 0.)
    }
    fn check_busy(&self, _eventtime: f64) -> bool {
        false
    }
    fn set_control(&self, control: Arc<dyn ControlApi>) -> Arc<dyn ControlApi> {
        control
    }
    fn alter_target(&self, _target_temp: f64) {}
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn min_temp(&self) -> f64 {
        0.
    }
    fn max_temp(&self) -> f64 {
        0.
    }
    fn min_extrude_temp(&self) -> f64 {
        0.
    }
    fn can_extrude(&self) -> bool {
        *self.ctx.can_extrude.lock()
    }
    fn target_temp(&self) -> f64 {
        0.
    }
}

klippy_api::printer_object!(FakeHeater; dyn HeaterApi);

struct FakeExtruder {
    ctx: Arc<Ctx>,
}

impl ExtraAxisApi for FakeExtruder {
    fn get_name(&self) -> String {
        String::new()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        None
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn check_move(&self, _mv: &dyn MoveApi, _ea_index: usize) -> KResult<()> {
        Ok(())
    }
    fn calc_junction(&self, _prev_move: &dyn MoveApi, _mv: &dyn MoveApi, _ea_index: usize) -> f64 {
        0.
    }
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {}
}

impl PrinterExtruderApi for FakeExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        if *self.ctx.heater_error.lock() {
            return Err(KlippyError::Command("Extruder not configured".to_string()));
        }
        Ok(Arc::new(FakeHeater { ctx: self.ctx.clone() }))
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(0.)
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        None
    }
}

struct FakeToolHead {
    ctx: Arc<Ctx>,
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by palette2")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by palette2")
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics is not used by palette2")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by palette2")
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        panic!("get_last_move_time is not used by palette2")
    }
    fn dwell(&self, _delay: f64) -> KResult<()> {
        panic!("dwell is not used by palette2")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by palette2")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by palette2")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by palette2")
    }
    fn wait_moves(&self) -> KResult<()> {
        panic!("wait_moves is not used by palette2")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by palette2")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by palette2")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by palette2")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by palette2")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by palette2")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        *self.ctx.busy.lock()
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by palette2")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by palette2")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        Arc::new(FakeExtruder { ctx: self.ctx.clone() })
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("add_extra_axis is not used by palette2")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> {
        panic!("remove_extra_axis is not used by palette2")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by palette2")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by palette2")
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory is not used by palette2")
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct FakeVSD {
    ctx: Arc<Ctx>,
}

impl VirtualSDApi for FakeVSD {
    fn get_file_list(&self, check_subdirs: bool) -> KResult<Vec<(String, u64)>> {
        self.ctx.sd_calls.lock().push(check_subdirs);
        if *self.ctx.files_error.lock() {
            return Err(KlippyError::Command("Unable to get file list".to_string()));
        }
        Ok(self.ctx.files.lock().clone())
    }

    fn is_active(&self) -> bool {
        panic!("is_active is not used by palette2")
    }

    fn do_pause(&self) {
        panic!("do_pause is not used by palette2")
    }

    fn do_resume(&self) -> KResult<()> {
        panic!("do_resume is not used by palette2")
    }

    fn do_cancel(&self) -> KResult<()> {
        panic!("do_cancel is not used by palette2")
    }

    fn is_cmd_from_sd(&self) -> bool {
        panic!("is_cmd_from_sd is not used by palette2")
    }

    fn get_file_position(&self) -> i64 {
        panic!("get_file_position is not used by palette2")
    }

    fn set_file_position(&self, _pos: i64) {
        panic!("set_file_position is not used by palette2")
    }
}

klippy_api::printer_object!(FakeVSD; dyn VirtualSDApi);

struct FakePauseResume {
    ctx: Arc<Ctx>,
}

impl PauseResumeApi for FakePauseResume {
    fn send_pause_command(&self) -> KResult<()> {
        self.ctx.pauses.lock().0 += 1;
        if *self.ctx.pause_error.lock() {
            return Err(KlippyError::Command("pause failed".to_string()));
        }
        Ok(())
    }

    fn send_resume_command(&self) -> KResult<()> {
        self.ctx.pauses.lock().1 += 1;
        if *self.ctx.pause_error.lock() {
            return Err(KlippyError::Command("resume failed".to_string()));
        }
        Ok(())
    }

    fn is_sd_active(&self) -> bool {
        panic!("is_sd_active is not used by palette2")
    }

    fn is_paused(&self) -> bool {
        panic!("is_paused is not used by palette2")
    }
}

klippy_api::printer_object!(FakePauseResume; dyn PauseResumeApi);

struct PlainObject;
klippy_api::printer_object!(PlainObject);

struct FakePrinter {
    reactor: Reactor,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    missing: Vec<String>,
    ctx: Arc<Ctx>,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        panic!("add_object is not used by palette2")
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::Config(format!("Unknown config object '{}'", name)))
            }
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        panic!("lookup_objects is not used by palette2")
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.ctx.loaded.lock().push(section.to_string());
        let objects = self.objects.lock();
        match objects.get(section) {
            Some(obj) if !self.missing.iter().any(|m| m == section) => Ok(Some(obj.clone())),
            _ => Err(KlippyError::Config(format!("Unable to load module '{}'", section))),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    values: IndexMap<String, String>,
    ctx: Arc<Ctx>,
}

fn opt_repr_i(v: Option<i64>) -> String {
    v.map(|v| v.to_string()).unwrap_or_else(|| "None".to_string())
}

fn opt_repr_f(v: Option<f64>) -> String {
    v.map(float_repr).unwrap_or_else(|| "None".to_string())
}

impl FakeConfig {
    fn record(&self, kind: &str, option: &str, default: String, minval: String, maxval: String) {
        self.ctx.reads.lock().push(vec![
            kind.to_string(),
            option.to_string(),
            default,
            minval,
            maxval,
        ]);
    }

    fn missing(option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{}' in section 'palette2' must be specified",
            option
        ))
    }

    fn unparsable(option: &str) -> KlippyError {
        KlippyError::Config(format!("Unable to parse option '{}' in section 'palette2'", option))
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }

    fn get_name(&self) -> String {
        "palette2".to_string()
    }

    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        let default = match &opts.default {
            None => "sentinel".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(s)) => str_repr(s),
        };
        self.record("get", option, default, "None".to_string(), "None".to_string());
        match self.values.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(Self::missing(option)),
                Some(d) => Ok(d),
            },
        }
    }

    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let default = match opts.default {
            None => "sentinel".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => v.to_string(),
        };
        self.record("getint", option, default, opt_repr_i(opts.minval), opt_repr_i(opts.maxval));
        let Some(raw) = self.values.get(option) else {
            return match opts.default {
                None => Err(Self::missing(option)),
                Some(d) => Ok(d),
            };
        };
        let v = int_from_string(raw, 10)
            .ok()
            .and_then(|n| n.inner().to_string().parse::<i64>().ok())
            .ok_or_else(|| Self::unparsable(option))?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::Config(format!(
                    "Option '{}' in section 'palette2' must have minimum of {}",
                    option, min
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::Config(format!(
                    "Option '{}' in section 'palette2' must have maximum of {}",
                    option, max
                )));
            }
        }
        Ok(Some(v))
    }

    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let default = match opts.default {
            None => "sentinel".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => float_repr(v),
        };
        self.record(
            "getfloat",
            option,
            default,
            opt_repr_f(opts.minval),
            opt_repr_f(opts.maxval),
        );
        let Some(raw) = self.values.get(option) else {
            return match opts.default {
                None => Err(Self::missing(option)),
                Some(d) => Ok(d),
            };
        };
        let v = float_from_string(raw).map_err(|_| Self::unparsable(option))?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::Config(format!(
                    "Option '{}' in section 'palette2' must have minimum of {}",
                    option,
                    float_repr(min)
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::Config(format!(
                    "Option '{}' in section 'palette2' must have maximum of {}",
                    option,
                    float_repr(max)
                )));
            }
        }
        Ok(Some(v))
    }

    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by palette2")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by palette2")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by palette2")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by palette2")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by palette2")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by palette2")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by palette2")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section is not used by palette2")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used by palette2")
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        panic!("get_prefix_options is not used by palette2")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {
        panic!("deprecate is not used by palette2")
    }
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

static TEST_LOCK: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));
static LOGS: LazyLock<Mutex<Vec<(String, String)>>> = LazyLock::new(|| Mutex::new(Vec::new()));
static LOG_ONCE: Once = Once::new();

fn install_log_sink() {
    LOG_ONCE.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOGS.lock().push((rec.level.name().to_string(), rec.message.to_string()));
        }));
    });
    LOGS.lock().clear();
}

fn norm_log(msg: &str) -> String {
    if let Some(i) = msg.find(" -> P2 : ") {
        if i > 0 && float_from_string(&msg[..i]).is_ok() {
            return format!("T{}", &msg[i..]);
        }
    }
    msg.to_string()
}

struct Fix {
    reactor: Reactor,
    gcode: Arc<FakeGCode>,
    ctx: Arc<Ctx>,
    config: Config,
    built: KResult<Arc<Palette2>>,
}

fn build(values: &[(&str, &str)], missing: &[&str], production_factory: bool) -> Fix {
    let reactor = Reactor::new(false);
    let ctx = Ctx::new();
    let gcode = FakeGCode::new(reactor.clone());
    let mut objects: IndexMap<String, ObjectRef> = IndexMap::new();
    objects.insert("gcode".to_string(), gcode.clone());
    objects.insert("toolhead".to_string(), Arc::new(FakeToolHead { ctx: ctx.clone() }));
    objects.insert("virtual_sdcard".to_string(), Arc::new(FakeVSD { ctx: ctx.clone() }));
    objects.insert("pause_resume".to_string(), Arc::new(FakePauseResume { ctx: ctx.clone() }));
    objects.insert("gcode_move".to_string(), Arc::new(PlainObject));
    let printer: Printer = Arc::new(FakePrinter {
        reactor: reactor.clone(),
        objects: Mutex::new(objects),
        missing: missing.iter().map(|m| m.to_string()).collect(),
        ctx: ctx.clone(),
    });
    let config: Config = Arc::new(FakeConfig {
        printer,
        values: values.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        ctx: ctx.clone(),
    });
    let built = if production_factory {
        let load_config = module_factory().load_config.expect("palette2 has load_config");
        load_config(&config).map(|obj| {
            klippy_api::downcast_arc::<Palette2>(&obj.expect("load_config returns the object"))
                .expect("the object is a Palette2")
        })
    } else {
        Palette2::new(&config, fake_opener(&ctx))
    };
    Fix { reactor, gcode, ctx, config, built }
}

fn str_v(s: &str) -> Value {
    Value::Str(s.to_string())
}

fn list_v(items: Vec<Value>) -> Value {
    Value::List(items)
}

fn py_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => match n.inner().to_string().parse::<i64>() {
            Ok(i) => Value::Int(i),
            Err(_) => Value::BigInt(n.inner().to_string()),
        },
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(py_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(py_to_value).collect()),
        PyValue::Dict(map) => Value::Dict(
            map.iter().map(|(k, v)| (py_to_value(k), py_to_value(v))).collect(),
        ),
        PyValue::Host(_) => Value::Null,
    }
}

fn error_value(e: &KlippyError) -> Value {
    list_v(vec![str_v("exc"), str_v(e.class_name()), str_v(e.message())])
}

fn result_value(r: KResult<Value>) -> Value {
    match r {
        Ok(v) => list_v(vec![str_v("ok"), v]),
        Err(e) => error_value(&e),
    }
}

impl Fix {
    fn p2(&self) -> &Arc<Palette2> {
        self.built.as_ref().expect("Palette2 built")
    }

    /// `norm_t` of the Python side: wall-clock results compare as `T`.
    fn norm_t(&self, v: f64, off: f64) -> Value {
        if v == NEVER {
            Value::float(v)
        } else if (v - (self.reactor.monotonic() + off)).abs() < 5.0 {
            str_v("T")
        } else {
            Value::float(v)
        }
    }

    fn exec(&self, step: &Step) -> Value {
        let p2 = self.p2();
        let r: KResult<Value> = match step {
            Step::Cmd(name, line) => self.gcode.dispatch(name, line).map(|_| Value::Null),
            Step::Read(t) => p2.run_read(*t).map(Value::float),
            Step::Write(t) => p2.run_write(*t).map(Value::float),
            Step::Smart(t) => p2.run_smart_load(*t).map(Value::float),
            Step::Heartbeat(t) => p2.run_heartbeat(*t).map(|v| self.norm_t(v, 5.0)),
            Step::P2cmd(line) => p2.p2cmd(line).map(|_| Value::Null),
            Step::Feed(chunks) => {
                self.ctx.serial.lock().rx = chunks.iter().cloned().collect();
                Ok(Value::Null)
            }
            Step::HeartbeatAgo(off) => {
                self.ctx.serial.lock().rx = VecDeque::from(vec![b"O99\n".to_vec()]);
                p2.run_read(self.reactor.monotonic() + *off).map(|v| self.norm_t(v, *off))
            }
            Step::Knob(k) => {
                match k {
                    Knob::OpenFail(b) => self.ctx.serial.lock().open_fail = *b,
                    Knob::OpenValueError(b) => self.ctx.serial.lock().open_value_error = *b,
                    Knob::ReadError(b) => self.ctx.serial.lock().read_error = *b,
                    Knob::WriteError(b) => self.ctx.serial.lock().write_error = *b,
                    Knob::CanExtrude(b) => *self.ctx.can_extrude.lock() = *b,
                    Knob::HeaterError(b) => *self.ctx.heater_error.lock() = *b,
                    Knob::Busy(a, b, c) => *self.ctx.busy.lock() = (*a, *b, *c),
                    Knob::Files(files) => {
                        *self.ctx.files.lock() =
                            files.iter().map(|(n, s)| (n.to_string(), *s as u64)).collect()
                    }
                    Knob::FilesError(b) => *self.ctx.files_error.lock() = *b,
                    Knob::PauseError(b) => *self.ctx.pause_error.lock() = *b,
                    Knob::RunError(msg) => {
                        *self.gcode.run_error.lock() = msg.map(|m| m.to_string())
                    }
                }
                Ok(Value::Null)
            }
        };
        result_value(r)
    }

    fn snapshot(&self, exact_hb: bool) -> Value {
        let p2 = self.p2();
        let (read_timer, write_timer, heartbeat_timer) = p2.timers_active();
        let hb = p2.heartbeat();
        let heartbeat = if exact_hb {
            hb.map(Value::float).unwrap_or(Value::Null)
        } else if hb.is_some() {
            str_v("set")
        } else {
            Value::Null
        };
        let status = py_to_value(&p2.get_status(0.0).expect("Palette2 has get_status"));
        let serial = self.ctx.serial.lock();
        let written: Vec<Value> = serial
            .written
            .iter()
            .map(|d| Value::Str(String::from_utf8_lossy(d).into_owned()))
            .collect();
        let opened: Vec<Value> = serial
            .opened
            .iter()
            .map(|(port, baud)| list_v(vec![str_v(port), Value::Int(*baud)]))
            .collect();
        let closed = serial.closed;
        drop(serial);
        let infos: Vec<Value> = self
            .gcode
            .infos
            .lock()
            .iter()
            .map(|(m, l)| list_v(vec![str_v(m), Value::Bool(*l)]))
            .collect();
        let scripts: Vec<Value> = self.gcode.scripts.lock().iter().map(|s| str_v(s)).collect();
        let created: Vec<Value> = self
            .gcode
            .created
            .lock()
            .iter()
            .map(|(c, l, params)| {
                list_v(vec![
                    str_v(c),
                    str_v(l),
                    Value::Dict(params.iter().map(|(k, v)| (str_v(k), str_v(v))).collect()),
                ])
            })
            .collect();
        let sd_calls: Vec<Value> =
            self.ctx.sd_calls.lock().iter().map(|b| Value::Bool(*b)).collect();
        let (pauses, resumes) = *self.ctx.pauses.lock();
        let logs: Vec<Value> = LOGS
            .lock()
            .iter()
            .map(|(l, m)| list_v(vec![str_v(l), Value::Str(norm_log(m))]))
            .collect();
        Value::Dict(vec![
            (str_v("serial"), Value::Bool(p2.serial_connected())),
            (
                str_v("timers"),
                list_v(vec![
                    Value::Bool(read_timer),
                    Value::Bool(write_timer),
                    Value::Bool(heartbeat_timer),
                ]),
            ),
            (str_v("smart"), Value::Bool(p2.smart_load_timer_active())),
            (
                str_v("flags"),
                list_v(vec![
                    Value::Bool(p2.is_printing()),
                    Value::Bool(p2.is_loading()),
                    Value::Bool(p2.is_splicing()),
                    Value::Bool(p2.is_setup_complete()),
                    Value::Bool(p2.signal_disconnect()),
                ]),
            ),
            (str_v("heartbeat"), heartbeat),
            (str_v("status"), status),
            (str_v("written"), list_v(written)),
            (str_v("opened"), list_v(opened)),
            (str_v("closed"), Value::Int(closed)),
            (str_v("infos"), list_v(infos)),
            (str_v("scripts"), list_v(scripts)),
            (str_v("created"), list_v(created)),
            (str_v("sd_calls"), list_v(sd_calls)),
            (str_v("pause"), list_v(vec![Value::Int(pauses), Value::Int(resumes)])),
            (str_v("logs"), list_v(logs)),
        ])
    }

    fn run_step(&self, step: &Step, exact_hb: bool) -> Value {
        let result = self.exec(step);
        list_v(vec![result, self.snapshot(exact_hb)])
    }

    /// The first `reactor_steps` steps run inside `reactor.run()`, the rest directly.
    fn run_steps(self: &Arc<Self>, steps: &[Step], reactor_steps: usize, exact_hb: bool) -> Vec<Value> {
        let results: Arc<Mutex<Vec<Value>>> = Arc::new(Mutex::new(Vec::new()));
        if reactor_steps > 0 {
            let fix = self.clone();
            let session: Vec<Step> = steps[..reactor_steps].to_vec();
            let out = results.clone();
            let reactor = self.reactor.clone();
            self.reactor.register_timer(
                move |_| {
                    for step in &session {
                        let v = fix.run_step(step, exact_hb);
                        out.lock().push(v);
                    }
                    reactor.end();
                    NEVER
                },
                NOW,
            );
            self.reactor.run();
        }
        for step in &steps[reactor_steps..] {
            let v = self.run_step(step, exact_hb);
            results.lock().push(v);
        }
        let mut out = results.lock();
        std::mem::take(&mut *out)
    }

    fn build_value(&self) -> Value {
        let result = match &self.built {
            Ok(_) => list_v(vec![str_v("ok")]),
            Err(e) => error_value(e),
        };
        let registered: Vec<Value> = self
            .gcode
            .commands
            .lock()
            .iter()
            .map(|(c, d, w)| {
                list_v(vec![
                    str_v(c),
                    d.as_deref().map(str_v).unwrap_or(Value::Null),
                    Value::Bool(*w),
                ])
            })
            .collect();
        let reads: Vec<Value> = self
            .ctx
            .reads
            .lock()
            .iter()
            .map(|r| list_v(r.iter().map(|s| str_v(s)).collect()))
            .collect();
        let loaded: Vec<Value> = self.ctx.loaded.lock().iter().map(|s| str_v(s)).collect();
        list_v(vec![result, list_v(registered), list_v(reads), list_v(loaded)])
    }
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

fn dummy_record(function: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.palette2".to_string(),
        scenario: "live".to_string(),
        seq: 0,
        function: function.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(function: &str, path: &str, expected: &Value, actual: &Value) {
    if let Err(d) = recording::assert_equal(&dummy_record(function), path, expected, actual) {
        panic!("parity divergence in {function} at {path}: {d}");
    }
}

fn start_oracle() -> Option<pyoracle::Oracle> {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the palette2 oracle test");
        return None;
    };
    py.exec(SETUP).unwrap_or_else(|e| panic!("oracle setup failed: {} {}", e.class_name, e.message));
    Some(py)
}

/// Builds on both sides (expecting `values` to configure successfully unless the test says
/// otherwise) and compares the construction result, registered commands and config reads.
fn compare_build(py: &mut pyoracle::Oracle, label: &str, values: &[(&str, &str)], missing: &[&str]) -> Fix {
    let expected = py
        .eval(&format!("try_build({}, {})", py_dict(values), py_list(missing)))
        .unwrap_or_else(|e| panic!("oracle try_build raised {}: {}", e.class_name, e.message));
    let fix = build(values, missing, false);
    check(label, "build", &expected, &fix.build_value());
    fix
}

/// Runs `steps` on both sides and compares result and snapshot after every step.
fn run_parity(
    label: &str,
    values: &[(&str, &str)],
    steps: Vec<Step>,
    reactor_steps: usize,
    exact_hb: bool,
) {
    let _guard = TEST_LOCK.lock();
    let Some(mut py) = start_oracle() else { return };

    let expected_build = py
        .eval(&format!("try_build({}, [])", py_dict(values)))
        .unwrap_or_else(|e| panic!("oracle try_build raised {}: {}", e.class_name, e.message));
    let exprs: Vec<String> =
        steps.iter().map(|s| format!("lambda: {}", py_expr(s))).collect();
    py.exec(&format!("SCRIPT[:] = [{}]", exprs.join(",\n    ")))
        .unwrap_or_else(|e| panic!("oracle SCRIPT failed: {} {}", e.class_name, e.message));
    let expected = py
        .eval(&format!("run_script({}, {})", reactor_steps, py_bool(exact_hb)))
        .unwrap_or_else(|e| panic!("oracle run_script raised {}: {}", e.class_name, e.message));

    install_log_sink();
    let fix = Arc::new(build(values, &[], false));
    check(label, "build", &expected_build, &fix.build_value());
    assert!(fix.built.is_ok(), "{label}: the port must build with the same config");
    let actual = fix.run_steps(&steps, reactor_steps, exact_hb);

    let Value::List(expected) = expected else {
        panic!("{label}: oracle returned {expected:?}");
    };
    for (i, step) in steps.iter().enumerate() {
        let path = format!("step {i} {step:?}");
        match (expected.get(i), actual.get(i)) {
            (Some(e), Some(a)) => check(label, &path, e, a),
            (e, a) => panic!("{label}: step {i} {step:?} missing: oracle {e:?}, port {a:?}"),
        }
    }
    assert_eq!(expected.len(), actual.len(), "{label}: extra steps on one side");
}

const SERIAL_PORT: &str = "/dev/ttyACM0";

fn base_values() -> Vec<(&'static str, &'static str)> {
    vec![("serial", SERIAL_PORT)]
}

fn tuned_values() -> Vec<(&'static str, &'static str)> {
    vec![
        ("serial", SERIAL_PORT),
        ("baud", "57600"),
        ("feedrate_splice", "0.5"),
        ("feedrate_normal", "0.9"),
        ("auto_load_speed", "3"),
        ("auto_cancel_variation", "0.05"),
    ]
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn construction_registers_commands_and_reads_options() {
    let _guard = TEST_LOCK.lock();
    let Some(mut py) = start_oracle() else { return };
    for (label, values) in [("defaults", base_values()), ("tuned", tuned_values())] {
        let expected = py
            .eval(&format!("try_build({}, [])", py_dict(&values)))
            .unwrap_or_else(|e| panic!("oracle try_build raised {}: {}", e.class_name, e.message));
        // Through the module factory, as the printer loads the section.
        let fix = build(&values, &[], true);
        check(label, "build via module_factory", &expected, &fix.build_value());
        let direct = build(&values, &[], false);
        check(label, "build via Palette2::new", &expected, &direct.build_value());
        let Value::List(parts) = &expected else { panic!("unexpected oracle result") };
        // Commands and reads are the interesting part: make sure they are not empty.
        assert!(matches!(&parts[1], Value::List(l) if l.len() == 5 + 33));
        assert!(matches!(&parts[2], Value::List(l) if l.len() == 6));
    }

    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn construction_error_paths() {
    let _guard = TEST_LOCK.lock();
    let Some(mut py) = start_oracle() else { return };
    type Case<'a> = (&'a str, Vec<(&'a str, &'a str)>, Vec<&'a str>);
    let cases: Vec<Case> = vec![
        ("no virtual_sdcard", base_values(), vec!["virtual_sdcard"]),
        ("no pause_resume", base_values(), vec!["pause_resume"]),
        ("no serial", vec![], vec![]),
        ("empty serial", vec![("serial", "")], vec![]),
        ("bad baud", vec![("serial", "x"), ("baud", "abc")], vec![]),
        ("feedrate_splice too high", vec![("serial", "x"), ("feedrate_splice", "1.5")], vec![]),
        ("feedrate_splice negative", vec![("serial", "x"), ("feedrate_splice", "-0.1")], vec![]),
        ("feedrate_normal too high", vec![("serial", "x"), ("feedrate_normal", "2")], vec![]),
        ("feedrate_normal negative", vec![("serial", "x"), ("feedrate_normal", "-1")], vec![]),
        ("feedrate_normal unparsable", vec![("serial", "x"), ("feedrate_normal", "fast")], vec![]),
        ("bad auto_load_speed", vec![("serial", "x"), ("auto_load_speed", "1.5")], vec![]),
        ("auto_cancel_variation too high", vec![("serial", "x"), ("auto_cancel_variation", "0.5")], vec![]),
        ("auto_cancel_variation too low", vec![("serial", "x"), ("auto_cancel_variation", "0.001")], vec![]),
        ("auto_cancel_variation ok", vec![("serial", "x"), ("auto_cancel_variation", "0.2")], vec![]),
    ];
    for (label, values, missing) in cases {
        let _ = compare_build(&mut py, label, &values, &missing);
    }
}

// ---------------------------------------------------------------------------
// Not connected
// ---------------------------------------------------------------------------

#[test]
fn not_connected_commands_and_connect_failures() {
    let mut steps = vec![
        cmd("PALETTE_CLEAR", "PALETTE_CLEAR"),
        cmd("PALETTE_CUT", "PALETTE_CUT"),
        cmd("PALETTE_SMART_LOAD", "PALETTE_SMART_LOAD"),
        // Disconnecting without a connection: the timers are `None`.
        cmd("PALETTE_DISCONNECT", "PALETTE_DISCONNECT"),
        cmd("O1", "O1"),
        cmd("O9", "O9 D1"),
        cmd("O31", "O31 D1"),
        cmd("O0", "O0 D5"),
        cmd("O2", "O2 D1 D2"),
        cmd("O5", "O5"),
        cmd("O10", "O10 D5"),
        cmd("O20", "O20 D1"),
        knob(Knob::OpenFail(true)),
        cmd("PALETTE_CONNECT", "PALETTE_CONNECT"),
        knob(Knob::OpenFail(false)),
        knob(Knob::OpenValueError(true)),
        cmd("PALETTE_CONNECT", "PALETTE_CONNECT"),
        knob(Knob::OpenValueError(false)),
        // The timers need a serial port.
        Step::Read(1.0),
        Step::Write(2.0),
        knob(Knob::Files(vec![
            ("a.mcf.gcode", 1),
            ("b.gcode", 2),
            ("sub/c.mcf.gcode", 3),
        ])),
        p2("O50"),
        Step::Write(3.0),
        Step::Write(4.0),
        knob(Knob::FilesError(true)),
        p2("O50"),
        knob(Knob::FilesError(false)),
        // Firmware version string comparison, including the lexicographic `10.0.0 < 9.0.9`.
        p2("O50 D9.1.0 D1"),
        p2("O50 D9.0.9 D1"),
        p2("O50 D9.0.8 D1"),
        p2("O50 D8.9.9 D1"),
        p2("O50 D10.0.0 D1"),
        // Without a heartbeat the heartbeat timer never disconnects.
        Step::Heartbeat(0.0),
        Step::Heartbeat(1.0),
        // `signal_disconnect` is false and there is no serial.
        Step::Read(5.0),
    ];
    steps.extend(writes(2, 6.0));
    run_parity("not_connected", &base_values(), steps, 0, true);
}

#[test]
fn header_commands_and_splice_parsing() {
    let mut steps = vec![
        cmd("O21", "O21 D1"),
        cmd("O22", "O22 D2"),
        cmd("O23", "O23 D3"),
        cmd("O24", "O24 D4"),
        cmd("O25", "O25 D1 D0 D1 D1 D0 D2"),
        cmd("O25", "O25"),
        cmd("O25", "O25 D1"),
        cmd("O25", "O25   D2   D1 D0"),
        cmd("O26", "O26 D5"),
        cmd("O27", "O27 D6"),
        cmd("O28", "O28 D7"),
        cmd("O29", "O29 D8"),
        cmd("O30", "O30 D1 D100"),
        cmd("O30", "O30 D2 D2.5"),
        cmd("O30", "O30 Dx D3"),
        cmd("O30", "O30"),
        cmd("O30", "O30 D"),
        cmd("O30", "O30 D7"),
        cmd("O32", "O32 D1 D2"),
        cmd("O32", "O32 D2 D3"),
        // O31 needs the connection.
        cmd("O31", "O31 D9"),
    ];
    // Palette 2 asks for the header, splice and algorithm data. The write queue cannot be
    // observed without a port, but the log lines and errors can.
    steps.extend(vec![p2("O20 D5")]);
    for _ in 0..10 {
        steps.push(p2("O20 D0"));
    }
    for _ in 0..4 {
        steps.push(p2("O20 D1"));
    }
    steps.extend(vec![
        p2("O20 D2"),
        p2("O20 D4"),
        p2("O20 D4"),
        p2("O20 D4"),
        p2("O20 D8"),
        p2("O20 D3"),
        p2("O20"),
        p2("O20 Dx"),
        p2("O20 D"),
        p2("O20 D100000000000000000000"),
    ]);
    // A new `O21` resets everything, including the counters.
    steps.push(cmd("O21", "O21 D2"));
    steps.push(p2("O20 D0"));
    steps.push(p2("O20 D1"));
    // `O9` ends the print: `O20` and `O34` then return early.
    steps.push(cmd("O9", "O9"));
    steps.push(p2("O20 D0"));
    steps.push(p2("O20"));
    steps.push(p2("O34 D1 D100 D0"));
    run_parity("header", &base_values(), steps, 0, true);
}

#[test]
fn p2_commands_without_serial() {
    let mut steps = vec![
        cmd("O21", "O21 D1"),
        // O34 pings and pongs, with the `auto_cancel_variation` of 0.05.
        p2("O34 D1 D95.5 D0"),
        p2("O34 D2 D104.2 D0"),
        p2("O34 D1 D100 D0"),
        p2("O34 D1 D94.5 D0"),
        p2("O34 D1 D105.5 D0"),
        p2("O34 D1 D95 D0"),
        p2("O34 D1 D105 D0"),
        p2("O34 D2 D1 D0"),
        p2("O34 D1 Dnan D0"),
        p2("O34 D1 Dinf D0"),
        p2("O34 D1 Dabc D0"),
        p2("O34 D3 D5 D0"),
        p2("O34 D1 D5"),
        p2("O34 D1"),
        p2("O34"),
        knob(Knob::RunError(Some("boom"))),
        p2("O34 D1 D80 D0"),
        knob(Knob::RunError(None)),
        // Resume and pause requests.
        p2("O40"),
        knob(Knob::PauseError(true)),
        p2("O40"),
        p2("O100"),
        knob(Knob::PauseError(false)),
        p2("O100"),
        // File list: `O53` indexes the reversed list of `.mcf.gcode` files.
        knob(Knob::Files(vec![
            ("a.mcf.gcode", 1),
            ("b.gcode", 2),
            ("sub/c.mcf.gcode", 3),
            ("d.mcf.gcode", 4),
        ])),
        p2("O50"),
        p2("O53 D1 D0"),
        p2("O53 D1 D1"),
        p2("O53 D1 D2"),
        p2("O53 D1 D3"),
        p2("O53 D1 D-1"),
        p2("O53 D1 D-3"),
        p2("O53 D1 D-4"),
        p2("O53 D1 D0x2"),
        p2("O53 D1 Dff"),
        p2("O53 D1 D99999999999999999999999999"),
        p2("O53 D1 Dxyz"),
        p2("O53 D1 D"),
        p2("O53 D2 D0"),
        p2("O53 D1"),
        p2("O53"),
        knob(Knob::RunError(Some("print failed"))),
        p2("O53 D1 D0"),
        knob(Knob::RunError(None)),
        // Errors reported by the Palette 2.
        p2("O88 D1F"),
        p2("O88 D0"),
        p2("O88 D-2A"),
        p2("O88 D0x10"),
        p2("O88"),
        p2("O88 Dzz"),
        p2("O88 D"),
        // Unknown O codes are ignored.
        p2("O1 D1"),
        p2("O77 D1 D2"),
        // Smart load needs the heater.
        p2("O102"),
        knob(Knob::CanExtrude(false)),
        p2("O102"),
        knob(Knob::CanExtrude(true)),
        knob(Knob::HeaterError(true)),
        p2("O102"),
        knob(Knob::HeaterError(false)),
        p2("O102"),
        p2("O102"),
        // `O97`: cancel, loading, feedrate.
        p2("O97 U0 D2"),
        p2("O97 U0 D3"),
        p2("O97 U39"),
        p2("O97 U39 D-100"),
        p2("O97 U39 D-40 D1"),
        p2("O97 U39 Dabc"),
        p2("O97 U39 D5"),
        p2("O97 U39 D0"),
        p2("O97 U39 D-3"),
        p2("O97 U25 D0 D0"),
        p2("O97 U25 D1 D0"),
        p2("O97 U25 D0"),
        p2("O97 U25 D2 D2"),
        p2("O97 U0"),
        p2("O97"),
        knob(Knob::RunError(Some("script failed"))),
        p2("O97 U25 D0 D0"),
        p2("O97 U0 D2"),
        knob(Knob::RunError(None)),
        // Outside a print only the feedrate matchers apply.
        cmd("O9", "O9"),
        p2("O97 U0 D2"),
        p2("O97 U0 D3"),
        p2("O97 U39"),
        p2("O97 U39 D-100"),
        p2("O97 U25 D1 D0"),
        p2("O97 U25 D0 D1"),
        p2("O34 D1 D95 D0"),
        // Lines of the Palette 2 protocol that are not O codes, and bad input.
        p2(""),
        p2("   "),
        p2("O99"),
        p2("O20x D1"),
    ];
    steps.push(cmd("O21", "O21 D3"));
    steps.push(p2("O97 U0 D3"));
    steps.push(p2("O97 U39"));
    run_parity("p2_commands", &tuned_values(), steps, 0, true);
}

#[test]
fn smart_load_timer_logic() {
    let mut steps = vec![
        // Not splicing and no remaining length yet: `None < 0` is a TypeError.
        Step::Smart(10.0),
        cmd("O21", "O21 D1"),
        Step::Smart(11.0),
        p2("O97 U39"),
        p2("O97 U39 D-100"),
        // Toolhead busy or recently idle: retry soon.
        knob(Knob::Busy(10.0, 10.2, true)),
        Step::Smart(12.0),
        knob(Knob::Busy(10.0, 9.0, false)),
        Step::Smart(13.0),
        knob(Knob::Busy(10.0, 12.5, false)),
        Step::Smart(14.0),
        knob(Knob::Busy(10.0, 10.4, true)),
        Step::Smart(15.0),
        knob(Knob::Busy(10.0, 5.0, true)),
        Step::Smart(16.0),
        knob(Knob::Busy(0.0, 5.0, true)),
    ];
    // `extrude = min(50, remaining / 2)`, `1` when it is 10 or less.
    for remaining in [-100, -101, -99, -30, -21, -20, -19, -15, -3, -1, -1000, -2] {
        steps.push(p2(&format!("O97 U39 D{}", remaining)));
        steps.push(Step::Smart(20.0));
    }
    steps.extend(vec![
        knob(Knob::RunError(Some("script failed"))),
        Step::Smart(21.0),
        knob(Knob::RunError(None)),
        p2("O97 U39 D-100000000000000000000000"),
        Step::Smart(22.0),
        // Splicing pauses the loader; a finished load waits for the next timer.
        p2("O97 U25 D0 D0"),
        Step::Smart(23.0),
        p2("O97 U25 D1 D0"),
        p2("O97 U39 D0"),
        Step::Smart(24.0),
        p2("O97 U39 D-8"),
        Step::Smart(25.0),
        // O102 starts the timer; a complete load ends it.
        p2("O102"),
        p2("O97 U39 D4"),
        p2("O102"),
        p2("O97 U39 D-4"),
        p2("O97 U39 D4"),
    ]);
    run_parity("smart_load", &tuned_values(), steps, 0, true);
}

#[test]
fn heartbeat_timer_without_connection() {
    let steps = vec![
        Step::Heartbeat(0.0),
        Step::Heartbeat(1.0),
        cmd("O21", "O21 D1"),
        Step::Heartbeat(0.0),
        Step::Write(1.0),
        Step::Write(2.0),
    ];
    run_parity("heartbeat_idle", &base_values(), steps, 0, true);
}

// ---------------------------------------------------------------------------
// Connected sessions
// ---------------------------------------------------------------------------

/// `PALETTE_CONNECT` against a Palette 2 that answers the first read with a heartbeat. The
/// reactor runs while the command waits.
fn connect_session() -> Vec<Step> {
    vec![
        feed(&[b"O99\n", b"Connection Okay\n"]),
        cmd("PALETTE_CONNECT", "PALETTE_CONNECT"),
    ]
}

#[test]
fn connected_commands_queue_and_p2_requests() {
    let mut steps = connect_session();
    steps.extend(vec![
        cmd("PALETTE_CONNECT", "PALETTE_CONNECT"),
        cmd("PALETTE_CLEAR", "PALETTE_CLEAR"),
    ]);
    steps.extend(writes(7, 1.0));
    steps.push(cmd("PALETTE_CUT", "PALETTE_CUT"));
    steps.extend(writes(2, 10.0));
    steps.push(cmd("PALETTE_SMART_LOAD", "PALETTE_SMART_LOAD"));
    steps.push(cmd("O0", "O0 D5"));
    steps.push(cmd("O12", "o12 lower"));
    steps.extend(writes(3, 20.0));
    steps.push(cmd("O31", "O31 D7"));
    steps.extend(writes(2, 30.0));
    // The print header.
    for (name, line) in [
        ("O21", "O21 D1"),
        ("O22", "O22 D2"),
        ("O23", "O23 D3"),
        ("O24", "O24 D4"),
        ("O25", "O25 D1 D0 D1"),
        ("O26", "O26 D5"),
        ("O27", "O27 D6"),
        ("O28", "O28 D7"),
        ("O29", "O29 D8"),
        ("O30", "O30 D1 D100"),
        ("O30", "O30 D2 D250"),
        ("O32", "O32 D1 D2"),
        ("O32", "O32 D2 D3"),
    ] {
        steps.push(cmd(name, line));
    }
    // Palette 2 requests the header, one line per `O20 D0`.
    for i in 0..9 {
        steps.push(p2("O20 D0"));
        steps.push(Step::Write(40.0 + i as f64));
    }
    steps.push(p2("O20 D0"));
    steps.extend(vec![
        p2("O20 D1"),
        Step::Write(50.0),
        p2("O20 D1"),
        Step::Write(51.0),
        p2("O20 D1"),
        p2("O20 D2"),
        Step::Write(52.0),
        p2("O20 D4"),
        Step::Write(53.0),
        p2("O20 D4"),
        Step::Write(54.0),
        p2("O20 D4"),
        p2("O20 D8"),
        Step::Write(55.0),
        p2("O20 D5"),
        Step::Write(56.0),
    ]);
    // The file list goes out as `O51` lines followed by `O52`.
    steps.push(knob(Knob::Files(vec![
        ("a.mcf.gcode", 1),
        ("b.gcode", 2),
        ("sub/c.mcf.gcode", 3),
    ])));
    steps.push(p2("O50"));
    steps.extend(writes(4, 60.0));
    // O1 with a fresh heartbeat pauses the print and waits for the user.
    steps.push(cmd("O1", "O1"));
    steps.extend(writes(2, 70.0));
    steps.push(knob(Knob::PauseError(true)));
    steps.push(cmd("O1", "O1"));
    steps.push(knob(Knob::PauseError(false)));
    steps.extend(writes(2, 80.0));
    // Lines arriving through the serial port, split across reads.
    steps.extend(vec![
        feed(&[b"O88 D1F\n", b"Connection Okay\n", b"O99\n", b"\xff\xfeO97 U39\n", b"x\n"]),
        Step::Read(100.0),
        feed(&[b"O97 U3", b"9 D-100\nO10", b"2\n"]),
        Step::Read(101.0),
        // A blank line makes `text_line[0]` raise; the next call continues with the queue.
        feed(&[b"\n", b"O88 D2\n", b"\xc3", b"\xa9\n", b"\xc3\xa9\n", b"O40\n"]),
        Step::Read(102.0),
        Step::Read(103.0),
        feed(&[b"  O97   U25 D0 D0  \r\n", b"O97 U25 D1 D0\n"]),
        Step::Read(104.0),
        feed(&[b"O53 D1 D0\n", b"O53 D1 D9\n", b"O20 D8\n"]),
        Step::Read(105.0),
        Step::Write(90.0),
        // The smart load timer end of the pipeline.
        knob(Knob::Busy(0.0, 5.0, true)),
        Step::Smart(110.0),
        cmd("PALETTE_SMART_LOAD", "PALETTE_SMART_LOAD"),
        feed(&[b"O97 U39 D-20\n"]),
        Step::Read(106.0),
        cmd("PALETTE_SMART_LOAD", "PALETTE_SMART_LOAD"),
        Step::Smart(111.0),
        knob(Knob::CanExtrude(false)),
        cmd("PALETTE_SMART_LOAD", "PALETTE_SMART_LOAD"),
        Step::Write(91.0),
        feed(&[b"O97 U39 D3\n"]),
        Step::Read(107.0),
        cmd("O9", "O9 D1"),
        Step::Write(92.0),
        cmd("PALETTE_DISCONNECT", "PALETTE_DISCONNECT"),
        cmd("PALETTE_DISCONNECT", "PALETTE_DISCONNECT"),
        cmd("PALETTE_CUT", "PALETTE_CUT"),
        Step::Write(93.0),
    ]);
    run_parity("connected", &tuned_values(), steps, 2, false);
}

#[test]
fn connected_heartbeat_paths() {
    let mut steps = connect_session();
    steps.extend(vec![
        // A fresh heartbeat keeps the link.
        Step::Heartbeat(0.0),
        Step::Write(1.0),
        Step::Write(2.0),
        Step::Heartbeat(1.0),
        // A stale heartbeat while printing only logs, until the setup is complete.
        cmd("O21", "O21 D1"),
        Step::HeartbeatAgo(-100.0),
        Step::Heartbeat(0.0),
        Step::Heartbeat(0.0),
        p2("O100"),
        Step::Heartbeat(0.0),
        // The link is gone.
        cmd("PALETTE_DISCONNECT", "PALETTE_DISCONNECT"),
        cmd("PALETTE_CLEAR", "PALETTE_CLEAR"),
        Step::Heartbeat(0.0),
        Step::Write(3.0),
    ]);
    run_parity("heartbeat_paths", &base_values(), steps, 2, false);
}

#[test]
fn connected_stale_heartbeat_disconnects_idle_printer() {
    let mut steps = connect_session();
    steps.extend(vec![
        cmd("PALETTE_CUT", "PALETTE_CUT"),
        Step::HeartbeatAgo(-100.0),
        // Not printing: a stale heartbeat disconnects.
        Step::Heartbeat(0.0),
        Step::Read(1.0),
        Step::Write(2.0),
        Step::Write(3.0),
    ]);
    run_parity("heartbeat_idle_disconnect", &base_values(), steps, 2, false);
}

#[test]
fn connected_read_error_disconnects() {
    let mut steps = connect_session();
    steps.extend(vec![
        knob(Knob::ReadError(true)),
        Step::Read(50.0),
        // The timers are gone after the disconnect.
        cmd("PALETTE_DISCONNECT", "PALETTE_DISCONNECT"),
        Step::Read(51.0),
        knob(Knob::ReadError(false)),
        cmd("PALETTE_CLEAR", "PALETTE_CLEAR"),
    ]);
    run_parity("read_error", &base_values(), steps, 2, false);
}

#[test]
fn connected_write_error_signals_disconnect() {
    let mut steps = connect_session();
    steps.extend(vec![
        cmd("PALETTE_CUT", "PALETTE_CUT"),
        knob(Knob::WriteError(true)),
        Step::Write(1.0),
        // `signal_disconnect` makes the next read disconnect.
        Step::Read(2.0),
        knob(Knob::WriteError(false)),
        Step::Write(3.0),
        Step::Read(4.0),
    ]);
    run_parity("write_error", &base_values(), steps, 2, false);
}

/// `O1` waits `SETUP_TIMEOUT` seconds on a stale heartbeat, then raises. Takes about eleven
/// seconds on each side.
#[test]
fn connected_o1_without_heartbeat_response() {
    let mut steps = connect_session();
    steps.extend(vec![
        cmd("O21", "O21 D1"),
        Step::HeartbeatAgo(-20.0),
        cmd("O1", "O1"),
        Step::Read(5.0),
    ]);
    run_parity("o1_no_response", &base_values(), steps, 5, false);
}

// ---------------------------------------------------------------------------
// The real serial port
// ---------------------------------------------------------------------------

fn read_until_len(fd: i32, want: usize) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..500 {
        let mut buf = [0u8; 64];
        if let Ok(n) = kos::fd::read(fd, &mut buf) {
            out.extend_from_slice(&buf[..n]);
        }
        if out.len() >= want {
            break;
        }
        kos::time::sleep(0.01);
    }
    out
}

#[test]
fn posix_serial_over_a_pty() {
    let path = format!("/tmp/palette2-test-{}.pty", std::process::id());
    let master = kos::pty::create_pty(&path).expect("create a pty");
    let mut port = match open_posix_serial(&path, 115200) {
        Ok(port) => port,
        Err(e) => panic!("open_posix_serial failed: {e:?}"),
    };

    // Nothing pending: an empty read, like `serial.read()` with `timeout=0`.
    assert_eq!(port.read().unwrap(), Vec::<u8>::new());

    // Bytes arrive one per read.
    kos::fd::write(master, b"O9").unwrap();
    let mut got = Vec::new();
    for _ in 0..500 {
        let chunk = port.read().unwrap();
        got.extend_from_slice(&chunk);
        assert!(chunk.len() <= 1);
        if got.len() == 2 {
            break;
        }
        kos::time::sleep(0.01);
    }
    assert_eq!(got, b"O9".to_vec());
    assert_eq!(port.read().unwrap(), Vec::<u8>::new());

    // Writes reach the other end unchanged (raw mode: no newline translation).
    assert_eq!(port.write(b"O99\n").unwrap(), 4);
    assert_eq!(read_until_len(master, 4), b"O99\n".to_vec());

    // A closed port raises.
    port.close();
    assert_eq!(
        port.read().unwrap_err(),
        SerialException("Attempting to use a port that is not open".to_string())
    );
    assert!(port.write(b"x").is_err());
    port.close();

    kos::fd::close(master);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn posix_serial_open_errors() {
    match open_posix_serial("/nonexistent/palette2", 115200) {
        Err(SerialOpenError::Serial(SerialException(msg))) => {
            assert!(msg.starts_with("could not open port /nonexistent/palette2: "), "{msg}");
        }
        Err(e) => panic!("unexpected error {e:?}"),
        Ok(_) => panic!("opened a missing port"),
    }
    match open_posix_serial("/nonexistent/palette2", -9600) {
        Err(SerialOpenError::Other(e)) => {
            assert_eq!(e.class_name(), "ValueError");
            assert_eq!(e.message(), "Not a valid baudrate: -9600");
        }
        Err(e) => panic!("unexpected error {e:?}"),
        Ok(_) => panic!("opened a missing port"),
    }
}

/// The fixture helpers must keep the build deterministic: building twice yields the same
/// registrations. Guards the harness itself (no oracle needed).
#[test]
fn fixture_builds_repeatably() {
    let _guard = TEST_LOCK.lock();
    let a = build(&base_values(), &[], false);
    let b = build(&base_values(), &[], false);
    check("fixture", "build", &a.build_value(), &b.build_value());
    assert!(a.config.get_name() == "palette2");
}
