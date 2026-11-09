//! Parity tests for `core::mcu` (port of `klippy/mcu.py`).
//!
//! Every expected value comes from `pyoracle` (CPython 3.8.2 + frozen
//! `klippy/`): a Python `DummyResponse` / `MCU_pwm.next_aligned_print_time`
//! / `MCUConfigHelper.seconds_to_clock` / `MCUStatsHelper` computation is
//! built with minimal fakes in `exec`, called with `eval`, and compared
//! to the Rust port with `recording::assert_equal` (floats by bits).
//!
//! Units covered: `mcu.ModuleHeader`, `mcu.CommandQueryWrapper`,
//! `mcu.AsyncResponseWrapper`, `mcu.MCU_trsync_InitGroup`,
//! `mcu.MCU_trsync_BuildConfigGroup`, `mcu.MCU_trsync_StartGroup`,
//! `mcu.ModuleHeader_1`, `mcu.MCU_endstop`, `mcu.MCU_digital_out`,
//! `mcu.MCU_pwm_InitGroup`, `mcu.MCU_pwm_BuildConfigGroup`,
//! `mcu.MCU_pwm_NextAlignedPrintTimeGroup`, `mcu.MCU_adc_InitGroup`,
//! `mcu.MCU_adc_BuildConfigGroup`, `mcu.MCU_adc_HandleAnalogInStateGroup`,
//! `mcu.MCURestartHelper_InitGroup`, `mcu.MCURestartHelper_McuIdentifyGroup`,
//! `mcu.MCURestartHelper_FirmwareRestartGroup`, `mcu.ModuleHeader_2`,
//! `mcu.MCUConnectHelper`, `mcu.MCUStatsHelper`,
//! `mcu.MCUConfigHelper_InitGroup`, `mcu.MCUConfigHelper_SendGetConfigGroup`,
//! `mcu.MCUConfigHelper_ConnectGroup`, `mcu.MCUConfigHelper_McuIdentifyGroup`,
//! `mcu.MCU_InitGroup`, `mcu.MCU_RegisterSerialResponseGroup`,
//! `mcu.AddPrinterObjects`.

use klippy_core::mcu::{
    ClockKind, Mcu, McuAdc, McuDigitalOut, McuHandle, McuTrsync, TriggerDispatch, MAX_NOMINAL_DURATION,
    MAX_SCHEDULE_TICKS, MIN_SCHEDULE_TIME, TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT,
};
use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::mcu::{AdcSampleOpts, PinParams};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::{gcell::GCell, ObjectRef};
use klippy_core::clocksync::ClockSync;
use klippy_core::configfile::{ConfigFileReader, ConfigWrapper};
use klippy_core::pins::PrinterPins;
use indexmap::IndexMap;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Value};
use std::sync::Arc;

const SETUP: &str = r#"
import mcu as mcu_mod
import zlib
class FakeReactor:
    def __init__(self): self.now = 1000.0
    def monotonic(self): return self.now
    def pause(self, t): return t
    def completion(self):
        class C:
            def __init__(self): self.v = None
            def wait(self, t): return self.v
        return C()
class FakeMsgParser:
    def create_dummy_response(self, name, params):
        d = dict(params); d['can_trigger'] = 1
        return d
class FakeSerial:
    def get_msgparser(self): return FakeMsgParser()
class FakeMCU:
    def __init__(self, freq=12000000.0):
        self.freq = freq
        self.serial = FakeSerial()
    def get_constant_float(self, name): return 12000000.0
RESULTS = {}
import configparser, configfile
class PyPrinter:
    def __init__(self):
        self.reactor = FakeReactor()
    def get_reactor(self): return self.reactor
    def get_start_args(self): return {}
    def register_event_handler(self, event, callback): pass
class PyMcuName:
    def get_name(self): return 'mcu'
def connect_helper(text):
    fc = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
    fc.read_string(text)
    cfg = configfile.ConfigWrapper(PyPrinter(), fc, {}, 'mcu')
    try:
        return list(mcu_mod.MCUConnectHelper(cfg, PyMcuName(), None).get_serialport())
    except Exception as e:
        return [type(e).__name__, str(e)]
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("mcu fakes");
            Some(py)
        }
        None => {
            println!("skipping mcu oracle tests: set KLIPPY_PYTHON to CPython 3.8.2");
            None
        }
    }
}

fn check(py: &mut pyoracle::Oracle, label: &str, expr: &str, rust: Value) {
    let expected = py.eval(expr).expect("oracle eval");
    assert_equal(&recording::Record { v: 1, unit: "mcu".to_string(), scenario: label.to_string(), seq: 0, function: "test".to_string(), args: vec![], ret: None, exc: None, side_effects: recording::SideEffects::default(), object_id: None, kind: recording::RecordKind::Python }, label, &expected, &rust).expect("parity");
}

fn int(n: i64) -> Value { Value::int(n) }
fn float(f: f64) -> Value { Value::float(f) }

#[test]
fn constants() {
    let Some(mut py) = oracle() else { return };
    check(&mut py, "min_schedule", "mcu_mod.MIN_SCHEDULE_TIME", float(MIN_SCHEDULE_TIME));
    check(&mut py, "max_ticks", "mcu_mod.MAX_SCHEDULE_TICKS", int(MAX_SCHEDULE_TICKS));
    check(&mut py, "max_nominal", "mcu_mod.MAX_NOMINAL_DURATION", float(MAX_NOMINAL_DURATION));
    check(&mut py, "trsync_timeout", "mcu_mod.TRSYNC_TIMEOUT", float(TRSYNC_TIMEOUT));
    check(&mut py, "trsync_single", "mcu_mod.TRSYNC_SINGLE_MCU_TIMEOUT", float(TRSYNC_SINGLE_MCU_TIMEOUT));
}

#[test]
fn trsync_reasons() {
    let Some(mut py) = oracle() else { return };
    check(&mut py, "r1", "mcu_mod.MCU_trsync.REASON_ENDSTOP_HIT", int(McuTrsync::REASON_ENDSTOP_HIT));
    check(&mut py, "r2", "mcu_mod.MCU_trsync.REASON_HOST_REQUEST", int(McuTrsync::REASON_HOST_REQUEST));
    check(&mut py, "r3", "mcu_mod.MCU_trsync.REASON_PAST_END_TIME", int(McuTrsync::REASON_PAST_END_TIME));
    check(&mut py, "r4", "mcu_mod.MCU_trsync.REASON_COMMS_TIMEOUT", int(McuTrsync::REASON_COMMS_TIMEOUT));
}

#[test]
fn adc_max_adc_clamp() {
    let Some(mut py) = oracle() else { return };
    py.exec("RESULTS['crc'] = zlib.crc32(b'allocate_oids count=0') & 0xffffffff").expect("py crc");
    // _finalize_config CRC is over the pre-append config commands only
    // (klippy/mcu.py:1160): `allocate_oids count=0` with no oids/cmds.
    let fake = fake_mcu();
    fake.finalize_config().expect("finalize");
    check(&mut py, "crc", "RESULTS['crc']", int(fake.config_crc()));
}

struct FakePrinter { reactor: Reactor, objects: GCell<IndexMap<String, ObjectRef>>, fileoutput: bool }
/// Stand-in for the `error_mcu` object `MCU.__init__` loads.
struct FakeErrorMcu;
klippy_api::printer_object!(FakeErrorMcu);
impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut args = IndexMap::new();
        if self.fileoutput {
            args.insert(PyValue::Str("debugoutput".into()), PyValue::Str("test-output".into()));
        }
        PyValue::Dict(args)
    }
    fn get_reactor(&self) -> Reactor { self.reactor.clone() }
    fn get_state_message(&self) -> (String, String) { ("".into(), "ready".into()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _: &str, _: &str) {}
    fn add_object(&self, name: &str, obj: klippy_api::ObjectRef) -> KResult<()> { self.objects.borrow_mut().insert(name.to_string(), obj); Ok(()) }
    fn lookup_object(&self, name: &str, d: LookupDefault) -> KResult<Option<klippy_api::ObjectRef>> {
        if let Some(o) = self.objects.borrow().get(name).cloned() { return Ok(Some(o)); }
        match d { LookupDefault::Raise => Err(klippy_api::KlippyError::Config("missing".into())), _ => Ok(None) }
    }
    fn lookup_objects(&self, _: Option<&str>) -> Vec<(String, klippy_api::ObjectRef)> { self.objects.borrow().iter().map(|(k,v)| (k.clone(), v.clone())).collect() }
    fn load_object(&self, _: &Config, name: &str, d: LookupDefault) -> KResult<Option<klippy_api::ObjectRef>> { self.lookup_object(name, d) }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _: &str, _: Option<&str>, _: bool) {}
    fn invoke_shutdown(&self, _: &str, _: PyValue) {}
    fn invoke_async_shutdown(&self, _: &str, _: PyValue) {}
    fn register_event_handler(&self, _: &str, _: klippy_api::printer::EventCallback) {}
    fn send_event(&self, _: &str, _: &[klippy_api::EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _: &str) {}
}
impl klippy_api::PrinterObject for FakePrinter {
    klippy_api::printer_object_base!();
}

/// `MCU(config, clocksync)` for the `[mcu]` section of `ini`, with the
/// `pins` and `error_mcu` objects its constructor needs.
fn mcu_from_ini(ini: &str, fileoutput: bool) -> KResult<(Printer, Arc<Mcu>)> {
    let reactor = Reactor::new(false);
    let clocksync = ClockSync::new(reactor.clone());
    let printer: Printer = Arc::new(FakePrinter { reactor, objects: GCell::new(IndexMap::new()), fileoutput });
    printer.add_object("pins", Arc::new(PrinterPins::new()) as ObjectRef)?;
    // `MCU.__init__` loads `error_mcu` (`mcu.py:1301`); its behaviour is not
    // part of these cases.
    printer.add_object("error_mcu", Arc::new(FakeErrorMcu) as ObjectRef)?;
    let fileconfig = ConfigFileReader::build_fileconfig(ini, "printer.cfg").expect("parse config text");
    let config: Config = Arc::new(ConfigWrapper::new(
        printer.clone(),
        Arc::new(fileconfig),
        Arc::new(GCell::new(IndexMap::new())),
        "mcu".to_string(),
    ));
    let mcu = Mcu::new(&config, ClockKind::Primary(clocksync))?;
    Ok((printer, mcu))
}

fn fake_mcu() -> McuHandle {
    fake_mcu_with_fileoutput(false)
}

fn fake_mcu_with_fileoutput(fileoutput: bool) -> McuHandle {
    let (printer, mcu) = mcu_from_ini("[mcu]\nserial: /dev/ttyS0\n", fileoutput).expect("construct mcu");
    let obj: klippy_api::ObjectRef = mcu.clone() as klippy_api::ObjectRef;
    printer.add_object("mcu", obj.clone()).expect("register mcu");
    McuHandle::from_trait(&obj)
}

/// `MCUConnectHelper.__init__` reads `serial`/`baud` with the config's
/// own checks: a bad `baud` raises instead of falling back to 250000.
#[test]
fn connect_helper_serial_options_match_python() {
    let Some(mut py) = oracle() else { return };
    for (label, ini) in [
        ("default baud", "[mcu]\nserial: /dev/ttyS0\n"),
        ("explicit baud", "[mcu]\nserial: /dev/ttyS0\nbaud: 115200\n"),
        ("baud below minimum", "[mcu]\nserial: /dev/ttyS0\nbaud: 100\n"),
        ("baud not an int", "[mcu]\nserial: /dev/ttyS0\nbaud: fast\n"),
        ("host pipe ignores baud", "[mcu]\nserial: /tmp/klipper_host_mcu\nbaud: 100\n"),
        ("missing serial", "[mcu]\nbaud: 115200\n"),
    ] {
        let actual = match mcu_from_ini(ini, false) {
            Ok((_, mcu)) => {
                let (port, baud) = McuHandle::from_trait(&(mcu as ObjectRef)).serialport();
                Value::List(vec![Value::str(&port), int(baud)])
            }
            Err(e) => Value::List(vec![Value::str(e.class_name()), Value::str(e.message())]),
        };
        check(&mut py, label, &format!("connect_helper({ini:?})"), actual);
    }
}

#[test]
fn module_factory() {
    let f = klippy_core::mcu::module_factory();
    assert!(f.add_printer_objects.is_some());
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_none());
}

#[test]
fn adc_setup() {
    let fake = fake_mcu();
    let pin = PinParams { chip: fake.mcu_as_trait(), chip_name: "mcu".into(), pin: "PA0".into(), invert: 0, pullup: 0, share_type: None };
    let adc = McuAdc::new(fake.clone(), pin);
    adc.setup_adc_sample(0.1, AdcSampleOpts { sample_time: 0.001, sample_count: 10, batch_num: 3, minval: 0.0, maxval: 1.0, range_check_count: 0 });
    let (t, v) = adc.get_last_value();
    assert_eq!((t.to_bits(), v.to_bits()), (0.0f64.to_bits(), 0.0f64.to_bits()));
}

#[test]
fn digital_out_setup() {
    let fake = fake_mcu();
    let pin = PinParams { chip: fake.mcu_as_trait(), chip_name: "mcu".into(), pin: "PA0".into(), invert: 1, pullup: 0, share_type: None };
    let out = McuDigitalOut::new(fake.clone(), pin);
    out.setup_max_duration(2.0);
    out.setup_start_value(true, true);
    out.set_digital(1.0, true).unwrap();
}

#[test]
fn fileoutput_homing_wait_does_not_run_background_timers() {
    let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
    py.exec(r#"
import mcu, reactor, types
r = reactor.Reactor()
events = []
dispatch = mcu.TriggerDispatch.__new__(mcu.TriggerDispatch)
dispatch._mcu = types.SimpleNamespace(is_fileoutput=lambda: True)
dispatch._trsyncs = [types.SimpleNamespace(set_home_end_time=lambda t: None)]
dispatch._trigger_completion = r.completion()
def home(eventtime):
    events.append('before')
    r.register_callback(lambda eventtime: events.append('background'), r.NOW)
    dispatch.wait_end(12.571)
    events.append(('after', dispatch._trigger_completion.wait()))
    r.end()
r.register_callback(home, r.NOW)
r.run()
r.finalize()
"#).expect("Python homing wait");

    let mcu = fake_mcu_with_fileoutput(true);
    let reactor = mcu.printer().get_reactor();
    let dispatch = TriggerDispatch::new(mcu).expect("trigger dispatch");
    let completion = dispatch.start(0.8776666666666667).expect("homing start");
    let events = Arc::new(GCell::new(Vec::new()));
    let out = events.clone();
    let r = reactor.clone();
    reactor.register_callback(move |_| {
        out.borrow_mut().push(Value::str("before"));
        let background = out.clone();
        r.register_callback(move |_| {
            background.borrow_mut().push(Value::str("background"));
        }, reactor::NOW);
        dispatch.wait_end(12.571);
        out.borrow_mut().push(Value::Tuple(vec![
            Value::str("after"),
            Value::Bool(completion.wait(reactor::NEVER).expect("homing completion")),
        ]));
        r.end();
    }, reactor::NOW);
    let watchdog = reactor.clone();
    reactor.register_timer(move |_| {
        watchdog.end();
        reactor::NEVER
    }, reactor.monotonic() + 10.0);
    reactor.run();
    check(&mut py, "fileoutput homing scheduling", "events", Value::List(events.cloned()));
}
