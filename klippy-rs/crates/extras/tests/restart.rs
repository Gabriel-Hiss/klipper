//! Printer-run ownership parity against Python's post-restart garbage collection.

use std::sync::Arc;
use std::os::unix::io::IntoRawFd;
use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::gcell::GCell;
use klippy_api::gcode::GCodeApi;
use klippy_api::printer::PrinterApi;
use klippy_api::stepper::Stepper;
use klippy_api::toolhead::{HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead as ToolHeadHandle, ToolHeadApi};
use klippy_api::{KResult, KlippyError, ObjectRef};
use klippy_core::clocksync::ClockSync;
use klippy_core::configfile::{ConfigFileReader, ConfigWrapper};
use klippy_core::mcu::{ClockKind, Mcu as CoreMcu, McuHandle};
use klippy_core::serialhdl::FdSerialPort;
use extras::motion_queuing::{NativeChelper, PrinterMotionQueuing};
use pycompat::PyValue;
use reactor::Reactor;
use recording::Value;

struct LifecycleErrorMcu;
klippy_api::printer_object!(LifecycleErrorMcu);

struct LifecycleKinematics { _toolhead: ToolHeadHandle }
impl KinematicsApi for LifecycleKinematics {
    fn class_name(&self) -> &'static str { "RestartKinematics" }
    fn get_steppers(&self) -> Vec<Stepper> { unreachable!("lifecycle-only kinematics") }
    fn calc_position(&self, _: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        unreachable!("lifecycle-only kinematics")
    }
    fn set_position(&self, _: Vec<f64>, _: &str) -> KResult<()> {
        unreachable!("lifecycle-only kinematics")
    }
    fn check_move(&self, _: &dyn MoveApi) -> KResult<()> { Ok(()) }
    fn home(&self, _: HomingStateHandle) -> KResult<()> {
        unreachable!("lifecycle-only kinematics")
    }
}
fn load_kinematics(th: &ToolHeadHandle, _: &Config) -> KResult<Kinematics> {
    Ok(Arc::new(LifecycleKinematics { _toolhead: th.clone() }))
}

fn worker_ids(name: &str) -> Vec<std::path::PathBuf> {
    std::fs::read_dir("/proc/self/task").unwrap().filter_map(|entry| {
        let path = entry.ok()?.path();
        let current = std::fs::read_to_string(path.join("comm")).ok()?;
        (current.trim() == name).then_some(path)
    }).collect()
}

#[test]
fn restart_releases_all_core_owners_and_joins_workers_like_python_gc() {
    let mut py = pyoracle::Oracle::start().expect("restart parity requires the Python oracle");
    py.exec(r#"
import collections, configparser, gc, os, sys, time, types, weakref
import klippy, reactor, gcode, pins, mcu, clocksync, configfile, toolhead, webhooks
from extras.motion_queuing import PrinterMotionQueuing

def worker_ids(name):
    out = []
    for tid in os.listdir('/proc/self/task'):
        try:
            with open('/proc/self/task/' + tid + '/comm') as f:
                if f.read().strip() == name:
                    out.append(tid)
        except FileNotFoundError:
            pass
    return out

class RestartKinematics:
    def __init__(self, th): self.toolhead = th
    def check_move(self, move): pass
_kinmod = types.ModuleType('kinematics.restart_gc_test')
_kinmod.load_kinematics = lambda th, cfg: RestartKinematics(th)
sys.modules[_kinmod.__name__] = _kinmod
_p = klippy.Printer.__new__(klippy.Printer)
_p.reactor = reactor.Reactor()
_p.start_args = {'debugoutput': '/dev/null'}
_p.event_handlers = {}
_p.objects = collections.OrderedDict()
_p.state_message = klippy.message_ready
_p.in_shutdown_state = False
_p.bglogger = None
_p.run_result = 'restart'
_p.debug_host_dumped = False

def fail_disconnect():
    raise RuntimeError('disconnect failed')
_p.register_event_handler('klippy:disconnect', fail_disconnect)
_gcode = gcode.GCodeDispatch(_p)
_p.add_object('gcode', _gcode)
_webhooks = webhooks.WebHooks(_p)
_p.add_object('webhooks', _webhooks)
_p.add_object('pins', pins.PrinterPins())
_p.add_object('error_mcu', object())
_fc = configparser.RawConfigParser()
_fc.read_string('[mcu rstsolfix]\nserial: /dev/ttyS1\n[printer]\nmax_velocity: 100\nmax_accel: 1000\nkinematics: restart_gc_test\n')
_mcfg = configfile.ConfigWrapper(_p, _fc, {}, 'mcu rstsolfix')
_cfg = configfile.ConfigWrapper(_p, _fc, {}, 'printer')
_mcu = mcu.MCU(_mcfg, clocksync.ClockSync(_p.reactor))
_p.add_object('mcu', _mcu)
_mcu._serial.connect_file(open('/dev/null', 'wb'), b'{"commands":{},"responses":{},"config":{"CLOCK_FREQ":12000000}}')
_motion = PrinterMotionQueuing(_cfg)
_p.add_object('motion_queuing', _motion)
_se = _motion.allocate_syncemitter(_mcu, 'rstsolfix', True)
_toolhead = toolhead.ToolHead(_cfg)
_p.add_object('toolhead', _toolhead)
_toolhead.move([1., 0., 0., 0.], 10.)
_toolhead.register_lookahead_callback(lambda t, th=_toolhead: None)
_gcode.register_command('RST_TEST', lambda cmd, th=_toolhead, gd=_gcode: None)
_gcode.register_output_handler(lambda msg, gd=_gcode: None)
for _ in range(1000):
    if worker_ids('rstsolfix') and worker_ids('serialq rstsolf'): break
    time.sleep(.001)
_before_step = len(worker_ids('rstsolfix'))
_before_serial = len(worker_ids('serialq rstsolf'))
_weak = [weakref.ref(o) for o in (_p, _gcode, _toolhead, _motion, _mcu, _webhooks)]
try:
    _p.send_event('klippy:disconnect')
    _disconnect_failed = False
except RuntimeError:
    _disconnect_failed = True
_p.reactor.finalize()
del _p, _gcode, _toolhead, _motion, _mcu, _cfg, _mcfg, _webhooks
gc.collect()
_expected = (_before_step, _before_serial, [w() is None for w in _weak], len(worker_ids('rstsolfix')), len(worker_ids('serialq rstsolf')), _disconnect_failed)
"#).unwrap();
    // Heaters read the bundled sensor configuration from the recorded klippy source directory.
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("crate lives three levels below the repo root");
    klippy_api::paths::set_klippy_dir(&repo.join("klippy").to_string_lossy());
    klippy_core::toolhead::register_kinematics_loader("restart_gc_test", load_kinematics);
    let reactor = Reactor::new(false);
    let printer = klippy_core::klippy::Printer::new(
        reactor.clone(), None,
        IndexMap::from([("debugoutput".to_string(), PyValue::Str("/dev/null".to_string()))]),
        Arc::new(klippy_core::klippy::HostEnv {
            modules: IndexMap::new(), klippy_dir: String::new(),
        }),
    ).unwrap();
    printer.register_event_handler("klippy:disconnect", Box::new(|_| {
        Err(KlippyError::Internal { class_name: "RuntimeError".into(), message: "disconnect failed".into() })
    }));
    let gcode = Arc::new(klippy_core::gcode::GCodeDispatch::new(printer.api()));
    printer.add_object("gcode", gcode.clone()).unwrap();
    printer.add_object("pins", Arc::new(klippy_core::pins::PrinterPins::new())).unwrap();
    printer.add_object("error_mcu", Arc::new(LifecycleErrorMcu)).unwrap();
    let webhooks = klippy_core::webhooks::WebHooks::new(&printer.api(), klippy_core::webhooks::HostInfo::current()).unwrap();
    printer.add_object("webhooks", webhooks.clone()).unwrap();
    let fileconfig = Arc::new(ConfigFileReader::build_fileconfig(
        "[mcu rstsolfix]\nserial: /dev/ttyS1\n[printer]\nmax_velocity: 100\nmax_accel: 1000\nkinematics: restart_gc_test\n", "printer.cfg",
    ).unwrap());
    let access = Arc::new(GCell::new(IndexMap::new()));
    let mcu_config: Config = Arc::new(ConfigWrapper::new(
        printer.api(), fileconfig.clone(), access.clone(), "mcu rstsolfix".into(),
    ));
    let config: Config = Arc::new(ConfigWrapper::new(
        printer.api(), fileconfig, access, "printer".into(),
    ));
    let mcu = CoreMcu::new(&mcu_config, ClockKind::Primary(ClockSync::new(reactor.clone()))).unwrap();
    printer.add_object("mcu", mcu.clone()).unwrap();
    let mcu_object: ObjectRef = mcu.clone();
    let mcu_handle = McuHandle::from_trait(&mcu_object);
    drop(mcu_object);
    let fd = std::fs::OpenOptions::new().write(true).open("/dev/null").unwrap().into_raw_fd();
    mcu_handle.serial().connect_file(
        Arc::new(FdSerialPort::new(fd, "/dev/null")),
        br#"{"commands":{},"responses":{},"config":{"CLOCK_FREQ":12000000}}"#, false,
    ).unwrap();
    let motion = PrinterMotionQueuing::new(
        printer.api(), reactor.clone(), vec![mcu.clone()], Arc::new(NativeChelper::new()),
    ).unwrap();
    let emitter = motion.allocate_syncemitter(mcu.clone(), "rstsolfix", true).unwrap();
    printer.add_object("motion_queuing", motion.clone()).unwrap();
    let toolhead = klippy_core::toolhead::ToolHead::new(&config).unwrap();
    printer.add_object("toolhead", toolhead.clone()).unwrap();
    toolhead.r#move(vec![1., 0., 0., 0.], 10.).unwrap();
    let captured_toolhead = toolhead.clone();
    toolhead.register_lookahead_callback(Box::new(move |_| {
        let _ = &captured_toolhead;
        Ok(())
    })).unwrap();
    let (captured_toolhead, captured_gcode) = (toolhead.clone(), gcode.clone());
    gcode.register_command("RST_TEST", Some(Box::new(move |_| {
        let _ = (&captured_toolhead, &captured_gcode);
        Ok(())
    })), Default::default()).unwrap();
    let output_gcode = gcode.clone();
    gcode.register_output_handler(Box::new(move |_| { let _ = &output_gcode; }));
    let timed_motion = motion.clone();
    reactor.register_timer(move |_| { let _ = &timed_motion; reactor::NEVER }, reactor::NEVER);
    let queued_motion = motion.clone();
    reactor.register_async_callback(move |_| { let _ = &queued_motion; }, reactor::NOW);
    let gc_motion = motion.clone();
    reactor.set_gc_hook(move |_| { let _ = &gc_motion; });
    let (read_fd, write_fd) = kos::fd::pipe().unwrap();
    let fd_motion = motion.clone();
    reactor.register_fd(read_fd, move |_| { let _ = &fd_motion; }, None::<fn(f64)>);
    let weak_printer = Arc::downgrade(&printer);
    let weak_gcode = Arc::downgrade(&gcode);
    let weak_toolhead = Arc::downgrade(&toolhead);
    let weak_motion = Arc::downgrade(&motion);
    let weak_mcu = Arc::downgrade(&mcu);
    let weak_webhooks = Arc::downgrade(&webhooks);
    for _ in 0..1000 {
        if !worker_ids("rstsolfix").is_empty() && !worker_ids("serialq rstsolf").is_empty() { break; }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    let before_step = worker_ids("rstsolfix");
    let before_serial = worker_ids("serialq rstsolf");
    assert!(!before_step.is_empty() && !before_serial.is_empty(), "old printer workers did not start");
    drop((motion, toolhead, gcode, mcu, mcu_handle, config, mcu_config, webhooks));
    let disconnect_failed = printer.send_event("klippy:disconnect", &[]).is_err();
    reactor.finalize();
    printer.teardown();
    drop(printer);
    assert_eq!(
        Value::Tuple(vec![
            Value::int(before_step.len() as i64), Value::int(before_serial.len() as i64),
            Value::List(vec![
                Value::Bool(weak_printer.upgrade().is_none()),
                Value::Bool(weak_gcode.upgrade().is_none()),
                Value::Bool(weak_toolhead.upgrade().is_none()),
                Value::Bool(weak_motion.upgrade().is_none()),
                Value::Bool(weak_mcu.upgrade().is_none()),
                Value::Bool(weak_webhooks.upgrade().is_none()),
            ]),
            Value::int(worker_ids("rstsolfix").len() as i64),
            Value::int(worker_ids("serialq rstsolf").len() as i64),
            Value::Bool(disconnect_failed),
        ]),
        py.eval("_expected").unwrap(),
    );
    assert!(before_step.iter().chain(&before_serial).all(|tid| !tid.exists()), "old printer workers were not joined");
    drop(emitter);
    kos::fd::close(read_fd);
    kos::fd::close(write_fd);
}
