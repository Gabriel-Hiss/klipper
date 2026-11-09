//! Parity tests for `klippy_core::pins` (port of `klippy/pins.py`).
//!
//! No recordings exist under `tests/fixtures/recordings/pins/` (checked at
//! port time), so every expected value comes from the live `pyoracle`
//! reference: generated pin descriptions, command strings and sequences
//! run against both `pins.PrinterPins` / `pins.PinResolver` and the Rust
//! port with identical fake chips, comparing return values, exception
//! class and message, and the full `active_pins` / `aliases` / `reserved`
//! state (insertion order included).

use klippy_api::config::ConfigApi;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, Chip, ChipApi, CommandQueue, CommandQueryWrapper, CommandWrapper,
    ConfigCallback, DigitalOutPin, DigitalOutPinApi, LookupPinOpts, LookupQueryCommandOpts,
    Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinResolverApi, PinsApi, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::printer::{ModuleFactory, Printer, PrinterApi};
use klippy_api::{interface, ObjectRef, PrinterObject};
use klippy_api::gcell::GCell;
use indexmap::IndexMap;
use klippy_core::pins::{PinResolver as ResolverConcrete, PrinterPins};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SETUP: &str = r#"
import pins

class FakeChip:
    def __init__(self, name):
        self.name = name
        self.calls = []
    def setup_pin(self, pin_type, pin_params):
        rec = (pin_type, pin_params['chip_name'], pin_params['pin'],
               pin_params['invert'], pin_params['pullup'],
               pin_params.get('share_type'))
        self.calls.append(rec)
        return ("%s:%s:%s:%d:%d:%s" % (pin_type, pin_params['chip_name'],
                pin_params['pin'], pin_params['invert'], pin_params['pullup'],
                pin_params.get('share_type')))

class FakeChipFail:
    name = 'bad'
    def setup_pin(self, pin_type, pin_params):
        raise pins.error("chip exploded")

def _pin_tuple(d):
    return (d['chip'].name, d['chip_name'], d['pin'], d['invert'], d['pullup'])

def _pin_tuple_share(d):
    return (d['chip'].name, d['chip_name'], d['pin'], d['invert'], d['pullup'], d.get('share_type'))

def _pp_dump():
    return [(k, v['chip_name'], v['pin'], v['invert'], v['pullup'], v.get('share_type')) for k, v in ppins.active_pins.items()]

def _make_ppins():
    global ppins, chip, chip2, chip3
    ppins = pins.PrinterPins()
    chip = FakeChip('mcu')
    chip2 = FakeChip('adc')
    chip3 = FakeChipFail()
    ppins.register_chip('mcu', chip)
    ppins.register_chip('adc', chip2)
    ppins.register_chip('bad', chip3)

def _make_resolver(validate=True):
    global res
    res = pins.PinResolver(validate_aliases=validate)
"#;

// ---------------------------------------------------------------------------
// Rust fakes.
// ---------------------------------------------------------------------------

fn never<T>(what: &str) -> T {
    panic!("{what} is not used by the pins test")
}

struct FakeMcu {
    name: String,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        never("get_printer")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        never("mcu setup_pin")
    }
    fn create_oid(&self) -> KResult<i64> {
        never("create_oid")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        never("register_config_callback")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        never("add_config_cmd")
    }
    fn request_move_queue_slot(&self) {
        never("request_move_queue_slot")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        never("get_query_slot")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        never("seconds_to_clock")
    }
    fn min_schedule_time(&self) -> f64 {
        never("min_schedule_time")
    }
    fn max_nominal_duration(&self) -> f64 {
        never("max_nominal_duration")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        never("lookup_command")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        never("lookup_query_command")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        never("try_lookup_command")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        never("alloc_command_queue")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        never("register_serial_response")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        indexmap::IndexMap::new()
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        never("get_constant_float")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        never("print_time_to_clock")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        never("clock_to_print_time")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        never("estimated_print_time")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        never("clock32_to_clock64")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        never("calibrate_clock")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> pycompat::PyValue {
        never("get_status")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        never("stats")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        never("create_trigger_dispatch")
    }
}

struct FakeDigitalOut {
    mcu: Mcu,
}

impl DigitalOutPinApi for FakeDigitalOut {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, _max_duration: f64) {
        never("setup_max_duration")
    }
    fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {
        never("setup_start_value")
    }
    fn set_digital(&self, _print_time: f64, _value: bool) {
        never("set_digital")
    }
}

#[allow(clippy::type_complexity)]
struct FakeChip {
    name: String,
    calls: GCell<Vec<(String, String, String, i64, i64, Option<String>)>>,
}

impl FakeChip {
    fn new(name: &str) -> Arc<FakeChip> {
        Arc::new(FakeChip { name: name.to_string(), calls: GCell::new(Vec::new()) })
    }
    fn mcu(&self) -> Mcu {
        Arc::new(FakeMcu { name: self.name.clone() }) as Mcu
    }
    fn calls(&self) -> Vec<(String, String, String, i64, i64, Option<String>)> {
        self.calls.borrow().clone()
    }
}

impl ChipApi for FakeChip {
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        let rec = (
            pin_type.to_string(),
            pin_params.chip_name.clone(),
            pin_params.pin.clone(),
            pin_params.invert,
            pin_params.pullup,
            pin_params.share_type.clone(),
        );
        self.calls.borrow_mut().push(rec);
        Ok(Pin::DigitalOut(
            Arc::new(FakeDigitalOut { mcu: pin_params.chip.clone() }) as DigitalOutPin,
        ))
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu()
    }
}

struct FakeChipFail {
    name: String,
}

impl FakeChipFail {
    fn new() -> Arc<FakeChipFail> {
        Arc::new(FakeChipFail { name: "bad".to_string() })
    }
}

impl ChipApi for FakeChipFail {
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Pins("chip exploded".to_string()))
    }
    fn get_mcu(&self) -> Mcu {
        Arc::new(FakeMcu { name: self.name.clone() }) as Mcu
    }
}

fn as_chip<T: ChipApi + 'static>(chip: Arc<T>) -> Chip {
    chip as Chip
}

/// Maps a `Pin` returned by [`PinsApi::setup_pin`] to the formatted string
/// the Python [`FakeChip.setup_pin`] returns, so both sides compare equal.
fn pin_value(chip: &FakeChip, pin: &Pin) -> Value {
    let (pin_type, chip_name, pin_name, invert, pullup, share) =
        chip.calls().into_iter().last().expect("chip was called");
    let _ = pin;
    Value::Str(format!(
        "{}:{}:{}:{}:{}:{}",
        pin_type,
        chip_name,
        pin_name,
        invert,
        pullup,
        share.as_deref().unwrap_or("None")
    ))
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("pins fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping pins oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_repr(s: &str) -> String {
    pycompat::text::str_repr(s)
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

/// Fresh (PrinterPins, chip, chip2) pair on both sides.
fn fresh_pair(
    py: &mut pyoracle::Oracle,
) -> (PrinterPins, Arc<FakeChip>, Arc<FakeChip>) {
    py.exec("_make_ppins()").expect("py make ppins");
    let chip = FakeChip::new("mcu");
    let chip2 = FakeChip::new("adc");
    let ppins = PrinterPins::new();
    ppins
        .register_chip("mcu", as_chip(chip.clone()))
        .expect("rust register mcu");
    ppins
        .register_chip("adc", as_chip(chip2.clone()))
        .expect("rust register adc");
    ppins
        .register_chip("bad", as_chip(FakeChipFail::new()))
        .expect("rust register bad");
    (ppins, chip, chip2)
}

fn pin_tuple_value(p: &PinParams, with_share: bool) -> Value {
    let mut items = vec![
        Value::Str(p.chip.get_name()),
        Value::Str(p.chip_name.clone()),
        Value::Str(p.pin.clone()),
        Value::Int(p.invert),
        Value::Int(p.pullup),
    ];
    if with_share {
        items.push(match &p.share_type {
            Some(s) => Value::Str(s.clone()),
            None => Value::Null,
        });
    }
    Value::Tuple(items)
}

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "pins".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

/// Compare an oracle result against a Rust [`KResult`]`Value`.
fn check_result(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    match (expected, actual) {
        (Ok(expected_value), Ok(actual_value)) => {
            if let Err(d) = assert_equal(&record(label), "ret", &expected_value, &actual_value) {
                panic!("{label}: {d}");
            }
        }
        (Err(exc), Err(err)) => {
            assert_eq!(
                exc.class_name,
                err.class_name(),
                "{label}: exception class",
            );
            assert_eq!(exc.message, err.message(), "{label}: exception message");
        }
        (Ok(v), Err(e)) => {
            panic!("{label}: expected value {v:?} but rust raised {}: {}", e.class_name(), e.message())
        }
        (Err(e), Ok(v)) => {
            panic!("{label}: expected exception {}: {} but rust returned {v:?}", e.class_name, e.message)
        }
    }
}

/// Compare oracle `Ok(value)` against a Rust value (no error expected).
fn check_ok(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected_value = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record(label), "ret", &expected_value, &actual) {
        panic!("{label}: {d}");
    }
}

fn pp_dump_value(ppins: &PrinterPins) -> Value {
    Value::List(
        ppins
            .active_pins()
            .into_iter()
            .map(|(k, v)| {
                Value::Tuple(vec![
                    Value::Str(k),
                    Value::Str(v.chip_name),
                    Value::Str(v.pin),
                    Value::Int(v.invert),
                    Value::Int(v.pullup),
                    match &v.share_type {
                        Some(s) => Value::Str(s.clone()),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

#[allow(clippy::type_complexity)]
fn chip_calls_value(calls: &[(String, String, String, i64, i64, Option<String>)]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|(a, b, c, d, e, f)| {
                Value::Tuple(vec![
                    Value::Str(a.clone()),
                    Value::Str(b.clone()),
                    Value::Str(c.clone()),
                    Value::Int(*d),
                    Value::Int(*e),
                    match f {
                        Some(s) => Value::Str(s.clone()),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

fn map_str_str_value(m: &[(String, String)]) -> Value {
    Value::List(
        m.iter()
            .map(|(k, v)| Value::Tuple(vec![Value::Str(k.clone()), Value::Str(v.clone())]))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// parse_pin battery.
// ---------------------------------------------------------------------------

#[test]
fn parse_pin_battery_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let (ppins, _chip, _chip2) = fresh_pair(&mut py);

    let descs: &[&str] = &[
        "PB1",
        "mcu:PB1",
        "adc:PB1",
        "  PB1  ",
        "\tPB1\t",
        "  mcu : PB1  ",
        "mcu :PB1",
        "mcu: PB1",
        " main:PA0",
        "mcu:PA0:",
        "a:b:c",
        ":PB1",
        "mcu:",
        ":",
        "",
        "   ",
        "!",
        "^",
        "~",
        "!!PB1",
        "^^PB1",
        "!PB1",
        "^PB1",
        "~PB1",
        "!^PB1",
        "^!PB1",
        "~!PB1",
        "!~PB1",
        "!  PB1",
        "^  PB1",
        "~  PB1",
        "^  !  PB1",
        "~  !  PB1",
        "!  ^PB1",
        "  !PB1",
        "  ^PB1",
        "x:PB1",
        "Mcu:PB1",
        "mcu:PB 1",
        "mcu:PB^1",
        "mcu:PB~1",
        "mcu:PB!1",
        "mcu:PB:1",
        "PB 1",
        "PB^1",
        "PB~1",
        "PB!1",
        "PB\t1",
        "PB\n1",
        "PB_1",
        "PB-1",
        "PB.CPU",
        "mcu:PB-REV",
        "None",
        "mcu:None",
        "0",
        "mcu:0",
        "P/B1",
        " ",
        "  mcu  ",
        " mcu : ",
        "PA 0 ",
        "ab:cd:ef:gh",
        "unknown:PB1",
        "unknown:",
        "bad:PB1",
        "bad:PB 1",
        "bad:PB^1",
    ];

    for desc in descs {
        for &can_invert in &[false, true] {
            for &can_pullup in &[false, true] {
                let expr = format!(
                    "_pin_tuple(ppins.parse_pin({}, {}, {}))",
                    py_repr(desc),
                    py_bool(can_invert),
                    py_bool(can_pullup),
                );
                let expected = py.eval(&expr);
                let actual = ppins
                    .parse_pin(
                        desc,
                        ParsePinOpts { can_invert, can_pullup },
                    )
                    .map(|p| pin_tuple_value(&p, false));
                check_result(
                    &format!("parse {desc:?} inv={can_invert} pull={can_pullup}"),
                    expected,
                    actual,
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// lookup_pin sharing.
// ---------------------------------------------------------------------------

#[test]
fn lookup_pin_sharing_matches_oracle() {
    let Some(mut py) = oracle() else { return };

    // (description, can_invert, can_pullup, share_type)
    type Step = (&'static str, bool, bool, Option<&'static str>);

    fn run_steps(label: &str, py: &mut pyoracle::Oracle, steps: &[Step]) {
        let (ppins, _chip, _chip2) = fresh_pair(py);
        for (i, (desc, can_invert, can_pullup, share_type)) in steps.iter().enumerate() {
            let share_expr = match share_type {
                Some(s) => py_repr(s),
                None => "None".to_string(),
            };
            let expr = format!(
                "_pin_tuple_share(ppins.lookup_pin({}, {}, {}, {}))",
                py_repr(desc),
                py_bool(*can_invert),
                py_bool(*can_pullup),
                share_expr,
            );
            let expected = py.eval(&expr);
            let actual = ppins
                .lookup_pin(
                    desc,
                    LookupPinOpts {
                        can_invert: *can_invert,
                        can_pullup: *can_pullup,
                        share_type: share_type.map(str::to_string),
                    },
                )
                .map(|p| pin_tuple_value(&p, true));
            check_result(&format!("{label} step {i} lookup {desc:?}"), expected, actual);
            check_ok(
                &format!("{label} step {i} active"),
                py.eval("_pp_dump()"),
                pp_dump_value(&ppins),
            );
        }
    }

    run_steps(
        "dup-none",
        &mut py,
        &[("PB1", false, false, None), ("PB1", false, false, None)],
    );
    run_steps(
        "dup-none-prefix",
        &mut py,
        &[("PB1", false, false, None), ("mcu:PB1", false, false, None)],
    );
    run_steps(
        "share-same",
        &mut py,
        &[("PB1", false, false, Some("cs")), ("PB1", false, false, Some("cs"))],
    );
    run_steps(
        "share-same-inv-err",
        &mut py,
        &[
            ("!PB1", true, false, Some("cs")),
            ("PB1", true, false, Some("cs")),
        ],
    );
    run_steps(
        "share-same-pull-err",
        &mut py,
        &[
            ("^PB1", true, true, Some("cs")),
            ("PB1", true, true, Some("cs")),
        ],
    );
    run_steps(
        "share-diff-type",
        &mut py,
        &[("PB1", false, false, Some("a")), ("PB1", false, false, Some("b"))],
    );
    run_steps(
        "share-none-vs-typed",
        &mut py,
        &[("PB1", false, false, None), ("PB1", false, false, Some("a"))],
    );
    run_steps(
        "share-typed-vs-none",
        &mut py,
        &[("PB1", false, false, Some("a")), ("PB1", false, false, None)],
    );
    run_steps(
        "two-chips",
        &mut py,
        &[("PB1", false, false, None), ("adc:PB1", false, false, None)],
    );
    run_steps(
        "two-pins",
        &mut py,
        &[("PB1", false, false, None), ("PB2", false, false, None), ("PB1", false, false, Some("x"))],
    );
    run_steps(
        "parse-err-then-ok",
        &mut py,
        &[("x:PB1", false, false, None), ("PB1", false, false, None)],
    );
    run_steps(
        "inv-vs-plain-none",
        &mut py,
        &[("!PB1", true, false, None), ("PB1", true, false, None)],
    );
}

#[test]
fn allow_multi_use_and_reset_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let (ppins, _chip, _chip2) = fresh_pair(&mut py);

    // allow_multi_use_pin then duplicates are allowed.
    check_result(
        "allow-multi",
        py.eval("ppins.allow_multi_use_pin('PB1')"),
        ppins.allow_multi_use_pin("PB1").map(|_| Value::Null),
    );
    check_ok(
        "allow-map",
        py.eval("list(ppins.allow_multi_use_pins.items())"),
        Value::List(
            ppins
                .allow_multi_use_pins()
                .into_iter()
                .map(|(k, v)| Value::Tuple(vec![Value::Str(k), Value::Bool(v)]))
                .collect(),
        ),
    );
    for i in 0..2 {
        let expected = py.eval("_pin_tuple_share(ppins.lookup_pin('PB1', False, False, None))");
        let actual = ppins
            .lookup_pin("PB1", LookupPinOpts::default())
            .map(|p| pin_tuple_value(&p, true));
        check_result(&format!("allow-multi lookup {i}"), expected, actual);
    }
    check_ok("allow active", py.eval("_pp_dump()"), pp_dump_value(&ppins));

    // reset_pin_sharing on the stored params, then lookup again succeeds.
    let params = ppins
        .lookup_pin("PB2", LookupPinOpts::default())
        .expect("lookup PB2 rust");
    py.eval("_pin_tuple_share(ppins.lookup_pin('PB2', False, False, None))")
        .expect("py lookup PB2");
    let rust_reset = { ppins.reset_pin_sharing(params); Ok(Value::Null) };
    check_result(
        "reset",
        py.eval("ppins.reset_pin_sharing(list(ppins.active_pins.values())[-1])"),
        rust_reset,
    );
    check_ok("after reset", py.eval("_pp_dump()"), pp_dump_value(&ppins));
    let expected = py.eval("_pin_tuple_share(ppins.lookup_pin('PB2', False, False, None))");
    let actual = ppins
        .lookup_pin("PB2", LookupPinOpts::default())
        .map(|p| pin_tuple_value(&p, true));
    check_result("re-lookup after reset", expected, actual);

    // reset on a params object never inserted raises KeyError (panics here,
    // since the `PinsApi` contract has no error channel for it).
    let ghost = ppins
        .parse_pin("PB9", ParsePinOpts::default())
        .expect("parse ghost rust");
    let rust_key_error = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        ppins.reset_pin_sharing(ghost);
    }))
    .err()
    .map(|payload| {
        payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_string()))
            .expect("panic payload is text")
    });
    let py_exc = py
        .eval("ppins.reset_pin_sharing(ppins.parse_pin('PB9', False, False))")
        .expect_err("py reset-missing raises");
    assert_eq!(py_exc.class_name, "KeyError", "py reset-missing class");
    assert_eq!(
        format!("KeyError: {}", py_exc.message),
        rust_key_error.expect("rust reset-missing panics"),
        "py reset-missing message matches rust panic text"
    );

    // A second reset of the same (now removed) params raises KeyError.
    let params2 = ppins
        .lookup_pin("PB3", LookupPinOpts::default())
        .expect("lookup PB3");
    py.eval("_pin_tuple_share(ppins.lookup_pin('PB3', False, False, None))")
        .expect("py lookup PB3");
    let rust_first = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        ppins.reset_pin_sharing(params2);
    }));
    let py_first = py.eval("ppins.reset_pin_sharing(ppins.parse_pin('PB3', False, False))");
    assert_eq!(py_first.is_ok(), rust_first.is_ok(), "reset-first ok match");
    // Python `del` removed the just-looked-up PB3 on both sides.
    check_ok("after reset-first", py.eval("_pp_dump()"), pp_dump_value(&ppins));
}

#[test]
fn allow_multi_use_pin_errors_match_oracle() {
    let Some(mut py) = oracle() else { return };
    let (ppins, _chip, _chip2) = fresh_pair(&mut py);
    for desc in ["x:PB1", "PB 1", "^PB1", "PB^1", "unknown:PB1", " bad:PB1"] {
        let expected = py.eval(&format!(
            "ppins.allow_multi_use_pin({})",
            py_repr(desc),
        ));
        let actual = ppins.allow_multi_use_pin(desc).map(|_| Value::Null);
        check_result(&format!("allow-multi err {desc:?}"), expected, actual);
    }
    // Untouched pins still check as untouched.
    check_ok(
        "allow-map-after-errors",
        py.eval("list(ppins.allow_multi_use_pins.items())"),
        Value::List(
            ppins
                .allow_multi_use_pins()
                .into_iter()
                .map(|(k, v)| Value::Tuple(vec![Value::Str(k), Value::Bool(v)]))
                .collect(),
        ),
    );
}

#[test]
fn register_chip_and_resolver_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let (ppins, _chip, _chip2) = fresh_pair(&mut py);

    // Duplicate (exact and after strip).
    for name in ["mcu", " mcu ", "\tmcu\t"] {
        let expected = py.eval(&format!("ppins.register_chip({}, chip)", py_repr(name)));
        let actual = ppins
            .register_chip(name, as_chip(FakeChip::new("mcu")))
            .map(|_| Value::Null);
        check_result(&format!("dup chip {name:?}"), expected, actual);
    }

    // Unknown resolver, and resolvers of registered chips.
    for name in ["nosuch", " mcu ", "", "adc"] {
        let expected = py.eval(&format!(
            "ppins.get_pin_resolver({}).validate_aliases",
            py_repr(name)
        ));
        let actual = match ppins.get_pin_resolver(name) {
            Ok(_) => ppins
                .resolver(name)
                .map(|r| Value::Bool(r.validate_aliases()))
                .ok_or_else(|| KlippyError::Pins("missing resolver".to_string())),
            Err(e) => Err(e),
        };
        check_result(&format!("resolver {name:?}"), expected, actual);
    }

    // PinResolver() created by register_chip validates by default.
    py.exec("res = ppins.get_pin_resolver('mcu')").expect("py resolver");
    let py_res = py.eval("res.validate_aliases").expect("validate_aliases");
    let rs_res = ppins.resolver("mcu").expect("rust resolver");
    check_ok(
        "resolver validate",
        Ok(py_res),
        Value::Bool(rs_res.validate_aliases()),
    );
}

// ---------------------------------------------------------------------------
// setup_pin dispatch.
// ---------------------------------------------------------------------------

#[test]
fn setup_pin_dispatch_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    // fresh_pair is called per case below.
    let types: &[&str] = &[
        "endstop",
        "digital_out",
        "pwm",
        "adc",
        "stepper",
        "",
        "ENDSTOP",
        "digitalout",
    ];
    let descs: &[&str] = &[
        "PB1",
        "!PB1",
        "^PB1",
        "~PB1",
        "^!PB1",
        "~!PB1",
        "!^PB1",
        "adc:PB1",
        "!adc:PB1",
        "^adc:PB1",
        "  PB2  ",
        "bad:PB1",
        "bad:!PB1",
        "bad:^PB1",
    ];

    for pin_type in types {
        for desc in descs {
            // Fresh state per (type, desc) so sharing cannot mask parse errors.
            let (one, c1, c2) = fresh_pair(&mut py);
            let expected = py.eval(&format!(
                "ppins.setup_pin({}, {})",
                py_repr(pin_type),
                py_repr(desc),
            ));
            let actual = one.setup_pin(pin_type, desc).map(|pin| {
                let called = if !c1.calls().is_empty() { &c1 } else { &c2 };
                pin_value(called, &pin)
            });
            check_result(&format!("setup {pin_type:?} {desc:?}"), expected, actual);
            check_ok(
                &format!("setup {pin_type:?} {desc:?} calls"),
                py.eval("(list(chip.calls), list(chip2.calls))"),
                Value::Tuple(vec![
                    chip_calls_value(&c1.calls()),
                    chip_calls_value(&c2.calls()),
                ]),
            );
        }
    }
}

#[test]
fn setup_pin_chips_and_second_call_match_oracle() {
    let Some(mut py) = oracle() else { return };
    let (ppins, chip, chip2) = fresh_pair(&mut py);

    // Second setup of the same pin raises "used multiple times".
    let expected = py.eval("ppins.setup_pin('pwm', 'PB1')");
    let actual = ppins.setup_pin("pwm", "PB1").map(|pin| pin_value(&chip, &pin));
    check_result("setup first", expected, actual);
    let expected = py.eval("ppins.setup_pin('pwm', 'PB1')");
    let actual = ppins.setup_pin("pwm", "PB1").map(|pin| pin_value(&chip, &pin));
    check_result("setup second", expected, actual);
    check_ok(
        "chip called once",
        py.eval("list(chip.calls)"),
        chip_calls_value(&chip.calls()),
    );

    // Chip-side errors propagate with the same class and message.
    let expected = py.eval("ppins.setup_pin('pwm', 'bad:PB1')");
    let actual = ppins.setup_pin("pwm", "bad:PB1").map(|pin| pin_value(&chip, &pin));
    check_result("chip error", expected, actual);
    // The failed setup left the pin marked active on both sides.
    check_ok("active after chip error", py.eval("_pp_dump()"), pp_dump_value(&ppins));
    let _ = chip2;
}

// ---------------------------------------------------------------------------
// PinResolver: reserve / alias / update_command.
// ---------------------------------------------------------------------------

fn resolver_state(label: &str, py: &mut pyoracle::Oracle, res: &ResolverConcrete) {
    check_ok(
        &format!("{label} reserved"),
        py.eval("list(res.reserved.items())"),
        map_str_str_value(&res.reserved()),
    );
    check_ok(
        &format!("{label} aliases"),
        py.eval("list(res.aliases.items())"),
        map_str_str_value(&res.aliases()),
    );
    check_ok(
        &format!("{label} active"),
        py.eval("list(res.active_pins.items())"),
        map_str_str_value(&res.active_pins()),
    );
}

#[test]
fn resolver_reserve_and_alias_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec("_make_resolver(True)").expect("py resolver");
    let res = ResolverConcrete::new(true);

    // reserve_pin: ok, same name again, conflict.
    for (pin, name) in [("PB1", "spi"), ("PB1", "spi"), ("PB1", "i2c"), ("PB2", "spi")] {
        let expected = py.eval(&format!(
            "res.reserve_pin({}, {})",
            py_repr(pin),
            py_repr(name),
        ));
        let actual = res.reserve_pin(pin, name).map(|_| Value::Null);
        check_result(&format!("reserve {pin:?} {name:?}"), expected, actual);
        resolver_state("after reserve", &mut py, &res);
    }

    // alias_pin: basic, same again, conflict, invalid targets.
    let cases: &[(&str, &str)] = &[
        ("cs", "PB1"),
        ("cs", "PB1"),
        ("cs", "PB2"),
        ("sclk", "PB^1"),
        ("sclk", "PB~1"),
        ("sclk", "PB!1"),
        ("sclk", "PB:1"),
        ("sclk", "PB 1"),
        ("sclk", "PB\t1"),
        ("sclk", ""),
        ("pull", "^PB1"),
        ("cs2", "cs"),
        ("x", "PB9"),
        ("y", "x"),
        ("z", "y"),
        ("w", "x"),
    ];
    for (alias, pin) in cases {
        let expected = py.eval(&format!(
            "res.alias_pin({}, {})",
            py_repr(alias),
            py_repr(pin),
        ));
        let actual = res.alias_pin(alias, pin).map(|_| Value::Null);
        check_result(&format!("alias {alias:?} -> {pin:?}"), expected, actual);
        resolver_state("after alias", &mut py, &res);
    }

    // Existing-alias remap: aliasing "x" again to a new target moves
    // everything that pointed at "x" (checked through update_command).
    let expected = py.eval("res.alias_pin('x', 'PB10')");
    let actual = res.alias_pin("x", "PB10").map(|_| Value::Null);
    check_result("alias remap", expected, actual);
    resolver_state("after remap", &mut py, &res);
}

#[test]
fn resolver_validate_variants_match_oracle() {
    let Some(mut py) = oracle() else { return };
    for validate in [true, false] {
        py.exec(&format!("_make_resolver({})", py_bool(validate)))
            .expect("py resolver");
        let res = ResolverConcrete::new(validate);
        // Manually drive with alias then conflicting name.
        py.eval("res.alias_pin('A1', 'PB1')").expect("py alias");
        res.alias_pin("A1", "PB1").expect("rust alias");
        let expected = py.eval("res.update_command('oid=1 pin=A1')");
        let actual = res.update_command("oid=1 pin=A1").map(Value::Str);
        check_result(&format!("validate={validate} first"), expected, actual);
        resolver_state("validate after first", &mut py, &res);
        let expected = py.eval("res.update_command('oid=2 pin=PB1')");
        let actual = res.update_command("oid=2 pin=PB1").map(Value::Str);
        check_result(&format!("validate={validate} second"), expected, actual);
        resolver_state("validate after second", &mut py, &res);
    }
}

#[test]
fn update_command_battery_matches_oracle() {
    let Some(mut py) = oracle() else { return };

    let cmds: &[&str] = &[
        "config_stepper dir_pin=PB1",
        "pin=PB1",
        " pin=PB1",
        "  pin=PB1",
        "_pin=PB1",
        "x_pin=PB1",
        "xpin=PB1",
        "\tpin=PB1",
        "\npin=PB1",
        " pin=PB1 pin=PB2",
        " pin=PB1 other pin=PB2 ",
        " pin=",
        " pin= ",
        " pin=  x",
        " pin =PB1",
        " pin==PB1",
        " pin=PB1X pin=PB1",
        " pin=PB1 pin=PB1",
        " pin=PB1\tpin=PB2",
        " pin=PB1\npin=PB2",
        " pin=PB1\thold",
        " a pin=B c_pin=D e",
        " pin=A pin=B pin=A",
        " no pins here",
        "",
        "pin=",
        " _pin=A_B",
        " pin=..::PB1",
        " pin=PB1\x0bpin=PB2",
        " pin=PB1\x0cpin=PB2",
        " pin=PB1 pin=A pin=PB1",
    ];

    for cmd in cmds {
        py.exec("_make_resolver(True)").expect("py resolver");
        let res = ResolverConcrete::new(true);
        // Uniform alias/reservation setup for every command.
        py.eval("res.alias_pin('A', 'PB1')").expect("py alias");
        py.eval("res.alias_pin('B', 'PB2')").expect("py alias");
        py.eval("res.reserve_pin('PB2', 'spi')").expect("py reserve");
        res.alias_pin("A", "PB1").expect("rust alias");
        res.alias_pin("B", "PB2").expect("rust alias");
        res.reserve_pin("PB2", "spi").expect("rust reserve");

        let expected = py.eval(&format!(
            "res.update_command({})",
            py_repr(cmd),
        ));
        let actual = res.update_command(cmd).map(Value::Str);
        check_result(&format!("update {cmd:?}"), expected, actual);
        resolver_state("update state", &mut py, &res);
    }

    // Reserved id reached through an alias: error names the alias.
    py.exec("_make_resolver(True)").expect("py resolver");
    let res = ResolverConcrete::new(true);
    py.eval("res.alias_pin('B', 'PB2')").expect("py alias");
    py.eval("res.reserve_pin('PB2', 'spi')").expect("py reserve");
    res.alias_pin("B", "PB2").expect("rust alias");
    res.reserve_pin("PB2", "spi").expect("rust reserve");
    let expected = py.eval("res.update_command('pin=B')");
    let actual = res.update_command("pin=B").map(Value::Str);
    check_result("update reserved-via-alias", expected, actual);
    resolver_state("reserved state", &mut py, &res);
}

#[test]
fn update_command_sequences_match_oracle() {
    let Some(mut py) = oracle() else { return };
    // Same resolver across several commands: active_pins accumulates.
    py.exec("_make_resolver(True)").expect("py resolver");
    let res = ResolverConcrete::new(true);
    py.eval("res.alias_pin('A', 'PB1')").expect("py alias");
    res.alias_pin("A", "PB1").expect("rust alias");

    let seq: &[&str] = &[
        "pin=PB1",
        "pin=PB1",
        "pin=A",
        "pin=PB1 pin=A",
        "pin=A pin=PB1",
        "pin=PB1",
        "pin=PB2",
        "pin=A",
    ];
    for cmd in seq {
        let expected = py.eval(&format!("res.update_command({})", py_repr(cmd)));
        let actual = res.update_command(cmd).map(Value::Str);
        check_result(&format!("seq {cmd:?}"), expected, actual);
        resolver_state("seq state", &mut py, &res);
    }

    // With validation off, aliasing the same physical pin under two names
    // is fine and the rewritten id stays the aliased one.
    py.exec("_make_resolver(False)").expect("py resolver");
    let res = ResolverConcrete::new(false);
    py.eval("res.alias_pin('A', 'PB1')").expect("py alias");
    res.alias_pin("A", "PB1").expect("rust alias");
    for cmd in ["pin=A", "pin=PB1", "pin=A", "pin=PB1 pin=A"] {
        let expected = py.eval(&format!("res.update_command({})", py_repr(cmd)));
        let actual = res.update_command(cmd).map(Value::Str);
        check_result(&format!("novalidate {cmd:?}"), expected, actual);
        resolver_state("novalidate state", &mut py, &res);
    }
}

// ---------------------------------------------------------------------------
// Contract: `printer_object!`, `module_factory` and v2 traits.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_registers_pins_object() {
    let factory = klippy_core::pins::module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_some());

    struct FakePrinter {
        objects: GCell<IndexMap<String, ObjectRef>>,
    }
    impl PrinterApi for FakePrinter {
        fn get_start_args(&self) -> pycompat::PyValue {
            never("get_start_args")
        }
        fn get_reactor(&self) -> reactor::Reactor {
            never("get_reactor")
        }
        fn get_state_message(&self) -> (String, String) {
            never("get_state_message")
        }
        fn is_shutdown(&self) -> bool {
            never("is_shutdown")
        }
        fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
            never("update_error_msg")
        }
        fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
            if self.objects.borrow().contains_key(name) {
                return Err(KlippyError::config(format!("duplicate object {name}")));
            }
            self.objects.borrow_mut().insert(name.to_string(), obj);
            Ok(())
        }
        fn lookup_object(
            &self,
            name: &str,
            default: klippy_api::printer::LookupDefault,
        ) -> KResult<Option<ObjectRef>> {
            let found = self.objects.borrow().get(name).cloned();
            match (found, default) {
                (Some(obj), _) => Ok(Some(obj)),
                (None, klippy_api::printer::LookupDefault::ReturnNone) => Ok(None),
                (None, klippy_api::printer::LookupDefault::Raise) => Err(KlippyError::config(
                    format!("Missing config option '{name}'"),
                )),
            }
        }
        fn lookup_objects(
            &self,
            _module: Option<&str>,
        ) -> Vec<(String, ObjectRef)> {
            self.objects
                .borrow()
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect()
        }
        fn load_object(
            &self,
            _config: &klippy_api::config::Config,
            _section: &str,
            _default: klippy_api::printer::LookupDefault,
        ) -> KResult<Option<ObjectRef>> {
            never("load_object")
        }
        fn run(&self) -> Option<String> {
            never("run")
        }
        fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
            never("set_rollover_info")
        }
        fn invoke_shutdown(&self, _msg: &str, _details: pycompat::PyValue) {
            never("invoke_shutdown")
        }
        fn invoke_async_shutdown(&self, _msg: &str, _details: pycompat::PyValue) {
            never("invoke_async_shutdown")
        }
        fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {
            never("register_event_handler")
        }
        fn send_event(&self, _event: &str, _params: &[klippy_api::EventArg]) -> KResult<Vec<pycompat::PyValue>> {
            never("send_event")
        }
        fn request_exit(&self, _result: &str) {
            never("request_exit")
        }
    }

    struct FakeConfig {
        printer: Printer,
    }
    impl ConfigApi for FakeConfig {
        fn get_printer(&self) -> Printer {
            self.printer.clone()
        }
        fn get_name(&self) -> String {
            never("config get_name")
        }
        fn get(&self, _option: &str, _opts: klippy_api::config::GetStrOpts) -> KResult<Option<String>> {
            never("config get")
        }
        fn getint(&self, _option: &str, _opts: klippy_api::config::GetIntOpts) -> KResult<Option<i64>> {
            never("config getint")
        }
        fn getfloat(&self, _option: &str, _opts: klippy_api::config::GetFloatOpts) -> KResult<Option<f64>> {
            never("config getfloat")
        }
        fn getboolean(&self, _option: &str, _opts: klippy_api::config::GetBoolOpts) -> KResult<Option<bool>> {
            never("config getboolean")
        }
        fn getchoice(&self, _option: &str, _choices: &pycompat::PyValue, _opts: klippy_api::config::GetChoiceOpts) -> KResult<pycompat::PyValue> {
            never("config getchoice")
        }
        fn getlists(&self, _option: &str, _opts: klippy_api::config::GetListsOpts) -> KResult<pycompat::PyValue> {
            never("config getlists")
        }
        fn getlist(&self, _option: &str, _opts: klippy_api::config::GetListOpts) -> KResult<Option<Vec<String>>> {
            never("config getlist")
        }
        fn getintlist(&self, _option: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
            never("config getintlist")
        }
        fn getfloatlist(&self, _option: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
            never("config getfloatlist")
        }
        fn getsection(&self, _section: &str) -> klippy_api::config::Config {
            never("config getsection")
        }
        fn has_section(&self, _section: &str) -> bool {
            never("config has_section")
        }
        fn get_prefix_sections(&self, _prefix: &str) -> Vec<klippy_api::config::Config> {
            never("config get_prefix_sections")
        }
        fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
            never("config get_prefix_options")
        }
        fn deprecate(&self, _option: &str, _value: Option<pycompat::PyValue>) -> KResult<()> {
            never("config deprecate")
        }
    }

    let printer: Printer = Arc::new(FakePrinter { objects: GCell::new(IndexMap::new()) });
    let config: klippy_api::config::Config =
        Arc::new(FakeConfig { printer: printer.clone() });
    factory
        .add_printer_objects
        .expect("add_printer_objects")
        (&config)
        .expect("add_printer_objects runs");
    let obj = printer
        .lookup_object("pins", klippy_api::printer::LookupDefault::Raise)
        .expect("lookup pins")
        .expect("pins object exists");
    let pins = interface::<dyn PinsApi>(&obj).expect("pins object provides dyn PinsApi");
    let chip = FakeChip::new("mcu");
    pins.register_chip("mcu", as_chip(chip)).expect("register");
    let params = pins
        .parse_pin("PB1", ParsePinOpts::default())
        .expect("parse via interface");
    assert_eq!(params.pin, "PB1");
    assert_eq!(params.chip_name, "mcu");
    let _ = ModuleFactory::default();
    let _ = std::any::TypeId::of::<PrinterPins>();
    let _: &dyn PrinterObject = &PrinterPins::new();
}

// ---------------------------------------------------------------------------
// Rust-only sanity (runs without the oracle too).
// ---------------------------------------------------------------------------

#[test]
fn rust_only_lookup_basic() {
    let chip = FakeChip::new("mcu");
    let ppins = PrinterPins::new();
    ppins.register_chip("mcu", as_chip(chip)).expect("register");
    let params = ppins.lookup_pin("PB1", LookupPinOpts::default()).expect("lookup");
    assert_eq!(params.pin, "PB1");
    assert_eq!(params.chip_name, "mcu");
    let err = ppins
        .lookup_pin("PB1", LookupPinOpts::default())
        .err()
        .expect("duplicate must fail");
    assert_eq!(err.message(), "pin PB1 used multiple times in config");
    assert_eq!(err.class_name(), "error");
    let err = ppins
        .parse_pin("PB 1", ParsePinOpts::default())
        .err()
        .expect("space must fail");
    assert_eq!(
        err.message(),
        "Invalid pin description 'PB 1'\nFormat is: [chip_name:] pin_name"
    );
    let _resolver = ppins.get_pin_resolver("mcu").expect("resolver");
    let concrete = ppins.resolver("mcu").expect("concrete resolver");
    assert!(concrete.validate_aliases());
    // `PinsApi` / `PinResolverApi` / `ChipApi` objects must be `Send + Sync`.
    fn assert_traits<T: Send + Sync>() {}
    assert_traits::<PrinterPins>();
    assert_traits::<ResolverConcrete>();
    assert_traits::<FakeChip>();
}
