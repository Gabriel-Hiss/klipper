//! Parity tests for `extras::ads131m0x` against the Python reference using
//! fakes of the `klippy_api` contract traits and `pyoracle` for expected
//! values. Only config parsing, sample-rate selection, status, sample
//! conversion and the register transactions are covered here; the SPI/bulk
//! plumbing is exercised through a fake MCU that records command sends and
//! serves canned register responses.

#![allow(clippy::too_many_arguments)]

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use extras::ads131m0x::{
    ADS131M0X, GC_DLY_TO_REG, GAIN_TO_REG, OSR_TO_REG,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetStrOpts,
    GetListsOpts, GetListOpts, GetIntListOpts, GetFloatListOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandWrapperApi, ConfigCallback, LookupQueryCommandOpts,
    McuApi, PinsApi, QuerySendOpts, SendOpts, PinParams,
};
use klippy_api::object::ObjectRef;
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use pycompat::PyValue;
use reactor::Reactor;
use recording::Value;

fn oracle_float(py: &mut pyoracle::Oracle, expr: &str) -> f64 {
    match py.eval(expr) {
        Ok(Value::Float(f)) => f.to_f64().expect("float hex decodes"),
        other => panic!("oracle: {expr} -> {other:?}"),
    }
}

fn oracle_ints(py: &mut pyoracle::Oracle, expr: &str) -> Vec<i64> {
    match py.eval(expr) {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Int(i) => *i,
                other => panic!("oracle int list: {other:?}"),
            })
            .collect(),
        other => panic!("oracle: {expr} -> {other:?}"),
    }
}

struct FakePrinter {
    objects: Mutex<HashMap<String, ObjectRef>>,
    reactor: Reactor,
    shutdown: Mutex<bool>,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("".to_string(), "".to_string())
    }
    fn is_shutdown(&self) -> bool {
        *self.shutdown.lock().unwrap()
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().unwrap().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let map = self.objects.lock().unwrap();
        Ok(map.get(name).cloned())
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects
            .lock()
            .unwrap()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[klippy_api::object::EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    sections: HashMap<String, FakeConfigValues>,
}

#[derive(Default, Clone)]
struct FakeConfigValues {
    strings: HashMap<String, String>,
    ints: HashMap<String, i64>,
    floats: HashMap<String, f64>,
    bools: HashMap<String, bool>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.sections.get(&self.name).and_then(|s| s.strings.get(option)) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!("option {option} not found"))),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.sections.get(&self.name).and_then(|s| s.ints.get(option)) {
            Some(v) => {
                if let Some(mn) = opts.minval {
                    if *v < mn {
                        return Err(KlippyError::config(format!("{option} below minimum")));
                    }
                }
                if let Some(mx) = opts.maxval {
                    if *v > mx {
                        return Err(KlippyError::config(format!("{option} above maximum")));
                    }
                }
                Ok(Some(*v))
            }
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!("option {option} not found"))),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.sections.get(&self.name).and_then(|s| s.floats.get(option)) {
            Some(v) => {
                if let Some(mn) = opts.minval {
                    if *v < mn {
                        return Err(KlippyError::config(format!("{option} below minimum")));
                    }
                }
                if let Some(mx) = opts.maxval {
                    if *v > mx {
                        return Err(KlippyError::config(format!("{option} above maximum")));
                    }
                }
                if let Some(above) = opts.above {
                    if *v <= above {
                        return Err(KlippyError::config(format!("{option} must be above {above}")));
                    }
                }
                Ok(Some(*v))
            }
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!("option {option} not found"))),
            },
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.sections.get(&self.name).and_then(|s| s.bools.get(option)) {
            Some(v) => Ok(Some(*v)),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!("option {option} not found"))),
            },
        }
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        match self.sections.get(&self.name) {
            Some(s) => {
                if let Some(v) = s.ints.get(option) {
                    if let PyValue::List(cs) = choices {
                        if cs.iter().any(|c| matches!(c, PyValue::Int(i) if i.0.to_string() == v.to_string())) {
                            return Ok(PyValue::Int(pycompat::int::PyInt::from_i64(*v)));
                        }
                    }
                    return Err(KlippyError::config(format!(
                        "invalid choice {v} for {option}"
                    )));
                }
                match opts.default {
                    Some(d) => Ok(d.unwrap_or(PyValue::None)),
                    None => Err(KlippyError::config(format!("option {option} not found"))),
                }
            }
            None => match opts.default {
                Some(d) => Ok(d.unwrap_or(PyValue::None)),
                None => Err(KlippyError::config(format!("option {option} not found"))),
            },
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::List(Vec::new()))
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
            sections: self.sections.clone(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.contains_key(section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakePins {
    mcu: klippy_api::mcu::Mcu,
    pin_name: String,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        Ok(self.pin_params())
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        Ok(self.pin_params())
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<klippy_api::mcu::Pin> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> { Ok(()) }
}

impl FakePins {
    fn pin_params(&self) -> PinParams {
        PinParams {
            chip: self.mcu.clone(),
            chip_name: self.mcu.get_name(),
            pin: self.pin_name.clone(),
            invert: 0,
            pullup: 0,
            share_type: None,
        }
    }
}
klippy_api::printer_object!(FakePins; dyn PinsApi);

#[derive(Clone, Default)]
struct FakeMcuState {
    oid_counter: i64,
    config_cmds: Vec<String>,
    send_calls: Vec<(String, Vec<PyValue>)>,
    spi_responses: Vec<Vec<u8>>,
    fileoutput: bool,
    enumerations: IndexMap<String, IndexMap<String, i64>>,
}

struct FakeMcu {
    printer: Printer,
    name: String,
    state: Arc<Mutex<FakeMcuState>>,
    callbacks: Arc<Mutex<Vec<ConfigCallback>>>,
}

type FakeMcuHandle = Arc<FakeMcu>;

fn run_config_callbacks(mcu: &FakeMcuHandle) -> KResult<()> {
    let mut cbs = mcu.callbacks.lock().unwrap();
    for cb in cbs.iter_mut() {
        cb()?;
    }
    cbs.clear();
    Ok(())
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        self.state.lock().unwrap().fileoutput
    }
    fn setup_pin(
        &self,
        _pin_type: &str,
        _pin_params: PinParams,
    ) -> KResult<klippy_api::mcu::Pin> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut st = self.state.lock().unwrap();
        st.oid_counter += 1;
        Ok(st.oid_counter)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.callbacks.lock().unwrap().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        self.state.lock().unwrap().config_cmds.push(cmd.to_string());
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * 16000000.0) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<klippy_api::mcu::CommandWrapper> {
        Ok(Arc::new(FakeCommand {
            msg: msgformat.to_string(),
            state: self.state.clone(),
        }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Ok(Arc::new(FakeQueryCommand {
            msg: msgformat.to_string(),
            state: self.state.clone(),
        }))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        self.state.lock().unwrap().enumerations.clone()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Err(KlippyError::Mcu("no constants".to_string()))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * 16000000.0) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 16000000.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, print_time))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, "".to_string())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
}

struct FakeCommand {
    msg: String,
    state: Arc<Mutex<FakeMcuState>>,
}

impl CommandWrapperApi for FakeCommand {
    fn send(&self, data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        self.state
            .lock()
            .unwrap()
            .send_calls
            .push((self.msg.clone(), data));
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        self.state
            .lock()
            .unwrap()
            .send_calls
            .push((self.msg.clone(), data));
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeQueryCommand {
    msg: String,
    state: Arc<Mutex<FakeMcuState>>,
}

impl klippy_api::mcu::CommandQueryWrapperApi for FakeQueryCommand {
    fn send(&self, data: Vec<PyValue>, _opts: QuerySendOpts) -> KResult<PyValue> {
        let mut st = self.state.lock().unwrap();
        st.send_calls.push((self.msg.clone(), data));
        if st.spi_responses.is_empty() {
            return Ok(PyValue::Bytes(Vec::new()));
        }
        let r = st.spi_responses.remove(0);
        Ok(PyValue::Bytes(r))
    }
    fn send_with_preface(
        &self,
        _preface_cmd: klippy_api::mcu::CommandWrapper,
        _preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        self.send(data, QuerySendOpts::default())
    }
}

struct FakeAsync;
impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

fn make_config(
    printer: Printer,
    name: &str,
    mut values: FakeConfigValues,
    mcu_state: &Arc<Mutex<FakeMcuState>>,
    pin: &str,
) -> Config {
    let _ = mcu_state;
    values.strings.entry("cs_pin".to_string()).or_insert_with(|| "PINS\0".to_string());
    values.strings.entry("data_ready_pin".to_string()).or_insert_with(|| pin.to_string());
    let mut sections = HashMap::new();
    sections.insert(name.to_string(), values);
    Arc::new(FakeConfig {
        printer,
        name: name.to_string(),
        sections,
    })
}

fn make_printer_with_mcu(mcu_state: Arc<Mutex<FakeMcuState>>, pin: &str) -> (Printer, FakeMcuHandle, FakeConfigValues) {
    let reactor = Reactor::new(false);
    let printer: Printer = Arc::new(FakePrinter {
        objects: Mutex::new(HashMap::new()),
        reactor,
        shutdown: Mutex::new(false),
    });
    let mcu: FakeMcuHandle = Arc::new(FakeMcu {
        printer: printer.clone(),
        name: "mcu".to_string(),
        state: mcu_state.clone(),
        callbacks: Arc::new(Mutex::new(Vec::new())),
    });
    let pins: Arc<FakePins> = Arc::new(FakePins {
        mcu: mcu.clone(),
        pin_name: pin.to_string(),
    });
    printer
        .add_object("pins", pins as ObjectRef)
        .expect("pins object");
    let values = FakeConfigValues {
        ints: [("clock_freq".to_string(), 8192000)].into_iter().collect(),
        ..Default::default()
    };
    (printer, mcu, values)
}

#[test]
fn constants_match_python_tables() {
    // Tables are the exact dict contents from ads131m0x.py:41-83.
    assert_eq!(OSR_TO_REG.len(), 9);
    assert_eq!(OSR_TO_REG[0], (64, 8));
    assert_eq!(OSR_TO_REG[8], (16384, 7));
    assert_eq!(GAIN_TO_REG.len(), 8);
    assert_eq!(GAIN_TO_REG[7], (128, 0x07));
    assert_eq!(GC_DLY_TO_REG.len(), 16);
    assert_eq!(GC_DLY_TO_REG[15], (65536, 0x0f));
}

#[test]
fn init_defaults_and_status() {
    let state = Arc::new(Mutex::new(FakeMcuState::default()));
    let (printer, _mcu, values) = make_printer_with_mcu(state.clone(), "PINS0");
    let config = make_config(printer, "ads131m02 sensor1", values, &state, "PINS0");
    let sensor = ADS131M0X::new(&config, "ADS131M02", 2).expect("init");
    // Default sample_rate 500 Hz with clock 8192000 picks OSR 8192
    // (8192000/(2*8192) = 500.0).
    assert_eq!(sensor.get_samples_per_second(), 500.0);
    let status = sensor.get_status(0.0);
    if let PyValue::Dict(d) = &status {
        assert_eq!(d.get(&PyValue::Str("errors".to_string())), Some(&PyValue::Int(pycompat::int::PyInt::from_i64(0))));
        assert_eq!(d.get(&PyValue::Str("overflows".to_string())), Some(&PyValue::Int(pycompat::int::PyInt::from_i64(0))));
        assert_eq!(d.get(&PyValue::Str("sample_rate".to_string())), Some(&PyValue::Float(500.0)));
    } else {
        panic!("status must be a dict");
    }
    assert_eq!(sensor.get_range(), (-8388608, 8388607));
}

#[test]
fn init_rejects_unavailable_sample_rate() {
    let state = Arc::new(Mutex::new(FakeMcuState::default()));
    let (printer, _mcu, values) = make_printer_with_mcu(state.clone(), "PINS0");
    let mut values = values;
    values.floats.insert("sample_rate".to_string(), 200000.0);
    let config = make_config(printer, "ads131m02 sensor1", values, &state, "PINS0");
    let err = match ADS131M0X::new(&config, "ADS131M02", 2) {
        Ok(_) => panic!("expected sample rate error"),
        Err(e) => e,
    };
    assert_eq!(
        err.message(),
        "Requested sample rate 200000.0 Hz is not available with the configured parameters"
    );
}

#[test]
fn convert_to_spi_frame_pads_to_four_words() {
    let frame = ADS131M0X::_convert_to_spi_frame(&[0xA123]);
    assert_eq!(frame, vec![0xA1, 0x23, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    let frame5 = ADS131M0X::_convert_to_spi_frame(&[0x6000, 0x0100, 1, 2, 3]);
    assert_eq!(frame5.len(), 15);
}

#[test]
fn convert_samples_drops_bad_top_byte() {
    let state = Arc::new(Mutex::new(FakeMcuState::default()));
    let (printer, _mcu, values) = make_printer_with_mcu(state.clone(), "PINS0");
    let config = make_config(printer, "ads131m02 sensor1", values, &state, "PINS0");
    let sensor = ADS131M0X::new(&config, "ADS131M02", 2).expect("init");
    let mut samples = vec![
        PyValue::Tuple(vec![
            PyValue::Float(1.0),
            PyValue::Int(pycompat::int::PyInt::from_i64(0x00123456)),
        ]),
        PyValue::Tuple(vec![
            PyValue::Float(2.0),
            PyValue::Int(pycompat::int::PyInt::from_i64(0x7F000000)),
        ]),
    ];
    sensor._convert_samples(&mut samples).expect("convert");
    // 0x7F000000 has top_byte 0x7F which is neither 0x00 nor 0xFF.
    assert_eq!(samples.len(), 1);
}

#[test]
fn read_reg_and_write_reg_use_spi_frame() {
    let state = Arc::new(Mutex::new(FakeMcuState::default()));
    let (printer, mcu, values) = make_printer_with_mcu(state.clone(), "PINS0");
    let config = make_config(printer, "ads131m02 sensor1", values, &state, "PINS0");
    let sensor = ADS131M0X::new(&config, "ADS131M02", 2).expect("init");
    run_config_callbacks(&mcu).expect("config callbacks");
    state
        .lock()
        .unwrap()
        .spi_responses
        .push(vec![0x12, 0x34]);
    let v = sensor.read_reg(0x02).expect("read_reg");
    assert_eq!(v, 0x1234);
    sensor.write_reg(0x02, 0x0100).expect("write_reg");
}

#[test]
fn pyoracle_sample_rate_and_frame_match() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping pyoracle comparison: KLIPPY_PYTHON unset");
        return;
    };
    py.exec("import sys; sys.path.insert(0, 'klippy')").expect("oracle path");
    py.exec("import extras.ads131m0x as adsmod").expect("oracle import");
    py.exec("obj = adsmod.ADS131M0X.__new__(adsmod.ADS131M0X); obj.clock_freq = 8192000.0; obj.enable_global_chop = False; obj.gc_dly = None").expect("oracle obj");
    let expected_sps = oracle_float(&mut py, "obj._calc_sample_rate(8192)");
    let state = Arc::new(Mutex::new(FakeMcuState::default()));
    let (printer, _mcu, values) = make_printer_with_mcu(state.clone(), "PINS0");
    let config = make_config(printer, "ads131m02 sensor1", values, &state, "PINS0");
    let sensor = ADS131M0X::new(&config, "ADS131M02", 2).expect("init");
    assert_eq!(expected_sps.to_bits(), sensor.get_samples_per_second().to_bits());
    let expected_frame = oracle_ints(&mut py, "obj._convert_to_spi_frame([0xA123])");
    let frame = ADS131M0X::_convert_to_spi_frame(&[0xA123]);
    let frame_vals: Vec<i64> = frame.iter().map(|b| *b as i64).collect();
    assert_eq!(expected_frame, frame_vals);
}

