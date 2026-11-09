//! Parity tests for `klippy/extras/adc_scaled.py`.
//!
//! The port is driven through fakes of the `klippy_api` contract traits
//! (`PrinterApi`, `ConfigApi`, `McuApi`, `PinsApi`, `AdcPinApi`) with the
//! same inputs given to Python fakes running on the live reference through
//! `pyoracle` (the test skips when `$KLIPPY_PYTHON` is unset). Compared:
//! `calc_smooth` results (floats by hex + repr spelling), the smoothed
//! `last_vref` / `last_vssa` after ADC callbacks, the scaled samples and
//! `get_last_value` of `MCU_scaled_adc`, config parsing (`smooth_time`
//! default 2.0 with `above=0.`, section-name token), the wiring
//! (`setup_adc_sample` arguments, `query_adc` registration names, chip
//! registration) and the error paths (non-ADC pin type, vref/vssa on
//! different MCUs, single-token section name).

use extras::adc_scaled::{calc_smooth, load_config_prefix, PrinterADCScaled, REPORT_TIME, SAMPLE_COUNT, SAMPLE_TIME};
use extras::query_adc::QueryAdcApi;
use klippy_api::config::{ConfigApi, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{
    AdcCallback, AdcPin, AdcPinApi, AdcSampleOpts, ChipApi, CommandQueue, CommandQueryWrapper, CommandWrapper,
    LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, PinsApi,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use pycompat::PyValue;
use reactor::Reactor;
use recording::value::{FloatVal, Value};
use std::collections::BTreeMap;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Fakes of the contract traits
// ---------------------------------------------------------------------------

struct FakePrinter {
    pins: Option<ObjectRef>,
    query: Option<ObjectRef>,
}

fn unknown_object(name: &str) -> KlippyError {
    KlippyError::config(format!("Unknown config object '{name}'"))
}

impl FakePrinter {
    fn find(&self, name: &str) -> Option<ObjectRef> {
        match name {
            "pins" => self.pins.clone(),
            "query_adc" => self.query.clone(),
            _ => None,
        }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        panic!("not used")
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        panic!("not used")
    }
    fn is_shutdown(&self) -> bool {
        panic!("not used")
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
        panic!("not used")
    }
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        panic!("not used")
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.find(name) {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(unknown_object(name)),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        panic!("not used")
    }
    fn load_object(
        &self,
        _config: &klippy_api::config::Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.find(section) {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(unknown_object(section)),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn run(&self) -> Option<String> {
        panic!("not used")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        panic!("not used")
    }
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("not used")
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("not used")
    }
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {
        panic!("not used")
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("not used")
    }
    fn request_exit(&self, _result: &str) {
        panic!("not used")
    }
}

fn fake_printer() -> Printer {
    Arc::new(FakePrinter { pins: None, query: None })
}

struct FakeConfig {
    printer: Printer,
    section: String,
    opts: BTreeMap<String, String>,
    floats: BTreeMap<String, f64>,
    /// `getfloat` calls as `(option, default, above)`, for assertions.
    getfloat_calls: GCell<Vec<(String, Option<f64>, Option<f64>)>>,
}

impl FakeConfig {
    fn new_with_printer(printer: Printer, section: &str, opts: &[(&str, &str)], floats: &[(&str, f64)]) -> Arc<Self> {
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            opts: opts.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
            floats: floats.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
            getfloat_calls: GCell::new(Vec::new()),
        })
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: klippy_api::config::GetIntOpts) -> KResult<Option<i64>> {
        panic!("not used")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.getfloat_calls
            .borrow_mut()
            .push((option.to_string(), opts.default.and_then(|d| d), opts.above));
        let value = match self.floats.get(option) {
            Some(v) => *v,
            None => match opts.default {
                Some(d) => d.unwrap_or(0.),
                None => return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
            },
        };
        if let Some(above) = opts.above {
            if value <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {above}",
                    self.section
                )));
            }
        }
        Ok(Some(value))
    }
    fn getboolean(&self, _option: &str, _opts: klippy_api::config::GetBoolOpts) -> KResult<Option<bool>> {
        panic!("not used")
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: klippy_api::config::GetChoiceOpts) -> KResult<PyValue> {
        panic!("not used")
    }
    fn getlists(&self, _option: &str, _opts: klippy_api::config::GetListsOpts) -> KResult<PyValue> {
        panic!("not used")
    }
    fn getlist(&self, _option: &str, _opts: klippy_api::config::GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("not used")
    }
    fn getintlist(&self, _option: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("not used")
    }
    fn getfloatlist(&self, _option: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("not used")
    }
    fn getsection(&self, _section: &str) -> klippy_api::config::Config {
        panic!("not used")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("not used")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<klippy_api::config::Config> {
        panic!("not used")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("not used")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> {
        panic!("not used")
    }
}

struct FakeAdcPin {
    mcu: Mcu,
    callback: GCell<Option<AdcCallback>>,
    sample_args: GCell<Option<(f64, AdcSampleOpts)>>,
}

impl FakeAdcPin {
    fn new(mcu: Mcu) -> Arc<Self> {
        Arc::new(FakeAdcPin { mcu, callback: GCell::new(None), sample_args: GCell::new(None) })
    }

    /// Drive the pin with a sample batch, as the MCU would.
    fn drive(&self, samples: Vec<(f64, f64)>) {
        let mut callback = self.callback.borrow_mut().take();
        if let Some(cb) = callback.as_mut() {
            cb(samples);
        }
        if callback.is_some() {
            *self.callback.borrow_mut() = callback;
        }
    }
}

impl AdcPinApi for FakeAdcPin {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        *self.sample_args.borrow_mut() = Some((report_time, opts));
    }
    fn setup_adc_callback(&self, callback: AdcCallback) {
        *self.callback.borrow_mut() = Some(callback);
    }
    fn get_last_value(&self) -> (f64, f64) {
        (0., 0.)
    }
}

struct FakeMcu {
    name: String,
    /// ADC pins created by `setup_pin`, by pin name (insertion order kept
    /// in a Vec for determinism).
    adcs: GCell<Vec<(String, Arc<FakeAdcPin>)>>,
}

impl FakeMcu {
    fn new(name: &str) -> Arc<Self> {
        Arc::new(FakeMcu { name: name.to_string(), adcs: GCell::new(Vec::new()) })
    }

    fn adc(&self, pin: &str) -> Arc<FakeAdcPin> {
        self.adcs.borrow().iter().find(|(n, _)| n == pin).map(|(_, a)| Arc::clone(a)).expect("adc pin exists")
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        fake_printer()
    }
    fn is_fileoutput(&self) -> bool {
        panic!("not used")
    }
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        assert_eq!(pin_type, "adc");
        // `self` by value is unavailable through the trait object; the pin
        // holds a fresh fake MCU handle created by the caller through
        // `pin_params.chip` identity: tests build pins with this MCU as
        // chip, and `get_mcu` must return an Arc equal by pointer to the
        // chip. Recover it by cloning `pin_params.chip` (the only MCU the
        // test wires in).
        let chip = pin_params.chip.clone();
        let adc = FakeAdcPin::new(chip);
        self.adcs.borrow_mut().push((pin_params.pin.clone(), Arc::clone(&adc)));
        Ok(Pin::Adc(adc))
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("not used")
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        panic!("not used")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        panic!("not used")
    }
    fn request_move_queue_slot(&self) {
        panic!("not used")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("not used")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("not used")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("not used")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("not used")
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("not used")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("not used")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("not used")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("not used")
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("not used")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("not used")
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        panic!("not used")
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        panic!("not used")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("not used")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("not used")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("not used")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("not used")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("not used")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("not used")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("not used")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("not used")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("not used")
    }
}

struct FakePins {
    mcu: Mcu,
    mcu2: Option<Mcu>,
    chips: GCell<Vec<(String, klippy_api::mcu::Chip)>>,
    setup_calls: GCell<Vec<(String, String)>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        panic!("not used")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        panic!("not used")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.setup_calls.borrow_mut().push((pin_type.to_string(), pin_desc.to_string()));
        // The second pin (vssa) is wired to a second MCU when `mcu2` is set,
        // to exercise the same-MCU check.
        let chip = if pin_desc == "VSSA" {
            self.mcu2.clone().unwrap_or_else(|| self.mcu.clone())
        } else {
            self.mcu.clone()
        };
        let params = PinParams {
            chip,
            chip_name: "mcu".to_string(),
            pin: pin_desc.to_string(),
            invert: 0,
            pullup: 0,
            share_type: None,
        };
        let chip = params.chip.clone();
        chip.setup_pin(pin_type, params)
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { panic!("not used") }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("not used")
    }
    fn register_chip(&self, chip_name: &str, chip: klippy_api::mcu::Chip) -> KResult<()> {
        self.chips.borrow_mut().push((chip_name.to_string(), chip));
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("not used")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeQueryAdc {
    registered: GCell<Vec<(String, AdcPin)>>,
}

impl QueryAdcApi for FakeQueryAdc {
    fn register_adc(&self, name: &str, adc: AdcPin) {
        self.registered.borrow_mut().push((name.to_string(), adc));
    }
}

klippy_api::printer_object!(FakeQueryAdc; dyn QueryAdcApi);

/// Build a wired world: one MCU, pins, query_adc, and the port object.
struct World {
    mcu: Arc<FakeMcu>,
    pins_obj: Arc<FakePins>,
    query: Arc<FakeQueryAdc>,
    obj: Arc<PrinterADCScaled>,
}

fn world(smooth_time: f64, mcu2: Option<Mcu>) -> World {
    let mcu = FakeMcu::new("mcu");
    let mcu_handle: Mcu = mcu.clone();
    let pins_obj = Arc::new(FakePins {
        mcu: mcu_handle,
        mcu2,
        chips: GCell::new(Vec::new()),
        setup_calls: GCell::new(Vec::new()),
    });
    let pins_ref: ObjectRef = pins_obj.clone() as ObjectRef;
    let query = Arc::new(FakeQueryAdc { registered: GCell::new(Vec::new()) });
    let query_ref: ObjectRef = query.clone() as ObjectRef;
    let printer: Printer = Arc::new(FakePrinter { pins: Some(pins_ref), query: Some(query_ref) });
    let config_obj = FakeConfig::new_with_printer(
        printer,
        "adc_scaled foo",
        &[("vref_pin", "VREF"), ("vssa_pin", "VSSA")],
        &[("smooth_time", smooth_time)],
    );
    let config: klippy_api::config::Config = config_obj.clone();
    let obj = PrinterADCScaled::new(&config).expect("construct adc_scaled");
    World { mcu, pins_obj, query, obj }
}

// ---------------------------------------------------------------------------
// Direct behaviour tests
// ---------------------------------------------------------------------------

fn assert_float(label: &str, expected: f64, actual: f64) {
    assert_eq!(FloatVal::new(expected), FloatVal::new(actual), "{label}");
}

#[test]
fn calc_smooth_matches_python_semantics() {
    // (inv_smooth_time, read_time, read_value, last) - exercised against the
    // oracle below as well.
    let cases = [
        (0.5, 1.5, 0.75, (1.0, 0.25)),
        (0.5, 1.0, 0.75, (1.0, 0.25)),
        (0.5, 3.0, 1.0, (1.0, 0.0)),
        (1.0, 0.1, 0.3, (0.0, 0.0)),
        (0.25, 100.0, 2.0, (99.5, 1.0)),
        (2.0, 1.001, 0.999, (1.0, 1.0)),
    ];
    for (inv, t, v, last) in cases {
        let (time, value) = calc_smooth(inv, t, v, last);
        let time_diff = t - last.0;
        let value_diff = v - last.1;
        let scaled = time_diff * inv;
        let adj = if 1. < scaled { 1. } else { scaled };
        assert_float("time", t, time);
        assert_float("value", last.1 + value_diff * adj, value);
    }
}

#[test]
fn constructor_wiring() {
    let w = world(2.0, None);
    assert_eq!(w.obj.name(), "foo");
    // Pins set up in vref, vssa order with the sampling constants.
    for pin in ["VREF", "VSSA"] {
        let (report_time, opts) = w.mcu.adc(pin).sample_args.borrow().clone().expect("sample args set");
        assert_float("report_time", REPORT_TIME, report_time);
        assert_float("sample_time", SAMPLE_TIME, opts.sample_time);
        assert_eq!(opts.sample_count, SAMPLE_COUNT);
    }
    // query_adc registration names.
    let names: Vec<String> = w.query.registered.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(names, ["foo:vref", "foo:vssa"]);
    // Chip registration under the section name.
    let chips: Vec<String> = w.pins_obj.chips.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(chips, ["foo"]);
    // Pins set up in vref, vssa order.
    assert_eq!(
        w.pins_obj.setup_calls.borrow().clone(),
        [("adc".to_string(), "VREF".to_string()), ("adc".to_string(), "VSSA".to_string())]
    );
    // Same MCU object (the fake MCU is the chip of both pins).
    assert_eq!(w.obj.mcu().get_name(), "mcu");
}


#[test]
fn smoothing_callbacks_update_last_values() {
    let w = world(2.0, None);
    assert_eq!(w.obj.last_vref(), (0., 0.));
    assert_eq!(w.obj.last_vssa(), (0., 0.));
    // vref: samples[-1] wins; time_diff 1.0 * inv 0.5 = 0.5 smoothing.
    w.mcu.adc("VREF").drive(vec![(0.5, 0.4), (1.0, 0.8)]);
    assert_float("vref time", 1.0, w.obj.last_vref().0);
    assert_float("vref value", 0.4, w.obj.last_vref().1);
    // vssa: time_diff 4.0 * 0.5 >= 1, fully smoothed to the new value.
    w.mcu.adc("VSSA").drive(vec![(4.0, 0.2)]);
    assert_float("vssa time", 4.0, w.obj.last_vssa().0);
    assert_float("vssa value", 0.2, w.obj.last_vssa().1);
    // Second vref sample: time_diff 0.5 * 0.5 = 0.25 toward 0.9.
    w.mcu.adc("VREF").drive(vec![(1.5, 0.9)]);
    assert_float("vref value 2", 0.4 + (0.9 - 0.4) * 0.25, w.obj.last_vref().1);
}

#[test]
fn scaled_pin_scales_between_vref_and_vssa() {
    let w = world(2.0, None);
    // Converge vref/vssa to known values (full smoothing).
    w.mcu.adc("VREF").drive(vec![(10.0, 1.0)]);
    w.mcu.adc("VSSA").drive(vec![(10.0, 0.0)]);
    let params = PinParams {
        chip: w.mcu.clone(),
        chip_name: "mcu".to_string(),
        pin: "SENSE".to_string(),
        invert: 0,
        pullup: 0,
        share_type: None,
    };
    let pin = ChipApi::setup_pin(&*w.obj, "adc", params).expect("scaled adc pin");
    let Pin::Adc(scaled) = pin else { panic!("expected adc pin") };
    // Register a collecting callback, then drive the underlying pin.
    let received: Arc<GCell<Vec<(f64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let received2 = Arc::clone(&received);
    scaled.setup_adc_callback(Box::new(move |samples| {
        received2.borrow_mut().extend(samples);
    }));
    w.mcu.adc("SENSE").drive(vec![(11.0, 0.25), (12.0, 0.75)]);
    let got = received.borrow().clone();
    assert_eq!(got.len(), 2);
    assert_float("scaled t0", 11.0, got[0].0);
    assert_float("scaled v0", 0.25, got[0].1);
    assert_float("scaled v1", 0.75, got[1].1);
    // get_last_value is the last adjusted sample.
    let last = scaled.get_last_value();
    assert_float("last t", 12.0, last.0);
    assert_float("last v", 0.75, last.1);
    // query_adc registration for the scaled pin.
    let names: Vec<String> = w.query.registered.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(names, ["foo:vref", "foo:vssa", "foo:SENSE"]);
}

#[test]
fn setup_pin_rejects_non_adc() {
    let w = world(2.0, None);
    let params = PinParams {
        chip: w.mcu.clone(),
        chip_name: "mcu".to_string(),
        pin: "X".to_string(),
        invert: 0,
        pullup: 0,
        share_type: None,
    };
    let err = match ChipApi::setup_pin(&*w.obj, "pwm", params) {
        Ok(_) => panic!("non-adc rejected"),
        Err(e) => e,
    };
    assert_eq!(err, KlippyError::Config("adc_scaled only supports adc pins".to_string()));
}

#[test]
fn different_mcus_rejected() {
    let mcu2: Mcu = FakeMcu::new("mcu2");
    let mcu = FakeMcu::new("mcu");
    let mcu_handle: Mcu = mcu.clone();
    let pins_obj = Arc::new(FakePins {
        mcu: mcu_handle,
        mcu2: Some(mcu2),
        chips: GCell::new(Vec::new()),
        setup_calls: GCell::new(Vec::new()),
    });
    let pins_ref: ObjectRef = pins_obj.clone() as ObjectRef;
    let query = Arc::new(FakeQueryAdc { registered: GCell::new(Vec::new()) });
    let query_ref: ObjectRef = query.clone() as ObjectRef;
    let printer: Printer = Arc::new(FakePrinter { pins: Some(pins_ref), query: Some(query_ref) });
    let config_obj = FakeConfig::new_with_printer(
        printer,
        "adc_scaled foo",
        &[("vref_pin", "VREF"), ("vssa_pin", "VSSA")],
        &[],
    );
    let config: klippy_api::config::Config = config_obj.clone();
    let err = match PrinterADCScaled::new(&config) {
        Ok(_) => panic!("different MCUs rejected"),
        Err(e) => e,
    };
    assert_eq!(err, KlippyError::Config("vref and vssa must be on same mcu".to_string()));
}
fn build_obj(
    opts: &[(&str, &str)],
    floats: &[(&str, f64)],
) -> (Arc<FakeMcu>, Arc<FakeConfig>, klippy_api::config::Config, Arc<PrinterADCScaled>) {
    let mcu = FakeMcu::new("mcu");
    let mcu_handle: Mcu = mcu.clone();
    let pins_obj = Arc::new(FakePins {
        mcu: mcu_handle,
        mcu2: None,
        chips: GCell::new(Vec::new()),
        setup_calls: GCell::new(Vec::new()),
    });
    let pins_ref: ObjectRef = pins_obj.clone() as ObjectRef;
    let query = Arc::new(FakeQueryAdc { registered: GCell::new(Vec::new()) });
    let query_ref: ObjectRef = query.clone() as ObjectRef;
    let printer: Printer = Arc::new(FakePrinter { pins: Some(pins_ref), query: Some(query_ref) });
    let mut all_opts: Vec<(&str, &str)> = vec![("vref_pin", "VREF"), ("vssa_pin", "VSSA")];
    all_opts.extend_from_slice(opts);
    let config_obj = FakeConfig::new_with_printer(printer, "adc_scaled foo", &all_opts, floats);
    let config: klippy_api::config::Config = config_obj.clone();
    let obj = PrinterADCScaled::new(&config).expect("build obj");
    (mcu, config_obj, config, obj)
}


#[test]
fn config_parsing() {
    // Name is the second whitespace token of the section.
    assert_eq!(PrinterADCScaled::parse_name("adc_scaled foo").unwrap(), "foo");
    assert_eq!(PrinterADCScaled::parse_name("adc_scaled  bar baz").unwrap(), "bar");
    let err = PrinterADCScaled::parse_name("adc_scaled").expect_err("single token");
    assert_eq!(err.class_name(), "IndexError");
    assert_eq!(err.message(), "list index out of range");

    // PrinterADCScaled::new: defaults smooth_time to 2.0, reads pins from config.
    let (mcu, config_obj, _config, obj) = build_obj(&[], &[]);
    assert_eq!(obj.name(), "foo");
    // Default smooth_time 2.0 -> inv 0.5, checked through a callback.
    mcu.adc("VREF").drive(vec![(1.0, 0.8)]);
    assert_float("default smooth", 0.4, obj.last_vref().1);
    // getfloat was called with default 2.0 and above 0.0.
    let calls = config_obj.getfloat_calls.borrow().clone();
    assert_eq!(calls, [("smooth_time".to_string(), Some(2.0), Some(0.0))]);

    // Explicit smooth_time.
    let (mcu, _config_obj, _config, obj) = build_obj(&[], &[("smooth_time", 4.0)]);
    mcu.adc("VREF").drive(vec![(1.0, 0.8)]);
    assert_float("explicit smooth", 0.2, obj.last_vref().1);

    // `load_config_prefix` builds the same object through the printer.
    let (_mcu, _config_obj, config, _obj) = build_obj(&[], &[("smooth_time", 4.0)]);
    let loaded = load_config_prefix(&config).expect("load_config_prefix");
    let loaded = loaded.expect("object");
    let down: Arc<PrinterADCScaled> = klippy_api::downcast_arc::<PrinterADCScaled>(&loaded).expect("downcast");
    assert_eq!(down.name(), "foo");
}

// ---------------------------------------------------------------------------
// Oracle comparison against the reference Python
// ---------------------------------------------------------------------------

const ORACLE_SETUP: &str = r#"
from extras.adc_scaled import PrinterADCScaled, MCU_scaled_adc
class Error(Exception):
    pass
class FakeAdc:
    def __init__(self, mcu, pin='P'):
        self.mcu = mcu; self.pin = pin; self.callback = None; self.sample_args = None
    def get_mcu(self): return self.mcu
    def setup_adc_sample(self, report_time, sample_time, sample_count):
        self.sample_args = (report_time, sample_time, sample_count)
    def setup_adc_callback(self, cb): self.callback = cb
    def get_last_value(self): return (0., 0.)
class FakeMcu:
    def __init__(self): self.adcs = {}
    def setup_pin(self, pin_type, pin_params):
        pin = pin_params['pin']
        adc = FakeAdc(self, pin); self.adcs[pin] = adc; return adc
class FakeQuery:
    def __init__(self): self.reg = {}
    def register_adc(self, name, adc): self.reg[name] = adc
class FakePins:
    def __init__(self, mcu): self.mcu = mcu; self.chips = {}
    def setup_pin(self, pin_type, pin_desc): return self.mcu.setup_pin(pin_type, {'pin': pin_desc})
    def register_chip(self, name, chip): self.chips[name] = chip
class FakePrinter:
    def __init__(self, pins, query): self.pins = pins; self.query = query
    def lookup_object(self, name): return {'pins': self.pins, 'query_adc': self.query}[name]
    def load_object(self, config, section): return self.query
    def config_error(self, msg): raise Error(msg)
class FakeConfig:
    def __init__(self, printer, opts=None):
        self.printer = printer; self.opts = opts or {}
    def get_printer(self): return self.printer
    def get_name(self): return 'adc_scaled foo'
    def get(self, name): return self.opts[name]
    def getfloat(self, name, default, above=None):
        v = self.opts.get(name, default)
        if above is not None and v <= above: raise Error('must be above %f' % (above,))
        return v
    def error(self, msg): raise Error(msg)
_mcu = FakeMcu(); _query = FakeQuery(); _pins = FakePins(_mcu)
_printer = FakePrinter(_pins, _query)
_cfg = FakeConfig(_printer, {'vref_pin': 'VREF', 'vssa_pin': 'VSSA'})
obj = PrinterADCScaled(_cfg)
"#;

fn oracle_float(py: &mut pyoracle::Oracle, expr: &str) -> f64 {
    match py.eval(expr) {
        Ok(Value::Float(f)) => f.to_f64().expect("float hex decodes"),
        other => panic!("oracle: {expr} -> {other:?}"),
    }
}

#[test]
fn oracle_matches_port() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping oracle comparison: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec(ORACLE_SETUP).expect("oracle setup");

    // calc_smooth over a grid.
    let w = world(2.0, None);
    let cases = [
        (1.5, 0.75, (1.0, 0.25)),
        (1.0, 0.75, (1.0, 0.25)),
        (3.0, 1.0, (1.0, 0.0)),
        (0.1, 0.3, (0.0, 0.0)),
        (100.0, 2.0, (99.5, 1.0)),
        (1.001, 0.999, (1.0, 1.0)),
        (0.0, 5.0, (10.0, 3.0)),
    ];
    for (t, v, last) in cases {
        let expected = oracle_float(&mut py, &format!("obj.calc_smooth({t:?}, {v:?}, {last:?})[1]"));
        let actual = w.obj.calc_smooth(t, v, last).1;
        assert_float(&format!("calc_smooth({t}, {v}, {last:?})"), expected, actual);
    }

    // Smoothing callbacks: drive both implementations with the same batches.
    let batches = [vec![(0.5, 0.4), (1.0, 0.8)], vec![(1.5, 0.9)], vec![(2.0, 0.1)]];
    for batch in &batches {
        let list = format!("{batch:?}");
        py.eval(&format!("_mcu.adcs['VREF'].callback({list})")).expect("drive vref");
        w.mcu.adc("VREF").drive(batch.clone());
        let expected = oracle_float(&mut py, "obj.last_vref[1]");
        assert_float("last_vref", expected, w.obj.last_vref().1);
    }

    // Scaled pin: converge vref/vssa, then scale a batch.
    py.eval("_mcu.adcs['VREF'].callback([(10.0, 1.0)])").expect("vref converge");
    py.eval("_mcu.adcs['VSSA'].callback([(10.0, 0.0)])").expect("vssa converge");
    w.mcu.adc("VREF").drive(vec![(10.0, 1.0)]);
    w.mcu.adc("VSSA").drive(vec![(10.0, 0.0)]);

    py.exec(
        "collected = []\n\
         scaled = MCU_scaled_adc(obj, {'pin': 'SENSE'})\n\
         scaled.setup_adc_callback(lambda s: collected.extend(s))",
    )
    .expect("oracle scaled pin");
    let params = PinParams {
        chip: w.mcu.clone(),
        chip_name: "mcu".to_string(),
        pin: "SENSE".to_string(),
        invert: 0,
        pullup: 0,
        share_type: None,
    };
    let pin = ChipApi::setup_pin(&*w.obj, "adc", params).expect("rust scaled pin");
    let Pin::Adc(scaled) = pin else { panic!("expected adc pin") };
    let received: Arc<GCell<Vec<(f64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let received2 = Arc::clone(&received);
    scaled.setup_adc_callback(Box::new(move |samples| received2.borrow_mut().extend(samples)));

    py.eval("_mcu.adcs['SENSE'].callback([(11.0, 0.25), (12.0, 0.75)])").expect("drive sense");
    w.mcu.adc("SENSE").drive(vec![(11.0, 0.25), (12.0, 0.75)]);
    let expected_last = oracle_float(&mut py, "scaled.get_last_value()[1]");
    assert_float("scaled last", expected_last, scaled.get_last_value().1);
    let expected_collected: Vec<f64> = match py.eval("collected") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|item| match item {
                Value::Tuple(parts) => match &parts[1] {
                    Value::Float(f) => f.to_f64().expect("float"),
                    other => panic!("collected value {other:?}"),
                },
                other => panic!("collected item {other:?}"),
            })
            .collect(),
        other => panic!("collected {other:?}"),
    };
    let actual_collected: Vec<f64> = received.borrow().iter().map(|s| s.1).collect();
    assert_eq!(expected_collected.len(), actual_collected.len());
    for (e, a) in expected_collected.iter().zip(actual_collected.iter()) {
        assert_float("scaled sample", *e, *a);
    }

    // Error paths: exception type and message match.
    let err = py.eval("obj.setup_pin('pwm', {'pin': 'X'})");
    match err {
        Err(e) => {
            assert_eq!(e.class_name, "Error");
            assert_eq!(e.message, "adc_scaled only supports adc pins");
        }
        other => panic!("expected oracle error, got {other:?}"),
    }
    let err = match ChipApi::setup_pin(&*w.obj, "pwm", PinParams {
        chip: w.mcu.clone(),
        chip_name: "mcu".to_string(),
        pin: "X".to_string(),
        invert: 0,
        pullup: 0,
        share_type: None,
    }) {
        Ok(_) => panic!("rust non-adc"),
        Err(e) => e,
    };
    assert_eq!(err.class_name(), "Error");
    assert_eq!(err.message(), "adc_scaled only supports adc pins");

    // Sampling constants recorded by the Python side.
    let sample_args = py.eval("_mcu.adcs['VREF'].sample_args").expect("oracle sample args");
    assert_eq!(
        sample_args,
        Value::Tuple(vec![Value::float(REPORT_TIME), Value::float(SAMPLE_TIME), Value::int(SAMPLE_COUNT)])
    );
    // query_adc registration names on the Python side.
    let reg_names = py.eval("sorted(_query.reg.keys())").expect("oracle reg names");
    assert_eq!(reg_names, Value::List(vec![Value::str("foo:SENSE"), Value::str("foo:vref"), Value::str("foo:vssa")]));
}
