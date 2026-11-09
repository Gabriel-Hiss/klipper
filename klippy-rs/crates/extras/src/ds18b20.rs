//! Port of the `ds18b20` module (`klippy/extras/ds18b20.py`).

use crate::heaters::{
    PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi,
};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{get_printer_mcu, AddConfigCmdOpts, ConfigCallback, Mcu};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{load_interface, GCell, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, round_py_nd, PctValue, PyError};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

/// `DS18_REPORT_TIME = 3.0` (`klippy/extras/ds18b20.py:9`).
pub const DS18_REPORT_TIME: f64 = 3.0;
/// `DS18_MIN_REPORT_TIME = 1.0` (`klippy/extras/ds18b20.py:12`).
pub const DS18_MIN_REPORT_TIME: f64 = 1.0;
/// `DS18_MAX_CONSECUTIVE_ERRORS = 4` (`klippy/extras/ds18b20.py:13`).
pub const DS18_MAX_CONSECUTIVE_ERRORS: i64 = 4;


fn pct_fail(e: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

fn dict_get<'a>(params: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match params {
        PyValue::Dict(d) => match d.get(&PyValue::Str(key.to_string())) {
            Some(v) => Ok(v),
            None => Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: format!("'{key}'"),
            }),
        },
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "'{}' object is not subscriptable",
                pycompat::value::type_name(other)
            ),
        }),
    }
}

fn value_as_f64(v: &PyValue) -> KResult<f64> {
    match v.to_f64() {
        Some(f) => Ok(f),
        None => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "unsupported operand type(s) for /: '{}' and 'float'",
                pycompat::value::type_name(v)
            ),
        }),
    }
}

fn value_as_i64(v: &PyValue) -> KResult<i64> {
    let n = match v {
        PyValue::Int(n) => n.clone(),
        PyValue::Bool(b) => PyInt::from_i64(if *b { 1 } else { 0 }),
        f => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!(
                    "unsupported operand type(s) for -: '{}' and 'int'",
                    pycompat::value::type_name(f)
                ),
            });
        }
    };
    // Values come from `%u`/`%i`/`%c` firmware fields and fit `i64`;
    // use the exact big-int conversion path via string to avoid
    // relying on `as` for float values.
    let s = n.inner().to_string();
    s.parse::<i64>().map_err(|_| KlippyError::Internal {
        class_name: "OverflowError".to_string(),
        message: "Python int too large to convert to C long".to_string(),
    })
}

fn value_as_pct(v: &PyValue) -> KResult<PctValue> {
    match v {
        PyValue::Int(n) => Ok(PctValue::Int(n.clone())),
        PyValue::Float(f) => Ok(PctValue::Float(*f)),
        PyValue::Bool(b) => Ok(PctValue::Bool(*b)),
        PyValue::Str(s) => Ok(PctValue::Str(s.clone())),
        PyValue::Bytes(b) => Ok(PctValue::Bytes(b.clone())),
        PyValue::None => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "%d format: a real number is required, not NoneType".to_string(),
        }),
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "%d format: a real number is required, not {}",
                pycompat::value::type_name(other)
            ),
        }),
    }
}

/// `DS18B20` (`klippy/extras/ds18b20.py:15`).
pub struct DS18B20 {
    printer: Printer,
    name: String,
    sensor_id: Vec<u8>,
    temp: GCell<f64>,
    min_temp: GCell<f64>,
    max_temp: GCell<f64>,
    report_clock: GCell<i64>,
    report_time: f64,
    mcu: Mcu,
    oid: i64,
    callback: GCell<Option<TemperatureCallback>>,
}

impl DS18B20 {
    /// `DS18B20.__init__(config)` (`klippy/extras/ds18b20.py:16`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let full_name = config.get_name();
        let name = match full_name.split_whitespace().last() {
            Some(s) => s.to_string(),
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                });
            }
        };
        let serial_no = match config.get("serial_no", GetStrOpts::default())? {
            Some(s) => s,
            None => {
                return Err(KlippyError::Config(format!(
                    "Option 'serial_no' in section '{}' must be specified",
                    full_name
                )));
            }
        };
        let sensor_id = serial_no.as_bytes().to_vec();
        let report_time = match config.getfloat(
            "ds18_report_time",
            GetFloatOpts {
                default: Some(Some(DS18_REPORT_TIME)),
                minval: Some(DS18_MIN_REPORT_TIME),
                maxval: None,
                above: None,
                below: None,
                note_valid: true,
                ..Default::default()
            },
        )? {
            Some(v) => v,
            None => DS18_REPORT_TIME,
        };
        let sensor_mcu = match config.get("sensor_mcu", GetStrOpts::default())? {
            Some(s) => s,
            None => {
                return Err(KlippyError::Config(format!(
                    "Option 'sensor_mcu' in section '{}' must be specified",
                    full_name
                )));
            }
        };
        let mcu = get_printer_mcu(&printer, &sensor_mcu)?;
        let oid = mcu.create_oid()?;

        let this = Arc::new(DS18B20 {
            printer,
            name,
            sensor_id,
            temp: GCell::new(0.0),
            min_temp: GCell::new(0.0),
            max_temp: GCell::new(0.0),
            report_clock: GCell::new(0),
            report_time,
            mcu,
            oid,
            callback: GCell::new(None),
        });
        let weak_serial = Arc::downgrade(&this);
        let serial_cb: klippy_api::mcu::SerialResponseCallback = Arc::new(move |params| {
            if let Some(s) = weak_serial.upgrade() {
                if let Err(e) = s.handle_response(&params) {
                    e.raise();
                }
            }
        });
        let mcu_for_serial = this.mcu.clone();
        mcu_for_serial.register_serial_response(
            serial_cb,
            "ds18b20_result oid=%c next_clock=%u value=%i fault=%u",
            Some(this.oid),
        );
        let weak_config = Arc::downgrade(&this);
        let config_cb: ConfigCallback = Box::new(move || {
            if let Some(s) = weak_config.upgrade() {
                s.build_config()
            } else {
                Ok(())
            }
        });
        this.mcu.register_config_callback(config_cb)?;
        Ok(this)
    }

    /// Short sensor name (`self.name`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Serial-number bytes (`self.sensor_id`).
    pub fn sensor_id(&self) -> &[u8] {
        &self.sensor_id
    }

    /// MCU object id (`self.oid`).
    pub fn oid(&self) -> i64 {
        self.oid
    }

    /// `_build_config` (`klippy/extras/ds18b20.py:34`).
    pub fn build_config(&self) -> KResult<()> {
        let mut sid = String::new();
        for &byte in &self.sensor_id {
            let s = percent_format("%02x", &[PctValue::Int(PyInt::from_i64(i64::from(byte)))])
                .map_err(pct_fail)?;
            sid.push_str(&s);
        }
        let cmd = percent_format(
            "config_ds18b20 oid=%d serial=%s max_error_count=%d",
            &[
                PctValue::Int(PyInt::from_i64(self.oid)),
                PctValue::Str(sid),
                PctValue::Int(PyInt::from_i64(DS18_MAX_CONSECUTIVE_ERRORS)),
            ],
        )
        .map_err(pct_fail)?;
        self.mcu.add_config_cmd(
            &cmd,
            AddConfigCmdOpts {
                is_init: false,
                on_restart: false,
            },
        )?;

        let clock = self.mcu.get_query_slot(self.oid);
        let report_clock = self.mcu.seconds_to_clock(self.report_time);
        self.report_clock.set(report_clock);
        let min_temp = self.min_temp.get();
        let max_temp = self.max_temp.get();
        let min_value = min_temp * 1000.0;
        let max_value = max_temp * 1000.0;
        let cmd = percent_format(
            "query_ds18b20 oid=%d clock=%u rest_ticks=%u min_value=%d max_value=%d",
            &[
                PctValue::Int(PyInt::from_i64(self.oid)),
                PctValue::Int(PyInt::from_i64(clock)),
                PctValue::Int(PyInt::from_i64(report_clock)),
                PctValue::Float(min_value),
                PctValue::Float(max_value),
            ],
        )
        .map_err(pct_fail)?;
        self.mcu.add_config_cmd(
            &cmd,
            AddConfigCmdOpts {
                is_init: true,
                on_restart: false,
            },
        )?;
        Ok(())
    }

    /// `_handle_ds18b20_response` (`klippy/extras/ds18b20.py:47`).
    pub fn handle_response(&self, params: &PyValue) -> KResult<()> {
        let value_v = dict_get(params, "value")?;
        let temp = value_as_f64(value_v)? / 1000.0;

        let fault_v = dict_get(params, "fault")?;
        if fault_v.truthy() {
            let fault_pct = value_as_pct(fault_v)?;
            let msg = percent_format(
                "ds18b20 reports fault %d (temp=%0.1f)",
                &[fault_pct, PctValue::Float(temp)],
            )
            .map_err(pct_fail)?;
            log::info(&msg);
            return Ok(());
        }

        let next_v = dict_get(params, "next_clock")?;
        let next_clock = value_as_i64(next_v)?;
        let next_clock64 = self.mcu.clock32_to_clock64(next_clock);
        let report_clock = self.report_clock.get();
        let last_read_clock = next_clock64.wrapping_sub(report_clock);
        let last_read_time = self.mcu.clock_to_print_time(last_read_clock as f64);

        // Take the callback out so no borrow spans the call, and reinstall it
        // only if the callback did not install a new one (`temperature_host`).
        let callback = self.callback.borrow_mut().take();
        match callback {
            Some(mut callback) => {
                callback(last_read_time, temp);
                self.callback.borrow_mut().get_or_insert(callback);
                Ok(())
            }
            None => Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'DS18B20' object has no attribute '_callback'".to_string(),
            }),
        }
    }

    fn setup_minmax_inner(&self, min_temp: f64, max_temp: f64) {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
    }

    /// `fault` (`klippy/extras/ds18b20.py:64`).
    pub fn fault(&self, msg: &str) {
        self.printer
            .invoke_async_shutdown(msg, PyValue::Dict(IndexMap::new()));
    }
}

impl TemperatureSensorApi for DS18B20 {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.setup_minmax_inner(min_temp, max_temp);
        Ok(())
    }

    fn setup_callback(&self, callback: TemperatureCallback) {
        self.callback.replace(Some(callback));
    }

    fn get_report_time_delta(&self) -> f64 {
        self.report_time
    }
}

impl PrinterObject for DS18B20 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemperatureSensorApi);

    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let temperature = round_py_nd(self.temp.get(), 2);
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(temperature),
        );
        Ok(Some(PyValue::Dict(d)))
    }
}

/// Sensor factory registered as `"DS18B20"` in `load_config`.
pub fn sensor_factory(
    config: &Config,
) -> KResult<std::sync::Arc<dyn TemperatureSensorApi>> {
    Ok(DS18B20::new(config)?)
}

/// `load_config(config)` (`klippy/extras/ds18b20.py:78`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let heaters = load_interface::<dyn PrinterHeatersApi>(
        &*printer,
        config,
        "heaters",
        LookupDefault::Raise,
    )?
    .expect("LookupDefault::Raise never yields None");
    let factory: SensorFactory = Arc::new(sensor_factory);
    heaters.add_sensor_factory("DS18B20", factory);
    Ok(None)
}

/// Module entry point for the `ds18b20` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
