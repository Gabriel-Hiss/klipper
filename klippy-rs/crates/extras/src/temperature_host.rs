//! Port of the `temperature_host` module (`klippy/extras/temperature_host.py`).
//!
//! Reports the host (Raspberry Pi) CPU temperature read from a thermal-zone
//! sysfs file to the heaters subsystem, invoking shutdown when the reading
//! leaves the `[min_temp, max_temp]` window configured by the heater.
//!
//! The sensor interface and the heaters registry are owned by
//! [`crate::heaters`]: [`TemperatureSensorApi`] is the duck-typed surface
//! `heaters.Heater` calls on every temperature sensor, and
//! [`PrinterHeatersApi`] is the `heaters` printer object.

use crate::heaters::{
    PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi,
};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::McuApi;
use klippy_api::printer::{EV_KLIPPY_CONNECT, LookupDefault, ModuleFactory, Printer};
use klippy_api::{load_interface, lookup_interface, GCell, ObjectRef, PrinterObject};
use pycompat::text::{float_from_string, percent_format, round_py_nd, PctValue};
use pycompat::PyValue;
use reactor::{Reactor, Timer, NEVER, NOW};
use std::io::{Read, Seek, SeekFrom};
use std::sync::Arc;


/// `HOST_REPORT_TIME = 1.0` (`klippy/extras/temperature_host.py:9`).
pub const HOST_REPORT_TIME: f64 = 1.0;

/// `RPI_PROC_TEMP_FILE` (`klippy/extras/temperature_host.py:10`).
pub const RPI_PROC_TEMP_FILE: &str = "/sys/class/thermal/thermal_zone0/temp";


struct Inner {
    printer: Printer,
    reactor: Reactor,
    name: String,
    path: String,
    temp: GCell<f64>,
    min_temp: GCell<f64>,
    max_temp: GCell<f64>,
    callback: GCell<Option<TemperatureCallback>>,
    file: GCell<Option<std::fs::File>>,
    timer: GCell<Option<Timer>>,
}

impl Inner {
    /// `handle_connect`: re-arm the sample timer for the next pass.
    fn handle_connect(&self) {
        let timer = self.timer.borrow();
        if let Some(timer) = timer.as_ref() {
            self.reactor.update_timer(timer, NOW);
        }
    }

    /// `_sample_pi_temperature(eventtime) -> waketime`.
    fn sample(&self, _eventtime: f64) -> KResult<f64> {
        let read_result: Result<f64, String> = {
            let mut file = self.file.borrow_mut();
            match file.as_mut() {
                None => Err(
                    "'Temperature_HOST' object has no attribute 'file_handle'".to_string(),
                ),
                Some(f) => match f.seek(SeekFrom::Start(0)).and_then(|_| {
                    let mut text = String::new();
                    f.read_to_string(&mut text).map(|_| text)
                }) {
                    Ok(text) => float_from_string(&text)
                        .map(|value| value / 1000.0)
                        .map_err(|err| err.display()),
                    Err(err) => Err(err.to_string()),
                },
            }
        };
        let temp = match read_result {
            Ok(value) => {
                self.temp.set(value);
                value
            }
            Err(exc_text) => {
                log::exception("temperature_host: Error reading data", &exc_text);
                self.temp.set(0.0);
                return Ok(NEVER);
            }
        };
        let min_temp = self.min_temp.get();
        let max_temp = self.max_temp.get();
        if temp < min_temp {
            let msg = percent_format(
                "HOST temperature %0.1f below minimum temperature of %0.1f.",
                &[PctValue::Float(temp), PctValue::Float(min_temp)],
            )
            .map_err(|err| KlippyError::Internal {
                class_name: err.kind.to_string(),
                message: err.msg,
            })?;
            self.printer
                .invoke_shutdown(&msg, PyValue::Dict(IndexMap::new()));
        }
        if temp > max_temp {
            let msg = percent_format(
                "HOST temperature %0.1f above maximum temperature of %0.1f.",
                &[PctValue::Float(temp), PctValue::Float(max_temp)],
            )
            .map_err(|err| KlippyError::Internal {
                class_name: err.kind.to_string(),
                message: err.msg,
            })?;
            self.printer
                .invoke_shutdown(&msg, PyValue::Dict(IndexMap::new()));
        }
        let mcu =
            lookup_interface::<dyn McuApi>(self.printer.as_ref(), "mcu", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
        let measured_time = self.reactor.monotonic();
        let print_time = mcu.estimated_print_time(measured_time);
        // `self._callback(...)`: take the callback out so the borrow is
        // never held across the call, then reinstall it unless the call
        // installed a new one (mirroring that a callback may reassign
        // `self._callback`).
        let callback = self.callback.borrow_mut().take();
        match callback {
            Some(mut callback) => {
                callback(print_time, temp);
                self.callback.borrow_mut().get_or_insert(callback);
            }
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'Temperature_HOST' object has no attribute '_callback'"
                        .to_string(),
                })
            }
        }
        Ok(measured_time + HOST_REPORT_TIME)
    }
}

fn debugoutput_active(printer: &Printer) -> bool {
    match printer.get_start_args() {
        PyValue::Dict(args) => match args.get(&PyValue::Str("debugoutput".to_string())) {
            Some(PyValue::None) | None => false,
            Some(_) => true,
        },
        _ => false,
    }
}

/// `Temperature_HOST` (`klippy/extras/temperature_host.py:12`).
#[allow(non_camel_case_types)]
pub struct Temperature_HOST {
    inner: Arc<Inner>,
}

impl Temperature_HOST {
    /// `Temperature_HOST.__init__(config)` (`klippy/extras/temperature_host.py:13`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let full_name = config.get_name();
        let name = match full_name.split_whitespace().last() {
            Some(name) => name.to_string(),
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                })
            }
        };
        let path = match config.get(
            "sensor_path",
            GetStrOpts {
                default: Some(Some(RPI_PROC_TEMP_FILE.to_string())),
                note_valid: true,
            },
        )? {
            Some(path) => path,
            None => RPI_PROC_TEMP_FILE.to_string(),
        };
        let inner = Arc::new(Inner {
            printer: printer.clone(),
            reactor,
            name,
            path,
            temp: GCell::new(0.0),
            min_temp: GCell::new(0.0),
            max_temp: GCell::new(0.0),
            callback: GCell::new(None),
            file: GCell::new(None),
            timer: GCell::new(None),
        });
        let object_name = format!("temperature_host {}", inner.name);
        let obj = Temperature_HOST { inner: inner.clone() };
        printer.add_object(&object_name, Arc::new(obj) as ObjectRef)?;
        if debugoutput_active(&printer) {
            return Ok(Temperature_HOST { inner });
        }
        let sample_inner = inner.clone();
        let timer = inner.reactor.register_timer(
            move |eventtime| match sample_inner.sample(eventtime) {
                Ok(waketime) => waketime,
                // Python lets the exception escape the timer callback and the
                // reactor dispatch loop dies with it; a panic is the Rust
                // equivalent. This is unreachable in a configured printer
                // ('mcu' exists and setup_callback ran before klippy:connect).
                Err(err) => panic!("{}", err.message()),
            },
            NEVER,
        );
        inner.timer.with_mut(|timer_slot| *timer_slot = Some(timer));
        match std::fs::File::open(&inner.path) {
            Ok(file) => inner.file.with_mut(|file_slot| *file_slot = Some(file)),
            Err(_) => {
                let msg = percent_format(
                    "Unable to open temperature file '%s'",
                    &[PctValue::Str(inner.path.clone())],
                )
                .map_err(|err| KlippyError::Internal {
                    class_name: err.kind.to_string(),
                    message: err.msg,
                })?;
                return Err(KlippyError::config(msg));
            }
        }
        let connect_inner = inner.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params| {
                connect_inner.handle_connect();
                Ok(PyValue::None)
            }),
        );
        Ok(Temperature_HOST { inner })
    }

    /// `handle_connect` (`klippy/extras/temperature_host.py:35`).
    pub fn handle_connect(&self) {
        self.inner.handle_connect();
    }

    /// `_sample_pi_temperature(eventtime) -> waketime`
    /// (`klippy/extras/temperature_host.py:48`).
    pub fn sample_pi_temperature(&self, eventtime: f64) -> KResult<f64> {
        self.inner.sample(eventtime)
    }

    /// `self.name` (`config.get_name().split()[-1]`).
    pub fn name(&self) -> &str {
        &self.inner.name
    }

    /// `self.path` (config `sensor_path`).
    pub fn path(&self) -> &str {
        &self.inner.path
    }
}

impl TemperatureSensorApi for Temperature_HOST {
    /// `setup_minmax(min_temp, max_temp)` (`klippy/extras/temperature_host.py:38`).
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.inner.min_temp.set(min_temp);
        self.inner.max_temp.set(max_temp);
        Ok(())
    }

    /// `setup_callback(cb)` (`klippy/extras/temperature_host.py:42`).
    fn setup_callback(&self, cb: TemperatureCallback) {
        self.inner.callback.with_mut(|callback_slot| *callback_slot = Some(cb));
    }

    /// `get_report_time_delta() -> HOST_REPORT_TIME`
    /// (`klippy/extras/temperature_host.py:45`).
    fn get_report_time_delta(&self) -> f64 {
        HOST_REPORT_TIME
    }
}

impl PrinterObject for Temperature_HOST {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemperatureSensorApi);

    /// `get_status(eventtime)` (`klippy/extras/temperature_host.py:71`).
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(round_py_nd(self.inner.temp.get(), 2)),
        );
        Some(PyValue::Dict(status))
    }
}

/// `load_config(config)` (`klippy/extras/temperature_host.py:77`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let pheaters = load_interface::<dyn PrinterHeatersApi>(
        printer.as_ref(),
        config,
        "heaters",
        LookupDefault::Raise,
    )?
    .expect("LookupDefault::Raise never yields None");
    let factory: SensorFactory = Arc::new(|config: &Config| {
        Ok(Arc::new(Temperature_HOST::new(config)?) as Arc<dyn TemperatureSensorApi>)
    });
    pheaters.add_sensor_factory("temperature_host", factory);
    Ok(None)
}

/// Module entry point for the `temperature_host` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
