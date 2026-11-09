//! Port of the `pulse_counter` module (`klippy/extras/pulse_counter.py`).
//!
//! Provides [`MCU_counter`], a GPIO input edge counter on a chip, and
//! [`FrequencyCounter`], which derives a pulse frequency from it (used by
//! `fan` for tachometer readings). The module has no `load_config` entry
//! point; other modules construct these classes directly, so
//! [`module_factory`] reports no entry points.
//!
//! `MCU_counter` keeps its mutable fields in an inner shared handle so the
//! bound-method callbacks the MCU holds (`build_config` at config time and
//! the `counter_state` serial response handler) can capture the same state
//! the outer object owns; `FrequencyCounter` wires its `_counter_callback`
//! to the inner counter through the same handle.

use std::sync::Arc;

use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{AddConfigCmdOpts, LookupPinOpts, Mcu, PinsApi};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::gcell::SCell;
use klippy_api::lookup_interface;
use pycompat::{value::type_name, PyValue};

/// `MCU_counter` sample callback (`pulse_counter.py:53`): `(time, count, count_time)`.
pub type CounterCallback = Arc<dyn Fn(f64, i64, f64) + Send + Sync>;

/// `FrequencyCounter` callback (`pulse_counter.py:77`): `(time, frequency)`.
pub type FreqCallback = Arc<dyn Fn(f64, f64) + Send + Sync>;

/// Serial response message registered by `build_config` (`pulse_counter.py:33`).
const COUNTER_STATE_MSG: &str = "counter_state oid=%c next_clock=%u count=%u count_clock=%u";

/// Mutable state shared by `MCU_counter` and its MCU-registered callbacks.
struct CounterInner {
    mcu: Mcu,
    oid: i64,
    pin: String,
    pullup: i64,
    poll_time: f64,
    sample_time: f64,
    poll_ticks: SCell<i64>,
    last_count: SCell<i64>,
    callback: SCell<Option<CounterCallback>>,
}

impl CounterInner {
    /// `MCU_counter.build_config()` (`pulse_counter.py:22`).
    fn build_config(&self) -> KResult<()> {
        let config_cmd = format!(
            "config_counter oid={} pin={} pull_up={}",
            self.oid, self.pin, self.pullup
        );
        self.mcu.add_config_cmd(&config_cmd, AddConfigCmdOpts::default())?;
        let clock = self.mcu.get_query_slot(self.oid);
        self.poll_ticks.set(self.mcu.seconds_to_clock(self.poll_time));
        let sample_ticks = self.mcu.seconds_to_clock(self.sample_time);
        let query_cmd = format!(
            "query_counter oid={} clock={} poll_ticks={} sample_ticks={}",
            self.oid,
            clock,
            self.poll_ticks.get(),
            sample_ticks
        );
        self.mcu
            .add_config_cmd(&query_cmd, AddConfigCmdOpts { is_init: true, on_restart: false })?;
        Ok(())
    }

    /// Registers the `counter_state` handler with the MCU (`pulse_counter.py:31`).
    fn register_serial_handler(self: &Arc<CounterInner>) {
        let inner = Arc::clone(self);
        self.mcu.register_serial_response(
            Arc::new(move |params: PyValue| {
                // An exception in the Python handler reaches the serial
                // reader, which logs `Exception in serial callback`; the
                // panic takes that same path (`serialhdl` `catch_unwind`).
                if let Err(e) = inner.handle_counter_state(&params) {
                    panic!("{}: {}", e.class_name(), e.message());
                }
            }),
            COUNTER_STATE_MSG,
            Some(self.oid),
        );
    }

    /// `MCU_counter._handle_counter_state(params)` (`pulse_counter.py:40`).
    fn handle_counter_state(&self, params: &PyValue) -> KResult<()> {
        // Each key is read right before its conversion, as in Python, so a
        // missing key raises after the earlier conversions ran.
        let next_clock64 = self.mcu.clock32_to_clock64(counter_param(params, "next_clock")?);
        let time = self
            .mcu
            .clock_to_print_time((next_clock64 - self.poll_ticks.get()) as f64);
        let count_clock64 = self.mcu.clock32_to_clock64(counter_param(params, "count_clock")?);
        let count_time = self.mcu.clock_to_print_time(count_clock64 as f64);
        let count = counter_param(params, "count")?;
        // Handle 32-bit counter overflow (pulse_counter.py:47).
        let last_count = self.last_count.get();
        let delta_count = (count - last_count) & 0xffff_ffff;
        let count = last_count + delta_count;
        self.last_count.set(count);
        if let Some(callback) = self.callback.cloned() {
            callback(time, count, count_time);
        }
        Ok(())
    }
}

/// Fetches field `key` from a `counter_state` params dict, mirroring
/// `params[key]` (`pulse_counter.py:41-49`) including the `KeyError` text.
fn counter_param(params: &PyValue, key: &str) -> KResult<i64> {
    let PyValue::Dict(entries) = params else {
        return Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!("'{}' object is not subscriptable", type_name(params)),
        });
    };
    let wanted = PyValue::Str(key.to_string());
    let value = entries
        .iter()
        .find(|(k, _)| pycompat::value::py_equal(k, &wanted))
        .map(|(_, v)| v)
        .ok_or_else(|| KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: pycompat::value::repr_key(key),
        })?;
    param_int(value)
}

/// Converts a `counter_state` field to `i64`; firmware fields are `u32`.
fn param_int(value: &PyValue) -> KResult<i64> {
    match value {
        PyValue::Int(n) => n
            .0
            .to_string()
            .parse::<i64>()
            .map_err(|_| KlippyError::Internal {
                class_name: "OverflowError".to_string(),
                message: format!("int too large to convert to i64: {}", value.str()),
            }),
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!("expected int, got {}", type_name(other)),
        }),
    }
}

/// `MCU_counter` (`klippy/extras/pulse_counter.py:7`).
#[allow(non_camel_case_types)]
pub struct MCU_counter {
    inner: Arc<CounterInner>,
}

impl MCU_counter {
    /// `MCU_counter.__init__(printer, pin, sample_time, poll_time)`
    /// (`pulse_counter.py:8`), in the same call order: `pins.lookup_pin`,
    /// `MCU.create_oid`, `MCU.register_config_callback`.
    pub fn new(
        printer: &Printer,
        pin: &str,
        sample_time: f64,
        poll_time: f64,
    ) -> KResult<MCU_counter> {
        let pins = lookup_interface::<dyn PinsApi>(&**printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let pin_params = pins.lookup_pin(
            pin,
            LookupPinOpts { can_invert: false, can_pullup: true, share_type: None },
        )?;
        let mcu = pin_params.chip.clone();
        let oid = mcu.create_oid()?;
        let inner = Arc::new(CounterInner {
            mcu: mcu.clone(),
            oid,
            pin: pin_params.pin.clone(),
            pullup: pin_params.pullup,
            poll_time,
            sample_time,
            poll_ticks: SCell::new(0),
            last_count: SCell::new(0),
            callback: SCell::new(None),
        });
        let build_inner = inner.clone();
        mcu.register_config_callback(Box::new(move || {
            build_inner.build_config()?;
            build_inner.register_serial_handler();
            Ok(())
        }))?;
        Ok(MCU_counter { inner })
    }

    /// `MCU_counter.build_config()` (`pulse_counter.py:22`), run once by the
    /// MCU at `klippy:connect` finalize.
    pub fn build_config(&self) -> KResult<()> {
        self.inner.build_config()?;
        self.inner.register_serial_handler();
        Ok(())
    }

    /// `MCU_counter.setup_callback(cb)` (`pulse_counter.py:37`).
    pub fn setup_callback(&self, cb: CounterCallback) {
        self.inner.callback.with_mut(|c| *c = Some(cb));
    }

    /// `MCU_counter._handle_counter_state(params)` (`pulse_counter.py:40`),
    /// normally invoked by the MCU as the `counter_state` serial response.
    pub fn handle_counter_state(&self, params: &PyValue) -> KResult<()> {
        self.inner.handle_counter_state(params)
    }

    /// `self._oid`.
    pub fn oid(&self) -> i64 {
        self.inner.oid
    }

    /// `self._pin` (the resolved pin name).
    pub fn pin(&self) -> &str {
        &self.inner.pin
    }

    /// `self._pullup`.
    pub fn pullup(&self) -> i64 {
        self.inner.pullup
    }

    /// `self._poll_time`.
    pub fn poll_time(&self) -> f64 {
        self.inner.poll_time
    }

    /// `self._sample_time`.
    pub fn sample_time(&self) -> f64 {
        self.inner.sample_time
    }

    /// `self._poll_ticks` (set by `build_config`).
    pub fn poll_ticks(&self) -> i64 {
        self.inner.poll_ticks.get()
    }

    /// `self._last_count`.
    pub fn last_count(&self) -> i64 {
        self.inner.last_count.get()
    }
}

struct FreqState {
    callback: Option<FreqCallback>,
    last_time: Option<f64>,
    last_count: Option<i64>,
    freq: f64,
}

/// Shared state behind the `FrequencyCounter` callback chain.
struct FreqInner {
    state: SCell<FreqState>,
}

impl FreqInner {
    /// `FrequencyCounter._counter_callback(time, count, count_time)`
    /// (`pulse_counter.py:64`).
    fn counter_callback(&self, time: f64, count: i64, count_time: f64) {
        let callback = {
            let mut state = self.state.borrow_mut();
            if state.last_time.is_none() {
                // First sample (pulse_counter.py:65).
                state.last_time = Some(time);
                None
            } else {
                let last_time = state.last_time.expect("checked above");
                let delta_time = count_time - last_time;
                if delta_time > 0.0 {
                    state.last_time = Some(count_time);
                    let last_count = state.last_count.expect("set on first sample");
                    state.freq = (count - last_count) as f64 / delta_time;
                } else {
                    // No counts since last sample (pulse_counter.py:73).
                    state.last_time = Some(time);
                    state.freq = 0.0;
                }
                state.callback.clone()
            }
        };
        if let Some(callback) = callback {
            callback(time, self.get_frequency());
        }
        self.state.borrow_mut().last_count = Some(count);
    }

    /// `FrequencyCounter.get_frequency()` (`pulse_counter.py:80`).
    fn get_frequency(&self) -> f64 {
        self.state.borrow().freq
    }
}

/// `FrequencyCounter` (`klippy/extras/pulse_counter.py:56`).
pub struct FrequencyCounter {
    counter: MCU_counter,
    inner: Arc<FreqInner>,
}

impl FrequencyCounter {
    /// `FrequencyCounter.__init__(printer, pin, sample_time, poll_time)`
    /// (`pulse_counter.py:57`).
    pub fn new(
        printer: &Printer,
        pin: &str,
        sample_time: f64,
        poll_time: f64,
    ) -> KResult<FrequencyCounter> {
        let counter = MCU_counter::new(printer, pin, sample_time, poll_time)?;
        let inner = Arc::new(FreqInner {
            state: SCell::new(FreqState {
                callback: None,
                last_time: None,
                last_count: None,
                freq: 0.0,
            }),
        });
        let chain = inner.clone();
        counter.setup_callback(Arc::new(move |time: f64, count: i64, count_time: f64| {
            chain.counter_callback(time, count, count_time);
        }));
        Ok(FrequencyCounter { counter, inner })
    }

    /// Assigns `self._callback` (`pulse_counter.py:58`).
    pub fn setup_callback(&self, cb: FreqCallback) {
        self.inner.state.borrow_mut().callback = Some(cb);
    }

    /// `FrequencyCounter._counter_callback(time, count, count_time)`
    /// (`pulse_counter.py:64`), wired to the inner counter at construction.
    pub fn do_counter_callback(&self, time: f64, count: i64, count_time: f64) {
        self.inner.counter_callback(time, count, count_time);
    }

    /// `FrequencyCounter.get_frequency()` (`pulse_counter.py:80`).
    pub fn get_frequency(&self) -> f64 {
        self.inner.get_frequency()
    }

    /// `self._counter`, the inner `MCU_counter` (for tests driving the
    /// `counter_state` handler through the real chain).
    pub fn mcu_counter(&self) -> &MCU_counter {
        &self.counter
    }
}

/// The Python module defines no `load_config`/`load_config_prefix`/
/// printer-object entry points (`pulse_counter.py` only declares the two
/// classes), so the factory has every entry point unset.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
