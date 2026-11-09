//! Port of the `load_cell` module (`klippy/extras/load_cell.py`).
//!
//! The load cell owns a `BulkSensorAdc` sensor (hx71x / ads1220 / ads131m0x),
//! collects its batches into a force buffer for `get_status`, and drives the
//! G-code commands and the guided calibration tool.
//!
//! Sample rows keep Python's shape: the sensor's batch rows are
//! `(time, counts, value)` tuples, and [`LoadCell::sensor_data_event`] rewrites
//! them to `[time, grams, counts, tare_counts]` before handing them to the
//! module's own clients. Collectors therefore store whole rows, exactly like
//! `LoadCellSampleCollector._samples` in the reference.

#![allow(clippy::too_many_arguments)]

use std::collections::VecDeque;
use std::sync::Arc;

use indexmap::IndexMap;

use crate::bulk_sensor::{BatchWebhooksClient, ClientCallback};
use klippy_api::config::{Config, GetChoiceOpts, GetFloatOpts, GetIntOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GCmd, GCode, GCodeApi, GCodeCommandHandler, GetFloatOpts as GCmdGetFloatOpts,
    RegisterCommandOpts,
};
use klippy_api::mcu::Mcu;
use klippy_api::object::{lookup_interface, EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    LookupDefault, Printer, EV_KLIPPY_READY, EV_LOAD_CELL_CALIBRATE, EV_LOAD_CELL_TARE,
};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback, WebRequest};
use pycompat::text::{
    percent_format, py_sum_floats, round_py_nd, trunc_float_to_bigint, PctValue,
};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, NOW};

/// `RETRY_DELAY = 0.05` (`load_cell.py:275`).
pub const RETRY_DELAY: f64 = 0.05;
/// `MIN_COUNTS_PER_GRAM = 1.` (`load_cell.py:374`).
pub const MIN_COUNTS_PER_GRAM: f64 = 1.0;

// ---------------------------------------------------------------------------
// Scalar helpers (CPython semantics)
// ---------------------------------------------------------------------------

/// An exception inside a reactor callback has no caller to catch it: Python
/// ends the process with a traceback.
fn unhandled(e: KlippyError) -> ! {
    panic!("{}: {}", e.class_name(), e.message())
}

fn internal(class_name: &str, message: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: message.to_string(),
    }
}

fn index_error() -> KlippyError {
    internal("IndexError", "list index out of range")
}

fn zero_division_error() -> KlippyError {
    internal("ZeroDivisionError", "float division by zero")
}

/// `%`-formatting with CPython conversions (`pycompat::text::percent_format`).
fn fmt(fmt_str: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt_str, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

/// `float(v)` for a Python number (`int`, `float` or `bool`), `None` for
/// anything else.
fn py_float(v: &PyValue) -> Option<f64> {
    match v {
        PyValue::Float(f) => Some(*f),
        PyValue::Int(n) => Some(n.inner().to_string().parse::<f64>().unwrap_or(f64::NAN)),
        PyValue::Bool(b) => Some(if *b { 1.0 } else { 0.0 }),
        _ => None,
    }
}

/// `int(v)` for a float (`Objects/floatobject.c`), including the ValueError and
/// OverflowError texts CPython raises for NaN and infinities.
fn py_int_from_f64(f: f64) -> KResult<i64> {
    if f.is_nan() {
        return Err(internal("ValueError", "cannot convert float NaN to integer"));
    }
    if f.is_infinite() {
        return Err(internal(
            "OverflowError",
            "cannot convert float infinity to integer",
        ));
    }
    let n = trunc_float_to_bigint(f);
    Ok(n.inner()
        .to_string()
        .parse::<i64>()
        .unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX }))
}

fn dict_get<'a>(v: &'a PyValue, key: &str) -> Option<&'a PyValue> {
    match v {
        PyValue::Dict(d) => d.get(&PyValue::Str(key.to_string())),
        _ => None,
    }
}

/// The elements of a sample row (Python iterates tuples and lists alike).
fn row_items(row: &PyValue) -> &[PyValue] {
    match row {
        PyValue::List(l) => l,
        PyValue::Tuple(t) => t,
        _ => &[],
    }
}

fn row_get(row: &PyValue, idx: usize) -> Option<&PyValue> {
    row_items(row).get(idx)
}

/// `row[0]` as a number (the sample print time).
pub fn sample_time(row: &PyValue) -> Option<f64> {
    row_get(row, 0).and_then(py_float)
}

// ---------------------------------------------------------------------------
// select_column / avg (`load_cell.py:20-24`)
// ---------------------------------------------------------------------------

/// `select_column(data, column_idx)` (`load_cell.py:21`).
///
/// `list(zip(*data))[column_idx]` in Python: the column's values keep their
/// types (`int` counts stay `int`), `zip` stops at the shortest row, and an
/// empty row list or a short row raises `IndexError`. The result is a tuple.
pub fn select_column(rows: &[PyValue], column_idx: usize) -> KResult<Vec<PyValue>> {
    if rows.is_empty() {
        return Err(index_error());
    }
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        match row_get(row, column_idx) {
            Some(v) => out.push(v.clone()),
            None => return Err(index_error()),
        }
    }
    Ok(out)
}

/// `select_column(data, column_idx)` of a numeric column, as floats for the
/// arithmetic that follows (`sum`, comparisons against the sensor range).
fn select_column_floats(rows: &[PyValue], column_idx: usize) -> KResult<Vec<f64>> {
    select_column(rows, column_idx)?
        .iter()
        .map(|v| {
            py_float(v).ok_or_else(|| {
                internal(
                    "TypeError",
                    "unsupported operand type(s) for +: 'int' and 'NoneType'",
                )
            })
        })
        .collect()
}

/// `avg(data)` (`load_cell.py:24`).
pub fn avg(values: &[f64]) -> KResult<f64> {
    if values.is_empty() {
        return Err(internal("ZeroDivisionError", "division by zero"));
    }
    Ok(py_sum_floats(values) / values.len() as f64)
}

// ---------------------------------------------------------------------------
// BulkSensorAdc interface (the three load-cell ADC drivers)
// ---------------------------------------------------------------------------

/// The `BulkSensorAdc` duck type ([`load_cell.py:380`](load_cell.py)):
/// `get_mcu`, `get_samples_per_second`, `get_range`, `add_client`,
/// `get_status`.
///
/// Client callbacks are [`ClientCallback`] from `bulk_sensor`
/// (`bulk_sensor.py:79`); the batch message is
/// `{'data', 'errors', 'overflows'}` and a falsy return unregisters the client.
pub trait BulkSensorAdc: Send + Sync + 'static {
    /// `get_mcu()`.
    fn get_mcu(&self) -> Mcu;
    /// `get_samples_per_second()` (`int` in the drivers).
    fn get_samples_per_second(&self) -> f64;
    /// `get_range()`: `(min, max)` counts, used for saturation detection.
    fn get_range(&self) -> (i64, i64);
    /// `add_client(callback)`.
    fn add_client(&self, callback: ClientCallback) -> KResult<()>;
    /// `get_status(eventtime)`.
    fn get_status(&self, eventtime: f64) -> PyValue;
}

impl BulkSensorAdc for crate::hx71x::HX71xBase {
    fn get_mcu(&self) -> Mcu {
        crate::hx71x::HX71xBase::get_mcu(self)
    }
    fn get_samples_per_second(&self) -> f64 {
        crate::hx71x::HX71xBase::get_samples_per_second(self) as f64
    }
    fn get_range(&self) -> (i64, i64) {
        crate::hx71x::HX71xBase::get_range(self)
    }
    fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        crate::hx71x::HX71xBase::add_client(self, callback)
    }
    fn get_status(&self, eventtime: f64) -> PyValue {
        crate::hx71x::HX71xBase::get_status(self, eventtime)
    }
}

impl BulkSensorAdc for crate::ads1220::Ads1220 {
    fn get_mcu(&self) -> Mcu {
        crate::ads1220::Ads1220::get_mcu(self)
    }
    fn get_samples_per_second(&self) -> f64 {
        crate::ads1220::Ads1220::get_samples_per_second(self) as f64
    }
    fn get_range(&self) -> (i64, i64) {
        crate::ads1220::Ads1220::get_range(self)
    }
    fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        crate::ads1220::Ads1220::add_client(self, callback)
    }
    fn get_status(&self, eventtime: f64) -> PyValue {
        crate::ads1220::Ads1220::get_status(self, eventtime)
    }
}

impl BulkSensorAdc for crate::ads131m0x::ADS131M0X {
    fn get_mcu(&self) -> Mcu {
        crate::ads131m0x::ADS131M0X::get_mcu(self)
    }
    fn get_samples_per_second(&self) -> f64 {
        crate::ads131m0x::ADS131M0X::get_samples_per_second(self)
    }
    fn get_range(&self) -> (i64, i64) {
        crate::ads131m0x::ADS131M0X::get_range(self)
    }
    fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        crate::ads131m0x::ADS131M0X::add_client(self, callback)
    }
    fn get_status(&self, eventtime: f64) -> PyValue {
        crate::ads131m0x::ADS131M0X::get_status(self, eventtime)
    }
}

// ---------------------------------------------------------------------------
// ApiClientHelper (`load_cell.py:27`)
// ---------------------------------------------------------------------------

/// One registered client; the id mirrors `list(self.client_cbs)` identity so a
/// callback can be taken out, run, and put back without holding a borrow
/// across the call (callbacks may pause the reactor).
struct Client {
    id: u64,
    cb: ClientCallback,
}

/// Helper for event driven webhooks and subscription based API clients
/// (`load_cell.py:27`).
pub struct ApiClientHelper {
    printer: Printer,
    client_cbs: GCell<Vec<Client>>,
    next_client_id: GCell<u64>,
    webhooks_start_resp: GCell<PyValue>,
}

impl ApiClientHelper {
    /// `ApiClientHelper(printer)` (`load_cell.py:28`).
    pub fn new(printer: Printer) -> Arc<Self> {
        Arc::new(ApiClientHelper {
            printer,
            client_cbs: GCell::new(Vec::new()),
            next_client_id: GCell::new(0),
            webhooks_start_resp: GCell::new(PyValue::Dict(IndexMap::new())),
        })
    }

    /// Send data to clients (`load_cell.py:34`). Clients that return a falsy
    /// value are removed; clients added during the dispatch are not visited
    /// this pass (Python iterates `list(self.client_cbs)`).
    pub fn send(&self, msg: &PyValue) -> KResult<()> {
        let ids: Vec<u64> = self.client_cbs.borrow().iter().map(|c| c.id).collect();
        for id in ids {
            let Some(idx) = self.client_cbs.borrow().iter().position(|c| c.id == id) else {
                continue;
            };
            let mut cb = std::mem::replace(
                &mut self.client_cbs.borrow_mut()[idx].cb,
                Box::new(|_| Ok(false)) as ClientCallback,
            );
            let result = cb(msg.clone());
            let still = self.client_cbs.borrow().iter().position(|c| c.id == id);
            match result {
                Ok(keep) => {
                    let Some(pos) = still else { continue };
                    if keep {
                        self.client_cbs.borrow_mut()[pos].cb = cb;
                    } else {
                        self.client_cbs.borrow_mut().remove(pos);
                    }
                }
                Err(e) => {
                    if let Some(pos) = still {
                        self.client_cbs.borrow_mut()[pos].cb = cb;
                    }
                    return Err(e);
                }
            }
        }
        Ok(())
    }

    /// Add a client that gets data callbacks (`load_cell.py:42`).
    pub fn add_client(&self, client_cb: ClientCallback) {
        let id = {
            let mut next = self.next_client_id.borrow_mut();
            let id = *next;
            *next += 1;
            id
        };
        self.client_cbs.borrow_mut().push(Client { id, cb: client_cb });
    }

    /// Add Webhooks client and send header (`load_cell.py:46`).
    pub fn add_webhooks_client(&self, web_request: &WebRequest) -> KResult<()> {
        let whbatch = BatchWebhooksClient::new(web_request)?;
        self.add_client(Box::new(move |msg: PyValue| whbatch.handle_batch(msg)));
        web_request.send(self.webhooks_start_resp.cloned())
    }

    /// Set up a webhooks endpoint with a static header (`load_cell.py:52`).
    pub fn add_mux_endpoint(
        self: &Arc<Self>,
        path: &str,
        key: &str,
        value: &str,
        webhooks_start_resp: PyValue,
    ) -> KResult<()> {
        self.webhooks_start_resp.replace(webhooks_start_resp);
        let webhooks =
            lookup_interface::<dyn WebHooksApi>(&*self.printer, "webhooks", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
        let me = Arc::clone(self);
        let cb: WebHooksCallback = Arc::new(move |req: WebRequest| me.add_webhooks_client(&req));
        webhooks.register_mux_endpoint(path, key, value, cb)
    }
}

// ---------------------------------------------------------------------------
// LoadCellSampleCollector (`load_cell.py:276`)
// ---------------------------------------------------------------------------

/// Mutable collector state (`load_cell.py:281-288`).
#[derive(Clone, Copy)]
struct CollectorState {
    min_time: f64,
    max_time: f64,
    min_count: f64,
    is_started: bool,
    errors: i64,
    overflows: i64,
    start_errors: i64,
    start_overflows: i64,
}

/// `(samples, errors)` as returned by the collector: `errors` is
/// `(errors, overflows)` or Python's falsy `0`, which is [`None`] here.
pub struct CollectResult {
    /// Sample rows, stored exactly as received (`[time, grams, counts,
    /// tare_counts]` in a live run, `(0., 0., 0.)` in fileoutput mode).
    pub samples: Vec<PyValue>,
    /// `(errors, overflows)` when either is non-zero.
    pub errors: Option<(i64, i64)>,
}

/// Utility to collect some samples from the LoadCell for later analysis
/// (`load_cell.py:276`).
pub struct LoadCellSampleCollector {
    load_cell: Arc<LoadCell>,
    reactor: Reactor,
    mcu: Mcu,
    state: GCell<CollectorState>,
    samples: GCell<Vec<PyValue>>,
}

impl LoadCellSampleCollector {
    /// `LoadCellSampleCollector(printer, load_cell)` (`load_cell.py:277`).
    pub fn new(printer: Printer, load_cell: Arc<LoadCell>) -> Arc<Self> {
        let mcu = load_cell.sensor.get_mcu();
        let reactor = printer.get_reactor();
        Arc::new(LoadCellSampleCollector {
            load_cell,
            reactor,
            mcu,
            state: GCell::new(CollectorState {
                min_time: 0.0,
                max_time: f64::INFINITY,
                min_count: f64::INFINITY,
                is_started: false,
                errors: 0,
                overflows: 0,
                start_errors: 0,
                start_overflows: 0,
            }),
            samples: GCell::new(Vec::new()),
        })
    }

    /// `_on_samples(msg)` (`load_cell.py:291`).
    fn on_samples(&self, msg: &PyValue) -> bool {
        let (is_started, min_time, max_time, min_count) = {
            let s = self.state.borrow();
            (s.is_started, s.min_time, s.max_time, s.min_count)
        };
        if !is_started {
            return false; // already stopped, ignore
        }
        let errors = dict_get(msg, "errors").and_then(py_float).unwrap_or(0.0);
        let overflows = dict_get(msg, "overflows").and_then(py_float).unwrap_or(0.0);
        let rows: Vec<PyValue> = match dict_get(msg, "data") {
            Some(PyValue::List(l)) => l.clone(),
            Some(PyValue::Tuple(t)) => t.clone(),
            _ => Vec::new(),
        };
        let mut accepted: Vec<PyValue> = Vec::new();
        let mut stop = false;
        for row in rows {
            let time = sample_time(&row).unwrap_or(0.0);
            if min_time <= time && time <= max_time {
                accepted.push(row);
            }
            if time > max_time {
                stop = true;
            }
        }
        {
            let mut s = self.state.borrow_mut();
            s.errors = py_int_from_f64(errors).unwrap_or(0);
            s.overflows = py_int_from_f64(overflows).unwrap_or(0);
            if stop {
                s.is_started = false;
            }
        }
        if !accepted.is_empty() {
            self.samples.with_mut(|v| v.extend(accepted));
        }
        let mut s = self.state.borrow_mut();
        if (self.samples.borrow().len() as f64) >= min_count {
            s.is_started = false;
        }
        s.is_started
    }

    /// `_finish_collecting()` (`load_cell.py:308`).
    fn finish_collecting(&self) -> CollectResult {
        let samples = self.samples.replace(Vec::new());
        let (errors0, overflows0, start_errors, start_overflows) = {
            let s = self.state.borrow();
            (s.errors, s.overflows, s.start_errors, s.start_overflows)
        };
        {
            let mut s = self.state.borrow_mut();
            s.is_started = false;
            s.min_time = 0.0;
            s.max_time = f64::INFINITY;
            s.min_count = f64::INFINITY;
            s.errors = 0;
            s.overflows = 0;
        }
        let errors = (errors0 - start_errors).max(0);
        let overflows = (overflows0 - start_overflows).max(0);
        let samples = if self.mcu.is_fileoutput() {
            vec![PyValue::Tuple(vec![
                PyValue::Float(0.0),
                PyValue::Float(0.0),
                PyValue::Float(0.0),
            ])]
        } else {
            samples
        };
        let err = if errors != 0 || overflows != 0 {
            Some((errors, overflows))
        } else {
            None
        };
        CollectResult { samples, errors: err }
    }

    /// `_collect_until(timeout)` (`load_cell.py:322`).
    fn _collect_until(self: &Arc<Self>, timeout: f64) -> KResult<CollectResult> {
        self.start_collecting(None);
        while self.state.borrow().is_started {
            let now = self.reactor.monotonic();
            if self.mcu.estimated_print_time(now) > timeout {
                let res = self.finish_collecting();
                let (errors, overflows) = res.errors.unwrap_or((0, 0));
                let msg = fmt(
                    "LoadCellSampleCollector timed out! Collected %i samples, Errors: %i, Overflows: %i",
                    &[
                        PctValue::Int(PyInt::from_i64(res.samples.len() as i64)),
                        PctValue::Int(PyInt::from_i64(errors)),
                        PctValue::Int(PyInt::from_i64(overflows)),
                    ],
                )?;
                return Err(KlippyError::Command(msg));
            }
            if self.mcu.is_fileoutput() {
                break;
            }
            self.reactor.pause(now + RETRY_DELAY);
        }
        Ok(self.finish_collecting())
    }

    /// Start collecting with no automatic end to collection
    /// (`load_cell.py:340`).
    pub fn start_collecting(self: &Arc<Self>, min_time: Option<f64>) {
        if self.state.borrow().is_started {
            return;
        }
        {
            let mut s = self.state.borrow_mut();
            if let Some(t) = min_time {
                s.min_time = t;
            }
            s.is_started = true;
        }
        let sensor_status = self.load_cell.sensor.get_status(self.reactor.monotonic());
        let se = dict_get(&sensor_status, "errors").and_then(py_float).unwrap_or(0.0);
        let so = dict_get(&sensor_status, "overflows").and_then(py_float).unwrap_or(0.0);
        {
            let mut s = self.state.borrow_mut();
            s.start_errors = py_int_from_f64(se).unwrap_or(0);
            s.start_overflows = py_int_from_f64(so).unwrap_or(0);
        }
        let clients = Arc::clone(&self.load_cell.clients);
        let me = Arc::clone(self);
        clients.add_client(Box::new(move |msg: PyValue| Ok(me.on_samples(&msg))));
    }

    /// Stop collecting immediately and return results (`load_cell.py:352`).
    pub fn stop_collecting(&self) -> CollectResult {
        self.finish_collecting()
    }

    /// Block execution until at least `min_count` samples are collected
    /// (`load_cell.py:357`).
    pub fn collect_min(self: &Arc<Self>, min_count: f64) -> KResult<CollectResult> {
        self.state.borrow_mut().min_count = min_count;
        if (self.samples.borrow().len() as f64) >= min_count {
            return Ok(self.finish_collecting());
        }
        let print_time = self.mcu.estimated_print_time(self.reactor.monotonic());
        let min_time = self.state.borrow().min_time;
        let start_time = if min_time > print_time { min_time } else { print_time };
        let sps = self.load_cell.sensor.get_samples_per_second();
        self._collect_until(start_time + 1.0 + (min_count / sps))
    }

    /// Returns when a sample is collected with a timestamp after `print_time`
    /// (`load_cell.py:367`).
    pub fn collect_until(self: &Arc<Self>, print_time: f64) -> KResult<CollectResult> {
        self.state.borrow_mut().max_time = print_time;
        let has_recent = match self.samples.borrow().last() {
            Some(row) => sample_time(row).unwrap_or(0.0) >= print_time,
            None => false,
        };
        if has_recent {
            return Ok(self.finish_collecting());
        }
        self._collect_until(print_time + 1.0)
    }
}

// ---------------------------------------------------------------------------
// LoadCell (`load_cell.py:375`)
// ---------------------------------------------------------------------------

/// Printer class that controls the load cell (`load_cell.py:375`).
pub struct LoadCell {
    /// `self.printer`.
    pub printer: Printer,
    /// `self.config_name`.
    pub config_name: String,
    /// `self.name`.
    pub name: String,
    /// `self.sensor` — must implement [`BulkSensorAdc`].
    pub sensor: Arc<dyn BulkSensorAdc>,
    /// `self.clients`.
    pub clients: Arc<ApiClientHelper>,
    /// `buffer_size` — `collections.deque(maxlen=buffer_size)`.
    buffer_size: i64,
    /// `self.invert`.
    invert: f64,
    /// `self.reference_tare_counts`.
    reference_tare_counts: GCell<Option<i64>>,
    /// `self.tare_counts`.
    tare_counts: GCell<Option<i64>>,
    /// `self.counts_per_gram`.
    counts_per_gram: GCell<Option<f64>>,
    /// `self._force_buffer`.
    force_buffer: GCell<VecDeque<f64>>,
}

impl LoadCell {
    /// `LoadCell.__init__` (`load_cell.py:376`).
    pub fn new(config: &Config, sensor: Arc<dyn BulkSensorAdc>) -> KResult<Arc<LoadCell>> {
        let printer = config.get_printer();
        let config_name = config.get_name();
        let name = config_name
            .split_whitespace()
            .last()
            .unwrap_or("")
            .to_string();
        let buffer_size = py_int_from_f64(sensor.get_samples_per_second() / 2.0)?;
        if buffer_size < 0 {
            return Err(internal("ValueError", "maxlen must be non-negative"));
        }
        let reference_tare_counts = config.getint(
            "reference_tare_counts",
            GetIntOpts {
                default: Some(None),
                ..Default::default()
            },
        )?;
        let counts_per_gram = config.getfloat(
            "counts_per_gram",
            GetFloatOpts {
                default: Some(None),
                minval: Some(MIN_COUNTS_PER_GRAM),
                ..Default::default()
            },
        )?;
        let mut choices = IndexMap::new();
        choices.insert(PyValue::Str("normal".to_string()), PyValue::Float(1.0));
        choices.insert(PyValue::Str("inverted".to_string()), PyValue::Float(-1.0));
        let orientation = config.getchoice(
            "sensor_orientation",
            &PyValue::Dict(choices),
            GetChoiceOpts {
                default: Some(Some(PyValue::Str("normal".to_string()))),
                ..Default::default()
            },
        )?;
        let invert = py_float(&orientation).unwrap_or(1.0);
        let cell = Arc::new(LoadCell {
            printer: printer.clone(),
            config_name,
            name,
            sensor,
            clients: ApiClientHelper::new(printer.clone()),
            buffer_size,
            invert,
            reference_tare_counts: GCell::new(reference_tare_counts),
            tare_counts: GCell::new(reference_tare_counts),
            counts_per_gram: GCell::new(counts_per_gram),
            force_buffer: GCell::new(VecDeque::new()),
        });
        LoadCellCommandHelper::new(config, Arc::clone(&cell))?;
        // Client support:
        let mut header = IndexMap::new();
        header.insert(
            PyValue::Str("header".to_string()),
            PyValue::List(vec![
                PyValue::Str("time".to_string()),
                PyValue::Str("force (g)".to_string()),
                PyValue::Str("counts".to_string()),
                PyValue::Str("tare_counts".to_string()),
            ]),
        );
        cell.clients.add_mux_endpoint(
            "load_cell/dump_force",
            "load_cell",
            &cell.name.clone(),
            PyValue::Dict(header),
        )?;
        // startup, when klippy is ready, start capturing data
        {
            let me = Arc::clone(&cell);
            printer.register_event_handler(
                EV_KLIPPY_READY,
                Box::new(move |_: &[EventArg]| {
                    let reactor = me.printer.get_reactor();
                    let me2 = Arc::clone(&me);
                    reactor.register_callback(
                        move |_eventtime: f64| {
                            if let Err(e) = me2.handle_do_ready() {
                                unhandled(e);
                            }
                        },
                        NOW,
                    );
                    Ok(PyValue::None)
                }),
            );
        }
        Ok(cell)
    }

    /// `_handle_do_ready(eventtime)` (`load_cell.py:400`).
    pub fn handle_do_ready(self: &Arc<Self>) -> KResult<()> {
        {
            let me = Arc::clone(self);
            self.sensor
                .add_client(Box::new(move |msg: PyValue| me.sensor_data_event(&msg)))?;
        }
        {
            let me = Arc::clone(self);
            self.add_client(Box::new(move |msg: PyValue| {
                let _ = me.track_force(&msg);
                Ok(true)
            }));
        }
        // announce calibration status on ready
        if self.is_calibrated() {
            let obj: ObjectRef = Arc::clone(self) as ObjectRef;
            self.printer
                .send_event(EV_LOAD_CELL_CALIBRATE, &[EventArg::Object(obj)])?;
        }
        if self.is_tared() {
            let obj: ObjectRef = Arc::clone(self) as ObjectRef;
            self.printer
                .send_event(EV_LOAD_CELL_TARE, &[EventArg::Object(obj)])?;
        }
        Ok(())
    }

    /// `_sensor_data_event(msg)` (`load_cell.py:411`): convert raw counts to
    /// grams and broadcast to clients.
    pub fn sensor_data_event(&self, msg: &PyValue) -> KResult<bool> {
        let Some(data) = dict_get(msg, "data") else {
            return Ok(false);
        };
        if matches!(data, PyValue::None) {
            return Ok(false);
        }
        let errors = dict_get(msg, "errors").cloned().unwrap_or(PyValue::None);
        let overflows = dict_get(msg, "overflows").cloned().unwrap_or(PyValue::None);
        let tare = match self.tare_counts.get() {
            Some(t) => PyValue::Int(PyInt::from_i64(t)),
            None => PyValue::None,
        };
        let mut samples: Vec<PyValue> = Vec::new();
        for row in row_items(data) {
            // [time, grams, counts, tare_counts]
            let time = row_get(row, 0).cloned().unwrap_or(PyValue::None);
            let counts = row_get(row, 1).cloned().unwrap_or(PyValue::None);
            let grams = match py_float(&counts) {
                Some(sample) => match self.counts_to_grams(sample) {
                    Some(g) => PyValue::Float(g),
                    None => PyValue::None,
                },
                None => PyValue::None,
            };
            samples.push(PyValue::List(vec![time, grams, counts, tare.clone()]));
        }
        let mut out = IndexMap::new();
        out.insert(PyValue::Str("data".to_string()), PyValue::List(samples));
        out.insert(PyValue::Str("errors".to_string()), errors);
        out.insert(PyValue::Str("overflows".to_string()), overflows);
        self.clients.send(&PyValue::Dict(out))?;
        Ok(true)
    }

    /// `LoadCell.add_client(callback)` (`load_cell.py:427`).
    pub fn add_client(&self, callback: ClientCallback) {
        self.clients.add_client(callback);
    }

    /// `LoadCell.tare(tare_counts)` (`load_cell.py:430`).
    pub fn tare(self: &Arc<Self>, tare_counts: f64) -> KResult<()> {
        self.tare_counts.set(Some(py_int_from_f64(tare_counts)?));
        let obj: ObjectRef = Arc::clone(self) as ObjectRef;
        self.printer
            .send_event(EV_LOAD_CELL_TARE, &[EventArg::Object(obj)])?;
        Ok(())
    }

    /// `LoadCell.set_calibration(counts_per_gram, tare_counts)`
    /// (`load_cell.py:434`).
    pub fn set_calibration(
        self: &Arc<Self>,
        counts_per_gram: Option<f64>,
        tare_counts: Option<f64>,
    ) -> KResult<()> {
        let Some(cpg) = counts_per_gram else {
            return Err(KlippyError::Command(
                "Invalid counts per gram value".to_string(),
            ));
        };
        if cpg.abs() < MIN_COUNTS_PER_GRAM {
            return Err(KlippyError::Command(
                "Invalid counts per gram value".to_string(),
            ));
        }
        let Some(tare) = tare_counts else {
            return Err(KlippyError::Command("Missing tare counts".to_string()));
        };
        self.counts_per_gram.set(Some(cpg));
        let reference_tare_counts = py_int_from_f64(tare)?;
        self.reference_tare_counts.set(Some(reference_tare_counts));
        let configfile =
            lookup_interface::<dyn PrinterConfigApi>(&*self.printer, "configfile", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
        configfile.set(
            &self.config_name,
            "counts_per_gram",
            &fmt("%.5f", &[PctValue::Float(cpg)])?,
        );
        configfile.set(
            &self.config_name,
            "reference_tare_counts",
            &fmt(
                "%i",
                &[PctValue::Int(PyInt::from_i64(reference_tare_counts))],
            )?,
        );
        let obj: ObjectRef = Arc::clone(self) as ObjectRef;
        self.printer
            .send_event(EV_LOAD_CELL_CALIBRATE, &[EventArg::Object(obj)])?;
        Ok(())
    }

    /// `LoadCell.counts_to_grams(sample)` (`load_cell.py:449`).
    pub fn counts_to_grams(&self, sample: f64) -> Option<f64> {
        if !self.is_calibrated() || !self.is_tared() {
            return None;
        }
        let tare = self.tare_counts.get().unwrap_or(0) as f64;
        let cpg = self.counts_per_gram.get()?;
        Some(self.invert * ((sample - tare) / cpg))
    }

    /// `LoadCell.saturation_range()` (`load_cell.py:456`): the maximum range of
    /// the sensor based on its bit width.
    pub fn saturation_range(&self) -> (i64, i64) {
        self.sensor.get_range()
    }

    /// `LoadCell.counts_to_percent(counts)` (`load_cell.py:460`): raw counts as
    /// a +/- percentage of the sensor's range.
    pub fn counts_to_percent(&self, counts: f64) -> KResult<f64> {
        let (_range_min, range_max) = self.saturation_range();
        if range_max == 0 {
            return Err(zero_division_error());
        }
        Ok((counts / range_max as f64) * 100.0)
    }

    /// `LoadCell.avg_counts(num_samples=None)` (`load_cell.py:466`): read one
    /// second of load cell data and average it, with saturation checks.
    pub fn avg_counts(self: &Arc<Self>, num_samples: Option<f64>) -> KResult<f64> {
        let num_samples = match num_samples {
            Some(n) => n,
            None => py_int_from_f64(self.sensor.get_samples_per_second())? as f64,
        };
        let collector = self.get_collector();
        let res = collector.collect_min(num_samples)?;
        if let Some((errors, overflows)) = res.errors {
            let msg = fmt(
                "Sensor reported %i errors while sampling",
                &[PctValue::Int(PyInt::from_i64(errors + overflows))],
            )?;
            return Err(KlippyError::Command(msg));
        }
        // check samples for saturated readings
        let (range_min, range_max) = self.saturation_range();
        for row in &res.samples {
            let counts = row_get(row, 2).and_then(py_float);
            if let Some(counts) = counts {
                if counts >= range_max as f64 || counts <= range_min as f64 {
                    return Err(KlippyError::Command(
                        "Some samples are saturated (+/-100%)".to_string(),
                    ));
                }
            }
        }
        avg(&select_column_floats(&res.samples, 2)?)
    }

    /// `_track_force(msg)` (`load_cell.py:483`): ongoing force tracking for
    /// status updates.
    pub fn track_force(&self, msg: &PyValue) -> bool {
        if !(self.is_calibrated() && self.is_tared()) {
            return true;
        }
        let Some(data) = dict_get(msg, "data") else {
            return true;
        };
        if self.buffer_size <= 0 {
            return true;
        }
        let buffer_size = self.buffer_size as usize;
        let mut buf = self.force_buffer.borrow_mut();
        for row in row_items(data) {
            let Some(force) = row_get(row, 1).and_then(py_float) else {
                continue;
            };
            if buf.len() >= buffer_size {
                buf.pop_front();
            }
            buf.push_back(force);
        }
        true
    }

    /// `_force_g()` (`load_cell.py:492`).
    fn force_g(&self) -> IndexMap<PyValue, PyValue> {
        let mut out = IndexMap::new();
        if self.is_calibrated() && self.is_tared() {
            let buf = self.force_buffer.borrow();
            if !buf.is_empty() {
                let vals: Vec<f64> = buf.iter().copied().collect();
                let mut min = vals[0];
                let mut max = vals[0];
                for &v in &vals {
                    if v < min {
                        min = v;
                    }
                    if v > max {
                        max = v;
                    }
                }
                out.insert(
                    PyValue::Str("force_g".to_string()),
                    PyValue::Float(round_py_nd(avg(&vals).unwrap_or(0.0), 1)),
                );
                out.insert(
                    PyValue::Str("min_force_g".to_string()),
                    PyValue::Float(round_py_nd(min, 1)),
                );
                out.insert(
                    PyValue::Str("max_force_g".to_string()),
                    PyValue::Float(round_py_nd(max, 1)),
                );
            }
        }
        out
    }

    /// `LoadCell.is_tared()` (`load_cell.py:500`).
    pub fn is_tared(&self) -> bool {
        self.tare_counts.get().is_some()
    }

    /// `LoadCell.is_calibrated()` (`load_cell.py:503`).
    pub fn is_calibrated(&self) -> bool {
        self.counts_per_gram.get().is_some() && self.reference_tare_counts.get().is_some()
    }

    /// `LoadCell.get_sensor()` (`load_cell.py:507`).
    pub fn get_sensor(&self) -> Arc<dyn BulkSensorAdc> {
        Arc::clone(&self.sensor)
    }

    /// `LoadCell.get_reference_tare_counts()` (`load_cell.py:510`).
    pub fn get_reference_tare_counts(&self) -> Option<i64> {
        self.reference_tare_counts.get()
    }

    /// `LoadCell.get_tare_counts()` (`load_cell.py:513`).
    pub fn get_tare_counts(&self) -> Option<i64> {
        self.tare_counts.get()
    }

    /// `LoadCell.get_counts_per_gram()` (`load_cell.py:516`).
    pub fn get_counts_per_gram(&self) -> Option<f64> {
        self.counts_per_gram.get()
    }

    /// `LoadCell.get_collector()` (`load_cell.py:519`).
    pub fn get_collector(self: &Arc<Self>) -> Arc<LoadCellSampleCollector> {
        LoadCellSampleCollector::new(self.printer.clone(), Arc::clone(self))
    }

    /// `LoadCell.get_status(eventtime)` (`load_cell.py:522`).
    pub fn get_status_value(&self, eventtime: f64) -> PyValue {
        let mut status = self.force_g();
        status.insert(
            PyValue::Str("is_calibrated".to_string()),
            PyValue::Bool(self.is_calibrated()),
        );
        status.insert(
            PyValue::Str("counts_per_gram".to_string()),
            match self.counts_per_gram.get() {
                Some(cpg) => PyValue::Float(cpg),
                None => PyValue::None,
            },
        );
        status.insert(
            PyValue::Str("reference_tare_counts".to_string()),
            match self.reference_tare_counts.get() {
                Some(counts) => PyValue::Int(PyInt::from_i64(counts)),
                None => PyValue::None,
            },
        );
        status.insert(
            PyValue::Str("tare_counts".to_string()),
            match self.tare_counts.get() {
                Some(counts) => PyValue::Int(PyInt::from_i64(counts)),
                None => PyValue::None,
            },
        );
        if let PyValue::Dict(sensor_status) = self.sensor.get_status(eventtime) {
            status.extend(sensor_status);
        }
        PyValue::Dict(status)
    }
}

impl PrinterObject for LoadCell {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.get_status_value(eventtime))
    }
}

// ---------------------------------------------------------------------------
// LoadCellCommandHelper (`load_cell.py:58`)
// ---------------------------------------------------------------------------

/// Class for handling commands related to load cells (`load_cell.py:58`).
pub struct LoadCellCommandHelper {
    printer: Printer,
    load_cell: Arc<LoadCell>,
    name: String,
}

impl LoadCellCommandHelper {
    /// `LoadCellCommandHelper(config, load_cell)` (`load_cell.py:59`).
    #[allow(clippy::new_ret_no_self)]
    pub fn new(config: &Config, load_cell: Arc<LoadCell>) -> KResult<()> {
        let printer = config.get_printer();
        let name_parts: Vec<String> = config
            .get_name()
            .split_whitespace()
            .map(str::to_string)
            .collect();
        let name = name_parts.last().cloned().unwrap_or_default();
        let helper = LoadCellCommandHelper {
            printer,
            load_cell,
            name,
        };
        helper.register_commands(Some(&helper.name.clone()))?;
        if name_parts.len() == 1 {
            helper.register_commands(None)?;
        }
        Ok(())
    }

    /// `register_commands(name)` (`load_cell.py:68`).
    fn register_commands(&self, name: Option<&str>) -> KResult<()> {
        let gcode = lookup_interface::<dyn GCodeApi>(&*self.printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        // Register commands
        let cell = Arc::clone(&self.load_cell);
        gcode.register_mux_command(
            "LOAD_CELL_TARE",
            "LOAD_CELL",
            name,
            Box::new(move |gcmd: &GCmd| {
                let tare_counts = cell.avg_counts(None)?;
                cell.tare(tare_counts)?;
                let tare_percent = cell.counts_to_percent(tare_counts)?;
                let msg = fmt(
                    "Load cell tare value: %.2f%% (%i)",
                    &[PctValue::Float(tare_percent), PctValue::Float(tare_counts)],
                )?;
                gcmd.respond_info(&msg, true);
                Ok(())
            }),
            Some("Set the Zero point of the load cell"),
        )?;
        let cell = Arc::clone(&self.load_cell);
        gcode.register_mux_command(
            "LOAD_CELL_CALIBRATE",
            "LOAD_CELL",
            name,
            Box::new(move |_: &GCmd| {
                let printer = cell.printer.clone();
                LoadCellGuidedCalibrationHelper::new(&printer, Arc::clone(&cell)).map(|_| ())
            }),
            Some("Start interactive calibration tool"),
        )?;
        let cell = Arc::clone(&self.load_cell);
        gcode.register_mux_command(
            "LOAD_CELL_READ",
            "LOAD_CELL",
            name,
            Box::new(move |gcmd: &GCmd| {
                let counts = cell.avg_counts(None)?;
                let percent = cell.counts_to_percent(counts)?;
                let force = cell.counts_to_grams(counts);
                if percent >= 100.0 || percent <= -100.0 {
                    let msg = fmt("Err (%.2f%%)", &[PctValue::Float(percent)])?;
                    gcmd.respond_info(&msg, true);
                }
                match force {
                    None => {
                        let msg = fmt("---.-g (%.2f%%)", &[PctValue::Float(percent)])?;
                        gcmd.respond_info(&msg, true);
                    }
                    Some(force) => {
                        let msg = fmt(
                            "%.1fg (%.2f%%)",
                            &[PctValue::Float(force), PctValue::Float(percent)],
                        )?;
                        gcmd.respond_info(&msg, true);
                    }
                }
                Ok(())
            }),
            Some("Take a reading from the load cell"),
        )?;
        let cell = Arc::clone(&self.load_cell);
        gcode.register_mux_command(
            "LOAD_CELL_DIAGNOSTIC",
            "LOAD_CELL",
            name,
            Box::new(move |gcmd: &GCmd| cmd_load_cell_diagnostic(&cell, gcmd)),
            Some("Check the health of the load cell"),
        )?;
        Ok(())
    }
}

/// `cmd_LOAD_CELL_DIAGNOSTIC(gcmd)` (`load_cell.py:109`).
fn cmd_load_cell_diagnostic(load_cell: &Arc<LoadCell>, gcmd: &GCmd) -> KResult<()> {
    gcmd.respond_info("Collecting load cell data for 10 seconds...", true);
    let collector = load_cell.get_collector();
    let reactor = load_cell.printer.get_reactor();
    collector.start_collecting(None);
    reactor.pause(reactor.monotonic() + 10.0);
    let (samples, errors) = {
        let res = collector.stop_collecting();
        (res.samples, res.errors)
    };
    match errors {
        Some((errs, overflows)) => {
            let msg = fmt(
                "Sensor reported errors: %i errors, %i overflows",
                &[
                    PctValue::Int(PyInt::from_i64(errs)),
                    PctValue::Int(PyInt::from_i64(overflows)),
                ],
            )?;
            gcmd.respond_info(&msg, true);
        }
        None => gcmd.respond_info("Sensor reported no errors", true),
    }
    if samples.is_empty() {
        return Err(KlippyError::Command(
            "No samples returned from sensor!".to_string(),
        ));
    }
    let counts = select_column_floats(&samples, 2)?;
    let (range_min, range_max) = load_cell.saturation_range();
    let mut good_count = 0i64;
    let mut saturation_count = 0i64;
    for &sample in &counts {
        if sample >= range_max as f64 || sample <= range_min as f64 {
            saturation_count += 1;
        } else {
            good_count += 1;
        }
    }
    let msg = fmt(
        "Samples Collected: %i",
        &[PctValue::Int(PyInt::from_i64(samples.len() as i64))],
    )?;
    gcmd.respond_info(&msg, true);
    if samples.len() > 2 {
        let sensor_sps = load_cell.sensor.get_samples_per_second();
        let last_time = sample_time(samples.last().expect("non-empty")).unwrap_or(0.0);
        let first_time = sample_time(&samples[0]).unwrap_or(0.0);
        let elapsed = last_time - first_time;
        if elapsed == 0.0 {
            return Err(zero_division_error());
        }
        let sps = samples.len() as f64 / elapsed;
        let msg = fmt(
            "Measured samples per second: %.1f, configured: %.1f",
            &[PctValue::Float(sps), PctValue::Float(sensor_sps)],
        )?;
        gcmd.respond_info(&msg, true);
    }
    let mut unique: Vec<f64> = Vec::new();
    for &c in &counts {
        if !unique.contains(&c) {
            unique.push(c);
        }
    }
    let msg = fmt(
        "Good samples: %i, Saturated samples: %i, Unique values: %i",
        &[
            PctValue::Int(PyInt::from_i64(good_count)),
            PctValue::Int(PyInt::from_i64(saturation_count)),
            PctValue::Int(PyInt::from_i64(unique.len() as i64)),
        ],
    )?;
    gcmd.respond_info(&msg, true);
    let mut max_counts = counts[0];
    let mut min_counts = counts[0];
    for &c in &counts {
        if c > max_counts {
            max_counts = c;
        }
        if c < min_counts {
            min_counts = c;
        }
    }
    let max_pct = load_cell.counts_to_percent(max_counts)?;
    let min_pct = load_cell.counts_to_percent(min_counts)?;
    let msg = fmt(
        "Sample range: [%.2f%% to %.2f%%]",
        &[PctValue::Float(min_pct), PctValue::Float(max_pct)],
    )?;
    gcmd.respond_info(&msg, true);
    let msg = fmt(
        "Sample range / sensor capacity: %.5f%%",
        &[PctValue::Float((max_pct - min_pct) / 2.0)],
    )?;
    gcmd.respond_info(&msg, true);
    Ok(())
}

// ---------------------------------------------------------------------------
// LoadCellGuidedCalibrationHelper (`load_cell.py:149`)
// ---------------------------------------------------------------------------

/// Class to guide the user through calibrating a load cell
/// (`load_cell.py:149`).
pub struct LoadCellGuidedCalibrationHelper {
    gcode: GCode,
    load_cell: Arc<LoadCell>,
    tare_counts: GCell<Option<f64>>,
    counts_per_gram: GCell<Option<f64>>,
    /// `self.tare_percent`.
    pub tare_percent: GCell<f64>,
}

impl LoadCellGuidedCalibrationHelper {
    /// `LoadCellGuidedCalibrationHelper(printer, load_cell)`
    /// (`load_cell.py:150`).
    pub fn new(printer: &Printer, load_cell: Arc<LoadCell>) -> KResult<Arc<Self>> {
        let gcode = lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let helper = Arc::new(LoadCellGuidedCalibrationHelper {
            gcode: Arc::clone(&gcode),
            load_cell,
            tare_counts: GCell::new(None),
            counts_per_gram: GCell::new(None),
            tare_percent: GCell::new(0.0),
        });
        helper.register_commands()?;
        gcode.respond_info(
            "Starting load cell calibration. \n\
             1.) Remove all load and run TARE. \n\
             2.) Apply a known load, run CALIBRATE GRAMS=nnn. \n\
             Complete calibration with the ACCEPT command.\n\
             Use the ABORT command to quit.",
            true,
        );
        Ok(helper)
    }

    /// `verify_no_active_calibration()` (`load_cell.py:164`).
    fn verify_no_active_calibration(&self) -> KResult<()> {
        // The reference registers a throwaway `'dummy'` handler to detect an
        // already running calibration; `register_command(.., None)` removes it.
        let dummy: GCodeCommandHandler = Box::new(|_: &GCmd| Ok(()));
        let registered =
            self.gcode
                .register_command("TARE", Some(dummy), RegisterCommandOpts::default());
        match registered {
            Ok(_) => {}
            Err(KlippyError::Config(_)) => {
                return Err(KlippyError::Command(
                    "Already Calibrating a Load Cell. Use ABORT to quit.".to_string(),
                ))
            }
            Err(e) => return Err(e),
        }
        self.gcode.register_command("TARE", None, RegisterCommandOpts::default())?;
        Ok(())
    }

    /// `register_commands()` (`load_cell.py:169`).
    fn register_commands(self: &Arc<Self>) -> KResult<()> {
        self.verify_no_active_calibration()?;
        let me = Arc::clone(self);
        let h = Arc::clone(&me);
        self.gcode.register_command(
            "ABORT",
            Some(Box::new(move |gcmd: &GCmd| h.cmd_abort(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Abort load cell calibration tool".to_string()),
            },
        )?;
        let h = Arc::clone(&me);
        self.gcode.register_command(
            "ACCEPT",
            Some(Box::new(move |gcmd: &GCmd| h.cmd_accept(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Accept calibration results and apply to load cell".to_string()),
            },
        )?;
        let h = Arc::clone(&me);
        self.gcode.register_command(
            "TARE",
            Some(Box::new(move |gcmd: &GCmd| h.cmd_tare(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Tare the load cell".to_string()),
            },
        )?;
        let h = Arc::clone(&me);
        self.gcode.register_command(
            "CALIBRATE",
            Some(Box::new(move |gcmd: &GCmd| h.cmd_calibrate(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Enter the load cell value in grams".to_string()),
            },
        )?;
        Ok(())
    }

    /// `counts_per_gram(grams, cal_counts)` (`load_cell.py:182`): the delta of
    /// counts as a counts/gram metric.
    pub fn counts_per_gram(&self, grams: f64, cal_counts: f64) -> KResult<f64> {
        let Some(tare) = self.tare_counts.get() else {
            return Err(internal(
                "TypeError",
                "unsupported operand type(s) for -: 'NoneType' and 'float'",
            ));
        };
        let delta = tare - cal_counts;
        let magnitude = py_int_from_f64(delta)?.unsigned_abs() as f64;
        if grams == 0.0 {
            return Err(zero_division_error());
        }
        Ok(magnitude / grams)
    }

    /// `capacity_kg(counts_per_gram)` (`load_cell.py:187`): the max force the
    /// load cell can register, given the tare bias, in kilograms.
    pub fn capacity_kg(&self, counts_per_gram: f64) -> KResult<f64> {
        let (_range_min, range_max) = self.load_cell.saturation_range();
        let tare = self.tare_counts.get().unwrap_or(0.0);
        let numerator = range_max as f64 - tare.abs();
        // Python: `(range_max - abs(tare)) / counts_per_gram`, which raises
        // ZeroDivisionError for a zero counts_per_gram before `int()` runs.
        if counts_per_gram == 0.0 {
            return Err(zero_division_error());
        }
        let counts = py_int_from_f64(numerator / counts_per_gram)?;
        Ok(counts as f64 / 1000.0)
    }

    /// `finalize(save_results=False)` (`load_cell.py:192`).
    pub fn finalize(&self, save_results: bool) -> KResult<()> {
        for name in ["ABORT", "ACCEPT", "TARE", "CALIBRATE"] {
            self.gcode.register_command(name, None, RegisterCommandOpts::default())?;
        }
        if !save_results {
            self.gcode
                .respond_info("Load cell calibration aborted", true);
            return Ok(());
        }
        if self.counts_per_gram.get().is_none() || self.tare_counts.get().is_none() {
            self.gcode.respond_info(
                "Calibration process is incomplete, \
                 aborting",
                true,
            );
        }
        self.load_cell
            .set_calibration(self.counts_per_gram.get(), self.tare_counts.get())?;
        let msg = fmt(
            "Load cell calibration settings:\n\n\
             counts_per_gram: %.6f\n\
             reference_tare_counts: %i\n\n\
             The SAVE_CONFIG command will update the printer config file \
             with the above and restart the printer.",
            &[
                PctValue::Float(self.counts_per_gram.get().unwrap_or(0.0)),
                PctValue::Float(self.tare_counts.get().unwrap_or(0.0)),
            ],
        )?;
        self.gcode.respond_info(&msg, true);
        self.load_cell.tare(self.tare_counts.get().unwrap_or(0.0))?;
        Ok(())
    }

    /// `cmd_ABORT(gcmd)` (`load_cell.py:211`).
    pub fn cmd_abort(&self, _gcmd: &GCmd) -> KResult<()> {
        self.finalize(false)
    }

    /// `cmd_ACCEPT(gcmd)` (`load_cell.py:215`).
    pub fn cmd_accept(&self, _gcmd: &GCmd) -> KResult<()> {
        self.finalize(true)
    }

    /// `cmd_TARE(gcmd)` (`load_cell.py:219`).
    pub fn cmd_tare(&self, gcmd: &GCmd) -> KResult<()> {
        let tare_counts = self.load_cell.avg_counts(None)?;
        self.tare_counts.set(Some(tare_counts));
        self.counts_per_gram.set(None); // require re-calibration on tare
        let tare_percent = self.load_cell.counts_to_percent(tare_counts)?;
        self.tare_percent.set(tare_percent);
        let msg = fmt(
            "Load cell tare value: %.2f%% (%i)",
            &[PctValue::Float(tare_percent), PctValue::Float(tare_counts)],
        )?;
        gcmd.respond_info(&msg, true);
        if tare_percent > 2.0 {
            gcmd.respond_info(
                "WARNING: tare value is more than 2% away from 0!\n\
                 The load cell's range will be impacted.\n\
                 Check for external force on the load cell.",
                true,
            );
        }
        gcmd.respond_info(
            "Now apply a known force to the load cell and enter                          the force value with:\n CALIBRATE GRAMS=nnn",
            true,
        );
        Ok(())
    }

    /// `cmd_CALIBRATE(gcmd)` (`load_cell.py:234`).
    pub fn cmd_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        if self.tare_counts.get().is_none() {
            gcmd.respond_info("You must use TARE first.", true);
            return Ok(());
        }
        let grams = gcmd
            .get_float(
                "GRAMS",
                GCmdGetFloatOpts {
                    default: None,
                    minval: Some(50.0),
                    maxval: Some(25000.0),
                    above: None,
                    below: None,
                },
            )?
            .unwrap_or(0.0);
        let cal_counts = self.load_cell.avg_counts(None)?;
        let cal_percent = self.load_cell.counts_to_percent(cal_counts)?;
        let c_per_g = self.counts_per_gram(grams, cal_counts)?;
        let cap_kg = self.capacity_kg(c_per_g)?;
        let msg = fmt(
            "Calibration value: %.2f%% (%i), Counts/gram: %.5f,             Total capacity: +/- %0.2fKg",
            &[
                PctValue::Float(cal_percent),
                PctValue::Float(cal_counts),
                PctValue::Float(c_per_g),
                PctValue::Float(cap_kg),
            ],
        )?;
        gcmd.respond_info(&msg, true);
        let (range_min, range_max) = self.load_cell.saturation_range();
        if cal_counts >= range_max as f64 || cal_counts <= range_min as f64 {
            return Err(KlippyError::Command(
                "ERROR: Sensor is saturated with too much load!\n\
                 Use less force to calibrate the load cell."
                    .to_string(),
            ));
        }
        if Some(cal_counts) == self.tare_counts.get() {
            return Err(KlippyError::Command(
                "ERROR: Tare and Calibration readings are the same!\n\
                 Check wiring and validate sensor with READ_LOAD_CELL command."
                    .to_string(),
            ));
        }
        if (cal_percent - self.tare_percent.get()).abs() < 1.0 {
            return Err(KlippyError::Command(
                "ERROR: Tare and Calibration readings are less than 1% different!\n\
                 Use more force when calibrating or a higher sensor gain."
                    .to_string(),
            ));
        }
        // only set _counts_per_gram after all errors are raised
        self.counts_per_gram.set(Some(c_per_g));
        if cap_kg < 1.0 {
            gcmd.respond_info(
                "WARNING: Load cell capacity is less than 1kg!\n\
                 Check wiring and consider using a lower sensor gain.",
                true,
            );
        }
        if cap_kg > 25.0 {
            gcmd.respond_info(
                "WARNING: Load cell capacity is more than 25Kg!\n\
                 Check wiring and consider using a higher sensor gain.",
                true,
            );
        }
        gcmd.respond_info("Accept calibration with the ACCEPT command.", true);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// load_config / load_config_prefix (`load_cell.py:532`)
// ---------------------------------------------------------------------------

/// A sensor constructor of `load_config`'s `sensors` dict.
type SensorFactory = Box<dyn Fn(&Config) -> KResult<Arc<dyn BulkSensorAdc>> + Send + Sync>;

/// `sensors` (`load_cell.py:534`): the hx71x, ads1220 and ads131m0x sensor
/// type dicts merged in Python's insertion order.
fn sensor_factories() -> IndexMap<&'static str, SensorFactory> {
    let mut sensors: IndexMap<&'static str, SensorFactory> = IndexMap::new();
    for (name, ctor) in crate::hx71x::HX71X_SENSOR_TYPES {
        sensors.insert(name, Box::new(move |c| Ok(ctor(c)? as Arc<dyn BulkSensorAdc>)));
    }
    sensors.insert(
        crate::ads1220::SENSOR_TYPE_NAME,
        Box::new(|c| Ok(crate::ads1220::Ads1220::new(c)? as Arc<dyn BulkSensorAdc>)),
    );
    for (name, ctor) in crate::ads131m0x::ADS131M0X_SENSOR_TYPES {
        sensors.insert(name, Box::new(move |c| Ok(ctor(c)? as Arc<dyn BulkSensorAdc>)));
    }
    sensors
}

/// `sensor_type` keys accepted by `load_config`, in Python dict order.
pub fn sensor_type_names() -> Vec<&'static str> {
    sensor_factories().keys().copied().collect()
}

/// `load_config(config)` (`load_cell.py:532`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let sensors = sensor_factories();
    let mut choices = IndexMap::new();
    for name in sensors.keys() {
        choices.insert(
            PyValue::Str((*name).to_string()),
            PyValue::Str((*name).to_string()),
        );
    }
    let sensor_class = config.getchoice(
        "sensor_type",
        &PyValue::Dict(choices),
        GetChoiceOpts::default(),
    )?;
    let sensor_type = match &sensor_class {
        PyValue::Str(s) => s.clone(),
        other => other.str(),
    };
    let make = sensors.get(sensor_type.as_str()).ok_or_else(|| {
        // `getchoice` only returns a key of the dict it was given.
        KlippyError::Config(format!(
            "Choice '{sensor_type}' for option 'sensor_type' in section '{}' is not a valid choice",
            config.get_name()
        ))
    })?;
    let sensor = make(config)?;
    let cell = LoadCell::new(config, sensor)?;
    Ok(Some(Arc::clone(&cell) as ObjectRef))
}

/// `load_config_prefix(config)` (`load_cell.py:541`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    load_config(config)
}

/// Module entry points matching the Python module (`load_config`,
/// `load_config_prefix`).
pub fn module_factory() -> klippy_api::printer::ModuleFactory {
    klippy_api::printer::ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
