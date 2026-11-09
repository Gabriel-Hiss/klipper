//! Port of `klippy/extras/bulk_sensor.py`.
//!
//! The module has no printer object, config or G-code surface: other
//! extras build sensors on these helpers: [`BatchBulkHelper`]
//! (periodic batching with webhooks clients), [`BulkDataQueue`]
//! (background-thread bulk-message queue), [`ClockSyncRegression`]
//! (chip-clock linear regression) and [`FixedFreqReader`] (fixed
//! sample-format unpacking with timestamp reconstruction). Like
//! `clocksync`, [`module_factory`] is empty.

use indexmap::IndexMap;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::mcu::{
    CommandQueryWrapper, LookupQueryCommandOpts, Mcu, QuerySendOpts, SerialResponseCallback,
};
use klippy_api::object::lookup_interface;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::webhooks::{ClientConnection, WebHooksApi, WebHooksCallback, WebRequest};
use pycompat::text::{float_true_div, PyError};
use pycompat::{PyInt, PyValue};

/// `BATCH_INTERVAL = 0.500` (`bulk_sensor.py:20`).
pub const BATCH_INTERVAL: f64 = 0.5;

/// `SENSOR_BULK_FMT` (`bulk_sensor.py:115`).
pub const SENSOR_BULK_FMT: &str = "sensor_bulk_data oid=%c sequence=%hu data=%*s";

/// Response format of the status query (`bulk_sensor.py:221-223`).
pub const SENSOR_BULK_STATUS_FMT: &str = "sensor_bulk_status oid=%c clock=%u \
query_ticks=%u next_sequence=%hu buffered=%u possible_overflows=%hu";

/// `MAX_BULK_MSG_SIZE = 51` (`bulk_sensor.py:203`).
pub const MAX_BULK_MSG_SIZE: i64 = 51;

/// `batch_cb(eventtime) -> msg` (`bulk_sensor.py:24`): `msg` is kept as
/// a `PyValue` (the Python list of samples) and tested for truthiness.
pub type BatchCallback = Box<dyn FnMut(f64) -> KResult<PyValue> + Send>;

/// `start_cb()` / `stop_cb()` (`bulk_sensor.py:44` / `:61`).
pub type StartStopCallback = Box<dyn FnMut() -> KResult<()> + Send>;

/// `client_cb(msg) -> bool` (`bulk_sensor.py:79-83`): the Python return
/// value is tested for truthiness.
pub type ClientCallback = Box<dyn FnMut(PyValue) -> KResult<bool> + Send>;

// ---------------------------------------------------------------------------
// Helpers mirroring CPython semantics.
// ---------------------------------------------------------------------------

fn internal(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message }
}

/// `KeyError` text is `repr(key)` (`Objects/exceptions.c:KeyError`).
fn key_error(key: &str) -> KlippyError {
    internal("KeyError", pycompat::value::repr_key(key))
}

fn type_error(message: String) -> KlippyError {
    internal("TypeError", message)
}

fn attribute_error(message: String) -> KlippyError {
    internal("AttributeError", message)
}

fn value_error(message: String) -> KlippyError {
    internal("ValueError", message)
}

fn zero_division_error(message: &str) -> KlippyError {
    internal("ZeroDivisionError", message.to_string())
}

fn raise_py_error(error: PyError) -> KlippyError {
    KlippyError::Internal { class_name: error.kind.to_string(), message: error.msg }
}
/// `pycompat::text::float_true_div`, translated to this module's errors.
fn true_div(a: f64, b: f64) -> KResult<f64> {
    float_true_div(a, b).map_err(raise_py_error)
}

/// Extract an `int` parameter (`msgproto` yields ints for `%c`/`%u`/`%hu`).
fn dict_get_i64(params: &PyValue, key: &str) -> KResult<i64> {
    let value = dict_get(params, key)?;
    match &value {
        PyValue::Int(n) => {
            // Genuine MCU fields fit in i64; saturate the unreachable rest.
            match n.0.to_string().parse::<i64>() {
                Ok(v) => Ok(v),
                Err(_) => Err(internal("OverflowError", "int too large to convert to i64".to_string())),
            }
        }
        PyValue::Bool(b) => Ok(i64::from(*b)),
        _ => Err(type_error(format!(
            "unsupported operand type(s) for -: '{}' and 'int'",
            pycompat::value::type_name(&value)
        ))),
    }
}

/// Extract a `bytes` parameter (`msgproto` yields bytes for `%*s`).
fn dict_get_bytes(params: &PyValue, key: &str) -> KResult<Vec<u8>> {
    match dict_get(params, key)? {
        PyValue::Bytes(data) => Ok(data),
        value => Err(type_error(format!(
            "a bytes-like object is required, not '{}'",
            pycompat::value::type_name(&value)
        ))),
    }
}

fn dict_get(params: &PyValue, key: &str) -> KResult<PyValue> {
    let PyValue::Dict(dict) = params else {
        // A params object that is not a dict cannot subscript in Python.
        return Err(type_error(format!(
            "'{}' object is not subscriptable",
            pycompat::value::type_name(params)
        )));
    };
    let key_value = PyValue::Str(key.to_string());
    dict.get(&key_value).cloned().ok_or_else(|| key_error(key))
}

/// Python `struct.error`: the exception class is `struct.error` (`error`).
fn struct_error(message: String) -> KlippyError {
    internal("error", message)
}

// ---------------------------------------------------------------------------
// `struct.Struct` subset (formats used by klippy consumers).
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StructKind {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,
    Bytes,
    Pad,
}

impl StructKind {
    fn size(self) -> usize {
        match self {
            StructKind::I8 | StructKind::U8 | StructKind::Pad => 1,
            StructKind::I16 | StructKind::U16 => 2,
            StructKind::I32 | StructKind::U32 | StructKind::F32 => 4,
            StructKind::I64 | StructKind::U64 | StructKind::F64 => 8,
            // `s` size is the repeat count, handled at the item level.
            StructKind::Bytes => 1,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct StructItem {
    kind: StructKind,
    /// Offset in bytes from the start of the buffer.
    offset: usize,
    /// Byte count for `s` (`Bytes`); otherwise the element size.
    size: usize,
}

/// Port of `struct.Struct` for the type characters klippy uses.
#[derive(Clone, Debug)]
pub struct StructUnpack {
    size: usize,
    items: Vec<StructItem>,
    big_endian: bool,
}

impl StructUnpack {
    /// `struct.Struct(fmt)` (`bulk_sensor.py:211`).
    pub fn new(fmt: &str) -> KResult<StructUnpack> {
        let mut chars = fmt.chars().peekable();
        // Byte-order prefix; absent means `@` (native, little endian here).
        let mut native = true;
        let mut big_endian = false;
        if let Some(&c) = chars.peek() {
            match c {
                '@' => {
                    chars.next();
                }
                '=' => {
                    native = false;
                    chars.next();
                }
                '<' => {
                    native = false;
                    chars.next();
                }
                '>' | '!' => {
                    native = false;
                    big_endian = true;
                    chars.next();
                }
                _ => {}
            }
        }
        let mut items = Vec::new();
        let mut size = 0usize;
        let mut max_align = 1usize;
        while let Some(&c) = chars.peek() {
            // `structmodule.c:get_next_format` skips whitespace.
            if c.is_whitespace() {
                chars.next();
                continue;
            }
            let mut count = 0usize;
            let mut count_digits = false;
            let mut leading_zero = false;
            while let Some(&d) = chars.peek() {
                let Some(digit) = d.to_digit(10) else { break };
                if !count_digits {
                    leading_zero = digit == 0;
                } else if leading_zero {
                    // A repeat count may not start with 0 unless it is
                    // exactly 0 (`struct.Struct("0h")` has size 0).
                    return Err(struct_error("repeat count can't start with 0".to_string()));
                }
                count_digits = true;
                // CPython errors on overflow; saturate the unreachable rest.
                count = count.saturating_mul(10).saturating_add(digit as usize);
                chars.next();
            }
            let Some(spec) = chars.next() else {
                return Err(struct_error(if count_digits {
                    "repeat count given without format specifier".to_string()
                } else {
                    "bad char in struct format".to_string()
                }));
            };
            let kind = match spec {
                'b' => StructKind::I8,
                'B' => StructKind::U8,
                'h' => StructKind::I16,
                'H' => StructKind::U16,
                'i' | 'l' => StructKind::I32,
                'I' | 'L' => StructKind::U32,
                'q' => StructKind::I64,
                'Q' => StructKind::U64,
                'f' => StructKind::F32,
                'd' => StructKind::F64,
                's' => StructKind::Bytes,
                'x' => StructKind::Pad,
                _ => return Err(struct_error("bad char in struct format".to_string())),
            };
            let repeat = if count_digits { count } else { 1 };
            if native {
                // Native alignment: each item is aligned to its size.
                let align = kind.size().max(1);
                max_align = max_align.max(align);
                let pad = (align - (size % align)) % align;
                size += pad;
            }
            // ``s`` is one item of ``repeat`` bytes; other types repeat.
            let (items_count, unit) = if kind == StructKind::Bytes {
                (1usize, repeat)
            } else {
                (repeat, kind.size())
            };
            for _ in 0..items_count {
                items.push(StructItem { kind, offset: size, size: unit });
                size += unit;
            }
        }
        if native {
            // CPython pads the struct up to the largest member alignment.
            size += (max_align - (size % max_align)) % max_align;
        }
        Ok(StructUnpack { size, items, big_endian })
    }

    /// `Struct.size` (`bulk_sensor.py:213`).
    pub fn size(&self) -> usize {
        self.size
    }

    /// `Struct.unpack_from(data, offset)` (`bulk_sensor.py:294`).
    pub fn unpack_from(&self, data: &[u8], offset: usize) -> KResult<Vec<PyValue>> {
        let need = offset + self.size;
        if need > data.len() {
            return Err(struct_error(format!(
                "unpack_from requires a buffer of at least {need} bytes for unpacking {} bytes at offset {offset} (actual buffer size is {})",
                self.size,
                data.len()
            )));
        }
        let buf = &data[offset..];
        let mut values = Vec::new();
        for item in &self.items {
            let bytes = &buf[item.offset..item.offset + item.size];
            let value = match item.kind {
                StructKind::I8 => PyValue::Int(PyInt::from_i64(bytes[0] as i8 as i64)),
                StructKind::U8 => PyValue::Int(PyInt::from_i64(bytes[0] as i64)),
                StructKind::I16 => PyValue::Int(PyInt::from_i64(self.read_i16(bytes))),
                StructKind::U16 => PyValue::Int(PyInt::from_i64(self.read_u16(bytes) as i64)),
                StructKind::I32 => PyValue::Int(PyInt::from_i64(self.read_i32(bytes) as i64)),
                StructKind::U32 => PyValue::Int(PyInt::from_i64(self.read_u32(bytes) as i64)),
                StructKind::I64 => PyValue::Int(PyInt::from_i64(self.read_i64(bytes))),
                StructKind::U64 => PyValue::Int(PyInt::from_i64(self.read_u64(bytes) as i64)),
                StructKind::F32 => PyValue::Float(self.read_f32(bytes) as f64),
                StructKind::F64 => PyValue::Float(self.read_f64(bytes)),
                StructKind::Bytes => PyValue::Bytes(bytes.to_vec()),
                // `x` pads consume a byte and produce no value.
                StructKind::Pad => continue,
            };
            values.push(value);
        }
        Ok(values)
    }

    fn read_u16(&self, bytes: &[u8]) -> u16 {
        let mut b = [0u8; 2];
        b.copy_from_slice(bytes);
        if self.big_endian { u16::from_be_bytes(b) } else { u16::from_le_bytes(b) }
    }
    fn read_i16(&self, bytes: &[u8]) -> i64 {
        self.read_u16(bytes) as i16 as i64
    }
    fn read_u32(&self, bytes: &[u8]) -> u32 {
        let mut b = [0u8; 4];
        b.copy_from_slice(bytes);
        if self.big_endian { u32::from_be_bytes(b) } else { u32::from_le_bytes(b) }
    }
    fn read_i32(&self, bytes: &[u8]) -> i32 {
        self.read_u32(bytes) as i32
    }
    fn read_u64(&self, bytes: &[u8]) -> u64 {
        let mut b = [0u8; 8];
        b.copy_from_slice(bytes);
        if self.big_endian { u64::from_be_bytes(b) } else { u64::from_le_bytes(b) }
    }
    fn read_i64(&self, bytes: &[u8]) -> i64 {
        self.read_u64(bytes) as i64
    }
    fn read_f32(&self, bytes: &[u8]) -> f32 {
        f32::from_bits(self.read_u32(bytes))
    }
    fn read_f64(&self, bytes: &[u8]) -> f64 {
        f64::from_bits(self.read_u64(bytes))
    }
}

// ---------------------------------------------------------------------------
// BatchBulkHelper (`bulk_sensor.py:23-100`).
// ---------------------------------------------------------------------------

struct Client {
    id: u64,
    cb: ClientCallback,
}

/// State shared with the periodic-timer closure (mirrors the Python
/// instance). Reactor-only state lives in [`GCell`]s; no borrow is
/// ever held across a callback call, because callbacks may pause.
struct BatchShared {
    printer: Printer,
    batch_interval: f64,
    batch_cb: GCell<Option<BatchCallback>>,
    start_cb: GCell<Option<StartStopCallback>>,
    stop_cb: GCell<Option<StartStopCallback>>,
    is_started: GCell<bool>,
    batch_timer: GCell<Option<reactor::Timer>>,
    client_cbs: GCell<Vec<Client>>,
    webhooks_start_resp: GCell<PyValue>,
    next_client_id: GCell<u64>,
}

impl BatchShared {
    /// `_start()` with the shared handle for the timer closure.
    fn start_with(&self, handle: &std::sync::Arc<BatchShared>) -> KResult<()> {
        if self.is_started.get() {
            return Ok(());
        }
        self.is_started.set(true);
        match call_start_stop(&self.start_cb) {
            Ok(()) => {}
            Err(e @ KlippyError::Command(_)) => {
                log::exception(
                    "BatchBulkHelper start callback error",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
                self.is_started.set(false);
                self.client_cbs.borrow_mut().clear();
                return Err(e);
            }
            Err(e) => return Err(e),
        }
        let reactor = self.printer.get_reactor();
        let systime = reactor.monotonic();
        let shared = std::sync::Arc::clone(handle);
        let waketime = systime + self.batch_interval;
        let timer = reactor.register_timer(
            move |eventtime| shared.proc_batch(&shared, eventtime)
                .unwrap_or_else(|e| panic!("{}: {}", e.class_name(), e.message())),
            waketime,
        );
        *self.batch_timer.borrow_mut() = Some(timer);
        Ok(())
    }

    fn stop_with(&self, handle: &std::sync::Arc<BatchShared>) -> KResult<()> {
        self.client_cbs.borrow_mut().clear();
        let timer = self.batch_timer.borrow_mut().take();
        let Some(timer) = timer else {
            // Python: `unregister_timer(None)` raises `AttributeError`.
            return Err(attribute_error(
                "'NoneType' object has no attribute 'waketime'".to_string(),
            ));
        };
        self.printer.get_reactor().unregister_timer(&timer);
        if !self.is_started.get() {
            return Ok(());
        }
        match call_start_stop(&self.stop_cb) {
            Ok(()) => {}
            Err(e @ KlippyError::Command(_)) => {
                log::exception(
                    "BatchBulkHelper stop callback error",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
                self.client_cbs.borrow_mut().clear();
            }
            Err(e) => return Err(e),
        }
        self.is_started.set(false);
        if !self.client_cbs.borrow().is_empty() {
            // New client started while in process of stopping.
            self.start_with(handle)?;
        }
        Ok(())
    }

    fn proc_batch(&self, handle: &std::sync::Arc<BatchShared>, eventtime: f64) -> KResult<f64> {
        let msg = match take_batch_cb(&self.batch_cb) {
            Some(mut cb) => {
                let result = cb(eventtime);
                *self.batch_cb.borrow_mut() = Some(cb);
                result
            }
            None => Ok(PyValue::None),
        };
        let msg = match msg {
            Ok(msg) => msg,
            Err(e @ KlippyError::Command(_)) => {
                log::exception(
                    "BatchBulkHelper batch callback error",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
                self.stop_with(handle)?;
                return Ok(reactor::NEVER);
            }
            Err(e) => return Err(e),
        };
        if !msg.truthy() {
            return Ok(eventtime + self.batch_interval);
        }
        // Snapshot the client ids: clients added during iteration are
        // not visited this pass (`list(self.client_cbs)`).
        let ids: Vec<u64> =
            self.client_cbs.borrow().iter().map(|client| client.id).collect();
        for id in ids {
            let Some(idx) = self
                .client_cbs
                .borrow()
                .iter()
                .position(|client| client.id == id)
            else {
                // Cleared by an intervening `_stop`; Python's snapshot
                // would still call it. No real caller stops mid-batch.
                continue;
            };
            let mut cb = std::mem::replace(
                &mut self.client_cbs.borrow_mut()[idx].cb,
                Box::new(|_| Ok(false)) as ClientCallback,
            );
            let result = cb(msg.clone());
            let still_registered = self
                .client_cbs
                .borrow()
                .iter()
                .position(|client| client.id == id);
            match result {
                Ok(false) => {
                    let Some(pos) = still_registered else {
                        // `_stop` cleared the registry during the call:
                        // `list.remove` raises `ValueError` in Python.
                        return Err(value_error("list.remove(x): x not in list".to_string()));
                    };
                    self.client_cbs.borrow_mut().remove(pos);
                    if self.client_cbs.borrow().is_empty() {
                        self.stop_with(handle)?;
                        return Ok(reactor::NEVER);
                    }
                }
                Ok(true) => {
                    if let Some(pos) = still_registered {
                        self.client_cbs.borrow_mut()[pos].cb = cb;
                    }
                }
                Err(e) => {
                    if let Some(pos) = self
                        .client_cbs
                        .borrow()
                        .iter()
                        .position(|client| client.id == id)
                    {
                        self.client_cbs.borrow_mut()[pos].cb = cb;
                    }
                    return Err(e);
                }
            }
        }
        Ok(eventtime + self.batch_interval)
    }

}

fn call_start_stop(cell: &GCell<Option<StartStopCallback>>) -> KResult<()> {
    let Some(mut cb) = cell.borrow_mut().take() else {
        return Ok(());
    };
    let result = cb();
    *cell.borrow_mut() = Some(cb);
    result
}

fn take_batch_cb(cell: &GCell<Option<BatchCallback>>) -> Option<BatchCallback> {
    cell.borrow_mut().take()
}


/// Periodic batching helper (`bulk_sensor.py:23`).
pub struct BatchBulkHelper {
    shared: std::sync::Arc<BatchShared>,
}

impl BatchBulkHelper {
    /// `BatchBulkHelper(printer, batch_cb, start_cb=None, stop_cb=None,
    /// batch_interval=BATCH_INTERVAL)` (`bulk_sensor.py:24`).
    pub fn new(
        printer: Printer,
        batch_cb: BatchCallback,
        start_cb: Option<StartStopCallback>,
        stop_cb: Option<StartStopCallback>,
        batch_interval: f64,
    ) -> BatchBulkHelper {
        let shared = std::sync::Arc::new(BatchShared {
            printer,
            batch_interval,
            batch_cb: GCell::new(Some(batch_cb)),
            start_cb: GCell::new(start_cb),
            stop_cb: GCell::new(stop_cb),
            is_started: GCell::new(false),
            batch_timer: GCell::new(None),
            client_cbs: GCell::new(Vec::new()),
            webhooks_start_resp: GCell::new(PyValue::Dict(IndexMap::new())),
            next_client_id: GCell::new(0),
        });
        BatchBulkHelper { shared }
    }

    /// `self.batch_interval` (`bulk_sensor.py:35`).
    pub fn batch_interval(&self) -> f64 {
        self.shared.batch_interval
    }

    /// `self.is_started` (`bulk_sensor.py:34`).
    pub fn is_started(&self) -> bool {
        self.shared.is_started.get()
    }

    /// `self._start()` (`bulk_sensor.py:40`).
    pub fn _start(&self) -> KResult<()> {
        self.shared.start_with(&self.shared)
    }

    /// `self._stop()` (`bulk_sensor.py:55`).
    pub fn _stop(&self) -> KResult<()> {
        self.shared.stop_with(&self.shared)
    }

    /// `self._proc_batch(eventtime)` (`bulk_sensor.py:70`): returns the
    /// next waketime; a `command_error` from the batch callback stops
    /// batching and yields `NEVER`, any other error propagates.
    pub fn _proc_batch(&self, eventtime: f64) -> KResult<f64> {
        self.shared.proc_batch(&self.shared, eventtime)
    }

    /// `self.add_client(client_cb)` (`bulk_sensor.py:89`).
    pub fn add_client(&self, client_cb: ClientCallback) -> KResult<()> {
        let mut next = self.shared.next_client_id.borrow_mut();
        let id = *next;
        *next += 1;
        drop(next);
        self.shared.client_cbs.borrow_mut().push(Client { id, cb: client_cb });
        self.shared.start_with(&self.shared)
    }

    /// `self._add_api_client(web_request)` (`bulk_sensor.py:93`).
    pub fn _add_api_client(&self, web_request: &WebRequest) -> KResult<()> {
        self.shared.add_api_client(&self.shared, web_request)
    }

    /// `self.add_mux_endpoint(path, key, value, webhooks_start_resp)`
    /// (`bulk_sensor.py:97`).
    pub fn add_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        webhooks_start_resp: PyValue,
    ) -> KResult<()> {
        *self.shared.webhooks_start_resp.borrow_mut() = webhooks_start_resp;
        let webhooks = lookup_interface::<dyn WebHooksApi>(
            &*self.shared.printer,
            "webhooks",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let shared = std::sync::Arc::clone(&self.shared);
        let callback: WebHooksCallback = std::sync::Arc::new(move |web_request| {
            shared.add_api_client(&shared, &web_request)
        });
        webhooks.register_mux_endpoint(path, key, value, callback)
    }
}

impl BatchShared {
    fn add_api_client(
        &self,
        handle: &std::sync::Arc<BatchShared>,
        web_request: &WebRequest,
    ) -> KResult<()> {
        let client = BatchWebhooksClient::new(web_request)?;
        let id = {
            let mut next = self.next_client_id.borrow_mut();
            let id = *next;
            *next += 1;
            id
        };
        self.client_cbs.borrow_mut().push(Client {
            id,
            cb: Box::new(move |msg| client.handle_batch(msg)),
        });
        self.start_with(handle)?;
        web_request.send(self.webhooks_start_resp.cloned())
    }
}



/// Webhooks client (`bulk_sensor.py:103`).
pub struct BatchWebhooksClient {
    cconn: ClientConnection,
    template: PyValue,
}

impl BatchWebhooksClient {
    /// `BatchWebhooksClient(web_request)` (`bulk_sensor.py:104`).
    pub fn new(web_request: &WebRequest) -> KResult<BatchWebhooksClient> {
        Ok(BatchWebhooksClient {
            cconn: web_request.get_client_connection(),
            template: web_request.get_dict(
                "response_template",
                Some(PyValue::Dict(IndexMap::new())),
            )?,
        })
    }

    /// `handle_batch(msg)` (`bulk_sensor.py:107`).
    pub fn handle_batch(&self, msg: PyValue) -> KResult<bool> {
        if self.cconn.is_closed() {
            return Ok(false);
        }
        let mut tmp = match &self.template {
            PyValue::Dict(dict) => dict.clone(),
            PyValue::None => {
                // `dict(None)` in Python.
                return Err(type_error("'NoneType' object is not iterable".to_string()));
            }
            other => {
                return Err(type_error(format!(
                    "'{}' object is not iterable",
                    pycompat::value::type_name(other)
                )));
            }
        };
        tmp.insert(PyValue::Str("params".to_string()), msg);
        self.cconn.send(PyValue::Dict(tmp));
        Ok(true)
    }
}

// ---------------------------------------------------------------------------
// BulkDataQueue (`bulk_sensor.py:118-134`).
// ---------------------------------------------------------------------------

/// Queued bulk-sensor messages; the MCU callback arrives on the
/// serial background thread, `pull_queue` runs on the reactor.
pub struct BulkDataQueue {
    queue: std::sync::Arc<kos::sync::Mutex<Vec<PyValue>>>,
}

impl BulkDataQueue {
    /// `BulkDataQueue(mcu, msg_fmt=SENSOR_BULK_FMT, oid=None)`
    /// (`bulk_sensor.py:119`).
    pub fn new(_mcu: &Mcu, msg_fmt: &str, oid: Option<i64>) -> BulkDataQueue {
        let queue = std::sync::Arc::new(kos::sync::Mutex::new(Vec::new()));
        let pending = std::sync::Arc::clone(&queue);
        let cb: SerialResponseCallback = std::sync::Arc::new(move |params| {
            pending.lock().push(params);
        });
        // Python discards the `AsyncResponseWrapper` handle.
        let _ = _mcu.register_serial_response(cb, msg_fmt, oid);
        BulkDataQueue { queue }
    }

    /// `_handle_data(params)` (`bulk_sensor.py:125`).
    pub fn _handle_data(&self, params: PyValue) {
        self.queue.lock().push(params);
    }

    /// `pull_queue()` (`bulk_sensor.py:128`).
    pub fn pull_queue(&self) -> Vec<PyValue> {
        std::mem::take(&mut *self.queue.lock())
    }

    /// `clear_queue()` (`bulk_sensor.py:133`).
    pub fn clear_queue(&self) {
        self.pull_queue();
    }
}

// ---------------------------------------------------------------------------
// ClockSyncRegression (`bulk_sensor.py:156-201`).
// ---------------------------------------------------------------------------

/// Chip-clock synchronization by linear regression.
pub struct ClockSyncRegression {
    /// `self.mcu` (`bulk_sensor.py:158`).
    pub mcu: Mcu,
    /// `self.chip_clock_smooth` (`bulk_sensor.py:159`).
    pub chip_clock_smooth: f64,
    /// `self.decay` (`bulk_sensor.py:160`).
    pub decay: f64,
    /// `self.last_chip_clock` (`bulk_sensor.py:161`).
    pub last_chip_clock: f64,
    /// `self.last_exp_mcu_clock` (`bulk_sensor.py:161`).
    pub last_exp_mcu_clock: f64,
    /// `self.mcu_clock_avg` (`bulk_sensor.py:162`).
    pub mcu_clock_avg: f64,
    /// `self.mcu_clock_variance` (`bulk_sensor.py:162`).
    pub mcu_clock_variance: f64,
    /// `self.chip_clock_avg` (`bulk_sensor.py:163`).
    pub chip_clock_avg: f64,
    /// `self.chip_clock_covariance` (`bulk_sensor.py:163`).
    pub chip_clock_covariance: f64,
    /// `self.last_mcu_clock` (`bulk_sensor.py:165`); never read.
    pub last_mcu_clock: f64,
}

impl ClockSyncRegression {
    /// `ClockSyncRegression(mcu, chip_clock_smooth, decay=1./20.)`
    /// (`bulk_sensor.py:157`).
    pub fn new(mcu: Mcu, chip_clock_smooth: f64) -> ClockSyncRegression {
        ClockSyncRegression {
            mcu,
            chip_clock_smooth,
            decay: 1. / 20.,
            last_chip_clock: 0.,
            last_exp_mcu_clock: 0.,
            mcu_clock_avg: 0.,
            mcu_clock_variance: 0.,
            chip_clock_avg: 0.,
            chip_clock_covariance: 0.,
            last_mcu_clock: 0.,
        }
    }

    /// `reset(mcu_clock, chip_clock)` (`bulk_sensor.py:164`).
    pub fn reset(&mut self, mcu_clock: f64, chip_clock: f64) {
        self.mcu_clock_avg = mcu_clock;
        self.last_mcu_clock = mcu_clock;
        self.chip_clock_avg = chip_clock;
        self.mcu_clock_variance = 0.;
        self.chip_clock_covariance = 0.;
        self.last_chip_clock = 0.;
        self.last_exp_mcu_clock = 0.;
    }

    /// `update(mcu_clock, chip_clock)` (`bulk_sensor.py:169`).
    pub fn update(&mut self, mcu_clock: f64, chip_clock: f64) {
        let decay = self.decay;
        let diff_mcu_clock = mcu_clock - self.mcu_clock_avg;
        self.mcu_clock_avg += decay * diff_mcu_clock;
        self.mcu_clock_variance =
            (1. - decay) * (self.mcu_clock_variance + diff_mcu_clock * diff_mcu_clock * decay);
        let diff_chip_clock = chip_clock - self.chip_clock_avg;
        self.chip_clock_avg += decay * diff_chip_clock;
        self.chip_clock_covariance = (1. - decay)
            * (self.chip_clock_covariance + diff_mcu_clock * diff_chip_clock * decay);
    }

    /// `set_last_chip_clock(chip_clock)` (`bulk_sensor.py:180`).
    pub fn set_last_chip_clock(&mut self, chip_clock: f64) -> KResult<()> {
        let (base_mcu, base_chip, inv_cfreq) = self.get_clock_translation()?;
        self.last_chip_clock = chip_clock;
        self.last_exp_mcu_clock = base_mcu + (chip_clock - base_chip) * inv_cfreq;
        Ok(())
    }

    /// `get_clock_translation()` (`bulk_sensor.py:184`).
    pub fn get_clock_translation(&self) -> KResult<(f64, f64, f64)> {
        let inv_chip_freq = true_div(self.mcu_clock_variance, self.chip_clock_covariance)?;
        if self.last_chip_clock == 0. {
            return Ok((self.mcu_clock_avg, self.chip_clock_avg, inv_chip_freq));
        }
        // Find mcu clock associated with future chip_clock.
        let s_chip_clock = self.last_chip_clock + self.chip_clock_smooth;
        let scdiff = s_chip_clock - self.chip_clock_avg;
        let s_mcu_clock = self.mcu_clock_avg + scdiff * inv_chip_freq;
        // Calculate frequency to converge at future point.
        let mdiff = s_mcu_clock - self.last_exp_mcu_clock;
        let s_inv_chip_freq = true_div(mdiff, self.chip_clock_smooth)?;
        Ok((self.last_exp_mcu_clock, self.last_chip_clock, s_inv_chip_freq))
    }

    /// `get_time_translation()` (`bulk_sensor.py:196`).
    pub fn get_time_translation(&self) -> KResult<(f64, f64, f64)> {
        let (base_mcu, base_chip, inv_cfreq) = self.get_clock_translation()?;
        let base_time = self.mcu.clock_to_print_time(base_mcu);
        let inv_freq = self.mcu.clock_to_print_time(base_mcu + inv_cfreq) - base_time;
        Ok((base_time, base_chip, inv_freq))
    }
}

// ---------------------------------------------------------------------------
// FixedFreqReader (`bulk_sensor.py:207-299`).
// ---------------------------------------------------------------------------

/// Reads `sensor_bulk_data` and calculates timestamps for samples at a
/// fixed frequency.
///
/// Every method takes `&self`: `_update_clock` sends a blocking query
/// that pauses the reactor, and Python lets other greenlets (for example
/// a sensor's `_finish_measurements` calling `note_end`) run on the same
/// reader during that pause. The mutable state therefore lives in a
/// [`GCell`] whose borrows never span the query.
pub struct FixedFreqReader {
    /// `self.mcu` (`bulk_sensor.py:209`).
    pub mcu: Mcu,
    /// `self.unpack_from` (`bulk_sensor.py:212`).
    pub unpack_from: StructUnpack,
    /// `self.bytes_per_sample` (`bulk_sensor.py:213`).
    pub bytes_per_sample: i64,
    /// `self.samples_per_block` (`bulk_sensor.py:214`).
    pub samples_per_block: i64,
    state: GCell<ReaderState>,
}

/// The attributes of `FixedFreqReader` that change after construction.
struct ReaderState {
    /// `self.clock_sync` (`bulk_sensor.py:210`).
    clock_sync: ClockSyncRegression,
    /// `self.last_sequence` (`bulk_sensor.py:215`).
    last_sequence: i64,
    /// `self.max_query_duration` (`bulk_sensor.py:215`).
    max_query_duration: i64,
    /// `self.last_overflows` (`bulk_sensor.py:216`).
    last_overflows: i64,
    /// `self.bulk_queue` (`bulk_sensor.py:217`).
    bulk_queue: Option<std::sync::Arc<BulkDataQueue>>,
    /// `self.oid` (`bulk_sensor.py:217`).
    oid: Option<i64>,
    /// `self.query_status_cmd` (`bulk_sensor.py:217`).
    query_status_cmd: Option<CommandQueryWrapper>,
}

impl FixedFreqReader {
    /// `FixedFreqReader(mcu, chip_clock_smooth, unpack_fmt)`
    /// (`bulk_sensor.py:208`).
    pub fn new(mcu: Mcu, chip_clock_smooth: f64, unpack_fmt: &str) -> KResult<FixedFreqReader> {
        let unpack = StructUnpack::new(unpack_fmt)?;
        let bytes_per_sample = unpack.size() as i64;
        if bytes_per_sample == 0 {
            // `MAX_BULK_MSG_SIZE // 0` in Python.
            return Err(zero_division_error("integer division or modulo by zero"));
        }
        Ok(FixedFreqReader {
            mcu: std::sync::Arc::clone(&mcu),
            unpack_from: unpack,
            bytes_per_sample,
            samples_per_block: MAX_BULK_MSG_SIZE.div_euclid(bytes_per_sample),
            state: GCell::new(ReaderState {
                clock_sync: ClockSyncRegression::new(mcu, chip_clock_smooth),
                last_sequence: 0,
                max_query_duration: 0,
                last_overflows: 0,
                bulk_queue: None,
                oid: None,
                query_status_cmd: None,
            }),
        })
    }

    /// `setup_query_command(msgformat, oid, cq)` (`bulk_sensor.py:218`).
    pub fn setup_query_command(
        &self,
        msgformat: &str,
        oid: i64,
        cq: Option<klippy_api::mcu::CommandQueue>,
    ) -> KResult<()> {
        // Lookup sensor query command (that responds with sensor_bulk_status).
        self.state.borrow_mut().oid = Some(oid);
        let cmd = self.mcu.lookup_query_command(
            msgformat,
            SENSOR_BULK_STATUS_FMT,
            LookupQueryCommandOpts { oid: Some(oid), cq, is_async: false },
        )?;
        self.state.borrow_mut().query_status_cmd = Some(cmd);
        // Read sensor_bulk_data messages and store in a queue.
        let queue = std::sync::Arc::new(BulkDataQueue::new(&self.mcu, SENSOR_BULK_FMT, Some(oid)));
        self.state.borrow_mut().bulk_queue = Some(queue);
        Ok(())
    }

    /// `self.last_sequence` (`bulk_sensor.py:215`).
    pub fn last_sequence(&self) -> i64 {
        self.state.borrow().last_sequence
    }

    /// `self.max_query_duration` (`bulk_sensor.py:215`).
    pub fn max_query_duration(&self) -> i64 {
        self.state.borrow().max_query_duration
    }

    /// `get_last_overflows()` (`bulk_sensor.py:227`).
    pub fn get_last_overflows(&self) -> i64 {
        self.state.borrow().last_overflows
    }

    fn _clear_duration_filter(&self) {
        self.state.borrow_mut().max_query_duration = 1 << 31;
    }

    /// `self.bulk_queue`, raising Python's `AttributeError` for `method`
    /// while it is still `None`.
    fn bulk_queue(&self, method: &str) -> KResult<std::sync::Arc<BulkDataQueue>> {
        self.state.borrow().bulk_queue.clone().ok_or_else(|| {
            attribute_error(format!("'NoneType' object has no attribute '{method}'"))
        })
    }

    /// `note_start()` (`bulk_sensor.py:231`).
    pub fn note_start(&self) -> KResult<()> {
        {
            let mut state = self.state.borrow_mut();
            state.last_sequence = 0;
            state.last_overflows = 0;
        }
        // Clear local queue (clear any stale samples from previous session).
        self.bulk_queue("clear_queue")?.clear_queue();
        // Set initial clock.
        self._clear_duration_filter();
        self._update_clock(true)?;
        self._clear_duration_filter();
        Ok(())
    }

    /// `note_end()` (`bulk_sensor.py:240`).
    pub fn note_end(&self) -> KResult<()> {
        // Clear local queue (free no longer needed memory).
        self.bulk_queue("clear_queue")?.clear_queue();
        Ok(())
    }

    /// `_update_clock(is_reset=False)` (`bulk_sensor.py:243`).
    pub fn _update_clock(&self, is_reset: bool) -> KResult<()> {
        let (cmd, oid) = {
            let state = self.state.borrow();
            (state.query_status_cmd.clone(), state.oid)
        };
        let Some(cmd) = cmd else {
            // Python: `self.query_status_cmd.send` on `None`.
            return Err(attribute_error("'NoneType' object has no attribute 'send'".to_string()));
        };
        let oid_value = oid.map_or(PyValue::None, |v| PyValue::Int(PyInt::from_i64(v)));
        // The query pauses the reactor; no borrow of `state` is held here.
        let params = cmd.send(vec![oid_value], QuerySendOpts::default())?;
        let mcu_clock = self.mcu.clock32_to_clock64(dict_get_i64(&params, "clock")?);
        let next_sequence = dict_get_i64(&params, "next_sequence")?;
        let buffered = dict_get_i64(&params, "buffered")?;
        let possible_overflows = dict_get_i64(&params, "possible_overflows")?;
        let duration = dict_get_i64(&params, "query_ticks")?;
        let mut state = self.state.borrow_mut();
        let seq_diff = (next_sequence - state.last_sequence) & 0xffff;
        state.last_sequence += seq_diff;
        let po_diff = (possible_overflows - state.last_overflows) & 0xffff;
        state.last_overflows += po_diff;
        if duration > state.max_query_duration {
            // Skip measurement as a high query time could skew clock tracking.
            let doubled = 2 * state.max_query_duration;
            let minimum = self.mcu.seconds_to_clock(0.000005);
            state.max_query_duration = doubled.max(minimum);
            return Ok(());
        }
        state.max_query_duration = 2 * duration;
        let msg_count = state.last_sequence * self.samples_per_block
            + buffered.div_euclid(self.bytes_per_sample);
        // The "chip clock" is the message counter plus .5 for average
        // inaccuracy of query responses and plus .5 for assumed offset
        // of hardware processing time.
        let chip_clock = msg_count + 1;
        let avg_mcu_clock = mcu_clock + duration.div_euclid(2);
        if is_reset {
            state.clock_sync.reset(avg_mcu_clock as f64, chip_clock as f64);
        } else {
            state.clock_sync.update(avg_mcu_clock as f64, chip_clock as f64);
        }
        Ok(())
    }

    /// `pull_samples()` (`bulk_sensor.py:270`): the list of samples.
    pub fn pull_samples(&self) -> KResult<Vec<PyValue>> {
        // Query MCU for sample timing and update clock synchronization.
        self._update_clock(false)?;
        // Pull sensor_bulk_data messages from local queue.
        let raw_samples = self.bulk_queue("pull_queue")?.pull_queue();
        if raw_samples.is_empty() {
            return Ok(Vec::new());
        }
        // Load variables to optimize inner loop below.
        let (last_sequence, (time_base, chip_base, inv_freq)) = {
            let state = self.state.borrow();
            (state.last_sequence, state.clock_sync.get_time_translation()?)
        };
        let bytes_per_sample = self.bytes_per_sample;
        let samples_per_block = self.samples_per_block;
        let unpack_from = &self.unpack_from;
        // Process every message in raw_samples.
        let mut count = 0usize;
        let mut seq = 0i64;
        let mut samples = vec![PyValue::None; raw_samples.len() * samples_per_block as usize];
        // `i` leaks across messages in Python; it stays bound from a
        // previous message when a message carries no whole sample.
        let mut leaked_i: Option<i64> = None;
        for params in &raw_samples {
            let mut seq_diff = (dict_get_i64(params, "sequence")? - last_sequence) & 0xffff;
            seq_diff -= (seq_diff & 0x8000) << 1;
            seq = last_sequence + seq_diff;
            let msg_cdiff = seq as f64 * samples_per_block as f64 - chip_base;
            let data = dict_get_bytes(params, "data")?;
            let per_message = (data.len() as i64).div_euclid(bytes_per_sample);
            for i in 0..per_message {
                leaked_i = Some(i);
                let ptime = time_base + (msg_cdiff + i as f64) * inv_freq;
                let udata = unpack_from.unpack_from(&data, (i * bytes_per_sample) as usize)?;
                if count >= samples.len() {
                    // `samples[count] = ...` past the end in Python.
                    return Err(internal(
                        "IndexError",
                        "list assignment index out of range".to_string(),
                    ));
                }
                let mut tuple = Vec::with_capacity(udata.len() + 1);
                tuple.push(PyValue::Float(ptime));
                tuple.extend(udata);
                samples[count] = PyValue::Tuple(tuple);
                count += 1;
            }
        }
        let Some(i) = leaked_i else {
            // `seq * samples_per_block + i` with no bound `i` in Python.
            return Err(internal("NameError", "name 'i' is not defined".to_string()));
        };
        self.state
            .borrow_mut()
            .clock_sync
            .set_last_chip_clock((seq * samples_per_block + i) as f64)?;
        samples.truncate(count);
        Ok(samples)
    }
}

/// The Python module defines no entry points (helpers only), so the
/// factory is empty (`klippy/extras/bulk_sensor.py`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
