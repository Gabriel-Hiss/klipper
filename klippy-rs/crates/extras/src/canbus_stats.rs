//! Port of the `canbus_stats` module (`klippy/extras/canbus_stats.py`).
//!
//! Reports CAN-bus error counters for one MCU over the
//! `get_canbus_status` query command, plus the optional
//! `usb_canbus_state` serial response of a USB-to-CAN bridge.

use indexmap::IndexMap;
use klippy_api::config::{Config, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{
    CommandQueryWrapper, LookupQueryCommandOpts, Mcu, McuApi, QuerySendOpts,
    SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, PctValue};
use pycompat::value::{repr_key, type_name};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, NOW};
use std::sync::Arc;

/// `get_canbus_status` query command name (`klippy/extras/canbus_stats.py:34`).
const GET_CANBUS_STATUS_MSG: &str = "get_canbus_status";
/// Response format of the query command (`klippy/extras/canbus_stats.py:39-41`).
const GET_CANBUS_STATUS_RESP: &str = "canbus_status rx_error=%u tx_error=%u tx_retries=%u canbus_bus_state=%u";
/// Optional USB-to-CAN bridge response (`klippy/extras/canbus_stats.py:43`).
const USB_CANBUS_STATE_MSG: &str = "usb_canbus_state discard=%u";

/// Counter dict (`self.status`, `klippy/extras/canbus_stats.py:15-16`).
struct Status {
    rx_error: Option<i64>,
    tx_error: Option<i64>,
    tx_retries: Option<i64>,
    bus_state: Option<PyValue>,
}

impl Default for Status {
    fn default() -> Self {
        Status {
            rx_error: None,
            tx_error: None,
            tx_retries: None,
            bus_state: None,
        }
    }
}

/// State shared with the `klippy:connect` / `klippy:shutdown` handlers and
/// the query timer, mirroring the Python instance referenced from
/// `__init__` and `handle_connect`.
struct Shared {
    state: GCell<Status>,
    mcu: GCell<Option<Mcu>>,
    cmd: GCell<Option<CommandQueryWrapper>>,
}

impl Default for Shared {
    fn default() -> Self {
        Shared {
            state: GCell::new(Status::default()),
            mcu: GCell::new(None),
            cmd: GCell::new(None),
        }
    }
}

/// Port of `PrinterCANBusStats` (`klippy/extras/canbus_stats.py:8`).
pub struct PrinterCANBusStats {
    printer: Printer,
    reactor: Reactor,
    name: String,
    shared: Arc<Shared>,
}

fn pct_str(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant canbus_stats format")
}

/// `params[key]` (`klippy/extras/canbus_stats.py:49,63-67`).
fn dict_get<'a>(params: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match params {
        PyValue::Dict(d) => {
            for (k, v) in d.iter() {
                if matches!(k, PyValue::Str(s) if s == key) {
                    return Ok(v);
                }
            }
            Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: repr_key(key),
            })
        }
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!("'{}' object is not subscriptable", type_name(other)),
        }),
    }
}

/// `int(params[key])` for the `%u` counter fields. Python accepts `bool`
/// (an `int` subclass) and rejects everything else with the `-`
/// `TypeError`; only the low 32 bits are ever observed after the mask,
/// but a value outside the `i64` range would overflow here where CPython
/// (unbounded) would not. MCU `%u` values are at most `2**32 - 1`.
fn dict_get_int(params: &PyValue, key: &str) -> KResult<i64> {
    let v = dict_get(params, key)?;
    match v {
        PyValue::Int(n) => {
            let s = n.inner().to_string();
            s.parse::<i64>().map_err(|_| KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!(
                    "unsupported operand type(s) for -: '{}' and 'int'",
                    type_name(v)
                ),
            })
        }
        PyValue::Bool(b) => Ok(if *b { 1 } else { 0 }),
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "unsupported operand type(s) for -: '{}' and 'int'",
                type_name(other)
            ),
        }),
    }
}

/// `handle_usb_canbus_state(params)` (`klippy/extras/canbus_stats.py:48-55`).
fn usb_state_impl(name: &str, params: &PyValue) -> KResult<()> {
    let discard = dict_get(params, "discard")?;
    if discard.truthy() {
        log::warning(&pct_str("USB CANBUS bridge '%s' is discarding!", &[PctValue::Str(name.to_string())]));
    } else {
        log::warning(&pct_str("USB CANBUS bridge '%s' is no longer discarding.", &[PctValue::Str(name.to_string())]));
    }
    Ok(())
}

/// `handle_shutdown` (`klippy/extras/canbus_stats.py:21-26`).
fn shutdown_impl(shared: &Shared) {
    let mut st = shared.state.borrow_mut();
    if st.bus_state.is_some() {
        st.bus_state = Some(PyValue::Str("unknown".to_string()));
    }
}

/// `query_event(eventtime)` (`klippy/extras/canbus_stats.py:56-70`).
fn query_event_impl(shared: &Shared, reactor: &Reactor, _eventtime: f64) -> KResult<f64> {
    let (prev_rx, prev_tx, prev_retries) = {
        let st = shared.state.borrow();
        match st.rx_error {
            None => (0, 0, 0),
            Some(prev_rx) => {
                let prev_tx = st.tx_error.ok_or_else(|| KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "unsupported operand type(s) for -: 'int' and 'NoneType'".to_string(),
                })?;
                let prev_retries = st.tx_retries.ok_or_else(|| KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "unsupported operand type(s) for -: 'int' and 'NoneType'".to_string(),
                })?;
                (prev_rx, prev_tx, prev_retries)
            }
        }
    };
    // Copy the handle out before `send`, which pauses the reactor.
    let cmd = (*shared.cmd.borrow()).clone();
    let cmd = match cmd {
        Some(c) => c,
        None => {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'send'".to_string(),
            });
        }
    };
    let params = cmd.send(Vec::new(), QuerySendOpts::default())?;
    let rx_cur = dict_get_int(&params, "rx_error")?;
    let tx_cur = dict_get_int(&params, "tx_error")?;
    let retries_cur = dict_get_int(&params, "tx_retries")?;
    let state = dict_get(&params, "canbus_bus_state")?.clone();
    // The hardware counters are 32-bit; unwrap the delta modulo 2**32.
    let rx = prev_rx.wrapping_add(rx_cur.wrapping_sub(prev_rx) & 0xffffffff);
    let tx = prev_tx.wrapping_add(tx_cur.wrapping_sub(prev_tx) & 0xffffffff);
    let retries = prev_retries.wrapping_add(retries_cur.wrapping_sub(prev_retries) & 0xffffffff);
    {
        let mut st = shared.state.borrow_mut();
        st.rx_error = Some(rx);
        st.tx_error = Some(tx);
        st.tx_retries = Some(retries);
        st.bus_state = Some(state);
    }
    Ok(reactor.monotonic() + 1.)
}

/// `handle_connect` (`klippy/extras/canbus_stats.py:27-47`).
fn connect_impl(printer: &Printer, reactor: &Reactor, name: &str, shared: &Arc<Shared>) -> KResult<()> {
    let mcu_name = if name == "mcu" { "mcu".to_string() } else { format!("mcu {name}") };
    let mcu = lookup_interface::<dyn McuApi>(
        &**printer,
        &mcu_name,
        LookupDefault::Raise,
    )?
    .expect("LookupDefault::Raise never yields None");
    *shared.mcu.borrow_mut() = Some(mcu.clone());
    if mcu.try_lookup_command(GET_CANBUS_STATUS_MSG).is_none() {
        let configfile = lookup_interface::<dyn PrinterConfigApi>(
            &**printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        configfile.deprecate_mcu_code(&mcu, GET_CANBUS_STATUS_MSG, None);
        return Ok(());
    }
    let cmd = mcu.lookup_query_command(
        GET_CANBUS_STATUS_MSG,
        GET_CANBUS_STATUS_RESP,
        LookupQueryCommandOpts::default(),
    )?;
    *shared.cmd.borrow_mut() = Some(cmd);
    if mcu.check_valid_response(USB_CANBUS_STATE_MSG) {
        let cb_name = name.to_string();
        let cb: SerialResponseCallback = Arc::new(move |params: PyValue| {
            if let Err(e) = usb_state_impl(&cb_name, &params) {
                e.raise();
            }
        });
        mcu.register_serial_response(cb, USB_CANBUS_STATE_MSG, None);
    }
    let timer_shared = Arc::clone(shared);
    let timer_reactor = reactor.clone();
    reactor.register_timer(
        move |eventtime| query_event_impl(&timer_shared, &timer_reactor, eventtime).unwrap_or_else(|e| e.raise()),
        NOW,
    );
    Ok(())
}

impl PrinterCANBusStats {
    /// `PrinterCANBusStats.__init__(config)` (`klippy/extras/canbus_stats.py:9-20`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })?
            .to_string();
        let shared = Arc::new(Shared::default());
        let connect_printer = printer.clone();
        let connect_reactor = reactor.clone();
        let connect_name = name.clone();
        let connect_shared = Arc::clone(&shared);
        let connect_cb: EventCallback = Box::new(move |_params: &[EventArg]| {
            connect_impl(&connect_printer, &connect_reactor, &connect_name, &connect_shared)?;
            Ok(PyValue::None)
        });
        printer.register_event_handler("klippy:connect", connect_cb);
        let shutdown_shared = Arc::clone(&shared);
        let shutdown_cb: EventCallback = Box::new(move |_params: &[EventArg]| {
            shutdown_impl(&shutdown_shared);
            Ok(PyValue::None)
        });
        printer.register_event_handler("klippy:shutdown", shutdown_cb);
        Ok(PrinterCANBusStats {
            printer,
            reactor,
            name,
            shared,
        })
    }

    /// Short MCU name (`self.name`, `klippy/extras/canbus_stats.py:12`).
    pub fn get_name(&self) -> &str {
        &self.name
    }

    /// Current `self.mcu` (`klippy/extras/canbus_stats.py:13`).
    pub fn get_mcu(&self) -> Option<Mcu> {
        self.shared.mcu.borrow().clone()
    }

    /// Whether `self.get_canbus_status_cmd` is set.
    pub fn has_query_cmd(&self) -> bool {
        self.shared.cmd.borrow().is_some()
    }

    /// `handle_connect` (`klippy/extras/canbus_stats.py:27`).
    pub fn handle_connect(&self) -> KResult<()> {
        connect_impl(&self.printer, &self.reactor, &self.name, &self.shared)
    }

    /// `handle_shutdown` (`klippy/extras/canbus_stats.py:21`).
    pub fn handle_shutdown(&self) {
        shutdown_impl(&self.shared);
    }

    /// `handle_usb_canbus_state(params)` (`klippy/extras/canbus_stats.py:48`).
    pub fn handle_usb_canbus_state(&self, params: &PyValue) -> KResult<()> {
        usb_state_impl(&self.name, params)
    }

    /// `query_event(eventtime)` (`klippy/extras/canbus_stats.py:56`).
    pub fn query_event(&self, eventtime: f64) -> KResult<f64> {
        query_event_impl(&self.shared, &self.reactor, eventtime)
    }

    /// Current `self.status` as a `PyValue` dict (`get_status` payload).
    pub fn status_value(&self) -> PyValue {
        let st = self.shared.state.borrow();
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("rx_error".to_string()),
            st.rx_error.map(|v| PyValue::Int(PyInt::from_i64(v))).unwrap_or(PyValue::None),
        );
        d.insert(
            PyValue::Str("tx_error".to_string()),
            st.tx_error.map(|v| PyValue::Int(PyInt::from_i64(v))).unwrap_or(PyValue::None),
        );
        d.insert(
            PyValue::Str("tx_retries".to_string()),
            st.tx_retries.map(|v| PyValue::Int(PyInt::from_i64(v))).unwrap_or(PyValue::None),
        );
        d.insert(
            PyValue::Str("bus_state".to_string()),
            st.bus_state.clone().unwrap_or(PyValue::None),
        );
        PyValue::Dict(d)
    }

    /// `stats(eventtime)` (`klippy/extras/canbus_stats.py:71-78`).
    pub fn stats_tuple(&self, _eventtime: f64) -> (bool, String) {
        let st = self.shared.state.borrow();
        let Some(rx) = st.rx_error else {
            return (false, String::new());
        };
        let tx = st.tx_error.unwrap_or(0);
        let retries = st.tx_retries.unwrap_or(0);
        let bus_state_text = match &st.bus_state {
            None => "None".to_string(),
            Some(v) => v.str(),
        };
        let msg = pct_str(
            "canstat_%s: bus_state=%s rx_error=%d tx_error=%d tx_retries=%d",
            &[
                PctValue::Str(self.name.clone()),
                PctValue::Str(bus_state_text),
                PctValue::Int(PyInt::from_i64(rx)),
                PctValue::Int(PyInt::from_i64(tx)),
                PctValue::Int(PyInt::from_i64(retries)),
            ],
        );
        (false, msg)
    }
}

impl PrinterObject for PrinterCANBusStats {
    klippy_api::printer_object_base!();

    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.status_value()))
    }

    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(self.stats_tuple(eventtime))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/canbus_stats.py:82-83`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PrinterCANBusStats::new(config)?) as ObjectRef))
}

/// Module entry point for the `canbus_stats` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
