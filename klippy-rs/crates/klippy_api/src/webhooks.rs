//! Contract of the `webhooks` service.
//!
//! Covers `klippy/webhooks.py` `WebHooks`, `WebRequest` and `ClientConnection`
//! as used by extras (`register_endpoint`, `register_mux_endpoint`,
//! `WebRequest`, `ClientConnection`, `call_remote_method`, `get_status`).
//! Reactor types live in the `reactor` crate; bulk sensor helpers are a
//! ported module, not contract.
//!
//! Mapping notes (faithfulness trade-offs, see report):
//! - `WebRequestError` (a `gcode.CommandError` subclass) is carried as
//!   `KlippyError::Other { class_name: "webhooks.WebRequestError", .. }` so
//!   `class_name()`/`message()` and `to_dict()` (`{'error', 'message'}`)
//!   keep the exact Python class and text.
//! - `WebRequest.get`/`get_dict` defaults use `Option<PyValue>` where outer
//!   `None` is the `Sentinel` (required) and `Some(PyValue::None)` is an
//!   explicit `None` default; `get_dict` returns `PyValue` (dict or `None`)
//!   so the `get_dict('client_info', None)` and `get_dict(..., {})` calls
//!   both type-check. `get_str`/`get_int`/`get_float` use typed defaults
//!   (`Option<String>`/`Option<i64>`/`Option<f64>`) because the inventory
//!   shows no `None` default on those.
//! - `WebRequest.id` is `Option<PyValue>` (any JSON id, `None` for
//!   notifications) rather than only `i64`.
//! - Python `web_request.error(...)` (class attribute `error =
//!   WebRequestError`) is not a trait method; handlers construct
//!   `KlippyError::Other { class_name: "webhooks.WebRequestError", .. }` and
//!   `set_error` wraps any `KlippyError` to that `to_dict` shape.
//! - Python public attributes (`id`, `method`, `params`, `response`,
//!   `is_error`, `client_conn`, `uid`) are accessor methods alongside the
//!   `get_*` methods; both Python forms are kept.

use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::PyValue;

use crate::error::{KResult, KlippyError};

/// Handle to the `webhooks` service (`klippy/webhooks.py:315`).
pub type WebHooks = Arc<dyn WebHooksApi>;
/// Handle to a web request (`klippy/webhooks.py:53`).
pub type WebRequest = Arc<dyn WebRequestApi>;
/// Handle to a client connection (`klippy/webhooks.py:187`).
pub type ClientConnection = Arc<dyn ClientConnectionApi>;

/// Endpoint/mux handler: `callback(web_request) -> None` (`klippy/webhooks.py:328`).
/// Stored and called many times from several places, hence `Arc<dyn Fn ..>`. An error
/// takes the `_process_request` path (`webhooks.py:265-276`): `command_error` and
/// `WebRequestError` become the request error, anything else also shuts down.
pub type WebHooksCallback = Arc<dyn Fn(WebRequest) -> KResult<()> + Send + Sync + 'static>;

/// The `webhooks` object (`klippy/webhooks.py:315`).
pub trait WebHooksApi: Send + Sync + 'static {
    /// klippy/webhooks.py:328
    fn register_endpoint(&self, path: &str, callback: WebHooksCallback) -> KResult<()>;
    /// klippy/webhooks.py:333
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        callback: WebHooksCallback,
    ) -> KResult<()>;
    /// klippy/webhooks.py:421
    fn call_remote_method(
        &self,
        method: &str,
        params: &IndexMap<String, PyValue>,
    ) -> KResult<()>;
    /// klippy/webhooks.py:414
    fn get_status(&self, eventtime: f64) -> PyValue;
}

/// A decoded webhooks request (`klippy/webhooks.py:53`).
pub trait WebRequestApi: Send + Sync + 'static {
    /// klippy/webhooks.py:56
    fn client_conn(&self) -> ClientConnection;
    /// klippy/webhooks.py:60
    fn id(&self) -> Option<PyValue>;
    /// klippy/webhooks.py:61
    fn method(&self) -> String;
    /// klippy/webhooks.py:62
    fn params(&self) -> PyValue;
    /// klippy/webhooks.py:65
    fn response(&self) -> Option<PyValue>;
    /// klippy/webhooks.py:66
    fn is_error(&self) -> bool;
    /// klippy/webhooks.py:68
    fn get_client_connection(&self) -> ClientConnection;
    /// klippy/webhooks.py:71
    fn get(&self, item: &str, default: Option<PyValue>) -> KResult<PyValue>;
    /// klippy/webhooks.py:80
    fn get_str(&self, item: &str, default: Option<String>) -> KResult<String>;
    /// klippy/webhooks.py:83
    fn get_int(&self, item: &str, default: Option<i64>) -> KResult<i64>;
    /// klippy/webhooks.py:86
    fn get_float(&self, item: &str, default: Option<f64>) -> KResult<f64>;
    /// klippy/webhooks.py:89
    fn get_dict(&self, item: &str, default: Option<PyValue>) -> KResult<PyValue>;
    /// klippy/webhooks.py:92
    fn get_method(&self) -> String;
    /// klippy/webhooks.py:95
    fn set_error(&self, error: KlippyError);
    /// klippy/webhooks.py:99
    fn send(&self, data: PyValue) -> KResult<()>;
    /// klippy/webhooks.py:104
    fn finish(&self) -> Option<PyValue>;
}

/// A webhooks client connection (`klippy/webhooks.py:187`).
pub trait ClientConnectionApi: Send + Sync + 'static {
    /// klippy/webhooks.py:193
    fn uid(&self) -> u64;
    /// klippy/webhooks.py:203
    fn dump_request_log(&self);
    /// klippy/webhooks.py:211
    fn set_client_info(&self, client_info: PyValue, state_msg: Option<String>);
    /// klippy/webhooks.py:222
    fn close(&self);
    /// klippy/webhooks.py:234
    fn is_closed(&self) -> bool;
    /// klippy/webhooks.py:282
    fn send(&self, data: PyValue);
}
