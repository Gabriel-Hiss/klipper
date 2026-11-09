//! Port of the `query_endstops` module (`klippy/extras/query_endstops.py`).

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCodeApi, GCodeCommandHandler, GCmd, RegisterCommandOpts};
use klippy_api::lookup_interface;
use klippy_api::mcu::Endstop;
use klippy_api::object::ObjectRef;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::stepper::QueryEndstopsApi;
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::webhooks::{WebHooksApi, WebRequest};
use pycompat::PyValue;

fn lookup_gcode(printer: &Printer) -> KResult<Arc<dyn GCodeApi>> {
    Ok(lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None"))
}

fn lookup_webhooks(printer: &Printer) -> KResult<Arc<dyn WebHooksApi>> {
    Ok(lookup_interface::<dyn WebHooksApi>(&**printer, "webhooks", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None"))
}

fn lookup_toolhead(printer: &Printer) -> KResult<Arc<dyn ToolHeadApi>> {
    Ok(lookup_interface::<dyn ToolHeadApi>(&**printer, "toolhead", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None"))
}

struct Inner {
    endstops: Vec<(Endstop, String)>,
    last_state: Vec<(String, i64)>,
}

/// Port of the `query_endstops` (`klippy/extras/query_endstops.py`).
pub struct QueryEndstops {
    printer: Printer,
    inner: GCell<Inner>,
}

impl QueryEndstops {
    /// `QueryEndstops(config)` (`klippy/extras/query_endstops.py:8-19`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let this = Arc::new(QueryEndstops {
            printer: config.get_printer(),
            inner: GCell::new(Inner {
                endstops: Vec::new(),
                last_state: Vec::new(),
            }),
        });
        let webhooks = lookup_webhooks(&this.printer)?;
        let this_cb = Arc::clone(&this);
        webhooks.register_endpoint(
            "query_endstops/status",
            Arc::new(move |web_request| this_cb.handle_web_request(web_request)),
        )?;
        let gcode = lookup_gcode(&this.printer)?;
        let this_g = Arc::clone(&this);
        let handler: GCodeCommandHandler = Box::new(move |gcmd: &GCmd| {
            this_g.cmd_query_endstops(gcmd)
        });
        gcode.register_command(
            "QUERY_ENDSTOPS",
            Some(handler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Report on the status of each endstop".to_string()),
            },
        )?;
        let this_m = Arc::clone(&this);
        let handler_m119: GCodeCommandHandler =
            Box::new(move |gcmd: &GCmd| this_m.cmd_query_endstops(gcmd));
        gcode.register_command(
            "M119",
            Some(handler_m119),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: None,
            },
        )?;
        Ok(this)
    }

    /// `get_status` (`klippy/extras/query_endstops.py:22-23`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut last_query = IndexMap::new();
        for (name, value) in self.inner.borrow().last_state.iter() {
            last_query.insert(PyValue::Str(name.clone()), PyValue::Int((*value).into()));
        }
        let mut out = IndexMap::new();
        out.insert(
            PyValue::Str("last_query".to_string()),
            PyValue::Dict(last_query),
        );
        PyValue::Dict(out)
    }

    /// Query every endstop and publish `last_state`
    /// (`klippy/extras/query_endstops.py:29-30,37-38`).
    ///
    /// The endstop list is cloned out before the reactor-pausing MCU
    /// queries, and `last_state` is assigned only after every query
    /// succeeds, so a failure leaves the previous state in place.
    fn query_all(&self, print_time: f64) -> KResult<Vec<(String, i64)>> {
        let endstops = self.inner.borrow().endstops.clone();
        let mut state = Vec::with_capacity(endstops.len());
        for (endstop, name) in &endstops {
            let value = endstop.query_endstop(print_time)?;
            state.push((name.clone(), value));
        }
        self.inner.with_mut(|inner| {
            inner.last_state = state.clone();
        });
        Ok(state)
    }

    /// `_handle_web_request` (`klippy/extras/query_endstops.py:24-32`).
    pub fn handle_web_request(&self, web_request: WebRequest) -> KResult<()> {
        let gc_mutex = lookup_gcode(&self.printer)?.get_mutex();
        let toolhead = lookup_toolhead(&self.printer)?;
        gc_mutex.lock();
        let result = (|| -> KResult<PyValue> {
            let print_time = toolhead.get_last_move_time()?;
            let state = self.query_all(print_time)?;
            let mut out = IndexMap::new();
            for (name, value) in &state {
                let label = if *value != 0 { "TRIGGERED" } else { "open" };
                out.insert(PyValue::Str(name.clone()), PyValue::Str(label.to_string()));
            }
            Ok(PyValue::Dict(out))
        })();
        gc_mutex.unlock();
        let payload = result?;
        web_request.send(payload)
    }

    /// `cmd_QUERY_ENDSTOPS` (`klippy/extras/query_endstops.py:34-42`).
    /// Also registered as `M119`.
    pub fn cmd_query_endstops(&self, gcmd: &GCmd) -> KResult<()> {
        let toolhead = lookup_toolhead(&self.printer)?;
        let print_time = toolhead.get_last_move_time()?;
        let state = self.query_all(print_time)?;
        let parts: Vec<String> = state
            .iter()
            .map(|(name, value)| {
                let label = if *value != 0 { "TRIGGERED" } else { "open" };
                format!("{}:{}", name, label)
            })
            .collect();
        gcmd.respond_raw(&parts.join(" "));
        Ok(())
    }
}

impl QueryEndstopsApi for QueryEndstops {
    /// `register_endstop` (`klippy/extras/query_endstops.py:20-21`).
    fn register_endstop(&self, mcu_endstop: Endstop, name: &str) {
        self.inner.with_mut(|inner| {
            inner.endstops.push((mcu_endstop, name.to_string()));
        });
    }
}

impl klippy_api::PrinterObject for QueryEndstops {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn QueryEndstopsApi);

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(QueryEndstops::get_status(self, eventtime)))
    }
}

/// `load_config` (`klippy/extras/query_endstops.py:44-45`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(QueryEndstops::new(config)? as ObjectRef))
}

/// Module factory for the `query_endstops` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
