//! Port of the `output_pin` module (`klippy/extras/output_pin.py`).
//!
//! Three Python helpers are ported here: the [`GCodeRequestQueue`] (the
//! look-ahead / async G-code request queue around a pin callback), the
//! [`PrinterTemplateEvaluator`] (the periodic display-template renderer
//! shared through the printer object `template_evaluator`) and
//! [`PrinterOutputPin`] (the `[output_pin <name>]` object behind
//! `SET_PIN`).
//!
//! The display templates come from `display.lookup_display_templates` and
//! the render context from `gcode_macro.create_template_context`, through
//! the interfaces their owners declare ([`DisplayTemplateApi`],
//! [`PrinterGCodeMacroApi`]). Python bugs are reproduced, not fixed.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

use crate::display::display::{lookup_display_templates, template_render_func, DisplayTemplateApi};
use crate::gcode_macro::PrinterGCodeMacroApi;
use extras_macros_templates::gcode_macro::{parse_literal, Value};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GetFloatOpts as GcmdGetFloatOpts, GetOpts as GcmdGetOpts, GCodeApi, GCmd};
use klippy_api::mcu::{DigitalOutPin, Mcu, Pin, PinsApi, PwmPin};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::toolhead::{MotionQueuing, MotionQueuingApi, ToolHead, ToolHeadApi};
use klippy_api::{GCell, ObjectRef, PrinterObject};
use pycompat::PyValue;

/// Template callback `callback(text)` (`PrinterOutputPin._template_update`).
pub type TemplateCallback = Arc<dyn Fn(&str) + Send + Sync>;
/// Optional flush callback `flush_callback()`.
pub type FlushCallback = Arc<dyn Fn() + Send + Sync>;

/// The `template_evaluator` printer object interface
/// (`PrinterTemplateEvaluator.set_template`).
pub trait TemplateEvaluatorApi: Send + Sync + 'static {
    fn set_template(&self, gcmd: &GCmd, callback: TemplateCallback, flush_callback: Option<FlushCallback>) -> KResult<()>;
}

/// Handle to a [`TemplateEvaluatorApi`].
pub type TemplateEvaluator = Arc<dyn TemplateEvaluatorApi>;

/// Time between each template update (`RENDER_TIME = 0.500`).
pub const RENDER_TIME: f64 = 0.500;

// ---------------------------------------------------------------------------
// G-Code request queuing helper (`output_pin.py:15`).
// ---------------------------------------------------------------------------

/// Return of a [`GcrqCallback`]: `(action, next_min_time)` where `action`
/// is `"discard"`, `"reschedule"` or `"repeat"`; `None` is a normal request.
pub type GcrqCallbackReturn = Option<(String, f64)>;
/// Request-queue callback `callback(print_time, value)` (`_set_pin`); its
/// exception leaves `_flush_notification` / `send_async_request`.
pub type GcrqCallback = Arc<dyn Fn(f64, f64) -> KResult<GcrqCallbackReturn> + Send + Sync>;

struct GcrqState {
    rqueue: Vec<(f64, f64)>,
    next_min_flush_time: f64,
    toolhead: Option<ToolHead>,
}

struct GcrqInner {
    state: GCell<GcrqState>,
}

/// Helper code to queue g-code requests (`GCodeRequestQueue`).
pub struct GCodeRequestQueue {
    printer: Printer,
    mcu: Mcu,
    callback: GcrqCallback,
    motion_queuing: MotionQueuing,
    inner: Arc<GcrqInner>,
}

impl GCodeRequestQueue {
    /// `GCodeRequestQueue(config, mcu, callback)` (`output_pin.py:16`).
    pub fn new(config: &Config, mcu: Mcu, callback: GcrqCallback) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let motion_queuing = klippy_api::load_interface::<dyn MotionQueuingApi>(
            &*printer, config, "motion_queuing", LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let inner = Arc::new(GcrqInner {
            state: GCell::new(GcrqState { rqueue: Vec::new(), next_min_flush_time: 0., toolhead: None }),
        });
        let queue = Arc::new(GCodeRequestQueue {
            printer: printer.clone(),
            mcu: mcu.clone(),
            callback: callback.clone(),
            motion_queuing: motion_queuing.clone(),
            inner: inner.clone(),
        });
        let weak = Arc::downgrade(&inner);
        let mq = motion_queuing.clone();
        let cb = callback.clone();
        let mcu_flush = mcu.clone();
        // `register_flush_callback(self._flush_notification)`
        // (`output_pin.py:24`): the reported name is the bound method's
        // `__qualname__` (`motion_queuing.py:35-42`).
        motion_queuing.register_flush_callback_named(
            Arc::new(move |must_flush_time, max_step_gen_time| {
                if let Some(inner) = weak.upgrade() {
                    Self::flush_notification_inner(&inner, &mq, &cb, &mcu_flush, must_flush_time, max_step_gen_time)?;
                }
                Ok(())
            }),
            false,
            Some("_flush_notification"),
            None,
        );
        let weak_connect = Arc::downgrade(&inner);
        let printer_connect = printer.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params| {
                if let Some(inner) = weak_connect.upgrade() {
                    let toolhead = klippy_api::lookup_interface::<dyn ToolHeadApi>(
                        &*printer_connect, "toolhead", LookupDefault::Raise,
                    )?
                    .expect("LookupDefault::Raise never yields None");
                    inner.state.borrow_mut().toolhead = Some(toolhead);
                }
                Ok(PyValue::None)
            }),
        );
        Ok(queue)
    }

    /// `_flush_notification(must_flush_time, max_step_gen_time)`.
    fn flush_notification_inner(
        inner: &GcrqInner,
        motion_queuing: &MotionQueuing,
        callback: &GcrqCallback,
        mcu: &Mcu,
        must_flush_time: f64,
        _max_step_gen_time: f64,
    ) -> KResult<()> {
        let min_sched_time = mcu.min_schedule_time();
        loop {
            let (next_time, pos, req_val) = {
                let st = inner.state.borrow();
                if st.rqueue.is_empty() {
                    return Ok(());
                }
                let next_time = st.rqueue[0].0.max(st.next_min_flush_time);
                if next_time > must_flush_time {
                    return Ok(());
                }
                let mut pos = 0;
                while pos + 1 < st.rqueue.len() && st.rqueue[pos + 1].0 <= next_time {
                    pos += 1;
                }
                (next_time, pos, st.rqueue[pos].1)
            };
            let ret = callback(next_time, req_val)?;
            let mut st = inner.state.borrow_mut();
            let mut action: Option<String> = None;
            if let Some((act, next_min_time)) = ret {
                st.next_min_flush_time = st.next_min_flush_time.max(next_min_time);
                action = Some(act);
                match action.as_deref() {
                    Some("discard") => {
                        st.rqueue.drain(..pos + 1);
                        continue;
                    }
                    Some("reschedule") => {
                        st.rqueue.drain(..pos);
                        continue;
                    }
                    Some("repeat") => {}
                    _ => {}
                }
            }
            let del_end = if action.as_deref() == Some("repeat") { pos } else { pos + 1 };
            st.rqueue.drain(..del_end);
            st.next_min_flush_time = st.next_min_flush_time.max(next_time + min_sched_time);
            let mq_time = st.next_min_flush_time;
            drop(st);
            motion_queuing.note_mcu_movequeue_activity(mq_time, false);
        }
    }

    /// `_queue_request(print_time, value)`.
    fn queue_request_inner(inner: &GcrqInner, motion_queuing: &MotionQueuing, print_time: f64, value: f64) {
        inner.state.borrow_mut().rqueue.push((print_time, value));
        motion_queuing.note_mcu_movequeue_activity(print_time, false);
    }

    /// `queue_gcode_request(value)`.
    pub fn queue_gcode_request(&self, value: f64) -> KResult<()> {
        let toolhead = match self.inner.state.borrow().toolhead.clone() {
            Some(t) => t,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'register_lookahead_callback'".to_string(),
                });
            }
        };
        let inner = self.inner.clone();
        let motion_queuing = self.motion_queuing.clone();
        toolhead.register_lookahead_callback(Box::new(move |print_time| {
            Self::queue_request_inner(&inner, &motion_queuing, print_time, value);
            Ok(())
        }))
    }

    /// `send_async_request(value, print_time=None)`.
    pub fn send_async_request(&self, value: f64, print_time: Option<f64>) -> KResult<()> {
        let min_sched_time = self.mcu.min_schedule_time();
        let mut print_time = print_time;
        if print_time.is_none() {
            let systime = self.printer.get_reactor().monotonic();
            print_time = Some(self.mcu.estimated_print_time(systime + min_sched_time));
        }
        let print_time = print_time.unwrap();
        loop {
            let next_time = {
                let st = self.inner.state.borrow();
                print_time.max(st.next_min_flush_time)
            };
            let ret = (self.callback)(next_time, value)?;
            let mut action: String = "normal".to_string();
            if let Some((act, nmt)) = ret {
                action = act;
                let cur = self.inner.state.borrow().next_min_flush_time;
                self.inner.state.borrow_mut().next_min_flush_time = cur.max(nmt);
                if action == "discard" {
                    break;
                }
                if action == "reschedule" {
                    continue;
                }
            }
            let nf = self.inner.state.borrow().next_min_flush_time.max(next_time + min_sched_time);
            self.inner.state.borrow_mut().next_min_flush_time = nf;
            if action != "repeat" {
                break;
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Template evaluation helper (`output_pin.py:101`).
// ---------------------------------------------------------------------------

/// Cache key: `(template, sorted(lparams))` (`_activate_template`).
struct UidKey {
    template: Arc<dyn DisplayTemplateApi>,
    lparams: Vec<(String, PyValue)>,
}

fn uid_key_eq(a: &UidKey, b: &UidKey) -> bool {
    if !Arc::ptr_eq(&a.template, &b.template) {
        return false;
    }
    if a.lparams.len() != b.lparams.len() {
        return false;
    }
    a.lparams
        .iter()
        .zip(b.lparams.iter())
        .all(|((ka, va), (kb, vb))| ka == kb && pycompat::value::py_equal(va, vb))
}

struct ActiveTemplate {
    callback: TemplateCallback,
    uid: Option<UidKey>,
    template: Arc<dyn DisplayTemplateApi>,
    lparams: IndexMap<String, PyValue>,
    flush_callback: Option<FlushCallback>,
}

struct EvaluatorInner {
    active_templates: GCell<Vec<ActiveTemplate>>,
    render_timer: GCell<Option<reactor::Timer>>,
}

/// Main template evaluation code (`PrinterTemplateEvaluator`).
pub struct PrinterTemplateEvaluator {
    printer: Printer,
    templates: IndexMap<String, Arc<dyn DisplayTemplateApi>>,
    gcode_macro: Arc<dyn PrinterGCodeMacroApi>,
    inner: Arc<EvaluatorInner>,
}

impl PrinterTemplateEvaluator {
    /// `PrinterTemplateEvaluator(config)` (`output_pin.py:102`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let templates = lookup_display_templates(config)?.get_display_templates();
        let gcode_macro = klippy_api::load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer, config, "gcode_macro", LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        Ok(Arc::new(PrinterTemplateEvaluator {
            printer,
            templates,
            gcode_macro,
            inner: Arc::new(EvaluatorInner {
                active_templates: GCell::new(Vec::new()),
                render_timer: GCell::new(None),
            }),
        }))
    }

    /// `_activate_timer()`.
    fn activate_timer(&self) {
        {
            let slot = self.inner.render_timer.borrow();
            if slot.is_some() {
                return;
            }
        }
        if self.inner.active_templates.borrow().is_empty() {
            return;
        }
        let reactor = self.printer.get_reactor();
        let weak = Arc::downgrade(&self.inner);
        let templates = Arc::new(self.templates.clone());
        let gcode_macro = self.gcode_macro.clone();
        let rc = reactor.clone();
        let timer = reactor.register_timer(
            move |eventtime| {
                if let Some(inner) = weak.upgrade() {
                    Self::render_inner(&inner, &templates, &*gcode_macro, rc.clone(), eventtime)
                } else {
                    reactor::NEVER
                }
            },
            reactor::NOW,
        );
        *self.inner.render_timer.borrow_mut() = Some(timer);
    }

    /// `_activate_template(callback, template, lparams, flush_callback)`.
    fn activate_template(
        &self,
        callback: TemplateCallback,
        template: Option<Arc<dyn DisplayTemplateApi>>,
        lparams: IndexMap<String, PyValue>,
        flush_callback: Option<FlushCallback>,
    ) {
        let mut active = self.inner.active_templates.borrow_mut();
        match template {
            Some(template) => {
                let mut items: Vec<(String, PyValue)> = lparams.clone().into_iter().collect();
                items.sort_by(|a, b| a.0.cmp(&b.0));
                let hashable = items.iter().all(|(_, v)| v.is_hashable());
                if hashable {
                    let uid = UidKey { template: template.clone(), lparams: items };
                    let entry = ActiveEntry::Cached { callback, uid, template, lparams, flush_callback };
                    Self::upsert_template(&mut active, entry);
                } else {
                    let entry = ActiveEntry::Uncached { callback, template, lparams, flush_callback };
                    Self::upsert_template(&mut active, entry);
                }
            }
            None => {
                if let Some(pos) = active.iter().position(|e| Arc::ptr_eq(&e.callback, &callback)) {
                    active.remove(pos);
                }
            }
        }
    }

    #[allow(clippy::type_complexity)]
    fn render_inner(
        inner: &EvaluatorInner,
        templates: &IndexMap<String, Arc<dyn DisplayTemplateApi>>,
        gcode_macro: &dyn PrinterGCodeMacroApi,
        reactor: reactor::Reactor,
        eventtime: f64,
    ) -> f64 {
        if inner.active_templates.borrow().is_empty() {
            let timer = inner.render_timer.borrow_mut().take();
            if let Some(timer) = timer {
                reactor.unregister_timer(&timer);
            }
            return reactor::NEVER;
        }
        let context: IndexMap<String, Value> = gcode_macro.create_template_context(Some(eventtime));
        let ctx_cell: Rc<RefCell<Option<IndexMap<String, Value>>>> = Rc::new(RefCell::new(Some(context)));
        let render_fn = template_render_func(templates.clone(), ctx_cell.clone());
        if let Some(map) = ctx_cell.borrow_mut().as_mut() {
            map.insert("render".to_string(), render_fn);
        }
        // Snapshot active templates (callback, uid, template, lparams, flush).
        // Python iterates self.active_templates.items() in insertion order.
        let entries: Vec<(TemplateCallback, Option<UidSnapshot>, Arc<dyn DisplayTemplateApi>, IndexMap<String, PyValue>, Option<FlushCallback>)> = inner
            .active_templates
            .borrow()
            .iter()
            .map(|e| {
                let uid = e.uid.as_ref().map(|u| UidSnapshot { template: u.template.clone(), lparams: u.lparams.clone() });
                (e.callback.clone(), uid, e.template.clone(), e.lparams.clone(), e.flush_callback.clone())
            })
            .collect();
        let mut flush_callbacks: Vec<FlushCallback> = Vec::new();
        let mut render_cache: Vec<(UidSnapshot, String)> = Vec::new();
        let base_context = ctx_cell.borrow().clone().unwrap_or_default();
        for (callback, uid, template, lparams, flush_callback) in &entries {
            let text = match uid {
                Some(u) => match render_cache.iter().find(|(cu, _)| uid_key_eq(&cur_to_key(u), &cur_to_key(cu))) {
                    Some((_, cached)) => cached.clone(),
                    None => {
                        let rendered = match template.render(&base_context, lparams) {
                            Ok(s) => s,
                            Err(e) => {
                                klippy_api::log::exception("display template render error", &e.exception_line());
                                String::new()
                            }
                        };
                        render_cache.push((u.clone(), rendered.clone()));
                        rendered
                    }
                },
                None => match template.render(&base_context, lparams) {
                    Ok(s) => s,
                    Err(e) => {
                        klippy_api::log::exception("display template render error", &e.exception_line());
                        String::new()
                    }
                },
            };
            if let Some(flush) = flush_callback {
                if !flush_callbacks.iter().any(|f| Arc::ptr_eq(f, flush)) {
                    flush_callbacks.push(flush.clone());
                }
            }
            callback(&text);
        }
        for flush_callback in &flush_callbacks {
            flush_callback();
        }
        // Drop the cycle between the context and `render`.
        ctx_cell.borrow_mut().take();
        eventtime + RENDER_TIME
    }

    /// `set_template(gcmd, callback, flush_callback=None)` (`output_pin.py:164`).
    pub fn set_template(
        &self,
        gcmd: &GCmd,
        callback: TemplateCallback,
        flush_callback: Option<FlushCallback>,
    ) -> KResult<()> {
        let mut template: Option<Arc<dyn DisplayTemplateApi>> = None;
        let mut lparams: IndexMap<String, PyValue> = IndexMap::new();
        let tpl_name = gcmd.get("TEMPLATE", GcmdGetOpts { default: Some(PyValue::None), ..Default::default() })?;
        let tpl_name = match tpl_name {
            PyValue::None => None,
            PyValue::Str(s) => if s.is_empty() { None } else { Some(s) },
            other => { let s = other.str(); if s.is_empty() { None } else { Some(s) } }
        };
        if let Some(tpl_name) = tpl_name {
            let found = self.templates.get(&tpl_name).cloned();
            match found {
                None => {
                    return Err(KlippyError::command(format!("Unknown display_template '{}'", tpl_name)));
                }
                Some(t) => {
                    let tparams = t.get_params();
                    for (p, v) in gcmd.get_command_parameters() {
                        if !p.starts_with("PARAM_") {
                            continue;
                        }
                        let p = p.to_lowercase();
                        if !tparams.contains_key(&p) {
                            return Err(KlippyError::command(format!(
                                "Invalid display_template parameter: {}",
                                p
                            )));
                        }
                        match parse_literal(&v) {
                            Ok(parsed) => {
                                lparams.insert(p, parsed);
                            }
                            Err(e) => {
                                if e.kind == "ValueError" {
                                    return Err(KlippyError::command(format!(
                                        "Unable to parse '{}' as a literal",
                                        v
                                    )));
                                }
                                return Err(KlippyError::Internal {
                                    class_name: e.kind.to_string(),
                                    message: e.msg.clone(),
                                });
                            }
                        }
                    }
                    template = Some(t);
                }
            }
        }
        self.activate_template(callback, template, lparams, flush_callback);
        self.activate_timer();
        Ok(())
    }
}



enum ActiveEntry {
    Cached { callback: TemplateCallback, uid: UidKey, template: Arc<dyn DisplayTemplateApi>, lparams: IndexMap<String, PyValue>, flush_callback: Option<FlushCallback> },
    Uncached { callback: TemplateCallback, template: Arc<dyn DisplayTemplateApi>, lparams: IndexMap<String, PyValue>, flush_callback: Option<FlushCallback> },
}


impl PrinterTemplateEvaluator {
    fn upsert_template(active: &mut Vec<ActiveTemplate>, entry: ActiveEntry) {
        // Python `self.active_templates[callback] = (...)` replaces the value
        // in place, keeping the key's original insertion order.
        let replacement = match entry {
            ActiveEntry::Cached { callback, uid, template, lparams, flush_callback } => {
                ActiveTemplate { callback, uid: Some(uid), template, lparams, flush_callback }
            }
            ActiveEntry::Uncached { callback, template, lparams, flush_callback } => {
                ActiveTemplate { callback, uid: None, template, lparams, flush_callback }
            }
        };
        if let Some(pos) = active.iter().position(|e| Arc::ptr_eq(&e.callback, &replacement.callback)) {
            active[pos] = replacement;
        } else {
            active.push(replacement);
        }
    }
}

#[derive(Clone)]
struct UidSnapshot {
    template: Arc<dyn DisplayTemplateApi>,
    lparams: Vec<(String, PyValue)>,
}

fn cur_to_key(u: &UidSnapshot) -> UidKey {
    UidKey { template: u.template.clone(), lparams: u.lparams.clone() }
}

/// `lookup_template_eval(config)` (`output_pin.py:187`).
pub fn lookup_template_eval(config: &Config) -> KResult<Arc<PrinterTemplateEvaluator>> {
    let printer = config.get_printer();
    if let Some(obj) = printer.lookup_object("template_evaluator", LookupDefault::ReturnNone)? {
        if let Some(existing) = klippy_api::downcast_arc::<PrinterTemplateEvaluator>(&obj) {
            return Ok(existing);
        }
        return Err(klippy_api::missing_interface::<dyn TemplateEvaluatorApi>("template_evaluator"));
    }
    let te = PrinterTemplateEvaluator::new(config)?;
    printer.add_object("template_evaluator", te.clone() as ObjectRef)?;
    Ok(te)
}

impl PrinterObject for PrinterTemplateEvaluator {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemplateEvaluatorApi);
}

impl TemplateEvaluatorApi for PrinterTemplateEvaluator {
    fn set_template(&self, gcmd: &GCmd, callback: TemplateCallback, flush_callback: Option<FlushCallback>) -> KResult<()> {
        PrinterTemplateEvaluator::set_template(self, gcmd, callback, flush_callback)
    }
}

// ---------------------------------------------------------------------------
// Main output pin handling (`output_pin.py:200`).
// ---------------------------------------------------------------------------

enum PinHandle {
    Pwm(PwmPin),
    Digital(DigitalOutPin),
}

struct PinShared {
    is_pwm: bool,
    scale: f64,
    last_value: GCell<f64>,
    pin: PinHandle,
}

impl PinShared {
    /// `_set_pin(print_time, value)` (`output_pin.py:237`).
    fn set_pin(&self, print_time: f64, value: f64) -> KResult<GcrqCallbackReturn> {
        if value == self.last_value.get() {
            return Ok(Some(("discard".to_string(), 0.)));
        }
        self.last_value.set(value);
        match &self.pin {
            PinHandle::Pwm(pin) => pin.set_pwm(print_time, value)?,
            PinHandle::Digital(pin) => pin.set_digital(print_time, value != 0.0),
        }
        Ok(None)
    }
}

/// `[output_pin <name>]` object (`PrinterOutputPin`).
pub struct PrinterOutputPin {
    shared: Arc<PinShared>,
    queue: Arc<GCodeRequestQueue>,
    evaluator: Arc<PrinterTemplateEvaluator>,
    template_callback: TemplateCallback,
}

impl PrinterOutputPin {
    /// `PrinterOutputPin(config)` (`output_pin.py:201`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let ppins = klippy_api::lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let is_pwm = config
            .getboolean("pwm", GetBoolOpts { default: Some(Some(false)), ..Default::default() })?
            .unwrap_or(false);
        let pin: PinHandle;
        let scale: f64;
        let mcu: Mcu;
        if is_pwm {
            let p = match ppins.setup_pin("pwm", &required_string(config, "pin")?)? {
                Pin::Pwm(p) => p,
                _ => {
                    return Err(KlippyError::Other {
                        class_name: "error".to_string(),
                        message: "setup_pin('pwm') returned unexpected pin type".to_string(),
                    });
                }
            };
            let max_duration = p.get_mcu().max_nominal_duration();
            let cycle_time = config
                .getfloat("cycle_time", GetFloatOpts { default: Some(Some(0.100)), above: Some(0.), maxval: Some(max_duration), ..Default::default() })?
                .unwrap_or(0.100);
            let hardware_pwm = config
                .getboolean("hardware_pwm", GetBoolOpts { default: Some(Some(false)), ..Default::default() })?
                .unwrap_or(false);
            p.setup_cycle_time(cycle_time, Some(hardware_pwm))?;
            scale = config
                .getfloat("scale", GetFloatOpts { default: Some(Some(1.)), above: Some(0.), ..Default::default() })?
                .unwrap_or(1.);
            mcu = p.get_mcu();
            pin = PinHandle::Pwm(p);
        } else {
            let p = match ppins.setup_pin("digital_out", &required_string(config, "pin")?)? {
                Pin::DigitalOut(p) => p,
                _ => {
                    return Err(KlippyError::Other {
                        class_name: "error".to_string(),
                        message: "setup_pin('digital_out') returned unexpected pin type".to_string(),
                    });
                }
            };
            scale = 1.;
            mcu = p.get_mcu();
            pin = PinHandle::Digital(p);
        }
        match &pin {
            PinHandle::Pwm(p) => p.setup_max_duration(0.),
            PinHandle::Digital(p) => p.setup_max_duration(0.),
        }
        let last_value = config
            .getfloat("value", GetFloatOpts { default: Some(Some(0.)), minval: Some(0.), maxval: Some(scale), ..Default::default() })?
            .unwrap_or(0.)
            / scale;
        let shutdown_value = config
            .getfloat("shutdown_value", GetFloatOpts { default: Some(Some(0.)), minval: Some(0.), maxval: Some(scale), ..Default::default() })?
            .unwrap_or(0.)
            / scale;
        match &pin {
            PinHandle::Pwm(p) => p.setup_start_value(last_value, shutdown_value),
            PinHandle::Digital(p) => p.setup_start_value(last_value != 0.0, shutdown_value != 0.0),
        }
        let shared = Arc::new(PinShared { is_pwm, scale, last_value: GCell::new(last_value), pin });
        let shared_cb = shared.clone();
        let callback: GcrqCallback = Arc::new(move |print_time, value| shared_cb.set_pin(print_time, value));
        let queue = GCodeRequestQueue::new(config, mcu, callback)?;
        let evaluator = lookup_template_eval(config)?;
        let pin_name = {
            let name = config.get_name();
            let mut parts = name.split_whitespace();
            let _first = parts.next();
            match parts.next() {
                Some(n) => n.to_string(),
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list index out of range".to_string(),
                    });
                }
            }
        };
        let queue_for_template = queue.clone();
        let template_callback: TemplateCallback = Arc::new(move |text: &str| {
            let value = match pycompat::text::float_from_string(text) {
                Ok(v) => v,
                Err(_) => {
                    klippy_api::log::exception("output_pin template render error", "ValueError");
                    0.
                }
            };
            let _ = queue_for_template.send_async_request(value, None);
        });
        let out = Arc::new(PrinterOutputPin { shared, queue, evaluator, template_callback });
        let gcode = klippy_api::lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let weak = Arc::downgrade(&out);
        gcode.register_mux_command(
            "SET_PIN",
            "PIN",
            Some(&pin_name),
            Box::new(move |gcmd| {
                if let Some(me) = weak.upgrade() {
                    me.cmd_set_pin(gcmd)
                } else {
                    Ok(())
                }
            }),
            Some("Set the value of an output pin"),
        )?;
        Ok(out)
    }

    /// `_template_update(text)` (`output_pin.py:245`).
    pub fn template_update(&self, text: &str) {
        (self.template_callback)(text);
    }

    /// `cmd_SET_PIN(gcmd)` (`output_pin.py:253`).
    pub fn cmd_set_pin(&self, gcmd: &GCmd) -> KResult<()> {
        let value = gcmd.get_float("VALUE", GcmdGetFloatOpts { default: Some(None), minval: Some(0.), maxval: Some(self.shared.scale), ..Default::default() })?;
        let template = gcmd.get("TEMPLATE", GcmdGetOpts { default: Some(PyValue::None), ..Default::default() })?;
        let template = match template {
            PyValue::None => None,
            PyValue::Str(s) => Some(s),
            other => Some(other.str()),
        };
        if value.is_none() == template.is_none() {
            return Err(KlippyError::command("SET_PIN command must specify VALUE or TEMPLATE"));
        }
        if template.is_some() {
            self.evaluator.set_template(gcmd, self.template_callback.clone(), None)?;
            return Ok(());
        }
        let mut value = value.unwrap();
        value /= self.shared.scale;
        if !self.shared.is_pwm && value != 0.0 && value != 1.0 {
            return Err(KlippyError::command("Invalid pin value"));
        }
        self.queue.queue_gcode_request(value)?;
        Ok(())
    }

    /// `get_status(eventtime)` (`output_pin.py:235`).
    pub fn get_status_value(&self) -> PyValue {
        let mut map: IndexMap<PyValue, PyValue> = IndexMap::new();
        map.insert(PyValue::Str("value".to_string()), PyValue::Float(self.shared.last_value.get()));
        PyValue::Dict(map)
    }
}

impl PrinterObject for PrinterOutputPin {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.get_status_value()))
    }
}

fn required_string(config: &Config, option: &str) -> KResult<String> {
    config.get(option, GetStrOpts::default())?.ok_or_else(|| {
        KlippyError::config(format!("Could not find option '{}' in section '{}'", option, config.get_name()))
    })
}

fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterOutputPin::new(config)? as ObjectRef))
}

/// Module entry point for `[output_pin <name>]` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
