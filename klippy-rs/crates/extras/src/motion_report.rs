//! Port of the `motion_report` module (`klippy/extras/motion_report.py`).
//!
//! Diagnostic dumps for stepper `queue_step` history and trapezoidal
//! motion queues, plus live requested-position status.
//!
//! Notes:
//! - The concrete trapq behind `Arc<dyn TrapqApi>` is core's
//!   `TrapqHandle`, recovered with `klippy_core::motion_handles::trapq_of`
//!   and read through its shared chelper trapq.
//! - `bulk_sensor.BatchBulkHelper` is the ported crate helper: the
//!   periodic batch timer and webhooks streaming come from it, the
//!   same as the Python `BatchBulkHelper`.
//! - `mcu_stepper.get_name()` in Python (no argument) is
//!   `get_name(short=False)`; the Rust call passes `Some(false)`.
//! - `mcu` shutdown lookup by short name reproduces the Python
//!   `printer.lookup_object(details.get("mcu"), None)`, including the
//!   alternate-MCU miss (short name does not match `"mcu <name>"`).

use chelper_motion::trapq::PullMove;
use klippy_core::motion_handles::trapq_of;
use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::GCodeApi;
use klippy_api::log;
use klippy_api::mcu::McuApi;
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::stepper::{PullHistoryStep, Stepper, StepperRegistryApi, Trapq};
use klippy_api::toolhead::{ExtraAxisApi, MotionQueuingApi, ToolHeadApi};
use crate::bulk_sensor::{BatchBulkHelper, BATCH_INTERVAL};
use klippy_api::{interface, lookup_interface, GCell};
use pycompat::text::{percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, OnceLock};

pub const NEVER_TIME: f64 = 9999999999999999.0;
pub const STATUS_REFRESH_TIME: f64 = 0.250;


fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant motion_report format")
}

fn int(n: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(n))
}

/// Python tuple/list display for `logging.info("... %s", pos)` where
/// `pos` is a tuple of floats: `str(pos)` == `repr(pos)`.
fn py_tuple_float_str(vals: &[f64]) -> String {
    PyValue::Tuple(vals.iter().map(|v| PyValue::Float(*v)).collect()).str()
}

fn coord_tuple(pos: &[f64]) -> PyValue {
    // `gcode.Coord` pads missing entries with the int `0`, not `0.0`.
    let mut v: Vec<PyValue> = pos.iter().map(|x| PyValue::Float(*x)).collect();
    if v.len() < 4 {
        while v.len() < 4 {
            v.push(PyValue::Int(PyInt::from_i64(0)));
        }
    }
    PyValue::Tuple(v)
}

// ---------------------------------------------------------------------------
// DumpStepper (`klippy/extras/motion_report.py:11-68`)
// ---------------------------------------------------------------------------

pub struct DumpStepper {
    #[allow(dead_code)]
    printer: Printer,
    mcu_stepper: Stepper,
    last_batch_clock: GCell<i64>,
    batch_bulk: OnceLock<BatchBulkHelper>,
}

impl DumpStepper {
    /// `DumpStepper.__init__(printer, mcu_stepper)`.
    pub fn new(printer: Printer, mcu_stepper: Stepper) -> Arc<Self> {
        Self::try_new(printer, mcu_stepper).expect("webhooks lookup in DumpStepper")
    }

    /// Fallible constructor; propagates `webhooks` lookup errors the way
    /// `BatchBulkHelper.add_mux_endpoint` does (`lookup_object` raises).
    pub fn try_new(printer: Printer, mcu_stepper: Stepper) -> KResult<Arc<Self>> {
        let this = Arc::new(Self {
            printer: printer.clone(),
            mcu_stepper: mcu_stepper.clone(),
            last_batch_clock: GCell::new(0),
            batch_bulk: OnceLock::new(),
        });
        let process = Arc::downgrade(&this);
        let batch_bulk = BatchBulkHelper::new(
            printer.clone(),
            Box::new(move |eventtime| match process.upgrade() {
                Some(dev) => Ok(dev.process_batch(eventtime)),
                None => Ok(PyValue::None),
            }),
            None,
            None,
            BATCH_INTERVAL,
        );
        let api_resp: PyValue = {
            let mut d = indexmap::IndexMap::new();
            d.insert(
                PyValue::Str("header".to_string()),
                PyValue::Tuple(vec![
                    PyValue::Str("interval".to_string()),
                    PyValue::Str("count".to_string()),
                    PyValue::Str("add".to_string()),
                ]),
            );
            PyValue::Dict(d)
        };
        batch_bulk.add_mux_endpoint(
            "motion_report/dump_stepper",
            "name",
            &mcu_stepper.get_name(Some(false)),
            api_resp,
        )?;
        let _ = this.batch_bulk.set(batch_bulk);
        Ok(this)
    }

    /// `self.batch_bulk` (`DumpStepper.__init__`).
    pub fn batch_bulk(&self) -> &BatchBulkHelper {
        self.batch_bulk.get().expect("batch_bulk built in try_new")
    }


    pub fn mcu_stepper(&self) -> &Stepper {
        &self.mcu_stepper
    }

    /// `DumpStepper.get_step_queue(start_clock, end_clock)`.
    pub fn get_step_queue(
        &self,
        start_clock: i64,
        end_clock_: i64,
    ) -> (Vec<PullHistoryStep>, Vec<(Vec<PullHistoryStep>, i64)>) {
        let mut res: Vec<(Vec<PullHistoryStep>, i64)> = Vec::new();
        let mut end_clock = end_clock_;
        loop {
            let (data, count) = self.mcu_stepper.dump_steps(128, start_clock, end_clock);
            if count == 0 {
                break;
            }
            res.push((data.clone(), count));
            if count < data.len() as i64 {
                break;
            }
            // `end_clock = data[count-1].first_clock`
            let idx = (count - 1) as usize;
            end_clock = data[idx].first_clock;
        }
        res.reverse();
        let flat: Vec<PullHistoryStep> = res
            .iter()
            .flat_map(|(d, cnt)| {
                let n = *cnt as usize;
                (0..n).rev().map(move |i| d[i])
            })
            .collect();
        (flat, res)
    }

    /// `DumpStepper.log_steps(data)`.
    pub fn log_steps(&self, data: &[PullHistoryStep]) {
        if data.is_empty() {
            return;
        }
        let mut out: Vec<String> = Vec::with_capacity(data.len() + 1);
        out.push(pct(
            "Dumping stepper '%s' (%s) %d queue_step:",
            &[
                PctValue::Str(self.mcu_stepper.get_name(Some(false))),
                PctValue::Str(self.mcu_stepper.get_mcu().get_name()),
                int(data.len() as i64),
            ],
        ));
        for (i, s) in data.iter().enumerate() {
            out.push(pct(
                "queue_step %d: t=%d p=%d i=%d c=%d a=%d",
                &[
                    int(i as i64),
                    int(s.first_clock),
                    int(s.start_position),
                    int(s.interval),
                    int(s.step_count),
                    int(s.add),
                ],
            ));
        }
        log::info(&out.join("\n"));
    }

    /// `DumpStepper._process_batch(eventtime)` — returns the dict payload,
    /// or an empty dict when there is no data.
    pub fn process_batch(&self, _eventtime: f64) -> PyValue {
        let last = self.last_batch_clock.get();
        let (mut data, _cdata) = self.get_step_queue(last, i64::MAX);
        if data.is_empty() {
            return PyValue::Dict(IndexMap::new());
        }
        // `del data[i+1:]` keeps the zero-step-count marker itself.
        for (i, d) in data.iter().enumerate() {
            if d.step_count == 0 {
                data.truncate(i + 1);
                break;
            }
        }
        let mcu = self.mcu_stepper.get_mcu();
        let first = data[0];
        let first_clock = first.first_clock;
        let first_time = mcu.clock_to_print_time(first_clock as f64);
        let last_clock = data[data.len() - 1].last_clock;
        let last_time = mcu.clock_to_print_time(last_clock as f64);
        self.last_batch_clock.set(last_clock);
        let mcu_pos = first.start_position;
        let start_position = self.mcu_stepper.mcu_to_commanded_position(mcu_pos);
        let step_dist = self.mcu_stepper.get_step_dist();
        let tdata: Vec<PyValue> = data
            .iter()
            .map(|s| {
                PyValue::Tuple(vec![
                    PyValue::Int(PyInt::from_i64(s.interval)),
                    PyValue::Int(PyInt::from_i64(s.step_count)),
                    PyValue::Int(PyInt::from_i64(s.add)),
                ])
            })
            .collect();
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("data".to_string()), PyValue::List(tdata));
        d.insert(
            PyValue::Str("start_position".to_string()),
            PyValue::Float(start_position),
        );
        d.insert(
            PyValue::Str("start_mcu_position".to_string()),
            PyValue::Int(PyInt::from_i64(mcu_pos)),
        );
        d.insert(
            PyValue::Str("step_distance".to_string()),
            PyValue::Float(step_dist),
        );
        d.insert(
            PyValue::Str("first_clock".to_string()),
            PyValue::Int(PyInt::from_i64(first_clock)),
        );
        d.insert(
            PyValue::Str("first_step_time".to_string()),
            PyValue::Float(first_time),
        );
        d.insert(
            PyValue::Str("last_clock".to_string()),
            PyValue::Int(PyInt::from_i64(last_clock)),
        );
        d.insert(
            PyValue::Str("last_step_time".to_string()),
            PyValue::Float(last_time),
        );
        PyValue::Dict(d)
    }

    pub fn last_batch_clock(&self) -> i64 {
        self.last_batch_clock.get()
    }
}

// ---------------------------------------------------------------------------
// DumpTrapQ (`klippy/extras/motion_report.py:73-141`)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TrapqBatchMsg {
    pub print_time: f64,
    pub move_t: f64,
    pub start_v: f64,
    pub accel: f64,
    pub start_x: f64,
    pub start_y: f64,
    pub start_z: f64,
    pub x_r: f64,
    pub y_r: f64,
    pub z_r: f64,
}

pub struct DumpTrapQ {
    #[allow(dead_code)]
    printer: Printer,
    name: String,
    trapq: Option<Trapq>,
    motion_queuing: Arc<dyn MotionQueuingApi>,
    last_batch_msg: GCell<TrapqBatchMsg>,
    batch_bulk: OnceLock<BatchBulkHelper>,
}

impl DumpTrapQ {
    /// `DumpTrapQ.__init__(printer, name, trapq)`.
    pub fn new(printer: Printer, name: String, trapq: Option<Trapq>) -> Arc<Self> {
        Self::try_new(printer, name, trapq).expect("webhooks/motion_queuing lookup in DumpTrapQ")
    }

    pub fn try_new(printer: Printer, name: String, trapq: Option<Trapq>) -> KResult<Arc<Self>> {
        // `printer.lookup_object("motion_queuing")` raises when missing in
        // Python; capture the object the same way.
        let motion_queuing = lookup_interface::<dyn MotionQueuingApi>(
            &*printer,
            "motion_queuing",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let this = Arc::new(Self {
            printer: printer.clone(),
            name: name.clone(),
            trapq,
            motion_queuing,
            last_batch_msg: GCell::new(TrapqBatchMsg::default()),
            batch_bulk: OnceLock::new(),
        });
        let process = Arc::downgrade(&this);
        let batch_bulk = BatchBulkHelper::new(
            printer.clone(),
            Box::new(move |eventtime| match process.upgrade() {
                Some(dev) => Ok(dev.process_batch(eventtime)),
                None => Ok(PyValue::None),
            }),
            None,
            None,
            BATCH_INTERVAL,
        );
        let api_resp: PyValue = {
            let mut d = indexmap::IndexMap::new();
            d.insert(
                PyValue::Str("header".to_string()),
                PyValue::Tuple(vec![
                    PyValue::Str("time".to_string()),
                    PyValue::Str("duration".to_string()),
                    PyValue::Str("start_velocity".to_string()),
                    PyValue::Str("acceleration".to_string()),
                    PyValue::Str("start_position".to_string()),
                    PyValue::Str("direction".to_string()),
                ]),
            );
            PyValue::Dict(d)
        };
        batch_bulk.add_mux_endpoint("motion_report/dump_trapq", "name", &name, api_resp)?;
        let _ = this.batch_bulk.set(batch_bulk);
        Ok(this)
    }

    /// `self.batch_bulk` (`DumpTrapQ.__init__`).
    pub fn batch_bulk(&self) -> &BatchBulkHelper {
        self.batch_bulk.get().expect("batch_bulk built in try_new")
    }


    pub fn name(&self) -> &str {
        &self.name
    }

    /// `DumpTrapQ.extract_trapq(start_time, end_time)`.
    pub fn extract_trapq(
        &self,
        start_time: f64,
        end_time_: f64,
    ) -> (Vec<PullMove>, Vec<(Vec<PullMove>, i32)>) {
        let Some(trapq) = self.trapq.as_ref() else {
            return (Vec::new(), Vec::new());
        };
        let Ok(handle) = trapq_of(trapq) else {
            return (Vec::new(), Vec::new());
        };
        let ct = handle.shared().lock();
        let mut res: Vec<(Vec<PullMove>, i32)> = Vec::new();
        let mut end_time = end_time_;
        loop {
            let mut buf = vec![PullMove::default(); 128];
            let count = ct.extract_old(&mut buf, 128, start_time, end_time);
            if count == 0 {
                break;
            }
            let n = count as usize;
            res.push((buf[..n].to_vec(), count));
            if count < 128 {
                break;
            }
            end_time = buf[n - 1].print_time;
        }
        res.reverse();
        let flat: Vec<PullMove> = res
            .iter()
            .flat_map(|(d, cnt)| {
                let n = *cnt as usize;
                (0..n).rev().map(move |i| d[i])
            })
            .collect();
        (flat, res)
    }

    /// `DumpTrapQ.log_trapq(data)`.
    pub fn log_trapq(&self, data: &[PullMove]) {
        if data.is_empty() {
            return;
        }
        let mut out: Vec<String> = Vec::with_capacity(data.len() + 1);
        out.push(pct(
            "Dumping trapq '%s' %d moves:",
            &[PctValue::Str(self.name.clone()), int(data.len() as i64)],
        ));
        for (i, m) in data.iter().enumerate() {
            out.push(pct(
                "move %d: pt=%.6f mt=%.6f sv=%.6f a=%.6f sp=(%.6f,%.6f,%.6f) ar=(%.6f,%.6f,%.6f)",
                &[
                    int(i as i64),
                    PctValue::Float(m.print_time),
                    PctValue::Float(m.move_t),
                    PctValue::Float(m.start_v),
                    PctValue::Float(m.accel),
                    PctValue::Float(m.start_x),
                    PctValue::Float(m.start_y),
                    PctValue::Float(m.start_z),
                    PctValue::Float(m.x_r),
                    PctValue::Float(m.y_r),
                    PctValue::Float(m.z_r),
                ],
            ));
        }
        log::info(&out.join("\n"));
    }

    /// `DumpTrapQ.get_trapq_position(print_time)`.
    pub fn get_trapq_position(
        &self,
        print_time: f64,
    ) -> (Option<[f64; 3]>, Option<f64>) {
        // Python calls `trapq_extract_old(trapq, data, 1, 0., print_time)`,
        // which returns the newest move overlapping `[0, print_time]`
        // (`extract_old` iterates newest first). Do not reuse
        // `extract_trapq`, whose flattened output is oldest first.
        let Some(trapq) = self.trapq.as_ref() else {
            return (None, None);
        };
        let Ok(handle) = trapq_of(trapq) else {
            return (None, None);
        };
        let ct = handle.shared().lock();
        let mut buf = [PullMove::default(); 1];
        let count = ct.extract_old(&mut buf, 1, 0.0, print_time);
        if count == 0 {
            return (None, None);
        }
        let mv = buf[0];
        let move_time = 0.0f64.max((print_time - mv.print_time).min(mv.move_t));
        // `dist = (start_v + .5*accel*move_time) * move_time`
        let dist = (mv.start_v + 0.5 * mv.accel * move_time) * move_time;
        let pos = [
            mv.start_x + mv.x_r * dist,
            mv.start_y + mv.y_r * dist,
            mv.start_z + mv.z_r * dist,
        ];
        let velocity = mv.start_v + mv.accel * move_time;
        (Some(pos), Some(velocity))
    }

    /// `DumpTrapQ._process_batch(eventtime)`.
    pub fn process_batch(&self, _eventtime: f64) -> PyValue {
        let last = self.last_batch_msg.get();
        let qtime = last.print_time + last.move_t.min(0.100);
        let (data, _c) = self.extract_trapq(qtime, NEVER_TIME);
        let mut d: Vec<TrapqBatchMsg> = data
            .iter()
            .map(|m| TrapqBatchMsg {
                print_time: m.print_time,
                move_t: m.move_t,
                start_v: m.start_v,
                accel: m.accel,
                start_x: m.start_x,
                start_y: m.start_y,
                start_z: m.start_z,
                x_r: m.x_r,
                y_r: m.y_r,
                z_r: m.z_r,
            })
            .collect();
        if !d.is_empty() {
            // `self.motion_queuing` was captured in `__init__` (stored ref).
            let start_drip = self.motion_queuing.check_drip_timing();
            if let Some(start_drip_time) = start_drip {
                while !d.is_empty()
                    && d[d.len() - 1].print_time + d[d.len() - 1].move_t >= start_drip_time
                {
                    d.pop();
                }
            }
        }
        if !d.is_empty() && d[0] == self.last_batch_msg.get() {
            d.remove(0);
        }
        if d.is_empty() {
            return PyValue::Dict(IndexMap::new());
        }
        self.last_batch_msg.set(d[d.len() - 1]);
        let list: Vec<PyValue> = d
            .iter()
            .map(|m| {
                PyValue::Tuple(vec![
                    PyValue::Float(m.print_time),
                    PyValue::Float(m.move_t),
                    PyValue::Float(m.start_v),
                    PyValue::Float(m.accel),
                    PyValue::Tuple(vec![
                        PyValue::Float(m.start_x),
                        PyValue::Float(m.start_y),
                        PyValue::Float(m.start_z),
                    ]),
                    PyValue::Tuple(vec![
                        PyValue::Float(m.x_r),
                        PyValue::Float(m.y_r),
                        PyValue::Float(m.z_r),
                    ]),
                ])
            })
            .collect();
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("data".to_string()), PyValue::List(list));
        PyValue::Dict(map)
    }

    pub fn last_batch_msg(&self) -> TrapqBatchMsg {
        self.last_batch_msg.get()
    }
}

// ---------------------------------------------------------------------------
// PrinterMotionReport (`klippy/extras/motion_report.py:145-267`)
// ---------------------------------------------------------------------------

pub struct PrinterMotionReport {
    printer: Printer,
    steppers: GCell<IndexMap<String, Arc<DumpStepper>>>,
    dtrapqs: GCell<IndexMap<String, Arc<DumpTrapQ>>>,
    next_status_time: GCell<f64>,
    last_status: GCell<PyValue>,
}

impl PrinterMotionReport {
    /// `PrinterMotionReport.__init__(config)`.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer: Printer = config.get_printer();
        // `gcode = printer.lookup_object('gcode')` (raises when missing).
        let _ = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let mut last_status = IndexMap::new();
        last_status.insert(
            PyValue::Str("live_position".to_string()),
            coord_tuple(&[0.0, 0.0, 0.0]),
        );
        last_status.insert(PyValue::Str("live_velocity".to_string()), PyValue::Float(0.0));
        last_status.insert(
            PyValue::Str("live_extruder_velocity".to_string()),
            PyValue::Float(0.0),
        );
        last_status.insert(PyValue::Str("steppers".to_string()), PyValue::List(Vec::new()));
        last_status.insert(PyValue::Str("trapq".to_string()), PyValue::List(Vec::new()));
        let this = Arc::new(Self {
            printer: printer.clone(),
            steppers: GCell::new(IndexMap::new()),
            dtrapqs: GCell::new(IndexMap::new()),
            next_status_time: GCell::new(0.0),
            last_status: GCell::new(PyValue::Dict(last_status)),
        });
        let this_connect = Arc::clone(&this);
        printer.register_event_handler("klippy:connect", Box::new(move |args| {
            this_connect.handle_connect(args)?;
            Ok(PyValue::None)
        }));
        let this_shutdown = Arc::clone(&this);
        printer.register_event_handler("klippy:analyze_shutdown", Box::new(move |args| {
            this_shutdown.handle_analyze_shutdown(args)?;
            Ok(PyValue::None)
        }));
        Ok(this)
    }

    /// `PrinterMotionReport.register_stepper(config, mcu_stepper)`.
    pub fn register_stepper(&self, _config: &Config, mcu_stepper: Stepper) -> KResult<()> {
        let ds = DumpStepper::try_new(self.printer.clone(), mcu_stepper.clone())?;
        let name = mcu_stepper.get_name(Some(false));
        self.steppers.borrow_mut().insert(name, ds);
        Ok(())
    }

    /// `PrinterMotionReport._connect()`.
    pub fn handle_connect(&self, _args: &[EventArg]) -> KResult<()> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        {
            let trapq = toolhead.get_trapq();
            let dt = DumpTrapQ::try_new(
                self.printer.clone(),
                "toolhead".to_string(),
                Some(trapq),
            )?;
            self.dtrapqs
                .borrow_mut()
                .insert("toolhead".to_string(), dt);
        }
        for i in 0..99i64 {
            let mut ename = format!("extruder{i}");
            if ename == "extruder0" {
                ename = "extruder".to_string();
            }
            let Some(obj) = self
                .printer
                .lookup_object(&ename, LookupDefault::ReturnNone)?
            else {
                break;
            };
            let Some(ea) = interface::<dyn ExtraAxisApi>(&obj) else {
                return Err(missing_interface("PrinterExtruder", &ename));
            };
            let etrapq = ea.get_trapq();
            let dt = DumpTrapQ::try_new(
                self.printer.clone(),
                ename.clone(),
                etrapq,
            )?;
            self.dtrapqs.borrow_mut().insert(ename, dt);
        }
        let manual = self.printer.lookup_objects(Some("manual_stepper"));
        for (msname, ms) in manual {
            let Some(ea) = interface::<dyn ExtraAxisApi>(&ms) else {
                return Err(missing_interface("ManualStepper", &msname));
            };
            let mstrapq = ea.get_trapq();
            let dt = DumpTrapQ::try_new(
                self.printer.clone(),
                msname.clone(),
                mstrapq,
            )?;
            self.dtrapqs.borrow_mut().insert(msname, dt);
        }
        let mut steppers: Vec<String> = self.steppers.borrow().keys().cloned().collect();
        steppers.sort();
        let mut trapqs: Vec<String> = self.dtrapqs.borrow().keys().cloned().collect();
        trapqs.sort();
        self.last_status.with_mut(|s| {
            if let PyValue::Dict(d) = s {
                d.insert(
                    PyValue::Str("steppers".to_string()),
                    PyValue::List(
                        steppers.into_iter().map(PyValue::Str).collect(),
                    ),
                );
                d.insert(
                    PyValue::Str("trapq".to_string()),
                    PyValue::List(trapqs.into_iter().map(PyValue::Str).collect()),
                );
            }
        });
        Ok(())
    }

    /// `PrinterMotionReport._dump_shutdown(dsteppers, shutdown_time, duration)`.
    pub fn dump_shutdown(
        &self,
        dsteppers: &[Arc<DumpStepper>],
        shutdown_time: f64,
        duration: f64,
    ) {
        let start_time = shutdown_time - duration;
        let end_time = shutdown_time + duration;
        for ds in dsteppers {
            let mcu = ds.mcu_stepper().get_mcu();
            let start_clock = mcu.print_time_to_clock(start_time).max(0);
            let end_clock = mcu.print_time_to_clock(end_time);
            let (data, _c) = ds.get_step_queue(start_clock, end_clock);
            ds.log_steps(&data);
        }
        let dtrapqs: Vec<Arc<DumpTrapQ>> =
            (*self.dtrapqs.borrow()).values().cloned().collect();
        for dt in &dtrapqs {
            let (data, _c) = dt.extract_trapq(start_time, end_time);
            dt.log_trapq(&data);
        }
        let toolhead_dt = dtrapqs
            .into_iter()
            .find(|d| d.name() == "toolhead");
        let Some(dt) = toolhead_dt else {
            return;
        };
        let (pos, _vel) = dt.get_trapq_position(shutdown_time);
        if let Some(p) = pos {
            let tup = py_tuple_float_str(&p);
            log::info(&pct(
                "Requested toolhead position at shutdown time %.6f: %s",
                &[PctValue::Float(shutdown_time), PctValue::Str(tup)],
            ));
        }
    }

    /// `PrinterMotionReport._handle_analyze_shutdown(msg, details)`.
    pub fn handle_analyze_shutdown(&self, args: &[EventArg]) -> KResult<()> {
        let (msg, details) = match (args.first(), args.get(1)) {
            (Some(EventArg::Value(PyValue::Str(s))), Some(EventArg::Value(d))) => {
                (s.clone(), d.clone())
            }
            (Some(EventArg::Value(PyValue::Str(s))), Some(EventArg::Object(_))
            | Some(EventArg::Any(_))
            | Some(EventArg::List(_))) => (s.clone(), PyValue::None),
            _ => return Ok(()),
        };
        if msg == "MCU shutdown" {
            let (mcu_name, shutdown_clock) = match &details {
                PyValue::Dict(d) => (
                    d.get(&PyValue::Str("mcu".to_string()))
                        .and_then(|v| match v {
                            PyValue::Str(s) => Some(s.clone()),
                            _ => None,
                        }),
                    d.get(&PyValue::Str("shutdown_clock".to_string()))
                        .and_then(|v| match v {
                            PyValue::Int(n) => n
                                .0
                                .to_string()
                                .parse::<i64>()
                                .ok(),
                            PyValue::Float(f) => Some(*f as i64),
                            _ => None,
                        }),
                ),
                _ => (None, None),
            };
            let (Some(mcu_name), Some(shutdown_clock)) = (mcu_name, shutdown_clock) else {
                return Ok(());
            };
            let Some(mcu_obj) = self
                .printer
                .lookup_object(&mcu_name, LookupDefault::ReturnNone)?
            else {
                return Ok(());
            };
            let Some(mcu) = interface::<dyn McuApi>(&mcu_obj) else {
                return Ok(());
            };
            let shutdown_time = mcu.clock_to_print_time(shutdown_clock as f64);
            let dsteppers: Vec<Arc<DumpStepper>> = self
                .steppers
                .borrow()
                .values()
                .filter(|ds| Arc::ptr_eq(&ds.mcu_stepper().get_mcu(), &mcu))
                .cloned()
                .collect();
            self.dump_shutdown(&dsteppers, shutdown_time, 0.100);
            return Ok(());
        }
        if msg == "Internal error in stepcompress" {
            log::info(&pct(
                "stepcompress error info: %s",
                &[PctValue::Str(details.str())],
            ));
            let (step_gen_time, queue_name) = match &details {
                PyValue::Dict(d) => (
                    d.get(&PyValue::Str("step_gen_time".to_string()))
                        .and_then(|v| match v {
                            PyValue::Float(f) => Some(*f),
                            PyValue::Int(n) => n.0.to_string().parse::<f64>().ok(),
                            _ => None,
                        }),
                    d.get(&PyValue::Str("queue_name".to_string()))
                        .and_then(|v| match v {
                            PyValue::Str(s) => Some(s.clone()),
                            _ => None,
                        }),
                ),
                _ => (None, None),
            };
            let Some(step_gen_time) = step_gen_time else {
                return Ok(());
            };
            let dsteppers: Vec<Arc<DumpStepper>> = match queue_name {
                Some(ref q) => self.steppers.borrow().get(q).cloned().into_iter().collect(),
                None => Vec::new(),
            };
            self.dump_shutdown(&dsteppers, step_gen_time, 1.0);
        }
        Ok(())
    }

    /// `PrinterMotionReport.get_status(eventtime)`.
    pub fn get_status_value(&self, eventtime: f64) -> KResult<PyValue> {
        if eventtime < self.next_status_time.get() || self.dtrapqs.borrow().is_empty() {
            return Ok(self.last_status.cloned());
        }
        self.next_status_time.set(eventtime + STATUS_REFRESH_TIME);
        let mut xyzvelocity = 0.0f64;
        let mut evelocity = 0.0f64;
        // Python raises when these lookups miss (`lookup_object` without a
        // default). Propagate instead of falling back to the stale status.
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let extra_axes = toolhead.get_extra_axes();
        let mut live_pos = vec![0.0f64; extra_axes.len()];
        let mcu = lookup_interface::<dyn McuApi>(
            &*self.printer,
            "mcu",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let print_time = mcu.estimated_print_time(eventtime);
        // `self.dtrapqs['toolhead']` (KeyError when absent).
        let dt = self
            .dtrapqs
            .borrow()
            .get("toolhead")
            .cloned()
            .ok_or_else(|| KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: pycompat::value::repr_key("toolhead"),
            })?;
        let (pos, vel) = dt.get_trapq_position(print_time);
        if let (Some(p), Some(v)) = (pos, vel) {
            for (i, val) in p.iter().enumerate().take(3) {
                if i < live_pos.len() {
                    live_pos[i] = *val;
                }
            }
            xyzvelocity = v;
        }
        let dtrapqs_snapshot: IndexMap<String, Arc<DumpTrapQ>> =
            (*self.dtrapqs.borrow()).clone();
        for (ea_index, ea) in extra_axes.iter().enumerate() {
            let Some(ea) = ea else { continue };
            let eaname = ea.get_name();
            let Some(eh) = dtrapqs_snapshot.get(&eaname) else {
                continue;
            };
            let (pos, vel) = eh.get_trapq_position(print_time);
            if let (Some(p), Some(v)) = (pos, vel) {
                if ea_index < live_pos.len() {
                    live_pos[ea_index] = p[0];
                }
                if ea_index == 3 {
                    evelocity = v;
                }
            }
        }
        self.last_status.with_mut(|s| {
            if let PyValue::Dict(d) = s {
                d.insert(
                    PyValue::Str("live_position".to_string()),
                    coord_tuple(&live_pos),
                );
                d.insert(
                    PyValue::Str("live_velocity".to_string()),
                    PyValue::Float(xyzvelocity),
                );
                d.insert(
                    PyValue::Str("live_extruder_velocity".to_string()),
                    PyValue::Float(evelocity),
                );
            }
        });
        Ok(self.last_status.cloned())
    }

    pub fn steppers_snapshot(&self) -> Vec<(String, Arc<DumpStepper>)> {
        self.steppers
            .borrow()
            .iter()
            .map(|(k, v)| (k.clone(), Arc::clone(v)))
            .collect()
    }

    pub fn dtrapqs_snapshot(&self) -> Vec<(String, Arc<DumpTrapQ>)> {
        self.dtrapqs
            .borrow()
            .iter()
            .map(|(k, v)| (k.clone(), Arc::clone(v)))
            .collect()
    }
}

impl klippy_api::PrinterObject for PrinterMotionReport {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn StepperRegistryApi);
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        self.get_status_value(eventtime).ok()
    }
}

impl StepperRegistryApi for PrinterMotionReport {
    fn register_stepper(&self, config: &Config, mcu_stepper: Stepper) -> KResult<()> {
        PrinterMotionReport::register_stepper(self, config, mcu_stepper)
    }
}

/// `load_config(config)` (`klippy/extras/motion_report.py:266`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterMotionReport::new(config)? as ObjectRef))
}

/// Module entry point for the `motion_report` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}

// `toolhead.rs` / `kinematics/extruder.py` style errors need a
// `KlippyError` named for the missing interface. Reuse the canonical
// helper through `object::missing_interface` shape.
fn missing_interface(class: &str, name: &str) -> KlippyError {
    KlippyError::Other {
        class_name: "AttributeError".to_string(),
        message: format!("'{class}' object has no attribute 'get_trapq' ({name})"),
    }
}

