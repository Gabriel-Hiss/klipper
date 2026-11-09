//! Port of `klippy/extras/delta_calibrate.py`.
//!
//! Delta calibration: probing points plus optional extended distance
//! measurements are reduced to stable positions, a coordinate descent
//! over the delta parameters fits them, and the result is saved through
//! `configfile` for `SAVE_CONFIG`.
//!
//! Services are reached with `lookup_interface` (contract v2). The probe
//! helper comes from `crate::probe::ProbePointsHelper`, the descent from
//! `klippy_core::mathutil`, and the delta math is only used through the
//! frozen `DeltaCalibrationApi` trait (implemented by the kinematics
//! delta port); this module keeps no copy of any of them.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatListOpts, GetFloatOpts, PrinterConfig, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GCmd, GCode, GCodeApi, GetFloatOpts as GCmdFloatOpts, GetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{lookup_interface, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::toolhead::{DeltaCalibration, ToolHead, ToolHeadApi};
use klippy_core::mathutil::{background_coordinate_descent, DescentParams};
use pycompat::libm;
use pycompat::text::{float_from_string, float_true_div, percent_format, PctValue, PyError};
use pycompat::PyValue;
use std::sync::Arc;

use crate::manual_probe::ProbeResult;
use crate::probe::{ProbeFinalizeCallback, ProbePointsHelper};

// ---------------------------------------------------------------------------
// Error helpers (Python exception classes stay part of the contract).
// ---------------------------------------------------------------------------

fn py_err(e: PyError) -> KlippyError {
    KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
}

fn value_error(msg: &str) -> KlippyError {
    KlippyError::Other { class_name: "ValueError".to_string(), message: msg.to_string() }
}

fn key_error(key: &str) -> KlippyError {
    KlippyError::Other {
        class_name: "KeyError".to_string(),
        message: pycompat::value::repr_key(key),
    }
}

fn index_error() -> KlippyError {
    KlippyError::Other {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

/// Python `math.sqrt` (`klippy/mathutil.py`, `klippy/kinematics/delta.py`):
/// negative inputs raise `ValueError: math domain error`.
fn math_sqrt(x: f64) -> KResult<f64> {
    if x < 0.0 {
        Err(value_error("math domain error"))
    } else {
        Ok(libm::sqrt(x))
    }
}

/// Python `math.radians`: `x * pi / 180` (one multiply by the correctly
/// rounded `pi/180` constant, as in CPython `mathmodule.c`).
fn math_radians(x: f64) -> f64 {
    x * (std::f64::consts::PI / 180.0)
}

fn pct(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(py_err)
}

fn float_pct(v: f64) -> PctValue {
    PctValue::Float(v)
}

fn pct_str(s: &str) -> PctValue {
    PctValue::Str(s.to_string())
}

fn lookup_gcode(printer: &Printer) -> KResult<GCode> {
    Ok(lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
        .expect("Raise never yields None"))
}

fn lookup_toolhead(printer: &Printer) -> KResult<ToolHead> {
    Ok(lookup_interface::<dyn ToolHeadApi>(&**printer, "toolhead", LookupDefault::Raise)?
        .expect("Raise never yields None"))
}

fn lookup_configfile(printer: &Printer) -> KResult<PrinterConfig> {
    Ok(lookup_interface::<dyn PrinterConfigApi>(
        &**printer,
        "configfile",
        LookupDefault::Raise,
    )?
    .expect("Raise never yields None"))
}

// ---------------------------------------------------------------------------
// `measurements_to_distances` + constants (`delta_calibrate.py:26`).
// ---------------------------------------------------------------------------

/// The angles and distances of the calibration object.
pub const MEASURE_ANGLES: [f64; 6] = [210., 270., 330., 30., 90., 150.];
/// Outer radius of the calibration object.
pub const MEASURE_OUTER_RADIUS: f64 = 65.;
/// Ridge radius of the calibration object (`5. - .5`).
pub const MEASURE_RIDGE_RADIUS: f64 = 5.0 - 0.5;
/// How much to prefer a distance measurement over a height measurement.
pub const MEASURE_WEIGHT: f64 = 0.5;

type V3 = [f64; 3];

/// A three-number Python sequence in a stable position. The reference
/// mixes the two container kinds: `config.getfloatlist` and
/// `DeltaCalibration.calc_stable_position` return lists, while
/// `add_manual_height` wraps its result in `tuple()`. The difference is
/// visible in the `"Calculating delta_calibrate with:"` log line, so the
/// container kind is part of the port's state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StablePosition {
    List(V3),
    Tuple(V3),
}

impl StablePosition {
    pub fn values(&self) -> &V3 {
        match self {
            StablePosition::List(v) | StablePosition::Tuple(v) => v,
        }
    }

    fn pyvalue(self) -> PyValue {
        let items = self.values().iter().map(|v| PyValue::Float(*v)).collect::<Vec<_>>();
        match self {
            StablePosition::List(_) => PyValue::List(items),
            StablePosition::Tuple(_) => PyValue::Tuple(items),
        }
    }
}

pub type HeightEntry = (f64, StablePosition);
pub type DistanceEntry = (f64, StablePosition, StablePosition);
pub type MeasuredParams = IndexMap<String, Vec<f64>>;

fn measured_index(m: &MeasuredParams, name: &str, i: usize) -> KResult<f64> {
    let Some(list) = m.get(name) else { return Err(key_error(name)) };
    list.get(i).copied().ok_or_else(index_error)
}

/// `measurements_to_distances` (`delta_calibrate.py:35`). The stable
/// positions come from the frozen `DeltaCalibrationApi` (`delta.py:212`
/// `calc_stable_position`); the Python zips truncate to the shortest
/// operand, so indexing only reaches the first six positions each side.
pub fn measurements_to_distances(
    measured_params: &MeasuredParams,
    cal: &DeltaCalibration,
) -> KResult<Vec<DistanceEntry>> {
    let scale = measured_index(measured_params, "SCALE", 0)?;
    let cpw = measured_params
        .get("CENTER_PILLAR_WIDTHS")
        .ok_or_else(|| key_error("CENTER_PILLAR_WIDTHS"))?;
    // Python materialises the six-entry list before reading CENTER_DISTS,
    // so a short CENTER_PILLAR_WIDTHS raises IndexError first.
    let cpw_at = |i: usize| cpw.get(i).copied().ok_or_else(index_error);
    let center_widths = [
        cpw_at(0)?,
        cpw_at(2)?,
        cpw_at(1)?,
        cpw_at(0)?,
        cpw_at(2)?,
        cpw_at(1)?,
    ];
    let dists = measured_params
        .get("CENTER_DISTS")
        .ok_or_else(|| key_error("CENTER_DISTS"))?;
    let mut center_dists = Vec::new();
    for (od, cw) in dists.iter().zip(center_widths.iter()) {
        center_dists.push(od - cw);
    }
    let dists = measured_params
        .get("OUTER_DISTS")
        .ok_or_else(|| key_error("OUTER_DISTS"))?;
    let widths = measured_params
        .get("OUTER_PILLAR_WIDTHS")
        .ok_or_else(|| key_error("OUTER_PILLAR_WIDTHS"))?;
    let mut outer_dists = Vec::new();
    for (od, opw) in dists.iter().zip(widths.iter()) {
        outer_dists.push(od - opw);
    }
    let obj_angles: Vec<f64> = MEASURE_ANGLES.iter().map(|a| math_radians(*a)).collect();
    let xy_angles: Vec<(f64, f64)> = obj_angles
        .iter()
        .map(|a| (libm::cos(*a), libm::sin(*a)))
        .collect();
    let inner_ridge = MEASURE_RIDGE_RADIUS * scale;
    let inner_pos: Vec<V3> = xy_angles
        .iter()
        .map(|(ax, ay)| [*ax * inner_ridge, *ay * inner_ridge, 0.0])
        .collect();
    let outer_ridge = (MEASURE_OUTER_RADIUS + MEASURE_RIDGE_RADIUS) * scale;
    let outer_pos: Vec<V3> = xy_angles
        .iter()
        .map(|(ax, ay)| [*ax * outer_ridge, *ay * outer_ridge, 0.0])
        .collect();
    let to_stable = |pos: V3| -> KResult<StablePosition> {
        let stable = cal.calc_stable_position(&pos)?;
        Ok(StablePosition::List([stable[0], stable[1], stable[2]]))
    };
    let mut center_positions = Vec::new();
    for (k, cd) in center_dists.iter().enumerate() {
        let ip = inner_pos[k];
        let op = outer_pos[k];
        center_positions.push((*cd, to_stable(ip)?, to_stable(op)?));
    }
    let outer_center = MEASURE_OUTER_RADIUS * scale;
    let start_pos: Vec<(f64, f64)> = xy_angles
        .iter()
        .map(|(ax, ay)| (*ax * outer_center, *ay * outer_center))
        .collect();
    let mut shifted: Vec<(f64, f64)> = xy_angles[2..].to_vec();
    shifted.extend_from_slice(&xy_angles[..2]);
    let first_pos: Vec<V3> = shifted
        .iter()
        .zip(start_pos.iter())
        .map(|((ax, ay), (spx, spy))| [*ax * inner_ridge + *spx, *ay * inner_ridge + *spy, 0.0])
        .collect();
    let second_pos: Vec<V3> = shifted
        .iter()
        .zip(start_pos.iter())
        .map(|((ax, ay), (spx, spy))| [*ax * outer_ridge + *spx, *ay * outer_ridge + *spy, 0.0])
        .collect();
    let mut outer_positions = Vec::new();
    for (k, od) in outer_dists.iter().enumerate() {
        let fp = first_pos[k];
        let sp = second_pos[k];
        outer_positions.push((*od, to_stable(fp)?, to_stable(sp)?));
    }
    center_positions.append(&mut outer_positions);
    Ok(center_positions)
}

/// Parse the frozen `DeltaCalibrationApi::coordinate_descent_params`
/// `PyValue` (`(adj_params_tuple, params_dict)`) into Rust form.
fn parse_coordinate_descent_params(
    v: &PyValue,
) -> KResult<(Vec<String>, DescentParams)> {
    let PyValue::Tuple(parts) = v else {
        return Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: "cannot unpack non-iterable".to_string(),
        });
    };
    if parts.len() != 2 {
        return Err(KlippyError::Other {
            class_name: "ValueError".to_string(),
            message: if parts.len() < 2 {
                format!("not enough values to unpack (expected 2, got {})", parts.len())
            } else {
                "too many values to unpack (expected 2)".to_string()
            },
        });
    }
    let PyValue::Tuple(adj) = &parts[0] else {
        return Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: "adj_params must be a tuple".to_string(),
        });
    };
    let PyValue::Dict(params) = &parts[1] else {
        return Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: "params must be a dict".to_string(),
        });
    };
    let mut adj_params = Vec::new();
    for item in adj {
        match item {
            PyValue::Str(s) => adj_params.push(s.clone()),
            other => {
                return Err(KlippyError::Other {
                    class_name: "TypeError".to_string(),
                    message: format!(
                        "adj_params must be str, got {}",
                        pycompat::value::type_name(other)
                    ),
                })
            }
        }
    }
    let mut map = IndexMap::new();
    for (k, val) in params {
        let PyValue::Str(name) = k else {
            return Err(KlippyError::Other {
                class_name: "TypeError".to_string(),
                message: "params keys must be str".to_string(),
            });
        };
        let f = val.to_f64().ok_or_else(|| KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: "params values must be floats".to_string(),
        })?;
        map.insert(name.clone(), f);
    }
    Ok((adj_params, map))
}

// ---------------------------------------------------------------------------
// `DeltaCalibrate` (`delta_calibrate.py:78`).
// ---------------------------------------------------------------------------

/// `load_config_stable` (`delta_calibrate.py:16`): the three floats
/// `config.getfloatlist` returns. `configfile.getlists` ends with
/// `return tuple(res)` (`configfile.py:119`), so this is a Python tuple,
/// unlike the lists `calc_stable_position` produces.
pub fn load_config_stable(config: &Config, option: &str) -> KResult<StablePosition> {
    let list = config.getfloatlist(option, GetFloatListOpts {
        default: None,
        sep: ",".to_string(),
        count: Some(3),
        note_valid: true,
        ..Default::default()
    })?;
    let list = list.expect("required");
    Ok(StablePosition::Tuple([list[0], list[1], list[2]]))
}

/// Port of the `DeltaCalibrate` class (`delta_calibrate.py:78`).
pub struct DeltaCalibrate {
    printer: Printer,
    gcode: GCell<Option<GCode>>,
    manual_heights: GCell<Vec<HeightEntry>>,
    last_probe_positions: GCell<Vec<HeightEntry>>,
    last_distances: GCell<Vec<DistanceEntry>>,
    delta_analyze_entry: GCell<MeasuredParams>,
    probe_helper: GCell<Option<Arc<ProbePointsHelper>>>,
}

klippy_api::printer_object!(DeltaCalibrate);

impl DeltaCalibrate {
    /// `DeltaCalibrate.__init__` (`delta_calibrate.py:79`).
    pub fn new(config: &Config) -> KResult<Arc<DeltaCalibrate>> {
        let printer = config.get_printer();
        let this = Arc::new(DeltaCalibrate {
            printer: printer.clone(),
            gcode: GCell::new(None),
            manual_heights: GCell::new(Vec::new()),
            last_probe_positions: GCell::new(Vec::new()),
            last_distances: GCell::new(Vec::new()),
            delta_analyze_entry: GCell::new(IndexMap::new()),
            probe_helper: GCell::new(None),
        });
        // Register the connect handler first (Python order).
        {
            let this_cb = Arc::clone(&this);
            printer.register_event_handler(
                EV_KLIPPY_CONNECT,
                Box::new(move |_args| {
                    this_cb.handle_connect()?;
                    Ok(PyValue::None)
                }),
            );
        }
        // Default probing points.
        let radius = config
            .getfloat(
                "radius",
                GetFloatOpts {
                    default: None,
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .expect("required");
        // Default probing points.
        let mut points = vec![vec![0.0, 0.0]];
        let scatter = [0.95, 0.90, 0.85, 0.70, 0.75, 0.80];
        for (i, s) in scatter.iter().enumerate() {
            let r = math_radians(90.0 + 60.0 * (i as f64));
            let dist = radius * s;
            points.push(vec![libm::cos(r) * dist, libm::sin(r) * dist]);
        }
        // `probe_finalize` returns None in Python: any other value would
        // mean "retry" to the helper; None means done.
        let this_fin = Arc::clone(&this);
        let finalize: ProbeFinalizeCallback = Box::new(move |results: &[ProbeResult]| {
            this_fin.probe_finalize(results)?;
            Ok(PyValue::None)
        });
        let probe_helper = ProbePointsHelper::new(config, finalize, Some(points))?;
        probe_helper.minimum_points(3)?;
        *this.probe_helper.borrow_mut() = Some(probe_helper);
        // Restore state.
        {
            let mut positions = Vec::new();
            for i in 0..999 {
                let Some(height) = config.getfloat(
                    &format!("height{i}"),
                    GetFloatOpts {
                        default: Some(None),
                        minval: None,
                        maxval: None,
                        above: None,
                        below: None,
                        note_valid: true,
                        ..Default::default()
                    },
                )? else {
                    break;
                };
                let height_pos = load_config_stable(config, &format!("height{i}_pos"))?;
                positions.push((height, height_pos));
            }
            *this.last_probe_positions.borrow_mut() = positions;
            let mut heights = Vec::new();
            for i in 0..999 {
                let Some(height) = config.getfloat(
                    &format!("manual_height{i}"),
                    GetFloatOpts {
                        default: Some(None),
                        minval: None,
                        maxval: None,
                        above: None,
                        below: None,
                        note_valid: true,
                        ..Default::default()
                    },
                )? else {
                    break;
                };
                let height_pos = load_config_stable(config, &format!("manual_height{i}_pos"))?;
                heights.push((height, height_pos));
            }
            *this.manual_heights.borrow_mut() = heights;
            let mut distances = Vec::new();
            for i in 0..999 {
                let Some(dist) = config.getfloat(
                    &format!("distance{i}"),
                    GetFloatOpts {
                        default: Some(None),
                        minval: None,
                        maxval: None,
                        above: None,
                        below: None,
                        note_valid: true,
                        ..Default::default()
                    },
                )? else {
                    break;
                };
                let pos1 = load_config_stable(config, &format!("distance{i}_pos1"))?;
                let pos2 = load_config_stable(config, &format!("distance{i}_pos2"))?;
                distances.push((dist, pos1, pos2));
            }
            *this.last_distances.borrow_mut() = distances;
            let mut entry = IndexMap::new();
            entry.insert("SCALE".to_string(), vec![1.0]);
            *this.delta_analyze_entry.borrow_mut() = entry;
        }
        // G-code commands.
        let gcode = lookup_gcode(&printer)?;
        *this.gcode.borrow_mut() = Some(gcode.clone());
        {
            let this_cal = Arc::clone(&this);
            gcode.register_command(
                "DELTA_CALIBRATE",
                Some(Box::new(move |gcmd: &GCmd| this_cal.cmd_delta_calibrate(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some("Delta calibration script".to_string()),
                },
            )?;
        }
        {
            let this_an = Arc::clone(&this);
            gcode.register_command(
                "DELTA_ANALYZE",
                Some(Box::new(move |gcmd: &GCmd| this_an.cmd_delta_analyze(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some("Extended delta calibration tool".to_string()),
                },
            )?;
        }
        Ok(this)
    }

    fn gcode(&self) -> GCode {
        self.gcode
            .borrow()
            .clone()
            .expect("gcode set during init")
    }

    fn toolhead(&self) -> KResult<ToolHead> {
        lookup_toolhead(&self.printer)
    }

    fn configfile(&self) -> KResult<PrinterConfig> {
        lookup_configfile(&self.printer)
    }

    /// `handle_connect` (`delta_calibrate.py:127`).
    pub fn handle_connect(&self) -> KResult<()> {
        let toolhead = self.toolhead()?;
        let kin = toolhead.get_kinematics();
        match kin.get_calibration() {
            Ok(_) => Ok(()),
            Err(e) if e.class_name() == "AttributeError" => Err(KlippyError::Config(
                "Delta calibrate is only for delta printers".to_string(),
            )),
            Err(e) => Err(e),
        }
    }

    /// `save_state` (`delta_calibrate.py:132`).
    pub fn save_state(
        &self,
        probe_positions: &[HeightEntry],
        distances: &[DistanceEntry],
        delta_params: &DeltaCalibration,
    ) -> KResult<()> {
        let configfile = self.configfile()?;
        delta_params.save_state(&configfile)?;
        let section = "delta_calibrate";
        configfile.remove_section(section);
        for (i, (z_offset, spos)) in probe_positions.iter().enumerate() {
            let spos = spos.values();
            configfile.set(section, &format!("height{i}"), &pycompat::text::float_str(*z_offset))?;
            configfile.set(
                section,
                &format!("height{i}_pos"),
                &pct("%.3f,%.3f,%.3f", &[float_pct(spos[0]), float_pct(spos[1]), float_pct(spos[2])])?,
            )?;
        }
        let manual_heights = self.manual_heights.borrow().clone();
        for (i, (z_offset, spos)) in manual_heights.iter().enumerate() {
            let spos = spos.values();
            configfile.set(section, &format!("manual_height{i}"), &pycompat::text::float_str(*z_offset))?;
            configfile.set(
                section,
                &format!("manual_height{i}_pos"),
                &pct("%.3f,%.3f,%.3f", &[float_pct(spos[0]), float_pct(spos[1]), float_pct(spos[2])])?,
            )?;
        }
        for (i, (dist, spos1, spos2)) in distances.iter().enumerate() {
            let spos1 = spos1.values();
            let spos2 = spos2.values();
            configfile.set(section, &format!("distance{i}"), &pycompat::text::float_str(*dist))?;
            configfile.set(
                section,
                &format!("distance{i}_pos1"),
                &pct("%.3f,%.3f,%.3f", &[float_pct(spos1[0]), float_pct(spos1[1]), float_pct(spos1[2])])?,
            )?;
            configfile.set(
                section,
                &format!("distance{i}_pos2"),
                &pct("%.3f,%.3f,%.3f", &[float_pct(spos2[0]), float_pct(spos2[1]), float_pct(spos2[2])])?,
            )?;
        }
        Ok(())
    }

    /// `probe_finalize` (`delta_calibrate.py:155`): convert positions into
    /// `(z_offset, stable_position)` pairs, then run the analysis. Exposed
    /// because `ProbePointsHelper` invokes it as its finalize callback.
    pub fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<()> {
        // Convert positions into (z_offset, stable_position) pairs
        let toolhead = self.toolhead()?;
        let kin = toolhead.get_kinematics();
        let csp = kin.get_calibration()?;
        let mut probe_positions = Vec::new();
        for p in positions {
            let stable = csp.calc_stable_position(&[p.test_x, p.test_y, p.test_z])?;
            probe_positions.push((
                p.test_z - p.bed_z,
                StablePosition::List([stable[0], stable[1], stable[2]]),
            ));
        }
        // Perform analysis
        let distances = self.last_distances.borrow().clone();
        self.calculate_params(&probe_positions, &distances)
    }

    /// `calculate_params` (`delta_calibrate.py:164`).
    pub fn calculate_params(
        &self,
        probe_positions: &[HeightEntry],
        distances: &[DistanceEntry],
    ) -> KResult<()> {
        let mut height_positions: Vec<HeightEntry> =
            self.manual_heights.borrow().clone();
        height_positions.extend_from_slice(probe_positions);
        let toolhead = self.toolhead()?;
        let kin = toolhead.get_kinematics();
        let orig_cal = kin.get_calibration()?;
        // Python passes the distances list as `is_extended` (truthiness of
        // the list decides): reproduced as "extended when non-empty".
        let (adj_params, params) = parse_coordinate_descent_params(
            &orig_cal.coordinate_descent_params(!distances.is_empty()),
        )?;
        log::info(&pct(
            "Calculating delta_calibrate with:\n%s\n%s\nInitial delta_calibrate parameters: %s",
            &[
                pct_str(&height_positions_pyvalue(&height_positions).repr()),
                pct_str(&distances_pyvalue(distances).repr()),
                pct_str(&params_pyvalue(&params).repr()),
            ],
        )?);
        let z_weight = if distances.is_empty() {
            1.0
        } else {
            float_true_div(
                distances.len() as f64,
                MEASURE_WEIGHT * (probe_positions.len() as f64),
            )
            .map_err(py_err)?
        };
        let orig_call = Arc::clone(&orig_cal);
        let height_positions_clone = height_positions.clone();
        let distances_clone = distances.to_vec();
        let error_func = move |p: &DescentParams| -> KResult<f64> {
            match delta_error(&orig_call, p, &height_positions_clone, &distances_clone, z_weight) {
                Ok(v) => Ok(v),
                Err(e) if e.class_name() == "ValueError" => Ok(9999999999999.9),
                Err(e) => Err(e),
            }
        };
        let new_params =
            background_coordinate_descent(&*self.printer, &adj_params, params, error_func)?;
        log::info(&pct(
            "Calculated delta_calibrate parameters: %s",
            &[pct_str(&params_pyvalue(&new_params).repr())],
        )?);
        let new_cal = orig_cal.new_calibration(&params_pyvalue(&new_params))?;
        for (z_offset, spos) in height_positions.iter() {
            let spos = spos.values();
            log::info(&pct(
                "height orig: %.6f new: %.6f goal: %.6f",
                &[
                    float_pct(orig_cal.get_position_from_stable(spos)?[2]),
                    float_pct(new_cal.get_position_from_stable(spos)?[2]),
                    float_pct(*z_offset),
                ],
            )?);
        }
        for (dist, spos1, spos2) in distances.iter() {
            let spos1 = spos1.values();
            let spos2 = spos2.values();
            let orig_dist = dist_between(&orig_cal.get_position_from_stable(spos1)?, &orig_cal.get_position_from_stable(spos2)?)?;
            let new_dist = dist_between(&new_cal.get_position_from_stable(spos1)?, &new_cal.get_position_from_stable(spos2)?)?;
            log::info(&pct(
                "distance orig: %.6f new: %.6f goal: %.6f",
                &[float_pct(orig_dist), float_pct(new_dist), float_pct(*dist)],
            )?);
        }
        // Store results for SAVE_CONFIG
        self.save_state(probe_positions, distances, &new_cal)?;
        self.gcode().respond_info(
            "The SAVE_CONFIG command will update the printer config file\n\
            with these parameters and restart the printer.",
            true,
        );
        Ok(())
    }

    /// `cmd_DELTA_CALIBRATE` (`delta_calibrate.py:222`).
    pub fn cmd_delta_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        let Some(helper) = self.probe_helper.borrow().clone() else {
            return Err(KlippyError::Config("ProbePointsHelper not initialized".to_string()));
        };
        helper.start_probe(gcmd)
    }

    /// `add_manual_height` (`delta_calibrate.py:224`).
    pub fn add_manual_height(&self, height: f64) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let kin = toolhead.get_kinematics();
        let mut kin_spos: IndexMap<String, f64> = IndexMap::new();
        for stepper in kin.get_steppers() {
            kin_spos.insert(stepper.get_name(None), stepper.get_commanded_position());
        }
        let kin_pos_raw = kin.calc_position(&kin_spos)?;
        let kin_pos: Vec<f64> = kin_pos_raw
            .into_iter()
            .map(|v| {
                v.ok_or_else(|| KlippyError::Other {
                    class_name: "TypeError".to_string(),
                    message: "unsupported operand type(s) for -: 'float' and 'NoneType'".to_string(),
                })
            })
            .collect::<KResult<Vec<_>>>()?;
        // Convert location to a stable position
        let delta_params = kin.get_calibration()?;
        let stable = delta_params.calc_stable_position(&kin_pos)?;
        // `tuple(...)` in Python: a tuple, unlike the lists elsewhere.
        let stable_pos = StablePosition::Tuple([stable[0], stable[1], stable[2]]);
        // Add to list of manual heights
        self.manual_heights
            .borrow_mut()
            .push((height, stable_pos));
        self.gcode().respond_info(
            &pct(
                "Adding manual height: %.3f,%.3f,%.3f is actually z=%.3f",
                &[
                    float_pct(kin_pos[0]),
                    float_pct(kin_pos[1]),
                    float_pct(kin_pos[2]),
                    float_pct(height),
                ],
            )?,
            true,
        );
        Ok(())
    }

    /// `do_extended_calibration` (`delta_calibrate.py:240`).
    pub fn do_extended_calibration(&self) -> KResult<()> {
        // Extract distance positions
        let entry = self.delta_analyze_entry.borrow().clone();
        let distances: Vec<DistanceEntry> = if entry.len() <= 1 {
            self.last_distances.borrow().clone()
        } else if entry.len() < 5 {
            return Err(KlippyError::Command("Not all measurements provided".to_string()));
        } else {
            let kin = self.toolhead()?.get_kinematics();
            let delta_params = kin.get_calibration()?;
            measurements_to_distances(&entry, &delta_params)?
        };
        if self.last_probe_positions.borrow().is_empty() {
            return Err(KlippyError::Command(
                "Must run basic calibration with DELTA_CALIBRATE first".to_string(),
            ));
        }
        // Perform analysis
        let probe_positions = self.last_probe_positions.borrow().clone();
        self.calculate_params(&probe_positions, &distances)
    }

    /// `cmd_DELTA_ANALYZE` (`delta_calibrate.py:257`).
    pub fn cmd_delta_analyze(&self, gcmd: &GCmd) -> KResult<()> {
        // Check for manual height entry
        let mheight = gcmd.get_float(
            "MANUAL_HEIGHT",
            GCmdFloatOpts { default: Some(None), minval: None, maxval: None, above: None, below: None, ..Default::default() },
        )?;
        if let Some(mheight) = mheight {
            self.add_manual_height(mheight)?;
            return Ok(());
        }
        // Parse distance measurements
        // Iteration order is part of the observable behaviour (log lines).
        let arg_counts: [(&str, usize); 5] = [
            ("CENTER_DISTS", 6),
            ("CENTER_PILLAR_WIDTHS", 3),
            ("OUTER_DISTS", 6),
            ("OUTER_PILLAR_WIDTHS", 6),
            ("SCALE", 1),
        ];
        for (name, count) in arg_counts {
            let data = gcmd.get(
                name,
                GetOpts {
                    default: Some(PyValue::None),
                    parser: GetParser::Str,
                    minval: None,
                    maxval: None,
                    above: None,
                    below: None,
                },
            )?;
            let PyValue::Str(s) = data else { continue };
            let mut parts = Vec::new();
            for piece in s.split(',') {
                match float_from_string(piece) {
                    Ok(f) => parts.push(f),
                    Err(_) => {
                        return Err(KlippyError::Command(format!("Unable to parse parameter '{name}'")));
                    }
                }
            }
            if parts.len() != count {
                return Err(KlippyError::Command(format!(
                    "Parameter '{name}' must have {count} values"
                )));
            }
            let parts_value = parts
                .iter()
                .map(|f| PyValue::Float(*f))
                .collect::<Vec<_>>();
            self.delta_analyze_entry
                .borrow_mut()
                .insert(name.to_string(), parts);
            log::info(&pct(
                "DELTA_ANALYZE %s = %s",
                &[pct_str(name), pct_str(&PyValue::List(parts_value).repr())],
            )?);
        }
        // Perform analysis if requested
        let action = gcmd.get(
            "CALIBRATE",
            GetOpts {
                default: Some(PyValue::None),
                parser: GetParser::Str,
                minval: None,
                maxval: None,
                above: None,
                below: None,
            },
        )?;
        if let PyValue::Str(action) = action {
            if action != "extended" {
                return Err(KlippyError::Command("Unknown calibrate action".to_string()));
            }
            self.do_extended_calibration()?;
        }
        Ok(())
    }

    /// Restored probe positions (for tests).
    pub fn last_probe_positions(&self) -> Vec<HeightEntry> {
        self.last_probe_positions.borrow().clone()
    }

    /// Restored manual heights (for tests).
    pub fn manual_heights(&self) -> Vec<HeightEntry> {
        self.manual_heights.borrow().clone()
    }

    /// Restored distances (for tests).
    pub fn last_distances(&self) -> Vec<DistanceEntry> {
        self.last_distances.borrow().clone()
    }

    /// Current analyze entry (for tests).
    pub fn delta_analyze_entry(&self) -> MeasuredParams {
        self.delta_analyze_entry.borrow().clone()
    }

    /// Configured probe points (for tests).
    pub fn probe_points(&self) -> Vec<(f64, f64)> {
        match self.probe_helper.borrow().clone() {
            Some(helper) => helper
                .probe_points()
                .into_iter()
                .map(|v| (v[0], v[1]))
                .collect(),
            None => Vec::new(),
        }
    }
}

/// `distance orig` / `distance new` helper: Euclidean distance between two
/// 3-coordinates (`math.sqrt` may raise `ValueError: math domain error`
/// for negative radius squares, exactly as Python would).
fn dist_between(p1: &[f64], p2: &[f64]) -> KResult<f64> {
    math_sqrt(
        libm::pow(p1[0] - p2[0], 2.0) + libm::pow(p1[1] - p2[1], 2.0) + libm::pow(p1[2] - p2[2], 2.0),
    )
}

/// The `delta_errorfunc` body (`delta_calibrate.py:177`): sum of squared
/// z-height errors weighted by `z_weight` plus squared distance errors.
fn delta_error(
    orig_cal: &DeltaCalibration,
    params: &DescentParams,
    height_positions: &[HeightEntry],
    distances: &[DistanceEntry],
    z_weight: f64,
) -> KResult<f64> {
    // Build new delta_params for params under test
    let delta_params = orig_cal.new_calibration(&params_pyvalue(params))?;
    let mut total_error = 0.0;
    for (z_offset, stable_pos) in height_positions {
        let z = delta_params.get_position_from_stable(stable_pos.values())?[2];
        total_error += libm::pow(z - *z_offset, 2.0);
    }
    total_error *= z_weight;
    for (dist, stable_pos1, stable_pos2) in distances {
        let p1 = delta_params.get_position_from_stable(stable_pos1.values())?;
        let p2 = delta_params.get_position_from_stable(stable_pos2.values())?;
        let d = dist_between(&p1, &p2)?;
        total_error += libm::pow(d - *dist, 2.0);
    }
    Ok(total_error)
}

fn height_positions_pyvalue(entries: &[HeightEntry]) -> PyValue {
    PyValue::List(
        entries
            .iter()
            .map(|(z, spos)| PyValue::Tuple(vec![PyValue::Float(*z), spos.pyvalue()]))
            .collect(),
    )
}

fn distances_pyvalue(entries: &[DistanceEntry]) -> PyValue {
    PyValue::List(
        entries
            .iter()
            .map(|(d, s1, s2)| {
                PyValue::Tuple(vec![
                    PyValue::Float(*d),
                    s1.pyvalue(),
                    s2.pyvalue(),
                ])
            })
            .collect(),
    )
}

fn params_pyvalue(params: &IndexMap<String, f64>) -> PyValue {
    let mut dict = indexmap::IndexMap::new();
    for (k, v) in params {
        dict.insert(
            PyValue::Str(k.clone()),
            PyValue::Float(*v),
        );
    }
    PyValue::Dict(dict)
}

/// `load_config` (`delta_calibrate.py:286`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj: Arc<DeltaCalibrate> = DeltaCalibrate::new(config)?;
    Ok(Some(obj))
}

/// Module factory (`klippy/extras/delta_calibrate.py` entry point).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
