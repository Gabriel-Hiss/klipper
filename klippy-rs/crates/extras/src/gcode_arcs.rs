//! Port of the `gcode_arcs` module (`klippy/extras/gcode_arcs.py`).
//!
//! Adds G2/G3 arc support by approximating each arc with small G1
//! segments. The arc length per segment is configured with `resolution`
//! (mm per segment, default 1.0, above 0.0). G17/G18/G19 select the XY,
//! XZ or YZ plane (default XY); the remaining axis is helical.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeMove, RegisterCommandOpts};
use klippy_api::gcode::{GCmd, GetFloatOpts as GCodeGetFloatOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{GCell, ObjectRef, load_interface, lookup_interface};
use pycompat::PyValue;
use pycompat::text::{float_str, str_repr, trunc_float_to_bigint};
use std::sync::Arc;

/// `ARC_PLANE_X_Y` (`klippy/extras/gcode_arcs.py`).
pub const ARC_PLANE_X_Y: i32 = 0;
/// `ARC_PLANE_X_Z` (`klippy/extras/gcode_arcs.py`).
pub const ARC_PLANE_X_Z: i32 = 1;
/// `ARC_PLANE_Y_Z` (`klippy/extras/gcode_arcs.py`).
pub const ARC_PLANE_Y_Z: i32 = 2;

/// `X_AXIS` (`klippy/extras/gcode_arcs.py`).
pub const X_AXIS: usize = 0;
/// `Y_AXIS` (`klippy/extras/gcode_arcs.py`).
pub const Y_AXIS: usize = 1;
/// `Z_AXIS` (`klippy/extras/gcode_arcs.py`).
pub const Z_AXIS: usize = 2;
/// `E_AXIS` (`klippy/extras/gcode_arcs.py`).
pub const E_AXIS: usize = 3;

/// `klippy/extras/gcode_arcs.py:ArcSupport`.
pub struct ArcSupport {
    #[allow(dead_code)]
    printer: Printer,
    gcode: GCode,
    gcode_move: GCodeMove,
    mm_per_arc_segment: f64,
    plane: GCell<i32>,
}

impl ArcSupport {
    /// `ArcSupport.__init__(config)` (`klippy/extras/gcode_arcs.py:29`).
    ///
    /// Reads `resolution`, loads `gcode_move`, looks up `gcode` and
    /// registers `G2`, `G3`, `G17`, `G18`, `G19` in that order.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let mm_per_arc_segment = config
            .getfloat(
                "resolution",
                GetFloatOpts {
                    default: Some(Some(1.)),
                    above: Some(0.),
                    ..Default::default()
                },
            )?
            .expect("default given");
        let gcode_move =
            load_interface::<dyn klippy_api::gcode::GCodeMoveApi>(&*printer, config, "gcode_move", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let gcode = lookup_interface::<dyn klippy_api::gcode::GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let support = Arc::new(ArcSupport {
            printer,
            gcode,
            gcode_move,
            mm_per_arc_segment,
            plane: GCell::new(ARC_PLANE_X_Y),
        });

        let g2 = Arc::clone(&support);
        support.gcode.register_command(
            "G2",
            Some(Box::new(move |gcmd: &GCmd| g2.cmd_g2(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let g3 = Arc::clone(&support);
        support.gcode.register_command(
            "G3",
            Some(Box::new(move |gcmd: &GCmd| g3.cmd_g3(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let g17 = Arc::clone(&support);
        support.gcode.register_command(
            "G17",
            Some(Box::new(move |gcmd: &GCmd| g17.cmd_g17(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let g18 = Arc::clone(&support);
        support.gcode.register_command(
            "G18",
            Some(Box::new(move |gcmd: &GCmd| g18.cmd_g18(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let g19 = Arc::clone(&support);
        support.gcode.register_command(
            "G19",
            Some(Box::new(move |gcmd: &GCmd| g19.cmd_g19(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        Ok(support)
    }

    /// `cmd_G2` (`klippy/extras/gcode_arcs.py:45`).
    pub fn cmd_g2(&self, gcmd: &GCmd) -> KResult<()> {
        self.cmd_inner(gcmd, true)
    }

    /// `cmd_G3` (`klippy/extras/gcode_arcs.py:48`).
    pub fn cmd_g3(&self, gcmd: &GCmd) -> KResult<()> {
        self.cmd_inner(gcmd, false)
    }

    /// `cmd_G17` (`klippy/extras/gcode_arcs.py:51`).
    pub fn cmd_g17(&self, _gcmd: &GCmd) -> KResult<()> {
        self.plane.set(ARC_PLANE_X_Y);
        Ok(())
    }

    /// `cmd_G18` (`klippy/extras/gcode_arcs.py:54`).
    pub fn cmd_g18(&self, _gcmd: &GCmd) -> KResult<()> {
        self.plane.set(ARC_PLANE_X_Z);
        Ok(())
    }

    /// `cmd_G19` (`klippy/extras/gcode_arcs.py:57`).
    pub fn cmd_g19(&self, _gcmd: &GCmd) -> KResult<()> {
        self.plane.set(ARC_PLANE_Y_Z);
        Ok(())
    }

    /// Current plane selection (`ArcSupport.plane`).
    pub fn current_plane(&self) -> i32 {
        self.plane.get()
    }

    /// `_cmd_inner(gcmd, clockwise)` (`klippy/extras/gcode_arcs.py:60`).
    fn cmd_inner(&self, gcmd: &GCmd, clockwise: bool) -> KResult<()> {
        let status = self.gcode_move.get_status(None);
        if !bool_field(&status, "absolute_coordinates")? {
            return Err(KlippyError::command("G2/G3 does not support relative move mode"));
        }
        let current_pos = position_field(&status, "gcode_position")?;
        let absolute_extrude = bool_field(&status, "absolute_extrude")?;

        let mut target_pos = vec![0.; 4];
        for (slot, name) in [(0, "X"), (1, "Y"), (2, "Z")] {
            target_pos[slot] = get_float(gcmd, name, Some(current_pos[slot]))?.expect("default given");
        }

        if get_float(gcmd, "R", None)?.is_some() {
            return Err(KlippyError::command("G2/G3 does not support R moves"));
        }

        let i = get_float(gcmd, "I", Some(0.))?.expect("default given");
        let j = get_float(gcmd, "J", Some(0.))?.expect("default given");
        let (planar, alpha_axis, beta_axis, helical_axis) = match self.current_plane() {
            ARC_PLANE_X_Z => {
                let k = get_float(gcmd, "K", Some(0.))?.expect("default given");
                ([i, k], X_AXIS, Z_AXIS, Y_AXIS)
            }
            ARC_PLANE_Y_Z => {
                let k = get_float(gcmd, "K", Some(0.))?.expect("default given");
                ([j, k], Y_AXIS, Z_AXIS, X_AXIS)
            }
            _ => ([i, j], X_AXIS, Y_AXIS, Z_AXIS),
        };

        if planar[0] == 0. && planar[1] == 0. {
            return Err(KlippyError::command("G2/G3 requires IJ, IK or JK parameters"));
        }

        self.plan_arc(
            &current_pos,
            &target_pos,
            planar,
            clockwise,
            gcmd,
            absolute_extrude,
            alpha_axis,
            beta_axis,
            helical_axis,
        )
    }

    /// `planArc(...)` (`klippy/extras/gcode_arcs.py:104`).
    ///
    /// The arc is approximated by generating many small linear segments.
    #[allow(clippy::too_many_arguments)]
    fn plan_arc(
        &self,
        current_pos: &[f64],
        target_pos: &[f64],
        offset: [f64; 2],
        clockwise: bool,
        gcmd: &GCmd,
        absolute_extrude: bool,
        alpha_axis: usize,
        beta_axis: usize,
        helical_axis: usize,
    ) -> KResult<()> {
        let r_p = -offset[0];
        let r_q = -offset[1];
        let center_p = current_pos[alpha_axis] - r_p;
        let center_q = current_pos[beta_axis] - r_q;
        let rt_alpha = target_pos[alpha_axis] - center_p;
        let rt_beta = target_pos[beta_axis] - center_q;
        let mut angular_travel = kos::libm::atan2(
            r_p * rt_beta - r_q * rt_alpha,
            r_p * rt_alpha + r_q * rt_beta,
        );
        if angular_travel < 0. {
            angular_travel += 2. * std::f64::consts::PI;
        }
        if clockwise {
            angular_travel -= 2. * std::f64::consts::PI;
        }
        if angular_travel == 0.
            && current_pos[alpha_axis] == target_pos[alpha_axis]
            && current_pos[beta_axis] == target_pos[beta_axis]
        {
            angular_travel = 2. * std::f64::consts::PI;
        }

        let linear_travel = target_pos[helical_axis] - current_pos[helical_axis];
        let radius = kos::libm::hypot(r_p, r_q);
        let flat_mm = radius * angular_travel;
        let mm_of_travel = if linear_travel != 0. {
            kos::libm::hypot(flat_mm, linear_travel)
        } else {
            kos::libm::fabs(flat_mm)
        };

        // `math.floor` raises on non-finite values (`klippy/extras/gcode_arcs.py:140`).
        let floored = kos::libm::floor(mm_of_travel / self.mm_per_arc_segment);
        if floored.is_nan() {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "cannot convert float NaN to integer".to_string(),
            });
        }
        if floored == f64::INFINITY {
            return Err(KlippyError::Internal {
                class_name: "OverflowError".to_string(),
                message: "cannot convert float infinity to integer".to_string(),
            });
        }
        let segments = f64::max(1., floored);

        let theta_per_segment = angular_travel / segments;
        let linear_per_segment = linear_travel / segments;

        let as_e = get_float(gcmd, "E", None)?;
        let as_f = get_float(gcmd, "F", None)?;

        let mut e_per_move = 0.;
        let mut e_base = 0.;
        if let Some(as_e) = as_e {
            if absolute_extrude {
                e_base = current_pos[E_AXIS];
            }
            e_per_move = (as_e - e_base) / segments;
        }

        let count = segment_count(segments)?;
        for index in 1..=count {
            let dist_helical = index as f64 * linear_per_segment;
            let c_theta = index as f64 * theta_per_segment;
            let cos_ti = kos::libm::cos(c_theta);
            let sin_ti = kos::libm::sin(c_theta);
            let step_r_p = -offset[0] * cos_ti + offset[1] * sin_ti;
            let step_r_q = -offset[0] * sin_ti - offset[1] * cos_ti;

            let mut c = [None; 3];
            c[alpha_axis] = Some(center_p + step_r_p);
            c[beta_axis] = Some(center_q + step_r_q);
            c[helical_axis] = Some(current_pos[helical_axis] + dist_helical);

            let coords = if index as f64 == segments {
                [target_pos[0], target_pos[1], target_pos[2]]
            } else {
                [c[0].unwrap(), c[1].unwrap(), c[2].unwrap()]
            };

            let mut g1_params: IndexMap<String, String> = IndexMap::new();
            g1_params.insert("X".to_string(), float_str(coords[0]));
            g1_params.insert("Y".to_string(), float_str(coords[1]));
            g1_params.insert("Z".to_string(), float_str(coords[2]));
            if e_per_move != 0. {
                g1_params.insert("E".to_string(), float_str(e_base + e_per_move));
                if absolute_extrude {
                    e_base += e_per_move;
                }
            }
            if let Some(as_f) = as_f {
                g1_params.insert("F".to_string(), float_str(as_f));
            }
            let g1_gcmd = self.gcode.create_gcode_command("G1", "G1", &g1_params);
            self.gcode_move.cmd_g1(&g1_gcmd)?;
        }
        Ok(())
    }
}

klippy_api::printer_object!(ArcSupport);

/// `int(segments)` with Python's truncation toward zero.
fn segment_count(segments: f64) -> KResult<i64> {
    let truncated = trunc_float_to_bigint(segments);
    let text = truncated.inner().to_string();
    text.parse::<i64>().map_err(|_| KlippyError::Internal {
        class_name: "OverflowError".to_string(),
        message: "cannot convert float to integer".to_string(),
    })
}

/// `gcmd.get_float(name, default)`.
fn get_float(gcmd: &GCmd, name: &str, default: Option<f64>) -> KResult<Option<f64>> {
    gcmd.get_float(
        name,
        GCodeGetFloatOpts {
            default: Some(default),
            ..Default::default()
        },
    )
}

/// Dict lookup of a boolean status field (`gcodestatus['absolute_*']`).
fn bool_field(status: &PyValue, key: &str) -> KResult<bool> {
    Ok(dict_get(status, key)?.truthy())
}

/// Dict lookup of the `gcode_position` Coord, padded to 4 like `gcode.Coord`.
fn position_field(status: &PyValue, key: &str) -> KResult<Vec<f64>> {
    let value = dict_get(status, key)?;
    let items = match value {
        PyValue::Tuple(items) | PyValue::List(items) => items,
        _ => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!("'{}' object is not subscriptable", pycompat::value::type_name(value)),
            });
        }
    };
    let mut position: Vec<f64> = items
        .iter()
        .map(|item| item.to_f64().unwrap_or(0.))
        .collect();
    while position.len() < 4 {
        position.push(0.);
    }
    Ok(position)
}

fn dict_get<'a>(status: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    let PyValue::Dict(entries) = status else {
        return Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!("'{}' object is not subscriptable", pycompat::value::type_name(status)),
        });
    };
    for (k, v) in entries.iter() {
        if let PyValue::Str(name) = k {
            if name == key {
                return Ok(v);
            }
        }
    }
    Err(KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: str_repr(key),
    })
}

/// `load_config(config)` (`klippy/extras/gcode_arcs.py:182`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(ArcSupport::new(config)? as ObjectRef))
}

/// Module entry point for the `gcode_arcs` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
