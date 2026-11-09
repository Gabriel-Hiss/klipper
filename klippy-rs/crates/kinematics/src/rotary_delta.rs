//! Port of the `rotary_delta` kinematics module
//! (`klippy/kinematics/rotary_delta.py`).
//!
//! `RotaryDeltaKinematics` is the `load_kinematics(toolhead, config)`
//! result: three tower rails configured with
//! `units_in_radians=True` and driven by the iterative solver
//! through the `rotary_delta_stepper_alloc` C allocation. The
//! `RotaryDeltaCalibration` helper owns the geometry parameters
//! used by the `DELTA_CALIBRATE` tool (`coordinate_descent_params`,
//! `new_calibration`, `save_state`); like the Python module it
//! allocates one `rotary_delta_stepper_alloc` stepper kinematics per
//! tower and computes `abs_endstops` / `calc_stable_position` with
//! `itersolve_calc_position_from_coord` on them, through
//! `klippy_core::motion_handles` and the `chelper_motion` port.
//! `actuator_to_cartesian` uses `klippy_core::mathutil::trilateration`.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, PrinterConfig};
use klippy_api::gcode::GCodeApi;
use klippy_api::log;
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::stepper::{Rail, RailOpts, Stepper};
use klippy_api::toolhead::{
    Coord, DeltaCalibration, DeltaCalibrationApi, HomingStateHandle, Kinematics, KinematicsApi,
    MoveApi, ToolHead,
};
use klippy_api::stepper::StepperKinematics;
use klippy_api::{lookup_interface, GCell, KResult, KlippyError};
use klippy_core::motion_handles::{alloc_stepper_kinematics, kinematics_of};
use pycompat::libm;
use pycompat::text::{percent_format, PctValue};
use pycompat::value::repr_key;
use pycompat::PyValue;
use std::sync::Arc;

/// `math.radians(x)` (`Modules/mathmodule.c`): `x * (pi / 180.0)`.
fn radians(x: f64) -> f64 {
    x * (std::f64::consts::PI / 180.0)
}

/// `x ** 2.0` (`Objects/floatobject.c`): C `pow(x, 2.0)` via libm.
fn sq(x: f64) -> f64 {
    libm::pow(x, 2.0)
}

/// Float `/`: raises `ZeroDivisionError: float division by zero`
/// when the divisor is `0.0` (or `-0.0`).
fn div_py(a: f64, b: f64) -> KResult<f64> {
    if b == 0.0 {
        Err(KlippyError::Internal {
            class_name: "ZeroDivisionError".to_string(),
            message: "float division by zero".to_string(),
        })
    } else {
        Ok(a / b)
    }
}

fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("valid format strings only")
}

/// Rotary delta parameter calibration for the `DELTA_CALIBRATE` tool
/// (`klippy/kinematics/rotary_delta.py:133`).
pub struct RotaryDeltaCalibration {
    shoulder_radius: f64,
    shoulder_height: f64,
    angles: [f64; 3],
    upper_arms: [f64; 3],
    lower_arms: [f64; 3],
    endstops: [f64; 3],
    stepdists: [f64; 3],
    abs_endstops: [f64; 3],
    // `math.cos/sin(math.radians(angle))` per tower, used by
    // `elbow_coord` (`rotary_delta.py:183-185`).
    cos_angle: [f64; 3],
    sin_angle: [f64; 3],
    // `self.sks`: one `rotary_delta_stepper_alloc` per tower
    // (`rotary_delta.py:145-148`).
    sks: [StepperKinematics; 3],
}

impl RotaryDeltaCalibration {
    /// `RotaryDeltaCalibration.__init__` (`rotary_delta.py:134`).
    pub fn new(
        shoulder_radius: f64,
        shoulder_height: f64,
        angles: [f64; 3],
        upper_arms: [f64; 3],
        lower_arms: [f64; 3],
        endstops: [f64; 3],
        stepdists: [f64; 3],
    ) -> RotaryDeltaCalibration {
        let mut cos_angle = [0.0f64; 3];
        let mut sin_angle = [0.0f64; 3];
        for i in 0..3 {
            let a = radians(angles[i]);
            cos_angle[i] = libm::cos(a);
            sin_angle[i] = libm::sin(a);
        }
        let sks = [0, 1, 2].map(|i| {
            let params = [
                PyValue::Float(shoulder_radius),
                PyValue::Float(shoulder_height),
                PyValue::Float(radians(angles[i])),
                PyValue::Float(upper_arms[i]),
                PyValue::Float(lower_arms[i]),
            ];
            alloc_stepper_kinematics("rotary_delta_stepper_alloc", &params)
                .expect("rotary_delta_stepper_alloc takes five doubles")
        });
        let mut cal = RotaryDeltaCalibration {
            shoulder_radius,
            shoulder_height,
            angles,
            upper_arms,
            lower_arms,
            endstops,
            stepdists,
            abs_endstops: [0.0; 3],
            cos_angle,
            sin_angle,
            sks,
        };
        for i in 0..3 {
            cal.abs_endstops[i] = cal.calc_position_from_coord(i, 0.0, 0.0, endstops[i]);
        }
        cal
    }

    /// Elbow position in the shoulder-joint coordinate system, shifted
    /// and rotated to main cartesian coordinates (`rotary_delta.py:178`).
    pub fn elbow_coord(&self, elbow_id: usize, spos: f64) -> (f64, f64, f64) {
        let sj_elbow_x = self.upper_arms[elbow_id] * libm::cos(spos);
        let sj_elbow_y = self.upper_arms[elbow_id] * libm::sin(spos);
        let x = (sj_elbow_x + self.shoulder_radius) * self.cos_angle[elbow_id];
        let y = (sj_elbow_x + self.shoulder_radius) * self.sin_angle[elbow_id];
        let z = sj_elbow_y + self.shoulder_height;
        (x, y, z)
    }

    /// `actuator_to_cartesian` (`rotary_delta.py:188`): trilateration
    /// over the three elbow sphere centers with the lower-arm radii.
    pub fn actuator_to_cartesian(&self, spos: &[f64]) -> KResult<[f64; 3]> {
        let elbow = |i: usize| {
            let (x, y, z) = self.elbow_coord(i, spos[i]);
            [x, y, z]
        };
        let sphere_coords = [elbow(0), elbow(1), elbow(2)];
        let lower_arm2 = [
            sq(self.lower_arms[0]),
            sq(self.lower_arms[1]),
            sq(self.lower_arms[2]),
        ];
        klippy_core::mathutil::trilateration(&sphere_coords, &lower_arm2)
    }

    /// `ffi_lib.itersolve_calc_position_from_coord(self.sks[tower], x, y, z)`
    /// (`rotary_delta.py:150,200`).
    fn calc_position_from_coord(&self, tower: usize, x: f64, y: f64, z: f64) -> f64 {
        kinematics_of(&self.sks[tower])
            .expect("allocated by rotary_delta_stepper_alloc")
            .with_sk(|sk| sk.calc_position_from_coord(x, y, z))
    }
}

fn param_f64(params: &PyValue, key: &str) -> KResult<f64> {
    let PyValue::Dict(map) = params else {
        return Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "'{}' object is not subscriptable",
                pycompat::value::type_name(params)
            ),
        });
    };
    let Some(value) = map.get(&PyValue::Str(key.to_string())) else {
        return Err(KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: repr_key(key),
        });
    };
    value.to_f64().ok_or_else(|| KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: format!(
            "unsupported operand type(s) for -: '{}' and 'float'",
            pycompat::value::type_name(value)
        ),
    })
}

impl DeltaCalibrationApi for RotaryDeltaCalibration {
    fn coordinate_descent_params(&self, is_extended: bool) -> PyValue {
        let mut adj_params: Vec<PyValue> = [
            "shoulder_height",
            "endstop_a",
            "endstop_b",
            "endstop_c",
        ]
        .iter()
        .map(|s| PyValue::Str(s.to_string()))
        .collect();
        if is_extended {
            adj_params.extend([
                PyValue::Str("shoulder_radius".to_string()),
                PyValue::Str("angle_a".to_string()),
                PyValue::Str("angle_b".to_string()),
            ]);
        }
        let mut params = IndexMap::new();
        params.insert(
            PyValue::Str("shoulder_radius".to_string()),
            PyValue::Float(self.shoulder_radius),
        );
        params.insert(
            PyValue::Str("shoulder_height".to_string()),
            PyValue::Float(self.shoulder_height),
        );
        for (i, axis) in ['a', 'b', 'c'].iter().enumerate() {
            params.insert(
                PyValue::Str(format!("angle_{axis}")),
                PyValue::Float(self.angles[i]),
            );
            params.insert(
                PyValue::Str(format!("upper_arm_{axis}")),
                PyValue::Float(self.upper_arms[i]),
            );
            params.insert(
                PyValue::Str(format!("lower_arm_{axis}")),
                PyValue::Float(self.lower_arms[i]),
            );
            params.insert(
                PyValue::Str(format!("endstop_{axis}")),
                PyValue::Float(self.endstops[i]),
            );
            params.insert(
                PyValue::Str(format!("stepdist_{axis}")),
                PyValue::Float(self.stepdists[i]),
            );
        }
        PyValue::Tuple(vec![
            PyValue::Tuple(adj_params),
            PyValue::Dict(params),
        ])
    }

    fn new_calibration(&self, params: &PyValue) -> KResult<DeltaCalibration> {
        let shoulder_radius = param_f64(params, "shoulder_radius")?;
        let shoulder_height = param_f64(params, "shoulder_height")?;
        let angles = [
            param_f64(params, "angle_a")?,
            param_f64(params, "angle_b")?,
            param_f64(params, "angle_c")?,
        ];
        let upper_arms = [
            param_f64(params, "upper_arm_a")?,
            param_f64(params, "upper_arm_b")?,
            param_f64(params, "upper_arm_c")?,
        ];
        let lower_arms = [
            param_f64(params, "lower_arm_a")?,
            param_f64(params, "lower_arm_b")?,
            param_f64(params, "lower_arm_c")?,
        ];
        let endstops = [
            param_f64(params, "endstop_a")?,
            param_f64(params, "endstop_b")?,
            param_f64(params, "endstop_c")?,
        ];
        let stepdists = [
            param_f64(params, "stepdist_a")?,
            param_f64(params, "stepdist_b")?,
            param_f64(params, "stepdist_c")?,
        ];
        let cal: DeltaCalibration = Arc::new(RotaryDeltaCalibration::new(
            shoulder_radius,
            shoulder_height,
            angles,
            upper_arms,
            lower_arms,
            endstops,
            stepdists,
        ));
        Ok(cal)
    }

    fn get_position_from_stable(&self, stable_position: &[f64]) -> KResult<Vec<f64>> {
        let spos = [
            self.abs_endstops[0] - stable_position[0] * self.stepdists[0],
            self.abs_endstops[1] - stable_position[1] * self.stepdists[1],
            self.abs_endstops[2] - stable_position[2] * self.stepdists[2],
        ];
        let p = self.actuator_to_cartesian(&spos)?;
        Ok(vec![p[0], p[1], p[2]])
    }

    fn calc_stable_position(&self, coord: &[f64]) -> KResult<Vec<f64>> {
        let mut out = Vec::with_capacity(3);
        for i in 0..3 {
            let pos = self.calc_position_from_coord(i, coord[0], coord[1], coord[2]);
            out.push(div_py(self.abs_endstops[i] - pos, self.stepdists[i])?);
        }
        Ok(out)
    }

    fn save_state(&self, configfile: &PrinterConfig) -> KResult<()> {
        configfile.set(
            "printer",
            "shoulder_radius",
            &pct("%.6f", &[PctValue::Float(self.shoulder_radius)]),
        )?;
        configfile.set(
            "printer",
            "shoulder_height",
            &pct("%.6f", &[PctValue::Float(self.shoulder_height)]),
        )?;
        for (i, axis) in ['a', 'b', 'c'].iter().enumerate() {
            configfile.set(
                &format!("stepper_{axis}"),
                "angle",
                &pct("%.6f", &[PctValue::Float(self.angles[i])]),
            )?;
            configfile.set(
                &format!("stepper_{axis}"),
                "position_endstop",
                &pct("%.6f", &[PctValue::Float(self.endstops[i])]),
            )?;
        }
        let gcode = lookup_interface::<dyn GCodeApi>(
            &*configfile.get_printer(),
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let msg = pct(
            "stepper_a: position_endstop: %.6f angle: %.6f\n\
             stepper_b: position_endstop: %.6f angle: %.6f\n\
             stepper_c: position_endstop: %.6f angle: %.6f\n\
             shoulder_radius: %.6f shoulder_height: %.6f",
            &[
                PctValue::Float(self.endstops[0]),
                PctValue::Float(self.angles[0]),
                PctValue::Float(self.endstops[1]),
                PctValue::Float(self.angles[1]),
                PctValue::Float(self.endstops[2]),
                PctValue::Float(self.angles[2]),
                PctValue::Float(self.shoulder_radius),
                PctValue::Float(self.shoulder_height),
            ],
        );
        gcode.respond_info(&msg, true);
        Ok(())
    }
}

/// Mutable kinematics state (`need_home`, `limit_xy2`), kept in a
/// `GCell` because `check_move` / `set_position` take `&self`.
struct KinState {
    limit_xy2: f64,
    need_home: bool,
}

/// `RotaryDeltaKinematics` (`rotary_delta.py:9`). The kinematics
/// object itself is not a printer object; it is returned by
/// [`load_kinematics`].
pub struct RotaryDeltaKinematics {
    rails: [Rail; 3],
    calibration: Arc<RotaryDeltaCalibration>,
    max_z_velocity: f64,
    max_z: f64,
    min_z: f64,
    limit_z: f64,
    max_xy2: f64,
    home_position: [f64; 3],
    axes_min: Coord,
    axes_max: Coord,
    state: GCell<KinState>,
}

impl RotaryDeltaKinematics {
    /// `RotaryDeltaKinematics.__init__(toolhead, config)`
    /// (`rotary_delta.py:10-76`).
    pub fn new(toolhead: &ToolHead, config: &Config) -> KResult<RotaryDeltaKinematics> {
        let stepper_configs: [Config; 3] = [
            config.getsection("stepper_a"),
            config.getsection("stepper_b"),
            config.getsection("stepper_c"),
        ];
        let factory = toolhead.stepper_factory();
        let rail_a = factory.lookup_rail(
            &stepper_configs[0],
            RailOpts {
                need_position_minmax: false,
                default_position_endstop: None,
                units_in_radians: true,
            },
        )?;
        let a_endstop = rail_a.get_homing_info().position_endstop;
        let rail_b = factory.lookup_rail(
            &stepper_configs[1],
            RailOpts {
                need_position_minmax: false,
                default_position_endstop: Some(a_endstop),
                units_in_radians: true,
            },
        )?;
        let rail_c = factory.lookup_rail(
            &stepper_configs[2],
            RailOpts {
                need_position_minmax: false,
                default_position_endstop: Some(a_endstop),
                units_in_radians: true,
            },
        )?;
        let rails: [Rail; 3] = [rail_a, rail_b, rail_c];

        let (max_velocity, _max_accel) = toolhead.get_max_velocity();
        let max_z_velocity = config
            .getfloat(
                "max_z_velocity",
                GetFloatOpts {
                    default: Some(Some(max_velocity)),
                    above: Some(0.0),
                    maxval: Some(max_velocity),
                    ..Default::default()
                },
            )?
            .expect("getfloat with default returns a value");
        let shoulder_radius = config
            .getfloat(
                "shoulder_radius",
                GetFloatOpts {
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("required getfloat returns a value");
        let shoulder_height = config
            .getfloat(
                "shoulder_height",
                GetFloatOpts {
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("required getfloat returns a value");
        let a_upper_arm = stepper_configs[0]
            .getfloat(
                "upper_arm_length",
                GetFloatOpts {
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("required getfloat returns a value");
        let upper_arms = [
            stepper_configs[0]
                .getfloat(
                    "upper_arm_length",
                    GetFloatOpts {
                        default: Some(Some(a_upper_arm)),
                        above: Some(0.0),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
            stepper_configs[1]
                .getfloat(
                    "upper_arm_length",
                    GetFloatOpts {
                        default: Some(Some(a_upper_arm)),
                        above: Some(0.0),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
            stepper_configs[2]
                .getfloat(
                    "upper_arm_length",
                    GetFloatOpts {
                        default: Some(Some(a_upper_arm)),
                        above: Some(0.0),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
        ];
        let a_lower_arm = stepper_configs[0]
            .getfloat(
                "lower_arm_length",
                GetFloatOpts {
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("required getfloat returns a value");
        let lower_arms = [
            stepper_configs[0]
                .getfloat(
                    "lower_arm_length",
                    GetFloatOpts {
                        default: Some(Some(a_lower_arm)),
                        above: Some(0.0),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
            stepper_configs[1]
                .getfloat(
                    "lower_arm_length",
                    GetFloatOpts {
                        default: Some(Some(a_lower_arm)),
                        above: Some(0.0),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
            stepper_configs[2]
                .getfloat(
                    "lower_arm_length",
                    GetFloatOpts {
                        default: Some(Some(a_lower_arm)),
                        above: Some(0.0),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
        ];
        let default_angles = [30.0f64, 150.0, 270.0];
        let angles = [
            stepper_configs[0]
                .getfloat(
                    "angle",
                    GetFloatOpts {
                        default: Some(Some(default_angles[0])),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
            stepper_configs[1]
                .getfloat(
                    "angle",
                    GetFloatOpts {
                        default: Some(Some(default_angles[1])),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
            stepper_configs[2]
                .getfloat(
                    "angle",
                    GetFloatOpts {
                        default: Some(Some(default_angles[2])),
                        ..Default::default()
                    },
                )?
                .expect("getfloat with default returns a value"),
        ];

        let endstops = [
            rails[0].get_homing_info().position_endstop,
            rails[1].get_homing_info().position_endstop,
            rails[2].get_homing_info().position_endstop,
        ];
        let stepdists = [
            rails[0].get_steppers()[0].get_step_dist(),
            rails[1].get_steppers()[0].get_step_dist(),
            rails[2].get_steppers()[0].get_step_dist(),
        ];
        let calibration = Arc::new(RotaryDeltaCalibration::new(
            shoulder_radius,
            shoulder_height,
            angles,
            upper_arms,
            lower_arms,
            endstops,
            stepdists,
        ));

        for (i, rail) in rails.iter().enumerate() {
            rail.setup_itersolve(
                "rotary_delta_stepper_alloc",
                &[
                    PyValue::Float(shoulder_radius),
                    PyValue::Float(shoulder_height),
                    PyValue::Float(radians(angles[i])),
                    PyValue::Float(upper_arms[i]),
                    PyValue::Float(lower_arms[i]),
                ],
            );
        }
        for rail in &rails {
            for stepper in rail.get_steppers() {
                stepper.set_trapq(Some(toolhead.get_trapq()));
            }
        }

        let eangles = [
            rails[0].calc_position_from_coord([0.0, 0.0, endstops[0]]),
            rails[1].calc_position_from_coord([0.0, 0.0, endstops[1]]),
            rails[2].calc_position_from_coord([0.0, 0.0, endstops[2]]),
        ];
        let home_position = calibration.actuator_to_cartesian(&eangles)?;
        let max_z = endstops[0].min(endstops[1]).min(endstops[2]);
        let min_z = config
            .getfloat(
                "minimum_z_position",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    maxval: Some(max_z),
                    ints: klippy_api::config::IntArgs::DEFAULT,
                    ..Default::default()
                },
            )?
            .expect("getfloat with default returns a value");
        let min_ua = (shoulder_radius + upper_arms[0])
            .min(shoulder_radius + upper_arms[1])
            .min(shoulder_radius + upper_arms[2]);
        let min_la = (lower_arms[0] - shoulder_radius)
            .min(lower_arms[1] - shoulder_radius)
            .min(lower_arms[2] - shoulder_radius);
        let max_xy2 = sq(min_ua.min(min_la));
        let arm_z = [
            calibration.elbow_coord(0, eangles[0]).2,
            calibration.elbow_coord(1, eangles[1]).2,
            calibration.elbow_coord(2, eangles[2]).2,
        ];
        let limit_z = (arm_z[0] - lower_arms[0])
            .min(arm_z[1] - lower_arms[1])
            .min(arm_z[2] - lower_arms[2]);
        log::info(&pct(
            "Delta max build height %.2fmm (radius tapered above %.2fmm)",
            &[PctValue::Float(max_z), PctValue::Float(limit_z)],
        ));
        let max_xy = libm::sqrt(max_xy2);
        let axes_min = Coord(vec![-max_xy, -max_xy, min_z, 0.0]);
        let axes_max = Coord(vec![max_xy, max_xy, max_z, 0.0]);
        let kin = RotaryDeltaKinematics {
            rails,
            calibration,
            max_z_velocity,
            max_z,
            min_z,
            limit_z,
            max_xy2,
            home_position,
            axes_min,
            axes_max,
            state: GCell::new(KinState {
                limit_xy2: -1.0,
                need_home: true,
            }),
        };
        kin.set_position(vec![0.0, 0.0, 0.0], "")?;
        Ok(kin)
    }
    /// `max_z_velocity` after config (`rotary_delta.py:22`).
    pub fn max_z_velocity(&self) -> f64 {
        self.max_z_velocity
    }
    /// `max_z` (`rotary_delta.py:71`).
    pub fn max_z(&self) -> f64 {
        self.max_z
    }
    /// `min_z` (`rotary_delta.py:72`).
    pub fn min_z(&self) -> f64 {
        self.min_z
    }
    /// `limit_z` (`rotary_delta.py:77`).
    pub fn limit_z(&self) -> f64 {
        self.limit_z
    }
    /// `max_xy2` (`rotary_delta.py:75`).
    pub fn max_xy2(&self) -> f64 {
        self.max_xy2
    }
    /// `home_position` (`rotary_delta.py:70`).
    pub fn home_position(&self) -> [f64; 3] {
        self.home_position
    }
    /// `axes_min` (`rotary_delta.py:81`).
    pub fn axes_min(&self) -> &Coord {
        &self.axes_min
    }
    /// `axes_max` (`rotary_delta.py:82`).
    pub fn axes_max(&self) -> &Coord {
        &self.axes_max
    }
    /// `limit_xy2` (`rotary_delta.py:68`).
    pub fn limit_xy2(&self) -> f64 {
        self.state.borrow().limit_xy2
    }
    /// `need_home` (`rotary_delta.py:67`).
    pub fn need_home(&self) -> bool {
        self.state.borrow().need_home
    }
}

fn coord_pyvalue(coord: &Coord) -> PyValue {
    // `toolhead.Coord` pads 3-tuples with the int `0` (`gcode.py:15-16`),
    // so the padding element is an `int`, not a `float`.
    let mut values = coord
        .0
        .iter()
        .take(3)
        .map(|v| PyValue::Float(*v))
        .collect::<Vec<_>>();
    values.push(PyValue::Int(pycompat::PyInt::from_i64(0)));
    PyValue::Tuple(values)
}

impl KinematicsApi for RotaryDeltaKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        self.rails
            .iter()
            .flat_map(|rail| rail.get_steppers())
            .collect()
    }

    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let mut spos = [0.0f64; 3];
        for (i, rail) in self.rails.iter().enumerate() {
            let name = rail.get_name(None);
            match stepper_positions.get(&name) {
                Some(&v) => spos[i] = v,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "KeyError".to_string(),
                        message: repr_key(&name),
                    });
                }
            }
        }
        let pos = self.calibration.actuator_to_cartesian(&spos)?;
        Ok(vec![Some(pos[0]), Some(pos[1]), Some(pos[2])])
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        let coord = [newpos[0], newpos[1], newpos[2]];
        for rail in &self.rails {
            rail.set_position(coord);
        }
        let mut state = self.state.borrow_mut();
        state.limit_xy2 = -1.0;
        if homing_axes == "xyz" {
            state.need_home = false;
        }
        Ok(())
    }

    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        let end_xy2 = sq(end_pos.0[0]) + sq(end_pos.0[1]);
        let (limit_xy2_read, need_home) = {
            let state = self.state.borrow();
            (state.limit_xy2, state.need_home)
        };
        if end_xy2 <= limit_xy2_read && mv.axes_d().0[2] == 0.0 {
            return Ok(());
        }
        if need_home {
            return Err(mv.move_error("Must home first".to_string()));
        }
        let end_z = end_pos.0[2];
        let mut limit_xy2 = self.max_xy2;
        if end_z > self.limit_z {
            limit_xy2 = limit_xy2.min(sq(self.max_z - end_z));
        }
        if end_xy2 > limit_xy2 || end_z > self.max_z || end_z < self.min_z {
            if end_pos.0[0] != self.home_position[0]
                || end_pos.0[1] != self.home_position[1]
                || end_z < self.min_z
                || end_z > self.home_position[2]
            {
                return Err(mv.move_error("Move out of range".to_string()));
            }
            limit_xy2 = -1.0;
        }
        if mv.axes_d().0[2] != 0.0 {
            mv.limit_speed(self.max_z_velocity, mv.accel())?;
            limit_xy2 = -1.0;
        }
        self.state.borrow_mut().limit_xy2 = limit_xy2;
        Ok(())
    }

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        homing_state.set_axes(vec![0, 1, 2]);
        let mut forcepos = self.home_position;
        forcepos[2] = -1.0;
        homing_state.home_rails(
            self.rails.to_vec(),
            forcepos.iter().map(|v| Some(*v)).collect(),
            self.home_position.iter().map(|v| Some(*v)).collect(),
        )
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        let homed_axes = if self.state.borrow().need_home {
            ""
        } else {
            "xyz"
        };
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str(homed_axes.to_string()),
        );
        status.insert(
            PyValue::Str("axis_minimum".to_string()),
            coord_pyvalue(&self.axes_min),
        );
        status.insert(
            PyValue::Str("axis_maximum".to_string()),
            coord_pyvalue(&self.axes_max),
        );
        PyValue::Dict(status)
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        if !clear_axes.is_empty() {
            let mut state = self.state.borrow_mut();
            state.limit_xy2 = -1.0;
            state.need_home = true;
        }
    }

    fn get_calibration(&self) -> KResult<DeltaCalibration> {
        let calibration: DeltaCalibration = self.calibration.clone();
        Ok(calibration)
    }
}

/// `load_kinematics(toolhead, config)` (`rotary_delta.py:226`).
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    let kin = RotaryDeltaKinematics::new(toolhead, config)?;
    let kin: Kinematics = Arc::new(kin);
    Ok(kin)
}

/// Module entry point. The Python module exposes only
/// `load_kinematics`, so no printer-object factory entries exist.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
