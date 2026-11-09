//! Port of `klippy/extras/bed_mesh.py`.
//!
// Medbed Mesh Leveling: mesh generation, interpolation, probing helpers,
// profiles, adaptive mesh, rapid scan, and move-transform splitting.
//!
// Faithful port of `klippy/extras/bed_mesh.py` (1770 lines): every branch,
// message text, float operation order and error class follows the Python.
// Python bugs are reproduced (e.g. `set_adaptive_mesh`'s
//! `min(list_of_xs)` calls, `RapidScanHelper._raise_tool` using
//! `self.scan_height` instead of its parameter).
//!
// Cross-module services are owned elsewhere and imported, never redefined
//! here: `PrinterProbeApi`/`ProbePointsHelper` from `crate::probe`,
//! `ProbeResult` from `crate::manual_probe`; `exclude_object` is reached
//! through `PrinterObject::get_status` on its `ObjectRef`, and the
//! `bed_mesh` back-reference is reached with `lookup_interface` on the
//! `BedMeshApi` trait declared below (implemented by `BedMesh`).

use crate::manual_probe::ProbeResult;
use crate::probe::{PrinterProbeApi, ProbePointsHelper};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatListOpts, GetFloatOpts, GetIntListOpts, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, MoveTransform, RegisterCommandOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{ToolHead, ToolHeadApi};
use klippy_api::{GCell, lookup_interface, ObjectRef, PrinterObject};
use pycompat::json;
use pycompat::text;
use kos::sync::Mutex;
use std::sync::Arc;

pub const PROFILE_VERSION: i64 = 1;

fn cmd_err(msg: String) -> KlippyError {
    KlippyError::command(msg)
}

// Display note: `logging` calls below build the message with the exact
// Python `%` format string first (via `pycompat::text::percent_format` or
// `format!` when literal), then emit through `klippy_api::log`.

/// Finalize callback (`BedMeshCalibrate.probe_finalize`).
pub type ProbeFinalize = Arc<dyn Fn(&[ProbeResult]) -> KResult<()> + Send + Sync + 'static>;

// ---------------------------------------------------------------------------
// Module-level helpers
// ---------------------------------------------------------------------------

/// PEP 485 isclose().
pub fn isclose(a: f64, b: f64) -> bool {
    (a - b).abs() <= (1e-9f64 * a.abs().max(b.abs())).max(0.0)
}

/// `within(coord, min_c, max_c, tol=0.0)`.
pub fn within(coord: (f64, f64), min_c: (f64, f64), max_c: (f64, f64), tol: f64) -> bool {
    (max_c.0 + tol) >= coord.0 && coord.0 >= (min_c.0 - tol)
        && (max_c.1 + tol) >= coord.1 && coord.1 >= (min_c.1 - tol)
}

/// `constrain(val, min_val, max_val)`.
pub fn constrain<T: PartialOrd + Copy>(val: T, min_val: T, max_val: T) -> T {
    if val < min_val { min_val } else if val > max_val { max_val } else { val }
}

/// `lerp(t, v0, v1)`.
pub fn lerp(t: f64, v0: f64, v1: f64) -> f64 {
    (1. - t) * v0 + t * v1
}

/// Retrieve comma separated pair from config.
pub fn parse_config_pair(
    config: &Config,
    option: &str,
    default: i64,
    minval: Option<i64>,
    maxval: Option<i64>,
) -> KResult<(i64, i64)> {
    let raw = config
        .getintlist(option, GetIntListOpts { default: Some(Some(vec![default, default])), ..Default::default() })?
        .unwrap_or(vec![default, default]);
    let pair: Vec<i64> = if raw.len() == 2 {
        raw
    } else if raw.len() == 1 {
        vec![raw[0], raw[0]]
    } else {
        let v = config.get(option, GetStrOpts::default())?.unwrap_or_default();
        return Err(KlippyError::config(format!("bed_mesh: malformed '{option}' value: {v}")));
    };
    if let Some(mv) = minval {
        if pair[0] < mv || pair[1] < mv {
            return Err(KlippyError::config(format!("Option '{option}' in section bed_mesh must have a minimum of {mv}")));
        }
    }
    if let Some(mv) = maxval {
        if pair[0] > mv || pair[1] > mv {
            return Err(KlippyError::config(format!("Option '{option}' in section bed_mesh must have a maximum of {mv}")));
        }
    }
    Ok((pair[0], pair[1]))
}

/// Retrieve comma separated pair from a g-code command.
pub fn parse_gcmd_pair(
    gcmd: &klippy_api::gcode::GCmd,
    name: &str,
    minval: Option<i64>,
    maxval: Option<i64>,
) -> KResult<(i64, i64)> {
    let pair: Vec<i64> = match gcmd_get_str(gcmd, name).map(|s| s.split(',').map(|v| v.trim().to_string()).collect::<Vec<_>>()) {
        Ok(parts) => match parts.iter().map(|p| p.parse::<i64>()).collect::<Result<Vec<i64>, _>>() {
            Ok(v) => v,
            Err(_) => return Err(cmd_err(format!("Unable to parse parameter '{name}'"))),
        },
        Err(_) => return Err(cmd_err(format!("Unable to parse parameter '{name}'"))),
    };
    let pair: Vec<i64> = if pair.len() == 2 {
        pair
    } else if pair.len() == 1 {
        vec![pair[0], pair[0]]
    } else {
        return Err(cmd_err(format!("Unable to parse parameter '{name}'")));
    };
    if let Some(mv) = minval {
        if pair[0] < mv || pair[1] < mv {
            return Err(cmd_err(format!("Parameter '{name}' must have a minimum of {mv}")));
        }
    }
    if let Some(mv) = maxval {
        if pair[0] > mv || pair[1] > mv {
            return Err(cmd_err(format!("Parameter '{name}' must have a maximum of {mv}")));
        }
    }
    Ok((pair[0], pair[1]))
}

/// Retrieve comma separated coordinate from a g-code command.
pub fn parse_gcmd_coord(gcmd: &klippy_api::gcode::GCmd, name: &str) -> KResult<(f64, f64)> {
    let s = gcmd_get_str(gcmd, name).map_err(|_| cmd_err(format!("Unable to parse parameter '{name}'")))?;
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        return Err(cmd_err(format!("Unable to parse parameter '{name}'")));
    }
    let v1 = parts[0].trim().parse::<f64>().map_err(|_| cmd_err(format!("Unable to parse parameter '{name}'")))?;
    let v2 = parts[1].trim().parse::<f64>().map_err(|_| cmd_err(format!("Unable to parse parameter '{name}'")))?;
    Ok((v1, v2))
}

fn gcmd_get_str(gcmd: &klippy_api::gcode::GCmd, name: &str) -> KResult<String> {
    let v = gcmd.get(name, GGetOpts::default())?;
    Ok(match v {
        pycompat::PyValue::Str(s) => s,
        pycompat::PyValue::None => String::new(),
        other => other.str(),
    })
}

// ---------------------------------------------------------------------------
// ZrefMode
// ---------------------------------------------------------------------------

/// `ZrefMode` (bed_mesh.py:322).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZrefMode {
    Disabled = 0,
    InMesh = 1,
    Probe = 2,
}

// ---------------------------------------------------------------------------
// ZMesh
// ---------------------------------------------------------------------------

type Matrix = Vec<Vec<f64>>;

/// Python lists are shared by reference: `probed_matrix` and `mesh_matrix`
/// alias the same list after `direct` sampling (`bed_mesh.py:1478`), so
/// `set_zero_reference` subtracts the offset from that storage twice. The
/// `Arc` models that single storage; interpolated algorithms store their own.
type SharedMatrix = Arc<Mutex<Matrix>>;

/// Stand-in for `id(self)` in `"adaptive-%X" % (id(self),)` (bed_mesh.py:1327).
/// CPython's value is the object's heap address, which no other implementation
/// (nor another CPython run) reproduces, so the parity harness normalizes the
/// whole `adaptive-<hex>` name. What the port keeps is the shape (uppercase hex,
/// no `0x`) and a nonzero value distinct per mesh object.
fn next_adaptive_mesh_id() -> usize {
    static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0x7F00_0010);
    NEXT.fetch_add(0x10, std::sync::atomic::Ordering::Relaxed)
}

/// `ZMesh` (bed_mesh.py:1325).
pub struct ZMesh {
    profile_name: String,
    probed_matrix: GCell<Option<SharedMatrix>>,
    mesh_matrix: GCell<Option<SharedMatrix>>,
    mesh_params: IndexMap<String, pycompat::PyValue>,
    mesh_offsets: GCell<[f64; 2]>,
    mesh_x_min: f64,
    mesh_x_max: f64,
    mesh_y_min: f64,
    mesh_y_max: f64,
    mesh_x_count: usize,
    mesh_y_count: usize,
    x_mult: i64,
    y_mult: i64,
    mesh_x_dist: f64,
    mesh_y_dist: f64,
    algo: String,
    tension: f64,
    x_count: i64,
    y_count: i64,
}

impl ZMesh {
    pub fn new(params: IndexMap<String, pycompat::PyValue>, name: Option<&str>) -> ZMesh {
        let profile_name = match name {
            Some(n) if !n.is_empty() => n.to_string(),
            _ => format!("adaptive-{:X}", next_adaptive_mesh_id()),
        };
        let get_f = |k: &str| -> f64 {
            match params.get(k) {
                Some(pycompat::PyValue::Float(f)) => *f,
                Some(pycompat::PyValue::Int(n)) => n.0.to_string().parse::<f64>().unwrap_or(0.0),
                _ => 0.0,
            }
        };
        let get_i = |k: &str| -> i64 {
            match params.get(k) {
                Some(pycompat::PyValue::Int(n)) => n.0.to_string().parse::<i64>().unwrap_or(0),
                Some(pycompat::PyValue::Float(f)) => *f as i64,
                _ => 0,
            }
        };
        let mesh_x_min = get_f("min_x");
        let mesh_x_max = get_f("max_x");
        let mesh_y_min = get_f("min_y");
        let mesh_y_max = get_f("max_y");
        let algo = match params.get("algo") {
            Some(pycompat::PyValue::Str(s)) => s.clone(),
            _ => "lagrange".to_string(),
        };
        let tension = get_f("tension");
        let x_count = get_i("x_count");
        let y_count = get_i("y_count");
        let mesh_x_pps = get_i("mesh_x_pps");
        let mesh_y_pps = get_i("mesh_y_pps");
        {
            let mut line = String::from("bed_mesh: probe/mesh parameters:");
            klippy_api::log::debug(&line);
            for (k, v) in params.iter() {
                line = format!("{k} :  {}", v.str());
                klippy_api::log::debug(&line);
            }
        }
        klippy_api::log::debug(&format!(
            "bed_mesh: Mesh Min: ({:.2},{:.2}) Mesh Max: ({:.2},{:.2})",
            mesh_x_min, mesh_y_min, mesh_x_max, mesh_y_max
        ));
        let mesh_x_count = ((x_count - 1) * mesh_x_pps + x_count).max(1) as usize;
        let mesh_y_count = ((y_count - 1) * mesh_y_pps + y_count).max(1) as usize;
        let x_mult = mesh_x_pps + 1;
        let y_mult = mesh_y_pps + 1;
        klippy_api::log::debug(&format!("bed_mesh: Mesh grid size - X:{mesh_x_count}, Y:{mesh_y_count}"));
        let mesh_x_dist = (mesh_x_max - mesh_x_min) / (mesh_x_count as f64 - 1.0);
        let mesh_y_dist = (mesh_y_max - mesh_y_min) / (mesh_y_count as f64 - 1.0);
        ZMesh {
            profile_name,
            probed_matrix: GCell::new(None),
            mesh_matrix: GCell::new(None),
            mesh_params: params,
            mesh_offsets: GCell::new([0.0, 0.0]),
            mesh_x_min,
            mesh_x_max,
            mesh_y_min,
            mesh_y_max,
            mesh_x_count,
            mesh_y_count,
            x_mult,
            y_mult,
            mesh_x_dist,
            mesh_y_dist,
            algo,
            tension,
            x_count,
            y_count,
        }
    }

    pub fn get_mesh_matrix(&self) -> Matrix {
        match self.mesh_matrix.borrow().clone() {
            Some(m) => m
                .lock()
                .iter()
                .map(|line| line.iter().map(|z| text::round_py_nd(*z, 6)).collect())
                .collect(),
            None => vec![vec![]],
        }
    }

    pub fn get_probed_matrix(&self) -> Matrix {
        match self.probed_matrix.borrow().clone() {
            Some(m) => m
                .lock()
                .iter()
                .map(|line| line.iter().map(|z| text::round_py_nd(*z, 6)).collect())
                .collect(),
            None => vec![vec![]],
        }
    }

    pub fn get_mesh_params(&self) -> IndexMap<String, pycompat::PyValue> {
        self.mesh_params.clone()
    }

    pub fn get_profile_name(&self) -> String {
        self.profile_name.clone()
    }

    pub fn print_probed_matrix<F: Fn(&str)>(&self, print_func: F) {
        if let Some(m) = self.probed_matrix.borrow().clone() {
            let m = m.lock().clone();
            let mut msg = String::from("Mesh Leveling Probed Z positions:\n");
            for line in &m {
                for x in line {
                    msg.push_str(&format!(" {x:.6}"));
                }
                msg.push('\n');
            }
            print_func(&msg);
        } else {
            print_func("bed_mesh: bed has not been probed");
        }
    }

    pub fn print_mesh<F: Fn(&str)>(&self, print_func: F, move_z: Option<f64>) {
        let mat = self.get_mesh_matrix();
        let offs = self.mesh_offsets.borrow().clone();
        let mut out = format!("Mesh X,Y: {},{}\n", self.mesh_x_count, self.mesh_y_count);
        if let Some(z) = move_z {
            out.push_str(&format!("Search Height: {z:.0}\n"));
        }
        out.push_str(&format!("Mesh Offsets: X={:.4}, Y={:.4}\n", offs[0], offs[1]));
        out.push_str(&format!("Mesh Average: {:.2}\n", self.get_z_average()));
        let range = self.get_z_range();
        out.push_str(&format!("Mesh Range: min={:.4} max={:.4}\n", range.0, range.1));
        out.push_str(&format!("Interpolation Algorithm: {}\n", self.mesh_params.get("algo").map(|v| v.str()).unwrap_or_default()));
        out.push_str("Measured points:\n");
        for y_line in (0..self.mesh_y_count).rev() {
            if let Some(row) = mat.get(y_line) {
                for z in row {
                    out.push_str(&format!("  {z:.6}"));
                }
            }
            out.push('\n');
        }
        print_func(&out);
    }

    pub fn build_mesh(&self, z_matrix: Matrix) -> KResult<()> {
        *self.probed_matrix.borrow_mut() = Some(Arc::new(Mutex::new(z_matrix.clone())));
        match self.algo.as_str() {
            "lagrange" => self.sample_lagrange(&z_matrix),
            "bicubic" => self.sample_bicubic(&z_matrix),
            "direct" => self.sample_direct(&z_matrix),
            _ => {
                return Err(KlippyError::Other {
                    class_name: "TypeError".to_string(),
                    message: "'NoneType' object is not callable".to_string(),
                });
            }
        }
        self.print_mesh(|m| klippy_api::log::debug(m), None);
        Ok(())
    }

    pub fn set_zero_reference(&self, xpos: f64, ypos: f64) {
        let offset = self.calc_z(xpos, ypos);
        klippy_api::log::info(&format!("bed_mesh: setting zero reference at ({:.2}, {:.2}, {:.6})", xpos, ypos, offset));
        if let Some(pm) = self.probed_matrix.borrow().clone() {
            let mut g = pm.lock();
            for row in g.iter_mut() {
                for v in row.iter_mut() {
                    *v -= offset;
                }
            }
        }
        if let Some(mm) = self.mesh_matrix.borrow().clone() {
            let mut g = mm.lock();
            for row in g.iter_mut() {
                for v in row.iter_mut() {
                    *v -= offset;
                }
            }
        }
    }

    pub fn set_mesh_offsets(&self, offsets: [Option<f64>; 2]) {
        let mut mo = self.mesh_offsets.borrow_mut();
        for (i, o) in offsets.iter().enumerate() {
            if let Some(v) = o {
                mo[i] = *v;
            }
        }
    }

    pub fn get_x_coordinate(&self, index: usize) -> f64 {
        self.mesh_x_min + self.mesh_x_dist * index as f64
    }

    pub fn get_y_coordinate(&self, index: usize) -> f64 {
        self.mesh_y_min + self.mesh_y_dist * index as f64
    }

    pub fn calc_z(&self, x: f64, y: f64) -> f64 {
        let offs = self.mesh_offsets.borrow().clone();
        match self.mesh_matrix.borrow().clone() {
            Some(m) => {
                let tbl = m.lock().clone();
                let (tx, xidx) = self.get_linear_index(x + offs[0], 0);
                let (ty, yidx) = self.get_linear_index(y + offs[1], 1);
                let z0 = lerp(tx, tbl[yidx][xidx], tbl[yidx][xidx + 1]);
                let z1 = lerp(tx, tbl[yidx + 1][xidx], tbl[yidx + 1][xidx + 1]);
                lerp(ty, z0, z1)
            }
            None => 0.0,
        }
    }

    pub fn get_z_range(&self) -> (f64, f64) {
        match self.mesh_matrix.borrow().clone() {
            Some(m) => {
                let m = m.lock().clone();
                let mut mn = f64::INFINITY;
                let mut mx = f64::NEG_INFINITY;
                for row in &m {
                    for v in row {
                        if *v < mn {
                            mn = *v;
                        }
                        if *v > mx {
                            mx = *v;
                        }
                    }
                }
                if m.is_empty() {
                    (0.0, 0.0)
                } else {
                    (mn, mx)
                }
            }
            None => (0.0, 0.0),
        }
    }

    pub fn get_z_average(&self) -> f64 {
        match self.mesh_matrix.borrow().clone() {
            Some(m) => {
                let m = m.lock().clone();
                let mut total = 0.0;
                let mut cnt = 0usize;
                for row in &m {
                    for v in row {
                        total += *v;
                    }
                    cnt += row.len();
                }
                text::round_py_nd(total / cnt as f64, 2)
            }
            None => 0.0,
        }
    }

    fn get_linear_index(&self, coord: f64, axis: u8) -> (f64, usize) {
        let (mesh_min, mesh_cnt, mesh_dist) = if axis == 0 {
            (self.mesh_x_min, self.mesh_x_count, self.mesh_x_dist)
        } else {
            (self.mesh_y_min, self.mesh_y_count, self.mesh_y_dist)
        };
        let idx = ((coord - mesh_min) / mesh_dist).floor() as i64;
        let idx = constrain(idx, 0, (mesh_cnt - 2) as i64) as usize;
        let cidx = if axis == 0 { self.get_x_coordinate(idx) } else { self.get_y_coordinate(idx) };
        let t = (coord - cidx) / mesh_dist;
        (constrain(t, 0.0, 1.0), idx)
    }

    fn sample_direct(&self, _z_matrix: &Matrix) {
        *self.mesh_matrix.borrow_mut() = self.probed_matrix.borrow().clone();
    }

    fn sample_lagrange(&self, z_matrix: &Matrix) {
        let x_mult = self.x_mult;
        let y_mult = self.y_mult;
        let mut m = vec![vec![0.0f64; self.mesh_x_count]; self.mesh_y_count];
        for j in 0..self.mesh_y_count {
            for i in 0..self.mesh_x_count {
                if (i as i64) % x_mult == 0 && (j as i64) % y_mult == 0 {
                    m[j][i] = z_matrix[j / y_mult as usize][i / x_mult as usize];
                }
            }
        }
        *self.mesh_matrix.borrow_mut() = Some(Arc::new(Mutex::new(m)));
        // Interpolate X coordinates.
        for i in 0..self.mesh_y_count {
            if (i as i64) % y_mult != 0 {
                continue;
            }
            for j in 0..self.mesh_x_count {
                if (j as i64) % x_mult == 0 {
                    continue;
                }
                let x = self.get_x_coordinate(j);
                let v = self.calc_lagrange_x(i, j, x);
                if let Some(mm) = self.mesh_matrix.borrow().clone() {
                    mm.lock()[i][j] = v;
                }
            }
        }
        // Interpolate Y coordinates.
        for i in 0..self.mesh_x_count {
            for j in 0..self.mesh_y_count {
                if (j as i64) % y_mult == 0 {
                    continue;
                }
                let y = self.get_y_coordinate(j);
                let v = self.calc_lagrange_y(i, j, y);
                if let Some(mm) = self.mesh_matrix.borrow().clone() {
                    mm.lock()[j][i] = v;
                }
            }
        }
    }

    fn get_lagrange_coords(&self) -> (Vec<f64>, Vec<f64>) {
        let mut xpts = Vec::new();
        let mut ypts = Vec::new();
        for i in 0..self.x_count {
            xpts.push(self.get_x_coordinate((i * self.x_mult) as usize));
        }
        for j in 0..self.y_count {
            ypts.push(self.get_y_coordinate((j * self.y_mult) as usize));
        }
        (xpts, ypts)
    }

    fn calc_lagrange_x(&self, vec: usize, _i: usize, c: f64) -> f64 {
        let (xpts, _) = self.get_lagrange_coords();
        self.calc_lagrange(&xpts, c, vec, 0)
    }
    fn calc_lagrange_y(&self, vec: usize, _i: usize, c: f64) -> f64 {
        let (_, ypts) = self.get_lagrange_coords();
        self.calc_lagrange(&ypts, c, vec, 1)
    }

    fn calc_lagrange(&self, lpts: &[f64], c: f64, vec: usize, axis: u8) -> f64 {
        let pt_cnt = lpts.len();
        let mut total = 0.0;
        for i in 0..pt_cnt {
            let mut n = 1.0;
            let mut d = 1.0;
            for j in 0..pt_cnt {
                if j == i {
                    continue;
                }
                n *= c - lpts[j];
                d *= lpts[i] - lpts[j];
            }
            let z = {
                let m = self.mesh_matrix.borrow().clone().unwrap();
                let g = m.lock();
                match g.get(if axis == 0 { vec } else { i * self.y_mult as usize }) {
                    Some(row) => row[if axis == 0 { i * self.x_mult as usize } else { vec }],
                    None => 0.0,
                }
            };
            total += z * n / d;
        }
        total
    }

    fn sample_bicubic(&self, z_matrix: &Matrix) {
        let x_mult = self.x_mult;
        let y_mult = self.y_mult;
        let c = self.tension;
        let mut m = vec![vec![0.0f64; self.mesh_x_count]; self.mesh_y_count];
        for j in 0..self.mesh_y_count {
            for i in 0..self.mesh_x_count {
                if (i as i64) % x_mult == 0 && (j as i64) % y_mult == 0 {
                    m[j][i] = z_matrix[j / y_mult as usize][i / x_mult as usize];
                }
            }
        }
        *self.mesh_matrix.borrow_mut() = Some(Arc::new(Mutex::new(m)));
        for y in 0..self.mesh_y_count {
            if (y as i64) % y_mult != 0 {
                continue;
            }
            for x in 0..self.mesh_x_count {
                if (x as i64) % x_mult == 0 {
                    continue;
                }
                let pts = self.get_x_ctl_pts(x, y);
                let v = cardinal_spline(&pts, c);
                if let Some(mm) = self.mesh_matrix.borrow().clone() {
                    mm.lock()[y][x] = v;
                }
            }
        }
        for x in 0..self.mesh_x_count {
            for y in 0..self.mesh_y_count {
                if (y as i64) % y_mult == 0 {
                    continue;
                }
                let pts = self.get_y_ctl_pts(x, y);
                let v = cardinal_spline(&pts, c);
                if let Some(mm) = self.mesh_matrix.borrow().clone() {
                    mm.lock()[y][x] = v;
                }
            }
        }
    }

    fn get_x_ctl_pts(&self, x: usize, y: usize) -> [f64; 5] {
        let x_mult = self.x_mult;
        let x_row = self.mesh_matrix.borrow().clone().unwrap().lock().clone()[y].clone();
        let last_pt = (self.mesh_x_count as i64 - 1 - x_mult) as usize;
        if x < x_mult as usize {
            let p0 = x_row[0];
            let p1 = x_row[0];
            let p2 = x_row[x_mult as usize];
            let p3 = x_row[(2 * x_mult) as usize];
            let t = x as f64 / x_mult as f64;
            [p0, p1, p2, p3, t]
        } else if x > last_pt {
            let p0 = x_row[last_pt - x_mult as usize];
            let p1 = x_row[last_pt];
            let p2 = x_row[last_pt + x_mult as usize];
            let p3 = x_row[last_pt + x_mult as usize];
            let t = (x - last_pt) as f64 / x_mult as f64;
            [p0, p1, p2, p3, t]
        } else {
            let mut found = false;
            let mut out = [0.0f64; 5];
            let mut i = x_mult as usize;
            while i < last_pt {
                if x > i && x < (i + x_mult as usize) {
                    let p0 = x_row[i - x_mult as usize];
                    let p1 = x_row[i];
                    let p2 = x_row[i + x_mult as usize];
                    let p3 = x_row[i + (2 * x_mult) as usize];
                    let t = (x - i) as f64 / x_mult as f64;
                    out = [p0, p1, p2, p3, t];
                    found = true;
                    break;
                }
                i += x_mult as usize;
            }
            if !found {
                klippy_api::log::info("bed_mesh: Error finding x control points");
            }
            out
        }
    }

    fn get_y_ctl_pts(&self, x: usize, y: usize) -> [f64; 5] {
        let y_mult = self.y_mult;
        let last_pt = (self.mesh_y_count as i64 - 1 - y_mult) as usize;
        let y_col = self.mesh_matrix.borrow().clone().unwrap().lock().clone();
        if y < y_mult as usize {
            let p0 = y_col[0][x];
            let p1 = y_col[0][x];
            let p2 = y_col[y_mult as usize][x];
            let p3 = y_col[(2 * y_mult) as usize][x];
            let t = y as f64 / y_mult as f64;
            [p0, p1, p2, p3, t]
        } else if y > last_pt {
            let p0 = y_col[last_pt - y_mult as usize][x];
            let p1 = y_col[last_pt][x];
            let p2 = y_col[last_pt + y_mult as usize][x];
            let p3 = y_col[last_pt + y_mult as usize][x];
            let t = (y - last_pt) as f64 / y_mult as f64;
            [p0, p1, p2, p3, t]
        } else {
            let mut found = false;
            let mut out = [0.0f64; 5];
            let mut i = y_mult as usize;
            while i < last_pt {
                if y > i && y < (i + y_mult as usize) {
                    let p0 = y_col[i - y_mult as usize][x];
                    let p1 = y_col[i][x];
                    let p2 = y_col[i + y_mult as usize][x];
                    let p3 = y_col[i + (2 * y_mult) as usize][x];
                    let t = (y - i) as f64 / y_mult as f64;
                    out = [p0, p1, p2, p3, t];
                    found = true;
                    break;
                }
                i += y_mult as usize;
            }
            if !found {
                klippy_api::log::info("bed_mesh: Error finding y control points");
            }
            out
        }
    }
}

fn cardinal_spline(p: &[f64; 5], tension: f64) -> f64 {
    let t = p[4];
    let t2 = t * t;
    let t3 = t2 * t;
    let m1 = tension * (p[2] - p[0]);
    let m2 = tension * (p[3] - p[1]);
    let a = p[1] * (2.0 * t3 - 3.0 * t2 + 1.0);
    let b = p[2] * (-2.0 * t3 + 3.0 * t2);
    let c = m1 * (t3 - 2.0 * t2 + t);
    let d = m2 * (t3 - t2);
    a + b + c + d
}

// ---------------------------------------------------------------------------
// MoveSplitter
// ---------------------------------------------------------------------------

const TRAVERSE_COMPLETE_ERROR: &str = "Mesh Leveling: Error splitting move ";

pub struct MoveSplitter {
    split_delta_z: f64,
    move_check_distance: f64,
    z_mesh: GCell<Option<Arc<ZMesh>>>,
    fade_offset: GCell<f64>,
    prev_pos: GCell<Vec<f64>>,
    next_pos: GCell<Vec<f64>>,
    current_pos: GCell<Vec<f64>>,
    z_factor: GCell<f64>,
    z_offset: GCell<f64>,
    traverse_complete: GCell<bool>,
    distance_checked: GCell<f64>,
    total_move_length: GCell<f64>,
    axis_move: GCell<Vec<bool>>,
}

impl MoveSplitter {
    pub fn new(config: &Config) -> KResult<MoveSplitter> {
        let split_delta_z = config
            .getfloat("split_delta_z", GetFloatOpts { default: Some(Some(0.025)), minval: Some(0.01), ..Default::default() })?
            .unwrap();
        let move_check_distance = config
            .getfloat("move_check_distance", GetFloatOpts { default: Some(Some(5.0)), minval: Some(3.0), ..Default::default() })?
            .unwrap();
        Ok(MoveSplitter {
            split_delta_z,
            move_check_distance,
            z_mesh: GCell::new(None),
            fade_offset: GCell::new(0.0),
            prev_pos: GCell::new(vec![]),
            next_pos: GCell::new(vec![]),
            current_pos: GCell::new(vec![]),
            z_factor: GCell::new(0.0),
            z_offset: GCell::new(0.0),
            traverse_complete: GCell::new(false),
            distance_checked: GCell::new(0.0),
            total_move_length: GCell::new(0.0),
            axis_move: GCell::new(vec![]),
        })
    }

    pub fn initialize(&self, mesh: Option<Arc<ZMesh>>, fade_offset: f64) {
        *self.z_mesh.borrow_mut() = mesh;
        self.fade_offset.set(fade_offset);
    }

    pub fn build_move(&self, prev_pos: Vec<f64>, next_pos: Vec<f64>, factor: f64) -> KResult<()> {
        *self.prev_pos.borrow_mut() = prev_pos.clone();
        *self.next_pos.borrow_mut() = next_pos.clone();
        *self.current_pos.borrow_mut() = prev_pos.clone();
        self.z_factor.set(factor);
        let zo = self.calc_z_offset(&prev_pos)?;
        self.z_offset.set(zo);
        self.traverse_complete.set(false);
        self.distance_checked.set(0.0);
        let axes_d: Vec<f64> = prev_pos
            .iter()
            .zip(next_pos.iter())
            .map(|(pp, np)| np - pp)
            .collect();
        let mut sum = 0.0;
        for d in axes_d.iter().take(3) {
            sum += d * d;
        }
        self.total_move_length.set(pycompat::libm::sqrt(sum));
        *self.axis_move.borrow_mut() = axes_d.iter().map(|d| !isclose(*d, 0.0)).collect();
        Ok(())
    }

    fn calc_z_offset(&self, pos: &[f64]) -> KResult<f64> {
        let mesh = self.z_mesh.borrow().clone();
        let z = match mesh {
            Some(m) => m.calc_z(pos[0], pos[1]),
            None => 0.0,
        };
        let offset = self.fade_offset.get();
        let factor = self.z_factor.get();
        Ok(factor * (z - offset) + offset)
    }

    fn set_next_move(&self, distance_from_prev: f64) -> KResult<()> {
        let t = distance_from_prev / self.total_move_length.get();
        if t > 1.0 || t < 0.0 {
            return Err(cmd_err("bed_mesh: Slice distance is negative or greater than entire move length".to_string()));
        }
        let prev = self.prev_pos.borrow().clone();
        let next = self.next_pos.borrow().clone();
        let mut cur = self.current_pos.borrow_mut();
        for i in 0..next.len() {
            if self.axis_move.borrow()[i] {
                cur[i] = lerp(t, prev[i], next[i]);
            }
        }
        Ok(())
    }

    pub fn traverse_done(&self) -> bool {
        self.traverse_complete.get()
    }
    /// Returns the split move, or `None` when traversal is complete.
    pub fn split(&self) -> KResult<Option<Vec<f64>>> {
        if !self.traverse_complete.get() {
            let axis_move = self.axis_move.borrow().clone();
            if axis_move[0] || (axis_move.len() > 1 && axis_move[1]) {
                while (self.distance_checked.get() + self.move_check_distance) < self.total_move_length.get() {
                    self.distance_checked.set(self.distance_checked.get() + self.move_check_distance);
                    self.set_next_move(self.distance_checked.get())?;
                    let next_z = self.calc_z_offset(&self.current_pos.borrow().clone())?;
                    if (next_z - self.z_offset.get()).abs() >= self.split_delta_z {
                        self.z_offset.set(next_z);
                        let mut newpos = self.current_pos.borrow().clone();
                        newpos[2] += self.z_offset.get();
                        return Ok(Some(newpos));
                    }
                }
            }
            *self.current_pos.borrow_mut() = self.next_pos.borrow().clone();
            let zo = self.calc_z_offset(&self.current_pos.borrow().clone())?;
            self.z_offset.set(zo);
            let mut cur = self.current_pos.borrow_mut();
            cur[2] += self.z_offset.get();
            self.traverse_complete.set(true);
            Ok(Some(cur.clone()))
        } else {
            Ok(None)
        }
    }
}

// ---------------------------------------------------------------------------
// ProbeManager
// ---------------------------------------------------------------------------

pub struct ProbeManager {
    printer: Printer,
    cfg_overshoot: f64,
    orig_config: IndexMap<String, pycompat::PyValue>,
    faulty_regions: Vec<((f64, f64), (f64, f64))>,
    overshoot: GCell<f64>,
    zero_ref_pos: GCell<Option<(f64, f64)>>,
    zref_mode: GCell<ZrefMode>,
    base_points: GCell<Vec<(f64, f64)>>,
    substitutes: GCell<IndexMap<usize, Vec<(f64, f64)>>>,
    is_round: bool,
    probe_helper: Arc<ProbePointsHelper>,
    rapid_scan_helper: Arc<RapidScanHelper>,
}

impl ProbeManager {
    pub fn new(config: &Config, orig_config: IndexMap<String, pycompat::PyValue>, finalize: ProbeFinalize) -> KResult<ProbeManager> {
        let printer = config.get_printer();
        let cfg_overshoot = config
            .getfloat("scan_overshoot", GetFloatOpts { default: Some(Some(0.0)), minval: Some(1.0), ints: klippy_api::config::IntArgs::DEFAULT, ..Default::default() })?
            .unwrap_or(0.0);
        let orig_radius = orig_config.get("radius").cloned().unwrap_or(pycompat::PyValue::None);
        let is_round = !matches!(orig_radius, pycompat::PyValue::None);
        let zero_ref_pos = config
            .getfloatlist("zero_reference_position", GetFloatListOpts { default: Some(None), count: Some(2), ..Default::default() })?
            .map(|v| (v[0], v[1]));
        let finalize_cb = finalize.clone();
        let probe_helper = ProbePointsHelper::new(config, Box::new(move |points: &[ProbeResult]| finalize_cb(points).map(|_| pycompat::PyValue::None)), Some(Vec::new()))?;
        probe_helper.use_xy_offsets(true);
        let rapid_scan_helper = Arc::new(RapidScanHelper::new(config, finalize.clone())?);
        let mut pm = ProbeManager {
            printer,
            cfg_overshoot,
            orig_config,
            faulty_regions: Vec::new(),
            overshoot: GCell::new(cfg_overshoot),
            zero_ref_pos: GCell::new(zero_ref_pos),
            zref_mode: GCell::new(ZrefMode::Disabled),
            base_points: GCell::new(Vec::new()),
            substitutes: GCell::new(IndexMap::new()),
            is_round,
            probe_helper,
            rapid_scan_helper,
        };
        pm.init_faulty_regions(config)?;
        Ok(pm)
    }

    fn init_faulty_regions(&mut self, config: &Config) -> KResult<()> {
        for i in 1..100 {
            let start = config
                .getfloatlist(&format!("faulty_region_{i}_min"), GetFloatListOpts { default: Some(None), count: Some(2), ..Default::default() })?;
            if start.is_none() {
                break;
            }
            let start = start.unwrap();
            let end = config
                .getfloatlist(&format!("faulty_region_{i}_max"), GetFloatListOpts { count: Some(2), ..Default::default() })?
                .unwrap();
            let c1 = (start[0].min(end[0]), start[1].min(end[1]));
            let c3 = (start[0].max(end[0]), start[1].max(end[1]));
            let c2 = (c1.0, c3.1);
            let c4 = (c3.0, c1.1);
            for (j, (prev_c1, prev_c3)) in self.faulty_regions.iter().enumerate() {
                let prev_c2 = (prev_c1.0, prev_c3.1);
                let prev_c4 = (prev_c3.0, prev_c1.1);
                for coord in [*prev_c1, prev_c2, *prev_c3, prev_c4] {
                    if within(coord, c1, c3, 0.0) {
                        return Err(KlippyError::config(format!(
                            "bed_mesh: Existing faulty_region_{} {:?} overlaps added faulty_region_{} {:?}",
                            j + 1,
                            [prev_c1, prev_c3],
                            i,
                            [c1, c3]
                        )));
                    }
                }
                for coord in [c1, c2, c3, c4] {
                    if within(coord, *prev_c1, *prev_c3, 0.0) {
                        return Err(KlippyError::config(format!(
                            "bed_mesh: Added faulty_region_{} {:?} overlaps existing faulty_region_{} {:?}",
                            i,
                            [c1, c3],
                            j + 1,
                            [prev_c1, prev_c3]
                        )));
                    }
                }
            }
            self.faulty_regions.push((c1, c3));
        }
        Ok(())
    }

    pub fn start_probe(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let method = gcmd
            .get("METHOD", GGetOpts { default: Some(pycompat::PyValue::Str("automatic".into())), ..Default::default() })
            .unwrap_or(pycompat::PyValue::Str("automatic".into()))
            .str()
            .to_lowercase();
        let praw = lookup_probe_raw(&self.printer)?;
        let mut can_scan = false;
        if let Some(o) = &praw {
            let eventtime = self.printer.get_reactor().monotonic();
            let name = match o.get_status(eventtime)?.unwrap_or(pycompat::PyValue::None) {
                pycompat::PyValue::Dict(d) => match d.get(&pycompat::PyValue::Str("name".into())) {
                    Some(pycompat::PyValue::Str(s)) => s.clone(),
                    _ => String::new(),
                },
                _ => String::new(),
            };
            can_scan = name.starts_with("probe_eddy_current");
        }
        if method == "rapid_scan" && can_scan {
            let path = self.iter_rapid_path();
            return self.rapid_scan_helper.perform_rapid_scan(gcmd, path);
        }
        self.probe_helper.start_probe(gcmd)
    }

    pub fn get_zero_ref_pos(&self) -> Option<(f64, f64)> {
        self.zero_ref_pos.borrow().clone()
    }

    pub fn get_zero_ref_mode(&self) -> ZrefMode {
        self.zref_mode.get()
    }

    pub fn get_substitutes(&self) -> IndexMap<usize, Vec<(f64, f64)>> {
        self.substitutes.borrow().clone()
    }

    pub fn generate_points(
        &self,
        mesh_config: &IndexMap<String, pycompat::PyValue>,
        mesh_min: (f64, f64),
        mesh_max: (f64, f64),
        radius: Option<f64>,
        origin: Option<(f64, f64)>,
        probe_method: &str,
    ) -> KResult<()> {
        let x_cnt = mesh_config.get("x_count").map(|v| v.str().parse::<usize>().unwrap_or(3)).unwrap_or(3);
        let y_cnt = mesh_config.get("y_count").map(|v| v.str().parse::<usize>().unwrap_or(3)).unwrap_or(3);
        let (min_x, min_y) = mesh_min;
        let (max_x, max_y) = mesh_max;
        let mut x_dist = (max_x - min_x) / (x_cnt as f64 - 1.0);
        let mut y_dist = (max_y - min_y) / (y_cnt as f64 - 1.0);
        x_dist = (x_dist * 100.0).floor() / 100.0;
        y_dist = (y_dist * 100.0).floor() / 100.0;
        if x_dist < 1.0 || y_dist < 1.0 {
            return Err(KlippyError::Other {
                class_name: "extras.bed_mesh.BedMeshError".to_string(),
                message: "bed_mesh: min/max points too close together".to_string(),
            });
        }
        #[allow(unused_assignments)]
        let (mut nx_min, mut nx_max, mut nx_min_y, mut nx_max_y) = (min_x, max_x, min_y, max_y);
        let mut points: Vec<(f64, f64)> = Vec::new();
        if let Some(r) = radius {
            let _ = r;
            let y_dist_r = x_dist;
            let new_r = (x_cnt / 2) as f64 * x_dist;
            let (rmin_x, rmax_x, rmin_y, rmax_y) = (-new_r, new_r, -new_r, new_r);
            nx_min = rmin_x;
            nx_max = rmax_x;
            nx_min_y = rmin_y;
            nx_max_y = rmax_y;
            let mut pos_y = rmin_y;
            for i in 0..y_cnt {
                for j in 0..x_cnt {
                    let pos_x = if i % 2 == 0 { rmin_x + j as f64 * x_dist } else { rmax_x - j as f64 * x_dist };
                    let dist = (pos_x * pos_x + pos_y * pos_y).sqrt();
                    let _ = y_dist_r;
                    if dist <= r {
                        points.push((origin.unwrap().0 + pos_x, origin.unwrap().1 + pos_y));
                    }
                }
                pos_y += y_dist_r;
            }
        } else {
            let new_max_x = min_x + x_dist * (x_cnt as f64 - 1.0);
            nx_max = new_max_x;
            let mut pos_y = min_y;
            for i in 0..y_cnt {
                for j in 0..x_cnt {
                    let pos_x = if i % 2 == 0 { min_x + j as f64 * x_dist } else { new_max_x - j as f64 * x_dist };
                    points.push((pos_x, pos_y));
                }
                pos_y += y_dist;
            }
        }
        if self.zero_ref_pos.borrow().is_none() || probe_method == "manual" {
            self.zref_mode.set(ZrefMode::Disabled);
        } else if within(self.zero_ref_pos.borrow().unwrap(), mesh_min, mesh_max, 0.0) {
            self.zref_mode.set(ZrefMode::InMesh);
        } else {
            self.zref_mode.set(ZrefMode::Probe);
        }
        *self.base_points.borrow_mut() = points;
        self.substitutes.borrow_mut().clear();
        let og_min_x = match self.orig_config.get("mesh_min") {
            Some(pycompat::PyValue::Tuple(t)) => {
                // tuple of 2 floats
                match (&t[0], &t[1]) {
                    (pycompat::PyValue::Float(a), pycompat::PyValue::Float(_b)) => *a,
                    _ => 0.0,
                }
            }
            _ => 0.0,
        };
        let og_max_x = match self.orig_config.get("mesh_max") {
            Some(pycompat::PyValue::Tuple(t)) => match (&t[0], &t[1]) {
                (pycompat::PyValue::Float(a), pycompat::PyValue::Float(_)) => *a,
                _ => 0.0,
            },
            _ => 0.0,
        };
        let add_ovs = (nx_min - og_min_x).max(0.0).min((og_max_x - nx_max).max(0.0));
        self.overshoot.set(self.cfg_overshoot + add_ovs.floor());
        let (min_pt, max_pt) = ((nx_min, nx_min_y), (nx_max, nx_max_y));
        self.process_faulty_regions(min_pt, max_pt, radius)?;
        let std_path: Vec<Vec<f64>> = self.get_std_path().into_iter().map(|p| vec![p.0, p.1]).collect();
        self.probe_helper.update_probe_points(std_path, 3)
    }

    fn process_faulty_regions(&self, min_pt: (f64, f64), max_pt: (f64, f64), radius: Option<f64>) -> KResult<()> {
        if self.faulty_regions.is_empty() {
            return Ok(());
        }
        if self.zref_mode.get() == ZrefMode::Probe {
            let zrp = self.zero_ref_pos.borrow().unwrap();
            for (min_c, max_c) in self.faulty_regions.clone() {
                if within(zrp, min_c, max_c, 0.0) {
                    return Err(KlippyError::Other {
                        class_name: "extras.bed_mesh.BedMeshError".to_string(),
                        message: format!(
                            "bed_mesh: Cannot probe zero reference position at ({:.2}, {:.2}) as it is located within a faulty region. Check the value for option 'zero_reference_position'",
                            zrp.0, zrp.1
                        ),
                    });
                }
            }
        }
        let base = self.base_points.borrow().clone();
        let mut last_y = base[0].1;
        let mut is_reversed = false;
        for (i, coord) in base.iter().enumerate() {
            if !isclose(coord.1, last_y) {
                is_reversed = !is_reversed;
            }
            last_y = coord.1;
            let mut adj_coords: Vec<(f64, f64)> = Vec::new();
            for (min_c, max_c) in self.faulty_regions.clone() {
                if within(*coord, min_c, max_c, 0.00001) {
                    adj_coords = vec![
                        (min_c.0, coord.1),
                        (coord.0, min_c.1),
                        (coord.0, max_c.1),
                        (max_c.0, coord.1),
                    ];
                    if is_reversed {
                        let first = adj_coords[0];
                        let last = adj_coords[adj_coords.len() - 1];
                        adj_coords[0] = last;
                        let n = adj_coords.len() - 1;
                        adj_coords[n] = first;
                    }
                    break;
                }
            }
            if adj_coords.is_empty() {
                continue;
            }
            let mut valid_coords = Vec::new();
            for ac in adj_coords {
                if radius.is_none() {
                    if within(ac, min_pt, max_pt, 0.000001) {
                        valid_coords.push(ac);
                    }
                } else {
                    let r = radius.unwrap();
                    let dist = (ac.0 * ac.0 + ac.1 * ac.1).sqrt();
                    if dist <= r {
                        valid_coords.push(ac);
                    }
                }
            }
            if valid_coords.is_empty() {
                return Err(KlippyError::Other {
                    class_name: "extras.bed_mesh.BedMeshError".to_string(),
                    message: format!("bed_mesh: Unable to generate coordinates for faulty region at index: {i}"),
                });
            }
            self.substitutes.borrow_mut().insert(i, valid_coords);
        }
        Ok(())
    }

    pub fn get_base_points(&self) -> Vec<(f64, f64)> {
        self.base_points.borrow().clone()
    }

    pub fn get_std_path(&self) -> Vec<(f64, f64)> {
        let base = self.base_points.borrow().clone();
        let subs = self.substitutes.borrow().clone();
        let mut path = Vec::new();
        for (idx, pt) in base.iter().enumerate() {
            if let Some(sp) = subs.get(&idx) {
                path.extend(sp.iter().cloned());
            } else {
                path.push(*pt);
            }
        }
        if self.zref_mode.get() == ZrefMode::Probe {
            path.push(self.zero_ref_pos.borrow().unwrap());
        }
        path
    }

    pub fn iter_rapid_path(&self) -> Vec<((f64, f64), bool)> {
        let mut out = Vec::new();
        let base = self.base_points.borrow().clone();
        let subs = self.substitutes.borrow().clone();
        let mut ascnd_x = true;
        let mut last_base_pt = base[0];
        let mut last_mv_pt = base[0];
        if self.overshoot.get() != 0.0 {
            let overshoot = self.overshoot.get().min(8.0);
            last_mv_pt = (last_base_pt.0 - overshoot, last_base_pt.1);
            out.push((last_mv_pt, false));
        }
        for (idx, pt) in base.iter().enumerate() {
            let dir_change = !isclose_tol(pt.1, last_base_pt.1, 1e-6);
            if subs.contains_key(&idx) {
                let fp = self.gen_faulty_path(last_mv_pt, idx, ascnd_x, dir_change);
                for (sub_pt, is_smp) in fp {
                    last_mv_pt = sub_pt;
                    out.push((sub_pt, is_smp));
                }
            } else {
                if dir_change {
                    for dpt in self.gen_dir_change(last_mv_pt, *pt, ascnd_x) {
                        out.push((dpt, false));
                    }
                }
                out.push((*pt, true));
                last_mv_pt = *pt;
            }
            last_base_pt = *pt;
            if dir_change {
                ascnd_x = !ascnd_x;
            }
        }
        if self.zref_mode.get() == ZrefMode::Probe {
            if self.overshoot.get() != 0.0 {
                let mut ovs = self.overshoot.get().min(4.0);
                if !ascnd_x {
                    ovs = -ovs;
                }
                out.push(((last_mv_pt.0 + ovs, last_mv_pt.1), false));
            }
            out.push((self.zero_ref_pos.borrow().unwrap(), true));
        }
        out
    }

    fn gen_faulty_path(&self, last_pt: (f64, f64), idx: usize, ascnd_x: bool, dir_change: bool) -> Vec<((f64, f64), bool)> {
        let subs = self.substitutes.borrow().get(&idx).cloned().unwrap_or_default();
        let sub_cnt = subs.len();
        let mut out: Vec<((f64, f64), bool)> = Vec::new();
        if dir_change {
            for dpt in self.gen_dir_change(last_pt, subs[0], ascnd_x) {
                out.push((dpt, false));
            }
        }
        if self.is_round {
            for pt in subs {
                out.push((pt, true));
            }
            return out;
        }
        let mut sorted_sub_idx: Vec<usize> = self.substitutes.borrow().keys().cloned().collect();
        sorted_sub_idx.sort_unstable();
        let first_corner = sub_cnt == 2 && idx < sorted_sub_idx.len() && sorted_sub_idx[idx] == idx;
        out.push((subs[0], true));
        if sub_cnt == 1 {
            return out;
        }
        let mut last = subs[0];
        let mut next = subs[1];
        if sub_cnt == 2 {
            if first_corner || dir_change {
                out.push(((next.0, last.1), false));
            } else {
                out.push(((last.0, next.1), false));
            }
            out.push((next, true));
        } else if sub_cnt >= 3 {
            if dir_change {
                if self.overshoot.get() != 0.0 {
                    for dpt in self.gen_dir_change(last, next, ascnd_x) {
                        out.push((dpt, false));
                    }
                } else {
                    let shift = if ascnd_x { -2.0 } else { 2.0 };
                    out.push(((last.0 + shift, last.1), false));
                    out.push(((last.0 + shift, next.1), false));
                }
                out.push((next, true));
                last = subs[1];
                next = subs[2];
            } else {
                out.push(((last.0, next.1), false));
                out.push((next, true));
                last = subs[1];
                next = subs[2];
                if sub_cnt == 4 {
                    let shift = if ascnd_x { 2.0 } else { -2.0 };
                    out.push(((last.0 + shift, last.1), false));
                    out.push(((next.0 - shift, next.1), false));
                    out.push((next, true));
                    last = subs[2];
                    next = subs[3];
                }
            }
            out.push(((next.0, last.1), false));
            out.push((next, true));
        }
        out
    }

    fn gen_dir_change(&self, last_pt: (f64, f64), next_pt: (f64, f64), ascnd_x: bool) -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        if self.overshoot.get() == 0.0 {
            return out;
        }
        let xdir = if ascnd_x { 1.0 } else { -1.0 };
        let overshoot = if self.overshoot.get() >= 3.0 { 2.0 } else { self.overshoot.get() };
        let xdir_pt = overshoot * xdir;
        let ovr_pt = (last_pt.0 + xdir_pt, last_pt.1);
        out.push(ovr_pt);
        if self.overshoot.get() < 3.0 {
            out.push((next_pt.0 + xdir_pt, next_pt.1));
        } else {
            let step_angle = 3;
            let start_angle = 270;
            let ydiff = (next_pt.1 - last_pt.1).abs();
            let xdiff = (next_pt.0 - last_pt.0).abs();
            let max_radius = (self.overshoot.get() - 2.0).min(8.0);
            let radius = (ydiff / 2.0).min(max_radius);
            let origin = [ovr_pt.0, last_pt.1 + radius];
            let next_origin_y = next_pt.1 - radius;
            let mut angle: i64;
            if xdiff < 0.01 {
                angle = 90;
                if next_origin_y - origin[1] < 0.05 {
                    angle = 180;
                }
            } else {
                angle = pycompat::libm::atan(ydiff / xdiff).to_degrees() as i64;
                if (ascnd_x && next_pt.0 < last_pt.0) || (!ascnd_x && next_pt.0 > last_pt.0) {
                    angle = 180 - angle;
                }
            }
            let mut count = (angle / step_angle) as i64;
            let step = step_angle * if ascnd_x { 1 } else { -1 };
            let start = start_angle + step;
            for arc_pt in self.gen_arc(origin, radius, start, step, count) {
                out.push(arc_pt);
            }
            if angle == 180 {
                return out;
            }
            let origin2 = [next_pt.0 + xdir_pt, next_origin_y];
            let start2 = start_angle + count * step;
            count = 61 - count;
            for arc_pt in self.gen_arc(origin2, radius, start2, step, count) {
                out.push(arc_pt);
            }
        }
        out
    }

    fn gen_arc(&self, origin: [f64; 2], radius: f64, start: i64, step: i64, count: i64) -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        let mut angle = start;
        let end = start + step * count;
        while angle < end {
            let rad = ((angle % 360) as f64).to_radians();
            let opp = pycompat::libm::sin(rad) * radius;
            let adj = pycompat::libm::cos(rad) * radius;
            out.push((origin[0] + adj, origin[1] + opp));
            angle += step;
        }
        out
    }

}

fn isclose_tol(a: f64, b: f64, abs_tol: f64) -> bool {
    (a - b).abs() <= (1e-9f64 * a.abs().max(b.abs())).max(abs_tol)
}
fn lookup_probe(printer: &Printer) -> KResult<Option<Arc<dyn PrinterProbeApi>>> {
    lookup_interface::<dyn PrinterProbeApi>(&**printer, "probe", LookupDefault::ReturnNone)
}

fn lookup_probe_raw(printer: &Printer) -> KResult<Option<ObjectRef>> {
    printer.lookup_object("probe", LookupDefault::ReturnNone)
}

// ---------------------------------------------------------------------------
// RapidScanHelper
// ---------------------------------------------------------------------------

const MAX_HIT_DIST: f64 = 2.0;
const MM_WIN_SPEED: f64 = 125.0;

pub struct RapidScanHelper {
    printer: Printer,
    speed: f64,
    scan_height: f64,
    finalize: ProbeFinalize,
}

impl RapidScanHelper {
    pub fn new(config: &Config, finalize: ProbeFinalize) -> KResult<RapidScanHelper> {
        let printer = config.get_printer();
        let speed = config
            .getfloat("speed", GetFloatOpts { default: Some(Some(50.0)), above: Some(0.0), ..Default::default() })?
            .unwrap();
        let scan_height = config
            .getfloat("horizontal_move_z", GetFloatOpts { default: Some(Some(5.0)), ..Default::default() })?
            .unwrap();
        Ok(RapidScanHelper { printer, speed, scan_height, finalize })
    }

    pub fn perform_rapid_scan(&self, gcmd: &klippy_api::gcode::GCmd, path: Vec<((f64, f64), bool)>) -> KResult<()> {
        let speed = gcmd
            .get_float("SCAN_SPEED", GGetFloatOpts { default: Some(Some(self.speed)), ..Default::default() })?.unwrap_or(self.speed);
        let scan_height = gcmd
            .get_float("HORIZONTAL_MOVE_Z", GGetFloatOpts { default: Some(Some(self.scan_height)), ..Default::default() })?.unwrap_or(self.scan_height);
        gcmd.respond_info(&format!("Beginning rapid surface scan at height {scan_height:.2}..."), true);
        let pprobe = lookup_interface::<dyn PrinterProbeApi>(&*self.printer, "probe", LookupDefault::Raise)?.expect("probe exists");
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?.expect("toolhead exists");
        let scale = (0.0f64.max(1.0 - speed / MM_WIN_SPEED)) + 1.0;
        let hit_dist = MAX_HIT_DIST.min(scale * speed / MM_WIN_SPEED);
        let half_window = hit_dist / speed;
        gcmd.respond_info(&format!("Sample hit distance +/- {hit_dist:.4}mm, time window +/- ms {:.4}", half_window * 1000.0), true);
        let session = pprobe.start_probe_session(gcmd)?;
        let offsets = pprobe.get_offsets(Some(gcmd))?;
        let _ = self.raise_tool(gcmd, scan_height, &toolhead, &pprobe)?;
        let mut initial_move = true;
        for (pos, is_probe_pt) in path {
            let adj = self.apply_offsets(&pos, &[offsets.0, offsets.1]);
            toolhead.manual_move(vec![Some(adj[0]), Some(adj[1]), None], speed)?;
            if initial_move {
                initial_move = false;
                self.move_to_scan_height(gcmd, scan_height, &toolhead, &pprobe)?;
            }
            if is_probe_pt {
                session.run_probe(gcmd)?;
            }
        }
        let results = session.pull_probed_results()?;
        let _ = toolhead.get_last_move_time()?;
        (self.finalize)(&results)?;
        session.end_probe_session()?;
        Ok(())
    }

    fn raise_tool(&self, gcmd: &klippy_api::gcode::GCmd, scan_height: f64, toolhead: &ToolHead, pprobe: &Arc<dyn PrinterProbeApi>) -> KResult<()> {
        let cur_pos = toolhead.get_position();
        if cur_pos[2] >= scan_height {
            return Ok(());
        }
        let pparams = pprobe.get_probe_params(Some(gcmd))?;
        let lift_speed = pparams.lift_speed;
        let mut pos = cur_pos.clone();
        pos[2] = self.scan_height + 0.5;
        toolhead.manual_move(vec![Some(pos[0]), Some(pos[1]), Some(pos[2])], lift_speed)?;
        Ok(())
    }

    fn move_to_scan_height(&self, gcmd: &klippy_api::gcode::GCmd, scan_height: f64, toolhead: &ToolHead, pprobe: &Arc<dyn PrinterProbeApi>) -> KResult<()> {
        let time_window = gcmd.get_float("SAMPLE_TIME", GGetFloatOpts::default()).unwrap_or(None).unwrap_or(0.0);
        let cur_pos = toolhead.get_position();
        let pparams = pprobe.get_probe_params(Some(gcmd))?;
        let lift_speed = pparams.lift_speed;
        let probe_speed = pparams.probe_speed;
        let mut pos = cur_pos.clone();
        pos[2] = scan_height + 0.5;
        toolhead.manual_move(vec![Some(pos[0]), Some(pos[1]), Some(pos[2])], lift_speed)?;
        let mut pos2 = pos.clone();
        pos2[2] = scan_height;
        toolhead.manual_move(vec![Some(pos2[0]), Some(pos2[1]), Some(pos2[2])], probe_speed)?;
        toolhead.dwell(time_window / 2.0 + 0.01)?;
        Ok(())
    }

    fn apply_offsets(&self, point: &(f64, f64), offsets: &[f64]) -> Vec<f64> {
        vec![point.0 - offsets[0], point.1 - offsets[1]]
    }
}

// ---------------------------------------------------------------------------
// BedMeshCalibrate
// ---------------------------------------------------------------------------

pub struct BedMeshCalibrate {
    printer: Printer,
    orig_config: GCell<IndexMap<String, pycompat::PyValue>>,
    radius: GCell<Option<f64>>,
    origin: GCell<Option<(f64, f64)>>,
    mesh_min: GCell<(f64, f64)>,
    mesh_max: GCell<(f64, f64)>,
    adaptive_margin: f64,
    mesh_config: GCell<IndexMap<String, pycompat::PyValue>>,
    probe_mgr: ProbeManager,
    profile_name: GCell<Option<String>>,
    gcode: klippy_api::gcode::GCode,
}

impl BedMeshCalibrate {
    pub fn new(config: &Config) -> KResult<BedMeshCalibrate> {
        let printer = config.get_printer();
        let mut orig: IndexMap<String, pycompat::PyValue> = IndexMap::new();
        orig.insert("radius".into(), pycompat::PyValue::None);
        orig.insert("origin".into(), pycompat::PyValue::None);
        let adaptive_margin = config
            .getfloat("adaptive_margin", GetFloatOpts { default: Some(Some(0.0)), ..Default::default() })?
            .unwrap();
        let printer_for_finalize = printer.clone();
        let finalize: ProbeFinalize = Arc::new(move |points: &[ProbeResult]| {
            let bm = lookup_interface::<dyn BedMeshApi>(&*printer_for_finalize, "bed_mesh", LookupDefault::Raise)?.expect("bed_mesh exists");
            bm.probe_finalize(points)
        });
        let gcode = lookup_gcode(&printer)?;
        let finalize2 = finalize.clone();
        let mut bmc = BedMeshCalibrate {
            printer: printer.clone(),
            orig_config: GCell::new(orig),
            radius: GCell::new(None),
            origin: GCell::new(None),
            mesh_min: GCell::new((0.0, 0.0)),
            mesh_max: GCell::new((0.0, 0.0)),
            adaptive_margin,
            mesh_config: GCell::new(IndexMap::new()),
            probe_mgr: ProbeManager::new(config, IndexMap::new(), finalize)?,
            profile_name: GCell::new(Some("default".to_string())),
            gcode,
        };
        bmc.init_mesh_config(config)?;
        // Python builds `ProbeManager` after `_init_mesh_config`, so its
        // `orig_config` (radius/origin/mesh bounds) and `is_round` follow
        // the filled values. Rebuild it with the filled snapshot.
        bmc.probe_mgr = ProbeManager::new(config, bmc.orig_config.borrow().clone(), finalize2)?;
        let mc = bmc.mesh_config.borrow().clone();
        if let Err(e) = bmc.probe_mgr.generate_points(&mc, bmc.mesh_min.borrow().clone(), bmc.mesh_max.borrow().clone(), bmc.radius.borrow().clone(), bmc.origin.borrow().clone(), "automatic") {
            if e.class_name() == "BedMeshError" {
                return Err(KlippyError::config(e.message().to_string()));
            }
            return Err(e);
        }
        Ok(bmc)
    }

    fn bedmesh(&self) -> KResult<Arc<dyn BedMeshApi>> {
        Ok(lookup_interface::<dyn BedMeshApi>(&*self.printer, "bed_mesh", LookupDefault::Raise)?.expect("bed_mesh exists"))
    }

    fn init_mesh_config(&mut self, config: &Config) -> KResult<()> {
        let mut mesh_cfg: IndexMap<String, pycompat::PyValue> = IndexMap::new();
        let radius_opt = config
            .getfloat("mesh_radius", GetFloatOpts { default: Some(None), above: Some(0.0), ..Default::default() })?;
        let (x_cnt, y_cnt, min_x, min_y, max_x, max_y);
        if let Some(mut radius) = radius_opt {
            let origin = config
                .getfloatlist("mesh_origin", GetFloatListOpts { default: Some(Some(vec![0.0, 0.0])), count: Some(2), ..Default::default() })?
                .unwrap();
            let origin = (origin[0], origin[1]);
            let x_cnt_r = config
                .getint("round_probe_count", GetIntOpts { default: Some(Some(5)), minval: Some(3), ..Default::default() })?
                .unwrap();
            if x_cnt_r % 2 == 0 {
                return Err(KlippyError::config("bed_mesh: probe_count must be odd for round beds"));
            }
            radius = (radius * 10.0).floor() / 10.0;
            self.orig_config.borrow_mut().insert("radius".into(), pycompat::PyValue::Float(radius));
            self.orig_config.borrow_mut().insert("origin".into(), pycompat::PyValue::Tuple(vec![pycompat::PyValue::Float(origin.0), pycompat::PyValue::Float(origin.1)]));
            x_cnt = x_cnt_r;
            y_cnt = x_cnt_r;
            min_x = -radius;
            min_y = -radius;
            max_x = radius;
            max_y = radius;
            self.radius.set(Some(radius));
            self.origin.set(Some(origin));
        } else {
            let (xc, yc) = parse_config_pair(config, "probe_count", 3, Some(3), None)?;
            x_cnt = xc;
            y_cnt = yc;
            let mm = config
                .getfloatlist("mesh_min", GetFloatListOpts { count: Some(2), ..Default::default() })?
                .unwrap_or(vec![0.0, 0.0]);
            let mx = config
                .getfloatlist("mesh_max", GetFloatListOpts { count: Some(2), ..Default::default() })?
                .unwrap_or(vec![0.0, 0.0]);
            min_x = mm[0];
            min_y = mm[1];
            max_x = mx[0];
            max_y = mx[1];
            if max_x <= min_x || max_y <= min_y {
                return Err(KlippyError::config("bed_mesh: invalid min/max points"));
            }
        }
        self.orig_config.borrow_mut().insert("x_count".into(), pycompat::PyValue::Int(x_cnt.into()));
        self.orig_config.borrow_mut().insert("y_count".into(), pycompat::PyValue::Int(y_cnt.into()));
        mesh_cfg.insert("x_count".into(), pycompat::PyValue::Int(x_cnt.into()));
        mesh_cfg.insert("y_count".into(), pycompat::PyValue::Int(y_cnt.into()));
        let min_t = pycompat::PyValue::Tuple(vec![pycompat::PyValue::Float(min_x), pycompat::PyValue::Float(min_y)]);
        let max_t = pycompat::PyValue::Tuple(vec![pycompat::PyValue::Float(max_x), pycompat::PyValue::Float(max_y)]);
        self.orig_config.borrow_mut().insert("mesh_min".into(), min_t.clone());
        self.orig_config.borrow_mut().insert("mesh_max".into(), max_t.clone());
        self.mesh_min.set((min_x, min_y));
        self.mesh_max.set((max_x, max_y));
        let (xpps, ypps) = parse_config_pair(config, "mesh_pps", 2, Some(0), None)?;
        self.orig_config.borrow_mut().insert("mesh_x_pps".into(), pycompat::PyValue::Int(xpps.into()));
        self.orig_config.borrow_mut().insert("mesh_y_pps".into(), pycompat::PyValue::Int(ypps.into()));
        mesh_cfg.insert("mesh_x_pps".into(), pycompat::PyValue::Int(xpps.into()));
        mesh_cfg.insert("mesh_y_pps".into(), pycompat::PyValue::Int(ypps.into()));
        let algo = config.get("algorithm", GetStrOpts { default: Some(Some("lagrange".into())), ..Default::default() })?
            .unwrap()
            .trim()
            .to_lowercase();
        let tension = config
            .getfloat("bicubic_tension", GetFloatOpts { default: Some(Some(0.2)), minval: Some(0.0), maxval: Some(2.0), ..Default::default() })?
            .unwrap();
        self.orig_config.borrow_mut().insert("algo".into(), pycompat::PyValue::Str(algo.clone()));
        self.orig_config.borrow_mut().insert("tension".into(), pycompat::PyValue::Float(tension));
        mesh_cfg.insert("algo".into(), pycompat::PyValue::Str(algo));
        mesh_cfg.insert("tension".into(), pycompat::PyValue::Float(tension));
        *self.mesh_config.borrow_mut() = mesh_cfg;
        let mc = self.mesh_config.borrow().clone();
        let mut mc2 = mc.clone();
        verify_algo(&mut mc2, &|m| KlippyError::config(m))?;
        *self.mesh_config.borrow_mut() = mc2;
        Ok(())
    }

    pub fn print_generated_points<F: Fn(&str)>(&self, print_func: &F, truncate: bool) {
        let mut x_offset = 0.0;
        let mut y_offset = 0.0;
        if let Ok(Some(p)) = lookup_probe(&self.printer) {
            if let Ok(offs) = p.get_offsets(None) {
                x_offset = offs.0;
                y_offset = offs.1;
            }
        }
        print_func("bed_mesh: generated points\nIndex |  Tool Adjusted  |   Probe");
        let points = self.probe_mgr.get_base_points();
        for (i, (x, y)) in points.iter().enumerate() {
            if i >= 50 && truncate {
                print_func(&format!("...points {} through {} truncated", i, points.len() - 1));
                break;
            }
            let adj_pt = format!("({:.1}, {:.1})", x - x_offset, y - y_offset);
            let mesh_pt = format!("({:.1}, {:.1})", x, y);
            print_func(&format!("  {:<4}| {:<16}| {}", i, adj_pt, mesh_pt));
        }
        if let Some(zrp) = self.probe_mgr.get_zero_ref_pos() {
            print_func(&format!("bed_mesh: zero_reference_position is ({:.2}, {:.2})", zrp.0, zrp.1));
        }
        let subs = self.probe_mgr.get_substitutes();
        if !subs.is_empty() {
            print_func("bed_mesh: faulty region points");
            for (i, v) in subs.iter() {
                let pt = points[*i];
                print_func(&format!("{} ({:.2}, {:.2}), substituted points: {}", i, pt.0, pt.1, py_repr_list(v)));
            }
        }
    }

    /// `self._verify_algorithm(gcmd.error)` (bed_mesh.py:620): failures are command errors.
    fn verify_algorithm(&self) -> KResult<()> {
        let mut mc = self.mesh_config.borrow().clone();
        verify_algo(&mut mc, &|m| KlippyError::command(m))?;
        *self.mesh_config.borrow_mut() = mc;
        Ok(())
    }
}
impl BedMeshCalibrate {
    fn set_adaptive_mesh(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<bool> {
        let adaptive = gcmd.get_int("ADAPTIVE", GGetIntOpts { default: Some(Some(0)), ..Default::default() })?.unwrap_or(0);
        if adaptive == 0 {
            return Ok(false);
        }
        let exclude_obj = self.printer.lookup_object("exclude_object", LookupDefault::ReturnNone)?;
        let exclude = match exclude_obj {
            Some(o) => o,
            None => {
                gcmd.respond_info("Exclude objects not enabled. Using full mesh...", true);
                return Ok(false);
            }
        };
        let eventtime = self.printer.get_reactor().monotonic();
        let status = exclude.get_status(eventtime)?.unwrap_or(pycompat::PyValue::None);
        let objects = match &status {
            pycompat::PyValue::Dict(d) => match d.get(&pycompat::PyValue::Str("objects".into())) {
                Some(pycompat::PyValue::List(l)) => l.clone(),
                _ => Vec::new(),
            },
            _ => Vec::new(),
        };
        if objects.is_empty() {
            return Ok(false);
        }
        let margin = gcmd.get_float("ADAPTIVE_MARGIN", GGetFloatOpts { default: Some(Some(self.adaptive_margin)), ..Default::default() })?.unwrap_or(self.adaptive_margin);
        let mut xs: Vec<f64> = Vec::new();
        let mut ys: Vec<f64> = Vec::new();
        gcmd.respond_info(&format!("Found {} objects", objects.len()), true);
        for obj in &objects {
            let polygon = match obj {
                pycompat::PyValue::Dict(d) => d.get(&pycompat::PyValue::Str("polygon".into())),
                _ => None,
            };
            // `for point in obj["polygon"]`: JSON strings iterate characters, objects keys.
            let points: std::borrow::Cow<[pycompat::PyValue]> = match polygon {
                Some(pycompat::PyValue::List(points)) => points.as_slice().into(),
                Some(pycompat::PyValue::Str(s)) => s.chars().map(|c| pycompat::PyValue::Str(c.to_string())).collect(),
                Some(pycompat::PyValue::Dict(d)) => d.keys().cloned().collect(),
                Some(other) => return Err(py_internal("TypeError", format!("'{}' object is not iterable", pycompat::value::type_name(other)))),
                None => return Err(py_internal("KeyError", "'polygon'".to_string())),
            };
            for point in points.iter() {
                xs.push(polygon_coord(point, 0)?);
                ys.push(polygon_coord(point, 1)?);
            }
        }
        // Python's `min`/`max` keep the first extreme item and replace it only on a strict
        // comparison, which decides the result when a coordinate is NaN.
        let py_min = |v: &[f64]| -> KResult<f64> {
            let (first, rest) = v.split_first().ok_or_else(|| py_internal("ValueError", "min() arg is an empty sequence".to_string()))?;
            Ok(rest.iter().fold(*first, |m, x| if *x < m { *x } else { m }))
        };
        let py_max = |v: &[f64]| -> KResult<f64> {
            let (first, rest) = v.split_first().ok_or_else(|| py_internal("ValueError", "max() arg is an empty sequence".to_string()))?;
            Ok(rest.iter().fold(*first, |m, x| if *x > m { *x } else { m }))
        };
        let mm = (py_min(&xs)?, py_min(&ys)?);
        let mx = (py_max(&xs)?, py_max(&ys)?);
        let orig_mm_py = self.orig_config.borrow().get("mesh_min").cloned().unwrap_or(pycompat::PyValue::None);
        let orig_mx_py = self.orig_config.borrow().get("mesh_max").cloned().unwrap_or(pycompat::PyValue::None);
        let orig_mm = py_tuple_to_pair(&orig_mm_py);
        let orig_mx = py_tuple_to_pair(&orig_mx_py);
        // `max(a, b)` returns `b` only when `b > a`; `min(a, b)` only when `b < a`.
        let py_max2 = |a: f64, b: f64| if b > a { b } else { a };
        let py_min2 = |a: f64, b: f64| if b < a { b } else { a };
        let adj_mm = (py_max2(mm.0 - margin, orig_mm.0), py_max2(mm.1 - margin, orig_mm.1));
        let adj_mx = (py_min2(mx.0 + margin, orig_mx.0), py_min2(mx.1 + margin, orig_mx.1));
        let size = (adj_mx.0 - adj_mm.0, adj_mx.1 - adj_mm.1);
        let ratio = (size.0 / (orig_mx.0 - orig_mm.0), size.1 / (orig_mx.1 - orig_mm.1));
        gcmd.respond_info(&format!("Original mesh bounds: ({},{})", orig_mm_py.str(), orig_mx_py.str()), true);
        let x_count = cfg_int(&self.mesh_config.borrow(), "x_count");
        let y_count = cfg_int(&self.mesh_config.borrow(), "y_count");
        gcmd.respond_info(&format!("Original probe count: ({},{})", x_count, y_count), true);
        gcmd.respond_info(
            &format!("Adapted mesh bounds: ([{}, {}],[{}, {}])", py_f(adj_mm.0), py_f(adj_mm.1), py_f(adj_mx.0), py_f(adj_mx.1)),
            true,
        );
        gcmd.respond_info(&format!("Ratio: ({}, {})", py_f(ratio.0), py_f(ratio.1)), true);
        let mut new_x = py_ceil_int(x_count as f64 * ratio.0)?;
        let mut new_y = py_ceil_int(y_count as f64 * ratio.1)?;
        let mut min_num = 3;
        if new_x.max(new_y) > 6 && new_x.min(new_y) < 4 {
            min_num = 4;
        }
        new_x = new_x.max(min_num);
        new_y = new_y.max(min_num);
        gcmd.respond_info(&format!("Adapted probe count: ({},{})", new_x, new_y), true);
        let adj_size = (size.0.max(new_x as f64), size.1.max(new_y as f64));
        let radius = *self.radius.borrow();
        if let Some(r) = radius {
            let adapted_radius = ((adj_size.0 * adj_size.0) + (adj_size.1 * adj_size.1)).sqrt() / 2.0;
            let adapted_origin = (adj_mm.0 + adj_size.0 / 2.0, adj_mm.1 + adj_size.1 / 2.0);
            let to_origin = (adapted_origin.0 * adapted_origin.0 + adapted_origin.1 * adapted_origin.1).sqrt();
            if adapted_radius + to_origin < r {
                let new_probe = new_x.max(new_y);
                let new_probe = new_probe + (1 - new_probe % 2);
                self.radius.set(Some(adapted_radius));
                self.origin.set(Some(adapted_origin));
                self.mesh_min.set((-adapted_radius, -adapted_radius));
                self.mesh_max.set((adapted_radius, adapted_radius));
                self.mesh_config.borrow_mut().insert("x_count".into(), pycompat::PyValue::Int(new_probe.into()));
                self.mesh_config.borrow_mut().insert("y_count".into(), pycompat::PyValue::Int(new_probe.into()));
            }
        } else {
            self.mesh_min.set(adj_mm);
            self.mesh_max.set(adj_mx);
            self.mesh_config.borrow_mut().insert("x_count".into(), pycompat::PyValue::Int(new_x.into()));
            self.mesh_config.borrow_mut().insert("y_count".into(), pycompat::PyValue::Int(new_y.into()));
        }
        *self.profile_name.borrow_mut() = None;
        Ok(true)
    }

    pub fn update_config(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        self.radius.set(match self.orig_config.borrow().get("radius") {
            Some(pycompat::PyValue::Float(f)) => Some(*f),
            Some(pycompat::PyValue::Int(n)) => Some(n.0.to_string().parse::<f64>().unwrap_or(0.0)),
            _ => None,
        });
        self.origin.set(match self.orig_config.borrow().get("origin") {
            Some(pycompat::PyValue::Tuple(t)) if t.len() == 2 => Some((py_to_f64(&t[0]), py_to_f64(&t[1]))),
            _ => None,
        });
        self.mesh_min.set(py_tuple_to_pair(self.orig_config.borrow().get("mesh_min").unwrap()));
        self.mesh_max.set(py_tuple_to_pair(self.orig_config.borrow().get("mesh_max").unwrap()));
        let keys: Vec<String> = self.mesh_config.borrow().keys().cloned().collect();
        for key in keys {
            if let Some(v) = self.orig_config.borrow().get(&key).cloned() {
                self.mesh_config.borrow_mut().insert(key, v);
            }
        }
        let params = gcmd.get_command_parameters();
        let mut need_cfg_update = false;
        if self.radius.borrow().is_some() {
            if params.contains_key("MESH_RADIUS") {
                let mut r = gcmd_get_str(gcmd, "MESH_RADIUS")?.parse::<f64>().unwrap_or(0.0);
                r = (r * 10.0).floor() / 10.0;
                self.radius.set(Some(r));
                self.mesh_min.set((-r, -r));
                self.mesh_max.set((r, r));
                need_cfg_update = true;
            }
            if params.contains_key("MESH_ORIGIN") {
                self.origin.set(Some(parse_gcmd_coord(gcmd, "MESH_ORIGIN")?));
                need_cfg_update = true;
            }
            if params.contains_key("ROUND_PROBE_COUNT") {
                let cnt = gcmd.get_int("ROUND_PROBE_COUNT", GGetIntOpts { minval: Some(3), ..Default::default() })?.unwrap();
                self.mesh_config.borrow_mut().insert("x_count".into(), pycompat::PyValue::Int(cnt.into()));
                self.mesh_config.borrow_mut().insert("y_count".into(), pycompat::PyValue::Int(cnt.into()));
                need_cfg_update = true;
            }
        } else {
            if params.contains_key("MESH_MIN") {
                self.mesh_min.set(parse_gcmd_coord(gcmd, "MESH_MIN")?);
                need_cfg_update = true;
            }
            if params.contains_key("MESH_MAX") {
                self.mesh_max.set(parse_gcmd_coord(gcmd, "MESH_MAX")?);
                need_cfg_update = true;
            }
            if params.contains_key("PROBE_COUNT") {
                let (x_cnt, y_cnt) = parse_gcmd_pair(gcmd, "PROBE_COUNT", Some(3), None)?;
                self.mesh_config.borrow_mut().insert("x_count".into(), pycompat::PyValue::Int(x_cnt.into()));
                self.mesh_config.borrow_mut().insert("y_count".into(), pycompat::PyValue::Int(y_cnt.into()));
                need_cfg_update = true;
            }
        }
        if params.contains_key("MESH_PPS") {
            let (x, y) = parse_gcmd_pair(gcmd, "MESH_PPS", Some(0), None)?;
            self.mesh_config.borrow_mut().insert("mesh_x_pps".into(), pycompat::PyValue::Int(x.into()));
            self.mesh_config.borrow_mut().insert("mesh_y_pps".into(), pycompat::PyValue::Int(y.into()));
            need_cfg_update = true;
        }
        if params.contains_key("ALGORITHM") {
            let a = gcmd_get_str(gcmd, "ALGORITHM")?.trim().to_lowercase();
            self.mesh_config.borrow_mut().insert("algo".into(), pycompat::PyValue::Str(a));
            need_cfg_update = true;
        }
        need_cfg_update |= self.set_adaptive_mesh(gcmd)?;
        let probe_method = gcmd_get_str_default(gcmd, "METHOD", "automatic");
        if need_cfg_update {
            self.verify_algorithm()?;
            self.probe_mgr.generate_points(&self.mesh_config.borrow().clone(), self.mesh_min.borrow().clone(), self.mesh_max.borrow().clone(), self.radius.borrow().clone(), self.origin.borrow().clone(), &probe_method)?;
            let msg = self.mesh_config.borrow().iter().map(|(k, v)| format!("{}: {}", k, v.str())).collect::<Vec<_>>().join("\n");
            klippy_api::log::info(&format!("Updated Mesh Configuration:\n{msg}"));
        } else {
            self.probe_mgr.generate_points(&self.mesh_config.borrow().clone(), self.mesh_min.borrow().clone(), self.mesh_max.borrow().clone(), self.radius.borrow().clone(), self.origin.borrow().clone(), &probe_method)?;
        }
        Ok(())
    }

    pub fn dump_calibration(&self, gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<pycompat::PyValue> {
        if let Some(g) = gcmd {
            if !g.get_command_parameters().is_empty() {
                self.update_config(g)?;
            }
        }
        let mut cfg: IndexMap<pycompat::PyValue, pycompat::PyValue> = IndexMap::new();
        for (k, v) in self.mesh_config.borrow().iter() {
            cfg.insert(pycompat::PyValue::Str(k.clone()), v.clone());
        }
        cfg.insert(pycompat::PyValue::Str("mesh_min".into()), py_pair_to_tuple_py(self.mesh_min.borrow().clone()));
        cfg.insert(pycompat::PyValue::Str("mesh_max".into()), py_pair_to_tuple_py(self.mesh_max.borrow().clone()));
        cfg.insert(pycompat::PyValue::Str("origin".into()), opt_pair_py(self.origin.borrow().clone()));
        cfg.insert(pycompat::PyValue::Str("radius".into()), match self.radius.borrow().clone() {
            Some(r) => pycompat::PyValue::Float(r),
            None => pycompat::PyValue::None,
        });
        let points: Vec<pycompat::PyValue> = self.probe_mgr.get_base_points().into_iter().map(|p| py_pair_to_tuple_py(p)).collect();
        let ppath: Vec<pycompat::PyValue> = self.probe_mgr.get_std_path().into_iter().map(|p| py_pair_to_tuple_py(p)).collect();
        let rapid: Vec<pycompat::PyValue> = self.probe_mgr.iter_rapid_path().into_iter().map(|(p, s)| pycompat::PyValue::Tuple(vec![py_pair_to_tuple_py(p), pycompat::PyValue::Bool(s)])).collect();
        let mut out: IndexMap<pycompat::PyValue, pycompat::PyValue> = IndexMap::new();
        out.insert(pycompat::PyValue::Str("points".into()), pycompat::PyValue::List(points));
        out.insert(pycompat::PyValue::Str("config".into()), pycompat::PyValue::Dict(cfg));
        out.insert(pycompat::PyValue::Str("probe_path".into()), pycompat::PyValue::List(ppath));
        out.insert(pycompat::PyValue::Str("rapid_path".into()), pycompat::PyValue::List(rapid));
        Ok(pycompat::PyValue::Dict(out))
    }

    pub fn cmd_bed_mesh_calibrate(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let name = gcmd_get_str_default(gcmd, "PROFILE", "default");
        if name.trim().is_empty() {
            return Err(cmd_err("Value for parameter 'PROFILE' must be specified".into()));
        }
        *self.profile_name.borrow_mut() = Some(name);
        self.bedmesh()?.set_mesh(None)?;
        if let Err(e) = self.update_config(gcmd) {
            if e.class_name() == "BedMeshError" {
                return Err(cmd_err(e.message().to_string()));
            }
            return Err(e);
        }
        self.probe_mgr.start_probe(gcmd)
    }

    pub fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<()> {
        let mut z_offset = 0.0;
        let mut positions: Vec<Vec<f64>> = positions
            .iter()
            .map(|p| vec![text::round_py_nd(p.bed_x, 2), text::round_py_nd(p.bed_y, 2), p.bed_z])
            .collect();
        let probed_pts = positions.clone();
        if self.probe_mgr.get_zero_ref_mode() == ZrefMode::Probe {
            let ref_pos = positions.pop().unwrap();
            klippy_api::log::info(&format!(
                "bed_mesh: z-offset replaced with probed z value at position ({:.2}, {:.2}, {:.6})",
                ref_pos[0], ref_pos[1], ref_pos[2]
            ));
            z_offset = ref_pos[2];
        }
        let base_points: Vec<(f64, f64)> = self.probe_mgr.get_base_points();
        let mut params: IndexMap<String, pycompat::PyValue> = self.mesh_config.borrow().clone();
        let min_x = base_points.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let max_x = base_points.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
        let min_y = base_points.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let max_y = base_points.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
        params.insert("min_x".into(), pycompat::PyValue::Float(min_x));
        params.insert("max_x".into(), pycompat::PyValue::Float(max_x));
        params.insert("min_y".into(), pycompat::PyValue::Float(min_y));
        params.insert("max_y".into(), pycompat::PyValue::Float(max_y));
        let x_cnt = cfg_int(&params, "x_count");
        let y_cnt = cfg_int(&params, "y_count");
        let subs = self.probe_mgr.get_substitutes();
        if !subs.is_empty() {
            let mut corrected: Vec<Vec<f64>> = Vec::new();
            let mut idx_offset: i64 = 0;
            let mut start_idx: usize = 0;
            for (i, pts) in subs.iter() {
                let mut fpt = vec![base_points[*i].0, base_points[*i].1];
                let idx = (*i as i64 + idx_offset) as usize;
                corrected.extend_from_slice(&positions[start_idx..idx]);
                let mut sum = 0.0;
                for p in &positions[idx..idx + pts.len()] {
                    sum += p[2];
                }
                let avg_z = sum / pts.len() as f64;
                idx_offset += pts.len() as i64 - 1;
                start_idx = idx + pts.len();
                fpt.push(avg_z);
                klippy_api::log::info(&format!(
                    "bed_mesh: Replacing value at faulty index {} ({:.4}, {:.4}): avg value = {:.6}, avg w/ z_offset = {:.6}",
                    i, fpt[0], fpt[1], avg_z, avg_z - z_offset
                ));
                corrected.push(fpt);
            }
            corrected.extend_from_slice(&positions[start_idx..]);
            positions = corrected;
        }
        if base_points.len() != positions.len() {
            self.dump_points(&probed_pts, &positions);
            return Err(cmd_err(format!(
                "bed_mesh: invalid position list size, generated count: {}, probed count: {}",
                base_points.len(),
                positions.len()
            )));
        }
        let mut probed_matrix: Vec<Vec<f64>> = Vec::new();
        let mut row: Vec<f64> = Vec::new();
        let mut prev_pos = base_points[0];
        for (pos, result) in base_points.iter().zip(positions.iter()) {
            if !isclose_tol(pos.0, result[0], 0.5) || !isclose_tol(pos.1, result[1], 0.5) {
                klippy_api::log::info(&format!(
                    "bed_mesh: point deviation > .5mm: orig pt = ({:.2}, {:.2}), probed pt = ({:.2}, {:.2})",
                    pos.0, pos.1, result[0], result[1]
                ));
            }
            let z_pos = result[2] - z_offset;
            if !isclose_tol(pos.1, prev_pos.1, 0.1) {
                probed_matrix.push(row.clone());
                row = Vec::new();
            }
            if pos.0 > prev_pos.0 {
                row.push(z_pos);
            } else {
                row.insert(0, z_pos);
            }
            prev_pos = *pos;
        }
        probed_matrix.push(row);
        if probed_matrix.len() != y_cnt as usize {
            return Err(cmd_err(format!(
                "bed_mesh: Invalid y-axis table length\nProbed table length: {} Probed Table:\n{}",
                probed_matrix.len(),
                py_repr_list_of_lists(&probed_matrix)
            )));
        }
        if let Some(_r) = self.radius.borrow().clone() {
            for row in probed_matrix.iter_mut() {
                let row_size = row.len();
                if row_size % 2 == 0 {
                    return Err(cmd_err("bed_mesh: incorrect number of points sampled on X\nProbed Table:\n".to_string() + &py_repr_list_of_lists(&probed_matrix)));
                }
                let buf_cnt = (x_cnt as usize - row_size) / 2;
                if buf_cnt == 0 {
                    continue;
                }
                let left = row[0];
                let right = row[row_size - 1];
                // Matches Python: row[0:0] = left_buffer; row.extend(right_buffer)
                let mut front = vec![left; buf_cnt];
                front.extend_from_slice(row);
                let mut back = vec![right; buf_cnt];
                front.append(&mut back);
                *row = front;
            }
        }
        for row in &probed_matrix {
            if row.len() != x_cnt as usize {
                return Err(cmd_err(format!(
                    "bed_mesh: invalid x-axis table length\nProbed table length: {} Probed Table:\n{}",
                    probed_matrix.len(),
                    py_repr_list_of_lists(&probed_matrix)
                )));
            }
        }
        let z_mesh = Arc::new(ZMesh::new(params, self.profile_name.borrow().as_deref()));
        z_mesh.build_mesh(probed_matrix)?;
        if self.probe_mgr.get_zero_ref_mode() == ZrefMode::InMesh {
            let zrp = self.probe_mgr.get_zero_ref_pos().unwrap();
            z_mesh.set_zero_reference(zrp.0, zrp.1);
        }
        let bm = self.bedmesh()?;
        bm.set_mesh(Some(z_mesh))?;
        self.gcode.respond_info("Mesh Bed Leveling Complete", true);
        if let Some(name) = self.profile_name.borrow().clone() {
            bm.save_profile(&name)?;
        }
        Ok(())
    }

    fn dump_points(&self, probed_pts: &Vec<Vec<f64>>, corrected_pts: &Vec<Vec<f64>>) {
        let points = self.probe_mgr.get_base_points();
        let max_len = points.len().max(probed_pts.len()).max(corrected_pts.len());
        klippy_api::log::info("bed_mesh: calibration point dump\nIndex | Generated Point| Probed Point| Corrected Point");
        for i in 0..max_len {
            let gen_pt = if i < points.len() {
                format!("({:.2}, {:.2})", points[i].0, points[i].1)
            } else {
                String::new()
            };
            let probed_pt = if i < probed_pts.len() {
                format!("({:.2}, {:.2}, {:.4})", probed_pts[i][0], probed_pts[i][1], probed_pts[i][2])
            } else {
                String::new()
            };
            let corr_pt = if i < corrected_pts.len() {
                format!("({:.2}, {:.2}, {:.4})", corrected_pts[i][0], corrected_pts[i][1], corrected_pts[i][2])
            } else {
                String::new()
            };
            klippy_api::log::info(&format!("  {:<4}| {:<17}| {:<25}| {}", i, gen_pt, probed_pt, corr_pt));
        }
    }
}

fn py_repr_list(v: &[(f64, f64)]) -> String {
    let parts: Vec<String> = v.iter().map(|p| format!("({:?}, {:?})", p.0, p.1)).collect();
    format!("[{}]", parts.join(", "))
}

fn py_repr_list_of_lists(m: &Vec<Vec<f64>>) -> String {
    let parts: Vec<String> = m
        .iter()
        .map(|r| format!("[{}]", r.iter().map(|v| format!("{}", py_f(*v))).collect::<Vec<_>>().join(", ")))
        .collect();
    format!("[{}]", parts.join(", "))
}

fn py_tuple_to_pair(v: &pycompat::PyValue) -> (f64, f64) {
    match v {
        pycompat::PyValue::Tuple(t) if t.len() == 2 => (py_to_f64(&t[0]), py_to_f64(&t[1])),
        _ => (0.0, 0.0),
    }
}

fn py_pair_to_tuple_py(p: (f64, f64)) -> pycompat::PyValue {
    pycompat::PyValue::Tuple(vec![pycompat::PyValue::Float(p.0), pycompat::PyValue::Float(p.1)])
}

fn opt_pair_py(p: Option<(f64, f64)>) -> pycompat::PyValue {
    match p {
        Some(v) => py_pair_to_tuple_py(v),
        None => pycompat::PyValue::None,
    }
}

fn py_to_f64(v: &pycompat::PyValue) -> f64 {
    match v {
        pycompat::PyValue::Float(f) => *f,
        pycompat::PyValue::Int(n) => n.0.to_string().parse::<f64>().unwrap_or(0.0),
        pycompat::PyValue::Bool(b) => *b as u8 as f64,
        _ => 0.0,
    }
}

fn py_f(f: f64) -> String {
    text::float_repr(f)
}

/// An exception Klippy does not catch by class (reported as an internal error).
fn py_internal(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message }
}

/// `point[index]` of an `exclude_object` polygon point, as the number
/// `set_adaptive_mesh` computes with (bed_mesh.py:476-477).
fn polygon_coord(point: &pycompat::PyValue, index: usize) -> KResult<f64> {
    let coord = match point {
        pycompat::PyValue::List(items) => items
            .get(index)
            .ok_or_else(|| py_internal("IndexError", "list index out of range".to_string()))?,
        pycompat::PyValue::Dict(_) => return Err(py_internal("KeyError", index.to_string())),
        pycompat::PyValue::Str(s) => {
            if s.chars().nth(index).is_none() {
                return Err(py_internal("IndexError", "string index out of range".to_string()));
            }
            point
        }
        other => {
            return Err(py_internal(
                "TypeError",
                format!("'{}' object is not subscriptable", pycompat::value::type_name(other)),
            ))
        }
    };
    match coord {
        pycompat::PyValue::Int(_) | pycompat::PyValue::Float(_) | pycompat::PyValue::Bool(_) => Ok(py_to_f64(coord)),
        other => Err(py_internal(
            "TypeError",
            format!("unsupported operand type(s) for -: '{}' and 'float'", pycompat::value::type_name(other)),
        )),
    }
}

/// `int(math.ceil(v))` for a float `v`.
fn py_ceil_int(v: f64) -> KResult<i64> {
    if v.is_nan() {
        return Err(py_internal("ValueError", "cannot convert float NaN to integer".to_string()));
    }
    if v.is_infinite() {
        return Err(py_internal("OverflowError", "cannot convert float infinity to integer".to_string()));
    }
    Ok(v.ceil() as i64)
}

fn cfg_int(m: &IndexMap<String, pycompat::PyValue>, k: &str) -> i64 {
    match m.get(k) {
        Some(pycompat::PyValue::Int(n)) => n.0.to_string().parse().unwrap_or(0),
        Some(pycompat::PyValue::Float(f)) => *f as i64,
        _ => 0,
    }
}

fn verify_algo(m: &mut IndexMap<String, pycompat::PyValue>, err: &impl Fn(String) -> KlippyError) -> KResult<()> {
    let x_pps = cfg_int(m, "mesh_x_pps");
    let y_pps = cfg_int(m, "mesh_y_pps");
    let algo = match m.get("algo") {
        Some(pycompat::PyValue::Str(s)) => s.clone(),
        _ => "lagrange".to_string(),
    };
    if algo != "lagrange" && algo != "bicubic" {
        return Err(err(format!("bed_mesh: Unknown algorithm <{algo}>")));
    }
    let max_cnt = cfg_int(m, "x_count").max(cfg_int(m, "y_count"));
    let min_cnt = cfg_int(m, "x_count").min(cfg_int(m, "y_count"));
    if x_pps.max(y_pps) == 0 {
        m.insert("algo".into(), pycompat::PyValue::Str("direct".into()));
        return Ok(());
    }
    if algo == "lagrange" && max_cnt > 6 {
        return Err(err(format!(
            "bed_mesh: cannot exceed a probe_count of 6 when using lagrange interpolation. Configured Probe Count: {}, {}",
            cfg_int(m, "x_count"),
            cfg_int(m, "y_count")
        )));
    }
    if algo == "bicubic" && min_cnt < 4 {
        if max_cnt > 6 {
            return Err(err(format!(
                "bed_mesh: invalid probe_count option when using bicubic interpolation.  Combination of 3 points on one axis with more than 6 on another is not permitted. Configured Probe Count: {}, {}",
                cfg_int(m, "x_count"),
                cfg_int(m, "y_count")
            )));
        } else {
            klippy_api::log::info(&format!(
                "bed_mesh: bicubic interpolation with a probe_count of less than 4 points detected.  Forcing lagrange interpolation. Configured Probe Count: {}, {}",
                cfg_int(m, "x_count"),
                cfg_int(m, "y_count")
            ));
            m.insert("algo".into(), pycompat::PyValue::Str("lagrange".into()));
        }
    }
    Ok(())
}

fn gcmd_get_str_default(gcmd: &klippy_api::gcode::GCmd, name: &str, default: &str) -> String {
    match gcmd.get(name, GGetOpts { default: Some(pycompat::PyValue::Str(default.into())), ..Default::default() }) {
        Ok(v) => v.str(),
        Err(_) => default.to_string(),
    }
}

fn lookup_gcode(printer: &Printer) -> KResult<klippy_api::gcode::GCode> {
    match printer.lookup_object("gcode", LookupDefault::Raise)? {
        None => Err(KlippyError::config("gcode object not found")),
        Some(o) => Ok(klippy_api::require_interface::<dyn GCodeApi>(&o, "gcode")?),
    }
}

// ---------------------------------------------------------------------------
// ProfileManager
// ---------------------------------------------------------------------------

pub struct ProfileManager {
    name: String,
    printer: Printer,
    gcode: klippy_api::gcode::GCode,
    profiles: GCell<IndexMap<String, IndexMap<String, pycompat::PyValue>>>,
    incompatible_profiles: GCell<Vec<String>>,
}

impl ProfileManager {
    fn bedmesh(&self) -> KResult<Arc<dyn BedMeshApi>> {
        Ok(lookup_interface::<dyn BedMeshApi>(&*self.printer, "bed_mesh", LookupDefault::Raise)?.expect("bed_mesh exists"))
    }


    pub fn new(config: &Config) -> KResult<ProfileManager> {
        let name = config.get_name();
        let printer = config.get_printer();
        let gcode = lookup_gcode(&printer)?;
        let mut profiles: IndexMap<String, IndexMap<String, pycompat::PyValue>> = IndexMap::new();
        let mut incompatible: Vec<String> = Vec::new();
        let stored = config.get_prefix_sections(&name);
        for s in &stored {
            if s.get_name() == name {
                continue;
            }
            let full = s.get_name();
            let pname = full.splitn(2, ' ').nth(1).unwrap_or("").to_string();
            let version = s.getint("version", GetIntOpts { default: Some(Some(0)), ..Default::default() })?.unwrap();
            if version != PROFILE_VERSION {
                klippy_api::log::info(&format!(
                    "bed_mesh: Profile [{pname}] not compatible with this version\nof bed_mesh.  Profile Version: {version} Current Version: {PROFILE_VERSION} "
                ));
                incompatible.push(pname);
                continue;
            }
            let mut params: IndexMap<String, pycompat::PyValue> = IndexMap::new();
            for (key, t) in profile_options() {
                match t {
                    "int" => {
                        let v = s.getint(key, GetIntOpts::default())?.unwrap_or(0);
                        params.insert(key.to_string(), pycompat::PyValue::Int(v.into()));
                    }
                    "float" => {
                        let v = s.getfloat(key, GetFloatOpts::default())?.unwrap_or(0.0);
                        params.insert(key.to_string(), pycompat::PyValue::Float(v));
                    }
                    _ => {
                        let v = s.get(key, GetStrOpts::default())?.unwrap_or_default();
                        params.insert(key.to_string(), pycompat::PyValue::Str(v));
                    }
                }
            }
            let points_raw = s.getlists("points", klippy_api::config::GetListsOpts { seps: vec![",".into(), "\n".into()], parser: klippy_api::config::ListParser::Float, ..Default::default() })?;
            let points = match points_raw {
                pycompat::PyValue::List(rows) => pycompat::PyValue::List(
                    rows.into_iter()
                        .map(|r| match r {
                            pycompat::PyValue::List(vals) => pycompat::PyValue::List(vals),
                            other => pycompat::PyValue::List(vec![other]),
                        })
                        .collect(),
                ),
                _ => pycompat::PyValue::List(vec![]),
            };
            let mut prof: IndexMap<String, pycompat::PyValue> = IndexMap::new();
            prof.insert("points".into(), points);
            prof.insert("mesh_params".into(), pycompat::PyValue::Dict(py_dict(params)));
            profiles.insert(pname, prof);
        }
        Ok(ProfileManager {
            name,
            printer,
            gcode,
            profiles: GCell::new(profiles),
            incompatible_profiles: GCell::new(incompatible),
        })
    }

    pub fn get_profiles(&self) -> IndexMap<String, IndexMap<String, pycompat::PyValue>> {
        self.profiles.borrow().clone()
    }

    pub fn save_profile(&self, prof_name: &str) -> KResult<()> {
        let bm = self.bedmesh()?;
        let z_mesh = match bm.get_mesh() {
            Some(m) => m,
            None => {
                self.gcode.respond_info(&format!("Unable to save to profile [{prof_name}], the bed has not been probed"), true);
                return Ok(());
            }
        };
        let probed = z_mesh.get_probed_matrix();
        let mesh_params = z_mesh.get_mesh_params();
        let cfg_obj = self.printer.lookup_object("configfile", LookupDefault::Raise)?.expect("configfile");
        let cfg = klippy_api::require_interface::<dyn klippy_api::config::PrinterConfigApi>(&cfg_obj, "configfile")?;
        let cfg_name = format!("{} {}", self.name, prof_name);
        let mut z_values = String::new();
        for line in &probed {
            z_values.push_str("\n  ");
            for p in line {
                z_values.push_str(&format!("{p:.6}, "));
            }
            z_values = z_values[..z_values.len() - 2].to_string();
        }
        cfg.set(&cfg_name, "version", &PROFILE_VERSION.to_string())?;
        cfg.set(&cfg_name, "points", &z_values)?;
        for (k, v) in &mesh_params {
            cfg.set(&cfg_name, k, &v.str())?;
        }
        let mut profiles = self.profiles.borrow().clone();
        let mut prof: IndexMap<String, pycompat::PyValue> = IndexMap::new();
        prof.insert("points".into(), matrix_to_py(&probed));
        prof.insert("mesh_params".into(), pycompat::PyValue::Dict(py_dict(mesh_params)));
        profiles.insert(prof_name.to_string(), prof);
        *self.profiles.borrow_mut() = profiles;
        bm.update_status();
        self.gcode.respond_info(&format!("Bed Mesh state has been saved to profile [{prof_name}]\nfor the current session.  The SAVE_CONFIG command will\nupdate the printer config file and restart the printer."), true);
        Ok(())
    }

    pub fn load_profile(&self, prof_name: &str) -> KResult<()> {
        let prof = self.profiles.borrow().get(prof_name).cloned();
        let prof = match prof {
            Some(p) => p,
            None => return Err(cmd_err(format!("bed_mesh: Unknown profile [{prof_name}]"))),
        };
        let points = py_to_matrix(prof.get("points").unwrap());
        let params = py_dict_to_map(prof.get("mesh_params").unwrap());
        let z_mesh = Arc::new(ZMesh::new(params, Some(prof_name)));
        if let Err(e) = z_mesh.build_mesh(points) {
            if e.class_name() == "BedMeshError" {
                return Err(cmd_err(e.message().to_string()));
            }
            return Err(e);
        }
        self.bedmesh()?.set_mesh(Some(z_mesh))
    }

    pub fn remove_profile(&self, prof_name: &str) -> KResult<()> {
        if self.profiles.borrow().contains_key(prof_name) {
            let cfg_obj = self.printer.lookup_object("configfile", LookupDefault::Raise)?.expect("configfile");
            let cfg = klippy_api::require_interface::<dyn klippy_api::config::PrinterConfigApi>(&cfg_obj, "configfile")?;
            cfg.remove_section(&format!("bed_mesh {prof_name}"));
            let mut profiles = self.profiles.borrow().clone();
            profiles.shift_remove(prof_name);
            *self.profiles.borrow_mut() = profiles;
            self.bedmesh()?.update_status();
            self.gcode.respond_info(&format!("Profile [{prof_name}] removed from storage for this session.\nThe SAVE_CONFIG command will update the printer\nconfiguration and restart the printer"), true);
        } else {
            self.gcode.respond_info(&format!("No profile named [{prof_name}] to remove"), true);
        }
        Ok(())
    }

    pub fn cmd_bed_mesh_profile(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        for key in ["LOAD", "SAVE", "REMOVE"] {
            let name = gcmd.get(key, GGetOpts { default: Some(pycompat::PyValue::None), ..Default::default() })?;
            let name_str = match name {
                pycompat::PyValue::Str(s) => s,
                pycompat::PyValue::None => continue,
                other => other.str(),
            };
            if name_str.trim().is_empty() {
                return Err(cmd_err(format!("Value for parameter '{key}' must be specified")));
            }
            if name_str == "default" && key == "SAVE" {
                gcmd.respond_info("Profile 'default' is reserved, please choose another profile name.", true);
                return Ok(());
            }
            match key {
                "LOAD" => return self.load_profile(&name_str),
                "SAVE" => return self.save_profile(&name_str),
                _ => return self.remove_profile(&name_str),
            }
        }
        gcmd.respond_info(&format!("Invalid syntax '{}'", gcmd.get_commandline()), true);
        Ok(())
    }
}

fn profile_options() -> Vec<(&'static str, &'static str)> {
    vec![
        ("min_x", "float"),
        ("max_x", "float"),
        ("min_y", "float"),
        ("max_y", "float"),
        ("x_count", "int"),
        ("y_count", "int"),
        ("mesh_x_pps", "int"),
        ("mesh_y_pps", "int"),
        ("algo", "str"),
        ("tension", "float"),
    ]
}

fn py_dict(m: IndexMap<String, pycompat::PyValue>) -> IndexMap<pycompat::PyValue, pycompat::PyValue> {
    m.into_iter().map(|(k, v)| (pycompat::PyValue::Str(k), v)).collect()
}

fn py_dict_to_map(v: &pycompat::PyValue) -> IndexMap<String, pycompat::PyValue> {
    match v {
        pycompat::PyValue::Dict(d) => d
            .iter()
            .map(|(k, val)| (k.str(), val.clone()))
            .collect(),
        _ => IndexMap::new(),
    }
}

fn matrix_to_py(m: &Vec<Vec<f64>>) -> pycompat::PyValue {
    pycompat::PyValue::List(
        m.iter()
            .map(|r| pycompat::PyValue::List(r.iter().map(|v| pycompat::PyValue::Float(*v)).collect()))
            .collect(),
    )
}

fn py_to_matrix(v: &pycompat::PyValue) -> Vec<Vec<f64>> {
    match v {
        pycompat::PyValue::List(rows) => rows
            .iter()
            .map(|r| match r {
                pycompat::PyValue::List(vals) => vals.iter().map(py_to_f64).collect(),
                pycompat::PyValue::Tuple(vals) => vals.iter().map(py_to_f64).collect(),
                _ => vec![],
            })
            .collect(),
        _ => vec![],
    }
}

// ---------------------------------------------------------------------------
// BedMesh
// ---------------------------------------------------------------------------

/// `BedMesh` (bed_mesh.py:86), the `bed_mesh` printer object.
/// `FADE_DISABLE` (bed_mesh.py:87).
pub const FADE_DISABLE: f64 = 2147483647.0;

pub struct BedMesh {
    printer: Printer,
    last_position: GCell<Vec<f64>>,
    z_mesh: GCell<Option<Arc<ZMesh>>>,
    toolhead: GCell<Option<ToolHead>>,
    horizontal_move_z: f64,
    fade_start: GCell<f64>,
    fade_end: GCell<f64>,
    fade_dist: GCell<f64>,
    log_fade_complete: GCell<bool>,
    base_fade_target: GCell<Option<f64>>,
    fade_target: GCell<f64>,
    tool_offset: GCell<f64>,
    gcode: klippy_api::gcode::GCode,
    splitter: MoveSplitter,
    pmgr: ProfileManager,
    bmc: BedMeshCalibrate,
    status: GCell<pycompat::PyValue>,
}

impl BedMesh {
    /// `BedMesh.__init__(config)` (bed_mesh.py:88).
    pub fn new(config: &Config) -> KResult<BedMesh> {
        let printer = config.get_printer();
        let gcode = lookup_gcode(&printer)?;
        let horizontal_move_z = config
            .getfloat("horizontal_move_z", GetFloatOpts { default: Some(Some(5.0)), ..Default::default() })?
            .unwrap();
        let fade_start0 = config
            .getfloat("fade_start", GetFloatOpts { default: Some(Some(1.0)), ..Default::default() })?
            .unwrap();
        let fade_end0 = config
            .getfloat("fade_end", GetFloatOpts { default: Some(Some(0.0)), ..Default::default() })?
            .unwrap();
        let mut fade_start = fade_start0;
        let mut fade_end = fade_end0;
        let mut fade_dist = fade_end - fade_start;
        if fade_dist <= 0.0 {
            fade_start = FADE_DISABLE;
            fade_end = FADE_DISABLE;
            fade_dist = fade_end - fade_start;
        }
        let base_fade_target = config
            .getfloat("fade_target", GetFloatOpts { default: Some(None), ..Default::default() })?;
        let splitter = MoveSplitter::new(config)?;
        let pmgr = ProfileManager::new(config)?;
        let bmc = BedMeshCalibrate::new(config)?;
        let bm = BedMesh {
            printer,
            last_position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            z_mesh: GCell::new(None),
            toolhead: GCell::new(None),
            horizontal_move_z,
            fade_start: GCell::new(fade_start),
            fade_end: GCell::new(fade_end),
            fade_dist: GCell::new(fade_dist),
            log_fade_complete: GCell::new(false),
            base_fade_target: GCell::new(base_fade_target),
            fade_target: GCell::new(0.0),
            tool_offset: GCell::new(0.0),
            gcode,
            splitter,
            pmgr,
            bmc,
            status: GCell::new(pycompat::PyValue::None),
        };
        bm.update_status();
        Ok(bm)
    }

    pub fn handle_connect(&self) -> KResult<()> {
        let th = lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?.expect("toolhead exists");
        *self.toolhead.borrow_mut() = Some(th);
        self.bmc.print_generated_points(&|m| klippy_api::log::info(m), true);
        Ok(())
    }

    pub fn set_mesh(&self, mesh: Option<Arc<ZMesh>>) -> KResult<()> {
        if let Some(m) = mesh.clone() {
            if self.fade_end.get() != FADE_DISABLE {
                self.log_fade_complete.set(true);
                if self.base_fade_target.borrow().is_none() {
                    self.fade_target.set(m.get_z_average());
                } else {
                    self.fade_target.set(self.base_fade_target.borrow().unwrap());
                    let (min_z, max_z) = m.get_z_range();
                    let target = self.fade_target.get();
                    if !(min_z <= target && target <= max_z) && target != 0.0 {
                        let err_target = self.fade_target.get();
                        *self.z_mesh.borrow_mut() = None;
                        self.fade_target.set(0.0);
                        return Err(cmd_err(format!(
                            "bed_mesh: ERROR, fade_target lies outside of mesh z range\nmin: {min_z:.4}, max: {max_z:.4}, fade_target: {err_target:.4}"
                        )));
                    }
                }
                let (min_z, max_z) = m.get_z_range();
                if self.fade_dist.get() <= min_z.abs().max(max_z.abs()) {
                    *self.z_mesh.borrow_mut() = None;
                    self.fade_target.set(0.0);
                    return Err(cmd_err(format!(
                        "bed_mesh:  Mesh extends outside of the fade range, please see the fade_start and fade_end options inexample-extras.cfg. fade distance: {:.2} mesh min: {:.4}mesh max: {:.4}",
                        self.fade_dist.get(), min_z, max_z
                    )));
                }
            } else {
                self.fade_target.set(0.0);
            }
        } else {
            self.fade_target.set(0.0);
        }
        self.tool_offset.set(0.0);
        *self.z_mesh.borrow_mut() = mesh.clone();
        self.splitter.initialize(mesh, self.fade_target.get());
        let gmove = lookup_gcode_move(&self.printer)?;
        gmove.reset_last_position()?;
        self.update_status();
        Ok(())
    }

    pub fn get_z_factor(&self, z_pos: f64) -> f64 {
        let z_pos = z_pos + self.tool_offset.get();
        if z_pos >= self.fade_end.get() {
            0.0
        } else if z_pos >= self.fade_start.get() {
            (self.fade_end.get() - z_pos) / self.fade_dist.get()
        } else {
            1.0
        }
    }

    pub fn get_position(&self) -> KResult<Vec<f64>> {
        match self.z_mesh.borrow().clone() {
            None => {
                let th = self.toolhead.borrow().clone().expect("toolhead");
                let mut pos = th.get_position();
                pos[2] -= self.fade_target.get();
                *self.last_position.borrow_mut() = pos.clone();
                Ok(pos)
            }
            Some(mesh) => {
                let th = self.toolhead.borrow().clone().expect("toolhead");
                let cur = th.get_position();
                let (x, y, z) = (cur[0], cur[1], cur[2]);
                let max_adj = mesh.calc_z(x, y);
                let mut factor = 1.0;
                let z_adj = max_adj - self.fade_target.get();
                let fade_z_pos = z + self.tool_offset.get();
                if fade_z_pos.min(fade_z_pos - max_adj) >= self.fade_end.get() {
                    factor = 0.0;
                } else if fade_z_pos.max(fade_z_pos - max_adj) >= self.fade_start.get() {
                    factor = (self.fade_end.get() + self.fade_target.get() - fade_z_pos) / (self.fade_dist.get() - z_adj);
                    factor = constrain(factor, 0.0, 1.0);
                }
                let final_z_adj = factor * z_adj + self.fade_target.get();
                let mut pos = vec![x, y, z - final_z_adj];
                pos.extend_from_slice(&cur[3..]);
                *self.last_position.borrow_mut() = pos.clone();
                Ok(pos)
            }
        }
    }

    /// `move(newpos, speed)` (bed_mesh.py:212). Pauses the reactor via toolhead.
    pub fn move_pos(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        let factor = self.get_z_factor(newpos[2]);
        if self.z_mesh.borrow().is_none() || factor == 0.0 {
            let (x, y, z) = (newpos[0], newpos[1], newpos[2]);
            if self.log_fade_complete.get() {
                self.log_fade_complete.set(false);
                klippy_api::log::info(&format!("bed_mesh fade complete: Current Z: {z:.4} fade_target: {:.4} ", self.fade_target.get()));
            }
            let th = self.toolhead.borrow().clone().expect("toolhead");
            let mut pos = vec![x, y, z + self.fade_target.get()];
            pos.extend_from_slice(&newpos[3..]);
            th.r#move(pos, speed)?;
        } else {
            let last = self.last_position.borrow().clone();
            self.splitter.build_move(last, newpos.clone(), factor)?;
            while !self.splitter.traverse_complete.get() {
                let split_move = self.splitter.split()?;
                match split_move {
                    Some(m) => {
                        let th = self.toolhead.borrow().clone().expect("toolhead");
                        th.r#move(m, speed)?;
                    }
                    None => return Err(cmd_err(TRAVERSE_COMPLETE_ERROR.to_string())),
                }
            }
        }
        *self.last_position.borrow_mut() = newpos;
        Ok(())
    }

    pub fn get_status_value(&self) -> pycompat::PyValue {
        self.status.borrow().clone()
    }

    pub fn update_status(&self) {
        let mut d: IndexMap<pycompat::PyValue, pycompat::PyValue> = IndexMap::new();
        d.insert(pycompat::PyValue::Str("profile_name".into()), pycompat::PyValue::Str(String::new()));
        d.insert(pycompat::PyValue::Str("mesh_min".into()), py_pair_to_tuple_py((0.0, 0.0)));
        d.insert(pycompat::PyValue::Str("mesh_max".into()), py_pair_to_tuple_py((0.0, 0.0)));
        d.insert(pycompat::PyValue::Str("probed_matrix".into()), pycompat::PyValue::List(vec![pycompat::PyValue::List(vec![])]));
        d.insert(pycompat::PyValue::Str("mesh_matrix".into()), pycompat::PyValue::List(vec![pycompat::PyValue::List(vec![])]));
        d.insert(pycompat::PyValue::Str("profiles".into()), pycompat::PyValue::Dict(py_dict(self.pmgr.get_profiles().into_iter().map(|(k, v)| (k, val_to_py(v))).collect())));
        if let Some(mesh) = self.z_mesh.borrow().clone() {
            let params = mesh.get_mesh_params();
            let get_f = |k: &str| py_to_f64(params.get(k).unwrap_or(&pycompat::PyValue::Float(0.0)));
            let mesh_min = (get_f("min_x"), get_f("min_y"));
            let mesh_max = (get_f("max_x"), get_f("max_y"));
            d.insert(pycompat::PyValue::Str("profile_name".into()), pycompat::PyValue::Str(mesh.get_profile_name()));
            d.insert(pycompat::PyValue::Str("mesh_min".into()), py_pair_to_tuple_py(mesh_min));
            d.insert(pycompat::PyValue::Str("mesh_max".into()), py_pair_to_tuple_py(mesh_max));
            d.insert(pycompat::PyValue::Str("probed_matrix".into()), matrix_to_py(&mesh.get_probed_matrix()));
            d.insert(pycompat::PyValue::Str("mesh_matrix".into()), matrix_to_py(&mesh.get_mesh_matrix()));
        }
        *self.status.borrow_mut() = pycompat::PyValue::Dict(d);
    }

    pub fn get_mesh(&self) -> Option<Arc<ZMesh>> {
        self.z_mesh.borrow().clone()
    }

    pub fn cmd_bed_mesh_output(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let pgp = gcmd.get_int("PGP", GGetIntOpts { default: Some(Some(0)), ..Default::default() })?.unwrap_or(0);
        // `if gcmd.get_int('PGP', 0):` is truthy for any nonzero value, and
        // `gcmd.respond_info` logs by default (bed_mesh.py:259-265).
        if pgp != 0 {
            self.bmc.print_generated_points(&|m| gcmd.respond_info(m, true), false);
        } else if self.z_mesh.borrow().is_none() {
            gcmd.respond_info("Bed has not been probed", true);
        } else {
            let mesh = self.z_mesh.borrow().clone().unwrap();
            mesh.print_probed_matrix(|m| gcmd.respond_info(m, true));
            mesh.print_mesh(|m| gcmd.respond_raw(m), Some(self.horizontal_move_z));
        }
        Ok(())
    }

    pub fn cmd_bed_mesh_map(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        if let Some(mesh) = self.z_mesh.borrow().clone() {
            let params = mesh.get_mesh_params();
            let mut out: IndexMap<pycompat::PyValue, pycompat::PyValue> = IndexMap::new();
            out.insert(pycompat::PyValue::Str("mesh_min".into()), py_pair_to_tuple_py((py_to_f64(params.get("min_x").unwrap_or(&pycompat::PyValue::Float(0.0))), py_to_f64(params.get("min_y").unwrap_or(&pycompat::PyValue::Float(0.0))))));
            out.insert(pycompat::PyValue::Str("mesh_max".into()), py_pair_to_tuple_py((py_to_f64(params.get("max_x").unwrap_or(&pycompat::PyValue::Float(0.0))), py_to_f64(params.get("max_y").unwrap_or(&pycompat::PyValue::Float(0.0))))));
            out.insert(pycompat::PyValue::Str("z_positions".into()), matrix_to_py(&mesh.get_probed_matrix()));
            gcmd.respond_raw(&format!("mesh_map_output {}", json::dumps(&pycompat::PyValue::Dict(out)).unwrap()));
        } else {
            gcmd.respond_info("Bed has not been probed", true);
        }
        Ok(())
    }

    pub fn cmd_bed_mesh_clear(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let _ = gcmd;
        self.set_mesh(None)
    }

    pub fn cmd_bed_mesh_offset(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        if let Some(mesh) = self.z_mesh.borrow().clone() {
            let mut offsets: [Option<f64>; 2] = [None, None];
            for (i, axis) in ["X", "Y"].iter().enumerate() {
                offsets[i] = gcmd.get_float(axis, GGetFloatOpts { default: Some(None), ..Default::default() })?;
            }
            mesh.set_mesh_offsets(offsets);
            let tool_offset = gcmd.get_float("ZFADE", GGetFloatOpts { default: Some(None), ..Default::default() })?;
            if let Some(v) = tool_offset {
                self.tool_offset.set(v);
            }
            let gmove = lookup_gcode_move(&self.printer)?;
            gmove.reset_last_position()?;
        } else {
            gcmd.respond_info("No mesh loaded to offset", true);
        }
        Ok(())
    }

    pub fn handle_dump_request(&self, web_request: &klippy_api::webhooks::WebRequest) -> KResult<()> {
        let eventtime = self.printer.get_reactor().monotonic();
        let prb = lookup_probe_raw(&self.printer)?;
        let th_obj = self.printer.lookup_object("toolhead", LookupDefault::Raise)?.expect("toolhead");
        let th = klippy_api::require_interface::<dyn ToolHeadApi>(&th_obj, "toolhead")?;
        let th_sts = th.get_status(eventtime)?;
        let mut result: IndexMap<pycompat::PyValue, pycompat::PyValue> = IndexMap::new();
        result.insert(pycompat::PyValue::Str("current_mesh".into()), pycompat::PyValue::Dict(IndexMap::new()));
        result.insert(pycompat::PyValue::Str("profiles".into()), pycompat::PyValue::Dict(py_dict(self.pmgr.get_profiles().into_iter().map(|(k, v)| (k, val_to_py(v))).collect())));
        if let Some(mesh) = self.z_mesh.borrow().clone() {
            let mut cm: IndexMap<pycompat::PyValue, pycompat::PyValue> = IndexMap::new();
            cm.insert(pycompat::PyValue::Str("name".into()), pycompat::PyValue::Str(mesh.get_profile_name()));
            cm.insert(pycompat::PyValue::Str("probed_matrix".into()), matrix_to_py(&mesh.get_probed_matrix()));
            cm.insert(pycompat::PyValue::Str("mesh_matrix".into()), matrix_to_py(&mesh.get_mesh_matrix()));
            cm.insert(pycompat::PyValue::Str("mesh_params".into()), pycompat::PyValue::Dict(py_dict(mesh.get_mesh_params())));
            result.insert(pycompat::PyValue::Str("current_mesh".into()), pycompat::PyValue::Dict(cm));
        }
        let mesh_args = web_request.get_dict("mesh_args", Some(pycompat::PyValue::Dict(IndexMap::new())))?;
        let has_args = match &mesh_args {
            pycompat::PyValue::Dict(d) => !d.is_empty(),
            pycompat::PyValue::List(l) => !l.is_empty(),
            _ => false,
        };
        let mut gcmd_opt: Option<klippy_api::gcode::GCmd> = None;
        if has_args {
            let mut params: IndexMap<String, String> = IndexMap::new();
            if let pycompat::PyValue::Dict(d) = &mesh_args {
                for (k, v) in d.iter() {
                    params.insert(k.str(), v.str());
                }
            }
            gcmd_opt = Some(self.gcode.create_gcode_command("", "", &params));
        }
        let calibration = if gcmd_opt.is_some() {
            // `with self.gcode.get_mutex():` (`bed_mesh.py:311`): unlocked
            // before an exception propagates.
            let mutex = self.gcode.get_mutex();
            mutex.lock();
            let calibration = self.bmc.dump_calibration(gcmd_opt.as_ref());
            mutex.unlock();
            calibration?
        } else {
            self.bmc.dump_calibration(None)?
        };
        result.insert(pycompat::PyValue::Str("calibration".into()), calibration);
        let offsets = match &prb {
            None => pycompat::PyValue::List(vec![pycompat::PyValue::Int(0i64.into()), pycompat::PyValue::Int(0i64.into()), pycompat::PyValue::Int(0i64.into())]),
            Some(o) => {
                let p = klippy_api::require_interface::<dyn PrinterProbeApi>(o, "probe")?;
                let offs = p.get_offsets(gcmd_opt.as_ref())?;
                pycompat::PyValue::List([offs.0, offs.1, offs.2].into_iter().map(pycompat::PyValue::Float).collect())
            }
        };
        result.insert(pycompat::PyValue::Str("probe_offsets".into()), offsets);
        let (axis_min, axis_max) = py_dict_get_pair(&th_sts);
        result.insert(pycompat::PyValue::Str("axis_minimum".into()), axis_min);
        result.insert(pycompat::PyValue::Str("axis_maximum".into()), axis_max);
        web_request.send(pycompat::PyValue::Dict(result))
    }

    pub fn handle_get_mesh_request(&self, web_request: &klippy_api::webhooks::WebRequest) -> KResult<()> {
        let probed = match self.status.borrow().clone() {
            pycompat::PyValue::Dict(ref d) => d.get(&pycompat::PyValue::Str("probed_matrix".into())).cloned().unwrap_or(pycompat::PyValue::List(vec![pycompat::PyValue::List(vec![])])),
            _ => pycompat::PyValue::List(vec![pycompat::PyValue::List(vec![])]),
        };
        let mut out: IndexMap<pycompat::PyValue, pycompat::PyValue> = IndexMap::new();
        out.insert(pycompat::PyValue::Str("probed_matrix".into()), probed);
        web_request.send(pycompat::PyValue::Dict(out))
    }
}

fn val_to_py(v: IndexMap<String, pycompat::PyValue>) -> pycompat::PyValue {
    pycompat::PyValue::Dict(py_dict(v))
}

fn py_dict_get_pair(v: &pycompat::PyValue) -> (pycompat::PyValue, pycompat::PyValue) {
    match v {
        pycompat::PyValue::Dict(d) => (
            d.get(&pycompat::PyValue::Str("axis_minimum".into())).cloned().unwrap_or(pycompat::PyValue::None),
            d.get(&pycompat::PyValue::Str("axis_maximum".into())).cloned().unwrap_or(pycompat::PyValue::None),
        ),
        _ => (pycompat::PyValue::None, pycompat::PyValue::None),
    }
}

fn lookup_gcode_move(printer: &Printer) -> KResult<klippy_api::gcode::GCodeMove> {
    match printer.lookup_object("gcode_move", LookupDefault::Raise)? {
        None => Err(KlippyError::config("gcode_move object not found")),
        Some(o) => Ok(klippy_api::require_interface::<dyn klippy_api::gcode::GCodeMoveApi>(&o, "gcode_move")?),
    }
}

/// `BedMesh` printer-object surface used by `BedMeshCalibrate`
/// (`self.bedmesh`) and `ProfileManager` (`self.bedmesh`) inside this
/// module. The methods keep their Python names; `probe_finalize` is the
/// finalize callback target registered with `ProbePointsHelper`.
pub trait BedMeshApi: Send + Sync + 'static {
    fn get_mesh(&self) -> Option<Arc<ZMesh>>;
    fn set_mesh(&self, mesh: Option<Arc<ZMesh>>) -> KResult<()>;
    fn update_status(&self);
    fn save_profile(&self, name: &str) -> KResult<()>;
    fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<()>;
}

impl BedMeshApi for BedMesh {
    fn get_mesh(&self) -> Option<Arc<ZMesh>> {
        self.get_mesh()
    }
    fn set_mesh(&self, mesh: Option<Arc<ZMesh>>) -> KResult<()> {
        self.set_mesh(mesh)
    }
    fn update_status(&self) {
        self.update_status()
    }
    fn save_profile(&self, name: &str) -> KResult<()> {
        self.pmgr.save_profile(name)
    }
    fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<()> {
        self.bmc.probe_finalize(positions)
    }
}

impl PrinterObject for BedMesh {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn BedMeshApi, dyn klippy_api::gcode::MoveTransformApi);
    fn get_status(&self, _eventtime: f64) -> KResult<Option<pycompat::PyValue>> {
        Ok(Some(self.get_status_value()))
    }
}

impl klippy_api::gcode::MoveTransformApi for BedMesh {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.move_pos(newpos, speed)
    }
    fn get_position(&self) -> KResult<Vec<f64>> {
        self.get_position()
    }
}

impl BedMesh {

    pub fn save_profile(&self, name: &str) -> KResult<()> {
        self.pmgr.save_profile(name)
    }
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let bm = Arc::new(BedMesh::new(config)?);
    {
        let bm1 = bm.clone();
        bm.printer.register_event_handler("klippy:connect", Box::new(move |_args| {
            bm1.handle_connect()?;
            Ok(pycompat::PyValue::None)
        }));
    }
    {
        let bm1 = bm.clone();
        bm.gcode.register_command("BED_MESH_OUTPUT", Some(Box::new(move |gcmd| bm1.cmd_bed_mesh_output(gcmd))), RegisterCommandOpts { desc: Some("Retrieve interpolated grid of probed z-points".into()), ..Default::default() })?;
    }
    {
        let bm1 = bm.clone();
        bm.gcode.register_command("BED_MESH_MAP", Some(Box::new(move |gcmd| bm1.cmd_bed_mesh_map(gcmd))), RegisterCommandOpts { desc: Some("Serialize mesh and output to terminal".into()), ..Default::default() })?;
    }
    {
        let bm1 = bm.clone();
        bm.gcode.register_command("BED_MESH_CLEAR", Some(Box::new(move |gcmd| bm1.cmd_bed_mesh_clear(gcmd))), RegisterCommandOpts { desc: Some("Clear the Mesh so no z-adjustment is made".into()), ..Default::default() })?;
    }
    {
        let bm1 = bm.clone();
        bm.gcode.register_command("BED_MESH_OFFSET", Some(Box::new(move |gcmd| bm1.cmd_bed_mesh_offset(gcmd))), RegisterCommandOpts { desc: Some("Add X/Y offsets to the mesh lookup".into()), ..Default::default() })?;
    }
    {
        let bm1 = bm.clone();
        bm.bmc.gcode.register_command("BED_MESH_CALIBRATE", Some(Box::new(move |gcmd| bm1.bmc.cmd_bed_mesh_calibrate(gcmd))), RegisterCommandOpts { desc: Some("Perform Mesh Bed Leveling".into()), ..Default::default() })?;
    }
    {
        let bm1 = bm.clone();
        bm.gcode.register_command("BED_MESH_PROFILE", Some(Box::new(move |gcmd| bm1.pmgr.cmd_bed_mesh_profile(gcmd))), RegisterCommandOpts { desc: Some("Bed Mesh Persistent Storage management".into()), ..Default::default() })?;
    }
    // Register dump webhooks.
    if let Ok(Some(wh_obj)) = bm.printer.lookup_object("webhooks", LookupDefault::Raise) {
        if let Ok(wh) = klippy_api::require_interface::<dyn klippy_api::webhooks::WebHooksApi>(&wh_obj, "webhooks") {
            let bm1 = bm.clone();
            wh.register_endpoint("bed_mesh/dump_mesh", Arc::new(move |req: klippy_api::webhooks::WebRequest| bm1.handle_dump_request(&req)))?;
            let bm1 = bm.clone();
            wh.register_endpoint("get_mesh", Arc::new(move |req: klippy_api::webhooks::WebRequest| bm1.handle_get_mesh_request(&req)))?;
        }
    }
    // Register transform on gcode_move.
    let gmove = lookup_gcode_move(&bm.printer)?;
    gmove.set_move_transform(Some(MoveTransform::Custom(bm.clone())), false)?;
    Ok(Some(bm as ObjectRef))
}

# [allow(dead_code)]
type MoveTransformHandle = Arc<dyn klippy_api::gcode::MoveTransformApi>;

/// Module entry point for the `bed_mesh` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
