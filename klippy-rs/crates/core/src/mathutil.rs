//! Port of the `mathutil` module (`klippy/mathutil.py`).
//!
//! Plain numeric helpers: coordinate descent (foreground and in a
//! background worker), trilateration, 3-vector/matrix arithmetic and
//! Gaussian elimination for least-squares solves. There are no printer
//! objects, config options or G-code commands, so [`module_factory`]
//! mirrors `mathutil.py` having no `load_config` /
//! `add_printer_objects`.

use indexmap::IndexMap;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::GCodeApi;
use klippy_api::log;
use klippy_api::printer::{LookupDefault, PrinterApi};
use pycompat::text::{float_pow, float_true_div, percent_format, py_sum_floats, PctValue, PyError};
use pycompat::PyInt;

/// A 3-vector (`mathutil.py` list-of-3 helpers).
pub type Vector3 = [f64; 3];
/// An NxM matrix (`mathutil.py` list-of-rows helpers).
pub type Matrix = Vec<Vec<f64>>;

/// Ordered parameter dict (`dict(params)` keeps insertion order).
pub type DescentParams = IndexMap<String, f64>;

fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
}

fn index_error() -> KlippyError {
    KlippyError::Other {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

fn value_error<T>(message: &str) -> KResult<T> {
    Err(KlippyError::Other { class_name: "ValueError".to_string(), message: message.to_string() })
}

fn type_error<T>(message: &str) -> KResult<T> {
    Err(KlippyError::Other { class_name: "TypeError".to_string(), message: message.to_string() })
}

/// `math.sqrt` (`mathutil.py` `math.sqrt(...)`).
///
/// CPython raises `ValueError: math domain error` for negative input
/// instead of returning the libm `NaN`, so the check happens here.
fn math_sqrt(x: f64) -> KResult<f64> {
    if x < 0. {
        return value_error("math domain error");
    }
    Ok(pycompat::libm::sqrt(x))
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_format(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant mathutil format")
}

/// `sum(map(operator.mul, xs, ys))`: left-to-right products, truncated
/// to the shorter sequence (Python `zip` semantics inside `map`).
fn sum_products(xs: &[f64], ys: &[f64]) -> f64 {
    let n = xs.len().min(ys.len());
    let mut acc = 0.0f64;
    for i in 0..n {
        acc += xs[i] * ys[i];
    }
    acc
}

/// Python `max` over a non-empty list (first maximal wins), and
/// `list.index` of the result (identity-free `==` scan).
fn list_index_of_max(xs: &[f64]) -> KResult<usize> {
    let mut best = xs[0];
    for &x in &xs[1..] {
        if x > best {
            best = x;
        }
    }
    for (i, &x) in xs.iter().enumerate() {
        if x == best {
            return Ok(i);
        }
    }
    value_error(&format!("{} is not in list", pycompat::text::float_repr(best)))
}

/// `matrix_cross` (`mathutil.py:120`).
pub fn matrix_cross(m1: &Vector3, m2: &Vector3) -> Vector3 {
    [
        m1[1] * m2[2] - m1[2] * m2[1],
        m1[2] * m2[0] - m1[0] * m2[2],
        m1[0] * m2[1] - m1[1] * m2[0],
    ]
}

/// `matrix_dot` (`mathutil.py:125`).
pub fn matrix_dot(m1: &Vector3, m2: &Vector3) -> f64 {
    m1[0] * m2[0] + m1[1] * m2[1] + m1[2] * m2[2]
}

/// `matrix_magsq` (`mathutil.py:128`); `** 2` is CPython `float.__pow__`.
pub fn matrix_magsq(m1: &Vector3) -> KResult<f64> {
    Ok(float_pow(m1[0], 2.).map_err(map_py_error)?
        + float_pow(m1[1], 2.).map_err(map_py_error)?
        + float_pow(m1[2], 2.).map_err(map_py_error)?)
}

/// `matrix_add` (`mathutil.py:131`).
pub fn matrix_add(m1: &Vector3, m2: &Vector3) -> Vector3 {
    [m1[0] + m2[0], m1[1] + m2[1], m1[2] + m2[2]]
}

/// `matrix_sub` (`mathutil.py:134`).
pub fn matrix_sub(m1: &Vector3, m2: &Vector3) -> Vector3 {
    [m1[0] - m2[0], m1[1] - m2[1], m1[2] - m2[2]]
}

/// `matrix_mul` (`mathutil.py:137`).
pub fn matrix_mul(m1: &Vector3, s: f64) -> Vector3 {
    [m1[0] * s, m1[1] * s, m1[2] * s]
}

/// `mat_transp` (`mathutil.py:145`). `a[0]` raises `IndexError` on an
/// empty matrix, exactly as in Python.
pub fn mat_transp(a: &Matrix) -> KResult<Matrix> {
    let cols = a.first().ok_or_else(index_error)?.len();
    let mut out = Vec::with_capacity(cols);
    for i in 0..cols {
        let mut row = Vec::with_capacity(a.len());
        for r in a {
            row.push(*r.get(i).ok_or_else(index_error)?);
        }
        out.push(row);
    }
    Ok(out)
}

/// `mat_mat_mul` (`mathutil.py:150`); returns `None` when the inner
/// dimensions disagree, otherwise raises on malformed inputs first.
pub fn mat_mat_mul(a: &Matrix, b: &Matrix) -> KResult<Option<Matrix>> {
    let cols = a.first().ok_or_else(index_error)?.len();
    if cols != b.len() {
        return Ok(None);
    }
    let bt = mat_transp(b)?;
    let mut out = Vec::with_capacity(a.len());
    for a_i in a {
        let mut row = Vec::with_capacity(bt.len());
        for bt_j in &bt {
            row.push(sum_products(a_i, bt_j));
        }
        out.push(row);
    }
    Ok(Some(out))
}

/// `mat_mul_transp` (`mathutil.py:159`): `a * transpose(a)` computed on
/// the lower triangle only, mirrored into the upper triangle.
pub fn mat_mul_transp(a: &Matrix) -> KResult<Matrix> {
    let mut res: Matrix = Vec::with_capacity(a.len());
    for (i, a_i) in a.iter().enumerate() {
        let mut row = Vec::with_capacity(i + 1);
        for a_j in &a[..i + 1] {
            row.push(sum_products(a_i, a_j));
        }
        res.push(row);
    }
    for i in 0..res.len() {
        let mut extra = Vec::with_capacity(res.len() - i - 1);
        for res_j in &res[i + 1..] {
            extra.push(*res_j.get(i).ok_or_else(index_error)?);
        }
        res[i].extend(extra.iter().copied());
    }
    Ok(res)
}

/// `gaussian_solve` (`mathutil.py:169`).
///
/// Backward (high-to-low) Gaussian elimination with partial pivoting
/// (first maximum of the absolute column wins), then a back-substitution
/// through the upper-triangular rows. Returns `None` on a singular
/// system unless `allow_underdetermined` zeroes the row instead.
/// Row updates truncate like Python `zip`/`slice`, so ragged inputs
/// keep Python's shortened rows.
pub fn gaussian_solve(
    a: &Matrix,
    rhs: &Matrix,
    allow_underdetermined: bool,
) -> KResult<Option<Matrix>> {
    let mut res: Matrix = rhs.clone();
    let mut m: Matrix = a.clone();
    let rows_m = m.len();

    for ii in (0..rows_m).rev() {
        // Find a pivot and swap the corresponding rows.
        let mut abs_col = Vec::with_capacity(ii + 1);
        for m_j in &m[..ii + 1] {
            abs_col.push(pycompat::libm::fabs(*m_j.get(ii).ok_or_else(index_error)?));
        }
        let j = list_index_of_max(&abs_col)?;
        if ii != j {
            m.swap(ii, j);
            res.get(ii).ok_or_else(index_error)?;
            res.get(j).ok_or_else(index_error)?;
            res.swap(ii, j);
        }

        // Scale the i-th row (and drop the last column).
        let m_i = m[ii].clone();
        let pivot = *m_i.get(ii).ok_or_else(index_error)?;
        let recipr = if pycompat::libm::fabs(pivot) < 1e-10 {
            if !allow_underdetermined {
                return Ok(None);
            }
            0.
        } else {
            float_true_div(1., pivot).map_err(map_py_error)?
        };
        let m_i_len = m_i.len().min(ii);
        let new_m_i: Vec<f64> = m_i[..m_i_len].iter().map(|v| v * recipr).collect();
        m[ii] = new_m_i.clone();
        let res_row = res.get(ii).ok_or_else(index_error)?.clone();
        let res_i: Vec<f64> = res_row.iter().map(|v| v * recipr).collect();
        res[ii] = res_i.clone();
        // Zero-out the last column in rows prior to i, and remove it.
        for jj in 0..ii {
            let m_j = m[jj].clone();
            let c = *m_j.get(ii).ok_or_else(index_error)?;
            let n = m_j.len().min(new_m_i.len());
            m[jj] = (0..n).map(|k| m_j[k] - c * new_m_i[k]).collect();
            let res_j = res.get(jj).ok_or_else(index_error)?.clone();
            let n = res_j.len().min(res_i.len());
            res[jj] = (0..n).map(|k| res_j[k] - c * res_i[k]).collect();
        }
    }

    // Solve the system with the lower-triangular matrix.
    let mut rest = mat_transp(&res)?;
    if rest.is_empty() {
        return Ok(Some(res));
    }
    for rest_k in rest.iter_mut() {
        for ii in 1..rows_m {
            let prod = sum_products(&m[ii], &rest_k[..ii.min(rest_k.len())]);
            let cur = *rest_k.get(ii).ok_or_else(index_error)?;
            rest_k[ii] = cur - prod;
        }
    }
    Ok(Some(mat_transp(&rest)?))
}

/// `pseudo_inverse` (`mathutil.py:210`).
pub fn pseudo_inverse(m: &Matrix) -> KResult<Option<Matrix>> {
    let mt = mat_transp(m)?;
    let mtm = mat_mul_transp(&mt)?;
    gaussian_solve(&mtm, &mt, false)
}
/// `solve_linear_equations` (`mathutil.py:216`): least-squares solution
/// via the normal equations. A dimension mismatch in the product
/// surfaces as Python's `TypeError` (`list(None)`).
pub fn solve_linear_equations(
    eqs: &Matrix,
    ans: &Matrix,
    allow_underdetermined: bool,
) -> KResult<Option<Matrix>> {
    let eqst = mat_transp(eqs)?;
    let eqst_eqs = mat_mul_transp(&eqst)?;
    let eqst_ans = mat_mat_mul(&eqst, ans)?;
    let eqst_ans = match eqst_ans {
        Some(v) => v,
        None => return type_error("'NoneType' object is not iterable"),
    };
    gaussian_solve(&eqst_eqs, &eqst_ans, allow_underdetermined)
}

/// `trilateration` (`mathutil.py:93`): intersection of three spheres.
pub fn trilateration(sphere_coords: &[Vector3; 3], radius2: &Vector3) -> KResult<Vector3> {
    let (sphere_coord1, sphere_coord2, sphere_coord3) =
        (sphere_coords[0], sphere_coords[1], sphere_coords[2]);
    let s21 = matrix_sub(&sphere_coord2, &sphere_coord1);
    let s31 = matrix_sub(&sphere_coord3, &sphere_coord1);

    let d = math_sqrt(matrix_magsq(&s21)?)?;
    let ex = matrix_mul(&s21, float_true_div(1., d).map_err(map_py_error)?);
    let i = matrix_dot(&ex, &s31);
    let vect_ey = matrix_sub(&s31, &matrix_mul(&ex, i));
    let ey = matrix_mul(&vect_ey, float_true_div(1., math_sqrt(matrix_magsq(&vect_ey)?)?).map_err(map_py_error)?);
    let ez = matrix_cross(&ex, &ey);
    let j = matrix_dot(&ey, &s31);

    let x = float_true_div(
        radius2[0] - radius2[1] + float_pow(d, 2.).map_err(map_py_error)?,
        2. * d,
    )
    .map_err(map_py_error)?;
    let y = float_true_div(
        radius2[0] - radius2[2] - float_pow(x, 2.).map_err(map_py_error)?
            + float_pow(x - i, 2.).map_err(map_py_error)?
            + float_pow(j, 2.).map_err(map_py_error)?,
        2. * j,
    )
    .map_err(map_py_error)?;
    let z = -math_sqrt(
        radius2[0] - float_pow(x, 2.).map_err(map_py_error)?
            - float_pow(y, 2.).map_err(map_py_error)?,
    )?;

    let ex_x = matrix_mul(&ex, x);
    let ey_y = matrix_mul(&ey, y);
    let ez_z = matrix_mul(&ez, z);
    Ok(matrix_add(&sphere_coord1, &matrix_add(&ex_x, &matrix_add(&ey_y, &ez_z))))
}

/// `coordinate_descent` (`mathutil.py:16`).
///
/// `error_func` receives the current candidate params (an ordered dict)
/// and returns its error; Python exceptions surface as the `Err` value,
/// mirroring exception propagation out of the Python loop. The Python
/// code permits `error_func` to mutate its dict argument; no klippy
/// caller does, so the port passes a read-only view.
pub fn coordinate_descent<F>(
    adj_params: &[String],
    params: DescentParams,
    mut error_func: F,
) -> KResult<DescentParams>
where
    F: FnMut(&DescentParams) -> KResult<f64>,
{
    let mut params = params;
    let mut dp: IndexMap<String, f64> = IndexMap::new();
    for name in adj_params {
        dp.insert(name.clone(), 1.);
    }

    let mut best_err = error_func(&params)?;
    log::info(&pct_format(
        "Coordinate descent initial error: %s",
        &[PctValue::Float(best_err)],
    ));

    let threshold = 0.00001f64;
    let mut rounds: i64 = 0;
    while py_sum_floats(&dp.values().copied().collect::<Vec<_>>()) > threshold && rounds < 10000 {
        rounds += 1;
        for param_name in adj_params {
            let orig = match params.get(param_name) {
                Some(v) => *v,
                None => {
                    return Err(KlippyError::Other {
                        class_name: "KeyError".to_string(),
                        message: pycompat::text::str_repr(param_name),
                    });
                }
            };
            params.insert(param_name.clone(), orig + dp[param_name]);
            let err = error_func(&params)?;
            if err < best_err {
                // There was some improvement
                best_err = err;
                let v = dp[param_name];
                dp.insert(param_name.clone(), v * 1.1);
                continue;
            }
            params.insert(param_name.clone(), orig - dp[param_name]);
            let err = error_func(&params)?;
            if err < best_err {
                // There was some improvement
                best_err = err;
                let v = dp[param_name];
                dp.insert(param_name.clone(), v * 1.1);
                continue;
            }
            params.insert(param_name.clone(), orig);
            let v = dp[param_name];
            dp.insert(param_name.clone(), v * 0.9);
        }
    }
    log::info(&pct_format(
        "Coordinate descent best_err: %s  rounds: %d",
        &[PctValue::Float(best_err), pct_int(rounds)],
    ));
    Ok(params)
}

/// `background_coordinate_descent` (`mathutil.py:53`).
///
/// Python forks a daemon `multiprocessing.Process` and polls it with
/// `reactor.pause(eventtime + .1)`, reporting "Working on calibration..."
/// through `gcode.respond_info(..., log=False)` every 5 s. The port runs
/// the descent on a `kos` thread and keeps the identical polling loop
/// over `reactor.pause`; the result crosses back through a `kos`
/// channel (the pipe half). The Python child credits the worker with
/// `queuelogger.clear_bg_logging()` first; the port has no separate
/// logging process to detach, so that call is the documented mapping
/// gap (calibration log lines now originate from the worker thread).
///
/// Python's `raise Exception("Error in coordinate descent: %s" % res)`
/// carries the child's `traceback.format_exc()`; the port has no Python
/// frame stack (parity/normalize replaces `Traceback ...` blocks), so
/// the worker renders `{Class}: {message}` instead.
pub fn background_coordinate_descent<F>(
    printer: &dyn PrinterApi,
    adj_params: &[String],
    params: DescentParams,
    error_func: F,
) -> KResult<DescentParams>
where
    F: FnMut(&DescentParams) -> KResult<f64> + Send + 'static,
{
    let (tx, rx) = kos::sync::channel::<(bool, Result<DescentParams, String>)>();
    let alive_tx = kos::sync::Mutex::new(true);
    let alive_flag = std::sync::Arc::new(alive_tx);
    let worker_flag = std::sync::Arc::clone(&alive_flag);
    let adj_params = adj_params.to_vec();
    let calc = kos::thread::spawn(move || {
        let res = coordinate_descent(&adj_params, params, error_func);
        match res {
            Ok(v) => tx.send((false, Ok(v))),
            Err(e) => tx.send((true, Err(format!("{}: {}", e.class_name(), e.message())))),
        }
        *worker_flag.lock() = false;
    });
    // Start a process to perform the calculation
    let reactor = printer.get_reactor();
    let gcode = klippy_api::lookup_interface::<dyn GCodeApi>(printer, "gcode", LookupDefault::Raise)?
        .expect("Raise never yields None");
    let mut eventtime = reactor.monotonic();
    let mut last_report_time = eventtime;
    while *alive_flag.lock() {
        if eventtime > last_report_time + 5. {
            last_report_time = eventtime;
            gcode.respond_info("Working on calibration...", false);
        }
        eventtime = reactor.pause(eventtime + 0.1);
    }
    // Return results
    let (is_err, res) = rx.get().expect("worker always reports its result");
    if is_err {
        let res = res.expect_err("err flag carries the error text");
        return Err(KlippyError::Internal {
            class_name: "Exception".to_string(),
            message: pct_format("Error in coordinate descent: %s", &[PctValue::Str(res)]),
        });
    }
    calc.join();
    res.map_err(|s| KlippyError::Internal {
        class_name: "Exception".to_string(),
        message: pct_format("Error in coordinate descent: %s", &[PctValue::Str(s)]),
    })
}

/// The frozen `klippy/mathutil.py` module has no `load_config`,
/// `load_config_prefix` or `add_printer_objects`: extras and kinematics
/// call its functions directly.
pub fn module_factory() -> klippy_api::printer::ModuleFactory {
    klippy_api::printer::ModuleFactory::default()
}
