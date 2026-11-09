//! Parity tests for `klippy_core::mathutil` (port of `klippy/mathutil.py`).
//!
//! Every expected value comes from the live reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`): deterministic
//! generated inputs are fed to both sides and results are compared
//! bit-for-bit (floats by hex float and repr via the recording value
//! format), including exception class and message, so no expected
//! value is transcribed by hand.
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so replay fixtures do not apply; expected values come
//! from the oracle alone.

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::GCodeApi;
use klippy_api::log;
use klippy_api::object::ObjectRef;
use klippy_api::printer::{LookupDefault, PrinterApi};
use klippy_core::mathutil::{
    background_coordinate_descent, coordinate_descent, gaussian_solve, mat_mat_mul,
    mat_mul_transp, mat_transp, matrix_add, matrix_cross, matrix_dot, matrix_magsq, matrix_mul,
    matrix_sub, pseudo_inverse, solve_linear_equations, trilateration, DescentParams, Matrix,
    Vector3,
};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SETUP: &str = r#"
import mathutil
import logging as _logging

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record): log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("mathutil setup");
            Some(py)
        }
        None => {
            println!(
                "skipping mathutil oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Deterministic input generation (splitmix64; same values rendered to
// Python source so both sides run identical numbers).
// ---------------------------------------------------------------------------

struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    /// A float in (-10, 10) with fractional bits, never an integer.
    fn f64(&mut self) -> f64 {
        let n = (self.next_u64() % 200_000) as f64 - 100_000.0;
        n / 10_000.0 + 0.05
    }
}

fn vec3(r: &mut Rng) -> Vector3 {
    [r.f64(), r.f64(), r.f64()]
}

fn matrix(r: &mut Rng, rows: usize, cols: usize) -> Matrix {
    (0..rows).map(|_| (0..cols).map(|_| r.f64()).collect()).collect()
}

fn pvec(v: &Vector3) -> String {
    format!("[{:?}, {:?}, {:?}]", v[0], v[1], v[2])
}

fn pmat(m: &Matrix) -> String {
    let rows: Vec<String> = m
        .iter()
        .map(|r| format!("[{}]", r.iter().map(|x| format!("{:?}", x)).collect::<Vec<_>>().join(", ")))
        .collect();
    format!("[{}]", rows.join(", "))
}
fn pbool(b: bool) -> &'static str {
    if b { "True" } else { "False" }
}

fn vvec(v: &Vector3) -> Value {
    Value::List(v.iter().map(|x| Value::float(*x)).collect())
}

fn mval(m: &Matrix) -> Value {
    Value::List(m.iter().map(|r| Value::List(r.iter().map(|x| Value::float(*x)).collect())).collect())
}

fn opt_mval(m: &Option<Matrix>) -> Value {
    match m {
        Some(m) => mval(m),
        None => Value::Null,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "mathutil".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_exc(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    let expected = match expected {
        Err(e) => e,
        Ok(v) => panic!("{label}: oracle did not raise (returned {v:?})"),
    };
    let actual = match actual {
        Ok(v) => panic!("{label}: port did not raise (returned {v:?})"),
        Err(e) => e,
    };
    assert_eq!(actual.class_name(), expected.class_name, "{label}: exception class");
    assert_eq!(actual.message(), expected.message, "{label}: exception message");
}

// Captured log lines of the Rust side on this test's thread; the sink is
// installed once per process and parallel tests keep their own lines.
thread_local! {
    static LOGS: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        log::set_sink(Box::new(|rec: &log::Record<'_>| {
            LOGS.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
            });
        }));
    });
}

fn log_capture() {
    install_log_sink();
    LOGS.with(|logs| logs.borrow_mut().clear());
}

fn captured_logs() -> Vec<(String, String)> {
    LOGS.with(|logs| logs.borrow().clone())
}
// ---------------------------------------------------------------------------
// 3-vector helpers.
// ---------------------------------------------------------------------------

#[test]
fn vector_helpers_match_python() {
    let Some(mut py) = oracle() else { return };
    let mut r = Rng(1);
    for case in 0..40 {
        let a = vec3(&mut r);
        let b = vec3(&mut r);
        let s = r.f64();
        let label = format!("vector case {case}");
        check(
            &format!("{label} cross"),
            py.eval(&format!("mathutil.matrix_cross({}, {})", pvec(&a), pvec(&b))),
            vvec(&matrix_cross(&a, &b)),
        );
        check(
            &format!("{label} dot"),
            py.eval(&format!("mathutil.matrix_dot({}, {})", pvec(&a), pvec(&b))),
            Value::float(matrix_dot(&a, &b)),
        );
        match (matrix_magsq(&a), py.eval(&format!("mathutil.matrix_magsq({})", pvec(&a)))) {
            (Ok(v), e) => check(&format!("{label} magsq"), e, Value::float(v)),
            (Err(e), e2) => check_exc(&format!("{label} magsq"), e2, Err(e)),
        }
        check(
            &format!("{label} add"),
            py.eval(&format!("mathutil.matrix_add({}, {})", pvec(&a), pvec(&b))),
            vvec(&matrix_add(&a, &b)),
        );
        check(
            &format!("{label} sub"),
            py.eval(&format!("mathutil.matrix_sub({}, {})", pvec(&a), pvec(&b))),
            vvec(&matrix_sub(&a, &b)),
        );
        check(
            &format!("{label} mul"),
            py.eval(&format!("mathutil.matrix_mul({}, {:?})", pvec(&a), s)),
            vvec(&matrix_mul(&a, s)),
        );
    }
}

// ---------------------------------------------------------------------------
// NxM matrix helpers.
// ---------------------------------------------------------------------------

#[test]
fn matrix_helpers_match_python() {
    let Some(mut py) = oracle() else { return };
    let mut r = Rng(2);
    for case in 0..40 {
        let (rows, cols) = (1 + (r.next_u64() % 5) as usize, 1 + (r.next_u64() % 5) as usize);
        let a = matrix(&mut r, rows, cols);
        let label = format!("matrix case {case}");
        match (mat_transp(&a), py.eval(&format!("mathutil.mat_transp({})", pmat(&a)))) {
            (Ok(v), e) => check(&format!("{label} transp"), e, mval(&v)),
            (Err(e), e2) => check_exc(&format!("{label} transp"), e2, Err(e)),
        }
        let b_cols = 1 + (r.next_u64() % 3) as usize;
        let b = matrix(&mut r, cols, b_cols);
        match (mat_mat_mul(&a, &b), py.eval(&format!("mathutil.mat_mat_mul({}, {})", pmat(&a), pmat(&b)))) {
            (Ok(v), e) => check(&format!("{label} matmul"), e, opt_mval(&v)),
            (Err(e), e2) => check_exc(&format!("{label} matmul"), e2, Err(e)),
        }
        match (mat_mul_transp(&a), py.eval(&format!("mathutil.mat_mul_transp({})", pmat(&a)))) {
            (Ok(v), e) => check(&format!("{label} mul_transp"), e, mval(&v)),
            (Err(e), e2) => check_exc(&format!("{label} mul_transp"), e2, Err(e)),
        }
    }
}

#[test]
fn mat_mat_mul_dimension_mismatch_is_none() {
    let Some(mut py) = oracle() else { return };
    let a = matrix(&mut Rng(3), 2, 3);
    let b = matrix(&mut Rng(4), 2, 2);
    match (mat_mat_mul(&a, &b), py.eval(&format!("mathutil.mat_mat_mul({}, {})", pmat(&a), pmat(&b)))) {
        (Ok(v), e) => check("matmul mismatch", e, opt_mval(&v)),
        (Err(e), e2) => check_exc("matmul mismatch", e2, Err(e)),
    }
    // Empty first argument raises IndexError on `a[0]`.
    check_exc(
        "matmul empty a",
        py.eval("mathutil.mat_mat_mul([], [[1.0]])"),
        mat_mat_mul(&Vec::new(), &vec![vec![1.0]]).map(|v| opt_mval(&v)),
    );
    // Empty second argument: `len(a[0]) != len(b)` returns `None`.
    let b_empty = mat_mat_mul(&vec![vec![1.0]], &Vec::new());
    match (b_empty, py.eval("mathutil.mat_mat_mul([[1.0]], [])")) {
        (Ok(v), e) => check("matmul empty b", e, opt_mval(&v)),
        (Err(e), e2) => check_exc("matmul empty b", e2, Err(e)),
    }
}

// ---------------------------------------------------------------------------
// Gaussian solve, pseudo inverse, least squares.
// ---------------------------------------------------------------------------

#[test]
fn gaussian_solve_matches_python() {
    let Some(mut py) = oracle() else { return };
    let mut r = Rng(5);
    for case in 0..40 {
        let n = 1 + (r.next_u64() % 4) as usize;
        let a = matrix(&mut r, n, n);
        let rhs = matrix(&mut r, n, 2);
        let allow = case % 2 == 0;
        let label = format!("gaussian case {case}");
        let expr = format!(
            "mathutil.gaussian_solve({}, {}, {})",
            pmat(&a),
            pmat(&rhs),
            pbool(allow)
        );
        match (gaussian_solve(&a, &rhs, allow), py.eval(&expr)) {
            (Ok(v), e) => check(&label, e, opt_mval(&v)),
            (Err(e), e2) => check_exc(&label, e2, Err(e)),
        }
    }
}

#[test]
fn gaussian_solve_singular_and_underdetermined() {
    let Some(mut py) = oracle() else { return };
    // Singular: duplicate row.
    let a = vec![vec![1.5, -2.25], vec![1.5, -2.25], vec![0.5, 4.0]];
    let rhs = vec![vec![1.0, 2.0], vec![1.0, 2.0], vec![3.0, -1.0]];
    for allow in [false, true] {
        let expr = format!("mathutil.gaussian_solve({}, {}, {})", pmat(&a), pmat(&rhs), pbool(allow));
        match (gaussian_solve(&a, &rhs, allow), py.eval(&expr)) {
            (Ok(v), e) => check(&format!("singular allow={allow}"), e, opt_mval(&v)),
            (Err(e), e2) => check_exc(&format!("singular allow={allow}"), e2, Err(e)),
        }
    }
    // All-zero system.
    let a = vec![vec![0.0, 0.0], vec![0.0, 0.0]];
    let rhs = vec![vec![1.0], vec![2.0]];
    for allow in [false, true] {
        let expr = format!("mathutil.gaussian_solve({}, {}, {})", pmat(&a), pmat(&rhs), pbool(allow));
        match (gaussian_solve(&a, &rhs, allow), py.eval(&expr)) {
            (Ok(v), e) => check(&format!("zero allow={allow}"), e, opt_mval(&v)),
            (Err(e), e2) => check_exc(&format!("zero allow={allow}"), e2, Err(e)),
        }
    }
    // Empty system raises IndexError inside mat_transp (Python evaluates
    // `rest = mat_transp(res)` before the emptiness check).
    check_exc(
        "gaussian empty",
        py.eval("mathutil.gaussian_solve([], [])"),
        gaussian_solve(&Vec::new(), &Vec::new(), false).map(|v| opt_mval(&v)),
    );
}

#[test]
fn pseudo_inverse_and_least_squares_match_python() {
    let Some(mut py) = oracle() else { return };
    let mut r = Rng(7);
    for case in 0..30 {
        let (rows, cols) = (1 + (r.next_u64() % 4) as usize, 1 + (r.next_u64() % 4) as usize);
        let m = matrix(&mut r, rows, cols);
        let label = format!("pinv case {case}");
        match (pseudo_inverse(&m), py.eval(&format!("mathutil.pseudo_inverse({})", pmat(&m)))) {
            (Ok(v), e) => check(&label, e, opt_mval(&v)),
            (Err(e), e2) => check_exc(&label, e2, Err(e)),
        }
    }
    for case in 0..30 {
        let (n_eqs, n_vars) = (2 + (r.next_u64() % 4) as usize, 1 + (r.next_u64() % 4) as usize);
        let eqs = matrix(&mut r, n_eqs, n_vars);
        let ans = matrix(&mut r, n_eqs, 1);
        let allow = case % 2 == 1;
        let label = format!("leastsq case {case}");
        let expr = format!(
            "mathutil.solve_linear_equations({}, {}, {})",
            pmat(&eqs),
            pmat(&ans),
            pbool(allow)
        );
        match (solve_linear_equations(&eqs, &ans, allow), py.eval(&expr)) {
            (Ok(v), e) => check(&label, e, opt_mval(&v)),
            (Err(e), e2) => check_exc(&label, e2, Err(e)),
        }
    }
    // Dimension mismatch: ans rows (2) != eqs rows (3) leaves the
    // product's inner dimensions disagreeing, so Python `list(None)`
    // raises TypeError.
    let eqs = matrix(&mut Rng(9), 3, 2);
    let ans = matrix(&mut Rng(10), 2, 1);
    check_exc(
        "leastsq mismatch",
        py.eval(&format!(
            "mathutil.solve_linear_equations({}, {}, False)",
            pmat(&eqs),
            pmat(&ans)
        )),
        solve_linear_equations(&eqs, &ans, false).map(|v| opt_mval(&v)),
    );
}

// ---------------------------------------------------------------------------
// Trilateration.
// ---------------------------------------------------------------------------

#[test]
fn trilateration_matches_python() {
    let Some(mut py) = oracle() else { return };
    let mut r = Rng(11);
    for case in 0..40 {
        // Well-separated sphere centers (like delta towers), radii drawn
        // from the 3 points' distances plus an offset so the sqrt
        // argument stays non-negative.
        let p1 = vec3(&mut r);
        let p2 = [p1[0] + 5.0 + r.f64().abs(), p1[1] + r.f64(), p1[2] + r.f64()];
        let p3 = [
            p1[0] + r.f64(),
            p1[1] + 5.0 + r.f64().abs(),
            p1[2] + r.f64(),
        ];
        let coords = [p1, p2, p3];
        let radius2: Vector3 = [25.0 + r.f64() * r.f64(), 30.0 + r.f64() * r.f64(), 40.0 + r.f64() * r.f64()];
        let label = format!("trilateration case {case}");
        let expr = format!(
            "mathutil.trilateration([{}, {}, {}], {})",
            pvec(&p1),
            pvec(&p2),
            pvec(&p3),
            pvec(&radius2)
        );
        match (trilateration(&coords, &radius2), py.eval(&expr)) {
            (Ok(v), e) => check(&label, e, vvec(&v)),
            (Err(e), e2) => check_exc(&label, e2, Err(e)),
        }
    }
    // Degenerate: coincident centers -> d == 0 -> float division by zero.
    let p = [1.0, 2.0, 3.0];
    let coords = [p, p, p];
    check_exc(
        "trilateration d=0",
        py.eval("mathutil.trilateration([[1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [1.0, 2.0, 3.0]], [9.0, 9.0, 9.0])"),
        trilateration(&coords, &[9.0, 9.0, 9.0]).map(|v| vvec(&v)),
    );
    // sqrt domain error: radii too small for the computed center.
    check_exc(
        "trilateration sqrt domain",
        py.eval("mathutil.trilateration([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]], [0.1, 0.1, 0.1])"),
        trilateration(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]], &[0.1, 0.1, 0.1]).map(|v| vvec(&v)),
    );
}

// ---------------------------------------------------------------------------
// Coordinate descent (foreground).
// ---------------------------------------------------------------------------

fn params_map(pairs: &[(&str, f64)]) -> DescentParams {
    let mut m = IndexMap::new();
    for (k, v) in pairs {
        m.insert((*k).to_string(), *v);
    }
    m
}

fn params_value(p: &DescentParams) -> Value {
    Value::Dict(
        p.iter()
            .map(|(k, v)| (Value::str(k), Value::float(*v)))
            .collect(),
    )
}

/// Human-readable p_expr list, e.g. `['a', 'b']` or `{}` variants.
fn p_names(names: &[String]) -> String {
    format!("[{}]", names.iter().map(|n| format!("{:?}", n)).collect::<Vec<_>>().join(", "))
}

fn p_dict(p: &DescentParams) -> String {
    let items: Vec<String> = p
        .iter()
        .map(|(k, v)| format!("{:?}: {:?}", k, v))
        .collect();
    format!("{{{}}}", items.join(", "))
}

#[test]
fn coordinate_descent_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    py.exec("log_lines.clear()").expect("clear py log");

    let names = vec!["a".to_string(), "b".to_string(), "c".to_string()];
    let params = params_map(&[("a", 0.5), ("b", -1.5), ("c", 2.25)]);
    // Quadratic bowl with minimum at (0.5, -1.0, 1.0); the 1.1 growth
    // overshoots, so this exercises both growth and shrink branches.
    py.exec(
        "def ef(params):\n    return (params['a']-0.5)**2 + 2.0*(params['b']+1.0)**2 + (params['c']-1.0)**2",
    )
    .expect("ef");
    // libm pow, exactly like Python `** 2`, so both error functions
    // agree bit-for-bit.
    let rust_ef = |p: &DescentParams| -> KResult<f64> {
        let a = p["a"] - 0.5;
        let b = p["b"] + 1.0;
        let c = p["c"] - 1.0;
        Ok(pycompat::libm::pow(a, 2.) + 2.0 * pycompat::libm::pow(b, 2.) + pycompat::libm::pow(c, 2.))
    };
    let expr = format!(
        "mathutil.coordinate_descent({}, {}, ef)",
        p_names(&names),
        p_dict(&params)
    );
    match (coordinate_descent(&names, params.clone(), rust_ef), py.eval(&expr)) {
        (Ok(v), e) => check("descent bowl", e, params_value(&v)),
        (Err(e), e2) => check_exc("descent bowl", e2, Err(e)),
    }
    // Compare the two info log lines (Rust goes through the klippy_api
    // sink; Python through the logging capture above).
    check(
        "descent log lines",
        py.eval("list(log_lines)"),
        Value::List(
            captured_logs()
                .iter()
                .filter(|(_, m)| m.contains("Coordinate descent"))
                .map(|(_, m)| Value::str(m))
                .collect(),
        ),
    );
}

#[test]
fn coordinate_descent_edge_cases_match_python() {
    let Some(mut py) = oracle() else { return };
    // Initial error already above nothing; constant error function.
    let names = vec!["x".to_string(), "y".to_string()];
    let params = params_map(&[("x", 1.0), ("y", 2.0)]);
    let expr = format!("mathutil.coordinate_descent({}, {}, lambda p: 3.5)", p_names(&names), p_dict(&params));
    match (coordinate_descent(&names, params.clone(), |_| Ok(3.5)), py.eval(&expr)) {
        (Ok(v), e) => check("descent constant", e, params_value(&v)),
        (Err(e), e2) => check_exc("descent constant", e2, Err(e)),
    }
    // No adjustable params: while condition is immediately false.
    let names: Vec<String> = Vec::new();
    let params = params_map(&[("x", 1.0)]);
    let expr = format!("mathutil.coordinate_descent({}, {}, lambda p: 1.0)", p_names(&names), p_dict(&params));
    match (coordinate_descent(&names, params.clone(), |_| Ok(1.0)), py.eval(&expr)) {
        (Ok(v), e) => check("descent no params", e, params_value(&v)),
        (Err(e), e2) => check_exc("descent no params", e2, Err(e)),
    }
    // error_func raising: the exception propagates unchanged.
    py.exec("def bad(params):\n    raise ValueError('boom')")
        .expect("bad");
    let expr = format!("mathutil.coordinate_descent({}, {}, bad)", p_names(&names), p_dict(&params));
    let raised: KResult<DescentParams> = coordinate_descent(&names, params.clone(), |_| {
        Err(KlippyError::Other { class_name: "ValueError".to_string(), message: "boom".to_string() })
    });
    check_exc("descent raising", py.eval(&expr), raised.map(|v| params_value(&v)));
    // adj name missing from params: KeyError.
    let names = vec!["zz".to_string()];
    let expr = format!("mathutil.coordinate_descent({}, {}, lambda p: 1.0)", p_names(&names), p_dict(&params));
    let missing: KResult<DescentParams> = coordinate_descent(&names, params.clone(), |_| Ok(1.0));
    check_exc("descent missing key", py.eval(&expr), missing.map(|v| params_value(&v)));
}

// ---------------------------------------------------------------------------
// Background coordinate descent (fake printer + gcode objects).
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakeGCode {
    infos: Arc<Mutex<Vec<String>>>,
    reactor: Reactor,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode { infos: Arc::new(Mutex::new(Vec::new())), reactor: Reactor::new(false) }
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<klippy_api::gcode::GCodeCommandHandler>,
        _opts: klippy_api::gcode::RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: klippy_api::gcode::GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        Arc::new(FakeGCmd { params: params.clone() })
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, log: bool) {
        if !log {
            self.infos.lock().push(msg.to_string());
        }
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

struct FakeGCmd {
    params: IndexMap<String, String>,
}

impl klippy_api::gcode::GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, _opts: klippy_api::gcode::GetOpts) -> KResult<PyValue> {
        Ok(self.params.get(name).map(|s| PyValue::Str(s.clone())).unwrap_or(PyValue::None))
    }
    fn get_int(&self, _name: &str, _opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, _name: &str, _opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn get_command(&self) -> String {
        String::new()
    }
    fn get_commandline(&self) -> String {
        String::new()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

struct FakePrinter {
    reactor: Reactor,
    gcode: ObjectRef,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("Printer is ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match name {
            "gcode" => Ok(Some(self.gcode.clone())),
            _ => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[klippy_api::object::EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

fn fake_printer() -> (FakePrinter, Arc<FakeGCode>) {
    let gcode = Arc::new(FakeGCode::new());
    let reactor = Reactor::new(false);
    let printer = FakePrinter { reactor: reactor.clone(), gcode: gcode.clone() as ObjectRef };
    (printer, gcode)
}

#[test]
fn background_descent_matches_python() {
    let Some(mut py) = oracle() else { return };
    // Python side: a fake printer whose gcode records respond_info
    // (log=False) and a reactor that advances eventtime by 0.1 per pause.
    py.exec(
        r#"
class _FakeGCode:
    def __init__(self): self.infos = []
    def respond_info(self, msg, log=True):
        if not log: self.infos.append(msg)
class _FakeReactor:
    def __init__(self): self.t = 0.0
    def monotonic(self): return self.t
    def pause(self, waketime):
        self.t = waketime
        return waketime
class _FakePrinter:
    reactor = _FakeReactor()
    gcode = _FakeGCode()
    def get_reactor(self): return self.reactor
    def lookup_object(self, name, default=None):
        assert name == 'gcode'
        return self.gcode
def ef(params):
    return params['a']**2 + params['b']**2
"#,
    )
    .expect("py background fakes");
    let names = vec!["a".to_string(), "b".to_string()];
    let params = params_map(&[("a", 1.5), ("b", -0.5)]);
    let expected = py
        .eval("mathutil.background_coordinate_descent(_FakePrinter(), ['a', 'b'], {'a': 1.5, 'b': -0.5}, ef)")
        .expect("py background descent");
    let (printer, gcode) = fake_printer();
    let actual = background_coordinate_descent(&printer, &names, params, |p: &DescentParams| {
        Ok(pycompat::libm::pow(p["a"], 2.) + pycompat::libm::pow(p["b"], 2.))
    })
    .expect("rust background descent");
    check("background result", Ok(expected), params_value(&actual));
    // Fast computation: the Rust worker finishes during the first
    // real-time pause, so no 5 s report fires. The Python side's
    // report count is real-clock dependent under a busy-wait fake
    // reactor, so it is not compared (see test docs).
    assert!(gcode.infos.lock().is_empty(), "fast descent reports nothing");
}

#[test]
fn background_descent_error_matches_python_shape() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        r#"
class _FakeGCode:
    def __init__(self): self.infos = []
    def respond_info(self, msg, log=True):
        if not log: self.infos.append(msg)
class _FakeReactor:
    def __init__(self): self.t = 0.0
    def monotonic(self): return self.t
    def pause(self, waketime):
        self.t = waketime
        return waketime
class _FakePrinter:
    reactor = _FakeReactor()
    gcode = _FakeGCode()
    def get_reactor(self): return self.reactor
    def lookup_object(self, name, default=None):
        assert name == 'gcode'
        return self.gcode
def bad(params):
    raise RuntimeError('kaboom')
"#,
    )
    .expect("py error fakes");
    let py_result = py.eval("mathutil.background_coordinate_descent(_FakePrinter(), ['a'], {'a': 1.0}, bad)");
    let (printer, _gcode) = fake_printer();
    let rust_result = background_coordinate_descent(&printer, &["a".to_string()], params_map(&[("a", 1.0)]), |_| -> KResult<f64> {
        Err(KlippyError::Other { class_name: "RuntimeError".to_string(), message: "kaboom".to_string() })
    });
    // Both raise `Exception("Error in coordinate descent: ...")`; the
    // child traceback body is not byte-identical (the normalizer
    // rewrites Traceback blocks), so compare class and prefix/body
    // containment instead of the whole message.
    let py_exc = match py_result {
        Err(e) => e,
        Ok(v) => panic!("py did not raise: {v:?}"),
    };
    assert_eq!(py_exc.class_name, "Exception");
    let rust_err = match rust_result {
        Err(e) => e,
        Ok(v) => panic!("rust did not raise: {v:?}"),
    };
    assert_eq!(rust_err.class_name(), "Exception");
    assert!(
        rust_err.message().starts_with("Error in coordinate descent: "),
        "rust message: {}",
        rust_err.message()
    );
    assert!(
        py_exc.message.starts_with("Error in coordinate descent: "),
        "py message: {}",
        py_exc.message
    );
    assert!(
        py_exc.message.contains("RuntimeError: kaboom"),
        "py message carries child exception: {}",
        py_exc.message
    );
}
