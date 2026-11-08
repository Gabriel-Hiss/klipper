//! Port of `klippy/extras/shaper_defs.py`.

use klippy_api::error::{KResult, KlippyError};
use klippy_core::mathutil::{mat_mat_mul, mat_transp, pseudo_inverse};
use pycompat::int::PyInt;
use pycompat::libm;
use pycompat::text::{
    float_from_string, float_pow, float_true_div, int_from_string, percent_format, PctValue,
};

pub const SHAPER_VIBRATION_REDUCTION: f64 = 20.0;
pub const DEFAULT_DAMPING_RATIO: f64 = 0.1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShaperKind {
    Zv,
    Mzv,
    Zvd,
    Ei,
    TwoHumpEi,
    ThreeHumpEi,
}

#[derive(Clone, Copy, Debug)]
pub struct InputShaperCfg {
    pub name: &'static str,
    pub kind: ShaperKind,
    pub min_freq: f64,
    pub max_damping_ratio: f64,
}

impl InputShaperCfg {
    pub fn init_func(&self, shaper_freq: f64, damping_ratio: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
        match self.kind {
            ShaperKind::Zv => get_zv_shaper(shaper_freq, damping_ratio),
            ShaperKind::Mzv => get_mzv_shaper(shaper_freq, damping_ratio),
            ShaperKind::Zvd => get_zvd_shaper(shaper_freq, damping_ratio),
            ShaperKind::Ei => get_ei_shaper(shaper_freq, damping_ratio),
            ShaperKind::TwoHumpEi => get_2hump_ei_shaper(shaper_freq, damping_ratio),
            ShaperKind::ThreeHumpEi => get_3hump_ei_shaper(shaper_freq, damping_ratio),
        }
    }
}

pub const INPUT_SHAPERS: &[InputShaperCfg] = &[
    InputShaperCfg { name: "zv", kind: ShaperKind::Zv, min_freq: 21.0, max_damping_ratio: 0.99 },
    InputShaperCfg { name: "mzv", kind: ShaperKind::Mzv, min_freq: 23.0, max_damping_ratio: 0.99 },
    InputShaperCfg { name: "zvd", kind: ShaperKind::Zvd, min_freq: 29.0, max_damping_ratio: 0.99 },
    InputShaperCfg { name: "ei", kind: ShaperKind::Ei, min_freq: 29.0, max_damping_ratio: 0.4 },
    InputShaperCfg { name: "2hump_ei", kind: ShaperKind::TwoHumpEi, min_freq: 39.0, max_damping_ratio: 0.3 },
    InputShaperCfg { name: "3hump_ei", kind: ShaperKind::ThreeHumpEi, min_freq: 48.0, max_damping_ratio: 0.2 },
];

fn internal(kind: &'static str, msg: String) -> KlippyError {
    KlippyError::Internal { class_name: kind.to_string(), message: msg }
}

fn shaper_error(msg: String) -> KlippyError {
    KlippyError::Other { class_name: "ShaperError".to_string(), message: msg }
}

fn is_shaper_error(e: &KlippyError) -> bool {
    matches!(e, KlippyError::Other { class_name, .. } if class_name == "ShaperError")
}

fn map_py_error(e: pycompat::text::PyError) -> KlippyError {
    internal(e.kind, e.msg)
}

fn fmt_pct(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(map_py_error)
}

fn py_sqrt(x: f64) -> KResult<f64> {
    if x < 0.0 {
        return Err(internal("ValueError", "math domain error".to_string()));
    }
    Ok(libm::sqrt(x))
}

fn py_exp(x: f64) -> KResult<f64> {
    let r = libm::exp(x);
    if x.is_finite() && r.is_infinite() {
        return Err(internal("OverflowError", "math range error".to_string()));
    }
    Ok(r)
}

fn py_sin(x: f64) -> KResult<f64> {
    if x.is_infinite() {
        return Err(internal("ValueError", "math domain error".to_string()));
    }
    Ok(libm::sin(x))
}

fn py_cos(x: f64) -> KResult<f64> {
    if x.is_infinite() {
        return Err(internal("ValueError", "math domain error".to_string()));
    }
    Ok(libm::cos(x))
}

fn py_div(a: f64, b: f64) -> KResult<f64> {
    float_true_div(a, b).map_err(map_py_error)
}

fn py_pow2(x: f64) -> KResult<f64> {
    float_pow(x, 2.0).map_err(map_py_error)
}

fn py_pow(a: f64, b: f64) -> KResult<f64> {
    float_pow(a, b).map_err(map_py_error)
}

const PI: f64 = std::f64::consts::PI;

// --- shaper functions ---

#[derive(Clone, Debug)]
pub enum ParsedArg {
    Int(PyInt),
    Float(f64),
}

impl ParsedArg {
    fn to_float(&self) -> KResult<f64> {
        match self {
            ParsedArg::Float(f) => Ok(*f),
            ParsedArg::Int(n) => n
                .to_f64_exact()
                .ok_or_else(|| internal("OverflowError", "int too large to convert to float".to_string())),
        }
    }

    fn is_false(&self) -> bool {
        match self {
            ParsedArg::Float(f) => *f == 0.0,
            ParsedArg::Int(n) => n.is_zero(),
        }
    }

    fn pct(&self) -> PctValue {
        match self {
            ParsedArg::Int(n) => PctValue::Int(n.clone()),
            ParsedArg::Float(f) => PctValue::Float(*f),
        }
    }

    fn to_i64_for_range(&self) -> KResult<i64> {
        match self {
            ParsedArg::Float(_) => Err(internal(
                "TypeError",
                "'float' object cannot be interpreted as an integer".to_string(),
            )),
            ParsedArg::Int(n) => {
                let s = n.0.to_string();
                s.parse::<i64>().map_err(|_| {
                    internal("OverflowError", "Python int too large to convert to C ssize_t".to_string())
                })
            }
        }
    }
}

pub fn get_none_shaper() -> (Vec<f64>, Vec<f64>) {
    (Vec::new(), Vec::new())
}

pub fn get_zv_shaper(shaper_freq: f64, damping_ratio: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
    let df = py_sqrt(1.0 - py_pow2(damping_ratio)?)?;
    let k = py_exp(py_div(-damping_ratio * PI, df)?)?;
    let t_d = py_div(1.0, shaper_freq * df)?;
    Ok((vec![1.0, k], vec![0.0, 0.5 * t_d]))
}

pub fn get_zvd_shaper(shaper_freq: f64, damping_ratio: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
    let df = py_sqrt(1.0 - py_pow2(damping_ratio)?)?;
    let k = py_exp(py_div(-damping_ratio * PI, df)?)?;
    let k2 = py_pow(k, 2.0)?;
    let t_d = py_div(1.0, shaper_freq * df)?;
    Ok((vec![1.0, 2.0 * k, k2], vec![0.0, 0.5 * t_d, t_d]))
}

pub fn get_mzv_coeffs(n: i64, t: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
    get_mzv_coeffs_inner(&ParsedArg::Int(PyInt::from_i64(n)), &ParsedArg::Float(t))
}

fn get_mzv_coeffs_inner(n_arg: &ParsedArg, t_arg: &ParsedArg) -> KResult<(Vec<f64>, Vec<f64>)> {
    let n_f = n_arg.to_float()?;
    let t_f = t_arg.to_float()?;
    if n_f < 3.0 {
        let msg = fmt_pct("Too small n=%d, must be at least 3", &[n_arg.pct()])?;
        return Err(shaper_error(msg));
    }
    let limit = 2.0 * t_f + 1.0 + 1e-7;
    if n_f <= limit {
        let upper = 0.5 * (n_f - 1.0);
        let msg = fmt_pct(
            "Too large t=%.6f for n=%d, must be less than %.6f",
            &[PctValue::Float(t_f), n_arg.pct(), PctValue::Float(upper)],
        )?;
        return Err(shaper_error(msg));
    }
    let tau = py_div(t_f * (n_f - 2.0), (n_f - 2.0 * t_f) - 1.0)?;
    let n_i64 = n_arg.to_i64_for_range()?;
    let n = n_i64;
    let mut t_list: Vec<f64> = Vec::with_capacity(n as usize);
    let denom_t = n_f - 1.0;
    for i in 0..n {
        let v = py_div((i as f64) * t_f, denom_t)?;
        t_list.push(v);
    }
    let mut m: Vec<Vec<f64>> = vec![vec![1.0f64; n as usize]];
    let mut f_vec: Vec<f64> = vec![1.0];
    for i in 0..(n - 1) {
        let i_f = i as f64;
        let it = py_div(i_f, tau)?;
        let s = 1.0 + it;
        let mut w: Vec<f64> = Vec::with_capacity(n as usize);
        for tj in &t_list {
            let wv = 2.0 * PI * s * *tj;
            w.push(wv);
        }
        let mut crow: Vec<f64> = Vec::with_capacity(n as usize);
        for wv in &w {
            crow.push(py_cos(*wv)?);
        }
        m.push(crow);
        let mut srow: Vec<f64> = Vec::with_capacity(n as usize);
        for wv in &w {
            srow.push(py_sin(*wv)?);
        }
        m.push(srow);
        f_vec.push(0.0);
        f_vec.push(0.0);
    }
    let m_inv = match pseudo_inverse(&m)? {
        Some(v) => v,
        None => {
            let msg = fmt_pct(
                "Ill-formed shaper with n=%d, t=%.6f",
                &[n_arg.pct(), PctValue::Float(t_f)],
            )?;
            return Err(shaper_error(msg));
        }
    };
    let m_inv_t = mat_transp(&m_inv)?;
    let f_mat = vec![f_vec.clone()];
    let prod = mat_mat_mul(&f_mat, &m_inv_t)?.ok_or_else(|| {
        internal("TypeError", "'NoneType' object is not subscriptable".to_string())
    })?;
    let a_vec = prod[0].clone();
    if a_vec.iter().any(|a| *a < -0.00001) {
        let msg = fmt_pct(
            "Negative-valued shaper with n=%d, t=%.6f",
            &[n_arg.pct(), PctValue::Float(t_f)],
        )?;
        return Err(shaper_error(msg));
    }
    Ok((a_vec, t_list))
}

pub fn get_mzv_shaper(shaper_freq: f64, damping_ratio: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
    get_mzv_shaper_full(shaper_freq, damping_ratio, 3, 0.0, 0.0)
}

pub fn get_mzv_shaper_full(
    shaper_freq: f64,
    damping_ratio: f64,
    n: i64,
    t: f64,
    tau: f64,
) -> KResult<(Vec<f64>, Vec<f64>)> {
    get_mzv_shaper_inner(
        shaper_freq,
        damping_ratio,
        &ParsedArg::Int(PyInt::from_i64(n)),
        &ParsedArg::Float(t),
        &ParsedArg::Float(tau),
    )
}

fn get_mzv_shaper_inner(
    shaper_freq: f64,
    damping_ratio: f64,
    n_arg: &ParsedArg,
    t_in: &ParsedArg,
    tau_in: &ParsedArg,
) -> KResult<(Vec<f64>, Vec<f64>)> {
    let t_arg: ParsedArg = if t_in.is_false() && tau_in.is_false() {
        ParsedArg::Float(0.75)
    } else if !tau_in.is_false() {
        let n_f = n_arg.to_float()?;
        let tau_f = tau_in.to_float()?;
        let t_v = py_div(tau_f * (n_f - 1.0), (n_f + 2.0 * tau_f) - 2.0)?;
        ParsedArg::Float(t_v)
    } else {
        t_in.clone()
    };
    let (mut a_vec, mut t_list) = get_mzv_coeffs_inner(n_arg, &t_arg)?;
    let t_f = t_arg.to_float()?;
    let n_f = n_arg.to_float()?;
    let df = py_sqrt(1.0 - py_pow2(damping_ratio)?)?;
    let numer = -2.0 * t_f * damping_ratio * PI;
    let denom = (n_f - 1.0) * df;
    let k = py_exp(py_div(numer, denom)?)?;
    let t_d = py_div(1.0, shaper_freq * df)?;
    let n_i64 = n_arg.to_i64_for_range()?;
    let mut kp = k;
    for i in 1..n_i64 {
        let idx = i as usize;
        t_list[idx] *= t_d;
        a_vec[idx] *= kp;
        kp *= k;
    }
    Ok((a_vec, t_list))
}

pub fn get_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
    let v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION;
    get_ei_shaper_v_tol(shaper_freq, damping_ratio, v_tol)
}

fn get_ei_shaper_v_tol(shaper_freq: f64, damping_ratio: f64, v_tol: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
    let df = py_sqrt(1.0 - py_pow2(damping_ratio)?)?;
    let t_d = py_div(1.0, shaper_freq * df)?;
    let dr = damping_ratio;
    let a1 = (0.24968 + 0.24961 * v_tol)
        + ((0.80008 + 1.23328 * v_tol) + (0.49599 + 3.17316 * v_tol) * dr) * dr;
    let a3 = (0.25149 + 0.21474 * v_tol)
        + ((-0.83249 + 1.41498 * v_tol) + (0.85181 - 4.90094 * v_tol) * dr) * dr;
    let a2 = 1.0 - a1 - a3;
    let t2 = 0.4999
        + (((0.46159 + 8.57843 * v_tol) * v_tol)
            + (((4.26169 - 108.644 * v_tol) * v_tol)
                + ((1.75601 + 336.989 * v_tol) * v_tol) * dr)
                * dr)
            * dr;
    Ok((vec![a1, a2, a3], vec![0.0, t2 * t_d, t_d]))
}

fn expansion_coeffs(
    shaper_freq: f64,
    damping_ratio: f64,
    t: &[&[f64]],
    a: &[&[f64]],
) -> KResult<(Vec<f64>, Vec<f64>)> {
    let tau = py_div(1.0, shaper_freq)?;
    let mut t_out: Vec<f64> = Vec::with_capacity(a.len());
    let mut a_out: Vec<f64> = Vec::with_capacity(a.len());
    let k = a[0].len();
    for i in 0..a.len() {
        let mut u = t[i][k - 1];
        let mut v = a[i][k - 1];
        for j in 0..(k - 1) {
            u = u * damping_ratio + t[i][k - j - 2];
            v = v * damping_ratio + a[i][k - j - 2];
        }
        t_out.push(u * tau);
        a_out.push(v);
    }
    Ok((a_out, t_out))
}

pub fn get_2hump_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
    let t: [&[f64]; 4] = [
        &[0.0, 0.0, 0.0, 0.0],
        &[0.49890, 0.16270, -0.54262, 6.16180],
        &[0.99748, 0.18382, -1.58270, 8.17120],
        &[1.49920, -0.09297, -0.28338, 1.85710],
    ];
    let a: [&[f64]; 4] = [
        &[0.16054, 0.76699, 2.26560, -1.22750],
        &[0.33911, 0.45081, -2.58080, 1.73650],
        &[0.34089, -0.61533, -0.68765, 0.42261],
        &[0.15997, -0.60246, 1.00280, -0.93145],
    ];
    expansion_coeffs(shaper_freq, damping_ratio, &t, &a)
}

pub fn get_3hump_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> KResult<(Vec<f64>, Vec<f64>)> {
    let t: [&[f64]; 5] = [
        &[0.0, 0.0, 0.0, 0.0],
        &[0.49974, 0.23834, 0.44559, 12.4720],
        &[0.99849, 0.29808, -2.36460, 23.3990],
        &[1.49870, 0.10306, -2.01390, 17.0320],
        &[1.99960, -0.28231, 0.61536, 5.40450],
    ];
    let a: [&[f64]; 5] = [
        &[0.11275, 0.76632, 3.29160, -1.44380],
        &[0.23698, 0.61164, -2.57850, 4.85220],
        &[0.30008, -0.19062, -2.14560, 0.13744],
        &[0.23775, -0.73297, 0.46885, -2.08650],
        &[0.11244, -0.45439, 0.96382, -1.46000],
    ];
    expansion_coeffs(shaper_freq, damping_ratio, &t, &a)
}

pub fn get_shaper_cfg(shaper_name: &str) -> Option<&'static InputShaperCfg> {
    let stripped: String = match pytext::pyre::compile(r"(\w+)\s*\((.*)\)$") {
        Ok(re) => match re.match_str(shaper_name) {
            Some(groups) => match groups.first() {
                Some(Some(g)) => g.clone(),
                _ => shaper_name.to_string(),
            },
            None => shaper_name.to_string(),
        },
        Err(_) => shaper_name.to_string(),
    };
    INPUT_SHAPERS.iter().find(|s| stripped.as_str() == s.name)
}

fn parse_val(s: &str) -> KResult<ParsedArg> {
    if s.contains('.') {
        float_from_string(s).map(ParsedArg::Float).map_err(map_py_error)
    } else {
        int_from_string(s, 10).map(ParsedArg::Int).map_err(map_py_error)
    }
}

pub fn init_shaper(
    shaper_name: &str,
    shaper_freq: f64,
    damping_ratio: f64,
    error: Option<&dyn Fn(String) -> KlippyError>,
) -> KResult<Option<(Vec<f64>, Vec<f64>)>> {
    let inner = init_shaper_inner(shaper_name, shaper_freq, damping_ratio);
    match inner {
        Ok(v) => Ok(v),
        Err(e) => {
            if is_shaper_error(&e) {
                if let Some(factory) = error {
                    let msg = fmt_pct("Failed to initialize shaper: %s", &[PctValue::Str(e.message().to_string())])?;
                    Err(factory(msg))
                } else {
                    Err(e)
                }
            } else {
                Err(e)
            }
        }
    }
}

fn init_shaper_inner(
    shaper_name_in: &str,
    shaper_freq: f64,
    damping_ratio: f64,
) -> KResult<Option<(Vec<f64>, Vec<f64>)>> {
    let mut shaper_name = shaper_name_in.to_string();
    let mut args_l: Vec<ParsedArg> = Vec::new();
    let mut args_kv: Vec<(String, ParsedArg)> = Vec::new();
    if let Ok(re) = pytext::pyre::compile(r"(\w+)\s*\((.*)\)$") {
        if let Some(groups) = re.match_str(shaper_name_in) {
            if groups.len() >= 2 {
                if let Some(n) = groups[0].clone() {
                    shaper_name = n;
                }
                let args_str = groups[1].clone().unwrap_or_default();
                if !args_str.is_empty() {
                    let re2 = pytext::pyre::compile(r"(?:(\w+)\s*=\s*)?\s*([\d.]+)").map_err(|e| {
                        internal("ValueError", e.to_string())
                    })?;
                    let parsed = re2.findall(&args_str);
                    for pair in &parsed {
                        let k = &pair[0];
                        let v = &pair[1];
                        if k.is_empty() {
                            args_l.push(parse_val(v)?);
                        }
                    }
                    for pair in &parsed {
                        let k = &pair[0];
                        let v = &pair[1];
                        if !k.is_empty() {
                            let pv = parse_val(v)?;
                            if let Some(pos) = args_kv.iter().position(|(ek, _)| ek == k) {
                                args_kv[pos].1 = pv;
                            } else {
                                args_kv.push((k.clone(), pv));
                            }
                        }
                    }
                    if !args_l.is_empty() && !args_kv.is_empty() {
                        return Err(shaper_error(
                            "Mixing named and non-named shaper parameters is not supported".to_string(),
                        ));
                    }
                }
            }
        }
    }
    let cfg = match INPUT_SHAPERS.iter().find(|s| s.name == shaper_name) {
        Some(c) => c,
        None => return Ok(None),
    };
    match cfg.kind {
        ShaperKind::Zv => {
            check_no_extra_args("get_zv_shaper", 2, 2, &args_l, &args_kv)?;
            let r = get_zv_shaper(shaper_freq, damping_ratio)?;
            Ok(Some(r))
        }
        ShaperKind::Zvd => {
            check_no_extra_args("get_zvd_shaper", 2, 2, &args_l, &args_kv)?;
            let r = get_zvd_shaper(shaper_freq, damping_ratio)?;
            Ok(Some(r))
        }
        ShaperKind::TwoHumpEi => {
            check_no_extra_args("get_2hump_ei_shaper", 2, 2, &args_l, &args_kv)?;
            let r = get_2hump_ei_shaper(shaper_freq, damping_ratio)?;
            Ok(Some(r))
        }
        ShaperKind::ThreeHumpEi => {
            check_no_extra_args("get_3hump_ei_shaper", 2, 2, &args_l, &args_kv)?;
            let r = get_3hump_ei_shaper(shaper_freq, damping_ratio)?;
            Ok(Some(r))
        }
        ShaperKind::Ei => {
            let v_tol_arg = bind_single_default("get_ei_shaper", 2, 3, &["v_tol"], &args_l, &args_kv)?;
            let v_tol = match v_tol_arg {
                Some(v) => v.to_float()?,
                None => 1.0 / SHAPER_VIBRATION_REDUCTION,
            };
            let r = get_ei_shaper_v_tol(shaper_freq, damping_ratio, v_tol)?;
            Ok(Some(r))
        }
        ShaperKind::Mzv => {
            let bound = bind_multi_default("get_mzv_shaper", 2, 5, &["n", "t", "tau"], &args_l, &args_kv)?;
            let n_arg = bound[0].clone().unwrap_or(ParsedArg::Int(PyInt::from_i64(3)));
            let t_arg = bound[1].clone().unwrap_or(ParsedArg::Float(0.0));
            let tau_arg = bound[2].clone().unwrap_or(ParsedArg::Float(0.0));
            let r = get_mzv_shaper_inner(shaper_freq, damping_ratio, &n_arg, &t_arg, &tau_arg)?;
            Ok(Some(r))
        }
    }
}

fn check_no_extra_args(
    fname: &str,
    _min_pos: usize,
    _max_pos: usize,
    args_l: &[ParsedArg],
    args_kv: &[(String, ParsedArg)],
) -> KResult<()> {
    let total = 2 + args_l.len();
    if total != 2 {
        return Err(internal(
            "TypeError",
            format!("{fname}() takes 2 positional arguments but {total} were given"),
        ));
    }
    if let Some((k, _)) = args_kv.first() {
        return Err(internal(
            "TypeError",
            format!("{fname}() got an unexpected keyword argument '{k}'"),
        ));
    }
    Ok(())
}

fn bind_single_default(
    fname: &str,
    min_pos: usize,
    max_pos: usize,
    param_names: &[&str],
    args_l: &[ParsedArg],
    args_kv: &[(String, ParsedArg)],
) -> KResult<Option<ParsedArg>> {
    let total = 2 + args_l.len();
    if total > max_pos {
        return Err(internal(
            "TypeError",
            format!(
                "{fname}() takes from {min_pos} to {max_pos} positional arguments but {total} were given"
            ),
        ));
    }
    let mut bound: Option<ParsedArg> = if args_l.is_empty() { None } else { Some(args_l[0].clone()) };
    for (k, v) in args_kv {
        if !param_names.contains(&k.as_str()) {
            return Err(internal(
                "TypeError",
                format!("{fname}() got an unexpected keyword argument '{k}'"),
            ));
        }
        if bound.is_some() {
            return Err(internal(
                "TypeError",
                format!("{fname}() got multiple values for argument '{k}'"),
            ));
        }
        bound = Some(v.clone());
    }
    Ok(bound)
}

fn bind_multi_default(
    fname: &str,
    min_pos: usize,
    max_pos: usize,
    param_names: &[&str],
    args_l: &[ParsedArg],
    args_kv: &[(String, ParsedArg)],
) -> KResult<Vec<Option<ParsedArg>>> {
    let total = 2 + args_l.len();
    if total > max_pos {
        return Err(internal(
            "TypeError",
            format!(
                "{fname}() takes from {min_pos} to {max_pos} positional arguments but {total} were given"
            ),
        ));
    }
    let mut bound: Vec<Option<ParsedArg>> = vec![None; param_names.len()];
    for (i, v) in args_l.iter().enumerate() {
        if i < bound.len() {
            bound[i] = Some(v.clone());
        }
    }
    for (k, v) in args_kv {
        let pos = param_names.iter().position(|p| *p == k.as_str());
        match pos {
            None => {
                return Err(internal(
                    "TypeError",
                    format!("{fname}() got an unexpected keyword argument '{k}'"),
                ));
            }
            Some(idx) => {
                if bound[idx].is_some() {
                    return Err(internal(
                        "TypeError",
                        format!("{fname}() got multiple values for argument '{k}'"),
                    ));
                }
                bound[idx] = Some(v.clone());
            }
        }
    }
    Ok(bound)
}
