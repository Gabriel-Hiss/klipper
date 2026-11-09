//! Bit-exact tests of `pycompat::numpy` against the NumPy 1.16.4 of the
//! reference oracle (`pyoracle`, `$KLIPPY_PYTHON`). Every expected value is
//! computed by NumPy on the same inputs; comparisons are on the `f64` bits
//! (NaN compares by NaN-ness). Tests skip with a message outside the oracle.

use pycompat::numpy::{
    add_reduce, arange, fft, i0, interp, kaiser, max_reduce, maximum, mean, min_reduce, minimum,
    pairwise_sum, rfft, rfftfreq, sequential_sum, Complex64, FftPlan, RfftPlan, BUFSIZE,
};
use pycompat::PyError;
use recording::Value;

fn oracle() -> Option<pyoracle::Oracle> {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: KLIPPY_PYTHON not set (no oracle)");
        return None;
    };
    py.exec("import numpy as np").expect("numpy import");
    Some(py)
}

/// Deterministic inputs: a 64-bit LCG mapped to `[-1, 1)` and scaled by a
/// power of ten in `1e-3..1e3`, so sums cancel and round in varied ways.
struct Lcg(u64);

impl Lcg {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0
    }
    fn unit(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    }
    fn value(&mut self) -> f64 {
        let scale = [1e-3, 1e-2, 0.1, 1.0, 10.0, 100.0, 1e3][(self.next_u64() >> 40) as usize % 7];
        self.unit() * scale
    }
    fn values(&mut self, n: usize) -> Vec<f64> {
        (0..n).map(|_| self.value()).collect()
    }
}

/// A Python expression for `v` that parses to the same bits.
fn py_float(v: f64) -> String {
    if v.is_nan() {
        "float('nan')".to_string()
    } else if v.is_infinite() {
        if v > 0.0 { "float('inf')" } else { "float('-inf')" }.to_string()
    } else {
        format!("{v:?}")
    }
}

fn py_list(vs: &[f64]) -> String {
    format!("[{}]", vs.iter().map(|&v| py_float(v)).collect::<Vec<_>>().join(","))
}

fn float_of(v: &Value) -> f64 {
    match v {
        Value::Float(f) => f.to_f64().expect("float hex"),
        Value::Int(i) => *i as f64,
        other => panic!("expected a float, got {other:?}"),
    }
}

fn floats(v: &Value) -> Vec<f64> {
    match v {
        Value::List(items) | Value::Tuple(items) => items.iter().map(float_of).collect(),
        other => panic!("expected a list, got {other:?}"),
    }
}

fn eval_float(py: &mut pyoracle::Oracle, expr: &str) -> f64 {
    float_of(&py.eval(expr).unwrap_or_else(|e| panic!("{expr}: {e:?}")))
}

fn eval_floats(py: &mut pyoracle::Oracle, expr: &str) -> Vec<f64> {
    floats(&py.eval(expr).unwrap_or_else(|e| panic!("{expr}: {e:?}")))
}

fn same(a: f64, b: f64) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

fn assert_bits(what: &str, actual: f64, expected: f64) {
    assert!(same(actual, expected), "{what}: {actual:?} vs NumPy {expected:?}");
}

fn assert_all_bits(what: &str, actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len(), "{what}: length");
    for (i, (&a, &e)) in actual.iter().zip(expected).enumerate() {
        assert!(same(a, e), "{what}[{i}]: {a:?} vs NumPy {e:?}");
    }
}

fn assert_error(what: &str, actual: Result<Vec<f64>, PyError>, expected: pyoracle::OracleResult) {
    let expected = expected.expect_err(&format!("{what}: NumPy did not raise"));
    let actual = actual.expect_err(&format!("{what}: port did not raise"));
    assert_eq!(
        (actual.kind, actual.msg.as_str()),
        (expected.class_name.as_str(), expected.message.as_str()),
        "{what}"
    );
}

// ---------------------------------------------------------------------------
// Reductions.
// ---------------------------------------------------------------------------

/// Sizes around the unroll (8), the pairwise block (128) and the ufunc
/// buffer (8192) boundaries.
fn reduction_sizes() -> Vec<usize> {
    let mut sizes: Vec<usize> = (0..=40).collect();
    sizes.extend([63, 64, 65, 120, 127, 128, 129, 130, 135, 136, 255, 256, 257, 300, 1000, 1031]);
    sizes.extend([BUFSIZE - 1, BUFSIZE, BUFSIZE + 1, BUFSIZE + 8, 9000, 2 * BUFSIZE + 1, 20000]);
    sizes
}

#[test]
fn add_reduce_and_mean_match_numpy() {
    let Some(mut py) = oracle() else { return };
    let mut rng = Lcg(1);
    for n in reduction_sizes() {
        let a = rng.values(n);
        py.exec(&format!("a = np.array({}, dtype=np.float64)", py_list(&a))).unwrap();
        assert_bits(&format!("sum n={n}"), add_reduce(&a), eval_float(&mut py, "float(a.sum())"));
        assert_bits(
            &format!("add.reduce n={n}"),
            add_reduce(&a),
            eval_float(&mut py, "float(np.add.reduce(a))"),
        );
        if n > 0 {
            assert_bits(&format!("mean n={n}"), mean(&a), eval_float(&mut py, "float(a.mean())"));
            assert_bits(
                &format!("average n={n}"),
                mean(&a),
                eval_float(&mut py, "float(np.average(a))"),
            );
        }
        if n <= BUFSIZE {
            assert_bits(&format!("pairwise n={n}"), 0. + pairwise_sum(&a), add_reduce(&a));
        }
    }
    // Signed zeros: the reduction starts at the identity `0.`.
    py.exec("a = np.array([-0.0, -0.0])").unwrap();
    assert_bits("sum of -0.0", add_reduce(&[-0.0, -0.0]), eval_float(&mut py, "float(a.sum())"));
}

#[test]
fn axis_reductions_match_numpy() {
    let Some(mut py) = oracle() else { return };
    let mut rng = Lcg(2);
    // `np.mean(x, axis=0)` of overlapping column windows (as in
    // `shaper_calibrate._split_into_windows`) reduces along the inner loop;
    // the `axis=-1` mean of the transposed `rfft` output adds rows in order.
    for (n, nfft) in [(300, 16), (3000, 64), (5000, 256), (12000, 2048), (50000, 12000)] {
        let x = rng.values(n);
        py.exec(&format!(
            "x = np.array({x})\n\
             step = {nfft} - {nfft} // 2\n\
             nw = (x.shape[-1] - {nfft} // 2) // step\n\
             w = np.lib.stride_tricks.as_strided(x, shape=({nfft}, nw), \
             strides=(x.strides[-1], step * x.strides[-1]), writeable=False)\n\
             r = np.fft.rfft(w, n={nfft}, axis=0)",
            x = py_list(&x)
        ))
        .unwrap();
        let step = nfft - nfft / 2;
        let nw = (n - nfft / 2) / step;
        let means: Vec<f64> =
            (0..nw).map(|j| mean(&x[j * step..j * step + nfft])).collect();
        assert_all_bits(
            &format!("window means n={n} nfft={nfft}"),
            &means,
            &eval_floats(&mut py, "np.mean(w, axis=0).tolist()"),
        );
        let flat = eval_floats(&mut py, "[v for row in r.real.tolist() for v in row]");
        let row_means: Vec<f64> = flat
            .chunks(nw)
            .map(|row| sequential_sum(row.iter().copied()) / nw as f64)
            .collect();
        assert_all_bits(
            &format!("transposed rfft means n={n} nfft={nfft}"),
            &row_means,
            &eval_floats(&mut py, "r.real.mean(axis=-1).tolist()"),
        );
    }
}

#[test]
fn maximum_minimum_match_numpy() {
    let Some(mut py) = oracle() else { return };
    let specials = [0.0, -0.0, 1.0, -1.0, 2.5, f64::NAN, f64::INFINITY, f64::NEG_INFINITY];
    let mut a = Vec::new();
    let mut b = Vec::new();
    for &x in &specials {
        for &y in &specials {
            a.push(x);
            b.push(y);
        }
    }
    py.exec(&format!("a = np.array({}); b = np.array({})", py_list(&a), py_list(&b))).unwrap();
    let max: Vec<f64> = a.iter().zip(&b).map(|(&x, &y)| maximum(x, y)).collect();
    let min: Vec<f64> = a.iter().zip(&b).map(|(&x, &y)| minimum(x, y)).collect();
    assert_all_bits("maximum", &max, &eval_floats(&mut py, "np.maximum(a, b).tolist()"));
    assert_all_bits("minimum", &min, &eval_floats(&mut py, "np.minimum(a, b).tolist()"));
    let zero: Vec<f64> = a.iter().map(|&x| maximum(x, 0.)).collect();
    assert_all_bits("maximum(a, 0)", &zero, &eval_floats(&mut py, "np.maximum(a, 0).tolist()"));
    let mut rng = Lcg(3);
    for n in [1, 2, 7, 8, 9, 100, 1000] {
        let mut v = rng.values(n);
        assert_bits(
            &format!("max n={n}"),
            max_reduce(&v).unwrap(),
            eval_float(&mut py, &format!("float(np.array({}).max())", py_list(&v))),
        );
        assert_bits(
            &format!("min n={n}"),
            min_reduce(&v).unwrap(),
            eval_float(&mut py, &format!("float(np.array({}).min())", py_list(&v))),
        );
        v[n / 2] = f64::NAN;
        assert_bits(
            &format!("max with nan n={n}"),
            max_reduce(&v).unwrap(),
            eval_float(&mut py, &format!("float(np.array({}).max())", py_list(&v))),
        );
    }
    assert_error(
        "max of empty",
        max_reduce(&[]).map(|v| vec![v]),
        py.eval("np.array([], dtype=np.float64).max()"),
    );
    assert_error(
        "min of empty",
        min_reduce(&[]).map(|v| vec![v]),
        py.eval("np.array([], dtype=np.float64).min()"),
    );
}

// ---------------------------------------------------------------------------
// `arange`.
// ---------------------------------------------------------------------------

#[test]
fn arange_matches_numpy() {
    let Some(mut py) = oracle() else { return };
    let mut cases: Vec<(f64, f64, f64)> = vec![
        (5.0, 150.0, 0.2),
        (5.0 - 1e-7, 150.0, 0.2),
        (149.9999999, 150.0, 0.2),
        (0.0, 200.0, 0.2),
        (1.5625, 200.0, 0.2),
        (20.0, 60.0, 5.0),
        (0.0, 1.0, 0.1),
        (0.0, 1.0, 0.3),
        (10.0, 0.0, -0.7),
        (0.0, 10.0, -1.0),
        (3.0, 3.0, 1.0),
        (0.0, 1e-300, 1e300),
        (0.0, -1e-300, 1e300),
        (1.0, 1.0 + 1e-15, 1e-16),
    ];
    let mut rng = Lcg(4);
    for _ in 0..200 {
        let start = rng.unit() * 100.0;
        let stop = start + rng.unit() * 300.0;
        let step = [0.2, 0.1, 0.3, 0.7, rng.unit() * 3.0][(rng.next_u64() >> 50) as usize % 5];
        cases.push((start, stop, step));
    }
    for (start, stop, step) in cases {
        let expr = format!(
            "np.arange({}, {}, {}).tolist()",
            py_float(start),
            py_float(stop),
            py_float(step)
        );
        let actual = arange(start, stop, step).unwrap();
        assert_all_bits(&expr, &actual, &eval_floats(&mut py, &expr));
    }
    for (start, stop, step) in
        [(1.0, 2.0, 0.0), (0.0, f64::NAN, 1.0), (0.0, f64::INFINITY, 1.0), (0.0, 1e300, 1e-300)]
    {
        let expr =
            format!("np.arange({}, {}, {})", py_float(start), py_float(stop), py_float(step));
        assert_error(&expr, arange(start, stop, step), py.eval(&expr));
    }
}

// ---------------------------------------------------------------------------
// `i0` and `kaiser`.
// ---------------------------------------------------------------------------

#[test]
fn i0_and_kaiser_match_numpy() {
    let Some(mut py) = oracle() else { return };
    let mut xs = vec![0.0, -0.0, 1.0, -1.0, 6.0, 7.999, 8.0, 8.001, -8.5, 14.0, 20.0, 100.0, 700.0];
    xs.extend([f64::NAN, f64::INFINITY]);
    let mut rng = Lcg(5);
    xs.extend((0..100).map(|_| rng.unit() * 30.0));
    let actual: Vec<f64> = xs.iter().map(|&x| i0(x)).collect();
    assert_all_bits(
        "i0",
        &actual,
        &eval_floats(&mut py, &format!("np.i0(np.array({})).tolist()", py_list(&xs))),
    );
    for m in [0usize, 1, 2, 3, 7, 8, 9, 64, 100, 127, 128, 129, 257, 1024, 2048] {
        for beta in [6.0, 0.5, 14.0, 20.0] {
            let expr = format!("np.kaiser({m}, {beta:?}).tolist()");
            assert_all_bits(&expr, &kaiser(m, beta), &eval_floats(&mut py, &expr));
        }
    }
}

// ---------------------------------------------------------------------------
// `interp`.
// ---------------------------------------------------------------------------

#[test]
fn interp_matches_numpy() {
    let Some(mut py) = oracle() else { return };
    let mut rng = Lcg(6);
    for lenxp in [1usize, 2, 3, 4, 5, 6, 10, 17, 200] {
        let mut xp: Vec<f64> = rng.values(lenxp).iter().map(|v| v.abs()).collect();
        xp.sort_by(|a, b| a.partial_cmp(b).unwrap());
        if lenxp > 4 {
            // A repeated sample point.
            xp[lenxp / 2] = xp[lenxp / 2 - 1];
        }
        let fp = rng.values(lenxp);
        let lo = xp[0] - 1.0;
        let hi = xp[lenxp - 1] + 1.0;
        for lenx in [1usize, 3, lenxp, 4 * lenxp + 7] {
            let mut x: Vec<f64> = (0..lenx).map(|_| lo + (rng.unit() + 1.0) / 2.0 * (hi - lo)).collect();
            // Exact sample points, the ends and a NaN.
            x.extend(xp.iter().copied().step_by(3));
            x.push(xp[lenxp - 1]);
            x.push(f64::NAN);
            let expr = format!(
                "np.interp(np.array({}), np.array({}), np.array({})).tolist()",
                py_list(&x),
                py_list(&xp),
                py_list(&fp)
            );
            assert_all_bits(&expr, &interp(&x, &xp, &fp).unwrap(), &eval_floats(&mut py, &expr));
        }
    }
    // Shaper-style use: a fine regular grid over coarse bins.
    let xp = rfftfreq(2048, 1.0 / 3199.7);
    let fp: Vec<f64> = (0..xp.len()).map(|_| rng.unit().abs()).collect();
    let x = arange(1.5625, 200.0, 0.2).unwrap();
    let expr = format!(
        "np.interp(np.array({}), np.array({}), np.array({})).tolist()",
        py_list(&x),
        py_list(&xp),
        py_list(&fp)
    );
    assert_all_bits("shaper grid", &interp(&x, &xp, &fp).unwrap(), &eval_floats(&mut py, &expr));
    // Errors.
    assert_error("empty xp", interp(&[1.0], &[], &[]), py.eval("np.interp([1.0], [], [])"));
    assert_error(
        "length mismatch",
        interp(&[1.0], &[1.0, 2.0], &[1.0]),
        py.eval("np.interp([1.0], [1.0, 2.0], [1.0])"),
    );
}

// ---------------------------------------------------------------------------
// FFT.
// ---------------------------------------------------------------------------

/// Powers of two, composites of the special radices (2, 3, 4, 5), primes
/// (the generic `radfg`/`passf` pass with `ido == 1`) and composites with
/// prime factors above 5 (the generic pass with `ido > 1`).
const FFT_SIZES: [usize; 36] = [
    1, 2, 4, 8, 16, 64, 128, 1024, 2048, // powers of two
    3, 5, 6, 12, 15, 30, 45, 60, 100, 360, 1000, 1536, 3200, // 2, 3, 4, 5
    7, 11, 13, 17, 101, 257, 1009, // primes
    14, 22, 49, 77, 98, 1001, 2310, // factors above 5
];

fn complex_parts(z: &[Complex64]) -> Vec<f64> {
    z.iter().flat_map(|c| [c.re, c.im]).collect()
}

#[test]
fn rfft_matches_numpy() {
    let Some(mut py) = oracle() else { return };
    let mut rng = Lcg(7);
    for n in FFT_SIZES {
        // Exact length, cropped and zero-padded input.
        for len in [n, n + 3, n.saturating_sub(2).max(1)] {
            let x = rng.values(len);
            py.exec(&format!("x = np.array({})", py_list(&x))).unwrap();
            let expected = eval_floats(
                &mut py,
                &format!("[v for c in np.fft.rfft(x, {n}).tolist() for v in (c.real, c.imag)]"),
            );
            assert_all_bits(&format!("rfft n={n} len={len}"), &complex_parts(&rfft(&x, n).unwrap()), &expected);
        }
    }
    // A plan reused over several rows (as `_psd` does per window).
    let plan = RfftPlan::new(360).unwrap();
    for _ in 0..3 {
        let x = rng.values(360);
        let expected = eval_floats(
            &mut py,
            &format!("[v for c in np.fft.rfft(np.array({})).tolist() for v in (c.real, c.imag)]", py_list(&x)),
        );
        assert_all_bits("rfft plan reuse", &complex_parts(&plan.rfft(&x)), &expected);
    }
    assert_error(
        "rfft n=0",
        rfft(&[1.0], 0).map(|z| complex_parts(&z)),
        py.eval("np.fft.rfft(np.array([1.0]), 0)"),
    );
}

#[test]
fn fft_matches_numpy() {
    let Some(mut py) = oracle() else { return };
    let mut rng = Lcg(8);
    for n in FFT_SIZES {
        for len in [n, n + 3, n.saturating_sub(2).max(1)] {
            let re = rng.values(len);
            let im = rng.values(len);
            py.exec(&format!(
                "z = np.empty({len}, dtype=complex)\nz.real = {}\nz.imag = {}",
                py_list(&re),
                py_list(&im)
            ))
            .unwrap();
            let z: Vec<Complex64> = re.iter().zip(&im).map(|(&a, &b)| Complex64::new(a, b)).collect();
            let expected = eval_floats(
                &mut py,
                &format!("[v for c in np.fft.fft(z, {n}).tolist() for v in (c.real, c.imag)]"),
            );
            assert_all_bits(&format!("fft n={n} len={len}"), &complex_parts(&fft(&z, n).unwrap()), &expected);
        }
    }
    let plan = FftPlan::new(77).unwrap();
    assert_eq!(plan.len(), 77);
    assert_error(
        "fft n=0",
        fft(&[Complex64::new(1.0, 0.0)], 0).map(|z| complex_parts(&z)),
        py.eval("np.fft.fft(np.array([1.0]), 0)"),
    );
}

#[test]
fn rfftfreq_and_complex_products_match_numpy() {
    let Some(mut py) = oracle() else { return };
    for n in [1usize, 2, 15, 16, 1024, 2048, 4096] {
        for d in [1.0 / 3200.0, 1.0 / 3199.7, 1.0 / 1600.123, 0.5, 1.0] {
            let expr = format!("np.fft.rfftfreq({n}, {d:?}).tolist()");
            assert_all_bits(&expr, &rfftfreq(n, d), &eval_floats(&mut py, &expr));
        }
    }
    // `np.conjugate(r) * r`, then `*= s` with a real scalar (`_psd`).
    let mut rng = Lcg(9);
    let x = rng.values(64);
    let s = 1.0 / 3.0 / 3199.7;
    let r = rfft(&x, 64).unwrap();
    let power: Vec<Complex64> = r
        .iter()
        .map(|&c| c.conj() * c * Complex64::new(s, 0.))
        .collect();
    let expected = eval_floats(
        &mut py,
        &format!(
            "[v for c in (lambda r: (np.conjugate(r) * r) * {s:?})(np.fft.rfft(np.array({}))).tolist() \
             for v in (c.real, c.imag)]",
            py_list(&x)
        ),
    );
    assert_all_bits("conj(r) * r * s", &complex_parts(&power), &expected);
}
