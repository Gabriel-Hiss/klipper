//! A deterministic accelerometer batch shaped like an Ender 3 V3 KE
//! `adxl345` capture (`config/ender-3-v3-ke/input_shaper.cfg`): 3 seconds
//! at the chip's ~3200 Hz rate, full-resolution counts (`0.0039 g` per LSB,
//! in mm/s^2) holding the X (38.5 Hz) and Y (32 Hz) resonances, harmonics,
//! gravity on Z and integer count noise. The rows are generated once in
//! Rust and handed to the oracle as exact float literals, so both sides see
//! the same batch.

/// `adxl345` full-resolution scale: `0.0039 * FREEFALL_ACCEL` (mm/s^2 per count).
const SCALE: f64 = 0.0039 * 9806.65;

/// Sample rate as estimated from the MCU clock (slightly off nominal).
const RATE: f64 = 3199.67;

/// Number of samples (3 seconds).
pub const SAMPLES: usize = 9600;

/// `(time, x, y, z)` rows of the batch.
pub fn ke_batch() -> Vec<(f64, f64, f64, f64)> {
    let tau = std::f64::consts::TAU;
    let mut state: u64 = 0x2545_f491_4f6c_dd1d;
    let mut noise = move || -> f64 {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((state >> 33) % 9) as f64 - 4.0
    };
    (0..SAMPLES)
        .map(|i| {
            let t = 0.0125 + i as f64 / RATE;
            let decay = kos::libm::exp(-0.35 * t);
            let x = 1450.0 * decay * kos::libm::sin(tau * 38.5 * t + 0.3)
                + 380.0 * kos::libm::sin(tau * 77.0 * t)
                + 120.0 * kos::libm::sin(tau * 61.3 * t + 1.1);
            let y = 1100.0 * decay * kos::libm::sin(tau * 32.0 * t + 0.9)
                + 260.0 * kos::libm::sin(tau * 64.0 * t + 0.2)
                + 90.0 * kos::libm::cos(tau * 47.7 * t);
            let z = 9806.65 + 150.0 * kos::libm::sin(tau * 25.0 * t);
            let counts = |v: f64, n: f64| (v / SCALE).round() + n;
            (
                t,
                counts(x, noise()) * SCALE,
                counts(y, noise()) * SCALE,
                counts(z, noise()) * SCALE,
            )
        })
        .collect()
}

/// A Python literal for the rows (exact float spellings).
pub fn py_rows(rows: &[(f64, f64, f64, f64)]) -> String {
    let body: Vec<String> = rows
        .iter()
        .map(|(t, x, y, z)| format!("({t:?},{x:?},{y:?},{z:?})"))
        .collect();
    format!("[{}]", body.join(","))
}
