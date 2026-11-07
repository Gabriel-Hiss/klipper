//! Replay and assertion helpers.
//!
//! A unit test loads a recording, calls the ported unit with the
//! recorded arguments, and asserts the actual result with
//! [`assert_equal`]. [`ReplayCursor`] walks a recording in `seq` order
//! and reports the first divergence with full context: the unit, the
//! scenario, the sequence number, the function, the field path where
//! the values differ, and both sides rendered.
//!
//! Equality is exact: integers and strings as-is, floats by hex float
//! and `repr` spelling, bytes byte for byte, structs field by field,
//! dicts in insertion order, and object identities by stable id.

use crate::value::{Record, Value};

/// The first divergence found between a recording and the ported unit.
#[derive(Debug, Clone, PartialEq)]
pub struct Divergence {
    pub unit: String,
    pub scenario: String,
    pub seq: u64,
    pub function: String,
    /// Path to the differing field, e.g. `args[1]`, `ret.fields[2].y`.
    pub path: String,
    pub expected: String,
    pub actual: String,
}

impl std::fmt::Display for Divergence {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "divergence in {} (scenario {}, seq {}): {} at {}: expected {}, got {}",
            self.unit, self.scenario, self.seq, self.function, self.path,
            self.expected, self.actual
        )
    }
}

/// Asserts that `actual` equals `expected` exactly, as [`Value`]s are
/// decoded from a recording. On mismatch, returns a [`Divergence`] for
/// the first differing field, with the path from the value root.
pub fn assert_equal(
    record: &Record,
    path: &str,
    expected: &Value,
    actual: &Value,
) -> Result<(), Divergence> {
    fn make_div(record: &Record, path: String, expected: &Value, actual: &Value) -> Divergence {
        Divergence {
            unit: record.unit.clone(),
            scenario: record.scenario.clone(),
            seq: record.seq,
            function: record.function.clone(),
            path,
            expected: expected.render(),
            actual: actual.render(),
        }
    }
    fn check(record: &Record, path: String, expected: &Value, actual: &Value) -> Option<Divergence> {
        match (expected, actual) {
            (Value::Null, Value::Null) => None,
            (Value::Bool(a), Value::Bool(b)) => {
                if a == b {
                    None
                } else {
                    Some(make_div(record, path, expected, actual))
                }
            }
            (Value::Int(a), Value::Int(b)) => {
                if a == b {
                    None
                } else {
                    Some(make_div(record, path, expected, actual))
                }
            }
            (Value::BigInt(a), Value::BigInt(b)) => {
                if a == b {
                    None
                } else {
                    Some(make_div(record, path, expected, actual))
                }
            }
            (Value::Float(a), Value::Float(b)) => {
                // Compare by bits (NaN payloads included); the hex spelling of the same
                // value can differ between CPython's float.hex() and a port's formatter.
                let bits = |f: &crate::value::FloatVal| {
                    crate::hexfloat::f64_from_hex(&f.hex).map(f64::to_bits)
                };
                if bits(a).is_some() && bits(a) == bits(b) {
                    None
                } else {
                    Some(make_div(record, path, expected, actual))
                }
            }
            (Value::Bytes(a), Value::Bytes(b)) => {
                if a == b {
                    None
                } else {
                    Some(make_div(record, path, expected, actual))
                }
            }
            (Value::Str(a), Value::Str(b)) => {
                if a == b {
                    None
                } else {
                    Some(make_div(record, path, expected, actual))
                }
            }
            (Value::List(a), Value::List(b)) => {
                if a.len() != b.len() {
                    return Some(make_div(record, path, expected, actual));
                }
                for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                    if let Some(d) = check(record, format!("{path}[{i}]"), x, y) {
                        return Some(d);
                    }
                }
                None
            }
            (Value::Tuple(a), Value::Tuple(b)) => {
                if a.len() != b.len() {
                    return Some(make_div(record, path, expected, actual));
                }
                for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                    if let Some(d) = check(record, format!("{path}[{i}]"), x, y) {
                        return Some(d);
                    }
                }
                None
            }
            (Value::Dict(a), Value::Dict(b)) => {
                if a.len() != b.len() {
                    return Some(make_div(record, path, expected, actual));
                }
                for (i, ((ak, av), (bk, bv))) in a.iter().zip(b.iter()).enumerate() {
                    if let Some(d) = check(record, format!("{path}[{i}].key"), ak, bk) {
                        return Some(d);
                    }
                    if let Some(d) = check(record, format!("{path}[{i}].value"), av, bv) {
                        return Some(d);
                    }
                }
                None
            }
            (
                Value::Struct { type_name: ta, fields: fa },
                Value::Struct { type_name: tb, fields: fb },
            ) => {
                if ta != tb || fa.len() != fb.len() {
                    return Some(make_div(record, path, expected, actual));
                }
                for ((na, va), (nb, vb)) in fa.iter().zip(fb.iter()) {
                    if na != nb {
                        return Some(make_div(record, path, expected, actual));
                    }
                    if let Some(d) = check(record, format!("{path}.fields[{na}]"), va, vb) {
                        return Some(d);
                    }
                }
                None
            }
            (
                Value::Opaque { type_name: ta, addr: aa },
                Value::Opaque { type_name: tb, addr: ab },
            ) => {
                if ta == tb && aa == ab {
                    None
                } else {
                    Some(make_div(record, path, expected, actual))
                }
            }
            (Value::ObjectId(a), Value::ObjectId(b)) => {
                if a == b {
                    None
                } else {
                    Some(make_div(record, path, expected, actual))
                }
            }
            _ => Some(make_div(record, path, expected, actual)),
        }
    }
    match check(record, path.to_string(), expected, actual) {
        Some(d) => Err(d),
        None => Ok(()),
    }
}

/// Walks one scenario recording in `seq` order for replay.
pub struct ReplayCursor {
    records: Vec<Record>,
    pos: usize,
}

impl ReplayCursor {
    pub fn new(mut records: Vec<Record>) -> ReplayCursor {
        records.sort_by_key(|r| r.seq);
        ReplayCursor { records, pos: 0 }
    }

    /// The next record, if any.
    pub fn next(&mut self) -> Option<&Record> {
        let r = self.records.get(self.pos);
        if r.is_some() {
            self.pos += 1;
        }
        r
    }

    /// Returns the next record if it has the expected function, else
    /// a divergence naming the function that was found instead.
    pub fn expect_function(&mut self, function: &str) -> Result<Record, Divergence> {
        let next = self.next().cloned();
        match next {
            None => {
                let (unit, scenario) = match self.records.last() {
                    Some(r) => (r.unit.clone(), r.scenario.clone()),
                    None => (String::new(), String::new()),
                };
                Err(Divergence {
                    unit,
                    scenario,
                    seq: 0,
                    function: function.to_string(),
                    path: "seq".to_string(),
                    expected: format!("call to {function}"),
                    actual: "end of recording".to_string(),
                })
            }
            Some(r) if r.function == function => Ok(r),
            Some(r) => Err(Divergence {
                unit: r.unit.clone(),
                scenario: r.scenario.clone(),
                seq: r.seq,
                function: r.function.clone(),
                path: "function".to_string(),
                expected: format!("call to {function}"),
                actual: format!("call to {}", r.function),
            }),
        }
    }

    /// Asserts that every record has been consumed.
    pub fn finish(&self) -> Result<(), Divergence> {
        if self.pos < self.records.len() {
            let r = &self.records[self.pos];
            return Err(Divergence {
                unit: r.unit.clone(),
                scenario: r.scenario.clone(),
                seq: r.seq,
                function: r.function.clone(),
                path: "seq".to_string(),
                expected: "end of recording".to_string(),
                actual: format!("unconsumed call to {}", r.function),
            });
        }
        Ok(())
    }

    /// True when every record has been consumed.
    pub fn is_done(&self) -> bool {
        self.pos >= self.records.len()
    }
}
