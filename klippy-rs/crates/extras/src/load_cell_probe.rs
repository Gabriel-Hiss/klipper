//! Port of `klippy/extras/load_cell_probe.py`.
//!
//! A probe that triggers on the force a load cell reports: the host tares the
//! sensor, programs an SOS filter and a trigger level into the MCU
//! (`trigger_analog`), then homes against the MCU trigger.
//!
//! NumPy is not used: `np.average` over a float64 column is reproduced with
//! NumPy's pairwise summation ([`numpy_average`]) and `np.asarray(..)` splits
//! the sample rows into columns ([`TapAnalysis`]), including the NumPy error
//! texts for malformed rows. The "requires the NumPy module" configuration
//! error of the reference cannot occur.

use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GetIntOpts as GcmdIntOpts, GetFloatOpts as GcmdFloatOpts,
    GetOpts as GcmdGetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{PrinterHomingApi, ToolHeadApi};
use klippy_api::{lookup_interface, GCell, ObjectRef, PrinterObject};
use pycompat::text::{float_from_string, float_repr, trunc_float_to_bigint, PyError};
use pycompat::value::type_name;
use pycompat::{libm, PyInt, PyValue};
use std::sync::{Arc, Weak};

use crate::ads1220::Ads1220;
use crate::ads131m0x::ADS131M0X_SENSOR_TYPES;
use crate::hx71x::HX71X_SENSOR_TYPES;
use crate::load_cell::{
    ApiClientHelper, BulkSensorAdc, CollectResult, LoadCell, LoadCellSampleCollector,
};
use crate::manual_probe::{create_probe_result, ProbeResult};
use crate::probe::{
    lookup_minimum_z, HomingViaProbeHelper, LookupZSteppers, PrinterProbeApi, ProbeCommandHelper,
    ProbeOffsetsHelper, ProbeOffsetsHelperApi, ProbeParameterHelper, ProbeParameterHelperApi,
    ProbeParams, ProbeSessionApi, SampleAveragingHelper,
};
use crate::trigger_analog::{
    CfgErrorKind, DigitalFilter, MCU_SosFilter, MCU_trigger_analog, TriggerAnalogSensor,
};

/// MCU SOS filter scaled to "fractional grams" for consistent sensor precision
/// (`load_cell_probe.py:16`).
pub const FRAC_GRAMS_CONV: f64 = 32768.0;

// ---------------------------------------------------------------------------
// Python and NumPy helpers
// ---------------------------------------------------------------------------

/// An exception the reference lets escape (`AttributeError`, `TypeError`, ...): not caught by
/// class anywhere, so it is an internal error.
fn other(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.into() }
}

fn py_error(e: PyError) -> KlippyError {
    KlippyError::Internal { class_name: e.kind.to_string(), message: e.msg }
}

/// `int(v)` of a float (`ValueError` for NaN, `OverflowError` for infinity).
fn float_to_int(v: f64) -> KResult<PyInt> {
    if v.is_nan() {
        return Err(other("ValueError", "cannot convert float NaN to integer"));
    }
    if v.is_infinite() {
        return Err(other("OverflowError", "cannot convert float infinity to integer"));
    }
    Ok(trunc_float_to_bigint(v))
}

/// A Python int as the `i64` the MCU command layer takes. Sensor ranges are 24 bit, so a
/// value outside `i64` only arises from absurd configuration.
fn int_to_i64(n: &PyInt) -> KResult<i64> {
    n.inner()
        .to_string()
        .parse::<i64>()
        .map_err(|_| other("OverflowError", "Python int too large to convert to C long"))
}

/// `int(v)` as an `i64`.
fn int_of_float(v: f64) -> KResult<i64> {
    int_to_i64(&float_to_int(v)?)
}

/// `math.ceil(v)` of a float.
fn math_ceil(v: f64) -> KResult<i64> {
    int_of_float(libm::ceil(v))
}

/// `math.floor(v)` of a float.
fn math_floor(v: f64) -> KResult<i64> {
    int_of_float(libm::floor(v))
}

/// NumPy's `pairwise_sum_DOUBLE` (`numpy/core/src/umath/loops.c.src`) over a contiguous
/// float64 array.
fn pairwise_sum(a: &[f64]) -> f64 {
    let n = a.len();
    if n < 8 {
        let mut res = 0.;
        for v in a {
            res += *v;
        }
        res
    } else if n <= 128 {
        let mut r = [a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]];
        let mut i = 8;
        while i < n - (n % 8) {
            for (j, acc) in r.iter_mut().enumerate() {
                *acc += a[i + j];
            }
            i += 8;
        }
        let mut res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        while i < n {
            res += a[i];
            i += 1;
        }
        res
    } else {
        let mut n2 = n / 2;
        n2 -= n2 % 8;
        pairwise_sum(&a[..n2]) + pairwise_sum(&a[n2..])
    }
}

/// `np.average(values)` of a float64 array: the `add.reduce` (starting from the identity 0)
/// divided by the element count.
pub fn numpy_average(values: &[f64]) -> f64 {
    let sum = 0. + pairwise_sum(values);
    sum / (values.len() as f64)
}

fn seq_items(v: &PyValue) -> Option<&Vec<PyValue>> {
    match v {
        PyValue::List(items) | PyValue::Tuple(items) => Some(items),
        _ => None,
    }
}

/// NumPy's object-to-float64 conversion of a sample element (`MyPyFloat_AsDouble`): `None`
/// becomes NaN, anything else goes through `float(v)`.
fn float_of(v: &PyValue) -> KResult<f64> {
    match v {
        PyValue::None => Ok(f64::NAN),
        PyValue::Float(f) => Ok(*f),
        PyValue::Bool(b) => Ok(if *b { 1. } else { 0. }),
        PyValue::Int(n) => n
            .to_f64_exact()
            .ok_or_else(|| other("OverflowError", "int too large to convert to float")),
        PyValue::Str(s) => float_from_string(s).map_err(py_error),
        other_value => Err(other(
            "TypeError",
            format!(
                "float() argument must be a string or a number, not '{}'",
                type_name(other_value)
            ),
        )),
    }
}

/// `np.array(rows)[:, column].astype(float)`: the rows must form a 2-D array that has the
/// column.
fn numpy_column(rows: &[PyValue], column: usize) -> KResult<Vec<f64>> {
    let too_many = || other("IndexError", "too many indices for array");
    let first = rows.first().and_then(seq_items).ok_or_else(too_many)?;
    let width = first.len();
    let mut items = Vec::with_capacity(rows.len());
    for row in rows {
        match seq_items(row) {
            Some(r) if r.len() == width => items.push(r),
            _ => return Err(too_many()),
        }
    }
    if column >= width {
        return Err(other(
            "IndexError",
            format!("index {column} is out of bounds for axis 1 with size {width}"),
        ));
    }
    items.iter().map(|r| float_of(&r[column])).collect()
}

/// `np.average(np.array(samples)[:, 2].astype(float))`: the average of the counts column of
/// the collected sample rows (the tare value).
pub fn average_sample_counts(samples: &[PyValue]) -> KResult<f64> {
    Ok(numpy_average(&numpy_column(samples, 2)?))
}

/// `np.asarray(rows, dtype=np.float64)[:, column]`.
fn numpy_float64_column(rows: &[PyValue], column: usize) -> KResult<Vec<f64>> {
    let first = match rows.first() {
        None => return Err(other("IndexError", "too many indices for array")),
        Some(first) => first,
    };
    let Some(first) = seq_items(first) else {
        // A 1-D array cannot take `[:, i]`.
        for row in rows {
            float_of(row)?;
        }
        return Err(other("IndexError", "too many indices for array"));
    };
    let width = first.len();
    let mut table = Vec::with_capacity(rows.len());
    for row in rows {
        match seq_items(row) {
            Some(r) if r.len() == width => {
                let mut converted = Vec::with_capacity(width);
                for item in r {
                    if seq_items(item).is_some() {
                        return Err(other("ValueError", "setting an array element with a sequence."));
                    }
                    converted.push(float_of(item)?);
                }
                table.push(converted);
            }
            _ => return Err(other("ValueError", "setting an array element with a sequence.")),
        }
    }
    if column >= width {
        return Err(other(
            "IndexError",
            format!("index {column} is out of bounds for axis 1 with size {width}"),
        ));
    }
    Ok(table.iter().map(|r| r[column]).collect())
}

/// `TapAnalysis` (`load_cell_probe.py:19`).
#[derive(Clone, Debug, PartialEq)]
pub struct TapAnalysis {
    /// `self.time`
    pub time: Vec<f64>,
    /// `self.force`
    pub force: Vec<f64>,
}

impl TapAnalysis {
    /// `TapAnalysis(samples)`.
    pub fn new(samples: &[PyValue]) -> KResult<TapAnalysis> {
        let time = numpy_float64_column(samples, 0)?;
        let force = numpy_float64_column(samples, 1)?;
        Ok(TapAnalysis { time, force })
    }

    /// `to_dict()`: the dictionary for the JSON encoder.
    pub fn to_dict(&self) -> PyValue {
        let floats = |values: &[f64]| PyValue::List(values.iter().map(|v| PyValue::Float(*v)).collect());
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("time".to_string()), floats(&self.time));
        d.insert(PyValue::Str("force".to_string()), floats(&self.force));
        d.insert(PyValue::Str("is_valid".to_string()), PyValue::Bool(true));
        PyValue::Dict(d)
    }
}

// ---------------------------------------------------------------------------
// ParamHelper
// ---------------------------------------------------------------------------

/// A Python number that keeps its `int`/`float` type: option limits are rendered with `%s`
/// in error messages (`10` versus `10.0`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Num {
    /// A Python `int`.
    Int(i64),
    /// A Python `float`.
    Float(f64),
}

impl Num {
    /// The value as a double (`float(n)`).
    pub fn as_f64(self) -> f64 {
        match self {
            Num::Int(i) => i as f64,
            Num::Float(f) => f,
        }
    }

    /// `bool(n)`.
    pub fn truthy(self) -> bool {
        match self {
            Num::Int(i) => i != 0,
            Num::Float(f) => f != 0.,
        }
    }

    /// `str(n)`.
    pub fn py_str(self) -> String {
        match self {
            Num::Int(i) => i.to_string(),
            Num::Float(f) => float_repr(f),
        }
    }

    fn to_py(self) -> PyValue {
        match self {
            Num::Int(i) => PyValue::Int(PyInt::from_i64(i)),
            Num::Float(f) => PyValue::Float(f),
        }
    }

    /// The Python type name, as used in `TypeError` texts.
    fn type_name(self) -> &'static str {
        match self {
            Num::Int(_) => "int",
            Num::Float(_) => "float",
        }
    }
}

/// Python's `a or b` for two optional numbers.
fn or_num(a: Option<Num>, b: Option<Num>) -> Option<Num> {
    match a {
        Some(n) if n.truthy() => a,
        _ => b,
    }
}

/// A list of floats that keeps Python's `list` versus `tuple` type: the two never compare
/// equal.
#[derive(Clone, Debug, PartialEq)]
pub enum FloatSeq {
    /// A `list` (defaults, values parsed from a G-code command).
    List(Vec<f64>),
    /// A `tuple` (what `config.getfloatlist` returns for a present option).
    Tuple(Vec<f64>),
}

impl FloatSeq {
    /// The elements.
    pub fn items(&self) -> &[f64] {
        match self {
            FloatSeq::List(v) | FloatSeq::Tuple(v) => v,
        }
    }

    /// `bool(seq)`.
    pub fn truthy(&self) -> bool {
        !self.items().is_empty()
    }
}

/// The value of a [`ParamHelper`]: `None`, a number or a list of floats.
#[derive(Clone, Debug, PartialEq)]
pub enum ParamValue {
    /// `None`
    None,
    /// An `int` or `float`.
    Num(Num),
    /// A list or tuple of floats.
    List(FloatSeq),
}

impl ParamValue {
    /// The number, when the value is one.
    pub fn num(&self) -> Option<Num> {
        match self {
            ParamValue::Num(n) => Some(*n),
            _ => None,
        }
    }

    /// The integer, when the value is an `int`.
    pub fn int(&self) -> Option<i64> {
        match self {
            ParamValue::Num(Num::Int(i)) => Some(*i),
            _ => None,
        }
    }

    /// The list, when the value is one.
    pub fn list(&self) -> Option<&FloatSeq> {
        match self {
            ParamValue::List(l) => Some(l),
            _ => None,
        }
    }
}

/// Which of the three helper flavours a [`ParamHelper`] is (`type_name`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParamType {
    /// `'int'`
    Int,
    /// `'float'`
    Float,
    /// `'float_list'`
    FloatList,
}

/// The constraints of a [`ParamHelper`] (the `minval`, `maxval`, `above`, `below` and
/// `max_len` constructor arguments).
#[derive(Clone, Copy, Debug, Default)]
pub struct ParamLimits {
    /// `minval`
    pub minval: Option<Num>,
    /// `maxval`
    pub maxval: Option<Num>,
    /// `above`
    pub above: Option<Num>,
    /// `below`
    pub below: Option<Num>,
    /// `max_len`
    pub max_len: Option<i64>,
}

#[derive(Clone, Copy)]
enum ErrKind {
    Config,
    Command,
}

fn make_error(kind: ErrKind, message: String) -> KlippyError {
    match kind {
        ErrKind::Config => KlippyError::Config(message),
        ErrKind::Command => KlippyError::Command(message),
    }
}

/// Where a parameter is read from.
enum Source<'a> {
    Config(&'a Config),
    Gcmd(&'a GCmd),
}

/// Access a parameter from config or a G-code command via a consistent interface; stores the
/// name and constraints (`ParamHelper`, `load_cell_probe.py:35`).
pub struct ParamHelper {
    config_section: String,
    /// `self.name`
    pub name: String,
    type_name: ParamType,
    /// `self.value`: the default, replaced by the config value at construction.
    pub value: ParamValue,
    /// `self.minval`
    pub minval: Option<Num>,
    /// `self.maxval`
    pub maxval: Option<Num>,
    /// `self.above`
    pub above: Option<Num>,
    /// `self.below`
    pub below: Option<Num>,
    /// `self.max_len`
    pub max_len: Option<i64>,
}

impl ParamHelper {
    /// `ParamHelper(config, name, type_name, default, ...)`: reads the option from the config
    /// once.
    pub fn new(
        config: &Config,
        name: &str,
        type_name: ParamType,
        default: ParamValue,
        limits: ParamLimits,
    ) -> KResult<ParamHelper> {
        let mut helper = ParamHelper {
            config_section: config.get_name(),
            name: name.to_string(),
            type_name,
            value: default,
            minval: limits.minval,
            maxval: limits.maxval,
            above: limits.above,
            below: limits.below,
            max_len: limits.max_len,
        };
        // read from config once
        helper.value = helper.get(None, None, None, None, None, Some(config))?;
        Ok(helper)
    }

    fn get_name(&self, gcmd: bool) -> String {
        if gcmd {
            self.name.to_uppercase()
        } else {
            self.name.clone()
        }
    }

    /// The default as the Python object `self.value`.
    fn value_py(&self) -> PyValue {
        match &self.value {
            ParamValue::None => PyValue::None,
            ParamValue::Num(n) => n.to_py(),
            ParamValue::List(FloatSeq::List(v)) => {
                PyValue::List(v.iter().map(|f| PyValue::Float(*f)).collect())
            }
            ParamValue::List(FloatSeq::Tuple(v)) => {
                PyValue::Tuple(v.iter().map(|f| PyValue::Float(*f)).collect())
            }
        }
    }

    /// `_validate_float` (`load_cell_probe.py:54`).
    fn validate_float(
        &self,
        description: &str,
        kind: ErrKind,
        value: f64,
        above: Option<Num>,
        below: Option<Num>,
    ) -> KResult<()> {
        let above = or_num(above, self.above);
        if let Some(above) = above {
            if value <= above.as_f64() {
                return Err(make_error(
                    kind,
                    format!("{} must be above {}", description, above.py_str()),
                ));
            }
        }
        let below = or_num(below, self.below);
        if let Some(below) = below {
            if value >= below.as_f64() {
                return Err(make_error(
                    kind,
                    format!("{} must be below {}", description, below.py_str()),
                ));
            }
        }
        Ok(())
    }

    /// `_validate_float_list` (`load_cell_probe.py:63`): validates the options of a list.
    fn validate_float_list(
        &self,
        gcmd: Option<&GCmd>,
        values: &[f64],
        above: Option<Num>,
        below: Option<Num>,
    ) -> KResult<()> {
        let (description, kind) = match gcmd {
            Some(gcmd) => (
                format!("Error on '{}': {}", gcmd.get_commandline(), self.get_name(true)),
                ErrKind::Command,
            ),
            None => (
                format!("Option '{}' in section '{}'", self.get_name(false), self.config_section),
                ErrKind::Config,
            ),
        };
        if let Some(max_len) = self.max_len {
            if (values.len() as i64) > max_len {
                return Err(make_error(
                    kind,
                    format!("{} has maximum length {}", description, max_len),
                ));
            }
        }
        for value in values {
            self.validate_float(&description, kind, *value, above, below)?;
        }
        Ok(())
    }

    fn bound_int(bound: Option<Num>) -> KResult<Option<i64>> {
        match bound {
            None => Ok(None),
            Some(Num::Int(i)) => Ok(Some(i)),
            Some(Num::Float(_)) => Err(other(
                "TypeError",
                "'float' object cannot be interpreted as an integer",
            )),
        }
    }

    /// `_get_int` (`load_cell_probe.py:78`).
    fn read_int(
        &self,
        source: Source<'_>,
        minval: Option<Num>,
        maxval: Option<Num>,
    ) -> KResult<ParamValue> {
        let minval = Self::bound_int(or_num(minval, self.minval))?;
        let maxval = Self::bound_int(or_num(maxval, self.maxval))?;
        let default = match &self.value {
            ParamValue::None => None,
            ParamValue::Num(Num::Int(i)) => Some(*i),
            _ => {
                return Err(other(
                    "TypeError",
                    "int parameter default must be an int or None",
                ))
            }
        };
        let value = match source {
            Source::Gcmd(gcmd) => gcmd.get_int(
                &self.get_name(true),
                GcmdIntOpts { default: Some(default), minval, maxval },
            )?,
            Source::Config(config) => config.getint(
                &self.get_name(false),
                GetIntOpts { default: Some(default), minval, maxval, note_valid: true },
            )?,
        };
        Ok(match value {
            Some(i) => ParamValue::Num(Num::Int(i)),
            None => ParamValue::None,
        })
    }

    /// `_get_float` (`load_cell_probe.py:83`).
    fn read_float(
        &self,
        source: Source<'_>,
        minval: Option<Num>,
        maxval: Option<Num>,
        above: Option<Num>,
        below: Option<Num>,
    ) -> KResult<ParamValue> {
        let minval = or_num(minval, self.minval);
        let maxval = or_num(maxval, self.maxval);
        let above = or_num(above, self.above);
        let below = or_num(below, self.below);
        match source {
            Source::Gcmd(gcmd) => {
                // `gcmd.get_float`, with the limits as Python objects so that an int limit
                // keeps its text in the error message.
                let to_py = |n: Option<Num>| n.map(Num::to_py);
                let value = gcmd.get(
                    &self.get_name(true),
                    GcmdGetOpts {
                        default: Some(self.value_py()),
                        parser: GetParser::Float,
                        minval: to_py(minval),
                        maxval: to_py(maxval),
                        above: to_py(above),
                        below: to_py(below),
                    },
                )?;
                match value {
                    PyValue::Float(f) => Ok(ParamValue::Num(Num::Float(f))),
                    PyValue::Int(n) => Ok(ParamValue::Num(Num::Int(int_to_i64(&n)?))),
                    _ => Ok(ParamValue::None),
                }
            }
            Source::Config(config) => {
                let default = match &self.value {
                    ParamValue::None => None,
                    ParamValue::Num(n) => Some(n.as_f64()),
                    ParamValue::List(_) => {
                        return Err(other("TypeError", "float parameter default must be a number"))
                    }
                };
                // The limits are checked here rather than by `getfloat`: the message renders
                // them with `%s`, and an int limit (`minval=10`) prints without ".0".
                let parsed = config.getfloat(
                    &self.get_name(false),
                    GetFloatOpts { default: Some(default), note_valid: true, ..GetFloatOpts::default() },
                )?;
                let present = config
                    .get(
                        &self.get_name(false),
                        GetStrOpts { default: Some(None), note_valid: false },
                    )?
                    .is_some();
                let value = match (present, parsed) {
                    (true, Some(v)) => v,
                    // An absent option returns the default as is, without validation.
                    _ => return Ok(self.value.clone()),
                };
                let section = &self.config_section;
                let option = &self.name;
                let fail = |what: &str, limit: Num| {
                    KlippyError::Config(format!(
                        "Option '{option}' in section '{section}' {what} {}",
                        limit.py_str()
                    ))
                };
                if let Some(m) = minval {
                    if value < m.as_f64() {
                        return Err(fail("must have minimum of", m));
                    }
                }
                if let Some(m) = maxval {
                    if value > m.as_f64() {
                        return Err(fail("must have maximum of", m));
                    }
                }
                if let Some(a) = above {
                    if value <= a.as_f64() {
                        return Err(fail("must be above", a));
                    }
                }
                if let Some(b) = below {
                    if value >= b.as_f64() {
                        return Err(fail("must be below", b));
                    }
                }
                Ok(ParamValue::Num(Num::Float(value)))
            }
        }
    }

    /// `_get_float_list` (`load_cell_probe.py:88`).
    fn read_float_list(
        &self,
        source: Source<'_>,
        above: Option<Num>,
        below: Option<Num>,
    ) -> KResult<ParamValue> {
        // this code defaults to the empty list, never return None
        let default = match &self.value {
            ParamValue::List(l) if l.truthy() => l.clone(),
            _ => FloatSeq::List(Vec::new()),
        };
        let (float_list, gcmd) = match source {
            Source::Gcmd(gcmd) => {
                let name = self.get_name(true);
                // if the parameter isn't part of the command, return the default
                if !gcmd.get_command_parameters().contains_key(&name) {
                    return Ok(ParamValue::List(default));
                }
                // parameter exists, always prefer whatever is in the command
                let value = gcmd.get(
                    &name,
                    GcmdGetOpts { default: Some(PyValue::Str(String::new())), ..GcmdGetOpts::default() },
                )?;
                let value = match value {
                    PyValue::Str(s) => s,
                    other_value => other_value.str(),
                };
                // Return an empty list for empty value
                if value.trim_matches(char::is_whitespace).is_empty() {
                    return Ok(ParamValue::List(FloatSeq::List(Vec::new())));
                }
                let parsed: Result<Vec<f64>, PyError> = value
                    .split(',')
                    .map(|p| float_from_string(p.trim_matches(char::is_whitespace)))
                    .collect();
                match parsed {
                    Ok(list) => (FloatSeq::List(list), Some(gcmd)),
                    Err(_) => {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': unable to parse {}",
                            gcmd.get_commandline(),
                            value
                        )))
                    }
                }
            }
            Source::Config(config) => {
                let name = self.get_name(false);
                let list = config.getfloatlist(
                    &name,
                    GetFloatListOpts {
                        default: Some(Some(default.items().to_vec())),
                        ..GetFloatListOpts::default()
                    },
                )?;
                let present = config
                    .get(&name, GetStrOpts { default: Some(None), note_valid: false })?
                    .is_some();
                let seq = match (present, list) {
                    // `getfloatlist` returns a tuple for a present option, the default as is
                    // otherwise.
                    (true, Some(v)) => FloatSeq::Tuple(v),
                    _ => default,
                };
                (seq, None)
            }
        };
        if float_list.truthy() {
            self.validate_float_list(gcmd, float_list.items(), above, below)?;
        }
        Ok(ParamValue::List(float_list))
    }

    /// `get(gcmd=None, minval=None, maxval=None, above=None, below=None, config=None)`.
    pub fn get(
        &self,
        gcmd: Option<&GCmd>,
        minval: Option<Num>,
        maxval: Option<Num>,
        above: Option<Num>,
        below: Option<Num>,
        config: Option<&Config>,
    ) -> KResult<ParamValue> {
        let source = match (gcmd, config) {
            (None, None) => return Ok(self.value.clone()),
            (Some(gcmd), _) => Source::Gcmd(gcmd),
            (None, Some(config)) => Source::Config(config),
        };
        match self.type_name {
            ParamType::Int => self.read_int(source, minval, maxval),
            ParamType::Float => self.read_float(source, minval, maxval, above, below),
            ParamType::FloatList => self.read_float_list(source, above, below),
        }
    }

    /// `get(gcmd)` of an int parameter.
    pub fn get_int(&self, gcmd: Option<&GCmd>) -> KResult<Option<i64>> {
        Ok(self.get(gcmd, None, None, None, None, None)?.int())
    }

    /// `get(gcmd)` of a float parameter.
    pub fn get_float(&self, gcmd: Option<&GCmd>) -> KResult<Option<Num>> {
        Ok(self.get(gcmd, None, None, None, None, None)?.num())
    }
}

/// `intParamHelper(config, name, default=None, minval=None, maxval=None)`.
pub fn int_param_helper(
    config: &Config,
    name: &str,
    default: Option<i64>,
    minval: Option<i64>,
    maxval: Option<i64>,
) -> KResult<ParamHelper> {
    ParamHelper::new(
        config,
        name,
        ParamType::Int,
        default.map_or(ParamValue::None, |d| ParamValue::Num(Num::Int(d))),
        ParamLimits {
            minval: minval.map(Num::Int),
            maxval: maxval.map(Num::Int),
            ..ParamLimits::default()
        },
    )
}

/// `floatParamHelper(config, name, default=None, minval=None, maxval=None, above=None,
/// below=None)`.
pub fn float_param_helper(
    config: &Config,
    name: &str,
    default: Option<Num>,
    limits: ParamLimits,
) -> KResult<ParamHelper> {
    ParamHelper::new(
        config,
        name,
        ParamType::Float,
        default.map_or(ParamValue::None, ParamValue::Num),
        limits,
    )
}

/// `floatListParamHelper(config, name, default=None, above=None, below=None, max_len=None)`.
pub fn float_list_param_helper(
    config: &Config,
    name: &str,
    default: Option<FloatSeq>,
    limits: ParamLimits,
) -> KResult<ParamHelper> {
    ParamHelper::new(
        config,
        name,
        ParamType::FloatList,
        default.map_or(ParamValue::None, ParamValue::List),
        limits,
    )
}

// ---------------------------------------------------------------------------
// Continuous tare filter
// ---------------------------------------------------------------------------

/// Container for filter parameters; allows different filter configurations to be compared
/// (`ContinuousTareFilter`, `load_cell_probe.py:143`).
#[derive(Clone, Debug, PartialEq)]
pub struct ContinuousTareFilter {
    /// `sps`
    pub sps: f64,
    /// `drift`
    pub drift: Option<f64>,
    /// `drift_delay`
    pub drift_delay: Option<i64>,
    /// `buzz`
    pub buzz: Option<f64>,
    /// `buzz_delay`
    pub buzz_delay: Option<i64>,
    /// `notches`
    pub notches: FloatSeq,
    /// `notch_quality`
    pub notch_quality: Option<f64>,
}

fn order_of(delay: Option<i64>) -> KResult<i64> {
    delay.ok_or_else(|| {
        other(
            "TypeError",
            "int() argument must be a string, a bytes-like object or a number, not 'NoneType'",
        )
    })
}

impl ContinuousTareFilter {
    /// `design_filter(error_func)`: creates a filter design from the parameters. `cfg_error`
    /// is `config.error` or `gcmd.error`.
    pub fn design_filter(&self, cfg_error: CfgErrorKind) -> KResult<DigitalFilter> {
        let df = DigitalFilter::new(self.sps, cfg_error);
        if let Some(drift) = self.drift.filter(|d| *d != 0.) {
            df.add_highpass(drift, order_of(self.drift_delay)?)?;
        }
        if let Some(buzz) = self.buzz.filter(|b| *b != 0.) {
            df.add_lowpass(buzz, order_of(self.buzz_delay)?)?;
        }
        for notch in self.notches.items() {
            let quality = self.notch_quality.ok_or_else(|| {
                other("TypeError", "unsupported operand type(s) for +: 'NoneType' and 'float'")
            })?;
            df.add_notch(*notch, quality)?;
        }
        Ok(df)
    }
}

/// Combine [`ContinuousTareFilter`] and the MCU SOS filter into an easy-to-use class
/// (`ContinuousTareFilterHelper`, `load_cell_probe.py:177`).
pub struct ContinuousTareFilterHelper {
    sps: f64,
    sos_filter: Arc<MCU_SosFilter>,
    drift_param: ParamHelper,
    drift_delay_param: ParamHelper,
    buzz_param: ParamHelper,
    buzz_delay_param: ParamHelper,
    notches_param: ParamHelper,
    notch_quality_param: ParamHelper,
    /// `_active_design`: the filter design currently inside the MCU.
    active_design: ContinuousTareFilter,
}

impl ContinuousTareFilterHelper {
    /// `ContinuousTareFilterHelper(config, sensor, sos_filter)`.
    pub fn new(
        config: &Config,
        sensor: &Arc<dyn BulkSensorAdc>,
        sos_filter: Arc<MCU_SosFilter>,
    ) -> KResult<ContinuousTareFilterHelper> {
        let sps = sensor.get_samples_per_second();
        let max_filter_frequency = math_floor(sps / 2.)?;
        // setup filter parameters
        let drift_param = float_param_helper(
            config,
            "drift_filter_cutoff_frequency",
            None,
            ParamLimits {
                minval: Some(Num::Float(0.1)),
                maxval: Some(Num::Float(20.0)),
                ..ParamLimits::default()
            },
        )?;
        let drift_delay_param = int_param_helper(config, "drift_filter_delay", Some(2), Some(1), Some(2))?;
        let upper = (max_filter_frequency as f64) - 1.0;
        // `min(80.0, max_filter_frequency - 1.0)`
        let buzz_above = if upper < 80.0 { upper } else { 80.0 };
        let buzz_param = float_param_helper(
            config,
            "buzz_filter_cutoff_frequency",
            None,
            ParamLimits {
                above: Some(Num::Float(buzz_above)),
                below: Some(Num::Int(max_filter_frequency)),
                ..ParamLimits::default()
            },
        )?;
        let buzz_delay_param = int_param_helper(config, "buzz_filter_delay", Some(2), Some(1), Some(2))?;
        let notches_param = float_list_param_helper(
            config,
            "notch_filter_frequencies",
            Some(FloatSeq::List(Vec::new())),
            ParamLimits {
                above: Some(Num::Float(0.)),
                below: Some(Num::Int(max_filter_frequency)),
                max_len: Some(2),
                ..ParamLimits::default()
            },
        )?;
        let notch_quality_param = float_param_helper(
            config,
            "notch_filter_quality",
            Some(Num::Float(2.0)),
            ParamLimits {
                minval: Some(Num::Float(0.5)),
                maxval: Some(Num::Float(6.0)),
                ..ParamLimits::default()
            },
        )?;
        let mut helper = ContinuousTareFilterHelper {
            sps,
            sos_filter,
            drift_param,
            drift_delay_param,
            buzz_param,
            buzz_delay_param,
            notches_param,
            notch_quality_param,
            // replaced right below, once the parameters can be read
            active_design: ContinuousTareFilter {
                sps,
                drift: None,
                drift_delay: None,
                buzz: None,
                buzz_delay: None,
                notches: FloatSeq::List(Vec::new()),
                notch_quality: None,
            },
        };
        // filter design specified in the config file, used for defaults
        helper.active_design = helper.build_filter(None)?;
        // filter design currently inside the MCU
        let design = helper.active_design.design_filter(CfgErrorKind::Config)?;
        helper.sos_filter.set_filter_design(Some(Arc::new(design)));
        Ok(helper)
    }

    /// `_build_filter(gcmd=None)`.
    pub fn build_filter(&self, gcmd: Option<&GCmd>) -> KResult<ContinuousTareFilter> {
        let drift = self.drift_param.get_float(gcmd)?;
        let drift_delay = self.drift_delay_param.get_int(gcmd)?;
        let buzz = self.buzz_param.get_float(gcmd)?;
        let buzz_delay = self.buzz_delay_param.get_int(gcmd)?;
        // notches must be between drift and buzz:
        let notches = self.notches_param.get(gcmd, None, None, drift, buzz, None)?;
        let notch_quality = self.notch_quality_param.get_float(gcmd)?;
        Ok(ContinuousTareFilter {
            sps: self.sps,
            drift: drift.map(Num::as_f64),
            drift_delay,
            buzz: buzz.map(Num::as_f64),
            buzz_delay,
            notches: notches.list().cloned().unwrap_or(FloatSeq::List(Vec::new())),
            notch_quality: notch_quality.map(Num::as_f64),
        })
    }

    /// `update_from_command(gcmd)`.
    pub fn update_from_command(&self, gcmd: &GCmd) -> KResult<()> {
        let gcmd_filter = self.build_filter(Some(gcmd))?;
        // if filters are identical, no change required
        if self.active_design == gcmd_filter {
            return Ok(());
        }
        // update MCU filter from GCode command. The reference designs the *active* filter
        // here, not `gcmd_filter`, and never updates `_active_design`.
        let design = self.active_design.design_filter(CfgErrorKind::Command)?;
        self.sos_filter.set_filter_design(Some(Arc::new(design)));
        Ok(())
    }
}

/// `check_sensor_errors(results, printer)`: check results from the collector for errors and
/// raise an exception if found.
pub fn check_sensor_errors(results: &CollectResult) -> KResult<Vec<PyValue>> {
    if let Some((errors, overflows)) = results.errors {
        return Err(KlippyError::Command(format!(
            "Load cell sensor reported errors while probing: {errors} errors, {overflows} overflows"
        )));
    }
    Ok(results.samples.clone())
}

// ---------------------------------------------------------------------------
// Config helper
// ---------------------------------------------------------------------------

/// `LoadCellProbeConfigHelper` (`load_cell_probe.py:239`).
pub struct LoadCellProbeConfigHelper {
    load_cell: Arc<LoadCell>,
    sensor: Arc<dyn BulkSensorAdc>,
    tare_time_param: ParamHelper,
    trigger_force_param: ParamHelper,
    force_safety_limit_param: ParamHelper,
}

fn none_operand(op: &str, right: &str) -> KlippyError {
    other(
        "TypeError",
        format!("unsupported operand type(s) for {op}: 'NoneType' and '{right}'"),
    )
}

impl LoadCellProbeConfigHelper {
    /// `LoadCellProbeConfigHelper(config, load_cell_inst)`.
    pub fn new(config: &Config, load_cell_inst: &Arc<LoadCell>) -> KResult<LoadCellProbeConfigHelper> {
        let sensor = load_cell_inst.get_sensor();
        // Collect 4 x 60hz power cycles of data to average across power noise
        let tare_time_param = float_param_helper(
            config,
            "tare_time",
            Some(Num::Float(4. / 60.)),
            ParamLimits {
                minval: Some(Num::Float(0.01)),
                maxval: Some(Num::Float(1.0)),
                ..ParamLimits::default()
            },
        )?;
        // triggering options
        let trigger_force_param = float_param_helper(
            config,
            "trigger_force",
            Some(Num::Int(75)),
            ParamLimits {
                minval: Some(Num::Int(10)),
                maxval: Some(Num::Int(250)),
                ..ParamLimits::default()
            },
        )?;
        let force_safety_limit_param = float_param_helper(
            config,
            "force_safety_limit",
            Some(Num::Int(2000)),
            ParamLimits {
                minval: Some(Num::Int(100)),
                maxval: Some(Num::Int(5000)),
                ..ParamLimits::default()
            },
        )?;
        Ok(LoadCellProbeConfigHelper {
            load_cell: load_cell_inst.clone(),
            sensor,
            tare_time_param,
            trigger_force_param,
            force_safety_limit_param,
        })
    }

    /// `get_tare_samples(gcmd=None)`.
    pub fn get_tare_samples(&self, gcmd: Option<&GCmd>) -> KResult<i64> {
        let tare_time = self
            .tare_time_param
            .get_float(gcmd)?
            .ok_or_else(|| none_operand("*", "float"))?;
        let sps = self.sensor.get_samples_per_second();
        Ok(std::cmp::max(2, math_ceil(tare_time.as_f64() * sps)?))
    }

    /// `get_trigger_force_grams(gcmd=None)`.
    pub fn get_trigger_force_grams(&self, gcmd: Option<&GCmd>) -> KResult<Option<Num>> {
        self.trigger_force_param.get_float(gcmd)
    }

    /// `get_safety_limit_grams(gcmd=None)`.
    pub fn get_safety_limit_grams(&self, gcmd: Option<&GCmd>) -> KResult<Option<Num>> {
        self.force_safety_limit_param.get_float(gcmd)
    }

    /// `get_safety_range(gcmd=None)`.
    pub fn get_safety_range(&self, gcmd: Option<&GCmd>) -> KResult<(i64, i64)> {
        let counts_per_gram = self.load_cell.get_counts_per_gram();
        // calculate the safety band
        let zero = self.load_cell.get_reference_tare_counts();
        let limit = self.get_safety_limit_grams(gcmd)?;
        let counts_per_gram = counts_per_gram.ok_or_else(|| {
            none_operand("*", limit.map_or("NoneType", Num::type_name))
        })?;
        let limit = limit
            .ok_or_else(|| other("TypeError", "unsupported operand type(s) for *: 'float' and 'NoneType'"))?;
        let safety_counts = float_to_int(counts_per_gram * limit.as_f64())?;
        let zero = zero.ok_or_else(|| none_operand("-", "int"))?;
        let zero_big = PyInt::from_i64(zero).0;
        let safety_min = PyInt(&zero_big - &safety_counts.0);
        let safety_max = PyInt(&zero_big + &safety_counts.0);
        // don't allow a safety range outside the sensor's real range
        let (sensor_min, sensor_max) = self.load_cell.get_sensor().get_range();
        if safety_min.0 <= PyInt::from_i64(sensor_min).0 || safety_max.0 >= PyInt::from_i64(sensor_max).0 {
            return Err(KlippyError::Command(
                "Load cell force_safety_limit exceeds sensor range!".to_string(),
            ));
        }
        Ok((int_to_i64(&safety_min)?, int_to_i64(&safety_max)?))
    }

    /// `get_grams_per_count()`: calculate 1/counts_per_gram.
    pub fn get_grams_per_count(&self) -> KResult<f64> {
        let counts_per_gram = self.load_cell.get_counts_per_gram().ok_or_else(|| {
            other(
                "TypeError",
                "'>=' not supported between instances of 'NoneType' and 'int'",
            )
        })?;
        // The counts_per_gram could be so large that it becomes 0.0 when
        // sent to the mcu. This would mean the ADC range only measures
        // a few grams which seems very unlikely. Treat this as an error:
        if counts_per_gram >= (1u64 << 29) as f64 {
            return Err(other("OverflowError", "counts_per_gram value is too large to filter"));
        }
        if counts_per_gram == 0. {
            return Err(other("ZeroDivisionError", "float division by zero"));
        }
        Ok(1. / counts_per_gram)
    }
}

// ---------------------------------------------------------------------------
// Probing move
// ---------------------------------------------------------------------------

fn lookup_toolhead(printer: &Printer) -> KResult<Arc<dyn ToolHeadApi>> {
    Ok(lookup_interface::<dyn ToolHeadApi>(&**printer, "toolhead", LookupDefault::Raise)?
        .expect("Raise never yields None"))
}

/// Execute probing moves using the `MCU_trigger_analog` (`LoadCellProbingMove`,
/// `load_cell_probe.py:290`).
pub struct LoadCellProbingMove {
    printer: Printer,
    load_cell: Arc<LoadCell>,
    mcu_trigger_analog: Arc<MCU_trigger_analog>,
    param_helper: Arc<ProbeParameterHelper>,
    continuous_tare_filter_helper: ContinuousTareFilterHelper,
    config_helper: LoadCellProbeConfigHelper,
    z_min_position: f64,
    /// `_tare_counts`: never updated by the reference.
    tare_counts: i64,
}

impl LoadCellProbingMove {
    /// `LoadCellProbingMove(config, load_cell_inst, mcu_trigger_analog, param_helper,
    /// continuous_tare_filter_helper, config_helper)`.
    pub fn new(
        config: &Config,
        load_cell_inst: Arc<LoadCell>,
        mcu_trigger_analog: Arc<MCU_trigger_analog>,
        param_helper: Arc<ProbeParameterHelper>,
        continuous_tare_filter_helper: ContinuousTareFilterHelper,
        config_helper: LoadCellProbeConfigHelper,
    ) -> KResult<Arc<LoadCellProbingMove>> {
        let printer = config.get_printer();
        let z_min_position = lookup_minimum_z(config)?;
        let dispatch = mcu_trigger_analog.get_dispatch();
        LookupZSteppers::new(
            config,
            Box::new(move |stepper| {
                // `AddStepperCb` has no error channel: the reference lets the multi-mcu
                // `config_error` of `TriggerDispatch.add_stepper` escape the handler.
                if let Err(e) = dispatch.add_stepper(stepper) {
                    panic!("{}: {}", e.class_name(), e.message());
                }
            }),
        );
        Ok(Arc::new(LoadCellProbingMove {
            printer,
            load_cell: load_cell_inst,
            mcu_trigger_analog,
            param_helper,
            continuous_tare_filter_helper,
            config_helper,
            z_min_position,
            // internal state tracking
            tare_counts: 0,
        }))
    }

    fn start_collector(&self) -> KResult<Arc<LoadCellSampleCollector>> {
        let toolhead = lookup_toolhead(&self.printer)?;
        // homing uses the toolhead last move time which gets special handling
        // to significantly buffer print_time if the move queue has drained
        let print_time = toolhead.get_last_move_time();
        let collector = self.load_cell.get_collector();
        collector.start_collecting(Some(print_time));
        Ok(collector)
    }

    /// Pauses for the last move to complete and then sets the endstop tare value and range
    /// (`_pause_and_tare`).
    fn pause_and_tare(&self, gcmd: &GCmd) -> KResult<()> {
        let collector = self.start_collector()?;
        let num_samples = self.config_helper.get_tare_samples(Some(gcmd))?;
        // use collect_min so collected samples are not wasted
        let results = collector.collect_min(num_samples as f64)?;
        let tare_samples = check_sensor_errors(&results)?;
        let tare_counts = average_sample_counts(&tare_samples)?;
        // update sos_filter with any gcode parameter changes
        self.continuous_tare_filter_helper.update_from_command(gcmd)?;
        // update the load cell so it reflects the new tare value
        self.load_cell.tare(tare_counts)?;
        // update raw range
        let (safety_min, safety_max) = self.config_helper.get_safety_range(Some(gcmd))?;
        self.mcu_trigger_analog.set_raw_range(safety_min, safety_max);
        // update internal tare value
        let gpc = self.config_helper.get_grams_per_count()? * FRAC_GRAMS_CONV;
        let sos_filter = self.mcu_trigger_analog.get_sos_filter().ok_or_else(|| {
            other("AttributeError", "'NoneType' object has no attribute 'set_offset_scale'")
        })?;
        sos_filter.set_offset_scale(int_of_float(-tare_counts)?, gpc, false);
        // update trigger
        let trigger_val = self
            .config_helper
            .get_trigger_force_grams(Some(gcmd))?
            .ok_or_else(|| none_operand("*", "float"))?;
        let trigger_frac_grams = int_of_float(trigger_val.as_f64() * FRAC_GRAMS_CONV)?;
        self.mcu_trigger_analog
            .set_trigger("abs_ge".to_string(), PyValue::Int(PyInt::from_i64(trigger_frac_grams)));
        Ok(())
    }

    /// Probe towards z_min until the trigger_analog on the MCU triggers; returns the end
    /// position and the sample collector.
    pub fn probing_move(&self, gcmd: &GCmd) -> KResult<(Vec<f64>, Arc<LoadCellSampleCollector>)> {
        // do not permit probing if the load cell is not calibrated
        if !self.load_cell.is_calibrated() {
            return Err(KlippyError::Command("Load Cell not calibrated".to_string()));
        }
        // tare the sensor just before probing
        self.pause_and_tare(gcmd)?;
        // get params for the homing move
        let toolhead = lookup_toolhead(&self.printer)?;
        let mut pos = toolhead.get_position();
        if pos.len() < 3 {
            return Err(other("IndexError", "list assignment index out of range"));
        }
        pos[2] = self.z_min_position;
        let ProbeParams { probe_speed: speed, .. } = self.param_helper.get_probe_params(Some(gcmd))?;
        let phoming = lookup_interface::<dyn PrinterHomingApi>(&*self.printer, "homing", LookupDefault::Raise)?
            .expect("Raise never yields None");
        // start collector after tare samples are consumed
        let collector = self.start_collector()?;
        // do homing move
        let epos = phoming.probing_move(
            self.mcu_trigger_analog.clone(),
            pos.into_iter().map(Some).collect(),
            speed,
            true,
        )?;
        Ok((epos, collector))
    }

    /// Wait for the MCU to trigger with no movement (`probing_test`).
    pub fn probing_test(&self, gcmd: &GCmd, timeout: f64) -> KResult<f64> {
        self.pause_and_tare(gcmd)?;
        let toolhead = lookup_toolhead(&self.printer)?;
        let print_time = toolhead.get_last_move_time();
        self.mcu_trigger_analog.home_start(print_time)?;
        self.mcu_trigger_analog.home_wait(print_time + timeout)
    }

    /// `get_status(eventtime)`.
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let trig_time = self.mcu_trigger_analog.get_last_trigger_time();
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("tare_counts".to_string()),
            PyValue::Int(PyInt::from_i64(self.tare_counts)),
        );
        d.insert(PyValue::Str("last_trigger_time".to_string()), PyValue::Float(trig_time));
        PyValue::Dict(d)
    }
}

// ---------------------------------------------------------------------------
// Tapping move and session
// ---------------------------------------------------------------------------

/// Perform a single complete tap (`TappingMove`, `load_cell_probe.py:376`).
pub struct TappingMove {
    printer: Printer,
    load_cell_probing_move: Arc<LoadCellProbingMove>,
    clients: Arc<ApiClientHelper>,
    /// `_last_result`
    last_result: GCell<Option<f64>>,
    /// `_is_last_result_valid`
    is_last_result_valid: GCell<bool>,
}

impl TappingMove {
    /// `TappingMove(config, load_cell_probing_move, config_helper)`; the reference never uses
    /// `config_helper` after storing it.
    pub fn new(config: &Config, load_cell_probing_move: Arc<LoadCellProbingMove>) -> KResult<TappingMove> {
        let printer = config.get_printer();
        // webhooks support
        let clients = ApiClientHelper::new(config.get_printer());
        let name = config.get_name();
        let mut header = IndexMap::new();
        header.insert(
            PyValue::Str("header".to_string()),
            PyValue::List(vec![PyValue::Str("probe_tap_event".to_string())]),
        );
        clients.add_mux_endpoint(
            "load_cell_probe/dump_taps",
            "load_cell_probe",
            &name,
            PyValue::Dict(header),
        )?;
        Ok(TappingMove {
            printer,
            load_cell_probing_move,
            clients,
            // track results of the last tap
            last_result: GCell::new(None),
            is_last_result_valid: GCell::new(false),
        })
    }

    /// Perform a probing move and a pullback move; returns the end position and whether the
    /// result is valid (`run_tap`).
    pub fn run_tap(&self, gcmd: &GCmd) -> KResult<(Vec<f64>, bool)> {
        // do the descending move
        let (epos, collector) = self.load_cell_probing_move.probing_move(gcmd)?;
        // collect samples from the tap
        let toolhead = lookup_toolhead(&self.printer)?;
        toolhead.flush_step_generation()?;
        let move_end = toolhead.get_last_move_time();
        let results = collector.collect_until(move_end)?;
        let samples = check_sensor_errors(&results)?;
        // Analyze the tap data
        let ppa = TapAnalysis::new(&samples)?;
        // broadcast tap event data:
        let mut msg = IndexMap::new();
        msg.insert(PyValue::Str("tap".to_string()), ppa.to_dict());
        self.clients.send(&PyValue::Dict(msg))?;
        self.is_last_result_valid.set(true);
        let z = *epos.get(2).ok_or_else(|| other("IndexError", "list index out of range"))?;
        self.last_result.set(Some(z));
        Ok((epos, self.is_last_result_valid.get()))
    }

    /// `get_status(eventtime)`.
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("last_z_result".to_string()),
            self.last_result.get().map_or(PyValue::None, PyValue::Float),
        );
        d.insert(
            PyValue::Str("is_last_tap_valid".to_string()),
            PyValue::Bool(self.is_last_result_valid.get()),
        );
        PyValue::Dict(d)
    }
}

/// `tapping_move.run_tap(gcmd)`: perform a probing move and a pullback move; returns the end
/// position and whether the result is valid.
pub type RunTapCb = Arc<dyn Fn(&GCmd) -> KResult<(Vec<f64>, bool)> + Send + Sync>;

/// ProbeSession that implements Tap logic (`TapSession`, `load_cell_probe.py:417`).
pub struct TapSession {
    run_tap: RunTapCb,
    probe_offsets: Arc<dyn ProbeOffsetsHelperApi>,
    results: GCell<Vec<ProbeResult>>,
}

impl TapSession {
    /// `TapSession(config, tapping_move, probe_offsets, probe_params_helper)`; the reference
    /// stores the config printer and parameter helper without using them.
    pub fn new(run_tap: RunTapCb, probe_offsets: Arc<dyn ProbeOffsetsHelperApi>) -> TapSession {
        TapSession { run_tap, probe_offsets, results: GCell::new(Vec::new()) }
    }

    /// `start_probe_session(gcmd)`: the session is the object itself.
    pub fn start_probe_session(self: &Arc<Self>, _gcmd: &GCmd) -> Arc<dyn ProbeSessionApi> {
        self.clone()
    }
}

impl ProbeSessionApi for TapSession {
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        let (epos, _is_good) = (self.run_tap)(gcmd)?;
        let offsets = self.probe_offsets.get_offsets(None);
        let res = create_probe_result(&epos, [offsets.0, offsets.1, offsets.2]);
        self.results.borrow_mut().push(res);
        Ok(())
    }

    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        Ok(std::mem::take(&mut *self.results.borrow_mut()))
    }

    fn end_probe_session(&self) -> KResult<()> {
        *self.results.borrow_mut() = Vec::new();
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// G-code commands
// ---------------------------------------------------------------------------

/// `load_cell_probing_move.probing_test(gcmd, timeout)`: wait for the MCU to trigger with no
/// movement; the result is the trigger time (`0.` on timeout).
pub type ProbingTestCb = Arc<dyn Fn(&GCmd, f64) -> KResult<f64> + Send + Sync>;

/// `LoadCellProbeCommands` (`load_cell_probe.py:446`).
pub struct LoadCellProbeCommands {
    printer: Printer,
    probing_test: ProbingTestCb,
}

impl LoadCellProbeCommands {
    /// `cmd_LOAD_CELL_TEST_TAP_help`.
    pub const CMD_LOAD_CELL_TEST_TAP_HELP: &'static str = "Tap the load cell probe to verify operation";

    /// `LoadCellProbeCommands(config, load_cell_probing_move)`: registers the commands.
    pub fn new(config: &Config, probing_test: ProbingTestCb) -> KResult<Arc<LoadCellProbeCommands>> {
        let this = Arc::new(LoadCellProbeCommands { printer: config.get_printer(), probing_test });
        this.register_commands()?;
        Ok(this)
    }

    fn register_commands(self: &Arc<Self>) -> KResult<()> {
        // Register commands
        let gcode = lookup_interface::<dyn GCodeApi>(&*self.printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let this = self.clone();
        gcode.register_command(
            "LOAD_CELL_TEST_TAP",
            Some(Box::new(move |gcmd| this.cmd_load_cell_test_tap(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(Self::CMD_LOAD_CELL_TEST_TAP_HELP.to_string()),
            },
        )?;
        Ok(())
    }

    /// `cmd_LOAD_CELL_TEST_TAP(gcmd)`.
    pub fn cmd_load_cell_test_tap(&self, gcmd: &GCmd) -> KResult<()> {
        let taps = gcmd
            .get_int("TAPS", GcmdIntOpts { default: Some(Some(3)), minval: Some(1), maxval: Some(10) })?
            .unwrap_or(3);
        let timeout = gcmd
            .get_float(
                "TIMEOUT",
                GcmdFloatOpts {
                    default: Some(Some(30.)),
                    minval: Some(1.),
                    maxval: Some(120.),
                    ..GcmdFloatOpts::default()
                },
            )?
            .unwrap_or(30.);
        gcmd.respond_info(&format!("Tap the load cell {taps} times:"), true);
        let reactor = self.printer.get_reactor();
        for _ in 0..taps {
            let result = (self.probing_test)(gcmd, timeout)?;
            if result == 0. {
                // notify of error, likely due to timeout
                return Err(KlippyError::Command("Test timeout out".to_string()));
            }
            gcmd.respond_info("Tap Detected!", true);
            // give the user some time for their finger to move away
            reactor.pause(reactor.monotonic() + 0.2);
        }
        gcmd.respond_info(&format!("Test complete, {taps} taps detected"), true);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// The probe object
// ---------------------------------------------------------------------------

fn no_attribute(attr: &str) -> KlippyError {
    other(
        "AttributeError",
        format!("'LoadCellPrinterProbe' object has no attribute '{attr}'"),
    )
}

/// A sensor class of the `*_SENSOR_TYPES` tables: builds the sensor from its config section.
/// The sensor is handed out through the two duck types the reference uses it as
/// (`LoadCell` and `MCU_trigger_analog`).
type SensorPair = (Arc<dyn BulkSensorAdc>, Arc<dyn TriggerAnalogSensor>);
type SensorCtor = Box<dyn Fn(&Config) -> KResult<SensorPair> + Send + Sync>;

fn sensor_pair<T: BulkSensorAdc + TriggerAnalogSensor>(sensor: Arc<T>) -> SensorPair {
    (sensor.clone() as Arc<dyn BulkSensorAdc>, sensor as Arc<dyn TriggerAnalogSensor>)
}

/// `sensors` (`load_cell_probe.py:488`): the hx71x, ads1220 and ads131m0x sensor type dicts
/// merged in Python's insertion order.
fn sensor_ctors() -> IndexMap<&'static str, SensorCtor> {
    let mut sensors: IndexMap<&'static str, SensorCtor> = IndexMap::new();
    for (name, ctor) in HX71X_SENSOR_TYPES {
        sensors.insert(name, Box::new(move |c| Ok(sensor_pair(ctor(c)?))));
    }
    sensors.insert(
        crate::ads1220::SENSOR_TYPE_NAME,
        Box::new(|c| Ok(sensor_pair(Ads1220::new(c)?))),
    );
    for (name, ctor) in ADS131M0X_SENSOR_TYPES {
        sensors.insert(name, Box::new(move |c| Ok(sensor_pair(ctor(c)?))));
    }
    sensors
}

/// The accepted `sensor_type` values, in the order of the reference's `sensors` dict.
pub fn sensor_type_names() -> Vec<&'static str> {
    sensor_ctors().keys().copied().collect()
}

/// `LoadCellPrinterProbe` (`load_cell_probe.py:476`): registered as the printer object
/// `probe`.
pub struct LoadCellPrinterProbe {
    load_cell: Arc<LoadCell>,
    param_helper: Arc<ProbeParameterHelper>,
    // The reference assigns these after `ProbeCommandHelper(config, self)`, which needs the
    // object itself: they are set once the `Arc` exists.
    cmd_helper: GCell<Option<Arc<ProbeCommandHelper>>>,
    probe_offsets: GCell<Option<Arc<ProbeOffsetsHelper>>>,
    tapping_move: GCell<Option<Arc<TappingMove>>>,
    probe_session: GCell<Option<Arc<SampleAveragingHelper>>>,
}

impl LoadCellPrinterProbe {
    /// `LoadCellPrinterProbe(config)`.
    pub fn new(config: &Config) -> KResult<Arc<LoadCellPrinterProbe>> {
        let printer = config.get_printer();
        // Sensor types supported by load_cell_probe
        let sensors = sensor_ctors();
        let choices: IndexMap<PyValue, PyValue> = sensors
            .keys()
            .map(|k| (PyValue::Str((*k).to_string()), PyValue::Str((*k).to_string())))
            .collect();
        let choice = config.getchoice("sensor_type", &PyValue::Dict(choices), GetChoiceOpts::default())?;
        let sensor_type = match &choice {
            PyValue::Str(name) => name.clone(),
            other_value => other_value.str(),
        };
        // `getchoice` only returns a value of the dict it was given.
        let sensor_class =
            sensors.get(sensor_type.as_str()).ok_or_else(|| other("KeyError", choice.repr()))?;
        let (sensor, trigger_sensor) = sensor_class(config)?;
        let load_cell = LoadCell::new(config, sensor.clone())?;
        // Read all user configuration and build modules
        let config_helper = LoadCellProbeConfigHelper::new(config, &load_cell)?;
        let mcu = load_cell.get_sensor().get_mcu();
        let mcu_trigger_analog = MCU_trigger_analog::new(trigger_sensor)?;
        let cmd_queue = mcu_trigger_analog.get_dispatch().get_command_queue();
        let sos_filter = MCU_SosFilter::new(mcu, cmd_queue, 4)?;
        mcu_trigger_analog.setup_sos_filter(sos_filter.clone());
        let continuous_tare_filter_helper =
            ContinuousTareFilterHelper::new(config, &sensor, sos_filter)?;
        // Probe Interface
        let param_helper = ProbeParameterHelper::new(config)?;
        let this = Arc::new(LoadCellPrinterProbe {
            load_cell: load_cell.clone(),
            param_helper: param_helper.clone(),
            cmd_helper: GCell::new(None),
            probe_offsets: GCell::new(None),
            tapping_move: GCell::new(None),
            probe_session: GCell::new(None),
        });
        let weak_self: Weak<LoadCellPrinterProbe> = Arc::downgrade(&this);
        let weak_probe: Weak<dyn PrinterProbeApi> = weak_self;
        let cmd_helper = ProbeCommandHelper::new(config, weak_probe, None, true)?;
        this.cmd_helper.replace(Some(cmd_helper));
        let probe_offsets = ProbeOffsetsHelper::new(config)?;
        this.probe_offsets.replace(Some(probe_offsets.clone()));
        let load_cell_probing_move = LoadCellProbingMove::new(
            config,
            load_cell,
            mcu_trigger_analog,
            param_helper.clone(),
            continuous_tare_filter_helper,
            config_helper,
        )?;
        let tapping_move = Arc::new(TappingMove::new(config, load_cell_probing_move.clone())?);
        this.tapping_move.replace(Some(tapping_move.clone()));
        let run_tap_move = tapping_move.clone();
        let tap_session = Arc::new(TapSession::new(
            Arc::new(move |gcmd| run_tap_move.run_tap(gcmd)),
            probe_offsets.clone(),
        ));
        let probe_session = SampleAveragingHelper::new(
            config,
            param_helper as Arc<dyn ProbeParameterHelperApi>,
            Box::new(move |gcmd| Ok(tap_session.start_probe_session(gcmd))),
        );
        this.probe_session.replace(Some(probe_session));
        // printer integration
        let probing_move = load_cell_probing_move.clone();
        LoadCellProbeCommands::new(
            config,
            Arc::new(move |gcmd, timeout| probing_move.probing_test(gcmd, timeout)),
        )?;
        HomingViaProbeHelper::new(config, this.get_offsets(None)?.2, None)?;
        printer.add_object("probe", this.clone() as ObjectRef)?;
        Ok(this)
    }

    /// `get_probe_params(gcmd=None)`.
    pub fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        self.param_helper.get_probe_params(gcmd)
    }

    fn probe_offsets(&self) -> Option<Arc<ProbeOffsetsHelper>> {
        self.probe_offsets.cloned()
    }

    /// `get_offsets(gcmd=None)`.
    pub fn get_offsets(&self, gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        let offsets = self.probe_offsets().ok_or_else(|| no_attribute("_probe_offsets"))?;
        Ok(offsets.get_offsets(gcmd))
    }

    /// `start_probe_session(gcmd)`.
    pub fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        let session = self.probe_session.cloned().ok_or_else(|| no_attribute("_probe_session"))?;
        Ok(session.start_probe_session(gcmd)? as Arc<dyn ProbeSessionApi>)
    }

    fn status(&self, eventtime: f64) -> Option<PyValue> {
        let cmd_helper = self.cmd_helper.cloned()?;
        let tapping_move = self.tapping_move.cloned()?;
        let mut status = match cmd_helper.get_status(eventtime) {
            PyValue::Dict(d) => d,
            _ => return None,
        };
        for part in [self.load_cell.get_status_value(eventtime), tapping_move.get_status(eventtime)] {
            if let PyValue::Dict(d) = part {
                for (k, v) in d {
                    status.insert(k, v);
                }
            }
        }
        Some(PyValue::Dict(status))
    }
}

impl PrinterProbeApi for LoadCellPrinterProbe {
    fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        LoadCellPrinterProbe::get_probe_params(self, gcmd)
    }

    fn get_offsets(&self, gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        LoadCellPrinterProbe::get_offsets(self, gcmd)
    }

    fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        LoadCellPrinterProbe::start_probe_session(self, gcmd)
    }
}

impl PrinterObject for LoadCellPrinterProbe {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterProbeApi);

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        self.status(eventtime)
    }
}

/// `load_config(config)` (`load_cell_probe.py:536`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(LoadCellPrinterProbe::new(config)? as ObjectRef))
}

/// Module entry point for the `load_cell_probe` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
