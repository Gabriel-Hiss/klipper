//! Oracle sweep of render-time `Undefined` behavior.
//!
//! Each case renders a template against the same context in the reference
//! Jinja2 2.11.3 (Klipper's `jinja2.Environment('{%', '%}', '{', '}')`)
//! and in the port, and compares the output or the raised exception
//! (class and `str()`). The cases cover what Jinja's `getattr`/`getitem`
//! return for every kind of value (dict, list, tuple, str, int, float,
//! bool, `None`, the `printer` status wrapper), nested lookups, `params`
//! as a `gcode_macro` sees it, the operations that print or raise an
//! `Undefined` (`|default`, `is defined`, filters, `in`, arithmetic,
//! comparisons), and the missing-`else` inline if.
//!
//! Integration owner runs (in the oracle image):
//!
//! ```sh
//! cargo test -p extras_macros_templates --test template_render_errors
//! ```

use extras_macros_templates::gcode_macro::{StatusSource, Template, Value};
use indexmap::IndexMap;
use pycompat::int::PyInt;
use pycompat::PyValue;
use recording::Value as RValue;
use std::path::PathBuf;

const CASES: &[&str] = &[
    // dict
    "{ d.missing }",
    "{ d.missing + 1 }",
    "{ d['missing'] + 1 }",
    "{ d[5] + 1 }",
    "{ d[[1]] + 1 }",
    "{ d['items'] }",
    "{ d.nested.missing + 1 }",
    "{ d.nested.k }",
    "{ d[missing] + 1 }",
    // values inside a dict
    "{ d.a.b + 1 }",
    "{ d.a['b'] + 1 }",
    "{ d.a[0] + 1 }",
    // list and tuple
    "{ l[5] }",
    "{ l[5] + 1 }",
    "{ l[-5] + 1 }",
    "{ l[-1] }{ l[True] }",
    "{ l.x + 1 }",
    "{ l['x'] + 1 }",
    "{ l[1.5] + 1 }",
    "{ l[missing] + 1 }",
    "{ l['append'] is defined }",
    "{ t[5] + 1 }",
    "{ t.x + 1 }",
    "{ t[0] }",
    // str
    "{ s[10] + 1 }",
    "{ s.x + 1 }",
    "{ s[0] }{ s[-1] }",
    "{ s.upper() }{ s['upper']() }",
    // numbers, bool, None
    "{ n.x + 1 }",
    "{ n[0] + 1 }",
    "{ n.real }{ n.imag }{ f.real }{ b.real }",
    "{ f.x + 1 }",
    "{ f[0] + 1 }",
    "{ b.x + 1 }",
    "{ none.x + 1 }",
    "{ none[0] + 1 }",
    "{ none.x }",
    // missing variables
    "{ missing }",
    "{ missing + 1 }",
    "{ missing.x }",
    "{ missing[0] }",
    "{ missing() }",
    "{ -missing }",
    "{ missing < 1 }",
    "{ missing ~ 'x' }",
    "{ '%s' % missing }",
    "{ '{}'.format(missing) }",
    "{ missing == missing }{ missing != 1 }",
    // filters
    "{ missing|int }",
    "{ missing|float }",
    "{ missing|length }",
    "{ missing|lower }",
    "{ missing|abs }",
    "{ missing|default(7) }",
    "{ d.missing|default(7) }",
    "{ l[9]|default(7) }",
    "{ none.x|default(7) }",
    "{ missing.x|default(7) }",
    "{ d.missing|float }",
    "{ l[9]|int }",
    // tests and membership
    "{ missing is defined }{ d.missing is defined }{ d.a is defined }{ l[9] is undefined }",
    "{ 'a' in missing }",
    "{ missing in [1] }",
    "{ missing in d }",
    "{ missing in 'abc' }",
    "{ 1 in 'abc' }",
    "{ 'a' in n }",
    "{ [1] in d }",
    // inline if without else
    "{ 1 if false }",
    "{ (1 if false) + 1 }",
    "{ (1\n if false) + 1 }",
    "{ (1 if false else 2\n if false) + 1 }",
    "line one\n{ (2 if false) + 1 }",
    // gcode_macro params
    "{ params.X }",
    "{ params.Y }",
    "{ params.Y + 1 }",
    "{ params.Y|float }",
    "{ params.Y|default(3)|float }",
    "{ params['Y']|int + 1 }",
    // printer status wrapper
    "{ printer.toolhead.nosuch + 1 }",
    "{ printer.toolhead.position[5] + 1 }",
    "{ printer.toolhead.position.x + 1 }",
    "{ printer['gcode_macro M'].w + 1 }",
    "{ printer.nosuch.x }",
    "{ printer.nosuch + 1 }",
    "{ printer[5] + 1 }",
    // iteration
    "{% for i in missing %}x{% else %}e{% endfor %}",
    "{% for i in d.missing %}x{% else %}e{% endfor %}",
];

const PY_SETUP: &str = r#"
import sys, contextlib
sys.path.insert(0, '__KLIPPY__')
import jinja2
import extras.gcode_macro as gcode_macro

class _Reactor:
    def monotonic(self):
        return 1.0
    def assert_no_pause(self):
        return contextlib.nullcontext()

class _Obj:
    def __init__(self, status):
        self.status = status
    def get_status(self, eventtime):
        return dict(self.status)

class _Printer:
    def __init__(self):
        self.objects = {
            'toolhead': _Obj({'position': [1.0, 2.0], 'homed_axes': 'xyz'}),
            'gcode_macro M': _Obj({'v': 1}),
        }
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def lookup_objects(self):
        return list(self.objects.items())
    def get_reactor(self):
        return _Reactor()

_env = jinja2.Environment('{%', '%}', '{', '}')

def render_case(src):
    ctx = dict(d={'a': 1, 'items': 2, 'nested': {'k': 1}}, l=[1, 2], t=(1, 2),
               s='abc', n=5, f=1.5, none=None, b=True, params={'X': '1'},
               printer=gcode_macro.GetStatusWrapper(_Printer()))
    try:
        return _env.from_string(src).render(ctx)
    except Exception as e:
        return '%s: %s' % (type(e).__name__, e)
"#;

/// The `_Printer` objects of the oracle side.
struct Statuses;

impl StatusSource for Statuses {
    fn object_names(&self) -> Vec<String> {
        vec!["toolhead".to_string(), "gcode_macro M".to_string()]
    }
    fn get_status(&self, name: &str, eventtime: &mut Option<f64>) -> Result<Option<PyValue>, extras_macros_templates::gcode_macro::TemplateError> {
        let status = match name {
            "toolhead" => dict(vec![
                ("position", PyValue::List(vec![PyValue::Float(1.0), PyValue::Float(2.0)])),
                ("homed_axes", PyValue::Str("xyz".to_string())),
            ]),
            "gcode_macro M" => dict(vec![("v", int(1))]),
            _ => return Ok(None),
        };
        eventtime.get_or_insert(1.0);
        Ok(Some(status))
    }
}

fn int(n: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(n))
}

fn dict(pairs: Vec<(&str, PyValue)>) -> PyValue {
    PyValue::Dict(
        pairs
            .into_iter()
            .map(|(k, v)| (PyValue::Str(k.to_string()), v))
            .collect(),
    )
}

/// The oracle's `render_case` context.
fn context() -> IndexMap<String, Value> {
    let mut ctx = IndexMap::new();
    let mut put = |k: &str, v: PyValue| {
        ctx.insert(k.to_string(), Value::py(v));
    };
    put(
        "d",
        dict(vec![("a", int(1)), ("items", int(2)), ("nested", dict(vec![("k", int(1))]))]),
    );
    put("l", PyValue::List(vec![int(1), int(2)]));
    put("t", PyValue::Tuple(vec![int(1), int(2)]));
    put("s", PyValue::Str("abc".to_string()));
    put("n", int(5));
    put("f", PyValue::Float(1.5));
    put("none", PyValue::None);
    put("b", PyValue::Bool(true));
    put("params", dict(vec![("X", PyValue::Str("1".to_string()))]));
    ctx.insert("printer".to_string(), Value::printer_status(Box::new(Statuses), None));
    ctx
}

fn klippy_path() -> String {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../klippy")
        .to_string_lossy()
        .replace('\\', "/")
}

/// A Python string literal for `s`.
fn py_literal(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            ' '..='~' => out.push(c),
            _ => out.push_str(&format!("\\U{:08x}", c as u32)),
        }
    }
    out.push('\'');
    out
}

#[test]
fn render_undefined_matches_jinja() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    if let Err(e) = py.exec(&PY_SETUP.replace("__KLIPPY__", &klippy_path())) {
        panic!("oracle setup: {}: {}", e.class_name, e.message);
    }
    let mut failures = Vec::new();
    for src in CASES {
        let expected = match py.eval(&format!("render_case({})", py_literal(src))) {
            Ok(RValue::Str(s)) => s,
            other => panic!("oracle at {src:?}: {other:?}"),
        };
        let template = Template::parse(src).unwrap_or_else(|e| panic!("{src:?}: {e}"));
        let got = match template.render(context()) {
            Ok(out) => out,
            Err(e) => format!("{}: {}", e.kind(), e.message()),
        };
        if got != expected {
            failures.push(format!("{src:?}:\n  jinja {expected:?}\n  rust  {got:?}"));
        }
    }
    assert!(
        failures.is_empty(),
        "{} of {} cases differ:\n{}",
        failures.len(),
        CASES.len(),
        failures.join("\n")
    );
}
