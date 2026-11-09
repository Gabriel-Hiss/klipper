//! Oracle sweep of template load errors.
//!
//! Every case is a template the reference Jinja2 2.11.3 rejects when
//! Klipper loads it: `klippy/extras/gcode_macro.py` builds its environment
//! as `jinja2.Environment('{%', '%}', '{', '}')` and `TemplateWrapper`
//! turns the exception into the user-visible `Error loading template`
//! text. The live oracle (`pyoracle`) runs both on each case; the Rust
//! `Template::parse` must raise the same exception class, message and
//! line, and `TemplateWrapper::load` must produce the same text.
//!
//! The cases cover every error path of the Rust lexer and parser (lexer
//! failures, `stream.expect`, `Parser.fail`, `fail_eof`,
//! `fail_unknown_tag`, the compiler's `TemplateAssertionError` checks and
//! the Python compile of the generated module) plus the common mistakes:
//! unclosed tags and comments, missing or stray end tags, bad filter and
//! test names, unbalanced brackets, trailing operators and multi-line
//! templates where the line number matters.
//!
//! Integration owner runs (in the oracle image):
//!
//! ```sh
//! cargo test -p extras_macros_templates --test template_errors
//! ```

use extras_macros_templates::gcode_macro::{Template, TemplateWrapper};
use recording::Value;
use std::path::PathBuf;

/// Templates Jinja rejects at load time.
const CASES: &[&str] = &[
    // print statements and expressions
    "{ }",
    "{",
    "abc {",
    "{ x",
    "{ x + }",
    "{ x y }",
    "{ (x }",
    "{ (x, y }",
    "{ [1, 2 }",
    "{ [1 2] }",
    "{ {'a': 1 }",
    "{ {'a' 1} }",
    "{ {'a': 1 'b': 2} }",
    "{ {**a} }",
    "{ x) }",
    "{ x] }",
    "{ (x] }",
    "{ [x) }",
    "{ f(1 2) }",
    "{ f(1, }",
    "{ f(a=1, 2) }",
    "{ f(**a, b) }",
    "{ f(*a, *b) }",
    "{ f(**a, **b) }",
    "{ f(**a, c=1) }",
    "{ f(*a) + }",
    "{ x | }",
    "{ x | 1 }",
    "{ x|nosuch }",
    "{ x|nosuch(1) }",
    "{ x|a.b }",
    "{ x|a. }",
    "{ x is }",
    "{ x is 1 }",
    "{ x is nosuch }",
    "{ x is defined is defined }",
    "{ x is not }",
    "{ x is a.b }",
    "{ x is in [1] and }",
    "{ x. }",
    "{ x.'a' }",
    "{ x[ }",
    "{ x[1 }",
    "{ x[1 2] }",
    "{ x[1:2:3:4] }",
    "{ x[:2 }",
    "{ x[1:2] + }",
    "{ 1 + }",
    "{ * 2 }",
    "{ not }",
    "{ x if }",
    "{ x if y else }",
    "{ x if y else z else w }",
    "{ 'abc }",
    "{ \"abc }",
    "{ 'abc",
    "{ x }{ 'abc",
    "{ 'a\\x4' }",
    "{ 'a\\xzz' }",
    "{ '\\u12' }",
    "{ '\\U1234' }",
    "{ '\\N' }",
    "{ '\\U00110000' }",
    "{ x ? y }",
    "{ x ! y }",
    "{ x @ y }",
    "{ x $ }",
    "{ x ; y }",
    "{ x ` y }",
    "{ x \\ y }",
    "{ 0x10 }",
    "{ 0b101 }",
    "{ 0o17 }",
    "{ 1_ }",
    "{ 1__0 }",
    "{ .5 }",
    "{ 1e }",
    "{ x = 1 }",
    "{ x == }",
    "{ x not y }",
    "{ x not in }",
    "{ x in }",
    "{ x and }",
    "{ x or }",
    "{ x ~ }",
    "{ x ** }",
    "{ x // }",
    "{ - }",
    "{ x.y( }",
    "{ x(1)(2 }",
    "{ , }",
    "{ x,, }",
    "{ (,) }",
    "{ [,] }",
    "{ x|default( }",
    "{ x|default(1 }",
    "{ x|default(1, }",
    "{ x +}",
    "{{ x }}",
    "{{ x }",
    "{ x }}{% if %}",
    // statements
    "{% %}",
    "{% 1 %}",
    "{% 'if' %}",
    "{%",
    "{% if",
    "{% if x",
    "{% if %}",
    "{% if x %}",
    "{% if x %}a{% else %}b",
    "{% if x %}a{% elif %}b{% endif %}",
    "{% if x %}a{% elif y %}b",
    "{% if x %}a{% else %}b{% elif y %}c{% endif %}",
    "{% if x %}a{% else %}b{% else %}c{% endif %}",
    "{% endif %}",
    "{% else %}",
    "{% elif x %}",
    "{% endfor %}",
    "{% endset %}",
    "{% endmacro %}",
    "{% endraw %}",
    "{% if x %}{% endfor %}",
    "{% for x in y %}{% endif %}",
    "{% if x %}{% for y in z %}{% endif %}",
    "{% for x in y %}{% if z %}{% endfor %}",
    "{% for x in y %}{% else %}{% endif %}",
    "{% if x %}{% endif %}{% endif %}",
    "{% if x y %}{% endif %}",
    "{% if x %}{% endif y %}",
    "{% if x %}{% else y %}{% endif %}",
    "{% if x if y %}{% endif %}",
    "{% if x == 1 %}a{% elif x == %}b{% endif %}",
    "{% if %}{ 'abc }",
    "{% for %}{% endfor %}",
    "{% for x %}{% endfor %}",
    "{% for x in %}{% endfor %}",
    "{% for in y %}{% endfor %}",
    "{% for 1 in y %}{% endfor %}",
    "{% for x.y in z %}{% endfor %}",
    "{% for [a] in z %}{% endfor %}",
    "{% for x x in y %}{% endfor %}",
    "{% for x in y %}",
    "{% for x in y %}a{% else %}b",
    "{% for loop in y %}{% endfor %}",
    "{% for x in y %}{% set loop = 1 %}{% endfor %}",
    "{% for x in y z %}{% endfor %}",
    "{% for x in y if %}{% endfor %}",
    "{% for x, in y %}",
    "{% for x in y recursive %}",
    "{% for x in y %}{% endfor x %}",
    "{% for a, b in x %}{% endfor %}{% endif %}",
    "{% set %}",
    "{% set x %}",
    "{% set x %}abc",
    "{% set = 1 %}",
    "{% set 1 = 2 %}",
    "{% set x = %}",
    "{% set x = 1 2 %}",
    "{% set true = 1 %}",
    "{% set x == 1 %}",
    "{% set x.y %}",
    "{% set x, = 1 %}{% endif %}",
    "{% set x.y = 1 %}{% endif %}",
    "{% set a, b = 1, 2 %}{% endif %}",
    "{% set x %}{% endset %}{% endif %}",
    "{% nosuch %}",
    "{% do x %}",
    "{% if x %}{% nosuch %}{% endif %}",
    "{% raw %}abc",
    "{% raw %}\n\nabc",
    "{% raw %}abc{% endraw %}{% endif %}",
    "{# abc",
    "abc {# x",
    "\n\n{# abc",
    "{% if x %}{# c",
    "{% if x %}{% endif %}{# c",
    "{% if x -%}",
    "{%- if %}",
    "{%+ if %}",
    "{% if x +%}{% endif %}",
    "{% if (x %}{% endif %}",
    "{% if x) %}{% endif %}",
    "{% if x } %}",
    "{% if [x %}",
    "{% if {x %}",
    // statement tags this port does not render
    "{% macro %}",
    "{% macro m( %}",
    "{% macro m(a=1, b) %}{% endmacro %}",
    "{% macro m() %}",
    "{% macro m() %}{% endmacro %}{% endif %}",
    "{% call %}",
    "{% call x %}{% endcall %}",
    "{% call m() %}",
    "{% filter %}",
    "{% filter upper %}",
    "{% filter nosuchf %}{% endfilter %}",
    "{% block %}",
    "{% block a-b %}",
    "{% block a %}",
    "{% block a %}{% endblock a b %}",
    "{% with %}",
    "{% with a %}",
    "{% with a = 1 b = 2 %}",
    "{% with a = 1 %}",
    "{% autoescape %}",
    "{% autoescape true %}",
    "{% include %}",
    "{% include 'a' ignore %}",
    "{% import %}",
    "{% import 'a' %}",
    "{% import 'a' as %}",
    "{% from %}",
    "{% from 'a' %}",
    "{% from 'a' import %}",
    "{% from 'a' import _b %}",
    "{% from 'a' import b c %}",
    "{% extends %}",
    "{% print %}{% endif %}",
    "{% print x y %}",
    "{% print x, %}",
    // line numbers
    "line1\n{% if x %}\nfoo\n",
    "a\nb\n{ x +\n y + }\n",
    "{% if x %}\n{% else %}\n{% else %}\n{% endif %}",
    "{% set x = [1,\n2,\n3 %}",
    "{ 'abc\ndef' + }",
    "{# multi\nline #}\n{ x y }",
    "{# multi\nline",
    "a\n\n{ ?? }",
    "a\n\n{ x\n\n ?? }",
    "{%- if x -%}\n\n  { y + }\n{%- endif %}",
    "first\n{ x\n+\n}",
    "{% for x in y %}\nA\n{% endfor %}\n{% endfor %}",
    "a\n{ x\nb",
    "{% if x %}\n{ y\n{% endif %}",
    "{ x }\n{% if %}",
    "{% if %}\nTURN_OFF_HEATERS\n{% endif %}\n",
    "{% if x %}\r\n{ y + }\r\n{% endif %}",
    "{% if x %}\n\n{% endif %}\n{% endif %}\n",
    "  {%- if x %}\n{% endif x %}",
    "{ x -}\n\n\n{ y + }",
    "{%if x-%}\n\n{%endif-%}\n\n{ ? }",
    "{ 'a'\n'b' + }",
    "{ x[\n1\n2] }",
    "{% for\nloop\nin y %}{% endfor %}",
    "{% set\n1 = 2 %}",
    // compile-time checks: unknown filters and tests, in visiting order
    "{ x|nosuch }\n{ y + }",
    "{ x|default(1) }\n{ y|nosuch }",
    "{ x|nosuch1|nosuch2 }",
    "{% if false %}{ x|nosuch }{% endif %}",
    "{ x is nosuch(1) }",
    "{ f(x|nosuch) }",
    "{ (x|nosuch) is nosuch2 }",
    "{ 1 if x|bad1 else y|bad2 }",
    "{ a|bad1 if x|bad2 else 2 }",
    "{ x|bad1 ~ y|bad2 }",
    "{ [x|bad1, y|bad2] }",
    "{ {x|bad1: y|bad2} }",
    "{ x.y(1|bad1) }",
    "{ x[1|bad1] }",
    "{ f(a=1|bad1) }",
    "{ not x|bad1 }",
    "{ -x|bad1 }",
    "{ x and y|bad1 }",
    "{ x < y|bad1 }",
    "{ x is defined(1|bad1) }",
    "{ x|default(1|bad1)|bad2 }",
    "{% set x = y|bad %}",
    "{% for x in y|bad %}{% endfor %}",
    "{% for x in y %}{ x|bad }{% endfor %}",
    "{% for loop in y|bad %}{% endfor %}",
    "{% if x|bad1 %}{ y|bad2 }{% endif %}",
    "{% if x %}{% elif y|bad1 %}{% else %}{ z|bad2 }{% endif %}",
    "{ x\n|\nnosuch }",
    "{ x\nis\nnosuch }",
    "{ 1 if false else x|bad }",
    "{ true and x|bad }",
    "{ x or y|bad }",
    "{ [1 < 2 < x|bad] }",
    // the generated Python fails to compile
    "{ f(a=1, a=2) }",
    "{ x|default(a=1, a=2) }",
];

/// Templates Jinja loads that run through the same lexer, parser and
/// compile-check paths: odd but valid syntax, and unknown filters or
/// repeated keywords the optimizer folds away before they are checked.
const ACCEPTED: &[&str] = &[
    "{ {'a': 1, } }",
    "{ x is in }",
    "{ x.1.5 }",
    "{ 007 }",
    "{ 1.e5 }",
    "{ f(if=1) }",
    "{ f(if=1, if=2) }",
    "{ x.if }",
    "{ \u{e9} }",
    "{ x, }",
    "{ () }",
    "{ x|default(1,) }",
    "{% if x: %}a{% endif %}",
    "{% if x %}a{% else: %}b{% endif %}",
    "{% if x, %}{% endif %}",
    "{% for (a) in z %}{ a }{% endfor %}",
    "{% raw %}{ x {% if %}{% endraw %}",
    "{%- raw -%} a {%- endraw -%}",
    "{ 1 if true else x|bad }",
    "{ true or x|bad }",
    "{ false and f(a=1, a=2) }",
    "{ [1 > 2 > x|bad, 2] }",
    "{% set v = 2 if true else x|bad %}",
];

const PY_SETUP: &str = r#"
import sys, logging, traceback
sys.path.insert(0, '__KLIPPY_EXTRAS__')
import jinja2
import gcode_macro
logging.disable(logging.CRITICAL)

class _GCode:
    class error(Exception):
        pass

class _GCodeMacro:
    def create_template_context(self, eventtime=None):
        return {}

class _Printer:
    class config_error(Exception):
        pass
    def lookup_object(self, name):
        return _GCode() if name == 'gcode' else _GCodeMacro()

_env = jinja2.Environment('{%', '%}', '{', '}')

def load_error(src):
    try:
        _env.from_string(src)
    except jinja2.exceptions.TemplateSyntaxError as e:
        exc = [type(e).__name__, e.message, e.lineno]
    except Exception as e:
        exc = [type(e).__name__, traceback.format_exception_only(type(e), e)[-1], None]
    else:
        return None
    try:
        gcode_macro.TemplateWrapper(_Printer(), _env, 'macro:gcode', src)
    except Exception as e:
        return exc + [str(e)]
    raise AssertionError('TemplateWrapper accepted %r' % (src,))
"#;

fn klippy_extras_path() -> String {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../klippy/extras")
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

fn as_str(v: &Value) -> &str {
    match v {
        Value::Str(s) => s,
        other => panic!("expected a string from the oracle, got {other:?}"),
    }
}

fn start_oracle() -> Option<pyoracle::Oracle> {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return None;
    };
    if let Err(e) = py.exec(&PY_SETUP.replace("__KLIPPY_EXTRAS__", &klippy_extras_path())) {
        panic!("oracle setup: {}: {}", e.class_name, e.message);
    }
    Some(py)
}

#[test]
fn templates_jinja_accepts_parse() {
    let Some(mut py) = start_oracle() else {
        return;
    };
    for src in ACCEPTED {
        match py.eval(&format!("load_error({})", py_literal(src))) {
            Ok(Value::Null) => {}
            other => panic!("oracle at {src:?}: expected acceptance, got {other:?}"),
        }
        if let Err(e) = Template::parse(src) {
            panic!("{src:?}: Jinja accepts it, rust raised {}", e.display_string());
        }
    }
}

#[test]
fn template_load_errors_match_jinja() {
    let Some(mut py) = start_oracle() else {
        return;
    };
    let mut failures = Vec::new();
    for src in CASES {
        let expected = match py.eval(&format!("load_error({})", py_literal(src))) {
            Ok(Value::List(items)) | Ok(Value::Tuple(items)) => items,
            Ok(other) => panic!("oracle accepted {src:?} ({other:?}); sweep cases must fail"),
            Err(e) => panic!("oracle at {src:?}: {}: {}", e.class_name, e.message),
        };
        let [class, message, lineno, wrapper] = expected.as_slice() else {
            panic!("oracle at {src:?}: unexpected result {expected:?}");
        };
        let (class, message, wrapper) = (as_str(class), as_str(message), as_str(wrapper));
        let lineno = match lineno {
            Value::Int(n) => Some(usize::try_from(*n).expect("line numbers are positive")),
            Value::Null => None,
            other => panic!("oracle at {src:?}: line number {other:?}"),
        };
        let err = match Template::parse(src) {
            Ok(_) => {
                failures.push(format!("{src:?}: accepted, Jinja raised {class}: {message}"));
                continue;
            }
            Err(e) => e,
        };
        let got = if lineno.is_some() {
            (err.kind().to_string(), err.message().to_string(), err.lineno())
        } else {
            // Not a TemplateSyntaxError: compare the exception-only line.
            (err.kind().to_string(), format!("{}\n", err.display_string()), None)
        };
        let want = (class.to_string(), message.to_string(), lineno);
        if got != want {
            failures.push(format!("{src:?}:\n  jinja {want:?}\n  rust  {got:?}"));
            continue;
        }
        let loaded = match TemplateWrapper::load("macro:gcode", src) {
            Ok(_) => "accepted".to_string(),
            // The non-syntax form leaves the traceback line's newline to
            // the printer-bound wrapper (`extras::gcode_macro`).
            Err(msg) if lineno.is_none() => format!("{msg}\n"),
            Err(msg) => msg,
        };
        if loaded != wrapper {
            failures.push(format!("{src:?}: TemplateWrapper\n  jinja {wrapper:?}\n  rust  {loaded:?}"));
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
