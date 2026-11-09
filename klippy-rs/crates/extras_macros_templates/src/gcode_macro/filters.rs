//! Jinja2 2.11.3 builtin filters (`jinja2/filters.py` `FILTERS`) and tests
//! (`jinja2/tests.py` `TESTS`), as Klipper's
//! `jinja2.Environment('{%', '%}', '{', '}')` runs them: no autoescape, the
//! default policies (`urlize.rel` "noopener", `truncate.leeway` 5,
//! `json.dumps_kwargs` `{"sort_keys": True}`) and markupsafe 1.1.1 with its C
//! speedups.
//!
//! Arguments bind like the Python functions, with CPython 3.8's `TypeError`
//! texts (the implicit `environment`/`eval_ctx`/`context` parameter counts
//! in "takes N positional arguments"). Results keep their Python types:
//! `Markup` ([`Value::Markup`]), generators and builtin iterators
//! ([`Value::Iter`], consumed lazily like Python's), `_GroupTuple`
//! ([`Value::Seq`]). Errors carry the class and `str()` Jinja raises
//! (`FilterArgumentError`, `TemplateRuntimeError`, builtin exceptions).
//!
//! Where a result depends on CPython object internals the port cannot see:
//! * `random` picks with its own generator; CPython's Mersenne Twister state
//!   is not reproduced, so which item comes out differs.
//! * `sameas` is identity: it holds for `None`, booleans, ints in
//!   -5..=256 (CPython caches those), equal one-character strings (cached
//!   too) and the same generator, `printer` or function object; any other
//!   two values count as distinct objects.
//! * The `repr()` of a generator or iterator prints `at 0x0` instead of its
//!   address.
//! * Regex classes follow Rust's Unicode tables (`\w` as alphanumeric or
//!   `_`), which differ from CPython 3.8's Unicode 12.1 only on rare
//!   characters (combining marks, non-ASCII digits).
//! * Jinja's optimizer folds filters on constants at compile time and
//!   writes a folded `nan`/`inf` float into the generated Python as a bare
//!   name, so e.g. `('nan'|float)|round(0, 'floor')` raises
//!   `NameError: name 'nan' is not defined` there; the port does not fold and
//!   evaluates the expression.
//! * A `Markup`, generator or `_GroupTuple` stored as a dict value (or
//!   passed to `list.append`) fails with an "unsupported" `TypeError`:
//!   the port's dicts and mutable lists hold plain Python values only.

use super::*;

type R<T> = Result<T, TemplateError>;

fn error(kind: &str, message: impl Into<String>, lineno: usize) -> TemplateError {
    TemplateError::new(kind, message.into(), Some(lineno))
}

fn type_error(message: impl Into<String>, lineno: usize) -> TemplateError {
    error("TypeError", message, lineno)
}

fn value_error(message: impl Into<String>, lineno: usize) -> TemplateError {
    error("ValueError", message, lineno)
}

fn attribute_error(obj: &Value, name: &str, lineno: usize) -> TemplateError {
    if obj.is_undefined() {
        return obj.fail_undefined(lineno);
    }
    error("AttributeError", format!("'{}' object has no attribute '{name}'", obj.type_name()), lineno)
}

fn filter_argument_error(message: impl Into<String>, lineno: usize) -> TemplateError {
    error("FilterArgumentError", message, lineno)
}

fn unsupported(what: String, lineno: usize) -> TemplateError {
    type_error(format!("unsupported: {what}"), lineno)
}

fn str_value(s: impl Into<String>) -> Value {
    Value::py(PyValue::Str(s.into()))
}

fn int_value(n: i64) -> Value {
    Value::py(PyValue::Int(PyInt::from_i64(n)))
}

fn bool_value(b: bool) -> Value {
    Value::py(PyValue::Bool(b))
}

fn markup_value(s: impl Into<String>) -> Value {
    Value::Markup(Rc::from(s.into()))
}

fn none_value() -> Value {
    Value::py(PyValue::None)
}

fn is_none(v: &Value) -> bool {
    matches!(v, Value::Py(PyValue::None))
}

/// `environment.undefined(hint)`.
fn undefined_hint(hint: &str) -> Value {
    Value::Undefined(Rc::from(hint))
}

fn pyint_i64(n: &PyInt) -> Option<i64> {
    i64::try_from(n.inner()).ok()
}

// ---------------------------------------------------------------------------
// argument binding
// ---------------------------------------------------------------------------

/// A Python function signature: positional-or-keyword `params` (the
/// implicit `environment`/`eval_ctx`/`context` ones first), the last
/// `defaults` of them optional, plus `*args`/`**kwargs`.
struct Sig {
    name: &'static str,
    params: &'static [&'static str],
    defaults: usize,
    implicit: usize,
    varargs: bool,
    varkw: bool,
}

const fn sig(name: &'static str, params: &'static [&'static str], defaults: usize, implicit: usize) -> Sig {
    Sig { name, params, defaults, implicit, varargs: false, varkw: false }
}

/// Bound arguments: one slot per parameter (`None` takes the default),
/// the extra positional and keyword arguments.
struct Bound {
    vals: Vec<Option<Value>>,
    rest: Vec<Value>,
    kw: Vec<(String, Value)>,
}

impl Bound {
    fn get(&self, i: usize) -> Option<&Value> {
        self.vals[i].as_ref()
    }
    fn take(&mut self, i: usize) -> Option<Value> {
        self.vals[i].take()
    }
}

fn plural(n: usize) -> &'static str {
    if n == 1 {
        ""
    } else {
        "s"
    }
}

/// `func(<implicit>, node, *args, **kwargs)` bound as CPython 3.8
/// `_PyEval_EvalCodeWithName` does, with its error order and texts.
fn bind(sig: &Sig, node: Value, args: Vec<Value>, kwargs: &[(String, Value)], lineno: usize) -> R<Bound> {
    let mut positional = Vec::with_capacity(sig.implicit + 1 + args.len());
    positional.resize(sig.implicit, none_value());
    positional.push(node);
    positional.extend(args);
    bind_positional(sig, positional, kwargs, lineno)
}

fn bind_positional(sig: &Sig, positional: Vec<Value>, kwargs: &[(String, Value)], lineno: usize) -> R<Bound> {
    let n = sig.params.len();
    let given = positional.len();
    let mut vals: Vec<Option<Value>> = vec![None; n];
    let mut rest = Vec::new();
    for (i, v) in positional.into_iter().enumerate() {
        if i < n {
            vals[i] = Some(v);
        } else {
            rest.push(v);
        }
    }
    let mut kw = Vec::new();
    for (key, value) in kwargs {
        match sig.params.iter().position(|p| p == key) {
            Some(i) if vals[i].is_some() => {
                return Err(type_error(format!("{}() got multiple values for argument '{key}'", sig.name), lineno))
            }
            Some(i) => vals[i] = Some(value.clone()),
            None if sig.varkw => kw.push((key.clone(), value.clone())),
            None => {
                return Err(type_error(
                    format!("{}() got an unexpected keyword argument '{key}'", sig.name),
                    lineno,
                ))
            }
        }
    }
    if given > n && !sig.varargs {
        let (takes, plural_s) = if sig.defaults > 0 {
            (format!("from {} to {n}", n - sig.defaults), "s")
        } else {
            (n.to_string(), plural(n))
        };
        return Err(type_error(
            format!(
                "{}() takes {takes} positional argument{plural_s} but {given} {} given",
                sig.name,
                if given == 1 { "was" } else { "were" }
            ),
            lineno,
        ));
    }
    let missing: Vec<String> =
        (0..n - sig.defaults).filter(|&i| vals[i].is_none()).map(|i| format!("'{}'", sig.params[i])).collect();
    if !missing.is_empty() {
        let names = match missing.len() {
            1 => missing[0].clone(),
            2 => format!("{} and {}", missing[0], missing[1]),
            m => format!("{}, and {}", missing[..m - 1].join(", "), missing[m - 1]),
        };
        return Err(type_error(
            format!(
                "{}() missing {} required positional argument{}: {names}",
                sig.name,
                missing.len(),
                plural(missing.len())
            ),
            lineno,
        ));
    }
    Ok(Bound { vals, rest, kw })
}

/// A C builtin taking exactly one positional argument (`METH_O`).
fn one_argument(name: &str, args: &[Value], kwargs: &[(String, Value)], lineno: usize) -> R<()> {
    if !kwargs.is_empty() {
        return Err(type_error(format!("{name}() takes no keyword arguments"), lineno));
    }
    if !args.is_empty() {
        return Err(type_error(
            format!("{name}() takes exactly one argument ({} given)", args.len() + 1),
            lineno,
        ));
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Python conversions
// ---------------------------------------------------------------------------

fn py_str(v: &Value, lineno: usize) -> R<String> {
    py_str_of(v, lineno)
}

/// `soft_unicode(v)`: a `str` or `Markup` unchanged, else `str(v)`.
fn soft_str(v: &Value, lineno: usize) -> R<Value> {
    match v {
        Value::Py(PyValue::Str(_)) | Value::Markup(_) => Ok(v.clone()),
        other => Ok(str_value(py_str(other, lineno)?)),
    }
}

/// A `str` method result: `Markup` when the receiver was one.
fn same_kind(receiver: &Value, s: String) -> Value {
    if matches!(receiver, Value::Markup(_)) {
        markup_value(s)
    } else {
        str_value(s)
    }
}

/// markupsafe's `escape_unicode`.
fn escape_text(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '>' => out.push_str("&gt;"),
            '<' => out.push_str("&lt;"),
            '\'' => out.push_str("&#39;"),
            '"' => out.push_str("&#34;"),
            c => out.push(c),
        }
    }
    out
}

/// `escape(v)`: a `Markup` (`__html__`) unchanged, else `str(v)` escaped.
fn escape(v: &Value, lineno: usize) -> R<String> {
    match v {
        Value::Markup(s) => Ok(s.to_string()),
        other => Ok(escape_text(&py_str(other, lineno)?)),
    }
}

/// `PyNumber_Index(v)` as a `Py_ssize_t`.
fn index_value(v: &Value, lineno: usize) -> R<i64> {
    match v {
        Value::Py(PyValue::Int(n)) => pyint_i64(n)
            .ok_or_else(|| error("OverflowError", "Python int too large to convert to C ssize_t", lineno)),
        Value::Py(PyValue::Bool(b)) => Ok(i64::from(*b)),
        other => Err(type_error(
            format!("'{}' object cannot be interpreted as an integer", other.type_name()),
            lineno,
        )),
    }
}

/// Argument Clinic's `Py_ssize_t` converter (CPython 3.8).
fn ssize_arg(v: &Value, lineno: usize) -> R<i64> {
    if let Value::Py(PyValue::Float(_)) = v {
        return Err(type_error("integer argument expected, got float", lineno));
    }
    index_value(v, lineno)
}

/// `_PyLong_AsInt(v)` (the `reverse` argument of `sorted`).
fn c_int_arg(v: &Value, lineno: usize) -> R<i64> {
    match v {
        Value::Py(PyValue::Float(_)) => Err(type_error("integer argument expected, got float", lineno)),
        Value::Py(PyValue::Int(n)) => pyint_i64(n)
            .filter(|i| i32::try_from(*i).is_ok())
            .ok_or_else(|| error("OverflowError", "Python int too large to convert to C int", lineno)),
        Value::Py(PyValue::Bool(b)) => Ok(i64::from(*b)),
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        other => Err(type_error(format!("an integer is required (got type {})", other.type_name()), lineno)),
    }
}

/// `float(v)` (CPython 3.8).
fn float_of(v: &Value, lineno: usize) -> R<f64> {
    match v {
        Value::Py(PyValue::Float(f)) => Ok(*f),
        Value::Py(PyValue::Bool(b)) => Ok(if *b { 1.0 } else { 0.0 }),
        Value::Py(PyValue::Int(n)) => n
            .to_f64_exact()
            .filter(|f| f.is_finite())
            .or_else(|| n.inner().to_string().parse::<f64>().ok().filter(|f| f.is_finite()))
            .ok_or_else(|| error("OverflowError", "int too large to convert to float", lineno)),
        Value::Py(PyValue::Str(_)) | Value::Markup(_) => {
            pytext::float_from_string(v.text().unwrap_or_default()).map_err(|e| from_pyerr(&e, lineno))
        }
        Value::Py(PyValue::Bytes(b)) => match std::str::from_utf8(b) {
            Ok(s) => pytext::float_from_string(s).map_err(|_| {
                value_error(format!("could not convert string to float: {}", pytext::bytes_repr(b)), lineno)
            }),
            Err(_) => Err(value_error(
                format!("could not convert string to float: {}", pytext::bytes_repr(b)),
                lineno,
            )),
        },
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        other => Err(type_error(
            format!("float() argument must be a string or a number, not '{}'", other.type_name()),
            lineno,
        )),
    }
}

/// `int(v)` (CPython 3.8).
fn int_of(v: &Value, lineno: usize) -> R<PyInt> {
    match v {
        Value::Py(PyValue::Int(n)) => Ok(n.clone()),
        Value::Py(PyValue::Bool(b)) => Ok(PyInt::from_i64(i64::from(*b))),
        Value::Py(PyValue::Float(f)) => float_to_int(*f, lineno),
        Value::Py(PyValue::Str(_)) | Value::Markup(_) => {
            pytext::int_from_string(v.text().unwrap_or_default(), 10).map_err(|e| from_pyerr(&e, lineno))
        }
        Value::Py(PyValue::Bytes(b)) => {
            let text = String::from_utf8_lossy(b);
            pytext::int_from_string(&text, 10).map_err(|_| {
                value_error(format!("invalid literal for int() with base 10: {}", pytext::bytes_repr(b)), lineno)
            })
        }
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        other => Err(type_error(
            format!(
                "int() argument must be a string, a bytes-like object or a number, not '{}'",
                other.type_name()
            ),
            lineno,
        )),
    }
}

/// `int(f)` for a float.
fn float_to_int(f: f64, lineno: usize) -> R<PyInt> {
    if f.is_nan() {
        return Err(value_error("cannot convert float NaN to integer", lineno));
    }
    if f.is_infinite() {
        return Err(error("OverflowError", "cannot convert float infinity to integer", lineno));
    }
    Ok(pytext::trunc_float_to_bigint(f))
}

/// `len(v)`.
fn len_of(v: &Value, lineno: usize) -> R<usize> {
    match v {
        Value::Undefined(_) => Ok(0),
        Value::Py(PyValue::Str(s)) => Ok(s.chars().count()),
        Value::Markup(s) => Ok(s.chars().count()),
        Value::Py(PyValue::Bytes(b)) => Ok(b.len()),
        Value::Py(PyValue::List(l) | PyValue::Tuple(l)) => Ok(l.len()),
        Value::Py(PyValue::Dict(d)) => Ok(d.len()),
        Value::Seq(_, items) => Ok(items.len()),
        other => Err(type_error(format!("object of type '{}' has no len()", other.type_name()), lineno)),
    }
}

fn len_value(v: &Value, lineno: usize) -> R<Value> {
    Ok(int_value(i64::try_from(len_of(v, lineno)?).unwrap_or(i64::MAX)))
}

/// `isinstance(v, str)` (a `Markup` is one).
fn is_str(v: &Value) -> bool {
    v.text().is_some()
}

// ---------------------------------------------------------------------------
// iteration
// ---------------------------------------------------------------------------

/// `iter(v)`.
enum Items {
    Eager(std::vec::IntoIter<Value>),
    Lazy(Rc<RefCell<LazyIter>>),
}

impl Items {
    fn next(&mut self) -> R<Option<Value>> {
        match self {
            Items::Eager(it) => Ok(it.next()),
            Items::Lazy(it) => it.borrow_mut().next_item(),
        }
    }
    fn collect(mut self) -> R<Vec<Value>> {
        let mut out = Vec::new();
        while let Some(item) = self.next()? {
            out.push(item);
        }
        Ok(out)
    }
}

fn py_iter(v: &Value, lineno: usize) -> R<Items> {
    match v {
        Value::Iter(it) => Ok(Items::Lazy(it.clone())),
        other => Ok(Items::Eager(iterate(other, lineno)?.into_iter())),
    }
}

/// A list of the items `v` iterates (`list(v)`).
fn list_of(v: &Value, lineno: usize) -> R<Vec<Value>> {
    py_iter(v, lineno)?.collect()
}

fn list_value(items: Vec<Value>) -> Value {
    make_seq(SeqKind::List, items)
}

/// A sequence's items without copying: a `PyValue` list/tuple or a template one.
enum SeqView<'a> {
    Py(&'a [PyValue]),
    Template(&'a [Value]),
}

impl SeqView<'_> {
    fn len(&self) -> usize {
        match self {
            SeqView::Py(items) => items.len(),
            SeqView::Template(items) => items.len(),
        }
    }
    fn get(&self, i: usize) -> Value {
        match self {
            SeqView::Py(items) => Value::py(items[i].clone()),
            SeqView::Template(items) => items[i].clone(),
        }
    }
}

/// `(is_list, items)` of a list or tuple (a `_GroupTuple` is a tuple).
fn seq_view(v: &Value) -> Option<(bool, SeqView<'_>)> {
    match v {
        Value::Py(PyValue::List(items)) => Some((true, SeqView::Py(items))),
        Value::Py(PyValue::Tuple(items)) => Some((false, SeqView::Py(items))),
        Value::Seq(kind, items) => Some((*kind == SeqKind::List, SeqView::Template(items))),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// equality, ordering, hashing
// ---------------------------------------------------------------------------

/// Whether `v` is a value only this module creates (`Markup`, template
/// list/tuple, iterator).
pub(super) fn is_template_object(v: &Value) -> bool {
    matches!(v, Value::Markup(_) | Value::Seq(..) | Value::Iter(_))
}

/// Python `a == b`.
pub(super) fn v_eq(a: &Value, b: &Value) -> bool {
    if let (Some(x), Some(y)) = (a.text(), b.text()) {
        return x == y;
    }
    match (a, b) {
        (Value::Py(x), Value::Py(y)) => return py_equal(x, y),
        (Value::Undefined(_), Value::Undefined(_)) => return true,
        (Value::Status(x), Value::Status(y)) => return Rc::ptr_eq(x, y),
        (Value::Iter(x), Value::Iter(y)) => return Rc::ptr_eq(x, y),
        (Value::Func(x), Value::Func(y)) => return std::ptr::addr_eq(Rc::as_ptr(x), Rc::as_ptr(y)),
        _ => {}
    }
    match (seq_view(a), seq_view(b)) {
        (Some((la, xa)), Some((lb, xb))) => {
            la == lb && xa.len() == xb.len() && (0..xa.len()).all(|i| v_eq(&xa.get(i), &xb.get(i)))
        }
        _ => false,
    }
}

fn ordering_holds(op: &str, o: Ordering) -> bool {
    match op {
        "<" => o == Ordering::Less,
        "<=" => o != Ordering::Greater,
        ">" => o == Ordering::Greater,
        _ => o != Ordering::Less,
    }
}

/// Python `a <op> b` for `<`, `<=`, `>`, `>=`.
pub(super) fn v_cmp(op: &str, a: &Value, b: &Value, lineno: usize) -> R<bool> {
    if a.is_undefined() {
        return Err(a.fail_undefined(lineno));
    }
    if b.is_undefined() {
        return Err(b.fail_undefined(lineno));
    }
    if let (Some(x), Some(y)) = (a.text(), b.text()) {
        return Ok(ordering_holds(op, x.cmp(y)));
    }
    if let (Some((la, xa)), Some((lb, xb))) = (seq_view(a), seq_view(b)) {
        if la == lb {
            for i in 0..xa.len().min(xb.len()) {
                let (p, q) = (xa.get(i), xb.get(i));
                if !v_eq(&p, &q) {
                    return v_cmp(op, &p, &q, lineno);
                }
            }
            return Ok(ordering_holds(op, xa.len().cmp(&xb.len())));
        }
    }
    if let (Value::Py(x), Value::Py(y)) = (a, b) {
        return pycompat::value::py_rich_compare(op, x, y).map_err(|message| {
            let message = match message.strip_prefix("'<'") {
                Some(rest) => format!("'{op}'{rest}"),
                None => message,
            };
            type_error(message, lineno)
        });
    }
    Err(type_error(
        format!("'{op}' not supported between instances of '{}' and '{}'", a.type_name(), b.type_name()),
        lineno,
    ))
}

/// The type `hash(v)` fails on, if any.
fn unhashable(v: &Value) -> Option<&'static str> {
    fn py(v: &PyValue) -> Option<&'static str> {
        match v {
            PyValue::List(_) => Some("list"),
            PyValue::Dict(_) => Some("dict"),
            PyValue::Tuple(items) => items.iter().find_map(py),
            _ => None,
        }
    }
    match v {
        Value::Py(p) => py(p),
        Value::Seq(SeqKind::List, _) => Some("list"),
        Value::Seq(_, items) => items.iter().find_map(unhashable),
        _ => None,
    }
}

fn check_hashable(v: &Value, lineno: usize) -> R<()> {
    match unhashable(v) {
        Some(t) => Err(type_error(format!("unhashable type: '{t}'"), lineno)),
        None => Ok(()),
    }
}

/// Python `elem in container` when either side is a template object.
pub(super) fn contains_value(container: &Value, elem: &Value, lineno: usize) -> R<bool> {
    if let Some(text) = container.text() {
        return match elem.text() {
            Some(sub) => Ok(text.contains(sub)),
            None => Err(type_error(
                format!("'in <string>' requires string as left operand, not {}", elem.type_name()),
                lineno,
            )),
        };
    }
    if let Some((_, items)) = seq_view(container) {
        return Ok((0..items.len()).any(|i| v_eq(&items.get(i), elem)));
    }
    match container {
        Value::Undefined(_) => Ok(false),
        Value::Py(PyValue::Dict(d)) => {
            check_hashable(elem, lineno)?;
            Ok(d.keys().any(|k| v_eq(&Value::py(k.clone()), elem)))
        }
        Value::Py(PyValue::Bytes(_)) => Err(type_error(
            format!("a bytes-like object is required, not '{}'", elem.type_name()),
            lineno,
        )),
        Value::Iter(it) => {
            while let Some(item) = it.borrow_mut().next_item()? {
                if v_eq(&item, elem) {
                    return Ok(true);
                }
            }
            Ok(false)
        }
        Value::Status(w) => w.contains(&elem.to_string()),
        other => Err(type_error(format!("argument of type '{}' is not iterable", other.type_name()), lineno)),
    }
}

// ---------------------------------------------------------------------------
// operators on template objects
// ---------------------------------------------------------------------------

/// `a + b` / `a * b` when either side is a template object (neither is
/// `Undefined`, which the caller handles).
pub(super) fn object_binop(op: &str, a: &Value, b: &Value, lineno: usize) -> R<Value> {
    let unsupported_operands = || {
        type_error(
            format!("unsupported operand type(s) for {op}: '{}' and '{}'", a.type_name(), b.type_name()),
            lineno,
        )
    };
    if op == "+" {
        if let Value::Markup(x) = a {
            return match b.text() {
                Some(_) => Ok(markup_value(format!("{x}{}", escape(b, lineno)?))),
                None => Err(unsupported_operands()),
            };
        }
        if let Value::Markup(y) = b {
            return match a.text() {
                Some(_) => Ok(markup_value(format!("{}{y}", escape(a, lineno)?))),
                None => Err(unsupported_operands()),
            };
        }
        return match (seq_view(a), seq_view(b)) {
            (Some((la, xa)), Some((lb, xb))) if la == lb => {
                let items = (0..xa.len()).map(|i| xa.get(i)).chain((0..xb.len()).map(|i| xb.get(i))).collect();
                Ok(make_seq(if la { SeqKind::List } else { SeqKind::Tuple }, items))
            }
            (Some((la, _)), _) => {
                let t = if la { "list" } else { "tuple" };
                Err(type_error(format!("can only concatenate {t} (not \"{}\") to {t}", b.type_name()), lineno))
            }
            _ if a.text().is_some() => Err(type_error(
                format!("can only concatenate str (not \"{}\") to str", b.type_name()),
                lineno,
            )),
            _ => Err(unsupported_operands()),
        };
    }
    // `*`: sequence repetition by an int. `Markup` defines `__mul__`, so
    // its type has no `sq_repeat` slot for CPython's fallback.
    let is_int = |v: &Value| matches!(v, Value::Py(PyValue::Int(_) | PyValue::Bool(_)));
    let repeats = |v: &Value| matches!(v, Value::Markup(_)) || seq_view(v).is_some();
    let has_sq_repeat = |v: &Value| {
        matches!(v, Value::Py(PyValue::Str(_) | PyValue::List(_) | PyValue::Tuple(_) | PyValue::Bytes(_)) | Value::Seq(..))
    };
    let (seq, count) = if repeats(a) && is_int(b) {
        (a, b)
    } else if repeats(b) && is_int(a) {
        (b, a)
    } else if has_sq_repeat(a) {
        return Err(type_error(format!("can't multiply sequence by non-int of type '{}'", b.type_name()), lineno));
    } else if has_sq_repeat(b) {
        return Err(type_error(format!("can't multiply sequence by non-int of type '{}'", a.type_name()), lineno));
    } else {
        return Err(unsupported_operands());
    };
    let count = index_value(count, lineno)
        .map_err(|_| error("OverflowError", "cannot fit 'int' into an index-sized integer", lineno))?;
    let times = usize::try_from(count).unwrap_or(0);
    match seq {
        Value::Markup(s) => Ok(markup_value(s.repeat(times))),
        _ => {
            let Some((is_list, items)) = seq_view(seq) else {
                return Err(unsupported_operands());
            };
            let mut out = Vec::with_capacity(items.len().saturating_mul(times));
            for _ in 0..times {
                out.extend((0..items.len()).map(|i| items.get(i)));
            }
            Ok(make_seq(if is_list { SeqKind::List } else { SeqKind::Tuple }, out))
        }
    }
}

// ---------------------------------------------------------------------------
// `str % args`
// ---------------------------------------------------------------------------

/// The right operand of `%`.
enum PctArgs<'a> {
    /// A tuple: one item per conversion.
    Tuple(Vec<Value>),
    /// Any other value; `mapping` when Python treats it as one
    /// (`PyMapping_Check`), which allows `%(key)s` and silences "not all
    /// arguments converted".
    Single { value: &'a Value, mapping: bool },
    /// The `**kwargs` of `|format`.
    Keywords(&'a [(String, Value)]),
}

/// `fmt % rhs`, or `Markup(fmt) % rhs` when `markup` (every argument then
/// goes through markupsafe's `_MarkupEscapeHelper`).
pub(super) fn percent_format(fmt: &str, rhs: &Value, markup: bool, lineno: usize) -> R<String> {
    let args = match rhs {
        Value::Py(PyValue::Tuple(t)) => PctArgs::Tuple(t.iter().cloned().map(Value::py).collect()),
        Value::Seq(SeqKind::Tuple | SeqKind::Group, items) => PctArgs::Tuple(items.to_vec()),
        value => PctArgs::Single {
            value,
            mapping: markup
                || matches!(
                    value,
                    Value::Py(PyValue::Dict(_) | PyValue::List(_) | PyValue::Bytes(_))
                        | Value::Seq(..)
                        | Value::Undefined(_)
                        | Value::Status(_)
                ),
        },
    };
    percent_format_args(fmt, &args, markup, lineno)
}

fn percent_format_args(fmt: &str, args: &PctArgs, markup: bool, lineno: usize) -> R<String> {
    let chars: Vec<char> = fmt.chars().collect();
    let (mut arglen, mut argidx): (isize, isize) = match args {
        PctArgs::Tuple(t) => (t.len() as isize, 0),
        _ => (-1, -2),
    };
    let is_mapping = matches!(args, PctArgs::Single { mapping: true, .. } | PctArgs::Keywords(_));
    let whole = |args: &PctArgs| -> Value {
        match args {
            PctArgs::Single { value, .. } => (*value).clone(),
            PctArgs::Keywords(kw) => {
                let parts: Vec<String> =
                    kw.iter().map(|(k, v)| format!("{}: {}", pytext::str_repr(k), v.repr())).collect();
                str_value(format!("{{{}}}", parts.join(", ")))
            }
            PctArgs::Tuple(_) => none_value(),
        }
    };
    let next_arg = |arglen: isize, argidx: &mut isize| -> R<Value> {
        if *argidx < arglen {
            *argidx += 1;
            if arglen < 0 {
                return Ok(whole(args));
            }
            if let PctArgs::Tuple(t) = args {
                return Ok(t[(*argidx - 1) as usize].clone());
            }
        }
        Err(type_error("not enough arguments for format string", lineno))
    };
    let mut out = String::new();
    let mut i = 0;
    while i < chars.len() {
        if chars[i] != '%' {
            out.push(chars[i]);
            i += 1;
            continue;
        }
        i += 1;
        let mut value: Option<Value> = None;
        if chars.get(i) == Some(&'(') {
            let mut depth = 1;
            let start = i + 1;
            i += 1;
            while i < chars.len() && depth > 0 {
                match chars[i] {
                    '(' => depth += 1,
                    ')' => depth -= 1,
                    _ => {}
                }
                i += 1;
            }
            if depth > 0 {
                return Err(value_error("incomplete format key", lineno));
            }
            let key: String = chars[start..i - 1].iter().collect();
            if !is_mapping {
                return Err(type_error("format requires a mapping", lineno));
            }
            value = Some(match args {
                PctArgs::Keywords(kw) => match kw.iter().find(|(k, _)| *k == key) {
                    Some((_, v)) => v.clone(),
                    None => return Err(error("KeyError", pytext::str_repr(&key), lineno)),
                },
                PctArgs::Single { value, .. } => mapping_item(value, &key, lineno)?,
                PctArgs::Tuple(_) => unreachable!("tuples are not mappings"),
            });
            // CPython makes the looked-up value the (single, now consumed)
            // argument: a later key-less conversion has none left.
            arglen = -1;
            argidx = -1;
        }
        let mut flags = String::new();
        while let Some(&c) = chars.get(i) {
            if "-+ #0".contains(c) {
                flags.push(c);
                i += 1;
            } else {
                break;
            }
        }
        let mut width = String::new();
        if chars.get(i) == Some(&'*') {
            let w = next_arg(arglen, &mut argidx)?;
            width = star_value(&w, lineno)?.to_string();
            i += 1;
        } else {
            while let Some(c) = chars.get(i).filter(|c| c.is_ascii_digit()) {
                width.push(*c);
                i += 1;
            }
        }
        let mut precision: Option<String> = None;
        if chars.get(i) == Some(&'.') {
            i += 1;
            if chars.get(i) == Some(&'*') {
                let p = next_arg(arglen, &mut argidx)?;
                precision = Some(star_value(&p, lineno)?.max(0).to_string());
                i += 1;
            } else {
                let mut p = String::new();
                while let Some(c) = chars.get(i).filter(|c| c.is_ascii_digit()) {
                    p.push(*c);
                    i += 1;
                }
                precision = Some(p);
            }
        }
        while matches!(chars.get(i), Some('h' | 'l' | 'L')) {
            i += 1;
        }
        let Some(&conv) = chars.get(i) else {
            return Err(value_error("incomplete format", lineno));
        };
        i += 1;
        if conv == '%' {
            out.push('%');
            continue;
        }
        let value = match value {
            Some(v) => v,
            None => next_arg(arglen, &mut argidx)?,
        };
        let spec = |c: char| {
            let mut s = format!("%{flags}{width}");
            if let Some(p) = &precision {
                s.push('.');
                s.push_str(p);
            }
            s.push(c);
            s
        };
        let text_arg = |text: String| -> R<String> {
            pytext::percent_format(&spec('s'), &[PctValue::Str(text)]).map_err(|e| from_pyerr(&e, lineno))
        };
        let piece = match conv {
            's' => text_arg(if markup { escape(&value, lineno)? } else { py_str(&value, lineno)? })?,
            'r' | 'a' => {
                let mut r = value.repr();
                if markup {
                    r = escape_text(&r);
                }
                if conv == 'a' {
                    r = ascii_escape(&r);
                }
                text_arg(r)?
            }
            'd' | 'i' | 'u' | 'o' | 'x' | 'X' | 'e' | 'E' | 'f' | 'F' | 'g' | 'G' | 'c' => {
                let pct = numeric_pct(conv, &value, markup, lineno)?;
                pytext::percent_format(&spec(conv), &[pct]).map_err(|e| from_pyerr(&e, lineno))?
            }
            other => {
                return Err(value_error(
                    format!(
                        "unsupported format character '{other}' (0x{:x}) at index {}",
                        other as u32,
                        i - 1
                    ),
                    lineno,
                ))
            }
        };
        out.push_str(&piece);
    }
    if argidx < arglen && !is_mapping {
        return Err(type_error("not all arguments converted during string formatting", lineno));
    }
    Ok(out)
}

/// `rhs[key]` for a `%(key)s` conversion.
fn mapping_item(rhs: &Value, key: &str, lineno: usize) -> R<Value> {
    match rhs {
        Value::Py(PyValue::Dict(d)) => match dict_lookup(d, &PyValue::Str(key.to_string())) {
            Some(v) => Ok(Value::py(v.clone())),
            None => Err(error("KeyError", pytext::str_repr(key), lineno)),
        },
        Value::Undefined(_) => Err(rhs.fail_undefined(lineno)),
        Value::Py(PyValue::Bytes(_)) => Err(type_error("byte indices must be integers or slices, not str", lineno)),
        Value::Py(PyValue::List(_)) | Value::Seq(SeqKind::List, _) => {
            Err(type_error("list indices must be integers or slices, not str", lineno))
        }
        Value::Status(_) => match get_item(rhs, &str_value(key), lineno)? {
            Value::Undefined(_) => Err(error("KeyError", pytext::str_repr(key), lineno)),
            found => Ok(found),
        },
        Value::Markup(_) | Value::Py(PyValue::Str(_)) => {
            Err(type_error("string indices must be integers", lineno))
        }
        Value::Seq(..) | Value::Py(PyValue::Tuple(_)) => {
            Err(type_error("tuple indices must be integers or slices, not str", lineno))
        }
        other => Err(type_error(format!("'{}' object is not subscriptable", other.type_name()), lineno)),
    }
}

/// A `*` width or precision.
fn star_value(v: &Value, lineno: usize) -> R<i64> {
    match v {
        Value::Py(PyValue::Int(_) | PyValue::Bool(_)) => index_value(v, lineno),
        _ => Err(type_error("* wants int", lineno)),
    }
}

/// `ascii()` escaping of a `repr` text.
fn ascii_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        let cp = c as u32;
        if cp < 0x80 {
            out.push(c);
        } else if cp < 0x100 {
            out.push_str(&format!("\\x{cp:02x}"));
        } else if cp < 0x10000 {
            out.push_str(&format!("\\u{cp:04x}"));
        } else {
            out.push_str(&format!("\\U{cp:08x}"));
        }
    }
    out
}

/// The numeric argument of a `%d`-family / `%f`-family / `%c` conversion.
fn numeric_pct(conv: char, value: &Value, markup: bool, lineno: usize) -> R<PctValue> {
    // `_MarkupEscapeHelper` converts through `int(obj)` / `float(obj)`.
    if markup {
        return match conv {
            'd' | 'i' | 'u' => Ok(PctValue::Int(int_of(value, lineno)?)),
            'e' | 'E' | 'f' | 'F' | 'g' | 'G' => Ok(PctValue::Float(float_of(value, lineno)?)),
            'c' => Err(type_error("%c requires int or char", lineno)),
            _ => Err(type_error(format!("%{conv} format: an integer is required, not _MarkupEscapeHelper"), lineno)),
        };
    }
    match value {
        Value::Py(PyValue::Int(n)) => Ok(PctValue::Int(n.clone())),
        Value::Py(PyValue::Bool(b)) => Ok(PctValue::Bool(*b)),
        Value::Py(PyValue::Float(f)) => Ok(PctValue::Float(*f)),
        Value::Py(PyValue::Str(s)) if conv == 'c' => Ok(PctValue::Str(s.clone())),
        Value::Undefined(_) if matches!(conv, 'd' | 'i' | 'u' | 'e' | 'E' | 'f' | 'F' | 'g' | 'G') => {
            Err(value.fail_undefined(lineno))
        }
        other => Err(type_error(
            match conv {
                'd' | 'i' | 'u' => format!("%{conv} format: a number is required, not {}", other.type_name()),
                'o' | 'x' | 'X' => format!("%{conv} format: an integer is required, not {}", other.type_name()),
                'c' => "%c requires int or char".to_string(),
                _ => format!("must be real number, not {}", other.type_name()),
            },
            lineno,
        )),
    }
}

// ---------------------------------------------------------------------------
// attributes and items of template objects
// ---------------------------------------------------------------------------

/// `str` methods a `Markup` value can call.
const MARKUP_METHODS: &[&str] = &[
    "capitalize", "endswith", "join", "lower", "lstrip", "replace", "rsplit", "rstrip", "split",
    "splitlines", "startswith", "strip", "striptags", "swapcase", "title", "unescape", "upper",
];

/// Python `getattr(base, attr)` for a `Markup`, template list/tuple or
/// iterator: `None` for `AttributeError`.
pub(super) fn object_attribute(base: &Value, attr: &str, lineno: usize) -> Option<R<Value>> {
    let dunder = attr.len() > 4 && attr.starts_with("__") && attr.ends_with("__");
    match base {
        Value::Markup(s) => {
            if attr == "__html__" {
                let s = s.clone();
                return Some(Ok(Value::Func(Rc::new(move |args, kwargs, ln| {
                    if !args.is_empty() || !kwargs.is_empty() {
                        return Err(type_error(
                            format!("__html__() takes 1 positional argument but {} were given", args.len() + 1),
                            ln,
                        ));
                    }
                    Ok(Value::Markup(s.clone()))
                }))));
            }
            if MARKUP_METHODS.contains(&attr) {
                let s = s.clone();
                let method = attr.to_string();
                return Some(Ok(Value::Func(Rc::new(move |args, kwargs, ln| {
                    markup_method(&s, &method, args, kwargs, ln)
                }))));
            }
            if py_attribute_names(&PyValue::Str(String::new())).contains(&attr)
                || dunder
                || ["escape", "partition", "rpartition"].contains(&attr)
            {
                return Some(Err(unsupported(format!("attribute '{attr}' of 'Markup' object"), lineno)));
            }
            None
        }
        Value::Seq(SeqKind::Group, items) => match attr {
            "grouper" => Some(Ok(items[0].clone())),
            "list" => Some(Ok(items[1].clone())),
            _ if dunder
                || ["count", "index", "_asdict", "_field_defaults", "_fields", "_fields_defaults", "_make", "_replace"]
                    .contains(&attr) =>
            {
                Some(Err(unsupported(format!("attribute '{attr}' of '_GroupTuple' object"), lineno)))
            }
            _ => None,
        },
        Value::Seq(kind, _) => {
            let names = py_attribute_names(&if *kind == SeqKind::List {
                PyValue::List(Vec::new())
            } else {
                PyValue::Tuple(Vec::new())
            });
            (names.contains(&attr) || dunder)
                .then(|| Err(unsupported(format!("attribute '{attr}' of '{}' object", base.type_name()), lineno)))
        }
        Value::Iter(it) => {
            let generator = it.borrow().type_name == "generator";
            let generator_attr = ["close", "gi_code", "gi_frame", "gi_running", "gi_yieldfrom", "send", "throw"]
                .contains(&attr);
            ((generator && generator_attr) || dunder)
                .then(|| Err(unsupported(format!("attribute '{attr}' of '{}' object", base.type_name()), lineno)))
        }
        _ => None,
    }
}

/// Jinja `Environment.getitem` for a `Markup`, template list/tuple or iterator.
pub(super) fn object_item(base: &Value, key: &Value, lineno: usize) -> R<Value> {
    let index = match key {
        Value::Py(PyValue::Int(n)) => pyint_i64(n),
        Value::Py(PyValue::Bool(b)) => Some(i64::from(*b)),
        _ => None,
    };
    if let Some(index) = index {
        let at = |len: usize| -> Option<usize> {
            let len = i64::try_from(len).ok()?;
            let i = if index < 0 { index + len } else { index };
            (0..len).contains(&i).then_some(i as usize)
        };
        match base {
            Value::Markup(s) => {
                let chars: Vec<char> = s.chars().collect();
                if let Some(i) = at(chars.len()) {
                    return Ok(markup_value(chars[i].to_string()));
                }
            }
            Value::Seq(_, items) => {
                if let Some(i) = at(items.len()) {
                    return Ok(items[i].clone());
                }
            }
            _ => {}
        }
    }
    if let Some(name) = key.text() {
        if let Some(found) = object_attribute(base, name, lineno) {
            return found;
        }
    }
    Ok(Value::undefined_member(base, key))
}

/// A `str` method called on a `Markup`: markupsafe escapes `str`
/// arguments and wraps the result back into `Markup`.
fn markup_method(s: &Rc<str>, method: &str, args: &[Value], kwargs: &[(String, Value)], lineno: usize) -> R<Value> {
    let escaped = |v: &Value| -> R<Value> {
        if is_str(v) {
            Ok(Value::Markup(escape(v, lineno)?.into()))
        } else {
            Ok(v.clone())
        }
    };
    let no_kwargs = || -> R<()> {
        match kwargs.is_empty() {
            true => Ok(()),
            false => Err(type_error(format!("{method}() takes no keyword arguments"), lineno)),
        }
    };
    let at_most = |n: usize| -> R<()> {
        no_kwargs()?;
        if args.len() > n {
            return Err(type_error(
                match n {
                    0 => format!("{method}() takes no arguments ({} given)", args.len()),
                    _ => format!("{method} expected at most {n} argument{}, got {}", plural(n), args.len()),
                },
                lineno,
            ));
        }
        Ok(())
    };
    match method {
        "upper" | "lower" | "capitalize" | "swapcase" | "title" => {
            at_most(0)?;
            let out = match method {
                "upper" => s.to_uppercase(),
                "lower" => s.to_lowercase(),
                "capitalize" => py_capitalize(s),
                "swapcase" => py_swapcase(s),
                _ => py_title(s),
            };
            Ok(markup_value(out))
        }
        "strip" | "lstrip" | "rstrip" => {
            at_most(1)?;
            let chars = args.first().map(escaped).transpose()?;
            Ok(markup_value(py_strip(s, chars.as_ref(), method, lineno)?))
        }
        "replace" => {
            no_kwargs()?;
            if args.len() < 2 || args.len() > 3 {
                return Err(type_error(
                    if args.len() < 2 {
                        format!("replace expected at least 2 arguments, got {}", args.len())
                    } else {
                        format!("replace expected at most 3 arguments, got {}", args.len())
                    },
                    lineno,
                ));
            }
            let old = escaped(&args[0])?;
            let new = escaped(&args[1])?;
            let count = match args.get(2) {
                Some(c) => ssize_arg(c, lineno)?,
                None => -1,
            };
            let (old, new) = (need_text(&old, lineno)?, need_text(&new, lineno)?);
            Ok(markup_value(py_replace(s, &old, &new, count)))
        }
        "startswith" | "endswith" => {
            no_kwargs()?;
            let Some(prefix) = args.first() else {
                return Err(type_error(format!("{method}() takes at least 1 argument (0 given)"), lineno));
            };
            let candidates: Vec<Value> = match seq_view(prefix) {
                Some((false, items)) => (0..items.len()).map(|i| items.get(i)).collect(),
                _ => vec![prefix.clone()],
            };
            for c in &candidates {
                let Some(t) = c.text() else {
                    return Err(type_error(
                        format!(
                            "{method} first arg must be str or a tuple of str, not {}",
                            prefix.type_name()
                        ),
                        lineno,
                    ));
                };
                if (method == "startswith" && s.starts_with(t)) || (method == "endswith" && s.ends_with(t)) {
                    return Ok(bool_value(true));
                }
            }
            Ok(bool_value(false))
        }
        "split" | "rsplit" => {
            if !kwargs.is_empty() || args.len() > 2 {
                return Err(unsupported(format!("Markup.{method} arguments"), lineno));
            }
            let sep = match args.first() {
                None => None,
                Some(v) if is_none(v) => None,
                Some(v) => Some(need_text(v, lineno)?),
            };
            let maxsplit = match args.get(1) {
                Some(v) => ssize_arg(v, lineno)?,
                None => -1,
            };
            if sep.as_deref() == Some("") {
                return Err(value_error("empty separator", lineno));
            }
            let parts = py_split(s, sep.as_deref(), maxsplit, method == "rsplit");
            Ok(list_value(parts.into_iter().map(markup_value).collect()))
        }
        "splitlines" => {
            at_most(1)?;
            let keepends = args.first().is_some_and(Value::truthy);
            let lines = splitlines_keep(s);
            Ok(list_value(
                lines
                    .into_iter()
                    .map(|(line, end)| markup_value(if keepends { format!("{line}{end}") } else { line.to_string() }))
                    .collect(),
            ))
        }
        "join" => {
            no_kwargs()?;
            if args.len() != 1 {
                return Err(type_error(format!("join() takes exactly one argument ({} given)", args.len()), lineno));
            }
            let mut parts = Vec::new();
            for item in list_of(&args[0], lineno)? {
                parts.push(escape(&item, lineno)?);
            }
            Ok(markup_value(parts.join(s)))
        }
        "unescape" => {
            at_most(0)?;
            Ok(str_value(unescape(s, lineno)?))
        }
        "striptags" => {
            at_most(0)?;
            Ok(str_value(striptags(s, lineno)?))
        }
        _ => Err(unsupported(format!("Markup.{method}()"), lineno)),
    }
}

fn need_text(v: &Value, lineno: usize) -> R<String> {
    match v.text() {
        Some(t) => Ok(t.to_string()),
        None => Err(type_error(format!("replace() argument must be str, not {}", v.type_name()), lineno)),
    }
}

// ---------------------------------------------------------------------------
// text helpers (CPython `str` methods)
// ---------------------------------------------------------------------------

/// Characters whose titlecase mapping differs from their uppercase one.
fn to_title(c: char) -> String {
    let mapped = match c {
        '\u{01C4}'..='\u{01C6}' => "\u{01C5}",
        '\u{01C7}'..='\u{01C9}' => "\u{01C8}",
        '\u{01CA}'..='\u{01CC}' => "\u{01CB}",
        '\u{01F1}'..='\u{01F3}' => "\u{01F2}",
        'ß' => "Ss",
        'ﬀ' => "Ff",
        'ﬁ' => "Fi",
        'ﬂ' => "Fl",
        'ﬃ' => "Ffi",
        'ﬄ' => "Ffl",
        'ﬅ' | 'ﬆ' => "St",
        'և' => "Եւ",
        'ﬓ' => "Մն",
        'ﬔ' => "Մե",
        'ﬕ' => "Մի",
        'ﬖ' => "Վն",
        'ﬗ' => "Մխ",
        'ŉ' => "ʼN",
        '\u{1F80}'..='\u{1F87}' => return char::from_u32(c as u32 + 8).map(String::from).unwrap_or_default(),
        '\u{1F90}'..='\u{1F97}' => return char::from_u32(c as u32 + 8).map(String::from).unwrap_or_default(),
        '\u{1FA0}'..='\u{1FA7}' => return char::from_u32(c as u32 + 8).map(String::from).unwrap_or_default(),
        '\u{1F88}'..='\u{1F8F}' | '\u{1F98}'..='\u{1F9F}' | '\u{1FA8}'..='\u{1FAF}' => {
            return c.to_string()
        }
        '\u{1FB3}' | '\u{1FBC}' => "\u{1FBC}",
        '\u{1FC3}' | '\u{1FCC}' => "\u{1FCC}",
        '\u{1FF3}' | '\u{1FFC}' => "\u{1FFC}",
        '\u{1FB2}' => "\u{1FBA}\u{0345}",
        '\u{1FB4}' => "\u{0386}\u{0345}",
        '\u{1FB7}' => "\u{0391}\u{0342}\u{0345}",
        '\u{1FC2}' => "\u{1FCA}\u{0345}",
        '\u{1FC4}' => "\u{0389}\u{0345}",
        '\u{1FC7}' => "\u{0397}\u{0342}\u{0345}",
        '\u{1FF2}' => "\u{1FFA}\u{0345}",
        '\u{1FF4}' => "\u{038F}\u{0345}",
        '\u{1FF7}' => "\u{03A9}\u{0342}\u{0345}",
        '\u{10D0}'..='\u{10FA}' | '\u{10FD}'..='\u{10FF}' => return c.to_string(),
        _ => return c.to_uppercase().collect(),
    };
    mapped.to_string()
}

fn is_titlecase(c: char) -> bool {
    matches!(
        c,
        '\u{01C5}' | '\u{01C8}' | '\u{01CB}' | '\u{01F2}' | '\u{1F88}'..='\u{1F8F}' | '\u{1F98}'..='\u{1F9F}'
            | '\u{1FA8}'..='\u{1FAF}' | '\u{1FBC}' | '\u{1FCC}' | '\u{1FFC}'
    )
}

fn is_cased(c: char) -> bool {
    c.is_uppercase() || c.is_lowercase() || is_titlecase(c)
}

/// `str.capitalize()` (3.8: the first character titlecased).
fn py_capitalize(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        Some(first) => {
            let mut out = to_title(first);
            out.push_str(&chars.as_str().to_lowercase());
            out
        }
        None => String::new(),
    }
}

/// `str.title()`.
fn py_title(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut previous_is_cased = false;
    for c in s.chars() {
        if previous_is_cased {
            out.extend(c.to_lowercase());
        } else {
            out.push_str(&to_title(c));
        }
        previous_is_cased = is_cased(c);
    }
    out
}

/// `str.swapcase()`.
fn py_swapcase(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        if c.is_uppercase() {
            out.extend(c.to_lowercase());
        } else if c.is_lowercase() {
            out.extend(c.to_uppercase());
        } else {
            out.push(c);
        }
    }
    out
}

/// `str.islower()` / `str.isupper()`.
fn py_is_case(s: &str, lower: bool) -> bool {
    let mut cased = false;
    for c in s.chars() {
        let (wanted, other) = if lower {
            (c.is_lowercase(), c.is_uppercase() || is_titlecase(c))
        } else {
            (c.is_uppercase(), c.is_lowercase() || is_titlecase(c))
        };
        if other {
            return false;
        }
        cased |= wanted;
    }
    cased
}

/// `str.strip/lstrip/rstrip(chars)`.
fn py_strip(s: &str, chars: Option<&Value>, method: &str, lineno: usize) -> R<String> {
    let set: Option<Vec<char>> = match chars {
        None => None,
        Some(v) if is_none(v) => None,
        Some(v) => match v.text() {
            Some(t) => Some(t.chars().collect()),
            None => return Err(type_error(format!("{method} arg must be None or str"), lineno)),
        },
    };
    let strip = |c: char| match &set {
        Some(set) => set.contains(&c),
        None => is_py_space(c),
    };
    Ok(match method {
        "lstrip" => s.trim_start_matches(strip).to_string(),
        "rstrip" => s.trim_end_matches(strip).to_string(),
        _ => s.trim_matches(strip).to_string(),
    })
}

/// `str.replace(old, new, count)`.
fn py_replace(s: &str, old: &str, new: &str, count: i64) -> String {
    let mut remaining = if count < 0 { usize::MAX } else { count as usize };
    if old.is_empty() {
        let mut out = String::with_capacity(s.len());
        for c in s.chars() {
            if remaining > 0 {
                out.push_str(new);
                remaining -= 1;
            }
            out.push(c);
        }
        if remaining > 0 {
            out.push_str(new);
        }
        return out;
    }
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while remaining > 0 {
        match rest.find(old) {
            Some(i) => {
                out.push_str(&rest[..i]);
                out.push_str(new);
                rest = &rest[i + old.len()..];
                remaining -= 1;
            }
            None => break,
        }
    }
    out.push_str(rest);
    out
}

/// `str.split(sep, maxsplit)` / `str.rsplit(...)`.
fn py_split(s: &str, sep: Option<&str>, maxsplit: i64, from_right: bool) -> Vec<String> {
    let limit = if maxsplit < 0 { usize::MAX } else { maxsplit as usize };
    match sep {
        Some(sep) => {
            let mut parts: Vec<String> = if from_right {
                let mut v: Vec<String> = s.rsplitn(limit.saturating_add(1), sep).map(str::to_string).collect();
                v.reverse();
                v
            } else {
                s.splitn(limit.saturating_add(1), sep).map(str::to_string).collect()
            };
            if parts.is_empty() {
                parts.push(String::new());
            }
            parts
        }
        None => {
            let chars: Vec<char> = s.chars().collect();
            let mut words: Vec<(usize, usize)> = Vec::new();
            let mut i = 0;
            while i < chars.len() {
                while i < chars.len() && is_py_space(chars[i]) {
                    i += 1;
                }
                if i == chars.len() {
                    break;
                }
                let start = i;
                while i < chars.len() && !is_py_space(chars[i]) {
                    i += 1;
                }
                words.push((start, i));
            }
            if words.len() <= limit {
                return words.iter().map(|&(a, b)| chars[a..b].iter().collect()).collect();
            }
            if from_right {
                let cut = words.len() - limit;
                let mut head_end = words[cut - 1].1;
                let _ = &mut head_end;
                let lead = words[0].0;
                let mut out = vec![chars[lead..head_end].iter().collect::<String>()];
                out.extend(words[cut..].iter().map(|&(a, b)| chars[a..b].iter().collect::<String>()));
                out
            } else {
                let mut out: Vec<String> = words[..limit].iter().map(|&(a, b)| chars[a..b].iter().collect()).collect();
                let tail_start = words[limit].0;
                let mut tail_end = chars.len();
                while tail_end > tail_start && is_py_space(chars[tail_end - 1]) {
                    tail_end -= 1;
                }
                out.push(chars[tail_start..tail_end].iter().collect());
                out
            }
        }
    }
}

/// `str.splitlines(keepends=True)` as `(line, line_end)` pairs.
fn splitlines_keep(s: &str) -> Vec<(&str, &str)> {
    let mut out = Vec::new();
    let mut start = 0;
    let mut it = s.char_indices().peekable();
    while let Some((i, c)) = it.next() {
        let end_len = match c {
            '\r' if matches!(it.peek(), Some((_, '\n'))) => {
                it.next();
                2
            }
            '\n' | '\r' | '\u{0b}' | '\u{0c}' | '\u{1c}' | '\u{1d}' | '\u{1e}' | '\u{85}' | '\u{2028}'
            | '\u{2029}' => c.len_utf8(),
            _ => continue,
        };
        out.push((&s[start..i], &s[i..i + end_len]));
        start = i + end_len;
    }
    if start < s.len() {
        out.push((&s[start..], ""));
    }
    out
}

/// `str.center(width)`.
fn py_center(s: &str, width: i64) -> String {
    let len = s.chars().count() as i64;
    if width <= len {
        return s.to_string();
    }
    let marg = width - len;
    let left = marg / 2 + (marg & width & 1);
    format!("{}{s}{}", " ".repeat(left as usize), " ".repeat((marg - left) as usize))
}

/// Python `\w`.
fn is_word(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

/// Python `\d`.
fn is_decimal(c: char) -> bool {
    c.is_ascii_digit() || (!c.is_ascii() && c.is_numeric() && !c.is_alphabetic())
}

// ---------------------------------------------------------------------------
// sorting (CPython `list.sort`)
// ---------------------------------------------------------------------------

/// `sorted(values, key=..., reverse=...)` with `keys[i]` the key of
/// `values[i]`, comparing with `lt` in CPython's order: one binary-insertion
/// run for fewer than 64 items (as CPython does), stable merges of
/// `minrun` runs beyond.
fn sort_by_keys<F>(keys: &[Value], values: Vec<Value>, reverse: bool, lt: F) -> R<Vec<Value>>
where
    F: Fn(&Value, &Value) -> R<bool>,
{
    let mut order: Vec<usize> = (0..keys.len()).collect();
    if reverse {
        order.reverse();
    }
    let less = |a: usize, b: usize| lt(&keys[a], &keys[b]);
    timsort(&mut order, &less)?;
    if reverse {
        order.reverse();
    }
    let mut slots: Vec<Option<Value>> = values.into_iter().map(Some).collect();
    Ok(order.into_iter().map(|i| slots[i].take().unwrap_or_else(none_value)).collect())
}

fn timsort<F: Fn(usize, usize) -> R<bool>>(a: &mut [usize], lt: &F) -> R<()> {
    let n = a.len();
    if n < 2 {
        return Ok(());
    }
    let minrun = {
        let (mut m, mut r) = (n, 0);
        while m >= 64 {
            r |= m & 1;
            m >>= 1;
        }
        m + r
    };
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut lo = 0;
    while lo < n {
        let (mut run, descending) = count_run(&a[lo..], lt)?;
        if descending {
            a[lo..lo + run].reverse();
        }
        if run < minrun {
            let force = minrun.min(n - lo);
            binary_sort(&mut a[lo..lo + force], run, lt)?;
            run = force;
        }
        runs.push((lo, run));
        lo += run;
    }
    while runs.len() > 1 {
        let (start, len_a) = runs[runs.len() - 2];
        let (_, len_b) = runs[runs.len() - 1];
        merge(&mut a[start..start + len_a + len_b], len_a, lt)?;
        runs.pop();
        let last = runs.len() - 1;
        runs[last] = (start, len_a + len_b);
    }
    Ok(())
}

fn count_run<F: Fn(usize, usize) -> R<bool>>(a: &[usize], lt: &F) -> R<(usize, bool)> {
    if a.len() == 1 {
        return Ok((1, false));
    }
    let mut n = 2;
    if lt(a[1], a[0])? {
        while n < a.len() && lt(a[n], a[n - 1])? {
            n += 1;
        }
        Ok((n, true))
    } else {
        while n < a.len() && !lt(a[n], a[n - 1])? {
            n += 1;
        }
        Ok((n, false))
    }
}

fn binary_sort<F: Fn(usize, usize) -> R<bool>>(a: &mut [usize], start: usize, lt: &F) -> R<()> {
    let mut start = if start == 0 { 1 } else { start };
    while start < a.len() {
        let pivot = a[start];
        let (mut l, mut r) = (0, start);
        while l < r {
            let p = l + ((r - l) >> 1);
            if lt(pivot, a[p])? {
                r = p;
            } else {
                l = p + 1;
            }
        }
        a.copy_within(l..start, l + 1);
        a[l] = pivot;
        start += 1;
    }
    Ok(())
}

fn merge<F: Fn(usize, usize) -> R<bool>>(a: &mut [usize], mid: usize, lt: &F) -> R<()> {
    let left = a[..mid].to_vec();
    let right = a[mid..].to_vec();
    let (mut i, mut j, mut k) = (0, 0, 0);
    while i < left.len() && j < right.len() {
        if lt(right[j], left[i])? {
            a[k] = right[j];
            j += 1;
        } else {
            a[k] = left[i];
            i += 1;
        }
        k += 1;
    }
    for &x in left[i..].iter().chain(&right[j..]) {
        a[k] = x;
        k += 1;
    }
    Ok(())
}

fn python_lt(lineno: usize) -> impl Fn(&Value, &Value) -> R<bool> {
    move |a, b| v_cmp("<", a, b, lineno)
}

// ---------------------------------------------------------------------------
// attribute getters (`make_attrgetter`)
// ---------------------------------------------------------------------------

/// `_prepare_attribute_parts(attr)`.
fn attribute_parts(attr: Option<&Value>, lineno: usize) -> R<Vec<Value>> {
    let Some(attr) = attr.filter(|a| !is_none(a)) else {
        return Ok(Vec::new());
    };
    match attr.text() {
        Some(text) => text
            .split('.')
            .map(|part| {
                if !part.is_empty() && part.chars().all(|c| c.is_ascii_digit()) {
                    pytext::int_from_string(part, 10).map(|n| Value::py(PyValue::Int(n))).map_err(|e| from_pyerr(&e, lineno))
                } else {
                    Ok(str_value(part))
                }
            })
            .collect(),
        None => Ok(vec![attr.clone()]),
    }
}

/// `make_attrgetter(environment, attribute, postprocess, default)`.
struct AttrGetter {
    parts: Vec<Value>,
    ignore_case: bool,
    default: Option<Value>,
}

impl AttrGetter {
    fn new(attr: Option<&Value>, ignore_case: bool, default: Option<Value>, lineno: usize) -> R<AttrGetter> {
        Ok(AttrGetter { parts: attribute_parts(attr, lineno)?, ignore_case, default })
    }
    fn get(&self, item: &Value, lineno: usize) -> R<Value> {
        let mut item = item.clone();
        for part in &self.parts {
            item = get_item(&item, part, lineno)?;
            if let Some(default) = self.default.as_ref().filter(|d| d.truthy()) {
                if item.is_undefined() {
                    item = default.clone();
                }
            }
        }
        Ok(if self.ignore_case { ignore_case(item) } else { item })
    }
}

/// `ignore_case(value)`: strings lowercased.
fn ignore_case(v: Value) -> Value {
    match &v {
        Value::Py(PyValue::Str(s)) => str_value(s.to_lowercase()),
        Value::Markup(s) => markup_value(s.to_lowercase()),
        _ => v,
    }
}

// ---------------------------------------------------------------------------
// filters
// ---------------------------------------------------------------------------

const SIG_ATTR: Sig = sig("do_attr", &["environment", "obj", "name"], 0, 1);
const SIG_BATCH: Sig = sig("do_batch", &["value", "linecount", "fill_with"], 1, 0);
const SIG_CAPITALIZE: Sig = sig("do_capitalize", &["s"], 0, 0);
const SIG_CENTER: Sig = sig("do_center", &["value", "width"], 1, 0);
const SIG_DEFAULT: Sig = sig("do_default", &["value", "default_value", "boolean"], 2, 0);
const SIG_DICTSORT: Sig = sig("do_dictsort", &["value", "case_sensitive", "by", "reverse"], 3, 0);
const SIG_FILESIZEFORMAT: Sig = sig("do_filesizeformat", &["value", "binary"], 1, 0);
const SIG_FIRST: Sig = sig("do_first", &["environment", "seq"], 0, 1);
const SIG_FLOAT: Sig = sig("do_float", &["value", "default"], 1, 0);
const SIG_FORCEESCAPE: Sig = sig("do_forceescape", &["value"], 0, 0);
const SIG_FORMAT: Sig =
    Sig { name: "do_format", params: &["value"], defaults: 0, implicit: 0, varargs: true, varkw: true };
const SIG_GROUPBY: Sig = sig("do_groupby", &["environment", "value", "attribute"], 0, 1);
const SIG_INDENT: Sig = sig("do_indent", &["s", "width", "first", "blank", "indentfirst"], 4, 0);
const SIG_INT: Sig = sig("do_int", &["value", "default", "base"], 2, 0);
const SIG_JOIN: Sig = sig("do_join", &["eval_ctx", "value", "d", "attribute"], 2, 1);
const SIG_LAST: Sig = sig("do_last", &["environment", "seq"], 0, 1);
const SIG_LIST: Sig = sig("do_list", &["value"], 0, 0);
const SIG_LOWER: Sig = sig("do_lower", &["s"], 0, 0);
const SIG_MIN: Sig = sig("do_min", &["environment", "value", "case_sensitive", "attribute"], 2, 1);
const SIG_MAX: Sig = sig("do_max", &["environment", "value", "case_sensitive", "attribute"], 2, 1);
const SIG_PPRINT: Sig = sig("do_pprint", &["value", "verbose"], 1, 0);
const SIG_RANDOM: Sig = sig("do_random", &["context", "seq"], 0, 1);
const SIG_REPLACE: Sig = sig("do_replace", &["eval_ctx", "s", "old", "new", "count"], 1, 1);
const SIG_REVERSE: Sig = sig("do_reverse", &["value"], 0, 0);
const SIG_ROUND: Sig = sig("do_round", &["value", "precision", "method"], 2, 0);
const SIG_SAFE: Sig = sig("do_mark_safe", &["value"], 0, 0);
const SIG_SLICE: Sig = sig("do_slice", &["value", "slices", "fill_with"], 1, 0);
const SIG_SORT: Sig = sig("do_sort", &["environment", "value", "reverse", "case_sensitive", "attribute"], 3, 1);
const SIG_STRIPTAGS: Sig = sig("do_striptags", &["value"], 0, 0);
const SIG_SUM: Sig = sig("do_sum", &["environment", "iterable", "attribute", "start"], 2, 1);
const SIG_TITLE: Sig = sig("do_title", &["s"], 0, 0);
const SIG_TRIM: Sig = sig("do_trim", &["value", "chars"], 1, 0);
const SIG_TRUNCATE: Sig = sig("do_truncate", &["env", "s", "length", "killwords", "end", "leeway"], 4, 1);
const SIG_UNIQUE: Sig = sig("do_unique", &["environment", "value", "case_sensitive", "attribute"], 2, 1);
const SIG_UPPER: Sig = sig("do_upper", &["s"], 0, 0);
const SIG_URLENCODE: Sig = sig("do_urlencode", &["value"], 0, 0);
const SIG_URLIZE: Sig =
    sig("do_urlize", &["eval_ctx", "value", "trim_url_limit", "nofollow", "target", "rel"], 4, 1);
const SIG_WORDCOUNT: Sig = sig("do_wordcount", &["s"], 0, 0);
const SIG_WORDWRAP: Sig =
    sig("do_wordwrap", &["environment", "s", "width", "break_long_words", "wrapstring", "break_on_hyphens"], 4, 1);
const SIG_XMLATTR: Sig = sig("do_xmlattr", &["_eval_ctx", "d", "autospace"], 1, 1);
const SIG_TOJSON: Sig = sig("do_tojson", &["eval_ctx", "value", "indent"], 1, 1);

/// `environment.call_filter(name, node, args, kwargs)` for a filter of
/// `DEFAULT_FILTERS` (the compiler rejects other names).
pub(super) fn eval_filter(
    name: &str,
    node: Value,
    args: Vec<Value>,
    kwargs: &[(String, Value)],
    lineno: usize,
) -> R<Value> {
    match name {
        "abs" => {
            one_argument("abs", &args, kwargs, lineno)?;
            py_abs(&node, lineno)
        }
        "attr" => {
            let b = bind(&SIG_ATTR, node, args, kwargs, lineno)?;
            do_attr(b.get(1).unwrap(), b.get(2).unwrap(), lineno)
        }
        "batch" => {
            let b = bind(&SIG_BATCH, node, args, kwargs, lineno)?;
            Ok(do_batch(b, lineno))
        }
        "capitalize" => {
            let b = bind(&SIG_CAPITALIZE, node, args, kwargs, lineno)?;
            let s = soft_str(b.get(0).unwrap(), lineno)?;
            Ok(same_kind(&s, py_capitalize(s.text().unwrap_or_default())))
        }
        "center" => {
            let b = bind(&SIG_CENTER, node, args, kwargs, lineno)?;
            let text = py_str(b.get(0).unwrap(), lineno)?;
            let width = match b.get(1) {
                Some(w) => ssize_arg(w, lineno)?,
                None => 80,
            };
            Ok(str_value(py_center(&text, width)))
        }
        "count" | "length" => {
            one_argument("len", &args, kwargs, lineno)?;
            len_value(&node, lineno)
        }
        "d" | "default" => {
            let mut b = bind(&SIG_DEFAULT, node, args, kwargs, lineno)?;
            let value = b.take(0).unwrap();
            let boolean = b.get(2).is_some_and(Value::truthy);
            if value.is_undefined() || (boolean && !value.truthy()) {
                Ok(b.take(1).unwrap_or_else(|| str_value("")))
            } else {
                Ok(value)
            }
        }
        "dictsort" => {
            let b = bind(&SIG_DICTSORT, node, args, kwargs, lineno)?;
            do_dictsort(&b, lineno)
        }
        "e" | "escape" => {
            one_argument("escape", &args, kwargs, lineno)?;
            Ok(markup_value(escape(&node, lineno)?))
        }
        "filesizeformat" => {
            let b = bind(&SIG_FILESIZEFORMAT, node, args, kwargs, lineno)?;
            do_filesizeformat(b.get(0).unwrap(), b.get(1).is_some_and(Value::truthy), lineno)
        }
        "first" => {
            let b = bind(&SIG_FIRST, node, args, kwargs, lineno)?;
            Ok(py_iter(b.get(1).unwrap(), lineno)?
                .next()?
                .unwrap_or_else(|| undefined_hint("No first item, sequence was empty.")))
        }
        "float" => {
            let mut b = bind(&SIG_FLOAT, node, args, kwargs, lineno)?;
            match float_of(b.get(0).unwrap(), lineno) {
                Ok(f) => Ok(Value::py(PyValue::Float(f))),
                Err(e) if matches!(e.kind(), "TypeError" | "ValueError") => {
                    Ok(b.take(1).unwrap_or(Value::py(PyValue::Float(0.0))))
                }
                Err(e) => Err(e),
            }
        }
        "forceescape" => {
            let b = bind(&SIG_FORCEESCAPE, node, args, kwargs, lineno)?;
            let text = match b.get(0).unwrap() {
                Value::Markup(s) => s.to_string(),
                other => py_str(other, lineno)?,
            };
            Ok(markup_value(escape_text(&text)))
        }
        "format" => {
            let b = bind(&SIG_FORMAT, node, args, kwargs, lineno)?;
            if !b.rest.is_empty() && !b.kw.is_empty() {
                return Err(filter_argument_error(
                    "can't handle positional and keyword arguments at the same time",
                    lineno,
                ));
            }
            let value = soft_str(b.get(0).unwrap(), lineno)?;
            let markup = matches!(value, Value::Markup(_));
            let pct_args = if b.kw.is_empty() { PctArgs::Tuple(b.rest.clone()) } else { PctArgs::Keywords(&b.kw) };
            let out = percent_format_args(value.text().unwrap_or_default(), &pct_args, markup, lineno)?;
            Ok(same_kind(&value, out))
        }
        "groupby" => {
            let b = bind(&SIG_GROUPBY, node, args, kwargs, lineno)?;
            do_groupby(b.get(1).unwrap(), b.get(2).unwrap(), lineno)
        }
        "indent" => {
            let b = bind(&SIG_INDENT, node, args, kwargs, lineno)?;
            do_indent(&b, lineno)
        }
        "int" => {
            let mut b = bind(&SIG_INT, node, args, kwargs, lineno)?;
            do_int(&mut b, lineno)
        }
        "join" => {
            let b = bind(&SIG_JOIN, node, args, kwargs, lineno)?;
            do_join(&b, lineno)
        }
        "last" => {
            let b = bind(&SIG_LAST, node, args, kwargs, lineno)?;
            do_last(b.get(1).unwrap(), lineno)
        }
        "list" => {
            let b = bind(&SIG_LIST, node, args, kwargs, lineno)?;
            Ok(list_value(list_of(b.get(0).unwrap(), lineno)?))
        }
        "lower" | "upper" => {
            let b = bind(if name == "lower" { &SIG_LOWER } else { &SIG_UPPER }, node, args, kwargs, lineno)?;
            let s = soft_str(b.get(0).unwrap(), lineno)?;
            let text = s.text().unwrap_or_default();
            Ok(same_kind(&s, if name == "lower" { text.to_lowercase() } else { text.to_uppercase() }))
        }
        "map" => do_map(node, args, kwargs, lineno),
        "min" | "max" => {
            let b = bind(if name == "min" { &SIG_MIN } else { &SIG_MAX }, node, args, kwargs, lineno)?;
            do_min_max(&b, name == "min", lineno)
        }
        "pprint" => {
            let b = bind(&SIG_PPRINT, node, args, kwargs, lineno)?;
            Ok(str_value(pformat(b.get(0).unwrap())))
        }
        "random" => {
            let b = bind(&SIG_RANDOM, node, args, kwargs, lineno)?;
            do_random(b.get(1).unwrap(), lineno)
        }
        "reject" | "rejectattr" | "select" | "selectattr" => {
            let attr = name.ends_with("attr");
            let select = name.starts_with("select");
            Ok(select_or_reject(node, args, kwargs.to_vec(), select, attr, lineno))
        }
        "replace" => {
            let b = bind(&SIG_REPLACE, node, args, kwargs, lineno)?;
            let s = py_str(b.get(1).unwrap(), lineno)?;
            let old = py_str(b.get(2).unwrap(), lineno)?;
            let new = py_str(b.get(3).unwrap(), lineno)?;
            let count = match b.get(4).filter(|c| !is_none(c)) {
                Some(c) => ssize_arg(c, lineno)?,
                None => -1,
            };
            Ok(str_value(py_replace(&s, &old, &new, count)))
        }
        "reverse" => {
            let b = bind(&SIG_REVERSE, node, args, kwargs, lineno)?;
            do_reverse(b.get(0).unwrap(), lineno)
        }
        "round" => {
            let b = bind(&SIG_ROUND, node, args, kwargs, lineno)?;
            do_round(&b, lineno)
        }
        "safe" => {
            let b = bind(&SIG_SAFE, node, args, kwargs, lineno)?;
            Ok(match b.get(0).unwrap() {
                Value::Markup(s) => Value::Markup(s.clone()),
                other => markup_value(py_str(other, lineno)?),
            })
        }
        "slice" => {
            let b = bind(&SIG_SLICE, node, args, kwargs, lineno)?;
            Ok(do_slice(b, lineno))
        }
        "sort" => {
            let b = bind(&SIG_SORT, node, args, kwargs, lineno)?;
            do_sort(&b, lineno)
        }
        "string" => {
            one_argument("soft_unicode", &args, kwargs, lineno)?;
            soft_str(&node, lineno)
        }
        "striptags" => {
            let b = bind(&SIG_STRIPTAGS, node, args, kwargs, lineno)?;
            let text = match b.get(0).unwrap() {
                Value::Markup(s) => s.to_string(),
                other => py_str(other, lineno)?,
            };
            Ok(str_value(striptags(&text, lineno)?))
        }
        "sum" => {
            let b = bind(&SIG_SUM, node, args, kwargs, lineno)?;
            do_sum(&b, lineno)
        }
        "title" => {
            let b = bind(&SIG_TITLE, node, args, kwargs, lineno)?;
            let s = soft_str(b.get(0).unwrap(), lineno)?;
            Ok(str_value(jinja_title(s.text().unwrap_or_default())))
        }
        "trim" => {
            let b = bind(&SIG_TRIM, node, args, kwargs, lineno)?;
            let s = soft_str(b.get(0).unwrap(), lineno)?;
            let chars = match (&s, b.get(1)) {
                (Value::Markup(_), Some(c)) if is_str(c) => Some(Value::Markup(escape(c, lineno)?.into())),
                (_, c) => c.cloned(),
            };
            Ok(same_kind(&s, py_strip(s.text().unwrap_or_default(), chars.as_ref(), "strip", lineno)?))
        }
        "truncate" => {
            let b = bind(&SIG_TRUNCATE, node, args, kwargs, lineno)?;
            do_truncate(&b, lineno)
        }
        "unique" => {
            let b = bind(&SIG_UNIQUE, node, args, kwargs, lineno)?;
            Ok(do_unique(b, lineno))
        }
        "urlencode" => {
            let b = bind(&SIG_URLENCODE, node, args, kwargs, lineno)?;
            do_urlencode(b.get(0).unwrap(), lineno)
        }
        "urlize" => {
            let b = bind(&SIG_URLIZE, node, args, kwargs, lineno)?;
            do_urlize(&b, lineno)
        }
        "wordcount" => {
            let b = bind(&SIG_WORDCOUNT, node, args, kwargs, lineno)?;
            let s = soft_str(b.get(0).unwrap(), lineno)?;
            Ok(int_value(word_count(s.text().unwrap_or_default())))
        }
        "wordwrap" => {
            let b = bind(&SIG_WORDWRAP, node, args, kwargs, lineno)?;
            do_wordwrap(&b, lineno)
        }
        "xmlattr" => {
            let b = bind(&SIG_XMLATTR, node, args, kwargs, lineno)?;
            do_xmlattr(&b, lineno)
        }
        "tojson" => {
            let b = bind(&SIG_TOJSON, node, args, kwargs, lineno)?;
            do_tojson(b.get(1).unwrap(), b.get(2), lineno)
        }
        _ => Err(error("TemplateRuntimeError", format!("no filter named {}", pytext::str_repr(name)), lineno)),
    }
}

/// `abs(v)`.
fn py_abs(v: &Value, lineno: usize) -> R<Value> {
    match v {
        Value::Py(PyValue::Int(n)) => {
            Ok(Value::py(PyValue::Int(if n.sign() < 0 { PyInt::new(-n.0.clone()) } else { n.clone() })))
        }
        Value::Py(PyValue::Bool(b)) => Ok(int_value(i64::from(*b))),
        Value::Py(PyValue::Float(f)) => Ok(Value::py(PyValue::Float(f.abs()))),
        other => Err(type_error(format!("bad operand type for abs(): '{}'", other.type_name()), lineno)),
    }
}

fn do_attr(obj: &Value, name: &Value, lineno: usize) -> R<Value> {
    let name = py_str(name, lineno)?;
    let dunder = name.starts_with("__");
    let found = match obj {
        Value::Undefined(_) if !dunder => return Err(obj.fail_undefined(lineno)),
        Value::Undefined(_) | Value::Func(_) => None,
        Value::Py(_) => py_attribute(obj, &name, lineno),
        Value::Status(_) => ["printer", "eventtime", "cache"]
            .contains(&name.as_str())
            .then(|| Err(unsupported(format!("attribute '{name}' of 'GetStatusWrapper' object"), lineno))),
        _ => object_attribute(obj, &name, lineno),
    };
    match found {
        Some(result) => result,
        None => Ok(Value::undefined_member(obj, &str_value(name))),
    }
}

/// `do_batch`: a generator of lists of `linecount` items.
fn do_batch(mut b: Bound, lineno: usize) -> Value {
    let value = b.take(0).unwrap();
    let linecount = b.take(1).unwrap();
    let fill_with = b.take(2).filter(|f| !is_none(f));
    let mut source: Option<Items> = None;
    let mut tmp: Vec<Value> = Vec::new();
    let mut done = false;
    LazyIter::generator(
        "do_batch",
        Box::new(move || {
            if done {
                return Ok(None);
            }
            if source.is_none() {
                source = Some(py_iter(&value, lineno)?);
            }
            let it = source.as_mut().unwrap();
            while let Some(item) = it.next()? {
                if v_eq(&int_value(tmp.len() as i64), &linecount) {
                    let full = std::mem::take(&mut tmp);
                    tmp.push(item);
                    return Ok(Some(list_value(full)));
                }
                tmp.push(item);
            }
            done = true;
            if tmp.is_empty() {
                return Ok(None);
            }
            let mut last = std::mem::take(&mut tmp);
            if let Some(fill) = &fill_with {
                if v_cmp("<", &int_value(last.len() as i64), &linecount, lineno)? {
                    let missing = eval_sub(&linecount, &int_value(last.len() as i64), lineno)?;
                    match object_binop("*", &Value::Seq(SeqKind::List, Rc::new(vec![fill.clone()])), &missing, lineno)? {
                        Value::Py(PyValue::List(extra)) => last.extend(extra.into_iter().map(Value::py)),
                        Value::Seq(_, extra) => last.extend(extra.iter().cloned()),
                        _ => {}
                    }
                }
            }
            Ok(Some(list_value(last)))
        }),
    )
}

fn do_dictsort(b: &Bound, lineno: usize) -> R<Value> {
    let by = b.get(2).cloned().unwrap_or_else(|| str_value("key"));
    let pos = if v_eq(&by, &str_value("key")) {
        0
    } else if v_eq(&by, &str_value("value")) {
        1
    } else {
        return Err(filter_argument_error("You can only sort by either \"key\" or \"value\"", lineno));
    };
    let value = b.get(0).unwrap();
    let Value::Py(PyValue::Dict(d)) = value else {
        return Err(attribute_error(value, "items", lineno));
    };
    let reverse = match b.get(3) {
        Some(r) => c_int_arg(r, lineno)? != 0,
        None => false,
    };
    let case_sensitive = b.get(1).is_some_and(Value::truthy);
    let pairs: Vec<Value> =
        d.iter().map(|(k, v)| Value::py(PyValue::Tuple(vec![k.clone(), v.clone()]))).collect();
    let keys: Vec<Value> = d
        .iter()
        .map(|(k, v)| {
            let key = Value::py(if pos == 0 { k.clone() } else { v.clone() });
            if case_sensitive {
                key
            } else {
                ignore_case(key)
            }
        })
        .collect();
    Ok(list_value(sort_by_keys(&keys, pairs, reverse, python_lt(lineno))?))
}

fn do_filesizeformat(value: &Value, binary: bool, lineno: usize) -> R<Value> {
    let bytes = float_of(value, lineno)?;
    let base: u32 = if binary { 1024 } else { 1000 };
    let prefixes: [&str; 8] = if binary {
        ["KiB", "MiB", "GiB", "TiB", "PiB", "EiB", "ZiB", "YiB"]
    } else {
        ["kB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"]
    };
    let pct = |fmt: &str, args: &[PctValue]| -> R<String> {
        pytext::percent_format(fmt, args).map_err(|e| from_pyerr(&e, lineno))
    };
    if bytes == 1.0 {
        return Ok(str_value("1 Byte"));
    }
    if bytes < f64::from(base) {
        return Ok(str_value(pct("%d Bytes", &[PctValue::Float(bytes)])?));
    }
    let mut unit = PyInt::from_i64(i64::from(base) * i64::from(base));
    let mut prefix = prefixes[0];
    for (i, p) in prefixes.iter().enumerate() {
        if i > 0 {
            unit = PyInt::new(unit.0.clone() * base);
        }
        prefix = p;
        if !bytes.is_nan() && pytext::cmp_bigint_float(unit.inner(), bytes) == Ordering::Greater {
            break;
        }
    }
    let unit_f = unit.to_f64_exact().unwrap_or_else(|| unit.inner().to_string().parse().unwrap_or(f64::INFINITY));
    let scaled = f64::from(base) * bytes / unit_f;
    Ok(str_value(pct("%.1f %s", &[PctValue::Float(scaled), PctValue::Str(prefix.to_string())])?))
}

fn do_groupby(value: &Value, attribute: &Value, lineno: usize) -> R<Value> {
    let getter = AttrGetter::new(Some(attribute), false, None, lineno)?;
    let items = list_of(value, lineno)?;
    let keys = items.iter().map(|item| getter.get(item, lineno)).collect::<R<Vec<Value>>>()?;
    let sorted = sort_by_keys(&keys, items, false, python_lt(lineno))?;
    let mut groups: Vec<Value> = Vec::new();
    let mut current: Option<(Value, Vec<Value>)> = None;
    for item in sorted {
        let key = getter.get(&item, lineno)?;
        match &mut current {
            Some((group_key, members)) if v_eq(group_key, &key) => members.push(item),
            _ => {
                if let Some((group_key, members)) = current.take() {
                    groups.push(Value::Seq(SeqKind::Group, Rc::new(vec![group_key, list_value(members)])));
                }
                current = Some((key, vec![item]));
            }
        }
    }
    if let Some((group_key, members)) = current {
        groups.push(Value::Seq(SeqKind::Group, Rc::new(vec![group_key, list_value(members)])));
    }
    Ok(list_value(groups))
}

fn do_indent(b: &Bound, lineno: usize) -> R<Value> {
    let s = b.get(0).unwrap();
    let width = b.get(1).cloned().unwrap_or_else(|| int_value(4));
    let first = match b.get(4).filter(|v| !is_none(v)) {
        Some(indentfirst) => indentfirst.truthy(),
        None => b.get(2).is_some_and(Value::truthy),
    };
    let blank = b.get(3).is_some_and(Value::truthy);
    let indention = match &width {
        Value::Py(PyValue::Int(_) | PyValue::Bool(_)) => " ".repeat(usize::try_from(index_value(&width, lineno)?).unwrap_or(0)),
        other => {
            return Err(type_error(
                format!("can't multiply sequence by non-int of type '{}'", other.type_name()),
                lineno,
            ))
        }
    };
    let text = match s {
        Value::Py(PyValue::Str(t)) => t.clone(),
        Value::Markup(t) => t.to_string(),
        Value::Undefined(_) => return Err(s.fail_undefined(lineno)),
        other => {
            return Err(type_error(
                format!("unsupported operand type(s) for +=: '{}' and 'str'", other.type_name()),
                lineno,
            ))
        }
    };
    let text = format!("{text}\n");
    let lines: Vec<&str> = splitlines_keep(&text).into_iter().map(|(line, _)| line).collect();
    let mut rv = if blank {
        lines.join(&format!("\n{indention}"))
    } else {
        let mut rv = lines[0].to_string();
        if lines.len() > 1 {
            rv.push('\n');
            let rest: Vec<String> = lines[1..]
                .iter()
                .map(|line| if line.is_empty() { String::new() } else { format!("{indention}{line}") })
                .collect();
            rv.push_str(&rest.join("\n"));
        }
        rv
    };
    if first {
        rv = format!("{indention}{rv}");
    }
    Ok(same_kind(s, rv))
}

fn do_int(b: &mut Bound, lineno: usize) -> R<Value> {
    let value = b.get(0).unwrap().clone();
    let first = if is_str(&value) {
        match b.get(2) {
            None => pytext::int_from_string(value.text().unwrap_or_default(), 10).map_err(|e| from_pyerr(&e, lineno)),
            Some(base) => match base {
                Value::Py(PyValue::Int(_) | PyValue::Bool(_)) => {
                    let base = index_value(base, lineno).unwrap_or(i64::MAX);
                    pytext::int_from_string(value.text().unwrap_or_default(), base).map_err(|e| from_pyerr(&e, lineno))
                }
                other => Err(type_error(
                    format!("'{}' object cannot be interpreted as an integer", other.type_name()),
                    lineno,
                )),
            },
        }
    } else {
        int_of(&value, lineno)
    };
    match first {
        Ok(n) => return Ok(Value::py(PyValue::Int(n))),
        Err(e) if matches!(e.kind(), "TypeError" | "ValueError") => {}
        Err(e) => return Err(e),
    }
    let second = float_of(&value, lineno).and_then(|f| float_to_int(f, lineno));
    match second {
        Ok(n) => Ok(Value::py(PyValue::Int(n))),
        Err(e) if matches!(e.kind(), "TypeError" | "ValueError") => Ok(b.take(1).unwrap_or_else(|| int_value(0))),
        Err(e) => Err(e),
    }
}

fn do_join(b: &Bound, lineno: usize) -> R<Value> {
    let separator = match b.get(2) {
        Some(d) => py_str(d, lineno)?,
        None => String::new(),
    };
    let getter = match b.get(3).filter(|a| !is_none(a)) {
        Some(attr) => Some(AttrGetter::new(Some(attr), false, None, lineno)?),
        None => None,
    };
    let mut items = py_iter(b.get(1).unwrap(), lineno)?;
    let mut parts = Vec::new();
    while let Some(item) = items.next()? {
        let item = match &getter {
            Some(g) => g.get(&item, lineno)?,
            None => item,
        };
        parts.push(py_str(&item, lineno)?);
    }
    Ok(str_value(parts.join(&separator)))
}

fn do_last(seq: &Value, lineno: usize) -> R<Value> {
    let last = match seq {
        Value::Py(PyValue::Str(s)) => s.chars().last().map(|c| str_value(c.to_string())),
        Value::Markup(s) => s.chars().last().map(|c| markup_value(c.to_string())),
        Value::Undefined(_) => None,
        Value::Py(PyValue::List(l) | PyValue::Tuple(l)) => l.last().cloned().map(Value::py),
        Value::Py(PyValue::Dict(d)) => d.keys().last().cloned().map(Value::py),
        Value::Py(PyValue::Bytes(b)) => b.last().map(|x| int_value(i64::from(*x))),
        Value::Seq(_, items) => items.last().cloned(),
        other => {
            return Err(type_error(format!("'{}' object is not reversible", other.type_name()), lineno))
        }
    };
    Ok(last.unwrap_or_else(|| undefined_hint("No last item, sequence was empty.")))
}

/// `do_map`: a generator; its arguments are only checked when it starts,
/// and the filter name is looked up for each item (`call_filter`).
fn do_map(node: Value, args: Vec<Value>, kwargs: &[(String, Value)], lineno: usize) -> R<Value> {
    let kwargs = kwargs.to_vec();
    let mut state: Option<(Items, MapFunc)> = None;
    let mut done = false;
    Ok(LazyIter::generator(
        "do_map",
        Box::new(move || {
            if done {
                return Ok(None);
            }
            if state.is_none() {
                let func = if args.is_empty() && kwargs.iter().any(|(k, _)| k == "attribute") {
                    let mut kw = kwargs.clone();
                    let attribute = take_kw(&mut kw, "attribute").unwrap();
                    let default = take_kw(&mut kw, "default");
                    if let Some((unexpected, _)) = kw.first() {
                        return Err(filter_argument_error(
                            format!("Unexpected keyword argument {}", pytext::str_repr(unexpected)),
                            lineno,
                        ));
                    }
                    MapFunc::Attr(AttrGetter::new(Some(&attribute), false, default.filter(|d| !is_none(d)), lineno)?)
                } else {
                    let Some(name) = args.first() else {
                        return Err(filter_argument_error("map requires a filter argument", lineno));
                    };
                    MapFunc::Filter(name.clone(), args[1..].to_vec(), kwargs.clone())
                };
                if !node.truthy() {
                    done = true;
                    return Ok(None);
                }
                state = Some((py_iter(&node, lineno)?, func));
            }
            let (items, func) = state.as_mut().unwrap();
            match items.next()? {
                Some(item) => Ok(Some(match func {
                    MapFunc::Attr(getter) => getter.get(&item, lineno)?,
                    MapFunc::Filter(name, args, kwargs) => {
                        eval_filter(&filter_name(name, lineno)?, item, args.clone(), kwargs, lineno)?
                    }
                })),
                None => {
                    done = true;
                    Ok(None)
                }
            }
        }),
    ))
}

enum MapFunc {
    Attr(AttrGetter),
    Filter(Value, Vec<Value>, Vec<(String, Value)>),
}

fn take_kw(kw: &mut Vec<(String, Value)>, name: &str) -> Option<Value> {
    let i = kw.iter().position(|(k, _)| k == name)?;
    Some(kw.remove(i).1)
}

/// The filter a `map` argument names (`environment.filters.get(name)`).
fn filter_name(name: &Value, lineno: usize) -> R<String> {
    callable_name(name, JINJA_FILTERS, "filter", lineno)
}

fn callable_name(name: &Value, table: &[&str], what: &str, lineno: usize) -> R<String> {
    check_hashable(name, lineno)?;
    if let Some(text) = name.text().filter(|t| table.contains(t)) {
        return Ok(text.to_string());
    }
    let mut message = format!("no {what} named {}", name.repr());
    if name.is_undefined() {
        message = format!(
            "{message} ({}; did you forget to quote the callable name?)",
            name.undefined_message()
        );
    }
    Err(error("TemplateRuntimeError", message, lineno))
}

fn do_min_max(b: &Bound, is_min: bool, lineno: usize) -> R<Value> {
    let mut items = py_iter(b.get(1).unwrap(), lineno)?;
    let Some(first) = items.next()? else {
        return Ok(undefined_hint("No aggregated item, sequence was empty."));
    };
    let case_sensitive = b.get(2).is_some_and(Value::truthy);
    let getter = AttrGetter::new(b.get(3), !case_sensitive, None, lineno)?;
    let op = if is_min { "<" } else { ">" };
    let mut best_key = getter.get(&first, lineno)?;
    let mut best = first;
    while let Some(item) = items.next()? {
        let key = getter.get(&item, lineno)?;
        if v_cmp(op, &key, &best_key, lineno)? {
            best_key = key;
            best = item;
        }
    }
    Ok(best)
}

fn do_random(seq: &Value, lineno: usize) -> R<Value> {
    let len = len_of(seq, lineno)?;
    if len == 0 {
        return Ok(undefined_hint("No random item, sequence was empty."));
    }
    let index = random_below(len);
    match seq {
        Value::Py(PyValue::Dict(d)) => match dict_lookup(d, &PyValue::Int(PyInt::from_i64(index as i64))) {
            Some(v) => Ok(Value::py(v.clone())),
            None => Err(error("KeyError", index.to_string(), lineno)),
        },
        other => object_index(other, index, lineno),
    }
}

/// `seq[index]` for a valid index of a sequence.
fn object_index(seq: &Value, index: usize, lineno: usize) -> R<Value> {
    match seq {
        Value::Py(PyValue::Str(s)) => Ok(str_value(s.chars().nth(index).unwrap_or_default().to_string())),
        Value::Markup(s) => Ok(markup_value(s.chars().nth(index).unwrap_or_default().to_string())),
        Value::Py(PyValue::List(l) | PyValue::Tuple(l)) => Ok(Value::py(l[index].clone())),
        Value::Py(PyValue::Bytes(b)) => Ok(int_value(i64::from(b[index]))),
        Value::Seq(_, items) => Ok(items[index].clone()),
        other => Err(type_error(format!("'{}' object is not subscriptable", other.type_name()), lineno)),
    }
}

/// A uniform index below `n` (Python's `random.choice` draws from its own
/// state); seeded from the process-random `RandomState` keys.
fn random_below(n: usize) -> usize {
    use std::hash::{BuildHasher, Hasher};
    thread_local! {
        static STATE: Cell<u64> =
            Cell::new(std::collections::hash_map::RandomState::new().build_hasher().finish() | 1);
    }
    STATE.with(|state| {
        let mut x = state.get();
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        state.set(x);
        (x % n as u64) as usize
    })
}

/// The started state of a `select_or_reject` generator.
struct SelectState {
    items: Items,
    getter: Option<AttrGetter>,
    test: Option<Value>,
    test_args: Vec<Value>,
}

/// `select`/`reject`/`selectattr`/`rejectattr`: `select_or_reject`
/// generators; the test name is looked up for each item (`call_test`).
fn select_or_reject(
    node: Value,
    args: Vec<Value>,
    kwargs: Vec<(String, Value)>,
    select: bool,
    attr: bool,
    lineno: usize,
) -> Value {
    let mut state: Option<SelectState> = None;
    let mut done = false;
    LazyIter::generator(
        "select_or_reject",
        Box::new(move || {
            if done {
                return Ok(None);
            }
            if state.is_none() {
                let (getter, offset) = if attr {
                    let Some(attribute) = args.first() else {
                        return Err(filter_argument_error("Missing parameter for attribute name", lineno));
                    };
                    (Some(AttrGetter::new(Some(attribute), false, None, lineno)?), 1)
                } else {
                    (None, 0)
                };
                let (test, test_args) = match args.get(offset) {
                    Some(name) => (Some(name.clone()), args[offset + 1..].to_vec()),
                    None => (None, Vec::new()),
                };
                if !node.truthy() {
                    done = true;
                    return Ok(None);
                }
                state = Some(SelectState { items: py_iter(&node, lineno)?, getter, test, test_args });
            }
            let SelectState { items, getter, test, test_args } = state.as_mut().unwrap();
            while let Some(item) = items.next()? {
                let subject = match getter {
                    Some(g) => g.get(&item, lineno)?,
                    None => item.clone(),
                };
                let passed = match test {
                    Some(name) => {
                        let name = callable_name(name, JINJA_TESTS, "test", lineno)?;
                        eval_test(&name, subject, test_args.clone(), &kwargs, lineno)?.truthy()
                    }
                    None => subject.truthy(),
                };
                if passed == select {
                    return Ok(Some(item));
                }
            }
            done = true;
            Ok(None)
        }),
    )
}

fn do_reverse(value: &Value, lineno: usize) -> R<Value> {
    let reversed = |type_name: &'static str, items: Vec<Value>| {
        let mut it = items.into_iter().rev();
        LazyIter::builtin(type_name, Box::new(move || Ok(it.next())))
    };
    match value {
        Value::Py(PyValue::Str(s)) => Ok(str_value(s.chars().rev().collect::<String>())),
        Value::Markup(s) => Ok(markup_value(s.chars().rev().collect::<String>())),
        Value::Py(PyValue::List(_)) | Value::Seq(SeqKind::List, _) => {
            Ok(reversed("list_reverseiterator", iterate(value, lineno)?))
        }
        Value::Py(PyValue::Tuple(_) | PyValue::Bytes(_)) | Value::Seq(..) | Value::Undefined(_) => {
            Ok(reversed("reversed", iterate(value, lineno)?))
        }
        Value::Py(PyValue::Dict(_)) => Ok(reversed("dict_reversekeyiterator", iterate(value, lineno)?)),
        Value::Iter(_) | Value::Status(_) => {
            let mut items = list_of(value, lineno)?;
            items.reverse();
            Ok(list_value(items))
        }
        _ => Err(filter_argument_error("argument must be iterable", lineno)),
    }
}

fn do_round(b: &Bound, lineno: usize) -> R<Value> {
    let value = b.get(0).unwrap();
    let method = b.get(2).cloned().unwrap_or_else(|| str_value("common"));
    check_hashable(&method, lineno)?;
    let method = match method.text() {
        Some(m @ ("common" | "ceil" | "floor")) => m.to_string(),
        _ => return Err(filter_argument_error("method must be common, ceil or floor", lineno)),
    };
    let precision = b.get(1).cloned().unwrap_or_else(|| int_value(0));
    if method == "common" {
        return py_round(value, &precision, lineno);
    }
    let scale = eval_pow(&int_value(10), &precision, lineno)?;
    let scaled = eval_mul(value, &scale, lineno)?;
    let rounded = match &scaled {
        Value::Py(PyValue::Int(n)) => n.clone(),
        Value::Py(PyValue::Bool(v)) => PyInt::from_i64(i64::from(*v)),
        Value::Py(PyValue::Float(f)) => {
            float_to_int(if method == "ceil" { f.ceil() } else { f.floor() }, lineno)?
        }
        other => return Err(type_error(format!("must be real number, not {}", other.type_name()), lineno)),
    };
    eval_div(&Value::py(PyValue::Int(rounded)), &scale, lineno)
}

/// `round(value, ndigits)`.
fn py_round(value: &Value, ndigits: &Value, lineno: usize) -> R<Value> {
    let (is_float, x) = match value {
        Value::Py(PyValue::Float(f)) => (true, *f),
        Value::Py(PyValue::Int(_) | PyValue::Bool(_)) => (false, 0.0),
        other => {
            return Err(type_error(format!("type {} doesn't define __round__ method", other.type_name()), lineno))
        }
    };
    if is_none(ndigits) {
        return if is_float {
            pytext::round_py(x).map(|n| Value::py(PyValue::Int(n))).map_err(|e| from_pyerr(&e, lineno))
        } else {
            Ok(Value::py(PyValue::Int(int_of(value, lineno)?)))
        };
    }
    if is_float {
        let n = match ndigits {
            Value::Py(PyValue::Int(n)) => pyint_i64(n).unwrap_or(if n.sign() < 0 { i64::MIN } else { i64::MAX }),
            Value::Py(PyValue::Bool(b)) => i64::from(*b),
            other => {
                return Err(type_error(
                    format!("'{}' object cannot be interpreted as an integer", other.type_name()),
                    lineno,
                ))
            }
        };
        return Ok(Value::py(PyValue::Float(pytext::round_py_nd(x, n))));
    }
    let n = match ndigits {
        Value::Py(PyValue::Int(n)) => n.clone(),
        Value::Py(PyValue::Bool(b)) => PyInt::from_i64(i64::from(*b)),
        other => {
            return Err(type_error(
                format!("'{}' object cannot be interpreted as an integer", other.type_name()),
                lineno,
            ))
        }
    };
    let x = int_of(value, lineno)?;
    if n.sign() >= 0 {
        return Ok(Value::py(PyValue::Int(x)));
    }
    let digits = pyint_i64(&PyInt::new(-n.0.clone())).unwrap_or(i64::MAX);
    if digits > 4300 {
        return Ok(int_value(0));
    }
    let pow = int_pow(&PyInt::from_i64(10), digits as u64);
    let (q, r) = int_divmod(&x, &pow);
    let twice = PyInt::new(r.0.clone() * 2);
    let up = match pycompat::int::int_cmp(&twice, &pow) {
        Ordering::Greater => true,
        Ordering::Equal => pycompat::int::int_is_odd(&q),
        Ordering::Less => false,
    };
    let q = if up { PyInt::new(q.0 + 1) } else { q };
    Ok(Value::py(PyValue::Int(PyInt::new(q.0 * pow.0))))
}

/// `do_slice`: a generator of `slices` lists.
fn do_slice(mut b: Bound, lineno: usize) -> Value {
    let value = b.take(0).unwrap();
    let slices = b.take(1).unwrap();
    let fill_with = b.take(2).filter(|f| !is_none(f));
    let mut pending: Option<std::vec::IntoIter<Value>> = None;
    let mut started = false;
    LazyIter::generator(
        "do_slice",
        Box::new(move || {
            if !started {
                started = true;
                let seq = list_of(&value, lineno)?;
                let length = int_value(seq.len() as i64);
                let per = eval_floordiv(&length, &slices, lineno)?;
                let extra = eval_mod(&length, &slices, lineno)?;
                let (Value::Py(PyValue::Int(per)), Value::Py(PyValue::Int(extra))) = (&per, &extra) else {
                    return Err(type_error(
                        format!("'{}' object cannot be interpreted as an integer", slices.type_name()),
                        lineno,
                    ));
                };
                let count = index_value(&slices, lineno)?;
                let per = pyint_i64(per).unwrap_or(0);
                let extra = pyint_i64(extra).unwrap_or(0);
                let mut out = Vec::new();
                let mut offset = 0;
                for slice_number in 0..count.max(0) {
                    let start = offset + slice_number * per;
                    if slice_number < extra {
                        offset += 1;
                    }
                    let end = offset + (slice_number + 1) * per;
                    let clamp = |i: i64| i.clamp(0, seq.len() as i64) as usize;
                    let (s, e) = (clamp(start), clamp(end));
                    let mut tmp: Vec<Value> = if s < e { seq[s..e].to_vec() } else { Vec::new() };
                    if let Some(fill) = &fill_with {
                        if slice_number >= extra {
                            tmp.push(fill.clone());
                        }
                    }
                    out.push(list_value(tmp));
                }
                pending = Some(out.into_iter());
            }
            Ok(pending.as_mut().and_then(Iterator::next))
        }),
    )
}

fn do_sort(b: &Bound, lineno: usize) -> R<Value> {
    let case_sensitive = b.get(3).is_some_and(Value::truthy);
    let attribute = b.get(4).cloned().unwrap_or_else(none_value);
    let getters: Vec<AttrGetter> = match attribute.text() {
        Some(text) => text
            .split(',')
            .map(|part| AttrGetter::new(Some(&str_value(part)), !case_sensitive, None, lineno))
            .collect::<R<_>>()?,
        None => vec![AttrGetter::new(Some(&attribute), !case_sensitive, None, lineno)?],
    };
    let items = list_of(b.get(1).unwrap(), lineno)?;
    let reverse = match b.get(2) {
        Some(r) => c_int_arg(r, lineno)? != 0,
        None => false,
    };
    let keys = items
        .iter()
        .map(|item| {
            let parts = getters.iter().map(|g| g.get(item, lineno)).collect::<R<Vec<Value>>>()?;
            Ok(Value::Seq(SeqKind::List, Rc::new(parts)))
        })
        .collect::<R<Vec<Value>>>()?;
    Ok(list_value(sort_by_keys(&keys, items, reverse, python_lt(lineno))?))
}

fn do_sum(b: &Bound, lineno: usize) -> R<Value> {
    let getter = match b.get(2).filter(|a| !is_none(a)) {
        Some(attr) => Some(AttrGetter::new(Some(attr), false, None, lineno)?),
        None => None,
    };
    let mut items = py_iter(b.get(1).unwrap(), lineno)?;
    let mut total = b.get(3).cloned().unwrap_or_else(|| int_value(0));
    if is_str(&total) {
        return Err(type_error("sum() can't sum strings [use ''.join(seq) instead]", lineno));
    }
    if let Value::Py(PyValue::Bytes(_)) = total {
        return Err(type_error("sum() can't sum bytes [use b''.join(seq) instead]", lineno));
    }
    while let Some(item) = items.next()? {
        let item = match &getter {
            Some(g) => g.get(&item, lineno)?,
            None => item,
        };
        total = eval_add(&total, &item, lineno)?;
    }
    Ok(total)
}

/// `do_title`.
fn jinja_title(s: &str) -> String {
    let is_sep = |c: char| c == '-' || c == '(' || c == '{' || c == '[' || c == '<' || is_py_space(c);
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while !rest.is_empty() {
        let sep_len: usize = rest.chars().take_while(|c| is_sep(*c)).map(char::len_utf8).sum();
        let (item, tail) = if sep_len > 0 {
            rest.split_at(sep_len)
        } else {
            let word_len: usize = rest.chars().take_while(|c| !is_sep(*c)).map(char::len_utf8).sum();
            rest.split_at(word_len)
        };
        let mut chars = item.chars();
        if let Some(first) = chars.next() {
            out.extend(first.to_uppercase());
            out.push_str(&chars.as_str().to_lowercase());
        }
        rest = tail;
    }
    out
}

fn do_truncate(b: &Bound, lineno: usize) -> R<Value> {
    let s = b.get(1).unwrap();
    let length = b.get(2).cloned().unwrap_or_else(|| int_value(255));
    let killwords = b.get(3).is_some_and(Value::truthy);
    let end = b.get(4).cloned().unwrap_or_else(|| str_value("..."));
    let leeway = match b.get(5).filter(|l| !is_none(l)) {
        Some(l) => l.clone(),
        None => int_value(5),
    };
    let end_len = int_value(len_of(&end, lineno)? as i64);
    if !v_cmp(">=", &length, &end_len, lineno)? {
        return Err(error(
            "AssertionError",
            format!("expected length >= {}, got {}", py_str(&end_len, lineno)?, py_str(&length, lineno)?),
            lineno,
        ));
    }
    if !v_cmp(">=", &leeway, &int_value(0), lineno)? {
        return Err(error("AssertionError", format!("expected leeway >= 0, got {}", py_str(&leeway, lineno)?), lineno));
    }
    let limit = eval_add(&length, &leeway, lineno)?;
    if v_cmp("<=", &int_value(len_of(s, lineno)? as i64), &limit, lineno)? {
        return Ok(s.clone());
    }
    let keep = eval_sub(&length, &end_len, lineno)?;
    let Value::Py(PyValue::Int(_) | PyValue::Bool(_)) = keep else {
        return Err(type_error("slice indices must be integers or None or have an __index__ method", lineno));
    };
    let keep = index_value(&keep, lineno)?;
    let Some(text) = s.text() else {
        return match s {
            Value::Py(PyValue::List(_) | PyValue::Tuple(_)) | Value::Seq(..) if killwords => Err(type_error(
                format!(
                    "can only concatenate {t} (not \"{}\") to {t}",
                    end.type_name(),
                    t = if matches!(s, Value::Py(PyValue::List(_)) | Value::Seq(SeqKind::List, _)) { "list" } else { "tuple" }
                ),
                lineno,
            )),
            Value::Py(PyValue::List(_) | PyValue::Tuple(_)) | Value::Seq(..) => Err(attribute_error(s, "rsplit", lineno)),
            Value::Py(PyValue::Dict(_)) => Err(type_error("unhashable type: 'slice'", lineno)),
            _ => Err(unsupported(format!("truncating a '{}' value", s.type_name()), lineno)),
        };
    };
    let chars: Vec<char> = text.chars().collect();
    let cut = if keep < 0 { (chars.len() as i64 + keep).max(0) } else { keep.min(chars.len() as i64) } as usize;
    let head: String = chars[..cut].iter().collect();
    let head = if killwords {
        head
    } else {
        match head.rfind(' ') {
            Some(i) => head[..i].to_string(),
            None => head,
        }
    };
    let head = same_kind(s, head);
    eval_add(&head, &end, lineno)
}

/// `do_unique`: a generator of the first item of each key.
fn do_unique(mut b: Bound, lineno: usize) -> Value {
    let value = b.take(1).unwrap();
    let case_sensitive = b.get(2).is_some_and(Value::truthy);
    let attribute = b.take(3);
    let mut state: Option<(Items, AttrGetter)> = None;
    let mut seen: Vec<Value> = Vec::new();
    LazyIter::generator(
        "do_unique",
        Box::new(move || {
            if state.is_none() {
                let getter = AttrGetter::new(attribute.as_ref(), !case_sensitive, None, lineno)?;
                state = Some((py_iter(&value, lineno)?, getter));
            }
            let (items, getter) = state.as_mut().unwrap();
            while let Some(item) = items.next()? {
                let key = getter.get(&item, lineno)?;
                check_hashable(&key, lineno)?;
                if !seen.iter().any(|s| v_eq(s, &key)) {
                    seen.push(key);
                    return Ok(Some(item));
                }
            }
            Ok(None)
        }),
    )
}

/// `unicode_urlencode(obj, for_qs)`.
fn url_quote(v: &Value, for_qs: bool, lineno: usize) -> R<String> {
    let text = match v.text() {
        Some(t) => t.to_string(),
        None => py_str(v, lineno)?,
    };
    let mut out = String::new();
    for &byte in text.as_bytes() {
        let c = byte as char;
        if c.is_ascii_alphanumeric() || "_.-~".contains(c) || (!for_qs && c == '/') {
            out.push(c);
        } else {
            out.push_str(&format!("%{byte:02X}"));
        }
    }
    Ok(if for_qs { out.replace("%20", "+") } else { out })
}

fn do_urlencode(value: &Value, lineno: usize) -> R<Value> {
    let iterable = !matches!(value, Value::Py(PyValue::None | PyValue::Int(_) | PyValue::Float(_) | PyValue::Bool(_)) | Value::Func(_));
    if is_str(value) || !iterable {
        return Ok(str_value(url_quote(value, false, lineno)?));
    }
    let pairs: Vec<(Value, Value)> = match value {
        Value::Py(PyValue::Dict(d)) => d.iter().map(|(k, v)| (Value::py(k.clone()), Value::py(v.clone()))).collect(),
        other => list_of(other, lineno)?
            .into_iter()
            .map(|item| unpack_pair(&item, lineno))
            .collect::<R<Vec<_>>>()?,
    };
    let mut parts = Vec::with_capacity(pairs.len());
    for (k, v) in &pairs {
        parts.push(format!("{}={}", url_quote(k, true, lineno)?, url_quote(v, true, lineno)?));
    }
    Ok(str_value(parts.join("&")))
}

/// `k, v = item`.
fn unpack_pair(item: &Value, lineno: usize) -> R<(Value, Value)> {
    let parts = match item {
        Value::Py(PyValue::None | PyValue::Int(_) | PyValue::Float(_) | PyValue::Bool(_)) | Value::Func(_) => {
            return Err(type_error(format!("cannot unpack non-iterable {} object", item.type_name()), lineno))
        }
        other => list_of(other, lineno)?,
    };
    match parts.len() {
        2 => {
            let mut it = parts.into_iter();
            Ok((it.next().unwrap(), it.next().unwrap()))
        }
        n if n > 2 => Err(value_error("too many values to unpack (expected 2)", lineno)),
        n => Err(value_error(format!("not enough values to unpack (expected 2, got {n})"), lineno)),
    }
}

fn do_urlize(b: &Bound, lineno: usize) -> R<Value> {
    let value = b.get(1).unwrap();
    let mut rel: Vec<String> = match b.get(5).filter(|r| r.truthy()) {
        Some(r) => match r.text() {
            Some(t) => py_split(t, None, -1, false),
            None => return Err(attribute_error(r, "split", lineno)),
        },
        None => Vec::new(),
    };
    if b.get(3).is_some_and(Value::truthy) {
        rel.push("nofollow".to_string());
    }
    rel.push("noopener".to_string());
    rel.sort();
    rel.dedup();
    let rel = rel.join(" ");
    let target = b.get(4).filter(|t| !is_none(t)).cloned();
    let limit = b.get(2).filter(|l| !is_none(l)).cloned();
    Ok(str_value(urlize(value, limit.as_ref(), &rel, target.as_ref(), lineno)?))
}

/// `jinja2.utils.urlize`.
fn urlize(text: &Value, limit: Option<&Value>, rel: &str, target: Option<&Value>, lineno: usize) -> R<String> {
    let trim_url = |x: &str| -> R<String> {
        let Some(limit) = limit else {
            return Ok(x.to_string());
        };
        let n = match limit {
            Value::Py(PyValue::Int(_) | PyValue::Bool(_)) => index_value(limit, lineno)?,
            _ => return Err(type_error("slice indices must be integers or None or have an __index__ method", lineno)),
        };
        let chars: Vec<char> = x.chars().collect();
        let len = chars.len() as i64;
        let cut = if n < 0 { (len + n).max(0) } else { n.min(len) } as usize;
        let mut out: String = chars[..cut].iter().collect();
        if len >= n {
            out.push_str("...");
        }
        Ok(if out.is_empty() { x.to_string() } else { out })
    };
    let escaped = escape(text, lineno)?;
    let rel_attr = if rel.is_empty() { String::new() } else { format!(" rel=\"{}\"", escape_text(rel)) };
    let target_attr = match target.filter(|t| t.truthy()) {
        Some(t) => format!(" target=\"{}\"", escape(t, lineno)?),
        None => String::new(),
    };
    let mut out = String::with_capacity(escaped.len());
    for (word, space) in split_whitespace_runs(&escaped) {
        let mut middle = word;
        let mut head = "";
        let head_len = lead_punct_len(middle);
        if head_len > 0 {
            head = &middle[..head_len];
            middle = &middle[head_len..];
        }
        let mut tail = "";
        if [")", ">", ".", ",", "\n", "&gt;"].iter().any(|e| middle.ends_with(e)) {
            let start = trail_punct_start(middle);
            tail = &middle[start..];
            middle = &middle[..start];
        }
        let mut result = middle.to_string();
        let ascii_alnum = middle.chars().next().is_some_and(|c| c.is_ascii_alphanumeric());
        if middle.starts_with("www.")
            || (!middle.contains('@')
                && !middle.starts_with("http://")
                && !middle.starts_with("https://")
                && ascii_alnum
                && (middle.ends_with(".org") || middle.ends_with(".net") || middle.ends_with(".com")))
        {
            result = format!("<a href=\"http://{middle}\"{rel_attr}{target_attr}>{}</a>", trim_url(middle)?);
        }
        if result.starts_with("http://") || result.starts_with("https://") {
            result = format!("<a href=\"{result}\"{rel_attr}{target_attr}>{}</a>", trim_url(&result)?);
        }
        if result.contains('@') && !result.starts_with("www.") && !result.contains(':') && is_email(&result) {
            result = format!("<a href=\"mailto:{result}\">{result}</a>");
        }
        out.push_str(head);
        out.push_str(&result);
        out.push_str(tail);
        out.push_str(space);
    }
    Ok(out)
}

/// `re.split(r"(\s+)", text)` as `(word, following whitespace)` pairs.
fn split_whitespace_runs(text: &str) -> Vec<(&str, &str)> {
    let mut out = Vec::new();
    let mut rest = text;
    loop {
        let word_len: usize = rest.chars().take_while(|c| !is_py_space(*c)).map(char::len_utf8).sum();
        let (word, after) = rest.split_at(word_len);
        let space_len: usize = after.chars().take_while(|c| is_py_space(*c)).map(char::len_utf8).sum();
        let (space, next) = after.split_at(space_len);
        out.push((word, space));
        if next.is_empty() {
            if !space.is_empty() {
                out.push(("", ""));
            }
            break;
        }
        rest = next;
    }
    out
}

/// Length of the `^([(<]|&lt;)+` match.
fn lead_punct_len(s: &str) -> usize {
    let mut i = 0;
    loop {
        let rest = &s[i..];
        if rest.starts_with('(') || rest.starts_with('<') {
            i += 1;
        } else if rest.starts_with("&lt;") {
            i += 4;
        } else {
            return i;
        }
    }
}

/// Start of the `([)>.,\n]|&gt;)+$` match (the string length when none).
fn trail_punct_start(s: &str) -> usize {
    let tokens_to_end = |mut i: usize| -> bool {
        if i >= s.len() {
            return false;
        }
        while i < s.len() {
            let rest = &s[i..];
            if rest.starts_with([')', '>', '.', ',', '\n']) {
                i += 1;
            } else if rest.starts_with("&gt;") {
                i += 4;
            } else {
                return false;
            }
        }
        true
    };
    s.char_indices().map(|(i, _)| i).find(|&i| tokens_to_end(i)).unwrap_or(s.len())
}

/// `re.match(r"^\S+@\w[\w.-]*\.\w+$", middle)`.
fn is_email(s: &str) -> bool {
    let Some(at) = s.rfind('@') else {
        return false;
    };
    if at == 0 || s[..at].chars().any(is_py_space) {
        return false;
    }
    let domain = &s[at + 1..];
    let mut chars = domain.chars();
    if !chars.next().is_some_and(is_word) {
        return false;
    }
    if !domain.chars().all(|c| is_word(c) || c == '.' || c == '-') {
        return false;
    }
    match domain.rfind('.') {
        Some(dot) if dot >= 1 => {
            let tld = &domain[dot + 1..];
            !tld.is_empty() && tld.chars().all(is_word)
        }
        _ => false,
    }
}

/// `len(re.findall(r"\w+", s))`.
fn word_count(s: &str) -> i64 {
    let mut count = 0;
    let mut in_word = false;
    for c in s.chars() {
        let w = is_word(c);
        if w && !in_word {
            count += 1;
        }
        in_word = w;
    }
    count
}

fn do_wordwrap(b: &Bound, lineno: usize) -> R<Value> {
    let s = b.get(1).unwrap();
    let width = b.get(2).cloned().unwrap_or_else(|| int_value(79));
    let break_long_words = b.get(3).is_none_or(Value::truthy);
    let wrapstring = match b.get(4).filter(|w| w.truthy()) {
        Some(w) => w.clone(),
        None => str_value("\n"),
    };
    let hyphens = matches!(b.get(5), None | Some(Value::Py(PyValue::Bool(true))));
    let Some(text) = s.text() else {
        return Err(attribute_error(s, "splitlines", lineno));
    };
    let Some(joiner) = wrapstring.text() else {
        return Err(attribute_error(&wrapstring, "join", lineno));
    };
    let markup_join = matches!(wrapstring, Value::Markup(_));
    let join = |parts: Vec<String>| -> String {
        if markup_join {
            parts.iter().map(|p| escape_text(p)).collect::<Vec<_>>().join(joiner)
        } else {
            parts.join(joiner)
        }
    };
    let width = WrapWidth::new(&width, lineno)?;
    let mut paragraphs = Vec::new();
    for (line, _) in splitlines_keep(text) {
        paragraphs.push(join(wrap_line(line, &width, break_long_words, hyphens, lineno)?));
    }
    // The outer join receives the already-joined (escaped) paragraphs.
    let out = if markup_join { paragraphs.join(joiner) } else { join(paragraphs) };
    Ok(if markup_join { markup_value(out) } else { str_value(out) })
}

/// A `textwrap` width: an int, or a float (Python compares it freely but
/// cannot slice with it).
struct WrapWidth {
    value: f64,
    is_int: bool,
    repr: String,
}

impl WrapWidth {
    fn new(v: &Value, lineno: usize) -> R<WrapWidth> {
        match v {
            Value::Py(PyValue::Int(n)) => Ok(WrapWidth {
                value: n.to_f64_exact().unwrap_or(f64::INFINITY),
                is_int: true,
                repr: v.repr(),
            }),
            Value::Py(PyValue::Bool(b)) => Ok(WrapWidth { value: f64::from(u8::from(*b)), is_int: true, repr: v.repr() }),
            Value::Py(PyValue::Float(f)) => Ok(WrapWidth { value: *f, is_int: false, repr: v.repr() }),
            other => Err(type_error(
                format!("'<=' not supported between instances of '{}' and 'int'", other.type_name()),
                lineno,
            )),
        }
    }
}

/// `textwrap.wrap(line, width, expand_tabs=False, replace_whitespace=False, ...)`.
fn wrap_line(line: &str, width: &WrapWidth, break_long_words: bool, hyphens: bool, lineno: usize) -> R<Vec<String>> {
    if width.value <= 0.0 {
        return Err(value_error(format!("invalid width {} (must be > 0)", width.repr), lineno));
    }
    let chars: Vec<char> = line.chars().collect();
    let mut chunks: Vec<Vec<char>> = wrap_chunks(&chars, hyphens);
    chunks.reverse();
    let is_blank = |c: &[char]| c.iter().all(|ch| is_py_space(*ch));
    let mut lines: Vec<String> = Vec::new();
    while !chunks.is_empty() {
        let mut cur_line: Vec<Vec<char>> = Vec::new();
        let mut cur_len = 0usize;
        let w = width.value;
        if is_blank(chunks.last().unwrap()) && !lines.is_empty() {
            chunks.pop();
        }
        while let Some(chunk) = chunks.last() {
            let l = chunk.len();
            if (cur_len + l) as f64 <= w {
                cur_len += l;
                cur_line.push(chunks.pop().unwrap());
            } else {
                break;
            }
        }
        if chunks.last().is_some_and(|c| c.len() as f64 > w) {
            // `_handle_long_word`
            let space_left = if w < 1.0 {
                Some(1usize)
            } else if width.is_int {
                Some((w as usize).saturating_sub(cur_len))
            } else {
                None
            };
            if break_long_words {
                let Some(space_left) = space_left else {
                    return Err(type_error(
                        "slice indices must be integers or None or have an __index__ method",
                        lineno,
                    ));
                };
                let last = chunks.last_mut().unwrap();
                let split = space_left.min(last.len());
                cur_line.push(last[..split].to_vec());
                last.drain(..split);
            } else if cur_line.is_empty() {
                cur_line.push(chunks.pop().unwrap());
            }
            cur_len = cur_line.iter().map(Vec::len).sum();
        }
        if cur_line.last().is_some_and(|c| is_blank(c)) {
            cur_len -= cur_line.last().unwrap().len();
            cur_line.pop();
        }
        let _ = cur_len;
        if !cur_line.is_empty() {
            lines.push(cur_line.iter().flatten().collect());
        }
    }
    Ok(lines)
}

/// `TextWrapper._split`: `wordsep_re` (or `wordsep_simple_re` without
/// `break_on_hyphens`) chunks.
fn wrap_chunks(t: &[char], hyphens: bool) -> Vec<Vec<char>> {
    let ws = |c: char| "\t\n\u{0b}\u{0c}\r ".contains(c);
    let letter = |c: char| is_word(c) && !is_decimal(c);
    let wp = |c: char| is_word(c) || "!\"'&.,?".contains(c);
    let n = t.len();
    let mut out = Vec::new();
    let mut p = 0;
    while p < n {
        let end = if ws(t[p]) {
            let mut e = p;
            while e < n && ws(t[e]) {
                e += 1;
            }
            e
        } else if !hyphens {
            let mut e = p;
            while e < n && !ws(t[e]) {
                e += 1;
            }
            e
        } else {
            let mut em_dash = None;
            if t[p] == '-' && p > 0 && wp(t[p - 1]) {
                let mut k = p;
                while k < n && t[k] == '-' {
                    k += 1;
                }
                if k - p >= 2 && k < n && is_word(t[k]) {
                    em_dash = Some(k);
                }
            }
            match em_dash {
                Some(k) => k,
                None => {
                    let mut e = p + 1;
                    loop {
                        if e < n && t[e] == '-' {
                            let q = e + 1;
                            let behind = (e >= 2 && letter(t[e - 2]) && letter(t[e - 1]))
                                || (e >= 3 && letter(t[e - 3]) && t[e - 2] == '-' && letter(t[e - 1]));
                            let ahead = q < n
                                && letter(t[q])
                                && ((q + 1 < n && letter(t[q + 1]))
                                    || (q + 2 < n && t[q + 1] == '-' && letter(t[q + 2])));
                            if behind && ahead {
                                break q;
                            }
                        }
                        if e == n || ws(t[e]) {
                            break e;
                        }
                        if wp(t[e - 1]) && e + 1 < n && t[e] == '-' && t[e + 1] == '-' {
                            let mut k = e;
                            while k < n && t[k] == '-' {
                                k += 1;
                            }
                            if k < n && is_word(t[k]) {
                                break e;
                            }
                        }
                        e += 1;
                    }
                }
            }
        };
        out.push(t[p..end].to_vec());
        p = end;
    }
    out
}

fn do_xmlattr(b: &Bound, lineno: usize) -> R<Value> {
    let d = b.get(1).unwrap();
    let Value::Py(PyValue::Dict(d)) = d else {
        return Err(attribute_error(d, "items", lineno));
    };
    let autospace = b.get(2).is_none_or(Value::truthy);
    let mut parts = Vec::new();
    for (key, value) in d {
        if matches!(value, PyValue::None) {
            continue;
        }
        parts.push(format!(
            "{}=\"{}\"",
            escape(&Value::py(key.clone()), lineno)?,
            escape(&Value::py(value.clone()), lineno)?
        ));
    }
    let mut rv = parts.join(" ");
    if autospace && !rv.is_empty() {
        rv.insert(0, ' ');
    }
    Ok(str_value(rv))
}

fn do_tojson(value: &Value, indent: Option<&Value>, lineno: usize) -> R<Value> {
    let indent = match indent.filter(|i| !is_none(i)) {
        None => None,
        Some(i) => Some(match i {
            Value::Py(PyValue::Str(s)) => s.clone(),
            Value::Markup(s) => s.to_string(),
            Value::Py(PyValue::Int(_) | PyValue::Bool(_)) => {
                " ".repeat(usize::try_from(index_value(i, lineno)?).unwrap_or(0))
            }
            other => {
                return Err(type_error(
                    format!("can't multiply sequence by non-int of type '{}'", other.type_name()),
                    lineno,
                ))
            }
        }),
    };
    let mut out = String::new();
    json_encode(value, indent.as_deref(), 0, &mut out, lineno)?;
    let out = out
        .replace('<', "\\u003c")
        .replace('>', "\\u003e")
        .replace('&', "\\u0026")
        .replace('\'', "\\u0027");
    Ok(markup_value(out))
}

/// `json.dumps(value, sort_keys=True, indent=indent)`.
fn json_encode(v: &Value, indent: Option<&str>, level: usize, out: &mut String, lineno: usize) -> R<()> {
    let not_serializable = |t: &str| type_error(format!("Object of type {t} is not JSON serializable"), lineno);
    let open = |out: &mut String, bracket: char, level: usize| {
        out.push(bracket);
        if let Some(ind) = indent {
            out.push('\n');
            out.push_str(&ind.repeat(level + 1));
        }
    };
    let separator = |out: &mut String, level: usize| match indent {
        Some(ind) => {
            out.push_str(",\n");
            out.push_str(&ind.repeat(level + 1));
        }
        None => out.push_str(", "),
    };
    let close = |out: &mut String, bracket: char, level: usize| {
        if let Some(ind) = indent {
            out.push('\n');
            out.push_str(&ind.repeat(level));
        }
        out.push(bracket);
    };
    match v {
        Value::Markup(s) => out.push_str(&json_string(s)),
        Value::Py(PyValue::Dict(d)) => {
            if d.is_empty() {
                out.push_str("{}");
                return Ok(());
            }
            let keys: Vec<Value> = d.keys().cloned().map(Value::py).collect();
            let entries: Vec<Value> = d.iter().map(|(k, v)| Value::py(PyValue::Tuple(vec![k.clone(), v.clone()]))).collect();
            let sorted = sort_by_keys(&keys, entries, false, python_lt(lineno))?;
            open(out, '{', level);
            for (i, entry) in sorted.iter().enumerate() {
                let Value::Py(PyValue::Tuple(kv)) = entry else { continue };
                if i > 0 {
                    separator(out, level);
                }
                let key = match &kv[0] {
                    PyValue::Str(s) => s.clone(),
                    PyValue::Float(f) if f.is_nan() => "NaN".to_string(),
                    PyValue::Float(f) if f.is_infinite() => {
                        if *f > 0.0 { "Infinity" } else { "-Infinity" }.to_string()
                    }
                    PyValue::Bool(b) => if *b { "true" } else { "false" }.to_string(),
                    PyValue::None => "null".to_string(),
                    k @ (PyValue::Int(_) | PyValue::Float(_)) => k.repr(),
                    other => {
                        return Err(type_error(
                            format!(
                                "keys must be str, int, float, bool or None, not {}",
                                pycompat::value::type_name(other)
                            ),
                            lineno,
                        ))
                    }
                };
                out.push_str(&json_string(&key));
                out.push_str(": ");
                json_encode(&Value::py(kv[1].clone()), indent, level + 1, out, lineno)?;
            }
            close(out, '}', level);
        }
        Value::Py(PyValue::Bytes(_)) => return Err(not_serializable("bytes")),
        Value::Py(PyValue::Host(_)) => return Err(not_serializable("object")),
        Value::Py(PyValue::List(_) | PyValue::Tuple(_)) | Value::Seq(..) => {
            let (_, items) = seq_view(v).unwrap();
            if items.len() == 0 {
                out.push_str("[]");
                return Ok(());
            }
            open(out, '[', level);
            for i in 0..items.len() {
                if i > 0 {
                    separator(out, level);
                }
                json_encode(&items.get(i), indent, level + 1, out, lineno)?;
            }
            close(out, ']', level);
        }
        Value::Py(p) => out.push_str(&pyjson::dumps(p).map_err(|e| from_pyerr(&e, lineno))?),
        Value::Undefined(_) => return Err(not_serializable("Undefined")),
        other => return Err(not_serializable(other.type_name())),
    }
    Ok(())
}

fn json_string(s: &str) -> String {
    pyjson::dumps(&PyValue::Str(s.to_string())).unwrap_or_default()
}

// ---------------------------------------------------------------------------
// striptags / unescape (markupsafe)
// ---------------------------------------------------------------------------

/// `Markup(text).striptags()`.
fn striptags(text: &str, lineno: usize) -> R<String> {
    let mut stripped = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(i) = rest.find('<') {
        stripped.push_str(&rest[..i]);
        let at = &rest[i..];
        let comment_end = at.strip_prefix("<!--").and_then(|body| {
            let line_end = body.find('\n').unwrap_or(body.len());
            body[..line_end].find("-->").map(|e| 4 + e + 3)
        });
        if let Some(len) = comment_end {
            rest = &at[len..];
        } else if let Some(close) = at.find('>') {
            rest = &at[close + 1..];
        } else {
            stripped.push('<');
            rest = &at[1..];
        }
    }
    stripped.push_str(rest);
    let joined = py_split(&stripped, None, -1, false).join(" ");
    unescape(&joined, lineno)
}

/// `Markup.unescape()`: `&name;` entities replaced.
fn unescape(text: &str, lineno: usize) -> R<String> {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(i) = rest.find('&') {
        out.push_str(&rest[..i]);
        let at = &rest[i + 1..];
        let name_len = at.find(['&', ' ', ';']).unwrap_or(at.len());
        if name_len == 0 || !at[name_len..].starts_with(';') {
            out.push('&');
            rest = at;
            continue;
        }
        let name = &at[..name_len];
        let whole = &rest[i..i + name_len + 2];
        rest = &at[name_len + 1..];
        if let Ok(k) = HTML_ENTITIES.binary_search_by(|(n, _)| (*n).cmp(name)) {
            out.extend(char::from_u32(HTML_ENTITIES[k].1));
            continue;
        }
        let number = if let Some(hex) = name.strip_prefix("#x").or_else(|| name.strip_prefix("#X")) {
            Some(pytext::int_from_string(hex, 16))
        } else {
            name.strip_prefix('#').map(|dec| pytext::int_from_string(dec, 10))
        };
        match number {
            Some(Ok(n)) => match pyint_i64(&n).filter(|v| i32::try_from(*v).is_ok()) {
                Some(cp) => match u32::try_from(cp).ok().filter(|cp| *cp <= 0x10FFFF) {
                    Some(cp) => out.push(char::from_u32(cp).unwrap_or('\u{FFFD}')),
                    None => out.push_str(whole),
                },
                None => return Err(error("OverflowError", "Python int too large to convert to C int", lineno)),
            },
            _ => out.push_str(whole),
        }
    }
    out.push_str(rest);
    Ok(out)
}

/// `markupsafe._constants.HTML_ENTITIES`, sorted by name.
const HTML_ENTITIES: &[(&str, u32)] = &[
    ("AElig", 198), ("Aacute", 193), ("Acirc", 194), ("Agrave", 192), ("Alpha", 913),
    ("Aring", 197), ("Atilde", 195), ("Auml", 196), ("Beta", 914), ("Ccedil", 199), ("Chi", 935),
    ("Dagger", 8225), ("Delta", 916), ("ETH", 208), ("Eacute", 201), ("Ecirc", 202),
    ("Egrave", 200), ("Epsilon", 917), ("Eta", 919), ("Euml", 203), ("Gamma", 915),
    ("Iacute", 205), ("Icirc", 206), ("Igrave", 204), ("Iota", 921), ("Iuml", 207), ("Kappa", 922),
    ("Lambda", 923), ("Mu", 924), ("Ntilde", 209), ("Nu", 925), ("OElig", 338), ("Oacute", 211),
    ("Ocirc", 212), ("Ograve", 210), ("Omega", 937), ("Omicron", 927), ("Oslash", 216),
    ("Otilde", 213), ("Ouml", 214), ("Phi", 934), ("Pi", 928), ("Prime", 8243), ("Psi", 936),
    ("Rho", 929), ("Scaron", 352), ("Sigma", 931), ("THORN", 222), ("Tau", 932), ("Theta", 920),
    ("Uacute", 218), ("Ucirc", 219), ("Ugrave", 217), ("Upsilon", 933), ("Uuml", 220), ("Xi", 926),
    ("Yacute", 221), ("Yuml", 376), ("Zeta", 918), ("aacute", 225), ("acirc", 226), ("acute", 180),
    ("aelig", 230), ("agrave", 224), ("alefsym", 8501), ("alpha", 945), ("amp", 38), ("and", 8743),
    ("ang", 8736), ("apos", 39), ("aring", 229), ("asymp", 8776), ("atilde", 227), ("auml", 228),
    ("bdquo", 8222), ("beta", 946), ("brvbar", 166), ("bull", 8226), ("cap", 8745),
    ("ccedil", 231), ("cedil", 184), ("cent", 162), ("chi", 967), ("circ", 710), ("clubs", 9827),
    ("cong", 8773), ("copy", 169), ("crarr", 8629), ("cup", 8746), ("curren", 164), ("dArr", 8659),
    ("dagger", 8224), ("darr", 8595), ("deg", 176), ("delta", 948), ("diams", 9830),
    ("divide", 247), ("eacute", 233), ("ecirc", 234), ("egrave", 232), ("empty", 8709),
    ("emsp", 8195), ("ensp", 8194), ("epsilon", 949), ("equiv", 8801), ("eta", 951), ("eth", 240),
    ("euml", 235), ("euro", 8364), ("exist", 8707), ("fnof", 402), ("forall", 8704),
    ("frac12", 189), ("frac14", 188), ("frac34", 190), ("frasl", 8260), ("gamma", 947),
    ("ge", 8805), ("gt", 62), ("hArr", 8660), ("harr", 8596), ("hearts", 9829), ("hellip", 8230),
    ("iacute", 237), ("icirc", 238), ("iexcl", 161), ("igrave", 236), ("image", 8465),
    ("infin", 8734), ("int", 8747), ("iota", 953), ("iquest", 191), ("isin", 8712), ("iuml", 239),
    ("kappa", 954), ("lArr", 8656), ("lambda", 955), ("lang", 9001), ("laquo", 171),
    ("larr", 8592), ("lceil", 8968), ("ldquo", 8220), ("le", 8804), ("lfloor", 8970),
    ("lowast", 8727), ("loz", 9674), ("lrm", 8206), ("lsaquo", 8249), ("lsquo", 8216), ("lt", 60),
    ("macr", 175), ("mdash", 8212), ("micro", 181), ("middot", 183), ("minus", 8722), ("mu", 956),
    ("nabla", 8711), ("nbsp", 160), ("ndash", 8211), ("ne", 8800), ("ni", 8715), ("not", 172),
    ("notin", 8713), ("nsub", 8836), ("ntilde", 241), ("nu", 957), ("oacute", 243), ("ocirc", 244),
    ("oelig", 339), ("ograve", 242), ("oline", 8254), ("omega", 969), ("omicron", 959),
    ("oplus", 8853), ("or", 8744), ("ordf", 170), ("ordm", 186), ("oslash", 248), ("otilde", 245),
    ("otimes", 8855), ("ouml", 246), ("para", 182), ("part", 8706), ("permil", 8240),
    ("perp", 8869), ("phi", 966), ("pi", 960), ("piv", 982), ("plusmn", 177), ("pound", 163),
    ("prime", 8242), ("prod", 8719), ("prop", 8733), ("psi", 968), ("quot", 34), ("rArr", 8658),
    ("radic", 8730), ("rang", 9002), ("raquo", 187), ("rarr", 8594), ("rceil", 8969),
    ("rdquo", 8221), ("real", 8476), ("reg", 174), ("rfloor", 8971), ("rho", 961), ("rlm", 8207),
    ("rsaquo", 8250), ("rsquo", 8217), ("sbquo", 8218), ("scaron", 353), ("sdot", 8901),
    ("sect", 167), ("shy", 173), ("sigma", 963), ("sigmaf", 962), ("sim", 8764), ("spades", 9824),
    ("sub", 8834), ("sube", 8838), ("sum", 8721), ("sup", 8835), ("sup1", 185), ("sup2", 178),
    ("sup3", 179), ("supe", 8839), ("szlig", 223), ("tau", 964), ("there4", 8756), ("theta", 952),
    ("thetasym", 977), ("thinsp", 8201), ("thorn", 254), ("tilde", 732), ("times", 215),
    ("trade", 8482), ("uArr", 8657), ("uacute", 250), ("uarr", 8593), ("ucirc", 251),
    ("ugrave", 249), ("uml", 168), ("upsih", 978), ("upsilon", 965), ("uuml", 252),
    ("weierp", 8472), ("xi", 958), ("yacute", 253), ("yen", 165), ("yuml", 255), ("zeta", 950),
    ("zwj", 8205), ("zwnj", 8204),
];

// ---------------------------------------------------------------------------
// pprint (`pprint.pformat`, width 80, sorted dicts)
// ---------------------------------------------------------------------------

fn pformat(v: &Value) -> String {
    let mut out = String::new();
    pp_format(v, &mut out, 0, 0, 0);
    out
}

/// `_safe_key.__lt__`: a `TypeError` falls back to comparing type names
/// (the object ids CPython compares next are not reproducible; ties keep
/// their order).
fn safe_lt(a: &Value, b: &Value) -> R<bool> {
    match v_cmp("<", a, b, 0) {
        Ok(lt) => Ok(lt),
        Err(e) if e.kind() == "TypeError" => Ok(a.type_name() < b.type_name()),
        Err(_) => Ok(false),
    }
}

fn sorted_dict_items(d: &IndexMap<PyValue, PyValue>) -> Vec<(PyValue, PyValue)> {
    let keys: Vec<Value> = d.keys().cloned().map(Value::py).collect();
    let indices: Vec<Value> = (0..d.len()).map(|i| int_value(i as i64)).collect();
    let order = sort_by_keys(&keys, indices, false, safe_lt).unwrap_or_default();
    order
        .iter()
        .filter_map(|i| match i {
            Value::Py(PyValue::Int(n)) => pyint_i64(n).and_then(|i| d.get_index(i as usize)),
            _ => None,
        })
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect()
}

/// `_safe_repr` (dicts sorted, no depth limit).
fn safe_repr(v: &Value) -> String {
    match v {
        Value::Py(PyValue::Dict(d)) => {
            let parts: Vec<String> = sorted_dict_items(d)
                .into_iter()
                .map(|(k, val)| format!("{}: {}", safe_repr(&Value::py(k)), safe_repr(&Value::py(val))))
                .collect();
            format!("{{{}}}", parts.join(", "))
        }
        _ => match seq_view(v) {
            Some((is_list, items)) => {
                let parts: Vec<String> = (0..items.len()).map(|i| safe_repr(&items.get(i))).collect();
                if is_list {
                    format!("[{}]", parts.join(", "))
                } else if parts.len() == 1 {
                    format!("({},)", parts[0])
                } else {
                    format!("({})", parts.join(", "))
                }
            }
            None => v.repr(),
        },
    }
}

fn pp_format(v: &Value, out: &mut String, indent: usize, allowance: usize, level: usize) {
    let rep = safe_repr(v);
    let max_width = 80i64 - indent as i64 - allowance as i64;
    if (rep.chars().count() as i64) <= max_width {
        out.push_str(&rep);
        return;
    }
    match v {
        Value::Py(PyValue::Dict(d)) => {
            out.push('{');
            let items = sorted_dict_items(d);
            let inner = indent + 1;
            let last_index = items.len().saturating_sub(1);
            for (i, (key, ent)) in items.iter().enumerate() {
                let last = i == last_index;
                let krep = safe_repr(&Value::py(key.clone()));
                out.push_str(&krep);
                out.push_str(": ");
                pp_format(
                    &Value::py(ent.clone()),
                    out,
                    inner + krep.chars().count() + 2,
                    if last { allowance + 1 } else { 1 },
                    level + 1,
                );
                if !last {
                    out.push_str(",\n");
                    out.push_str(&" ".repeat(inner));
                }
            }
            out.push('}');
        }
        Value::Py(PyValue::Str(s)) => pp_str(s, &rep, out, indent, allowance, level + 1),
        Value::Py(PyValue::Bytes(b)) => pp_bytes(b, out, indent, allowance, level + 1),
        _ => match seq_view(v) {
            Some((is_list, items)) => {
                let endchar = if is_list {
                    "]"
                } else if items.len() == 1 {
                    ",)"
                } else {
                    ")"
                };
                out.push(if is_list { '[' } else { '(' });
                let inner = indent + 1;
                let allowance = allowance + endchar.len();
                for i in 0..items.len() {
                    if i > 0 {
                        out.push_str(",\n");
                        out.push_str(&" ".repeat(inner));
                    }
                    let last = i + 1 == items.len();
                    pp_format(&items.get(i), out, inner, if last { allowance } else { 1 }, level + 1);
                }
                out.push_str(endchar);
            }
            None => out.push_str(&rep),
        },
    }
}

fn pp_str(s: &str, rep: &str, out: &mut String, indent: usize, allowance: usize, level: usize) {
    if s.is_empty() {
        out.push_str(rep);
        return;
    }
    let (indent, allowance) = if level == 1 { (indent + 1, allowance + 1) } else { (indent, allowance) };
    let max_width = 80i64 - indent as i64;
    let mut max_width1 = max_width;
    let lines: Vec<String> = splitlines_keep(s).into_iter().map(|(l, e)| format!("{l}{e}")).collect();
    let width = |t: &str| pytext::str_repr(t).chars().count() as i64;
    let mut chunks: Vec<String> = Vec::new();
    let mut last_rep = String::new();
    for (i, line) in lines.iter().enumerate() {
        last_rep = pytext::str_repr(line);
        let last_line = i + 1 == lines.len();
        if last_line {
            max_width1 -= allowance as i64;
        }
        if width(line) <= max_width1 {
            chunks.push(last_rep.clone());
            continue;
        }
        let chars: Vec<char> = line.chars().collect();
        let mut parts: Vec<String> = Vec::new();
        let mut p = 0;
        while p < chars.len() {
            let start = p;
            while p < chars.len() && !is_py_space(chars[p]) {
                p += 1;
            }
            while p < chars.len() && is_py_space(chars[p]) {
                p += 1;
            }
            parts.push(chars[start..p].iter().collect());
        }
        let mut max_width2 = max_width;
        let mut current = String::new();
        for (j, part) in parts.iter().enumerate() {
            let candidate = format!("{current}{part}");
            if j + 1 == parts.len() && last_line {
                max_width2 -= allowance as i64;
            }
            if width(&candidate) > max_width2 {
                if !current.is_empty() {
                    chunks.push(pytext::str_repr(&current));
                }
                current = part.clone();
            } else {
                current = candidate;
            }
        }
        if !current.is_empty() {
            chunks.push(pytext::str_repr(&current));
        }
    }
    if chunks.len() == 1 {
        out.push_str(&last_rep);
        return;
    }
    if level == 1 {
        out.push('(');
    }
    for (i, chunk) in chunks.iter().enumerate() {
        if i > 0 {
            out.push('\n');
            out.push_str(&" ".repeat(indent));
        }
        out.push_str(chunk);
    }
    if level == 1 {
        out.push(')');
    }
}

fn pp_bytes(b: &[u8], out: &mut String, indent: usize, allowance: usize, level: usize) {
    if b.len() <= 4 {
        out.push_str(&pytext::bytes_repr(b));
        return;
    }
    let parens = level == 1;
    let (indent, allowance) = if parens { (indent + 1, allowance + 1) } else { (indent, allowance) };
    if parens {
        out.push('(');
    }
    let mut width = 80i64 - indent as i64;
    let last = b.len() / 4 * 4;
    let mut current: Vec<u8> = Vec::new();
    let mut reprs: Vec<String> = Vec::new();
    for i in (0..b.len()).step_by(4) {
        let part = &b[i..(i + 4).min(b.len())];
        let mut candidate = current.clone();
        candidate.extend_from_slice(part);
        if i == last {
            width -= allowance as i64;
        }
        if pytext::bytes_repr(&candidate).chars().count() as i64 > width {
            if !current.is_empty() {
                reprs.push(pytext::bytes_repr(&current));
            }
            current = part.to_vec();
        } else {
            current = candidate;
        }
    }
    if !current.is_empty() {
        reprs.push(pytext::bytes_repr(&current));
    }
    out.push_str(&reprs.join(&format!("\n{}", " ".repeat(indent))));
    if parens {
        out.push(')');
    }
}

// ---------------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------------

const SIG_TEST_VALUE: [&str; 1] = ["value"];

fn test_sig(name: &'static str) -> Sig {
    sig(name, &SIG_TEST_VALUE, 0, 0)
}

/// `environment.call_test(name, node, args, kwargs)` for a test of
/// `DEFAULT_TESTS`.
pub(super) fn eval_test(name: &str, node: Value, args: Vec<Value>, kwargs: &[(String, Value)], lineno: usize) -> R<Value> {
    let operator = match name {
        "==" | "eq" | "equalto" => Some(("eq", "==")),
        "!=" | "ne" => Some(("ne", "!=")),
        ">" | "gt" | "greaterthan" => Some(("gt", ">")),
        ">=" | "ge" => Some(("ge", ">=")),
        "<" | "lt" | "lessthan" => Some(("lt", "<")),
        "<=" | "le" => Some(("le", "<=")),
        _ => None,
    };
    if let Some((func, op)) = operator {
        if !kwargs.is_empty() {
            return Err(type_error(format!("{func}() takes no keyword arguments"), lineno));
        }
        if args.len() != 1 {
            return Err(type_error(format!("{func} expected 2 arguments, got {}", args.len() + 1), lineno));
        }
        let other = &args[0];
        return Ok(bool_value(match op {
            "==" => v_eq(&node, other),
            "!=" => !v_eq(&node, other),
            _ => v_cmp(op, &node, other, lineno)?,
        }));
    }
    if name == "callable" {
        one_argument("callable", &args, kwargs, lineno)?;
        return Ok(bool_value(matches!(node, Value::Func(_) | Value::Undefined(_))));
    }
    let two = |func: &'static str, second: &'static str| Sig {
        name: func,
        params: match second {
            "num" => &["value", "num"],
            "other" => &["value", "other"],
            _ => &["value", "seq"],
        },
        defaults: 0,
        implicit: 0,
        varargs: false,
        varkw: false,
    };
    let result = match name {
        "divisibleby" | "sameas" | "in" => {
            let (func, second) = match name {
                "divisibleby" => ("test_divisibleby", "num"),
                "sameas" => ("test_sameas", "other"),
                _ => ("test_in", "seq"),
            };
            let b = bind(&two(func, second), node, args, kwargs, lineno)?;
            let (value, other) = (b.get(0).unwrap(), b.get(1).unwrap());
            match name {
                "divisibleby" => v_eq(&eval_mod(value, other, lineno)?, &int_value(0)),
                "sameas" => same_object(value, other),
                _ => contains(other, value, lineno)?,
            }
        }
        _ => {
            let func: &'static str = match name {
                "odd" => "test_odd",
                "even" => "test_even",
                "defined" => "test_defined",
                "undefined" => "test_undefined",
                "none" => "test_none",
                "boolean" => "test_boolean",
                "false" => "test_false",
                "true" => "test_true",
                "integer" => "test_integer",
                "float" => "test_float",
                "lower" => "test_lower",
                "upper" => "test_upper",
                "string" => "test_string",
                "mapping" => "test_mapping",
                "number" => "test_number",
                "sequence" => "test_sequence",
                "iterable" => "test_iterable",
                "escaped" => "test_escaped",
                _ => {
                    return Err(error(
                        "TemplateRuntimeError",
                        format!("no test named {}", pytext::str_repr(name)),
                        lineno,
                    ))
                }
            };
            let b = bind(&test_sig(func), node, args, kwargs, lineno)?;
            let value = b.get(0).unwrap();
            match name {
                "odd" => v_eq(&eval_mod(value, &int_value(2), lineno)?, &int_value(1)),
                "even" => v_eq(&eval_mod(value, &int_value(2), lineno)?, &int_value(0)),
                "defined" => !value.is_undefined(),
                "undefined" => value.is_undefined(),
                "none" => is_none(value),
                "boolean" => matches!(value, Value::Py(PyValue::Bool(_))),
                "false" => matches!(value, Value::Py(PyValue::Bool(false))),
                "true" => matches!(value, Value::Py(PyValue::Bool(true))),
                "integer" => matches!(value, Value::Py(PyValue::Int(_))),
                "float" => matches!(value, Value::Py(PyValue::Float(_))),
                "lower" => py_is_case(&py_str(value, lineno)?, true),
                "upper" => py_is_case(&py_str(value, lineno)?, false),
                "string" => is_str(value),
                "mapping" => matches!(value, Value::Py(PyValue::Dict(_))),
                "number" => matches!(value, Value::Py(PyValue::Int(_) | PyValue::Float(_) | PyValue::Bool(_))),
                "sequence" => len_of(value, lineno).is_ok(),
                "iterable" => !matches!(
                    value,
                    Value::Py(PyValue::None | PyValue::Int(_) | PyValue::Float(_) | PyValue::Bool(_) | PyValue::Host(_))
                        | Value::Func(_)
                ),
                _ => matches!(value, Value::Markup(_)),
            }
        }
    };
    Ok(bool_value(result))
}

/// `a is b`, as far as the port can tell object identity.
fn same_object(a: &Value, b: &Value) -> bool {
    match (a, b) {
        (Value::Py(PyValue::None), Value::Py(PyValue::None)) => true,
        (Value::Py(PyValue::Bool(x)), Value::Py(PyValue::Bool(y))) => x == y,
        (Value::Py(PyValue::Int(x)), Value::Py(PyValue::Int(y))) => {
            pyint_i64(x).is_some_and(|v| (-5..=256).contains(&v)) && py_equal(&PyValue::Int(x.clone()), &PyValue::Int(y.clone()))
        }
        (Value::Py(PyValue::Str(x)), Value::Py(PyValue::Str(y))) => {
            x == y && x.chars().count() == 1 && x.chars().all(|c| (c as u32) < 256)
        }
        (Value::Status(x), Value::Status(y)) => Rc::ptr_eq(x, y),
        (Value::Iter(x), Value::Iter(y)) => Rc::ptr_eq(x, y),
        (Value::Func(x), Value::Func(y)) => std::ptr::addr_eq(Rc::as_ptr(x), Rc::as_ptr(y)),
        _ => false,
    }
}
