//! Reading recordings from disk.
//!
//! Recordings live at `tests/fixtures/recordings/<unit_id>/<scenario>.jsonl`.
//! [`load_file`] parses one JSONL file into [`Record`]s. [`load_unit`]
//! parses every scenario file for a unit under a recordings root.
//! [`record_files`] lists the files without parsing them.

use std::fs;
use std::path::{Path, PathBuf};

use crate::value::{hex_decode, Exception, Record, RecordKind, SideEffects, Value};

// A small JSON parser for exactly the subset the recorder emits.
// Objects keep insertion order. Bare numbers must be integers; the
// recorder never writes non-integer JSON numbers (floats are tagged).
#[derive(Debug, Clone, PartialEq)]
enum Json {
    Null,
    Bool(bool),
    Int(i64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn new(s: &'a str) -> Parser<'a> {
        Parser { bytes: s.as_bytes(), pos: 0 }
    }

    fn error(&self, msg: &str) -> String {
        format!("{msg} at byte {}", self.pos)
    }

    fn ws(&mut self) {
        while self.pos < self.bytes.len() {
            match self.bytes[self.pos] {
                b' ' | b'\t' | b'\n' | b'\r' => self.pos += 1,
                _ => break,
            }
        }
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn expect(&mut self, c: u8) -> Result<(), String> {
        self.ws();
        if self.peek() == Some(c) {
            self.pos += 1;
            Ok(())
        } else {
            Err(self.error(&format!("expected {}",
                (c as char).escape_default())))
        }
    }

    fn value(&mut self) -> Result<Json, String> {
        self.ws();
        match self.peek() {
            Some(b'n') => self.literal("null", Json::Null),
            Some(b't') => self.literal("true", Json::Bool(true)),
            Some(b'f') => self.literal("false", Json::Bool(false)),
            Some(b'"') => Ok(Json::Str(self.string()?)),
            Some(b'[') => self.array(),
            Some(b'{') => self.object(),
            Some(c) if c == b'-' || c.is_ascii_digit() => self.number(),
            _ => Err(self.error("unexpected character")),
        }
    }

    fn literal(&mut self, lit: &str, out: Json) -> Result<Json, String> {
        if self.bytes[self.pos..].starts_with(lit.as_bytes()) {
            self.pos += lit.len();
            Ok(out)
        } else {
            Err(self.error(&format!("expected {lit}")))
        }
    }

    fn string(&mut self) -> Result<String, String> {
        if self.peek() != Some(b'"') {
            return Err(self.error("expected string"));
        }
        self.pos += 1;
        let raw = &self.bytes[self.pos..];
        // Find the closing quote and handle escapes.
        let mut out: Vec<u8> = Vec::new();
        let mut i = 0;
        while i < raw.len() {
            let c = raw[i];
            if c == b'"' {
                break;
            }
            if c == b'\\' {
                if i + 1 >= raw.len() {
                    return Err(self.error("bad escape"));
                }
                let e = raw[i + 1];
                match e {
                    b'"' => {
                        out.push(b'"');
                        i += 2;
                    }
                    b'\\' => {
                        out.push(b'\\');
                        i += 2;
                    }
                    b'/' => {
                        out.push(b'/');
                        i += 2;
                    }
                    b'b' => {
                        out.push(0x08);
                        i += 2;
                    }
                    b'f' => {
                        out.push(0x0c);
                        i += 2;
                    }
                    b'n' => {
                        out.push(b'\n');
                        i += 2;
                    }
                    b'r' => {
                        out.push(b'\r');
                        i += 2;
                    }
                    b't' => {
                        out.push(b'\t');
                        i += 2;
                    }
                    b'u' => {
                        let unit = |at: usize| -> Option<u32> {
                            let hex = raw.get(at..at + 4)?;
                            u32::from_str_radix(std::str::from_utf8(hex).ok()?, 16).ok()
                        };
                        let hi = unit(i + 2).ok_or_else(|| self.error("bad unicode escape"))?;
                        i += 6;
                        // CPython `json.dumps` writes non-BMP characters as a
                        // UTF-16 surrogate pair (`\ud83d\ude00`).
                        let code = if (0xD800..0xDC00).contains(&hi) {
                            let lo = match (raw.get(i..i + 2), unit(i + 2)) {
                                (Some(b"\\u"), Some(lo)) if (0xDC00..0xE000).contains(&lo) => lo,
                                _ => return Err(self.error("unpaired surrogate")),
                            };
                            i += 6;
                            0x10000 + ((hi - 0xD800) << 10) + (lo - 0xDC00)
                        } else {
                            hi
                        };
                        let ch = char::from_u32(code).ok_or_else(|| {
                            self.error("bad unicode escape")
                        })?;
                        let mut buf = [0u8; 4];
                        let s = ch.encode_utf8(&mut buf);
                        out.extend_from_slice(s.as_bytes());
                    }
                    _ => return Err(self.error("bad escape")),
                }
            } else {
                out.push(c);
                i += 1;
            }
        }
        if i >= raw.len() {
            return Err(self.error("unterminated string"));
        }
        self.pos += i + 1;
        String::from_utf8(out).map_err(|_| self.error("invalid UTF-8"))
    }

    fn number(&mut self) -> Result<Json, String> {
        let start = self.pos;
        if self.peek() == Some(b'-') {
            self.pos += 1;
        }
        let digits_start = self.pos;
        while matches!(self.peek(), Some(c) if c.is_ascii_digit()) {
            self.pos += 1;
        }
        if self.pos == digits_start {
            return Err(self.error("expected number"));
        }
        if matches!(self.peek(), Some(b'.') | Some(b'e') | Some(b'E')) {
            return Err(self.error("non-integer JSON number"));
        }
        let text = std::str::from_utf8(&self.bytes[start..self.pos])
            .map_err(|_| self.error("expected number"))?;
        text.parse::<i64>()
            .map(Json::Int)
            .map_err(|_| self.error("integer out of range"))
    }

    fn array(&mut self) -> Result<Json, String> {
        self.pos += 1; // '['
        let mut out = Vec::new();
        self.ws();
        if self.peek() == Some(b']') {
            self.pos += 1;
            return Ok(Json::Arr(out));
        }
        loop {
            out.push(self.value()?);
            self.ws();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                }
                Some(b']') => {
                    self.pos += 1;
                    return Ok(Json::Arr(out));
                }
                _ => return Err(self.error("expected , or ]")),
            }
        }
    }

    fn object(&mut self) -> Result<Json, String> {
        self.pos += 1; // '{'
        let mut out = Vec::new();
        self.ws();
        if self.peek() == Some(b'}') {
            self.pos += 1;
            return Ok(Json::Obj(out));
        }
        loop {
            self.ws();
            let key = self.string()?;
            self.expect(b':')?;
            let val = self.value()?;
            out.push((key, val));
            self.ws();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                }
                Some(b'}') => {
                    self.pos += 1;
                    return Ok(Json::Obj(out));
                }
                _ => return Err(self.error("expected , or }")),
            }
        }
    }
}

/// Parse one recording line (one JSON object) into a [`Record`].
pub fn parse_line(line: &str) -> Result<Record, String> {
    let mut p = Parser::new(line);
    let json = p.value()?;
    p.ws();
    if p.pos != p.bytes.len() {
        return Err(p.error("trailing data"));
    }
    record_from_json(json)
}

fn obj_get<'j>(obj: &'j [(String, Json)], key: &str) -> Result<&'j Json, String> {
    obj.iter()
        .find(|(k, _)| k == key)
        .map(|(_, v)| v)
        .ok_or_else(|| format!("missing field {key}"))
}

fn as_str(j: &Json) -> Result<String, String> {
    match j {
        Json::Str(s) => Ok(s.clone()),
        _ => Err("expected string".to_string()),
    }
}

fn as_u64(j: &Json) -> Result<u64, String> {
    match j {
        Json::Int(i) if *i >= 0 => Ok(*i as u64),
        _ => Err("expected unsigned integer".to_string()),
    }
}

fn decode_value(j: &Json) -> Result<Value, String> {
    match j {
        Json::Null => Ok(Value::Null),
        Json::Bool(b) => Ok(Value::Bool(*b)),
        Json::Int(i) => Ok(Value::Int(*i)),
        Json::Str(s) => Ok(Value::Str(s.clone())),
        Json::Arr(items) => {
            let mut out = Vec::with_capacity(items.len());
            for item in items {
                out.push(decode_value(item)?);
            }
            Ok(Value::List(out))
        }
        Json::Obj(obj) => {
            if obj.len() == 1 {
                match obj[0].0.as_str() {
                    "$int" => {
                        let s = as_str(&obj[0].1)?;
                        return Ok(Value::BigInt(s));
                    }
                    "$float" => {
                        let inner = as_obj(&obj[0].1)?;
                        let hex = as_str(obj_get(inner, "hex")?)?;
                        let repr = as_str(obj_get(inner, "repr")?)?;
                        return Ok(Value::Float(crate::value::FloatVal { hex, repr }));
                    }
                    "$bytes" => {
                        let s = as_str(&obj[0].1)?;
                        return Ok(Value::Bytes(hex_decode(&s)?));
                    }
                    "$tuple" => {
                        if let Json::Arr(items) = &obj[0].1 {
                            let mut out = Vec::with_capacity(items.len());
                            for item in items {
                                out.push(decode_value(item)?);
                            }
                            return Ok(Value::Tuple(out));
                        }
                    }
                    "$dict" => {
                        if let Json::Arr(items) = &obj[0].1 {
                            let mut out = Vec::with_capacity(items.len());
                            for item in items {
                                if let Json::Arr(pair) = item {
                                    if pair.len() == 2 {
                                        out.push((
                                            decode_value(&pair[0])?,
                                            decode_value(&pair[1])?,
                                        ));
                                        continue;
                                    }
                                }
                                return Err("bad $dict entry".to_string());
                            }
                            return Ok(Value::Dict(out));
                        }
                    }
                    "$struct" => {
                        let inner = as_obj(&obj[0].1)?;
                        let type_name = as_str(obj_get(inner, "type")?)?;
                        let fields_j = obj_get(inner, "fields")?;
                        let mut fields = Vec::new();
                        if let Json::Arr(items) = fields_j {
                            for item in items {
                                if let Json::Arr(pair) = item {
                                    if pair.len() == 2 {
                                        let name = as_str(&pair[0])?;
                                        fields.push((name, decode_value(&pair[1])?));
                                        continue;
                                    }
                                }
                                return Err("bad struct field entry".to_string());
                            }
                            return Ok(Value::Struct { type_name, fields });
                        }
                        return Err("bad struct fields".to_string());
                    }
                    "$opaque" => {
                        let inner = as_obj(&obj[0].1)?;
                        let type_name = as_str(obj_get(inner, "type")?)?;
                        let addr = as_str(obj_get(inner, "addr")?)?;
                        return Ok(Value::Opaque { type_name, addr });
                    }
                    "$oid" => {
                        let s = as_str(&obj[0].1)?;
                        return Ok(Value::ObjectId(s));
                    }
                    _ => {}
                }
            }
            Err("unknown tagged object".to_string())
        }
    }
}

fn as_obj(j: &Json) -> Result<&[(String, Json)], String> {
    match j {
        Json::Obj(obj) => Ok(obj),
        _ => Err("expected object".to_string()),
    }
}

fn decode_side_effects(j: &Json) -> Result<SideEffects, String> {
    let obj = as_obj(j)?;
    let mut out = SideEffects::default();
    for (key, val) in obj {
        match key.as_str() {
            "mutated_args" => {
                if let Json::Arr(items) = val {
                    for item in items {
                        if let Json::Arr(pair) = item {
                            if pair.len() == 2 {
                                if let Json::Int(i) = &pair[0] {
                                    out.mutated_args
                                        .push((*i as usize, decode_value(&pair[1])?));
                                    continue;
                                }
                            }
                        }
                        return Err("bad mutated_args entry".to_string());
                    }
                } else {
                    return Err("bad mutated_args".to_string());
                }
            }
            "mcu_messages" => {
                if let Json::Arr(items) = val {
                    for item in items {
                        let s = as_str(item)?;
                        out.mcu_messages.push(hex_decode(&s)?);
                    }
                } else {
                    return Err("bad mcu_messages".to_string());
                }
            }
            "log_lines" => {
                if let Json::Arr(items) = val {
                    for item in items {
                        out.log_lines.push(as_str(item)?);
                    }
                } else {
                    return Err("bad log_lines".to_string());
                }
            }
            _ => {}
        }
    }
    Ok(out)
}

fn record_from_json(j: Json) -> Result<Record, String> {
    let obj = as_obj(&j)?;
    let v = as_u64(obj_get(obj, "v")?)?;
    let unit = as_str(obj_get(obj, "unit")?)?;
    let scenario = as_str(obj_get(obj, "scenario")?)?;
    let seq = as_u64(obj_get(obj, "seq")?)?;
    let function = as_str(obj_get(obj, "function")?)?;
    let kind = match as_str(obj_get(obj, "kind")?)?.as_str() {
        "python" => RecordKind::Python,
        "c_boundary" => RecordKind::CBoundary,
        "c_internal" => RecordKind::CInternal,
        other => return Err(format!("unknown kind {other}")),
    };
    let mut args = Vec::new();
    if let Json::Arr(items) = obj_get(obj, "args")? {
        for item in items {
            args.push(decode_value(item)?);
        }
    } else {
        return Err("bad args".to_string());
    }
    let (ret, exc) = match obj_get(obj, "ret") {
        Ok(rj) => (Some(decode_value(rj)?), None),
        Err(_) => {
            let ej = obj_get(obj, "exc")?;
            let eobj = as_obj(ej)?;
            let class_name = as_str(obj_get(eobj, "type")?)?;
            let message = as_str(obj_get(eobj, "message")?)?;
            (None, Some(Exception { class_name, message }))
        }
    };
    let side_effects = decode_side_effects(obj_get(obj, "side_effects")?)?;
    let object_id = match obj_get(obj, "object_id") {
        Ok(Json::Str(s)) => Some(s.clone()),
        _ => None,
    };
    Ok(Record {
        v,
        unit,
        scenario,
        seq,
        function,
        args,
        ret,
        exc,
        side_effects,
        object_id,
        kind,
    })
}

/// Parse one JSONL recording file. Blank lines are ignored.
pub fn load_file(path: &Path) -> Result<Vec<Record>, String> {
    let text = fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let mut out = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let record = parse_line(line)
            .map_err(|e| format!("{}:{}: {e}", path.display(), n + 1))?;
        out.push(record);
    }
    Ok(out)
}

/// All recording files for `unit` under `root`, sorted by file name.
pub fn record_files(root: &Path, unit: &str) -> Result<Vec<PathBuf>, String> {
    let dir = root.join(unit);
    let entries = fs::read_dir(&dir)
        .map_err(|e| format!("cannot list {}: {e}", dir.display()))?;
    let mut out = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|e| e.to_string())?;
        let path = entry.path();
        if path.extension().map(|e| e == "jsonl").unwrap_or(false) {
            out.push(path);
        }
    }
    out.sort();
    Ok(out)
}

/// Parse every scenario recording for `unit` under `root`.
/// Returns one vector per scenario file, in sorted file-name order.
pub fn load_unit(root: &Path, unit: &str) -> Result<Vec<Vec<Record>>, String> {
    let mut out = Vec::new();
    for path in record_files(root, unit)? {
        out.push(load_file(&path)?);
    }
    Ok(out)
}
