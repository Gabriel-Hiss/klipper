// Port of klippy/msgproto.py.

use indexmap::IndexMap;
use pycompat::text::{bytes_repr, float_from_string, int_from_string, percent_format, trunc_float_to_bigint, PctValue, PyError};
use pycompat::{PyInt, PyValue};

pub const MESSAGE_MIN: usize = 5;
pub const MESSAGE_MAX: usize = 64;
pub const MESSAGE_HEADER_SIZE: usize = 2;
pub const MESSAGE_TRAILER_SIZE: usize = 3;
pub const MESSAGE_POS_LEN: usize = 0;
pub const MESSAGE_POS_SEQ: usize = 1;
pub const MESSAGE_TRAILER_CRC: usize = 3;
pub const MESSAGE_TRAILER_SYNC: usize = 1;
pub const MESSAGE_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;
pub const MESSAGE_SEQ_MASK: u8 = 0x0f;
pub const MESSAGE_DEST: u8 = 0x10;
pub const MESSAGE_SYNC: u8 = 0x7e;

pub fn crc16_ccitt(buf: &[u8]) -> [u8; 2] {
    let mut crc: u16 = 0xffff;
    for &d in buf {
        let mut data: u16 = (d as u16) ^ (crc & 0xff);
        data ^= (data & 0x0f) << 4;
        crc = ((data << 8) | (crc >> 8)) ^ (data >> 4) ^ (data << 3);
    }
    [(crc >> 8) as u8, (crc & 0xff) as u8]
}

#[derive(Clone, Debug)]
pub enum MsgError {
    Error { message: String },
    EnumerationError { enum_name: String, value: String, message: String },
    IndexError { message: String },
    TypeError { message: String },
    ValueError { message: String },
}

impl std::fmt::Display for MsgError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message())
    }
}
impl std::error::Error for MsgError {}

impl MsgError {
    pub fn error(message: String) -> Self { MsgError::Error { message } }
    pub fn message(&self) -> &str {
        match self { MsgError::Error { message } | MsgError::EnumerationError { message, .. } | MsgError::IndexError { message } | MsgError::TypeError { message } | MsgError::ValueError { message } => message }
    }
    pub fn type_name(&self) -> &'static str {
        match self { MsgError::Error { .. } => "error", MsgError::EnumerationError { .. } => "enumeration_error", MsgError::IndexError { .. } => "IndexError", MsgError::TypeError { .. } => "TypeError", MsgError::ValueError { .. } => "ValueError" }
    }
    pub fn get_enum_params(&self) -> Option<(&str, &str)> {
        match self { MsgError::EnumerationError { enum_name, value, .. } => Some((enum_name, value)), _ => None }
    }
}
pub type Result<T, E = MsgError> = std::result::Result<T, E>;

fn enumeration_error(enum_name: &str, value: &str) -> MsgError {
    let message = format!("Unknown value '{}' in enumeration '{}'", value, enum_name);
    MsgError::EnumerationError { enum_name: enum_name.to_string(), value: value.to_string(), message }
}

#[derive(Clone, Debug, PartialEq)]
pub enum ParamValue {
    Int(i64),
    Bytes(Vec<u8>),
    Str(String),
}

impl ParamValue {
    pub fn py_str(&self) -> String {
        match self { ParamValue::Int(v) => v.to_string(), ParamValue::Bytes(b) => bytes_repr(b), ParamValue::Str(s) => s.clone() }
    }
    pub fn py_repr(&self) -> String {
        match self { ParamValue::Int(v) => v.to_string(), ParamValue::Bytes(b) => bytes_repr(b), ParamValue::Str(s) => pycompat::text::str_repr(s) }
    }
    pub fn to_pct(&self) -> PctValue {
        match self { ParamValue::Int(v) => PctValue::Int(PyInt::from_i64(*v)), ParamValue::Bytes(b) => PctValue::Bytes(b.clone()), ParamValue::Str(s) => PctValue::Str(s.clone()) }
    }
}

pub type Params = IndexMap<String, ParamValue>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind { Uint32, Int32, Uint16, Int16, Byte, String, ProgmemBuffer, Buffer }

impl Kind {
    pub fn is_int(&self) -> bool { matches!(self, Kind::Uint32|Kind::Int32|Kind::Uint16|Kind::Int16|Kind::Byte) }
    pub fn is_dynamic_string(&self) -> bool { matches!(self, Kind::String|Kind::ProgmemBuffer|Kind::Buffer) }
    pub fn signed(&self) -> bool { matches!(self, Kind::Int32|Kind::Int16) }
    pub fn max_length(&self) -> usize { match self { Kind::Uint32|Kind::Int32 => 5, Kind::Uint16|Kind::Int16 => 3, Kind::Byte => 2, _ => 64 } }
}

#[derive(Clone, Debug)]
pub struct ParamType {
    pub kind: Kind,
    pub enum_name: Option<String>,
    pub enums: Option<IndexMap<String, i64>>,
    pub reverse: Option<IndexMap<i64, String>>,
    pub max_length_override: Option<usize>,
}

impl ParamType {
    pub fn base(kind: Kind) -> Self { ParamType { kind, enum_name: None, enums: None, reverse: None, max_length_override: None } }
    pub fn is_int(&self) -> bool { if self.enum_name.is_some() { false } else { self.kind.is_int() } }
    pub fn is_dynamic_string(&self) -> bool { if self.enum_name.is_some() { false } else { self.kind.is_dynamic_string() } }
    pub fn max_length(&self) -> usize { self.max_length_override.unwrap_or(self.kind.max_length()) }
}

pub fn kind_for_fmt(fmt: &str) -> Option<Kind> {
    match fmt { "%u" => Some(Kind::Uint32), "%i" => Some(Kind::Int32), "%hu" => Some(Kind::Uint16), "%hi" => Some(Kind::Int16), "%c" => Some(Kind::Byte), "%s" => Some(Kind::String), "%.*s" => Some(Kind::ProgmemBuffer), "%*s" => Some(Kind::Buffer), _ => None }
}

pub fn pt_encode(out: &mut Vec<u8>, t: &ParamType, v: &ParamValue) -> Result<()> {
    if let Some(enum_name) = &t.enum_name {
        let enums = t.enums.as_ref().expect("enumeration map");
        let key = match v { ParamValue::Str(s) => s.clone(), ParamValue::Int(n) => n.to_string(), ParamValue::Bytes(b) => bytes_repr(b) };
        let tv = match enums.get(&key) { Some(x) => *x, None => return Err(enumeration_error(enum_name, &key)) };
        encode_int(out, tv);
        return Ok(());
    }
    if t.kind.is_int() {
        let n = match v { ParamValue::Int(n) => *n, ParamValue::Str(_) | ParamValue::Bytes(_) => return Err(MsgError::TypeError { message: "an integer is required".to_string() }) };
        encode_int(out, n);
    } else {
        let b = match v { ParamValue::Bytes(b) => b.clone(), ParamValue::Int(_) | ParamValue::Str(_) => return Err(MsgError::TypeError { message: "a bytes-like object is required".to_string() }) };
        out.push((b.len() & 0xff) as u8);
        out.extend_from_slice(&b);
    }
    Ok(())
}

pub fn pt_parse(s: &[u8], pos: usize, t: &ParamType) -> Result<(ParamValue, usize)> {
    if let Some(_enum_name) = &t.enum_name {
        let (v, npos) = parse_int(s, pos, t.kind.signed())?;
        let rev = t.reverse.as_ref().expect("enumeration reverse");
        let sv = match rev.get(&v) { Some(x) => x.clone(), None => format!("?{}", v) };
        return Ok((ParamValue::Str(sv), npos));
    }
    if t.kind.is_int() {
        let (v, npos) = parse_int(s, pos, t.kind.signed())?;
        Ok((ParamValue::Int(v), npos))
    } else {
        let l = *s.get(pos).ok_or_else(|| MsgError::IndexError { message: "index out of range".to_string() })? as usize;
        let start = pos + 1;
        let end = core::cmp::min(start + l, s.len());
        let b = if start <= s.len() && start <= end { s[start..end].to_vec() } else { Vec::new() };
        Ok((ParamValue::Bytes(b), start + l))
    }
}

pub fn encode_int(out: &mut Vec<u8>, v: i64) {
    if v >= 0xc000000 || v < -0x4000000 { out.push((((v >> 28) & 0x7f) | 0x80) as u8); }
    if v >= 0x180000 || v < -0x80000 { out.push((((v >> 21) & 0x7f) | 0x80) as u8); }
    if v >= 0x3000 || v < -0x1000 { out.push((((v >> 14) & 0x7f) | 0x80) as u8); }
    if v >= 0x60 || v < -0x20 { out.push((((v >> 7) & 0x7f) | 0x80) as u8); }
    out.push((v & 0x7f) as u8);
}

pub fn parse_int(s: &[u8], pos: usize, signed: bool) -> Result<(i64, usize)> {
    let mut p = pos;
    let mut c = *s.get(p).ok_or_else(|| MsgError::IndexError { message: "index out of range".to_string() })? as i64;
    p += 1;
    let mut v: i64 = c & 0x7f;
    if (c & 0x60) == 0x60 { v |= -0x20; }
    while c & 0x80 != 0 {
        c = *s.get(p).ok_or_else(|| MsgError::IndexError { message: "index out of range".to_string() })? as i64;
        p += 1;
        v = (v << 7) | (c & 0x7f);
    }
    if !signed { v &= 0xffff_ffff; }
    Ok((v, p))
}

pub fn lookup_params(msgformat: &str, enumerations: &IndexMap<String, IndexMap<String, i64>>) -> Result<Vec<(String, ParamType)>> {
    let mut out = Vec::new();
    let parts: Vec<&str> = msgformat.split_whitespace().collect();
    for arg in parts.iter().skip(1) {
        let segs: Vec<&str> = arg.split('=').collect();
        if segs.len() != 2 {
            return Err(MsgError::ValueError { message: "too many values to unpack (expected 2)".to_string() });
        }
        let name = segs[0].to_string();
        let fmt = segs[1];
        let kind = kind_for_fmt(fmt).ok_or_else(|| MsgError::Error { message: format!("Invalid message type for '{}'", fmt) })?;
        let mut t = ParamType::base(kind);
        for (enum_name, enums) in enumerations.iter() {
            if name == *enum_name || name.ends_with(&format!("_{}", enum_name)) {
                let mut reverse = IndexMap::new();
                for (k, v) in enums.iter() { reverse.insert(*v, k.clone()); }
                t.enum_name = Some(enum_name.clone());
                t.enums = Some(enums.clone());
                t.reverse = Some(reverse);
                break;
            }
        }
        out.push((name, t));
    }
    Ok(out)
}

pub fn lookup_output_params(msgformat: &str) -> Result<Vec<ParamType>> {
    let mut param_types = Vec::new();
    let mut args = msgformat;
    loop {
        let pos = match args.find('%') { Some(p) => p, None => break };
        let after = pos + 1;
        if after >= args.len() || args.as_bytes()[after] != b'%' {
            let mut found: Option<ParamType> = None;
            for i in 0..4usize {
                let end = pos + 1 + i;
                if end > args.len() { break; }
                if let Some(k) = kind_for_fmt(&args[pos..end]) { found = Some(ParamType::base(k)); break; }
            }
            match found { Some(t) => param_types.push(t), None => return Err(MsgError::error(format!("Invalid output format for '{}'", msgformat))) }
        }
        if after > args.len() { break; }
        args = &args[after..];
        if args.is_empty() { break; }
    }
    Ok(param_types)
}

pub fn convert_msg_format(msgformat: &str) -> String {
    let mut f = msgformat.to_string();
    for c in ["%u", "%i", "%hu", "%hi", "%c", "%.*s", "%*s"] { f = f.replace(c, "%s"); }
    f
}

#[derive(Clone, Debug)]
pub struct MessageFormat {
    pub msgid: i64,
    pub msgid_bytes: Vec<u8>,
    pub msgformat: String,
    pub debugformat: String,
    pub name: String,
    pub param_names: Vec<(String, ParamType)>,
}

impl MessageFormat {
    pub fn new(msgid_bytes: Vec<u8>, msgid: i64, msgformat: &str, enumerations: &IndexMap<String, IndexMap<String, i64>>) -> Result<Self> {
        let param_names = lookup_params(msgformat, enumerations)?;
        let name = msgformat.split_whitespace().next().ok_or_else(|| MsgError::IndexError { message: "list index out of range".to_string() })?.to_string();
        Ok(MessageFormat { msgid, msgid_bytes, msgformat: msgformat.to_string(), debugformat: convert_msg_format(msgformat), name, param_names })
    }
    pub fn param_types(&self) -> Vec<ParamType> { self.param_names.iter().map(|(_, t)| t.clone()).collect() }
    pub fn name_to_type(&self, name: &str) -> Option<&ParamType> { self.param_names.iter().find(|(n, _)| n == name).map(|(_, t)| t) }
    pub fn encode(&self, params: &[ParamValue]) -> Result<Vec<u8>> {
        let mut out = self.msgid_bytes.clone();
        for (i, (_, t)) in self.param_names.iter().enumerate() {
            let v = params.get(i).ok_or_else(|| MsgError::IndexError { message: "list index out of range".to_string() })?;
            pt_encode(&mut out, t, v)?;
        }
        Ok(out)
    }
    pub fn encode_by_name(&self, params: &Params) -> Result<Vec<u8>> {
        let mut out = self.msgid_bytes.clone();
        for (name, t) in self.param_names.iter() {
            let v = params.get(name).ok_or_else(|| MsgError::Error { message: format!("'{}'", name) })?;
            pt_encode(&mut out, t, v)?;
        }
        Ok(out)
    }
    pub fn parse(&self, s: &[u8], pos: usize) -> Result<(Params, usize)> {
        let mut p = pos + self.msgid_bytes.len();
        let mut out = Params::new();
        for (name, t) in self.param_names.iter() {
            let (v, np) = pt_parse(s, p, t)?;
            out.insert(name.clone(), v);
            p = np;
        }
        Ok((out, p))
    }
    pub fn format_params(&self, params: &Params) -> Result<String> {
        let mut vals = Vec::new();
        for (name, t) in self.param_names.iter() {
            let v = params.get(name).ok_or_else(|| MsgError::Error { message: format!("'{}'", name) })?;
            let pv = if t.is_dynamic_string() {
                let r = v.py_repr();
                PctValue::Str(r)
            } else { v.to_pct() };
            vals.push(pv);
        }
        percent_format(&self.debugformat, &vals).map_err(map_py_error)
    }
}

#[derive(Clone, Debug)]
pub struct OutputFormat {
    pub msgid: i64,
    pub msgid_bytes: Vec<u8>,
    pub msgformat: String,
    pub debugformat: String,
    pub param_types: Vec<ParamType>,
}
impl OutputFormat {
    pub fn new(msgid_bytes: Vec<u8>, msgid: i64, msgformat: &str) -> Result<Self> {
        Ok(OutputFormat { msgid, msgid_bytes, msgformat: msgformat.to_string(), debugformat: convert_msg_format(msgformat), param_types: lookup_output_params(msgformat)? })
    }
    pub fn parse(&self, s: &[u8], pos: usize) -> Result<(Params, usize)> {
        let mut p = pos + self.msgid_bytes.len();
        let mut vals = Vec::new();
        for t in self.param_types.iter() {
            let (v, np) = pt_parse(s, p, t)?;
            let pv = if t.is_dynamic_string() { PctValue::Str(v.py_repr()) } else { v.to_pct() };
            vals.push(pv);
            p = np;
        }
        let outmsg = percent_format(&self.debugformat, &vals).map_err(map_py_error)?;
        let mut m = Params::new();
        m.insert("#msg".to_string(), ParamValue::Str(outmsg));
        Ok((m, p))
    }
    pub fn format_params(&self, params: &Params) -> Result<String> {
        let v = params.get("#msg").ok_or_else(|| MsgError::Error { message: "'#msg'".to_string() })?;
        percent_format("#output %s", &[v.to_pct()]).map_err(map_py_error)
    }
}

#[derive(Clone, Debug)]
pub struct UnknownFormat;
impl UnknownFormat {
    pub fn parse(&self, s: &[u8], pos: usize) -> Result<(Params, usize)> {
        let (msgid, _) = parse_int(s, pos, true)?;
        let mut m = Params::new();
        m.insert("#msgid".to_string(), ParamValue::Int(msgid));
        m.insert("#msg".to_string(), ParamValue::Bytes(s.to_vec()));
        Ok((m, s.len().saturating_sub(MESSAGE_TRAILER_SIZE)))
    }
    pub fn format_params(&self, params: &Params) -> Result<String> {
        let v = params.get("#msg").ok_or_else(|| MsgError::Error { message: "'#msg'".to_string() })?;
        let r = match v { ParamValue::Bytes(b) => bytes_repr(b), other => other.py_repr() };
        percent_format("#unknown %s", &[PctValue::Str(r)]).map_err(map_py_error)
    }
}

fn map_py_error(e: PyError) -> MsgError {
    match e.kind { "TypeError" => MsgError::TypeError { message: e.msg }, "ValueError" => MsgError::ValueError { message: e.msg }, "OverflowError" => MsgError::ValueError { message: e.msg }, _ => MsgError::ValueError { message: e.msg } }
}

#[derive(Clone, Debug)]
pub enum FormatKind { Message(MessageFormat), Output(OutputFormat), Unknown(UnknownFormat) }

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConstParser { Str, Int, Float }

pub struct MessageParser {
    pub warn_prefix: String,
    pub unknown: UnknownFormat,
    pub enumerations: IndexMap<String, IndexMap<String, i64>>,
    pub messages: Vec<(i64, String, String)>,
    pub messages_by_id: IndexMap<i64, FormatKind>,
    pub messages_by_name: IndexMap<String, MessageFormat>,
    pub msgid_by_format: IndexMap<String, i64>,
    pub config: IndexMap<String, PyValue>,
    pub version: String,
    pub build_versions: String,
    pub raw_identify_data: Vec<u8>,
}

impl MessageParser {
    pub fn new(warn_prefix: &str) -> Self {
        let mut p = MessageParser { warn_prefix: warn_prefix.to_string(), unknown: UnknownFormat, enumerations: IndexMap::new(), messages: Vec::new(), messages_by_id: IndexMap::new(), messages_by_name: IndexMap::new(), msgid_by_format: IndexMap::new(), config: IndexMap::new(), version: String::new(), build_versions: String::new(), raw_identify_data: Vec::new() };
        let defaults: IndexMap<String, i64> = [("identify_response offset=%u data=%.*s".to_string(), 0i64), ("identify offset=%u count=%c".to_string(), 1i64)].into_iter().collect();
        let _ = p.init_messages(&defaults, &[], &[]);
        p
    }
    fn _error(&self, message: String) -> MsgError { MsgError::error(format!("{}{}", self.warn_prefix, message)) }

    pub fn check_packet(&self, s: &[u8]) -> i64 {
        if s.len() < MESSAGE_MIN { return 0; }
        let msglen = s[MESSAGE_POS_LEN] as usize;
        if msglen < MESSAGE_MIN || msglen > MESSAGE_MAX { return -1; }
        let msgseq = s[MESSAGE_POS_SEQ];
        if (msgseq & !MESSAGE_SEQ_MASK) != MESSAGE_DEST { return -1; }
        if s.len() < msglen { return 0; }
        if s[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC { return -1; }
        let msgcrc = [s[msglen - MESSAGE_TRAILER_CRC], s[msglen - MESSAGE_TRAILER_CRC + 1]];
        let crc = crc16_ccitt(&s[..msglen - MESSAGE_TRAILER_SIZE]);
        if crc != msgcrc { return -1; }
        msglen as i64
    }

    pub fn dump(&self, s: &[u8]) -> Result<Vec<String>> {
        let msgseq = *s.get(MESSAGE_POS_SEQ).ok_or_else(|| MsgError::IndexError { message: "index out of range".to_string() })?;
        let mut out = vec![percent_format("seq: %02x", &[PctValue::Int(PyInt::from_i64(msgseq as i64))]).map_err(map_py_error)?];
        let mut pos = MESSAGE_HEADER_SIZE;
        loop {
            let (msgid, _) = parse_int(s, pos, true)?;
            let fmt = self.messages_by_id.get(&msgid).cloned().unwrap_or(FormatKind::Unknown(UnknownFormat));
            let (params, np) = match &fmt { FormatKind::Message(m) => m.parse(s, pos)?, FormatKind::Output(o) => o.parse(s, pos)?, FormatKind::Unknown(u) => u.parse(s, pos)? };
            let s2 = match &fmt { FormatKind::Message(m) => m.format_params(&params)?, FormatKind::Output(o) => o.format_params(&params)?, FormatKind::Unknown(u) => u.format_params(&params)? };
            out.push(s2);
            pos = np;
            if pos >= s.len().saturating_sub(MESSAGE_TRAILER_SIZE) { break; }
            if s.len() < MESSAGE_TRAILER_SIZE { break; }
        }
        Ok(out)
    }

    pub fn format_params_p(&self, params: &Params) -> Result<String> {
        let name = params.get("#name").cloned().unwrap_or(ParamValue::Str("None".to_string()));
        if let ParamValue::Str(n) = &name {
            if let Some(m) = self.messages_by_name.get(n) { return m.format_params(params); }
        }
        if let Some(msg) = params.get("#msg") {
            return percent_format("%s %s", &[name.to_pct(), msg.to_pct()]).map_err(map_py_error);
        }
        Ok(pyvalue_str_params(params))
    }

    pub fn parse(&self, s: &[u8]) -> Result<Params> {
        let (msgid, _) = parse_int(s, MESSAGE_HEADER_SIZE, true)?;
        let fmt = self.messages_by_id.get(&msgid).cloned().unwrap_or(FormatKind::Unknown(UnknownFormat));
        let (mut params, pos) = match &fmt { FormatKind::Message(m) => m.parse(s, MESSAGE_HEADER_SIZE)?, FormatKind::Output(o) => o.parse(s, MESSAGE_HEADER_SIZE)?, FormatKind::Unknown(u) => u.parse(s, MESSAGE_HEADER_SIZE)? };
        if pos != s.len().saturating_sub(MESSAGE_TRAILER_SIZE) { return Err(self._error("Extra data at end of message".to_string())); }
        let nm = match &fmt { FormatKind::Message(m) => m.name.clone(), FormatKind::Output(_) => "#output".to_string(), FormatKind::Unknown(_) => "#unknown".to_string() };
        params.insert("#name".to_string(), ParamValue::Str(nm));
        Ok(params)
    }

    pub fn encode_msgblock(&self, seq: u8, cmd: &[u8]) -> Vec<u8> {
        let msglen = MESSAGE_MIN + cmd.len();
        let sq = (seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
        let mut out = vec![msglen as u8, sq];
        out.extend_from_slice(cmd);
        let c = crc16_ccitt(&out);
        out.extend_from_slice(&c);
        out.push(MESSAGE_SYNC);
        out
    }

    pub fn lookup_command(&self, msgformat: &str) -> Result<&MessageFormat> {
        let parts: Vec<&str> = msgformat.split_whitespace().collect();
        let msgname = parts.first().ok_or_else(|| MsgError::IndexError { message: "list index out of range".to_string() })?;
        let mp = self.messages_by_name.get(*msgname).ok_or_else(|| self._error(format!("Unknown command: {}", msgname)))?;
        if msgformat != mp.msgformat { return Err(self._error(format!("Command format mismatch: {} vs {}", msgformat, mp.msgformat))); }
        Ok(mp)
    }
    pub fn lookup_msgid(&self, msgformat: &str) -> Result<i64> {
        self.msgid_by_format.get(msgformat).cloned().ok_or_else(|| self._error(format!("Unknown command: {}", msgformat)))
    }

    pub fn create_command(&self, msg: &str) -> Result<Vec<u8>> {
        let parts: Vec<&str> = msg.split_whitespace().collect();
        if parts.is_empty() { return Ok(Vec::new()); }
        let msgname = parts[0];
        let mp = self.messages_by_name.get(msgname).cloned().ok_or_else(|| self._error(format!("Unknown command: {}", msgname)))?;
        let mut argparts = Params::new();
        let mut order: Vec<(String, String)> = Vec::new();
        for arg in parts.iter().skip(1) {
            let eq = arg.find('=');
            match eq { None => return Err(self._error(format!("Unable to extract params from: {}", msgname))), Some(i) => { order.push((arg[..i].to_string(), arg[i + 1..].to_string())); } }
        }
        for (k, _) in order.iter() { if !argparts.contains_key(k) { argparts.insert(k.clone(), ParamValue::Str(String::new())); } }
        for (name, value) in order.iter() {
            let t = match mp.name_to_type(name) { None => return Err(self._error(format!("Unable to extract params from: {}", msgname))), Some(t) => t.clone() };
            let tv: ParamValue;
            if t.is_dynamic_string() {
                let b = parse_buffer(value).map_err(|_| self._error(format!("Unable to extract params from: {}", msgname)))?;
                tv = ParamValue::Bytes(b);
            } else if t.is_int() {
                let n = int_from_string(value, 0).map_err(|_| self._error(format!("Unable to extract params from: {}", msgname)))?;
                let i = bigint_to_i64_range(&n);
                tv = ParamValue::Int(i);
            } else {
                tv = ParamValue::Str(value.clone());
            }
            argparts.insert(name.clone(), tv);
        }
        match mp.encode_by_name(&argparts) {
            Ok(c) => Ok(c),
            Err(e) => {
                if matches!(e, MsgError::EnumerationError { .. }) { Err(e) } else { Err(self._error(format!("Unable to encode: {}", msgname))) }
            }
        }
    }

    pub fn create_dummy_response(&self, msgname: &str, params: &Params) -> Result<Params> {
        let mp = self.messages_by_name.get(msgname).cloned().ok_or_else(|| self._error(format!("Unknown response: {}", msgname)))?;
        let mut argparts = params.clone();
        for (name, t) in mp.param_names.iter() {
            if !argparts.contains_key(name) {
                let v = if t.is_dynamic_string() { ParamValue::Bytes(Vec::new()) } else { ParamValue::Int(0) };
                argparts.insert(name.clone(), v);
            }
        }
        let msg = match mp.encode_by_name(&argparts) { Ok(c) => c, Err(e) => { if matches!(e, MsgError::EnumerationError { .. }) { return Err(e); } return Err(self._error(format!("Unable to encode: {}", msgname))); } };
        let (mut res, _) = mp.parse(&msg, 0)?;
        res.insert("#name".to_string(), ParamValue::Str(msgname.to_string()));
        Ok(res)
    }

    pub fn fill_enumerations(&mut self, enumerations: &IndexMap<String, IndexMap<String, EnumValue>>) -> Result<()> {
        for (add_name, add_enums) in enumerations.iter() {
            let enums = self.enumerations.entry(add_name.clone()).or_default();
            for (key, value) in add_enums.iter() {
                match value {
                    EnumValue::Simple(v) => { enums.insert(key.clone(), *v); }
                    EnumValue::Range { start_value, count } => {
                        let enum_s = key.clone();
                        let mut enum_root = enum_s.clone();
                        while let Some(ch) = enum_root.chars().last() { if ch.is_ascii_digit() { enum_root.pop(); } else { break; } }
                        let start_enum: i64 = if enum_root.len() != enum_s.len() { enum_s[enum_root.len()..].parse::<i64>().unwrap_or(0) } else { 0 };
                        for i in 0..*count { let kk = format!("{}{}", enum_root, start_enum + i); enums.insert(kk, start_value + i); }
                    }
                }
            }
        }
        Ok(())
    }

    fn init_messages(&mut self, messages: &IndexMap<String, i64>, command_ids: &[i64], output_ids: &[i64]) -> Result<()> {
        for (msgformat, msgid) in messages.iter() {
            let msgtype = if command_ids.contains(msgid) { "command" } else if output_ids.contains(msgid) { "output" } else { "response" };
            self.messages.push((*msgid, msgtype.to_string(), msgformat.clone()));
            self.msgid_by_format.insert(msgformat.clone(), *msgid);
            let mut msgid_bytes = Vec::new();
            encode_int(&mut msgid_bytes, *msgid);
            if msgtype == "output" {
                let o = OutputFormat::new(msgid_bytes, *msgid, msgformat)?;
                self.messages_by_id.insert(*msgid, FormatKind::Output(o));
            } else {
                let m = MessageFormat::new(msgid_bytes, *msgid, msgformat, &self.enumerations)?;
                self.messages_by_name.insert(m.name.clone(), m.clone());
                self.messages_by_id.insert(*msgid, FormatKind::Message(m));
            }
        }
        Ok(())
    }

    pub fn process_identify(&mut self, data: &[u8], decompress: bool) -> Result<()> {
        let data_owned: Vec<u8>;
        let data: &[u8] = if decompress {
            data_owned = zlib_decompress(data).map_err(|e| self._error(format!("Error during identify: {}", e)))?;
            &data_owned
        } else { data };
        let raw = data.to_vec();
        let text = match std::str::from_utf8(data) { Ok(t) => t.to_string(), Err(e) => return Err(self._error(format!("Error during identify: {}", e))) };
        let parsed = pycompat::json::loads(&text).map_err(|e| self._error(format!("Error during identify: {}", e.msg)))?;
        let map = match &parsed { PyValue::Dict(d) => pyvalue_to_str_map(d), _ => IndexMap::new() };
        self.raw_identify_data = raw;
        let process: std::result::Result<(), MsgError> = {
            let r: std::result::Result<(), MsgError> = Ok(());
            if let Some(ev) = map.get("enumerations") {
                let e = pyvalue_to_enum_spec(ev)?;
                self.fill_enumerations(&e)?;
            }
            let commands = map.get("commands").cloned().unwrap_or(PyValue::None);
            let responses = map.get("responses").cloned().unwrap_or(PyValue::None);
            let output = map.get("output").cloned().unwrap_or_else(|| PyValue::Dict(IndexMap::new()));
            let commands_map = pyvalue_to_msg_map(&commands).ok_or_else(|| MsgError::ValueError { message: "'NoneType' object is not iterable".to_string() })?;
            let responses_map = pyvalue_to_msg_map(&responses).ok_or_else(|| MsgError::ValueError { message: "'NoneType' object is not iterable".to_string() })?;
            let output_map = pyvalue_to_msg_map(&output).ok_or_else(|| MsgError::ValueError { message: "'NoneType' object is not iterable".to_string() })?;
            let mut all = commands_map.clone();
            for (k, v) in responses_map.iter() { all.insert(k.clone(), *v); }
            for (k, v) in output_map.iter() { all.insert(k.clone(), *v); }
            let cmd_ids: Vec<i64> = commands_map.values().cloned().collect();
            let out_ids: Vec<i64> = output_map.values().cloned().collect();
            self.init_messages(&all, &cmd_ids, &out_ids)?;
            if let Some(cv) = map.get("config") {
                if let PyValue::Dict(d) = cv { for (k, v) in pyvalue_to_str_map_values(d).iter() { self.config.insert(k.clone(), v.clone()); } }
            }
            if let Some(v) = map.get("version") { self.version = v.str(); }
            if let Some(v) = map.get("build_versions") { self.build_versions = v.str(); }
            r
        };
        if let Err(e) = process {
            match e { MsgError::Error { message } => Err(MsgError::error(message)), _ => Err(self._error(format!("Error during identify: {}", e.message()))) }
        } else { Ok(()) }
    }

    pub fn get_raw_data_dictionary(&self) -> &[u8] { &self.raw_identify_data }
    pub fn get_version_info(&self) -> (String, String) { (self.version.clone(), self.build_versions.clone()) }
    pub fn get_messages(&self) -> Vec<(i64, String, String)> { self.messages.clone() }
    pub fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> { self.enumerations.clone() }
    pub fn get_constants(&self) -> IndexMap<String, PyValue> { self.config.clone() }
    pub fn get_constant(&self, name: &str, default: Option<PyValue>, parser: ConstParser) -> Result<PyValue> {
        match self.config.get(name) {
            None => { if let Some(d) = default { Ok(d) } else { Err(self._error(format!("Firmware constant '{}' not found", name))) } }
            Some(cv) => apply_parser(cv, parser).map_err(|_| self._error(format!("Unable to parse firmware constant {}: {}", name, cv.str())))
        }
    }
    pub fn get_constant_float(&self, name: &str, default: Option<PyValue>) -> Result<f64> {
        match self.get_constant(name, default, ConstParser::Float)? { PyValue::Float(f) => Ok(f), PyValue::Int(n) => n.0.to_string().parse::<f64>().map_err(|_| MsgError::ValueError { message: "float overflow".to_string() }), PyValue::Bool(b) => Ok(if b { 1.0 } else { 0.0 }), _ => Err(MsgError::ValueError { message: "float conversion failed".to_string() }) }
    }
    pub fn get_constant_int(&self, name: &str, default: Option<PyValue>) -> Result<i64> {
        match self.get_constant(name, default, ConstParser::Int)? { PyValue::Int(n) => Ok(bigint_to_i64_range(&n)), PyValue::Bool(b) => Ok(if b { 1 } else { 0 }), PyValue::Float(f) => Ok(bigint_to_i64_range(&trunc_float_to_bigint(f))), _ => Err(MsgError::ValueError { message: "int conversion failed".to_string() }) }
    }
}

fn pyvalue_str_params(params: &Params) -> String {
    let mut parts = Vec::new();
    for (k, v) in params.iter() { parts.push(format!("{}: {}", pycompat::text::str_repr(k), v.py_repr())); }
    format!("{{{}}}", parts.join(", "))
}

fn bigint_to_i64_range(n: &PyInt) -> i64 {
    let s = n.0.to_string();
    s.parse::<i64>().unwrap_or(if s.starts_with('-') { i64::MIN } else { i64::MAX })
}

fn apply_parser(v: &PyValue, p: ConstParser) -> Result<PyValue> {
    match p {
        ConstParser::Str => Ok(PyValue::Str(v.str())),
        ConstParser::Int => match v {
            PyValue::Int(n) => Ok(PyValue::Int(n.clone())),
            PyValue::Bool(b) => Ok(PyValue::Int(PyInt::from_i64(if *b { 1 } else { 0 }))),
            PyValue::Float(f) => { if f.is_nan() || f.is_infinite() { return Err(MsgError::ValueError { message: "cannot convert float NaN to integer".to_string() }); } Ok(PyValue::Int(trunc_float_to_bigint(*f))) }
            PyValue::Str(s) => { let n = int_from_string(s, 10).map_err(map_py_error)?; Ok(PyValue::Int(n)) }
            _ => Err(MsgError::TypeError { message: "int() argument must be an integer, float, or string".to_string() }),
        },
        ConstParser::Float => match v {
            PyValue::Float(f) => Ok(PyValue::Float(*f)),
            PyValue::Int(n) => { let f = n.0.to_string().parse::<f64>().map_err(|_| MsgError::ValueError { message: "float conversion failed".to_string() })?; Ok(PyValue::Float(f)) }
            PyValue::Bool(b) => Ok(PyValue::Float(if *b { 1.0 } else { 0.0 })),
            PyValue::Str(s) => { let f = float_from_string(s).map_err(map_py_error)?; Ok(PyValue::Float(f)) }
            _ => Err(MsgError::TypeError { message: "float() argument must be a string or a real number".to_string() }),
        },
    }
}

#[derive(Clone, Debug)]
pub enum EnumValue { Simple(i64), Range { start_value: i64, count: i64 } }

fn pyvalue_to_str_map(d: &IndexMap<PyValue, PyValue>) -> IndexMap<String, PyValue> {
    let mut out = IndexMap::new();
    for (k, v) in d.iter() { if let PyValue::Str(s) = k { out.insert(s.clone(), v.clone()); } }
    out
}
fn pyvalue_to_str_map_values(d: &IndexMap<PyValue, PyValue>) -> IndexMap<String, PyValue> { pyvalue_to_str_map(d) }
fn pyvalue_to_msg_map(v: &PyValue) -> Option<IndexMap<String, i64>> {
    match v { PyValue::Dict(d) => { let mut out = IndexMap::new(); for (k, val) in d.iter() { if let (PyValue::Str(s), PyValue::Int(n)) = (k, val) { out.insert(s.clone(), bigint_to_i64_range(n)); } } Some(out) } _ => None }
}
fn pyvalue_to_enum_spec(v: &PyValue) -> Result<IndexMap<String, IndexMap<String, EnumValue>>> {
    let m = match v { PyValue::Dict(d) => pyvalue_to_str_map(d), _ => return Ok(IndexMap::new()) };
    let mut out = IndexMap::new();
    for (name, val) in m.iter() {
        let inner = match val { PyValue::Dict(d) => pyvalue_to_str_map(d), _ => continue };
        let mut im = IndexMap::new();
        for (k, ev) in inner.iter() {
            match ev {
                PyValue::Int(n) => { im.insert(k.clone(), EnumValue::Simple(bigint_to_i64_range(n))); }
                PyValue::List(l) => {
                    if l.len() < 2 { return Err(MsgError::ValueError { message: format!("not enough values to unpack (expected 2, got {})", l.len()) }); }
                    if l.len() > 2 { return Err(MsgError::ValueError { message: "too many values to unpack (expected 2)".to_string() }); }
                    let sv = match &l[0] { PyValue::Int(n) => bigint_to_i64_range(n), _ => return Err(MsgError::ValueError { message: "range bound must be integer".to_string() }) };
                    let ct = match &l[1] { PyValue::Int(n) => bigint_to_i64_range(n), _ => return Err(MsgError::ValueError { message: "range bound must be integer".to_string() }) };
                    im.insert(k.clone(), EnumValue::Range { start_value: sv, count: ct });
                }
                _ => continue,
            }
        }
        out.insert(name.clone(), im);
    }
    Ok(out)
}

pub fn parse_buffer(value: &str) -> std::result::Result<Vec<u8>, PyError> {
    if value.is_empty() { return Ok(Vec::new()); }
    let py = int_from_string(value, 16)?;
    let n = value.len() / 2;
    if n == 0 { return Ok(Vec::new()); }
    let sign = py.sign();
    let raw = py.inner().to_bytes_le().1;
    let mut low = raw.clone();
    low.truncate(n);
    while low.len() < n { low.push(0); }
    let bytes_le = if sign >= 0 { low } else {
        if low.iter().all(|&b| b == 0) { low } else {
            let mut out: Vec<u8> = low.iter().map(|b| !b).collect();
            let mut carry: u16 = 1;
            for b in out.iter_mut() { let s = (*b as u16) + carry; *b = (s & 0xff) as u8; carry = s >> 8; }
            out
        }
    };
    let mut out = bytes_le;
    out.reverse();
    Ok(out)
}

mod zlib {
    struct BitReader<'a> { data: &'a [u8], pos: usize, bit: u32 }
    impl<'a> BitReader<'a> {
        fn new(data: &'a [u8]) -> Self { BitReader { data, pos: 0, bit: 0 } }
        fn get(&mut self, n: u32) -> Option<u32> {
            let mut v = 0u32;
            for i in 0..n {
                if self.pos >= self.data.len() { return None; }
                let bit = ((self.data[self.pos] >> self.bit) & 1) as u32;
                v |= bit << i;
                self.bit += 1;
                if self.bit == 8 { self.bit = 0; self.pos += 1; }
            }
            Some(v)
        }
        fn byte_align(&mut self) { if self.bit != 0 { self.bit = 0; self.pos += 1; } }
    }
    fn build(lengths: &[u8]) -> Option<Vec<(u16, u8, u16)>> {
        let mut bl = [0u16; 16];
        for &l in lengths { bl[l as usize] += 1; }
        let mut code = 0u16;
        let mut next = [0u16; 16];
        for i in 1..16 { code = (code + bl[i - 1]) << 1; next[i] = code; }
        let mut out = Vec::new();
        for (i, &l) in lengths.iter().enumerate() {
            if l != 0 { let c = next[l as usize]; out.push((c, l, i as u16)); next[l as usize] += 1; }
        }
        Some(out)
    }
    fn decode(br: &mut BitReader, table: &[(u16, u8, u16)]) -> Option<u16> {
        let mut code: u16 = 0; let mut len: u8 = 0;
        loop {
            let b = br.get(1)?;
            code = (code << 1) | (b as u16);
            len += 1;
            if len > 15 { return None; }
            for &(c, l, s) in table.iter() { if l == len && c == code { return Some(s); } }
        }
    }
    const LN: [u16; 29] = [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258];
    const LE: [u32; 29] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
    const DN: [u16; 30] = [1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577];
    const DE: [u32; 30] = [0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12];
    fn inflate_block(br: &mut BitReader, out: &mut Vec<u8>, lit: &[(u16, u8, u16)], dist: &[(u16, u8, u16)]) -> Option<()> {
        loop {
            let s = decode(br, lit)? as usize;
            if s < 256 { out.push(s as u8); }
            else if s == 256 { return Some(()); }
            else {
                let idx = s - 257;
                if idx >= 29 { return None; }
                let length = LN[idx] as usize + br.get(LE[idx])? as usize;
                let ds = decode(br, dist)? as usize;
                if ds >= 30 { return None; }
                let d = DN[ds] as usize + br.get(DE[ds])? as usize;
                if d > out.len() { return None; }
                let start = out.len() - d;
                for i in 0..length { let b = out[start + i]; out.push(b); }
            }
        }
    }
    pub fn zlib_decompress(data: &[u8]) -> std::result::Result<Vec<u8>, String> {
        if data.len() < 6 { return Err("Error -3 while decompressing data: incorrect header check".to_string()); }
        let cmf = data[0]; let flg = data[1];
        if cmf & 0x0f != 8 { return Err("Error -3 while decompressing data: unknown compression method".to_string()); }
        if (((cmf as u16) << 8 | flg as u16) % 31) != 0 { return Err("Error -3 while decompressing data: incorrect header check".to_string()); }
        let def = &data[2..];
        let mut br = BitReader::new(def);
        let mut out = Vec::new();
        loop {
            let bfinal = br.get(1).ok_or("Error -5 while decompressing data: incomplete or truncated stream")?;
            let btype = br.get(2).ok_or("Error -5 while decompressing data: incomplete or truncated stream")?;
            match btype {
                0 => { br.byte_align(); let ln = ((*def.get(br.pos).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as u16) | ((*def.get(br.pos + 1).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as u16) << 8)) as usize; let nln = ((*def.get(br.pos + 2).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as u16) | ((*def.get(br.pos + 3).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as u16) << 8)) as usize; if (ln ^ nln) != 0xffff { return Err("Error -3 while decompressing data: invalid stored block lengths".to_string()); } br.pos += 4; for i in 0..ln { out.push(*def.get(br.pos + i).ok_or("Error -5 while decompressing data: incomplete or truncated stream")?); } br.pos += ln; }
                1 => {
                    let lit_len: Vec<u8> = (0..288).map(|i| if i < 144 { 8 } else if i < 256 { 9 } else if i < 280 { 7 } else { 8 }).collect();
                    let dist_len: Vec<u8> = vec![5; 30];
                    let lit = build(&lit_len).ok_or("Error -3 while decompressing data: invalid code lengths set")?;
                    let dist = build(&dist_len).ok_or("Error -3 while decompressing data: invalid code lengths set")?;
                    inflate_block(&mut br, &mut out, &lit, &dist).ok_or("Error -3 while decompressing data: invalid block type")?;
                }
                2 => {
                    let hlit = br.get(5).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as usize + 257;
                    let hdist = br.get(5).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as usize + 1;
                    let hclen = br.get(4).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as usize + 4;
                    const ORDER: [usize; 19] = [16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15];
                    let mut cl = vec![0u8; 19];
                    for i in 0..hclen { cl[ORDER[i]] = br.get(3).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as u8; }
                    let cltab = build(&cl).ok_or("Error -3 while decompressing data: invalid code lengths set")?;
                    let total = hlit + hdist;
                    let mut lengths = vec![0u8; total];
                    let mut i = 0;
                    while i < total {
                        let s = decode(&mut br, &cltab).ok_or("Error -3 while decompressing data: invalid code lengths set")?;
                        match s {
                            0..=15 => { lengths[i] = s as u8; i += 1; }
                            16 => { if i == 0 { return Err("Error -3 while decompressing data: invalid code lengths set".to_string()); } let prev = lengths[i - 1]; let rep = 3 + br.get(2).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as usize; for _ in 0..rep { if i >= total { break; } lengths[i] = prev; i += 1; } }
                            17 => { let rep = 3 + br.get(3).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as usize; for _ in 0..rep { if i >= total { break; } lengths[i] = 0; i += 1; } }
                            18 => { let rep = 11 + br.get(7).ok_or("Error -5 while decompressing data: incomplete or truncated stream")? as usize; for _ in 0..rep { if i >= total { break; } lengths[i] = 0; i += 1; } }
                            _ => return Err("Error -3 while decompressing data: invalid code lengths set".to_string()),
                        }
                    }
                    let lit = build(&lengths[..hlit]).ok_or("Error -3 while decompressing data: invalid code lengths set")?;
                    let dist = build(&lengths[hlit..]).ok_or("Error -3 while decompressing data: invalid code lengths set")?;
                    inflate_block(&mut br, &mut out, &lit, &dist).ok_or("Error -3 while decompressing data: invalid block type")?;
                }
                _ => return Err("Error -3 while decompressing data: invalid block type".to_string()),
            }
            if bfinal == 1 { break; }
        }
        if data.len() < 6 || out.len() > (1usize << 26) { return Err("Error -5 while decompressing data: incomplete or truncated stream".to_string()); }
        Ok(out)
    }
}
pub use zlib::zlib_decompress;
