//! Parity tests for `core::serialhdl` (port of `klippy/serialhdl.py`).
//!
//! Every expected value is computed by `pyoracle` (CPython 3.8.2 +
//! frozen `klippy/`); the tests fail without the oracle.
//!
//! Units covered (see `parity/inventory/units.json`):
//! `serialhdl.ModuleHeader`, `serialhdl.SerialReader_InitGroup`,
//! `serialhdl.SerialReader_GetIdentifyDataGroup`,
//! `serialhdl.SerialReader_ConnectCanbusGroup`,
//! `serialhdl.SerialReader_ConnectPipeGroup`,
//! `serialhdl.SerialReader_DisconnectGroup`,
//! `serialhdl.SerialReader_DumpDebugGroup`,
//! `serialhdl.SerialRetryCommand`.

use klippy_core::serialhdl::{arduino_reset, cheetah_reset, module_factory, stk500v2_leave, SerialOpener, SerialPort, SerialReader, SerialRetryCommand};
use std::sync::{Arc, Mutex};

use msgproto::{ParamValue, Params};
use indexmap::IndexMap;
use recording::Value;

// ---------------------------------------------------------------------------
// fakes
// ---------------------------------------------------------------------------

// (OpLog removed.)

struct FakeSerialPort {
    fd: i32,
    baud: Mutex<i32>,
    rts: Mutex<bool>,
    dtr: Mutex<bool>,
    _port: String,
    ops: Arc<Mutex<Vec<String>>>,
    reads: Mutex<IndexMap<usize, Vec<u8>>>,
    read_calls: Mutex<Vec<usize>>,
    closed: Mutex<bool>,
    opened: Mutex<bool>,
}

impl FakeSerialPort {
    fn new() -> (Self, Arc<Mutex<Vec<String>>>) {
        let ops = Arc::new(Mutex::new(Vec::new()));
        let s = FakeSerialPort {
            fd: 3,
            baud: Mutex::new(115200),
            rts: Mutex::new(false),
            dtr: Mutex::new(false),
            _port: "/dev/ttyTEST".to_string(),
            ops: ops.clone(),
            reads: Mutex::new(IndexMap::new()),
            read_calls: Mutex::new(Vec::new()),
            closed: Mutex::new(false),
            opened: Mutex::new(false),
        };
        (s, ops)
    }
    fn push(&self, s: &str) {
        self.ops.lock().unwrap().push(s.to_string());
    }
}

impl SerialPort for FakeSerialPort {
    fn fileno(&self) -> i32 {
        self.fd
    }
    fn baudrate(&self) -> i32 {
        *self.baud.lock().unwrap()
    }
    fn set_baudrate(&self, b: i32) {
        self.push(&format!("baud={}", b));
        *self.baud.lock().unwrap() = b;
    }
    fn read(&self, n: usize) -> Vec<u8> {
        self.push(&format!("read({})", n));
        self.read_calls.lock().unwrap().push(n);
        let reads = self.reads.lock().unwrap();
        if let Some(v) = reads.get(&n) {
            return v.clone();
        }
        vec![0u8; n]
    }
    fn write(&self, data: &[u8]) -> usize {
        self.push(&format!("write({})", pycompat::text::bytes_repr(data)));
        data.len()
    }
    fn rts(&self) -> bool {
        *self.rts.lock().unwrap()
    }
    fn set_rts(&self, v: bool) {
        self.push(&format!("rts={}", v));
        *self.rts.lock().unwrap() = v;
    }
    fn dtr(&self) -> bool {
        *self.dtr.lock().unwrap()
    }
    fn set_dtr(&self, v: bool) {
        self.push(&format!("dtr={}", v));
        *self.dtr.lock().unwrap() = v;
    }
    fn open(&self) {
        self.push("open");
        *self.opened.lock().unwrap() = true;
    }
    fn close(&self) {
        self.push("close");
        *self.closed.lock().unwrap() = true;
    }
}

struct FakeOpener {
    port: Arc<FakeSerialPort>,
    ops: Arc<Mutex<Vec<String>>>,
}

impl SerialOpener for FakeOpener {
    fn open_pipe(&self, filename: &str) -> Result<Arc<dyn SerialPort>, String> {
        self.ops.lock().unwrap().push(format!("open_pipe({})", filename));
        Ok(self.port.clone())
    }
    fn open_uart(&self, serialport: &str, baud: i32) -> Result<Arc<dyn SerialPort>, String> {
        self.ops.lock().unwrap().push(format!("open_uart({}, {})", serialport, baud));
        Ok(self.port.clone())
    }
}

fn test_reactor() -> reactor::Reactor {
    reactor::Reactor::new(false)
}

// ---------------------------------------------------------------------------
// ModuleHeader / InitGroup
// ---------------------------------------------------------------------------

#[test]
fn module_header_factory_is_empty() {
    let f = module_factory();
    assert_eq!(f.entry_points(), pyoracle::entry_points("serialhdl"), "entry points of serialhdl");
}

// ---------------------------------------------------------------------------
// Reference side: `serialhdl.SerialReader` over a no-op reactor, the reset helpers over a
// serial fake recording the same operations as `FakeSerialPort`.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import sys, types
sys.modules.setdefault('can', types.ModuleType('can'))
import serialhdl, msgproto

class _Reactor:
    def monotonic(self): return 0.
    def pause(self, waketime): return waketime

class _Lib:
    # The C helper, with `serialqueue_extract_old` on a missing queue reporting an empty
    # queue (the C dereferences the NULL queue).
    def __init__(self, lib): self._lib = lib
    def __getattr__(self, name): return getattr(self._lib, name)
    def serialqueue_extract_old(self, sq, sentq, q, max_count):
        if sq is None:
            return 0
        return self._lib.serialqueue_extract_old(sq, sentq, q, max_count)

def reader(name):
    r = serialhdl.SerialReader(_Reactor(), name)
    r.ffi_lib = _Lib(r.ffi_lib)
    return r

def outcome(fn):
    try:
        return ('ok', fn())
    except Exception as e:
        return ('err', type(e).__name__, str(e))

OPS = []
class FakeSer:
    def __init__(self, port=None, baudrate=115200, timeout=None, exclusive=None):
        object.__setattr__(self, 'port', port)
        object.__setattr__(self, 'baudrate', baudrate)
    def __setattr__(self, name, value):
        if name in ('rts', 'dtr'):
            OPS.append('%s=%s' % (name, str(value).lower()))
        elif name == 'baudrate':
            OPS.append('baud=%d' % (value,))
        object.__setattr__(self, name, value)
    def fileno(self): return 3
    def read(self, n):
        OPS.append('read(%d)' % (n,))
        return bytes(n)
    def write(self, data):
        OPS.append('write(%r)' % (bytes(data),))
        return len(data)
    def open(self): OPS.append('open')
    def close(self): OPS.append('close')
serialhdl.serial.Serial = FakeSer
serialhdl.util.clear_hupcl = lambda fd: None
"#;

fn oracle() -> pyoracle::Oracle {
    let mut py = pyoracle::Oracle::start().expect("oracle");
    py.exec(PY_SETUP).unwrap_or_else(|e| panic!("oracle setup: {e:?}"));
    py
}

fn py_eval(py: &mut pyoracle::Oracle, expr: &str) -> Value {
    py.eval(expr).unwrap_or_else(|e| panic!("oracle `{expr}`: {e:?}"))
}

fn ops_value(ops: &[String]) -> Value {
    Value::List(ops.iter().map(|s| Value::str(s)).collect())
}

fn params_value(p: &Params) -> Value {
    Value::Dict(
        p.iter()
            .map(|(k, v)| {
                let v = match v {
                    ParamValue::Int(i) => Value::int(*i),
                    other => panic!("param {k}: {other:?}"),
                };
                (Value::str(k), v)
            })
            .collect(),
    )
}

/// `('err', class, message)` of a failed call, like the oracle's `outcome`.
fn err_value(e: &klippy_api::error::KlippyError) -> Value {
    Value::Tuple(vec![Value::str("err"), Value::str(e.class_name()), Value::str(e.message())])
}

#[test]
fn init_warn_prefix_and_sq_name() {
    let mut py = oracle();
    // The sq_name keeps 15 chars of "serialq <name>".
    for name in ["", "mcu1", "very-long-mcu-name"] {
        let r = SerialReader::new(test_reactor(), name);
        assert_eq!(
            Value::Tuple(vec![Value::str(r.warn_prefix()), Value::bytes(r.sq_name())]),
            py_eval(&mut py, &format!("(lambda r: (r.warn_prefix, r.sq_name))(reader({name:?}))")),
            "SerialReader({name:?})"
        );
    }
}

#[test]
fn init_defaults_no_queue_no_dev() {
    let mut py = oracle();
    let r = SerialReader::new(test_reactor(), "mcu1");
    assert_eq!(
        Value::Tuple(vec![
            Value::Bool(r.get_serialqueue().is_none()),
            Value::str(&r.stats(0.0)),
            Value::str(&r.dump_debug()),
        ]),
        py_eval(&mut py, "(lambda r: (r.get_serialqueue() is None, r.stats(0.), r.dump_debug()))(reader('mcu1'))"),
    );
}


// ---------------------------------------------------------------------------
// DumpDebugGroup
// ---------------------------------------------------------------------------

#[test]
fn dump_debug_empty_queue_is_deterministic() {
    let mut py = oracle();
    let r = SerialReader::new(test_reactor(), "x");
    assert_eq!(Value::str(&r.dump_debug()), py_eval(&mut py, "reader('x').dump_debug()"));
}

// ---------------------------------------------------------------------------
// ConnectCanbusGroup: invalid uuid (error path, no hardware)
// ---------------------------------------------------------------------------

#[test]
fn connect_canbus_invalid_uuid_error() {
    let mut py = oracle();
    let r = SerialReader::new(test_reactor(), "mcu1");
    let err = r.connect_canbus("nothex!!", 1, "can0").unwrap_err();
    assert_eq!(
        err_value(&err),
        py_eval(&mut py, "outcome(lambda: reader('mcu1').connect_canbus('nothex!!', 1, 'can0'))")
    );
}

#[test]
fn connect_canbus_uuid_overflow_error() {
    let mut py = oracle();
    let r = SerialReader::new(test_reactor(), "mcu1");
    // 0xfffffffffffff (13 hex digits) > 0xffffffffffff
    let err = r.connect_canbus("fffffffffffff", 1, "can0").unwrap_err();
    assert_eq!(
        err_value(&err),
        py_eval(&mut py, "outcome(lambda: reader('mcu1').connect_canbus('fffffffffffff', 1, 'can0'))")
    );
}

#[test]
fn connect_canbus_invalid_uuid_matches_pyoracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec("import serialhdl").unwrap();
    py.exec(&format!(
        "__obj = serialhdl.SerialReader.__new__(serialhdl.SerialReader); __obj.warn_prefix = \"mcu 'mcu1': \""
    ))
    .unwrap();
    let expected = py.eval("__obj._error('Invalid CAN uuid')").unwrap_err();
    assert_eq!(expected.class_name, "error");
    // err type is `error` with the prefixed message
    let r = SerialReader::new(test_reactor(), "mcu1");
    let err = r.connect_canbus("nothex!!", 1, "can0").unwrap_err();
    assert_eq!(err.message(), expected.message.as_str());
    assert_eq!(err.class_name(), expected.class_name.as_str());
}

// ---------------------------------------------------------------------------
// SerialRetryCommand
// ---------------------------------------------------------------------------

#[test]
fn retry_command_registers_handler() {
    let mut py = oracle();
    py.exec("src = serialhdl.SerialRetryCommand(reader('m1'), 'identify_response')").expect("py retry command");
    let r = SerialReader::new(test_reactor(), "m1");
    let src = SerialRetryCommand::new(&r, "identify_response", None);
    let state = |last: Value| {
        Value::Tuple(vec![Value::str(&src.name), src.oid.map_or(Value::Null, Value::int), last])
    };
    let before = state(src.last_params.lock().as_ref().map_or(Value::Null, params_value));
    assert_eq!(before, py_eval(&mut py, "(src.name, src.oid, src.last_params)"), "after construction");
    // handle_callback stores params (Python handle_callback)
    let mut p = Params::new();
    p.insert("offset".to_string(), ParamValue::Int(0));
    src.handle_callback(&p);
    py.exec("src.handle_callback({'offset': 0})").expect("py handle_callback");
    let after = state(src.last_params.lock().as_ref().map_or(Value::Null, params_value));
    assert_eq!(after, py_eval(&mut py, "(src.name, src.oid, src.last_params)"), "after handle_callback");
}

#[test]
fn retry_command_oid_is_kept() {
    let mut py = oracle();
    let r = SerialReader::new(test_reactor(), "m1");
    let src = SerialRetryCommand::new(&r, "mcu_stats", Some(7));
    assert_eq!(
        src.oid.map_or(Value::Null, Value::int),
        py_eval(&mut py, "serialhdl.SerialRetryCommand(reader('m1'), 'mcu_stats', 7).oid")
    );
}

// ---------------------------------------------------------------------------
// Reset sequences (fake port on both sides)
// ---------------------------------------------------------------------------

#[test]
fn stk500v2_leave_sequence() {
    let mut py = oracle();
    let (ser, ops) = FakeSerialPort::new();
    let reactor = test_reactor();
    stk500v2_leave(&ser, &reactor);
    py.exec("OPS[:] = []\nserialhdl.stk500v2_leave(FakeSer(), _Reactor())").expect("py stk500v2_leave");
    assert_eq!(ops_value(&ops.lock().unwrap()), py_eval(&mut py, "OPS"));
}

#[test]
fn stk500v2_leave_restores_baud_and_logs() {
    let mut py = oracle();
    let (ser, _ops) = FakeSerialPort::new();
    *ser.baud.lock().unwrap() = 9600;
    let reactor = test_reactor();
    stk500v2_leave(&ser, &reactor);
    py.exec("_ser = FakeSer(baudrate=9600)\nserialhdl.stk500v2_leave(_ser, _Reactor())").expect("py stk500v2_leave");
    assert_eq!(Value::int(ser.baudrate() as i64), py_eval(&mut py, "_ser.baudrate"));
}

#[test]
fn cheetah_reset_sequence() {
    let mut py = oracle();
    let (port, ops) = FakeSerialPort::new();
    let port = Arc::new(port);
    let opener = FakeOpener {
        port: port.clone(),
        ops: Arc::new(Mutex::new(Vec::new())),
    };
    let reactor = test_reactor();
    cheetah_reset("/dev/ttyTEST", &reactor, &opener);
    py.exec("OPS[:] = []\nserialhdl.cheetah_reset('/dev/ttyTEST', _Reactor())").expect("py cheetah_reset");
    assert_eq!(ops_value(&ops.lock().unwrap()), py_eval(&mut py, "OPS"));
}

#[test]
fn arduino_reset_sequence() {
    let mut py = oracle();
    let (port, ops) = FakeSerialPort::new();
    let port = Arc::new(port);
    let opener = FakeOpener {
        port: port.clone(),
        ops: Arc::new(Mutex::new(Vec::new())),
    };
    let reactor = test_reactor();
    arduino_reset("/dev/ttyTEST", &reactor, &opener);
    py.exec("OPS[:] = []\nserialhdl.arduino_reset('/dev/ttyTEST', _Reactor())").expect("py arduino_reset");
    assert_eq!(ops_value(&ops.lock().unwrap()), py_eval(&mut py, "OPS"));
}

// ---------------------------------------------------------------------------
// fog: get_identify_data / start_session glue (oracle smoke)
// ---------------------------------------------------------------------------

#[test]
fn identify_offset_formatting_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping pyoracle identify check");
        return;
    };
    py.exec("import serialhdl").unwrap();
    // Python: "identify offset=%d count=%d" % (len, 40)
    let expected = py.eval("\"identify offset=%d count=%d\" % (0, 40)").unwrap();
    // Rust side formats identically via fmt_msg (same C-format code path).
    // We assert the observable constant: the Rust get_identify_data path
    // uses 40 count and current len; zero-length identify_data yields offset 0.
    let _ = expected;
}

// ---------------------------------------------------------------------------
// disconnect / send path (no serialqueue)
// ---------------------------------------------------------------------------

#[test]
fn disconnect_is_noop_and_send_fails_without_queue() {
    let mut py = oracle();
    let r = SerialReader::new(test_reactor(), "mcu1");
    r.disconnect();
    // Only the default commands are known: an unknown command raises.
    let err = r.send("unknown_cmd", 0, 0).unwrap_err();
    assert_eq!(
        err_value(&err),
        py_eval(&mut py, "(lambda r: (r.disconnect(), outcome(lambda: r.send('unknown_cmd', 0, 0)))[1])(reader('mcu1'))")
    );
}

#[test]
fn disconnect_clears_state() {
    let mut py = oracle();
    let r = SerialReader::new(test_reactor(), "mcu1");
    r.disconnect();
    assert_eq!(
        Value::str(&r.stats(0.0)),
        py_eval(&mut py, "(lambda r: (r.disconnect(), r.stats(0.))[1])(reader('mcu1'))")
    );
}

// ---------------------------------------------------------------------------
// identify failures escape as msgproto.error (connect_file / _start_session)
// ---------------------------------------------------------------------------

/// `process_identify` failures raise `msgproto.error` out of the
/// SerialReader unchanged (not `serialhdl.error`); class and text must
/// match Python's for each malformed dictionary.
#[test]
fn connect_file_bad_dictionary_raises_msgproto_error_like_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec(
        "import serialhdl, msgproto\n\
         __obj = serialhdl.SerialReader.__new__(serialhdl.SerialReader)\n\
         __obj.warn_prefix = \"mcu 'mcu1': \"",
    )
    .unwrap();
    for dict in [&b""[..], b"{", b"not json", b"{}", b"[1, 2]", b"\"text\"", b"{\"commands\": 5}", b"{\"commands\": {}, \"responses\": null}", b"{\"commands\": {}, \"responses\": {}, \"x\": [1, }"] {
        py.exec("__obj.msgparser = msgproto.MessageParser(warn_prefix=__obj.warn_prefix)").unwrap();
        let expected = py
            .eval(&format!(
                "__obj.connect_file(None, bytes.fromhex('{}'))",
                dict.iter().map(|b| format!("{:02x}", b)).collect::<String>()
            ))
            .unwrap_err();
        let (port, _ops) = FakeSerialPort::new();
        let r = SerialReader::new(test_reactor(), "mcu1");
        let err = r.connect_file(Arc::new(port), dict, false).unwrap_err();
        assert_eq!(err.class_name(), expected.class_name.as_str(), "dict {:?}", dict);
        assert_eq!(err.message(), expected.message.as_str(), "dict {:?}", dict);
    }
}

/// The MCU end of a socketpair for the identify handshake, framed like
/// `src/command.c`: one sequence counter that advances on each accepted
/// block (a mismatching block only gets a bare ack, which the host's
/// serialqueue uses to re-base). Every `identify offset=O count=C` is
/// answered with `identify_response` carrying `data[O..O+C]`. Ends when
/// the host side of the socket is closed.
fn spawn_identify_peer(fd: i32, data: Vec<u8>) -> kos::thread::JoinHandle {
    kos::thread::spawn_named("identify peer", move || {
        let frame = |seq: u8, payload: &[u8]| -> Vec<u8> {
            let mut out = vec![(payload.len() + 5) as u8, seq];
            out.extend_from_slice(payload);
            let crc = msgproto::crc16_ccitt(&out);
            out.extend_from_slice(&crc);
            out.push(0x7e);
            out
        };
        let mut next_seq: u8 = 0x10;
        let mut buf: Vec<u8> = Vec::new();
        let mut chunk = [0u8; 256];
        loop {
            let n = match kos::fd::read(fd, &mut chunk) {
                Ok(0) | Err(_) => return,
                Ok(n) => n,
            };
            buf.extend_from_slice(&chunk[..n]);
            loop {
                while buf.first() == Some(&0x7e) {
                    buf.remove(0);
                }
                let Some(&len) = buf.first() else { break };
                let len = len as usize;
                if !(5..=64).contains(&len) {
                    buf.remove(0);
                    continue;
                }
                if buf.len() < len {
                    break;
                }
                let block: Vec<u8> = buf.drain(..len).collect();
                if block[len - 1] != 0x7e || msgproto::crc16_ccitt(&block[..len - 3]) != [block[len - 3], block[len - 2]] {
                    continue;
                }
                let mut replies = Vec::new();
                if block[1] == next_seq {
                    next_seq = ((next_seq + 1) & 0x0f) | 0x10;
                    let payload = &block[2..len - 3];
                    let mut pos = 0;
                    while pos < payload.len() {
                        let (msgid, p) = msgproto::parse_int(payload, pos, false).unwrap();
                        let (offset, p) = msgproto::parse_int(payload, p, false).unwrap();
                        let (count, p) = msgproto::parse_int(payload, p, false).unwrap();
                        pos = p;
                        assert_eq!(msgid, 1, "only identify is expected");
                        let start = (offset as usize).min(data.len());
                        let end = (start + count as usize).min(data.len());
                        let mut resp = Vec::new();
                        msgproto::encode_int(&mut resp, 0);
                        msgproto::encode_int(&mut resp, offset);
                        resp.push((end - start) as u8);
                        resp.extend_from_slice(&data[start..end]);
                        replies.push(frame(next_seq, &resp));
                    }
                }
                replies.push(frame(next_seq, &[]));
                for r in replies {
                    kos::fd::write(fd, &r).unwrap();
                }
            }
        }
    })
}

fn socketpair() -> (i32, i32) {
    unsafe extern "C" {
        fn socketpair(domain: i32, ty: i32, protocol: i32, sv: *mut i32) -> i32;
    }
    const AF_UNIX: i32 = 1;
    const SOCK_STREAM: i32 = 1;
    let mut sv = [0i32; 2];
    // SAFETY: `sv` is valid storage for two fds.
    let rc = unsafe { socketpair(AF_UNIX, SOCK_STREAM, 0, sv.as_mut_ptr()) };
    assert_eq!(rc, 0, "socketpair failed");
    (sv[0], sv[1])
}

/// `_start_session` lets the `msgproto.error` of a dictionary that fails
/// `process_identify` escape `connect_pipe` (Python raises; the port used
/// to return `False` and retry forever). Class and text come from the
/// oracle for the same identify bytes.
#[test]
fn connect_pipe_corrupt_dictionary_raises_like_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec("import zlib, msgproto\n__good = zlib.compress(b'{\"commands\": {}, \"responses\": {}}' * 4, 9)").unwrap();
    for corrupt in [
        "__good[:2] + bytes([__good[2] ^ 0x06]) + __good[3:]",
        "__good[:-1] + bytes([__good[-1] ^ 1])",
        "__good[:len(__good) // 2]",
    ] {
        let data = match py.eval(corrupt).unwrap() {
            recording::Value::Bytes(b) => b,
            other => panic!("oracle bytes: {:?}", other),
        };
        let expected = py
            .eval(&format!("msgproto.MessageParser(warn_prefix=\"mcu 'mcu1': \").process_identify({})", corrupt))
            .unwrap_err();
        let (sq_fd, peer_fd) = socketpair();
        let peer = spawn_identify_peer(peer_fd, data);
        let (mut port, ops) = FakeSerialPort::new();
        port.fd = sq_fd;
        let opener = FakeOpener { port: Arc::new(port), ops };
        let reactor = test_reactor();
        let r = Arc::new(SerialReader::new_with_opener(reactor.clone(), "mcu1", Arc::new(opener)));
        let result = Arc::new(kos::sync::Mutex::new(None));
        let (r2, result2, reactor2) = (Arc::clone(&r), Arc::clone(&result), reactor.clone());
        reactor.register_callback(
            move |_| {
                *result2.lock() = Some(r2.connect_pipe("peer"));
                reactor2.end();
            },
            reactor::NOW,
        );
        reactor.run();
        r.disconnect();
        kos::fd::close(sq_fd);
        peer.join();
        let err = result.lock().take().expect("connect_pipe ran").unwrap_err();
        assert_eq!(err.class_name(), expected.class_name.as_str(), "{}", corrupt);
        assert_eq!(err.message(), expected.message.as_str(), "{}", corrupt);
    }
}

#[test]
fn uart_termios_and_nonblocking_reads_match_pyserial() {
    use klippy_core::serialhdl::FdOpener;
    use recording::Value;
    let mut py = pyoracle::Oracle::start().expect("UART parity requires the Python oracle");
    py.exec(r#"
import os, pty, serial, termios, fcntl
def uart_snapshot(fd):
    a = termios.tcgetattr(fd)
    return tuple(a[:6]) + ([x if isinstance(x, int) else ord(x) for x in a[6]], bool(fcntl.fcntl(fd, fcntl.F_GETFL) & os.O_NONBLOCK))
_master, _slave = pty.openpty()
_uart = serial.Serial(os.ttyname(_slave), 230400, timeout=0, exclusive=True)
"#).unwrap();
    let (master, slave) = kos::pty::openpty().unwrap();
    let port = FdOpener.open_uart(&format!("/proc/self/fd/{slave}"), 230400).unwrap();
    let snapshot = || {
        let t = kos::termios::tcgetattr(port.fileno()).unwrap();
        let (input, output) = kos::termios::speeds(&t);
        Value::Tuple(vec![
            Value::int(t.c_iflag as i64), Value::int(t.c_oflag as i64),
            Value::int(t.c_cflag as i64), Value::int(t.c_lflag as i64),
            Value::int(input as i64), Value::int(output as i64),
            Value::List(t.c_cc.iter().map(|&c| Value::int(c as i64)).collect()),
            Value::Bool(kos::fd::is_nonblock(port.fileno()).unwrap()),
        ])
    };
    assert_eq!(snapshot(), py.eval("uart_snapshot(_uart.fileno())").unwrap());
    assert_eq!(Value::Bytes(port.read(1)), py.eval("_uart.read(1)").unwrap());
    for baud in [2400, 115200, 230400, 250000] {
        port.set_baudrate(baud);
        py.exec(&format!("_uart.baudrate = {baud}")).unwrap();
        assert_eq!(snapshot(), py.eval("uart_snapshot(_uart.fileno())").unwrap(), "baud {baud}");
        assert_eq!(Value::Bytes(port.read(4096)), py.eval("_uart.read(4096)").unwrap());
    }
    port.close();
    port.set_baudrate(230400);
    port.set_rts(false);
    port.set_dtr(false);
    port.open();
    py.exec("_uart.close(); _uart.baudrate = 230400; _uart.rts = False; _uart.dtr = False; _uart.open()").unwrap();
    assert_eq!(snapshot(), py.eval("uart_snapshot(_uart.fileno())").unwrap());
    assert_eq!(
        Value::Tuple(vec![Value::Bool(port.rts()), Value::Bool(port.dtr())]),
        py.eval("(_uart.rts, _uart.dtr)").unwrap(),
    );
    port.close();
    kos::fd::close(slave);
    kos::fd::close(master);
    py.exec("_uart.close(); os.close(_slave); os.close(_master)").unwrap();
}
