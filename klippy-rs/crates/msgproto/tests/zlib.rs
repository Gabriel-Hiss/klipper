// Differential tests for msgproto::zlib_decompress against CPython's
// zlib.decompress (the decoder behind MessageParser.process_identify).
//
// Every stream and every expected result comes from the oracle:
// - valid streams: each mcu dictionary in tests/fixtures/dict plus
//   generated inputs (empty, random bytes around deflate block and
//   window boundaries, highly repetitive data, long-distance repeats),
//   compressed by zlib.compress at levels 0, 1, 6 and 9 and by
//   zlib.compressobj with the filtered, huffman-only, rle and fixed
//   strategies. The decoded bytes must equal the original input.
// - malformed streams: header and trailer variants, every truncation
//   point of a few streams, and a fixed-seed corpus of single-bit flips.
//   Where Python raises zlib.error the Rust error text must be
//   str(exc) exactly; where Python decodes, the bytes must match.

use recording::Value;

fn start_oracle() -> Option<pyoracle::Oracle> {
    let oracle = pyoracle::Oracle::start();
    if oracle.is_none() {
        eprintln!("skipping zlib oracle comparison: set KLIPPY_PYTHON to run against the reference");
    }
    oracle
}

fn bytes_of(v: &Value, what: &str) -> Vec<u8> {
    match v {
        Value::Bytes(b) => b.clone(),
        other => panic!("{}: expected bytes from the oracle, got {:?}", what, other),
    }
}

fn int_of(v: &Value, what: &str) -> i64 {
    match v {
        Value::Int(n) => *n,
        other => panic!("{}: expected int from the oracle, got {:?}", what, other),
    }
}

/// Python-side helpers shared by both tests.
const SETUP: &str = r#"
import glob, os, random, zlib

def __result(stream):
    try:
        return ('ok', zlib.decompress(stream))
    except zlib.error as exc:
        return ('err', str(exc))

def __inputs(dict_dir):
    rng = random.Random(20261109)
    out = [open(p, 'rb').read() for p in sorted(glob.glob(os.path.join(dict_dir, '*')))]
    out += [b'', b'a', bytes(range(256))]
    for n in (1, 255, 256, 257, 16383, 16384, 16385, 32767, 32768, 32769,
              65535, 65536, 65537, 100000):
        out.append(bytes(rng.getrandbits(8) for _ in range(n)))
    out.append(b'\x00' * 100000)
    out.append(b'ab' * 50000)
    out.append(b''.join(b'line %d: G1 X%d Y%d F6000\n' % (i, i % 220, i * 7 % 220)
                        for i in range(5000)))
    block = bytes(rng.getrandbits(8) for _ in range(32000))
    out.append(block + b'0123456789' + block)
    block = bytes(rng.getrandbits(8) for _ in range(40000))
    out.append(block + block)
    return out

def __streams(data):
    out = [zlib.compress(data, level) for level in (0, 1, 6, 9)]
    for strategy in (1, 2, 3, 4):  # Z_FILTERED, Z_HUFFMAN_ONLY, Z_RLE, Z_FIXED
        c = zlib.compressobj(6, zlib.DEFLATED, 15, 9, strategy)
        out.append(c.compress(data) + c.flush())
    return out
"#;

#[test]
fn zlib_decompress_valid_streams_match_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(SETUP).unwrap();
    let dict_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../tests/fixtures/dict");
    py.exec(&format!("__in = __inputs({:?})", dict_dir.to_str().unwrap())).unwrap();
    let count = int_of(&py.eval("len(__in)").unwrap(), "input count");
    assert!(count > 40, "expected the fixture dictionaries plus generated inputs, got {}", count);
    for i in 0..count {
        let original = bytes_of(&py.eval(&format!("__in[{}]", i)).unwrap(), "input");
        py.exec(&format!("__st = __streams(__in[{}])", i)).unwrap();
        let nstreams = int_of(&py.eval("len(__st)").unwrap(), "stream count");
        for j in 0..nstreams {
            let stream = bytes_of(&py.eval(&format!("__st[{}]", j)).unwrap(), "stream");
            match msgproto::zlib_decompress(&stream) {
                Ok(out) => assert!(out == original, "input {} stream {}: decoded bytes differ ({} vs {} bytes)", i, j, out.len(), original.len()),
                Err(e) => panic!("input {} stream {} ({} bytes): {}", i, j, stream.len(), e),
            }
        }
    }
}

#[test]
fn zlib_decompress_malformed_streams_match_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(SETUP).unwrap();
    let ke = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../tests/fixtures/dict/ke-gd32f303.dict");
    py.exec(&format!(
        r#"
__ke = open({:?}, 'rb').read()
__text = b''.join(b'line %d: G1 X%d\n' % (i, i % 220) for i in range(400))
__bases = [zlib.compress(__ke, 9), zlib.compress(__ke, 1), zlib.compress(__text, 0),
           __streams(__text)[7], __streams(__ke)[5]]
__cases = []
for b in __bases:
    __cases += [b[:n] for n in range(len(b)) if n < 64 or n % 97 == 0 or n > len(b) - 8]
    __cases += [b + b'trailing', b[:-1] + bytes([b[-1] ^ 1])]
def __hdr(cmf, fdict=0):
    flg = fdict << 5
    return bytes([cmf, flg + (31 - (cmf * 256 + flg) % 31) % 31])
__cases += [b'', b'\x78', b'\x78\x9c', b'\x78\x00\x00', __hdr(0x78, 1) + b'\x00' * 10,
            __hdr(0x78, 1) + b'\x00', __hdr(0x79) + b'\x00' * 8, __hdr(0x88) + b'\x00' * 8,
            b'\x78\x9c\x07' + b'\x00' * 8, b'\x78\x9c\x01\x05\x00\x00\x00' + b'\x00' * 8]
__rng = random.Random(4242)
for b in __bases:
    for _ in range(600):
        f = bytearray(b)
        pos = __rng.randrange(2, len(f))
        f[pos] ^= 1 << __rng.randrange(8)
        __cases.append(bytes(f))
"#,
        ke.to_str().unwrap()
    ))
    .unwrap();
    let count = int_of(&py.eval("len(__cases)").unwrap(), "case count");
    let mut errors = std::collections::BTreeMap::<String, usize>::new();
    for i in 0..count {
        let stream = bytes_of(&py.eval(&format!("__cases[{}]", i)).unwrap(), "case");
        let expected = py.eval(&format!("__result(__cases[{}])", i)).unwrap();
        let Value::Tuple(items) = &expected else { panic!("case {}: oracle result {:?}", i, expected) };
        let actual = msgproto::zlib_decompress(&stream);
        match (&items[0], &items[1]) {
            (Value::Str(tag), Value::Bytes(out)) if tag == "ok" => {
                assert!(actual.as_ref() == Ok(out), "case {} ({} bytes): python decodes, rust gives {:?}", i, stream.len(), actual.as_ref().map(|v| v.len()));
            }
            (Value::Str(tag), Value::Str(msg)) if tag == "err" => {
                assert_eq!(actual.as_ref().err(), Some(msg), "case {} ({} bytes)", i, stream.len());
                *errors.entry(msg.clone()).or_default() += 1;
            }
            other => panic!("case {}: oracle result {:?}", i, other),
        }
    }
    // The corpus must exercise the distinct zlib error paths.
    for msg in [
        "Error -5 while decompressing data: incomplete or truncated stream",
        "Error -3 while decompressing data: incorrect header check",
        "Error -3 while decompressing data: unknown compression method",
        "Error -3 while decompressing data: invalid window size",
        "Error 2 while decompressing data",
        "Error -3 while decompressing data: invalid block type",
        "Error -3 while decompressing data: invalid stored block lengths",
        "Error -3 while decompressing data: incorrect data check",
        "Error -3 while decompressing data: invalid distance too far back",
        "Error -3 while decompressing data: invalid code lengths set",
        "Error -3 while decompressing data: invalid literal/lengths set",
        "Error -3 while decompressing data: invalid distances set",
        "Error -3 while decompressing data: invalid bit length repeat",
        "Error -3 while decompressing data: invalid code -- missing end-of-block",
        "Error -3 while decompressing data: invalid literal/length code",
        "Error -3 while decompressing data: invalid distance code",
    ] {
        assert!(errors.contains_key(msg), "corpus never produced {:?}; seen {:?}", msg, errors);
    }
}
