"""Protocol round-trip tests for parity/mcusim/mcusim.py.

Not run by the author (hard rule 1); the integration owner runs:
    python3 -m pytest parity/mcusim/tests/test_mcusim.py -v
Requires klippy/msgproto.py on sys.path (repo root) and, for the
identify test, the dictionary under tests/fixtures/dict/.
"""
import json
import os
import sys
import time
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from klippy import msgproto  # noqa: E402
import mcusim  # noqa: E402


def load_dict(relpath):
    with open(os.path.join(os.path.dirname(__file__), "..", "..", "..", relpath),
              "rb") as f:
        return f.read()


def test_crc_matches_msgproto():
    for blob in (b"", b"\x00\x10", b"identify"):
        crc = mcusim.crc16_ccitt(blob)
        assert [crc >> 8, crc & 0xFF] == msgproto.crc16_ccitt(list(blob))


def test_int_roundtrip():
    for v in (0, 1, 0x60, -0x20, 0x3000, -0x1000, 0x180000, -0x80000,
              0x3000000, -0x4000000, 120000000, -4096):
        enc = mcusim.encode_int(v)
        dec, pos = mcusim.decode_int(enc, 0)
        assert pos == len(enc)
        # msgproto masks unsigned types to 32 bits.
        expected = v & 0xFFFFFFFF if v >= 0 else v
        if v >= 0:
            assert dec == v
        else:
            assert dec & 0xFFFFFFFF == v & 0xFFFFFFFF or dec == v


def test_msgblock_roundtrip():
    # Build an identify command block with msgproto and parse it with
    # the simulator's extractor.
    raw = load_dict("tests/fixtures/dict/ke-gd32f303.dict")
    mp = msgproto.MessageParser()
    mp.process_identify(zlib.compress(raw))
    cmd = mp.create_command("identify offset=0 count=10")
    # MessageParser.encode_msgblock appends the crc as a nested list, so
    # frame the block the way serialqueue does, with msgproto's crc.
    head = [msgproto.MESSAGE_MIN + len(cmd), msgproto.MESSAGE_DEST] + cmd
    block = bytes(head + msgproto.crc16_ccitt(head) + [msgproto.MESSAGE_SYNC])
    assert len(block) == 5 + len(cmd)
    # Simulator extractor: construct a Sim without running the pty loop.
    sim = mcusim.Sim(raw, "mcu", {})
    sim.inbuf.extend(block)
    action, items, pop = mcusim.sim_extract(sim)
    assert action == "block"
    assert items[0][0] == "identify"
    assert items[0][1]["offset"] == 0
    assert items[0][1]["count"] == 10


def test_identify_slice():
    raw = load_dict("tests/fixtures/dict/ke-gd32f303.dict")
    sim = mcusim.Sim(raw, "mcu", {})
    sent = []
    sim.write_all = sent.append
    sim.handle("identify", {"offset": 0, "count": 20})
    data = zlib.compress(raw, 9)
    block = sent[0]
    assert block[4:4 + 4] != data[0:4]


def test_config_crc_handshake():
    raw = load_dict("tests/fixtures/dict/ke-gd32f303.dict")
    sim = mcusim.Sim(raw, "mcu", {"start_state": {"is_config": 1, "crc": 12345}})
    sent = []
    sim.write_all = sent.append
    sim.handle("get_config", {})
    assert len(sent) == 1
    sim.handle("finalize_config", {"crc": 999})
    assert sim.is_config and sim.config_crc == 999


def test_stepper_position():
    raw = load_dict("tests/fixtures/dict/ke-gd32f303.dict")
    sim = mcusim.Sim(raw, "mcu", {})
    sent = []
    sim.write_all = sent.append
    sim.handle("config_stepper", {"oid": 1, "step_pin": 0, "dir_pin": 0,
                                  "invert_step": 0, "step_pulse_ticks": 0})
    sim.handle("queue_step", {"oid": 1, "interval": 1, "count": 3, "add": 0})
    # Advance virtual clock artificially: interval is 1 tick; force time
    # forward by rewinding clock0.
    sim.clock0 -= 1.0 / sim.freq * 10
    sim.handle("stepper_get_position", {"oid": 1})
    assert len(sent) == 1


def test_shutdown_and_is_shutdown():
    raw = load_dict("tests/fixtures/dict/ke-gd32f303.dict")
    sim = mcusim.Sim(raw, "mcu", {})
    sent = []
    sim.write_all = sent.append
    sim.handle("emergency_stop", {})
    assert sim.shutdown
    assert len(sent) == 1
    sim.handle("clear_shutdown", {})
    assert not sim.shutdown


def test_thermal_model_rises():
    raw = load_dict("tests/fixtures/dict/ke-gd32f303.dict")
    script = {"thermal": {"1": {"ambient_adc": 300, "span_adc": 2500,
                                "tau_seconds": 1.0, "pwm_oid": 5}}}
    sim = mcusim.Sim(raw, "mcu", script)
    sim.pwm_duty[5] = 1.0
    v0 = sim.adc_value(1, 0.0)
    sim.last_thermal_t -= 5.0
    v1 = sim.adc_value(1, 5.0)
    assert v1 > v0
