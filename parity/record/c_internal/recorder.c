// C internal recorder: wrap interposition shims.
// Build with the other C sources; see build.sh and README.md.
//
// Each __wrap_* function logs one JSONL line in the FORMAT.md shape,
// then calls the matching __real_* (the original, un-modified C
// function), so the wrapped call itself is never altered. The shims
// only read their arguments; they never write through a pointer, so
// no in/out buffer is mutated by instrumentation.

#define _GNU_SOURCE
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

static FILE *record_fp;
static uint64_t record_seq;

static void
record_open(void)
{
    if (record_fp)
        return;
    const char *dir = getenv("KLIPPY_RECORD_DIR");
    if (!dir)
        return;
    const char *unit = getenv("KLIPPY_RECORD_C_UNIT");
    char path[4096];
    if (unit)
        snprintf(path, sizeof(path), "%s/%s/c_internal.jsonl", dir, unit);
    else
        snprintf(path, sizeof(path), "%s/c_internal.jsonl", dir);
    record_fp = fopen(path, "a");
}

static void
record_line(const char *unit, const char *function, const char *args_json,
            const char *ret_json, const char *kind)
{
    record_open();
    if (!record_fp)
        return;
    record_seq++;
    const char *scenario = getenv("KLIPPY_RECORD_SCENARIO");
    if (!scenario)
        scenario = "default";
    fprintf(record_fp, "{\"v\":1,\"unit\":\"%s\",\"scenario\":\"%s\","
            "\"seq\":%llu,\"function\":\"%s\",\"args\":[%s],"
            "\"ret\":%s,\"side_effects\":{\"mutated_args\":[],"
            "\"mcu_messages\":[],\"log_lines\":[]},\"object_id\":null,"
            "\"kind\":\"%s\"}\n",
            unit, scenario, (unsigned long long)record_seq, function,
            args_json, ret_json, kind);
    fflush(record_fp);
}

// Hex-float spelling mirrors Python float.hex(): 0x1.xxxp±Y, exact.
static void
hexfloat(double v, char *buf, size_t n)
{
    if (v != v) {
        snprintf(buf, n, "nan");
        return;
    }
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));
    int sign = (bits >> 63) & 1;
    int exp = (int)((bits >> 52) & 0x7ff);
    uint64_t mant = bits & ((1ULL << 52) - 1);
    if (exp == 0 && mant == 0) {
        snprintf(buf, n, "%s0x0.0p+0", sign ? "-" : "");
        return;
    }
    if (exp == 0) {
        // Subnormal: 0x0.xxx p-1022
        char digits[14];
        uint64_t m = mant;
        for (int i = 12; i >= 0; i--) {
            int d = (int)(m & 0xf);
            digits[i] = d < 10 ? '0' + d : 'a' + d - 10;
            m >>= 4;
        }
        digits[13] = 0;
        // Trim trailing zeros.
        int len = 13;
        while (len > 1 && digits[len - 1] == '0')
            len--;
        digits[len] = 0;
        snprintf(buf, n, "%s0x0.%sp-1022", sign ? "-" : "", digits);
        return;
    }
    int e = exp - 1023;
    if (mant == 0) {
        snprintf(buf, n, "%s0x1.0p%+d", sign ? "-" : "", e);
        return;
    }
    char digits[14];
    uint64_t m = mant;
    for (int i = 12; i >= 0; i--) {
        int d = (int)(m & 0xf);
        digits[i] = d < 10 ? '0' + d : 'a' + d - 10;
        m >>= 4;
    }
    digits[13] = 0;
    int len = 13;
    while (len > 1 && digits[len - 1] == '0')
        len--;
    digits[len] = 0;
    snprintf(buf, n, "%s0x1.%sp%+d", sign ? "-" : "", digits, e);
}

static void
float_json(double v, char *buf, size_t n)
{
    char hex[64];
    hexfloat(v, hex, sizeof(hex));
    snprintf(buf, n, "{\"$float\":{\"hex\":\"%s\",\"repr\":\"%.17g\"}}", hex,
             v);
}

static void
oid_json(void *p, char *buf, size_t n)
{
    snprintf(buf, n, "{\"$opaque\":{\"type\":\"void*\",\"addr\":\"%p\"}}", p);
}

// stepcompress_append(sc, sdir, print_time, step_time) -> int
int __real_stepcompress_append(void *sc, int sdir, double print_time,
                               double step_time);
int
__wrap_stepcompress_append(void *sc, int sdir, double print_time,
                           double step_time)
{
    char oid[80], sdir_json[32], pt[1024], st[1024];
    oid_json(sc, oid, sizeof(oid));
    snprintf(sdir_json, sizeof(sdir_json), "%d", sdir);
    float_json(print_time, pt, sizeof(pt));
    float_json(step_time, st, sizeof(st));
    char args_json[4096];
    snprintf(args_json, sizeof(args_json), "%s,%s,%s,%s", oid, sdir_json,
             pt, st);
    int ret = __real_stepcompress_append(sc, sdir, print_time, step_time);
    char ret_json[32];
    snprintf(ret_json, sizeof(ret_json), "%d", ret);
    record_line("stepcompress", "stepcompress_append", args_json, ret_json,
                "c_internal");
    return ret;
}

// stepcompress_flush(sc, move_clock) -> int
int __real_stepcompress_flush(void *sc, uint64_t move_clock);
int
__wrap_stepcompress_flush(void *sc, uint64_t move_clock)
{
    char oid[80], mc[32];
    oid_json(sc, oid, sizeof(oid));
    snprintf(mc, sizeof(mc), "%llu", (unsigned long long)move_clock);
    char args_json[512];
    snprintf(args_json, sizeof(args_json), "%s,%s", oid, mc);
    int ret = __real_stepcompress_flush(sc, move_clock);
    char ret_json[32];
    snprintf(ret_json, sizeof(ret_json), "%d", ret);
    record_line("stepcompress", "stepcompress_flush", args_json, ret_json,
                "c_internal");
    return ret;
}

// stepcompress_history_expire(sc, end_clock) -> void
void __real_stepcompress_history_expire(void *sc, uint64_t end_clock);
void
__wrap_stepcompress_history_expire(void *sc, uint64_t end_clock)
{
    char oid[80], ec[32];
    oid_json(sc, oid, sizeof(oid));
    snprintf(ec, sizeof(ec), "%llu", (unsigned long long)end_clock);
    char args_json[512];
    snprintf(args_json, sizeof(args_json), "%s,%s", oid, ec);
    char ret_json[8] = "null";
    __real_stepcompress_history_expire(sc, end_clock);
    record_line("stepcompress", "stepcompress_history_expire", args_json,
                ret_json, "c_internal");
}

// itersolve_generate_steps(sk, sc, flush_time) -> int32_t
int32_t __real_itersolve_generate_steps(void *sk, void *sc, double flush_time);
int32_t
__wrap_itersolve_generate_steps(void *sk, void *sc, double flush_time)
{
    char sk_json[80], sc_json[80], ft[1024];
    oid_json(sk, sk_json, sizeof(sk_json));
    oid_json(sc, sc_json, sizeof(sc_json));
    float_json(flush_time, ft, sizeof(ft));
    char args_json[4096];
    snprintf(args_json, sizeof(args_json), "%s,%s,%s", sk_json, sc_json, ft);
    int32_t ret = __real_itersolve_generate_steps(sk, sc, flush_time);
    char ret_json[32];
    snprintf(ret_json, sizeof(ret_json), "%d", ret);
    record_line("itersolve", "itersolve_generate_steps", args_json, ret_json,
                "c_internal");
    return ret;
}

// serialqueue_send_batch(sq, cq, msgs) -> void
void __real_serialqueue_send_batch(void *sq, void *cq, void *msgs);
void
__wrap_serialqueue_send_batch(void *sq, void *cq, void *msgs)
{
    char sq_json[80], cq_json[80], msgs_json[80];
    oid_json(sq, sq_json, sizeof(sq_json));
    oid_json(cq, cq_json, sizeof(cq_json));
    oid_json(msgs, msgs_json, sizeof(msgs_json));
    char args_json[1024];
    snprintf(args_json, sizeof(args_json), "%s,%s,%s", sq_json, cq_json,
             msgs_json);
    char ret_json[8] = "null";
    __real_serialqueue_send_batch(sq, cq, msgs);
    record_line("serialqueue", "serialqueue_send_batch", args_json, ret_json,
                "c_internal");
}
