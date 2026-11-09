//! Oracle tests for the `kos::time` local-time helpers against CPython
//! 3.8.2 `time.localtime` / `time.strftime` / `time.asctime`, under several
//! `TZ` settings (POSIX strings, so no tzdata is needed). `TZ` is process
//! state, so one test walks every zone in turn on both sides.

use kos::time::{asctime, localtime, strftime, tzset, Tm};
use recording::Value;

const ZONES: [&str; 5] = [
    "UTC0",
    "EST5EDT,M3.2.0,M11.1.0",
    "CET-1CEST,M3.5.0,M10.5.0/3",
    "<+0530>-5:30",
    "NZST-12NZDT,M9.5.0,M4.1.0/3",
];

const TIMES: [&str; 16] = [
    "0.0",
    "-0.5",
    "-1.0",
    "1.5",
    "86399.999",
    // EST5EDT spring forward (2023-03-12 07:00 UTC) and fall back
    // (2023-11-05 06:00 UTC), one second each side.
    "1678604399.0",
    "1678604400.0",
    "1699163999.0",
    "1699164000.0",
    "1700000000.5",
    "5e9",
    "-2208988801.0",
    "-1e12",
    "1e15",
    "1e17",
    "float('nan')",
];

const FORMATS: [&str; 7] = [
    "%Y-%m-%d",
    "-%Y%m%d_%H%M%S",
    "%Y%m%d_%H%M%S",
    "%a %b %e %H:%M:%S %Y",
    "%Z %z %j %U %w",
    "%%",
    "",
];

/// `tuple(time.localtime(t))`: Python counts months and year days from 1
/// and weekdays from Monday.
fn struct_time(tm: &Tm) -> Value {
    Value::Tuple(
        [
            1900 + i64::from(tm.tm_year),
            i64::from(tm.tm_mon) + 1,
            i64::from(tm.tm_mday),
            i64::from(tm.tm_hour),
            i64::from(tm.tm_min),
            i64::from(tm.tm_sec),
            (i64::from(tm.tm_wday) + 6) % 7,
            i64::from(tm.tm_yday) + 1,
            i64::from(tm.tm_isdst),
        ]
        .into_iter()
        .map(Value::int)
        .collect(),
    )
}

fn eval_time(time: &str) -> f64 {
    match time {
        "float('nan')" => f64::NAN,
        other => other.parse().expect("time literal"),
    }
}

#[test]
fn local_time_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping kos local-time oracle test: KLIPPY_PYTHON is not set");
        return;
    };
    py.exec("import os, time").expect("oracle setup");
    for zone in ZONES {
        std::env::set_var("TZ", zone);
        tzset();
        py.exec(&format!("os.environ['TZ'] = {zone:?}\ntime.tzset()")).expect("python tzset");
        for time in TIMES {
            let label = format!("TZ={zone} t={time}");
            let tm = localtime(eval_time(time));
            let expected = py.eval(&format!("tuple(time.localtime({time}))"));
            let Some(tm) = tm else {
                assert!(expected.is_err(), "{label}: Python converted, kos did not: {expected:?}");
                continue;
            };
            assert_eq!(expected.expect("python localtime"), struct_time(&tm), "{label}: localtime");
            let expected = py.eval(&format!("time.asctime(time.localtime({time}))"));
            assert_eq!(expected.expect("python asctime"), Value::str(&asctime(&tm)), "{label}: asctime");
            for format in FORMATS {
                let expected = py.eval(&format!("time.strftime({format:?}, time.localtime({time}))"));
                assert_eq!(
                    expected.expect("python strftime"),
                    Value::str(&strftime(format, &tm)),
                    "{label}: strftime({format:?})"
                );
            }
        }
    }
}
