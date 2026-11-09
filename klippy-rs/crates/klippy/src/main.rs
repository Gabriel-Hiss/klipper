//! The klippy host binary: `klippy/klippy.py`'s
//! `if __name__ == '__main__': main()`. It supplies what Python gets from
//! `import`: the module registry and the directory that holds `extras/`
//! and `kinematics/`. The binary is installed in place of `klippy.py`, so
//! that directory is the one it is started from.

mod registry;

use std::io::Write;
use std::sync::Arc;

use klippy_core::klippy::{run_main, HostEnv};

fn main() {
    let argv: Vec<String> = std::env::args().collect();
    let klippy_dir = klippy_dir(argv.first().map(String::as_str).unwrap_or(""));
    kinematics::register_with_toolhead();
    let env = Arc::new(HostEnv { modules: registry::modules(), klippy_dir });
    let exit = run_main(&argv, env);
    let mut stdout = std::io::stdout();
    stdout.write_all(exit.stdout.as_bytes()).expect("write stdout");
    stdout.flush().expect("flush stdout");
    let mut stderr = std::io::stderr();
    stderr.write_all(exit.stderr.as_bytes()).expect("write stderr");
    stderr.flush().expect("flush stderr");
    std::process::exit(exit.code);
}

/// `os.path.dirname(__file__)` of `klippy.py`. Python 3.8 keeps the main
/// script's `__file__` as given on the command line, so the directory comes
/// from `argv[0]`; a bare name found through `PATH` has no directory there
/// and resolves through the executable's own path instead.
fn klippy_dir(argv0: &str) -> String {
    if argv0.contains('/') {
        return posix_dirname(argv0);
    }
    let exe = std::env::current_exe().expect("locate the klippy executable");
    posix_dirname(&exe.to_string_lossy())
}

/// `posixpath.dirname(path)`: everything before the last `/`, with trailing
/// slashes removed unless the head is only slashes.
fn posix_dirname(path: &str) -> String {
    match path.rfind('/') {
        None => String::new(),
        Some(i) => {
            let head = &path[..=i];
            if head.bytes().all(|b| b == b'/') {
                head.to_string()
            } else {
                head.trim_end_matches('/').to_string()
            }
        }
    }
}
