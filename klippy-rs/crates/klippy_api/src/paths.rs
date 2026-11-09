//! Runtime location of the klippy source tree.
//!
//! Python modules find their bundled files through
//! `os.path.dirname(__file__)` (`heaters.py:264`, `display.py:118`,
//! `menu.py:716`, `webhooks.py:377`). The port records the directory that
//! holds `klippy.py`, `extras/` and `kinematics/` when `klippy::run_main`
//! starts, and resolves those files from it at run time.

use std::sync::LazyLock;

use kos::sync::Mutex;

static KLIPPY_DIR: LazyLock<Mutex<Option<String>>> = LazyLock::new(|| Mutex::new(None));

/// Records `os.path.dirname(__file__)` of `klippy.py` for this process.
/// `klippy::run_main` calls it before any module loads; tests that load
/// modules reading bundled files call it with the checkout's `klippy/`.
pub fn set_klippy_dir(dir: &str) {
    *KLIPPY_DIR.lock() = Some(dir.to_string());
}

/// `os.path.dirname(__file__)` of `klippy.py`.
///
/// # Panics
///
/// When no host recorded it with [`set_klippy_dir`].
pub fn klippy_dir() -> String {
    KLIPPY_DIR
        .lock()
        .clone()
        .expect("klippy source directory not recorded; klippy::run_main sets it")
}

/// `os.path.join(klippy_dir(), *parts)`.
pub fn klippy_path(parts: &[&str]) -> String {
    let mut out = klippy_dir();
    for part in parts {
        posix_join(&mut out, part);
    }
    out
}

/// `posixpath.join(path, part)` in place: an absolute `part` restarts the
/// path, otherwise a `/` separates it from a non-empty `path` that does not
/// already end with one.
fn posix_join(path: &mut String, part: &str) {
    if part.starts_with('/') {
        path.clear();
    } else if !path.is_empty() && !path.ends_with('/') {
        path.push('/');
    }
    path.push_str(part);
}
