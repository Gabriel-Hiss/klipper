// Compiles the unchanged klippy/chelper C sources into a static library so Rust tests can call
// the reference implementation side by side with the port. Flags follow
// klippy/chelper/__init__.py (-O2, -mfpmath=sse -msse2 on x86) without -flto/-fwhole-program,
// so non-static functions stay externally visible; neither flag changes floating point results.
use std::path::PathBuf;

const SOURCES: &[&str] = &[
    "pyhelper.c", "serialqueue.c", "stepcompress.c", "steppersync.c", "itersolve.c", "trapq.c",
    "pollreactor.c", "msgblock.c", "trdispatch.c", "kin_cartesian.c", "kin_corexy.c",
    "kin_corexz.c", "kin_delta.c", "kin_deltesian.c", "kin_polar.c", "kin_rotary_delta.c",
    "kin_winch.c", "kin_extruder.c", "kin_shaper.c", "kin_idex.c", "kin_generic.c",
];

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let chelper = manifest.join("../../../klippy/chelper").canonicalize().unwrap();
    let mut build = cc::Build::new();
    build.opt_level(2).warnings(false).flag("-fno-lto");
    if std::env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("x86_64") {
        build.flag("-mfpmath=sse").flag("-msse2");
    }
    for src in SOURCES {
        let path = chelper.join(src);
        println!("cargo:rerun-if-changed={}", path.display());
        build.file(path);
    }
    for hdr in std::fs::read_dir(&chelper).unwrap().flatten() {
        if hdr.path().extension().is_some_and(|e| e == "h") {
            println!("cargo:rerun-if-changed={}", hdr.path().display());
        }
    }
    build.compile("chelper_ref");
    println!("cargo:rustc-link-lib=m");
    println!("cargo:rustc-link-lib=pthread");
}
