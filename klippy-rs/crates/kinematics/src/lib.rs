//! Ports of klippy kinematics.
pub mod cartesian;
pub mod corexy;
pub mod corexz;
pub mod delta;
pub mod deltesian;
pub mod extruder;
pub mod generic_cartesian;
pub mod hybrid_corexy;
pub mod hybrid_corexz;
pub mod idex_modes;
pub mod kinematic_stepper;
pub mod none;
pub mod polar;
pub mod rotary_delta;
pub mod winch;

/// Registers every ported kinematics module with the toolhead's loader
/// registry, under the name `[printer] kinematics:` selects. Python's
/// `ToolHead.__init__` imports `kinematics.<name>` by that name; the core
/// crate cannot import this crate, so the binary calls this once before the
/// printer loads its config. A name that is not registered fails exactly
/// like a missing Python module ("Error loading kinematics '<name>'").
pub fn register_with_toolhead() {
    use klippy_core::toolhead::register_kinematics_loader;
    register_kinematics_loader("cartesian", cartesian::load_kinematics);
    register_kinematics_loader("corexy", corexy::load_kinematics);
    register_kinematics_loader("corexz", corexz::load_kinematics);
    register_kinematics_loader("delta", delta::load_kinematics);
    register_kinematics_loader("deltesian", deltesian::load_kinematics);
    register_kinematics_loader("hybrid_corexy", hybrid_corexy::load_kinematics);
    register_kinematics_loader("hybrid_corexz", hybrid_corexz::load_kinematics);
    register_kinematics_loader("none", none::load_kinematics);
    register_kinematics_loader("polar", polar::load_kinematics);
    register_kinematics_loader("rotary_delta", rotary_delta::load_kinematics);
    register_kinematics_loader("winch", winch::load_kinematics);
    // `toolhead.add_printer_objects` calls `kinematics.extruder.add_printer_objects`.
    klippy_core::toolhead::register_extruder_objects(extruder::add_printer_objects);
}
