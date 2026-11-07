# Unit configuration for the recorders.
#
# Each entry maps a unit id to the functions the Python tracer records.
# The pilot units are msgproto, gcode_parse, configfile_parse,
# gcode_macro_template, trapq, kin_cartesian, itersolve, stepcompress,
# serialqueue, and thermistor. The Python tracer covers the pure units
# (msgproto, gcode_parse, pins, mathutil, shaper_defs, thermistor,
# configfile_parse, gcode_macro_template); the C units (trapq,
# kin_cartesian, itersolve, stepcompress, serialqueue) are recorded by
# the C boundary wrapper and the C internal recorder instead, and appear
# here so the driver knows the unit ids.
#
# `kind` is one of "python", "c_boundary", "c_internal", or "both".
# `module` is the dotted module the traced functions live in; `names`
# lists qualified method names within it (a name is a function name or
# `ClassName.method`).

C_UNITS = (
    "trapq",
    "kin_cartesian",
    "itersolve",
    "stepcompress",
    "serialqueue",
)

UNIT_MAP = {
    "msgproto": {
        "kind": "python",
        "module": "msgproto",
        "names": (
            "crc16_ccitt",
            "MessageParser.parse",
            "MessageParser.dump",
            "MessageParser.check_packet",
            "MessageParser.encode_msgblock",
            "MessageParser.create_command",
            "MessageParser.create_dummy_response",
            "MessageParser.lookup_command",
            "MessageParser.lookup_msgid",
            "MessageParser.get_messages",
            "MessageParser.get_enumerations",
            "MessageParser.get_constants",
            "MessageParser.get_constant",
            "MessageParser.get_constant_float",
            "MessageParser.get_constant_int",
            "MessageParser.get_version_info",
            "MessageParser.get_raw_data_dictionary",
            "MessageParser.format_params",
            "MessageParser.fill_enumerations",
            "MessageParser.process_identify",
        ),
    },
    "gcode_parse": {
        "kind": "python",
        "module": "gcode",
        "names": (
            "GCodeCommand.get",
            "GCodeCommand.get_int",
            "GCodeCommand.get_float",
            "GCodeCommand.get_raw_command_parameters",
            "GCodeDispatch.is_traditional_gcode",
            "GCodeDispatch._process_commands",
            "GCodeDispatch._get_extended_params",
            "GCodeDispatch.create_gcode_command",
            "GCodeIO._process_data",
        ),
    },
    "pins": {
        "kind": "python",
        "module": "pins",
        "names": (
            "PinResolver.reserve_pin",
            "PinResolver.alias_pin",
            "PinResolver.update_command",
            "PrinterPins.parse_pin",
            "PrinterPins.lookup_pin",
            "PrinterPins.setup_pin",
            "PrinterPins.reset_pin_sharing",
            "PrinterPins.register_chip",
            "PrinterPins.allow_multi_use_pin",
        ),
    },
    "mathutil": {
        "kind": "python",
        "module": "mathutil",
        "names": (
            "coordinate_descent",
            "trilateration",
            "matrix_cross",
            "matrix_dot",
            "matrix_magsq",
            "matrix_add",
            "matrix_sub",
            "matrix_mul",
            "mat_transp",
            "mat_mat_mul",
            "mat_mul_transp",
            "gaussian_solve",
            "pseudo_inverse",
            "solve_linear_equations",
        ),
    },
    "shaper_defs": {
        "kind": "python",
        "module": "extras.shaper_defs",
        "names": (
            "get_none_shaper",
            "get_zv_shaper",
            "get_zvd_shaper",
            "get_mzv_coeffs",
            "get_mzv_shaper",
            "get_ei_shaper",
            "get_2hump_ei_shaper",
            "get_3hump_ei_shaper",
            "get_shaper_cfg",
            "init_shaper",
        ),
    },
    "thermistor": {
        "kind": "python",
        "module": "extras.thermistor",
        "names": (
            "Thermistor.setup_coefficients",
            "Thermistor.setup_coefficients_beta",
            "Thermistor.calc_temp",
            "Thermistor.calc_adc",
        ),
    },
    "configfile_parse": {
        "kind": "python",
        "module": "configfile",
        "names": (
            "ConfigWrapper._get_wrapper",
            "ConfigWrapper.get",
            "ConfigWrapper.getint",
            "ConfigWrapper.getfloat",
            "ConfigWrapper.getboolean",
            "ConfigWrapper.getchoice",
            "ConfigWrapper.getlists",
            "ConfigWrapper.getlist",
            "ConfigWrapper.getintlist",
            "ConfigWrapper.getfloatlist",
            "ConfigWrapper.getsection",
            "ConfigWrapper.get_prefix_sections",
            "ConfigWrapper.get_prefix_options",
        ),
    },
    "gcode_macro_template": {
        "kind": "python",
        "module": "extras.gcode_macro",
        "names": (
            "GetStatusWrapper.__getitem__",
            "TemplateWrapper.render",
            "TemplateWrapper.run_gcode_from_command",
            "PrinterGCodeMacro.load_template",
            "PrinterGCodeMacro.create_template_context",
        ),
    },
    "trapq": {
        "kind": "c_boundary",
        "module": "chelper",
        "names": (
            "trapq_alloc",
            "trapq_free",
            "trapq_append",
            "trapq_finalize_moves",
            "trapq_set_position",
            "trapq_extract_old",
        ),
    },
    "kin_cartesian": {
        "kind": "c_boundary",
        "module": "chelper",
        "names": (
            "cartesian_stepper_alloc",
        ),
    },
    "itersolve": {
        "kind": "both",
        "module": "chelper",
        "names": (
            "itersolve_check_active",
            "itersolve_is_active_axis",
            "itersolve_set_trapq",
            "itersolve_get_trapq",
            "itersolve_calc_position_from_coord",
            "itersolve_set_position",
            "itersolve_get_commanded_pos",
            "itersolve_get_gen_steps_pre_active",
            "itersolve_get_gen_steps_post_active",
        ),
    },
    "stepcompress": {
        "kind": "c_boundary",
        "module": "chelper",
        "names": (
            "stepcompress_fill",
            "stepcompress_set_invert_sdir",
            "stepcompress_reset",
            "stepcompress_set_last_position",
            "stepcompress_find_past_position",
            "stepcompress_extract_old",
        ),
    },
    "serialqueue": {
        "kind": "c_boundary",
        "module": "chelper",
        "names": (
            "serialqueue_alloc",
            "serialqueue_exit",
            "serialqueue_free",
            "serialqueue_alloc_commandqueue",
            "serialqueue_free_commandqueue",
            "serialqueue_send",
            "serialqueue_pull",
            "serialqueue_set_wire_frequency",
            "serialqueue_set_receive_window",
            "serialqueue_set_clock_est",
            "serialqueue_get_stats",
            "serialqueue_extract_old",
        ),
    },
}

# Pilot units for the acceptance run, in reporting order.
PILOT_UNITS = (
    "msgproto",
    "gcode_parse",
    "configfile_parse",
    "gcode_macro_template",
    "trapq",
    "kin_cartesian",
    "itersolve",
    "stepcompress",
    "serialqueue",
    "thermistor",
)

# C-to-C internal edges captured by the recording c_helper.so.
# Each entry is (callee unit, callee function, caller edge).
C_INTERNAL_EDGES = (
    ("stepcompress", "stepcompress_append",
     "itersolve.c itersolve_gen_steps_range -> stepcompress_append"),
    ("stepcompress", "stepcompress_commit",
     "itersolve.c itersolve_gen_steps_range -> stepcompress_commit"),
    ("stepcompress", "stepcompress_flush",
     "steppersync.c se_generate_steps -> stepcompress_flush"),
    ("stepcompress", "stepcompress_history_expire",
     "steppersync.c se_generate_steps -> stepcompress_history_expire"),
    ("itersolve", "itersolve_generate_steps",
     "steppersync.c se_generate_steps (background thread)"
     " -> itersolve_generate_steps"),
    ("serialqueue", "serialqueue_send_batch",
     "steppersync.c steppersync_flush -> serialqueue_send_batch"),
)


def unit_names(unit_id):
    """Return the (module, names) config for a unit id."""
    return UNIT_MAP[unit_id]
