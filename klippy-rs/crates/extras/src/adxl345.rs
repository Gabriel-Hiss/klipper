//! Port of `klippy/extras/adxl345.py`: the ADXL345 SPI accelerometer, the
//! `AccelQueryHelper` / `AccelCommandHelper` helpers shared with the other
//! accelerometer modules (`lis2dw`, `mpu9250`, `icm20948`, `bmi160`,
//! `lis3dh`) and `read_axes_map`.
//!
//! `ADXL345.__init__` registers the G-code commands through
//! `AccelCommandHelper(config, self)` before it reads `axes_map`, `rate` and
//! the SPI bus. The `chip` handed to the helper must therefore exist before
//! those values do, so [`ADXL345`] holds them in a [`OnceLock`] that
//! `ADXL345::new` fills right after the helper is registered. Using the chip
//! before that point raises the `AttributeError` Python raises for the missing
//! attribute.
//!
//! `AccelQueryHelper.write_to_file` forks a daemon process in Python. The
//! port writes the file on a detached `kos` thread; the process priority
//! change (`os.nice(20)`) has no `kos` equivalent and is not reproduced.

use crate::bulk_sensor::{BatchBulkHelper, FixedFreqReader};
use crate::bus::{mcu_spi_from_config, McuSpi, McuSpiApi};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts, GetListOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GetOpts, GetParser};
use klippy_api::log;
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandWrapper, Mcu, QuerySendOpts, SendOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, GCell, ObjectRef};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, OnceLock, Weak};

// ---------------------------------------------------------------------------
// Register map and constants (`adxl345.py:9-28`)
// ---------------------------------------------------------------------------

/// `REG_DEVID`.
pub const REG_DEVID: i64 = 0x00;
/// `REG_BW_RATE`.
pub const REG_BW_RATE: i64 = 0x2C;
/// `REG_POWER_CTL`.
pub const REG_POWER_CTL: i64 = 0x2D;
/// `REG_DATA_FORMAT`.
pub const REG_DATA_FORMAT: i64 = 0x31;
/// `REG_FIFO_CTL`.
pub const REG_FIFO_CTL: i64 = 0x38;
/// `REG_MOD_READ`.
pub const REG_MOD_READ: i64 = 0x80;
/// `REG_MOD_MULTI`.
pub const REG_MOD_MULTI: i64 = 0x40;
/// `ADXL345_DEV_ID`.
pub const ADXL345_DEV_ID: i64 = 0xE5;
/// `SET_FIFO_CTL`.
pub const SET_FIFO_CTL: i64 = 0x90;
/// `FREEFALL_ACCEL`.
pub const FREEFALL_ACCEL: f64 = 9.80665 * 1000.;
/// `SCALE_XY` (1 / 265 at 3.3V, mg per LSB).
pub const SCALE_XY: f64 = 0.003774 * FREEFALL_ACCEL;
/// `SCALE_Z` (1 / 256 at 3.3V, mg per LSB).
pub const SCALE_Z: f64 = 0.003906 * FREEFALL_ACCEL;
/// `BATCH_UPDATES` (`adxl345.py:199`).
pub const BATCH_UPDATES: f64 = 0.100;

/// `QUERY_RATES` (`adxl345.py:18-21`): `QUERY_RATES.get(rate)`.
pub fn query_rate(rate: i64) -> Option<i64> {
    match rate {
        25 => Some(0x8),
        50 => Some(0x9),
        100 => Some(0xA),
        200 => Some(0xB),
        400 => Some(0xC),
        800 => Some(0xD),
        1600 => Some(0xE),
        3200 => Some(0xF),
        _ => None,
    }
}

/// `Accel_Measurement` (`adxl345.py:30`): `(time, accel_x, accel_y, accel_z)`.
pub type AccelMeasurement = (f64, f64, f64, f64);

// ---------------------------------------------------------------------------
// Python exception helpers
// ---------------------------------------------------------------------------

fn internal(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.into() }
}

fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant adxl345 format renders")
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn int_to_i64(n: &PyInt) -> KResult<i64> {
    i64::try_from(&n.0).map_err(|_| {
        internal("OverflowError", "Python int too large to convert to C long")
    })
}

/// `d[key]` on a dict with string keys (`KeyError` carries `repr(key)`).
fn dict_item<'a>(dict: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match dict {
        PyValue::Dict(d) => d
            .get(&PyValue::Str(key.to_string()))
            .ok_or_else(|| internal("KeyError", pycompat::text::str_repr(key))),
        other => Err(internal(
            "TypeError",
            pct(
                "'%s' object is not subscriptable",
                &[PctValue::Str(pycompat::value::type_name(other).to_string())],
            ),
        )),
    }
}

/// Elements of a list or tuple (`for x in seq` / `len(seq)` / `seq[i]`).
fn sequence_items(value: &PyValue) -> KResult<&[PyValue]> {
    match value {
        PyValue::List(items) | PyValue::Tuple(items) => Ok(items),
        other => Err(internal(
            "TypeError",
            pct(
                "'%s' object is not iterable",
                &[PctValue::Str(pycompat::value::type_name(other).to_string())],
            ),
        )),
    }
}

/// `a, b, ... = seq` with `N` targets.
fn unpack<const N: usize>(value: &PyValue) -> KResult<&[PyValue]> {
    let items = sequence_items(value)?;
    if items.len() > N {
        return Err(internal(
            "ValueError",
            format!("too many values to unpack (expected {N})"),
        ));
    }
    if items.len() < N {
        return Err(internal(
            "ValueError",
            format!("not enough values to unpack (expected {N}, got {})", items.len()),
        ));
    }
    Ok(items)
}

/// A number used as a float (`float(x)` for the int/float values samples carry).
fn as_float(value: &PyValue) -> KResult<f64> {
    value.to_f64().ok_or_else(|| {
        internal(
            "TypeError",
            pct(
                "must be real number, not %s",
                &[PctValue::Str(pycompat::value::type_name(value).to_string())],
            ),
        )
    })
}

fn as_int(value: &PyValue) -> KResult<i64> {
    match value {
        PyValue::Int(n) => int_to_i64(n),
        PyValue::Bool(b) => Ok(*b as i64),
        other => Err(internal(
            "TypeError",
            pct(
                "unsupported operand type(s) for &: '%s' and 'int'",
                &[PctValue::Str(pycompat::value::type_name(other).to_string())],
            ),
        )),
    }
}

fn lookup_toolhead(printer: &Printer) -> KResult<Arc<dyn ToolHeadApi>> {
    Ok(lookup_interface::<dyn ToolHeadApi>(&**printer, "toolhead", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None"))
}

fn lookup_gcode(printer: &Printer) -> KResult<Arc<dyn GCodeApi>> {
    Ok(lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None"))
}

// ---------------------------------------------------------------------------
// `AccelQueryHelper` (`adxl345.py:34-104`)
// ---------------------------------------------------------------------------

/// `msg['data']`.
fn msg_data(msg: &PyValue) -> KResult<&[PyValue]> {
    sequence_items(dict_item(msg, "data")?)
}

/// `row[0]`.
fn row_time(row: &PyValue) -> KResult<f64> {
    let items = sequence_items(row)?;
    match items.first() {
        Some(t) => as_float(t),
        None => Err(internal("IndexError", "tuple index out of range")),
    }
}

/// `samp_time, x, y, z = row`.
fn row_measurement(row: &PyValue) -> KResult<AccelMeasurement> {
    let items = unpack::<4>(row)?;
    Ok((
        as_float(&items[0])?,
        as_float(&items[1])?,
        as_float(&items[2])?,
        as_float(&items[3])?,
    ))
}

struct QueryState {
    is_finished: bool,
    request_start_time: f64,
    request_end_time: f64,
    msgs: Vec<PyValue>,
    samples: Vec<AccelMeasurement>,
}

impl QueryState {
    /// The body of `get_samples` after the `if not self.msgs` shortcut.
    fn collect_samples(&self) -> KResult<Vec<AccelMeasurement>> {
        let mut samples = Vec::new();
        for msg in &self.msgs {
            for row in msg_data(msg)? {
                let (samp_time, x, y, z) = row_measurement(row)?;
                if samp_time < self.request_start_time {
                    continue;
                }
                if samp_time > self.request_end_time {
                    break;
                }
                samples.push((samp_time, x, y, z));
            }
        }
        Ok(samples)
    }
}

/// Port of the `AccelQueryHelper` class: collects the batches of one client
/// of the bulk reader and returns the samples inside the request window.
pub struct AccelQueryHelper {
    printer: Printer,
    state: GCell<QueryState>,
}

impl AccelQueryHelper {
    /// `AccelQueryHelper.__init__`.
    pub fn new(printer: Printer) -> KResult<AccelQueryHelper> {
        let print_time = lookup_toolhead(&printer)?.get_last_move_time();
        Ok(AccelQueryHelper {
            printer,
            state: GCell::new(QueryState {
                is_finished: false,
                request_start_time: print_time,
                request_end_time: print_time,
                msgs: Vec::new(),
                samples: Vec::new(),
            }),
        })
    }

    /// `finish_measurements`. Pauses the reactor.
    pub fn finish_measurements(&self) -> KResult<()> {
        let toolhead = lookup_toolhead(&self.printer)?;
        self.state.borrow_mut().request_end_time = toolhead.get_last_move_time();
        toolhead.wait_moves();
        self.state.borrow_mut().is_finished = true;
        Ok(())
    }

    /// `handle_batch(msg)`: the client callback of the bulk reader.
    pub fn handle_batch(&self, msg: PyValue) -> bool {
        let mut st = self.state.borrow_mut();
        if st.is_finished {
            return false;
        }
        if st.msgs.len() >= 10000 {
            // Avoid filling up memory with too many samples
            return false;
        }
        st.msgs.push(msg);
        true
    }

    /// `has_valid_samples`.
    pub fn has_valid_samples(&self) -> KResult<bool> {
        let st = self.state.borrow();
        for msg in &st.msgs {
            let data = msg_data(msg)?;
            let first_sample_time = match data.first() {
                Some(row) => row_time(row)?,
                None => return Err(internal("IndexError", "list index out of range")),
            };
            let last_sample_time = row_time(&data[data.len() - 1])?;
            if first_sample_time > st.request_end_time
                || last_sample_time < st.request_start_time
            {
                continue;
            }
            return Ok(true);
        }
        Ok(false)
    }

    /// `get_samples`.
    pub fn get_samples(&self) -> KResult<Vec<AccelMeasurement>> {
        let mut st = self.state.borrow_mut();
        if st.msgs.is_empty() {
            return Ok(st.samples.clone());
        }
        let samples = st.collect_samples()?;
        st.samples = samples.clone();
        Ok(samples)
    }

    /// `write_to_file`: the file is written on a detached thread.
    pub fn write_to_file(&self, filename: &str) {
        // `self.samples or self.get_samples()` runs in the forked child in
        // Python, so the parent's cached samples stay untouched.
        let rows = {
            let st = self.state.borrow();
            if !st.samples.is_empty() {
                Ok(st.samples.clone())
            } else if st.msgs.is_empty() {
                Ok(Vec::new())
            } else {
                st.collect_samples()
            }
        };
        let filename = filename.to_string();
        kos::thread::spawn(move || {
            use std::io::Write;
            // The child opens (truncates) the file before it can fail.
            let Ok(mut f) = std::fs::File::create(&filename) else { return };
            let Ok(rows) = rows else { return };
            let mut text = String::from("#time,accel_x,accel_y,accel_z\n");
            for (t, accel_x, accel_y, accel_z) in rows {
                text.push_str(&pct(
                    "%.6f,%.6f,%.6f,%.6f\n",
                    &[
                        PctValue::Float(t),
                        PctValue::Float(accel_x),
                        PctValue::Float(accel_y),
                        PctValue::Float(accel_z),
                    ],
                ));
            }
            let _ = f.write_all(text.as_bytes());
        })
        .detach();
    }
}

// ---------------------------------------------------------------------------
// `read_axes_map` (`adxl345.py:191-197`)
// ---------------------------------------------------------------------------

fn axes_entry(name: &str, scale_x: f64, scale_y: f64, scale_z: f64) -> Option<(usize, f64)> {
    match name {
        "x" => Some((0, scale_x)),
        "y" => Some((1, scale_y)),
        "z" => Some((2, scale_z)),
        "-x" => Some((0, -scale_x)),
        "-y" => Some((1, -scale_y)),
        "-z" => Some((2, -scale_z)),
        _ => None,
    }
}

/// `read_axes_map(config, scale_x, scale_y, scale_z)`: `(index, scale)` for
/// each of the three output axes.
pub fn read_axes_map(
    config: &Config,
    scale_x: f64,
    scale_y: f64,
    scale_z: f64,
) -> KResult<Vec<(usize, f64)>> {
    let axes_map = config
        .getlist(
            "axes_map",
            GetListOpts {
                default: Some(Some(vec!["x".to_string(), "y".to_string(), "z".to_string()])),
                count: Some(3),
                ..GetListOpts::default()
            },
        )?
        .expect("getlist with a default returns a list");
    if axes_map.iter().any(|a| axes_entry(a, scale_x, scale_y, scale_z).is_none()) {
        return Err(KlippyError::Config("Invalid axes_map parameter".to_string()));
    }
    Ok(axes_map
        .iter()
        .map(|a| axes_entry(a.trim(), scale_x, scale_y, scale_z).expect("validated above"))
        .collect())
}

// ---------------------------------------------------------------------------
// `AccelCommandHelper` (`adxl345.py:107-188`)
// ---------------------------------------------------------------------------

/// The chip methods `AccelCommandHelper` and `resonance_tester` call on an
/// accelerometer object.
pub trait AccelChipApi: Send + Sync + 'static {
    /// `start_internal_client()`. Pauses the reactor.
    fn start_internal_client(&self) -> KResult<Arc<AccelQueryHelper>>;
    /// `read_reg(reg)`. Pauses the reactor.
    fn read_reg(&self, reg: i64) -> KResult<i64>;
    /// `set_reg(reg, val)`. Pauses the reactor.
    fn set_reg(&self, reg: i64, val: i64) -> KResult<()>;
    /// The `name` attribute (last word of the config section name).
    fn name(&self) -> String;
}

/// Port of the `AccelCommandHelper` class: registers the
/// `ACCELEROMETER_*` mux commands for one chip.
pub struct AccelCommandHelper {
    printer: Printer,
    chip: Arc<dyn AccelChipApi>,
    bg_client: GCell<Option<Arc<AccelQueryHelper>>>,
    base_name: String,
    name: String,
}

impl AccelCommandHelper {
    /// `AccelCommandHelper.__init__`.
    pub fn new(
        config: &Config,
        chip: Arc<dyn AccelChipApi>,
    ) -> KResult<Arc<AccelCommandHelper>> {
        let printer = config.get_printer();
        let section = config.get_name();
        let name_parts: Vec<&str> = section.split_whitespace().collect();
        let helper = Arc::new(AccelCommandHelper {
            printer,
            chip,
            bg_client: GCell::new(None),
            base_name: name_parts[0].to_string(),
            name: name_parts[name_parts.len() - 1].to_string(),
        });
        match helper.register_commands(Some(&helper.name)) {
            Err(KlippyError::Config(_)) => {
                return Err(KlippyError::Config(pct(
                    "Accelerometer with name '%s' already defined",
                    &[PctValue::Str(helper.name.clone())],
                )));
            }
            other => other?,
        }
        if name_parts.len() == 1 {
            // Try to register default mux commands for the accelerometer
            // without explicit name. If default accelerometer has already
            // been registered, raise an error.
            match helper.register_commands(None) {
                Err(KlippyError::Config(_)) => {
                    return Err(KlippyError::Config(pct(
                        "Default accelerometer already defined; section '%s' must \
                         include an additional name, e.g. '%s second_accelerometer'",
                        &[
                            PctValue::Str(helper.base_name.clone()),
                            PctValue::Str(helper.base_name.clone()),
                        ],
                    )));
                }
                other => other?,
            }
        }
        Ok(helper)
    }

    /// `register_commands(name)`.
    fn register_commands(self: &Arc<Self>, name: Option<&str>) -> KResult<()> {
        let gcode = lookup_gcode(&self.printer)?;
        let helper = Arc::clone(self);
        gcode.register_mux_command(
            "ACCELEROMETER_MEASURE",
            "CHIP",
            name,
            Box::new(move |gcmd| helper.cmd_accelerometer_measure(gcmd)),
            Some("Start/stop accelerometer"),
        )?;
        let helper = Arc::clone(self);
        gcode.register_mux_command(
            "ACCELEROMETER_QUERY",
            "CHIP",
            name,
            Box::new(move |gcmd| helper.cmd_accelerometer_query(gcmd)),
            Some("Query accelerometer for the current values"),
        )?;
        let helper = Arc::clone(self);
        gcode.register_mux_command(
            "ACCELEROMETER_DEBUG_READ",
            "CHIP",
            name,
            Box::new(move |gcmd| helper.cmd_accelerometer_debug_read(gcmd)),
            Some("Query register (for debugging)"),
        )?;
        let helper = Arc::clone(self);
        gcode.register_mux_command(
            "ACCELEROMETER_DEBUG_WRITE",
            "CHIP",
            name,
            Box::new(move |gcmd| helper.cmd_accelerometer_debug_write(gcmd)),
            Some("Set register (for debugging)"),
        )
    }

    /// `cmd_ACCELEROMETER_MEASURE`. Pauses the reactor.
    pub fn cmd_accelerometer_measure(&self, gcmd: &GCmd) -> KResult<()> {
        if self.bg_client.borrow().is_none() {
            // Start measurements
            let client = self.chip.start_internal_client()?;
            *self.bg_client.borrow_mut() = Some(client);
            gcmd.respond_info("accelerometer measurements started", true);
            return Ok(());
        }
        // End measurements
        let default_name = {
            let datestr = klippy_core::configfile::datestr_from_wall_time(kos::time::wall_time());
            datestr[1..].to_string()
        };
        let name = match gcmd.get(
            "NAME",
            GetOpts {
                default: Some(PyValue::Str(default_name)),
                parser: GetParser::Str,
                ..GetOpts::default()
            },
        )? {
            PyValue::Str(s) => s,
            other => other.str(),
        };
        if !py_isalnum(&name.replace(['-', '_'], "")) {
            return Err(KlippyError::Command("Invalid NAME parameter".to_string()));
        }
        let bg_client = self.bg_client.replace(None).expect("checked above");
        bg_client.finish_measurements()?;
        // Write data to file
        let filename = if self.base_name == self.name {
            pct(
                "/tmp/%s-%s.csv",
                &[PctValue::Str(self.base_name.clone()), PctValue::Str(name)],
            )
        } else {
            pct(
                "/tmp/%s-%s-%s.csv",
                &[
                    PctValue::Str(self.base_name.clone()),
                    PctValue::Str(self.name.clone()),
                    PctValue::Str(name),
                ],
            )
        };
        bg_client.write_to_file(&filename);
        gcmd.respond_info(
            &pct("Writing raw accelerometer data to %s file", &[PctValue::Str(filename)]),
            true,
        );
        Ok(())
    }

    /// `cmd_ACCELEROMETER_QUERY`. Pauses the reactor.
    pub fn cmd_accelerometer_query(&self, gcmd: &GCmd) -> KResult<()> {
        let aclient = self.chip.start_internal_client()?;
        lookup_toolhead(&self.printer)?.dwell(1.);
        aclient.finish_measurements()?;
        let values = aclient.get_samples()?;
        let Some(&(_, accel_x, accel_y, accel_z)) = values.last() else {
            return Err(KlippyError::Command(
                "No accelerometer measurements found".to_string(),
            ));
        };
        gcmd.respond_info(
            &pct(
                "accelerometer values (x, y, z): %.6f, %.6f, %.6f",
                &[
                    PctValue::Float(accel_x),
                    PctValue::Float(accel_y),
                    PctValue::Float(accel_z),
                ],
            ),
            true,
        );
        Ok(())
    }

    /// `gcmd.get(name, minval=.., maxval=.., parser=lambda x: int(x, 0))`.
    fn get_register_arg(gcmd: &GCmd, name: &str, maxval: i64) -> KResult<i64> {
        match gcmd.get(
            name,
            GetOpts {
                default: None,
                parser: GetParser::IntAutoBase,
                minval: Some(py_int(0)),
                maxval: Some(py_int(maxval)),
                ..GetOpts::default()
            },
        )? {
            PyValue::Int(n) => int_to_i64(&n),
            other => as_int(&other),
        }
    }

    /// `cmd_ACCELEROMETER_DEBUG_READ`. Pauses the reactor.
    pub fn cmd_accelerometer_debug_read(&self, gcmd: &GCmd) -> KResult<()> {
        let reg = Self::get_register_arg(gcmd, "REG", 127)?;
        let val = self.chip.read_reg(reg)?;
        gcmd.respond_info(
            &pct("Accelerometer REG[0x%x] = 0x%x", &[pct_int(reg), pct_int(val)]),
            true,
        );
        Ok(())
    }

    /// `cmd_ACCELEROMETER_DEBUG_WRITE`. Pauses the reactor.
    pub fn cmd_accelerometer_debug_write(&self, gcmd: &GCmd) -> KResult<()> {
        let reg = Self::get_register_arg(gcmd, "REG", 127)?;
        let val = Self::get_register_arg(gcmd, "VAL", 255)?;
        self.chip.set_reg(reg, val)
    }
}

/// `str.isalnum()`: non-empty and every character alphanumeric.
fn py_isalnum(s: &str) -> bool {
    !s.is_empty() && s.chars().all(|c| c.is_alphanumeric())
}

// ---------------------------------------------------------------------------
// `ADXL345` (`adxl345.py:202-311`)
// ---------------------------------------------------------------------------

/// Everything `ADXL345.__init__` assigns after `AccelCommandHelper` ran.
struct Setup {
    axes_map: Vec<(usize, f64)>,
    data_rate: i64,
    spi: Arc<McuSpi>,
    mcu: Mcu,
    oid: i64,
    query_adxl345_cmd: GCell<Option<CommandWrapper>>,
    ffreader: FixedFreqReader,
    last_error_count: GCell<i64>,
    batch_bulk: BatchBulkHelper,
    name: String,
}

/// Port of the `ADXL345` class.
pub struct ADXL345 {
    printer: Printer,
    setup: OnceLock<Setup>,
}

impl ADXL345 {
    /// `ADXL345.__init__`.
    pub fn new(config: &Config) -> KResult<Arc<ADXL345>> {
        let printer = config.get_printer();
        let dev = Arc::new(ADXL345 { printer: Arc::clone(&printer), setup: OnceLock::new() });
        AccelCommandHelper::new(config, Arc::clone(&dev) as Arc<dyn AccelChipApi>)?;
        let axes_map = read_axes_map(config, SCALE_XY, SCALE_XY, SCALE_Z)?;
        let data_rate = config
            .getint(
                "rate",
                GetIntOpts { default: Some(Some(3200)), ..GetIntOpts::default() },
            )?
            .expect("getint with a default returns a value");
        if query_rate(data_rate).is_none() {
            return Err(KlippyError::Config(pct(
                "Invalid rate parameter: %d",
                &[pct_int(data_rate)],
            )));
        }
        // Setup mcu sensor_adxl345 bulk query code
        let spi = mcu_spi_from_config(config, 3, "cs_pin", 5000000, None, false)?;
        let mcu = spi.get_mcu();
        let oid = mcu.create_oid()?;
        mcu.add_config_cmd(
            &pct(
                "config_adxl345 oid=%d spi_oid=%d",
                &[pct_int(oid), pct_int(spi.get_oid())],
            ),
            AddConfigCmdOpts::default(),
        )?;
        mcu.add_config_cmd(
            &pct("query_adxl345 oid=%d rest_ticks=0", &[pct_int(oid)]),
            AddConfigCmdOpts { on_restart: true, ..AddConfigCmdOpts::default() },
        )?;
        let weak: Weak<ADXL345> = Arc::downgrade(&dev);
        let build = weak.clone();
        mcu.register_config_callback(Box::new(move || match build.upgrade() {
            Some(dev) => dev._build_config(),
            None => Ok(()),
        }))?;
        // Bulk sample message reading
        let chip_smooth = data_rate as f64 * BATCH_UPDATES * 2.0;
        let ffreader = FixedFreqReader::new(Arc::clone(&mcu), chip_smooth, "BBBBB")?;
        // Process messages in batches
        let process = weak.clone();
        let start = weak.clone();
        let finish = weak;
        let batch_bulk = BatchBulkHelper::new(
            Arc::clone(&printer),
            Box::new(move |eventtime| match process.upgrade() {
                Some(dev) => dev._process_batch(eventtime),
                None => Ok(PyValue::None),
            }),
            Some(Box::new(move || match start.upgrade() {
                Some(dev) => dev._start_measurements(),
                None => Ok(()),
            })),
            Some(Box::new(move || match finish.upgrade() {
                Some(dev) => dev._finish_measurements(),
                None => Ok(()),
            })),
            BATCH_UPDATES,
        );
        let section = config.get_name();
        let name = section
            .split_whitespace()
            .last()
            .expect("section names are not empty")
            .to_string();
        let header = PyValue::Tuple(
            ["time", "x_acceleration", "y_acceleration", "z_acceleration"]
                .iter()
                .map(|h| PyValue::Str(h.to_string()))
                .collect(),
        );
        let mut start_resp = IndexMap::new();
        start_resp.insert(PyValue::Str("header".to_string()), header);
        batch_bulk.add_mux_endpoint(
            "adxl345/dump_adxl345",
            "sensor",
            &name,
            PyValue::Dict(start_resp),
        )?;
        let setup = Setup {
            axes_map,
            data_rate,
            spi,
            mcu,
            oid,
            query_adxl345_cmd: GCell::new(None),
            ffreader,
            last_error_count: GCell::new(0),
            batch_bulk,
            name,
        };
        if dev.setup.set(setup).is_err() {
            unreachable!("ADXL345 setup is assigned once");
        }
        Ok(dev)
    }

    /// The attributes assigned after the command helper; `attr` names the one
    /// Python would fail on if the object were used before `__init__` ended.
    fn setup(&self, attr: &str) -> KResult<&Setup> {
        self.setup.get().ok_or_else(|| {
            internal(
                "AttributeError",
                pct(
                    "'ADXL345' object has no attribute '%s'",
                    &[PctValue::Str(attr.to_string())],
                ),
            )
        })
    }

    /// `self.axes_map`.
    pub fn axes_map(&self) -> KResult<Vec<(usize, f64)>> {
        Ok(self.setup("axes_map")?.axes_map.clone())
    }

    /// `self.data_rate`.
    pub fn data_rate(&self) -> KResult<i64> {
        Ok(self.setup("data_rate")?.data_rate)
    }

    /// `self.oid`.
    pub fn oid(&self) -> KResult<i64> {
        Ok(self.setup("oid")?.oid)
    }

    /// `self.last_error_count`.
    pub fn last_error_count(&self) -> KResult<i64> {
        Ok(self.setup("last_error_count")?.last_error_count.get())
    }

    /// `self.batch_bulk`.
    pub fn batch_bulk(&self) -> KResult<&BatchBulkHelper> {
        Ok(&self.setup("batch_bulk")?.batch_bulk)
    }

    /// `_build_config`: the MCU config callback.
    pub fn _build_config(&self) -> KResult<()> {
        let setup = self.setup("spi")?;
        let cmdqueue = setup.spi.get_command_queue();
        *setup.query_adxl345_cmd.borrow_mut() = Some(setup.mcu.lookup_command(
            "query_adxl345 oid=%c rest_ticks=%u",
            Some(cmdqueue),
        )?);
        setup.ffreader.setup_query_command(
            "query_adxl345_status oid=%c",
            setup.oid,
            Some(cmdqueue),
        )
    }

    /// `read_reg(reg)`. Pauses the reactor.
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        let setup = self.setup("spi")?;
        let params = setup.spi.spi_transfer(
            &[(reg | REG_MOD_READ) as u8, 0x00],
            QuerySendOpts::default(),
        )?;
        let PyValue::Bytes(response) = dict_item(&params, "response")? else {
            return Err(internal("TypeError", "cannot convert 'response' to bytearray"));
        };
        match response.get(1) {
            Some(b) => Ok(*b as i64),
            None => Err(internal("IndexError", "bytearray index out of range")),
        }
    }

    /// `set_reg(reg, val, minclock=0)`. Pauses the reactor.
    pub fn set_reg_minclock(&self, reg: i64, val: i64, minclock: i64) -> KResult<()> {
        let setup = self.setup("spi")?;
        setup.spi.spi_send(
            &[reg as u8, (val & 0xFF) as u8],
            SendOpts { minclock, reqclock: 0 },
        )?;
        let stored_val = self.read_reg(reg)?;
        if stored_val != val {
            return Err(KlippyError::Command(pct(
                "Failed to set ADXL345 register [0x%x] to 0x%x: got 0x%x. \
                 This is generally indicative of connection problems \
                 (e.g. faulty wiring) or a faulty adxl345 chip.",
                &[pct_int(reg), pct_int(val), pct_int(stored_val)],
            )));
        }
        Ok(())
    }

    /// `start_internal_client()`.
    pub fn start_internal_client(&self) -> KResult<Arc<AccelQueryHelper>> {
        let aqh = Arc::new(AccelQueryHelper::new(Arc::clone(&self.printer))?);
        let client = Arc::clone(&aqh);
        self.batch_bulk()?
            .add_client(Box::new(move |msg| Ok(client.handle_batch(msg))))?;
        Ok(aqh)
    }

    /// `_convert_samples(samples)`: converts the raw bulk frames
    /// `(ptime, xlow, ylow, zlow, xzhigh, yzhigh)` in place into
    /// `(time, x, y, z)` tuples and drops frames flagged as overflowed.
    pub fn _convert_samples(&self, samples: &mut Vec<PyValue>) -> KResult<()> {
        let setup = self.setup("axes_map")?;
        let (x_pos, x_scale) = setup.axes_map[0];
        let (y_pos, y_scale) = setup.axes_map[1];
        let (z_pos, z_scale) = setup.axes_map[2];
        let mut count = 0usize;
        for i in 0..samples.len() {
            let frame = unpack::<6>(&samples[i])?;
            let ptime = &frame[0];
            let xlow = as_int(&frame[1])?;
            let ylow = as_int(&frame[2])?;
            let zlow = as_int(&frame[3])?;
            let xzhigh = as_int(&frame[4])?;
            let yzhigh = as_int(&frame[5])?;
            if yzhigh & 0x80 != 0 {
                setup.last_error_count.set(setup.last_error_count.get() + 1);
                continue;
            }
            let rx = (xlow | ((xzhigh & 0x1f) << 8)) - ((xzhigh & 0x10) << 9);
            let ry = (ylow | ((yzhigh & 0x1f) << 8)) - ((yzhigh & 0x10) << 9);
            let rz = (zlow | ((xzhigh & 0xe0) << 3) | ((yzhigh & 0xe0) << 6))
                - ((yzhigh & 0x40) << 7);
            let raw_xyz = [rx, ry, rz];
            let x = round_py_nd(raw_xyz[x_pos] as f64 * x_scale, 6);
            let y = round_py_nd(raw_xyz[y_pos] as f64 * y_scale, 6);
            let z = round_py_nd(raw_xyz[z_pos] as f64 * z_scale, 6);
            let ptime = round_py_nd(as_float(ptime)?, 6);
            samples[count] = PyValue::Tuple(vec![
                PyValue::Float(ptime),
                PyValue::Float(x),
                PyValue::Float(y),
                PyValue::Float(z),
            ]);
            count += 1;
        }
        samples.truncate(count);
        Ok(())
    }

    /// `_start_measurements`: the start callback of the batch helper.
    /// Pauses the reactor.
    pub fn _start_measurements(&self) -> KResult<()> {
        let setup = self.setup("data_rate")?;
        // In case of miswiring, testing ADXL345 device ID prevents treating
        // noise or wrong signal as a correctly initialized device
        let dev_id = self.read_reg(REG_DEVID)?;
        if dev_id != ADXL345_DEV_ID {
            return Err(KlippyError::Command(pct(
                "Invalid adxl345 id (got %x vs %x).\n\
                 This is generally indicative of connection problems\n\
                 (e.g. faulty wiring) or a faulty adxl345 chip.",
                &[pct_int(dev_id), pct_int(ADXL345_DEV_ID)],
            )));
        }
        // Setup chip in requested query rate
        self.set_reg_minclock(REG_POWER_CTL, 0x00, 0)?;
        self.set_reg_minclock(REG_DATA_FORMAT, 0x0B, 0)?;
        self.set_reg_minclock(REG_FIFO_CTL, 0x00, 0)?;
        let rate_code = query_rate(setup.data_rate).expect("validated in ADXL345::new");
        self.set_reg_minclock(REG_BW_RATE, rate_code, 0)?;
        self.set_reg_minclock(REG_FIFO_CTL, SET_FIFO_CTL, 0)?;
        // Start bulk reading
        let rest_ticks = setup.mcu.seconds_to_clock(4. / setup.data_rate as f64);
        let query_cmd = setup.query_adxl345_cmd.cloned();
        match query_cmd {
            Some(cmd) => cmd.send(vec![py_int(setup.oid), py_int(rest_ticks)], SendOpts::default())?,
            None => {
                return Err(internal(
                    "AttributeError",
                    "'NoneType' object has no attribute 'send'",
                ))
            }
        }
        self.set_reg_minclock(REG_POWER_CTL, 0x08, 0)?;
        log::info(&pct(
            "ADXL345 starting '%s' measurements",
            &[PctValue::Str(setup.name.clone())],
        ));
        // Initialize clock tracking
        setup.ffreader.note_start()?;
        setup.last_error_count.set(0);
        Ok(())
    }

    /// `_finish_measurements`: the stop callback of the batch helper.
    /// Pauses the reactor.
    pub fn _finish_measurements(&self) -> KResult<()> {
        let setup = self.setup("name")?;
        // Halt bulk reading
        self.set_reg_minclock(REG_POWER_CTL, 0x00, 0)?;
        let query_cmd = setup.query_adxl345_cmd.cloned();
        match query_cmd {
            Some(cmd) => cmd.send_wait_ack(vec![py_int(setup.oid), py_int(0)], SendOpts::default())?,
            None => {
                return Err(internal(
                    "AttributeError",
                    "'NoneType' object has no attribute 'send_wait_ack'",
                ))
            }
        }
        setup.ffreader.note_end()?;
        log::info(&pct(
            "ADXL345 finished '%s' measurements",
            &[PctValue::Str(setup.name.clone())],
        ));
        Ok(())
    }

    /// `_process_batch(eventtime)`: the batch callback of the batch helper.
    /// Pauses the reactor.
    pub fn _process_batch(&self, _eventtime: f64) -> KResult<PyValue> {
        let setup = self.setup("ffreader")?;
        let mut samples = setup.ffreader.pull_samples()?;
        self._convert_samples(&mut samples)?;
        if samples.is_empty() {
            return Ok(PyValue::Dict(IndexMap::new()));
        }
        let overflows = setup.ffreader.get_last_overflows();
        let mut msg = IndexMap::new();
        msg.insert(PyValue::Str("data".to_string()), PyValue::List(samples));
        msg.insert(
            PyValue::Str("errors".to_string()),
            py_int(setup.last_error_count.get()),
        );
        msg.insert(PyValue::Str("overflows".to_string()), py_int(overflows));
        Ok(PyValue::Dict(msg))
    }
}

impl AccelChipApi for ADXL345 {
    fn start_internal_client(&self) -> KResult<Arc<AccelQueryHelper>> {
        ADXL345::start_internal_client(self)
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        ADXL345::read_reg(self, reg)
    }

    fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        self.set_reg_minclock(reg, val, 0)
    }

    fn name(&self) -> String {
        self.setup
            .get()
            .map(|setup| setup.name.clone())
            .unwrap_or_default()
    }
}

klippy_api::printer_object!(ADXL345; dyn AccelChipApi);

// ---------------------------------------------------------------------------
// Module entry points (`adxl345.py:313-317`)
// ---------------------------------------------------------------------------

/// `load_config(config)`.
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let dev: ObjectRef = ADXL345::new(config)?;
    Ok(Some(dev))
}

/// `load_config_prefix(config)`.
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    load_config(config)
}

/// Module entry points for the `adxl345` and `adxl345 <name>` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
