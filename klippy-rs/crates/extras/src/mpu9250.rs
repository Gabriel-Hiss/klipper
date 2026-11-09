//! Port of `klippy/extras/mpu9250.py`: the MPU9250 I2C accelerometer.
//!
//! Like [`crate::adxl345`], `MPU9250.__init__` registers the G-code
//! commands through `AccelCommandHelper(config, self)` before it reads
//! `axes_map`, `rate` and the I2C bus. The chip therefore exists before
//! those values do, so [`MPU9250`] holds them in a [`OnceLock`] that
//! `MPU9250::new` fills right after the helper is registered.

use crate::adxl345::{read_axes_map, AccelChipApi, AccelCommandHelper, AccelQueryHelper};
use crate::bulk_sensor::{BatchBulkHelper, FixedFreqReader};
use crate::bus::{mcu_i2c_from_config, McuI2c, McuI2cApi};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{AddConfigCmdOpts, CommandWrapper, Mcu, QuerySendOpts, SendOpts};
use klippy_api::printer::{ModuleFactory, Printer};
use klippy_api::{GCell, ObjectRef};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, OnceLock, Weak};

// Register map and constants (`mpu9250.py:10-55`)
// ---------------------------------------------------------------------------

/// `MPU9250_ADDR`.
pub const MPU9250_ADDR: i64 = 0x68;

/// `MPU_DEV_IDS` (`mpu9250.py:12-21`).
pub fn mpu_dev_id(dev_id: i64) -> Option<&'static str> {
    match dev_id {
        0x74 => Some("mpu-9515"),
        0x73 => Some("mpu-9255"),
        0x71 => Some("mpu-9250"),
        0x70 => Some("mpu-6500"),
        0x68 => Some("mpu-6050"),
        0x75 | 0x69 => Some("mpu-unknown (DEFECTIVE! USE WITH CAUTION!)"),
        _ => None,
    }
}

/// `REG_DEVID`.
pub const REG_DEVID: i64 = 0x75;
/// `REG_FIFO_EN`.
pub const REG_FIFO_EN: i64 = 0x23;
/// `REG_SMPLRT_DIV`.
pub const REG_SMPLRT_DIV: i64 = 0x19;
/// `REG_CONFIG`.
pub const REG_CONFIG: i64 = 0x1A;
/// `REG_ACCEL_CONFIG`.
pub const REG_ACCEL_CONFIG: i64 = 0x1C;
/// `REG_ACCEL_CONFIG2`.
pub const REG_ACCEL_CONFIG2: i64 = 0x1D;
/// `REG_USER_CTRL`.
pub const REG_USER_CTRL: i64 = 0x6A;
/// `REG_PWR_MGMT_1`.
pub const REG_PWR_MGMT_1: i64 = 0x6B;
/// `REG_PWR_MGMT_2`.
pub const REG_PWR_MGMT_2: i64 = 0x6C;
/// `REG_INT_STATUS`.
pub const REG_INT_STATUS: i64 = 0x3A;

/// `SAMPLE_RATE_DIVS` (`mpu9250.py:35`): `SAMPLE_RATE_DIVS.get(rate)`.
pub fn sample_rate_div(rate: i64) -> Option<i64> {
    match rate {
        4000 => Some(0x00),
        _ => None,
    }
}

/// `SET_CONFIG`.
pub const SET_CONFIG: i64 = 0x01;
/// `SET_ACCEL_CONFIG`.
pub const SET_ACCEL_CONFIG: i64 = 0x10;
/// `SET_ACCEL_CONFIG2`.
pub const SET_ACCEL_CONFIG2: i64 = 0x08;
/// `SET_PWR_MGMT_1_WAKE`.
pub const SET_PWR_MGMT_1_WAKE: i64 = 0x00;
/// `SET_PWR_MGMT_1_SLEEP`.
pub const SET_PWR_MGMT_1_SLEEP: i64 = 0x40;
/// `SET_PWR_MGMT_2_ACCEL_ON`.
pub const SET_PWR_MGMT_2_ACCEL_ON: i64 = 0x07;
/// `SET_PWR_MGMT_2_OFF`.
pub const SET_PWR_MGMT_2_OFF: i64 = 0x3F;
/// `SET_USER_FIFO_RESET`.
pub const SET_USER_FIFO_RESET: i64 = 0x04;
/// `SET_USER_FIFO_EN`.
pub const SET_USER_FIFO_EN: i64 = 0x40;
/// `SET_ENABLE_FIFO`.
pub const SET_ENABLE_FIFO: i64 = 0x08;
/// `SET_DISABLE_FIFO`.
pub const SET_DISABLE_FIFO: i64 = 0x00;

/// `FREEFALL_ACCEL`.
pub const FREEFALL_ACCEL: f64 = 9.80665 * 1000.;
/// `SCALE` (1/4096 g/LSB @8g scale * Earth gravity in mm/s**2).
pub const SCALE: f64 = 0.000244140625 * FREEFALL_ACCEL;

/// `FIFO_SIZE`.
pub const FIFO_SIZE: i64 = 512;
/// `BATCH_UPDATES` (`mpu9250.py:55`).
pub const BATCH_UPDATES: f64 = 0.100;

// ---------------------------------------------------------------------------
// Python exception helpers
// ---------------------------------------------------------------------------

fn internal(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.into() }
}

fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant mpu9250 format renders")
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

/// Elements of a list or tuple (`for x in seq` / `seq[i]`).
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
                "unsupported operand type(s) for *: '%s' and 'float'",
                &[PctValue::Str(pycompat::value::type_name(other).to_string())],
            ),
        )),
    }
}

// ---------------------------------------------------------------------------
// `MPU9250` (`mpu9250.py:58-169`)
// ---------------------------------------------------------------------------

/// Everything `MPU9250.__init__` assigns after `AccelCommandHelper` ran.
struct Setup {
    axes_map: Vec<(usize, f64)>,
    data_rate: i64,
    i2c: Arc<McuI2c>,
    mcu: Mcu,
    oid: i64,
    query_mpu9250_cmd: GCell<Option<CommandWrapper>>,
    ffreader: FixedFreqReader,
    last_error_count: GCell<i64>,
    batch_bulk: BatchBulkHelper,
    name: String,
}

/// Port of the `MPU9250` class.
pub struct MPU9250 {
    printer: Printer,
    setup: OnceLock<Setup>,
}

impl MPU9250 {
    /// `MPU9250.__init__`.
    pub fn new(config: &Config) -> KResult<Arc<MPU9250>> {
        let printer = config.get_printer();
        let dev = Arc::new(MPU9250 { printer: Arc::clone(&printer), setup: OnceLock::new() });
        AccelCommandHelper::new(config, Arc::clone(&dev) as Arc<dyn AccelChipApi>)?;
        let axes_map = read_axes_map(config, SCALE, SCALE, SCALE)?;
        let data_rate = config
            .getint(
                "rate",
                GetIntOpts { default: Some(Some(4000)), ..GetIntOpts::default() },
            )?
            .expect("getint with a default returns a value");
        if sample_rate_div(data_rate).is_none() {
            return Err(KlippyError::Config(pct(
                "Invalid rate parameter: %d",
                &[pct_int(data_rate)],
            )));
        }
        // Setup mcu sensor_mpu9250 bulk query code
        let i2c = mcu_i2c_from_config(config, Some(MPU9250_ADDR), 400000, false)?;
        let mcu = i2c.get_mcu();
        let oid = mcu.create_oid()?;
        let weak: Weak<MPU9250> = Arc::downgrade(&dev);
        let build = weak.clone();
        mcu.register_config_callback(Box::new(move || match build.upgrade() {
            Some(dev) => dev._build_config(),
            None => Ok(()),
        }))?;
        // Bulk sample message reading
        let chip_smooth = data_rate as f64 * BATCH_UPDATES * 2.0;
        let ffreader = FixedFreqReader::new(Arc::clone(&mcu), chip_smooth, ">hhh")?;
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
            "mpu9250/dump_mpu9250",
            "sensor",
            &name,
            PyValue::Dict(start_resp),
        )?;
        let setup = Setup {
            axes_map,
            data_rate,
            i2c,
            mcu,
            oid,
            query_mpu9250_cmd: GCell::new(None),
            ffreader,
            last_error_count: GCell::new(0),
            batch_bulk,
            name,
        };
        if dev.setup.set(setup).is_err() {
            unreachable!("MPU9250 setup is assigned once");
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
                    "'MPU9250' object has no attribute '%s'",
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
        let setup = self.setup("i2c")?;
        let cmdqueue = setup.i2c.get_command_queue();
        setup.mcu.add_config_cmd(
            &pct(
                "config_mpu9250 oid=%d i2c_oid=%d",
                &[pct_int(setup.oid), pct_int(setup.i2c.get_oid())],
            ),
            AddConfigCmdOpts::default(),
        )?;
        setup.mcu.add_config_cmd(
            &pct("query_mpu9250 oid=%d rest_ticks=0", &[pct_int(setup.oid)]),
            AddConfigCmdOpts { on_restart: true, ..AddConfigCmdOpts::default() },
        )?;
        *setup.query_mpu9250_cmd.borrow_mut() = Some(setup.mcu.lookup_command(
            "query_mpu9250 oid=%c rest_ticks=%u",
            Some(cmdqueue),
        )?);
        setup.ffreader.setup_query_command(
            "query_mpu9250_status oid=%c",
            setup.oid,
            Some(cmdqueue),
        )
    }

    /// `read_reg(reg)`. Pauses the reactor.
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        let setup = self.setup("i2c")?;
        let params = setup.i2c.i2c_read(
            &[reg as u8],
            1,
            QuerySendOpts::default(),
        )?;
        let PyValue::Bytes(response) = dict_item(&params, "response")? else {
            return Err(internal("TypeError", "cannot convert 'response' to bytearray"));
        };
        match response.first() {
            Some(b) => Ok(*b as i64),
            None => Err(internal("IndexError", "bytearray index out of range")),
        }
    }

    /// `set_reg(reg, val, minclock=0)`. Pauses the reactor.
    pub fn set_reg_minclock(&self, reg: i64, val: i64, minclock: i64) -> KResult<()> {
        let setup = self.setup("i2c")?;
        setup.i2c.i2c_write(
            &[reg as u8, (val & 0xFF) as u8],
            QuerySendOpts { minclock, reqclock: 0, retry: true },
        )
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
    /// `(ptime, rx, ry, rz)` in place into `(time, x, y, z)` tuples.
    pub fn _convert_samples(&self, samples: &mut [PyValue]) -> KResult<()> {
        let setup = self.setup("axes_map")?;
        let (x_pos, x_scale) = setup.axes_map[0];
        let (y_pos, y_scale) = setup.axes_map[1];
        let (z_pos, z_scale) = setup.axes_map[2];
        for (count, i) in (0..samples.len()).enumerate() {
            let frame = unpack::<4>(&samples[i])?;
            let ptime = &frame[0];
            let rx = as_int(&frame[1])?;
            let ry = as_int(&frame[2])?;
            let rz = as_int(&frame[3])?;
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
        }
        Ok(())
    }

    /// `_start_measurements`: the start callback of the batch helper.
    /// Pauses the reactor.
    pub fn _start_measurements(&self) -> KResult<()> {
        let setup = self.setup("data_rate")?;
        // In case of miswiring, testing MPU9250 device ID prevents treating
        // noise or wrong signal as a correctly initialized device
        let dev_id = self.read_reg(REG_DEVID)?;
        match mpu_dev_id(dev_id) {
            Some(dev_name) => {
                log::info(&pct(
                    "Found %s with id %x",
                    &[PctValue::Str(dev_name.to_string()), pct_int(dev_id)],
                ));
            }
            None => {
                return Err(KlippyError::Command(pct(
                    "Invalid mpu id (got %x).\n\
                     This is generally indicative of connection problems\n\
                     (e.g. faulty wiring) or a faulty chip.",
                    &[pct_int(dev_id)],
                )));
            }
        }
        // Setup chip in requested query rate
        self.set_reg_minclock(REG_PWR_MGMT_1, SET_PWR_MGMT_1_WAKE, 0)?;
        self.set_reg_minclock(REG_PWR_MGMT_2, SET_PWR_MGMT_2_ACCEL_ON, 0)?;
        // Add 20ms pause for accelerometer chip wake up
        self.read_reg(REG_DEVID)?; // Dummy read to ensure queues flushed
        let systime = self.printer.get_reactor().monotonic();
        let next_time = setup.mcu.estimated_print_time(systime) + 0.020;
        self.set_reg_minclock(
            REG_SMPLRT_DIV,
            sample_rate_div(setup.data_rate).expect("validated in MPU9250::new"),
            setup.mcu.print_time_to_clock(next_time),
        )?;
        self.set_reg_minclock(REG_CONFIG, SET_CONFIG, 0)?;
        self.set_reg_minclock(REG_ACCEL_CONFIG, SET_ACCEL_CONFIG, 0)?;
        self.set_reg_minclock(REG_ACCEL_CONFIG2, SET_ACCEL_CONFIG2, 0)?;
        // Reset fifo
        self.set_reg_minclock(REG_FIFO_EN, SET_DISABLE_FIFO, 0)?;
        self.set_reg_minclock(REG_USER_CTRL, SET_USER_FIFO_RESET, 0)?;
        self.set_reg_minclock(REG_USER_CTRL, SET_USER_FIFO_EN, 0)?;
        self.read_reg(REG_INT_STATUS)?; // clear FIFO overflow flag
        // Start bulk reading
        let rest_ticks = setup.mcu.seconds_to_clock(4. / setup.data_rate as f64);
        let query_cmd = setup.query_mpu9250_cmd.cloned();
        match query_cmd {
            Some(cmd) => cmd.send(vec![py_int(setup.oid), py_int(rest_ticks)], SendOpts::default())?,
            None => {
                return Err(internal(
                    "AttributeError",
                    "'NoneType' object has no attribute 'send'",
                ))
            }
        }
        self.set_reg_minclock(REG_FIFO_EN, SET_ENABLE_FIFO, 0)?;
        log::info(&pct(
            "MPU9250 starting '%s' measurements",
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
        self.set_reg_minclock(REG_FIFO_EN, SET_DISABLE_FIFO, 0)?;
        let query_cmd = setup.query_mpu9250_cmd.cloned();
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
            "MPU9250 finished '%s' measurements",
            &[PctValue::Str(setup.name.clone())],
        ));
        self.set_reg_minclock(REG_PWR_MGMT_1, SET_PWR_MGMT_1_SLEEP, 0)?;
        self.set_reg_minclock(REG_PWR_MGMT_2, SET_PWR_MGMT_2_OFF, 0)?;
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

impl AccelChipApi for MPU9250 {
    fn start_internal_client(&self) -> KResult<Arc<AccelQueryHelper>> {
        MPU9250::start_internal_client(self)
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        MPU9250::read_reg(self, reg)
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

klippy_api::printer_object!(MPU9250; dyn AccelChipApi);

// ---------------------------------------------------------------------------
// Module entry points (`mpu9250.py:171-175`)
// ---------------------------------------------------------------------------

/// `load_config(config)`.
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let dev: ObjectRef = MPU9250::new(config)?;
    Ok(Some(dev))
}

/// `load_config_prefix(config)`.
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    load_config(config)
}

/// Module entry points for the `mpu9250` and `mpu9250 <name>` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
