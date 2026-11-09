//! Port of `klippy/extras/lis2dw.py`: the LIS2DW / LIS3DH accelerometers on
//! an SPI or I2C bus.
//!
//! `lis3dh` builds the same class with `LIS3DH_TYPE`, so [`LIS2DW::new`]
//! takes the chip type like `LIS2DW.__init__(config, lis_type)`.
//!
//! `LIS2DW.__init__` registers the G-code commands through
//! `AccelCommandHelper(config, self)` before it reads `axes_map`, the bus and
//! the rest. The `chip` handed to the helper must therefore exist before
//! those values do, so [`LIS2DW`] holds them in a [`OnceLock`] that
//! `LIS2DW::new` fills right after the helper is registered. Using the chip
//! before that point raises the `AttributeError` Python raises for the
//! missing attribute.

use crate::adxl345::{read_axes_map, AccelChipApi, AccelCommandHelper, AccelQueryHelper};
use crate::bulk_sensor::{BatchBulkHelper, FixedFreqReader};
use crate::bus::{
    mcu_i2c_from_config, mcu_spi_from_config, McuI2c, McuI2cApi, McuSpi, McuSpiApi,
};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandWrapper, Mcu, QuerySendOpts, SendOpts,
};
use klippy_api::printer::{ModuleFactory, Printer};
use klippy_api::{GCell, ObjectRef};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, OnceLock, Weak};

// ---------------------------------------------------------------------------
// Register map and constants (`lis2dw.py:10-46`)
// ---------------------------------------------------------------------------

/// `REG_LIS2DW_WHO_AM_I_ADDR`.
pub const REG_LIS2DW_WHO_AM_I_ADDR: i64 = 0x0F;
/// `REG_LIS2DW_CTRL_REG1_ADDR`.
pub const REG_LIS2DW_CTRL_REG1_ADDR: i64 = 0x20;
/// `REG_LIS2DW_CTRL_REG2_ADDR`.
pub const REG_LIS2DW_CTRL_REG2_ADDR: i64 = 0x21;
/// `REG_LIS2DW_CTRL_REG3_ADDR`.
pub const REG_LIS2DW_CTRL_REG3_ADDR: i64 = 0x22;
/// `REG_LIS2DW_CTRL_REG4_ADDR`.
pub const REG_LIS2DW_CTRL_REG4_ADDR: i64 = 0x23;
/// `REG_LIS2DW_CTRL_REG5_ADDR`.
pub const REG_LIS2DW_CTRL_REG5_ADDR: i64 = 0x24;
/// `REG_LIS2DW_CTRL_REG6_ADDR`.
pub const REG_LIS2DW_CTRL_REG6_ADDR: i64 = 0x25;
/// `REG_LIS2DW_STATUS_REG_ADDR`.
pub const REG_LIS2DW_STATUS_REG_ADDR: i64 = 0x27;
/// `REG_LIS2DW_OUT_XL_ADDR`.
pub const REG_LIS2DW_OUT_XL_ADDR: i64 = 0x28;
/// `REG_LIS2DW_OUT_XH_ADDR`.
pub const REG_LIS2DW_OUT_XH_ADDR: i64 = 0x29;
/// `REG_LIS2DW_OUT_YL_ADDR`.
pub const REG_LIS2DW_OUT_YL_ADDR: i64 = 0x2A;
/// `REG_LIS2DW_OUT_YH_ADDR`.
pub const REG_LIS2DW_OUT_YH_ADDR: i64 = 0x2B;
/// `REG_LIS2DW_OUT_ZL_ADDR`.
pub const REG_LIS2DW_OUT_ZL_ADDR: i64 = 0x2C;
/// `REG_LIS2DW_OUT_ZH_ADDR`.
pub const REG_LIS2DW_OUT_ZH_ADDR: i64 = 0x2D;
/// `REG_LIS2DW_FIFO_CTRL`.
pub const REG_LIS2DW_FIFO_CTRL: i64 = 0x2E;
/// `REG_LIS2DW_FIFO_SAMPLES`.
pub const REG_LIS2DW_FIFO_SAMPLES: i64 = 0x2F;
/// `REG_MOD_READ`.
pub const REG_MOD_READ: i64 = 0x80;

/// `LIS2DW_DEV_ID`.
pub const LIS2DW_DEV_ID: i64 = 0x44;
/// `LIS3DH_DEV_ID`.
pub const LIS3DH_DEV_ID: i64 = 0x33;

/// `LIS_I2C_ADDR`.
pub const LIS_I2C_ADDR: i64 = 0x19;

/// `FREEFALL_ACCEL`.
pub const FREEFALL_ACCEL: f64 = 9.80665;
/// `LIS2DW_SCALE`.
pub const LIS2DW_SCALE: f64 = FREEFALL_ACCEL * 1.952 / 4.0;
/// `LIS3DH_SCALE`.
pub const LIS3DH_SCALE: f64 = FREEFALL_ACCEL * 11.718 / 16.0;

/// `BATCH_UPDATES`.
pub const BATCH_UPDATES: f64 = 0.100;

/// `LIS2DW_TYPE`.
pub const LIS2DW_TYPE: &str = "LIS2DW";
/// `LIS3DH_TYPE`.
pub const LIS3DH_TYPE: &str = "LIS3DH";

/// `SPI_SERIAL_TYPE`.
pub const SPI_SERIAL_TYPE: &str = "spi";
/// `I2C_SERIAL_TYPE`.
pub const I2C_SERIAL_TYPE: &str = "i2c";

// ---------------------------------------------------------------------------
// Python helpers
// ---------------------------------------------------------------------------

fn internal(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.into() }
}

fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant lis2dw format renders")
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_str(s: &str) -> PctValue {
    PctValue::Str(s.to_string())
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
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
                &[pct_str(pycompat::value::type_name(other))],
            ),
        )),
    }
}

/// `bytearray(params['response'])`.
fn response_bytes(params: &PyValue) -> KResult<&[u8]> {
    match dict_item(params, "response")? {
        PyValue::Bytes(bytes) => Ok(bytes),
        other => Err(internal(
            "TypeError",
            pct(
                "cannot convert '%s' object to bytearray",
                &[pct_str(pycompat::value::type_name(other))],
            ),
        )),
    }
}

/// `a, b, c, d = seq` with `N` targets.
fn unpack<const N: usize>(value: &PyValue) -> KResult<&[PyValue]> {
    let items = match value {
        PyValue::List(items) | PyValue::Tuple(items) => items,
        other => {
            return Err(internal(
                "TypeError",
                pct(
                    "cannot unpack non-iterable %s object",
                    &[pct_str(pycompat::value::type_name(other))],
                ),
            ))
        }
    };
    if items.len() > N {
        return Err(internal("ValueError", format!("too many values to unpack (expected {N})")));
    }
    if items.len() < N {
        return Err(internal(
            "ValueError",
            format!("not enough values to unpack (expected {N}, got {})", items.len()),
        ));
    }
    Ok(items)
}

/// The left operand of `raw * scale` (`scale` is a float).
fn raw_as_float(value: &PyValue) -> KResult<f64> {
    value.to_f64().ok_or_else(|| {
        internal(
            "TypeError",
            pct(
                "unsupported operand type(s) for *: '%s' and 'float'",
                &[pct_str(pycompat::value::type_name(value))],
            ),
        )
    })
}

/// `round(ptime, 6)`'s argument.
fn time_as_float(value: &PyValue) -> KResult<f64> {
    value.to_f64().ok_or_else(|| {
        internal(
            "TypeError",
            pct(
                "type %s doesn't define __round__ method",
                &[pct_str(pycompat::value::type_name(value))],
            ),
        )
    })
}

// ---------------------------------------------------------------------------
// `LIS2DW` (`lis2dw.py:49-203`)
// ---------------------------------------------------------------------------

/// `self.bus`: the `MCU_SPI` or `MCU_I2C` chosen by `cs_pin`.
enum Bus {
    Spi(Arc<McuSpi>),
    I2c(Arc<McuI2c>),
}

impl Bus {
    fn get_mcu(&self) -> Mcu {
        match self {
            Bus::Spi(spi) => spi.get_mcu(),
            Bus::I2c(i2c) => i2c.get_mcu(),
        }
    }

    fn get_oid(&self) -> i64 {
        match self {
            Bus::Spi(spi) => spi.get_oid(),
            Bus::I2c(i2c) => i2c.get_oid(),
        }
    }

    fn get_command_queue(&self) -> CommandQueue {
        match self {
            Bus::Spi(spi) => spi.get_command_queue(),
            Bus::I2c(i2c) => i2c.get_command_queue(),
        }
    }
}

/// Everything `LIS2DW.__init__` assigns after `AccelCommandHelper` ran.
struct Setup {
    axes_map: Vec<(usize, f64)>,
    data_rate: i64,
    bus_type: &'static str,
    bus: Bus,
    mcu: Mcu,
    oid: i64,
    query_lis2dw_cmd: GCell<Option<CommandWrapper>>,
    ffreader: FixedFreqReader,
    last_error_count: GCell<i64>,
    batch_bulk: BatchBulkHelper,
    name: String,
}

/// Port of the `LIS2DW` class.
pub struct LIS2DW {
    printer: Printer,
    lis_type: String,
    setup: OnceLock<Setup>,
}

impl LIS2DW {
    /// `LIS2DW.__init__(config, lis_type)`.
    pub fn new(config: &Config, lis_type: &str) -> KResult<Arc<LIS2DW>> {
        let printer = config.get_printer();
        let dev = Arc::new(LIS2DW {
            printer: Arc::clone(&printer),
            lis_type: lis_type.to_string(),
            setup: OnceLock::new(),
        });
        AccelCommandHelper::new(config, Arc::clone(&dev) as Arc<dyn AccelChipApi>)?;
        let (axes_map, data_rate) = if lis_type == LIS2DW_TYPE {
            (read_axes_map(config, LIS2DW_SCALE, LIS2DW_SCALE, LIS2DW_SCALE)?, 1600)
        } else {
            (read_axes_map(config, LIS3DH_SCALE, LIS3DH_SCALE, LIS3DH_SCALE)?, 1344)
        };
        // Check for spi or i2c
        let cs_pin = config.get("cs_pin", GetStrOpts { default: Some(None), note_valid: true })?;
        let bus_type = if cs_pin.is_some() { SPI_SERIAL_TYPE } else { I2C_SERIAL_TYPE };
        // Setup mcu sensor_lis2dw bulk query code
        let bus = if bus_type == SPI_SERIAL_TYPE {
            Bus::Spi(mcu_spi_from_config(config, 3, "cs_pin", 5000000, None, false)?)
        } else {
            Bus::I2c(mcu_i2c_from_config(config, Some(LIS_I2C_ADDR), 400000, false)?)
        };
        let mcu = bus.get_mcu();
        let oid = mcu.create_oid()?;
        mcu.add_config_cmd(
            &pct(
                "config_lis2dw oid=%d bus_oid=%d bus_oid_type=%s lis_chip_type=%s",
                &[pct_int(oid), pct_int(bus.get_oid()), pct_str(bus_type), pct_str(lis_type)],
            ),
            AddConfigCmdOpts::default(),
        )?;
        mcu.add_config_cmd(
            &pct("query_lis2dw oid=%d rest_ticks=0", &[pct_int(oid)]),
            AddConfigCmdOpts { on_restart: true, ..AddConfigCmdOpts::default() },
        )?;
        let weak: Weak<LIS2DW> = Arc::downgrade(&dev);
        let build = weak.clone();
        mcu.register_config_callback(Box::new(move || match build.upgrade() {
            Some(dev) => dev._build_config(),
            None => Ok(()),
        }))?;
        // Bulk sample message reading
        let chip_smooth = data_rate as f64 * BATCH_UPDATES * 2.0;
        let ffreader = FixedFreqReader::new(Arc::clone(&mcu), chip_smooth, "<hhh")?;
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
            "lis2dw/dump_lis2dw",
            "sensor",
            &name,
            PyValue::Dict(start_resp),
        )?;
        let setup = Setup {
            axes_map,
            data_rate,
            bus_type,
            bus,
            mcu,
            oid,
            query_lis2dw_cmd: GCell::new(None),
            ffreader,
            last_error_count: GCell::new(0),
            batch_bulk,
            name,
        };
        if dev.setup.set(setup).is_err() {
            unreachable!("LIS2DW setup is assigned once");
        }
        Ok(dev)
    }

    /// The attributes assigned after the command helper; `attr` names the one
    /// Python would fail on if the object were used before `__init__` ended.
    fn setup(&self, attr: &str) -> KResult<&Setup> {
        self.setup.get().ok_or_else(|| {
            internal(
                "AttributeError",
                pct("'LIS2DW' object has no attribute '%s'", &[pct_str(attr)]),
            )
        })
    }

    /// `self.lis_type`.
    pub fn lis_type(&self) -> &str {
        &self.lis_type
    }

    /// `self.axes_map`.
    pub fn axes_map(&self) -> KResult<Vec<(usize, f64)>> {
        Ok(self.setup("axes_map")?.axes_map.clone())
    }

    /// `self.data_rate`.
    pub fn data_rate(&self) -> KResult<i64> {
        Ok(self.setup("data_rate")?.data_rate)
    }

    /// `self.bus_type`.
    pub fn bus_type(&self) -> KResult<&'static str> {
        Ok(self.setup("bus_type")?.bus_type)
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
        let setup = self.setup("bus")?;
        let cmdqueue = setup.bus.get_command_queue();
        *setup.query_lis2dw_cmd.borrow_mut() = Some(
            setup
                .mcu
                .lookup_command("query_lis2dw oid=%c rest_ticks=%u", Some(cmdqueue))?,
        );
        setup.ffreader.setup_query_command(
            "query_lis2dw_status oid=%c",
            setup.oid,
            Some(cmdqueue),
        )
    }

    /// `read_reg(reg)`. Pauses the reactor.
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        let setup = self.setup("bus_type")?;
        match &setup.bus {
            Bus::Spi(spi) => {
                let params = spi.spi_transfer(
                    &[(reg | REG_MOD_READ) as u8, 0x00],
                    QuerySendOpts::default(),
                )?;
                match response_bytes(&params)?.get(1) {
                    Some(b) => Ok(*b as i64),
                    None => Err(internal("IndexError", "bytearray index out of range")),
                }
            }
            Bus::I2c(i2c) => {
                let params = i2c.i2c_read(&[reg as u8], 1, QuerySendOpts::default())?;
                match response_bytes(&params)?.first() {
                    Some(b) => Ok(*b as i64),
                    None => Err(internal("IndexError", "bytearray index out of range")),
                }
            }
        }
    }

    /// `set_reg(reg, val)`. Pauses the reactor.
    pub fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        self.set_reg_minclock(reg, val, 0)
    }

    /// `set_reg(reg, val, minclock=0)`. Pauses the reactor.
    pub fn set_reg_minclock(&self, reg: i64, val: i64, minclock: i64) -> KResult<()> {
        let setup = self.setup("bus_type")?;
        let data = [reg as u8, (val & 0xFF) as u8];
        match &setup.bus {
            Bus::Spi(spi) => spi.spi_send(&data, SendOpts { minclock, reqclock: 0 })?,
            Bus::I2c(i2c) => i2c.i2c_write(
                &data,
                QuerySendOpts { minclock, ..QuerySendOpts::default() },
            )?,
        }
        let stored_val = self.read_reg(reg)?;
        if stored_val != val {
            return Err(KlippyError::Command(pct(
                concat!(
                    "Failed to set LIS2DW register [0x%x] to 0x%x: got 0x%x. ",
                    "This is generally indicative of connection problems ",
                    "(e.g. faulty wiring) or a faulty lis2dw chip.",
                ),
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
    /// `(ptime, rx, ry, rz)` in place into `(time, x, y, z)` tuples.
    pub fn _convert_samples(&self, samples: &mut [PyValue]) -> KResult<()> {
        let setup = self.setup("axes_map")?;
        let (x_pos, x_scale) = setup.axes_map[0];
        let (y_pos, y_scale) = setup.axes_map[1];
        let (z_pos, z_scale) = setup.axes_map[2];
        for sample in samples.iter_mut() {
            let frame = unpack::<4>(sample)?;
            let (ptime, rx, ry, rz) = (&frame[0], &frame[1], &frame[2], &frame[3]);
            let raw_xyz = [rx, ry, rz];
            let x = round_py_nd(raw_as_float(raw_xyz[x_pos])? * x_scale, 6);
            let y = round_py_nd(raw_as_float(raw_xyz[y_pos])? * y_scale, 6);
            let z = round_py_nd(raw_as_float(raw_xyz[z_pos])? * z_scale, 6);
            let ptime = round_py_nd(time_as_float(ptime)?, 6);
            *sample = PyValue::Tuple(vec![
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
        let setup = self.setup("bus_type")?;
        // In case of miswiring, testing LIS2DW device ID prevents treating
        // noise or wrong signal as a correctly initialized device
        let dev_id = self.read_reg(REG_LIS2DW_WHO_AM_I_ADDR)?;
        log::info(&pct("%s_dev_id: %x", &[pct_str(&self.lis_type), pct_int(dev_id)]));
        if self.lis_type == LIS2DW_TYPE {
            if dev_id != LIS2DW_DEV_ID {
                return Err(KlippyError::Command(pct(
                    concat!(
                        "Invalid lis2dw id (got %x vs %x).\n",
                        "This is generally indicative of connection problems\n",
                        "(e.g. faulty wiring) or a faulty lis2dw chip.",
                    ),
                    &[pct_int(dev_id), pct_int(LIS2DW_DEV_ID)],
                )));
            }
            if setup.bus_type == SPI_SERIAL_TYPE {
                // Disable I2C
                self.set_reg(REG_LIS2DW_CTRL_REG2_ADDR, 0x06)?;
            }
            // Setup chip in requested query rate
            // ODR/2, +-16g, low-pass filter, Low-noise abled
            self.set_reg(REG_LIS2DW_CTRL_REG6_ADDR, 0x34)?;
            // Continuous mode: If the FIFO is full
            // the new sample overwrites the older sample.
            self.set_reg(REG_LIS2DW_FIFO_CTRL, 0xC0)?;
            // High-Performance / Low-Power mode 1600/200 Hz
            // High-Performance Mode (14-bit resolution)
            self.set_reg(REG_LIS2DW_CTRL_REG1_ADDR, 0x94)?;
        } else {
            if dev_id != LIS3DH_DEV_ID {
                return Err(KlippyError::Command(pct(
                    concat!(
                        "Invalid lis3dh id (got %x vs %x).\n",
                        "This is generally indicative of connection problems\n",
                        "(e.g. faulty wiring) or a faulty lis3dh chip.",
                    ),
                    &[pct_int(dev_id), pct_int(LIS3DH_DEV_ID)],
                )));
            }
            // High Resolution / Low Power mode 1344/5376 Hz
            // High Resolution mode (12-bit resolution)
            // Enable X Y Z axes
            self.set_reg(REG_LIS2DW_CTRL_REG1_ADDR, 0x97)?;
            // Disable all filtering
            self.set_reg(REG_LIS2DW_CTRL_REG2_ADDR, 0)?;
            // Set +-16g, High Resolution mode
            self.set_reg(REG_LIS2DW_CTRL_REG4_ADDR, 0x38)?;
            // Enable FIFO
            self.set_reg(REG_LIS2DW_CTRL_REG5_ADDR, 0x40)?;
            // Stream mode
            self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x80)?;
        }
        // Start bulk reading
        let rest_ticks = setup.mcu.seconds_to_clock(4. / setup.data_rate as f64);
        let query_cmd = setup.query_lis2dw_cmd.cloned();
        match query_cmd {
            Some(cmd) => {
                cmd.send(vec![py_int(setup.oid), py_int(rest_ticks)], SendOpts::default())?
            }
            None => {
                return Err(internal(
                    "AttributeError",
                    "'NoneType' object has no attribute 'send'",
                ))
            }
        }
        if self.lis_type == LIS2DW_TYPE {
            self.set_reg(REG_LIS2DW_FIFO_CTRL, 0xC0)?;
        } else {
            self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x80)?;
        }
        log::info(&pct(
            "%s starting '%s' measurements",
            &[pct_str(&self.lis_type), pct_str(&setup.name)],
        ));
        // Initialize clock tracking
        setup.ffreader.note_start()?;
        setup.last_error_count.set(0);
        Ok(())
    }

    /// `_finish_measurements`: the stop callback of the batch helper.
    /// Pauses the reactor.
    pub fn _finish_measurements(&self) -> KResult<()> {
        let setup = self.setup("bus_type")?;
        // Halt bulk reading
        self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x00)?;
        let query_cmd = setup.query_lis2dw_cmd.cloned();
        match query_cmd {
            Some(cmd) => {
                cmd.send_wait_ack(vec![py_int(setup.oid), py_int(0)], SendOpts::default())?
            }
            None => {
                return Err(internal(
                    "AttributeError",
                    "'NoneType' object has no attribute 'send_wait_ack'",
                ))
            }
        }
        setup.ffreader.note_end()?;
        log::info(&pct(
            "%s finished '%s' measurements",
            &[pct_str(&self.lis_type), pct_str(&setup.name)],
        ));
        self.set_reg(REG_LIS2DW_FIFO_CTRL, 0x00)
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

impl AccelChipApi for LIS2DW {
    fn start_internal_client(&self) -> KResult<Arc<AccelQueryHelper>> {
        LIS2DW::start_internal_client(self)
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        LIS2DW::read_reg(self, reg)
    }

    fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        LIS2DW::set_reg(self, reg, val)
    }

    fn name(&self) -> String {
        self.setup
            .get()
            .map(|setup| setup.name.clone())
            .unwrap_or_default()
    }
}

klippy_api::printer_object!(LIS2DW; dyn AccelChipApi);

// ---------------------------------------------------------------------------
// Module entry points (`lis2dw.py:205-209`)
// ---------------------------------------------------------------------------

/// `load_config(config)`.
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let dev: ObjectRef = LIS2DW::new(config, LIS2DW_TYPE)?;
    Ok(Some(dev))
}

/// `load_config_prefix(config)`.
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let dev: ObjectRef = LIS2DW::new(config, LIS2DW_TYPE)?;
    Ok(Some(dev))
}

/// Module entry points for the `lis2dw` and `lis2dw <name>` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
