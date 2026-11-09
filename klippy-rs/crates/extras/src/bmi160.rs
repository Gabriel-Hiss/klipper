//! Port of the `bmi160` module (`klippy/extras/bmi160.py`).
//!
//! The helper classes shared by every accelerometer (`read_axes_map`,
//! `AccelQueryHelper`, `AccelCommandHelper`) come from the real
//! `crate::adxl345` port. The chip object is created before the command
//! helper is registered and its setup is filled in afterwards through a
//! [`OnceLock`], mirroring `ADXL345` (see `adxl345.rs`).
//!
//! `AccelCommandHelper(config, self)` is the first statement of
//! `BMI160.__init__`, so registering it before the bus setup also matches
//! the order in which errors surface (duplicate accelerometer names are
//! reported before any MCU config command is queued).

use crate::adxl345::{AccelChipApi, AccelCommandHelper, AccelQueryHelper};
use crate::bus::{mcu_i2c_from_config, mcu_spi_from_config, McuI2c, McuI2cApi, McuSpi, McuSpiApi};
use crate::bulk_sensor::{BatchBulkHelper, BatchCallback, ClientCallback, FixedFreqReader, StartStopCallback};
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{AddConfigCmdOpts, CommandQueue, CommandWrapper, Mcu, QuerySendOpts, SendOpts};
use klippy_api::object::ObjectRef;
use klippy_api::printer::{ModuleFactory, Printer};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::PyValue;
use reactor::Reactor;
use std::sync::{Arc, OnceLock, Weak};

pub use crate::adxl345::read_axes_map;

pub const REG_CHIPID: i64 = 0x00;
pub const REG_ACC_DATA_START: i64 = 0x12;
pub const REG_ACC_CONF: i64 = 0x40;
pub const REG_ACC_RANGE: i64 = 0x41;
pub const REG_FIFO_DOWNS: i64 = 0x45;
pub const REG_FIFO_CONFIG_0: i64 = 0x46;
pub const REG_FIFO_CONFIG_1: i64 = 0x47;
pub const REG_FIFO_DATA: i64 = 0x24;
pub const REG_FIFO_LENGTH_0: i64 = 0x22;
pub const REG_CMD: i64 = 0x7E;
pub const REG_MOD_READ: i64 = 0x80;

pub const CMD_ACC_PM_SUSPEND: i64 = 0x10;
pub const CMD_ACC_PM_NORMAL: i64 = 0x11;
pub const CMD_FIFO_FLUSH: i64 = 0xB0;

pub const BMI160_DEV_ID: i64 = 0xD1;
pub const SET_ACC_CONF_1600HZ: i64 = 0x2C;
pub const SET_ACC_RANGE_16G: i64 = 0x0C;
pub const SET_FIFO_CONFIG_1: i64 = 0x40;
pub const SET_FIFO_DOWNS: i64 = 0x00;

pub const FREEFALL_ACCEL: f64 = 9.80665 * 1000.;
pub const SCALE: f64 = FREEFALL_ACCEL / 2048.;
pub const BATCH_UPDATES: f64 = 0.100;
pub const BMI_I2C_ADDR: i64 = 0x69;

/// Batch header registered on `bmi160/dump_bmi160`.
pub const HEADER: [&str; 4] = ["time", "x_acceleration", "y_acceleration", "z_acceleration"];

// ---------------------------------------------------------------------------
// Bus selection (`bus.MCU_SPI` / `bus.MCU_I2C` union).
// ---------------------------------------------------------------------------

/// The SPI or I2C bus behind a BMI160 (`bmi160.py:57-64`), delegating to
/// the real bus ports.
pub enum BusKind {
    Spi(Arc<McuSpi>),
    I2c(Arc<McuI2c>),
}

impl BusKind {
    pub fn bus_type(&self) -> &'static str {
        match self {
            BusKind::Spi(_) => "spi",
            BusKind::I2c(_) => "i2c",
        }
    }

    pub fn get_oid(&self) -> i64 {
        match self {
            BusKind::Spi(spi) => spi.get_oid(),
            BusKind::I2c(i2c) => i2c.get_oid(),
        }
    }
    pub fn get_mcu(&self) -> Mcu {
        match self {
            BusKind::Spi(spi) => spi.get_mcu(),
            BusKind::I2c(i2c) => i2c.get_mcu(),
        }
    }

    pub fn get_command_queue(&self) -> CommandQueue {
        match self {
            BusKind::Spi(spi) => spi.get_command_queue(),
            BusKind::I2c(i2c) => i2c.get_command_queue(),
        }
    }

    pub fn spi_transfer(&self, data: &[u8], opts: QuerySendOpts) -> KResult<PyValue> {
        match self {
            BusKind::Spi(spi) => spi.spi_transfer(data, opts),
            BusKind::I2c(_) => Err(KlippyError::Internal {
                class_name: "AttributeError".into(),
                message: "'MCU_I2C' object has no attribute 'spi_transfer'".into(),
            }),
        }
    }

    pub fn i2c_read(&self, write: &[u8], read_len: i64, opts: QuerySendOpts) -> KResult<PyValue> {
        match self {
            BusKind::Spi(_) => Err(KlippyError::Internal {
                class_name: "AttributeError".into(),
                message: "'MCU_SPI' object has no attribute 'i2c_read'".into(),
            }),
            BusKind::I2c(i2c) => i2c.i2c_read(write, read_len, opts),
        }
    }

    pub fn i2c_write(&self, data: &[u8], opts: QuerySendOpts) -> KResult<()> {
        match self {
            BusKind::Spi(_) => Err(KlippyError::Internal {
                class_name: "AttributeError".into(),
                message: "'MCU_SPI' object has no attribute 'i2c_write'".into(),
            }),
            BusKind::I2c(i2c) => i2c.i2c_write(data, opts),
        }
    }
}

// ---------------------------------------------------------------------------
// `BMI160` (`klippy/extras/bmi160.py:48`).
// ---------------------------------------------------------------------------

struct Setup {
    bus: BusKind,
    mcu: Mcu,
    oid: i64,
    query_bmi160_cmd: GCell<Option<CommandWrapper>>,
    ffreader: FixedFreqReader,
    batch_bulk: BatchBulkHelper,
    axes_map: Vec<(usize, f64)>,
    data_rate: i64,
    name: String,
    last_error_count: GCell<i64>,
}

/// Port of the `BMI160` class. `printer` and `reactor` exist before the
/// command helper runs; everything assigned after it lives in `setup`.
pub struct Bmi160 {
    printer: Printer,
    reactor: Reactor,
    setup: OnceLock<Setup>,
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(v.into())
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(v.into())
}


fn missing_query_cmd() -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".into(),
        message: "'NoneType' object has no attribute 'send'".into(),
    }
}

/// `bytearray(params['response'])[n]` with a response shorter than `n + 1`.
fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".into(),
        message: "bytearray index out of range".into(),
    }
}

/// `params['response']` when the MCU response dict has no such key.
fn missing_response() -> KlippyError {
    KlippyError::Internal {
        class_name: "KeyError".into(),
        message: "'response'".into(),
    }
}

/// `msg % args`; every call site passes the arguments its message declares.
fn fmt(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("message and arguments agree")
}

impl Bmi160 {
    /// `BMI160.__init__` (`bmi160.py:49`).
    pub fn new(config: &Config) -> KResult<Arc<Bmi160>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let dev = Arc::new(Bmi160 { printer: Arc::clone(&printer), reactor, setup: OnceLock::new() });
        AccelCommandHelper::new(config, Arc::clone(&dev) as Arc<dyn AccelChipApi>)?;
        let axes_map = read_axes_map(config, SCALE, SCALE, SCALE)?;
        let data_rate = 1600i64;
        // Check for SPI or I2C (`bmi160.py:57`).
        let has_cs_pin = config
            .get("cs_pin", GetStrOpts { default: Some(None), ..Default::default() })?
            .is_some();
        let bus = if has_cs_pin {
            // Using 1MHz to match working Arduino test
            BusKind::Spi(mcu_spi_from_config(config, 0, "cs_pin", 1_000_000, None, false)?)
        } else {
            BusKind::I2c(mcu_i2c_from_config(config, Some(BMI_I2C_ADDR), 400_000, false)?)
        };
        let bus_type = bus.bus_type().to_string();
        let mcu = bus.get_mcu();
        let oid = mcu.create_oid()?;
        let bus_oid = bus.get_oid();
        mcu.add_config_cmd(
            &fmt(
                "config_bmi160 oid=%d bus_oid=%d bus_oid_type=%s",
                &[pct_int(oid), pct_int(bus_oid), PctValue::Str(bus_type)],
            ),
            AddConfigCmdOpts::default(),
        )?;
        mcu.add_config_cmd(
            &fmt("query_bmi160 oid=%d rest_ticks=0", &[pct_int(oid)]),
            AddConfigCmdOpts { on_restart: true, ..Default::default() },
        )?;
        let weak: Weak<Bmi160> = Arc::downgrade(&dev);
        let build = weak.clone();
        mcu.register_config_callback(Box::new(move || match build.upgrade() {
            Some(dev) => dev.build_config(),
            None => Ok(()),
        }))?;
        // Bulk sample message reading
        let chip_smooth = data_rate as f64 * BATCH_UPDATES * 2.;
        let ffreader = FixedFreqReader::new(Arc::clone(&mcu), chip_smooth, "<hhh")?;
        // Process messages in batches
        let process = weak.clone();
        let start = weak.clone();
        let finish = weak;
        let batch_cb: BatchCallback = Box::new(move |_eventtime| match process.upgrade() {
            Some(dev) => dev.process_batch(),
            None => Ok(PyValue::None),
        });
        let start_cb: StartStopCallback = Box::new(move || match start.upgrade() {
            Some(dev) => dev.start_measurements(),
            None => Ok(()),
        });
        let stop_cb: StartStopCallback = Box::new(move || match finish.upgrade() {
            Some(dev) => dev.finish_measurements(),
            None => Ok(()),
        });
        let batch_bulk = BatchBulkHelper::new(Arc::clone(&printer), batch_cb, Some(start_cb), Some(stop_cb), BATCH_UPDATES);
        let section = config.get_name();
        let name = section
            .split_whitespace()
            .last()
            .expect("section names are not empty")
            .to_string();
        let mut start_resp = indexmap::IndexMap::new();
        start_resp.insert(
            PyValue::Str("header".into()),
            PyValue::Tuple(HEADER.iter().map(|s| PyValue::Str((*s).to_string())).collect()),
        );
        batch_bulk.add_mux_endpoint("bmi160/dump_bmi160", "sensor", &name, PyValue::Dict(start_resp))?;
        let setup = Setup {
            bus,
            mcu,
            oid,
            query_bmi160_cmd: GCell::new(None),
            ffreader,
            batch_bulk,
            axes_map,
            data_rate,
            name,
            last_error_count: GCell::new(0),
        };
        if dev.setup.set(setup).is_err() {
            unreachable!("BMI160 setup is assigned once");
        }
        Ok(dev)
    }

    /// The attributes assigned after the command helper; `attr` names the
    /// one Python would fail on if the object were used before `__init__` ended.
    fn setup(&self, attr: &str) -> KResult<&Setup> {
        self.setup.get().ok_or_else(|| {
            KlippyError::Internal {
                class_name: "AttributeError".into(),
                message: fmt("'BMI160' object has no attribute '%s'", &[PctValue::Str(attr.to_string())]),
            }
        })
    }

    /// `self.name`: last word of the config section name.
    pub fn name(&self) -> String {
        self.setup.get().map(|setup| setup.name.clone()).unwrap_or_default()
    }

    /// `self.bus_type`: `spi` or `i2c`.
    pub fn bus_type(&self) -> String {
        self.setup.get().map(|setup| setup.bus.bus_type().to_string()).unwrap_or_default()
    }

    /// `_build_config` (`bmi160.py:85`).
    pub fn build_config(&self) -> KResult<()> {
        let setup = self.setup("bus")?;
        let cmdqueue = setup.bus.get_command_queue();
        let cmd = setup.mcu.lookup_command("query_bmi160 oid=%c rest_ticks=%u", Some(cmdqueue))?;
        *setup.query_bmi160_cmd.borrow_mut() = Some(cmd);
        setup.ffreader.setup_query_command("query_bmi160_status oid=%c", setup.oid, Some(cmdqueue))?;
        Ok(())
    }

    /// `read_reg` (`bmi160.py:91`).
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        let setup = self.setup("bus")?;
        let byte = if setup.bus.bus_type() == "spi" {
            let first = (reg as u8) | (REG_MOD_READ as u8);
            let params = setup.bus.spi_transfer(&[first, 0x00], QuerySendOpts::default())?;
            let response = extract_response_bytes(&params)?;
            // `bytearray(params['response'])[1]`
            response.get(1).copied()
        } else {
            let params = setup.bus.i2c_read(&[reg as u8], 1, QuerySendOpts::default())?;
            let response = extract_response_bytes(&params)?;
            // `bytearray(params['response'])[0]`
            response.first().copied()
        };
        match byte {
            Some(byte) => Ok(byte as i64),
            None => Err(index_error()),
        }
    }

    /// `set_reg(reg, val)` (`bmi160.py:99`).
    pub fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        self.set_reg_with_clock(reg, val, 0)
    }

    /// `set_reg(reg, val, minclock)` (`bmi160.py:99`).
    pub fn set_reg_with_clock(&self, reg: i64, val: i64, minclock: i64) -> KResult<()> {
        let setup = self.setup("bus")?;
        let masked = (val & 0xFF) as u8;
        if setup.bus.bus_type() == "spi" {
            // spi_transfer to ensure command completes on MCU before continuing
            setup
                .bus
                .spi_transfer(&[reg as u8, masked], QuerySendOpts { minclock, ..Default::default() })?;
        } else {
            // I2C already waits for completion by default
            setup
                .bus
                .i2c_write(&[reg as u8, masked], QuerySendOpts { minclock, ..Default::default() })?;
        }
        // Small delay between register writes for stability
        self.reactor.pause(0.002);
        // Don't verify CMD register (0x7E) or registers below 0x40
        if reg >= 0x40 && reg != REG_CMD {
            let stored_val = self.read_reg(reg)?;
            if stored_val != val {
                return Err(KlippyError::command(fmt(
                    "Failed to set BMI160 register [0x%x] to 0x%x: got 0x%x. This is generally indicative of connection problems (e.g. faulty wiring) or a faulty bmi160 chip.",
                    &[pct_int(reg), pct_int(val), pct_int(stored_val)],
                )));
            }
        }
        Ok(())
    }

    /// `start_internal_client` (`bmi160.py:117`).
    pub fn start_internal_client(&self) -> KResult<Arc<AccelQueryHelper>> {
        let aqh = Arc::new(AccelQueryHelper::new(Arc::clone(&self.printer))?);
        let client = Arc::clone(&aqh);
        let cb: ClientCallback = Box::new(move |msg| Ok(client.handle_batch(msg)));
        self.setup("batch_bulk")?.batch_bulk.add_client(cb)?;
        Ok(aqh)
    }

    /// `_convert_samples(samples)` (`bmi160.py:121`), in place.
    pub fn convert_samples(&self, samples: &mut [PyValue]) -> KResult<()> {
        let axes_map = self.setup("axes_map")?.axes_map.clone();
        let (x_pos, x_scale) = axes_map[0];
        let (y_pos, y_scale) = axes_map[1];
        let (z_pos, z_scale) = axes_map[2];
        for slot in samples.iter_mut() {
            let (ptime, rx, ry, rz) = match slot.clone() {
                PyValue::Tuple(items) if items.len() == 4 => {
                    let as_f64 = |v: &PyValue| match v {
                        PyValue::Float(f) => Some(*f),
                        PyValue::Int(n) => n.to_f64_exact(),
                        _ => None,
                    };
                    (
                        as_f64(&items[0]).unwrap_or(f64::NAN),
                        as_f64(&items[1]).unwrap_or(f64::NAN),
                        as_f64(&items[2]).unwrap_or(f64::NAN),
                        as_f64(&items[3]).unwrap_or(f64::NAN),
                    )
                }
                PyValue::Tuple(items) if items.len() < 4 => {
                    return Err(KlippyError::Internal {
                        class_name: "ValueError".into(),
                        message: format!(
                            "not enough values to unpack (expected 4, got {})",
                            items.len()
                        ),
                    });
                }
                PyValue::Tuple(_) => {
                    return Err(KlippyError::Internal {
                        class_name: "ValueError".into(),
                        message: "too many values to unpack (expected 4)".into(),
                    });
                }
                other => {
                    return Err(KlippyError::Internal {
                        class_name: "TypeError".into(),
                        message: format!(
                            "cannot unpack non-iterable {} object",
                            pycompat::value::type_name(&other)
                        ),
                    });
                }
            };
            let raw = [rx, ry, rz];
            let x = round_py_nd(raw[x_pos] * x_scale, 6);
            let y = round_py_nd(raw[y_pos] * y_scale, 6);
            let z = round_py_nd(raw[z_pos] * z_scale, 6);
            *slot = PyValue::Tuple(vec![
                PyValue::Float(round_py_nd(ptime, 6)),
                PyValue::Float(x),
                PyValue::Float(y),
                PyValue::Float(z),
            ]);
        }
        Ok(())
    }

    /// `_start_measurements` (`bmi160.py:132`).
    pub fn start_measurements(&self) -> KResult<()> {
        let setup = self.setup("bus")?;
        // 1. Force SPI Mode (Dummy Read)
        if setup.bus.bus_type() == "spi" {
            self.read_reg(0x7F)?;
            self.reactor.pause(0.010); // 10ms for mode switch
        }
        // 2. Verify ID
        let dev_id = self.read_reg(REG_CHIPID)?;
        if dev_id != BMI160_DEV_ID {
            return Err(KlippyError::command(fmt(
                "Invalid bmi160 id (got %x vs %x).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty bmi160 chip.",
                &[pct_int(dev_id), pct_int(BMI160_DEV_ID)],
            )));
        }
        // 3. Wake Up FIRST
        self.set_reg(REG_CMD, CMD_ACC_PM_NORMAL)?;
        // CRITICAL: Wait 50ms for startup/PLL locking
        self.reactor.pause(0.050);
        // 4. Configure Registers (While Awake)
        self.set_reg(REG_ACC_CONF, SET_ACC_CONF_1600HZ)?;
        self.set_reg(REG_ACC_RANGE, SET_ACC_RANGE_16G)?;
        self.set_reg(REG_FIFO_DOWNS, SET_FIFO_DOWNS)?;
        self.set_reg(REG_FIFO_CONFIG_1, SET_FIFO_CONFIG_1)?;
        // 5. Flush FIFO
        self.set_reg(REG_CMD, CMD_FIFO_FLUSH)?;
        self.reactor.pause(0.010);
        // 6. Start Bulk Reading
        // Start timer roughly immediately
        let rest_ticks = setup.mcu.seconds_to_clock(4. / setup.data_rate as f64);
        let cmd = setup
            .query_bmi160_cmd
            .borrow()
            .clone()
            .ok_or_else(missing_query_cmd)?;
        cmd.send(vec![py_int(setup.oid), py_int(rest_ticks)], SendOpts::default())?;
        klippy_api::log::info(&fmt(
            "BMI160 starting '%s' measurements",
            &[PctValue::Str(setup.name.clone())],
        ));
        setup.ffreader.note_start()?;
        setup.last_error_count.set(0);
        Ok(())
    }

    /// `_finish_measurements` (`bmi160.py:171`).
    pub fn finish_measurements(&self) -> KResult<()> {
        let setup = self.setup("bus")?;
        self.set_reg(REG_CMD, CMD_ACC_PM_SUSPEND)?;
        let cmd = setup
            .query_bmi160_cmd
            .borrow()
            .clone()
            .ok_or_else(missing_query_cmd)?;
        cmd.send_wait_ack(vec![py_int(setup.oid), py_int(0)], SendOpts::default())?;
        setup.ffreader.note_end()?;
        klippy_api::log::info(&fmt(
            "BMI160 finished '%s' measurements",
            &[PctValue::Str(setup.name.clone())],
        ));
        Ok(())
    }

    /// `_process_batch(eventtime)` (`bmi160.py:176`).
    pub fn process_batch(&self) -> KResult<PyValue> {
        let setup = self.setup("ffreader")?;
        let mut samples = setup.ffreader.pull_samples()?;
        self.convert_samples(&mut samples)?;
        if samples.is_empty() {
            return Ok(PyValue::Dict(indexmap::IndexMap::new()));
        }
        let mut d = indexmap::IndexMap::new();
        d.insert(PyValue::Str("data".into()), PyValue::List(samples));
        d.insert(PyValue::Str("errors".into()), py_int(setup.last_error_count.get()));
        d.insert(PyValue::Str("overflows".into()), py_int(setup.ffreader.get_last_overflows()));
        Ok(PyValue::Dict(d))
    }
}

impl AccelChipApi for Bmi160 {
    fn start_internal_client(&self) -> KResult<Arc<AccelQueryHelper>> {
        Bmi160::start_internal_client(self)
    }

    fn read_reg(&self, reg: i64) -> KResult<i64> {
        Bmi160::read_reg(self, reg)
    }

    fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        Bmi160::set_reg(self, reg, val)
    }

    fn name(&self) -> String {
        Bmi160::name(self)
    }
}

klippy_api::printer_object!(Bmi160; dyn AccelChipApi);

/// `bytearray(params['response'])` for the MCU response dict.
fn extract_response_bytes(params: &PyValue) -> KResult<Vec<u8>> {
    let response = match params {
        PyValue::Dict(d) => d
            .iter()
            .find(|(k, _)| matches!(k, PyValue::Str(s) if s == "response"))
            .map(|(_, v)| v.clone())
            .ok_or_else(missing_response)?,
        other => other.clone(),
    };
    match response {
        PyValue::Bytes(b) => Ok(b),
        PyValue::List(items) => {
            let mut out = Vec::with_capacity(items.len());
            for item in &items {
                match item {
                    PyValue::Int(n) => match n.inner().to_string().parse::<i64>() {
                        Ok(v) if (0..=255).contains(&v) => out.push(v as u8),
                        _ => {
                            return Err(KlippyError::Internal {
                                class_name: "ValueError".into(),
                                message: "byte must be in range(0, 256)".into(),
                            })
                        }
                    },
                    other => {
                        return Err(KlippyError::Internal {
                            class_name: "TypeError".into(),
                            message: format!(
                                "cannot convert '{}' object to bytes",
                                pycompat::value::type_name(other)
                            ),
                        })
                    }
                }
            }
            Ok(out)
        }
        other => Err(KlippyError::Internal {
            class_name: "TypeError".into(),
            message: format!(
                "cannot convert '{}' object to bytes",
                pycompat::value::type_name(&other)
            ),
        }),
    }
}

/// `load_config` / `load_config_prefix` (`bmi160.py:184-188`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Bmi160::new(config)? as ObjectRef))
}

pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    load_config(config)
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
