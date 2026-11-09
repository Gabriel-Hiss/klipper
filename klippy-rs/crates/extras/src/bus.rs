//! Port of the `bus` module (`klippy/extras/bus.py`).
//!
//! Helper code for SPI and I2C bus communication, plus bus-synchronized
//! digital outputs. This module has no printer-object entry points in
//! Python (no `load_config` / `load_config_prefix` /
//! `add_printer_objects`), so [`module_factory`] returns `None` for
//! every entry point, exactly as the reference module would.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts, GetStrOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    get_printer_mcu, CommandQueue, CommandWrapper, CommandQueryWrapper, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, PinsApi, QuerySendOpts, SendOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, EV_KLIPPY_CONNECT};
use klippy_api::{lookup_interface, EventArg, GCell};
use pycompat::text::{percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

fn fmt(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

fn int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn string(s: &str) -> PctValue {
    PctValue::Str(s.to_string())
}

fn hex_data(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len() * 2);
    for &x in data {
        out.push_str(&format!("{x:02x}"));
    }
    out
}

fn pins_error(msg: String) -> KlippyError {
    KlippyError::Other { class_name: "error".to_string(), message: msg }
}

fn attr_error(what: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'NoneType' object has no attribute '{what}'"),
    }
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn py_bytes(v: &[u8]) -> PyValue {
    PyValue::Bytes(v.to_vec())
}

fn dict_get(params: &PyValue, key: &str) -> Option<PyValue> {
    if let PyValue::Dict(d) = params {
        for (k, v) in d {
            if let PyValue::Str(s) = k {
                if s == key {
                    return Some(v.clone());
                }
            }
        }
    }
    None
}

fn lookup_pins(mcu: &Mcu) -> KResult<Arc<dyn PinsApi>> {
    let printer = mcu.get_printer();
    lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
        .ok_or_else(|| pins_error("pins object not found".to_string()))
}

fn lookup_configfile(mcu: &Mcu) -> KResult<Arc<dyn PrinterConfigApi>> {
    let printer = mcu.get_printer();
    lookup_interface::<dyn PrinterConfigApi>(&*printer, "configfile", LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "configfile object not found".to_string(),
        })
}

// ---------------------------------------------------------------------------
// resolve_bus_name
// ---------------------------------------------------------------------------

/// `resolve_bus_name(mcu, param, bus)` (`klippy/extras/bus.py:9`).
///
/// Returns the bus name as a string. When no enumeration exists and no
/// bus was given, Python returns the integer `0`; the `"0"` string is
/// identical for every use here (config formatting and `BUS_PINS_0`).
pub fn resolve_bus_name(mcu: &Mcu, param: &str, bus: Option<&str>) -> KResult<String> {
    let enumerations = mcu.get_enumerations();
    let enums = enumerations.get(param).or_else(|| enumerations.get("bus"));
    let Some(enums) = enums else {
        return Ok(match bus {
            None => "0".to_string(),
            Some(b) => b.to_string(),
        });
    };
    let ppins = lookup_pins(mcu)?;
    let mcu_name = mcu.get_name();
    let bus = match bus {
        None => {
            // rev_enums = {v: k for k, v in enums.items()} (last wins).
            let mut rev: IndexMap<i64, String> = IndexMap::new();
            for (k, v) in enums.iter() {
                rev.insert(*v, k.clone());
            }
            match rev.get(&0) {
                None => {
                    return Err(pins_error(fmt(
                        "Must specify %s on mcu '%s'",
                        &[string(param), string(&mcu_name)],
                    )?))
                }
                Some(k) => k.clone(),
            }
        }
        Some(b) => b.to_string(),
    };
    if !enums.contains_key(&bus) {
        return Err(pins_error(fmt("Unknown %s '%s'", &[string(param), string(&bus)])?));
    }
    let constants = mcu.get_constants();
    let reserve_key = fmt("BUS_PINS_%s", &[string(&bus)])?;
    let reserve_pins = constants.get(&reserve_key);
    let pin_resolver = ppins.get_pin_resolver(&mcu_name)?;
    if let Some(reserve_pins) = reserve_pins {
        for pin in reserve_pins.split(',') {
            pin_resolver.reserve_pin(pin, &bus)?;
        }
    }
    Ok(bus)
}

// ---------------------------------------------------------------------------
// SPI
// ---------------------------------------------------------------------------

/// `MCU_SPI` trait (`klippy/extras/bus.py:42`), for consumers that hold
/// the bus as an interface.
pub trait McuSpiApi: Send + Sync + 'static {
    fn get_oid(&self) -> i64;
    fn get_mcu(&self) -> Mcu;
    fn get_command_queue(&self) -> CommandQueue;
    fn build_config(&self) -> KResult<()>;
    fn setup_shutdown_msg(&self, shutdown_seq: &[u8]) -> KResult<()>;
    fn spi_send(&self, data: &[u8], opts: SendOpts) -> KResult<()>;
    fn spi_transfer(&self, data: &[u8], opts: QuerySendOpts) -> KResult<PyValue>;
    fn spi_transfer_with_preface(
        &self,
        preface_data: &[u8],
        data: &[u8],
        opts: QuerySendOpts,
    ) -> KResult<PyValue>;
}

/// `MCU_SPI` (`klippy/extras/bus.py:42`).
pub struct McuSpi {
    mcu: Mcu,
    bus: Option<String>,
    speed: i64,
    oid: i64,
    config_fmt: GCell<String>,
    config_fmt_ticks: Option<String>,
    cmd_queue: CommandQueue,
    spi_send_cmd: GCell<Option<CommandWrapper>>,
    spi_transfer_cmd: GCell<Option<CommandQueryWrapper>>,
}

impl McuSpi {
    /// `MCU_SPI.__init__` (`klippy/extras/bus.py:43`).
    pub fn new(
        mcu: Mcu,
        bus: Option<String>,
        pin: Option<String>,
        mode: i64,
        speed: i64,
        sw_pins: Option<[String; 3]>,
        cs_active_high: bool,
    ) -> KResult<Arc<McuSpi>> {
        let oid = mcu.create_oid()?;
        match &pin {
            None => {
                mcu.add_config_cmd(
                    &fmt("config_spi_without_cs oid=%d", &[int(oid)])?,
                    Default::default(),
                )?;
            }
            Some(pin) => {
                mcu.add_config_cmd(
                    &fmt(
                        "config_spi oid=%d pin=%s cs_active_high=%d",
                        &[int(oid), string(pin), PctValue::Bool(cs_active_high)],
                    )?,
                    Default::default(),
                )?;
            }
        }
        let (config_fmt, config_fmt_ticks) = match &sw_pins {
            Some(sw_pins) => {
                let config_fmt = fmt(
                    "spi_set_software_bus oid=%d miso_pin=%s mosi_pin=%s sclk_pin=%s mode=%d rate=%d",
                    &[
                        int(oid),
                        string(&sw_pins[0]),
                        string(&sw_pins[1]),
                        string(&sw_pins[2]),
                        int(mode),
                        int(speed),
                    ],
                )?;
                let config_fmt_ticks = fmt(
                    "spi_set_sw_bus oid=%d miso_pin=%s mosi_pin=%s sclk_pin=%s mode=%d pulse_ticks=%%d",
                    &[
                        int(oid),
                        string(&sw_pins[0]),
                        string(&sw_pins[1]),
                        string(&sw_pins[2]),
                        int(mode),
                    ],
                )?;
                (config_fmt, Some(config_fmt_ticks))
            }
            None => (
                fmt(
                    "spi_set_bus oid=%d spi_bus=%%s mode=%d rate=%d",
                    &[int(oid), int(mode), int(speed)],
                )?,
                None,
            ),
        };
        let cmd_queue = mcu.alloc_command_queue();
        let this = Arc::new(McuSpi {
            mcu: mcu.clone(),
            bus,
            speed,
            oid,
            config_fmt: GCell::new(config_fmt),
            config_fmt_ticks,
            cmd_queue,
            spi_send_cmd: GCell::new(None),
            spi_transfer_cmd: GCell::new(None),
        });
        let this2 = this.clone();
        mcu.register_config_callback(Box::new(move || this2.build_config()))?;
        Ok(this)
    }

    /// `setup_shutdown_msg` (`klippy/extras/bus.py:74`).
    pub fn setup_shutdown_msg(&self, shutdown_seq: &[u8]) -> KResult<()> {
        let shutdown_msg = hex_data(shutdown_seq);
        let oid = self.mcu.create_oid()?;
        self.mcu.add_config_cmd(
            &fmt(
                "config_spi_shutdown oid=%d spi_oid=%d shutdown_msg=%s",
                &[int(oid), int(self.oid), string(&shutdown_msg)],
            )?,
            Default::default(),
        )
    }

    /// `build_config` (`klippy/extras/bus.py:85`).
    pub fn build_config(&self) -> KResult<()> {
        let mut config_fmt = self.config_fmt.cloned();
        if config_fmt.contains('%') {
            let bus = resolve_bus_name(&self.mcu, "spi_bus", self.bus.as_deref())?;
            config_fmt = fmt(&config_fmt, &[string(&bus)])?;
            *self.config_fmt.borrow_mut() = config_fmt.clone();
        }
        if let Some(config_fmt_ticks) = &self.config_fmt_ticks {
            if self.mcu.try_lookup_command(
                "spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u",
            ).is_some() {
                let pulse_ticks = self.mcu.seconds_to_clock(1. / self.speed as f64);
                config_fmt = fmt(config_fmt_ticks, &[int(pulse_ticks)])?;
                *self.config_fmt.borrow_mut() = config_fmt.clone();
            } else {
                let configfile = lookup_configfile(&self.mcu)?;
                configfile.deprecate_mcu_code(&self.mcu, "spi_set_sw_bus", None);
            }
        }
        self.mcu.add_config_cmd(&config_fmt, Default::default())?;
        let spi_send_cmd = self.mcu.lookup_command(
            "spi_send oid=%c data=%*s",
            Some(self.cmd_queue),
        )?;
        *self.spi_send_cmd.borrow_mut() = Some(spi_send_cmd);
        let spi_transfer_cmd = self.mcu.lookup_query_command(
            "spi_transfer oid=%c data=%*s",
            "spi_transfer_response oid=%c response=%*s",
            LookupQueryCommandOpts {
                oid: Some(self.oid),
                cq: Some(self.cmd_queue),
                is_async: false,
            },
        )?;
        *self.spi_transfer_cmd.borrow_mut() = Some(spi_transfer_cmd);
        Ok(())
    }

    /// `spi_send` (`klippy/extras/bus.py:105`).
    pub fn spi_send(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        let cmd = self.spi_send_cmd.borrow().clone();
        match cmd {
            None => {
                let data_msg = hex_data(data);
                self.mcu.add_config_cmd(
                    &fmt(
                        "spi_send oid=%d data=%s",
                        &[int(self.oid), string(&data_msg)],
                    )?,
                    klippy_api::mcu::AddConfigCmdOpts { is_init: true, on_restart: false },
                )
            }
            Some(cmd) => cmd.send(vec![py_int(self.oid), py_bytes(data)], opts),
        }
    }

    /// `spi_transfer` (`klippy/extras/bus.py:114`).
    pub fn spi_transfer(&self, data: &[u8], opts: QuerySendOpts) -> KResult<PyValue> {
        let cmd = self.spi_transfer_cmd.borrow().clone();
        match cmd {
            None => Err(attr_error("send")),
            Some(cmd) => cmd.send(vec![py_int(self.oid), py_bytes(data)], opts),
        }
    }

    /// `spi_transfer_with_preface` (`klippy/extras/bus.py:117`).
    pub fn spi_transfer_with_preface(
        &self,
        preface_data: &[u8],
        data: &[u8],
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        let send_cmd = self.spi_send_cmd.borrow().clone();
        let transfer_cmd = self.spi_transfer_cmd.borrow().clone();
        match (send_cmd, transfer_cmd) {
            (_, None) => Err(attr_error("send_with_preface")),
            (None, _) => Err(attr_error("_cmd")),
            (Some(send_cmd), Some(transfer_cmd)) => transfer_cmd.send_with_preface(
                send_cmd,
                vec![py_int(self.oid), py_bytes(preface_data)],
                vec![py_int(self.oid), py_bytes(data)],
                opts,
            ),
        }
    }
}

impl McuSpiApi for McuSpi {
    fn get_oid(&self) -> i64 { self.oid }
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn get_command_queue(&self) -> CommandQueue { self.cmd_queue }
    fn build_config(&self) -> KResult<()> { McuSpi::build_config(self) }
    fn setup_shutdown_msg(&self, shutdown_seq: &[u8]) -> KResult<()> {
        McuSpi::setup_shutdown_msg(self, shutdown_seq)
    }
    fn spi_send(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        McuSpi::spi_send(self, data, opts)
    }
    fn spi_transfer(&self, data: &[u8], opts: QuerySendOpts) -> KResult<PyValue> {
        McuSpi::spi_transfer(self, data, opts)
    }
    fn spi_transfer_with_preface(
        &self,
        preface_data: &[u8],
        data: &[u8],
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        McuSpi::spi_transfer_with_preface(self, preface_data, data, opts)
    }
}

klippy_api::printer_object!(McuSpi; dyn McuSpiApi);

/// `MCU_SPI_from_config` (`klippy/extras/bus.py:124`).
pub fn mcu_spi_from_config(
    config: &Config,
    mode: i64,
    pin_option: &str,
    default_speed: i64,
    share_type: Option<&str>,
    cs_active_high: bool,
) -> KResult<Arc<McuSpi>> {
    let printer = config.get_printer();
    let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
        .ok_or_else(|| pins_error("pins object not found".to_string()))?;
    let cs_pin = config.get(pin_option, GetStrOpts::default())?.expect("required");
    let cs_pin_params = ppins.lookup_pin(
        &cs_pin,
        LookupPinOpts { can_invert: false, can_pullup: false, share_type: share_type.map(str::to_string) },
    )?;
    let pin = if cs_pin_params.pin == "None" {
        ppins.reset_pin_sharing(cs_pin_params.clone());
        None
    } else {
        Some(cs_pin_params.pin.clone())
    };
    let mcu = cs_pin_params.chip.clone();
    let speed = config.getint(
        "spi_speed",
        GetIntOpts { default: Some(Some(default_speed)), minval: Some(100000), maxval: None, note_valid: true },
    )?.expect("default");
    let (sw_pins, bus) = if config.get(
        "spi_software_sclk_pin",
        GetStrOpts { default: Some(None), note_valid: true },
    )?.is_some() {
        // `sw_pin_params = [ppins.lookup_pin(config.get(name), share_type=name)
        //  for name in sw_pin_names]`: every pin is looked up, with the option
        // name as its share type, before any chip is checked.
        let mut sw_pin_params = Vec::new();
        for name in ["miso", "mosi", "sclk"] {
            let opt = format!("spi_software_{name}_pin");
            let val = config.get(&opt, GetStrOpts::default())?.expect("required");
            sw_pin_params.push(ppins.lookup_pin(
                &val,
                LookupPinOpts { can_invert: false, can_pullup: false, share_type: Some(opt) },
            )?);
        }
        for pin_params in &sw_pin_params {
            if !Arc::ptr_eq(&pin_params.chip, &mcu) {
                return Err(pins_error(fmt(
                    "%s: spi pins must be on same mcu",
                    &[string(&config.get_name())],
                )?));
            }
        }
        let sw_pins: Vec<String> = sw_pin_params.iter().map(|p| p.pin.clone()).collect();
        (Some([sw_pins[0].clone(), sw_pins[1].clone(), sw_pins[2].clone()]), None)
    } else {
        let bus = config.get(
            "spi_bus",
            GetStrOpts { default: Some(None), note_valid: true },
        )?;
        (None, bus)
    };
    McuSpi::new(mcu, bus, pin, mode, speed, sw_pins, cs_active_high)
}

// ---------------------------------------------------------------------------
// I2C
// ---------------------------------------------------------------------------

/// `MCU_I2C` trait (`klippy/extras/bus.py:161`).
pub trait McuI2cApi: Send + Sync + 'static {
    fn get_oid(&self) -> i64;
    fn get_mcu(&self) -> Mcu;
    fn get_i2c_address(&self) -> i64;
    fn get_command_queue(&self) -> CommandQueue;
    fn build_config(&self) -> KResult<()>;
    fn i2c_write_noack(&self, data: &[u8], opts: SendOpts) -> KResult<()>;
    fn i2c_write(&self, data: &[u8], opts: QuerySendOpts) -> KResult<()>;
    fn i2c_read(&self, write: &[u8], read_len: i64, opts: QuerySendOpts) -> KResult<PyValue>;
    fn i2c_transfer(
        &self,
        write: &[u8],
        read_len: i64,
        opts: QuerySendOpts,
    ) -> KResult<PyValue>;
}

/// `MCU_I2C` (`klippy/extras/bus.py:161`).
pub struct McuI2c {
    mcu: Mcu,
    bus: Option<String>,
    i2c_address: i64,
    speed: i64,
    async_write_only: bool,
    oid: i64,
    config_fmt: GCell<String>,
    config_fmt_ticks: Option<String>,
    cmd_queue: CommandQueue,
    i2c_write_cmd: GCell<Option<CommandWrapper>>,
    i2c_read_cmd: GCell<Option<CommandQueryWrapper>>,
    i2c_transfer_cmd: GCell<Option<CommandQueryWrapper>>,
    i2c_write_only_cmd: GCell<Option<CommandWrapper>>,
    i2c_transfer_plain_cmd: GCell<Option<CommandWrapper>>,
    to_write: GCell<Vec<Vec<u8>>>,
    configured: GCell<bool>,
}

impl McuI2c {
    /// `MCU_I2C.__init__` (`klippy/extras/bus.py:162`).
    pub fn new(
        mcu: Mcu,
        bus: Option<String>,
        addr: i64,
        speed: i64,
        sw_pins: Option<[String; 2]>,
        async_write_only: bool,
    ) -> KResult<Arc<McuI2c>> {
        let oid = mcu.create_oid()?;
        mcu.add_config_cmd(&fmt("config_i2c oid=%d", &[int(oid)])?, Default::default())?;
        let (config_fmt, config_fmt_ticks) = match &sw_pins {
            Some(sw_pins) => {
                let config_fmt = fmt(
                    "i2c_set_software_bus oid=%d scl_pin=%s sda_pin=%s rate=%d address=%d",
                    &[
                        int(oid),
                        string(&sw_pins[0]),
                        string(&sw_pins[1]),
                        int(speed),
                        int(addr),
                    ],
                )?;
                let config_fmt_ticks = fmt(
                    "i2c_set_sw_bus oid=%d scl_pin=%s sda_pin=%s pulse_ticks=%%d address=%d",
                    &[int(oid), string(&sw_pins[0]), string(&sw_pins[1]), int(addr)],
                )?;
                (config_fmt, Some(config_fmt_ticks))
            }
            None => (
                fmt(
                    "i2c_set_bus oid=%d i2c_bus=%%s rate=%d address=%d",
                    &[int(oid), int(speed), int(addr)],
                )?,
                None,
            ),
        };
        let cmd_queue = mcu.alloc_command_queue();
        let this = Arc::new(McuI2c {
            mcu: mcu.clone(),
            bus,
            i2c_address: addr,
            speed,
            async_write_only,
            oid,
            config_fmt: GCell::new(config_fmt),
            config_fmt_ticks,
            cmd_queue,
            i2c_write_cmd: GCell::new(None),
            i2c_read_cmd: GCell::new(None),
            i2c_transfer_cmd: GCell::new(None),
            i2c_write_only_cmd: GCell::new(None),
            i2c_transfer_plain_cmd: GCell::new(None),
            to_write: GCell::new(Vec::new()),
            configured: GCell::new(false),
        });
        let this2 = this.clone();
        mcu.register_config_callback(Box::new(move || this2.build_config()))?;
        let this3 = this.clone();
        let printer = mcu.get_printer();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_args: &[EventArg]| {
                this3.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        Ok(this)
    }

    fn handle_connect(&self) -> KResult<()> {
        let mut idx = 0usize;
        loop {
            let data = {
                let to_write = self.to_write.borrow();
                if idx >= to_write.len() {
                    break;
                }
                to_write[idx].clone()
            };
            self.i2c_write(&data, QuerySendOpts::default())?;
            idx += 1;
        }
        Ok(())
    }


    /// `build_config` (`klippy/extras/bus.py:214`).
    pub fn build_config(&self) -> KResult<()> {
        let mut config_fmt = self.config_fmt.cloned();
        if config_fmt.contains('%') {
            let bus = resolve_bus_name(&self.mcu, "i2c_bus", self.bus.as_deref())?;
            config_fmt = fmt(&config_fmt, &[string(&bus)])?;
            *self.config_fmt.borrow_mut() = config_fmt.clone();
        }
        if let Some(config_fmt_ticks) = &self.config_fmt_ticks {
            if self.mcu.try_lookup_command(
                "i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u",
            ).is_some() {
                let pulse_ticks = self.mcu.seconds_to_clock(1. / self.speed as f64 / 2.);
                config_fmt = fmt(config_fmt_ticks, &[int(pulse_ticks)])?;
                *self.config_fmt.borrow_mut() = config_fmt.clone();
            } else {
                let configfile = lookup_configfile(&self.mcu)?;
                configfile.deprecate_mcu_code(&self.mcu, "i2c_set_sw_bus", None);
            }
        }
        self.mcu.add_config_cmd(&config_fmt, Default::default())?;
        if self.mcu.try_lookup_command("i2c_read oid=%c reg=%*s read_len=%u").is_some() {
            let i2c_write_cmd = self.mcu.lookup_command(
                "i2c_write oid=%c data=%*s",
                Some(self.cmd_queue),
            )?;
            *self.i2c_write_only_cmd.borrow_mut() = Some(i2c_write_cmd.clone());
            *self.i2c_write_cmd.borrow_mut() = Some(i2c_write_cmd);
            let i2c_read_cmd = self.mcu.lookup_query_command(
                "i2c_read oid=%c reg=%*s read_len=%u",
                "i2c_read_response oid=%c response=%*s",
                LookupQueryCommandOpts {
                    oid: Some(self.oid),
                    cq: Some(self.cmd_queue),
                    is_async: false,
                },
            )?;
            *self.i2c_read_cmd.borrow_mut() = Some(i2c_read_cmd);
            let configfile = lookup_configfile(&self.mcu)?;
            configfile.deprecate_mcu_code(&self.mcu, "i2c_transfer", None);
        } else {
            let i2c_transfer_cmd = self.mcu.lookup_query_command(
                "i2c_transfer oid=%c write=%*s read_len=%u",
                "i2c_response oid=%c i2c_bus_status=%c response=%*s",
                LookupQueryCommandOpts {
                    oid: Some(self.oid),
                    cq: Some(self.cmd_queue),
                    is_async: false,
                },
            )?;
            *self.i2c_transfer_cmd.borrow_mut() = Some(i2c_transfer_cmd);
            let i2c_write_only_cmd = self.mcu.lookup_command(
                "i2c_transfer oid=%c write=%*s read_len=%u",
                Some(self.cmd_queue),
            )?;
            *self.i2c_write_only_cmd.borrow_mut() = Some(i2c_write_only_cmd);
            if self.mcu.is_fileoutput() {
                let plain = self.mcu.lookup_command(
                    "i2c_transfer oid=%c write=%*s read_len=%u",
                    Some(self.cmd_queue),
                )?;
                *self.i2c_transfer_plain_cmd.borrow_mut() = Some(plain);
            }
            if self.async_write_only {
                let mcu_name = self.mcu.get_name();
                let addr = self.i2c_address;
                self.mcu.register_serial_response(
                    Arc::new(move |params: PyValue| {
                        let status = dict_get(&params, "i2c_bus_status");
                        let status_str = status.as_ref().map(|v| v.str()).unwrap_or_default();
                        if status_str == "SUCCESS" {
                            return;
                        }
                        let err_msg = fmt(
                            "MCU '%s' I2C request to addr %i reports error %s",
                            &[string(&mcu_name), int(addr), string(&status_str)],
                        )
                        .unwrap_or_else(|_| "I2C error".to_string());
                        klippy_api::log::error(&err_msg);
                    }),
                    "i2c_response oid=%c i2c_bus_status=%c response=%*s",
                    Some(self.oid),
                );
            }
        }
        self.configured.set(true);
        Ok(())
    }

    /// `i2c_write_noack` (`klippy/extras/bus.py:256`).
    pub fn i2c_write_noack(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        if self.async_write_only {
            let cmd = self.i2c_write_only_cmd.borrow().clone();
            return match cmd {
                None => Err(attr_error("send")),
                Some(cmd) => cmd.send(vec![py_int(self.oid), py_bytes(data), py_int(0)], opts),
            };
        }
        let cmd = self.i2c_write_cmd.borrow().clone();
        match cmd {
            None => Err(attr_error("send")),
            Some(cmd) => cmd.send(vec![py_int(self.oid), py_bytes(data)], opts),
        }
    }

    /// `i2c_write` (`klippy/extras/bus.py:263`).
    pub fn i2c_write(&self, data: &[u8], opts: QuerySendOpts) -> KResult<()> {
        if !self.configured.get() {
            self.to_write.borrow_mut().push(data.to_vec());
            return Ok(());
        }
        if self.async_write_only {
            let cmd = self.i2c_write_only_cmd.borrow().clone();
            return match cmd {
                None => Err(attr_error("send")),
                Some(cmd) => cmd.send(
                    vec![py_int(self.oid), py_bytes(data), py_int(0)],
                    SendOpts { minclock: opts.minclock, reqclock: opts.reqclock },
                ),
            };
        }
        let has_transfer = self.i2c_transfer_cmd.borrow().is_some()
            || self.i2c_transfer_plain_cmd.borrow().is_some();
        if has_transfer {
            self.i2c_transfer(data, 0, opts)?;
            return Ok(());
        }
        let cmd = self.i2c_write_cmd.borrow().clone();
        match cmd {
            None => Err(attr_error("send_wait_ack")),
            Some(cmd) => cmd.send_wait_ack(
                vec![py_int(self.oid), py_bytes(data)],
                SendOpts { minclock: opts.minclock, reqclock: opts.reqclock },
            ),
        }
    }

    /// `i2c_read` (`klippy/extras/bus.py:277`).
    pub fn i2c_read(&self, write: &[u8], read_len: i64, opts: QuerySendOpts) -> KResult<PyValue> {
        if self.async_write_only {
            return Err(KlippyError::Mcu("I2C dev is write only".to_string()));
        }
        let opts = QuerySendOpts { minclock: 0, reqclock: 0, retry: opts.retry };
        let has_transfer = self.i2c_transfer_cmd.borrow().is_some()
            || self.i2c_transfer_plain_cmd.borrow().is_some();
        if has_transfer {
            return self.i2c_transfer(write, read_len, opts);
        }
        let cmd = self.i2c_read_cmd.borrow().clone();
        match cmd {
            None => Err(attr_error("send")),
            Some(cmd) => cmd.send(vec![py_int(self.oid), py_bytes(write), py_int(read_len)], opts),
        }
    }

    /// `i2c_transfer` (`klippy/extras/bus.py:284`).
    pub fn i2c_transfer(&self, write: &[u8], read_len: i64, opts: QuerySendOpts) -> KResult<PyValue> {
        if self.async_write_only {
            return Err(KlippyError::Mcu("I2C dev is write only".to_string()));
        }
        if self.mcu.is_fileoutput() {
            let cmd = self.i2c_transfer_plain_cmd.borrow().clone();
            return match cmd {
                None => Err(attr_error("send")),
                Some(cmd) => {
                    cmd.send(
                        vec![py_int(self.oid), py_bytes(write), py_int(read_len)],
                        SendOpts { minclock: opts.minclock, reqclock: opts.reqclock },
                    )?;
                    Ok(PyValue::None)
                }
            };
        }
        let cmd = self.i2c_transfer_cmd.borrow().clone();
        let cmd = match cmd {
            None => return Err(attr_error("send")),
            Some(c) => c,
        };
        let param = cmd.send(vec![py_int(self.oid), py_bytes(write), py_int(read_len)], opts)?;
        let status = dict_get(&param, "i2c_bus_status");
        let status_str = status.as_ref().map(|v| v.str()).unwrap_or_default();
        if status_str == "SUCCESS" {
            return Ok(param);
        }
        let err_msg = fmt(
            "MCU '%s' I2C request to addr %i reports error %s",
            &[string(&self.mcu.get_name()), int(self.i2c_address), string(&status_str)],
        )?;
        self.mcu.get_printer().invoke_shutdown(&err_msg, PyValue::Dict(IndexMap::new()));
        Ok(PyValue::None)
    }
}

impl McuI2cApi for McuI2c {
    fn get_oid(&self) -> i64 { self.oid }
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn get_i2c_address(&self) -> i64 { self.i2c_address }
    fn get_command_queue(&self) -> CommandQueue { self.cmd_queue }
    fn build_config(&self) -> KResult<()> { McuI2c::build_config(self) }
    fn i2c_write_noack(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        McuI2c::i2c_write_noack(self, data, opts)
    }
    fn i2c_write(&self, data: &[u8], opts: QuerySendOpts) -> KResult<()> {
        McuI2c::i2c_write(self, data, opts)
    }
    fn i2c_read(&self, write: &[u8], read_len: i64, opts: QuerySendOpts) -> KResult<PyValue> {
        McuI2c::i2c_read(self, write, read_len, opts)
    }
    fn i2c_transfer(
        &self,
        write: &[u8],
        read_len: i64,
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        McuI2c::i2c_transfer(self, write, read_len, opts)
    }
}

klippy_api::printer_object!(McuI2c; dyn McuI2cApi);

/// `MCU_I2C_from_config` (`klippy/extras/bus.py:302`).
pub fn mcu_i2c_from_config(
    config: &Config,
    default_addr: Option<i64>,
    default_speed: i64,
    async_write_only: bool,
) -> KResult<Arc<McuI2c>> {
    let printer = config.get_printer();
    let i2c_mcu_name = config.get(
        "i2c_mcu",
        GetStrOpts { default: Some(Some("mcu".to_string())), note_valid: true },
    )?.expect("default");
    let i2c_mcu = get_printer_mcu(&printer, &i2c_mcu_name)?;
    let speed = config.getint(
        "i2c_speed",
        GetIntOpts { default: Some(Some(default_speed)), minval: Some(100000), maxval: None, note_valid: true },
    )?.expect("default");
    let addr = match default_addr {
        None => config.getint(
            "i2c_address",
            GetIntOpts { default: None, minval: Some(0), maxval: Some(127), note_valid: true },
        )?.expect("required"),
        Some(default_addr) => config.getint(
            "i2c_address",
            GetIntOpts { default: Some(Some(default_addr)), minval: Some(0), maxval: Some(127), note_valid: true },
        )?.expect("default"),
    };
    let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
        .ok_or_else(|| pins_error("pins object not found".to_string()))?;
    let (sw_pins, bus) = if config.get(
        "i2c_software_scl_pin",
        GetStrOpts { default: Some(None), note_valid: true },
    )?.is_some() {
        // Same shape as the SPI case: look up every pin, then check chips.
        let mut sw_pin_params = Vec::new();
        for name in ["scl", "sda"] {
            let opt = format!("i2c_software_{name}_pin");
            let val = config.get(&opt, GetStrOpts::default())?.expect("required");
            sw_pin_params.push(ppins.lookup_pin(
                &val,
                LookupPinOpts { can_invert: false, can_pullup: false, share_type: Some(opt) },
            )?);
        }
        for pin_params in &sw_pin_params {
            if !Arc::ptr_eq(&pin_params.chip, &i2c_mcu) {
                return Err(pins_error(fmt(
                    "%s: i2c pins must be on same mcu",
                    &[string(&config.get_name())],
                )?));
            }
        }
        let sw_pins: Vec<String> = sw_pin_params.iter().map(|p| p.pin.clone()).collect();
        (Some([sw_pins[0].clone(), sw_pins[1].clone()]), None)
    } else {
        let bus = config.get(
            "i2c_bus",
            GetStrOpts { default: Some(None), note_valid: true },
        )?;
        (None, bus)
    };
    McuI2c::new(i2c_mcu, bus, addr, speed, sw_pins, async_write_only)
}

// ---------------------------------------------------------------------------
// Bus synchronized digital outputs
// ---------------------------------------------------------------------------

/// `MCU_bus_digital_out` trait (`klippy/extras/bus.py:337`).
pub trait McuBusDigitalOutApi: Send + Sync + 'static {
    fn get_oid(&self) -> i64;
    fn get_mcu(&self) -> Mcu;
    fn get_command_queue(&self) -> CommandQueue;
    fn build_config(&self) -> KResult<()>;
    fn update_digital_out(&self, value: bool, opts: SendOpts) -> KResult<()>;
}

/// `MCU_bus_digital_out` (`klippy/extras/bus.py:337`).
pub struct McuBusDigitalOut {
    mcu: Mcu,
    oid: i64,
    cmd_queue: CommandQueue,
    update_pin_cmd: GCell<Option<CommandWrapper>>,
}

impl McuBusDigitalOut {
    /// `MCU_bus_digital_out.__init__` (`klippy/extras/bus.py:338`).
    pub fn new(
        mcu: Mcu,
        pin_desc: &str,
        cmd_queue: Option<CommandQueue>,
        value: i64,
    ) -> KResult<Arc<McuBusDigitalOut>> {
        let oid = mcu.create_oid()?;
        let ppins = lookup_pins(&mcu)?;
        let pin_params = ppins.lookup_pin(
            pin_desc,
            LookupPinOpts { can_invert: false, can_pullup: false, share_type: None },
        )?;
        if !Arc::ptr_eq(&pin_params.chip, &mcu) {
            return Err(pins_error(fmt(
                "Pin %s must be on mcu %s",
                &[string(pin_desc), string(&mcu.get_name())],
            )?));
        }
        mcu.add_config_cmd(
            &fmt(
                "config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
                &[int(oid), string(&pin_params.pin), int(value), int(value), int(0)],
            )?,
            Default::default(),
        )?;
        let cmd_queue = match cmd_queue {
            None => mcu.alloc_command_queue(),
            Some(cq) => cq,
        };
        let this = Arc::new(McuBusDigitalOut {
            mcu: mcu.clone(),
            oid,
            cmd_queue,
            update_pin_cmd: GCell::new(None),
        });
        let this2 = this.clone();
        mcu.register_config_callback(Box::new(move || this2.build_config()))?;
        Ok(this)
    }

    /// `build_config` (`klippy/extras/bus.py:360`).
    pub fn build_config(&self) -> KResult<()> {
        let cmd = self.mcu.lookup_command(
            "update_digital_out oid=%c value=%c",
            Some(self.cmd_queue),
        )?;
        *self.update_pin_cmd.borrow_mut() = Some(cmd);
        Ok(())
    }

    /// `update_digital_out` (`klippy/extras/bus.py:363`).
    pub fn update_digital_out(&self, value: bool, opts: SendOpts) -> KResult<()> {
        let cmd = self.update_pin_cmd.borrow().clone();
        match cmd {
            None => {
                self.mcu.add_config_cmd(
                    &fmt(
                        "update_digital_out oid=%c value=%c",
                        &[int(self.oid), PctValue::Bool(value)],
                    )?,
                    Default::default(),
                )
            }
            Some(cmd) => cmd.send(vec![py_int(self.oid), PyValue::Bool(value)], opts),
        }
    }
}

impl McuBusDigitalOutApi for McuBusDigitalOut {
    fn get_oid(&self) -> i64 { self.oid }
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn get_command_queue(&self) -> CommandQueue { self.cmd_queue }
    fn build_config(&self) -> KResult<()> { McuBusDigitalOut::build_config(self) }
    fn update_digital_out(&self, value: bool, opts: SendOpts) -> KResult<()> {
        McuBusDigitalOut::update_digital_out(self, value, opts)
    }
}

klippy_api::printer_object!(McuBusDigitalOut; dyn McuBusDigitalOutApi);

// ---------------------------------------------------------------------------
// Module factory
// ---------------------------------------------------------------------------

/// Module entry point. `bus.py` defines no `load_config`,
/// `load_config_prefix`, `add_early_printer_objects` or
/// `add_printer_objects`, so every entry point is `None`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
