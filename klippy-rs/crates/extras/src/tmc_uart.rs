//! Port of the `tmc_uart` module (`klippy/extras/tmc_uart.py`).
//!
//! Bit-banged TMC UART communication, the analog mux that multiplexes a
//! single UART over several drivers, and the per-MCU mutexes shared by all
//! UART users on one MCU.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts, GetListOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandQueryWrapper, CommandWrapper, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, ParsePinOpts, PinsApi, PinParams, QuerySendOpts,
    SendOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{downcast_arc, missing_interface, ObjectRef};
use pycompat::{PyInt, PyValue};
use reactor::{ReactorMutex};
use std::sync::Arc;

pub const TMC_BAUD_RATE: i64 = 40000;
pub const TMC_BAUD_RATE_AVR: i64 = 9000;

/// Result of a bit-banged register read (`{'data': ..., '#receive_time': ...}`).
#[derive(Debug, Clone, PartialEq)]
pub struct RegRead {
    pub data: Option<i64>,
    pub receive_time: f64,
}

impl RegRead {
    pub fn to_pyvalue(&self) -> PyValue {
        let data = match self.data {
            Some(v) => PyValue::Int(PyInt::from_i64(v)),
            None => PyValue::None,
        };
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("data".to_string()), data);
        map.insert(PyValue::Str("#receive_time".to_string()), PyValue::Float(self.receive_time));
        PyValue::Dict(map)
    }
}

fn key_error(key: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: pycompat::value::repr_key(key),
    }
}

fn value_error_byte() -> KlippyError {
    KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "byte must be in range(0, 256)".to_string(),
    }
}

fn to_byte(v: i64) -> KResult<u8> {
    if (0..=255).contains(&v) {
        // `as` is on integers only (no float), an exact narrowing check above
        // guarantees the value fits.
        Ok(v as u8)
    } else {
        Err(value_error_byte())
    }
}

// ---------------------------------------------------------------------------
// CRC8 and framing (pure functions)
// ---------------------------------------------------------------------------

/// Generate a CRC8-ATM value (`_calc_crc8`).
pub fn calc_crc8(data: &[u8]) -> u8 {
    let mut crc: u32 = 0;
    for &b0 in data {
        let mut b = b0 as u32;
        for _ in 0..8 {
            if ((crc >> 7) ^ (b & 0x01)) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            crc &= 0xff;
            b >>= 1;
        }
    }
    crc as u8
}

/// Add serial start and stop bits (`_add_serial_bits`).
pub fn add_serial_bits(data: &[u8]) -> Vec<u8> {
    let mut out: u128 = 0;
    let mut pos: u32 = 0;
    for &d in data {
        let b: u128 = ((d as u128) << 1) | 0x200;
        out |= b << pos;
        pos += 10;
    }
    let mut res = Vec::new();
    let count = pos.div_ceil(8) as usize;
    for i in 0..count {
        res.push(((out >> (i * 8)) & 0xff) as u8);
    }
    res
}

/// Generate a UART read register message (`_encode_read`).
pub fn encode_read(sync: u8, addr: u8, reg: u8) -> Vec<u8> {
    let mut msg = vec![sync, addr, reg];
    let crc = calc_crc8(&msg);
    msg.push(crc);
    add_serial_bits(&msg)
}

/// Generate a UART write register message (`_encode_write`).
pub fn encode_write(sync: u8, addr: u8, reg: u8, val: i64) -> Vec<u8> {
    let mut msg = vec![
        sync,
        addr,
        reg,
        (((val >> 24) & 0xff) as u8),
        (((val >> 16) & 0xff) as u8),
        (((val >> 8) & 0xff) as u8),
        ((val & 0xff) as u8),
    ];
    let crc = calc_crc8(&msg);
    msg.push(crc);
    add_serial_bits(&msg)
}

/// Extract a UART read response (`_decode_read`); `None` on failure.
pub fn decode_read(reg: u8, data: &[u8]) -> Option<i64> {
    if data.len() != 10 {
        return None;
    }
    let mut mval: u128 = 0;
    let mut pos: u32 = 0;
    for &d in data {
        mval |= (d as u128) << pos;
        pos += 8;
    }
    let val: i64 = ((((mval >> 31) & 0xff) as i64) << 24)
        | ((((mval >> 41) & 0xff) as i64) << 16)
        | ((((mval >> 51) & 0xff) as i64) << 8)
        | (((mval >> 61) & 0xff) as i64);
    let encoded = encode_write(0x05, 0xff, reg, val);
    if data != encoded.as_slice() {
        return None;
    }
    Some(val)
}

// ---------------------------------------------------------------------------
// Pins helpers
// ---------------------------------------------------------------------------

fn lookup_pins(printer: &Printer) -> KResult<Arc<dyn PinsApi>> {
    let opt = klippy_api::lookup_interface::<dyn PinsApi>(&**printer, "pins", LookupDefault::Raise)?;
    Ok(opt.expect("Raise never yields None"))
}

fn get_pins_for_mcu(mcu: &Mcu) -> KResult<Arc<dyn PinsApi>> {
    lookup_pins(&mcu.get_printer())
}

fn is_debugoutput(printer: &Printer) -> bool {
    match printer.get_start_args() {
        PyValue::Dict(map) => match map.get(&PyValue::Str("debugoutput".to_string())) {
            None => false,
            Some(PyValue::None) => false,
            Some(_) => true,
        },
        PyValue::None => false,
        _ => false,
    }
}

// ---------------------------------------------------------------------------
// Analog mux
// ---------------------------------------------------------------------------

/// `MCU_analog_mux` (`klippy/extras/tmc_uart.py:13`).
pub struct McuAnalogMux {
    mcu: Mcu,
    cmd_queue: CommandQueue,
    oids: Vec<i64>,
    pins: Vec<String>,
    pin_values: klippy_api::GCell<Vec<i64>>,
    update_pin_cmd: klippy_api::GCell<Option<CommandWrapper>>,
}

impl McuAnalogMux {
    /// `MCU_analog_mux.__init__(mcu, cmd_queue, select_pins_desc)`.
    pub fn new(
        mcu: Mcu,
        cmd_queue: CommandQueue,
        select_pins_desc: &[String],
    ) -> KResult<Arc<McuAnalogMux>> {
        let pins_api = get_pins_for_mcu(&mcu)?;
        let mut select_pin_params: Vec<PinParams> = Vec::new();
        for spd in select_pins_desc {
            select_pin_params.push(pins_api.lookup_pin(
                spd,
                LookupPinOpts { can_invert: true, can_pullup: false, share_type: None },
            )?);
        }
        let mut oids: Vec<i64> = Vec::new();
        for _pp in &select_pin_params {
            oids.push(mcu.create_oid()?);
        }
        let pins: Vec<String> = select_pin_params.iter().map(|pp| pp.pin.clone()).collect();
        let pin_values: Vec<i64> = select_pin_params.iter().map(|_| -1).collect();
        for (oid, pin) in oids.iter().zip(pins.iter()) {
            mcu.add_config_cmd(
                &format!(
                    "config_digital_out oid={oid} pin={pin} value=0 default_value=0 max_duration=0"
                ),
                AddConfigCmdOpts::default(),
            )?;
        }
        let this = Arc::new(McuAnalogMux {
            mcu: mcu.clone(),
            cmd_queue,
            oids,
            pins,
            pin_values: klippy_api::GCell::new(pin_values),
            update_pin_cmd: klippy_api::GCell::new(None),
        });
        let this2 = this.clone();
        mcu.register_config_callback(Box::new(move || this2.build_config()))?;
        Ok(this)
    }

    /// `MCU_analog_mux.build_config()`.
    pub fn build_config(&self) -> KResult<()> {
        let cmd = self
            .mcu
            .lookup_command("update_digital_out oid=%c value=%c", Some(self.cmd_queue))?;
        *self.update_pin_cmd.borrow_mut() = Some(cmd);
        Ok(())
    }

    /// `MCU_analog_mux.get_instance_id(select_pins_desc)`.
    pub fn get_instance_id(&self, select_pins_desc: &[String]) -> KResult<Vec<bool>> {
        let pins_api = get_pins_for_mcu(&self.mcu)?;
        let mut select_pin_params: Vec<PinParams> = Vec::new();
        for spd in select_pins_desc {
            select_pin_params.push(pins_api.parse_pin(
                spd,
                ParsePinOpts { can_invert: true, can_pullup: false },
            )?);
        }
        for pp in &select_pin_params {
            if !Arc::ptr_eq(&pp.chip, &self.mcu) {
                return Err(KlippyError::config("TMC mux pins must be on the same mcu"));
            }
        }
        let pins: Vec<String> = select_pin_params.iter().map(|pp| pp.pin.clone()).collect();
        if pins != self.pins {
            return Err(KlippyError::config("All TMC mux instances must use identical pins"));
        }
        Ok(select_pin_params.iter().map(|pp| pp.invert == 0).collect())
    }

    /// `MCU_analog_mux.activate(instance_id)`.
    pub fn activate(&self, instance_id: &[bool]) -> KResult<()> {
        let update: Option<CommandWrapper> = self.update_pin_cmd.borrow().clone();
        let oids = self.oids.clone();
        let old_vals = self.pin_values.cloned();
        let n = oids.len().min(old_vals.len()).min(instance_id.len());
        for i in 0..n {
            let new_bool = instance_id[i];
            let new_i: i64 = if new_bool { 1 } else { 0 };
            if old_vals[i] != new_i {
                let cmd = match &update {
                    Some(c) => c.clone(),
                    None => {
                        return Err(KlippyError::Internal {
                            class_name: "AttributeError".to_string(),
                            message: "'NoneType' object has no attribute 'send'".to_string(),
                        })
                    }
                };
                cmd.send(
                    vec![PyValue::Int(PyInt::from_i64(oids[i])), PyValue::Bool(new_bool)],
                    SendOpts::default(),
                )?;
            }
        }
        *self.pin_values.borrow_mut() =
            instance_id.iter().map(|b| if *b { 1 } else { 0 }).collect();
        Ok(())
    }

    pub fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    pub fn get_oids(&self) -> Vec<i64> {
        self.oids.clone()
    }
    pub fn get_pins(&self) -> Vec<String> {
        self.pins.clone()
    }
    pub fn get_pin_values(&self) -> Vec<i64> {
        self.pin_values.cloned()
    }
}

// ---------------------------------------------------------------------------
// Shared per-printer state (mutexes and bit-banged UARTs)
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct PrinterTmcUartMutexes {
    mcu_to_mutex: klippy_api::GCell<Vec<(Mcu, Arc<ReactorMutex>)>>,
    pin_to_uart: klippy_api::GCell<Vec<(String, Arc<McuTmcUartBitbang>)>>,
}

impl PrinterTmcUartMutexes {
    pub fn new() -> Self {
        PrinterTmcUartMutexes {
            mcu_to_mutex: klippy_api::GCell::new(Vec::new()),
            pin_to_uart: klippy_api::GCell::new(Vec::new()),
        }
    }
}

klippy_api::printer_object!(PrinterTmcUartMutexes);

fn get_shared_state(printer: &Printer) -> KResult<Arc<PrinterTmcUartMutexes>> {
    match printer.lookup_object("tmc_uart", LookupDefault::ReturnNone)? {
        Some(obj) => {
            downcast_arc::<PrinterTmcUartMutexes>(&obj)
                .ok_or_else(|| missing_interface::<PrinterTmcUartMutexes>("tmc_uart"))
        }
        None => {
            let new = Arc::new(PrinterTmcUartMutexes::new());
            printer.add_object("tmc_uart", new.clone() as ObjectRef)?;
            Ok(new)
        }
    }
}

/// `lookup_tmc_uart_mutex(mcu)` (`klippy/extras/tmc_uart.py:61`).
pub fn lookup_tmc_uart_mutex(mcu: &Mcu) -> KResult<Arc<ReactorMutex>> {
    let printer = mcu.get_printer();
    let shared = get_shared_state(&printer)?;
    {
        let existing = shared.mcu_to_mutex.borrow();
        for (m, mu) in existing.iter() {
            if Arc::ptr_eq(m, mcu) {
                return Ok(mu.clone());
            }
        }
    }
    let reactor = printer.get_reactor();
    let mutex = Arc::new(reactor.mutex(false));
    shared.mcu_to_mutex.borrow_mut().push((mcu.clone(), mutex.clone()));
    Ok(mutex)
}

// ---------------------------------------------------------------------------
// Bit-banged UART
// ---------------------------------------------------------------------------

/// Trait for the bit-banged UART (`MCU_TMC_uart_bitbang`).
pub trait TmcUartBitbangApi: Send + Sync + 'static {
    /// `reg_read(instance_id, addr, reg)`.
    fn reg_read(
        &self,
        instance_id: Option<&[bool]>,
        addr: i64,
        reg: i64,
    ) -> KResult<RegRead>;
    /// `reg_write(instance_id, addr, reg, val, print_time)`.
    fn reg_write(
        &self,
        instance_id: Option<&[bool]>,
        addr: i64,
        reg: i64,
        val: i64,
        print_time: Option<f64>,
    ) -> KResult<()>;
    /// `get_mcu()`.
    fn get_mcu(&self) -> Mcu;
    /// Shared mutex (`mutex` attribute).
    fn get_mutex(&self) -> Arc<ReactorMutex>;
}

/// `MCU_TMC_uart_bitbang` (`klippy/extras/tmc_uart.py:77`).
pub struct McuTmcUartBitbang {
    mcu: Mcu,
    mutex: Arc<ReactorMutex>,
    pullup: i64,
    rx_pin: String,
    tx_pin: String,
    oid: i64,
    cmd_queue: CommandQueue,
    analog_mux: Option<Arc<McuAnalogMux>>,
    instances: klippy_api::GCell<Vec<(Option<Vec<bool>>, i64)>>,
    tmcuart_send_cmd: klippy_api::GCell<Option<CommandQueryWrapper>>,
}

impl McuTmcUartBitbang {
    /// `MCU_TMC_uart_bitbang.__init__(rx_pin_params, tx_pin_params, select_pins_desc)`.
    pub fn new(
        rx_pin_params: &PinParams,
        tx_pin_params: &PinParams,
        select_pins_desc: Option<&[String]>,
    ) -> KResult<Arc<McuTmcUartBitbang>> {
        let mcu = rx_pin_params.chip.clone();
        let mutex = lookup_tmc_uart_mutex(&mcu)?;
        let pullup = rx_pin_params.pullup;
        let rx_pin = rx_pin_params.pin.clone();
        let tx_pin = tx_pin_params.pin.clone();
        let oid = mcu.create_oid()?;
        let cmd_queue = mcu.alloc_command_queue();
        let analog_mux = match select_pins_desc {
            Some(descs) => Some(McuAnalogMux::new(mcu.clone(), cmd_queue, descs)?),
            None => None,
        };
        let this = Arc::new(McuTmcUartBitbang {
            mcu: mcu.clone(),
            mutex,
            pullup,
            rx_pin,
            tx_pin,
            oid,
            cmd_queue,
            analog_mux,
            instances: klippy_api::GCell::new(Vec::new()),
            tmcuart_send_cmd: klippy_api::GCell::new(None),
        });
        let this2 = this.clone();
        mcu.register_config_callback(Box::new(move || this2.build_config()))?;
        Ok(this)
    }

    /// `MCU_TMC_uart_bitbang.build_config()`.
    pub fn build_config(&self) -> KResult<()> {
        let mut baud = TMC_BAUD_RATE;
        let mcu_type = self
            .mcu
            .get_constants()
            .get("MCU")
            .cloned()
            .unwrap_or_default();
        if mcu_type.starts_with("atmega") || mcu_type.starts_with("at90usb") {
            baud = TMC_BAUD_RATE_AVR;
        }
        let bit_ticks = self.mcu.seconds_to_clock(1.0 / (baud as f64));
        self.mcu.add_config_cmd(
            &format!(
                "config_tmcuart oid={} rx_pin={} pull_up={} tx_pin={} bit_time={}",
                self.oid, self.rx_pin, self.pullup, self.tx_pin, bit_ticks
            ),
            AddConfigCmdOpts::default(),
        )?;
        let cmd = self.mcu.lookup_query_command(
            "tmcuart_send oid=%c write=%*s read=%c",
            "tmcuart_response oid=%c read=%*s",
            LookupQueryCommandOpts { oid: Some(self.oid), cq: Some(self.cmd_queue), is_async: true },
        )?;
        *self.tmcuart_send_cmd.borrow_mut() = Some(cmd);
        Ok(())
    }

    /// `MCU_TMC_uart_bitbang.register_instance(...)`.
    pub fn register_instance(
        &self,
        rx_pin_params: &PinParams,
        tx_pin_params: &PinParams,
        select_pins_desc: Option<&[String]>,
        addr: i64,
    ) -> KResult<Option<Vec<bool>>> {
        if rx_pin_params.pin != self.rx_pin
            || tx_pin_params.pin != self.tx_pin
            || (select_pins_desc.is_none() != self.analog_mux.is_none())
        {
            return Err(KlippyError::config("Shared TMC uarts must use the same pins"));
        }
        let instance_id: Option<Vec<bool>> = match &self.analog_mux {
            Some(mux) => {
                let descs = select_pins_desc.expect("checked above");
                Some(mux.get_instance_id(descs)?)
            }
            None => None,
        };
        {
            let existing = self.instances.borrow();
            for (id, a) in existing.iter() {
                let same_id = id == &instance_id;
                if same_id && *a == addr {
                    return Err(KlippyError::config(
                        "Shared TMC uarts need unique address or select_pins polarity",
                    ));
                }
            }
        }
        self.instances.borrow_mut().push((instance_id.clone(), addr));
        Ok(instance_id)
    }

    fn send_cmd(&self) -> KResult<CommandQueryWrapper> {
        match self.tmcuart_send_cmd.borrow().clone() {
            Some(c) => Ok(c),
            None => Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'send'".to_string(),
            }),
        }
    }

    /// `reg_read`.
    pub fn reg_read(
        &self,
        instance_id: Option<&[bool]>,
        addr: i64,
        reg: i64,
    ) -> KResult<RegRead> {
        if let Some(mux) = &self.analog_mux {
            match instance_id {
                Some(ids) => mux.activate(ids)?,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "TypeError".to_string(),
                        message: "'NoneType' object is not iterable".to_string(),
                    })
                }
            }
        }
        let addr_b = to_byte(addr)?;
        let reg_b = to_byte(reg)?;
        let msg = encode_read(0xf5, addr_b, reg_b);
        let cmd = self.send_cmd()?;
        let params = cmd.send(
            vec![
                PyValue::Int(PyInt::from_i64(self.oid)),
                PyValue::Bytes(msg),
                PyValue::Int(PyInt::from_i64(10)),
            ],
            QuerySendOpts::default(),
        )?;
        let (read_bytes, receive_time) = match params {
            PyValue::Dict(map) => {
                let read = match map.get(&PyValue::Str("read".to_string())) {
                    Some(PyValue::Bytes(b)) => b.clone(),
                    Some(PyValue::None) => {
                        return Err(KlippyError::Internal {
                            class_name: "TypeError".to_string(),
                            message: "object of type 'NoneType' has no len()".to_string(),
                        })
                    }
                    Some(_) => {
                        return Err(KlippyError::Internal {
                            class_name: "TypeError".to_string(),
                            message: "object of type 'str' has no len()".to_string(),
                        })
                    }
                    None => return Err(key_error("read")),
                };
                let rt = match map.get(&PyValue::Str("#receive_time".to_string())) {
                    Some(PyValue::Float(f)) => *f,
                    Some(PyValue::Int(n)) => n.inner().to_string().parse::<f64>().unwrap_or(0.0),
                    Some(_) | None => return Err(key_error("#receive_time")),
                };
                (read, rt)
            }
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "argument must be a dict".to_string(),
                })
            }
        };
        let data: Option<i64> = decode_read(reg_b, &read_bytes);
        // `data` is also subject to length check already inside decode_read.
        Ok(RegRead { data, receive_time })
    }

    /// `reg_write`.
    pub fn reg_write(
        &self,
        instance_id: Option<&[bool]>,
        addr: i64,
        reg: i64,
        val: i64,
        print_time: Option<f64>,
    ) -> KResult<()> {
        let mut minclock: i64 = 0;
        if let Some(pt) = print_time {
            minclock = self.mcu.print_time_to_clock(pt);
        }
        if let Some(mux) = &self.analog_mux {
            match instance_id {
                Some(ids) => mux.activate(ids)?,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "TypeError".to_string(),
                        message: "'NoneType' object is not iterable".to_string(),
                    })
                }
            }
        }
        let addr_b = to_byte(addr)?;
        let reg_b = to_byte(reg | 0x80)?;
        let msg = encode_write(0xf5, addr_b, reg_b, val);
        let cmd = self.send_cmd()?;
        let _result = cmd.send(
            vec![
                PyValue::Int(PyInt::from_i64(self.oid)),
                PyValue::Bytes(msg),
                PyValue::Int(PyInt::from_i64(0)),
            ],
            QuerySendOpts { minclock, reqclock: 0, retry: true },
        )?;
        Ok(())
    }

    pub fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    pub fn get_mutex(&self) -> Arc<ReactorMutex> {
        self.mutex.clone()
    }
    pub fn get_oid(&self) -> i64 {
        self.oid
    }
    pub fn get_cmd_queue(&self) -> CommandQueue {
        self.cmd_queue
    }
    pub fn get_rx_pin(&self) -> &str {
        &self.rx_pin
    }
    pub fn get_tx_pin(&self) -> &str {
        &self.tx_pin
    }
    pub fn get_pullup(&self) -> i64 {
        self.pullup
    }
}

impl TmcUartBitbangApi for McuTmcUartBitbang {
    fn reg_read(
        &self,
        instance_id: Option<&[bool]>,
        addr: i64,
        reg: i64,
    ) -> KResult<RegRead> {
        McuTmcUartBitbang::reg_read(self, instance_id, addr, reg)
    }
    fn reg_write(
        &self,
        instance_id: Option<&[bool]>,
        addr: i64,
        reg: i64,
        val: i64,
        print_time: Option<f64>,
    ) -> KResult<()> {
        McuTmcUartBitbang::reg_write(self, instance_id, addr, reg, val, print_time)
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_mutex(&self) -> Arc<ReactorMutex> {
        self.mutex.clone()
    }
}

// ---------------------------------------------------------------------------
// Lookup (shared instance)
// ---------------------------------------------------------------------------

/// `lookup_tmc_uart_bitbang(config, max_addr)` (`klippy/extras/tmc_uart.py:194`).
pub fn lookup_tmc_uart_bitbang(
    config: &Config,
    max_addr: i64,
) -> KResult<(Option<Vec<bool>>, i64, Arc<McuTmcUartBitbang>)> {
    let printer = config.get_printer();
    let pins_api = lookup_pins(&printer)?;
    let uart_pin = config.get("uart_pin", GetStrOpts::default())?.expect("required");
    let rx_pin_params = pins_api.lookup_pin(
        &uart_pin,
        LookupPinOpts { can_invert: false, can_pullup: true, share_type: Some("tmc_uart_rx".to_string()) },
    )?;
    let tx_pin_desc = config.get(
        "tx_pin",
        GetStrOpts { default: Some(None), note_valid: true },
    )?;
    let tx_pin_params = match tx_pin_desc {
        None => rx_pin_params.clone(),
        Some(desc) => pins_api.lookup_pin(
            &desc,
            LookupPinOpts { can_invert: false, can_pullup: false, share_type: Some("tmc_uart_tx".to_string()) },
        )?,
    };
    if !Arc::ptr_eq(&rx_pin_params.chip, &tx_pin_params.chip) {
        return Err(KlippyError::Other {
            class_name: "error".to_string(),
            message: "TMC uart rx and tx pins must be on the same mcu".to_string(),
        });
    }
    let select_pins_desc_owned: Option<Vec<String>> = config.getlist(
        "select_pins",
        GetListOpts { default: Some(None), sep: ",".to_string(), count: None, note_valid: true },
    )?;
    let select_pins_ref: Option<&[String]> = select_pins_desc_owned.as_deref();
    let addr = config
        .getint(
            "uart_address",
            GetIntOpts {
                default: Some(Some(0)),
                minval: Some(0),
                maxval: Some(max_addr),
                note_valid: true,
            },
        )?
        .expect("default 0");
    let shared = get_shared_state(&printer)?;
    let key = format!("{}:{}", rx_pin_params.chip_name, rx_pin_params.pin);
    let mcu_uart = {
        let existing = shared.pin_to_uart.borrow();
        match existing.iter().find(|(k, _)| *k == key) {
            Some((_, u)) => u.clone(),
            None => {
                drop(existing);
                let created = McuTmcUartBitbang::new(&rx_pin_params, &tx_pin_params, select_pins_ref)?;
                shared.pin_to_uart.borrow_mut().push((key, created.clone()));
                created
            }
        }
    };
    let instance_id = mcu_uart.register_instance(
        &rx_pin_params,
        &tx_pin_params,
        select_pins_ref,
        addr,
    )?;
    Ok((instance_id, addr, mcu_uart))
}

// ---------------------------------------------------------------------------
// High-level UART helper
// ---------------------------------------------------------------------------

/// Trait for the high-level UART helper (`MCU_TMC_uart`).
pub trait TmcUartApi: Send + Sync + 'static {
    /// `get_register_raw(reg_name)` — `{'data': ..., '#receive_time': ...}`.
    fn get_register_raw(&self, reg_name: &str) -> KResult<PyValue>;
    /// `get_register(reg_name)`.
    fn get_register(&self, reg_name: &str) -> KResult<i64>;
    /// `set_register(reg_name, val, print_time)`.
    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()>;
    /// `get_tmc_frequency()`.
    fn get_tmc_frequency(&self) -> f64;
    /// `get_mcu()`.
    fn get_mcu(&self) -> Mcu;
}

/// `MCU_TMC_uart` (`klippy/extras/tmc_uart.py:217`).
pub struct McuTmcUart<F> {
    printer: Printer,
    name: String,
    name_to_reg: IndexMap<String, i64>,
    fields: F,
    ifcnt: klippy_api::GCell<Option<i64>>,
    instance_id: Option<Vec<bool>>,
    addr: i64,
    mcu_uart: Arc<McuTmcUartBitbang>,
    mutex: Arc<ReactorMutex>,
    tmc_frequency: f64,
}

impl<F> McuTmcUart<F> {
    /// `MCU_TMC_uart.__init__(config, name_to_reg, fields, max_addr, tmc_frequency)`.
    pub fn new(
        config: &Config,
        name_to_reg: IndexMap<String, i64>,
        fields: F,
        max_addr: i64,
        tmc_frequency: f64,
    ) -> KResult<Self> {
        let printer = config.get_printer();
        let section_name = config.get_name();
        let parts: Vec<&str> = section_name.split_whitespace().collect();
        let name = match parts.last() {
            Some(s) => (*s).to_string(),
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                })
            }
        };
        let (instance_id, addr, mcu_uart) = lookup_tmc_uart_bitbang(config, max_addr)?;
        let mutex = mcu_uart.get_mutex();
        Ok(McuTmcUart {
            printer,
            name,
            name_to_reg,
            fields,
            ifcnt: klippy_api::GCell::new(None),
            instance_id,
            addr,
            mcu_uart,
            mutex,
            tmc_frequency,
        })
    }

    pub fn get_fields(&self) -> &F {
        &self.fields
    }
    pub fn get_name(&self) -> &str {
        &self.name
    }
    pub fn get_name_to_reg(&self) -> &IndexMap<String, i64> {
        &self.name_to_reg
    }
    pub fn get_addr(&self) -> i64 {
        self.addr
    }
    pub fn get_instance_id(&self) -> Option<&[bool]> {
        self.instance_id.as_deref()
    }
    pub fn get_mcu_uart(&self) -> Arc<McuTmcUartBitbang> {
        self.mcu_uart.clone()
    }

    fn lookup_reg(&self, reg_name: &str) -> KResult<i64> {
        match self.name_to_reg.get(reg_name) {
            Some(v) => Ok(*v),
            None => Err(key_error(reg_name)),
        }
    }

    fn do_get_register(&self, reg_name: &str) -> KResult<RegRead> {
        let reg = self.lookup_reg(reg_name)?;
        if is_debugoutput(&self.printer) {
            return Ok(RegRead { data: Some(0), receive_time: 0.0 });
        }
        for _ in 0..5 {
            let val = self.mcu_uart.reg_read(self.instance_id.as_deref(), self.addr, reg)?;
            if val.data.is_some() {
                return Ok(val);
            }
        }
        Err(KlippyError::command(format!(
            "Unable to read tmc uart '{}' register {}",
            self.name, reg_name
        )))
    }

    fn do_set_register_body(
        &self,
        reg: i64,
        val: i64,
        reg_name: &str,
        print_time: Option<f64>,
    ) -> KResult<()> {
        for _ in 0..5 {
            let prev_opt = self.ifcnt.cloned();
            let prev = match prev_opt {
                Some(v) => v,
                None => {
                    let v = self.do_get_register("IFCNT")?.data.expect("do_get returns data");
                    *self.ifcnt.borrow_mut() = Some(v);
                    v
                }
            };
            self.mcu_uart
                .reg_write(self.instance_id.as_deref(), self.addr, reg, val, print_time)?;
            let new = self.do_get_register("IFCNT")?.data.expect("do_get returns data");
            *self.ifcnt.borrow_mut() = Some(new);
            if new == ((prev + 1) & 0xff) {
                return Ok(());
            }
        }
        Err(KlippyError::command(format!(
            "Unable to write tmc uart '{}' register {}",
            self.name, reg_name
        )))
    }

    /// `get_register_raw(reg_name)`.
    pub fn get_register_raw(&self, reg_name: &str) -> KResult<PyValue> {
        self.mutex.lock();
        let result = self.do_get_register(reg_name);
        self.mutex.unlock();
        result.map(|r| r.to_pyvalue())
    }

    /// `get_register(reg_name)`.
    pub fn get_register(&self, reg_name: &str) -> KResult<i64> {
        self.mutex.lock();
        let result = self.do_get_register(reg_name);
        self.mutex.unlock();
        let v = result?;
        Ok(v.data.expect("do_get returns data"))
    }

    /// `set_register(reg_name, val, print_time)`.
    pub fn set_register(
        &self,
        reg_name: &str,
        val: i64,
        print_time: Option<f64>,
    ) -> KResult<()> {
        let reg = self.lookup_reg(reg_name)?;
        if is_debugoutput(&self.printer) {
            return Ok(());
        }
        self.mutex.lock();
        let result = self.do_set_register_body(reg, val, reg_name, print_time);
        self.mutex.unlock();
        result
    }

    pub fn get_tmc_frequency(&self) -> f64 {
        self.tmc_frequency
    }
    pub fn get_mcu(&self) -> Mcu {
        self.mcu_uart.get_mcu()
    }
}

impl<F: Send + Sync + 'static> TmcUartApi for McuTmcUart<F> {
    fn get_register_raw(&self, reg_name: &str) -> KResult<PyValue> {
        McuTmcUart::get_register_raw(self, reg_name)
    }
    fn get_register(&self, reg_name: &str) -> KResult<i64> {
        McuTmcUart::get_register(self, reg_name)
    }
    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        McuTmcUart::set_register(self, reg_name, val, print_time)
    }
    fn get_tmc_frequency(&self) -> f64 {
        self.tmc_frequency
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu_uart.get_mcu()
    }
}

/// Module entry point: `tmc_uart` has no load hooks (`load_config` etc.).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
