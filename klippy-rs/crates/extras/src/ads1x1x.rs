//! Port of the `ads1x1x` module (`klippy/extras/ads1x1x.py`).
//!
//! Support for I2C based ADS1013, ADS1014, ADS1015, ADS1113, ADS1114 and
//! ADS1115 chips. The chip registers itself as a pin chip providing
//! virtual ADC pins, and each pin polls its conversion in single-shot
//! mode on its own reactor timer.
//!
//! The I2C handle is the `bus.MCU_I2C` object (`klippy/extras/bus.py`),
//! owned by the `bus` port behind `crate::bus::McuI2cApi`.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetChoiceOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{AdcCallback, AdcPinApi, AdcSampleOpts, ChipApi, Mcu, Pin, PinParams, PinsApi, QuerySendOpts};
use klippy_api::printer::{EV_KLIPPY_CONNECT, LookupDefault, ModuleFactory, Printer};
use klippy_api::{GCell, ObjectRef, lookup_interface};
use pycompat::{PctValue, PyInt, PyValue};
use reactor::{Reactor, ReactorMutex, Timer};
use std::sync::Arc;

use crate::bus::{McuI2cApi, mcu_i2c_from_config};

pub const CHIP_ADS1013: i64 = 3;
pub const CHIP_ADS1014: i64 = 4;
pub const CHIP_ADS1015: i64 = 5;
pub const CHIP_ADS1113: i64 = 13;
pub const CHIP_ADS1114: i64 = 14;
pub const CHIP_ADS1115: i64 = 15;

pub const CHIP_TYPE_CHOICES: [(&str, i64); 6] = [
    ("ADS1013", CHIP_ADS1013),
    ("ADS1014", CHIP_ADS1014),
    ("ADS1015", CHIP_ADS1015),
    ("ADS1113", CHIP_ADS1113),
    ("ADS1114", CHIP_ADS1114),
    ("ADS1115", CHIP_ADS1115),
];

pub const ADDR_GND: i64 = 0x48;
pub const ADDR_VCC: i64 = 0x49;
pub const ADDR_SDA: i64 = 0x4a;
pub const ADDR_SCL: i64 = 0x4b;

pub const CHIP_ADDR_CHOICES: [(&str, i64); 4] = [
    ("GND", ADDR_GND),
    ("VCC", ADDR_VCC),
    ("SDA", ADDR_SDA),
    ("SCL", ADDR_SCL),
];

pub const REG_POINTER_MASK: i64 = 0x03;
pub const REG_CONVERSION: i64 = 0x00;
pub const REG_CONFIG: i64 = 0x01;
pub const REG_LO_THRESH: i64 = 0x02;
pub const REG_HI_THRESH: i64 = 0x03;

pub const REG_CONFIG_OS_MASK: i64 = 0x8000;
pub const REG_CONFIG_MULTIPLEXER_MASK: i64 = 0x7000;
pub const REG_CONFIG_PGA_MASK: i64 = 0x0E00;
pub const REG_CONFIG_MODE_MASK: i64 = 0x0100;
pub const REG_CONFIG_DATA_RATE_MASK: i64 = 0x00E0;
pub const REG_CONFIG_COMPARATOR_MODE_MASK: i64 = 0x0010;
pub const REG_CONFIG_COMPARATOR_POLARITY_MASK: i64 = 0x0008;
pub const REG_CONFIG_COMPARATOR_LATCHING_MASK: i64 = 0x0004;
pub const REG_CONFIG_COMPARATOR_QUEUE_MASK: i64 = 0x0003;

pub const OS_IDLE: i64 = 0x8000;
pub const OS_SINGLE: i64 = 0x8000;

pub const MUX_DIFF01: i64 = 0x0000;
pub const MUX_DIFF03: i64 = 0x1000;
pub const MUX_DIFF13: i64 = 0x2000;
pub const MUX_DIFF23: i64 = 0x3000;
pub const MUX_AIN0: i64 = 0x4000;
pub const MUX_AIN1: i64 = 0x5000;
pub const MUX_AIN2: i64 = 0x6000;
pub const MUX_AIN3: i64 = 0x7000;

pub const PGA_CHOICES: [(&str, i64); 6] = [
    ("6.144V", 0x0000),
    ("4.096V", 0x0200),
    ("2.048V", 0x0400),
    ("1.024V", 0x0600),
    ("0.512V", 0x0800),
    ("0.256V", 0x0A00),
];

pub const ADS111X_RESOLUTION: f64 = 32767.0;
pub const ADS101X_RESOLUTION: f64 = 2047.0;

pub const MODE_CONTINUOUS: i64 = 0x0000;
pub const MODE_SINGLE: i64 = 0x0100;

pub const ADS101X_SAMPLES_PER_SECOND: [(&str, i64); 7] = [
    ("128", 0x0000),
    ("250", 0x0020),
    ("490", 0x0040),
    ("920", 0x0060),
    ("1600", 0x0080),
    ("2400", 0x00a0),
    ("3300", 0x00c0),
];

pub const ADS111X_SAMPLES_PER_SECOND: [(&str, i64); 8] = [
    ("8", 0x0000),
    ("16", 0x0020),
    ("32", 0x0040),
    ("64", 0x0060),
    ("128", 0x0080),
    ("250", 0x00a0),
    ("475", 0x00c0),
    ("860", 0x00e0),
];

pub const COMPARATOR_MODE_TRADITIONAL: i64 = 0x0000;
pub const COMPARATOR_MODE_WINDOW: i64 = 0x0010;
pub const COMPARATOR_POLARITY_ACTIVE_LO: i64 = 0x0000;
pub const COMPARATOR_POLARITY_ACTIVE_HI: i64 = 0x0008;
pub const COMPARATOR_LATCHING_NON_LATCHING: i64 = 0x0000;
pub const COMPARATOR_LATCHING_LATCHING: i64 = 0x0004;
pub const COMPARATOR_QUEUE_1: i64 = 0x0000;
pub const COMPARATOR_QUEUE_2: i64 = 0x0001;
pub const COMPARATOR_QUEUE_4: i64 = 0x0002;
pub const COMPARATOR_QUEUE_NONE: i64 = 0x0003;

pub const OPERATION_SET_MUX: i64 = 0;
pub const OPERATION_READ_CONVERSION: i64 = 1;

pub fn is_ads101x(chip: i64) -> bool {
    chip == CHIP_ADS1013 || chip == CHIP_ADS1014 || chip == CHIP_ADS1015
}

pub fn is_ads111x(chip: i64) -> bool {
    chip == CHIP_ADS1113 || chip == CHIP_ADS1114 || chip == CHIP_ADS1115
}

/// `ADS1X1X_PGA_VALUE[pga]` (`klippy/extras/ads1x1x.py:88-95`).
pub fn pga_value(pga: i64) -> Option<f64> {
    match pga {
        0x0000 => Some(6.144),
        0x0200 => Some(4.096),
        0x0400 => Some(2.048),
        0x0600 => Some(1.024),
        0x0800 => Some(0.512),
        0x0A00 => Some(0.256),
        _ => None,
    }
}

/// `ADS111X_PGA_SCALAR[pga]` (`klippy/extras/ads1x1x.py:97-104`).
pub fn ads111x_pga_scalar(pga: i64) -> Option<f64> {
    match pga {
        0x0000 => Some(6.144 / ADS111X_RESOLUTION),
        0x0200 => Some(4.096 / ADS111X_RESOLUTION),
        0x0400 => Some(2.048 / ADS111X_RESOLUTION),
        0x0600 => Some(1.024 / ADS111X_RESOLUTION),
        0x0800 => Some(0.512 / ADS111X_RESOLUTION),
        0x0A00 => Some(0.256 / ADS111X_RESOLUTION),
        _ => None,
    }
}

/// `ADS101X_PGA_SCALAR[pga]` (`klippy/extras/ads1x1x.py:106-113`).
pub fn ads101x_pga_scalar(pga: i64) -> Option<f64> {
    match pga {
        0x0000 => Some(6.144 / ADS101X_RESOLUTION),
        0x0200 => Some(4.096 / ADS101X_RESOLUTION),
        0x0400 => Some(2.048 / ADS101X_RESOLUTION),
        0x0600 => Some(1.024 / ADS101X_RESOLUTION),
        0x0800 => Some(0.512 / ADS101X_RESOLUTION),
        0x0A00 => Some(0.256 / ADS101X_RESOLUTION),
        _ => None,
    }
}

fn mux_bits(pin: &str) -> Option<i64> {
    match pin {
        "DIFF01" => Some(MUX_DIFF01),
        "DIFF03" => Some(MUX_DIFF03),
        "DIFF13" => Some(MUX_DIFF13),
        "DIFF23" => Some(MUX_DIFF23),
        "AIN0" => Some(MUX_AIN0),
        "AIN1" => Some(MUX_AIN1),
        "AIN2" => Some(MUX_AIN2),
        "AIN3" => Some(MUX_AIN3),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// I2C handle (`klippy/extras/bus.py:161-329`, the `MCU_I2C` class, owned by
// `crate::bus` as `McuI2cApi`; this module uses `Arc<dyn McuI2cApi>`).
// ---------------------------------------------------------------------------

/// The I2C handle type held by the chip core, mirroring
/// `bus.MCU_I2C_from_config`'s return value.
pub type McuI2c = Arc<dyn McuI2cApi>;

// ---------------------------------------------------------------------------
// Config helpers
// ---------------------------------------------------------------------------

fn choices_dict(pairs: &[(&str, i64)]) -> PyValue {
    let mut map = IndexMap::new();
    for (k, v) in pairs {
        map.insert(PyValue::Str((*k).to_string()), PyValue::Int(PyInt::from_i64(*v)));
    }
    PyValue::Dict(map)
}

fn py_int_to_i64(v: &PyValue) -> KResult<i64> {
    match v {
        PyValue::Int(n) => n
            .inner()
            .to_string()
            .parse::<i64>()
            .map_err(|_| KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "int value out of range".to_string(),
            }),
        _ => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "expected int from getchoice".to_string(),
        }),
    }
}

fn getchoice_int(
    config: &Config,
    option: &str,
    choices: &[(&str, i64)],
    default: Option<String>,
) -> KResult<i64> {
    let default = default.map(|s| Some(PyValue::Str(s)));
    let v = config.getchoice(
        option,
        &choices_dict(choices),
        GetChoiceOpts { default, note_valid: true },
    )?;
    py_int_to_i64(&v)
}

/// Address resolution (`klippy/extras/ads1x1x.py:178-181`): the `GND`
/// address unless `address_pin` is configured, in which case it is the
/// choice for the pin wiring. The resolved address is the default
/// address of the I2C bus (`bus.MCU_I2C_from_config(config, address)`),
/// which a user can still override with `i2c_address`.
pub fn resolve_chip_address(config: &Config) -> KResult<i64> {
    let has_address_pin = config
        .get(
            "address_pin",
            GetStrOpts { default: Some(None), note_valid: true },
        )?
        .is_some();
    if has_address_pin {
        getchoice_int(config, "address_pin", &CHIP_ADDR_CHOICES, None)
    } else {
        Ok(ADDR_GND)
    }
}

fn pins_error(message: String) -> KlippyError {
    // `pins.error` (`klippy/pins.py:8`, class name "error").
    KlippyError::Pins(message)
}

// ---------------------------------------------------------------------------
// Chip core: state shared by the chip object and its pins
// ---------------------------------------------------------------------------

pub struct ChipCore {
    pub name: String,
    pub printer: Printer,
    pub reactor: Reactor,
    pub i2c: McuI2c,
    pub mcu: Mcu,
    pub chip: i64,
    pub pga: GCell<Option<i64>>,
    pub adc_voltage: GCell<Option<f64>>,
    pub comp_mode: i64,
    pub comp_polarity: i64,
    pub comp_latching: i64,
    pub comp_queue: i64,
    pub pins: GCell<IndexMap<String, Arc<ADS1X1X_pin>>>,
    pub delay: GCell<Option<f64>>,
    pub mutex: ReactorMutex,
}

impl ChipCore {
    pub fn pga_value(&self) -> i64 {
        self.pga.get().expect("pga is set in ADS1X1X_chip::from_config")
    }

    pub fn adc_voltage_value(&self) -> f64 {
        self.adc_voltage.get().expect("adc_voltage is set in ADS1X1X_chip::from_config")
    }

    /// `ADS1X1X_chip._handle_connect` (`klippy/extras/ads1x1x.py:245-250`).
    pub fn handle_connect(&self) {
        if let Err(e) = self.i2c.i2c_write(&[0x06, 0x00, 0x00], QuerySendOpts::default()) {
            klippy_api::log::exception(
                "ADS1X1X: error while resetting device",
                &e.exception_line(),
            );
        }
    }

    /// `ADS1X1X_chip.is_ready` (`klippy/extras/ads1x1x.py:252-255`).
    pub fn is_ready(&self) -> KResult<bool> {
        let cfg = self.read_register(REG_CONFIG)?;
        Ok((cfg & REG_CONFIG_OS_MASK) == OS_IDLE)
    }

    /// `ADS1X1X_chip.calculate_sample_rate` (`klippy/extras/ads1x1x.py:257-278`).
    pub fn calculate_sample_rate(&self) -> (i64, i64) {
        let pins = self.pins.borrow();
        let pin_count = pins.len();
        let mut lowest_report_time = 1.0f64;
        for pin in pins.values() {
            lowest_report_time = lowest_report_time.min(pin.report_time.get());
        }
        drop(pins);
        let sample_rate = 1.0 / lowest_report_time * pin_count as f64;
        let table: &[(&str, i64)] = if is_ads101x(self.chip) {
            &ADS101X_SAMPLES_PER_SECOND
        } else {
            &ADS111X_SAMPLES_PER_SECOND
        };
        let mut sorted: Vec<(i64, i64)> =
            table.iter().map(|(r, b)| (parse_rate(r), *b)).collect();
        sorted.sort_by_key(|(r, _)| *r);
        let mut last = (0i64, 0i64);
        for (rate_number, bits) in sorted {
            last = (rate_number, bits);
            if sample_rate <= rate_number as f64 {
                return (rate_number, bits);
            }
        }
        let msg = pycompat::text::percent_format(
            "ADS1X1X: requested sample rate %s is higher than supported by %s.",
            &[PctValue::Float(sample_rate), PctValue::Str(self.name.clone())],
        )
        .expect("percent format is valid");
        klippy_api::log::warning(&msg);
        last
    }

    /// `ADS1X1X_chip.handle_report_time_update` (`klippy/extras/ads1x1x.py:280-287`).
    pub fn handle_report_time_update(&self) {
        let (sample_rate, sample_rate_bits) = self.calculate_sample_rate();
        for pin in self.pins.borrow().values() {
            let pcfg = pin.pcfg.get();
            pin.pcfg.set(
                (pcfg & !REG_CONFIG_DATA_RATE_MASK)
                    | (sample_rate_bits & REG_CONFIG_DATA_RATE_MASK),
            );
        }
        self.delay.set(Some(1.0 / sample_rate as f64));
    }

    /// `ADS1X1X_chip.sample` (`klippy/extras/ads1x1x.py:289-304`).
    pub fn sample(&self, pin: &ADS1X1X_pin) -> Option<i64> {
        let pcfg = pin.pcfg.get();
        self.mutex.lock();
        let outcome = self.sample_locked(pcfg);
        self.mutex.unlock();
        match outcome {
            Ok(v) => v,
            Err(e) => {
                let msg = pycompat::text::percent_format(
                    "ADS1X1X: error while sampling: %s",
                    &[PctValue::Str(e.message().to_string())],
                )
                .expect("percent format is valid");
                klippy_api::log::exception(
                    &msg,
                    &e.exception_line(),
                );
                None
            }
        }
    }

    fn sample_locked(&self, pcfg: i64) -> KResult<Option<i64>> {
        // Python writes the config register first, then reads `delay`,
        // so emit the write first for the same failure order.
        self.write_register(REG_CONFIG, pcfg)?;
        let delay = match self.delay.get() {
            Some(d) => d,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'ADS1X1X_chip' object has no attribute 'delay'".to_string(),
                });
            }
        };
        self.reactor.pause(self.reactor.monotonic() + delay);
        let start_time = self.reactor.monotonic();
        while !self.is_ready()? {
            self.reactor.pause(self.reactor.monotonic() + 0.001);
            if start_time + delay < self.reactor.monotonic() {
                klippy_api::log::warning("ADS1X1X: timeout during sampling");
                return Ok(None);
            }
        }
        self.read_register(REG_CONVERSION).map(Some)
    }

    fn response_bytes(params: &PyValue) -> KResult<Vec<u8>> {
        let PyValue::Dict(d) = params else {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "i2c_read params must be a dict".to_string(),
            });
        };
        let PyValue::List(items) =
            d.get(&PyValue::Str("response".to_string())).cloned().unwrap_or(PyValue::None)
        else {
            return Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: "'response'".to_string(),
            });
        };
        let mut out = Vec::with_capacity(items.len());
        for v in &items {
            let PyValue::Int(n) = v else {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "i2c response must be ints".to_string(),
                });
            };
            let n: i64 = n
                .inner()
                .to_string()
                .parse()
                .map_err(|_| KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: "int value out of range".to_string(),
                })?;
            out.push(u8::try_from(n).map_err(|_| KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "byte must be in range(0, 256)".to_string(),
            })?);
        }
        Ok(out)
    }

    /// `ADS1X1X_chip._read_register` (`klippy/extras/ads1x1x.py:306-310`).
    pub fn read_register(&self, reg: i64) -> KResult<i64> {
        let params = self.i2c.i2c_read(&[reg as u8], 2, QuerySendOpts::default())?;
        let buff = Self::response_bytes(&params)?;
        let b0 = *buff.first().ok_or_else(|| KlippyError::Internal {
            class_name: "IndexError".to_string(),
            message: "list index out of range".to_string(),
        })?;
        let b1 = *buff.get(1).ok_or_else(|| KlippyError::Internal {
            class_name: "IndexError".to_string(),
            message: "list index out of range".to_string(),
        })?;
        Ok(((b0 as i64) << 8) | (b1 as i64))
    }

    /// `ADS1X1X_chip._write_register` (`klippy/extras/ads1x1x.py:312-318`).
    pub fn write_register(&self, reg: i64, data: i64) -> KResult<()> {
        let bytes = [
            (reg & 0xFF) as u8,
            ((data >> 8) & 0xFF) as u8,
            (data & 0xFF) as u8,
        ];
        self.i2c.i2c_write(&bytes, QuerySendOpts::default())
    }

    /// `ADS1X1X_chip.setup_pin` (`klippy/extras/ads1x1x.py:206-243`),
    /// shared behind [`ChipApi`] and direct test calls.
    pub fn setup_pin_for_chip(
        core: &Arc<ChipCore>,
        pin_type: &str,
        pin_params: &PinParams,
    ) -> KResult<Pin> {
        let pin = pin_params.pin.as_str();
        if pin_type == "adc" {
            let Some(mux) = mux_bits(pin) else {
                return Err(pins_error(format!("ADS1x1x pin {pin} is not valid")));
            };
            let pga = core.pga_value();
            let mut pcfg: i64 = 0;
            pcfg |= OS_SINGLE & REG_CONFIG_OS_MASK;
            pcfg |= mux & REG_CONFIG_MULTIPLEXER_MASK;
            pcfg |= pga & REG_CONFIG_PGA_MASK;
            pcfg |= MODE_SINGLE & REG_CONFIG_MODE_MASK;
            pcfg |= core.comp_mode & REG_CONFIG_COMPARATOR_MODE_MASK;
            pcfg |= core.comp_polarity & REG_CONFIG_COMPARATOR_POLARITY_MASK;
            pcfg |= core.comp_latching & REG_CONFIG_COMPARATOR_LATCHING_MASK;
            pcfg |= core.comp_queue & REG_CONFIG_COMPARATOR_QUEUE_MASK;

            let pin_obj = ADS1X1X_pin::new(core.clone(), pcfg);
            if core.pins.borrow().contains_key(pin) {
                return Err(pins_error(format!(
                    "pin {pin} for chip {} is used multiple times",
                    core.name
                )));
            }
            core.pins.borrow_mut().insert(pin.to_string(), pin_obj.clone());
            return Ok(Pin::Adc(pin_obj));
        }
        Err(pins_error(format!(
            "Wrong pin or incompatible type: {pin} with type {pin_type}! "
        )))
    }
}

fn parse_rate(rate: &str) -> i64 {
    rate.parse::<i64>().expect("sample rate keys are integers")
}

// ---------------------------------------------------------------------------
// `ADS1X1X_chip` (`klippy/extras/ads1x1x.py:170-318`)
// ---------------------------------------------------------------------------

pub struct ADS1X1X_chip {
    pub core: Arc<ChipCore>,
}

impl ADS1X1X_chip {
    /// `ADS1X1X_chip.__init__(config)` with the I2C handle that
    /// `bus.MCU_I2C_from_config(config, address)` would return.
    pub fn from_config(config: &Config, i2c: McuI2c) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();

        let name = config.get_name().split_whitespace().next_back().unwrap_or("").to_string();
        let chip = getchoice_int(config, "chip", &CHIP_TYPE_CHOICES, None)?;
        let _address = resolve_chip_address(config)?;

        let mcu = i2c.get_mcu();
        let core = Arc::new(ChipCore {
            comp_mode: COMPARATOR_MODE_TRADITIONAL,
            comp_polarity: COMPARATOR_POLARITY_ACTIVE_LO,
            comp_latching: COMPARATOR_LATCHING_NON_LATCHING,
            comp_queue: COMPARATOR_QUEUE_NONE,
            name: name.clone(),
            printer: printer.clone(),
            reactor: reactor.clone(),
            i2c,
            mcu,
            chip,
            pga: GCell::new(None),
            adc_voltage: GCell::new(None),
            pins: GCell::new(IndexMap::new()),
            delay: GCell::new(None),
            mutex: reactor.mutex(false),
        });
        let chip_arc = Arc::new(ADS1X1X_chip { core: core.clone() });

        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        ppins.register_chip(&name, chip_arc.clone())?;

        let pga = getchoice_int(config, "pga", &PGA_CHOICES, Some("4.096V".to_string()))?;
        core.pga.set(Some(pga));
        let adc_voltage = config
            .getfloat(
                "adc_voltage",
                GetFloatOpts {
                    default: Some(Some(3.3)),
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .expect("getfloat with a default never returns None");
        core.adc_voltage.set(Some(adc_voltage));

        let obj: ObjectRef = chip_arc.clone();
        printer.add_object(&format!("ads1x1x {name}"), obj)?;
        let chip_handler = chip_arc.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_: &[klippy_api::EventArg]| {
                chip_handler.core.handle_connect();
                Ok(PyValue::None)
            }),
        );
        Ok(chip_arc)
    }

    pub fn name(&self) -> &str {
        &self.core.name
    }

    pub fn chip(&self) -> i64 {
        self.core.chip
    }

    pub fn pga(&self) -> i64 {
        self.core.pga_value()
    }

    pub fn adc_voltage(&self) -> f64 {
        self.core.adc_voltage_value()
    }
}

impl ChipApi for ADS1X1X_chip {
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        ChipCore::setup_pin_for_chip(&self.core, pin_type, &pin_params)
    }

    fn get_mcu(&self) -> KResult<Mcu> {
        Ok(self.core.mcu.clone())
    }
}

klippy_api::printer_object!(ADS1X1X_chip; dyn ChipApi);

// ---------------------------------------------------------------------------
// `ADS1X1X_pin` (`klippy/extras/ads1x1x.py:320-396`)
// ---------------------------------------------------------------------------

pub struct ADS1X1X_pin {
    pub core: Arc<ChipCore>,
    pub pcfg: GCell<i64>,
    pub last_state: GCell<(f64, f64)>,
    pub invalid_count: GCell<i64>,
    pub callback: GCell<Option<AdcCallback>>,
    pub report_time: GCell<f64>,
    pub minval: GCell<f64>,
    pub maxval: GCell<f64>,
    pub range_check_count: GCell<i64>,
    pub reactor: GCell<Option<Reactor>>,
    pub sample_timer: GCell<Option<Timer>>,
}

impl ADS1X1X_pin {
    /// `ADS1X1X_pin.__init__(chip, pcfg)` (`klippy/extras/ads1x1x.py:321-329`):
    /// registers the `klippy:connect` handler that arms the sample timer.
    pub fn new(core: Arc<ChipCore>, pcfg: i64) -> Arc<Self> {
        let pin = Arc::new(ADS1X1X_pin {
            core: core.clone(),
            pcfg: GCell::new(pcfg),
            last_state: GCell::new((0., 0.)),
            invalid_count: GCell::new(0),
            callback: GCell::new(None),
            report_time: GCell::new(0.),
            minval: GCell::new(0.),
            maxval: GCell::new(1.),
            range_check_count: GCell::new(0),
            reactor: GCell::new(None),
            sample_timer: GCell::new(None),
        });
        let pin_handler = pin.clone();
        core.printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_: &[klippy_api::EventArg]| {
                pin_handler.handle_connect();
                let pin_timer = pin_handler.clone();
                let reactor = pin_handler.pin_reactor();
                let timer = reactor.register_timer(
                    move |eventtime| pin_timer.process_sample(eventtime),
                    reactor::NOW,
                );
                pin_handler.sample_timer.replace(Some(timer));
                Ok(PyValue::None)
            }),
        );
        pin
    }

    /// `ADS1X1X_pin._handle_connect` (`klippy/extras/ads1x1x.py:331-335`):
    /// fetches the reactor and (via the registered handler) arms the
    /// sample timer.
    pub fn handle_connect(&self) {
        self.reactor.replace(Some(self.core.printer.get_reactor()));
    }

    fn pin_reactor(&self) -> Reactor {
        self.reactor.borrow().clone().unwrap_or_else(|| self.core.reactor.clone())
    }

    /// `ADS1X1X_pin._process_sample` (`klippy/extras/ads1x1x.py:337-372`).
    pub fn process_sample(&self, eventtime: f64) -> f64 {
        let sample = self.core.sample(self);
        if let Some(sample) = sample {
            // The sample is encoded in the top 12 or full 16 bits.
            let target_value = if is_ads101x(self.core.chip) {
                ((sample >> 4) as f64) / ADS101X_RESOLUTION
            } else {
                (sample as f64) / ADS111X_RESOLUTION
            };
            // Remap the PGA range to 0..1 against the reference voltage.
            let target_value = target_value
                * (pga_value(self.core.pga_value())
                    .expect("pga is a valid choice key")
                    / self.core.adc_voltage_value());

            if target_value > self.maxval.get() || target_value < self.minval.get() {
                self.invalid_count.set(self.invalid_count.get() + 1);
                klippy_api::log::warning("ADS1X1X: temperature outside range");
                self.check_invalid();
            } else {
                self.invalid_count.set(0);
            }

            let systime = self.pin_reactor().monotonic();
            let measured_time = self.core.mcu.estimated_print_time(systime);
            self.last_state.set((measured_time, target_value));
            let mut cb = self.callback.replace(None);
            if let Some(f) = cb.as_mut() {
                f(vec![(measured_time, target_value)]);
            }
            self.callback.replace(cb);
        } else {
            self.invalid_count.set(self.invalid_count.get() + 1);
            self.check_invalid();
        }
        eventtime + self.report_time.get()
    }

    /// `ADS1X1X_pin.check_invalid` (`klippy/extras/ads1x1x.py:374-377`).
    pub fn check_invalid(&self) {
        if self.invalid_count.get() > self.range_check_count.get() {
            self.core.printer.invoke_shutdown(
                "ADS1X1X temperature check failed",
                PyValue::Dict(IndexMap::new()),
            );
        }
    }

    pub fn last_value(&self) -> (f64, f64) {
        self.last_state.get()
    }
}

impl AdcPinApi for ADS1X1X_pin {
    fn get_mcu(&self) -> Mcu {
        self.core.mcu.clone()
    }

    /// `ADS1X1X_pin.setup_adc_sample` (`klippy/extras/ads1x1x.py:385-392`).
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        self.report_time.set(report_time);
        self.minval.set(opts.minval);
        self.maxval.set(opts.maxval);
        self.range_check_count.set(opts.range_check_count);
        self.core.handle_report_time_update();
    }

    /// `ADS1X1X_pin.setup_adc_callback` (`klippy/extras/ads1x1x.py:382-383`).
    fn setup_adc_callback(&self, callback: AdcCallback) {
        self.callback.replace(Some(callback));
    }

    /// `ADS1X1X_pin.get_last_value` (`klippy/extras/ads1x1x.py:394-395`).
    fn get_last_value(&self) -> (f64, f64) {
        self.last_state.get()
    }
}

// ---------------------------------------------------------------------------
// Module entry point (`klippy/extras/ads1x1x.py:397-398`)
// ---------------------------------------------------------------------------

fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let address = resolve_chip_address(config)?;
    // `MCU_I2C_from_config(config, address)` (`klippy/extras/bus.py:302`,
    // defaults: speed 100000, not async-write-only).
    let i2c = mcu_i2c_from_config(config, Some(address), 100000, false)?;
    let chip = ADS1X1X_chip::from_config(config, i2c)?;
    let obj: ObjectRef = chip;
    Ok(Some(obj))
}

/// Module entry point for `[ads1x1x name]` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
