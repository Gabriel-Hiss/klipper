//! Port of `klippy/extras/temperature_mcu.py`.
//!
//! `PrinterTemperatureMCU` exposes the MCU's own temperature sensor as a
//! temperature sensor for `heaters`. It reads the factory calibration words
//! over `debug_read`, derives a linear `base_temperature`/`slope` pair from
//! them (or from the optional `sensor_temperature1/2` + `sensor_adc1/2`
//! override), and converts the `ADC_TEMPERATURE` pseudo pin of `sensor_mcu`
//! with it.

use crate::adc_temperature::{AdcConvert, HelperTemperatureDiagnostics};
use crate::heaters::{
    PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi,
};
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{
    AdcPin, AdcSampleOpts, CommandQueryWrapper, LookupQueryCommandOpts, Pin, PinsApi,
    QuerySendOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_MCU_IDENTIFY};
use klippy_api::{load_interface, lookup_interface, EventArg, ObjectRef};
use klippy_api::gcell::SCell;
use pycompat::text::{percent_format, PctValue};
use pycompat::value::{repr_key, type_name};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

/// `SAMPLE_TIME = 0.001` (`temperature_mcu.py:10`).
pub const SAMPLE_TIME: f64 = 0.001;
/// `SAMPLE_COUNT = 8` (`temperature_mcu.py:11`).
pub const SAMPLE_COUNT: i64 = 8;
/// `REPORT_TIME = 0.300` (`temperature_mcu.py:12`).
pub const REPORT_TIME: f64 = 0.300;
/// `RANGE_CHECK_COUNT = 4` (`temperature_mcu.py:13`).
pub const RANGE_CHECK_COUNT: i64 = 4;

/// `SAMPLE_COUNT * SAMPLE_TIME`, the offset `adc_callback` adds to the read
/// time (line 54).
const SAMPLE_DELAY: f64 = SAMPLE_COUNT as f64 * SAMPLE_TIME;

/// `config_samd21(addr=0x00806030)` (line 120).
const SAMD21_CAL_ADDR: i64 = 0x0080_6030;
/// `config_samd51()` -> `config_samd21(addr=0x00800100)` (line 135).
const SAMD51_CAL_ADDR: i64 = 0x0080_0100;
/// `config_stm32f4(addr1=0x1FFF7A2C, addr2=0x1FFF7A2E)` (line 143).
const STM32F4_CAL_ADDR1: i64 = 0x1FFF_7A2C;
/// `config_stm32f4(addr2=...)` (line 143).
const STM32F4_CAL_ADDR2: i64 = 0x1FFF_7A2E;
/// `config_stm32f0x2()` -> `config_stm32f4(addr1=0x1FFFF7B8, ...)` (line 148).
const STM32F0X2_CAL_ADDR1: i64 = 0x1FFF_F7B8;
/// `config_stm32f0x2()` -> `config_stm32f4(addr2=0x1FFFF7C2)` (line 148).
const STM32F0X2_CAL_ADDR2: i64 = 0x1FFF_F7C2;
/// `config_stm32f070()` (line 151).
const STM32F070_CAL_ADDR: i64 = 0x1FFF_F7B8;
/// `config_stm32g0()` first word (line 155).
const STM32G0_CAL_ADDR_30: i64 = 0x1FFF_75A8;
/// `config_stm32g0()` second word (line 156).
const STM32G0_CAL_ADDR_130: i64 = 0x1FFF_75CA;
/// `config_stm32h723()`/`config_stm32h7()` first word (lines 160, 165).
const STM32H7_CAL_ADDR_30: i64 = 0x1FF1_E820;
/// `config_stm32h723()`/`config_stm32h7()` second word (lines 161, 166).
const STM32H7_CAL_ADDR_HIGH: i64 = 0x1FF1_E840;

/// State shared by the ADC callback, the diagnostics helper and the MCU
/// configuration functions (`self.slope`, `self.base_temperature`,
/// `self.min_temp`, `self.max_temp`, `self.mcu_type`,
/// `self.temperature_callback`).
struct Shared {
    base_temperature: SCell<Option<f64>>,
    slope: SCell<Option<f64>>,
    temperature_callback: SCell<Option<TemperatureCallback>>,
    min_temp: SCell<f64>,
    max_temp: SCell<f64>,
    mcu_type: SCell<String>,
}

impl Shared {
    fn new() -> Shared {
        Shared {
            base_temperature: SCell::new(None),
            slope: SCell::new(None),
            temperature_callback: SCell::new(None),
            min_temp: SCell::new(0.),
            max_temp: SCell::new(0.),
            mcu_type: SCell::new(String::new()),
        }
    }

    fn slope(&self) -> Option<f64> {
        self.slope.get()
    }

    fn set_slope(&self, slope: f64) {
        self.slope.set(Some(slope));
    }

    /// `self.base_temperature = self.calc_base(temp, adc)`.
    fn set_base_from(&self, temp: f64, adc: f64) {
        self.base_temperature.set(Some(self.calc_base(temp, adc)));
    }

    /// `calc_temp(self, adc)` (lines 57-58). Python raises `TypeError` when
    /// this runs before `handle_mcu_identify` set the base and slope; no
    /// ADC sample reaches it before that event, so the port panics on that
    /// unreachable state instead.
    fn calc_temp(&self, adc: f64) -> f64 {
        let base = self.base_temperature.get().expect("base_temperature is set by handle_mcu_identify");
        let slope = self.slope.get().expect("slope is set by handle_mcu_identify");
        base + adc * slope
    }

    /// `calc_adc(self, temp)` (lines 59-60).
    fn calc_adc(&self, temp: f64) -> f64 {
        let base = self.base_temperature.get().expect("base_temperature is set by handle_mcu_identify");
        let slope = self.slope.get().expect("slope is set by handle_mcu_identify");
        (temp - base) / slope
    }

    /// `calc_base(self, temp, adc)` (lines 61-62).
    fn calc_base(&self, temp: f64, adc: f64) -> f64 {
        let slope = self.slope.get().expect("slope is set before calc_base");
        temp - adc * slope
    }
}

/// The `self.calc_temp` bound method handed to `HelperTemperatureDiagnostics`
/// (line 36); the helper keeps it as its `calc_temp_cb`.
struct CalcTempConverter {
    shared: Arc<Shared>,
}

impl AdcConvert for CalcTempConverter {
    fn calc_temp(&self, adc: f64) -> KResult<f64> {
        Ok(self.shared.calc_temp(adc))
    }

    fn calc_adc(&self, temp: f64) -> KResult<f64> {
        Ok(self.shared.calc_adc(temp))
    }
}

/// `PrinterTemperatureMCU` (`temperature_mcu.py:15`).
pub struct PrinterTemperatureMCU {
    shared: Arc<Shared>,
    mcu_adc: AdcPin,
    diag_helper: Arc<HelperTemperatureDiagnostics>,
    debug_read_cmd: SCell<Option<CommandQueryWrapper>>,
    temp1: Option<f64>,
    adc1: Option<f64>,
    temp2: Option<f64>,
    adc2: Option<f64>,
}

impl PrinterTemperatureMCU {
    /// `PrinterTemperatureMCU.__init__(config)` (lines 16-43). Returns an
    /// `Arc` because the `klippy:mcu_identify` handler and the ADC callback
    /// both capture the object, like the Python bound methods do.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let shared = Arc::new(Shared::new());
        // Read config (lines 24-30).
        let mcu_name = config
            .get(
                "sensor_mcu",
                GetStrOpts { default: Some(Some("mcu".to_string())), ..Default::default() },
            )?
            .expect("default 'mcu'");
        let temp1 = config.getfloat(
            "sensor_temperature1",
            GetFloatOpts { default: Some(None), ..Default::default() },
        )?;
        let (adc1, temp2, adc2) = match temp1 {
            None => (None, None, None),
            Some(_) => {
                let adc1 = config
                    .getfloat(
                        "sensor_adc1",
                        GetFloatOpts { minval: Some(0.), maxval: Some(1.), ..Default::default() },
                    )?
                    .expect("sensor_adc1 is required with sensor_temperature1");
                let temp2 = config.getfloat(
                    "sensor_temperature2",
                    GetFloatOpts { default: Some(None), ..Default::default() },
                )?;
                let adc2 = match temp2 {
                    None => None,
                    Some(_) => Some(
                        config
                            .getfloat(
                                "sensor_adc2",
                                GetFloatOpts {
                                    minval: Some(0.),
                                    maxval: Some(1.),
                                    ..Default::default()
                                },
                            )?
                            .expect("sensor_adc2 is required with sensor_temperature2"),
                    ),
                };
                (Some(adc1), temp2, adc2)
            }
        };
        // Setup ADC port (lines 31-34).
        let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let pin_desc = format!("{mcu_name}:ADC_TEMPERATURE");
        let mcu_adc = match pins.setup_pin("adc", &pin_desc)? {
            Pin::Adc(adc) => adc,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "pins.setup_pin('adc', ...) did not return an ADC pin".to_string(),
                });
            }
        };
        {
            let shared = Arc::clone(&shared);
            mcu_adc.setup_adc_callback(Box::new(move |samples| {
                let Some(&(read_time, read_value)) = samples.last() else { return };
                let temp = shared.calc_temp(read_value);
                let mut callback = shared.temperature_callback.take_callback();
                if let Some(callback) = callback.as_mut() {
                    callback(read_time + SAMPLE_DELAY, temp);
                }
            }));
        }
        // `adc_temperature.HelperTemperatureDiagnostics(config, self.mcu_adc,
        // self.calc_temp)` (lines 35-36).
        let diag_helper = HelperTemperatureDiagnostics::new(
            config,
            mcu_adc.clone(),
            Arc::new(CalcTempConverter { shared: Arc::clone(&shared) }),
        )?;
        let sensor = Arc::new(PrinterTemperatureMCU {
            shared,
            mcu_adc,
            diag_helper,
            debug_read_cmd: SCell::new(None),
            temp1,
            adc1,
            temp2,
            adc2,
        });
        // Register callbacks (lines 37-43).
        if is_debugoutput(&printer) {
            sensor.mcu_adc.setup_adc_sample(
                REPORT_TIME,
                AdcSampleOpts {
                    sample_time: SAMPLE_TIME,
                    sample_count: SAMPLE_COUNT,
                    ..Default::default()
                },
            );
            return Ok(sensor);
        }
        let handler = Arc::clone(&sensor);
        printer.register_event_handler(
            EV_KLIPPY_MCU_IDENTIFY,
            Box::new(move |_args: &[EventArg]| {
                handler.handle_mcu_identify()?;
                Ok(PyValue::None)
            }),
        );
        Ok(sensor)
    }

    /// `self.base_temperature` (line 18, `None` until `handle_mcu_identify`).
    pub fn base_temperature(&self) -> Option<f64> {
        self.shared.base_temperature.get()
    }

    /// `self.slope` (line 18, `None` until `handle_mcu_identify`).
    pub fn slope(&self) -> Option<f64> {
        self.shared.slope()
    }

    /// `self.temp1` (line 25).
    pub fn temp1(&self) -> Option<f64> {
        self.temp1
    }

    /// `self.adc1` (line 27).
    pub fn adc1(&self) -> Option<f64> {
        self.adc1
    }

    /// `self.temp2` (line 28).
    pub fn temp2(&self) -> Option<f64> {
        self.temp2
    }

    /// `self.adc2` (line 30).
    pub fn adc2(&self) -> Option<f64> {
        self.adc2
    }

    /// `self.diag_helper` (line 35).
    pub fn diag_helper(&self) -> Arc<HelperTemperatureDiagnostics> {
        Arc::clone(&self.diag_helper)
    }

    /// `setup_callback(self, temperature_callback)` (lines 44-45).
    pub fn setup_callback(&self, temperature_callback: TemperatureCallback) {
        *self.shared.temperature_callback.borrow_mut() = Some(temperature_callback);
    }

    /// `get_report_time_delta(self)` (lines 47-48).
    pub fn get_report_time_delta(&self) -> f64 {
        REPORT_TIME
    }

    /// `setup_minmax(self, min_temp, max_temp)` (lines 49-51).
    pub fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        self.shared.min_temp.set(min_temp);
        self.shared.max_temp.set(max_temp);
    }

    /// `calc_temp(self, adc)` (lines 57-58).
    pub fn calc_temp(&self, adc: f64) -> f64 {
        self.shared.calc_temp(adc)
    }

    /// `calc_adc(self, temp)` (lines 59-60).
    pub fn calc_adc(&self, temp: f64) -> f64 {
        self.shared.calc_adc(temp)
    }

    /// `calc_base(self, temp, adc)` (lines 61-62).
    pub fn calc_base(&self, temp: f64, adc: f64) -> f64 {
        self.shared.calc_base(temp, adc)
    }

    /// `handle_mcu_identify(self)` (lines 63-104). `config_unknown` raises a
    /// config error, so the result is fallible.
    pub fn handle_mcu_identify(&self) -> KResult<()> {
        // Obtain mcu information (lines 64-68).
        let mcu = self.mcu_adc.get_mcu();
        let debug_read_cmd = mcu.lookup_query_command(
            "debug_read order=%c addr=%u",
            "debug_result val=%u",
            LookupQueryCommandOpts::default(),
        )?;
        let mcu_type = mcu.get_constants().get("MCU").cloned().unwrap_or_default();
        // `self.debug_read_cmd` is set before the configuration functions
        // run: `config_samd21` and the stm32 variants read calibration words.
        *self.debug_read_cmd.borrow_mut() = Some(debug_read_cmd);
        *self.shared.mcu_type.borrow_mut() = mcu_type;
        // Run MCU specific configuration (lines 69-89).
        self.dispatch_config()?;
        // Log (lines 90-91).
        let base = self.shared.base_temperature.get().expect("the config function sets the base");
        let slope = self.shared.slope.get().expect("the config function sets the slope");
        log::info(&identify_log_line(&mcu.get_name(), base, slope));
        // Setup manual base/slope override (lines 92-96).
        if let Some(temp1) = self.temp1 {
            let adc1 = self.adc1.expect("adc1 is read with temp1");
            if let Some(temp2) = self.temp2 {
                let adc2 = self.adc2.expect("adc2 is read with temp2");
                self.shared.set_slope((temp2 - temp1) / (adc2 - adc1));
            }
            self.shared.set_base_from(temp1, adc1);
        }
        // Setup min/max checks (lines 97-104).
        let min_temp = self.shared.min_temp.get();
        let max_temp = self.shared.max_temp.get();
        let adc_a = self.shared.calc_adc(min_temp);
        let adc_b = self.shared.calc_adc(max_temp);
        let (min_adc, max_adc) = if adc_b < adc_a { (adc_b, adc_a) } else { (adc_a, adc_b) };
        self.mcu_adc.setup_adc_sample(
            REPORT_TIME,
            AdcSampleOpts {
                sample_time: SAMPLE_TIME,
                sample_count: SAMPLE_COUNT,
                batch_num: 1,
                minval: min_adc,
                maxval: max_adc,
                range_check_count: RANGE_CHECK_COUNT,
            },
        );
        self.diag_helper.setup_diag_minmax(min_temp, max_temp, min_adc, max_adc);
        Ok(())
    }

    /// The `cfg_funcs` table of `handle_mcu_identify` (lines 69-89): the
    /// first `startswith` match wins, and the empty name catches everything
    /// else with `config_unknown`.
    fn dispatch_config(&self) -> KResult<()> {
        let mcu_type = self.shared.mcu_type.cloned();
        if mcu_type.starts_with("rp2") {
            self.config_rp2040();
        } else if mcu_type.starts_with("sam3") {
            self.config_sam3();
        } else if mcu_type.starts_with("sam4") {
            self.config_sam4();
        } else if mcu_type.starts_with("same70") {
            self.config_same70();
        } else if mcu_type.starts_with("samd21") {
            self.config_samd21(SAMD21_CAL_ADDR)?;
        } else if mcu_type.starts_with("samd51") || mcu_type.starts_with("same5") {
            self.config_samd51()?;
        } else if mcu_type.starts_with("stm32f1") {
            self.config_stm32f1();
        } else if mcu_type.starts_with("stm32f2") {
            self.config_stm32f2();
        } else if mcu_type.starts_with("stm32f4") {
            self.config_stm32f4(STM32F4_CAL_ADDR1, STM32F4_CAL_ADDR2)?;
        } else if mcu_type.starts_with("stm32f042") {
            self.config_stm32f0x2()?;
        } else if mcu_type.starts_with("stm32f070") {
            self.config_stm32f070()?;
        } else if mcu_type.starts_with("stm32f072") {
            self.config_stm32f0x2()?;
        } else if mcu_type.starts_with("stm32g0")
            || mcu_type.starts_with("stm32g4")
            || mcu_type.starts_with("stm32l4")
        {
            self.config_stm32g0()?;
        } else if mcu_type.starts_with("stm32h723") {
            self.config_stm32h723()?;
        } else if mcu_type.starts_with("stm32h7") {
            self.config_stm32h7()?;
        } else {
            return self.config_unknown();
        }
        Ok(())
    }

    /// `config_unknown(self)` (lines 105-107).
    fn config_unknown(&self) -> KResult<()> {
        let message = percent_format(
            "MCU temperature not supported on %s",
            &[PctValue::Str(self.shared.mcu_type.cloned())],
        )
        .expect("constant format");
        Err(KlippyError::Config(message))
    }

    /// `config_rp2040(self)` (lines 108-110).
    fn config_rp2040(&self) {
        self.shared.set_slope(3.3 / -0.001721);
        self.shared.set_base_from(27., 0.706 / 3.3);
    }

    /// `config_sam3(self)` (lines 111-113).
    fn config_sam3(&self) {
        self.shared.set_slope(3.3 / 0.002650);
        self.shared.set_base_from(27., 0.8 / 3.3);
    }

    /// `config_sam4(self)` (lines 114-116).
    fn config_sam4(&self) {
        self.shared.set_slope(3.3 / 0.004700);
        self.shared.set_base_from(27., 1.44 / 3.3);
    }

    /// `config_same70(self)` (lines 117-119).
    fn config_same70(&self) {
        self.shared.set_slope(3.3 / 0.002330);
        self.shared.set_base_from(25., 0.72 / 3.3);
    }

    /// `config_samd21(self, addr=0x00806030)` (lines 120-134).
    fn config_samd21(&self, addr: i64) -> KResult<()> {
        fn get1v(val: i64) -> f64 {
            let val = if val & 0x80 != 0 { 0x100 - val } else { val };
            1. - (val as f64) / 1000.
        }
        let cal1 = self.read32(addr)?;
        let cal2 = self.read32(addr + 4)?;
        let room_temp = (cal1 & 0xff) as f64 + ((cal1 >> 8) & 0xf) as f64 / 10.;
        let hot_temp = ((cal1 >> 12) & 0xff) as f64 + ((cal1 >> 20) & 0xf) as f64 / 10.;
        let room_1v = get1v((cal1 >> 24) & 0xff);
        let hot_1v = get1v(cal2 & 0xff);
        let room_adc = ((cal2 >> 8) & 0xfff) as f64 * room_1v / (3.3 * 4095.);
        let hot_adc = ((cal2 >> 20) & 0xfff) as f64 * hot_1v / (3.3 * 4095.);
        self.shared.set_slope((hot_temp - room_temp) / (hot_adc - room_adc));
        self.shared.set_base_from(room_temp, room_adc);
        Ok(())
    }

    /// `config_samd51(self)` (lines 135-136).
    fn config_samd51(&self) -> KResult<()> {
        self.config_samd21(SAMD51_CAL_ADDR)
    }

    /// `config_stm32f1(self)` (lines 137-139).
    fn config_stm32f1(&self) {
        self.shared.set_slope(3.3 / -0.004300);
        self.shared.set_base_from(25., 1.43 / 3.3);
    }

    /// `config_stm32f2(self)` (lines 140-142).
    fn config_stm32f2(&self) {
        self.shared.set_slope(3.3 / 0.002500);
        self.shared.set_base_from(25., 0.76 / 3.3);
    }

    /// `config_stm32f4(self, addr1=0x1FFF7A2C, addr2=0x1FFF7A2E)`
    /// (lines 143-147).
    fn config_stm32f4(&self, addr1: i64, addr2: i64) -> KResult<()> {
        let cal_adc_30 = (self.read16(addr1)? as f64) / 4095.;
        let cal_adc_110 = (self.read16(addr2)? as f64) / 4095.;
        self.shared.set_slope((110. - 30.) / (cal_adc_110 - cal_adc_30));
        self.shared.set_base_from(30., cal_adc_30);
        Ok(())
    }

    /// `config_stm32f0x2(self)` (lines 148-149).
    fn config_stm32f0x2(&self) -> KResult<()> {
        self.config_stm32f4(STM32F0X2_CAL_ADDR1, STM32F0X2_CAL_ADDR2)
    }

    /// `config_stm32f070(self)` (lines 150-153).
    fn config_stm32f070(&self) -> KResult<()> {
        self.shared.set_slope(3.3 / -0.004300);
        let cal_adc_30 = (self.read16(STM32F070_CAL_ADDR)? as f64) / 4095.;
        self.shared.set_base_from(30., cal_adc_30);
        Ok(())
    }

    /// `config_stm32g0(self)` (lines 154-158), shared by `stm32g4` and
    /// `stm32l4` (lines 81-82).
    fn config_stm32g0(&self) -> KResult<()> {
        let cal_adc_30 = (self.read16(STM32G0_CAL_ADDR_30)? as f64) * 3.0 / (3.3 * 4095.);
        let cal_adc_130 = (self.read16(STM32G0_CAL_ADDR_130)? as f64) * 3.0 / (3.3 * 4095.);
        self.shared.set_slope((130. - 30.) / (cal_adc_130 - cal_adc_30));
        self.shared.set_base_from(30., cal_adc_30);
        Ok(())
    }

    /// `config_stm32h723(self)` (lines 159-163).
    fn config_stm32h723(&self) -> KResult<()> {
        let cal_adc_30 = (self.read16(STM32H7_CAL_ADDR_30)? as f64) / 4095.;
        let cal_adc_130 = (self.read16(STM32H7_CAL_ADDR_HIGH)? as f64) / 4095.;
        self.shared.set_slope((130. - 30.) / (cal_adc_130 - cal_adc_30));
        self.shared.set_base_from(30., cal_adc_30);
        Ok(())
    }

    /// `config_stm32h7(self)` (lines 164-168).
    fn config_stm32h7(&self) -> KResult<()> {
        let cal_adc_30 = (self.read16(STM32H7_CAL_ADDR_30)? as f64) / 65535.;
        let cal_adc_110 = (self.read16(STM32H7_CAL_ADDR_HIGH)? as f64) / 65535.;
        self.shared.set_slope((110. - 30.) / (cal_adc_110 - cal_adc_30));
        self.shared.set_base_from(30., cal_adc_30);
        Ok(())
    }

    /// `read16(self, addr)` (lines 169-171).
    pub fn read16(&self, addr: i64) -> KResult<i64> {
        self.read_word(1, addr)
    }

    /// `read32(self, addr)` (lines 172-174).
    pub fn read32(&self, addr: i64) -> KResult<i64> {
        self.read_word(2, addr)
    }

    /// `params = self.debug_read_cmd.send([order, addr]); params['val']`.
    fn read_word(&self, order: i64, addr: i64) -> KResult<i64> {
        let cmd = self.debug_read_cmd.borrow().clone();
        let Some(cmd) = cmd else {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'send'".to_string(),
            });
        };
        let params = cmd.send(
            vec![PyValue::Int(PyInt::from_i64(order)), PyValue::Int(PyInt::from_i64(addr))],
            QuerySendOpts::default(),
        )?;
        let PyValue::Dict(map) = params else {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!("'{}' object is not subscriptable", type_name(&params)),
            });
        };
        match map.get(&PyValue::Str("val".to_string())) {
            Some(PyValue::Int(val)) => Ok(i64::try_from(val.inner()).expect("debug_result val fits i64")),
            _ => Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: repr_key("val"),
            }),
        }
    }
}

impl TemperatureSensorApi for PrinterTemperatureMCU {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        PrinterTemperatureMCU::setup_minmax(self, min_temp, max_temp);
        Ok(())
    }

    fn setup_callback(&self, callback: TemperatureCallback) {
        PrinterTemperatureMCU::setup_callback(self, callback);
    }

    fn get_report_time_delta(&self) -> f64 {
        PrinterTemperatureMCU::get_report_time_delta(self)
    }
}

/// `"mcu_temperature '%s' nominal base=%.6f slope=%.6f"` (line 90).
fn identify_log_line(mcu_name: &str, base_temperature: f64, slope: f64) -> String {
    percent_format(
        "mcu_temperature '%s' nominal base=%.6f slope=%.6f",
        &[
            PctValue::Str(mcu_name.to_string()),
            PctValue::Float(base_temperature),
            PctValue::Float(slope),
        ],
    )
    .expect("constant format")
}

/// `printer.get_start_args().get('debugoutput') is not None` (line 38).
fn is_debugoutput(printer: &Printer) -> bool {
    match printer.get_start_args() {
        PyValue::Dict(map) => !matches!(
            map.get(&PyValue::Str("debugoutput".to_string())),
            Some(PyValue::None) | None
        ),
        _ => false,
    }
}

/// `load_config(config)` (lines 176-178).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let pheaters = load_interface::<dyn PrinterHeatersApi>(
        &*printer,
        config,
        "heaters",
        LookupDefault::Raise,
    )?
    .expect("LookupDefault::Raise never yields None");
    let factory: SensorFactory = Arc::new(|config: &Config| {
        Ok(PrinterTemperatureMCU::new(config)? as Arc<dyn TemperatureSensorApi>)
    });
    pheaters.add_sensor_factory("temperature_mcu", factory);
    Ok(None)
}

/// Module entry point for the `temperature_mcu` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config: Some(load_config), ..Default::default() }
}
