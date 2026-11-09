use super::*;

pub(super) fn dict_item<'a>(value: &'a PyValue, key: &str) -> Option<&'a PyValue> {
    match value {
        PyValue::Dict(items) => items.get(&PyValue::Str(key.into())),
        _ => None,
    }
}

fn integer(value: PyInt) -> KResult<i64> {
    value.inner().to_string().parse().map_err(|_| KlippyError::Other {
        class_name: "OverflowError".into(), message: "Python int too large to convert to C long".into(),
    })
}

fn count(value: Option<usize>) -> Option<i64> {
    value.map(|n| i64::try_from(n).expect("config list count fits i64"))
}

fn optional<T>(value: Option<T>, wrap: impl FnOnce(T) -> PyValue) -> PyValue {
    value.map(wrap).unwrap_or(PyValue::None)
}

/// The Python sequence a `getlist`-family default is: a `list` or a
/// `tuple` (what the file parser always produces).
fn sequence<T>(items: Vec<T>, is_list: bool, wrap: impl Fn(T) -> PyValue) -> PyValue {
    let items = items.into_iter().map(wrap).collect();
    if is_list { PyValue::List(items) } else { PyValue::Tuple(items) }
}

// The typed `ConfigApi` defaults only produce values of the getter's own
// type (or the marked `int`/`list` forms matched by each getter), and
// values parsed from the file always have that type.
fn unexpected(value: &PyValue, getter: &str) -> ! {
    unreachable!("{getter} produced {}", value.repr())
}

fn items(value: PyValue, getter: &str) -> Option<Vec<PyValue>> {
    match value {
        PyValue::None => None,
        PyValue::Tuple(items) | PyValue::List(items) => Some(items),
        other => unexpected(&other, getter),
    }
}

impl ConfigApi for ConfigWrapper {
    fn get_printer(&self) -> Printer { Arc::clone(&self.inner.printer) }
    fn get_name(&self) -> String { self.section_name().into() }
    fn get(&self, option: &str, opts: api::GetStrOpts) -> KResult<Option<String>> {
        let default = opts.default.map(|d| optional(d, PyValue::Str));
        match ConfigWrapper::get(self, option, default.as_ref(), opts.note_valid)? {
            PyValue::None => Ok(None),
            PyValue::Str(s) => Ok(Some(s)),
            other => unexpected(&other, "get"),
        }
    }
    fn getint(&self, option: &str, opts: api::GetIntOpts) -> KResult<Option<i64>> {
        let int = |n: i64| PyValue::Int(PyInt::from_i64(n));
        let default = opts.default.map(|d| optional(d, int));
        match ConfigWrapper::getint(self, option, default.as_ref(), opts.minval.map(int),
            opts.maxval.map(int), opts.note_valid)? {
            PyValue::None => Ok(None),
            PyValue::Int(n) => integer(n).map(Some),
            other => unexpected(&other, "getint"),
        }
    }
    fn getfloat(&self, option: &str, opts: api::GetFloatOpts) -> KResult<Option<f64>> {
        use api::IntArgs;
        let ints = opts.ints;
        let default = opts.default.map(|d| optional(d, |v| ints.value(IntArgs::DEFAULT, v)));
        match ConfigWrapper::getfloat(self, option, default.as_ref(),
            opts.minval.map(|v| ints.value(IntArgs::MINVAL, v)),
            opts.maxval.map(|v| ints.value(IntArgs::MAXVAL, v)),
            opts.above.map(|v| ints.value(IntArgs::ABOVE, v)),
            opts.below.map(|v| ints.value(IntArgs::BELOW, v)), opts.note_valid)? {
            PyValue::None => Ok(None),
            PyValue::Float(f) => Ok(Some(f)),
            // An `int` default, returned unconverted by Python.
            PyValue::Int(n) => integer(n).map(|n| Some(n as f64)),
            other => unexpected(&other, "getfloat"),
        }
    }
    fn getboolean(&self, option: &str, opts: api::GetBoolOpts) -> KResult<Option<bool>> {
        let int_default = opts.default_is_int;
        let default = opts.default.map(|d| optional(d, |b| if int_default {
            PyValue::Int(PyInt::from_i64(i64::from(b)))
        } else {
            PyValue::Bool(b)
        }));
        match ConfigWrapper::getboolean(self, option, default.as_ref(), opts.note_valid)? {
            PyValue::None => Ok(None),
            PyValue::Bool(b) => Ok(Some(b)),
            // An `int` default (`0`/`1`), returned unconverted by Python.
            PyValue::Int(n) => integer(n).map(|n| Some(n != 0)),
            other => unexpected(&other, "getboolean"),
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: api::GetChoiceOpts) -> KResult<PyValue> {
        let default = opts.default.map(|d| d.unwrap_or(PyValue::None));
        ConfigWrapper::getchoice(self, option, choices, default.as_ref(), opts.note_valid)
    }
    fn getlists(&self, option: &str, opts: api::GetListsOpts) -> KResult<PyValue> {
        let default = opts.default.map(|d| d.unwrap_or(PyValue::None));
        ConfigWrapper::getlists(self, option, default.as_ref(), &opts.seps, count(opts.count),
            opts.parser, opts.note_valid)
    }
    fn getlist(&self, option: &str, opts: api::GetListOpts) -> KResult<Option<Vec<String>>> {
        let default = opts.default.map(|d| optional(d, |v| sequence(v, opts.default_is_list, PyValue::Str)));
        let value = ConfigWrapper::getlist(self, option, default.as_ref(), &opts.sep,
            count(opts.count), opts.note_valid)?;
        Ok(items(value, "getlist").map(|items| items.into_iter().map(|item| match item {
            PyValue::Str(s) => s,
            other => unexpected(&other, "getlist"),
        }).collect()))
    }
    fn getintlist(&self, option: &str, opts: api::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        let default = opts.default.map(|d| optional(d, |v| sequence(v, opts.default_is_list, |n| PyValue::Int(PyInt::from_i64(n)))));
        let value = ConfigWrapper::getintlist(self, option, default.as_ref(), &opts.sep,
            count(opts.count), opts.note_valid)?;
        items(value, "getintlist").map(|items| items.into_iter().map(|item| match item {
            PyValue::Int(n) => integer(n),
            other => unexpected(&other, "getintlist"),
        }).collect()).transpose()
    }
    fn getfloatlist(&self, option: &str, opts: api::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        let default = opts.default.map(|d| optional(d, |v| sequence(v, opts.default_is_list, PyValue::Float)));
        let value = ConfigWrapper::getfloatlist(self, option, default.as_ref(), &opts.sep,
            count(opts.count), opts.note_valid)?;
        Ok(items(value, "getfloatlist").map(|items| items.into_iter().map(|item| match item {
            PyValue::Float(f) => f,
            other => unexpected(&other, "getfloatlist"),
        }).collect()))
    }
    fn getsection(&self, section: &str) -> Config { Arc::new(ConfigWrapper::getsection(self, section)) }
    fn has_section(&self, section: &str) -> bool { ConfigWrapper::has_section(self, section) }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        ConfigWrapper::get_prefix_sections(self, prefix).into_iter().map(|w| Arc::new(w) as Config).collect()
    }
    fn get_prefix_options(&self, prefix: &str) -> KResult<Vec<String>> {
        ConfigWrapper::get_prefix_options(self, prefix).map_err(config_error)
    }
    fn deprecate(&self, option: &str, value: Option<PyValue>) -> KResult<()> {
        ConfigWrapper::deprecate(self, option, value)
    }
}

impl PrinterConfig {
    fn expose(&self, wrapper: ConfigWrapper) -> Config {
        let parser = Arc::clone(&wrapper.inner.fileconfig);
        let config: Config = Arc::new(wrapper);
        let mut known = self.fileconfigs.borrow_mut();
        known.retain(|(wrapper, _)| wrapper.strong_count() != 0);
        known.push((Arc::downgrade(&config), parser));
        config
    }

    // ConfigApi intentionally exposes getters only; preserve the parser's
    // DEFAULT section and write order for wrappers handed out by this service.
    fn parser_for(&self, config: &Config) -> Arc<RawConfigParser> {
        let weak = Arc::downgrade(config);
        self.fileconfigs.borrow().iter().find(|(w,_)| w.ptr_eq(&weak))
            .map(|(_,p)| Arc::clone(p)).expect("config was created by this PrinterConfig")
    }
}

impl PrinterConfigApi for PrinterConfig {
    fn get_printer(&self) -> Printer { Arc::clone(&self.printer) }
    fn read_config(&self, filename: &str) -> KResult<Config> {
        PrinterConfig::read_config(self, filename).map(|w| self.expose(w))
    }
    fn read_main_config(&self) -> KResult<Config> {
        PrinterConfig::read_main_config(self).map(|w| self.expose(w))
    }
    fn log_config(&self, config: &Config) {
        let text = ConfigFileReader::build_config_string(&self.parser_for(config));
        self.printer.set_rollover_info("config", Some(&format!(
            "===== Config file =====\n{text}\n=======================")), true);
    }
    fn check_unused_options(&self, config: &Config) -> KResult<()> {
        let names = self.printer.lookup_objects(None).into_iter().map(|(name,_)| name).collect::<Vec<_>>();
        let names = names.iter().map(String::as_str).collect::<Vec<_>>();
        self.validate.borrow_mut().check_unused(&self.parser_for(config), &names).map_err(config_error)
    }
    fn runtime_warning(&self, msg: &str) { PrinterConfig::runtime_warning(self, msg) }
    fn deprecate(&self, section: &str, option: &str, value: Option<PyValue>, msg: Option<String>) {
        PrinterConfig::deprecate(self, section, option, value, msg)
    }
    fn deprecate_gcode(&self, cmd: &str, opts: api::DeprecateGcodeOpts) {
        let value = opts.value.filter(|v| *v != PyValue::None).map(|v| v.str());
        PrinterConfig::deprecate_gcode(self, cmd, opts.param.as_deref(), value.as_deref(), opts.msg)
    }
    fn deprecate_mcu_code(&self, mcu: &klippy_api::mcu::Mcu, feature: &str, msg: Option<String>) {
        let versions = if msg.is_none() {
            let args = self.printer.get_start_args();
            let status = mcu.get_status(None);
            (dict_item(&args, "software_version").map(PyValue::str).expect("software_version"),
             dict_item(&status, "mcu_version").map(PyValue::str).expect("mcu_version"))
        } else { (String::new(), String::new()) };
        PrinterConfig::deprecate_mcu_code(self, &mcu.get_name(), feature, msg, &versions.0, &versions.1)
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { PrinterConfig::get_status(self) }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        PrinterConfig::set(self, section, option, value).map_err(config_error)
    }
    fn remove_section(&self, section: &str) { PrinterConfig::remove_section(self, section) }
}

impl klippy_api::PrinterObject for PrinterConfig {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterConfigApi);
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(PrinterConfig::get_status(self)))
    }
}

fn add_early_printer_objects(printer: &Printer) -> KResult<()> {
    let config = PrinterConfig::new(Arc::clone(printer))?;
    printer.add_object("configfile", config)?;
    Ok(())
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory { add_early_printer_objects: Some(add_early_printer_objects), ..Default::default() }
}
