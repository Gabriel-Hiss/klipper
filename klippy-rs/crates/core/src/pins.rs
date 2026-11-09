//! Port of `klippy/pins.py`.
//!
//! Pin description parsing ([`PinsApi::parse_pin`] / [`PinsApi::lookup_pin`]),
//! chip registration and pin-sharing bookkeeping on [`PrinterPins`], plus
//! [`PinResolver`] pin aliases, reservations and command rewriting. Errors
//! are [`KlippyError::Pins`] (`pins.error`, `klippy/pins.py:8`).
//!
//! Mutable state lives in [`GCell`]s so the objects stay `Send + Sync`
//! like every other `klippy_api` contract trait.

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{
    Chip, LookupPinOpts, ParsePinOpts, Pin, PinParams, PinResolverApi, PinsApi,
};
use klippy_api::printer::ModuleFactory;
use klippy_api::ObjectRef;
use pytext::pyre;

/// `re_pin` pattern from `klippy/pins.py:16`.
const RE_PIN_PATTERN: &str = r"(?P<prefix>[ _]pin=)(?P<name>[^ ]*)";

/// `pins.error(message)` (`klippy/pins.py:8`).
fn pins_error(message: impl Into<String>) -> KlippyError {
    KlippyError::Pins(message.into())
}

/// Python whitespace for `str.strip()` / `str.split()`. Mirrors
/// `crates/core/src/gcode_parse.rs` (`\u{1c}`-`\u{1f}` included).
fn is_py_space(c: char) -> bool {
    c.is_whitespace() || matches!(c, '\u{1c}' | '\u{1d}' | '\u{1e}' | '\u{1f}')
}

fn py_strip(s: &str) -> String {
    s.trim_matches(is_py_space).to_string()
}

/// `''.join(pin.split()) != pin` is true iff `pin` holds any Python
/// whitespace character.
fn pin_has_whitespace(pin: &str) -> bool {
    pin.chars().any(is_py_space)
}

fn pin_has_forbidden(pin: &str) -> bool {
    pin.chars().any(|c| matches!(c, '^' | '~' | '!' | ':')) || pin_has_whitespace(pin)
}

fn invalid_pin_description(pin_desc: &str, can_invert: bool, can_pullup: bool) -> String {
    let mut format = String::new();
    if can_pullup {
        format.push_str("[^~] ");
    }
    if can_invert {
        format.push_str("[!] ");
    }
    format!("Invalid pin description '{pin_desc}'\nFormat is: {format}[chip_name:] pin_name")
}

// ---------------------------------------------------------------------------
// PinResolver (`klippy/pins.py:18`)
// ---------------------------------------------------------------------------

/// Mutable state of [`PinResolver`], mirroring the Python instance dict
/// (`klippy/pins.py:19-23`).
struct ResolverState {
    validate_aliases: bool,
    reserved: IndexMap<String, String>,
    aliases: IndexMap<String, String>,
    active_pins: IndexMap<String, String>,
}

/// Port of `pins.PinResolver` (`klippy/pins.py:18`).
pub struct PinResolver {
    state: GCell<ResolverState>,
}

impl PinResolver {
    /// `PinResolver(validate_aliases=True)` (`klippy/pins.py:19`).
    pub fn new(validate_aliases: bool) -> PinResolver {
        PinResolver {
            state: GCell::new(ResolverState {
                validate_aliases,
                reserved: IndexMap::new(),
                aliases: IndexMap::new(),
                active_pins: IndexMap::new(),
            }),
        }
    }

    /// `validate_aliases` attribute.
    pub fn validate_aliases(&self) -> bool {
        self.state.borrow().validate_aliases
    }

    /// `reserved` entries in insertion order.
    pub fn reserved(&self) -> Vec<(String, String)> {
        self.state
            .borrow()
            .reserved
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }

    /// `aliases` entries in insertion order.
    pub fn aliases(&self) -> Vec<(String, String)> {
        self.state
            .borrow()
            .aliases
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }

    /// `active_pins` entries in insertion order.
    pub fn active_pins(&self) -> Vec<(String, String)> {
        self.state
            .borrow()
            .active_pins
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
}

impl Default for PinResolver {
    fn default() -> Self {
        PinResolver::new(true)
    }
}

impl PinResolverApi for PinResolver {
    /// `reserve_pin` (`klippy/pins.py:24`).
    fn reserve_pin(&self, pin: &str, reserve_name: &str) -> KResult<()> {
        let mut state = self.state.borrow_mut();
        if let Some(existing) = state.reserved.get(pin) {
            if existing != reserve_name {
                return Err(pins_error(format!(
                    "Pin {pin} reserved for {existing} - can't reserve for {reserve_name}"
                )));
            }
        }
        state.reserved.insert(pin.to_string(), reserve_name.to_string());
        Ok(())
    }

    /// `alias_pin` (`klippy/pins.py:29`).
    fn alias_pin(&self, alias: &str, pin: &str) -> KResult<()> {
        let mut state = self.state.borrow_mut();
        if let Some(existing) = state.aliases.get(alias) {
            if existing != pin {
                let existing = existing.clone();
                return Err(pins_error(format!(
                    "Alias {alias} mapped to {existing} - can't alias to {pin}"
                )));
            }
        }
        if pin_has_forbidden(pin) {
            return Err(pins_error(format!("Invalid pin alias '{pin}'\n")));
        }
        let mut pin = pin.to_string();
        if let Some(deref) = state.aliases.get(&pin) {
            pin = deref.clone();
        }
        state.aliases.insert(alias.to_string(), pin.clone());
        // Update existing entries that pointed at the alias. Values (not
        // keys) change, so insertion order is preserved, as in Python.
        let keys: Vec<String> = state.aliases.keys().cloned().collect();
        for existing_alias in &keys {
            if state.aliases.get(existing_alias).map(String::as_str) == Some(alias) {
                state.aliases.insert(existing_alias.clone(), pin.clone());
            }
        }
        Ok(())
    }

    /// `update_command` (`klippy/pins.py:41`): `re_pin.sub(pin_fixup, cmd)`.
    ///
    /// If `pin_fixup` raises, Python's `re.sub` aborts and the
    /// `active_pins` insertions already made stay; this does the same by
    /// keeping the single cell borrow for the whole substitution (no
    /// reactor pause happens inside, so the borrow is safe).
    fn update_command(&self, cmd: &str) -> KResult<String> {
        let re = pyre::compile(RE_PIN_PATTERN).expect("re_pin is a supported pattern");
        let mut state = self.state.borrow_mut();
        let mut error: Option<KlippyError> = None;
        let result = re.sub_pin(cmd, |prefix: &str, name: &str| -> String {
            if error.is_some() {
                return prefix.to_string();
            }
            let pin_id = state.aliases.get(name).cloned().unwrap_or_else(|| name.to_string());
            // `active_pins.setdefault(pin_id, name)`.
            let assigned_to = state
                .active_pins
                .entry(pin_id.clone())
                .or_insert_with(|| name.to_string())
                .clone();
            if name != assigned_to && state.validate_aliases {
                error = Some(pins_error(format!("pin {name} is an alias for {assigned_to}")));
                return prefix.to_string();
            }
            if let Some(reserved_for) = state.reserved.get(&pin_id) {
                error = Some(pins_error(format!(
                    "pin {name} is reserved for {reserved_for}"
                )));
                return prefix.to_string();
            }
            format!("{prefix}{pin_id}")
        });
        drop(state);
        if let Some(err) = error {
            return Err(err);
        }
        Ok(result)
    }
}

// ---------------------------------------------------------------------------
// PrinterPins (`klippy/pins.py:60`)
// ---------------------------------------------------------------------------

/// Mutable state of [`PrinterPins`], mirroring the Python instance dict
/// (`klippy/pins.py:62-66`).
struct PinsState {
    chips: IndexMap<String, Chip>,
    active_pins: IndexMap<String, PinParams>,
    pin_resolvers: IndexMap<String, Arc<PinResolver>>,
    allow_multi_use_pins: IndexMap<String, bool>,
}

/// Port of `pins.PrinterPins` (`klippy/pins.py:60`).
pub struct PrinterPins {
    state: GCell<PinsState>,
}

impl PrinterPins {
    /// `PrinterPins.__init__` (`klippy/pins.py:62`).
    pub fn new() -> PrinterPins {
        PrinterPins {
            state: GCell::new(PinsState {
                chips: IndexMap::new(),
                active_pins: IndexMap::new(),
                pin_resolvers: IndexMap::new(),
                allow_multi_use_pins: IndexMap::new(),
            }),
        }
    }

    /// Registered chip names in insertion order.
    pub fn chip_names(&self) -> Vec<String> {
        self.state.borrow().chips.keys().cloned().collect()
    }
    /// The concrete resolver for a registered chip (`register_chip` creates
    /// one with `validate_aliases=True`, `klippy/pins.py:127-131`).
    pub fn resolver(&self, chip_name: &str) -> Option<Arc<PinResolver>> {
        self.state.borrow().pin_resolvers.get(chip_name).cloned()
    }

    /// Stored `active_pins` entries (`share_name` -> params) in insertion order.
    pub fn active_pins(&self) -> Vec<(String, PinParams)> {
        self.state
            .borrow()
            .active_pins
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }

    /// `allow_multi_use_pins` entries in insertion order.
    pub fn allow_multi_use_pins(&self) -> Vec<(String, bool)> {
        self.state
            .borrow()
            .allow_multi_use_pins
            .iter()
            .map(|(k, v)| (k.clone(), *v))
            .collect()
    }

    /// `parse_pin` (`klippy/pins.py:67`) against the current state.
    fn parse_pin_with(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams> {
        let mut desc = py_strip(pin_desc);
        let mut invert: i64 = 0;
        let mut pullup: i64 = 0;
        if opts.can_pullup && (desc.starts_with('^') || desc.starts_with('~')) {
            pullup = 1;
            if desc.starts_with('~') {
                pullup = -1;
            }
            desc = py_strip(&desc[1..]);
        }
        if opts.can_invert && desc.starts_with('!') {
            invert = 1;
            desc = py_strip(&desc[1..]);
        }
        let (chip_name, pin) = if let Some((chip_raw, pin_raw)) = desc.split_once(':') {
            (py_strip(chip_raw), py_strip(pin_raw))
        } else {
            ("mcu".to_string(), desc.clone())
        };
        let chip = {
            let state = self.state.borrow();
            state
                .chips
                .get(&chip_name)
                .cloned()
                .ok_or_else(|| pins_error(format!("Unknown pin chip name '{chip_name}'")))?
        };
        if pin_has_forbidden(&pin) {
            return Err(pins_error(invalid_pin_description(
                pin_desc,
                opts.can_invert,
                opts.can_pullup,
            )));
        }
        Ok(PinParams {
            chip: chip.get_mcu(),
            chip_name,
            pin,
            invert,
            pullup,
            share_type: None,
        })
    }
}

impl Default for PrinterPins {
    fn default() -> Self {
        PrinterPins::new()
    }
}

impl PinsApi for PrinterPins {
    /// `parse_pin` (`klippy/pins.py:67`).
    fn parse_pin(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams> {
        self.parse_pin_with(pin_desc, opts)
    }

    /// `lookup_pin` (`klippy/pins.py:96`).
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let pin_params = self.parse_pin_with(
            pin_desc,
            ParsePinOpts { can_invert: opts.can_invert, can_pullup: opts.can_pullup },
        )?;
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        let mut state = self.state.borrow_mut();
        if let Some(share_params) = state.active_pins.get(&share_name) {
            let share_params = share_params.clone();
            if state.allow_multi_use_pins.contains_key(&share_name) {
                return Ok(share_params);
            }
            // `share_params['share_type']` exists on every stored entry.
            let stored = share_params.share_type.clone();
            if opts.share_type.is_none() || stored.as_deref() != opts.share_type.as_deref() {
                let pin = share_params.pin.clone();
                return Err(pins_error(format!("pin {pin} used multiple times in config")));
            }
            if pin_params.invert != share_params.invert
                || pin_params.pullup != share_params.pullup
            {
                let pin = share_params.pin.clone();
                return Err(pins_error(format!("Shared pin {pin} must have same polarity")));
            }
            return Ok(share_params);
        }
        let mut pin_params = pin_params;
        pin_params.share_type = opts.share_type.clone();
        let stored = pin_params.clone();
        state.active_pins.insert(share_name, stored);
        Ok(pin_params)
    }

    /// `setup_pin` (`klippy/pins.py:114`).
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        let can_invert = matches!(pin_type, "endstop" | "digital_out" | "pwm");
        let can_pullup = pin_type == "endstop";
        let params = self.lookup_pin(
            pin_desc,
            LookupPinOpts { can_invert, can_pullup, share_type: None },
        )?;
        let chip = {
            let state = self.state.borrow();
            state
                .chips
                .get(&params.chip_name)
                .cloned()
                .expect("parse_pin only returns registered chips")
        };
        chip.setup_pin(pin_type, params)
    }

    /// `reset_pin_sharing` (`klippy/pins.py:119`): `del
    /// self.active_pins[share_name]`.
    ///
    /// The `PinsApi` contract returns `()` with no error channel, so a
    /// missing entry (Python `KeyError`) panics with the `KeyError` text,
    /// as an uncaught exception would crash the Python host too.
    fn reset_pin_sharing(&self, pin_params: PinParams) {
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        let mut state = self.state.borrow_mut();
        if state.active_pins.shift_remove(&share_name).is_none() {
            panic!("KeyError: {}", pycompat::text::str_repr(&share_name));
        }
    }

    /// `get_pin_resolver` (`klippy/pins.py:122`).
    fn get_pin_resolver(&self, chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        let state = self.state.borrow();
        state
            .pin_resolvers
            .get(chip_name)
            .cloned()
            .map(|r| r as klippy_api::mcu::PinResolver)
            .ok_or_else(|| pins_error(format!("Unknown chip name '{chip_name}'")))
    }

    /// `register_chip` (`klippy/pins.py:126`).
    fn register_chip(&self, chip_name: &str, chip: Chip) -> KResult<()> {
        let chip_name = py_strip(chip_name);
        let mut state = self.state.borrow_mut();
        if state.chips.contains_key(&chip_name) {
            return Err(pins_error(format!("Duplicate chip name '{chip_name}'")));
        }
        state.chips.insert(chip_name.clone(), chip);
        state.pin_resolvers.insert(chip_name, Arc::new(PinResolver::new(true)));
        Ok(())
    }

    /// `allow_multi_use_pin` (`klippy/pins.py:132`).
    fn allow_multi_use_pin(&self, pin_desc: &str) -> KResult<()> {
        let pin_params = self.parse_pin_with(pin_desc, ParsePinOpts::default())?;
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        let mut state = self.state.borrow_mut();
        state.allow_multi_use_pins.insert(share_name, true);
        Ok(())
    }
}

klippy_api::printer_object!(PrinterPins; dyn PinsApi);

/// `add_printer_objects(config)` (`klippy/pins.py:137`).
fn add_printer_objects(config: &Config) -> KResult<()> {
    config
        .get_printer()
        .add_object("pins", Arc::new(PrinterPins::new()) as ObjectRef)
}

/// Module entry point for the `pins` printer object (`klippy/pins.py:137`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: Some(add_printer_objects),
    }
}
