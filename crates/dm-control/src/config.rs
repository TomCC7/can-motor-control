//! TOML config schema, loader, and vendor codec registry.

use std::collections::HashMap;
use std::path::Path;

use motor_codec::{MotorCodec, MotorTypeId};
use serde::Deserialize;

use crate::error::Error;
use crate::robot::{Robot, RobotBuilder};
use crate::spec::MotorSpec;
use crate::transport::{CanBus, SocketCanBus};

/// Factory function that produces a vendor codec on demand.
pub type CodecFactory = Box<dyn Fn() -> Box<dyn MotorCodec> + Send + Sync>;

/// Function that resolves a motor type string to a `MotorTypeId` for a given
/// vendor.
pub type MotorTypeParser = Box<dyn Fn(&str) -> Option<MotorTypeId> + Send + Sync>;

/// Registry of vendor name → (codec factory, type parser).
///
/// `dm-control` ships empty by default — populating "damiao" lives in
/// `dm-codec` (or in a higher-level binding crate). The Python binding
/// (`dm-control-py`) registers damiao automatically; Rust users either build
/// a registry themselves or use the helper exposed by their vendor codec
/// crate.
pub struct CodecRegistry {
    factories: HashMap<String, (CodecFactory, MotorTypeParser)>,
}

impl Default for CodecRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl CodecRegistry {
    /// Empty registry.
    pub fn new() -> Self {
        Self {
            factories: HashMap::new(),
        }
    }

    /// Register a vendor codec + its type-string parser.
    pub fn register(
        &mut self,
        name: impl Into<String>,
        factory: impl Fn() -> Box<dyn MotorCodec> + Send + Sync + 'static,
        parser: impl Fn(&str) -> Option<MotorTypeId> + Send + Sync + 'static,
    ) -> &mut Self {
        self.factories
            .insert(name.into(), (Box::new(factory), Box::new(parser)));
        self
    }

    fn produce(&self, vendor: &str) -> Option<Box<dyn MotorCodec>> {
        self.factories.get(vendor).map(|(f, _)| f())
    }

    fn parse_type(&self, vendor: &str, s: &str) -> Option<MotorTypeId> {
        self.factories.get(vendor).and_then(|(_, p)| p(s))
    }

    /// Names of every registered vendor.
    pub fn vendor_names(&self) -> impl Iterator<Item = &str> {
        self.factories.keys().map(String::as_str)
    }
}

/// Top-level config schema.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RobotConfig {
    #[serde(default)]
    bus: HashMap<String, BusConfig>,
    #[serde(default)]
    group: Vec<GroupConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BusConfig {
    kind: String,
    interface: Option<String>,
    #[serde(default)]
    fd: bool,
    vendor: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct GroupConfig {
    name: String,
    kind: String,
    bus: String,
    #[serde(default)]
    default_control_mode: Option<String>,
    #[serde(default)]
    motors: Vec<MotorConfig>,
    motor: Option<MotorConfig>,
    /// Forbidden: vendor lives on the bus, not on the group. We accept it in
    /// the schema only so we can produce a helpful error message.
    vendor: Option<String>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct MotorConfig {
    name: String,
    #[serde(rename = "type")]
    type_str: String,
    send_id: u32,
    recv_id: u32,
}

impl Robot {
    /// Parse a TOML config file and build a Robot using the supplied codec
    /// registry. Performs schema validation (including codec-supports-motor)
    /// before opening any socket. A bus with `fd = true` opens in CAN-FD mode.
    pub fn from_config<P: AsRef<Path>>(path: P, registry: &CodecRegistry) -> Result<Self, Error> {
        let text = std::fs::read_to_string(path.as_ref())?;
        Self::from_config_str(&text, registry)
    }

    /// Parse a TOML config string. Same semantics as
    /// [`Robot::from_config`] but skips the file read.
    pub fn from_config_str(toml_text: &str, registry: &CodecRegistry) -> Result<Self, Error> {
        let cfg: RobotConfig = toml::from_str(toml_text)
            .map_err(|e| Error::ConfigSchema(format!("parse error: {e}")))?;
        // A bus's `fd = true` opens that bus in CAN-FD mode (see SocketCanBus::open).
        // 1) Detect vendor-on-group with a helpful error.
        for g in &cfg.group {
            if let Some(_v) = &g.vendor {
                return Err(Error::ConfigSchema(format!(
                    "group '{}': vendor belongs on [bus.<name>], not on [[group]]",
                    g.name
                )));
            }
        }
        // 2) Build buses.
        let mut builder = RobotBuilder::new();
        // Insertion order in TOML's HashMap is non-deterministic, so we sort
        // by name for stable behavior.
        let mut bus_entries: Vec<_> = cfg.bus.into_iter().collect();
        bus_entries.sort_by(|a, b| a.0.cmp(&b.0));
        for (bus_name, bus_cfg) in bus_entries {
            let transport: Box<dyn CanBus> = match bus_cfg.kind.as_str() {
                "socketcan" => {
                    let iface = bus_cfg.interface.ok_or_else(|| {
                        Error::ConfigSchema(format!(
                            "bus '{bus_name}': socketcan kind requires 'interface' field"
                        ))
                    })?;
                    Box::new(SocketCanBus::open(&iface, bus_cfg.fd)?)
                }
                other => {
                    return Err(Error::ConfigSchema(format!(
                        "bus '{bus_name}': unsupported kind '{other}' (v1 supports only 'socketcan')"
                    )));
                }
            };
            let codec = registry
                .produce(&bus_cfg.vendor)
                .ok_or_else(|| Error::UnknownVendor(bus_cfg.vendor.clone()))?;
            builder = builder.add_bus(bus_name, transport, codec);
        }
        // 3) Build groups; resolve motor type strings via the bus's vendor.
        // We need the bus's vendor for type resolution — look it up by walking
        // the original cfg.bus, which we no longer have. Workaround: rebuild
        // a name → vendor map from the second copy below. To keep things
        // simple we re-parse to a local lookup.
        let cfg2: RobotConfig = toml::from_str(toml_text)
            .map_err(|e| Error::ConfigSchema(format!("re-parse error: {e}")))?;
        let bus_vendor: HashMap<String, String> = cfg2
            .bus
            .iter()
            .map(|(n, b)| (n.clone(), b.vendor.clone()))
            .collect();
        for g in cfg2.group {
            let vendor = bus_vendor
                .get(&g.bus)
                .ok_or_else(|| Error::UnknownBusName(g.bus.clone()))?;
            let resolve = |m: &MotorConfig| -> Result<MotorSpec, Error> {
                let mt = registry.parse_type(vendor, &m.type_str).ok_or_else(|| {
                    Error::ConfigSchema(format!(
                        "group '{}': vendor '{}' does not recognize motor type '{}'",
                        g.name, vendor, m.type_str
                    ))
                })?;
                Ok(MotorSpec::new(m.name.clone(), mt, m.send_id, m.recv_id))
            };
            match g.kind.as_str() {
                "arm" => {
                    let motors: Result<Vec<_>, _> = g.motors.iter().map(resolve).collect();
                    builder = builder.add_arm(g.name, g.bus, motors?);
                }
                "gripper" => {
                    let motor_cfg = g.motor.ok_or_else(|| {
                        Error::ConfigSchema(format!(
                            "group '{}': kind='gripper' requires 'motor' (singular), not 'motors'",
                            g.name
                        ))
                    })?;
                    let m = resolve(&motor_cfg)?;
                    builder = builder.add_gripper(g.name, g.bus, m);
                }
                "generic" => {
                    let motors: Result<Vec<_>, _> = g.motors.iter().map(resolve).collect();
                    builder = builder.add_generic(g.name, g.bus, motors?);
                }
                other => {
                    return Err(Error::ConfigSchema(format!(
                        "group '{}': unknown kind '{other}' (expected arm|gripper|generic)",
                        g.name
                    )))
                }
            }
        }
        builder.build()
    }
}
