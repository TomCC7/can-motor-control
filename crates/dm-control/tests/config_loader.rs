//! Integration tests for the TOML config loader.

use dm_codec::{parse_motor_type, DamiaoCodec, VENDOR_NAME};
use dm_control::{CodecRegistry, Error, Robot};

fn registry() -> CodecRegistry {
    let mut r = CodecRegistry::new();
    r.register(
        VENDOR_NAME,
        || Box::new(DamiaoCodec::new()),
        parse_motor_type,
    );
    r
}

fn vcan_available(name: &str) -> bool {
    std::path::Path::new(&format!("/sys/class/net/{name}")).exists()
}

#[test]
fn minimal_arm_config_parses_against_vcan0() {
    if !vcan_available("vcan0") {
        eprintln!("skipping: vcan0 not present");
        return;
    }
    let toml = r#"
[bus.main]
kind = "socketcan"
interface = "vcan0"
fd = false
vendor = "damiao"

[[group]]
name = "arm"
kind = "arm"
bus = "main"
motors = [
  { name = "j0", type = "DM4340", send_id = 0x01, recv_id = 0x11 },
  { name = "j1", type = "DM4340", send_id = 0x02, recv_id = 0x12 },
]
"#;
    let robot = Robot::from_config_str(toml, &registry()).expect("parse");
    assert_eq!(robot.bus_names().collect::<Vec<_>>(), vec!["main"]);
    assert_eq!(robot.group_names().collect::<Vec<_>>(), vec!["arm"]);
}

#[test]
fn fd_true_opens_fd_bus() {
    // fd = true is now a valid request to open the bus in CAN-FD mode (it is no
    // longer rejected at parse time). It opens a real socket, so guard on vcan0.
    if !vcan_available("vcan0") {
        eprintln!("skipping: vcan0 not present");
        return;
    }
    let toml = r#"
[bus.main]
kind = "socketcan"
interface = "vcan0"
fd = true
vendor = "damiao"

[[group]]
name = "arm"
kind = "arm"
bus = "main"
motors = []
"#;
    let robot = Robot::from_config_str(toml, &registry()).expect("fd=true should build");
    assert_eq!(robot.bus_names().collect::<Vec<_>>(), vec!["main"]);
}

#[test]
fn fd_false_accepted() {
    if !vcan_available("vcan0") {
        return;
    }
    let toml = r#"
[bus.main]
kind = "socketcan"
interface = "vcan0"
fd = false
vendor = "damiao"
"#;
    let robot = Robot::from_config_str(toml, &registry()).expect("parse");
    assert!(!robot.is_connected());
}

#[test]
fn fd_key_absent_accepted_as_false() {
    if !vcan_available("vcan0") {
        return;
    }
    let toml = r#"
[bus.main]
kind = "socketcan"
interface = "vcan0"
vendor = "damiao"
"#;
    Robot::from_config_str(toml, &registry()).expect("parse");
}

#[test]
fn unknown_top_level_key_rejected() {
    let toml = r#"
[bus.main]
kind = "socketcan"
interface = "vcan0"
vendor = "damiao"
enable_speed = 5

[[group]]
name = "arm"
kind = "arm"
bus = "main"
motors = []
"#;
    let r = Robot::from_config_str(toml, &registry());
    assert!(
        matches!(r, Err(Error::ConfigSchema(ref s)) if s.contains("enable_speed") || s.contains("unknown field"))
    );
}

#[test]
fn vendor_on_group_rejected_with_helpful_message() {
    let toml = r#"
[bus.main]
kind = "socketcan"
interface = "vcan0"
vendor = "damiao"

[[group]]
name = "arm"
kind = "arm"
bus = "main"
vendor = "damiao"
motors = []
"#;
    let r = Robot::from_config_str(toml, &registry());
    match r {
        Err(Error::ConfigSchema(msg)) => {
            assert!(
                msg.contains("vendor belongs on [bus."),
                "missing helpful hint: {msg}"
            );
        }
        Ok(_) => panic!("expected ConfigSchema, got Ok"),
        Err(e) => panic!("expected ConfigSchema, got Err({e})"),
    }
}

#[test]
fn unknown_vendor_rejected() {
    if !vcan_available("vcan0") {
        return;
    }
    let toml = r#"
[bus.main]
kind = "socketcan"
interface = "vcan0"
vendor = "ghost"
"#;
    let r = Robot::from_config_str(toml, &registry());
    assert!(matches!(r, Err(Error::UnknownVendor(ref s)) if s == "ghost"));
}

#[test]
fn unknown_bus_reference_rejected() {
    let toml = r#"
[[group]]
name = "arm"
kind = "arm"
bus = "ghost"
motors = []
"#;
    let r = Robot::from_config_str(toml, &registry());
    assert!(matches!(r, Err(Error::UnknownBusName(ref s)) if s == "ghost"));
}

#[test]
fn openarm_single_toml_loads() {
    // The shipped config uses interface="can0". Pick whichever CAN-like
    // interface is actually available on this host (if any).
    let iface = if vcan_available("vcan0") {
        "vcan0"
    } else if vcan_available("can0") {
        "can0"
    } else {
        eprintln!("skipping: no can0/vcan0");
        return;
    };
    let text = std::fs::read_to_string("../../configs/openarm_single.toml").unwrap();
    let text = text.replace("interface = \"can0\"", &format!("interface = \"{iface}\""));
    let robot = match Robot::from_config_str(&text, &registry()) {
        Ok(r) => r,
        Err(Error::Transport(_)) => {
            // Real CAN hardware may exist but be DOWN or otherwise unopenable
            // in the test environment. The schema parse already succeeded
            // (that's the test's intent); skip the lifecycle assertion.
            eprintln!("skipping post-parse assertion: CAN interface present but unopenable");
            return;
        }
        Err(e) => panic!("parse openarm: {e}"),
    };
    assert_eq!(robot.bus_names().collect::<Vec<_>>(), vec!["main"]);
    assert_eq!(robot.group_names().collect::<Vec<_>>(), vec!["arm"]);
}
