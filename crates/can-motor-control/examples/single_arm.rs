//! End-to-end Rust example: open a single Damiao arm on SocketCAN, run a
//! 10-second 1 kHz MIT control loop holding zero position, disable.
//!
//! Usage:
//!
//! ```bash
//! # one-time vcan0 setup (or use a real CAN interface):
//! sudo ip link add dev vcan0 type vcan
//! sudo ip link set vcan0 up
//!
//! cargo run --example single_arm -- vcan0
//!
//! # CAN-FD bus (interface must be FD-capable, e.g. `ip link set <iface> mtu 72`):
//! cargo run --example single_arm -- canfd0 --fd
//! ```

use std::env;
use std::time::{Duration, Instant};

use can_motor_control::{
    CanBus, MitCmd, MotorCodec, MotorSpec, MotorTypeId, RobotBuilder, SocketCanBus,
};
use damiao_codec::{parse_motor_type, DamiaoCodec, DamiaoMotorType};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let _ = parse_motor_type; // silence import for the curious reader.

    // Args: an optional positional interface name and an optional `--fd` flag.
    let mut interface = "vcan0".to_string();
    let mut fd = false;
    for arg in env::args().skip(1) {
        if arg == "--fd" {
            fd = true;
        } else {
            interface = arg;
        }
    }
    println!("opening SocketCAN interface: {interface} (fd={fd})");

    let transport: Box<dyn CanBus> = Box::new(SocketCanBus::open(&interface, fd)?);
    let codec: Box<dyn MotorCodec> = Box::new(DamiaoCodec::new());

    let mut robot = RobotBuilder::new()
        .add_bus("main", transport, codec)
        .add_arm(
            "arm",
            "main",
            vec![
                MotorSpec::new("j0", MotorTypeId::from(DamiaoMotorType::DM4340), 0x01, 0x11),
                MotorSpec::new("j1", MotorTypeId::from(DamiaoMotorType::DM4340), 0x02, 0x12),
                MotorSpec::new("j2", MotorTypeId::from(DamiaoMotorType::DM4340), 0x03, 0x13),
                MotorSpec::new("j3", MotorTypeId::from(DamiaoMotorType::DM4340), 0x04, 0x14),
            ],
        )
        .build()?;

    println!("connecting...");
    robot.connect()?;
    println!("enabling...");
    robot.enable()?;

    let deadline_per_tick = Duration::from_micros(500);
    let loop_period = Duration::from_millis(1);
    let start = Instant::now();

    println!("running 10s control loop (1 kHz) holding zero position via MIT...");
    while start.elapsed() < Duration::from_secs(10) {
        let tick_start = Instant::now();
        robot.tick(deadline_per_tick)?;
        let arm = robot
            .group_mut("arm")
            .and_then(|g| g.as_arm_mut())
            .expect("arm group");
        let n = arm.len();
        let cmds: Vec<MitCmd> = (0..n)
            .map(|_| MitCmd {
                kp: 50.0,
                kd: 1.0,
                q: 0.0,
                dq: 0.0,
                tau: 0.0,
            })
            .collect();
        arm.mit_control(&cmds)?;
        if let Some(rem) = loop_period.checked_sub(tick_start.elapsed()) {
            std::thread::sleep(rem);
        }
    }

    println!("disabling...");
    robot.disable()?;
    println!("done.");
    Ok(())
}
