//! Tier 1 Rust bring-up: enable/disable one Damiao motor, with no motion commands.
//!
//! Build a one-motor robot from explicit CAN IDs, connect, enable, tick briefly
//! so state replies can be parsed, then disable. This example sends only the
//! lifecycle enable/disable frames and read-oriented ticks.
//!
//! Run only after `00_can_interface_check.rs` confirms the interface and
//! `candump <iface>` shows a quiet bus.
//!
//! Usage:
//!
//! ```bash
//! cargo run -p can-motor-control --example 01_single_motor_enable_disable -- \
//!   --interface can0 --send-id 0x01 --recv-id 0x11 --motor-type DM4340
//! ```

use std::env;
use std::io::{self, Write};
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use can_motor_control::{CanBus, MotorCodec, MotorSpec, RobotBuilder, SocketCanBus};
use damiao_codec::{parse_motor_type, DamiaoCodec};

const DEFAULT_INTERFACE: &str = "can0";
const DEFAULT_SEND_ID: u32 = 0x01;
const DEFAULT_RECV_ID: u32 = 0x11;
const DEFAULT_MOTOR_TYPE: &str = "DM4340";
const DEFAULT_SECONDS: f64 = 1.0;
const DEFAULT_DEADLINE_US: u64 = 500;

struct Args {
    interface: String,
    fd: bool,
    send_id: u32,
    recv_id: u32,
    motor_type: String,
    seconds: f64,
    deadline_us: u64,
}

fn print_help() {
    println!(
        "Tier 1 Rust bring-up: enable/disable one motor, with no motion commands.\n\
\n\
Usage:\n\
  cargo run -p can-motor-control --example 01_single_motor_enable_disable -- [options]\n\
\n\
Options:\n\
  --interface <iface>    SocketCAN interface name (default: can0)\n\
  --fd                   Open the bus in CAN-FD mode (interface must be FD-capable)\n\
  --send-id <id>         CAN id host->motor, hex or decimal (default: 0x01)\n\
  --recv-id <id>         CAN id motor->host, hex or decimal (default: 0x11)\n\
  --motor-type <sku>     Damiao motor model, e.g. DM4310, DM4340 (default: DM4340)\n\
  --seconds <seconds>    Bounded tick duration, >0 and <=10 (default: 1.0)\n\
  --deadline-us <us>     Per tick bus deadline in microseconds (default: 500)\n\
  -h, --help            Print this help text"
    );
}

fn parse_can_id(text: &str) -> Result<u32, String> {
    let trimmed = text.trim();
    if let Some(hex) = trimmed
        .strip_prefix("0x")
        .or_else(|| trimmed.strip_prefix("0X"))
    {
        u32::from_str_radix(hex, 16).map_err(|_| format!("invalid CAN id: {text}"))
    } else {
        trimmed
            .parse::<u32>()
            .map_err(|_| format!("invalid CAN id: {text}"))
    }
}

fn parse_args() -> Result<Option<Args>, String> {
    let mut parsed = Args {
        interface: DEFAULT_INTERFACE.to_string(),
        fd: false,
        send_id: DEFAULT_SEND_ID,
        recv_id: DEFAULT_RECV_ID,
        motor_type: DEFAULT_MOTOR_TYPE.to_string(),
        seconds: DEFAULT_SECONDS,
        deadline_us: DEFAULT_DEADLINE_US,
    };
    let mut args = env::args().skip(1);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => return Ok(None),
            "--fd" => parsed.fd = true,
            "--interface" => {
                parsed.interface = args
                    .next()
                    .ok_or_else(|| "--interface requires a value".to_string())?;
            }
            "--send-id" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--send-id requires a value".to_string())?;
                parsed.send_id = parse_can_id(&value)?;
            }
            "--recv-id" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--recv-id requires a value".to_string())?;
                parsed.recv_id = parse_can_id(&value)?;
            }
            "--motor-type" => {
                parsed.motor_type = args
                    .next()
                    .ok_or_else(|| "--motor-type requires a value".to_string())?;
            }
            "--seconds" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--seconds requires a value".to_string())?;
                parsed.seconds = value
                    .parse::<f64>()
                    .map_err(|_| format!("invalid --seconds value: {value}"))?;
            }
            "--deadline-us" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--deadline-us requires a value".to_string())?;
                parsed.deadline_us = value
                    .parse::<u64>()
                    .map_err(|_| format!("invalid --deadline-us value: {value}"))?;
            }
            _ => return Err(format!("unknown argument: {arg}")),
        }
    }

    if parsed.seconds <= 0.0 {
        return Err(format!("--seconds must be > 0, got {}", parsed.seconds));
    }
    if parsed.seconds > 10.0 {
        return Err(format!(
            "--seconds={} exceeds bring-up cap of 10s",
            parsed.seconds
        ));
    }

    Ok(Some(parsed))
}

fn print_wire_format(fd: bool, interface: &str) {
    if fd {
        println!("  This example opens the bus in CAN-FD mode (fd=true).");
        println!("  The interface must be CAN-FD-capable (e.g. `ip link set <iface> mtu 72`).");
    } else {
        println!("  This example uses classical CAN (pass --fd for a CAN-FD bus).");
    }
    println!(
        "  Confirm the bus is up and `candump {interface}` shows traffic before powering motors."
    );
}

fn print_assumptions(args: &Args) {
    println!("=== Rust 01 single motor enable / disable ===");
    println!("  - interface         : {}", args.interface);
    println!(
        "  - send_id / recv_id : 0x{:02X} / 0x{:02X}",
        args.send_id, args.recv_id
    );
    println!("  - motor_type        : {}", args.motor_type);
    println!(
        "  - duration          : {:.2} s of ticks after enable",
        args.seconds
    );
    println!("  - sends             : enable + state-read ticks + disable");
    println!("  - motion commands   : NONE (no MIT/PosVel/Vel/PosForce frames)");
    println!("  - safety            : keep clear of the actuator; powered hardware");
    println!();
    print_wire_format(args.fd, &args.interface);
    println!("================================================");
    let _ = io::stdout().flush();
}

fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let motor_type = parse_motor_type(&args.motor_type)
        .ok_or_else(|| format!("unknown --motor-type {}", args.motor_type))?;

    print_assumptions(&args);

    let transport: Box<dyn CanBus> = Box::new(SocketCanBus::open(&args.interface, args.fd)?);
    let codec: Box<dyn MotorCodec> = Box::new(DamiaoCodec::new());
    let mut robot = RobotBuilder::new()
        .add_bus("main", transport, codec)
        .add_arm(
            "arm",
            "main",
            vec![MotorSpec::new("j0", motor_type, args.send_id, args.recv_id)],
        )
        .build()?;

    println!("connecting...");
    robot.connect()?;

    println!("enabling...");
    robot.enable()?;

    let result = (|| -> Result<usize, Box<dyn std::error::Error>> {
        let deadline = Instant::now() + Duration::from_secs_f64(args.seconds);
        let tick_deadline = Duration::from_micros(args.deadline_us);
        let mut ticks = 0;

        while Instant::now() < deadline {
            robot.tick(tick_deadline)?;
            ticks += 1;
            thread::sleep(Duration::from_millis(1));
        }

        Ok(ticks)
    })();

    println!("disabling...");
    let disable_result = robot.disable();

    let ticks = result?;
    disable_result?;

    let motor = robot
        .group("arm")
        .and_then(|group| group.as_arm())
        .and_then(|arm| arm.motor("j0"))
        .ok_or("missing j0 after run")?;
    println!("ticked {ticks} times");
    println!(
        "motor j0: enabled={} pos={:.4} vel={:.4} tau={:.4} t_mos={} t_rotor={} fault={:?}",
        motor.is_enabled(),
        motor.position(),
        motor.velocity(),
        motor.torque(),
        motor.temperature_mos(),
        motor.temperature_rotor(),
        motor.fault()
    );
    println!("done.");

    Ok(())
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(Some(args)) => args,
        Ok(None) => {
            print_help();
            return ExitCode::SUCCESS;
        }
        Err(message) => {
            eprintln!("ERROR: {message}");
            eprintln!("Try --help for usage.");
            return ExitCode::from(2);
        }
    };

    match run(args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("ERROR: {error}");
            ExitCode::from(1)
        }
    }
}
