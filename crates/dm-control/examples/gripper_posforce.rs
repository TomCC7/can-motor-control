//! Tier 3 Rust bring-up: one-motor gripper PosForce control.
//!
//! This follows openarm_can's gripper PosForce pattern: `π/2 → 0 → π/2 → 0`
//! with speed limit `25.0 rad/s` and current/torque per-unit `0.15`.
//!
//! Usage:
//!
//! ```bash
//! cargo run -p dm-control --example gripper_posforce -- \
//!   --interface can0 --send-id 0x08 --recv-id 0x18 --motor-type DM4310
//! ```

use std::env;
use std::f64::consts::FRAC_PI_2;
use std::io::{self, Write};
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use dm_codec::{parse_motor_type, DamiaoCodec};
use dm_control::{
    CanBus, CanFrame, MotorCodec, MotorSpec, PosForceCmd, RobotBuilder, SocketCanBus,
};

const DEFAULT_INTERFACE: &str = "can0";
const DEFAULT_SEND_ID: u32 = 0x08;
const DEFAULT_RECV_ID: u32 = 0x18;
const DEFAULT_MOTOR_TYPE: &str = "DM4310";
const DEFAULT_SECONDS: f64 = 2.0;
const DEFAULT_DEADLINE_US: u64 = 500;
const DAMIAO_PARAM_ID: u32 = 0x7FF;
const DAMIAO_WRITE_PARAM: u8 = 0x55;
const DAMIAO_CTRL_MODE_RID: u8 = 10;
const DAMIAO_CTRL_MODE_POS_FORCE: u32 = 4;

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
        "Tier 3 Rust bring-up: one-motor gripper PosForce control.\n\
\n\
Usage:\n\
  cargo run -p dm-control --example gripper_posforce -- [options]\n\
\n\
Openarm-style sequence:\n\
  q:    π/2, 0, π/2, 0\n\
  dq:   25.0 rad/s\n\
  i-pu: 0.15\n\
\n\
Options:\n\
  --interface <iface>    SocketCAN interface name (default: can0)\n\
  --fd                   Open the bus in CAN-FD mode (interface must be FD-capable)\n\
  --send-id <id>         CAN id host->motor, hex or decimal (default: 0x08)\n\
  --recv-id <id>         CAN id motor->host, hex or decimal (default: 0x18)\n\
  --motor-type <sku>     Damiao motor model, e.g. DM4310, DM4340 (default: DM4310)\n\
  --seconds <seconds>    Per-position duration, >0 and <=10 (default: 2.0)\n\
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

fn next_value(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    args.next()
        .ok_or_else(|| format!("{flag} requires a value"))
}

fn parse_f64(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<f64, String> {
    let value = next_value(args, flag)?;
    value
        .parse::<f64>()
        .map_err(|_| format!("invalid {flag} value: {value}"))
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
            "--interface" => parsed.interface = next_value(&mut args, "--interface")?,
            "--send-id" => parsed.send_id = parse_can_id(&next_value(&mut args, "--send-id")?)?,
            "--recv-id" => parsed.recv_id = parse_can_id(&next_value(&mut args, "--recv-id")?)?,
            "--motor-type" => parsed.motor_type = next_value(&mut args, "--motor-type")?,
            "--seconds" => parsed.seconds = parse_f64(&mut args, "--seconds")?,
            "--deadline-us" => {
                let value = next_value(&mut args, "--deadline-us")?;
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

fn openarm_sequence() -> Vec<PosForceCmd> {
    [FRAC_PI_2, 0.0, FRAC_PI_2, 0.0]
        .into_iter()
        .map(|q| PosForceCmd {
            q,
            dq: 25.0,
            i_pu: 0.15,
        })
        .collect()
}

fn print_assumptions(args: &Args, commands: &[PosForceCmd]) {
    println!("=== Rust gripper PosForce openarm sequence ===");
    println!("  - interface         : {}", args.interface);
    println!(
        "  - send_id / recv_id : 0x{:02X} / 0x{:02X}",
        args.send_id, args.recv_id
    );
    println!("  - motor_type        : {}", args.motor_type);
    println!("  - setup             : write CTRL_MODE=POS_FORCE before enable");
    println!(
        "  - duration          : {:.2} s per position (hard cap 10 s)",
        args.seconds
    );
    for (index, command) in commands.iter().enumerate() {
        println!(
            "  - command[{index}]       : pos-force q={} dq={} i_pu={}",
            command.q, command.dq, command.i_pu
        );
    }
    println!("  - safety            : clear the gripper jaws before sending PosForce");
    println!();
    print_wire_format(args.fd, &args.interface);
    println!("=============================================");
    let _ = io::stdout().flush();
}

fn print_wire_format(fd: bool, interface: &str) {
    if fd {
        println!("  This example opens the bus in CAN-FD mode (fd=true).");
        println!("  The interface must be CAN-FD-capable (e.g. `ip link set <iface> mtu 72`).");
    } else {
        println!("  This example uses classical CAN (pass --fd for a CAN-FD bus).");
    }
    println!("  Confirm `candump {interface}` is running before continuing.");
}

fn set_pos_force_mode(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let mut transport = SocketCanBus::open(&args.interface, args.fd)?;
    let mut payload = [0u8; 8];
    payload[0..2].copy_from_slice(&(args.send_id as u16).to_le_bytes());
    payload[2] = DAMIAO_WRITE_PARAM;
    payload[3] = DAMIAO_CTRL_MODE_RID;
    payload[4..8].copy_from_slice(&DAMIAO_CTRL_MODE_POS_FORCE.to_le_bytes());
    let frame = CanFrame::classical(DAMIAO_PARAM_ID, &payload)?;
    println!("setting motor control mode to POS_FORCE...");
    transport.send(&frame)?;
    thread::sleep(Duration::from_millis(100));
    Ok(())
}

fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let motor_type = parse_motor_type(&args.motor_type)
        .ok_or_else(|| format!("unknown --motor-type {}", args.motor_type))?;
    let commands = openarm_sequence();
    print_assumptions(&args, &commands);
    set_pos_force_mode(&args)?;

    let transport: Box<dyn CanBus> = Box::new(SocketCanBus::open(&args.interface, args.fd)?);
    let codec: Box<dyn MotorCodec> = Box::new(DamiaoCodec::new());
    let mut robot = RobotBuilder::new()
        .add_bus("main", transport, codec)
        .add_gripper(
            "grip",
            "main",
            MotorSpec::new("g", motor_type, args.send_id, args.recv_id),
        )
        .build()?;

    println!("connecting...");
    robot.connect()?;
    println!("enabling gripper...");
    robot
        .group_mut("grip")
        .and_then(|group| group.as_gripper_mut())
        .ok_or("missing gripper group")?
        .enable()?;

    let result = (|| -> Result<usize, Box<dyn std::error::Error>> {
        let tick_deadline = Duration::from_micros(args.deadline_us);
        let period = Duration::from_millis(1);
        let mut ticks = 0;

        for command in commands {
            println!(
                "commanding q={} dq={} i_pu={}...",
                command.q, command.dq, command.i_pu
            );
            let deadline = Instant::now() + Duration::from_secs_f64(args.seconds);
            while Instant::now() < deadline {
                let tick_start = Instant::now();
                robot.tick(tick_deadline)?;
                let gripper = robot
                    .group_mut("grip")
                    .and_then(|group| group.as_gripper_mut())
                    .ok_or("missing gripper group")?;
                gripper.pos_force_control(command)?;
                ticks += 1;
                if let Some(remaining) = period.checked_sub(tick_start.elapsed()) {
                    thread::sleep(remaining);
                }
            }
        }

        Ok(ticks)
    })();

    println!("disabling gripper...");
    let gripper_disable = robot
        .group_mut("grip")
        .and_then(|group| group.as_gripper_mut())
        .ok_or("missing gripper group")?
        .disable();
    let robot_disable = robot.disable();
    let ticks = result?;
    gripper_disable?;
    robot_disable?;

    let motor = robot
        .group("grip")
        .and_then(|group| group.as_gripper())
        .map(|gripper| gripper.motor())
        .ok_or("missing gripper motor after run")?;
    println!(
        "completed {ticks} ticks; gripper pos={:+.5} vel={:+.5} tau={:+.5} t_mos={} t_rotor={}",
        motor.position(),
        motor.velocity(),
        motor.torque(),
        motor.temperature_mos(),
        motor.temperature_rotor()
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
