//! Tier 3 Rust bring-up: one-motor gripper control using MIT or PosVel.
//!
//! Run only after the gripper motor passes `single_motor_enable_disable.rs` and
//! a bounded single-motor hold. This example uses only the current Gripper API:
//! enable, disable, MIT control, and PosVel control.
//!
//! Usage:
//!
//! ```bash
//! cargo run -p can-motor-control --example gripper_control -- \
//!   --interface can0 --send-id 0x08 --recv-id 0x18 --motor-type DM4310
//! ```

use std::env;
use std::io::{self, Write};
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use can_motor_control::{
    CanBus, MitCmd, MotorCodec, MotorSpec, PosVelCmd, RobotBuilder, SocketCanBus,
};
use damiao_codec::{parse_motor_type, DamiaoCodec};

const DEFAULT_INTERFACE: &str = "can0";
const DEFAULT_SEND_ID: u32 = 0x08;
const DEFAULT_RECV_ID: u32 = 0x18;
const DEFAULT_MOTOR_TYPE: &str = "DM4310";
const DEFAULT_SECONDS: f64 = 2.0;
const DEFAULT_DEADLINE_US: u64 = 500;

#[derive(Copy, Clone, Eq, PartialEq)]
enum Mode {
    Mit,
    PosVel,
}

struct Args {
    mode: Mode,
    interface: String,
    fd: bool,
    send_id: u32,
    recv_id: u32,
    motor_type: String,
    seconds: f64,
    deadline_us: u64,
    kp: f64,
    kd: f64,
    q: f64,
    dq: f64,
    tau: f64,
}

fn print_help() {
    println!(
        "Tier 3 Rust bring-up: one-motor gripper control using MIT or PosVel.\n\
\n\
Usage:\n\
  cargo run -p can-motor-control --example gripper_control -- [options]\n\
\n\
Options:\n\
  --mode <mit|pos-vel>   Gripper control mode (default: mit)\n\
  --interface <iface>    SocketCAN interface name (default: can0)\n\
  --fd                   Open the bus in CAN-FD mode (interface must be FD-capable)\n\
  --send-id <id>         CAN id host->motor, hex or decimal (default: 0x08)\n\
  --recv-id <id>         CAN id motor->host, hex or decimal (default: 0x18)\n\
  --motor-type <sku>     Damiao motor model, e.g. DM4310, DM4340 (default: DM4310)\n\
  --seconds <seconds>    Bounded run duration, >0 and <=10 (default: 2.0)\n\
  --deadline-us <us>     Per tick bus deadline in microseconds (default: 500)\n\
  --kp <gain>            MIT position gain (default: 2.0)\n\
  --kd <gain>            MIT velocity gain (default: 0.5)\n\
  --q <rad>              Target position radians (default: 0.0)\n\
  --dq <rad/s>           Target velocity rad/s (default: 0.0)\n\
  --tau <nm>             MIT feed-forward torque Nm (default: 0.0)\n\
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
        mode: Mode::Mit,
        interface: DEFAULT_INTERFACE.to_string(),
        fd: false,
        send_id: DEFAULT_SEND_ID,
        recv_id: DEFAULT_RECV_ID,
        motor_type: DEFAULT_MOTOR_TYPE.to_string(),
        seconds: DEFAULT_SECONDS,
        deadline_us: DEFAULT_DEADLINE_US,
        kp: 2.0,
        kd: 0.5,
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
    };
    let mut args = env::args().skip(1);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => return Ok(None),
            "--mode" => {
                let mode = next_value(&mut args, "--mode")?;
                parsed.mode = match mode.as_str() {
                    "mit" => Mode::Mit,
                    "pos-vel" => Mode::PosVel,
                    _ => return Err(format!("unknown --mode {mode}; valid: mit, pos-vel")),
                };
            }
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
            "--kp" => parsed.kp = parse_f64(&mut args, "--kp")?,
            "--kd" => parsed.kd = parse_f64(&mut args, "--kd")?,
            "--q" => parsed.q = parse_f64(&mut args, "--q")?,
            "--dq" => parsed.dq = parse_f64(&mut args, "--dq")?,
            "--tau" => parsed.tau = parse_f64(&mut args, "--tau")?,
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
    if parsed.kp < 0.0 || parsed.kd < 0.0 {
        return Err("--kp and --kd must be non-negative".to_string());
    }

    Ok(Some(parsed))
}

fn mode_name(mode: Mode) -> &'static str {
    match mode {
        Mode::Mit => "mit",
        Mode::PosVel => "pos-vel",
    }
}

fn print_assumptions(args: &Args) {
    println!("=== Rust 07 gripper control ({}) ===", mode_name(args.mode));
    println!("  - interface         : {}", args.interface);
    println!(
        "  - send_id / recv_id : 0x{:02X} / 0x{:02X}",
        args.send_id, args.recv_id
    );
    println!("  - motor_type        : {}", args.motor_type);
    println!(
        "  - duration          : {:.2} s (hard cap 10 s)",
        args.seconds
    );
    match args.mode {
        Mode::Mit => println!(
            "  - command           : mit kp={} kd={} q={} dq={} tau={}",
            args.kp, args.kd, args.q, args.dq, args.tau
        ),
        Mode::PosVel => println!(
            "  - command           : pos-vel q={} dq={}",
            args.q, args.dq
        ),
    }
    println!("  - safety            : verify nothing fragile is between the gripper jaws");
    println!();
    print_wire_format(args.fd, &args.interface);
    println!("=====================================");
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

fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let motor_type = parse_motor_type(&args.motor_type)
        .ok_or_else(|| format!("unknown --motor-type {}", args.motor_type))?;
    print_assumptions(&args);

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
        let mit = MitCmd {
            kp: args.kp,
            kd: args.kd,
            q: args.q,
            dq: args.dq,
            tau: args.tau,
        };
        let pos_vel = PosVelCmd {
            q: args.q,
            dq: args.dq,
        };
        let deadline = Instant::now() + Duration::from_secs_f64(args.seconds);
        let tick_deadline = Duration::from_micros(args.deadline_us);
        let period = Duration::from_millis(1);
        let mut ticks = 0;

        while Instant::now() < deadline {
            let tick_start = Instant::now();
            robot.tick(tick_deadline)?;
            let gripper = robot
                .group_mut("grip")
                .and_then(|group| group.as_gripper_mut())
                .ok_or("missing gripper group")?;
            match args.mode {
                Mode::Mit => gripper.mit_control(mit)?,
                Mode::PosVel => gripper.pos_vel_control(pos_vel)?,
            }
            ticks += 1;
            if let Some(remaining) = period.checked_sub(tick_start.elapsed()) {
                thread::sleep(remaining);
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
