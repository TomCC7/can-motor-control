//! Tier 2 Rust bring-up: conservative MIT hold on one Damiao motor.
//!
//! Run only after `single_motor_enable_disable.rs` shows sane state replies for
//! the same motor. This example sends bounded MIT control commands with soft
//! default gains and disables on exit.
//!
//! Usage:
//!
//! ```bash
//! cargo run -p can-motor-control --example single_motor_mit_hold -- \
//!   --interface can0 --send-id 0x08 --recv-id 0x18 --motor-type DM4310
//! ```

use std::env;
use std::io::{self, Write};
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use can_motor_control::{CanBus, MitCmd, MotorCodec, MotorSpec, RobotBuilder, SocketCanBus};
use damiao_codec::{parse_motor_type, DamiaoCodec};

const DEFAULT_INTERFACE: &str = "can0";
const DEFAULT_SEND_ID: u32 = 0x08;
const DEFAULT_RECV_ID: u32 = 0x18;
const DEFAULT_MOTOR_TYPE: &str = "DM4310";
const DEFAULT_SECONDS: f64 = 2.0;
const DEFAULT_DEADLINE_US: u64 = 500;

struct Args {
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
        "Tier 2 Rust bring-up: conservative MIT hold on one motor.\n\
\n\
Usage:\n\
  cargo run -p can-motor-control --example single_motor_mit_hold -- [options]\n\
\n\
Options:\n\
  --interface <iface>    SocketCAN interface name (default: can0)\n\
  --fd                   Open the bus in CAN-FD mode (interface must be FD-capable)\n\
  --send-id <id>         CAN id host->motor, hex or decimal (default: 0x08)\n\
  --recv-id <id>         CAN id motor->host, hex or decimal (default: 0x18)\n\
  --motor-type <sku>     Damiao motor model, e.g. DM4310, DM4340 (default: DM4310)\n\
  --seconds <seconds>    Bounded run duration, >0 and <=10 (default: 2.0)\n\
  --deadline-us <us>     Per tick bus deadline in microseconds (default: 500)\n\
  --kp <gain>            MIT position gain (default: 5.0)\n\
  --kd <gain>            MIT velocity gain (default: 1.0)\n\
  --q <rad>              Target position radians (default: 0.0)\n\
  --dq <rad/s>           Target velocity rad/s (default: 0.0)\n\
  --tau <nm>             Feed-forward torque Nm (default: 0.0)\n\
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
        kp: 5.0,
        kd: 1.0,
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
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

fn print_assumptions(args: &Args) {
    println!("=== Rust 03 single motor MIT hold ===");
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
    println!("  - mit gains         : kp={} kd={}", args.kp, args.kd);
    println!(
        "  - mit setpoint      : q={} dq={} tau={}",
        args.q, args.dq, args.tau
    );
    println!("  - safety            : motor will hold softly; clear workspace around it");
    println!();
    print_wire_format(args.fd, &args.interface);
    println!("=======================================");
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
        let cmd = MitCmd {
            kp: args.kp,
            kd: args.kd,
            q: args.q,
            dq: args.dq,
            tau: args.tau,
        };
        let deadline = Instant::now() + Duration::from_secs_f64(args.seconds);
        let tick_deadline = Duration::from_micros(args.deadline_us);
        let period = Duration::from_millis(1);
        let mut ticks = 0;

        while Instant::now() < deadline {
            let tick_start = Instant::now();
            robot.tick(tick_deadline)?;
            let arm = robot
                .group_mut("arm")
                .and_then(|group| group.as_arm_mut())
                .ok_or("missing arm group")?;
            arm.mit_control(&[cmd])?;
            ticks += 1;
            if let Some(remaining) = period.checked_sub(tick_start.elapsed()) {
                thread::sleep(remaining);
            }
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
    println!(
        "completed {ticks} ticks; final pos={:+.5} vel={:+.5} tau={:+.5} t_mos={} t_rotor={}",
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
