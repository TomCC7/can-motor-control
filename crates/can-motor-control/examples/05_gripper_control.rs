//! Tier 3 Rust bring-up: gripper PosForce sequence, with MIT and PosVel alternatives.
//!
//! Default behavior mirrors `examples/05_gripper_control.py`: set the gripper to
//! PosForce mode, then play `-π/4 -> 0 -> -π/4 -> 0`. PosForce and PosVel use
//! `dq=25.0 rad/s`; PosForce also uses `i_pu=0.15`. The same sequence is used
//! for PosForce, MIT, and PosVel; pass `--positions` or `--q` to play a custom
//! target in any mode.
//!
//! Usage:
//!
//! ```bash
//! cargo run -p can-motor-control --example 05_gripper_control -- \
//!   --interface can0 --send-id 0x08 --recv-id 0x18 --motor-type DM4310
//! ```

use std::env;
use std::f64::consts::{FRAC_PI_4, PI};
use std::io::{self, Write};
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use can_motor_control::{
    CanBus, CommandKind, MitCmd, MockCanBus, MotorCodec, MotorSpec, PosForceCmd, PosVelCmd,
    RobotBuilder, SocketCanBus,
};
use damiao_codec::{parse_motor_type, DamiaoCodec};

const DEFAULT_INTERFACE: &str = "can0";
const DEFAULT_SEND_ID: u32 = 0x08;
const DEFAULT_RECV_ID: u32 = 0x18;
const DEFAULT_MOTOR_TYPE: &str = "DM4310";
const DEFAULT_SECONDS: f64 = 2.0;
const DEFAULT_DEADLINE_US: u64 = 500;
const DEFAULT_I_PU: f64 = 0.15;
const MODE_LATCH: Duration = Duration::from_millis(100);

#[derive(Copy, Clone, Eq, PartialEq)]
enum Mode {
    PosForce,
    Mit,
    PosVel,
}

impl Mode {
    fn parse(text: &str) -> Result<Self, String> {
        match text {
            "pos-force" => Ok(Self::PosForce),
            "mit" => Ok(Self::Mit),
            "pos-vel" => Ok(Self::PosVel),
            _ => Err(format!(
                "unknown --mode {text}; valid: pos-force, mit, pos-vel"
            )),
        }
    }

    fn cli_name(self) -> &'static str {
        match self {
            Self::PosForce => "pos-force",
            Self::Mit => "mit",
            Self::PosVel => "pos-vel",
        }
    }

    fn command_kind(self) -> CommandKind {
        match self {
            Self::PosForce => CommandKind::PosForce,
            Self::Mit => CommandKind::Mit,
            Self::PosVel => CommandKind::PosVel,
        }
    }
}

struct Args {
    mode: Mode,
    mock: bool,
    interface: String,
    fd: bool,
    send_id: u32,
    recv_id: u32,
    motor_type: String,
    seconds: f64,
    deadline_us: u64,
    kp: f64,
    kd: f64,
    q: Option<f64>,
    dq: Option<f64>,
    tau: f64,
    i_pu: f64,
    positions: Option<Vec<f64>>,
}

fn print_help() {
    println!(
        "Tier 3 Rust bring-up: gripper PosForce sequence, with MIT and PosVel alternatives.\n\
\n\
Usage:\n\
  cargo run -p can-motor-control --example 05_gripper_control -- [options]\n\
\n\
Options:\n\
  --mode <pos-force|mit|pos-vel>  Gripper control mode (default: pos-force)\n\
  --mock                          Use MockCanBus (no hardware)\n\
  --interface <iface>             SocketCAN interface name (default: can0)\n\
  --fd                            Open the bus in CAN-FD mode (interface must be FD-capable)\n\
  --send-id <id>                  CAN id host->motor, hex or decimal (default: 0x08)\n\
  --recv-id <id>                  CAN id motor->host, hex or decimal (default: 0x18)\n\
  --motor-type <sku>              Damiao motor model, e.g. DM4310, DM4340 (default: DM4310)\n\
  --seconds <seconds>             Per-position duration, >0 and <=10 (default: 2.0)\n\
  --deadline-us <us>              Per tick bus deadline in microseconds (default: 500)\n\
  --kp <gain>                     MIT position gain (default: 2.0)\n\
  --kd <gain>                     MIT velocity gain (default: 0.5)\n\
  --q <rad>                       Single target position, used instead of the default sequence\n\
  --dq <rad/s>                    Target velocity (default: 25.0 for pos-force/pos-vel, 0.0 for mit)\n\
  --tau <nm>                      MIT feed-forward torque Nm (default: 0.0)\n\
  --i-pu <value>                  PosForce current per-unit (default: 0.15)\n\
  --positions <csv>               q sequence, accepts pi, pi/2, pi/4 (default: -pi/4,0,-pi/4,0)\n\
  -h, --help                     Print this help text"
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

fn parse_position_token(token: &str) -> Result<f64, String> {
    match token.trim().to_ascii_lowercase().as_str() {
        "pi" => Ok(PI),
        "-pi" => Ok(-PI),
        "pi/2" => Ok(PI / 2.0),
        "-pi/2" => Ok(-PI / 2.0),
        "pi/4" => Ok(FRAC_PI_4),
        "-pi/4" => Ok(-FRAC_PI_4),
        text => text
            .parse::<f64>()
            .map_err(|_| format!("invalid position value: {text}")),
    }
}

fn parse_positions(text: &str) -> Result<Vec<f64>, String> {
    let positions = text
        .split(',')
        .filter(|part| !part.trim().is_empty())
        .map(parse_position_token)
        .collect::<Result<Vec<_>, _>>()?;
    if positions.is_empty() {
        return Err("--positions must include at least one value".to_string());
    }
    Ok(positions)
}

fn parse_args() -> Result<Option<Args>, String> {
    let mut parsed = Args {
        mode: Mode::PosForce,
        mock: false,
        interface: DEFAULT_INTERFACE.to_string(),
        fd: false,
        send_id: DEFAULT_SEND_ID,
        recv_id: DEFAULT_RECV_ID,
        motor_type: DEFAULT_MOTOR_TYPE.to_string(),
        seconds: DEFAULT_SECONDS,
        deadline_us: DEFAULT_DEADLINE_US,
        kp: 2.0,
        kd: 0.5,
        q: None,
        dq: None,
        tau: 0.0,
        i_pu: DEFAULT_I_PU,
        positions: None,
    };
    let mut args = env::args().skip(1);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => return Ok(None),
            "--mode" => parsed.mode = Mode::parse(&next_value(&mut args, "--mode")?)?,
            "--mock" => parsed.mock = true,
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
            "--q" => parsed.q = Some(parse_f64(&mut args, "--q")?),
            "--dq" => parsed.dq = Some(parse_f64(&mut args, "--dq")?),
            "--tau" => parsed.tau = parse_f64(&mut args, "--tau")?,
            "--i-pu" => parsed.i_pu = parse_f64(&mut args, "--i-pu")?,
            "--positions" => {
                parsed.positions = Some(parse_positions(&next_value(&mut args, "--positions")?)?)
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
    if parsed.kp < 0.0 || parsed.kd < 0.0 {
        return Err("--kp and --kd must be non-negative".to_string());
    }

    Ok(Some(parsed))
}

fn default_positions() -> Vec<f64> {
    vec![-FRAC_PI_4, 0.0, -FRAC_PI_4, 0.0]
}

fn selected_positions(args: &Args) -> Vec<f64> {
    if let Some(positions) = &args.positions {
        return positions.clone();
    }
    if let Some(q) = args.q {
        return vec![q];
    }
    default_positions()
}

fn selected_dq(args: &Args) -> f64 {
    args.dq.unwrap_or(match args.mode {
        Mode::Mit => 0.0,
        Mode::PosForce | Mode::PosVel => 25.0,
    })
}

fn command_description(args: &Args, positions: &[f64], dq: f64) -> String {
    let q_values = positions
        .iter()
        .map(|q| format!("{q:.5}"))
        .collect::<Vec<_>>()
        .join(", ");
    match args.mode {
        Mode::Mit => format!(
            "mit q=[{q_values}] kp={} kd={} dq={} tau={}",
            args.kp, args.kd, dq, args.tau
        ),
        Mode::PosVel => format!("pos-vel q=[{q_values}] dq={dq}"),
        Mode::PosForce => format!("pos-force q=[{q_values}] dq={dq} i_pu={}", args.i_pu),
    }
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

fn print_assumptions(args: &Args, positions: &[f64], dq: f64) {
    println!("=== Rust 05 gripper control ({}) ===", args.mode.cli_name());
    println!("  - interface         : {}", args.interface);
    println!(
        "  - target            : {}",
        if args.mock {
            "MockCanBus"
        } else {
            &args.interface
        }
    );
    println!(
        "  - send_id / recv_id : 0x{:02X} / 0x{:02X}",
        args.send_id, args.recv_id
    );
    println!("  - motor_type        : {}", args.motor_type);
    println!(
        "  - duration          : {:.2} s per position (hard cap 10 s)",
        args.seconds
    );
    println!(
        "  - mode setup        : set_mode({}) before enable",
        args.mode.cli_name()
    );
    println!(
        "  - command           : {}",
        command_description(args, positions, dq)
    );
    println!("  - safety            : clear the gripper jaws before sending commands");
    println!();
    print_wire_format(args.fd, &args.interface);
    println!("=====================================");
    let _ = io::stdout().flush();
}

fn run_position(
    robot: &mut can_motor_control::Robot,
    mode: Mode,
    q: f64,
    dq: f64,
    args: &Args,
) -> Result<usize, Box<dyn std::error::Error>> {
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
        match mode {
            Mode::Mit => gripper.mit_control(MitCmd {
                kp: args.kp,
                kd: args.kd,
                q,
                dq,
                tau: args.tau,
            })?,
            Mode::PosVel => gripper.pos_vel_control(PosVelCmd { q, dq })?,
            Mode::PosForce => gripper.pos_force_control(PosForceCmd {
                q,
                dq,
                i_pu: args.i_pu,
            })?,
        }
        ticks += 1;
        if let Some(remaining) = period.checked_sub(tick_start.elapsed()) {
            thread::sleep(remaining);
        }
    }

    Ok(ticks)
}

fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let motor_type = parse_motor_type(&args.motor_type)
        .ok_or_else(|| format!("unknown --motor-type {}", args.motor_type))?;
    let positions = selected_positions(&args);
    let dq = selected_dq(&args);
    print_assumptions(&args, &positions, dq);

    let transport: Box<dyn CanBus> = if args.mock && args.fd {
        Box::new(MockCanBus::new_fd("vcan_mock"))
    } else if args.mock {
        Box::new(MockCanBus::new("vcan_mock"))
    } else {
        Box::new(SocketCanBus::open(&args.interface, args.fd)?)
    };
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
    println!("setting gripper mode to {}...", args.mode.cli_name());
    robot
        .group_mut("grip")
        .and_then(|group| group.as_gripper_mut())
        .ok_or("missing gripper group")?
        .set_mode(args.mode.command_kind())?;
    thread::sleep(MODE_LATCH);
    println!("enabling gripper...");
    robot
        .group_mut("grip")
        .and_then(|group| group.as_gripper_mut())
        .ok_or("missing gripper group")?
        .enable()?;

    let result = (|| -> Result<usize, Box<dyn std::error::Error>> {
        let mut ticks = 0;
        for (index, q) in positions.iter().enumerate() {
            println!("command[{index}] q={q:.5}...");
            ticks += run_position(&mut robot, args.mode, *q, dq, &args)?;
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
