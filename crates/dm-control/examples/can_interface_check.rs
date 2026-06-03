//! Tier 0 Rust bring-up: verify a SocketCAN interface without sending frames.
//!
//! This example never opens a CAN socket and never constructs a Robot. It only
//! checks `/sys/class/net/<interface>` and prints the assumptions needed before
//! moving to hardware-affecting examples.
//!
//! Usage:
//!
//! ```bash
//! cargo run -p dm-control --example can_interface_check -- --interface can0
//! ```

use std::env;
use std::io::{self, Write};
use std::path::Path;
use std::process::ExitCode;

const DEFAULT_INTERFACE: &str = "can0";

fn print_help() {
    println!(
        "Tier 0 Rust bring-up: verify a SocketCAN interface without sending frames.\n\
\n\
Usage:\n\
  cargo run -p dm-control --example can_interface_check -- [--interface <iface>]\n\
\n\
Options:\n\
  --interface <iface>  SocketCAN interface name (default: can0)\n\
  -h, --help          Print this help text"
    );
}

fn parse_args() -> Result<Option<String>, String> {
    let mut interface = DEFAULT_INTERFACE.to_string();
    let mut args = env::args().skip(1);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => return Ok(None),
            "--interface" => {
                interface = args
                    .next()
                    .ok_or_else(|| "--interface requires a value".to_string())?;
            }
            _ => return Err(format!("unknown argument: {arg}")),
        }
    }

    Ok(Some(interface))
}

fn print_assumptions(interface: &str) {
    println!("=== Rust 00 CAN interface check ===");
    println!("  - interface         : {interface}");
    println!("  - sends CAN frames? : no -- this example never opens the bus");
    println!("  - hardware required : none");
    println!();
    println!("  dm_control v1 assumes classical CAN (no CAN-FD).");
    println!("  Confirm the bus is up at the expected bitrate (commonly 1 Mbit/s)");
    println!("  and that `candump {interface}` shows traffic before powering motors.");
    println!("===================================");
    let _ = io::stdout().flush();
}

fn main() -> ExitCode {
    let interface = match parse_args() {
        Ok(Some(interface)) => interface,
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

    print_assumptions(&interface);

    let sysfs_path = format!("/sys/class/net/{interface}");
    if !Path::new(&sysfs_path).exists() {
        eprintln!("\nERROR: {sysfs_path} does not exist.");
        eprintln!("  * For a real interface, plug in the adapter and check `dmesg`.");
        eprintln!("  * For a virtual interface, set it up with:");
        eprintln!("      sudo ip link add dev {interface} type vcan");
        eprintln!("      sudo ip link set {interface} up");
        eprintln!("  * Verify with: ip link show");
        return ExitCode::from(2);
    }

    println!("\nOK: {sysfs_path} is present.");
    println!("Next manual checks (run in another terminal):");
    println!("  ip link show {interface}");
    println!("  candump {interface}");
    println!("\nWhen `candump` is quiet and the interface is `UP`, proceed to:");
    println!("  cargo run -p dm-control --example single_motor_enable_disable -- --interface {interface}");

    ExitCode::SUCCESS
}
