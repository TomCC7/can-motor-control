use std::error::Error;

use can_motor_control::CanBus;

#[cfg(target_os = "linux")]
pub fn open_native_transport(selector: &str, fd: bool) -> Result<Box<dyn CanBus>, Box<dyn Error>> {
    Ok(Box::new(can_motor_control::SocketCanBus::open(
        selector, fd,
    )?))
}

#[cfg(target_os = "macos")]
pub fn open_native_transport(selector: &str, fd: bool) -> Result<Box<dyn CanBus>, Box<dyn Error>> {
    if fd {
        return Err("the macOS gs_usb transport supports classical CAN only".into());
    }
    let (vendor, product) = selector
        .split_once(':')
        .ok_or("on macOS, --interface must be a gs_usb VID:PID pair such as 1d50:606f")?;
    let parse_id = |value: &str| {
        u16::from_str_radix(value.trim_start_matches("0x"), 16)
            .map_err(|_| format!("invalid hexadecimal USB id '{value}'"))
    };
    let config = can_motor_control::GsUsbConfig::new(parse_id(vendor)?, parse_id(product)?);
    Ok(Box::new(can_motor_control::GsUsbBus::open(config)?))
}
