//! Linux SocketCAN transport.
//!
//! Built directly on libc syscalls: `socket(PF_CAN, SOCK_RAW, CAN_RAW)`,
//! `ioctl(SIOCGIFINDEX)`, `bind`, non-blocking via `fcntl`. Avoids the
//! `socketcan` crate so we own the FD-vs-classical decode path without
//! upstream coupling.

use std::ffi::CString;
use std::io;
use std::mem::{size_of, zeroed};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};

use motor_codec::{BusCapabilities, CanFrame, FrameFlags};

use super::{CanBus, TransportError};

// SocketCAN constants. The libc crate doesn't expose these uniformly across
// versions, so we define them ourselves matching <linux/can.h>.
const AF_CAN: libc::c_int = 29;
const PF_CAN: libc::c_int = AF_CAN;
const CAN_RAW: libc::c_int = 1;
const SIOCGIFINDEX: libc::c_ulong = 0x8933;

// setsockopt level/option for enabling CAN-FD reception on a raw CAN socket.
// `SOL_CAN_RAW` = `SOL_CAN_BASE (100)` + `CAN_RAW (1)`; `CAN_RAW_FD_FRAMES` = 5.
// Matches <linux/can/raw.h>. With this set, the socket may both read and write
// `canfd_frame`s in addition to classical `can_frame`s.
const SOL_CAN_BASE: libc::c_int = 100;
const SOL_CAN_RAW: libc::c_int = SOL_CAN_BASE + CAN_RAW;
const CAN_RAW_FD_FRAMES: libc::c_int = 5;

const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
const CAN_SFF_MASK: u32 = 0x0000_07FF;

// Kernel struct layouts. Sized for x86_64 Linux; the alignment of u32 + u8s
// matches the kernel's `struct can_frame` / `struct canfd_frame`.

#[repr(C)]
#[derive(Copy, Clone, Default)]
struct KernelCanFrame {
    can_id: u32,
    can_dlc: u8,
    __pad: u8,
    __res0: u8,
    len8_dlc: u8,
    data: [u8; 8],
}

#[repr(C)]
#[derive(Copy, Clone)]
struct KernelCanFdFrame {
    can_id: u32,
    len: u8,
    flags: u8,
    __res0: u8,
    __res1: u8,
    data: [u8; 64],
}

const KERNEL_CLASSICAL_LEN: usize = size_of::<KernelCanFrame>(); // 16
const KERNEL_FD_LEN: usize = size_of::<KernelCanFdFrame>(); // 72

#[repr(C)]
struct SockaddrCan {
    can_family: u16,
    _pad: u16,
    can_ifindex: i32,
    rx_id: u32,
    tx_id: u32,
}

#[repr(C)]
struct Ifreq {
    name: [u8; libc::IFNAMSIZ],
    index_or_other: [u8; 24],
}

/// Linux SocketCAN bus.
pub struct SocketCanBus {
    name: String,
    fd: OwnedFd,
    caps: BusCapabilities,
}

impl SocketCanBus {
    /// Open a SocketCAN interface.
    ///
    /// With `fd_enabled = true` the socket is configured for CAN-FD
    /// (`CAN_RAW_FD_FRAMES`) and the bus advertises `BusCapabilities::fd()`; the
    /// interface itself must be FD-capable. With `fd_enabled = false` the bus is
    /// classical-only (no FD socket option, 8-byte payload cap).
    pub fn open(interface: &str, fd_enabled: bool) -> Result<Self, TransportError> {
        if interface.len() >= libc::IFNAMSIZ {
            return Err(TransportError::InterfaceNotFound(interface.to_string()));
        }

        // Resolve interface index BEFORE creating the socket so we can return
        // InterfaceNotFound without leaking an open fd. We use a probe socket
        // for the ioctl (it doesn't need to be CAN-specific to do SIOCGIFINDEX).
        let ifindex = resolve_ifindex(interface)?;

        // SAFETY: socket() with valid family/type/protocol; returns -1 on error.
        let raw_fd = unsafe { libc::socket(PF_CAN, libc::SOCK_RAW, CAN_RAW) };
        if raw_fd < 0 {
            return Err(map_open_error(io::Error::last_os_error()));
        }
        // SAFETY: raw_fd is a kernel-owned fd; OwnedFd takes exclusive ownership.
        let owned = unsafe { OwnedFd::from_raw_fd(raw_fd) };

        // Enable CAN-FD frames on the socket before bind when requested. Without
        // this option the kernel delivers and accepts only classical frames.
        if fd_enabled {
            let enable: libc::c_int = 1;
            // SAFETY: setsockopt with a valid level/option and an int-sized value.
            let rc = unsafe {
                libc::setsockopt(
                    raw_fd,
                    SOL_CAN_RAW,
                    CAN_RAW_FD_FRAMES,
                    &enable as *const libc::c_int as *const libc::c_void,
                    size_of::<libc::c_int>() as libc::socklen_t,
                )
            };
            if rc < 0 {
                return Err(map_open_error(io::Error::last_os_error()));
            }
        }

        // bind(sockaddr_can)
        let addr = SockaddrCan {
            can_family: AF_CAN as u16,
            _pad: 0,
            can_ifindex: ifindex,
            rx_id: 0,
            tx_id: 0,
        };
        // SAFETY: addr lives until bind returns; cast to *const sockaddr is the
        // canonical SocketCAN bind pattern.
        let rc = unsafe {
            libc::bind(
                raw_fd,
                &addr as *const SockaddrCan as *const libc::sockaddr,
                size_of::<SockaddrCan>() as libc::socklen_t,
            )
        };
        if rc < 0 {
            return Err(map_open_error(io::Error::last_os_error()));
        }

        // Set non-blocking.
        // SAFETY: F_GETFL returns the current flag value; F_SETFL applies the OR.
        unsafe {
            let flags = libc::fcntl(raw_fd, libc::F_GETFL);
            if flags < 0 {
                return Err(TransportError::Io(io::Error::last_os_error()));
            }
            if libc::fcntl(raw_fd, libc::F_SETFL, flags | libc::O_NONBLOCK) < 0 {
                return Err(TransportError::Io(io::Error::last_os_error()));
            }
        }

        Ok(Self {
            name: interface.to_string(),
            fd: owned,
            caps: if fd_enabled {
                BusCapabilities::fd()
            } else {
                BusCapabilities::classical()
            },
        })
    }

    /// Write one already-serialized kernel frame (`expected` bytes), retrying
    /// on `EAGAIN`/`EINTR` within the send budget. Shared by the classical and
    /// FD send paths.
    fn write_frame_bytes(&self, bytes: &[u8], expected: usize) -> Result<(), TransportError> {
        let raw = self.fd.as_raw_fd();
        let mut retries = 0u32;
        loop {
            // SAFETY: write() to an owned fd with a valid byte slice.
            let n = unsafe { libc::write(raw, bytes.as_ptr() as *const libc::c_void, expected) };
            if n == expected as isize {
                return Ok(());
            }
            if n < 0 {
                let err = io::Error::last_os_error();
                match err.raw_os_error() {
                    Some(libc::EAGAIN) => {
                        retries += 1;
                        if retries >= SEND_RETRY_BUDGET {
                            return Err(TransportError::SendBufferFull);
                        }
                        continue;
                    }
                    Some(libc::EINTR) => continue,
                    _ => return Err(TransportError::Io(err)),
                }
            }
            // Short write — shouldn't happen for CAN, treat as IO error.
            return Err(TransportError::Io(io::Error::other(format!(
                "short write: {n} bytes"
            ))));
        }
    }
}

fn resolve_ifindex(interface: &str) -> Result<i32, TransportError> {
    // Open a temporary AF_INET/SOCK_DGRAM socket for the ioctl.
    // SAFETY: socket() with valid args.
    let probe_fd = unsafe { libc::socket(libc::AF_INET, libc::SOCK_DGRAM, 0) };
    if probe_fd < 0 {
        return Err(TransportError::Io(io::Error::last_os_error()));
    }
    // SAFETY: take ownership so it's auto-closed.
    let _probe_owned = unsafe { OwnedFd::from_raw_fd(probe_fd) };

    let cname = CString::new(interface)
        .map_err(|_| TransportError::InterfaceNotFound(interface.to_string()))?;
    let cname_bytes = cname.as_bytes_with_nul();
    if cname_bytes.len() > libc::IFNAMSIZ {
        return Err(TransportError::InterfaceNotFound(interface.to_string()));
    }

    // SAFETY: zeroed Ifreq is valid for all fields.
    let mut req: Ifreq = unsafe { zeroed() };
    req.name[..cname_bytes.len()].copy_from_slice(cname_bytes);

    // SAFETY: ioctl with SIOCGIFINDEX expects a writable ifreq.
    let rc = unsafe { libc::ioctl(probe_fd, SIOCGIFINDEX, &mut req as *mut Ifreq) };
    if rc < 0 {
        let err = io::Error::last_os_error();
        if err.raw_os_error() == Some(libc::ENODEV) {
            return Err(TransportError::InterfaceNotFound(interface.to_string()));
        }
        return Err(map_open_error(err));
    }
    // index_or_other[0..4] is now a little-endian i32 on Linux.
    let mut bytes = [0u8; 4];
    bytes.copy_from_slice(&req.index_or_other[0..4]);
    Ok(i32::from_ne_bytes(bytes))
}

fn map_open_error(e: io::Error) -> TransportError {
    match e.raw_os_error() {
        Some(libc::EACCES) | Some(libc::EPERM) => TransportError::PermissionDenied,
        Some(libc::ENODEV) | Some(libc::ENXIO) => {
            TransportError::InterfaceNotFound(format!("os errno {}", libc::ENODEV))
        }
        _ => TransportError::Io(e),
    }
}

fn validate_send(caps: &BusCapabilities, frame: &CanFrame) -> Result<(), TransportError> {
    if frame.is_fd() && !caps.supports_fd {
        return Err(TransportError::FdFrameOnNonFdBus);
    }
    if frame.len > caps.max_payload_len {
        return Err(TransportError::PayloadExceedsBusCapacity {
            len: frame.len,
            max: caps.max_payload_len,
        });
    }
    Ok(())
}

const SEND_RETRY_BUDGET: u32 = 8;

impl CanBus for SocketCanBus {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> BusCapabilities {
        self.caps
    }

    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        validate_send(&self.caps, frame)?;
        // Format is chosen per-frame: an FD frame goes out as a `canfd_frame`,
        // a classical frame as a `can_frame` — even on an FD-capable socket,
        // which can carry both. `validate_send` has already rejected an FD frame
        // on a classical bus, so reaching the FD arm implies `caps.supports_fd`.
        if frame.is_fd() {
            let mut kf = KernelCanFdFrame {
                can_id: encode_can_id(frame),
                len: frame.len,
                flags: fd_flags_byte(frame),
                __res0: 0,
                __res1: 0,
                data: [0u8; 64],
            };
            kf.data[..frame.len as usize].copy_from_slice(frame.payload());
            // SAFETY: viewing a #[repr(C)] struct as its byte representation.
            let bytes = unsafe {
                std::slice::from_raw_parts(
                    &kf as *const KernelCanFdFrame as *const u8,
                    KERNEL_FD_LEN,
                )
            };
            self.write_frame_bytes(bytes, KERNEL_FD_LEN)
        } else {
            let mut kf = KernelCanFrame {
                can_id: encode_can_id(frame),
                can_dlc: frame.len,
                ..Default::default()
            };
            kf.data[..frame.len as usize].copy_from_slice(frame.payload());
            // SAFETY: viewing a #[repr(C)] struct as its byte representation.
            let bytes = unsafe {
                std::slice::from_raw_parts(
                    &kf as *const KernelCanFrame as *const u8,
                    KERNEL_CLASSICAL_LEN,
                )
            };
            self.write_frame_bytes(bytes, KERNEL_CLASSICAL_LEN)
        }
    }

    fn drain_inbound_nonblocking(&mut self) -> Result<Vec<CanFrame>, TransportError> {
        let mut out = Vec::new();
        let raw = self.fd.as_raw_fd();
        // Buffer sized for FD frames. A classical-only socket reads only
        // classical frames; an FD socket (opened with fd=true) may read either,
        // discriminated below by the kernel read length.
        let mut buf = [0u8; KERNEL_FD_LEN];
        loop {
            // SAFETY: read() into an owned buffer; the returned count is the
            // exact bytes filled.
            let n =
                unsafe { libc::read(raw, buf.as_mut_ptr() as *mut libc::c_void, KERNEL_FD_LEN) };
            if n < 0 {
                let err = io::Error::last_os_error();
                match err.raw_os_error() {
                    Some(libc::EAGAIN) => return Ok(out),
                    Some(libc::EINTR) => continue,
                    _ => return Err(TransportError::Io(err)),
                }
            }
            match n as usize {
                KERNEL_CLASSICAL_LEN => out.push(decode_kernel_classical(&buf)),
                KERNEL_FD_LEN => out.push(decode_kernel_fd(&buf)),
                0 => return Ok(out),
                other => {
                    return Err(TransportError::Io(io::Error::other(format!(
                        "unexpected SocketCAN read size: {other}"
                    ))));
                }
            }
        }
    }

    fn raw_fd(&self) -> Option<RawFd> {
        Some(self.fd.as_raw_fd())
    }
}

/// Pack a `CanFrame`'s FD-only flags into the kernel `canfd_frame.flags` byte
/// (bit 0 = BRS, bit 1 = ESI), matching `decode_kernel_fd`'s reader.
fn fd_flags_byte(frame: &CanFrame) -> u8 {
    let mut f = 0u8;
    if frame.flags.contains(FrameFlags::BIT_RATE_SWITCH) {
        f |= 0x01;
    }
    if frame.flags.contains(FrameFlags::ERROR_STATE) {
        f |= 0x02;
    }
    f
}

fn encode_can_id(frame: &CanFrame) -> u32 {
    let mut id = frame.id
        & if frame.is_extended() {
            CAN_EFF_MASK
        } else {
            CAN_SFF_MASK
        };
    if frame.is_extended() {
        id |= CAN_EFF_FLAG;
    }
    if frame.flags.contains(FrameFlags::REMOTE_REQUEST) {
        id |= CAN_RTR_FLAG;
    }
    id
}

pub(crate) fn decode_kernel_classical(buf: &[u8; KERNEL_FD_LEN]) -> CanFrame {
    let kf = unsafe { &*(buf.as_ptr() as *const KernelCanFrame) };
    let extended = kf.can_id & CAN_EFF_FLAG != 0;
    let rtr = kf.can_id & CAN_RTR_FLAG != 0;
    let id = kf.can_id & if extended { CAN_EFF_MASK } else { CAN_SFF_MASK };
    let len = kf.can_dlc.min(8);
    let mut flags = FrameFlags::empty();
    if extended {
        flags |= FrameFlags::EXTENDED_ID;
    }
    if rtr {
        flags |= FrameFlags::REMOTE_REQUEST;
    }
    let payload = &kf.data[..len as usize];
    let mut frame = if extended {
        CanFrame::classical_extended(id, payload)
    } else {
        CanFrame::classical(id, payload)
    }
    .expect("kernel-bounded payload");
    frame.flags = flags;
    frame
}

pub(crate) fn decode_kernel_fd(buf: &[u8; KERNEL_FD_LEN]) -> CanFrame {
    let kf = unsafe { &*(buf.as_ptr() as *const KernelCanFdFrame) };
    let extended = kf.can_id & CAN_EFF_FLAG != 0;
    let id = kf.can_id & if extended { CAN_EFF_MASK } else { CAN_SFF_MASK };
    let len = kf.len.min(64);
    let mut flags = FrameFlags::FD_FORMAT;
    if extended {
        flags |= FrameFlags::EXTENDED_ID;
    }
    if kf.flags & 0x01 != 0 {
        flags |= FrameFlags::BIT_RATE_SWITCH;
    }
    if kf.flags & 0x02 != 0 {
        flags |= FrameFlags::ERROR_STATE;
    }
    let payload = &kf.data[..len as usize];
    let mut frame = if extended {
        CanFrame::fd_extended(id, payload)
    } else {
        CanFrame::fd(id, payload)
    }
    .expect("kernel-bounded FD payload");
    frame.flags = flags;
    frame
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vcan_available(name: &str) -> bool {
        std::path::Path::new(&format!("/sys/class/net/{name}")).exists()
    }

    /// The kernel `canfd_frame` layout we mirror is 72 bytes on Linux; the
    /// receive path discriminates classical vs FD purely by read length, so a
    /// layout drift would silently mis-parse frames. Assert it loudly instead.
    #[test]
    fn kernel_fd_frame_layout_is_72_bytes() {
        assert_eq!(size_of::<KernelCanFdFrame>(), 72);
        assert_eq!(size_of::<KernelCanFrame>(), 16);
        assert_eq!(KERNEL_FD_LEN, 72);
        assert_eq!(KERNEL_CLASSICAL_LEN, 16);
    }

    #[test]
    fn nonexistent_interface_returns_interface_not_found() {
        let r = SocketCanBus::open("ghost-iface-x", false);
        assert!(matches!(r, Err(TransportError::InterfaceNotFound(_))));
    }

    #[test]
    fn vcan0_open_and_capabilities() {
        if !vcan_available("vcan0") {
            eprintln!("skipping: vcan0 not present");
            return;
        }
        let bus = SocketCanBus::open("vcan0", false).expect("open vcan0");
        assert_eq!(bus.name(), "vcan0");
        let caps = bus.capabilities();
        assert!(!caps.supports_fd);
        assert_eq!(caps.max_payload_len, 8);
        assert!(bus.raw_fd().is_some());
    }

    #[test]
    fn vcan0_send_round_trip() {
        if !vcan_available("vcan0") {
            eprintln!("skipping: vcan0 not present");
            return;
        }
        let mut tx = SocketCanBus::open("vcan0", false).expect("open vcan0 tx");
        let mut rx = SocketCanBus::open("vcan0", false).expect("open vcan0 rx");
        let f = CanFrame::classical(0x101, &[1, 2, 3, 4]).unwrap();
        tx.send(&f).unwrap();
        // Best-effort: a small busy-wait so the kernel can route the frame.
        let mut got = Vec::new();
        for _ in 0..100 {
            got = rx.drain_inbound_nonblocking().unwrap();
            if !got.is_empty() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].id, 0x101);
        assert_eq!(got[0].payload(), &[1, 2, 3, 4]);
    }

    #[test]
    fn fd_frame_rejected_on_classical_socketcanbus() {
        if !vcan_available("vcan0") {
            eprintln!("skipping: vcan0 not present");
            return;
        }
        let mut bus = SocketCanBus::open("vcan0", false).unwrap();
        let f = CanFrame::fd(0x100, &[0; 16]).unwrap();
        assert!(matches!(
            bus.send(&f),
            Err(TransportError::FdFrameOnNonFdBus)
        ));
    }

    #[test]
    fn synthetic_fd_frame_decodes_with_fd_format_flag() {
        // Direct unit test of the FD parsing path (v1 SocketCanBus never
        // receives real FD frames because we don't enable CAN_RAW_FD_FRAMES,
        // but the decode path must be correct so v2 unlocks without re-touch).
        let mut buf = [0u8; KERNEL_FD_LEN];
        // Build a canfd_frame: id=0x101, len=16, BRS flag set, data[0..16] = 0xAA
        let kf = KernelCanFdFrame {
            can_id: 0x101,
            len: 16,
            flags: 0x01, // BRS
            __res0: 0,
            __res1: 0,
            data: {
                let mut d = [0u8; 64];
                d[..16].fill(0xAA);
                d
            },
        };
        // SAFETY: writing a #[repr(C)] struct into a byte buffer of equal size.
        unsafe {
            std::ptr::copy_nonoverlapping(
                &kf as *const KernelCanFdFrame as *const u8,
                buf.as_mut_ptr(),
                KERNEL_FD_LEN,
            );
        }
        let frame = decode_kernel_fd(&buf);
        assert!(frame.is_fd());
        assert!(frame.flags.contains(FrameFlags::BIT_RATE_SWITCH));
        assert_eq!(frame.id, 0x101);
        assert_eq!(frame.len, 16);
        assert_eq!(frame.payload(), &[0xAA; 16]);
    }

    #[test]
    fn fd_open_advertises_fd_capabilities() {
        // Requires an FD-capable interface (e.g. `ip link add vcanfd0 type
        // vcan; ip link set vcanfd0 mtu 72`). Skipped when absent.
        if !vcan_available("vcanfd0") {
            eprintln!("skipping: vcanfd0 not present");
            return;
        }
        let bus = SocketCanBus::open("vcanfd0", true).expect("open vcanfd0 fd");
        let caps = bus.capabilities();
        assert!(caps.supports_fd);
        assert_eq!(caps.max_payload_len, 64);
    }

    #[test]
    fn fd_round_trip_over_fd_socketcan() {
        if !vcan_available("vcanfd0") {
            eprintln!("skipping: vcanfd0 not present");
            return;
        }
        let mut tx = SocketCanBus::open("vcanfd0", true).expect("open tx");
        let mut rx = SocketCanBus::open("vcanfd0", true).expect("open rx");
        let f = CanFrame::fd(0x123, &[0xAB; 16]).unwrap();
        tx.send(&f).unwrap();
        let mut got = Vec::new();
        for _ in 0..100 {
            got = rx.drain_inbound_nonblocking().unwrap();
            if !got.is_empty() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        assert_eq!(got.len(), 1);
        assert!(got[0].is_fd());
        assert_eq!(got[0].id, 0x123);
        assert_eq!(got[0].payload(), &[0xAB; 16]);
    }
}
