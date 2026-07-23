//! In-process mock transport for tests.

use std::collections::VecDeque;
use std::os::fd::RawFd;
use std::sync::{Arc, Mutex};

use motor_codec::{BusCapabilities, CanFrame};

use super::{CanBus, TransportError};

/// Record of method calls a [`MockCanBus`] has seen — used by tests to assert
/// that group code calls only `send` and never `drain_inbound_nonblocking`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MockRecordedCall {
    /// `send(frame)` was invoked.
    Send,
    /// `drain_inbound_nonblocking()` was invoked.
    Drain,
}

#[derive(Default)]
struct Shared {
    sent: Vec<CanFrame>,
    pending_inbound: VecDeque<CanFrame>,
    recorded: Vec<MockRecordedCall>,
}

/// Single-process mock CAN bus.
///
/// Construct via [`MockCanBus::new`] for a self-loopback (frames you send come
/// straight back from `drain_inbound_nonblocking`), or via
/// [`MockCanBus::pair`] for an A↔B loopback (frames sent on A appear on B's
/// inbound and vice-versa).
#[derive(Clone)]
pub struct MockCanBus {
    name: String,
    caps: BusCapabilities,
    me: Arc<Mutex<Shared>>,
    peer: Option<Arc<Mutex<Shared>>>,
}

impl MockCanBus {
    /// Classical-capability mock with self-loopback.
    pub fn new(name: impl Into<String>) -> Self {
        Self::with_capabilities(name, BusCapabilities::classical())
    }

    /// FD-capability mock with self-loopback — accepts and loops back FD frames
    /// so the FD send/receive path is testable without an FD-capable interface.
    pub fn new_fd(name: impl Into<String>) -> Self {
        Self::with_capabilities(name, BusCapabilities::fd())
    }

    /// Mock with explicit capabilities (tests that need FD-flagged behavior).
    pub fn with_capabilities(name: impl Into<String>, caps: BusCapabilities) -> Self {
        Self {
            name: name.into(),
            caps,
            me: Arc::new(Mutex::new(Shared::default())),
            peer: None,
        }
    }

    /// Pair of mocks that loopback to each other (`a.send(f)` → `b.drain` sees `f`).
    pub fn pair(name_a: impl Into<String>, name_b: impl Into<String>) -> (Self, Self) {
        let a_shared = Arc::new(Mutex::new(Shared::default()));
        let b_shared = Arc::new(Mutex::new(Shared::default()));
        let caps = BusCapabilities::classical();
        let a = Self {
            name: name_a.into(),
            caps,
            me: a_shared.clone(),
            peer: Some(b_shared.clone()),
        };
        let b = Self {
            name: name_b.into(),
            caps,
            me: b_shared,
            peer: Some(a_shared),
        };
        (a, b)
    }

    /// Snapshot of every frame this bus has transmitted via `send`.
    pub fn sent_frames(&self) -> Vec<CanFrame> {
        self.me.lock().unwrap().sent.clone()
    }

    /// Snapshot of the call sequence for assertion of "sends never read".
    pub fn recorded_calls(&self) -> Vec<MockRecordedCall> {
        self.me.lock().unwrap().recorded.clone()
    }

    /// Push a frame onto this bus's inbound queue (as if a peer wrote it).
    pub fn inject_frame(&self, frame: CanFrame) {
        self.me.lock().unwrap().pending_inbound.push_back(frame);
    }
}

impl CanBus for MockCanBus {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> BusCapabilities {
        self.caps
    }

    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        // Capability validation (mirrors what SocketCanBus does).
        if frame.is_fd() && !self.caps.supports_fd {
            return Err(TransportError::FdFrameOnNonFdBus);
        }
        if frame.len > self.caps.max_payload_len {
            return Err(TransportError::PayloadExceedsBusCapacity {
                len: frame.len,
                max: self.caps.max_payload_len,
            });
        }
        {
            let mut me = self.me.lock().unwrap();
            me.sent.push(*frame);
            me.recorded.push(MockRecordedCall::Send);
        }
        // Deliver to the peer (or to self if unpaired — self-loopback).
        match &self.peer {
            Some(p) => p.lock().unwrap().pending_inbound.push_back(*frame),
            None => self.me.lock().unwrap().pending_inbound.push_back(*frame),
        }
        Ok(())
    }

    fn drain_inbound_nonblocking(&mut self) -> Result<Vec<CanFrame>, TransportError> {
        let mut me = self.me.lock().unwrap();
        me.recorded.push(MockRecordedCall::Drain);
        Ok(me.pending_inbound.drain(..).collect())
    }

    fn raw_fd(&self) -> Option<RawFd> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pair_loopback_delivers() {
        let (mut a, mut b) = MockCanBus::pair("a", "b");
        let f = CanFrame::classical(0x101, &[1, 2, 3]).unwrap();
        a.send(&f).unwrap();
        let got = b.drain_inbound_nonblocking().unwrap();
        assert_eq!(got, vec![f]);
        // Sender's own drain is empty (no echo to self in pair mode).
        assert!(a.drain_inbound_nonblocking().unwrap().is_empty());
    }

    #[test]
    fn self_loopback_delivers() {
        let mut bus = MockCanBus::new("solo");
        let f = CanFrame::classical(0x101, &[0xFF; 8]).unwrap();
        bus.send(&f).unwrap();
        let got = bus.drain_inbound_nonblocking().unwrap();
        assert_eq!(got, vec![f]);
    }

    #[test]
    fn inject_and_drain() {
        let bus = MockCanBus::new("solo");
        let f = CanFrame::classical(0x10, &[1; 8]).unwrap();
        bus.inject_frame(f);
        let mut bus = bus;
        assert_eq!(bus.drain_inbound_nonblocking().unwrap(), vec![f]);
    }

    #[test]
    fn sent_frames_records_in_order() {
        let mut bus = MockCanBus::new("solo");
        let f1 = CanFrame::classical(0x10, &[1; 8]).unwrap();
        let f2 = CanFrame::classical(0x11, &[2; 8]).unwrap();
        bus.send(&f1).unwrap();
        bus.send(&f2).unwrap();
        assert_eq!(bus.sent_frames(), vec![f1, f2]);
    }

    #[test]
    fn fd_frame_rejected_on_classical_mock() {
        let mut bus = MockCanBus::new("solo");
        let f = CanFrame::fd(0x100, &[0; 16]).unwrap();
        assert!(matches!(
            bus.send(&f),
            Err(TransportError::FdFrameOnNonFdBus)
        ));
    }

    #[test]
    fn fd_mock_accepts_fd_frames() {
        let mut bus = MockCanBus::with_capabilities("fd", BusCapabilities::fd());
        let f = CanFrame::fd(0x100, &[0; 16]).unwrap();
        bus.send(&f).unwrap();
        assert_eq!(bus.drain_inbound_nonblocking().unwrap(), vec![f]);
    }

    #[test]
    fn new_fd_round_trips_fd_frame_preserving_flag_and_payload() {
        let mut bus = MockCanBus::new_fd("fd0");
        assert!(bus.capabilities().supports_fd);
        let f = CanFrame::fd(0x123, &[0xAB; 24]).unwrap();
        bus.send(&f).unwrap();
        let got = bus.drain_inbound_nonblocking().unwrap();
        assert_eq!(got.len(), 1);
        assert!(got[0].is_fd());
        assert_eq!(got[0].payload(), &[0xAB; 24]);
    }

    #[test]
    fn recorded_calls_track_send_vs_drain() {
        let mut bus = MockCanBus::new("solo");
        let _ = bus.send(&CanFrame::classical(0x10, &[]).unwrap());
        let _ = bus.drain_inbound_nonblocking();
        let _ = bus.send(&CanFrame::classical(0x11, &[]).unwrap());
        assert_eq!(
            bus.recorded_calls(),
            vec![
                MockRecordedCall::Send,
                MockRecordedCall::Drain,
                MockRecordedCall::Send
            ]
        );
    }

    #[test]
    fn raw_fd_is_none() {
        let bus = MockCanBus::new("solo");
        assert!(bus.raw_fd().is_none());
    }
}
