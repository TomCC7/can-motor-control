//! USB-independent queue, statistics, echo-credit, and terminal-state core.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use motor_codec::CanFrame;

use super::protocol::{decode_host_frame, ProtocolError, RX_ECHO_ID};

pub(crate) const RX_CAPACITY: usize = 8_192;
pub(crate) const TX_CAPACITY: usize = 256;
pub(crate) const ECHO_CREDITS: usize = 32;

#[derive(Default)]
struct Counters {
    rx_received: AtomicU64,
    rx_dropped: AtomicU64,
    tx_accepted: AtomicU64,
    tx_completed: AtomicU64,
}

/// Shared read-only gs_usb runtime counters.
///
/// Clones remain valid after a Python or Rust transport handle is consumed.
#[derive(Clone, Default)]
pub struct GsUsbStatistics {
    inner: Arc<Counters>,
}

impl GsUsbStatistics {
    /// Number of classical CAN frames accepted from USB into the RX FIFO.
    pub fn rx_received(&self) -> u64 {
        self.inner.rx_received.load(Ordering::Relaxed)
    }
    /// Number of oldest RX frames evicted when the FIFO was full.
    pub fn rx_dropped(&self) -> u64 {
        self.inner.rx_dropped.load(Ordering::Relaxed)
    }
    /// Number of frames accepted by the bounded caller-facing TX queue.
    pub fn tx_accepted(&self) -> u64 {
        self.inner.tx_accepted.load(Ordering::Relaxed)
    }
    /// Number of successful bulk-OUT completions.
    pub fn tx_completed(&self) -> u64 {
        self.inner.tx_completed.load(Ordering::Relaxed)
    }

    pub(crate) fn accepted(&self) {
        self.inner.tx_accepted.fetch_add(1, Ordering::Relaxed);
    }
    pub(crate) fn completed(&self) {
        self.inner.tx_completed.fetch_add(1, Ordering::Relaxed);
    }
}

#[derive(Default)]
pub(crate) struct RxQueue {
    frames: Mutex<VecDeque<CanFrame>>,
    statistics: GsUsbStatistics,
}

impl RxQueue {
    pub(crate) fn new(statistics: GsUsbStatistics) -> Self {
        Self {
            frames: Mutex::new(VecDeque::with_capacity(RX_CAPACITY)),
            statistics,
        }
    }

    pub(crate) fn push(&self, frame: CanFrame) {
        let mut frames = self.frames.lock().expect("gs_usb RX queue poisoned");
        if frames.len() == RX_CAPACITY {
            frames.pop_front();
            self.statistics
                .inner
                .rx_dropped
                .fetch_add(1, Ordering::Relaxed);
        }
        frames.push_back(frame);
        self.statistics
            .inner
            .rx_received
            .fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn drain(&self) -> Vec<CanFrame> {
        self.frames
            .lock()
            .expect("gs_usb RX queue poisoned")
            .drain(..)
            .collect()
    }
}

pub(crate) struct EchoCredits {
    allocated: [bool; ECHO_CREDITS],
    cursor: usize,
}

impl Default for EchoCredits {
    fn default() -> Self {
        Self {
            allocated: [false; ECHO_CREDITS],
            cursor: 0,
        }
    }
}

impl EchoCredits {
    pub(crate) fn allocate(&mut self) -> Option<u32> {
        for offset in 0..ECHO_CREDITS {
            let index = (self.cursor + offset) % ECHO_CREDITS;
            if !self.allocated[index] {
                self.allocated[index] = true;
                self.cursor = (index + 1) % ECHO_CREDITS;
                return Some(index as u32);
            }
        }
        None
    }

    pub(crate) fn release(&mut self, echo_id: u32) -> Result<(), ProtocolError> {
        let index = usize::try_from(echo_id).unwrap_or(usize::MAX);
        if index >= ECHO_CREDITS || !self.allocated[index] {
            return Err(ProtocolError::InvalidEchoId(echo_id));
        }
        self.allocated[index] = false;
        Ok(())
    }

    pub(crate) fn in_use(&self) -> usize {
        self.allocated.iter().filter(|used| **used).count()
    }
}

pub(crate) fn process_input(
    bytes: &[u8],
    echoes: &mut EchoCredits,
    rx: &RxQueue,
) -> Result<(), String> {
    let (echo_id, frame) = decode_host_frame(bytes).map_err(|error| error.to_string())?;
    if echo_id == RX_ECHO_ID {
        rx.push(frame);
        Ok(())
    } else {
        echoes.release(echo_id).map_err(|error| error.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::gs_usb::protocol::encode_host_frame;

    #[test]
    fn fifo_evicts_oldest_and_counts_each_drop() {
        let stats = GsUsbStatistics::default();
        let rx = RxQueue::new(stats.clone());
        for id in 0..RX_CAPACITY + 2 {
            rx.push(CanFrame::classical((id % 0x7ff) as u32, &[]).unwrap());
        }
        let drained = rx.drain();
        assert_eq!(drained.len(), RX_CAPACITY);
        assert_eq!(drained[0].id, 2);
        assert_eq!(
            drained.last().unwrap().id,
            ((RX_CAPACITY + 1) % 0x7ff) as u32
        );
        assert_eq!(stats.rx_received(), (RX_CAPACITY + 2) as u64);
        assert_eq!(stats.rx_dropped(), 2);
    }

    #[test]
    fn echo_credits_are_finite_and_echoes_never_enter_rx() {
        let mut echoes = EchoCredits::default();
        let ids: Vec<u32> = (0..ECHO_CREDITS)
            .map(|_| echoes.allocate().unwrap())
            .collect();
        assert!(echoes.allocate().is_none());
        let stats = GsUsbStatistics::default();
        let rx = RxQueue::new(stats);
        let echo = encode_host_frame(ids[3], &CanFrame::classical(1, &[1]).unwrap()).unwrap();
        process_input(&echo, &mut echoes, &rx).unwrap();
        assert!(rx.drain().is_empty());
        assert_eq!(echoes.in_use(), ECHO_CREDITS - 1);
        assert!(process_input(&echo, &mut echoes, &rx)
            .unwrap_err()
            .contains("duplicate"));
        let unknown = encode_host_frame(999, &CanFrame::classical(1, &[]).unwrap()).unwrap();
        assert!(process_input(&unknown, &mut echoes, &rx).is_err());
    }

    #[test]
    fn received_frame_enters_fifo_in_arrival_order() {
        let stats = GsUsbStatistics::default();
        let rx = RxQueue::new(stats.clone());
        let mut echoes = EchoCredits::default();
        for id in [5, 2, 9] {
            let bytes =
                encode_host_frame(RX_ECHO_ID, &CanFrame::classical(id, &[id as u8]).unwrap())
                    .unwrap();
            process_input(&bytes, &mut echoes, &rx).unwrap();
        }
        assert_eq!(
            rx.drain().iter().map(|frame| frame.id).collect::<Vec<_>>(),
            [5, 2, 9]
        );
        assert_eq!(stats.rx_received(), 3);
    }

    #[test]
    fn sustained_interleaved_rx_tx_and_echo_traffic_preserves_credits() {
        let stats = GsUsbStatistics::default();
        let rx = RxQueue::new(stats.clone());
        let mut echoes = EchoCredits::default();
        let in_flight_transfers = 4;
        for sequence in 0..4_000_u32 {
            let echo_id = echoes.allocate().expect("echo credit");
            let tx_echo = encode_host_frame(
                echo_id,
                &CanFrame::classical(sequence % 0x7ff, &[1]).unwrap(),
            )
            .unwrap();
            process_input(&tx_echo, &mut echoes, &rx).unwrap();

            let inbound = encode_host_frame(
                RX_ECHO_ID,
                &CanFrame::classical(sequence % 0x7ff, &[2]).unwrap(),
            )
            .unwrap();
            process_input(&inbound, &mut echoes, &rx).unwrap();
            assert_eq!(
                in_flight_transfers, 4,
                "completed IN buffer must be resubmitted"
            );
        }
        assert_eq!(echoes.in_use(), 0);
        assert_eq!(rx.drain().len(), 4_000);
        assert_eq!(stats.rx_dropped(), 0);
    }
}
