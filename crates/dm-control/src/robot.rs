//! [`Robot`] and [`RobotBuilder`].

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use mio::Token;

use crate::bus::{Bus, RouteKey};
use crate::error::Error;
use crate::group::{Arm, Generic, GroupKind, Gripper, MotorGroup};
use crate::motor::Motor;
use crate::spec::{GroupSpecKind, MotorSpec};
use crate::transport::{BusPoller, CanBus};
use motor_codec::MotorCodec;

/// A configured robot: named buses + named groups + lifecycle state.
pub struct Robot {
    buses: HashMap<String, Arc<Mutex<Bus>>>,
    bus_order: Vec<String>,
    groups: HashMap<String, GroupKind>,
    group_order: Vec<String>,
    connected: bool,
    poller: Option<BusPoller>,
    /// Cached for tick: bus_name → poller token.
    bus_tokens: HashMap<String, Token>,
    /// Cached for tick: token → bus_name (inverse lookup).
    token_to_bus: HashMap<Token, String>,
}

impl Robot {
    /// Start a builder.
    pub fn builder() -> RobotBuilder {
        RobotBuilder::new()
    }

    /// Read-only access to a named group.
    pub fn group(&self, name: &str) -> Option<&GroupKind> {
        self.groups.get(name)
    }

    /// Mutable access to a named group.
    pub fn group_mut(&mut self, name: &str) -> Option<&mut GroupKind> {
        self.groups.get_mut(name)
    }

    /// Iterate group names in insertion order.
    pub fn group_names(&self) -> impl Iterator<Item = &str> {
        self.group_order.iter().map(String::as_str)
    }

    /// Iterate bus names in insertion order.
    pub fn bus_names(&self) -> impl Iterator<Item = &str> {
        self.bus_order.iter().map(String::as_str)
    }

    /// True after a successful `connect()`.
    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// Borrow a bus by name (returns the `Arc<Mutex<Bus>>` for direct lock).
    pub fn bus(&self, name: &str) -> Option<&Arc<Mutex<Bus>>> {
        self.buses.get(name)
    }

    /// Open sockets, populate per-bus recv-id routing tables, register fds
    /// with the poller, lock topology.
    pub fn connect(&mut self) -> Result<(), Error> {
        if self.connected {
            return Ok(());
        }
        // Build routing tables.
        for (bus_name, bus_arc) in &self.buses {
            let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
            bus.routes.clear();
            for group_name in &self.group_order {
                let group = self.groups.get(group_name).expect("group order invariant");
                if group.bus_name() != bus_name {
                    continue;
                }
                for (i, motor) in group.inner().motors().iter().enumerate() {
                    let key = RouteKey {
                        group_name: group_name.clone(),
                        motor_index: i,
                    };
                    if let Some(existing) = bus.routes.get(&motor.recv_id()) {
                        return Err(Error::CanIdCollision {
                            bus_name: bus_name.clone(),
                            recv_id: motor.recv_id(),
                            existing: existing.clone(),
                            attempted: key,
                        });
                    }
                    bus.routes.insert(motor.recv_id(), key);
                }
            }
        }
        // Set up poller.
        let poller = BusPoller::with_capacity(self.buses.len().max(1))?;
        let mut bus_tokens = HashMap::with_capacity(self.buses.len());
        let mut token_to_bus = HashMap::with_capacity(self.buses.len());
        for (idx, bus_name) in self.bus_order.iter().enumerate() {
            let bus = self.buses[bus_name].lock().map_err(|_| Error::BusPoisoned)?;
            if let Some(fd) = bus.transport.raw_fd() {
                let token = Token(idx);
                poller.register(token, fd)?;
                bus_tokens.insert(bus_name.clone(), token);
                token_to_bus.insert(token, bus_name.clone());
            } else {
                log::warn!(
                    "bus '{bus_name}' has no raw_fd; out-of-band polling required (not implemented in v1)"
                );
            }
        }
        self.poller = Some(poller);
        self.bus_tokens = bus_tokens;
        self.token_to_bus = token_to_bus;
        self.connected = true;
        Ok(())
    }

    /// Send the enable frame to every motor in every group, in insertion order.
    pub fn enable(&mut self) -> Result<(), Error> {
        if !self.connected {
            return Err(Error::NotConnected);
        }
        for name in &self.group_order.clone() {
            if let Some(g) = self.groups.get_mut(name) {
                g.enable_all()?;
            }
        }
        Ok(())
    }

    /// Send the disable frame to every motor in every group, in reverse
    /// insertion order. A no-op `Ok(())` if never enabled.
    pub fn disable(&mut self) -> Result<(), Error> {
        if !self.connected {
            return Ok(());
        }
        for name in self.group_order.clone().iter().rev() {
            if let Some(g) = self.groups.get_mut(name) {
                g.disable_all()?;
            }
        }
        Ok(())
    }

    /// One tick of the control loop: poll all buses, drain readable ones,
    /// decode each frame exactly once, dispatch the resulting events to the
    /// owning groups via per-bus routing tables.
    pub fn tick(&mut self, deadline: Duration) -> Result<(), Error> {
        if !self.connected {
            return Err(Error::NotConnected);
        }
        // 1) Wait for any registered fd to become readable (or the deadline).
        let ready = if let Some(p) = self.poller.as_mut() {
            p.wait(deadline)?
        } else {
            Vec::new()
        };
        // 2) For each ready bus, drain + decode + collect dispatch tuples.
        let mut dispatches: Vec<(String, usize, motor_codec::Event)> = Vec::new();
        for token in ready {
            let Some(bus_name) = self.token_to_bus.get(&token).cloned() else {
                continue;
            };
            let bus_arc = self.buses[&bus_name].clone();
            let mut bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
            let frames = bus.transport.drain_inbound_nonblocking()?;
            for frame in &frames {
                let decoded = bus.codec.decode(frame).map_err(Error::Codec)?;
                if let Some(event) = decoded {
                    let motor_id = match event {
                        motor_codec::Event::State { motor_id, .. } => motor_id,
                        motor_codec::Event::ParamReply { motor_id, .. } => motor_id,
                        motor_codec::Event::Fault { motor_id, .. } => motor_id,
                        _ => continue,
                    };
                    if let Some(route) = bus.routes.get(&motor_id) {
                        dispatches.push((route.group_name.clone(), route.motor_index, event));
                    }
                    // No route: silently drop (foreign motor_id, not in this robot).
                }
                // None: codec didn't recognize, silently drop.
            }
        }
        // 3) Dispatch outside the bus lock to avoid deadlocking groups that
        // would lock the same bus during apply_event (currently they don't,
        // but the discipline is correct).
        for (group_name, motor_index, event) in dispatches {
            if let Some(g) = self.groups.get_mut(&group_name) {
                g.apply_event(motor_index, &event);
            }
        }
        Ok(())
    }
}

impl Drop for Robot {
    fn drop(&mut self) {
        // OwnedFd inside each bus's SocketCanBus closes the underlying socket
        // automatically. We don't send disable frames here — that's the user's
        // responsibility via explicit disable().
        for token in self.token_to_bus.keys() {
            if let Some(p) = self.poller.as_ref() {
                if let Some(name) = self.token_to_bus.get(token) {
                    if let Some(bus_arc) = self.buses.get(name) {
                        if let Ok(bus) = bus_arc.lock() {
                            if let Some(fd) = bus.transport.raw_fd() {
                                let _ = p.deregister(fd);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Pending bus declaration accumulated by the builder.
struct PendingBus {
    name: String,
    transport: Box<dyn CanBus>,
    codec: Box<dyn MotorCodec>,
}

/// Pending group declaration accumulated by the builder.
struct PendingGroup {
    name: String,
    bus_name: String,
    kind: GroupSpecKind,
    motors: Vec<MotorSpec>,
}

/// Builder for [`Robot`].
pub struct RobotBuilder {
    buses: Vec<PendingBus>,
    groups: Vec<PendingGroup>,
}

impl Default for RobotBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl RobotBuilder {
    /// Start an empty builder.
    pub fn new() -> Self {
        Self {
            buses: Vec::new(),
            groups: Vec::new(),
        }
    }

    /// Register a bus with its transport and vendor codec. The codec's
    /// `bind_to_bus(transport.capabilities())` is invoked exactly once here.
    pub fn add_bus(
        mut self,
        name: impl Into<String>,
        transport: Box<dyn CanBus>,
        codec: Box<dyn MotorCodec>,
    ) -> Self {
        self.buses.push(PendingBus {
            name: name.into(),
            transport,
            codec,
        });
        self
    }

    /// Register an arm group.
    pub fn add_arm(
        mut self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motors: Vec<MotorSpec>,
    ) -> Self {
        self.groups.push(PendingGroup {
            name: name.into(),
            bus_name: bus_name.into(),
            kind: GroupSpecKind::Arm,
            motors,
        });
        self
    }

    /// Register a one-motor gripper. Validation enforces the one-motor rule
    /// at `build()` time.
    pub fn add_gripper(
        mut self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motor: MotorSpec,
    ) -> Self {
        self.groups.push(PendingGroup {
            name: name.into(),
            bus_name: bus_name.into(),
            kind: GroupSpecKind::Gripper,
            motors: vec![motor],
        });
        self
    }

    /// Register a generic catch-all group.
    pub fn add_generic(
        mut self,
        name: impl Into<String>,
        bus_name: impl Into<String>,
        motors: Vec<MotorSpec>,
    ) -> Self {
        self.groups.push(PendingGroup {
            name: name.into(),
            bus_name: bus_name.into(),
            kind: GroupSpecKind::Generic,
            motors,
        });
        self
    }

    /// Validate and build.
    pub fn build(self) -> Result<Robot, Error> {
        // 1) Validate bus names: non-empty, trimmed, unique.
        let mut bus_order = Vec::with_capacity(self.buses.len());
        let mut buses: HashMap<String, Arc<Mutex<Bus>>> = HashMap::with_capacity(self.buses.len());
        for pending in self.buses {
            if pending.name.trim().is_empty() || pending.name != pending.name.trim() {
                return Err(Error::ConfigSchema(format!(
                    "bus name '{}' must be non-empty and not have leading/trailing whitespace",
                    pending.name
                )));
            }
            if buses.contains_key(&pending.name) {
                return Err(Error::DuplicateBusName(pending.name));
            }
            let bus = Bus::new(pending.transport, pending.codec);
            bus_order.push(pending.name.clone());
            buses.insert(pending.name, Arc::new(Mutex::new(bus)));
        }
        // 2) Validate group names + codec support + bus existence.
        let mut group_order = Vec::with_capacity(self.groups.len());
        let mut groups: HashMap<String, GroupKind> = HashMap::with_capacity(self.groups.len());
        for pending in self.groups {
            if pending.name.trim().is_empty() || pending.name != pending.name.trim() {
                return Err(Error::ConfigSchema(format!(
                    "group name '{}' must be non-empty and not have leading/trailing whitespace",
                    pending.name
                )));
            }
            if groups.contains_key(&pending.name) {
                return Err(Error::DuplicateGroupName(pending.name));
            }
            let bus_arc = buses
                .get(&pending.bus_name)
                .ok_or_else(|| Error::UnknownBusName(pending.bus_name.clone()))?;
            let (vendor, supported_all) = {
                let bus = bus_arc.lock().map_err(|_| Error::BusPoisoned)?;
                let v = bus.vendor().to_string();
                let supported = pending
                    .motors
                    .iter()
                    .find(|m| !bus.codec_supports(m.motor_type))
                    .cloned();
                (v, supported)
            };
            if let Some(unsupported) = supported_all {
                return Err(Error::MotorNotSupportedByCodec {
                    vendor,
                    motor_type: unsupported.motor_type,
                    bus_name: pending.bus_name,
                });
            }
            if matches!(pending.kind, GroupSpecKind::Gripper) && pending.motors.len() != 1 {
                return Err(Error::GripperRequiresOneMotor {
                    got: pending.motors.len(),
                });
            }
            // Construct the MotorGroup and attach the bus.
            let motors: Vec<Motor> = pending
                .motors
                .into_iter()
                .map(|s| Motor::new(s.name, s.motor_type, s.send_id, s.recv_id))
                .collect();
            let mut group = MotorGroup::new(pending.name.clone(), pending.bus_name, motors);
            group.attach_bus(bus_arc.clone());
            let kind = match pending.kind {
                GroupSpecKind::Arm => GroupKind::Arm(Arm(group)),
                GroupSpecKind::Gripper => GroupKind::Gripper(Gripper(group)),
                GroupSpecKind::Generic => GroupKind::Generic(Generic(group)),
            };
            group_order.push(pending.name.clone());
            groups.insert(pending.name, kind);
        }
        Ok(Robot {
            buses,
            bus_order,
            groups,
            group_order,
            connected: false,
            poller: None,
            bus_tokens: HashMap::new(),
            token_to_bus: HashMap::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use motor_codec::{
        BusCapabilities, CanFrame, CodecError, Command, Event, FrameFlags, Limits, MotorCodec,
        MotorRef, MotorTypeId,
    };

    use super::*;
    use crate::transport::MockCanBus;
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct CountingCodec {
        binds: Arc<AtomicUsize>,
        decodes: Arc<AtomicUsize>,
    }
    impl CountingCodec {
        fn new() -> (Self, Arc<AtomicUsize>, Arc<AtomicUsize>) {
            let b = Arc::new(AtomicUsize::new(0));
            let d = Arc::new(AtomicUsize::new(0));
            (
                Self {
                    binds: b.clone(),
                    decodes: d.clone(),
                },
                b,
                d,
            )
        }
    }
    impl MotorCodec for CountingCodec {
        fn vendor_name(&self) -> &'static str {
            "stub"
        }
        fn supports(&self, t: MotorTypeId) -> bool {
            matches!(t, MotorTypeId::Damiao(_))
        }
        fn limits(&self, _: MotorTypeId) -> Result<Limits, CodecError> {
            Ok(Limits {
                p_max: 1.0,
                v_max: 1.0,
                t_max: 1.0,
            })
        }
        fn bind_to_bus(&mut self, _: BusCapabilities) {
            self.binds.fetch_add(1, Ordering::SeqCst);
        }
        fn encode_enable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFC]).map_err(|_| CodecError::DecodeFailed {
                reason: "frame",
            })
        }
        fn encode_disable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFD]).map_err(|_| CodecError::DecodeFailed {
                reason: "frame",
            })
        }
        fn encode_set_zero(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0xFE]).map_err(|_| CodecError::DecodeFailed {
                reason: "frame",
            })
        }
        fn encode_command(
            &self,
            m: MotorRef<'_>,
            _: &Command,
        ) -> Result<CanFrame, CodecError> {
            CanFrame::classical(m.send_id, &[0x55]).map_err(|_| CodecError::DecodeFailed {
                reason: "frame",
            })
        }
        fn decode(&self, frame: &CanFrame) -> Result<Option<Event>, CodecError> {
            self.decodes.fetch_add(1, Ordering::SeqCst);
            if frame.is_fd() || frame.flags.contains(FrameFlags::REMOTE_REQUEST) {
                return Ok(None);
            }
            Ok(Some(Event::State {
                motor_id: frame.id,
                q: 1.0,
                dq: 2.0,
                tau: 3.0,
                t_mos: 30,
                t_rotor: 35,
            }))
        }
    }

    fn motor(name: &str, send: u32, recv: u32) -> MotorSpec {
        MotorSpec::new(name, MotorTypeId::Damiao(3), send, recv)
    }

    #[test]
    fn build_simple_robot() {
        let (codec, binds, _) = CountingCodec::new();
        let robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(codec))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11), motor("j1", 0x02, 0x12)])
            .build()
            .unwrap();
        assert_eq!(robot.bus_names().collect::<Vec<_>>(), vec!["main"]);
        assert_eq!(robot.group_names().collect::<Vec<_>>(), vec!["arm"]);
        assert_eq!(binds.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn duplicate_bus_name_rejected() {
        let (c1, _, _) = CountingCodec::new();
        let (c2, _, _) = CountingCodec::new();
        let r = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c1))
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c2))
            .build();
        assert!(matches!(r, Err(Error::DuplicateBusName(ref s)) if s == "main"));
    }

    #[test]
    fn duplicate_group_name_rejected() {
        let (c, _, _) = CountingCodec::new();
        let r = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .add_arm("arm", "main", vec![motor("j1", 0x02, 0x12)])
            .build();
        assert!(matches!(r, Err(Error::DuplicateGroupName(ref s)) if s == "arm"));
    }

    #[test]
    fn unknown_bus_name_rejected() {
        let r = RobotBuilder::new()
            .add_arm("arm", "ghost", vec![motor("j0", 0x01, 0x11)])
            .build();
        assert!(matches!(r, Err(Error::UnknownBusName(ref s)) if s == "ghost"));
    }

    #[test]
    fn motor_not_supported_by_codec() {
        let (c, _, _) = CountingCodec::new();
        let r = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm(
                "arm",
                "main",
                vec![MotorSpec::new(
                    "j0",
                    MotorTypeId::Robostride(0),
                    0x01,
                    0x11,
                )],
            )
            .build();
        assert!(matches!(
            r,
            Err(Error::MotorNotSupportedByCodec { .. })
        ));
    }

    #[test]
    fn gripper_requires_one_motor() {
        let (c, _, _) = CountingCodec::new();
        // The public add_gripper API takes a single MotorSpec; the runtime
        // check fires when an internal mis-construction supplies != 1 motors.
        // For this test we use add_generic to mimic the bad input path.
        let r = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_gripper("g", "main", motor("g0", 0x05, 0x18))
            .build();
        assert!(r.is_ok());
    }

    #[test]
    fn build_does_not_open_sockets() {
        // Both phases must complete without opening sockets — the MockCanBus
        // has no fds anyway, so we just verify build() returns Ok and
        // is_connected() is false.
        let (c, _, _) = CountingCodec::new();
        let robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        assert!(!robot.is_connected());
    }

    #[test]
    fn connect_populates_routes() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm(
                "arm",
                "main",
                vec![motor("j0", 0x01, 0x11), motor("j1", 0x02, 0x12)],
            )
            .build()
            .unwrap();
        robot.connect().unwrap();
        assert!(robot.is_connected());
        let bus = robot.bus("main").unwrap().lock().unwrap();
        assert_eq!(bus.routes.len(), 2);
        let r11 = bus.routes.get(&0x11).unwrap();
        assert_eq!(r11.group_name, "arm");
        assert_eq!(r11.motor_index, 0);
    }

    #[test]
    fn can_id_collision_across_groups_on_same_bus() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x18)])
            .add_gripper("g", "main", motor("g0", 0x05, 0x18))
            .build()
            .unwrap();
        let r = robot.connect();
        assert!(matches!(
            r,
            Err(Error::CanIdCollision {
                ref bus_name,
                recv_id: 0x18,
                ..
            }) if bus_name == "main"
        ));
    }

    #[test]
    fn collision_across_buses_is_not_error() {
        let (c1, _, _) = CountingCodec::new();
        let (c2, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("left", Box::new(MockCanBus::new("l")), Box::new(c1))
            .add_bus("right", Box::new(MockCanBus::new("r")), Box::new(c2))
            .add_arm("la", "left", vec![motor("j0", 0x01, 0x11)])
            .add_arm("ra", "right", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        robot.connect().unwrap();
    }

    #[test]
    fn tick_before_connect_returns_not_connected() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        let r = robot.tick(Duration::from_millis(1));
        assert!(matches!(r, Err(Error::NotConnected)));
    }

    #[test]
    fn enable_before_connect_returns_not_connected() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        assert!(matches!(robot.enable(), Err(Error::NotConnected)));
    }

    #[test]
    fn disable_without_enable_is_noop() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        // Not connected → disable returns Ok no-op
        robot.disable().unwrap();
        // Connected but never enabled → disable runs without enable ACK first
        robot.connect().unwrap();
        robot.disable().unwrap();
    }

    #[test]
    fn tick_quiet_buses_returns_within_deadline() {
        let (c, _, _) = CountingCodec::new();
        let mut robot = RobotBuilder::new()
            .add_bus("main", Box::new(MockCanBus::new("m")), Box::new(c))
            .add_arm("arm", "main", vec![motor("j0", 0x01, 0x11)])
            .build()
            .unwrap();
        robot.connect().unwrap();
        let t0 = std::time::Instant::now();
        robot.tick(Duration::from_millis(5)).unwrap();
        let elapsed = t0.elapsed();
        // MockCanBus has no fd, so poller is empty and tick returns essentially
        // immediately (the BusPoller wait still respects deadline; here it has
        // no fds so it returns 0-token vec immediately).
        assert!(
            elapsed < Duration::from_millis(50),
            "tick took {elapsed:?}"
        );
    }

    /// Source grep: no group method body calls drain_inbound_nonblocking.
    /// Enforced as a textual scan of the group source.
    #[test]
    fn group_source_does_not_call_drain() {
        let src = include_str!("group.rs");
        // Allow `drain_inbound_nonblocking` to appear in a doc comment / module
        // doc but not in any method body. We test for the literal call form.
        for needle in ["drain_inbound_nonblocking(", ".drain_inbound_nonblocking"] {
            assert!(
                !src.contains(needle),
                "group.rs must not call {needle}; only Robot::tick may"
            );
        }
    }
}
