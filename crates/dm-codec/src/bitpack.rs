//! MIT-mode bit-packing utilities.
//!
//! Damiao's MIT control packs five floats into 64 bits via per-field linear
//! quantization. The exact byte layout matches both reference implementations
//! (cmjang `DM_CAN.py::controlMIT`, openarm_can `MITModePacker`).

/// Quantize `x` ∈ [`x_min`, `x_max`] into an unsigned integer of `bits` width.
///
/// Out-of-range inputs are clamped to the boundary before quantization.
pub(crate) fn float_to_uint(x: f64, x_min: f64, x_max: f64, bits: u32) -> u16 {
    let span = x_max - x_min;
    let clamped = x.clamp(x_min, x_max);
    let levels = (1u32 << bits) - 1;
    // Round-to-nearest. `(clamped - x_min)` is always >= 0 since we clamped,
    // so adding 0.5 and casting to u32 (which truncates) implements
    // round-half-up. `core::f64::round` is libm and unavailable in no_std.
    let scaled = (clamped - x_min) * (levels as f64) / span + 0.5;
    let q = scaled as u32;
    q.min(levels) as u16
}

/// Inverse of [`float_to_uint`].
pub(crate) fn uint_to_float(u: u16, x_min: f64, x_max: f64, bits: u32) -> f64 {
    let span = x_max - x_min;
    let levels = (1u32 << bits) - 1;
    x_min + (u as f64) * span / (levels as f64)
}

/// Pack MIT (q, dq, kp, kd, tau) into an 8-byte payload using Damiao's layout.
///
/// Byte layout (matches cmjang `DM_CAN.py::controlMIT`):
///
/// ```text
/// byte 0: q[15:8]
/// byte 1: q[7:0]
/// byte 2: dq[11:4]
/// byte 3: (dq[3:0] << 4) | kp[11:8]
/// byte 4: kp[7:0]
/// byte 5: kd[11:4]
/// byte 6: (kd[3:0] << 4) | tau[11:8]
/// byte 7: tau[7:0]
/// ```
#[allow(clippy::too_many_arguments)]
pub(crate) fn pack_mit_payload(
    q: f64,
    dq: f64,
    kp: f64,
    kd: f64,
    tau: f64,
    p_max: f64,
    v_max: f64,
    t_max: f64,
) -> [u8; 8] {
    let q_u = float_to_uint(q, -p_max, p_max, 16);
    let dq_u = float_to_uint(dq, -v_max, v_max, 12);
    let kp_u = float_to_uint(kp, 0.0, 500.0, 12);
    let kd_u = float_to_uint(kd, 0.0, 5.0, 12);
    let tau_u = float_to_uint(tau, -t_max, t_max, 12);
    [
        ((q_u >> 8) & 0xff) as u8,
        (q_u & 0xff) as u8,
        ((dq_u >> 4) & 0xff) as u8,
        ((((dq_u & 0xf) << 4) | ((kp_u >> 8) & 0xf)) & 0xff) as u8,
        (kp_u & 0xff) as u8,
        ((kd_u >> 4) & 0xff) as u8,
        ((((kd_u & 0xf) << 4) | ((tau_u >> 8) & 0xf)) & 0xff) as u8,
        (tau_u & 0xff) as u8,
    ]
}

/// Unpack a Damiao state response payload into (q, dq, tau).
///
/// Layout (8 bytes total; bytes 6 and 7 carry the temperatures, decoded by the
/// caller):
///
/// ```text
/// byte 0:           motor id (lower 4 bits) | err (upper 4 bits)
/// byte 1:           q[15:8]
/// byte 2:           q[7:0]
/// byte 3:           dq[11:4]
/// byte 4 (hi nib):  dq[3:0]
/// byte 4 (lo nib):  tau[11:8]
/// byte 5:           tau[7:0]
/// byte 6:           t_mos (degrees C)
/// byte 7:           t_rotor (degrees C)
/// ```
pub(crate) fn unpack_state_payload(
    payload: &[u8; 8],
    p_max: f64,
    v_max: f64,
    t_max: f64,
) -> (u8, u8, f64, f64, f64, i16, i16) {
    let motor_id_low = payload[0] & 0x0f;
    let err = (payload[0] >> 4) & 0x0f;
    let q_u = ((payload[1] as u16) << 8) | (payload[2] as u16);
    let dq_u = ((payload[3] as u16) << 4) | (((payload[4] as u16) >> 4) & 0xf);
    let tau_u = (((payload[4] as u16) & 0xf) << 8) | (payload[5] as u16);
    let q = uint_to_float(q_u, -p_max, p_max, 16);
    let dq = uint_to_float(dq_u, -v_max, v_max, 12);
    let tau = uint_to_float(tau_u, -t_max, t_max, 12);
    let t_mos = payload[6] as i16;
    let t_rotor = payload[7] as i16;
    (motor_id_low, err, q, dq, tau, t_mos, t_rotor)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn float_to_uint_round_trip_within_lsb() {
        let cases: &[(f64, f64, u32)] = &[
            (-12.5, 12.5, 16),
            (-10.0, 10.0, 12),
            (0.0, 500.0, 12),
            (0.0, 5.0, 12),
            (-28.0, 28.0, 12),
        ];
        for &(min, max, bits) in cases {
            for k in 0..20 {
                let x = min + (max - min) * (k as f64) / 19.0;
                let u = float_to_uint(x, min, max, bits);
                let back = uint_to_float(u, min, max, bits);
                let lsb = (max - min) / ((1u32 << bits) - 1) as f64;
                assert!(
                    (x - back).abs() <= lsb,
                    "x={x} back={back} lsb={lsb} bits={bits}"
                );
            }
        }
    }

    #[test]
    fn out_of_range_clamps_to_boundary() {
        let u = float_to_uint(1000.0, -28.0, 28.0, 12);
        assert_eq!(u, (1u32 << 12) as u16 - 1);
        let u = float_to_uint(-1000.0, -28.0, 28.0, 12);
        assert_eq!(u, 0);
    }

    #[test]
    fn zero_mit_packs_to_midpoints() {
        // kp=0, kd=0 are at the bottom of their ranges. q=dq=tau=0 are at the
        // midpoint of their symmetric ranges.
        let payload = pack_mit_payload(0.0, 0.0, 0.0, 0.0, 0.0, 12.5, 10.0, 28.0);
        // q midpoint of 16-bit range = 32767 or 32768
        let q_u = ((payload[0] as u16) << 8) | (payload[1] as u16);
        assert!((q_u as i32 - 32767).abs() <= 1, "q_u={q_u}");
        // dq midpoint of 12-bit range = 2047
        let dq_u = ((payload[2] as u16) << 4) | (((payload[3] as u16) >> 4) & 0xf);
        assert!((dq_u as i32 - 2047).abs() <= 1, "dq_u={dq_u}");
    }

    #[test]
    fn mit_round_trip_through_state_layout() {
        // pack_mit and the state-frame unpack use the same scaling for q/dq/tau,
        // so a synthetic state payload built from a packed MIT payload's q/dq/tau
        // bits should decode back to the originals within ±1 LSB.
        let (q, dq, tau) = (1.0, 0.5, 5.0);
        let (p, v, t) = (12.5, 10.0, 28.0);
        let q_u = float_to_uint(q, -p, p, 16);
        let dq_u = float_to_uint(dq, -v, v, 12);
        let tau_u = float_to_uint(tau, -t, t, 12);
        let payload = [
            0x11, // motor id 1, no err
            ((q_u >> 8) & 0xff) as u8,
            (q_u & 0xff) as u8,
            ((dq_u >> 4) & 0xff) as u8,
            ((((dq_u & 0xf) << 4) | ((tau_u >> 8) & 0xf)) & 0xff) as u8,
            (tau_u & 0xff) as u8,
            30, // t_mos
            35, // t_rotor
        ];
        let (mid_low, _err, qo, dqo, tauo, t_mos, t_rotor) =
            unpack_state_payload(&payload, p, v, t);
        assert_eq!(mid_low, 1);
        let lsb_q = (2.0 * p) / 65535.0;
        let lsb_d = (2.0 * v) / 4095.0;
        let lsb_t = (2.0 * t) / 4095.0;
        assert!((qo - q).abs() <= lsb_q);
        assert!((dqo - dq).abs() <= lsb_d);
        assert!((tauo - tau).abs() <= lsb_t);
        assert_eq!(t_mos, 30);
        assert_eq!(t_rotor, 35);
    }
}
