//! Deterministic nominal bit-timing selection from adapter capabilities.

use super::protocol::{BitTimingConstraints, NominalBitTiming};

pub(crate) fn calculate_nominal_timing(
    constraints: BitTimingConstraints,
    bitrate: u32,
) -> Result<NominalBitTiming, String> {
    validate_constraints(constraints, bitrate)?;

    let mut best: Option<(u64, u32, u32, NominalBitTiming)> = None;
    let mut brp = constraints.brp_min;
    while brp <= constraints.brp_max {
        let denominator = u64::from(brp) * u64::from(bitrate);
        if u64::from(constraints.clock_hz) % denominator == 0 {
            let total_tq = u64::from(constraints.clock_hz) / denominator;
            if let Ok(total_tq_u32) = u32::try_from(total_tq) {
                for tseg2 in constraints.tseg2_min..=constraints.tseg2_max {
                    let Some(tseg1) = total_tq_u32.checked_sub(1 + tseg2) else {
                        continue;
                    };
                    if !(constraints.tseg1_min..=constraints.tseg1_max).contains(&tseg1)
                        || tseg1 < 2
                    {
                        continue;
                    }
                    let sample_error = (u64::from(1 + tseg1) * 1_000).abs_diff(total_tq * 875);
                    let timing = NominalBitTiming {
                        prop_seg: 1,
                        phase_seg1: tseg1 - 1,
                        phase_seg2: tseg2,
                        sjw: 1,
                        brp,
                    };
                    // Lower sample-point error wins; then prefer more time
                    // quanta for resolution, then the lower BRP and TSEG2.
                    let score = (sample_error, u32::MAX - total_tq_u32, brp, timing);
                    if best
                        .as_ref()
                        .is_none_or(|current| score_key(&score) < score_key(current))
                    {
                        best = Some(score);
                    }
                }
            }
        }
        let Some(next) = brp.checked_add(constraints.brp_increment) else {
            break;
        };
        brp = next;
    }

    best.map(|(_, _, _, timing)| timing).ok_or_else(|| {
        format!(
            "bitrate {bitrate} cannot be produced from {} Hz with tseg1={}..{}, tseg2={}..{}, brp={}..{} step {}",
            constraints.clock_hz,
            constraints.tseg1_min,
            constraints.tseg1_max,
            constraints.tseg2_min,
            constraints.tseg2_max,
            constraints.brp_min,
            constraints.brp_max,
            constraints.brp_increment,
        )
    })
}

fn score_key(score: &(u64, u32, u32, NominalBitTiming)) -> (u64, u32, u32, u32) {
    (score.0, score.1, score.2, score.3.phase_seg2)
}

fn validate_constraints(c: BitTimingConstraints, bitrate: u32) -> Result<(), String> {
    if bitrate == 0
        || c.clock_hz == 0
        || c.tseg1_min == 0
        || c.tseg1_min > c.tseg1_max
        || c.tseg2_min == 0
        || c.tseg2_min > c.tseg2_max
        || c.sjw_max == 0
        || c.brp_min == 0
        || c.brp_min > c.brp_max
        || c.brp_increment == 0
    {
        return Err(format!(
            "invalid gs_usb timing constraints for bitrate {bitrate}: {c:?}"
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn constraints(clock_hz: u32) -> BitTimingConstraints {
        BitTimingConstraints {
            features: 0,
            clock_hz,
            tseg1_min: 2,
            tseg1_max: 256,
            tseg2_min: 1,
            tseg2_max: 128,
            sjw_max: 128,
            brp_min: 1,
            brp_max: 512,
            brp_increment: 1,
        }
    }

    fn actual_bitrate(clock_hz: u32, timing: NominalBitTiming) -> u32 {
        clock_hz / (timing.brp * (1 + timing.tseg1() + timing.phase_seg2))
    }

    #[test]
    fn finds_exact_one_megabit_for_common_and_observed_clocks() {
        for clock in [40_000_000, 48_000_000, 80_000_000, 170_000_000] {
            let timing = calculate_nominal_timing(constraints(clock), 1_000_000).unwrap();
            assert_eq!(actual_bitrate(clock, timing), 1_000_000);
            assert_eq!(timing.prop_seg, 1);
            assert_eq!(timing.sjw, 1);
        }
    }

    #[test]
    fn solution_respects_every_reported_constraint() {
        let c = BitTimingConstraints {
            tseg1_min: 5,
            tseg1_max: 16,
            tseg2_min: 2,
            tseg2_max: 8,
            sjw_max: 4,
            brp_min: 2,
            brp_max: 64,
            brp_increment: 2,
            ..constraints(80_000_000)
        };
        let timing = calculate_nominal_timing(c, 1_000_000).unwrap();
        assert!((c.tseg1_min..=c.tseg1_max).contains(&timing.tseg1()));
        assert!((c.tseg2_min..=c.tseg2_max).contains(&timing.phase_seg2));
        assert!((c.brp_min..=c.brp_max).contains(&timing.brp));
        assert_eq!((timing.brp - c.brp_min) % c.brp_increment, 0);
        assert!(timing.sjw <= c.sjw_max && timing.sjw <= timing.phase_seg2);
    }

    #[test]
    fn rejects_impossible_bitrate_and_invalid_constraints() {
        let mut tight = constraints(40_000_000);
        tight.tseg1_max = 2;
        tight.tseg2_max = 1;
        tight.brp_max = 1;
        assert!(calculate_nominal_timing(tight, 1_000_000).is_err());

        let mut invalid = constraints(40_000_000);
        invalid.brp_increment = 0;
        assert!(calculate_nominal_timing(invalid, 1_000_000).is_err());
        assert!(calculate_nominal_timing(constraints(40_000_000), 0).is_err());
    }
}
