//! Device selector and descriptor-layout validation independent of nusb.

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Candidate {
    pub(crate) vendor_id: u16,
    pub(crate) product_id: u16,
    pub(crate) serial_number: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Selector {
    Serial(String),
    Index(usize),
}

pub(crate) fn validated_selector(
    serial_number: Option<&str>,
    index: Option<usize>,
) -> Result<Selector, String> {
    match (serial_number, index) {
        (Some(_), Some(_)) => {
            Err("serial number and enumeration index are mutually exclusive".into())
        }
        (Some(""), None) => Err("serial number must not be empty".into()),
        (Some(serial), None) => Ok(Selector::Serial(serial.to_owned())),
        (None, Some(index)) => Ok(Selector::Index(index)),
        (None, None) => Ok(Selector::Index(0)),
    }
}

pub(crate) fn select_candidate(
    candidates: &[Candidate],
    vendor_id: u16,
    product_id: u16,
    selector: &Selector,
) -> Result<usize, String> {
    let matches: Vec<usize> = candidates
        .iter()
        .enumerate()
        .filter(|(_, candidate)| {
            candidate.vendor_id == vendor_id && candidate.product_id == product_id
        })
        .map(|(index, _)| index)
        .collect();

    match selector {
        Selector::Index(index) => matches.get(*index).copied().ok_or_else(|| {
            format!(
                "gs_usb adapter {vendor_id:04x}:{product_id:04x} index {index} is out of range ({} matches)",
                matches.len()
            )
        }),
        Selector::Serial(serial) => {
            let serial_matches: Vec<usize> = matches
                .into_iter()
                .filter(|index| candidates[*index].serial_number.as_deref() == Some(serial))
                .collect();
            match serial_matches.as_slice() {
                [index] => Ok(*index),
                [] => Err(format!(
                    "gs_usb adapter {vendor_id:04x}:{product_id:04x} with serial '{serial}' was not found"
                )),
                _ => Err(format!(
                    "gs_usb adapter serial '{serial}' is ambiguous ({} matches)",
                    serial_matches.len()
                )),
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum EndpointKind {
    BulkIn,
    BulkOut,
    Other,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct AlternateSetting {
    pub(crate) interface_number: u8,
    pub(crate) alternate_setting: u8,
    pub(crate) endpoints: Vec<(u8, EndpointKind)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct EndpointLayout {
    pub(crate) interface_number: u8,
    pub(crate) alternate_setting: u8,
    pub(crate) bulk_in: u8,
    pub(crate) bulk_out: u8,
}

pub(crate) fn discover_endpoint_layout(
    settings: &[AlternateSetting],
) -> Result<EndpointLayout, String> {
    let mut layouts = Vec::new();
    for setting in settings {
        let inputs: Vec<u8> = setting
            .endpoints
            .iter()
            .filter_map(|(address, kind)| (*kind == EndpointKind::BulkIn).then_some(*address))
            .collect();
        let outputs: Vec<u8> = setting
            .endpoints
            .iter()
            .filter_map(|(address, kind)| (*kind == EndpointKind::BulkOut).then_some(*address))
            .collect();
        if inputs.len() == 1 && outputs.len() == 1 {
            layouts.push(EndpointLayout {
                interface_number: setting.interface_number,
                alternate_setting: setting.alternate_setting,
                bulk_in: inputs[0],
                bulk_out: outputs[0],
            });
        } else if inputs.len() > 1 || outputs.len() > 1 {
            return Err(format!(
                "gs_usb interface {} alternate {} has ambiguous bulk endpoints",
                setting.interface_number, setting.alternate_setting
            ));
        }
    }
    match layouts.as_slice() {
        [layout] => Ok(*layout),
        [] => Err(
            "gs_usb descriptor has no alternate setting with one bulk-IN and one bulk-OUT endpoint"
                .into(),
        ),
        _ => Err("gs_usb descriptor has multiple usable bulk endpoint layouts".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn devices() -> Vec<Candidate> {
        vec![
            Candidate {
                vendor_id: 1,
                product_id: 2,
                serial_number: Some("A".into()),
            },
            Candidate {
                vendor_id: 9,
                product_id: 9,
                serial_number: None,
            },
            Candidate {
                vendor_id: 1,
                product_id: 2,
                serial_number: Some("B".into()),
            },
        ]
    }

    #[test]
    fn selectors_choose_serial_index_and_default_zero() {
        let candidates = devices();
        assert_eq!(
            select_candidate(&candidates, 1, 2, &Selector::Serial("B".into())).unwrap(),
            2
        );
        assert_eq!(
            select_candidate(&candidates, 1, 2, &Selector::Index(1)).unwrap(),
            2
        );
        assert_eq!(validated_selector(None, None).unwrap(), Selector::Index(0));
        assert_eq!(
            select_candidate(&candidates, 1, 2, &validated_selector(None, None).unwrap()).unwrap(),
            0
        );
    }

    #[test]
    fn invalid_and_ambiguous_selectors_fail() {
        assert!(validated_selector(Some("A"), Some(0)).is_err());
        assert!(validated_selector(Some(""), None).is_err());
        assert!(select_candidate(&devices(), 1, 2, &Selector::Index(2)).is_err());
        let duplicate = vec![
            Candidate {
                vendor_id: 1,
                product_id: 2,
                serial_number: Some("A".into()),
            },
            Candidate {
                vendor_id: 1,
                product_id: 2,
                serial_number: Some("A".into()),
            },
        ];
        assert!(
            select_candidate(&duplicate, 1, 2, &Selector::Serial("A".into()))
                .unwrap_err()
                .contains("ambiguous")
        );
    }

    #[test]
    fn descriptor_discovery_uses_addresses_and_rejects_bad_layouts() {
        let valid = AlternateSetting {
            interface_number: 3,
            alternate_setting: 2,
            endpoints: vec![(0x84, EndpointKind::BulkIn), (0x01, EndpointKind::BulkOut)],
        };
        assert_eq!(
            discover_endpoint_layout(std::slice::from_ref(&valid)).unwrap(),
            EndpointLayout {
                interface_number: 3,
                alternate_setting: 2,
                bulk_in: 0x84,
                bulk_out: 0x01
            }
        );

        for settings in [
            vec![AlternateSetting {
                endpoints: vec![(0x84, EndpointKind::BulkIn)],
                ..valid.clone()
            }],
            vec![AlternateSetting {
                endpoints: vec![(0x84, EndpointKind::Other), (0x01, EndpointKind::BulkOut)],
                ..valid.clone()
            }],
            vec![AlternateSetting {
                endpoints: vec![
                    (0x84, EndpointKind::BulkIn),
                    (0x85, EndpointKind::BulkIn),
                    (0x01, EndpointKind::BulkOut),
                ],
                ..valid.clone()
            }],
            vec![
                valid.clone(),
                AlternateSetting {
                    interface_number: 4,
                    ..valid.clone()
                },
            ],
        ] {
            assert!(discover_endpoint_layout(&settings).is_err());
        }
    }
}
