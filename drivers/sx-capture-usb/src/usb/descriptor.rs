//! Standard USB descriptors, parsed from the raw bytes a usbfs descriptor read returns: the
//! 18-byte device descriptor followed by every configuration descriptor with its interfaces,
//! endpoints and class-specific descriptors (USB 2.0 §9.6). Pure: no device is touched.

use std::fmt;

pub const DT_DEVICE: u8 = 0x01;
pub const DT_CONFIG: u8 = 0x02;
pub const DT_INTERFACE: u8 = 0x04;
pub const DT_ENDPOINT: u8 = 0x05;
pub const DT_INTERFACE_ASSOCIATION: u8 = 0x0b;
pub const DT_SS_ENDPOINT_COMPANION: u8 = 0x30;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DescriptorError(pub String);

impl fmt::Display for DescriptorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "USB descriptors: {}", self.0)
    }
}

impl std::error::Error for DescriptorError {}

fn malformed(detail: impl Into<String>) -> DescriptorError {
    DescriptorError(detail.into())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceDescriptor {
    pub usb_version: u16,
    pub class: u8,
    pub subclass: u8,
    pub protocol: u8,
    pub max_packet_size0: u8,
    pub vendor_id: u16,
    pub product_id: u16,
    pub device_version: u16,
    pub manufacturer_index: u8,
    pub product_index: u8,
    pub serial_index: u8,
    pub configurations: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Endpoint {
    pub address: u8,
    pub attributes: u8,
    pub max_packet_size: u16,
    pub interval: u8,
    /// SuperSpeed companion: (bMaxBurst, bmAttributes, wBytesPerInterval).
    pub companion: Option<(u8, u8, u16)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferType {
    Control,
    Isochronous,
    Bulk,
    Interrupt,
}

impl Endpoint {
    pub fn is_in(&self) -> bool {
        self.address & 0x80 != 0
    }

    pub fn transfer_type(&self) -> TransferType {
        match self.attributes & 0x03 {
            0 => TransferType::Control,
            1 => TransferType::Isochronous,
            2 => TransferType::Bulk,
            _ => TransferType::Interrupt,
        }
    }

    /// Bytes the endpoint may move per service interval: at high speed bits 12..11 of
    /// wMaxPacketSize add up to two extra transactions per microframe; at SuperSpeed the
    /// companion descriptor says it outright.
    pub fn bytes_per_interval(&self) -> u32 {
        if let Some((_, _, bytes)) = self.companion {
            return u32::from(bytes);
        }
        let size = u32::from(self.max_packet_size & 0x07ff);
        let extra = u32::from((self.max_packet_size >> 11) & 0x03);
        size * (1 + extra)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AltSetting {
    pub number: u8,
    pub alternate: u8,
    pub class: u8,
    pub subclass: u8,
    pub protocol: u8,
    pub endpoints: Vec<Endpoint>,
    /// Class-specific descriptors between this interface descriptor and its first endpoint
    /// (UVC's VC/VS descriptors live here), each complete with its length and type bytes.
    pub class_specific: Vec<Vec<u8>>,
    /// Class-specific descriptors following an endpoint (UVC's CS_ENDPOINT interrupt).
    pub endpoint_specific: Vec<Vec<u8>>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Configuration {
    pub value: u8,
    pub attributes: u8,
    pub max_power: u8,
    pub alt_settings: Vec<AltSetting>,
}

impl Configuration {
    /// Every alternate setting of interface `number`, in descriptor order.
    pub fn interface(&self, number: u8) -> impl Iterator<Item = &AltSetting> {
        self.alt_settings.iter().filter(move |a| a.number == number)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Descriptors {
    pub device: DeviceDescriptor,
    pub configurations: Vec<Configuration>,
}

impl Descriptors {
    /// The configuration Android and Linux select on enumeration: the first one.
    pub fn active(&self) -> Result<&Configuration, DescriptorError> {
        self.configurations
            .first()
            .ok_or_else(|| malformed("device has no configuration"))
    }
}

fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}

/// Split a descriptor stream into whole descriptors, refusing a zero or overlong length.
fn records(bytes: &[u8]) -> Result<Vec<&[u8]>, DescriptorError> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < bytes.len() {
        let length = usize::from(bytes[at]);
        if length < 2 || at + length > bytes.len() {
            return Err(malformed(format!(
                "descriptor at byte {at} has length {length} of {} remaining",
                bytes.len() - at
            )));
        }
        out.push(&bytes[at..at + length]);
        at += length;
    }
    Ok(out)
}

pub fn parse(bytes: &[u8]) -> Result<Descriptors, DescriptorError> {
    if bytes.len() < 18 || bytes[0] != 18 || bytes[1] != DT_DEVICE {
        return Err(malformed("no 18-byte device descriptor at the start"));
    }
    let device = DeviceDescriptor {
        usb_version: u16_at(bytes, 2),
        class: bytes[4],
        subclass: bytes[5],
        protocol: bytes[6],
        max_packet_size0: bytes[7],
        vendor_id: u16_at(bytes, 8),
        product_id: u16_at(bytes, 10),
        device_version: u16_at(bytes, 12),
        manufacturer_index: bytes[14],
        product_index: bytes[15],
        serial_index: bytes[16],
        configurations: bytes[17],
    };
    let mut configurations = Vec::new();
    let mut rest = &bytes[18..];
    while !rest.is_empty() {
        if rest.len() < 9 || rest[1] != DT_CONFIG {
            return Err(malformed("expected a configuration descriptor"));
        }
        let total = usize::from(u16_at(rest, 2));
        if total < 9 || total > rest.len() {
            return Err(malformed(format!(
                "configuration wTotalLength {total} exceeds the {} bytes read",
                rest.len()
            )));
        }
        configurations.push(parse_configuration(&rest[..total])?);
        rest = &rest[total..];
    }
    Ok(Descriptors {
        device,
        configurations,
    })
}

fn parse_configuration(bytes: &[u8]) -> Result<Configuration, DescriptorError> {
    let records = records(bytes)?;
    let head = records[0];
    let mut configuration = Configuration {
        value: head[5],
        attributes: head[7],
        max_power: head[8],
        alt_settings: Vec::new(),
    };
    for record in &records[1..] {
        match record[1] {
            DT_INTERFACE => {
                if record.len() < 9 {
                    return Err(malformed("short interface descriptor"));
                }
                configuration.alt_settings.push(AltSetting {
                    number: record[2],
                    alternate: record[3],
                    class: record[5],
                    subclass: record[6],
                    protocol: record[7],
                    endpoints: Vec::new(),
                    class_specific: Vec::new(),
                    endpoint_specific: Vec::new(),
                });
            }
            DT_ENDPOINT => {
                if record.len() < 7 {
                    return Err(malformed("short endpoint descriptor"));
                }
                let alt = configuration
                    .alt_settings
                    .last_mut()
                    .ok_or_else(|| malformed("endpoint before any interface"))?;
                alt.endpoints.push(Endpoint {
                    address: record[2],
                    attributes: record[3],
                    max_packet_size: u16_at(record, 4),
                    interval: record[6],
                    companion: None,
                });
            }
            DT_SS_ENDPOINT_COMPANION => {
                if record.len() < 6 {
                    return Err(malformed("short SuperSpeed companion"));
                }
                if let Some(endpoint) = configuration
                    .alt_settings
                    .last_mut()
                    .and_then(|alt| alt.endpoints.last_mut())
                {
                    endpoint.companion = Some((record[2], record[3], u16_at(record, 4)));
                }
            }
            DT_INTERFACE_ASSOCIATION => {}
            _ => {
                // Class-specific (or unknown) descriptors belong to the interface they follow.
                if let Some(alt) = configuration.alt_settings.last_mut() {
                    if alt.endpoints.is_empty() {
                        alt.class_specific.push(record.to_vec());
                    } else {
                        alt.endpoint_specific.push(record.to_vec());
                    }
                }
            }
        }
    }
    Ok(configuration)
}

/// Decode a USB string descriptor (UTF-16LE after the two header bytes).
pub fn string(bytes: &[u8]) -> Result<String, DescriptorError> {
    if bytes.len() < 2 || bytes[1] != 0x03 || usize::from(bytes[0]) > bytes.len() {
        return Err(malformed("not a string descriptor"));
    }
    let length = usize::from(bytes[0]);
    let units: Vec<u16> = bytes[2..length]
        .chunks_exact(2)
        .map(|pair| u16::from_le_bytes([pair[0], pair[1]]))
        .collect();
    String::from_utf16(&units).map_err(|_| malformed("string descriptor is not UTF-16"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn high_bandwidth_iso_and_superspeed_companion_sizes() {
        let hs = Endpoint {
            address: 0x81,
            attributes: 0x05,
            max_packet_size: 0x1400,
            interval: 1,
            companion: None,
        };
        // 0x1400: 1024 bytes, two extra transactions per microframe.
        assert_eq!(hs.bytes_per_interval(), 3 * 1024);
        assert_eq!(hs.transfer_type(), TransferType::Isochronous);
        assert!(hs.is_in());
        let ss = Endpoint {
            companion: Some((3, 0, 8192)),
            ..hs
        };
        assert_eq!(ss.bytes_per_interval(), 8192);
    }

    #[test]
    fn truncated_and_zero_length_descriptors_are_refused() {
        assert!(parse(&[]).is_err());
        let mut device = vec![
            18, 1, 0, 2, 0, 0, 0, 64, 0xe4, 0x32, 0x30, 0x92, 0, 1, 1, 2, 3, 1,
        ];
        assert_eq!(parse(&device).unwrap().device.vendor_id, 0x32e4);
        device.extend_from_slice(&[9, 2, 200, 0, 1, 1, 0, 0x80, 250]);
        assert!(parse(&device).unwrap_err().0.contains("wTotalLength"));
        let mut zero = device[..18].to_vec();
        zero.extend_from_slice(&[9, 2, 11, 0, 1, 1, 0, 0x80, 250, 0, 4]);
        assert!(parse(&zero).unwrap_err().0.contains("length 0"));
    }

    #[test]
    fn string_descriptors_decode_utf16() {
        let mac = "F0:F5:BD:01:02:0A";
        let mut bytes = vec![(2 + 2 * mac.len()) as u8, 3];
        for unit in mac.encode_utf16() {
            bytes.extend_from_slice(&unit.to_le_bytes());
        }
        assert_eq!(string(&bytes).unwrap(), mac);
        assert!(string(&[4, 2, 0, 0]).is_err());
    }
}
