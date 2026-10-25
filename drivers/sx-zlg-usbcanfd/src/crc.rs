//! The bulk messages' CRC (`docs/PROTOCOL.md` §2.3): CRC-16/MODBUS — polynomial 0x8005
//! reflected (0xA001), initial 0xFFFF, no final XOR — stored little-endian after the payload.

/// The 256-entry table of the reflected polynomial, built at compile time.
const TABLE: [u16; 256] = {
    let mut table = [0_u16; 256];
    let mut byte = 0;
    while byte < 256 {
        #[allow(clippy::cast_possible_truncation)]
        let mut crc = byte as u16;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 == 1 {
                (crc >> 1) ^ 0xA001
            } else {
                crc >> 1
            };
            bit += 1;
        }
        table[byte] = crc;
        byte += 1;
    }
    table
};

/// The CRC of `bytes`.
#[must_use]
pub fn crc16(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0xFFFF, |crc, byte| {
        (crc >> 8) ^ TABLE[usize::from((crc ^ u16::from(*byte)) & 0xFF)]
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The spec's bitwise definition, kept beside the table as its check.
    fn bitwise(bytes: &[u8]) -> u16 {
        let mut crc = 0xFFFF_u16;
        for byte in bytes {
            crc ^= u16::from(*byte);
            for _ in 0..8 {
                crc = if crc & 1 == 1 {
                    (crc >> 1) ^ 0xA001
                } else {
                    crc >> 1
                };
            }
        }
        crc
    }

    #[test]
    fn every_worked_example_of_the_spec_holds() {
        let examples: [(&[u8], u16); 9] = [
            (b"123456789", 0x4B37),
            (&[0x55, 0x01, 0, 0, 0, 0, 0, 0], 0xC895),
            (&[0x55, 0x41, 0, 0, 0, 0, 0, 0], 0x0CD4),
            (&[0x55, 0x12, 0, 0, 0, 0, 0, 0], 0x09B7),
            (&[0x55, 0x13, 0, 0, 0, 0, 0, 0], 0xC9A7),
            (&[0x55, 0x02, 0, 0, 0, 0, 0, 0], 0xC8A6),
            (&[0x55, 0x46, 0, 0, 0, 0, 0, 0], 0xCCA2),
            (&[0x55, 0x47, 0, 0, 1, 0, 0, 0, 0x01], 0x7570),
            (&[0x55, 0x18, 0, 0, 1, 0, 0, 0, 0x55], 0xF634),
        ];
        for (bytes, expected) in examples {
            assert_eq!(crc16(bytes), expected, "{bytes:02X?}");
            assert_eq!(bitwise(bytes), expected);
        }
    }

    #[test]
    fn the_table_is_the_bitwise_definition() {
        let bytes: Vec<u8> = (0..=255).chain((0..=255).rev()).collect();
        for end in 0..bytes.len() {
            assert_eq!(crc16(&bytes[..end]), bitwise(&bytes[..end]));
        }
    }
}
