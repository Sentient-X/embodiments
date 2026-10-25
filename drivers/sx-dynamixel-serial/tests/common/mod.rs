//! A scripted serial link for the crate's suites.

use std::collections::VecDeque;
use std::io;
use std::time::Duration;

use sx_embodiment_drivers::serial::SerialPort;

pub fn bytes(hex: &str) -> Vec<u8> {
    let hex: String = hex.chars().filter(|c| !c.is_whitespace()).collect();
    (0..hex.len())
        .step_by(2)
        .map(|index| u8::from_str_radix(&hex[index..index + 2], 16).expect("hex"))
        .collect()
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut text, byte| {
        use std::fmt::Write as _;
        write!(text, "{byte:02x}").expect("hex");
        text
    })
}

/// A link that accepts only the expected requests, in order, and answers each with its
/// recorded status packets. A request with no expectation left is recorded and unanswered.
/// Like the mint's port, its packet timeout runs out as soon as nothing more is pending.
#[derive(Default)]
pub struct FakePort {
    pub expected: VecDeque<(Vec<u8>, Vec<u8>)>,
    pub pending: VecDeque<u8>,
    pub written: Vec<Vec<u8>>,
}

impl FakePort {
    pub fn expecting<'a>(exchanges: impl IntoIterator<Item = (&'a str, &'a str)>) -> Self {
        Self {
            expected: exchanges
                .into_iter()
                .map(|(request, response)| (bytes(request), bytes(response)))
                .collect(),
            ..Self::default()
        }
    }
}

impl SerialPort for FakePort {
    fn clear_port(&mut self) -> io::Result<()> {
        self.pending.clear();
        Ok(())
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        self.written.push(packet.to_vec());
        // A broadcast has no answer: it consumes an expectation only when one names it.
        let broadcast = packet.get(4) == Some(&0xFE);
        if broadcast
            && self
                .expected
                .front()
                .is_none_or(|(request, _)| request.as_slice() != packet)
        {
            return Ok(packet.len());
        }
        if let Some((request, response)) = self.expected.pop_front() {
            assert_eq!(hex(packet), hex(&request), "request bytes");
            self.pending.extend(response);
        }
        Ok(packet.len())
    }

    fn read_port(&mut self, length: usize) -> io::Result<Vec<u8>> {
        let available = length.min(self.pending.len());
        Ok(self.pending.drain(..available).collect())
    }

    fn timeout(&self) -> Duration {
        Duration::from_millis(10)
    }

    fn set_packet_timeout(&mut self, _packet_length: usize) {}

    fn is_packet_timeout(&mut self) -> bool {
        self.pending.is_empty()
    }
}
