//! The device over `nusb`: the data endpoints as engine [`Pipe`]s, and the command channel
//! (0x01 / 0x81) as synchronous transactions (`docs/PROTOCOL.md` §1, §3.1).

use std::task::{Context, Poll};
use std::time::Duration;

use nusb::transfer::{Buffer, Bulk, ControlIn, ControlType, In, Out, Recipient, TransferError};
use nusb::{Endpoint, Interface, MaybeFuture};
use thiserror::Error;

use crate::engine::{Buf, Done, Pipe, TransferFailure};
use crate::wire::{self, COMMAND_BLOCK, Command, REPLY_HEADER_LEN, ReplyError, ReplyHeader};

impl Buf for Buffer {
    fn received(&self, len: usize) -> &[u8] {
        let bytes: &[u8] = self;
        &bytes[..len.min(bytes.len())]
    }

    fn load(&mut self, bytes: &[u8]) {
        self.clear();
        self.extend_from_slice(bytes);
    }

    fn request(&mut self, len: usize) {
        self.clear();
        self.set_requested_len(len);
    }
}

#[must_use]
pub fn failure(error: TransferError) -> TransferFailure {
    match error {
        TransferError::Cancelled => TransferFailure::Cancelled,
        TransferError::Disconnected => TransferFailure::Disconnected,
        TransferError::Stall => TransferFailure::Stall,
        TransferError::Fault => TransferFailure::Fault,
        _ => TransferFailure::Other,
    }
}

/// One bulk endpoint as an engine pipe.
pub struct BulkPipe<Dir: nusb::transfer::EndpointDirection>(pub Endpoint<Bulk, Dir>);

macro_rules! pipe {
    ($dir:ty) => {
        impl Pipe for BulkPipe<$dir> {
            type Buf = Buffer;

            fn submit(&mut self, buf: Buffer) {
                self.0.submit(buf);
            }

            fn pending(&self) -> usize {
                self.0.pending()
            }

            fn poll_complete(&mut self, cx: &mut Context<'_>) -> Poll<Done<Buffer>> {
                self.0.poll_next_complete(cx).map(|completion| Done {
                    len: completion.actual_len,
                    status: completion.status.map_err(failure),
                    buf: completion.buffer,
                })
            }

            fn cancel_all(&mut self) {
                self.0.cancel_all();
            }
        }
    };
}

pipe!(In);
pipe!(Out);

/// A command that did not complete.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum CommandError {
    #[error("command {command:?}: USB {stage} failed after {attempts} attempts: {failure:?}")]
    Usb {
        command: Command,
        stage: &'static str,
        attempts: usize,
        failure: TransferFailure,
    },
    #[error("command {command:?}: {error}")]
    Reply { command: Command, error: ReplyError },
}

/// Each bulk transfer's timeout, as the reference's default (§3.1).
pub const COMMAND_TIMEOUT: Duration = Duration::from_millis(500);
/// Attempts per command on a USB-level failure (§3.1).
const COMMAND_ATTEMPTS: usize = 3;

/// The command endpoints, used one transaction at a time.
pub struct Commands {
    out: Endpoint<Bulk, Out>,
    reply: Endpoint<Bulk, In>,
    out_buffer: Option<Buffer>,
    in_buffer: Option<Buffer>,
}

impl Commands {
    #[must_use]
    pub fn new(out: Endpoint<Bulk, Out>, reply: Endpoint<Bulk, In>) -> Self {
        // A reply read asks for whole packets: at least the 1024-byte data phase.
        let packet = reply.max_packet_size().max(1);
        let request = COMMAND_BLOCK.div_ceil(packet) * packet;
        Self {
            out,
            reply,
            out_buffer: Some(Buffer::new(COMMAND_BLOCK)),
            in_buffer: Some(Buffer::new(request)),
        }
    }

    /// Run one command (§3.1): header, 1024-byte block, 12-byte reply header, reply data.
    /// The whole transaction is repeated, up to three attempts, when the header write or a
    /// reply read fails at the USB level; a failed block write alone is carried past, and a
    /// refusal by the device is never retried. Returns the reply payload.
    ///
    /// # Errors
    ///
    /// Returns the USB failure of the last attempt, or why the reply was not accepted.
    pub fn run(
        &mut self,
        command: Command,
        argument: u8,
        payload: &[u8],
    ) -> Result<Vec<u8>, CommandError> {
        let (header, block) =
            wire::command(command, argument, payload).map_err(|error| CommandError::Reply {
                command,
                error: error.into(),
            })?;
        let mut last = None;
        for _ in 0..COMMAND_ATTEMPTS {
            match self.attempt(command, &header, &block) {
                Ok(reply) => return Ok(reply),
                Err(Attempt::Usb(stage, failure)) => last = Some((stage, failure)),
                Err(Attempt::Reply(error)) => return Err(CommandError::Reply { command, error }),
            }
        }
        let (stage, failure) = last.unwrap_or(("transfer", TransferFailure::Other));
        Err(CommandError::Usb {
            command,
            stage,
            attempts: COMMAND_ATTEMPTS,
            failure,
        })
    }

    fn attempt(
        &mut self,
        command: Command,
        header: &[u8],
        block: &[u8],
    ) -> Result<Vec<u8>, Attempt> {
        self.write(header)
            .map_err(|failure| Attempt::Usb("header write", failure))?;
        // The reference carries on to read the reply when only the block write fails.
        let _ = self.write(block);
        let first = self
            .read()
            .map_err(|failure| Attempt::Usb("reply header read", failure))?;
        let reply = ReplyHeader::parse(&first).map_err(|error| Attempt::Reply(error.into()))?;
        reply.accept(command).map_err(Attempt::Reply)?;
        // A device that sends the header and its data in one transfer leaves the data behind
        // the header; otherwise the data phase is its own read.
        let data = if first.len() > REPLY_HEADER_LEN {
            first[REPLY_HEADER_LEN..].to_vec()
        } else {
            self.read()
                .map_err(|failure| Attempt::Usb("reply data read", failure))?
        };
        // The reference requires at least two bytes, the presumed CRC, even of an empty reply.
        if data.len() < reply.length.max(2) {
            return Err(Attempt::Reply(ReplyError::DataShort(data.len())));
        }
        Ok(data[..reply.length].to_vec())
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), TransferFailure> {
        let mut buffer = self
            .out_buffer
            .take()
            .unwrap_or_else(|| Buffer::new(COMMAND_BLOCK));
        buffer.load(bytes);
        let completion = self.out.transfer_blocking(buffer, COMMAND_TIMEOUT);
        let status = completion.status.map_err(failure);
        self.out_buffer = Some(completion.buffer);
        status
    }

    fn read(&mut self) -> Result<Vec<u8>, TransferFailure> {
        let packet = self.reply.max_packet_size().max(1);
        let request = COMMAND_BLOCK.div_ceil(packet) * packet;
        let mut buffer = self
            .in_buffer
            .take()
            .unwrap_or_else(|| Buffer::new(request));
        buffer.request(request);
        let completion = self.reply.transfer_blocking(buffer, COMMAND_TIMEOUT);
        let status = completion.status.map_err(failure);
        let bytes = completion.buffer.received(completion.actual_len).to_vec();
        self.in_buffer = Some(completion.buffer);
        status.map(|()| bytes)
    }
}

enum Attempt {
    Usb(&'static str, TransferFailure),
    Reply(ReplyError),
}

/// A class control IN request on interface 0 (§1.2): bmRequestType 0xA1, wValue 0, wIndex 0.
///
/// # Errors
///
/// Returns the transfer's failure.
pub fn control_in(
    interface: &Interface,
    request: u8,
    length: u16,
) -> Result<Vec<u8>, TransferFailure> {
    interface
        .control_in(
            ControlIn {
                control_type: ControlType::Class,
                recipient: Recipient::Interface,
                request,
                value: 0,
                index: 0,
                length,
            },
            Duration::from_secs(5),
        )
        .wait()
        .map_err(failure)
}
