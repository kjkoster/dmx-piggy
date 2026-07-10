//! Vendor status and identity queries on the widget's command channel.
//!
//! The loaded device answers a small set of single-byte vendor queries sent on
//! the command bulk OUT endpoint; each reply arrives on the status bulk IN
//! endpoint, echoing the opcode in byte 0 followed by a little-endian value. The
//! opcodes and their meaning were recovered empirically by fuzzing the command
//! surface and cross-checked against the device's own USB descriptors — the
//! serial ([`read_serial`]) matches `iSerial`, and the unique ID ([`read_id`])
//! matches `iConfiguration`, bit for bit.
//!
//! These are input-independent reads with no argument fields, so they make a
//! cheap, safe connectivity/identity check to run once at bring-up — proving the
//! command/response transport before any DMX is streamed.

use crate::dmx::DEFAULT_OUT_ENDPOINT;
use crate::error::Error;
use crate::transport::Transport;

/// Command (bulk OUT) endpoint that carries vendor queries — the single live OUT
/// endpoint, shared with the DMX chunk stream (see [`DEFAULT_OUT_ENDPOINT`]).
const CMD_ENDPOINT: u8 = DEFAULT_OUT_ENDPOINT;
/// Status (bulk IN) endpoint that carries the replies.
const STATUS_ENDPOINT: u8 = 0x82;

/// Query opcode: read the device serial number.
const OP_SERIAL: u8 = 0x09;
/// Query opcode: read the 48-bit unique ID.
const OP_ID: u8 = 0x23;

/// Length of the unique-ID value, in bytes (a 48-bit identifier).
pub const ID_LEN: usize = 6;

/// Opcode `0x0B` is a *reserved, destructive* command: it runs a RAM self-test
/// (firmware routine `0x14A7`, dispatched at `0x0634`) that fills, verifies, and
/// zeros XDATA `0x2000..0x7B00` — a range that **contains the DMX frame buffers**
/// (`0x6000`/`0x6500`/`0x6A00`) — then replies with a pass/fail byte. It takes no
/// argument, but it wipes the outgoing universe and stalls the device while it
/// runs. This crate must never send it; the value is named here so it is
/// documented rather than silently avoided.
///
/// Note also that its reply puts the pass/fail result in byte 0, *not* an opcode
/// echo, so it would fail [`query`]'s echo check even if it were sent.
pub const RESERVED_UNSAFE_OP: u8 = 0x0B;

/// Size of a reply buffer. Replies observed so far are at most 7 bytes
/// (opcode + 6), but the status endpoint ([`STATUS_ENDPOINT`]) is a bulk IN
/// endpoint with a 64-byte max packet size, and Linux `usbdevfs` on the Pi
/// rejects any bulk-IN submission whose length is not a whole multiple of that
/// (`EINVAL`). So the buffer must be a multiple of 64, not merely "big enough":
/// the device short-replies with a partial packet, which terminates the
/// transfer, and [`query`] returns the real byte count.
const REPLY_MAX: usize = 64;

/// Read the device serial number.
///
/// Returns the value read (the sample unit reports `4095`). Sends [`OP_SERIAL`] on
/// the command endpoint and reads the echoed reply from the status endpoint.
pub async fn read_serial<T: Transport>(transport: &mut T) -> Result<u32, Error<T::Error>> {
    let mut reply = [0u8; REPLY_MAX];
    let n = query(transport, OP_SERIAL, &mut reply).await?;
    Ok(le_u32(&reply[1..n]))
}

/// Read the device's 48-bit unique ID (the sample unit reports `A93E5601ED8D`).
pub async fn read_id<T: Transport>(transport: &mut T) -> Result<[u8; ID_LEN], Error<T::Error>> {
    let mut reply = [0u8; REPLY_MAX];
    let n = query(transport, OP_ID, &mut reply).await?;
    if n < 1 + ID_LEN {
        return Err(Error::UnexpectedReply);
    }
    let mut id = [0u8; ID_LEN];
    id.copy_from_slice(&reply[1..1 + ID_LEN]);
    Ok(id)
}

/// Send a single-byte query and capture its reply, validating that the device
/// echoed the opcode back in byte 0. Returns the number of bytes read.
async fn query<T: Transport>(
    transport: &mut T,
    opcode: u8,
    reply: &mut [u8],
) -> Result<usize, Error<T::Error>> {
    transport
        .bulk_out(CMD_ENDPOINT, &[opcode])
        .await
        .map_err(Error::Transport)?;
    let n = transport
        .bulk_in(STATUS_ENDPOINT, reply)
        .await
        .map_err(Error::Transport)?;
    // Every reply echoes the opcode first; anything else is not our answer.
    if n == 0 || reply[0] != opcode {
        return Err(Error::UnexpectedReply);
    }
    Ok(n)
}

/// Little-endian decode of up to four value bytes.
fn le_u32(bytes: &[u8]) -> u32 {
    let mut value = 0u32;
    for (i, &b) in bytes.iter().take(4).enumerate() {
        value |= (b as u32) << (8 * i);
    }
    value
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};

    // Minimal blocking poll for the always-ready futures the fake transport yields.
    // A noop waker and a single poll suffice — no real executor, and no `unsafe`.
    fn block_on<F: Future>(fut: F) -> F::Output {
        let mut cx = Context::from_waker(Waker::noop());
        let mut fut = pin!(fut);
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// Replays the real 0x82 replies captured from hardware, keyed on the last
    /// command byte written to the OUT endpoint.
    struct Fake {
        last: u8,
    }

    impl Transport for Fake {
        type Error = ();
        async fn control_out(&mut self, _: u8, _: u8, _: u16, _: u16, _: &[u8]) -> Result<(), ()> {
            Ok(())
        }
        async fn control_in(
            &mut self,
            _: u8,
            _: u8,
            _: u16,
            _: u16,
            _: &mut [u8],
        ) -> Result<usize, ()> {
            Ok(0)
        }
        async fn bulk_out(&mut self, _: u8, data: &[u8]) -> Result<(), ()> {
            self.last = data[0];
            Ok(())
        }
        async fn bulk_in(&mut self, _: u8, buf: &mut [u8]) -> Result<usize, ()> {
            let reply: &[u8] = match self.last {
                0x09 => &[0x09, 0xff, 0x0f, 0x00],
                0x23 => &[0x23, 0xa9, 0x3e, 0x56, 0x01, 0xed, 0x8d],
                _ => &[],
            };
            let n = reply.len().min(buf.len());
            buf[..n].copy_from_slice(&reply[..n]);
            Ok(n)
        }
    }

    #[test]
    fn reads_serial_matching_the_descriptor() {
        let mut t = Fake { last: 0 };
        assert_eq!(block_on(read_serial(&mut t)).unwrap(), 4095);
    }

    #[test]
    fn reads_the_unique_id() {
        let mut t = Fake { last: 0 };
        assert_eq!(
            block_on(read_id(&mut t)).unwrap(),
            [0xa9, 0x3e, 0x56, 0x01, 0xed, 0x8d],
        );
    }

    #[test]
    fn rejects_a_reply_that_does_not_echo_the_opcode() {
        struct Bad;
        impl Transport for Bad {
            type Error = ();
            async fn control_out(
                &mut self,
                _: u8,
                _: u8,
                _: u16,
                _: u16,
                _: &[u8],
            ) -> Result<(), ()> {
                Ok(())
            }
            async fn control_in(
                &mut self,
                _: u8,
                _: u8,
                _: u16,
                _: u16,
                _: &mut [u8],
            ) -> Result<usize, ()> {
                Ok(0)
            }
            async fn bulk_out(&mut self, _: u8, _: &[u8]) -> Result<(), ()> {
                Ok(())
            }
            async fn bulk_in(&mut self, _: u8, buf: &mut [u8]) -> Result<usize, ()> {
                buf[0] = 0xff; // wrong opcode echo
                Ok(1)
            }
        }
        assert!(matches!(
            block_on(read_serial(&mut Bad)),
            Err(Error::UnexpectedReply)
        ));
    }
}
