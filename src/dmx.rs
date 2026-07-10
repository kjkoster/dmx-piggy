//! DMX-512 universe buffer and frame transmission.

use crate::error::Error;
use crate::transport::Transport;

/// Channels in one DMX-512 universe.
pub const CHANNELS: usize = 512;

/// The bulk OUT endpoints the loaded device advertises.
///
/// The DMX data channel is `0x02` (confirmed from the firmware: it is where the
/// chunk reassembler reads commands). `0x04` stalls, `0x05` is not used for data.
/// Kept as a list only for [`crate::device::Widget::with_endpoint`] experiments.
pub const OUT_ENDPOINT_CANDIDATES: [u8; 3] = [0x02, 0x04, 0x05];

/// The DMX data endpoint that the chunked frame is delivered to.
pub const DEFAULT_OUT_ENDPOINT: u8 = OUT_ENDPOINT_CANDIDATES[0];

/// Channel address outside the DMX range `1..=512`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfRange;

/// A DMX-512 universe: 512 channel slots.
#[derive(Clone)]
pub struct Universe {
    slots: [u8; CHANNELS],
}

impl Universe {
    /// A universe with every channel at zero.
    pub const fn new() -> Self {
        Self {
            slots: [0; CHANNELS],
        }
    }

    /// Set a channel using 1-based DMX addressing.
    ///
    /// Rejects `0` and out-of-range addresses rather than silently aliasing them
    /// onto a valid slot.
    pub fn set(&mut self, channel: u16, value: u8) -> Result<(), OutOfRange> {
        if channel == 0 || channel as usize > CHANNELS {
            return Err(OutOfRange);
        }
        self.slots[channel as usize - 1] = value;
        Ok(())
    }

    /// The channel slots, indexed 0-based.
    pub fn channels(&self) -> &[u8; CHANNELS] {
        &self.slots
    }

    /// The channel slots for direct mutation, indexed 0-based.
    pub fn channels_mut(&mut self) -> &mut [u8; CHANNELS] {
        &mut self.slots
    }

    /// The bytes placed on the wire for one frame: 512 channel slots, no start
    /// code. The firmware prepends the DMX start code itself (confirmed from the
    /// firmware disassembly), so all 512 bytes are channel data and channel `n`
    /// maps to slot `n`.
    pub fn frame(&self) -> &[u8] {
        &self.slots
    }
}

impl Default for Universe {
    fn default() -> Self {
        Self::new()
    }
}

/// Per-chunk header: a tag byte and a 16-bit little-endian destination offset.
const CHUNK_HEADER: usize = 3;

/// Largest channel span per chunk: the 64-byte packet less the 3-byte header.
const MAX_CHUNK_DATA: usize = 64 - CHUNK_HEADER;

/// Tag for a fill chunk: data lands in the frame buffer, not yet committed.
const TAG_FILL: u8 = 0x30;

/// Tag for the committing chunk: writes its data, then swaps the double buffer
/// and releases the frame for transmission.
const TAG_COMMIT: u8 = 0x31;

// NOTE: tag 0x32 also commits, but its distinction from 0x31 (most likely
// one-shot vs continuous retransmit) is unconfirmed in the firmware, so this
// path uses 0x31 for the committing chunk and never emits 0x32.

/// Send one universe as a sequence of tagged chunks.
///
/// The firmware reassembles its 512-channel buffer from chunks written to the DMX
/// endpoint: each packet is a [`CHUNK_HEADER`]-byte header (tag, then the 16-bit
/// little-endian destination offset) followed by up to [`MAX_CHUNK_DATA`] channel
/// bytes. Every span but the last is a fill chunk ([`TAG_FILL`]); the final span
/// commits the frame ([`TAG_COMMIT`]), which swaps the double buffer and triggers
/// transmission. Channel data only — the firmware inserts the DMX start code.
///
/// Call once per refresh; a full universe is ~9 chunk writes.
pub async fn send<T: Transport>(
    transport: &mut T,
    endpoint: u8,
    universe: &Universe,
) -> Result<(), Error<T::Error>> {
    let frame = universe.frame();
    let mut offset = 0;
    while offset < frame.len() {
        let end = (offset + MAX_CHUNK_DATA).min(frame.len());
        let span = &frame[offset..end];
        // The last span commits the frame; every earlier one only fills.
        let tag = if end == frame.len() { TAG_COMMIT } else { TAG_FILL };

        let mut chunk = [0u8; 64];
        chunk[0] = tag;
        chunk[1] = offset as u8; // offset low
        chunk[2] = (offset >> 8) as u8; // offset high
        chunk[CHUNK_HEADER..CHUNK_HEADER + span.len()].copy_from_slice(span);

        transport
            .bulk_out(endpoint, &chunk[..CHUNK_HEADER + span.len()])
            .await
            .map_err(Error::Transport)?;

        offset = end;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};

    fn block_on<F: Future>(fut: F) -> F::Output {
        let mut cx = Context::from_waker(Waker::noop());
        let mut fut = pin!(fut);
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// Records each chunk and reassembles the frame the way the firmware would,
    /// so a test can assert both the framing and the reconstructed universe.
    struct Recorder {
        count: usize,
        tags: [u8; 16],
        offsets: [u16; 16],
        endpoints: [u8; 16],
        rebuilt: [u8; CHANNELS],
    }

    impl Recorder {
        fn new() -> Self {
            Self {
                count: 0,
                tags: [0; 16],
                offsets: [0; 16],
                endpoints: [0; 16],
                rebuilt: [0; CHANNELS],
            }
        }
    }

    impl Transport for Recorder {
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
        async fn bulk_in(&mut self, _: u8, _: &mut [u8]) -> Result<usize, ()> {
            Ok(0)
        }
        async fn bulk_out(&mut self, endpoint: u8, data: &[u8]) -> Result<(), ()> {
            let offset = data[1] as usize | (data[2] as usize) << 8;
            let span = &data[CHUNK_HEADER..];
            self.rebuilt[offset..offset + span.len()].copy_from_slice(span);
            let i = self.count;
            self.tags[i] = data[0];
            self.offsets[i] = offset as u16;
            self.endpoints[i] = endpoint;
            self.count += 1;
            Ok(())
        }
    }

    #[test]
    fn chunks_a_universe_into_fill_then_commit() {
        let mut u = Universe::new();
        u.set(1, 255).unwrap(); // channel 1 -> slot 0
        u.set(512, 42).unwrap(); // channel 512 -> slot 511

        let mut rec = Recorder::new();
        block_on(send(&mut rec, DEFAULT_OUT_ENDPOINT, &u)).unwrap();

        // 512 bytes in 61-byte spans -> 9 chunks.
        assert_eq!(rec.count, 9);
        // All but the last are fill chunks; the last commits.
        assert!(rec.tags[..8].iter().all(|&t| t == TAG_FILL));
        assert_eq!(rec.tags[8], TAG_COMMIT);
        // Offsets tile 0..512 in MAX_CHUNK_DATA steps.
        for i in 0..9 {
            assert_eq!(rec.offsets[i] as usize, i * MAX_CHUNK_DATA);
        }
        // Every chunk went to the DMX endpoint.
        assert!(rec.endpoints[..9].iter().all(|&e| e == 0x02));
        // The reassembled buffer matches the universe (start-code free).
        assert_eq!(&rec.rebuilt[..], u.frame());
        assert_eq!(rec.rebuilt[0], 255);
        assert_eq!(rec.rebuilt[511], 42);
    }
}
