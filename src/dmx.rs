//! DMX-512 universe buffer and frame transmission.

use crate::error::Error;
use crate::transport::Transport;

/// Channels in one DMX-512 universe.
pub const CHANNELS: usize = 512;

/// The bulk OUT endpoints the loaded device exposes for output.
///
/// Three are advertised; which one carries the universe has not been pinned from
/// the firmware alone.
// TODO: confirm the DMX OUT endpoint. `0x02` (the first bulk OUT) is the leading
// candidate; resolve by tracing the 8051 transmit engine to the endpoint FIFO it
// drains, or by observing a fixture, then reduce this to a single constant.
pub const OUT_ENDPOINT_CANDIDATES: [u8; 3] = [0x02, 0x04, 0x05];

/// The endpoint assumed for output until confirmed.
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

    /// The bytes placed on the wire for one frame.
    // TODO: confirm whether the firmware prepends the DMX start code itself
    // (assumed here, so all 512 bytes are channel data) or expects it in the
    // first slot; that decision fixes the addressing in `set`.
    pub fn frame(&self) -> &[u8] {
        &self.slots
    }
}

impl Default for Universe {
    fn default() -> Self {
        Self::new()
    }
}

/// Send one universe frame to the given bulk OUT endpoint.
///
/// Transmission is one transfer per frame with no wire header; the device frames
/// and clocks out DMX on its own timing, so the caller need only repeat this at
/// the desired refresh rate.
pub async fn send<T: Transport>(
    transport: &mut T,
    endpoint: u8,
    universe: &Universe,
) -> Result<(), Error<T::Error>> {
    transport
        .bulk_out(endpoint, universe.frame())
        .await
        .map_err(Error::Transport)
}
