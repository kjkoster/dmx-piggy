//! High-level driver over a loaded widget, plus state detection.

use crate::dmx::{send, Universe, DEFAULT_OUT_ENDPOINT};
use crate::error::Error;
use crate::identity::{classify, State};
use crate::transport::Transport;

/// Standard `GET_DESCRIPTOR` request.
const GET_DESCRIPTOR: u8 = 0x06;
/// `wValue` selecting the device descriptor (type 1, index 0).
const DESC_DEVICE: u16 = 0x0100;
/// `bmRequestType` for a standard device-to-host request.
const STANDARD_IN: u8 = 0x80;
/// A device descriptor is fixed at 18 bytes; `idVendor`/`idProduct` live at 8/10.
const DEVICE_DESCRIPTOR_LEN: usize = 18;

/// Read the device descriptor and classify which enumeration state it is in.
///
/// Use this to decide whether a firmware upload is still required before
/// streaming.
pub async fn probe<T: Transport>(transport: &mut T) -> Result<State, Error<T::Error>> {
    let mut descriptor = [0u8; DEVICE_DESCRIPTOR_LEN];
    let read = transport
        .control_in(STANDARD_IN, GET_DESCRIPTOR, DESC_DEVICE, 0, &mut descriptor)
        .await
        .map_err(Error::Transport)?;
    if read < DEVICE_DESCRIPTOR_LEN {
        return Err(Error::ShortDescriptor);
    }
    let vid = u16::from_le_bytes([descriptor[8], descriptor[9]]);
    let pid = u16::from_le_bytes([descriptor[10], descriptor[11]]);
    classify(vid, pid).ok_or(Error::NotAWidget)
}

/// A loaded widget, ready to stream DMX.
///
/// Construct one only for a device that has already re-enumerated as loaded
/// (see [`probe`]); it does not itself upload firmware.
pub struct Widget<T> {
    transport: T,
    endpoint: u8,
}

impl<T: Transport> Widget<T> {
    /// Wrap a transport, assuming the default output endpoint.
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            endpoint: DEFAULT_OUT_ENDPOINT,
        }
    }

    /// Wrap a transport with an explicit output endpoint.
    ///
    /// Useful while the correct endpoint is still being confirmed against
    /// [`crate::dmx::OUT_ENDPOINT_CANDIDATES`].
    pub fn with_endpoint(transport: T, endpoint: u8) -> Self {
        Self { transport, endpoint }
    }

    /// Send one universe frame. Call at the desired refresh rate.
    pub async fn send(&mut self, universe: &Universe) -> Result<(), Error<T::Error>> {
        send(&mut self.transport, self.endpoint, universe).await
    }

    /// Recover the wrapped transport.
    pub fn release(self) -> T {
        self.transport
    }
}
