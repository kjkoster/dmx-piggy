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

/// Vendor mode-command opcode (byte 0 of the command; byte 1 is the mode).
const MODE_COMMAND: u8 = 0x01;

/// Bulk OUT endpoint that carries vendor commands — the same single live OUT
/// endpoint the DMX chunks use (see [`crate::dmx::DEFAULT_OUT_ENDPOINT`]).
pub(crate) const COMMAND_ENDPOINT: u8 = 0x02;

/// The widget's operating mode, selected with the mode command.
///
/// At power-up the device boots into [`Mode::Stop`]: the RS-485 driver is
/// disabled and the frame engine is off, so it drives nothing in either
/// direction until told to — good manners on a shared bus. Streaming DMX
/// therefore requires entering [`Mode::Transmit`] first (see [`Widget::start`]);
/// it is the symmetric partner of [`Mode::Receive`], not an automatic state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Idle: RS-485 driver disabled, frame engine off. The boot state.
    Stop = 0,
    /// Transmit: asserts the RS-485 driver and runs the DMX frame engine.
    Transmit = 1,
    /// Receive: enables the UART receiver; the driver stays disabled.
    Receive = 2,
}

/// A loaded widget, ready to stream DMX.
///
/// Construct one only for a device that has already re-enumerated as loaded
/// (see [`probe`]); it does not itself upload firmware.
pub struct Widget<T> {
    transport: T,
}

impl<T: Transport> Widget<T> {
    /// Wrap a loaded widget's transport.
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    /// Enter an operating [`Mode`].
    ///
    /// The device boots idle and drives nothing; you must select a mode before it
    /// does anything on the DMX line.
    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<T::Error>> {
        send_mode(&mut self.transport, mode).await
    }

    /// Enter transmit mode — shorthand for [`set_mode`](Self::set_mode) with
    /// [`Mode::Transmit`].
    ///
    /// Call once before streaming frames with [`send`](Self::send). Without it the
    /// RS-485 driver-enable never goes high and no DMX reaches the wire, however
    /// correct the frames are.
    pub async fn start(&mut self) -> Result<(), Error<T::Error>> {
        self.set_mode(Mode::Transmit).await
    }

    /// Return the device to the idle/stop state (releases the bus).
    pub async fn stop(&mut self) -> Result<(), Error<T::Error>> {
        self.set_mode(Mode::Stop).await
    }

    /// Send one universe frame. Call at the desired refresh rate, after [`start`](Self::start).
    pub async fn send(&mut self, universe: &Universe) -> Result<(), Error<T::Error>> {
        send(&mut self.transport, DEFAULT_OUT_ENDPOINT, universe).await
    }

    /// Recover the wrapped transport.
    pub fn release(self) -> T {
        self.transport
    }
}

/// Write the mode command on the command endpoint.
///
/// Shared by [`Widget`] (transmit) and [`crate::receive::Receiver`] (receive):
/// the device has a single mode register that selects the RS-485 direction, so
/// both ends drive it through this one command.
pub(crate) async fn send_mode<T: Transport>(
    transport: &mut T,
    mode: Mode,
) -> Result<(), Error<T::Error>> {
    transport
        .bulk_out(COMMAND_ENDPOINT, &[MODE_COMMAND, mode as u8])
        .await
        .map_err(Error::Transport)
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

    /// Captures the last bulk OUT write so a test can inspect the mode command.
    struct Recorder {
        endpoint: u8,
        data: [u8; 8],
        len: usize,
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
            self.endpoint = endpoint;
            self.len = data.len();
            self.data[..data.len()].copy_from_slice(data);
            Ok(())
        }
    }

    #[test]
    fn start_enters_transmit_mode_on_the_command_endpoint() {
        let rec = Recorder { endpoint: 0, data: [0; 8], len: 0 };
        let mut widget = Widget::new(rec);
        block_on(widget.start()).unwrap();
        let rec = widget.release();
        assert_eq!(rec.endpoint, 0x02);
        assert_eq!(&rec.data[..rec.len], &[0x01, 0x01]); // opcode, Mode::Transmit
    }

    #[test]
    fn stop_selects_mode_zero() {
        let rec = Recorder { endpoint: 0, data: [0; 8], len: 0 };
        let mut widget = Widget::new(rec);
        block_on(widget.stop()).unwrap();
        let rec = widget.release();
        assert_eq!(&rec.data[..rec.len], &[0x01, 0x00]); // opcode, Mode::Stop
    }
}
