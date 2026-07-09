//! The host-side USB transport the driver runs on.
//!
//! This crate implements the widget's protocol but performs no USB host I/O of
//! its own, so it can sit on any stack. Implement this trait over your host
//! controller and the rest of the crate composes on top of it. The methods are
//! `async` to suit Embassy and other executors.

/// The USB host operations the driver needs.
pub trait Transport {
    /// Transport-specific failure: timeout, stall, disconnect, and so on.
    type Error;

    /// A control transfer whose data stage travels host-to-device (or is absent).
    async fn control_out(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<(), Self::Error>;

    /// A control transfer that reads into `buf`, returning the number of bytes read.
    async fn control_in(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        buf: &mut [u8],
    ) -> Result<usize, Self::Error>;

    /// Write `data` to a bulk OUT endpoint.
    async fn bulk_out(&mut self, endpoint: u8, data: &[u8]) -> Result<(), Self::Error>;
}
