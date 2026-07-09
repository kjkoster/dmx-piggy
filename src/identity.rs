//! USB identities for the widget's two enumeration states.

/// Vendor ID shared by both states.
///
/// This value is self-assigned by the manufacturer and is not a USB-IF
/// allocation, so it must not be treated as globally unique.
pub const VID: u16 = 0x0CB0;

/// Product ID of the bootloader, before firmware is uploaded.
pub const PID_UNLOADED: u16 = 0x0001;

/// Product ID of the running device, after firmware is uploaded.
pub const PID_LOADED: u16 = 0x0002;

/// Which side of the two-stage enumeration a device is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Bootloader present; awaiting a firmware download.
    Unloaded,
    /// Firmware running; ready to stream DMX.
    Loaded,
    /// A `VID` device with a product ID this crate does not recognise.
    Unknown(u16),
}

/// Classify a device from its vendor and product IDs.
///
/// Returns `None` for anything that is not this vendor, so callers can reject
/// unrelated devices before acting on them.
pub const fn classify(vid: u16, pid: u16) -> Option<State> {
    if vid != VID {
        return None;
    }
    Some(match pid {
        PID_UNLOADED => State::Unloaded,
        PID_LOADED => State::Loaded,
        other => State::Unknown(other),
    })
}
