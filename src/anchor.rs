//! EZ-USB "Anchor" RAM download for the AN2131.
//!
//! The bootloader accepts a vendor request that writes bytes straight into 8051
//! RAM, addressed by `wValue`. Uploading the image is therefore just a reset,
//! a run of such writes, and a release — no handshake or authentication is
//! involved.

use crate::error::Error;
use crate::firmware::{HexReader, RecordKind, MAX_RECORD_PAYLOAD};
use crate::transport::Transport;

/// Vendor request that writes bytes into 8051 RAM (`wValue` = target address).
const ANCHOR_LOAD: u8 = 0xA0;

/// `bmRequestType` for a host-to-device vendor transfer.
const VENDOR_OUT: u8 = 0x40;

/// AN2131 CPU control/status register; bit 0 holds the 8051 in reset.
///
/// Confirmed present in the vendor's own uploader. Bracketing the download in
/// reset is what makes it safe: the CPU is stopped before its RAM is rewritten
/// and only started once the whole image is in place.
const CPUCS: u16 = 0x7F92;
const CPU_HOLD: [u8; 1] = [0x01];
const CPU_RUN: [u8; 1] = [0x00];

/// Upload an Intel HEX `firmware` image into an unloaded widget and start the CPU.
///
/// On success the device drops off the bus and re-enumerates with
/// [`crate::identity::PID_LOADED`]. Waiting for that re-enumeration is the
/// caller's responsibility, because how an attach event surfaces depends on the
/// host stack.
pub async fn upload<T: Transport>(
    transport: &mut T,
    firmware: &[u8],
) -> Result<(), Error<T::Error>> {
    transport
        .control_out(VENDOR_OUT, ANCHOR_LOAD, CPUCS, 0, &CPU_HOLD)
        .await
        .map_err(Error::Transport)?;

    let mut reader = HexReader::new(firmware);
    let mut payload = [0u8; MAX_RECORD_PAYLOAD];
    while let Some(record) = reader.next(&mut payload) {
        let record = record.map_err(Error::Hex)?;
        match record.kind {
            RecordKind::Data => {
                transport
                    .control_out(
                        VENDOR_OUT,
                        ANCHOR_LOAD,
                        record.address,
                        0,
                        &payload[..record.len as usize],
                    )
                    .await
                    .map_err(Error::Transport)?;
            }
            RecordKind::Eof => break,
            // The Mk3 image is a flat sub-64K load, so extended-address records
            // are not expected; tolerate rather than reject them.
            RecordKind::Other(_) => {}
        }
    }

    transport
        .control_out(VENDOR_OUT, ANCHOR_LOAD, CPUCS, 0, &CPU_RUN)
        .await
        .map_err(Error::Transport)?;
    Ok(())
}
