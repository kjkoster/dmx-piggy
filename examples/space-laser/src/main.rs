//! Embassy application that drives a JB Systems Space-4 laser through the widget,
//! from Linux user space, with a `nusb` [`Transport`]. Cloned from the
//! `piggy-embassy` example; the USB/firmware/transport plumbing is identical, and
//! only the DMX frame content differs — here it paints a diagonal line by walking
//! the laser's X and Y position channels together, one step per frame.
//!
//! The crate's async driver runs on the Embassy executor unchanged; the transport
//! bridges to `nusb`'s *blocking* calls, since `nusb`'s async path requires a
//! tokio/smol reactor that Embassy does not provide. Each USB transfer briefly
//! blocks the (single-threaded) executor, which is fine for a one-widget streamer.
//!
//! Channel map (from JB-Systems-Space-4-Laser.txt, 8-channel mode):
//!   CH1 MODE (must be ≥192 for DMX mode), CH2 PATTERN, CH3 ZOOM,
//!   CH4 Y-ROLL, CH5 X-ROLL, CH6 Z-ROTATE, CH7 X-MOVE, CH8 Y-MOVE.
//! On CH7/CH8, values 0–127 are 128 fixed positions on that axis; 128+ switch to
//! auto rolling motion, which we avoid.
//!
//! Configure at runtime via the environment: `SPACE_LASER_4_ADDRESS` (DMX start
//! address, default 1), `SPACE_LASER_PATTERN` (CH2 pattern, default 0), and
//! `SPACE_LASER_ZOOM` (CH3 size percent 5–100, default 100).
//!
//! Calibration note: this unit's CH7/CH8 value→position curve is a triangle —
//! value 32 is the top-right end of the diagonal, 96 the bottom-left — so we sweep
//! only 32..=96 (the monotonic segment) to paint a clean line. See `POS_MIN`/`POS_MAX`.

use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration as StdDuration;

use anyhow::{bail, Context, Result};
use dmx_piggy::identity::{PID_LOADED, PID_UNLOADED, VID};
use dmx_piggy::{
    device,
    dmx::{Universe, CHANNELS},
    identity::State,
    Transport, Widget,
};
use embassy_executor::Executor;
use embassy_time::{Duration, Ticker, Timer};
use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Out, Recipient, TransferError};
use nusb::MaybeFuture;
use static_cell::StaticCell;

/// DMX wants a steady refresh even when nothing changes. ~44 Hz is the fastest
/// DMX-512 allows: a full 512-slot frame at 250 kbit/s takes ~22.7 ms.
const REFRESH: Duration = Duration::from_micros(22_727);

/// Per-transfer timeout for control transfers.
const USB_TIMEOUT: StdDuration = StdDuration::from_millis(500);

/// Short timeout for bulk IN status reads. An idle status endpoint NAKs until
/// this elapses, at which point we treat the read as "nothing available".
const USB_IN_TIMEOUT: StdDuration = StdDuration::from_millis(100);

/// Environment variable naming the laser's DMX start address (CH1 lands there).
const ADDRESS_VAR: &str = "SPACE_LASER_4_ADDRESS";
/// Environment variable naming the pattern number sent on CH2.
const PATTERN_VAR: &str = "SPACE_LASER_PATTERN";
/// Environment variable naming the CH3 zoom/size percentage (5–100).
const ZOOM_VAR: &str = "SPACE_LASER_ZOOM";

/// Default DMX start address when `SPACE_LASER_4_ADDRESS` is unset.
///
/// The datasheet lists a valid starting-address range of **001–505** (505 =
/// 512 − 8 + 1, the highest start that still fits the fixture's 8 channels) and
/// names no explicit factory default, so we use address **1**, the conventional
/// default. The channel numbers (CH1..CH8) are derived from the chosen address
/// at run time, since it is no longer known at compile time.
const DEFAULT_ADDRESS: u16 = 1;

/// Default pattern on CH2 when `SPACE_LASER_PATTERN` is unset (the first pattern).
const DEFAULT_PATTERN: u8 = 0;

/// Default zoom percentage when `SPACE_LASER_ZOOM` is unset: 100% size (CH3 = 0).
const DEFAULT_ZOOM_PCT: u8 = 100;

/// CH1 value that selects full 8-channel DMX mode (datasheet: 192–255). Below
/// this the fixture ignores CH2–CH8 and runs its own auto/sound/blackout shows.
const MODE_DMX: u8 = 255;

/// The CH7/CH8 sweep bounces between these two values. This unit's value→position
/// curve is a triangle — value 32 sits at the top-right end of the diagonal and 96
/// at the bottom-left — so 32..=96 is the monotonic segment that paints the whole
/// diagonal without the mid-sweep fold. (0..=127 are the fixed positions; 128+
/// selects a *moving* mode, which we avoid.) Measured empirically on the hardware.
const POS_MIN: u8 = 32;
const POS_MAX: u8 = 96;

// The firmware is not shipped with this crate; point DMX_WIDGET_FIRMWARE at your
// own image at build time. It is embedded into the binary, so the running program
// needs no firmware file on disk. Do not commit the image.
const FIRMWARE: &[u8] = include_bytes!(env!(
    "DMX_WIDGET_FIRMWARE",
    "set DMX_WIDGET_FIRMWARE to the path of your widget firmware (.bin, Intel HEX)"
));
const _: () = assert!(
    dmx_piggy::firmware::is_intel_hex(FIRMWARE),
    "DMX_WIDGET_FIRMWARE must point at an Intel HEX image"
);
const _: () = assert!(FIRMWARE.len() > 32, "firmware image is implausibly small");

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

/// Set by the Ctrl-C / SIGTERM handler so the paint loop can break out and put
/// the laser back to idle, instead of leaving it stuck in transmit mode with the
/// LEDs blinking in panic.
static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() {
    // Default to info so the lifecycle and sweep-config lines show without the user
    // having to set RUST_LOG; RUST_LOG still overrides when set.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| spawner.spawn(run()).unwrap());
}

#[embassy_executor::task]
async fn run() {
    // The Embassy executor never returns, so exit the process explicitly once the
    // driver is done — cleanly on Ok (after the laser was stopped), non-zero on error.
    match drive().await {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            log::error!("{e:#}");
            std::process::exit(1);
        }
    }
}

async fn drive() -> Result<()> {
    // Runtime configuration from the environment: DMX start address, CH2 pattern,
    // and CH3 zoom (given as a size percentage, mapped onto the DMX value below).
    let address = env_num(ADDRESS_VAR, DEFAULT_ADDRESS)?;
    let pattern = env_num(PATTERN_VAR, DEFAULT_PATTERN)?;
    let zoom_pct = env_num(ZOOM_VAR, DEFAULT_ZOOM_PCT)?;
    if !(5..=100).contains(&zoom_pct) {
        bail!("{ZOOM_VAR}={zoom_pct} out of range (valid 5..=100 percent)");
    }
    let zoom = zoom_pct_to_dmx(zoom_pct);
    log::info!("laser config: DMX address {address}, pattern {pattern}, zoom {zoom_pct}% (CH3={zoom})");

    // The 8-channel block must fit inside the 512-slot universe; reject a start
    // address that would push CH8 past the end before we touch the hardware.
    if address == 0 || address as usize + 7 > CHANNELS {
        bail!("{ADDRESS_VAR}={address} leaves no room for 8 channels (valid range 1..=505)");
    }

    // The fixture's eight channels, as 1-based DMX addresses from the start address.
    let ch_mode = address; // CH1: working mode
    let ch_pattern = address + 1; // CH2: pattern select
    let ch_zoom = address + 2; // CH3: zoom / size
    let ch_y_roll = address + 3; // CH4: Y-axis rolling
    let ch_x_roll = address + 4; // CH5: X-axis rolling
    let ch_z_rotate = address + 5; // CH6: Z-axis rotating
    let ch_x_move = address + 6; // CH7: X-axis position (0–127 = fixed)
    let ch_y_move = address + 7; // CH8: Y-axis position (0–127 = fixed)

    // Trap Ctrl-C / SIGTERM. The handler only flips a flag; the paint loop sees it
    // and calls widget.stop() so the laser is never left transmitting.
    ctrlc::set_handler(|| SHUTDOWN.store(true, Ordering::SeqCst))
        .context("installing shutdown handler")?;

    // The firmware services exactly one bulk OUT endpoint (0x02); 0x04 stalls and
    // 0x05 is silently dropped. Settled from the decompilation — see USB-PATHS.md.
    let endpoint = dmx_piggy::dmx::DEFAULT_OUT_ENDPOINT;
    log::info!("output config: endpoint {endpoint:#04x}");

    // Open whichever enumeration stage is currently on the bus.
    let mut host = PiUsbHost::open().context("no widget found on USB")?;

    match device::probe(&mut host).await {
        Ok(State::Unloaded) => {
            log::info!("bootloader present; uploading firmware");
            dmx_piggy::upload(&mut host, FIRMWARE)
                .await
                .map_err(|e| anyhow::anyhow!("firmware upload failed: {e:?}"))?;
            // The device now drops off the bus and returns as the loaded product.
            // On a hosted target we simply wait for it to re-enumerate and reopen
            // it — the very step the bare-metal examples have to leave as a TODO.
            log::info!("waiting for the device to re-enumerate as the loaded product");
            host = PiUsbHost::wait_for(PID_LOADED)
                .await
                .context("device did not re-enumerate after upload")?;
        }
        Ok(State::Loaded) => log::info!("firmware already loaded"),
        Ok(other) => bail!("unexpected device state: {other:?}"),
        Err(e) => bail!("probe failed: {e:?}"),
    }

    // Bring-up identity round-trip: a cheap, input-independent vendor query that
    // proves the command/response transport before any DMX goes out.
    match dmx_piggy::read_serial(&mut host).await {
        Ok(serial) => log::info!("widget serial: {serial}"),
        Err(e) => log::warn!("serial query failed: {e:?}"),
    }
    match dmx_piggy::read_id(&mut host).await {
        Ok(id) => log::info!("widget id: 0x{}", id.iter().map(|b| format!("{b:02x}")).collect::<String>()),
        Err(e) => log::warn!("id query failed: {e:?}"),
    }

    let mut widget = Widget::new(host);

    // The widget boots idle (driver disabled, frame engine off). Enter transmit
    // mode before streaming, or nothing reaches the XLR however correct the frames.
    widget
        .start()
        .await
        .map_err(|e| anyhow::anyhow!("failed to enter transmit mode: {e:?}"))?;
    log::info!("entered transmit mode");

    // Set the constant part of the laser frame once: DMX mode on, the chosen
    // pattern, full size, and no auto roll/rotate so only our X/Y moves the beam.
    // A single-dot/beam pattern paints a clean line; set SPACE_LASER_PATTERN to one
    // from the datasheet's pattern list if the default draws more than a point.
    let mut universe = Universe::new();
    universe.set(ch_mode, MODE_DMX).expect("CH1 in range");
    universe.set(ch_pattern, pattern).expect("CH2 in range");
    universe.set(ch_zoom, zoom).expect("CH3 in range");
    universe.set(ch_y_roll, 0).expect("CH4 in range");
    universe.set(ch_x_roll, 0).expect("CH5 in range");
    universe.set(ch_z_rotate, 0).expect("CH6 in range");

    log::info!(
        "painting a diagonal (pattern {pattern}) on laser at DMX address {address} (CH7/CH8) over {POS_MIN}..={POS_MAX}, streaming at ~{} Hz",
        1_000_000 / REFRESH.as_micros().max(1)
    );

    // The beam position: x and y walk together between POS_MIN and POS_MAX, tracing
    // the diagonal and bouncing back at each end, one step per frame.
    let mut x: u8 = POS_MIN;
    let mut y: u8 = POS_MIN;
    let mut dir: i8 = 1;

    let mut ticker = Ticker::every(REFRESH);
    loop {
        if SHUTDOWN.load(Ordering::Relaxed) {
            break;
        }
        universe.set(ch_x_move, x).expect("CH7 in range");
        universe.set(ch_y_move, y).expect("CH8 in range");
        widget
            .send(&universe)
            .await
            .map_err(|e| anyhow::anyhow!("DMX send failed: {e:?}"))?;

        // Step the diagonal: reverse at either end, then move both axes one slot.
        if (dir > 0 && (x >= POS_MAX || y >= POS_MAX)) || (dir < 0 && (x <= POS_MIN || y <= POS_MIN)) {
            dir = -dir;
        }
        x = (x as i8 + dir) as u8;
        y = (y as i8 + dir) as u8;

        ticker.next().await;
    }

    // Left the loop on a shutdown signal. Before releasing the bus, black the
    // laser out: an all-zero frame puts CH1 in the 000–063 "Laser Block Out" band,
    // so the beam goes dark. Send it three times, one refresh apart, so the laser
    // latches the blackout even if a frame is dropped — otherwise stopping the
    // widget mid-pattern can leave the beam on with the LEDs blinking in panic.
    log::info!("shutdown requested; blacking out the laser");
    let blackout = Universe::new();
    for _ in 0..3 {
        widget
            .send(&blackout)
            .await
            .map_err(|e| anyhow::anyhow!("blackout send failed: {e:?}"))?;
        Timer::after(REFRESH).await;
    }

    // Now return the widget to idle so it stops driving the DMX line.
    log::info!("stopping the laser and releasing the bus");
    widget
        .stop()
        .await
        .map_err(|e| anyhow::anyhow!("failed to stop the widget: {e:?}"))?;
    log::info!("laser stopped cleanly");
    Ok(())
}

/// USB host transport for the widget, backed by `nusb`.
///
/// This is the single per-platform integration point, and on Linux it is fully
/// realised. It claims interface 0 for control transfers and opens the DMX bulk
/// OUT endpoint lazily, on first use.
struct PiUsbHost {
    interface: nusb::Interface,
    dmx_out: Option<nusb::Endpoint<Bulk, Out>>,
    /// One frame buffer, allocated once and handed back and forth to the endpoint
    /// so streaming does no per-frame allocation. `None` only while it is in
    /// flight inside a `bulk_out` call.
    frame: Option<Buffer>,
}

impl PiUsbHost {
    /// Open whichever widget is attached now, in either enumeration stage.
    fn open() -> Result<Self> {
        for pid in [PID_UNLOADED, PID_LOADED] {
            if let Some(host) = Self::try_open(pid)? {
                return Ok(host);
            }
        }
        bail!("no widget ({VID:#06x}) on the USB bus");
    }

    /// Poll the bus until a device with `pid` appears, then open it. Used to
    /// catch the loaded device as it re-enumerates after a firmware upload.
    async fn wait_for(pid: u16) -> Result<Self> {
        for _ in 0..50 {
            if let Some(host) = Self::try_open(pid)? {
                return Ok(host);
            }
            Timer::after(Duration::from_millis(100)).await;
        }
        bail!("timed out waiting for {VID:#06x}:{pid:#06x} to appear");
    }

    fn try_open(pid: u16) -> Result<Option<Self>> {
        let Some(info) = nusb::list_devices()
            .wait()?
            .find(|d| d.vendor_id() == VID && d.product_id() == pid)
        else {
            return Ok(None);
        };
        let device = info.open().wait().context("opening device")?;
        let interface = device
            .claim_interface(0)
            .wait()
            .context("claiming interface 0")?;
        Ok(Some(Self {
            interface,
            dmx_out: None,
            frame: None,
        }))
    }
}

/// Read a numeric environment variable, falling back to `default` when it is
/// unset. A present-but-unparseable value is a hard error rather than being
/// silently ignored, so a typo in the config surfaces instead of misconfiguring
/// the laser.
fn env_num<T>(name: &str, default: T) -> Result<T>
where
    T: FromStr,
    T::Err: std::fmt::Display,
{
    match std::env::var(name) {
        Ok(s) => s
            .trim()
            .parse::<T>()
            .map_err(|e| anyhow::anyhow!("{name}: invalid value {s:?}: {e}")),
        Err(std::env::VarError::NotPresent) => Ok(default),
        Err(std::env::VarError::NotUnicode(_)) => bail!("{name}: value is not valid UTF-8"),
    }
}

/// Map a zoom percentage (5–100) onto CH3's 0–127 "size" band. Per the datasheet
/// that band runs 100% → 0 down to 5% → 127, linearly, so a larger percentage is
/// a *smaller* DMX value. Returns the DMX value (0..=127). The caller must ensure
/// `pct` is in 5..=100; outside that the subtraction below would misbehave.
fn zoom_pct_to_dmx(pct: u8) -> u8 {
    // value = round((100 - pct) / 95 * 127), in integer math (+47 ≈ 95/2 rounds).
    (((100 - pct as u16) * 127 + 47) / 95) as u8
}

/// Split a raw `bmRequestType` byte into the `type` and `recipient` fields that
/// `nusb` models separately (direction is implied by `control_in`/`control_out`).
fn control_type(request_type: u8) -> ControlType {
    match (request_type >> 5) & 0b11 {
        0 => ControlType::Standard,
        1 => ControlType::Class,
        // The widget speaks only vendor requests; treat the reserved value as one.
        _ => ControlType::Vendor,
    }
}

fn recipient(request_type: u8) -> Recipient {
    match request_type & 0x1f {
        0 => Recipient::Device,
        1 => Recipient::Interface,
        2 => Recipient::Endpoint,
        _ => Recipient::Other,
    }
}

impl Transport for PiUsbHost {
    type Error = anyhow::Error;

    async fn control_out(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<()> {
        self.interface
            .control_out(
                ControlOut {
                    control_type: control_type(request_type),
                    recipient: recipient(request_type),
                    request,
                    value,
                    index,
                    data,
                },
                USB_TIMEOUT,
            )
            .wait()?;
        Ok(())
    }

    async fn control_in(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        buf: &mut [u8],
    ) -> Result<usize> {
        let got = self
            .interface
            .control_in(
                ControlIn {
                    control_type: control_type(request_type),
                    recipient: recipient(request_type),
                    request,
                    value,
                    index,
                    length: buf.len() as u16,
                },
                USB_TIMEOUT,
            )
            .wait()?;
        let n = got.len().min(buf.len());
        buf[..n].copy_from_slice(&got[..n]);
        Ok(n)
    }

    async fn bulk_out(&mut self, endpoint: u8, data: &[u8]) -> Result<()> {
        // Open the OUT endpoint and allocate the frame buffer once; every frame
        // thereafter reuses both, so streaming allocates nothing — keeping to the
        // crate's no_alloc spirit even on a hosted target.
        if self.dmx_out.is_none() {
            self.dmx_out = Some(self.interface.endpoint::<Bulk, Out>(endpoint)?);
            self.frame = Some(Buffer::new(CHANNELS));
        }
        let ep = self.dmx_out.as_mut().expect("just populated");
        let mut buf = self.frame.take().expect("frame buffer is reclaimed after each send");
        buf.clear();
        buf.extend_from_slice(data);
        // Blocking transfer: no async reactor is involved, so this works under the
        // Embassy executor. Reclaim the buffer from the completion before surfacing
        // any error, so a failed transfer does not leak it and leave `frame` empty.
        let completion = ep.transfer_blocking(buf, USB_TIMEOUT);
        self.frame = Some(completion.buffer);
        completion.status?;
        Ok(())
    }

    async fn bulk_in(&mut self, endpoint: u8, buf: &mut [u8]) -> Result<usize> {
        // A fresh endpoint handle per call: bulk_in is only used out of band (at
        // bring-up, or after a command), never in the refresh loop, so this is not
        // a hot path and need not be cached.
        let mut ep = self.interface.endpoint::<Bulk, In>(endpoint)?;
        let completion = ep.transfer_blocking(Buffer::new(buf.len()), USB_IN_TIMEOUT);
        match completion.status {
            Ok(()) => {
                let got = &completion.buffer[..];
                let n = got.len().min(buf.len());
                buf[..n].copy_from_slice(&got[..n]);
                Ok(n)
            }
            // The read timed out with nothing queued (the endpoint NAKed throughout).
            // For a status/response channel that is normal, not an error.
            Err(TransferError::Cancelled) => Ok(0),
            // A stall or disconnect is a genuine failure.
            Err(e) => Err(e.into()),
        }
    }
}
