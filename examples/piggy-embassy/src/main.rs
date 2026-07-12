//! Embassy application driving the widget from Linux user space, with a `nusb`
//! [`Transport`]. See the README for what it does and how to run it.
//!
//! The crate's async driver runs on the Embassy executor unchanged; the transport
//! bridges to `nusb`'s *blocking* calls, since `nusb`'s async path requires a
//! tokio/smol reactor that Embassy does not provide. Each USB transfer briefly
//! blocks the (single-threaded) executor, which is fine for a one-widget streamer.

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
use embassy_time::{Duration, Instant, Ticker, Timer};
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

/// Set by the Ctrl-C / SIGTERM handler so the stream loop can break out and put
/// the widget back to idle, instead of leaving it stuck in transmit mode with the
/// LEDs blinking in panic.
static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() {
    // Default to info so the lifecycle and sweep-config lines show without the user
    // having to set RUST_LOG; RUST_LOG still overrides when set.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let executor = EXECUTOR.init(Executor::new());
    // embassy-executor 0.10: the `#[task]` fn returns `Result<SpawnToken, _>`
    // (pool allocation happens here) and `spawn` consumes the token infallibly, so
    // the `.unwrap()` sits on `run()`, not on `spawn(..)`.
    executor.run(|spawner| spawner.spawn(run().unwrap()));
}

#[embassy_executor::task]
async fn run() {
    // The Embassy executor never returns, so exit the process explicitly once the
    // driver is done — cleanly on Ok (after the widget was stopped), non-zero on error.
    match drive().await {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            log::error!("{e:#}");
            std::process::exit(1);
        }
    }
}

async fn drive() -> Result<()> {
    // Trap Ctrl-C / SIGTERM. The handler only flips a flag; the stream loop sees it
    // and calls widget.stop() so the device is never left transmitting.
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

    let mut universe = Universe::new();
    universe.set(1, 255).expect("channel 1 is in range");

    log::info!("streaming DMX at ~{} Hz", 1_000_000 / REFRESH.as_micros().max(1));
    let mut ticker = Ticker::every(REFRESH);
    let mut frames: u64 = 0;
    let mut window: u32 = 0;
    let mut last_report = Instant::now();
    loop {
        if SHUTDOWN.load(Ordering::Relaxed) {
            break;
        }
        widget
            .send(&universe)
            .await
            .map_err(|e| anyhow::anyhow!("DMX send failed: {e:?}"))?;
        frames += 1;
        window += 1;
        // A steadily climbing counter is a live stream; a frozen one (with the
        // error above) is a stall. Report roughly every two seconds.
        let dt = last_report.elapsed();
        if dt >= Duration::from_secs(2) {
            let fps = window as u64 * 1000 / dt.as_millis().max(1);
            log::info!("streaming: {frames} frames sent (~{fps} fps)");
            window = 0;
            last_report = Instant::now();
        }
        ticker.next().await;
    }

    // Left the loop on a shutdown signal: return the widget to idle so it stops
    // driving the DMX line and does not sit blinking in panic.
    log::info!("shutdown requested; stopping DMX and releasing the bus");
    widget
        .stop()
        .await
        .map_err(|e| anyhow::anyhow!("failed to stop the widget: {e:?}"))?;
    log::info!("widget stopped cleanly");
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
