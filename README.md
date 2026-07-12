# DMX Piggy

A `no_std`, `no_alloc` Rust driver for the Flying Pig Systems / High End **HOG
3PC USB DMX Widget**. That is that oddly-shaped blue box with the stabby feet.
The device is built around a Cypress EZ-USB AN2131, and turns your USB port into
a single DMX-512 universe on a 5-pin XLR.

The crate is transport-agnostic and Embassy-friendly: it implements the widget's
protocol and nothing else, so it runs on any USB host stack you can express as a
handful of `async` transfers. Due to license restrictions, Firmware is **not**
included in this repository. See [Bring your own
firmware](#bring-your-own-firmware).

Status: _early_. The firmware upload and DMX framing are implemented and the
output path (endpoint `0x02`) is settled from the decompilation (see
[Status](#status)).

---

## The Device From Outside

<!-- TODO: replace with better photographs. -->
![The widget, exterior](docs/img/outside.jpg)

A palm-sized enclosure with a USB Type-B socket at one end and a 5-pin male XLR
at the other. A small cluster of LEDs on the rear reports power and activity.
There are no switches, no configuration, and no external power input. It is
powered off of the USB bus.

Two things are worth knowing before anything else:

- It ships with **no firmware in it**. Freshly plugged in, it is an empty
  bootloader. The host must upload firmware on every cold connect.
- It therefore presents **two different USB identities** depending on that state
  (both under vendor ID `0x0CB0`): `0x0001` unloaded, `0x0002` loaded.

## The Device On The Inside

<!-- TODO: replace with better photographs. -->
![The widget, interior](docs/img/inside.jpg)

The board is unremarkable in the best way, and every part of it informs the
driver:

- **Cypress EZ-USB AN2131QC**: an 8051 with a USB core, and *no* non-volatile
  program store. Firmware is downloaded into RAM over USB and runs until power is
  removed.
- **32 KiB external SRAM** (ISSI IS62LV256): where the firmware image lives once
  uploaded; volatile, so nothing survives an unplug.
- **MAX483 RS-485 driver** into the XLR, behind **opto-isolators** and an
  isolated **DC-DC converter**: the DMX output is fully galvanically isolated
  from USB. The optocouplers are socketed, being the parts most likely to be
  sacrificed by a wiring accident.

The consequence of the missing flash is the whole shape of this crate: the widget
is useless until a host hands it firmware, and it forgets that firmware the moment
it loses power.

## Sample Code

```rust
use dmx_piggy::{device::probe, dmx::Universe, identity::State, Widget};

// `transport` is your implementation of `dmx_piggy::Transport`.
match probe(&mut transport).await? {
    State::Unloaded => {
        dmx_piggy::upload(&mut transport, FIRMWARE).await?;
        // ... wait for the device to re-enumerate, then rebind `transport` ...
    }
    State::Loaded => {}
    State::Unknown(pid) => panic!("unexpected product id {pid:#06x}"),
}

let mut widget = Widget::new(transport);
widget.start().await?;          // enter transmit mode (required before sending)
let mut universe = Universe::new();
universe.set(1, 255)?;          // channel 1 to full
widget.send(&universe).await?;  // repeat at ~40 Hz
```

---

## Use case 1 — Upload firmware

**In plain terms.** The box arrives empty. Before it can do anything you copy a
firmware file into it. That copy is lost when you unplug it, so your program does
it again next time — check first, upload only if needed.

**Technically.** The AN2131 bootloader implements the EZ-USB *Anchor download*: a
vendor control request (`bmRequestType 0x40`, `bRequest 0xA0`) writes a block of
bytes into 8051 RAM at the address given in `wValue`. [`upload`](src/anchor.rs):

1. writes `1` to the CPU control register (`CPUCS`, `0x7F92`) to hold the 8051 in
   reset;
2. streams the firmware, one Intel HEX record at a time, as `0xA0` writes;
3. writes `0` to `CPUCS` to release the CPU.

The image is Intel HEX (despite the vendor's `.bin` extension), so it is parsed
and checksummed record-by-record rather than decoded up front — a corrupt block
would otherwise run as corrupt code until the next power cycle. There is no
signature, challenge, or key anywhere in this exchange; the bootloader runs
whatever it is given.

After step 3 the device disconnects and returns as `0x0002`. Waiting for that
re-enumeration is left to the caller, because how an attach event surfaces is a
property of your host stack, not of the widget.

## Use case 2 — Send DMX

**In plain terms.** Once firmware is running, you first tell the box to start
transmitting (it boots idle and drives nothing until told), then send it a full
frame of 512 channel values, over and over, a few dozen times a second. The box
does the fiddly electrical timing of DMX itself.

**Technically.** The loaded device boots idle — the RS-485 driver is disabled and
the frame engine is off — so [`Widget::start`](src/device.rs) must first send the
mode command (`01 01`) to enter transmit mode; without it the driver-enable never
goes high and no DMX reaches the wire. Then a universe is delivered to endpoint
`0x02` as a sequence of tagged **chunks**: each packet is a 3-byte header (a tag, then a 16-bit little-endian
offset into the channel buffer) followed by up to 61 channel bytes. Fill chunks
(`0x30`) load the buffer; the final chunk (`0x31`) commits the frame, swapping the
firmware's double buffer and triggering transmission. Send channel data only — the
firmware inserts the DMX start code and does the electrical timing (break,
mark-after-break, byte framing). A full 512-channel universe is ~9 chunk writes,
repeated at your chosen refresh rate. [`Universe`](src/dmx.rs) is a fixed 512-byte
buffer with 1-based channel addressing; [`dmx::send`](src/dmx.rs) does the
chunking.

## Use case 3 — Status LEDs

**In plain terms.** The lights on the back look after themselves. Once firmware is
running and you are streaming, they indicate activity on their own. There is
nothing to switch on and nothing this crate needs to do.

**Technically.** The LEDs are driven by GPIO from the 8051, off internal state of
the firmware's transmit engine — not by the host and not by fixed hardware. There
is no host command to set them, so the crate deliberately exposes none. They are
useful bring-up feedback: if your framing and cadence are right, they show it.

---

## How to get this working

1. **Obtain firmware.** You need the widget's firmware image. See
   [Bring your own firmware](#bring-your-own-firmware) for what is and is not
   permitted.

2. **Add the crate.**
   ```toml
   [dependencies]
   dmx-piggy = "0.1"
   ```

3. **Implement [`Transport`](src/transport.rs)** for your USB host — four
   `async` methods: `control_out`, `control_in`, `bulk_out`, `bulk_in`.

4. **Embed the firmware** in your binary and validate it at build time (see
   [Embedding the firmware](#embedding-the-firmware)).

5. **Drive the lifecycle:** [`probe`] → [`upload`] if unloaded → wait for
   re-enumeration → [`Widget::start`] (enter transmit mode) → [`Widget::send`] in
   a loop.

### Embedding the firmware

The idiomatic way to compile a binary blob into a Rust program is
[`include_bytes!`], which yields a `&'static [u8]`. Point it at a path supplied
through the environment so the firmware stays out of the crate, and assert its
shape at compile time so a wrong path fails the build rather than the hardware:

```rust
const FIRMWARE: &[u8] = include_bytes!(env!("DMX_WIDGET_FIRMWARE"));

const _: () = assert!(
    dmx_piggy::firmware::is_intel_hex(FIRMWARE),
    "DMX_WIDGET_FIRMWARE must point at an Intel HEX image",
);
const _: () = assert!(FIRMWARE.len() > 32, "firmware image is implausibly small");
```

[`firmware::is_intel_hex`] is a `const fn` precisely so it can be used in these
assertions. Build with the path set:

```console
$ DMX_WIDGET_FIRMWARE=/path/to/firmware.bin cargo build --release
```

## The Embassy Sample

[`examples/piggy-embassy`](examples/piggy-embassy) is a minimal Embassy
application that drives the widget from Linux user space — for instance a
Raspberry Pi 3B running Raspberry Pi OS. A Linux host already *is* a USB host, so
unlike a bare-metal target nothing is left blank: the transport is implemented in
full against [`nusb`](https://docs.rs/nusb), and the whole two-stage lifecycle
(probe, upload, wait for re-enumeration, then stream) runs end to end.

`DMX_WIDGET_FIRMWARE` is a **build-time** path: the image is embedded into the
binary with `include_bytes!`, so a missing, non-HEX, or implausibly small image
fails the build (via `const` assertions) rather than the hardware, and the
resulting binary is self-contained — no firmware file is needed at run time.

```console
$ cd examples/piggy-embassy
$ DMX_WIDGET_FIRMWARE=/path/to/firmware.bin cargo build --release
$ ./target/release/piggy-embassy-dmx      # self-contained; copy to the Pi if built elsewhere
```

Because `env!` ties the build to the variable, every command that *compiles* needs
it set — including `cargo run`, which recompiles whenever the value changes. Either
`export DMX_WIDGET_FIRMWARE=…` once for the shell, or run the built binary directly
(as above), which needs nothing.

Build it on the Pi itself, or cross-compile for `aarch64-unknown-linux-gnu`. The
example is a detached workspace with its own dependencies, so it does not
interfere with `cargo test` at the repository root. Its Embassy and `nusb`
versions are known-good as of writing; expect to bump them.

## USB permissions on Linux

By default only root can open a raw USB device, so the driver fails with
`permission denied (errno 13)`. Install the bundled udev rule to grant your user
access ([`rules.d/99-hog-dmx-widget.rules`](rules.d/99-hog-dmx-widget.rules)):

```console
$ sudo cp rules.d/99-hog-dmx-widget.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then unplug and replug the widget so the rule applies, and run without `sudo`. The
rule covers both product IDs, because the device re-enumerates from `0x0CB0:0x0001`
(bootloader) to `0x0CB0:0x0002` (loaded) after the firmware upload and the driver
needs access to both. `MODE="0666"` lets any local user open it; edit the rule to
`MODE="0660", GROUP="plugdev"` if you would rather restrict access to that group.

## Bringing up the DMX output

The interface advertises three bulk OUT endpoints (`0x02` / `0x04` / `0x05`), but
the firmware services exactly one: `0x02`. The decompilation settles this — its
`cmd_dispatch` and `dmx_chunk_write` read only the `0x02` buffer/byte-count
registers, `0x04` stalls (its byte count is never armed), and `0x05` is accepted
by the SIE but silently dropped (a shared-descriptor artifact). See
`bore-hog/USB-PATHS.md`. The example streams to `0x02` unconditionally; patch a
fixture to channel 1, which it drives to full, and watch the widget's `tx mode` /
`dmx ok` LEDs:

```console
$ ./target/debug/piggy-embassy-dmx
```

Each run logs its `output config` at startup.

The frame is a bare 512-byte universe with **no** start code: the firmware
prepends the DMX start code itself (confirmed from the firmware disassembly).

## Receiving DMX

The widget is a full transceiver, and the crate now drives both directions: the
mode command's `Mode::Receive` (`01 02`) enables the UART receiver, and
[`Receiver`](src/receive.rs) reassembles the frames the firmware streams back on
bulk IN endpoint `0x84` (a `0x40`/`0x41`/`0x42` chunk run per frame, with an
optional start-code filter). Transmit and receive are mutually exclusive — one
mode register drives the RS-485 direction — which is why they are two types
(`Widget` / `Receiver`) rather than two methods. See `RECEIVE.md` for the wire
format and firmware provenance.

## The validation rig

[`examples/dmx-validator`](examples/dmx-validator) proves the driver against an
independent reference transceiver — the
[`zihatec-rs-485-dmx`](https://crates.io/crates/zihatec-rs-485-dmx) crate
driving a serial DMX interface on a Raspberry Pi — over a physical XLR loop:
serial → widget (forward), then widget → serial (reverse). The Pi and
interface setup the rig assumes (UART overlay, transceiver direction control)
is documented with that crate. USB access for the widget itself is covered in
[USB permissions on Linux](#usb-permissions-on-linux).

### Known device behaviour (measured, usbmon-verified)

Findings from validator runs cross-checked against a bus capture and the
firmware disassembly — expect these; they are properties of the hardware, not
of the rig:

- **Widget receive loses ~2–4 % of frames at any rate.** The DMX side is
  clean (frame releases stay on schedule); what glitches is the EP4 IN
  claim/arm machinery under concurrent UART-RX load, in two modes: a whole
  stream is skipped and the firmware's unconditional triple-buffer rotation
  overwrites the unclaimed frame (`LOST`, preceded by a ≈2×-period arrival
  gap), or the stream stops being armed mid-frame (`RESYNC — abandoned seq N`).
  The widget was never a lossless capture instrument; the validator counts
  both modes but they don't indict the wire.
- **Stale `0x42` re-delivery** (`stale42=` in the summaries): the device
  occasionally re-arms the final packet of the frame it just delivered,
  byte-identical, ~9 ms later. Benign — the receiver absorbs it — but its rate
  tracks the arm-glitch behaviour above.
- **Widget transmit keep-alive**: in transmit mode the widget re-sends its
  committed universe every ~500 ms on its own (`dups=` in reverse summaries),
  and its double buffer is *latest-commit-wins* — committing faster than the
  ~23 ms wire frame silently replaces un-transmitted frames. The validator's
  top phase is 40 Hz for exactly this reason.

When capturing USB traffic for analysis, always keep the pcap and the
validator log together — start `sudo tcpdump -i usbmon1 -s 0 -w run.pcap`
first, then run the validator with `… cargo run 2>&1 | tee run.log`. The
in-band sequence numbers make a lone pcap decodable, but correlating anomalies
is mechanical only with both files from the same run.

## 4G uplink watchdog

The validation Pi hangs off a 4G USB dongle, and a power brownout can wedge the
dongle's firmware so badly that only cutting its power brings it back. Since
the dongle *is* the Pi's uplink, nobody can ssh in to fix it — recovery has to
be automatic. `tools/4g-watchdog/` holds a watchdog that pings out over the
dongle once a minute and, while the link stays down, walks an escalation
ladder: restart the connection (minute 3), power-cycle the dongle's USB port
(minute 6), reboot the Pi (minute 10, and never within 15 minutes of boot, so
a dead SIM degrades into one reboot per boot instead of a reboot loop).

### Installation on the Pi

Copy the three files from `tools/4g-watchdog/` to the Pi, then place them:

```sh
sudo apt install uhubctl
sudo install -m 0755 4g-watchdog.sh /usr/local/sbin/4g-watchdog.sh
sudo install -m 0644 4g-watchdog.service 4g-watchdog.timer /etc/systemd/system/
```

Before enabling it, edit the configuration block at the top of
`/usr/local/sbin/4g-watchdog.sh` to match the dongle:

- `IFACE` — the dongle's interface, from `ip -br link`. QMI/MBIM modems appear
  as `wwan0`, serial modems as `ppp0`, HiLink dongles as an extra ethernet
  interface.
- `UHUBCTL_LOCATION` / `UHUBCTL_PORT` — run `sudo uhubctl` and pick the hub and
  port whose listed device is the dongle. Note that on a Pi 3B+/4 the ports
  are ganged: the power cycle briefly cuts power to *all* USB devices,
  including the DMX widget, which will re-enumerate.
- `USB_DEV` — the sysfs bus path (like `1-1.2`) used as fallback when uhubctl
  cannot switch the hub: compare `lsusb` with `ls /sys/bus/usb/devices` and
  pick the entry whose `idVendor`/`idProduct` files match the dongle.

Then enable the timer:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now 4g-watchdog.timer
```

Watch it work with `journalctl -t 4g-watchdog -f`; `systemctl list-timers
4g-watchdog.timer` confirms the once-a-minute schedule. To test the ladder end
to end without touching the hardware, set `PING_TARGETS` to an unreachable
address (e.g. `192.0.2.1`), let it escalate through the journal, then restore
the real targets.

### Belt and braces

Two companion measures are worth taking at the same time. First, enable the
Pi's hardware watchdog so a fully hung kernel also recovers:

```sh
sudo mkdir -p /etc/systemd/system.conf.d
printf '[Manager]\nRuntimeWatchdogSec=15\n' | \
    sudo tee /etc/systemd/system.conf.d/10-watchdog.conf
sudo systemctl daemon-reexec
```

Second, chase the brownout itself: `vcgencmd get_throttled` reports non-zero
when the 5 V rail has sagged since boot. A proper 5.1 V/3 A supply — or moving
the dongle to a powered USB hub (a uhubctl-switchable one also removes the
ganged-port caveat) — may stop the dongle wedging in the first place.

## Bring your own firmware

The crate contains no firmware and never will. The vendor's image is their
copyrighted work; uploading it into a widget you own, for your own use, is what
the official software already does, but redistributing it is not ours or yours to grant.
So: supply your own image at build time, and **do not commit it** — the
`.gitignore` is set up to help you avoid doing so by accident.

If you want an unencumbered or extended solution: the chip is an ordinary 8051
and the parts are well understood; clean-room firmware built with SDCC is
entirely feasible, and sidesteps the copyright question altogether.

## On "Security"

There is none to speak of, and that is the point. Nothing in the upload path or
the run-time protocol authenticates anything: the bootloader runs whatever image
it is handed, and the loaded device streams whatever frames it is sent. The
"protection", such as it was, was simply that the protocol was undocumented. Once
documented, as here, there is nothing left to bypass.

## Status

Confirmed and implemented:

- Two-stage identity and detection (`0x0CB0:0x0001` → `0x0002`).
- Anchor firmware upload, with per-record Intel HEX validation.
- Chunked universe framing on endpoint `0x02`: tagged 3-byte-header chunks
  (`0x30` fill, `0x31` commit), decoded from the firmware reassembler.
- Vendor identity queries: serial (`0x09`) and unique ID (`0x23`), cross-checked
  against the USB descriptors.
- Start code is prepended by the firmware; the host sends 512 bare channel bytes,
  so channel _n_ is slot _n_ (confirmed from the firmware disassembly).
- Mode command (`0x01`): the device boots idle and drives nothing until told;
  `Widget::start` enters transmit mode (`Mode::Transmit`), the symmetric partner
  of receive (`Mode::Receive`). Traced from the firmware.
- LEDs are firmware-driven and autonomous (nothing for the crate to do).
- DMX receive: `Mode::Receive` plus frame readback on endpoint `0x84`
  (`0x40`/`0x41`/`0x42` chunk reassembly, optional start-code filter `0x04`) —
  hardware-verified by the validator's forward run.
- No authentication anywhere in the protocol.

Open items (tracked as `TODO` in the source):

- **Commit tag `0x32`** — shares the `0x31` commit path; its distinction (likely
  one-shot vs continuous retransmit) is unconfirmed, so the send path uses `0x31`.
- **Example transport** — the RP2040 USB host wiring, and the Embassy version
  pins.
- **Receive trailer** — the 4-byte trailer on the final (`0x42`) receive chunk
  is set aside, not yet decoded (likely a byte count or timing stamp).
- **Other modes** — stop / transmit / receive are traced; any further modes
  (e.g. RDM) are not yet explored.

