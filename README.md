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

Status: _early_. The firmware upload and DMX framing are implemented; one detail
of the output path is still to be confirmed on hardware (see [Status](#status)).

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

**In plain terms.** Once firmware is running, you send it a full frame of 512
channel values, over and over, a few dozen times a second. The box does the
fiddly electrical timing of DMX itself.

**Technically.** The loaded device exposes a vendor interface with bulk
endpoints. A universe is one bulk OUT transfer of 512 bytes, with no header on
the wire; the firmware generates the DMX break, mark-after-break and byte framing
on its own timing. You need only keep sending frames at your chosen refresh rate.
[`Universe`](src/dmx.rs) is a fixed 512-byte buffer with 1-based channel
addressing.

> **Not yet pinned:** the device advertises three bulk OUT endpoints
> (`0x02`, `0x04`, `0x05`) and it is not yet proven from the firmware alone which
> one carries the universe. The default is `0x02`; [`Widget::with_endpoint`]
> lets you try the others. See [Status](#status).

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

3. **Implement [`Transport`](src/transport.rs)** for your USB host — three
   `async` methods: `control_out`, `control_in`, `bulk_out`.

4. **Embed the firmware** in your binary and validate it at build time (see
   [Embedding the firmware](#embedding-the-firmware)).

5. **Drive the lifecycle:** [`probe`] → [`upload`] if unloaded → wait for
   re-enumeration → [`Widget::send`] in a loop.

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

## Receiving DMX

This firmware cannot receive DMX. The rear panel has a receive indicator and the
hardware is capable (the MAX483 is a transceiver, and the loaded device even
advertises bulk IN endpoints), but the shipped image configures the UART
transmit-only. The receiver is never enabled and received bytes are never read.
Receiving, for troubleshooting or otherwise, would need a different firmware
variant or your own. In fact, we are not even sure that the MAX483 is wired to the main MCU to receive data. This is out of scope for the crate.

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
- 512-byte, header-less universe framing.
- LEDs are firmware-driven and autonomous (nothing for the crate to do).
- No authentication anywhere; firmware is transmit-only (no DMX receive).

Open items (tracked as `TODO` in the source):

- **DMX OUT endpoint** — one of `0x02` / `0x04` / `0x05`; confirm on hardware or
  by tracing the transmit engine, then reduce to a single constant.
- **Start-code placement** — whether the firmware prepends the DMX start code or
  expects it in the first slot; this fixes channel-to-index mapping.
- **Example transport** — the RP2040 USB host wiring, and the Embassy version
  pins.
- **DMX Receive Mode** - Need to check that the wiring allows for that.

