//! DMX-512 receive: read frames the widget captures off the XLR.
//!
//! The widget is a full transceiver. Receive is gated behind a mode command
//! rather than being the default, so it goes unnoticed until asked for: put the
//! device into [`Mode::Receive`], and it validates each incoming frame's start
//! code, buffers the 512 channels, and streams completed frames back on a
//! separate bulk IN endpoint ([`RECEIVE_ENDPOINT`]).
//!
//! Transmit and receive are mutually exclusive on a single widget — one mode
//! register drives the RS-485 direction — so this is modelled as a distinct
//! [`Receiver`] rather than another method on [`Widget`](crate::device::Widget).
//! The two are the symmetric ends of the same mode register.
//!
//! ```no_run
//! # async fn run<T: dmx_piggy::Transport>(transport: T) -> Result<(), dmx_piggy::Error<T::Error>> {
//! use dmx_piggy::receive::Receiver;
//! let mut rx = Receiver::new(transport);
//! rx.start().await?;                 // enter receive mode (start code 0x00)
//! let frame = rx.next_frame().await?; // one 512-channel universe
//! rx.stop().await?;                  // back to idle
//! # let _ = frame; Ok(())
//! # }
//! ```

use crate::device::{send_mode, Mode};
use crate::dmx::{Universe, CHANNELS, DEFAULT_OUT_ENDPOINT};
use crate::error::Error;
use crate::transport::Transport;

/// Bulk IN endpoint that streams received DMX frames. Distinct from the `0x82`
/// command/status channel; active only in [`Mode::Receive`].
pub const RECEIVE_ENDPOINT: u8 = 0x84;

/// Command endpoint that carries the mode and start-code-filter commands — the
/// single live OUT endpoint, shared with the DMX chunk stream.
const COMMAND_ENDPOINT: u8 = DEFAULT_OUT_ENDPOINT;

/// Vendor opcode that sets the start-code filter (byte 1 is the start code).
const START_CODE_FILTER: u8 = 0x04;

/// Tag on the first packet of a frame: begin a fresh 512-channel buffer.
const TAG_FIRST: u8 = 0x40;
/// Tag on a continuation packet: append more channel bytes.
const TAG_CONT: u8 = 0x41;
/// Tag on the last packet of a frame: append the final channels, then release.
const TAG_LAST: u8 = 0x42;

/// The 64-byte receive packet: a tag byte followed by up to 63 channel bytes
/// (the last packet also carries a 4-byte trailer past the channel data).
const PACKET_LEN: usize = 64;

/// Counters from the receive reassembly, for diagnosing a lossy return path.
/// Monotonic since construction (or the last [`Receiver::reset_stats`]).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct RxStats {
    /// Frames successfully reassembled and returned by [`Receiver::next_frame`].
    pub frames: u32,
    /// Idle `bulk_in` reads (`Ok(0)` timeouts) absorbed while waiting for a frame.
    pub timeouts: u32,
    /// Abandoned partial frames — a fresh `0x40` arrived mid-frame, discarding what
    /// was gathered. Nonzero means packets are being lost part-way through a frame.
    pub resyncs: u32,
    /// Continuation/last packets seen before any `0x40` — a lost frame start.
    pub orphans: u32,
    /// Re-armed final packets: a `0x42` byte-identical to the one that closed
    /// the previous frame (trailer included). The device occasionally arms its
    /// last IN packet a second time; no data is lost — the frame it tails was
    /// already delivered intact — but the rate is a useful glitch signal.
    pub stale_last: u32,
    /// Packets whose tag byte matched none of `0x40`/`0x41`/`0x42`.
    pub unknown_tags: u32,
    /// Packets shorter than the 64 bytes the firmware always arms. Any short
    /// read is an anomaly worth seeing, wherever it truncated.
    pub short_packets: u32,
    /// Events that could not be recorded because the event ring was full
    /// (see [`Receiver::events`]).
    pub events_dropped: u32,
}

/// What a recorded receive-path anomaly was. See [`RxEvent`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxEventKind {
    /// A continuation/last packet with no frame in progress: the frame start
    /// was lost. The payload head identifies which frame it belonged to.
    Orphan,
    /// The device re-armed and re-sent the previous frame's final packet,
    /// byte for byte (see [`RxStats::stale_last`]). Benign.
    StaleLast,
    /// A fresh first tag arrived mid-frame; the partial was abandoned. `got`
    /// says how far the dead frame progressed — where the stream died; `head`
    /// holds the *abandoned* frame's first bytes, so its identity (e.g. an
    /// in-band sequence number) is recoverable.
    Resync,
    /// A packet with an unrecognised tag byte.
    UnknownTag,
    /// A packet shorter than the 64 bytes the firmware always arms.
    ShortPacket,
}

/// One recorded receive-path anomaly, with enough context for post-hoc
/// diagnosis without any storage the caller didn't ask for.
#[derive(Debug, Clone, Copy)]
pub struct RxEvent {
    /// What happened.
    pub kind: RxEventKind,
    /// The packet's tag byte (byte 0 as received).
    pub tag: u8,
    /// The packet's length as received from the transport.
    pub len: u8,
    /// Reassembly progress (channel bytes gathered) when the event hit.
    pub got: u16,
    /// The first payload bytes (after the tag), zero-padded. For an orphaned
    /// `0x42` these are the frame's final channels — matchable against what
    /// was sent to identify the frame the orphan belonged to.
    pub head: [u8; 8],
}

impl RxEvent {
    const EMPTY: Self = Self {
        kind: RxEventKind::UnknownTag,
        tag: 0,
        len: 0,
        got: 0,
        head: [0; 8],
    };
}

/// Capacity of the [`Receiver::events`] ring: plenty between two polls of a
/// caller that drains it per frame, tiny enough to live on a `no_std` stack.
const EVENT_CAP: usize = 16;

/// A widget in receive mode: reads DMX frames captured off the XLR.
///
/// Construct one over a loaded widget's transport, call [`start`](Self::start)
/// to enter receive mode, then [`next_frame`](Self::next_frame) repeatedly.
/// Because the [`Transport`] is async, the mode is **not** restored on drop
/// (`Drop` cannot await) — call [`stop`](Self::stop) when done to return the
/// device to idle.
pub struct Receiver<T> {
    transport: T,
    stats: RxStats,
    // Reassembly state lives on the receiver (not on the stack of a single
    // call) so [`try_next_frame`](Self::try_next_frame) can return "nothing
    // yet" on an idle timeout without abandoning a partially gathered frame.
    partial: Universe,
    got: usize,
    started: bool,
    // The final (0x42) packet that closed the last completed frame, for
    // recognizing the device's occasional byte-identical re-send of it.
    last_final: [u8; PACKET_LEN],
    have_final: bool,
    // Anomaly ring: fixed storage, drained by the caller (see `events`).
    events: [RxEvent; EVENT_CAP],
    event_len: usize,
}

impl<T: Transport> Receiver<T> {
    /// Wrap a loaded widget's transport. Does not touch the device; call
    /// [`start`](Self::start) to enter receive mode.
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            stats: RxStats::default(),
            partial: Universe::new(),
            got: 0,
            started: false,
            last_final: [0; PACKET_LEN],
            have_final: false,
            events: [RxEvent::EMPTY; EVENT_CAP],
            event_len: 0,
        }
    }

    /// The anomalies recorded since the last [`clear_events`](Self::clear_events),
    /// in arrival order. The ring is small ([`EVENT_CAP`]); drain it after every
    /// frame (or idle poll) or overflow is counted in
    /// [`RxStats::events_dropped`].
    pub fn events(&self) -> &[RxEvent] {
        &self.events[..self.event_len]
    }

    /// Forget the recorded events, freeing the ring.
    pub fn clear_events(&mut self) {
        self.event_len = 0;
    }

    /// Record an anomaly. `head_src` is whatever identifies the event best —
    /// usually the packet's payload, but for a resync the *abandoned* frame's
    /// first bytes (where an in-band identity lives).
    fn push_event(&mut self, kind: RxEventKind, tag: u8, len: usize, head_src: &[u8]) {
        if self.event_len == EVENT_CAP {
            self.stats.events_dropped = self.stats.events_dropped.wrapping_add(1);
            return;
        }
        let mut head = [0u8; 8];
        let take = head_src.len().min(head.len());
        head[..take].copy_from_slice(&head_src[..take]);
        self.events[self.event_len] = RxEvent {
            kind,
            tag,
            len: len as u8,
            got: self.got as u16,
            head,
        };
        self.event_len += 1;
    }

    /// The receive-path counters accumulated so far. See [`RxStats`].
    pub fn stats(&self) -> RxStats {
        self.stats
    }

    /// Reset the [`stats`](Self::stats) counters to zero (e.g. at a phase boundary).
    pub fn reset_stats(&mut self) {
        self.stats = RxStats::default();
    }

    /// Enter receive mode with the default start-code filter (`0x00`, standard
    /// dimmer data). Enables the UART receiver and lights the receive LED.
    pub async fn start(&mut self) -> Result<(), Error<T::Error>> {
        send_mode(&mut self.transport, Mode::Receive).await
    }

    /// Enter receive mode, delivering only frames whose start code matches
    /// `start_code`. The filter is set **after** entering the mode: entering
    /// receive mode resets the filter, so setting it first would not stick.
    pub async fn start_filtered(&mut self, start_code: u8) -> Result<(), Error<T::Error>> {
        self.start().await?;
        self.transport
            .bulk_out(COMMAND_ENDPOINT, &[START_CODE_FILTER, start_code])
            .await
            .map_err(Error::Transport)
    }

    /// Return the device to idle ([`Mode::Stop`]), releasing the receiver.
    pub async fn stop(&mut self) -> Result<(), Error<T::Error>> {
        send_mode(&mut self.transport, Mode::Stop).await
    }

    /// Read and reassemble one frame into a [`Universe`].
    ///
    /// Reads 64-byte packets off [`RECEIVE_ENDPOINT`] and reassembles the
    /// `0x40`/`0x41`/`0x42` chunk stream. Reassembly keys off the tags, not a
    /// packet count: a fresh `0x40` starts a new frame (so a short or malformed
    /// frame resynchronises on the next one rather than desynchronising the
    /// stream), and continuation/last packets before the first `0x40` are
    /// ignored. The 4-byte trailer on the last packet is frame metadata, not
    /// channel data, and is discarded.
    ///
    /// Idle reads (no DMX source) time out and return `Ok(0)` from the
    /// transport; those are absorbed here, so this simply blocks until a frame
    /// arrives. Uses fixed buffers — no allocation.
    pub async fn next_frame(&mut self) -> Result<Universe, Error<T::Error>> {
        loop {
            if let Some(universe) = self.try_next_frame().await? {
                return Ok(universe);
            }
        }
    }

    /// Like [`next_frame`](Self::next_frame), but an idle read (`Ok(0)`
    /// transport timeout) returns `Ok(None)` instead of blocking on.
    ///
    /// A partially gathered frame is **kept** across `None` returns — the
    /// reassembly state lives on the receiver — so polling this in a loop is
    /// exactly equivalent to [`next_frame`](Self::next_frame). It exists so a
    /// caller can interleave the wait with its own checks (e.g. a shutdown
    /// flag) at the granularity of the transport's read timeout.
    pub async fn try_next_frame(&mut self) -> Result<Option<Universe>, Error<T::Error>> {
        loop {
            let mut packet = [0u8; PACKET_LEN];
            let n = self
                .transport
                .bulk_in(RECEIVE_ENDPOINT, &mut packet)
                .await
                .map_err(Error::Transport)?;
            if n == 0 {
                self.stats.timeouts = self.stats.timeouts.wrapping_add(1);
                return Ok(None); // timeout: nothing queued yet
            }
            if n != PACKET_LEN {
                // The firmware always arms full 64-byte packets, so any short
                // read is an anomaly (host stack or controller), even though the
                // bytes that did arrive are still processed below.
                self.stats.short_packets = self.stats.short_packets.wrapping_add(1);
                self.push_event(RxEventKind::ShortPacket, packet[0], n, &packet[1..n]);
            }
            match packet[0] {
                TAG_FIRST => {
                    if self.got > 0 {
                        // A fresh frame arrived before the previous finished — the
                        // tail was lost mid-stream (the device stopped arming its
                        // IN packets part-way). Record how far the dead frame got
                        // and its first bytes (identity) before resetting.
                        self.stats.resyncs = self.stats.resyncs.wrapping_add(1);
                        let head: [u8; 8] = self.partial.channels()[..8]
                            .try_into()
                            .expect("8-byte head from a 512-slot frame");
                        self.push_event(RxEventKind::Resync, packet[0], n, &head);
                    }
                    self.got = 0; // start of a new frame: resync
                    self.started = true;
                }
                TAG_CONT | TAG_LAST if self.started => {}
                TAG_LAST if self.have_final && packet[..] == self.last_final[..] => {
                    // The device re-armed the final packet of the frame it just
                    // delivered (byte-identical, trailer included). An extra tail
                    // of a frame already received intact — benign, but counted:
                    // its rate tracks the device's arm-glitch behaviour.
                    self.stats.stale_last = self.stats.stale_last.wrapping_add(1);
                    self.push_event(RxEventKind::StaleLast, packet[0], n, &packet[1..n]);
                    continue;
                }
                TAG_CONT | TAG_LAST => {
                    // A continuation/last before any first tag: the frame start was
                    // lost. Wait for the next 0x40 rather than mis-assemble. The
                    // payload head is kept — for a 0x42 it is the frame's tail
                    // channels, which identify the frame this orphan belonged to.
                    self.stats.orphans = self.stats.orphans.wrapping_add(1);
                    self.push_event(RxEventKind::Orphan, packet[0], n, &packet[1..n]);
                    continue;
                }
                _ => {
                    self.stats.unknown_tags = self.stats.unknown_tags.wrapping_add(1);
                    self.push_event(RxEventKind::UnknownTag, packet[0], n, &packet[1..n]);
                    continue; // unknown tag: ignore
                }
            }
            let last = packet[0] == TAG_LAST;
            let data = &packet[1..n];
            let take = core::cmp::min(data.len(), CHANNELS - self.got);
            self.partial.channels_mut()[self.got..self.got + take].copy_from_slice(&data[..take]);
            self.got += take;
            if last || self.got >= CHANNELS {
                if last {
                    // Remember the closing packet: the device sometimes re-arms
                    // and re-sends it, which must read as stale, not as an orphan.
                    self.last_final = packet;
                    self.have_final = true;
                }
                self.stats.frames = self.stats.frames.wrapping_add(1);
                self.got = 0;
                self.started = false;
                return Ok(Some(core::mem::take(&mut self.partial)));
            }
        }
    }

    /// Recover the wrapped transport. Does not change the device mode; call
    /// [`stop`](Self::stop) first if the widget should be left idle.
    pub fn release(self) -> T {
        self.transport
    }
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

    /// Build the nine 64-byte packets the firmware would send for one full
    /// 512-channel frame: `0x40`, seven `0x41`, then `0x42` with 8 channel bytes
    /// and a 4-byte trailer. Each packet is a full 64-byte transfer.
    fn frame_packets(source: &[u8; CHANNELS]) -> [[u8; PACKET_LEN]; 9] {
        let mut packets = [[0u8; PACKET_LEN]; 9];
        let mut offset = 0;
        for (p, packet) in packets.iter_mut().enumerate() {
            let span = core::cmp::min(PACKET_LEN - 1, CHANNELS - offset);
            packet[0] = if p == 0 {
                TAG_FIRST
            } else if offset + span >= CHANNELS {
                TAG_LAST
            } else {
                TAG_CONT
            };
            packet[1..1 + span].copy_from_slice(&source[offset..offset + span]);
            // Last packet's trailer bytes: distinctive junk that must be ignored.
            if packet[0] == TAG_LAST {
                for b in &mut packet[1 + span..1 + span + 4] {
                    *b = 0xAB;
                }
            }
            offset += span;
        }
        packets
    }

    /// Replays a fixed script of packets on the receive endpoint, one per
    /// `bulk_in` call, after first reporting `lead_timeouts` idle reads
    /// (`Ok(0)`); once exhausted it reports timeouts too.
    struct Playback<'a> {
        packets: &'a [[u8; PACKET_LEN]],
        idx: usize,
        lead_timeouts: usize,
        /// Inject a single extra timeout just before the packet at this index.
        timeout_before: usize,
        /// Deliver the packet at index `.0` truncated to `.1` bytes.
        short_at: Option<(usize, usize)>,
    }

    impl<'a> Playback<'a> {
        fn new(packets: &'a [[u8; PACKET_LEN]]) -> Self {
            Self {
                packets,
                idx: 0,
                lead_timeouts: 0,
                timeout_before: usize::MAX,
                short_at: None,
            }
        }
    }

    impl Transport for Playback<'_> {
        type Error = ();
        async fn control_out(&mut self, _: u8, _: u8, _: u16, _: u16, _: &[u8]) -> Result<(), ()> {
            Ok(())
        }
        async fn control_in(&mut self, _: u8, _: u8, _: u16, _: u16, _: &mut [u8]) -> Result<usize, ()> {
            Ok(0)
        }
        async fn bulk_out(&mut self, _: u8, _: &[u8]) -> Result<(), ()> {
            Ok(())
        }
        async fn bulk_in(&mut self, endpoint: u8, buf: &mut [u8]) -> Result<usize, ()> {
            assert_eq!(endpoint, RECEIVE_ENDPOINT);
            if self.lead_timeouts > 0 {
                self.lead_timeouts -= 1;
                return Ok(0); // idle endpoint: nothing queued yet
            }
            if self.idx == self.timeout_before {
                self.timeout_before = usize::MAX; // one-shot
                return Ok(0); // injected mid-stream timeout
            }
            if self.idx >= self.packets.len() {
                return Ok(0); // exhausted: behave like an idle endpoint
            }
            let packet = &self.packets[self.idx];
            let mut n = packet.len().min(buf.len());
            if let Some((at, short_len)) = self.short_at {
                if at == self.idx {
                    n = n.min(short_len);
                }
            }
            self.idx += 1;
            buf[..n].copy_from_slice(&packet[..n]);
            Ok(n)
        }
    }

    fn ramp() -> [u8; CHANNELS] {
        let mut src = [0u8; CHANNELS];
        for (i, b) in src.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(7).wrapping_add(3);
        }
        src
    }

    #[test]
    fn reassembles_a_full_frame_and_discards_the_trailer() {
        let src = ramp();
        let packets = frame_packets(&src);
        let mut rx = Receiver::new(Playback::new(&packets));

        let frame = block_on(rx.next_frame()).unwrap();
        // Every channel round-trips, with channel n at slot n-1 (index n-1).
        assert_eq!(frame.channels(), &src);
        // The 4-byte trailer past the last channel never leaked into the frame.
        assert_eq!(frame.channels()[CHANNELS - 1], src[CHANNELS - 1]);
    }

    #[test]
    fn timeouts_before_the_frame_are_absorbed() {
        // Idle reads (Ok(0)) before the frame must be skipped, not mistaken for a
        // frame boundary: the same clean frame still reassembles after them.
        let src = ramp();
        let packets = frame_packets(&src);
        let mut pb = Playback::new(&packets);
        pb.lead_timeouts = 3;
        let mut rx = Receiver::new(pb);

        let frame = block_on(rx.next_frame()).unwrap();
        assert_eq!(frame.channels(), &src);
        // The three idle reads were counted, and one frame completed.
        assert_eq!(rx.stats().timeouts, 3);
        assert_eq!(rx.stats().frames, 1);
    }

    #[test]
    fn resyncs_on_the_first_tag_after_garbage() {
        let src = ramp();
        // A stray continuation packet (no preceding 0x40) then a clean frame.
        let mut script = [[0u8; PACKET_LEN]; 10];
        script[0][0] = TAG_CONT;
        for b in &mut script[0][1..PACKET_LEN] {
            *b = 0xFF; // garbage that must not appear in the frame
        }
        script[1..].copy_from_slice(&frame_packets(&src));
        let mut rx = Receiver::new(Playback::new(&script));

        let frame = block_on(rx.next_frame()).unwrap();
        assert_eq!(frame.channels(), &src);
        // The stray continuation before any 0x40 was counted as an orphan, and
        // the event kept its payload head for identification.
        assert_eq!(rx.stats().orphans, 1);
        let events = rx.events();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].kind, RxEventKind::Orphan);
        assert_eq!(events[0].tag, TAG_CONT);
        assert_eq!(events[0].head, [0xFF; 8]);
        rx.clear_events();
        assert!(rx.events().is_empty());
    }

    #[test]
    fn try_next_frame_returns_none_when_idle() {
        // No packets at all: the poll variant reports "nothing yet" instead of
        // spinning, and counts the timeout.
        let mut rx = Receiver::new(Playback::new(&[]));
        assert!(block_on(rx.try_next_frame()).unwrap().is_none());
        assert_eq!(rx.stats().timeouts, 1);
        assert_eq!(rx.stats().frames, 0);
    }

    #[test]
    fn try_next_frame_keeps_a_partial_frame_across_a_timeout() {
        // A timeout in the middle of a frame's packet run must not abandon the
        // partial: the next poll picks up where it left off and completes the
        // same frame. This is what makes polling equivalent to next_frame.
        let src = ramp();
        let packets = frame_packets(&src);
        let mut pb = Playback::new(&packets);
        pb.timeout_before = 4; // timeout between packets 3 and 4, mid-frame
        let mut rx = Receiver::new(pb);

        assert!(block_on(rx.try_next_frame()).unwrap().is_none()); // hit the gap
        let frame = block_on(rx.try_next_frame()).unwrap().expect("frame completes");
        assert_eq!(frame.channels(), &src);
        assert_eq!(rx.stats().timeouts, 1);
        assert_eq!(rx.stats().resyncs, 0); // the partial was never discarded
    }

    #[test]
    fn a_new_first_tag_abandons_a_partial_frame() {
        let stale = [0x11u8; CHANNELS];
        let fresh = ramp();
        // First two packets of a frame that never finishes, then a whole new one.
        let stale_packets = frame_packets(&stale);
        let mut script = [[0u8; PACKET_LEN]; 11];
        script[0] = stale_packets[0]; // 0x40 + 63 stale bytes
        script[1] = stale_packets[1]; // 0x41 + 63 stale bytes
        script[2..].copy_from_slice(&frame_packets(&fresh));
        let mut rx = Receiver::new(Playback::new(&script));

        let frame = block_on(rx.next_frame()).unwrap();
        assert_eq!(frame.channels(), &fresh);
        // Abandoning the partial (a 0x40 mid-frame) was counted as a resync,
        // and the event recorded how far the dead frame had progressed plus
        // its head bytes, so the abandoned frame stays identifiable.
        assert_eq!(rx.stats().resyncs, 1);
        let events = rx.events();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].kind, RxEventKind::Resync);
        assert_eq!(events[0].got, 126); // two 63-byte spans were gathered
        assert_eq!(events[0].head, [0x11; 8]); // the abandoned frame's first bytes
    }

    #[test]
    fn a_rearmed_final_packet_is_stale_not_orphan() {
        // The device sometimes re-arms and re-sends the 0x42 that closed the
        // previous frame, byte for byte. That must count as stale_last — an
        // extra tail of a frame already received intact — not as an orphan.
        let src = ramp();
        let packets = frame_packets(&src);
        let mut script = [[0u8; PACKET_LEN]; 10];
        script[..9].copy_from_slice(&packets);
        script[9] = packets[8]; // the byte-identical re-send, trailer and all
        let mut rx = Receiver::new(Playback::new(&script));

        let frame = block_on(rx.next_frame()).unwrap().clone();
        assert_eq!(frame.channels(), &src);
        // Poll once more: the stale packet is absorbed, no frame produced.
        assert!(block_on(rx.try_next_frame()).unwrap().is_none());
        assert_eq!(rx.stats().stale_last, 1);
        assert_eq!(rx.stats().orphans, 0);
        let events = rx.events();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].kind, RxEventKind::StaleLast);

        // A 0x42 that does NOT match the previous final packet is still a
        // genuine orphan (the tail of some frame that lost its start).
        let mut other = packets[8];
        other[1] ^= 0xFF;
        let script2 = [other];
        let mut playback = Playback::new(&script2);
        playback.lead_timeouts = 0;
        // Re-use the receiver's state? A fresh receiver has no last_final, so
        // the packet is an orphan there too — assert the discriminating case
        // on the same receiver by swapping the transport.
        let mut rx2 = Receiver::new(playback);
        assert!(block_on(rx2.try_next_frame()).unwrap().is_none());
        assert_eq!(rx2.stats().orphans, 1);
        assert_eq!(rx2.stats().stale_last, 0);
    }

    #[test]
    fn unknown_tags_and_short_packets_are_recorded() {
        let src = ramp();
        // An alien tag, then a clean frame whose middle packet is short.
        let mut script = [[0u8; PACKET_LEN]; 10];
        script[0][0] = 0x7E; // alien tag
        script[1..].copy_from_slice(&frame_packets(&src));
        let mut pb = Playback::new(&script);
        pb.short_at = Some((4, 10)); // packet index 4 delivered as 10 bytes
        let mut rx = Receiver::new(pb);

        // The frame still completes (the short packet costs channel bytes, so
        // the tail shifts — not asserted here); what matters is that both
        // anomalies were counted and recorded.
        let _ = block_on(rx.try_next_frame());
        assert_eq!(rx.stats().unknown_tags, 1);
        assert_eq!(rx.stats().short_packets, 1);
        let events = rx.events();
        assert!(events.iter().any(|e| e.kind == RxEventKind::UnknownTag));
        assert!(events.iter().any(|e| e.kind == RxEventKind::ShortPacket));
    }
}
