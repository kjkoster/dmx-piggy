//! Intel HEX handling for the firmware image.
//!
//! The vendor firmware ships as Intel HEX text despite its `.bin` extension, so
//! the image is validated and streamed record-by-record rather than decoded into
//! a single blob. That keeps the uploader `no_alloc` and, more importantly, lets
//! each record be checksummed before it is clocked into the CPU's RAM: a corrupt
//! upload would run corrupt code until the next power cycle.

/// The canonical Intel HEX end-of-file record.
const EOF_RECORD: &[u8] = b":00000001FF";

/// Longest payload a single Intel HEX record can carry.
///
/// The record length field is one byte, so a caller's per-record buffer never
/// needs to exceed this.
pub const MAX_RECORD_PAYLOAD: usize = 255;

/// Compile-time check that a blob is Intel HEX.
///
/// Intended for a `const` assertion at the point of `include_bytes!`, so that a
/// missing or wrong firmware path fails the build rather than the hardware.
pub const fn is_intel_hex(bytes: &[u8]) -> bool {
    if bytes.is_empty() || bytes[0] != b':' {
        return false;
    }
    contains(bytes, EOF_RECORD)
}

const fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    if needle.len() > haystack.len() {
        return false;
    }
    let mut i = 0;
    while i + needle.len() <= haystack.len() {
        let mut j = 0;
        while j < needle.len() && eq_ignore_case(haystack[i + j], needle[j]) {
            j += 1;
        }
        if j == needle.len() {
            return true;
        }
        i += 1;
    }
    false
}

const fn eq_ignore_case(a: u8, b: u8) -> bool {
    to_lower(a) == to_lower(b)
}

const fn to_lower(c: u8) -> u8 {
    if c >= b'A' && c <= b'Z' {
        c + (b'a' - b'A')
    } else {
        c
    }
}

/// Failure while decoding an Intel HEX record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HexError {
    /// A record ended before its declared length was satisfied.
    Truncated,
    /// A character outside `[0-9A-Fa-f]` appeared where a hex digit was required.
    BadDigit,
    /// A record's checksum did not agree with its contents.
    BadChecksum,
    /// The caller's payload buffer was smaller than the record required.
    BufferTooSmall,
}

/// The kind of an Intel HEX record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordKind {
    /// Payload to be written at the record's address.
    Data,
    /// End-of-file marker.
    Eof,
    /// A record type this image is not expected to contain.
    Other(u8),
}

/// A decoded record header; the payload is written to the caller's buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Record {
    /// Load address. The EZ-USB download uses this directly as the `wValue` of
    /// the write request.
    pub address: u16,
    /// The record type.
    pub kind: RecordKind,
    /// Number of payload bytes written to the caller's buffer.
    pub len: u8,
}

/// A pull-based reader over an Intel HEX image.
///
/// This is a lending reader rather than an [`Iterator`]: each record's payload is
/// written into a caller-owned buffer, which avoids carrying a 255-byte array in
/// every yielded value and keeps the whole path allocation-free.
pub struct HexReader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> HexReader<'a> {
    /// Start reading `data`.
    pub const fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    /// Decode the next record, writing its payload into `out`.
    ///
    /// Returns `None` once no further record marks remain. Whitespace and line
    /// endings between records are skipped.
    pub fn next(&mut self, out: &mut [u8]) -> Option<Result<Record, HexError>> {
        while self.pos < self.data.len() && self.data[self.pos] != b':' {
            self.pos += 1;
        }
        if self.pos >= self.data.len() {
            return None;
        }
        Some(self.parse(out))
    }

    fn parse(&mut self, out: &mut [u8]) -> Result<Record, HexError> {
        self.pos += 1; // consume ':'
        let len = self.byte()?;
        let addr_hi = self.byte()?;
        let addr_lo = self.byte()?;
        let kind = self.byte()?;

        // The checksum covers every field, so it is accumulated as bytes are read
        // and must fold to zero once the trailing checksum byte is included.
        let mut sum = len
            .wrapping_add(addr_hi)
            .wrapping_add(addr_lo)
            .wrapping_add(kind);

        if len as usize > out.len() {
            return Err(HexError::BufferTooSmall);
        }
        for slot in out.iter_mut().take(len as usize) {
            let b = self.byte()?;
            *slot = b;
            sum = sum.wrapping_add(b);
        }

        let checksum = self.byte()?;
        if sum.wrapping_add(checksum) != 0 {
            return Err(HexError::BadChecksum);
        }

        Ok(Record {
            address: ((addr_hi as u16) << 8) | addr_lo as u16,
            kind: match kind {
                0x00 => RecordKind::Data,
                0x01 => RecordKind::Eof,
                other => RecordKind::Other(other),
            },
            len,
        })
    }

    fn byte(&mut self) -> Result<u8, HexError> {
        let hi = self.nibble()?;
        let lo = self.nibble()?;
        Ok((hi << 4) | lo)
    }

    fn nibble(&mut self) -> Result<u8, HexError> {
        let c = *self.data.get(self.pos).ok_or(HexError::Truncated)?;
        self.pos += 1;
        match c {
            b'0'..=b'9' => Ok(c - b'0'),
            b'a'..=b'f' => Ok(c - b'a' + 10),
            b'A'..=b'F' => Ok(c - b'A' + 10),
            _ => Err(HexError::BadDigit),
        }
    }
}
