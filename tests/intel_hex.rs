//! Host-side checks for the Intel HEX path, which is the one piece of logic that
//! can be exercised without a USB stack or hardware.

use dmx_piggy::firmware::{is_intel_hex, HexError, HexReader, RecordKind};

// One data record (three bytes at 0x0000) followed by the EOF record.
// Data checksum: 03+00+00+00+01+02+03 = 0x09, so the trailing byte is 0xF7.
const SAMPLE: &[u8] = b":03000000010203F7\r\n:00000001FF\r\n";

#[test]
fn accepts_valid_image() {
    assert!(is_intel_hex(SAMPLE));
}

#[test]
fn rejects_non_hex() {
    assert!(!is_intel_hex(b""));
    assert!(!is_intel_hex(b"not intel hex"));
    // Starts correctly but never terminates: the missing EOF record must fail.
    assert!(!is_intel_hex(b":03000000010203F7"));
}

#[test]
fn decodes_records_in_order() {
    let mut reader = HexReader::new(SAMPLE);
    let mut payload = [0u8; 255];

    let record = reader.next(&mut payload).unwrap().unwrap();
    assert_eq!(record.kind, RecordKind::Data);
    assert_eq!(record.address, 0x0000);
    assert_eq!(record.len, 3);
    assert_eq!(&payload[..3], &[0x01, 0x02, 0x03]);

    let record = reader.next(&mut payload).unwrap().unwrap();
    assert_eq!(record.kind, RecordKind::Eof);

    assert!(reader.next(&mut payload).is_none());
}

#[test]
fn rejects_corrupt_checksum() {
    // Same record as SAMPLE with the checksum byte off by one.
    let corrupt = b":03000000010203F6\r\n";
    let mut reader = HexReader::new(corrupt);
    let mut payload = [0u8; 255];
    assert_eq!(reader.next(&mut payload), Some(Err(HexError::BadChecksum)));
}

#[test]
fn reports_short_payload_buffer() {
    let mut reader = HexReader::new(SAMPLE);
    let mut payload = [0u8; 2]; // smaller than the 3-byte record
    assert_eq!(reader.next(&mut payload), Some(Err(HexError::BufferTooSmall)));
}
