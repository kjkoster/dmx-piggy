//! Crate error type.

use crate::firmware::HexError;

/// Anything that can go wrong while driving the widget.
///
/// Generic over the transport's own error so that host-stack failures propagate
/// without being flattened into a single opaque variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// The underlying USB transport failed.
    Transport(E),
    /// The firmware image was not valid Intel HEX.
    Hex(HexError),
    /// A descriptor read returned fewer bytes than the descriptor requires.
    ShortDescriptor,
    /// The device did not identify as this vendor's widget.
    NotAWidget,
    /// A vendor query's reply was absent or did not echo the opcode it answers.
    UnexpectedReply,
}
