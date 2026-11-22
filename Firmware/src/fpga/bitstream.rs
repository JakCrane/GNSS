/// The bitstream, baked into the MCU firmware at build time.
/// Cargo rebuilds the firmware automatically whenever this file changes.
pub static BITSTREAM: &[u8] = include_bytes!("../../../Gateware/build/top.bit");
