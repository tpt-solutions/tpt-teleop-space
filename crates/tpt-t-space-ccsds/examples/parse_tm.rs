//! Parse a CCSDS telemetry packet with the zero-copy `PacketView` and
//! print its primary header and payload.
//!
//! Run with: `cargo run -p tpt-t-space-ccsds --example parse_tm`

use tpt_t_space_ccsds::packet::PacketView;

fn main() {
    // Primary header: version 0, telemetry, APID 0x123, unsegmented,
    // sequence count 5, data length 3 (4 payload bytes).
    let frame = [0x01, 0x23, 0xC0, 0x05, 0x00, 0x03, 0xDE, 0xAD, 0xBE, 0xEF];
    match PacketView::parse(&frame) {
        Ok(view) => {
            println!("apid        : {:#05x}", view.apid);
            println!("sequence    : {}", view.sequence_count);
            println!("type        : {:?}", view.packet_type);
            println!("flags       : {:?}", view.sequence_flags);
            println!("total bytes : {}", view.total_len());
            println!("payload     : {:02X?}", view.payload());
        }
        Err(e) => println!("rejected: {e}"),
    }
}
