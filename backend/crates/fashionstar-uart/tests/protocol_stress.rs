//! Independent public-API protocol regression, no execution node or hardware.
use fashionstar_uart::{PacketDecoder, request_packet, response_packet};
use std::time::Instant;

#[test]
fn every_payload_length_and_fragment_boundary_round_trips() {
    for size in 0..=255 {
        let data: Vec<u8> = (0..size).map(|x| (x * 73 + 19) as u8).collect();
        for split in 0..size + 5 {
            let bytes = response_packet(3, &data).unwrap();
            let mut decoder = PacketDecoder::responses();
            let mut packets = vec![];
            for part in [&bytes[..split], &bytes[split..]] {
                for &b in part {
                    if let Some(p) = decoder.push(b) {
                        packets.push(p.unwrap());
                    }
                }
            }
            assert_eq!(packets.len(), 1);
            assert_eq!(packets[0].params, data);
        }
    }
    assert!(request_packet(3, &[0; 256]).is_err());
}

#[test]
fn long_stream_recovers_from_bad_checksums_and_preserves_packet_order() {
    let start = Instant::now();
    let mut decoder = PacketDecoder::responses();
    let mut valid = 0;
    let mut invalid = 0;
    for sequence in 0u32..100_000 {
        let mut bytes = response_packet(3, &sequence.to_le_bytes()).unwrap();
        if sequence % 7 == 0 {
            let last = bytes.len() - 1;
            bytes[last] ^= 1;
        }
        for b in bytes {
            if let Some(result) = decoder.push(b) {
                match result {
                    Ok(p) => {
                        assert_eq!(p.params, sequence.to_le_bytes());
                        valid += 1;
                    }
                    Err(_) => {
                        assert_eq!(sequence % 7, 0);
                        invalid += 1;
                    }
                }
            }
        }
    }
    assert_eq!(valid + invalid, 100_000);
    assert_eq!(invalid, 14_286);
    println!(
        "protocol_stress frames=100000 valid={valid} corrupt={invalid} elapsed_ms={:.3}",
        start.elapsed().as_secs_f64() * 1000.
    );
}
