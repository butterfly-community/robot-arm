//! Real PTY transport, no robot. A slow seventh servo must not hold up writes.
use super::*;
use serialport::TTYPort;

pub(super) fn pair() -> (FashionStarBus, TTYPort) {
    let (mut peer, port) = TTYPort::pair().unwrap();
    peer.set_timeout(Duration::from_secs(2)).unwrap();
    (
        FashionStarBus {
            port: Box::new(port),
            decoder: PacketDecoder::responses(),
            monitor_read: None,
            data_read: None,
            command: None,
        },
        peer,
    )
}

#[test]
fn firmware_read_is_read_only_correlated_and_does_not_block_commands() {
    let (mut bus, mut peer) = pair();
    bus.begin_data_read(DataRequest::Register { id: 4, address: 7 })
        .unwrap();
    let request = packet(&mut peer);
    assert_eq!((request.code, request.params), (3, vec![4, 7]));
    assert!(bus.poll_data_read().unwrap().is_none());
    bus.write_positions(&[PositionCommand {
        id: 0,
        position_tenths_degree: 0,
        motion_time_ms: 100,
        acceleration_time_ms: 50,
        deceleration_time_ms: 50,
        power_mw: 0,
    }])
    .unwrap();
    assert_eq!(packet(&mut peer).code, CODE_SYNC_COMMAND);
    peer.write_all(&response_packet(3, &[3, 7, 0x30, 3]).unwrap())
        .unwrap();
    peer.write_all(&response_packet(3, &[4, 8, 0, 0]).unwrap())
        .unwrap();
    assert!(bus.poll_data_read().unwrap().is_none());
    let reply = response_packet(3, &[4, 7, 0x25, 2]).unwrap();
    peer.write_all(&reply[..4]).unwrap();
    assert!(bus.poll_data_read().unwrap().is_none());
    peer.write_all(&reply[4..]).unwrap();
    let mut result = None;
    for _ in 0..100 {
        result = bus.poll_data_read().unwrap();
        if result.is_some() {
            break;
        }
        thread::sleep(Duration::from_millis(1));
    }
    assert_eq!(result, Some(vec![0x25, 2]));
    assert!(!bus.data_read_pending());
    bus.begin_monitor_read(&[4]).unwrap();
}

#[test]
fn register_timeout_is_local_and_releases_bus_after_one_retry() {
    let (mut bus, mut peer) = pair();
    bus.begin_data_read(DataRequest::Register { id: 4, address: 53 })
        .unwrap();
    packet(&mut peer);
    thread::sleep(DEFAULT_TIMEOUT + Duration::from_millis(5));
    assert!(bus.poll_data_read().unwrap().is_none());
    assert_eq!(packet(&mut peer).params, vec![4, 53]);
    thread::sleep(DEFAULT_TIMEOUT + Duration::from_millis(5));
    assert!(bus.poll_data_read().is_err());
    assert!(!bus.data_read_pending());
    bus.begin_monitor_read(&[4]).unwrap();
}

pub(super) fn packet(peer: &mut TTYPort) -> Packet {
    let mut decoder = PacketDecoder::requests();
    loop {
        let mut byte = [0];
        peer.read_exact(&mut byte).unwrap();
        if let Some(packet) = decoder.push(byte[0]) {
            return packet.unwrap();
        }
    }
}

fn reply(id: u8) -> Vec<u8> {
    let mut params = vec![0; 16];
    params[0] = id;
    params[10..14].copy_from_slice(&(i32::from(id) * 10).to_le_bytes());
    response_packet(CODE_QUERY_MONITOR, &params).unwrap()
}

fn wait_bytes(bus: &FashionStarBus) {
    let start = Instant::now();
    while bus.port.bytes_to_read().unwrap() == 0 {
        assert!(start.elapsed() < Duration::from_secs(2));
        thread::yield_now();
    }
}

#[test]
fn slow_monitor_never_prevents_position_writes_and_partial_feedback_is_not_published() {
    let (mut bus, mut peer) = pair();
    bus.begin_monitor_read(&[0, 1, 2, 3, 4, 5, 6]).unwrap();
    assert_eq!(packet(&mut peer).params, [22, 1, 7, 0, 1, 2, 3, 4, 5, 6]);
    // No reply yet: this must return immediately, not wait for a timeout.
    assert!(bus.poll_monitor_read().unwrap().is_none());
    for id in 0..6 {
        let command = PositionCommand {
            id,
            position_tenths_degree: 100,
            motion_time_ms: 100,
            acceleration_time_ms: 50,
            deceleration_time_ms: 50,
            power_mw: 0,
        };
        bus.write_positions(&[command]).unwrap();
        assert_eq!(packet(&mut peer).params[..4], [14, 15, 1, id]);
        // Optional action ack and fragmented Monitor packets share the stream.
        peer.write_all(&response_packet(14, &[id, 1]).unwrap())
            .unwrap();
        let response = reply(id);
        peer.write_all(&response[..3]).unwrap();
        wait_bytes(&bus);
        assert!(bus.poll_monitor_read().unwrap().is_none());
        peer.write_all(&response[3..]).unwrap();
        wait_bytes(&bus);
        assert!(bus.poll_monitor_read().unwrap().is_none());
    }
    peer.write_all(&reply(6)).unwrap();
    // A PTY may expose only the first fragment even after write_all returned.
    let started = Instant::now();
    let result = loop {
        if let Some(result) = bus.poll_monitor_read().unwrap() {
            break result;
        }
        assert!(started.elapsed() < DEFAULT_TIMEOUT);
        thread::yield_now();
    };
    assert_eq!(result.len(), 7);
    assert_eq!(result[6].position_tenths_degree, 60);
    assert!(!bus.monitor_read_pending());
}

#[test]
fn timeout_retries_once_without_blocking_or_mixing_partial_samples() {
    let (mut bus, mut peer) = pair();
    bus.begin_monitor_read(&[0, 1]).unwrap();
    packet(&mut peer);
    peer.write_all(&reply(0)).unwrap();
    wait_bytes(&bus);
    assert!(bus.poll_monitor_read().unwrap().is_none());
    bus.monitor_read
        .as_mut()
        .unwrap()
        .received_bytes(Instant::now() - DEFAULT_TIMEOUT);
    assert!(bus.poll_monitor_read().unwrap().is_none());
    assert_eq!(packet(&mut peer).params, [22, 1, 2, 0, 1]);
    peer.write_all(&[reply(1), reply(0)].concat()).unwrap();
    wait_bytes(&bus);
    assert_eq!(bus.poll_monitor_read().unwrap().unwrap().len(), 2);
    bus.begin_monitor_read(&[0]).unwrap();
    packet(&mut peer);
    for attempt in 0..2 {
        bus.monitor_read
            .as_mut()
            .unwrap()
            .received_bytes(Instant::now() - DEFAULT_TIMEOUT);
        let result = bus.poll_monitor_read();
        if attempt == 0 {
            assert!(result.unwrap().is_none());
            packet(&mut peer);
        } else {
            assert!(result.unwrap_err().to_string().contains("重试仍失败"));
        }
    }
    assert!(!bus.monitor_read_pending());
}

#[test]
fn duplicate_reply_cannot_complete_a_batch_and_cancel_releases_transaction() {
    let (mut bus, mut peer) = pair();
    bus.begin_monitor_read(&[0, 1]).unwrap();
    packet(&mut peer);
    peer.write_all(&[reply(0), reply(0)].concat()).unwrap();
    wait_bytes(&bus);
    assert!(bus.poll_monitor_read().unwrap().is_none());
    assert_eq!(packet(&mut peer).params, [22, 1, 2, 0, 1]);
    assert!(bus.begin_monitor_read(&[0]).is_err());
    bus.cancel_monitor_read().unwrap();
    assert!(!bus.monitor_read_pending());
    bus.begin_monitor_read(&[0]).unwrap();
    assert_eq!(packet(&mut peer).params, [22, 1, 1, 0]);
}

#[test]
fn unrelated_motion_ack_does_not_extend_monitor_timeout() {
    let (mut bus, mut peer) = pair();
    bus.begin_monitor_read(&[0]).unwrap();
    packet(&mut peer);
    peer.write_all(&response_packet(14, &[0, 1]).unwrap())
        .unwrap();
    wait_bytes(&bus);
    bus.monitor_read
        .as_mut()
        .unwrap()
        .received_bytes(Instant::now() - DEFAULT_TIMEOUT);
    assert!(bus.poll_monitor_read().unwrap().is_none());
    assert_eq!(packet(&mut peer).params, [22, 1, 1, 0]);
}
