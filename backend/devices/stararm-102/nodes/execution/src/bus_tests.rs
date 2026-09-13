//! Test the production adapter on a PTY with half-duplex scheduling assertions.
//! The peer is only a byte-level servo fixture, never a production robot path.
use super::*;
use fashionstar_uart::{DataRequest, PacketDecoder, response_packet};
use serialport::{SerialPort, TTYPort};
use std::io::{Read, Write};

fn pair() -> (StarArmBus, TTYPort) {
    let (mut peer, slave) = TTYPort::pair().unwrap();
    peer.set_timeout(Duration::from_secs(2)).unwrap();
    let bus = FashionStarBus::open(&slave.name().unwrap()).unwrap();
    (
        StarArmBus {
            bus,
            last_commands: None,
            pending_commands: None,
        },
        peer,
    )
}
fn packet(peer: &mut TTYPort) -> fashionstar_uart::Packet {
    let mut decoder = PacketDecoder::requests();
    loop {
        let mut b = [0];
        peer.read_exact(&mut b).unwrap();
        if let Some(p) = decoder.push(b[0]) {
            return p.unwrap();
        }
    }
}
fn command(value: f64) -> ArmCommand {
    ArmCommand {
        schema_version: SCHEMA_VERSION,
        sequence: 1,
        controller_time_ns: now_ns(),
        model_revision: MODEL_REVISION.into(),
        joints_rad: vec![value; 6],
        actuators_rad: vec![0.],
    }
}
fn poll_until(mut poll: impl FnMut() -> bool) {
    let start = Instant::now();
    while !poll() {
        assert!(start.elapsed() < Duration::from_secs(2));
        std::thread::sleep(Duration::from_millis(1));
    }
}
#[test]
fn monitor_owns_wire_latest_target_waits_without_blocking_or_backlog() {
    let (mut bus, mut peer) = pair();
    bus.bus.begin_monitor_read(&SERVO_IDS).unwrap();
    packet(&mut peer);
    for i in 0..1000 {
        bus.write(&command(f64::from(i) / 10000.), 400).unwrap();
    }
    assert_eq!(
        peer.bytes_to_read().unwrap(),
        0,
        "motion must not interrupt replies"
    );
    assert!(bus.last_commands.is_none(), "queued is not transmitted");
    for id in SERVO_IDS {
        let mut p = [0; 16];
        p[0] = id;
        peer.write_all(&response_packet(22, &p).unwrap()).unwrap();
    }
    poll_until(|| bus.read_state(1).unwrap().is_some());
    bus.flush_motion().unwrap();
    let p = packet(&mut peer);
    assert_eq!(&p.params[..3], &[14, 15, 7]);
    assert_eq!(i32::from_le_bytes(p.params[4..8].try_into().unwrap()), 57);
    assert_eq!(
        peer.bytes_to_read().unwrap(),
        0,
        "only latest target is sent"
    );
    bus.flush_motion().unwrap();
    assert_eq!(peer.bytes_to_read().unwrap(), 0);
}
#[test]
fn information_read_also_owns_wire_and_deferred_target_is_not_lost() {
    let (mut bus, mut peer) = pair();
    bus.bus
        .begin_data_read(DataRequest::Register { id: 4, address: 7 })
        .unwrap();
    packet(&mut peer);
    bus.write(&command(0.1), 400).unwrap();
    assert_eq!(peer.bytes_to_read().unwrap(), 0);
    peer.write_all(&response_packet(3, &[4, 7, 0x25, 2]).unwrap())
        .unwrap();
    poll_until(|| bus.bus.poll_data_read().unwrap().is_some());
    bus.flush_motion().unwrap();
    assert_eq!(packet(&mut peer).code, 25);
}
#[test]
fn stop_discards_queued_target_instead_of_replaying_it() {
    let (mut bus, mut peer) = pair();
    bus.bus.begin_monitor_read(&SERVO_IDS).unwrap();
    packet(&mut peer);
    bus.write(&command(0.1), 400).unwrap();
    // Explicit stop preempts normal motion; no stale target may follow it.
    bus.set_torque(false).unwrap();
    packet(&mut peer);
    bus.flush_motion().unwrap();
    assert!(bus.pending_commands.is_none());
    assert_eq!(peer.bytes_to_read().unwrap(), 0);
}

#[test]
fn repeated_partial_and_delayed_replies_never_interleave_motion() {
    let (mut bus, mut peer) = pair();
    let mut enqueue_us = vec![];
    let mut completed = 0;
    for round in 0u32..100 {
        bus.bus.begin_monitor_read(&SERVO_IDS).unwrap();
        packet(&mut peer);
        for id in SERVO_IDS {
            let now = Instant::now();
            bus.write(&command(f64::from(round) / 1000.), 400).unwrap();
            enqueue_us.push(now.elapsed().as_secs_f64() * 1e6);
            assert_eq!(peer.bytes_to_read().unwrap(), 0);
            let mut p = [0; 16];
            p[0] = id;
            let response = response_packet(22, &p).unwrap();
            peer.write_all(&response[..4]).unwrap();
            assert!(bus.read_state(u64::from(round)).unwrap().is_none());
            peer.write_all(&response[4..]).unwrap();
            if id == 6 {
                poll_until(|| bus.read_state(u64::from(round)).unwrap().is_some());
            } else {
                assert!(bus.read_state(u64::from(round)).unwrap().is_none());
            }
        }
        let send = !changed_servo_commands(
            bus.last_commands.as_ref(),
            bus.pending_commands.as_ref().unwrap(),
        )
        .is_empty();
        bus.flush_motion().unwrap();
        // Successive 0.001 rad targets can quantize to the same 0.1 degree.
        if send {
            assert_eq!(packet(&mut peer).code, 25);
        }
        completed += 1;
    }
    enqueue_us.sort_by(f64::total_cmp);
    println!(
        "SERVO_BENCH {}",
        serde_json::json!({"rounds":completed,"queued_updates":enqueue_us.len(),
        "enqueue_us_median":enqueue_us[enqueue_us.len()/2],"enqueue_us_p95":enqueue_us[enqueue_us.len()*95/100],
        "enqueue_us_max":enqueue_us.last(),"scope":"PTY adapter overhead, not hardware latency"})
    );
}
