//! Wire fixtures from vendor protocol examples, plus real PTY transactions.
use super::*;
use crate::monitor_tests::{packet, pair};

fn wait(bus: &mut FashionStarBus) -> Result<CommandReply, Error> {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        if let Some(r) = bus.poll_command()? {
            return Ok(r);
        }
        assert!(Instant::now() < deadline);
        thread::sleep(Duration::from_millis(1));
    }
}
fn register(key: &str) -> Register {
    *REGISTERS.iter().find(|r| r.key == key).unwrap()
}

#[test]
fn startup_ping_ignores_unrelated_ack_and_another_servo() {
    let (mut bus, mut peer) = pair();
    let thread = thread::spawn(move || {
        assert_eq!(
            packet(&mut peer),
            Packet {
                code: 1,
                params: vec![0]
            }
        );
        peer.write_all(&response_packet(24, &[0, 1]).unwrap())
            .unwrap();
        peer.write_all(&response_packet(1, &[3]).unwrap()).unwrap();
        peer.write_all(&response_packet(1, &[0]).unwrap()).unwrap();
        thread::sleep(Duration::from_millis(20));
    });
    bus.ping(0).unwrap();
    thread.join().unwrap();
}

#[test]
fn all_six_position_layouts_match_vendor_sdk_and_documented_examples() {
    let cases = [
        (
            MotionProfile::Single {
                position: 900,
                time_ms: 500,
                power_mw: 0,
            },
            8,
            vec![0, 0x84, 3, 0xf4, 1, 0, 0],
        ),
        (
            MotionProfile::SingleInterval {
                position: 900,
                time_ms: 600,
                accel_ms: 100,
                decel_ms: 200,
                power_mw: 0,
            },
            11,
            vec![0, 0x84, 3, 0x58, 2, 100, 0, 200, 0, 0, 0],
        ),
        (
            MotionProfile::SingleVelocity {
                position: 900,
                speed_tenths_dps: 2000,
                accel_ms: 100,
                decel_ms: 200,
                power_mw: 0,
            },
            12,
            vec![0, 0x84, 3, 0xd0, 7, 100, 0, 200, 0, 0, 0],
        ),
        (
            MotionProfile::Multi {
                position: 4000,
                time_ms: 5000,
                power_mw: 0,
            },
            13,
            vec![0, 0xa0, 0x0f, 0, 0, 0x88, 0x13, 0, 0, 0, 0],
        ),
        (
            MotionProfile::MultiInterval(PositionCommand {
                id: 9,
                position_tenths_degree: 6000,
                motion_time_ms: 1200,
                acceleration_time_ms: 100,
                deceleration_time_ms: 100,
                power_mw: 0,
            }),
            14,
            vec![0, 0x70, 0x17, 0, 0, 0xb0, 4, 0, 0, 100, 0, 100, 0, 0, 0],
        ),
        (
            MotionProfile::MultiVelocity {
                position: 6000,
                speed_tenths_dps: 2000,
                accel_ms: 100,
                decel_ms: 100,
                power_mw: 0,
            },
            15,
            vec![0, 0x70, 0x17, 0, 0, 0xd0, 7, 100, 0, 100, 0, 0, 0],
        ),
    ];
    for (profile, code, params) in cases {
        assert_eq!(
            profile.command(0),
            Packet {
                code,
                params: params.clone()
            }
        );
        assert_eq!(
            synchronized_packet(&[(0, profile)]).unwrap(),
            request_packet(25, &[vec![code, params.len() as u8, 1], params].concat()).unwrap()
        );
    }
}
#[test]
fn origin_stop_async_reset_and_extended_sync_match_vendor_bytes() {
    let cases = [
        (
            ServoCommand::Origin {
                id: 4,
                restore: false,
            },
            23,
            vec![4, 0],
        ),
        (
            ServoCommand::Origin {
                id: 4,
                restore: true,
            },
            23,
            vec![4, 1],
        ),
        (
            ServoCommand::Stop {
                id: 4,
                mode: StopMode::Damping,
                power_mw: 6000,
            },
            24,
            vec![4, 0x12, 0x70, 0x17],
        ),
        (ServoCommand::ResetTurns(4), 17, vec![4]),
        (ServoCommand::BeginAsync, 18, vec![]),
        (ServoCommand::EndAsync { cancel: true }, 19, vec![1]),
    ];
    for (c, code, params) in cases {
        assert_eq!(c.packet().unwrap(), Packet { code, params });
    }
    let p = MotionProfile::Multi {
        position: 0,
        time_ms: 100,
        power_mw: 0,
    };
    let frame = synchronized_packet(&(0..24).map(|id| (id, p)).collect::<Vec<_>>()).unwrap();
    assert_eq!(&frame[..6], &[0x12, 0x4c, 25, 255, 0x0b, 1]); // 3 + 24*11 = 267
    assert_eq!(*frame.last().unwrap(), checksum(&frame[..frame.len() - 1]));
    assert!(synchronized_packet(&[]).is_err());
}
#[test]
fn register_write_requires_matching_ack_and_readback_keeps_signed_value() {
    let (mut bus, mut peer) = pair();
    let r = register("angle_limit_low");
    bus.begin_command(ServoCommand::WriteRegister {
        id: 4,
        register: r,
        value: -650,
    })
    .unwrap();
    assert_eq!(
        packet(&mut peer),
        Packet {
            code: 4,
            params: vec![4, 52, 0x76, 0xfd]
        }
    );
    assert!(bus.begin_monitor_read(&[4]).is_err());
    assert!(
        bus.write_synchronized(&[(
            4,
            MotionProfile::Multi {
                position: 0,
                time_ms: 100,
                power_mw: 0,
            }
        )])
        .is_err()
    );
    assert_eq!(peer.bytes_to_read().unwrap(), 0);
    peer.write_all(&response_packet(4, &[3, 52, 1]).unwrap())
        .unwrap();
    peer.write_all(&response_packet(4, &[4, 51, 1]).unwrap())
        .unwrap();
    peer.write_all(&response_packet(4, &[4, 52, 1]).unwrap())
        .unwrap();
    assert_eq!(wait(&mut bus).unwrap(), CommandReply::Acknowledged);
    bus.begin_command(ServoCommand::ReadRegister { id: 4, register: r })
        .unwrap();
    assert_eq!(packet(&mut peer).params, vec![4, 52]);
    peer.write_all(&response_packet(3, &[4, 52, 0x76, 0xfd]).unwrap())
        .unwrap();
    assert_eq!(wait(&mut bus).unwrap(), CommandReply::Register(-650.0));
}
#[test]
fn failed_ack_and_missing_ack_are_not_verified_success_and_writes_are_not_retried() {
    let (mut bus, mut peer) = pair();
    let r = register("response_switch");
    bus.begin_command(ServoCommand::WriteRegister {
        id: 4,
        register: r,
        value: 0,
    })
    .unwrap();
    packet(&mut peer);
    peer.write_all(&response_packet(4, &[4, 33, 0]).unwrap())
        .unwrap();
    let error = wait(&mut bus).unwrap_err().to_string();
    assert!(error.contains("ID 4"));
    assert!(error.contains("设备返回状态 0x00"));
    bus.begin_command(ServoCommand::WriteRegister {
        id: 4,
        register: r,
        value: 0,
    })
    .unwrap();
    packet(&mut peer);
    assert_eq!(wait(&mut bus).unwrap(), CommandReply::SentUnconfirmed);
    assert_eq!(peer.bytes_to_read().unwrap(), 0);
    bus.begin_monitor_read(&[4]).unwrap();
}
#[test]
fn typed_registers_reject_truncation_and_identity_writes() {
    for r in REGISTERS {
        if r.writable() {
            assert_eq!(r.decode(&r.encode(0).unwrap()).unwrap(), 0.0)
        } else {
            assert!(r.encode(0).is_err())
        }
    }
    assert!(register("response_switch").encode(256).is_err());
    assert!(register("over_power").encode(-1).is_err());
    assert!(register("angle_limit_low").encode(-32769).is_err());
}
#[test]
fn multi_angle_reply_is_signed_and_not_clipped_to_one_turn() {
    let (mut bus, mut peer) = pair();
    bus.begin_command(ServoCommand::ReadMultiAngle(4)).unwrap();
    packet(&mut peer);
    let mut p = vec![4];
    p.extend((-3003i32).to_le_bytes());
    p.extend((-1i16).to_le_bytes());
    peer.write_all(&response_packet(16, &p).unwrap()).unwrap();
    assert_eq!(
        wait(&mut bus).unwrap(),
        CommandReply::Angle {
            position_tenths_degree: -3003,
            turns: Some(-1)
        }
    );
}
#[test]
fn private_writes_keep_vendor_header_and_ack_validation() {
    let (mut bus, mut peer) = pair();
    let mut params = vec![4];
    params.extend([0u8; 25]);
    let value =
        InternalParameters::from_response(4, &response_packet(0xc5, &params).unwrap()).unwrap();
    bus.begin_command(ServoCommand::WriteInternal(value))
        .unwrap();
    let mut bytes = [0u8; 31];
    peer.read_exact(&mut bytes).unwrap();
    assert_eq!(bytes, value.write_request());
    peer.write_all(&response_packet(0xc4, &[4, 1]).unwrap())
        .unwrap();
    assert_eq!(wait(&mut bus).unwrap(), CommandReply::Acknowledged);
    bus.begin_command(ServoCommand::ReadInternal(4)).unwrap();
    let mut read = [0u8; 6];
    peer.read_exact(&mut read).unwrap();
    assert_eq!(&read[..5], &[0x13, 0x4d, 0xc5, 1, 4]);
    peer.write_all(&response_packet(0xc5, &params).unwrap())
        .unwrap();
    assert_eq!(wait(&mut bus).unwrap(), CommandReply::Internal(value));
}
