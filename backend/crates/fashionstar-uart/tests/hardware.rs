//! Explicit standalone hardware benchmark. The execution service must first
//! release the port through its UI. Never run concurrently with another owner.
//! Read-only: no torque, motion, zero, ID, baud or EEPROM changes.
use fashionstar_uart::{DataRequest, FashionStarBus, REGISTERS};
use std::{
    thread,
    time::{Duration, Instant},
};

fn statistics(name: &str, values: &mut [f64]) {
    values.sort_by(f64::total_cmp);
    println!(
        "{name} count={} min_ms={:.3} median_ms={:.3} p95_ms={:.3} max_ms={:.3}",
        values.len(),
        values[0],
        values[values.len() / 2],
        values[values.len() * 95 / 100],
        values[values.len() - 1]
    );
}
#[test]
#[ignore = "requires exclusive real serial port via SERVO_TEST_PORT"]
fn independent_real_bus_three_rounds() {
    let port = std::env::var("SERVO_TEST_PORT").expect("explicit SERVO_TEST_PORT required");
    let ids = [0, 1, 2, 3, 4, 5, 6];
    let mut bus = FashionStarBus::open(&port).unwrap();
    for round in 1..=3 {
        let mut ping_ms = vec![];
        for id in ids {
            let start = Instant::now();
            bus.ping(id).unwrap();
            ping_ms.push(start.elapsed().as_secs_f64() * 1000.);
        }
        let mut read_ms = vec![];
        for _ in 0..100 {
            let start = Instant::now();
            let mut values = bus.read_monitors(&ids).unwrap();
            read_ms.push(start.elapsed().as_secs_f64() * 1000.);
            values.sort_by_key(|v| v.id);
            assert_eq!(values.iter().map(|v| v.id).collect::<Vec<_>>(), ids);
            thread::sleep(Duration::from_millis(200)); // test load, not production rate
        }
        let start = Instant::now();
        let mut fields = 0;
        for id in ids {
            for register in REGISTERS {
                bus.begin_data_read(DataRequest::Register {
                    id,
                    address: register.address,
                })
                .unwrap();
                loop {
                    if let Some(bytes) = bus.poll_data_read().unwrap() {
                        let value = register.decode(&bytes).unwrap();
                        if register.key == "firmware_version" {
                            println!("round={round} id={id} firmware_raw={value}");
                        }
                        fields += 1;
                        break;
                    }
                    thread::sleep(Duration::from_millis(1));
                }
            }
        }
        println!(
            "round={round} public_fields={fields} scan_ms={:.3}",
            start.elapsed().as_secs_f64() * 1000.
        );
        statistics("ping", &mut ping_ms);
        statistics("monitor_7", &mut read_ms);
    }
}
