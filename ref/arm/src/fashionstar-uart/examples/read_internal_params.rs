use fashionstar_uart::FashionStarBus;
use std::{env, error::Error};

const SERVO_IDS: [u8; 7] = [0, 1, 2, 3, 4, 5, 6];
fn main() -> Result<(), Box<dyn Error>> {
    let path = env::args().nth(1).ok_or("缺少串口路径")?;
    let mut bus = FashionStarBus::open(&path)?;

    println!("id\tkp\tkd\tki\tbias\thold_kp\thold_kd\thold_bias\tdirection\tdead_zone");
    for id in SERVO_IDS {
        let value = bus.read_internal_parameters(id)?;
        println!(
            "{id}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
            value.kp,
            value.kd,
            value.ki,
            value.bias,
            value.hold_kp,
            value.hold_kd,
            value.hold_bias,
            value.direction,
            value.dead_zone,
        );
    }
    Ok(())
}
