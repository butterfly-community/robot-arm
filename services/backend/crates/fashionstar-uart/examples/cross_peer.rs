use fashionstar_uart::{
    CODE_PING, CODE_QUERY_MONITOR, CODE_SET_MTURN_BY_INTERVAL, CODE_SYNC_COMMAND, FashionStarBus,
    Packet, PacketDecoder, PositionCommand, degrees_to_tenths, response_packet,
};
use std::{
    env,
    error::Error,
    fs::OpenOptions,
    io::{self, Read, Write},
};

const IDS: [u8; 7] = [0, 1, 2, 3, 4, 5, 6];
const MOTION_TIME_MS: u32 = 350;
const ACCELERATION_TIME_MS: u16 = 50;
const DECELERATION_TIME_MS: u16 = 50;

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let mode = args.next().ok_or("缺少 client/server 参数")?;
    if mode == "rounding" {
        for value in args {
            println!("{}", degrees_to_tenths(value.parse()?)?);
        }
        return Ok(());
    }
    let port = args.next().ok_or("缺少串口路径")?;
    let rounds = args.next().ok_or("缺少轮数")?.parse::<u32>()?;
    match mode.as_str() {
        "client" => run_client(&port, rounds),
        "server" => run_server(&port, rounds),
        _ => Err(format!("未知模式 {mode}").into()),
    }
}

fn run_client(port: &str, rounds: u32) -> Result<(), Box<dyn Error>> {
    let mut bus = FashionStarBus::open(port)?;
    for id in IDS {
        bus.ping(id)?;
    }
    for round in 0..rounds {
        let monitors = bus.read_monitors(&IDS)?;
        if monitors.len() != IDS.len() {
            return Err("Monitor 数量不正确".into());
        }
        for monitor in monitors {
            if monitor.position_tenths_degree != monitor_position(round, monitor.id) {
                return Err(format!("Monitor ID {} 位置不一致", monitor.id).into());
            }
            if monitor.voltage_mv != 7400 + u16::from(monitor.id)
                || monitor.current_ma != 100 + u16::from(monitor.id)
                || monitor.power_mw != 200 + u16::from(monitor.id)
                || monitor.temperature_raw != 2048
                || monitor.status != round as u8
                || monitor.turns != i16::from(monitor.id) - 3
            {
                return Err(format!("Monitor ID {} 字段不一致", monitor.id).into());
            }
        }
        let commands = IDS.map(|id| PositionCommand {
            id,
            position_tenths_degree: command_position(round, id),
            motion_time_ms: MOTION_TIME_MS,
            acceleration_time_ms: ACCELERATION_TIME_MS,
            deceleration_time_ms: DECELERATION_TIME_MS,
            power_mw: 0,
        });
        bus.write_positions(&commands)?;
    }
    Ok(())
}

fn run_server(port: &str, rounds: u32) -> Result<(), Box<dyn Error>> {
    let mut port = OpenOptions::new().read(true).write(true).open(port)?;
    let mut decoder = PacketDecoder::requests();
    for expected_id in IDS {
        let packet = read_packet(&mut port, &mut decoder)?;
        require(
            packet.code == CODE_PING && packet.params == [expected_id],
            "Ping 请求不一致",
        )?;
        port.write_all(&response_packet(CODE_PING, &[expected_id])?)?;
    }
    for round in 0..rounds {
        let monitor_request = read_packet(&mut port, &mut decoder)?;
        require(
            monitor_request.code == CODE_SYNC_COMMAND
                && monitor_request.params == [CODE_QUERY_MONITOR, 1, 7, 0, 1, 2, 3, 4, 5, 6],
            "Monitor 请求不一致",
        )?;
        for id in IDS.into_iter().rev() {
            port.write_all(&monitor_response(round, id)?)?;
        }

        let write = read_packet(&mut port, &mut decoder)?;
        validate_position_write(&write, round)?;
    }
    Ok(())
}

fn read_packet(
    reader: &mut impl Read,
    decoder: &mut PacketDecoder,
) -> Result<Packet, Box<dyn Error>> {
    loop {
        let mut byte = [0];
        reader.read_exact(&mut byte)?;
        if let Some(result) = decoder.push(byte[0]) {
            return result.map_err(Into::into);
        }
    }
}

fn monitor_response(round: u32, id: u8) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut params = vec![id];
    params.extend_from_slice(&(7400 + u16::from(id)).to_le_bytes());
    params.extend_from_slice(&(100 + u16::from(id)).to_le_bytes());
    params.extend_from_slice(&(200 + u16::from(id)).to_le_bytes());
    params.extend_from_slice(&2048_u16.to_le_bytes());
    params.push(round as u8);
    params.extend_from_slice(&monitor_position(round, id).to_le_bytes());
    params.extend_from_slice(&(i16::from(id) - 3).to_le_bytes());
    Ok(response_packet(CODE_QUERY_MONITOR, &params)?)
}

fn validate_position_write(packet: &Packet, round: u32) -> Result<(), Box<dyn Error>> {
    require(packet.code == CODE_SYNC_COMMAND, "同步写功能码不一致")?;
    require(packet.params.len() == 108, "同步写参数长度不一致")?;
    require(
        packet.params[..3] == [CODE_SET_MTURN_BY_INTERVAL, 15, 7],
        "同步写子命令头不一致",
    )?;
    for (index, data) in packet.params[3..].chunks_exact(15).enumerate() {
        let id = index as u8;
        require(data[0] == id, "同步写 ID 不一致")?;
        require(
            i32::from_le_bytes(data[1..5].try_into().unwrap()) == command_position(round, id),
            "同步写位置不一致",
        )?;
        require(
            u32::from_le_bytes(data[5..9].try_into().unwrap()) == MOTION_TIME_MS
                && u16::from_le_bytes(data[9..11].try_into().unwrap()) == ACCELERATION_TIME_MS
                && u16::from_le_bytes(data[11..13].try_into().unwrap()) == DECELERATION_TIME_MS
                && u16::from_le_bytes(data[13..15].try_into().unwrap()) == 0,
            "同步写时间或功率字段不一致",
        )?;
    }
    Ok(())
}

fn monitor_position(round: u32, id: u8) -> i32 {
    (round as i32 - 32) * 10 + i32::from(id)
}

fn command_position(round: u32, id: u8) -> i32 {
    match id {
        0 => i32::MIN + round as i32,
        6 => i32::MAX - round as i32,
        _ => (round as i32 - 32) * 100 + i32::from(id),
    }
}

fn require(condition: bool, message: &str) -> Result<(), io::Error> {
    if condition {
        Ok(())
    } else {
        Err(io::Error::new(io::ErrorKind::InvalidData, message))
    }
}
