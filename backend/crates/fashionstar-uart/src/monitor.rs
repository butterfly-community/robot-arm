//! Incremental Monitor transaction, inspired by the vendor Browser SDK's
//! serial_servo_bus.js readLoop / independently awaited write(). No motion filter.
use crate::{CODE_QUERY_MONITOR, DEFAULT_TIMEOUT, Error, Monitor, Packet};
use std::time::Instant;

pub(crate) struct MonitorRead {
    pub ids: Vec<u8>,
    pub monitors: Vec<Monitor>,
    pub first_error: Option<String>,
    last_byte: Instant,
}

impl MonitorRead {
    pub fn new(ids: &[u8], now: Instant) -> Result<Self, Error> {
        if ids.is_empty() || ids.iter().enumerate().any(|(i, id)| ids[..i].contains(id)) {
            return Err(Error::Protocol("Monitor ID 必须非空且不重复".into()));
        }
        Ok(Self {
            ids: ids.to_vec(),
            monitors: Vec::with_capacity(ids.len()),
            first_error: None,
            last_byte: now,
        })
    }

    pub fn received_bytes(&mut self, now: Instant) {
        self.last_byte = now;
    }

    pub fn accept(&mut self, packet: Result<Packet, Error>) -> Result<(), Error> {
        // A corrupt frame can be followed by a valid one in the same stream.
        let Ok(packet) = packet else {
            return Ok(());
        };
        if packet.code != CODE_QUERY_MONITOR {
            return Ok(()); // Includes optional position-control acknowledgements.
        }
        let monitor = Monitor::from_params(&packet.params)?;
        if !self.ids.contains(&monitor.id) || self.monitors.iter().any(|m| m.id == monitor.id) {
            return Err(Error::Protocol(format!(
                "Monitor 返回了无效或重复的 ID {}",
                monitor.id
            )));
        }
        self.monitors.push(monitor);
        // Motion acknowledgements must not keep a lost Monitor query alive.
        self.received_bytes(Instant::now());
        Ok(())
    }

    pub fn complete(&self) -> bool {
        self.monitors.len() == self.ids.len()
    }

    pub fn check_timeout(&self, now: Instant) -> Result<(), Error> {
        if !self.complete() && now.duration_since(self.last_byte) >= DEFAULT_TIMEOUT {
            return Err(Error::Protocol("Monitor 回读超时".into()));
        }
        Ok(())
    }

    pub fn retry(&mut self, error: String, now: Instant) {
        self.first_error = Some(error);
        self.monitors.clear();
        self.last_byte = now;
    }
}
