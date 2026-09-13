//! Explicit, incremental information scan on the SAME bus as normal feedback.
use super::*;
use fashionstar_uart::{DataRequest, REGISTERS, Register};

pub(super) struct InformationScan {
    queue: std::collections::VecDeque<(u8, Option<Register>)>,
    pub values: Vec<ParameterValue>,
    pub completed: usize,
    pub total: usize,
}
impl InformationScan {
    pub fn new() -> Self {
        let queue: std::collections::VecDeque<_> = SERVO_IDS
            .into_iter()
            .flat_map(|id| {
                REGISTERS
                    .iter()
                    .copied()
                    .map(move |r| (id, Some(r)))
                    .chain(std::iter::once((id, None)))
            })
            .collect();
        Self {
            total: queue.len(),
            queue,
            values: vec![],
            completed: 0,
        }
    }
    pub fn done(&self) -> bool {
        self.queue.is_empty()
    }
    pub fn poll(&mut self, bus: &mut FashionStarBus) -> bool {
        let Some(&(id, register)) = self.queue.front() else {
            return false;
        };
        if bus.monitor_read_pending() {
            return false;
        }
        let result = if bus.data_read_pending() {
            bus.poll_data_read()
        } else {
            bus.begin_data_read(match register {
                Some(r) => DataRequest::Register {
                    id,
                    address: r.address,
                },
                None => DataRequest::Internal { id },
            })
            .map(|_| None)
        };
        let result = match result {
            Ok(None) => return false,
            Ok(Some(bytes)) => Ok(bytes),
            Err(e) => Err(e.to_string()),
        };
        let time = now_ns();
        let parsed = result.and_then(|bytes| match register {
            Some(r) => r
                .decode(&bytes)
                .map(|value| {
                    vec![ParameterValue {
                        actuator_key: actuator_key(id),
                        field_key: r.key.into(),
                        value: Some(value),
                        unit: r.unit.into(),
                        read_time_ns: time,
                        original_error: None,
                    }]
                })
                .map_err(|e| e.to_string()),
            None => InternalParameters::from_response(id, &bytes)
                .map(|v| parameter_values(v, time))
                .map_err(|e| e.to_string()),
        });
        self.values.extend(parsed.unwrap_or_else(|error| {
            vec![ParameterValue {
                actuator_key: actuator_key(id),
                field_key: register.map_or("internal_parameters", |r| r.key).into(),
                value: None,
                unit: register.map_or("", |r| r.unit).into(),
                read_time_ns: time,
                original_error: Some(error),
            }]
        }));
        self.queue.pop_front();
        self.completed += 1;
        true
    }
}
