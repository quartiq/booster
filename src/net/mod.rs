use crate::hardware::NetworkStack;

use crate::delay::AsmDelay;

mod mqtt_control;
mod shared;
mod telemetry;

use mqtt_control::ControlState;

use shared::NetworkManager;
type NetworkStackProxy = shared::NetworkStackProxy<'static, NetworkStack>;

pub struct NetworkDevices {
    pub controller: mqtt_control::ControlState,
    pub telemetry: telemetry::TelemetryClient,

    // The stack reference is only used if the ENC424J600 PHY is used.
    #[allow(dead_code)]
    stack: NetworkStackProxy,
}

impl NetworkDevices {
    pub fn new(
        broker: minimq::embedded_nal::IpAddr,
        stack: NetworkStack,
        identifier: &str,
        delay: AsmDelay,
    ) -> Self {
        let shared =
            cortex_m::singleton!(: NetworkManager<NetworkStack> = NetworkManager::new(stack))
                .unwrap();

        Self {
            telemetry: telemetry::TelemetryClient::new(broker, shared.acquire_stack(), identifier),
            controller: ControlState::new(broker, shared.acquire_stack(), identifier, delay),
            stack: shared.acquire_stack(),
        }
    }

    pub fn process(&mut self) -> bool {
        self.telemetry.process();

        #[cfg(feature = "phy_enc424j600")]
        return self
            .stack
            .lock(|stack| stack.poll())
            .map_err(|_| Ok(true))
            .unwrap();

        false
    }
}
