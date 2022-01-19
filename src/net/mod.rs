use crate::hardware::{Channel, clock::EpochClock, AsmDelay, NetworkStack};
use heapless::String;
use serde::Serialize;

mod shared;
mod mqtt_control;

use mqtt_control::ControlState;

use shared::NetworkManager;
type NetworkStackProxy = super::shared::NetworkStackProxy<'static, NetworkStack>;
type MqttClient = minimq::Minimq<NetworkStackProxy, EpochClock, 128, 1>;

pub struct NetworkDevices {
    pub controller: mqtt_control::ControlState,
    pub telemetry: TelemetryClient,
    stack: NetworkStackProxy,
}

impl NetworkDevices {
    pub fn new(broker: minimq::embedded_nal::IpAddr, stack: NetworkStack, identifier: &str, delay: AsmDelay) -> Self {
        let shared = cortex_m::singleton!(: NetworkManager<NetworkStack> = NetworkManager::new(network)).unwrap();

        Self {
            telemetry: TelemetryClient::new(broker, shared.acquire_stack(), identifier),
            controller: ControlState::new(broker, shared.acquire_stack(), identifier, delay),
            stack: shared.acquire_stack(),
        }
    }

    pub fn process(&mut self) -> bool {
        #[cfg(feature = "phy_enc424j600")]
        return self.stack.lock(|stack| stack.poll()).map_err(|_| Ok(true)).unwrap();

        false
    }
}

pub struct TelemetryClient {
    mqtt_client: Mqttclient,
    prefix: String<128>,
}

impl TelemetryClient {

    pub fn new(broker: minimq::embedded_nal::IpAddr, stack: NetworkStackProxy, identifier: &str) {
        let mut prefix: String<128> = String::new();
        write!(&mut prefix, "dt/sinara/{}/telemetry", identifier).unwrap();

        Self {
            mqtt_client: minimq::Minimq::new(broker, &get_client_id(identifier, "tlm"), stack, EpochClock::new()),
            prefix: prefix,
        }
    }

    pub fn report_telemetry(&mut self, telemetry: &impl Serialize, channel: Channel) {
        let mut topic: String<64> = String::new();
        write!(&mut topic, "{}/ch{}", self.prefix, channel as u8).unwrap();

        let message: String<1024> =
            serde_json_core::to_string(telemetry).unwrap();

        // All telemtry is published in a best-effort manner.
        self.mqtt_client.publish(
            topic.as_str(),
            &message.into_bytes(),
            minimq::QoS::AtMostOnce,
            &[],
        ).ok();
    }
}

/// Get an MQTT client ID for a client.
///
/// # Args
/// * `identifier` - Unique Booster identifier.
/// * `client` - The unique tag of the client
///
/// # Returns
/// A client ID that may be used for MQTT client identification.
pub fn get_client_id(
    identifier: &str,
    client: &str,
) -> String<64> {
    let mut identifier = String::new();
    write!(&mut identifier, "{}-{}", identifier, client).unwrap();
    identifier
}
