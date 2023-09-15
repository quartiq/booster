//! Booster network management definitions

use crate::hardware::{setup::MainBus, NetworkStack, SystemTimer};

use core::fmt::Write;
use heapless::String;

pub mod mqtt_control;

type NetworkStackProxy = smoltcp_nal::shared::NetworkStackProxy<'static, NetworkStack>;

pub struct MqttStorage {
    telemetry: [u8; 1024],
    settings: [u8; 1024],
    control: [u8; 1024],
    minireq_handlers: [minireq::HandlerSlot<'static, MainBus, mqtt_control::Error>; 2],
}

impl Default for MqttStorage {
    fn default() -> Self {
        Self {
            telemetry: [0u8; 1024],
            settings: [0u8; 1024],
            control: [0u8; 1024],
            minireq_handlers: [None, None],
        }
    }
}

/// Container structure for holding all network devices.
///
/// # Note
/// All devices accessing the shared stack must be contained within a single structure to prevent
/// potential pre-emption when using the `shared` network stack.
pub struct NetworkDevices {
    pub telemetry: mqtt_control::TelemetryClient,
    pub settings: miniconf::MqttClient<
        'static,
        crate::RuntimeSettings,
        NetworkStackProxy,
        SystemTimer,
        minireq::minimq::broker::IpBroker,
        4,
    >,
    pub control: minireq::Minireq<
        'static,
        MainBus,
        NetworkStackProxy,
        SystemTimer,
        minireq::minimq::broker::IpBroker,
        mqtt_control::Error,
    >,
    stack: NetworkStackProxy,
}

impl NetworkDevices {
    /// Construct all of Booster's Network devices.
    ///
    /// # Args
    /// * `broker` - The broker IP address for MQTT.
    /// * `stack` - The network stack to use for communications.
    /// * `identifier` - The unique identifier of this device.
    pub fn new(
        broker: minimq::embedded_nal::IpAddr,
        stack: NetworkStack,
        identifier: &str,
        settings: crate::RuntimeSettings,
        clock: SystemTimer,
        metadata: &'static crate::hardware::metadata::ApplicationMetadata,
    ) -> Self {
        let shared =
            cortex_m::singleton!(: smoltcp_nal::shared::NetworkManager<'static, crate::hardware::Mac, crate::hardware::SystemTimer> = smoltcp_nal::shared::NetworkManager::new(stack))
                .unwrap();

        let store = cortex_m::singleton!(: MqttStorage = MqttStorage::default()).unwrap();

        let mut prefix: String<128> = String::new();
        write!(&mut prefix, "dt/sinara/booster/{}", identifier).unwrap();

        let control = {
            let mut client_id: String<128> = String::new();
            write!(&mut client_id, "booster-{}-req", identifier).unwrap();

            let broker = minireq::minimq::broker::IpBroker::new(broker);
            let config = minireq::minimq::ConfigBuilder::new(broker, &mut store.settings)
                .client_id(&client_id)
                .unwrap();
            let mqtt = minireq::minimq::Minimq::new(shared.acquire_stack(), clock, config);

            let mut control =
                minireq::Minireq::new(&prefix, mqtt, &mut store.minireq_handlers).unwrap();

            control
                .register("save", mqtt_control::save_settings)
                .unwrap();
            control
                .register("read-bias", mqtt_control::read_bias)
                .unwrap();

            control
        };

        let telemetry = {
            let mut client_id: String<64> = String::new();
            write!(&mut client_id, "booster-{}-tlm", identifier).unwrap();

            let broker = minireq::minimq::broker::IpBroker::new(broker);
            let config = miniconf::minimq::ConfigBuilder::new(broker, &mut store.telemetry)
                // The telemetry client doesn't do much in terms of receiving data, so reserve the
                // buffer for transmission.
                .rx_buffer(miniconf::minimq::config::BufferConfig::Maximum(100))
                .client_id(&client_id)
                .unwrap();
            mqtt_control::TelemetryClient::new(
                minimq::Minimq::new(shared.acquire_stack(), clock, config),
                metadata,
                &prefix,
            )
        };

        let settings = {
            let mut client_id: String<128> = String::new();
            write!(&mut client_id, "booster-{}-settings", identifier).unwrap();

            let broker = minireq::minimq::broker::IpBroker::new(broker);
            let config = miniconf::minimq::ConfigBuilder::new(broker, &mut store.control)
                .client_id(&client_id)
                .unwrap();
            miniconf::MqttClient::new(shared.acquire_stack(), &prefix, clock, settings, config)
                .unwrap()
        };

        Self {
            telemetry,
            control,
            settings,
            stack: shared.acquire_stack(),
        }
    }

    /// Process the network stack.
    ///
    /// # Note
    /// This function must be called periodically to handle ingress/egress of packets and update
    /// state management.
    pub fn process(&mut self) -> bool {
        self.telemetry.update();

        self.stack.lock(|stack| stack.poll()).unwrap_or(true)
    }
}
