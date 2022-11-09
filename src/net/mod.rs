//! Booster network management definitions
//!
//! # Copyright
///! Copyright (C) 2020-2022 QUARTIQ GmbH
use crate::hardware::{setup::MainBus, NetworkStack, SystemTimer};

use core::fmt::Write;
use heapless::String;

pub mod mqtt_control;
mod shared;

use shared::NetworkManager;

type NetworkStackProxy = shared::NetworkStackProxy<'static, NetworkStack>;

/// Container structure for holding all network devices.
///
/// # Note
/// All devices accessing the shared stack must be contained within a single structure to prevent
/// potential pre-emption when using the `shared` network stack.
pub struct NetworkDevices {
    pub telemetry: mqtt_control::TelemetryClient,
    pub settings: miniconf::MqttClient<crate::RuntimeSettings, NetworkStackProxy, SystemTimer, 256>,
    pub control: minireq::Minireq<MainBus, NetworkStackProxy, SystemTimer, 256, 5>,
    stack: NetworkStackProxy,
    manager: crate::hardware::NetworkManager,
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
        devices: crate::hardware::setup::NetworkDevices,
        identifier: &str,
        settings: crate::RuntimeSettings,
        clock: SystemTimer,
        metadata: &'static crate::hardware::metadata::ApplicationMetadata,
    ) -> Self {
        let crate::hardware::setup::NetworkDevices {
            manager,
            network_stack,
        } = devices;
        let shared =
            cortex_m::singleton!(: NetworkManager<NetworkStack> = NetworkManager::new(network_stack))
                .unwrap();

        let mut miniconf_client: String<128> = String::new();
        write!(&mut miniconf_client, "booster-{}-settings", identifier).unwrap();

        let mut minireq_client: String<128> = String::new();
        write!(&mut minireq_client, "booster-{}-req", identifier).unwrap();

        let mut prefix: String<128> = String::new();
        write!(&mut prefix, "dt/sinara/booster/{}", identifier).unwrap();

        let mut control = minireq::Minireq::new(
            shared.acquire_stack(),
            &minireq_client,
            &prefix,
            broker,
            clock,
        )
        .unwrap();

        control
            .register("save", mqtt_control::save_settings)
            .unwrap();
        control
            .register("read-bias", mqtt_control::read_bias)
            .unwrap();

        Self {
            telemetry: mqtt_control::TelemetryClient::new(
                broker,
                shared.acquire_stack(),
                clock,
                identifier,
                metadata,
            ),
            settings: miniconf::MqttClient::new(
                shared.acquire_stack(),
                &miniconf_client,
                &prefix,
                broker,
                clock,
                settings,
            )
            .unwrap(),
            control,
            manager,
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

        self.manager.process();

        self.stack.lock(|stack| stack.poll()).unwrap_or(true)
    }
}
