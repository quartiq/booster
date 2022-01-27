//! Booster network management definitions
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use crate::hardware::{clock::SystemTimer, NetworkStack};

use core::fmt::Write;
use heapless::String;

use crate::delay::AsmDelay;

mod mqtt_control;
mod shared;
mod telemetry;

use mqtt_control::ControlState;
use shared::NetworkManager;

type NetworkStackProxy = shared::NetworkStackProxy<'static, NetworkStack>;

/// Container structure for holding all network devices.
///
/// # Note
/// All devices accessing the shared stack must be contained within a single structure to prevent
/// potential pre-emption when using the `shared` network stack.
pub struct NetworkDevices {
    pub controller: mqtt_control::ControlState,
    pub telemetry: telemetry::TelemetryClient,
    pub settings: miniconf::MqttClient<crate::Settings, NetworkStackProxy, SystemTimer, 128>,

    // The stack reference is only used if the ENC424J600 PHY is used.
    #[allow(dead_code)]
    stack: NetworkStackProxy,
}

impl NetworkDevices {
    /// Construct all of Booster's Network devices.
    ///
    /// # Args
    /// * `broker` - The broker IP address for MQTT.
    /// * `stack` - The network stack to use for communications.
    /// * `identifier` - The unique identifier of this device.
    /// * `delay` - A delay mechanism.
    pub fn new(
        broker: minimq::embedded_nal::IpAddr,
        stack: NetworkStack,
        identifier: &str,
        delay: AsmDelay,
    ) -> Self {
        let shared =
            cortex_m::singleton!(: NetworkManager<NetworkStack> = NetworkManager::new(stack))
                .unwrap();

        let mut miniconf_client: String<128> = String::new();
        write!(&mut miniconf_client, "{}-settings", identifier).unwrap();

        let mut miniconf_prefix: String<128> = String::new();
        write!(&mut miniconf_prefix, "sinara/booster/{}", identifier).unwrap();

        Self {
            telemetry: telemetry::TelemetryClient::new(broker, shared.acquire_stack(), identifier),
            controller: ControlState::new(broker, shared.acquire_stack(), identifier, delay),
            settings: miniconf::MqttClient::new(
                shared.acquire_stack(),
                &miniconf_client,
                &miniconf_prefix,
                broker,
                SystemTimer::default(),
            )
            .unwrap(),
            stack: shared.acquire_stack(),
        }
    }

    /// Process the network stack.
    ///
    /// # Note
    /// This function must be called periodically to handle ingress/egress of packets and update
    /// state management.
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
