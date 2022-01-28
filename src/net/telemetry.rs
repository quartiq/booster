//! Booster telemetry client management
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::NetworkStackProxy;
use crate::hardware::{clock::SystemTimer, Channel};
use core::fmt::Write;
use heapless::String;
use serde::Serialize;

/// Telemetry reporting MQTT client.
pub struct TelemetryClient {
    mqtt: minimq::Minimq<NetworkStackProxy, SystemTimer, 512, 1>,
    prefix: String<128>,
}

impl TelemetryClient {
    /// Construct the telemetry client.
    ///
    /// # Args
    /// * `broker` - The Booster MQTT broker IP address.
    /// * `stack` - A reference to the shared network stack.
    /// * `identifier` - The unique Booster ID.
    pub fn new(
        broker: minimq::embedded_nal::IpAddr,
        stack: NetworkStackProxy,
        identifier: &str,
    ) -> Self {
        let mut prefix: String<128> = String::new();
        write!(&mut prefix, "dt/sinara/booster/{}/telemetry", identifier).unwrap();

        let mut client_id: String<64> = String::new();
        write!(&mut client_id, "booster-{}-tlm", identifier).unwrap();

        Self {
            mqtt: minimq::Minimq::new(broker, &client_id, stack, SystemTimer::default()).unwrap(),
            prefix: prefix,
        }
    }

    /// Publish telemetry for a specific channel.
    ///
    /// # Args
    /// * `channel` - The channel that telemetry is being reported for.
    /// * `telemetry` - The associated telemetry of the channel to report.
    pub fn report_telemetry(&mut self, channel: Channel, telemetry: &impl Serialize) {
        let mut topic: String<64> = String::new();
        write!(&mut topic, "{}/ch{}", self.prefix, channel as u8).unwrap();

        let message: String<1024> = serde_json_core::to_string(telemetry).unwrap();

        // All telemtry is published in a best-effort manner.
        self.mqtt
            .client
            .publish(
                topic.as_str(),
                &message.into_bytes(),
                minimq::QoS::AtMostOnce,
                minimq::Retain::NotRetained,
                &[],
            )
            .unwrap();
    }

    /// Update the telemetry client.
    ///
    /// # Note
    /// This must be called periodically to advance the MQTT state machine.
    pub fn process(&mut self) {
        self.mqtt
            .poll(|_client, _topic, _message, _properties| {})
            .ok();
    }
}
