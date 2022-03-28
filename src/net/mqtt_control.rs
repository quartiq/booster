//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use crate::{
    hardware::{clock::SystemTimer, setup::MainBus},
    Channel,
};

use minimq::embedded_nal;

use super::NetworkStackProxy;

use core::fmt::Write;
use heapless::String;
use serde::Serialize;

type MinireqResponse = Result<
    minireq::Response<128>,
    minireq::Error<<super::NetworkStack as embedded_nal::TcpClientStack>::Error>,
>;

/// The default telemetry period.
pub const DEFAULT_TELEMETRY_PERIOD_SECS: f32 = 10.0;

/// Specifies a generic request for a specific channel.
#[derive(serde::Deserialize, Debug)]
struct ChannelRequest {
    pub channel: Channel,
}

/// Indicates the result of a channel bias setting request.
#[derive(serde::Serialize)]
struct ChannelBiasResponse {
    pub vgs: f32,
    pub ids: f32,
}

/// Represents a means of handling MQTT-based control interface.
pub struct TelemetryClient {
    mqtt: minimq::Minimq<NetworkStackProxy, SystemTimer, 512, 1>,
    telemetry_prefix: String<128>,
    telemetry_period: f32,
}

impl TelemetryClient {
    /// Construct the MQTT control manager.
    pub fn new(
        broker: minimq::embedded_nal::IpAddr,
        stack: super::NetworkStackProxy,
        id: &str,
    ) -> Self {
        let mut client_id: String<64> = String::new();
        write!(&mut client_id, "booster-{}-tlm", id).unwrap();

        let mut telemetry_prefix: String<128> = String::new();
        write!(&mut telemetry_prefix, "dt/sinara/booster/{}/telemetry", id).unwrap();

        Self {
            mqtt: minimq::Minimq::new(broker, &client_id, stack, SystemTimer::default()).unwrap(),
            telemetry_prefix,
            telemetry_period: DEFAULT_TELEMETRY_PERIOD_SECS,
        }
    }

    /// Publish telemetry for a specific channel.
    ///
    /// # Args
    /// * `channel` - The channel that telemetry is being reported for.
    /// * `telemetry` - The associated telemetry of the channel to report.
    pub fn report_telemetry(&mut self, channel: Channel, telemetry: &impl Serialize) {
        let mut topic: String<64> = String::new();
        write!(&mut topic, "{}/ch{}", self.telemetry_prefix, channel as u8).unwrap();

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
            .ok();
    }

    /// Handle the MQTT-based telemetry interface.
    pub fn update(&mut self) {
        self.mqtt.poll(|_, _, _, _| {}).ok();
    }

    /// Get the period between telemetry updates in CPU cycles.
    pub fn telemetry_period_cycles(&self) -> u32 {
        let period = (crate::CPU_FREQ as f32) * self.telemetry_period;

        // Elapsed cycles must always be less than half of the container size because of the
        // wrapping nature of cycle counting. Specifically, cycles > MAX/2 in the future are
        // indistinguishable from cycles in the past due to integer wrap. Because of this, we cap
        // the cycle period to less than half an integer wrap.
        if period >= (u32::MAX / 2) as f32 {
            u32::MAX / 2 - 1
        } else {
            period as u32
        }
    }

    /// Set the telemetry period.
    ///
    /// # Note
    /// The telemetry period has a minimum period of 0.5 seconds
    ///
    /// # Args
    /// * `period` - The telemetry period in seconds.
    pub fn set_telemetry_period(&mut self, period: f32) {
        self.telemetry_period = period.clamp(0.5, period);
    }
}

/// Read bias transistor parameters.
///
/// # Note
/// This is a handler function for the control interface.
///
/// # Args
/// * `main_bus` - The main I2C bus to communicate with RF channels.
/// * `_topic` - Unused, but reserved for the incoming topic of the request.
/// * `request` - The serialized [ChannelRequest] to process.
///
/// # Returns
/// A [minireq::Response] containing a serialized [ChannelBiasResponse].
pub fn read_bias(main_bus: &mut MainBus, _topic: &str, request: &[u8]) -> MinireqResponse {
    let request: ChannelRequest = serde_json_core::from_slice(request)?.0;

    let response = main_bus
        .channels
        .channel_mut(request.channel)
        .map(|(channel, _)| {
            minireq::Response::data(ChannelBiasResponse {
                vgs: channel.context_mut().get_bias_voltage(),
                ids: channel.context_mut().get_p28v_current(),
            })
        })
        .unwrap_or_else(|| minireq::Response::error("Channel not found"));

    Ok(response)
}

/// Persist channel settings to EEPROM.
///
/// # Note
/// This is a handler function for the control interface.
///
/// # Args
/// * `main_bus` - The main I2C bus to communicate with RF channels.
/// * `_topic` - Unused, but reserved for the incoming topic of the request.
/// * `request` - The serialized [ChannelRequest] to process.
///
/// # Returns
/// A [minireq::Response] containing no data, which indicates the success of the command
/// processing.
pub fn save_settings(main_bus: &mut MainBus, _topic: &str, request: &[u8]) -> MinireqResponse {
    let request: ChannelRequest = serde_json_core::from_slice(request)?.0;

    let response = main_bus
        .channels
        .channel_mut(request.channel)
        .map(|(channel, _)| {
            channel.context_mut().save_configuration();
            minireq::Response::ok()
        })
        .unwrap_or_else(|| minireq::Response::error("Channel not found"));

    Ok(response)
}
