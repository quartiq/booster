//! Booster NGFW Application

use crate::{
    hardware::{metadata::ApplicationMetadata, setup::MainBus, SystemTimer},
    Channel,
};

use minimq::{DeferredPublication, Publication};

use super::NetworkStackProxy;

use core::fmt::Write;
use heapless::String;
use serde::Serialize;

/// Default metadata message if formatting errors occur.
const DEFAULT_METADATA: &str = "{\"message\":\"Truncated: See USB terminal\"}";

/// The default telemetry period.
pub const DEFAULT_TELEMETRY_PERIOD_SECS: u64 = 10;

pub enum Error {
    JsonDe(serde_json_core::de::Error),
    JsonSer(serde_json_core::ser::Error),
    Other(&'static str),
}

impl From<serde_json_core::de::Error> for Error {
    fn from(e: serde_json_core::de::Error) -> Self {
        Self::JsonDe(e)
    }
}

impl From<serde_json_core::ser::Error> for Error {
    fn from(e: serde_json_core::ser::Error) -> Self {
        Self::JsonSer(e)
    }
}

impl From<&'static str> for Error {
    fn from(e: &'static str) -> Self {
        Self::Other(e)
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Other(msg) => {
                write!(f, "{}", msg)
            }
            Error::JsonDe(e) => {
                write!(f, "{}", e)
            }
            Error::JsonSer(e) => {
                write!(f, "{}", e)
            }
        }
    }
}

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
    mqtt: minimq::Minimq<
        'static,
        NetworkStackProxy,
        SystemTimer,
        minimq::broker::NamedBroker<NetworkStackProxy>,
    >,
    prefix: String<128>,
    telemetry_period: u64,
    meta_published: bool,
    metadata: &'static ApplicationMetadata,
}

impl TelemetryClient {
    /// Construct the MQTT control manager.
    pub fn new(
        mqtt: minimq::Minimq<
            'static,
            NetworkStackProxy,
            SystemTimer,
            minimq::broker::NamedBroker<NetworkStackProxy>,
        >,
        metadata: &'static ApplicationMetadata,
        prefix: &str,
    ) -> Self {
        Self {
            mqtt,
            prefix: String::from(prefix),
            telemetry_period: DEFAULT_TELEMETRY_PERIOD_SECS,
            meta_published: false,
            metadata,
        }
    }

    /// Publish telemetry for a specific channel.
    ///
    /// # Args
    /// * `channel` - The channel that telemetry is being reported for.
    /// * `telemetry` - The associated telemetry of the channel to report.
    pub fn report_telemetry(&mut self, channel: Channel, telemetry: &impl Serialize) {
        let mut topic: String<64> = String::new();
        write!(&mut topic, "{}/telemetry/ch{}", self.prefix, channel as u8).unwrap();

        // All telemtry is published in a best-effort manner.
        self.mqtt
            .client()
            .publish(
                DeferredPublication::new(|buf| serde_json_core::to_slice(telemetry, buf))
                    .topic(&topic)
                    .finish()
                    .unwrap(),
            )
            .ok();
    }

    /// Handle the MQTT-based telemetry interface.
    pub fn update(&mut self) {
        self.mqtt.poll(|_, _, _, _| {}).ok();

        if !self.mqtt.client().is_connected() {
            self.meta_published = false;
            return;
        }

        // If the metadata has not yet been published, but we can publish it, do so now.
        if !self.meta_published && self.mqtt.client().can_publish(minimq::QoS::AtMostOnce) {
            let mut topic: String<64> = String::new();
            write!(&mut topic, "{}/alive/meta", self.prefix).unwrap();

            let Self {
                ref mut mqtt,
                metadata,
                ..
            } = self;

            if mqtt
                .client()
                .publish(
                    DeferredPublication::new(|buf| serde_json_core::to_slice(&metadata, buf))
                        .topic(&topic)
                        .finish()
                        .unwrap(),
                )
                .is_err()
            {
                // Note(unwrap): We can guarantee that this message will be sent because we checked
                // for ability to publish above.
                mqtt.client()
                    .publish(
                        Publication::new(DEFAULT_METADATA.as_bytes())
                            .topic(&topic)
                            .finish()
                            .unwrap(),
                    )
                    .unwrap();
            }

            self.meta_published = true;
        }
    }

    /// Get the period between telemetry updates in CPU cycles.
    pub fn telemetry_period_secs(&self) -> u64 {
        self.telemetry_period
    }

    /// Set the telemetry period.
    ///
    /// # Note
    /// The telemetry period has a minimum period of 1 seconds
    ///
    /// # Args
    /// * `period` - The telemetry period in seconds.
    pub fn set_telemetry_period(&mut self, period: u64) {
        self.telemetry_period = period.clamp(1, period);
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
pub fn read_bias(
    main_bus: &mut MainBus,
    _topic: &str,
    request: &[u8],
    output: &mut [u8],
) -> Result<usize, Error> {
    let request: ChannelRequest = serde_json_core::from_slice(request)?.0;

    let Some((channel, _)) = main_bus.channels.channel_mut(request.channel) else {
        return Err("Channel not found".into());
    };
    let response = ChannelBiasResponse {
        vgs: channel.context_mut().get_bias_voltage(),
        ids: channel.context_mut().get_p28v_current(),
    };

    Ok(serde_json_core::to_slice(&response, output)?)
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
pub fn save_settings(
    main_bus: &mut MainBus,
    _topic: &str,
    request: &[u8],
    _buffer: &mut [u8],
) -> Result<usize, Error> {
    let request: ChannelRequest = serde_json_core::from_slice(request)?.0;

    let Some((channel, _)) = main_bus.channels.channel_mut(request.channel) else {
        return Err("Channel not found".into());
    };

    channel.context_mut().save_configuration();

    Ok(0)
}
