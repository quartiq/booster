//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use crate::{
    hardware::{booster_channels::BoosterChannels, clock::SystemTimer},
    Channel, MainBus,
};

use super::NetworkStackProxy;

use core::fmt::Write;
use heapless::String;
use minimq::{Property, QoS};

/// Specifies an action to take on a channel.
#[derive(serde::Deserialize, Debug)]
enum ChannelAction {
    Save,
    ReadBiasCurrent,
}

/// Specifies a generic request for a specific channel.
#[derive(serde::Deserialize, Debug)]
struct ChannelRequest {
    pub channel: Channel,
    pub action: ChannelAction,
}

/// Indicates the result of a channel bias setting request.
#[derive(serde::Serialize)]
struct ChannelBiasResponse {
    code: u32,
    pub vgs: f32,
    pub ids: f32,
}

impl ChannelBiasResponse {
    /// Indicate that a channel bias setting command was successfully processed.
    ///
    /// # Args
    /// * `vgs` - The resulting gate voltage of the RF amplifier.
    /// * `ids` - The resulting drain current of the RF amplifier.
    pub fn okay(vgs: f32, ids: f32) -> String<256> {
        let response = Self {
            code: 200,
            vgs,
            ids,
        };

        serde_json_core::to_string(&response).unwrap()
    }
}

/// Represents a generic response to a command.
#[derive(serde::Serialize)]
struct Response {
    code: u32,
    msg: String<256>,
}

impl Response {
    /// Indicate that a command was successfully processed.
    ///
    /// # Args
    /// * `msg` - An additional user-readable message.
    pub fn okay(msg: &str) -> String<256> {
        let response = Response {
            code: 200,
            msg: String::from(msg),
        };

        serde_json_core::to_string(&response).unwrap()
    }

    /// Indicate that a command failed to be processed.
    ///
    /// # Args
    /// * `msg` - An additional user-readable message.
    pub fn error_msg(msg: &str) -> String<256> {
        let response = Response {
            code: 400,
            msg: String::from(msg),
        };

        serde_json_core::to_string(&response).unwrap()
    }
}

/// Represents a means of handling MQTT-based control interface.
pub struct ControlState {
    mqtt: minimq::Minimq<NetworkStackProxy, SystemTimer, 256, 1>,
    subscribed: bool,
    control_topic: String<64>,
    default_response_topic: String<64>,
}

impl ControlState {
    /// Construct the MQTT control state manager.
    pub fn new(
        broker: minimq::embedded_nal::IpAddr,
        stack: super::NetworkStackProxy,
        id: &str,
    ) -> Self {
        let mut client_id: String<64> = String::new();
        write!(&mut client_id, "booster-{}-ctrl", id).unwrap();

        let mut control_topic: String<64> = String::new();
        write!(&mut control_topic, "dt/sinara/booster/{}/control", id).unwrap();

        let mut default_response_topic: String<64> = String::new();
        write!(&mut default_response_topic, "dt/sinara/booster/{}/log", id).unwrap();

        Self {
            mqtt: minimq::Minimq::new(broker, &client_id, stack, SystemTimer::default()).unwrap(),
            subscribed: false,
            control_topic,
            default_response_topic,
        }
    }

    /// Handle the MQTT-based control interface.
    ///
    /// # Args
    /// * `resources` - The `idle` resources containing the client and RF channels.
    pub fn update(&mut self, main_bus: &mut impl rtic::Mutex<T = MainBus>) {
        // Subscribe to any control topics necessary.
        if !self.subscribed && self.mqtt.client.is_connected() {
            self.mqtt
                .client
                .subscribe(&self.control_topic, &[])
                .unwrap();
            self.subscribed = true;
        }

        let response_topic = &self.default_response_topic;
        let expected_topic = &self.control_topic;

        match self.mqtt.poll(|client, topic, message, properties| {
            let response = if topic == expected_topic {
                main_bus.lock(|main_bus| handle_channel_update(message, &mut main_bus.channels))
            } else {
                Response::error_msg("Unexpected topic")
            };

            if let Property::ResponseTopic(topic) = properties
                .iter()
                .find(|&prop| matches!(*prop, Property::ResponseTopic(_)))
                .or(Some(&Property::ResponseTopic(response_topic)))
                .unwrap()
            {
                client
                    .publish(
                        topic,
                        &response.into_bytes(),
                        QoS::AtMostOnce,
                        minimq::Retain::NotRetained,
                        &[],
                    )
                    .unwrap();
            }
        }) {
            Ok(_) => {}

            // Whenever the MQTT broker stops maintaining the session,
            // this MQTT client will reset the session,
            // and we will lose our pending subscriptions.
            // We will need to re-establish them once we reconnect.
            Err(minimq::Error::SessionReset) => self.subscribed = false,

            // Note: There's a race condition where the W5500 may disconnect the socket
            // immediately before Minimq tries to use it. In these cases, a NotReady error is
            // returned to indicate the socket is no longer connected. On the next processing
            // cycle of Minimq, the device should detect and handle the broker disconnection.
            #[cfg(feature = "phy_w5500")]
            Err(minimq::Error::Network(w5500::tcp::TcpSocketError::NotReady)) => {}

            Err(e) => error!("Unexpected error: {:?}", e),
        }
    }
}

/// Handle a request to update a booster RF channel state.
///
/// # Args
/// * `message` - The serialized message request.
/// * `channels` - The booster RF channels to configure.
///
/// # Returns
/// A String response indicating the result of the request.
fn handle_channel_update(message: &[u8], channels: &mut BoosterChannels) -> String<256> {
    let mut response: String<256> = String::new();

    let request = match serde_json_core::from_slice::<ChannelRequest>(message) {
        Ok((data, _)) => data,
        Err(_) => return Response::error_msg("Failed to decode data"),
    };
    channels
        .channel_mut(request.channel)
        .map(|(channel, _)| {
            Response::okay(match request.action {
                ChannelAction::Save => {
                    channel.context_mut().save_configuration();
                    "Channel saved"
                }
                ChannelAction::ReadBiasCurrent => {
                    response = ChannelBiasResponse::okay(
                        channel.context_mut().get_bias_voltage(),
                        channel.context_mut().get_p28v_current(),
                    );
                    &response
                }
            })
        })
        .unwrap_or_else(|| Response::error_msg("Channel not present"))
}
