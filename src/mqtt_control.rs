//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::{idle::Resources, BoosterChannels, Channel, Error};
use minimq::{Property, QoS};
use heapless::{consts, String};
use core::fmt::Write;

/// Specifies a generic request for a specific channel.
#[derive(serde::Deserialize)]
struct ChannelRequest {
    pub channel: Channel,
}

/// Specifies the desired channel RF bias current.
#[derive(serde::Deserialize)]
struct ChannelTuneRequest {
    pub channel: Channel,
    pub current: f32,
}

/// Indicates the result of a channel tuning request.
#[derive(serde::Serialize)]
struct ChannelTuneResponse {
    code: u32,
    pub vgs: f32,
    pub ids: f32,
}

/// Specifies the desired interlock thresholds for a channel.
#[derive(serde::Deserialize)]
struct ChannelThresholds {
    pub channel: Channel,
    pub reflected_power: f32,
    pub output_power: f32,
}

impl ChannelTuneResponse {
    /// Indicate that a channel bias tuning command was successfully processed.
    ///
    /// # Args
    /// * `vgs` - The resulting gate voltage of the RF amplifier.
    /// * `ids` - The resulting drain current of the RF amplifier.
    pub fn okay(vgs: f32, ids: f32) -> String<consts::U256> {
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
    msg: String<heapless::consts::U256>,
}

impl Response {
    /// Indicate that a command was successfully processed.
    ///
    /// # Args
    /// * `msg` - An additional user-readable message.
    pub fn okay<'a>(msg: &'a str) -> String<consts::U256> {
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
    pub fn error_msg<'a>(msg: &'a str) -> String<consts::U256> {
        let response = Response {
            code: 400,
            msg: String::from(msg),
        };

        serde_json_core::to_string(&response).unwrap()
    }

    /// Indicate that a command failed to be processed.
    ///
    /// # Args
    /// * `error` - The error that was encountered while the command was being processed.
    pub fn error(error: Error) -> String<consts::U256> {
        let mut msg = String::<consts::U256>::new();
        write!(&mut msg, "{:?}", error).unwrap();

        let response = Response { code: 400, msg };

        serde_json_core::to_string(&response).unwrap()
    }
}

/// Represents a means of handling MQTT-based control interface.
pub struct ControlState {
    subscribed: bool,
}

impl ControlState {
    /// Construct the MQTT control state manager.
    pub fn new() -> Self {
        Self { subscribed: false }
    }

    /// Handle the MQTT-based control interface.
    ///
    /// # Args
    /// * `resources` - The `idle` resources containing the client and RF channels.
    pub fn update(&mut self, resources: &mut Resources) {
        use rtic::Mutex as _;
        // Subscribe to any control topics necessary.
        if !self.subscribed {
            resources.mqtt_client.lock(|client| {
                if client.is_connected() {
                    client.subscribe("booster/enable", &[]).unwrap();
                    client.subscribe("booster/disable", &[]).unwrap();
                    client.subscribe("booster/tune", &[]).unwrap();
                    client.subscribe("booster/thresholds", &[]).unwrap();
                    self.subscribed = true;
                }
            });
        }

        let channels = &mut resources.channels;

        resources.mqtt_client.lock(|client| {
            match client
                .poll(|client, topic, message, properties| {
                    channels.lock(|channels| {
                        let response = match topic {
                            "booster/enable" => handle_channel_enable(message, channels),
                            "booster/disable" => handle_channel_disable(message, channels),
                            "booster/tune" => handle_channel_tune(message, channels),
                            "booster/thresholds" => handle_channel_thresholds(message, channels),
                            _ => Response::error_msg("Unexpected topic"),
                        };

                        if let Property::ResponseTopic(topic) = properties
                            .iter()
                            .find(|&prop| {
                                if let minimq::Property::ResponseTopic(_) = *prop {
                                    true
                                } else {
                                    false
                                }
                            })
                            .or(Some(&Property::ResponseTopic("booster/log")))
                            .unwrap()
                        {
                            client
                                .publish(topic, &response.into_bytes(), QoS::AtMostOnce, &[])
                                .unwrap();
                        }
                    });
                }) {
                Ok(_) => {},

                // Whenever MQTT disconnects, we will lose our pending subscriptions. We will need
                // to re-establish them once we reconnect.
                Err(minimq::Error::Disconnected) => self.subscribed = false,

                Err(e) => panic!("Unexpected error: {:?}", e),
            }
        });
    }
}

/// Handle a request to enable a booster RF output.
///
/// # Args
/// * `message` - The serialized message request.
/// * `channels` - The booster RF channels to configure.
///
/// # Returns
/// A String response indicating the result of the request.
fn handle_channel_enable(message: &[u8], channels: &mut BoosterChannels) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelRequest>(message) {
        Ok(data) => data,
        Err(_) => return Response::error_msg("Failed to decode data"),
    };

    match channels.enable_channel(request.channel) {
        Ok(_) => Response::okay("Channel enabled"),
        Err(error) => Response::error(error),
    }
}

/// Handle a request to disable a booster RF output.
///
/// # Args
/// * `message` - The serialized message request.
/// * `channels` - The booster RF channels to configure.
///
/// # Returns
/// A String response indicating the result of the request.
fn handle_channel_disable(message: &[u8], channels: &mut BoosterChannels) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelRequest>(message) {
        Ok(data) => data,
        Err(_) => return Response::error_msg("Failed to decode data"),
    };

    match channels.disable_channel(request.channel) {
        Err(error) => Response::error(error),
        Ok(_) => Response::okay("Channel disabled"),
    }
}

/// Handle a request to configure interlock thresholds of a channel.
///
/// # Args
/// * `message` - The serialized message request.
/// * `channels` - The booster RF channels to configure.
///
/// # Returns
/// A String response indicating the result of the request.
fn handle_channel_thresholds(
    message: &[u8],
    channels: &mut BoosterChannels,
) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelThresholds>(message) {
        Ok(data) => data,
        Err(_) => return Response::error_msg("Failed to decode data"),
    };

    match channels.set_interlock_thresholds(
        request.channel,
        request.output_power,
        request.reflected_power,
    ) {
        Ok(_) => Response::okay("Thresholds set"),
        Err(error) => Response::error(error),
    }
}

/// Handle a request to tune the bias current of a channel.
///
/// # Args
/// * `message` - The serialized message request.
/// * `channels` - The booster RF channels to configure.
///
/// # Returns
/// A String response indicating the result of the request.
fn handle_channel_tune(message: &[u8], channels: &mut BoosterChannels) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelTuneRequest>(message) {
        Ok(data) => data,
        Err(_) => return Response::error_msg("Failed to decode data"),
    };

    match channels.tune_channel(request.channel, request.current) {
        Ok((vgs, ids)) => ChannelTuneResponse::okay(vgs, ids),
        Err(error) => Response::error(error),
    }
}
