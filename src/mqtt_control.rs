//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::{idle::Resources, BoosterChannels, Channel, Error};
use core::fmt::Write;
use embedded_hal::blocking::delay::DelayUs;
use heapless::{consts, String};
use minimq::{Property, QoS};

use crate::rf_channel::{Property as ChannelProperty, PropertyId as ChannelPropertyId};

use crate::linear_transformation::LinearTransformation;

#[derive(serde::Deserialize)]
struct PropertyReadRequest {
    pub channel: Channel,
    pub prop: ChannelPropertyId,
}

#[derive(serde::Serialize)]
struct PropertyReadResponse {
    code: u32,
    data: String<consts::U64>,
}

impl PropertyReadResponse {
    /// Indicate that a property read response was successful.
    ///
    /// # Args
    /// * `vgs` - The resulting gate voltage of the RF amplifier.
    /// * `ids` - The resulting drain current of the RF amplifier.
    pub fn okay(prop: ChannelProperty) -> String<consts::U256> {
        // Serialize the property.
        let data: String<consts::U64> = match prop {
            ChannelProperty::OutputInterlockThreshold(threshold) => {
                serde_json_core::to_string(&threshold).unwrap()
            }
            ChannelProperty::InputPowerTransform(transform) => {
                serde_json_core::to_string(&transform).unwrap()
            }
            ChannelProperty::OutputPowerTransform(transform) => {
                serde_json_core::to_string(&transform).unwrap()
            }
            ChannelProperty::ReflectedPowerTransform(transform) => {
                serde_json_core::to_string(&transform).unwrap()
            }
        };

        let response = Self {
            code: 200,
            data: String::from(data.as_str()),
        };

        serde_json_core::to_string(&response).unwrap()
    }
}

#[derive(serde::Deserialize)]
struct PropertyWriteRequest {
    pub channel: Channel,
    prop: ChannelPropertyId,
    data: String<consts::U64>,
}

impl PropertyWriteRequest {
    /// Get the property from the serialized field.
    ///
    /// # Returns
    /// The property if it could be deserialized. Otherwise, an error is returned.
    pub fn property(&self) -> Result<ChannelProperty, Error> {
        // Convert the property
        let prop = match self.prop {
            ChannelPropertyId::OutputInterlockThreshold => {
                // Due to a bug in serde-json-core, trailing data must be present for a float to be
                // properly parsed. For more information, refer to:
                // https://github.com/rust-embedded-community/serde-json-core/issues/47
                let mut data: String<consts::U65> = String::from(self.data.as_str());
                data.push(' ').unwrap();
                ChannelProperty::OutputInterlockThreshold(
                    serde_json_core::from_str::<f32>(&data)
                        .map_err(|_| Error::Invalid)?
                        .0,
                )
            }
            ChannelPropertyId::OutputPowerTransform => ChannelProperty::OutputPowerTransform(
                serde_json_core::from_str::<LinearTransformation>(&self.data)
                    .map_err(|_| Error::Invalid)?
                    .0,
            ),
            ChannelPropertyId::InputPowerTransform => ChannelProperty::InputPowerTransform(
                serde_json_core::from_str::<LinearTransformation>(&self.data)
                    .map_err(|_| Error::Invalid)?
                    .0,
            ),
            ChannelPropertyId::ReflectedPowerTransform => ChannelProperty::ReflectedPowerTransform(
                serde_json_core::from_str::<LinearTransformation>(&self.data)
                    .map_err(|_| Error::Invalid)?
                    .0,
            ),
        };

        Ok(prop)
    }
}

/// Specifies an action to take on a channel.
#[derive(serde::Deserialize)]
enum ChannelAction {
    Enable,
    Disable,
    Powerup,
    Save,
}

/// Specifies a generic request for a specific channel.
#[derive(serde::Deserialize)]
struct ChannelRequest {
    pub channel: Channel,
    pub action: ChannelAction,
}

/// Specifies the desired channel RF bias voltage.
#[derive(serde::Deserialize)]
struct ChannelBiasRequest {
    pub channel: Channel,
    pub voltage: f32,
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
    id: String<heapless::consts::U32>,
}

impl ControlState {
    /// Construct the MQTT control state manager.
    pub fn new<'a>(id: &'a str) -> Self {
        Self {
            subscribed: false,
            id: String::from(id),
        }
    }

    fn generate_topic_string<'a>(&self, topic_postfix: &'a str) -> String<heapless::consts::U64> {
        let mut topic_string: String<heapless::consts::U64> = String::new();
        write!(&mut topic_string, "{}/{}", self.id, topic_postfix).unwrap();
        topic_string
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
                if client.is_connected().unwrap() {
                    for topic in [
                        "channel/state",
                        "channel/bias",
                        "channel/read",
                        "channel/write",
                    ]
                    .iter()
                    {
                        client
                            .subscribe(&self.generate_topic_string(topic), &[])
                            .unwrap();
                    }
                    self.subscribed = true;
                }
            });
        }

        let main_bus = &mut resources.main_bus;
        let delay = &mut resources.delay;

        resources.mqtt_client.lock(|client| {
            match client.poll(|client, topic, message, properties| {
                main_bus.lock(|main_bus| {
                    let (id, route) = topic.split_at(topic.find('/').unwrap());
                    let route = &route[1..];

                    if id != self.id {
                        warn!("Ignoring topic for identifier: {}", id);
                        return;
                    }

                    let response = match route {
                        "channel/state" => handle_channel_update(message, &mut main_bus.channels),
                        "channel/bias" => {
                            handle_channel_bias(message, &mut main_bus.channels, *delay)
                        }
                        "channel/read" => {
                            handle_channel_property_read(message, &mut main_bus.channels)
                        }
                        "channel/write" => {
                            handle_channel_property_write(message, &mut main_bus.channels)
                        }
                        _ => Response::error_msg("Unexpected topic"),
                    };

                    if let Property::ResponseTopic(topic) = properties
                        .iter()
                        .find(|&prop| {
                            if let Property::ResponseTopic(_) = *prop {
                                true
                            } else {
                                false
                            }
                        })
                        .or(Some(&Property::ResponseTopic(
                            &self.generate_topic_string("log"),
                        )))
                        .unwrap()
                    {
                        client
                            .publish(topic, &response.into_bytes(), QoS::AtMostOnce, &[])
                            .unwrap();
                    }
                });
            }) {
                Ok(_) => {}

                // Whenever MQTT disconnects, we will lose our pending subscriptions. We will need
                // to re-establish them once we reconnect.
                Err(minimq::Error::Disconnected) => self.subscribed = false,

                Err(e) => error!("Unexpected error: {:?}", e),
            }
        });
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
fn handle_channel_update(message: &[u8], channels: &mut BoosterChannels) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelRequest>(message) {
        Ok((data, _)) => data,
        Err(_) => return Response::error_msg("Failed to decode data"),
    };
    channels
        .map(request.channel, |ch, _| match request.action {
            ChannelAction::Powerup => {
                ch.start_powerup(false)?;
                Ok("Channel powered")
            }
            ChannelAction::Enable => {
                ch.start_powerup(true)?;
                Ok("Channel enabled")
            }
            ChannelAction::Disable => {
                ch.start_disable();
                Ok("Channel disabled")
            }
            ChannelAction::Save => {
                ch.save_configuration();
                Ok("Channel saved")
            }
        })
        .map_or_else(Response::error, Response::okay)
}

/// Handle a request to read a property of an RF channel.
///
/// # Args
/// * `message` - The serialized message request.
/// * `channels` - The booster RF channels to read.
///
/// # Returns
/// A String response indicating the result of the request.
fn handle_channel_property_read(
    message: &[u8],
    channels: &mut BoosterChannels,
) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<PropertyReadRequest>(message) {
        Ok((data, _)) => data,
        Err(_) => return Response::error_msg("Failed to decode read request"),
    };

    match channels.map(request.channel, |ch, _| Ok(ch.get_property(request.prop))) {
        Ok(prop) => PropertyReadResponse::okay(prop),
        Err(error) => Response::error(error),
    }
}

/// Handle a request to write a property of an RF channel.
///
/// # Args
/// * `message` - The serialized message request.
/// * `channels` - The booster RF channels to write.
///
/// # Returns
/// A String response indicating the result of the request.
fn handle_channel_property_write(
    message: &[u8],
    channels: &mut BoosterChannels,
) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<PropertyWriteRequest>(message) {
        Ok((data, _)) => data,
        Err(_) => return Response::error_msg("Failed to decode write request"),
    };

    let property = match request.property() {
        Ok(property) => property,
        Err(_) => return Response::error_msg("Failed to decode property"),
    };

    match channels.map(request.channel, |ch, _| ch.set_property(property)) {
        Ok(_) => Response::okay("Property update successful"),
        Err(error) => Response::error(error),
    }
}

/// Handle a request to set the bias of a channel.
///
/// # Args
/// * `message` - The serialized message request.
/// * `channels` - The booster RF channels to configure.
/// * `delay` - A means of delaying during tuning.
///
/// # Returns
/// A String response indicating the result of the request.
fn handle_channel_bias(
    message: &[u8],
    channels: &mut BoosterChannels,
    delay: &mut impl DelayUs<u16>,
) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelBiasRequest>(message) {
        Ok((data, _)) => data,
        Err(_) => return Response::error_msg("Failed to decode data"),
    };

    match channels.map(request.channel, |ch, _| {
        ch.set_bias(request.voltage)?;

        // Wait for 11 ms > 10.04 ms total cycle time to ensure an up-to-date
        // current measurement.
        delay.delay_us(11000);
        Ok((ch.get_bias_voltage(), ch.get_p28v_current()))
    }) {
        Ok((vgs, ids)) => ChannelBiasResponse::okay(vgs, ids),
        Err(error) => Response::error(error),
    }
}
