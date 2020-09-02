use super::{
    idle::Resources,
    Error,
    Channel,
    BoosterChannels,
};

use core::fmt::Write;

use minimq::{QoS, Property};

use heapless::{String, consts};

pub struct ControlState {
    subscribed: bool
}

impl ControlState {
    pub fn new() -> Self {
        Self { subscribed: false }
    }

    pub fn update(&mut self, resources: &mut Resources) {
        use rtic::Mutex as _;
        // Subscribe to any control topics necessary.
        if !self.subscribed {
            resources.mqtt_client.lock(|client| {
                if client.is_connected() {
                    client.subscribe("booster/enable", &[]).unwrap();
                    client.subscribe("booster/disable", &[]).unwrap();
                    client.subscribe("booster/tune", &[]).unwrap();
                }
            });

            self.subscribed = true;
        }

        let channels = &mut resources.channels;

        resources.mqtt_client.lock(|client| {
            channels.lock(|c| {
                client
                    .poll(|client, topic, message, properties| {
                        let response = match topic {
                            "booster/enable" => handle_channel_enable(message, c),
                            "booster/disable" => handle_channel_disable(message, c),
                            "booster/tune" => handle_channel_tune(message, c),
                            _ => Response::error_msg("Unexpected topic"),
                        };

                    if let Property::ResponseTopic(topic) = properties.iter().find(|&prop|
                            if let minimq::Property::ResponseTopic(_) = *prop {
                                true
                            } else {
                                false
                            })
                        .or(Some(&Property::ResponseTopic("booster/log")))
                        .unwrap()
                    {
                        client.publish(topic, &response.into_bytes(), QoS::AtMostOnce, &[]).unwrap();
                    }

                })
                .unwrap()
            });
        });
    }
}

#[derive(serde::Deserialize)]
struct ChannelRequest {
    pub channel: Channel,
}

#[derive(serde::Deserialize)]
struct ChannelTuneRequest {
    pub channel: Channel,
    pub current: f32,
}

#[derive(serde::Serialize)]
struct ChannelTuneResponse {
    code: u32,
    pub vgs: f32,
    pub ids: f32,
}

#[derive(serde::Deserialize)]
struct ChannelThresholds {
    pub channel: Channel,
    pub reflected_power: f32,
    pub output_power: f32,
}

impl ChannelTuneResponse {
    pub fn okay(vgs: f32, ids: f32) -> String<consts::U256> {
        let response = Self {
            code: 200,
            vgs,
            ids,
        };

        serde_json_core::to_string(&response).unwrap()
    }
}

#[derive(serde::Serialize)]
struct Response {
    code: u32,
    msg: String<heapless::consts::U256>,
}

impl Response {
    pub fn okay<'a>(msg: &'a str) -> String<consts::U256> {
        let response = Response {
            code: 200,
            msg: String::from(msg),
        };

        serde_json_core::to_string(&response).unwrap()
    }

    pub fn error_msg<'a>(msg: &'a str) -> String<consts::U256> {
        let response = Response {
            code: 400,
            msg: String::from(msg),
        };

        serde_json_core::to_string(&response).unwrap()
    }

    pub fn error(error: Error) -> String<consts::U256> {
        let mut msg = String::<consts::U256>::new();
        write!(&mut msg, "{:?}", error).unwrap();

        let response = Response {
            code: 400,
            msg,
        };

        serde_json_core::to_string(&response).unwrap()
    }
}

fn handle_channel_enable(message: &[u8], channels: &mut BoosterChannels) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelRequest>(message) {
        Ok(data) => data,
        Err(_) => return Response::error_msg("Failed to decode data")
    };

    match channels.enable_channel(request.channel) {
        Ok(_) => Response::okay("Channel enabled"),
        Err(error) => Response::error(error),
    }
}

fn handle_channel_disable(message: &[u8], channels: &mut BoosterChannels) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelRequest>(message) {
        Ok(data) => data,
        Err(_) => return Response::error_msg("Failed to decode data")
    };

    match channels.disable_channel(request.channel) {
        Err(error) => Response::error(error),
        Ok(_) => Response::okay("Channel disabled"),
    }
}

fn handle_channel_tune(message: &[u8], channels: &mut BoosterChannels) -> String<consts::U256> {
    let request = match serde_json_core::from_slice::<ChannelTuneRequest>(message) {
        Ok(data) => data,
        Err(_) => return Response::error_msg("Failed to decode data")
    };

    match channels.tune_channel(request.channel, request.current) {
        Ok((vgs, ids)) => ChannelTuneResponse::okay(vgs, ids),
        Err(error) => Response::error(error),
    }
}
