//! Booster network management definitions
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use crate::hardware::{clock::SystemTimer, NetworkStack, NetworkStackDrivers};

#[cfg(feature = "phy_enc424j600")]
use crate::hardware::enc424j600_api::{PhyManager, SmoltcpDevice};

use crate::delay::AsmDelay;

#[cfg(feature = "phy_w5500")]
mod shared;

mod mqtt_control;
mod telemetry;

use mqtt_control::ControlState;

#[cfg(feature = "phy_w5500")]
type NetworkManager = shared::NetworkManager<NetworkStack>;
#[cfg(feature = "phy_w5500")]
type NetworkStackProxy = shared::NetworkStackProxy<'static, NetworkStack>;

#[cfg(feature = "phy_enc424j600")]
type NetworkManager = smoltcp_nal::shared::NetworkManager<'static, SmoltcpDevice, SystemTimer>;

#[cfg(feature = "phy_enc424j600")]
type NetworkStackProxy = smoltcp_nal::shared::NetworkStackProxy<'static, NetworkStack>;

/// Container structure for holding all network devices.
///
/// # Note
/// All devices accessing the shared stack must be contained within a single structure to prevent
/// potential pre-emption when using the `shared` network stack.
pub struct NetworkDevices {
    pub controller: mqtt_control::ControlState,
    pub telemetry: telemetry::TelemetryClient,

    #[cfg(feature = "phy_enc424j600")]
    stack: NetworkStackProxy,
    #[cfg(feature = "phy_enc424j600")]
    phy: PhyManager,
}

impl NetworkDevices {
    /// Construct all of Booster's Network devices.
    ///
    /// # Args
    /// * `broker` - The broker IP address for MQTT.
    /// * `devices` - All network devices used for managing the network stack.
    /// * `identifier` - The unique identifier of this device.
    /// * `delay` - A delay mechanism.
    pub fn new(
        broker: minimq::embedded_nal::IpAddr,
        drivers: NetworkStackDrivers,
        identifier: &str,
        delay: AsmDelay,
    ) -> Self {
        #[cfg(feature = "phy_enc424j600")]
        let (phy, stack) = drivers;

        #[cfg(feature = "phy_w5500")]
        let stack = drivers;

        let shared = cortex_m::singleton!(: NetworkManager = NetworkManager::new(stack)).unwrap();

        Self {
            telemetry: telemetry::TelemetryClient::new(broker, shared.acquire_stack(), identifier),
            controller: ControlState::new(broker, shared.acquire_stack(), identifier, delay),

            #[cfg(feature = "phy_enc424j600")]
            stack: shared.acquire_stack(),

            #[cfg(feature = "phy_enc424j600")]
            phy,
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
        self.phy.process();

        #[cfg(feature = "phy_enc424j600")]
        match self.stack.lock(|stack| stack.poll()) {
            Err(_) => true,
            Ok(update) => update,
        }

        #[cfg(feature = "phy_w5500")]
        false
    }
}
