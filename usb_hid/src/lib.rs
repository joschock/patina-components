//! USB HID Driver — produces the HidIo protocol on USB HID devices.
//!
//! This crate implements a UEFI Driver Binding that consumes the USB IO protocol
//! and produces the HidIo protocol for each USB HID device it manages.
//!
//! ## License
//!
//! Copyright (c) Microsoft Corporation.
//!
//! SPDX-License-Identifier: Apache-2.0
//!
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![cfg_attr(coverage, feature(coverage_attribute))]

extern crate alloc;

pub(crate) mod control_transfers;
pub(crate) mod descriptors;
pub(crate) mod device;
pub(crate) mod driver;
pub(crate) mod hid_io_impl;
pub(crate) mod interrupt_transfers;
pub(crate) mod usb_hid_defs;

#[cfg(test)]
pub(crate) mod test_stubs;

use patina::{
    component::{
        component,
        service::{
            Service,
            uefi_services::{
                controller_driver::install_controller_driver, event::EventServices, protocol::ProtocolServices,
                timer_event::TimerEventServices,
            },
        },
    },
    error::Result,
};

/// USB HID Patina component.
///
/// When dispatched, installs a UEFI Driver Binding that consumes USB IO
/// protocol instances on HID devices and produces the HidIo protocol.
pub struct UsbHidComponent;

#[component]
impl UsbHidComponent {
    fn entry_point(
        self,
        protocols: Service<dyn ProtocolServices>,
        events: Service<dyn EventServices>,
        timers: Service<dyn TimerEventServices>,
    ) -> Result<()> {
        install_controller_driver(protocols, driver::UsbHidDriver::new(protocols, events, timers))?;
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use patina::component::service::uefi_services::{
        event::MockEventServices,
        handle::Handle,
        protocol::{MockProtocolServices, ProtocolError},
        timer_event::MockTimerEventServices,
    };
    use r_efi::efi;

    fn event_and_timer_mocks() -> (Service<dyn EventServices>, Service<dyn TimerEventServices>) {
        (Service::mock(Box::new(MockEventServices::new())), Service::mock(Box::new(MockTimerEventServices::new())))
    }

    #[test]
    fn install_usb_hid_binding_should_install_a_binding() {
        let mut protocols = MockProtocolServices::new();
        protocols.expect_register_agent().times(1).returning(|| Ok(Handle::from_raw(0x5678 as efi::Handle).unwrap()));
        protocols.expect_image_handle().times(1).returning(|| Handle::from_raw(0x1234 as efi::Handle).unwrap());
        protocols
            .expect_install_interface()
            .withf(|handle, guid, _| {
                *handle == Handle::from_raw(0x5678 as efi::Handle)
                    && *guid == efi::protocols::driver_binding::PROTOCOL_GUID
            })
            .times(1)
            .returning(|handle, _, _| Ok(handle.unwrap()));
        let (events, timers) = event_and_timer_mocks();
        let protocols: Service<dyn ProtocolServices> = Service::mock(Box::new(protocols));

        install_controller_driver(protocols, driver::UsbHidDriver::new(protocols, events, timers))
            .expect("install should succeed");
    }

    #[test]
    fn install_usb_hid_binding_handles_failure() {
        let mut protocols = MockProtocolServices::new();
        protocols.expect_register_agent().returning(|| Err(ProtocolError::Internal));
        let (events, timers) = event_and_timer_mocks();
        let protocols: Service<dyn ProtocolServices> = Service::mock(Box::new(protocols));
        assert!(install_controller_driver(protocols, driver::UsbHidDriver::new(protocols, events, timers)).is_err());

        let mut protocols = MockProtocolServices::new();
        protocols.expect_register_agent().returning(|| Ok(Handle::from_raw(0x5678 as efi::Handle).unwrap()));
        protocols.expect_image_handle().times(1).returning(|| Handle::from_raw(0x1234 as efi::Handle).unwrap());
        protocols.expect_install_interface().returning(|_, _, _| Err(ProtocolError::OutOfResources));
        let (events, timers) = event_and_timer_mocks();
        let protocols: Service<dyn ProtocolServices> = Service::mock(Box::new(protocols));
        assert!(matches!(
            install_controller_driver(protocols, driver::UsbHidDriver::new(protocols, events, timers)),
            Err(ProtocolError::OutOfResources)
        ));
    }
}
