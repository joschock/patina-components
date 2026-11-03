//! Per-device state for USB HID devices.
//!
//! ## License
//!
//! Copyright (c) Microsoft Corporation.
//!
//! SPDX-License-Identifier: Apache-2.0
//!
use alloc::vec::Vec;
use core::ffi::c_void;

use r_efi::efi::{self, protocols::usb_io};

use crate::driver::UsbHidDriver;
use patina::{
    BinaryGuid,
    component::service::{
        Service,
        uefi_services::{
            controller_driver::{ControllerError, ControllerStateRef},
            event::Event,
            protocol::InstalledProtocol,
            timer_event::TimerEventServices,
        },
    },
    protocol::ProtocolInterface,
    vendor_protocols::hid_io::{HidIoProtocol, HidIoReportCallback},
};

/// USB HID descriptor set read from the device during initialization.
#[derive(Debug)]
pub struct UsbHidDescriptors {
    pub interface_descriptor: usb_io::InterfaceDescriptor,
    pub int_in_endpoint_descriptor: usb_io::EndpointDescriptor,
    pub report_descriptor: Vec<u8>,
}

/// Registered callback state for asynchronous input report notifications.
#[derive(Clone, Copy, Default)]
pub struct ReportCallbackState {
    pub callback: Option<HidIoReportCallback>,
    pub context: *mut c_void,
}

/// Per-device context for a USB HID device managed by this driver.
///
/// Owned by Patina until a successful controller driver `stop`.
pub struct UsbHidDevice {
    /// Ownership token retained until the protocol is successfully uninstalled.
    pub interface: Option<InstalledProtocol<HidIoInterface>>,
    pub descriptors: UsbHidDescriptors,
    pub report_callback: ReportCallbackState,
    /// Timer event services used to arm the delayed error recovery timer.
    pub(crate) timer_services: Service<dyn TimerEventServices>,
    /// Timer event armed by the interrupt callback on transfer errors. The event's
    /// notify function re-submits the async interrupt transfer after a delay.
    pub(crate) recovery_event: Option<Event>,
}

/// Installed HidIo interface and USB callback context with checked access to controller state.
#[repr(C)]
pub struct HidIoInterface {
    pub hid_io: HidIoProtocol,
    pub state: ControllerStateRef<UsbHidDriver>,
}

// SAFETY: HidIoInterface is repr(C) with the canonical HidIo protocol as its first field.
unsafe impl ProtocolInterface for HidIoInterface {
    const PROTOCOL_GUID: BinaryGuid = patina::vendor_protocols::hid_io::HID_IO_PROTOCOL_GUID;
}

impl HidIoInterface {
    /// Runs a protocol operation with checked access to the device.
    ///
    /// # Safety
    ///
    /// `this` must be null or point to the `hid_io` field of a live `HidIoInterface`.
    pub unsafe fn with_device(
        this: *const HidIoProtocol,
        operation: impl FnOnce(&mut UsbHidDevice, &usb_io::Protocol) -> efi::Status,
    ) -> efi::Status {
        // SAFETY: HidIoInterface is repr(C) with hid_io first; the caller guarantees its lifetime.
        let Some(interface) = (unsafe { this.cast::<Self>().as_ref() }) else {
            return efi::Status::INVALID_PARAMETER;
        };
        let state = interface.state;
        state.with_protocol(|device, protocol| operation(device, &protocol.0)).unwrap_or_else(controller_error_status)
    }
}

pub(crate) fn controller_error_status(error: ControllerError) -> efi::Status {
    match error {
        ControllerError::NotManaged => efi::Status::NOT_STARTED,
        ControllerError::Busy => efi::Status::ACCESS_DENIED,
    }
}
