//! HID driver binding implementation.
//!
//! The [`HidDriver`] implements [`ControllerDriver`] to
//! manage HID instances on controllers that support the HidIo protocol.
//!
//! When started on a controller, it creates a [`crate::hid_io::UefiHidIo`]
//! instance, instantiates keyboard and pointer receivers, initializes them
//! via `&dyn HidIo`, and starts report reception through the device.
//!
//! ## License
//!
//! Copyright (c) Microsoft Corporation.
//!
//! SPDX-License-Identifier: Apache-2.0
//!

use alloc::{boxed::Box, vec::Vec};

use patina::{
    component::service::uefi_services::{
        controller_driver::{ControllerDriver, StartContext, StopContext, SupportedContext},
        protocol::ProtocolError,
    },
    error::EfiError,
};

use crate::{
    HidServices,
    hid_io::{HidReportReceiver, UefiHidIo, protocol::HidIoProtocol},
    keyboard::KeyboardReceiver,
    pointer::PointerReceiver,
};

/// HID driver that implements [`ControllerDriver`].
///
/// Creates and manages HID instances on controllers that expose the HidIo
/// protocol. Directly constructs [`UefiHidIo`] devices and creates keyboard
/// and pointer handlers as report receivers.
pub struct HidDriver {
    services: HidServices,
}

impl HidDriver {
    /// Creates a new HID driver using the given UEFI services.
    pub fn new(services: HidServices) -> Self {
        Self { services }
    }
}

impl ControllerDriver for HidDriver {
    type Protocol = HidIoProtocol;
    type State = UefiHidIo;

    /// Tests if the given controller supports the HidIo protocol and is not already managed.
    fn supported(&self, _context: &SupportedContext<'_, Self>) -> Result<(), ProtocolError> {
        Ok(())
    }

    /// Starts HID support for the given controller.
    ///
    /// Returns the device state for Patina to retain until stop.
    fn start(&self, context: &StartContext<'_, Self>) -> Result<Self::State, ProtocolError> {
        let controller = context.controller();
        log::trace!("HidDriver::start: starting HID on controller {:?}", controller);
        let mut receivers: Vec<Box<dyn HidReportReceiver>> = Vec::new();
        if let Ok(receiver) = PointerReceiver::new(self.services, controller, context.protocol()) {
            receivers.push(Box::new(receiver));
        }
        if let Ok(receiver) = KeyboardReceiver::new(self.services, controller, context.protocol()) {
            receivers.push(Box::new(receiver));
        }
        // SAFETY: Patina holds HidIo BY_DRIVER until stop succeeds and all event/callback uses are released.
        unsafe { UefiHidIo::new(self.services, context.protocol(), controller, receivers, context.state_ref()) }
            .map_err(|status| ProtocolError::from(EfiError::from(status)))
    }

    /// Stops HID support for the given controller.
    ///
    /// Releases callback and receiver resources before Patina closes HidIo and drops the state.
    fn stop(&self, context: &StopContext<'_, Self>, device: &mut Self::State) -> Result<(), ProtocolError> {
        log::trace!("HidDriver::stop: stopping HID on controller {:?}", context.controller());
        device.stop(context.protocol()).map_err(|status| ProtocolError::from(EfiError::from(status)))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::test_stubs::{hid_io_stub, mock_services, test_handle};
    use patina::component::service::uefi_services::{
        controller_driver::install_controller_driver,
        protocol::{OpenAttributes, ProtocolPtr},
    };
    use r_efi::efi;
    use std::sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    };

    #[test]
    fn managed_binding_probes_and_releases_failed_start_without_private_protocol() {
        let binding = Arc::new(AtomicUsize::new(0));
        let installed = binding.clone();
        let closes = Arc::new(AtomicUsize::new(0));
        let closed = closes.clone();
        let services = mock_services(|protocols, _, _| {
            protocols.expect_register_agent().returning(|| Ok(test_handle(1)));
            protocols.expect_image_handle().returning(|| test_handle(1));
            protocols.expect_install_interface().times(1).returning(move |handle, guid, ptr| {
                assert_eq!(guid, efi::protocols::driver_binding::PROTOCOL_GUID);
                installed.store(ptr.as_raw() as usize, Ordering::SeqCst);
                Ok(handle.unwrap())
            });
            protocols.expect_open_interface().returning(|controller, guid, agent, attributes| {
                assert_eq!(guid, crate::hid_io::protocol::HID_IO_PROTOCOL_GUID);
                assert_eq!(agent, test_handle(1));
                assert_eq!(attributes, OpenAttributes::ByDriver { controller: None });
                if controller == test_handle(3) {
                    return Err(ProtocolError::AlreadyStarted);
                }
                Ok(ProtocolPtr::from_raw((hid_io_stub() as *mut HidIoProtocol).cast()).unwrap())
            });
            protocols
                .expect_interface_on_handle()
                .returning(|_, _| Ok(ProtocolPtr::from_raw((hid_io_stub() as *mut HidIoProtocol).cast()).unwrap()));
            protocols.expect_close_interface().returning(move |_, _, _, _| {
                closed.fetch_add(1, Ordering::SeqCst);
                Ok(())
            });
            protocols.expect_uninstall_interface().never();
        });
        let driver = install_controller_driver(services.protocols, HidDriver::new(services)).unwrap();
        let binding = binding.load(Ordering::SeqCst) as *mut efi::protocols::driver_binding::Protocol;
        // SAFETY: binding is the installed driver binding; test handles are non-null, and no device path is supplied.
        unsafe {
            assert_eq!(
                ((*binding).supported)(binding, test_handle(2).as_raw(), core::ptr::null_mut()),
                efi::Status::SUCCESS
            );
            assert_eq!(
                ((*binding).supported)(binding, test_handle(3).as_raw(), core::ptr::null_mut()),
                efi::Status::ALREADY_STARTED
            );
            assert_eq!(
                ((*binding).start)(binding, test_handle(2).as_raw(), core::ptr::null_mut()),
                efi::Status::UNSUPPORTED
            );
            assert_eq!(
                ((*binding).stop)(binding, test_handle(2).as_raw(), 0, core::ptr::null_mut()),
                efi::Status::NOT_FOUND
            );
        }
        assert_eq!(closes.load(Ordering::SeqCst), 2);
        assert!(!driver.is_managing(test_handle(2)));
    }

    #[test]
    fn managed_stop_retries_and_reclaims_input_protocol_before_closing_hid_io() {
        use patina::component::service::uefi_services::event::EventNotifyCallback;
        use std::{cell::RefCell, rc::Rc};

        const DESCRIPTOR: &[u8] = &[
            0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95,
            0x02, 0x81, 0x02, 0xC0,
        ];
        thread_local! {
            static UNREGISTER_ATTEMPTS: core::cell::Cell<usize> = const { core::cell::Cell::new(0) };
        }
        unsafe extern "efiapi" fn descriptor(
            _: *const HidIoProtocol,
            size: *mut usize,
            buffer: *mut core::ffi::c_void,
        ) -> efi::Status {
            // SAFETY: the descriptor helper supplies a live length and a buffer of that length.
            unsafe {
                let available = *size;
                *size = DESCRIPTOR.len();
                if available < DESCRIPTOR.len() {
                    return efi::Status::BUFFER_TOO_SMALL;
                }
                core::ptr::copy_nonoverlapping(DESCRIPTOR.as_ptr(), buffer.cast(), DESCRIPTOR.len());
            }
            efi::Status::SUCCESS
        }
        unsafe extern "efiapi" fn unregister(
            _: *const HidIoProtocol,
            _: Option<patina::vendor_protocols::hid_io::HidIoReportCallback>,
        ) -> efi::Status {
            let attempt = UNREGISTER_ATTEMPTS.get();
            UNREGISTER_ATTEMPTS.set(attempt + 1);
            if attempt == 0 { efi::Status::ACCESS_DENIED } else { efi::Status::SUCCESS }
        }
        UNREGISTER_ATTEMPTS.set(0);
        let protocol = hid_io_stub();
        protocol.get_report_descriptor = descriptor;
        protocol.unregister_report_callback = unregister;
        let protocol_address = protocol as *mut HidIoProtocol as usize;
        let binding = Arc::new(AtomicUsize::new(0));
        let installed = binding.clone();
        let input_active = Arc::new(AtomicUsize::new(0));
        let input_install = input_active.clone();
        let input_uninstall = input_active.clone();
        let input_at_close = input_active.clone();
        let closes = Arc::new(AtomicUsize::new(0));
        let closed = closes.clone();
        let notify: Rc<RefCell<Option<EventNotifyCallback>>> = Rc::default();
        let event_created = notify.clone();
        let event_closed = notify.clone();
        let services = mock_services(|protocols, events, _| {
            protocols.expect_register_agent().returning(|| Ok(test_handle(1)));
            protocols.expect_image_handle().returning(|| test_handle(1));
            protocols.expect_install_interface().returning(move |handle, guid, interface| {
                if guid == efi::protocols::driver_binding::PROTOCOL_GUID {
                    installed.store(interface.as_raw() as usize, Ordering::Relaxed);
                    Ok(test_handle(2))
                } else {
                    assert_eq!(guid, efi::protocols::absolute_pointer::PROTOCOL_GUID);
                    input_install.store(1, Ordering::Relaxed);
                    Ok(handle.unwrap())
                }
            });
            protocols.expect_open_interface().times(1).returning(move |_, guid, _, attributes| {
                assert_eq!(guid, crate::hid_io::protocol::HID_IO_PROTOCOL_GUID);
                assert_eq!(attributes, OpenAttributes::ByDriver { controller: None });
                Ok(ProtocolPtr::from_raw((protocol_address as *mut HidIoProtocol).cast()).unwrap())
            });
            protocols.expect_interface_on_handle().returning(move |_, _| {
                Ok(ProtocolPtr::from_raw((protocol_address as *mut HidIoProtocol).cast()).unwrap())
            });
            protocols.expect_uninstall_interface().times(1).returning(move |_, guid, _| {
                assert_eq!(guid, efi::protocols::absolute_pointer::PROTOCOL_GUID);
                input_uninstall.store(0, Ordering::Relaxed);
                Ok(())
            });
            protocols.expect_close_interface().times(1).returning(move |_, guid, _, _| {
                assert_eq!(guid, crate::hid_io::protocol::HID_IO_PROTOCOL_GUID);
                assert_eq!(input_at_close.load(Ordering::Relaxed), 0);
                closed.fetch_add(1, Ordering::Relaxed);
                Ok(())
            });
            events.expect_create_event().returning_st(move |_, callback| {
                *event_created.borrow_mut() = Some(callback);
                Ok(crate::test_stubs::test_event(3))
            });
            events
                .expect_close_event()
                .withf(|event| *event == crate::test_stubs::test_event(3))
                .times(1)
                .returning_st(move |_| {
                    event_closed.borrow_mut().take();
                    Ok(())
                });
        });
        let driver = install_controller_driver(services.protocols, HidDriver::new(services)).unwrap();
        let binding = binding.load(Ordering::Relaxed) as *mut efi::protocols::driver_binding::Protocol;
        let controller = test_handle(4).as_raw();
        // SAFETY: the installed binding and protocol are retained by the mocks for the whole test.
        unsafe {
            assert_eq!(((*binding).start)(binding, controller, core::ptr::null_mut()), efi::Status::SUCCESS);
            assert_eq!(input_active.load(Ordering::Relaxed), 1);
            assert_eq!(((*binding).stop)(binding, controller, 0, core::ptr::null_mut()), efi::Status::ACCESS_DENIED);
            assert!(driver.is_managing(test_handle(4)));
            assert_eq!(input_active.load(Ordering::Relaxed), 1);
            assert!(notify.borrow().is_some());
            assert_eq!(closes.load(Ordering::Relaxed), 0);
            assert_eq!(((*binding).stop)(binding, controller, 0, core::ptr::null_mut()), efi::Status::SUCCESS);
        }
        assert!(!driver.is_managing(test_handle(4)));
        assert!(notify.borrow().is_none());
        assert_eq!(closes.load(Ordering::Relaxed), 1);
        assert_eq!(UNREGISTER_ATTEMPTS.get(), 2);
    }
}
