//! USB HID controller driver implementation.
//!
//! ## License
//!
//! Copyright (c) Microsoft Corporation.
//!
//! SPDX-License-Identifier: Apache-2.0
//!
use alloc::boxed::Box;

use patina::{
    BinaryGuid,
    component::service::{
        Service,
        uefi_services::{
            controller_driver::{ControllerDriver, StartContext, StopContext, SupportedContext},
            event::EventServices,
            protocol::{ProtocolError, ProtocolServices, ProtocolServicesExt},
            timer_event::{TimerEventServices, TimerEventServicesExt, TimerType, Tpl},
        },
    },
    error::EfiError,
    protocol::ProtocolInterface,
};
use r_efi::{efi, efi::protocols::usb_io};

use crate::{
    control_transfers, descriptors,
    device::{HidIoInterface, UsbHidDevice},
    hid_io_impl, interrupt_transfers,
    usb_hid_defs::*,
};

/// Typed USB IO interface for Patina, which does not yet bind this r-efi protocol's GUID.
#[repr(transparent)]
pub struct UsbIoProtocol(pub(crate) usb_io::Protocol);

// SAFETY: the transparent wrapper has the canonical USB IO layout for this GUID.
unsafe impl ProtocolInterface for UsbIoProtocol {
    const PROTOCOL_GUID: BinaryGuid = BinaryGuid(usb_io::PROTOCOL_GUID);
}

/// USB HID driver using Patina-managed controller lifetimes.
pub struct UsbHidDriver {
    protocols: Service<dyn ProtocolServices>,
    events: Service<dyn EventServices>,
    timers: Service<dyn TimerEventServices>,
}

impl UsbHidDriver {
    /// Creates a new USB HID driver using the given UEFI services.
    pub fn new(
        protocols: Service<dyn ProtocolServices>,
        events: Service<dyn EventServices>,
        timers: Service<dyn TimerEventServices>,
    ) -> Self {
        Self { protocols, events, timers }
    }
}

fn is_usb_hid(usb_io_protocol: &usb_io::Protocol) -> bool {
    let mut interface_descriptor = empty_usb_interface_descriptor();
    // SAFETY: usb_io_protocol and interface_descriptor are valid.
    let status = unsafe {
        (usb_io_protocol.get_interface_descriptor)(
            usb_io_protocol as *const usb_io::Protocol as *mut usb_io::Protocol,
            &mut interface_descriptor,
        )
    };
    status == efi::Status::SUCCESS && interface_descriptor.interface_class == CLASS_HID
}

impl ControllerDriver for UsbHidDriver {
    type Protocol = UsbIoProtocol;
    type State = UsbHidDevice;

    /// Tests whether the controller is a USB HID interface.
    fn supported(&self, context: &SupportedContext<'_, Self>) -> Result<(), ProtocolError> {
        if is_usb_hid(&context.protocol().0) { Ok(()) } else { Err(ProtocolError::NotFound) }
    }

    /// Starts USB HID support for the controller and installs HidIo on it.
    fn start(&self, context: &StartContext<'_, Self>) -> Result<Self::State, ProtocolError> {
        let controller = context.controller();
        log::trace!("USB HID: start on controller {:?}", controller);
        let usb_io = &context.protocol().0;

        let descriptors = match descriptors::read_descriptors(usb_io) {
            Ok(descriptors) => descriptors,
            Err(status) => {
                log::error!("USB HID: failed to read descriptors: {status:x?}");
                return Err(EfiError::from(status).into());
            }
        };

        // Boot devices: explicitly set report protocol mode.
        if descriptors.interface_descriptor.interface_sub_class == SUBCLASS_BOOT
            && let Err(status) = control_transfers::set_protocol_request(
                usb_io,
                descriptors.interface_descriptor.interface_number,
                REPORT_PROTOCOL,
            )
        {
            log::warn!("USB HID: failed to set report protocol: {status:x?}");
        }

        let mut device = UsbHidDevice {
            interface: None,
            descriptors,
            report_callback: crate::device::ReportCallbackState::default(),
            timer_services: self.timers,
            recovery_event: None,
        };

        // Recovery timer for delayed re-submission on transfer errors. It runs at TPL_CALLBACK, below the
        // typical interrupt transfer TPL.
        let state = context.state_ref();
        let recovery_event = self.timers.on_timer_event(Tpl::Callback, move || {
            let _ =
                state.with_protocol(|device, protocol| interrupt_transfers::recovery_timer_notify(device, &protocol.0));
        });
        let recovery_event = match recovery_event {
            Ok(event) => event,
            Err(error) => {
                log::error!("USB HID: failed to create recovery event: {error:?}");
                return Err(ProtocolError::Internal);
            }
        };
        device.recovery_event = Some(recovery_event);

        let interface =
            Box::new(HidIoInterface { hid_io: hid_io_impl::new_hid_io_protocol(), state: context.state_ref() });
        device.interface = Some(match self.protocols.install_protocol_owned(Some(controller), interface) {
            Ok(installed) => installed,
            Err(error) => {
                log::error!("USB HID: failed to install HidIo protocol: {error:?}");
                let _ = self.events.close_event(recovery_event);
                return Err(error);
            }
        });

        Ok(device)
    }

    /// Stops USB HID support for the controller.
    fn stop(&self, context: &StopContext<'_, Self>, device: &mut Self::State) -> Result<(), ProtocolError> {
        log::trace!("USB HID: stop on controller {:?}", context.controller());
        // Shutdown async transfers first so that no further interrupt callbacks can
        // fire and attempt to arm the recovery timer after it is closed.
        interrupt_transfers::shutdown_async_interrupt_input_transfers(device, &context.protocol().0)
            .map_err(|status| ProtocolError::from(EfiError::from(status)))?;

        if let Some(installed) = device.interface.take() {
            match self.protocols.uninstall_protocol(installed) {
                Ok(interface) => drop(interface),
                Err((error, installed)) => {
                    device.interface = Some(installed);
                    log::error!("USB HID: failed to uninstall HidIo: {error:?}");
                    return Err(error);
                }
            }
        }

        if let Some(event) = device.recovery_event.take() {
            let _ = self.timers.set_timer(event, TimerType::Cancel);
            let _ = self.events.close_event(event);
        }

        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod test {
    use super::*;
    use core::sync::atomic::{AtomicU8, AtomicUsize, Ordering};
    use patina::vendor_protocols::hid_io;
    use std::{cell::RefCell, rc::Rc, sync::Arc};

    use patina::component::service::uefi_services::{
        controller_driver::{ControllerDriverRef, install_controller_driver},
        event::{Event, EventError, EventNotifyCallback, MockEventServices},
        handle::Handle,
        protocol::{MockProtocolServices, OpenAttributes, ProtocolPtr},
        timer_event::MockTimerEventServices,
    };

    // Interface class returned by the stub get_interface_descriptor; 0 means "fail the call".
    static INTERFACE_CLASS: AtomicU8 = AtomicU8::new(0);

    extern "efiapi" fn stub_get_interface_descriptor(
        _this: *mut usb_io::Protocol,
        descriptor: *mut usb_io::InterfaceDescriptor,
    ) -> efi::Status {
        match INTERFACE_CLASS.load(Ordering::SeqCst) {
            0 => efi::Status::DEVICE_ERROR,
            class => {
                // SAFETY: descriptor is a valid pointer supplied by the caller.
                unsafe { (*descriptor).interface_class = class };
                efi::Status::SUCCESS
            }
        }
    }

    fn leaked_usb_io() -> ProtocolPtr {
        let mut usb_io = crate::test_stubs::usb_io_stub();
        usb_io.get_interface_descriptor = stub_get_interface_descriptor;
        ProtocolPtr::from_raw(Box::into_raw(Box::new(usb_io)).cast()).unwrap()
    }

    fn handle(value: usize) -> Handle {
        Handle::from_raw(value as efi::Handle).unwrap()
    }

    pub(crate) struct InstalledDriver {
        binding: *mut efi::protocols::driver_binding::Protocol,
        pub(crate) managed: ControllerDriverRef<UsbHidDriver>,
    }

    impl InstalledDriver {
        fn supported(&self) -> efi::Status {
            // SAFETY: binding is the installed protocol, and the controller handle is non-null.
            unsafe { ((*self.binding).supported)(self.binding, handle(0x2).as_raw(), core::ptr::null_mut()) }
        }

        fn start(&self) -> efi::Status {
            // SAFETY: binding is the installed protocol, and the controller handle is non-null.
            unsafe { ((*self.binding).start)(self.binding, handle(0x2).as_raw(), core::ptr::null_mut()) }
        }

        pub(crate) fn stop(&self) -> efi::Status {
            // SAFETY: binding is the installed protocol; no child handles are supplied.
            unsafe { ((*self.binding).stop)(self.binding, handle(0x2).as_raw(), 0, core::ptr::null_mut()) }
        }
    }

    fn install(
        mut protocols: MockProtocolServices,
        events: MockEventServices,
        timers: MockTimerEventServices,
    ) -> InstalledDriver {
        let binding = Arc::new(AtomicUsize::new(0));
        let installed_binding = binding.clone();
        protocols.expect_register_agent().times(1).returning(|| Ok(handle(0x1)));
        protocols.expect_image_handle().returning(|| handle(0x1));
        protocols
            .expect_install_interface()
            .withf(|_, guid, _| *guid == efi::protocols::driver_binding::PROTOCOL_GUID)
            .times(1)
            .returning(move |agent, _, interface| {
                installed_binding.store(interface.as_raw() as usize, Ordering::SeqCst);
                Ok(agent.unwrap())
            });
        let protocols: Service<dyn ProtocolServices> = Service::mock(Box::new(protocols));
        let managed = install_controller_driver(
            protocols,
            UsbHidDriver::new(protocols, Service::mock(Box::new(events)), Service::mock(Box::new(timers))),
        )
        .unwrap();
        InstalledDriver { binding: binding.load(Ordering::SeqCst) as *mut _, managed }
    }

    fn driver(protocols: MockProtocolServices) -> InstalledDriver {
        install(protocols, MockEventServices::new(), MockTimerEventServices::new())
    }

    fn protocols_with_usb_io(close_count: &'static AtomicUsize) -> MockProtocolServices {
        let mut protocols = MockProtocolServices::new();
        protocols.expect_open_interface().returning(|_, guid, agent, attributes| {
            assert_eq!(guid, usb_io::PROTOCOL_GUID);
            assert_eq!(agent, handle(0x1));
            assert_eq!(attributes, OpenAttributes::ByDriver { controller: None });
            Ok(leaked_usb_io())
        });
        protocols.expect_close_interface().returning(move |controller, guid, agent, by_controller| {
            assert_eq!(guid, usb_io::PROTOCOL_GUID);
            assert_eq!(agent, handle(0x1));
            assert!(by_controller.is_none() || by_controller == Some(controller));
            close_count.fetch_add(1, Ordering::SeqCst);
            Ok(())
        });
        protocols
    }

    // Serializes tests that depend on INTERFACE_CLASS.
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn supported_returns_error_when_no_usb_io() {
        let mut protocols = MockProtocolServices::new();
        protocols.expect_open_interface().returning(|_, _, _, _| Err(ProtocolError::NotFound));
        assert_eq!(driver(protocols).supported(), efi::Status::NOT_FOUND);
    }

    #[test]
    fn supported_accepts_hid_and_rejects_non_hid() {
        let _lock = LOCK.lock().unwrap();
        static CLOSES: AtomicUsize = AtomicUsize::new(0);
        let driver = driver(protocols_with_usb_io(&CLOSES));

        INTERFACE_CLASS.store(CLASS_HID, Ordering::SeqCst);
        assert_eq!(driver.supported(), efi::Status::SUCCESS);

        INTERFACE_CLASS.store(CLASS_HID + 1, Ordering::SeqCst);
        assert_eq!(driver.supported(), efi::Status::NOT_FOUND);

        INTERFACE_CLASS.store(0, Ordering::SeqCst);
        assert_eq!(driver.supported(), efi::Status::NOT_FOUND);

        // Every supported() probe must release its BY_DRIVER open.
        assert_eq!(CLOSES.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn start_propagates_open_failure() {
        let mut protocols = MockProtocolServices::new();
        protocols.expect_open_interface().returning(|_, _, _, _| Err(ProtocolError::AccessDenied));
        assert_eq!(driver(protocols).start(), efi::Status::ACCESS_DENIED);
    }

    #[test]
    fn start_closes_usb_io_when_descriptor_read_fails() {
        let _lock = LOCK.lock().unwrap();
        static CLOSES: AtomicUsize = AtomicUsize::new(0);
        INTERFACE_CLASS.store(0, Ordering::SeqCst);

        let driver = driver(protocols_with_usb_io(&CLOSES));
        assert_eq!(driver.start(), efi::Status::DEVICE_ERROR);
        assert!(!driver.managed.is_managing(handle(0x2)));
        assert_eq!(CLOSES.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn stop_rejects_unmanaged_controller() {
        assert_eq!(driver(MockProtocolServices::new()).stop(), efi::Status::NOT_FOUND);
    }

    const CONFIG: [u8; 18] = [
        9,
        USB_DESC_TYPE_INTERFACE,
        0,
        0,
        1,
        CLASS_HID,
        0,
        0,
        0,
        9,
        USB_DESC_TYPE_HID,
        0x11,
        0x01,
        0,
        1,
        USB_DESC_TYPE_REPORT,
        1,
        0,
    ];

    extern "efiapi" fn get_interface(
        _this: *mut usb_io::Protocol,
        descriptor: *mut usb_io::InterfaceDescriptor,
    ) -> efi::Status {
        // SAFETY: descriptor is a valid output pointer supplied by read_descriptors.
        unsafe {
            *descriptor = empty_usb_interface_descriptor();
            (*descriptor).interface_class = CLASS_HID;
            (*descriptor).num_endpoints = 1;
        }
        efi::Status::SUCCESS
    }

    extern "efiapi" fn get_endpoint(
        _this: *mut usb_io::Protocol,
        _index: u8,
        descriptor: *mut usb_io::EndpointDescriptor,
    ) -> efi::Status {
        // SAFETY: descriptor is a valid output pointer supplied by read_descriptors.
        unsafe {
            *descriptor = empty_usb_endpoint_descriptor();
            (*descriptor).length = 7;
            (*descriptor).endpoint_address = 0x81;
            (*descriptor).attributes = USB_ENDPOINT_INTERRUPT;
            (*descriptor).max_packet_size = 8;
            (*descriptor).interval = 10;
        }
        efi::Status::SUCCESS
    }

    extern "efiapi" fn get_config(
        _this: *mut usb_io::Protocol,
        descriptor: *mut usb_io::ConfigDescriptor,
    ) -> efi::Status {
        // SAFETY: descriptor is a valid output pointer supplied by read_descriptors.
        unsafe {
            *descriptor = empty_usb_config_descriptor();
            (*descriptor).total_length = CONFIG.len() as u16;
            (*descriptor).configuration_value = 1;
        }
        efi::Status::SUCCESS
    }

    extern "efiapi" fn control_transfer(
        _this: *mut usb_io::Protocol,
        request: *mut usb_io::DeviceRequest,
        _direction: usb_io::DataDirection,
        _timeout: u32,
        data: *mut core::ffi::c_void,
        data_length: usize,
        _status: *mut u32,
    ) -> efi::Status {
        // SAFETY: request and data are valid buffers supplied by descriptor reading.
        unsafe {
            let source: &[u8] = if (*request).value >> 8 == USB_DESC_TYPE_CONFIG as u16 { &CONFIG } else { &[0x05] };
            assert_eq!(data_length, source.len());
            core::ptr::copy_nonoverlapping(source.as_ptr(), data.cast(), source.len());
        }
        efi::Status::SUCCESS
    }

    static ASYNC_STARTS: AtomicUsize = AtomicUsize::new(0);
    static ASYNC_CANCELS: AtomicUsize = AtomicUsize::new(0);
    std::thread_local! {
        static FAIL_CANCEL: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    }

    extern "efiapi" fn async_transfer(
        _this: *mut usb_io::Protocol,
        _endpoint: u8,
        is_new: efi::Boolean,
        _interval: usize,
        _length: usize,
        _callback: Option<usb_io::AsyncUsbTransferCallback>,
        _context: *mut core::ffi::c_void,
    ) -> efi::Status {
        if bool::from(is_new) {
            ASYNC_STARTS.fetch_add(1, Ordering::SeqCst);
        } else {
            if FAIL_CANCEL.get() {
                return efi::Status::DEVICE_ERROR;
            }
            ASYNC_CANCELS.fetch_add(1, Ordering::SeqCst);
        }
        efi::Status::SUCCESS
    }

    fn working_protocols(closes: &'static AtomicUsize) -> MockProtocolServices {
        working_protocols_with_lookup(closes, None)
    }

    fn working_protocols_with_lookup(closes: &'static AtomicUsize, lookup: Option<usize>) -> MockProtocolServices {
        let mut protocols = MockProtocolServices::new();
        let opened = Arc::new(AtomicUsize::new(0));
        let opened_usb = opened.clone();
        protocols.expect_open_interface().returning(move |_, guid, _, attributes| {
            assert_eq!(guid, usb_io::PROTOCOL_GUID);
            assert_eq!(attributes, OpenAttributes::ByDriver { controller: None });
            let mut usb = crate::test_stubs::usb_io_stub();
            usb.get_interface_descriptor = get_interface;
            usb.get_endpoint_descriptor = get_endpoint;
            usb.get_config_descriptor = get_config;
            usb.control_transfer = control_transfer;
            usb.async_interrupt_transfer = async_transfer;
            let ptr = Box::into_raw(Box::new(usb));
            opened_usb.store(ptr as usize, Ordering::SeqCst);
            Ok(ProtocolPtr::from_raw(ptr.cast()).unwrap())
        });
        protocols.expect_interface_on_handle().returning(move |controller, guid| {
            assert_eq!(controller, handle(0x2));
            assert_eq!(guid, usb_io::PROTOCOL_GUID);
            Ok(ProtocolPtr::from_raw(lookup.unwrap_or_else(|| opened.load(Ordering::SeqCst)) as *mut _).unwrap())
        });
        protocols.expect_close_interface().returning(move |_, _, _, _| {
            closes.fetch_add(1, Ordering::SeqCst);
            Ok(())
        });
        protocols
    }

    fn event() -> Event {
        Event::from_raw(0x3 as efi::Event).unwrap()
    }

    pub(crate) fn managed_device() -> InstalledDriver {
        managed_device_with_lookup(None)
    }

    pub(crate) fn managed_device_with_lookup(lookup: Option<usize>) -> InstalledDriver {
        static CLOSES: AtomicUsize = AtomicUsize::new(0);
        let mut protocols = working_protocols_with_lookup(&CLOSES, lookup);
        protocols
            .expect_install_interface()
            .withf(|_, guid, _| *guid == hid_io::HID_IO_PROTOCOL_GUID)
            .returning(|controller, _, _| Ok(controller.unwrap()));
        protocols.expect_uninstall_interface().returning(|_, _, _| Ok(()));
        let mut events = MockEventServices::new();
        events.expect_close_event().returning(|_| Ok(()));
        let mut timers = MockTimerEventServices::new();
        timers.expect_create_timer_event().returning(|_, _| Ok(event()));
        timers.expect_set_timer().returning(|_, _| Ok(()));
        let driver = install(protocols, events, timers);
        assert_eq!(driver.start(), efi::Status::SUCCESS);
        driver
    }

    #[test]
    fn start_failure_releases_usb_io_and_recovery_event() {
        let _lock = LOCK.lock().unwrap();
        static CLOSES: AtomicUsize = AtomicUsize::new(0);
        for fail_timer in [true, false] {
            let mut protocols = working_protocols(&CLOSES);
            if !fail_timer {
                protocols
                    .expect_install_interface()
                    .withf(|_, guid, _| *guid == hid_io::HID_IO_PROTOCOL_GUID)
                    .returning(|_, _, _| Err(ProtocolError::OutOfResources));
            }
            let closed_events = Arc::new(AtomicUsize::new(0));
            let closed = closed_events.clone();
            let mut events = MockEventServices::new();
            events.expect_close_event().returning(move |_| {
                closed.fetch_add(1, Ordering::SeqCst);
                Ok(())
            });
            let mut timers = MockTimerEventServices::new();
            timers
                .expect_create_timer_event()
                .returning(move |_, _| if fail_timer { Err(EventError::Internal) } else { Ok(event()) });
            let driver = install(protocols, events, timers);
            assert_eq!(
                driver.start(),
                if fail_timer { efi::Status::UNSUPPORTED } else { efi::Status::OUT_OF_RESOURCES }
            );
            assert!(!driver.managed.is_managing(handle(0x2)));
            assert_eq!(closed_events.load(Ordering::SeqCst), usize::from(!fail_timer));
        }
        assert_eq!(CLOSES.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn managed_state_survives_failed_stop_and_is_released_on_retry() {
        let _lock = LOCK.lock().unwrap();
        static CLOSES: AtomicUsize = AtomicUsize::new(0);
        ASYNC_STARTS.store(0, Ordering::SeqCst);
        ASYNC_CANCELS.store(0, Ordering::SeqCst);
        let mut protocols = working_protocols(&CLOSES);
        let published = Arc::new(AtomicUsize::new(0));
        let installed = published.clone();
        protocols.expect_install_interface().withf(|_, guid, _| *guid == hid_io::HID_IO_PROTOCOL_GUID).returning(
            move |controller, _, interface| {
                installed.store(interface.as_raw() as usize, Ordering::SeqCst);
                let mut size = 0;
                // SAFETY: the installed interface is live, but start has not yet returned its state to Patina.
                let hid_io = unsafe { &*interface.as_raw().cast::<hid_io::HidIoProtocol>() };
                // SAFETY: size is valid; the null output buffer is not accessed while state is unavailable.
                let status = unsafe { (hid_io.get_report_descriptor)(hid_io, &mut size, core::ptr::null_mut()) };
                assert_eq!(status, efi::Status::NOT_STARTED);
                Ok(controller.unwrap())
            },
        );
        let uninstalled = published.clone();
        let mut attempts = 0;
        protocols.expect_uninstall_interface().returning(move |controller, guid, interface| {
            assert_eq!(controller, handle(0x2));
            assert_eq!(guid, hid_io::HID_IO_PROTOCOL_GUID);
            assert_eq!(interface.as_raw() as usize, uninstalled.load(Ordering::SeqCst));
            let mut size = 0;
            // SAFETY: the interface remains live throughout stop, but Patina has marked the state as stopping.
            let hid_io = unsafe { &*interface.as_raw().cast::<hid_io::HidIoProtocol>() };
            // SAFETY: size is valid; the null output buffer is not accessed while state is unavailable.
            let status = unsafe { (hid_io.get_report_descriptor)(hid_io, &mut size, core::ptr::null_mut()) };
            assert_eq!(status, efi::Status::NOT_STARTED);
            attempts += 1;
            if attempts == 1 { Err(ProtocolError::AccessDenied) } else { Ok(()) }
        });
        let callback: Rc<RefCell<Option<EventNotifyCallback>>> = Rc::default();
        let saved_callback = callback.clone();
        let mut timers = MockTimerEventServices::new();
        timers.expect_create_timer_event().returning_st(move |tpl, notify| {
            assert_eq!(tpl, Tpl::Callback);
            *saved_callback.borrow_mut() = Some(notify);
            Ok(event())
        });
        timers.expect_set_timer().withf(|e, kind| *e == event() && *kind == TimerType::Cancel).returning(|_, _| Ok(()));
        let mut events = MockEventServices::new();
        events.expect_close_event().withf(|e| *e == event()).returning(|_| Ok(()));
        let driver = install(protocols, events, timers);
        assert_eq!(driver.start(), efi::Status::SUCCESS);
        driver
            .managed
            .with_state(handle(0x2), |device| {
                assert_eq!(
                    (&raw const device.interface.as_ref().unwrap().interface().hid_io) as usize,
                    published.load(Ordering::SeqCst),
                );
                device.report_callback.callback = Some(report_callback);
            })
            .unwrap();
        callback.borrow_mut().as_mut().unwrap()(event());
        assert_eq!(ASYNC_STARTS.load(Ordering::SeqCst), 1);

        assert_eq!(driver.stop(), efi::Status::ACCESS_DENIED);
        assert!(driver.managed.is_managing(handle(0x2)));
        assert_eq!(CLOSES.load(Ordering::SeqCst), 0);
        assert_eq!(driver.stop(), efi::Status::SUCCESS);
        assert!(!driver.managed.is_managing(handle(0x2)));
        assert_eq!(CLOSES.load(Ordering::SeqCst), 1);
        assert_eq!(ASYNC_CANCELS.load(Ordering::SeqCst), 2);
        // Even a late timer notification cannot access the freed controller state.
        callback.borrow_mut().as_mut().unwrap()(event());
        assert_eq!(ASYNC_STARTS.load(Ordering::SeqCst), 1);
    }

    unsafe extern "efiapi" fn report_callback(
        _length: u16,
        _report: *mut core::ffi::c_void,
        _context: *mut core::ffi::c_void,
    ) {
    }

    #[test]
    fn failed_transfer_cancellation_keeps_callback_context_alive() {
        let _lock = LOCK.lock().unwrap();
        let driver = managed_device();
        let interface = driver
            .managed
            .with_state(handle(0x2), |device| &raw const device.interface.as_ref().unwrap().interface().hid_io)
            .unwrap();
        FAIL_CANCEL.set(true);
        let status = driver.stop();
        FAIL_CANCEL.set(false);
        assert_eq!(status, efi::Status::DEVICE_ERROR);
        assert!(driver.managed.is_managing(handle(0x2)));
        driver
            .managed
            .with_state(handle(0x2), |device| {
                assert_eq!(&raw const device.interface.as_ref().unwrap().interface().hid_io, interface);
                assert!(device.recovery_event.is_some());
            })
            .unwrap();
        assert_eq!(driver.stop(), efi::Status::SUCCESS);
    }
}
