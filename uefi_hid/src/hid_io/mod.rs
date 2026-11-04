//! HidIo Support.
//!
//! Abstractions for interacting with HID devices via the UEFI HidIo protocol.
//!
//! ## Architecture
//!
//! Incoming reports are buffered through a `ReportQueue` rather than being
//! processed inline from the HidIo producer's callback. This ensures all report
//! processing occurs at a consistent TPL_CALLBACK regardless of the producer's
//! calling TPL:
//!
//! 1. **Report callback** (any TPL): locks [`TplMutex`] at TPL_NOTIFY, pushes
//!    raw bytes onto a [`Vec`], signals a TPL_CALLBACK event.
//! 2. **Event handler** (TPL_CALLBACK): locks the same mutex, dequeues all
//!    pending reports, then dispatches them to all receivers. The receivers are
//!    owned by the event's notify closure, so they are only ever touched at
//!    TPL_CALLBACK and are dropped when the event is closed.
//!
//! ## Traits
//!
//! - [`HidIo`] — narrow, receiver-facing interface for reading descriptors and
//!   sending output reports.
//! - [`HidReportReceiver`] — interface for logic that receives HID reports.
//!
//! ## License
//!
//! Copyright (c) Microsoft Corporation.
//!
//! SPDX-License-Identifier: Apache-2.0
//!
pub mod protocol;
use alloc::{boxed::Box, vec::Vec};
use core::{ffi::c_void, ptr::NonNull, slice::from_raw_parts};

#[cfg(test)]
use mockall::automock;
use r_efi::efi;

use self::protocol::HidIoProtocol;
use crate::HidServices;
use hidparser::ReportDescriptor;
use patina::{
    component::service::{
        Service,
        uefi_services::{
            controller_driver::{ControllerDriver, ControllerStateRef},
            event::{Event, EventServicesExt, Tpl as ServiceTpl},
            handle::Handle,
            tpl::TplServices,
        },
    },
    error::EfiError,
    uefi::{boot_services::tpl::Tpl, tpl_mutex::TplMutex},
};

/// Interface for logic that receives HID reports from a device.
#[cfg_attr(test, automock)]
pub trait HidReportReceiver {
    /// Passes an incoming report to the receiver for processing.
    fn receive_report(&mut self, report: &[u8], hid_io: &dyn HidIo);
}

/// Receiver-facing interface for interacting with a HID device.
///
/// Provides read-only access to the report descriptor and the ability to send
/// output reports (e.g. keyboard LED state). Does not manage device lifecycle.
#[cfg_attr(test, automock)]
pub trait HidIo {
    /// Returns the parsed report descriptor for the device.
    fn get_report_descriptor(&self) -> Result<ReportDescriptor, efi::Status>;
    /// Sends an output report to the device.
    fn set_output_report(&self, id: Option<u8>, report: &[u8]) -> Result<(), efi::Status>;
}

// -- ReportQueue ----------------------------------------------------------

/// State shared between the HidIo report callback and the TPL_CALLBACK drain event.
struct ReportQueue {
    /// Pending report bytes, protected at TPL_NOTIFY.
    queue: TplMutex<Vec<Vec<u8>>, Service<dyn TplServices>>,
    /// Services used for signaling the drain event.
    services: HidServices,
    /// TPL_CALLBACK event that drains the queue and dispatches reports to the receivers. Written only before the
    /// report callback is registered.
    process_queue_event: Option<Event>,
}

impl ReportQueue {
    fn new(services: HidServices) -> Self {
        Self { queue: TplMutex::new(services.tpl, Tpl::NOTIFY, Vec::new()), services, process_queue_event: None }
    }

    /// Removes and returns all pending reports.
    fn take_reports(&self) -> Vec<Vec<u8>> {
        core::mem::take(&mut *self.queue.lock())
    }
}

/// Dispatches each report to every receiver.
///
/// Called from the TPL_CALLBACK drain event, ensuring that report processing happens at the lowest possible TPL
/// regardless of the TPL the report was received at.
fn dispatch_reports(reports: &[Vec<u8>], receivers: &mut [Box<dyn HidReportReceiver>], hid_io: &dyn HidIo) {
    log::trace!("dispatch_reports: dispatching {:?} queued reports", reports.len());
    for report in reports {
        for receiver in receivers.iter_mut() {
            receiver.receive_report(report, hid_io);
        }
    }
}

// -- UefiHidIo --------------------------------------------------------------

/// HID device using UEFI services to interact with HidIo controllers.
///
/// Reports are buffered in a [`TplMutex`]-protected queue and processed at
/// TPL_CALLBACK via a UEFI event, rather than inline from the producer's
/// callback. The device is fully active from construction; teardown happens on
/// explicit stop, before Patina releases the protocol.
pub struct UefiHidIo {
    services: HidServices,
    /// report_queue is the context for hid report callbacks.
    report_queue: Box<ReportQueue>,
}

impl UefiHidIo {
    /// Creates a new UefiHidIo bound to the given controller.
    ///
    /// Takes ownership of initialized receivers, creates the report queue, and registers the
    /// protocol callback. Returns `UNSUPPORTED` if no receivers are provided.
    ///
    /// # Safety
    ///
    /// The protocol must remain installed until `stop` succeeds. The owner must pass this same
    /// protocol to `stop` before releasing it. `state` must refer to this controller's device;
    /// report dispatch borrows its managed state and protocol to prevent reentrant stop.
    /// A failed registration must neither invoke nor retain the report callback.
    pub unsafe fn new<D: ControllerDriver<State = Self, Protocol = HidIoProtocol>>(
        services: HidServices,
        hid_io_ref: &HidIoProtocol,
        controller: Handle,
        mut receivers: Vec<Box<dyn HidReportReceiver>>,
        state: ControllerStateRef<D>,
    ) -> Result<Self, efi::Status> {
        if receivers.is_empty() {
            log::trace!("UefiHidIo::new: no receivers initialized, returning UNSUPPORTED");
            return Err(efi::Status::UNSUPPORTED);
        }

        let receiver_count = receivers.len();
        let mut report_queue = Box::new(ReportQueue::new(services));
        let queue_ptr = NonNull::from(report_queue.as_ref());

        // The drain closure owns the receivers: they are only used at TPL_CALLBACK and are dropped when the event
        // is closed, which happens before HidIo is closed.
        let process_queue_event = services
            .events
            .on_event(ServiceTpl::Callback, move || {
                // Holding the state borrow prevents stop from closing this event during dispatch.
                let _ = state.with_protocol(|device, protocol| {
                    // A failed start can leave this event alive; state references also follow controller restarts.
                    if NonNull::from(device.report_queue.as_ref()) != queue_ptr {
                        return;
                    }
                    let queue = device.report_queue.as_ref();
                    let reports = queue.take_reports();
                    // Reports queued during dispatch re-signal this event.
                    dispatch_reports(&reports, &mut receivers, protocol);
                });
            })
            .map_err(|error| efi::Status::from(EfiError::from(error)))?;

        // Set before the queue is published to report_callback so it can safely signal the event.
        report_queue.process_queue_event = Some(process_queue_event);

        // Register the HidIo callback. This also configures HidIo to start sending reports.
        // SAFETY: hid_io_ref references a valid protocol; callback and context are valid.
        match unsafe {
            (hid_io_ref.register_report_callback)(hid_io_ref, Some(Self::report_callback), queue_ptr.as_ptr().cast())
        } {
            efi::Status::SUCCESS => (),
            err => {
                if let Err(error) = services.events.close_event(process_queue_event) {
                    // Failed registration never invokes or retains the callback, so this event cannot
                    // have a pending notification and the queue can be freed despite failed event cleanup.
                    log::error!("Failed to close report event after registration failure: {error:?}");
                }
                return Err(err);
            }
        }

        log::trace!("UefiHidIo::new: initialized with {:?} receivers on controller {:?}", receiver_count, controller);

        Ok(Self { services, report_queue })
    }

    /// Unregisters reports and closes the event owning the receivers before the protocol is released.
    ///
    /// A failure leaves the state alive for another stop attempt.
    pub fn stop(&mut self, hid_io: &HidIoProtocol) -> Result<(), efi::Status> {
        // SAFETY: the owner keeps HidIo installed, and this is the callback registered in new.
        let status = unsafe { (hid_io.unregister_report_callback)(hid_io, Some(Self::report_callback)) };
        // Event-close retries may find that the previous stop attempt already unregistered the callback.
        if status != efi::Status::SUCCESS && status != efi::Status::NOT_STARTED {
            return Err(status);
        }
        if let Some(event) = self.report_queue.process_queue_event {
            self.services.events.close_event(event).map_err(|error| efi::Status::from(EfiError::from(error)))?;
            self.report_queue.process_queue_event = None;
        }
        Ok(())
    }

    /// HidIo protocol report callback. Enqueues the report and signals the drain event. This runs at
    /// whatever TPL the HidIo instance generating the report runs at, which may vary by controller and report type.
    ///
    /// # Safety
    ///
    /// `context` must be a valid pointer to the [`ReportQueue`] that was passed during
    /// `register_report_callback`. `report_buffer` must be valid for `report_buffer_size` bytes.
    unsafe extern "efiapi" fn report_callback(
        report_buffer_size: u16,
        report_buffer: *mut c_void,
        context: *mut c_void,
    ) {
        // SAFETY: context is a valid *mut ReportQueue set during register_report_callback in new().
        // The HidIo protocol callback signature requires *mut c_void, but this function only takes a
        // shared reference; no mutable aliasing occurs because mutation goes through the TplMutex on the queue.
        let report_queue = unsafe { (context as *const ReportQueue).as_ref() }.expect("null report_callback context");
        // SAFETY: report_buffer is valid for report_buffer_size bytes per HidIo protocol contract.
        let report = unsafe { from_raw_parts(report_buffer as *const u8, report_buffer_size as usize) };
        log::trace!("report_callback: received report, size: {:?}", report_buffer_size);
        {
            let mut queue = report_queue.queue.lock();
            queue.push(report.to_vec());
        }
        if let Some(event) = report_queue.process_queue_event {
            let _ = report_queue.services.events.signal_event(event);
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::test_stubs::{hid_io_stub, mock_services, test_event, test_handle};
    use patina::component::service::uefi_services::event::{EventError, EventNotifyCallback};
    use patina::component::service::uefi_services::{
        controller_driver::{
            ControllerDriverRef, StartContext, StopContext, SupportedContext, install_controller_driver,
        },
        protocol::{ProtocolError, ProtocolPtr},
    };
    use std::{
        cell::RefCell,
        rc::Rc,
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
    };

    type ReceiverSlot = Rc<RefCell<Option<Vec<Box<dyn HidReportReceiver>>>>>;

    struct TestDriver {
        services: HidServices,
        receivers: ReceiverSlot,
    }

    impl ControllerDriver for TestDriver {
        type Protocol = HidIoProtocol;
        type State = UefiHidIo;

        fn supported(&self, _: &SupportedContext<'_, Self>) -> Result<(), ProtocolError> {
            Ok(())
        }

        fn start(&self, context: &StartContext<'_, Self>) -> Result<Self::State, ProtocolError> {
            // SAFETY: the managed binding retains the protocol until stop releases callbacks and the event.
            unsafe {
                UefiHidIo::new(
                    self.services,
                    context.protocol(),
                    context.controller(),
                    self.receivers.borrow_mut().take().unwrap(),
                    context.state_ref(),
                )
            }
            .map_err(|status| ProtocolError::from(EfiError::from(status)))
        }

        fn stop(&self, context: &StopContext<'_, Self>, state: &mut Self::State) -> Result<(), ProtocolError> {
            state.stop(context.protocol()).map_err(|status| ProtocolError::from(EfiError::from(status)))
        }
    }

    struct TestDevice {
        driver: ControllerDriverRef<TestDriver>,
        binding: *mut efi::protocols::driver_binding::Protocol,
        receivers: ReceiverSlot,
    }

    impl TestDevice {
        fn new(
            services: HidServices,
            protocol: &HidIoProtocol,
            receivers: Vec<Box<dyn HidReportReceiver>>,
        ) -> Result<Self, efi::Status> {
            let device = Self::install(services, protocol, receivers);
            device.start()?;
            Ok(device)
        }

        fn install(
            services: HidServices,
            protocol: &HidIoProtocol,
            receivers: Vec<Box<dyn HidReportReceiver>>,
        ) -> Self {
            let binding = Rc::new(RefCell::new(core::ptr::null_mut()));
            let installed = binding.clone();
            let address = protocol as *const HidIoProtocol as usize;
            let binding_services = mock_services(|protocols, _, _| {
                protocols.expect_register_agent().returning(|| Ok(test_handle(1)));
                protocols.expect_image_handle().returning(|| test_handle(1));
                protocols.expect_install_interface().returning_st(move |handle, _, ptr| {
                    *installed.borrow_mut() = ptr.as_raw().cast();
                    Ok(handle.unwrap())
                });
                protocols.expect_open_interface().returning(move |_, _, _, _| {
                    Ok(ProtocolPtr::from_raw((address as *mut HidIoProtocol).cast()).unwrap())
                });
                protocols
                    .expect_interface_on_handle()
                    .returning(move |_, _| Ok(ProtocolPtr::from_raw((address as *mut HidIoProtocol).cast()).unwrap()));
                protocols.expect_close_interface().returning(|_, _, _, _| Ok(()));
            });
            let receivers = Rc::new(RefCell::new(Some(receivers)));
            let driver = install_controller_driver(
                binding_services.protocols,
                TestDriver { services, receivers: receivers.clone() },
            )
            .unwrap();
            Self { driver, binding: *binding.borrow(), receivers }
        }

        fn start(&self) -> Result<(), efi::Status> {
            // SAFETY: mocks retain the binding and the leaked protocol throughout the test.
            let status =
                unsafe { ((*self.binding).start)(self.binding, test_handle(2).as_raw(), core::ptr::null_mut()) };
            if status == efi::Status::SUCCESS { Ok(()) } else { Err(status) }
        }

        fn stop(&self) -> Result<(), efi::Status> {
            // SAFETY: the installed binding remains live and the test controller has no children.
            let status =
                unsafe { ((*self.binding).stop)(self.binding, test_handle(2).as_raw(), 0, core::ptr::null_mut()) };
            if status == efi::Status::SUCCESS { Ok(()) } else { Err(status) }
        }
    }

    #[test]
    fn new_rejects_unsupported_receivers_and_event_failure_without_protocol_operations() {
        let services = mock_services(|protocols, _, _| {
            protocols.expect_open_interface().never();
            protocols.expect_close_interface().never();
        });
        let protocol = hid_io_stub();
        let result = TestDevice::new(services, protocol, vec![]);
        assert_eq!(result.err(), Some(efi::Status::UNSUPPORTED));

        let services = mock_services(|protocols, events, _| {
            protocols.expect_open_interface().never();
            protocols.expect_close_interface().never();
            events.expect_create_event().returning(|_, _| Err(EventError::Internal));
        });
        let result = TestDevice::new(services, protocol, vec![Box::new(MockHidReportReceiver::new())]);
        assert_eq!(result.err(), Some(efi::Status::UNSUPPORTED));
    }

    extern "efiapi" fn failing_register(
        _this: *const HidIoProtocol,
        _callback: Option<protocol::HidIoReportCallback>,
        _context: *mut c_void,
    ) -> efi::Status {
        efi::Status::DEVICE_ERROR
    }

    #[test]
    fn new_closes_event_on_registration_failure() {
        let closes = Arc::new(AtomicUsize::new(0));
        let closed = closes.clone();
        let services = mock_services(|protocols, events, _| {
            protocols.expect_close_interface().never();
            events.expect_close_event().returning(move |_| {
                closed.fetch_add(1, Ordering::SeqCst);
                Ok(())
            });
        });
        let protocol = hid_io_stub();
        protocol.register_report_callback = failing_register;
        let result = TestDevice::new(services, protocol, vec![Box::new(MockHidReportReceiver::new())]);
        assert_eq!(result.err(), Some(efi::Status::DEVICE_ERROR));
        assert_eq!(closes.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn failed_registration_leaves_event_unsignaled_when_cleanup_fails() {
        thread_local! {
            static REGISTRATIONS: core::cell::Cell<usize> = const { core::cell::Cell::new(0) };
        }
        extern "efiapi" fn register_then_fail_once(
            _: *const HidIoProtocol,
            callback: Option<protocol::HidIoReportCallback>,
            context: *mut c_void,
        ) -> efi::Status {
            CONTEXT.set(context);
            let attempt = REGISTRATIONS.get();
            REGISTRATIONS.set(attempt + 1);
            if attempt == 0 {
                return efi::Status::DEVICE_ERROR;
            }
            let mut report = [0x42u8];
            // SAFETY: construction publishes the live queue before registration; report is valid for one byte.
            unsafe { callback.unwrap()(1, report.as_mut_ptr().cast(), context) };
            efi::Status::SUCCESS
        }
        REGISTRATIONS.set(0);
        let callbacks: Rc<RefCell<Vec<Option<EventNotifyCallback>>>> = Rc::default();
        let created = callbacks.clone();
        let closed = callbacks.clone();
        let signals = Arc::new(AtomicUsize::new(0));
        let signaled = signals.clone();
        let services = mock_services(|_, events, _| {
            events.expect_signal_event().returning(move |_| {
                signaled.fetch_add(1, Ordering::SeqCst);
                Ok(())
            });
            events.expect_create_event().returning_st(move |_, callback| {
                let mut callbacks = created.borrow_mut();
                callbacks.push(Some(callback));
                Ok(test_event(callbacks.len()))
            });
            events.expect_close_event().returning_st(move |event| {
                if event == test_event(1) {
                    return Err(EventError::Internal);
                }
                closed.borrow_mut()[1].take();
                Ok(())
            });
        });
        let protocol = hid_io_stub();
        protocol.register_report_callback = register_then_fail_once;
        let mut old_receiver = MockHidReportReceiver::new();
        old_receiver.expect_receive_report().never();
        let device = TestDevice::install(services, protocol, vec![Box::new(old_receiver)]);
        assert_eq!(device.start(), Err(efi::Status::DEVICE_ERROR));
        assert!(!device.driver.is_managing(test_handle(2)));
        assert_eq!(signals.load(Ordering::SeqCst), 0);

        let mut new_receiver = MockHidReportReceiver::new();
        new_receiver.expect_receive_report().times(1).returning(|report, _| assert_eq!(report, &[0x42]));
        *device.receivers.borrow_mut() = Some(vec![Box::new(new_receiver)]);
        assert_eq!(device.start(), Ok(()));
        assert_eq!(signals.load(Ordering::SeqCst), 1);
        callbacks.borrow_mut()[1].as_mut().unwrap()(test_event(2));
        assert_eq!(device.stop(), Ok(()));
        callbacks.borrow_mut()[0].take();
    }

    std::thread_local! {
        static CONTEXT: core::cell::Cell<*mut c_void> = const { core::cell::Cell::new(core::ptr::null_mut()) };
        static UNREGISTER_FAIL: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
        static UNREGISTERS: core::cell::Cell<usize> = const { core::cell::Cell::new(0) };
        static UNREGISTERED: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    }

    extern "efiapi" fn register(
        _this: *const HidIoProtocol,
        _callback: Option<protocol::HidIoReportCallback>,
        context: *mut c_void,
    ) -> efi::Status {
        CONTEXT.set(context);
        efi::Status::SUCCESS
    }

    extern "efiapi" fn unregister(
        _this: *const HidIoProtocol,
        _callback: Option<protocol::HidIoReportCallback>,
    ) -> efi::Status {
        UNREGISTERS.set(UNREGISTERS.get() + 1);
        if UNREGISTER_FAIL.get() {
            efi::Status::DEVICE_ERROR
        } else if UNREGISTERED.replace(true) {
            efi::Status::NOT_STARTED
        } else {
            efi::Status::SUCCESS
        }
    }

    #[test]
    fn stop_preserves_state_on_unregister_or_event_failure_and_retries() {
        UNREGISTERS.set(0);
        UNREGISTERED.set(false);
        let closes = Arc::new(AtomicUsize::new(0));
        let closed = closes.clone();
        let services = mock_services(|protocols, events, _| {
            protocols.expect_close_interface().never();
            events.expect_close_event().returning(move |_| {
                if closed.fetch_add(1, Ordering::SeqCst) == 0 { Err(EventError::Internal) } else { Ok(()) }
            });
        });
        let protocol = hid_io_stub();
        protocol.unregister_report_callback = unregister;
        let device = TestDevice::new(services, protocol, vec![Box::new(MockHidReportReceiver::new())]).unwrap();
        let state = device.driver.state_ref(test_handle(2));
        let queue = state.with(|device| NonNull::from(device.report_queue.as_ref())).unwrap();
        UNREGISTER_FAIL.set(true);
        assert_eq!(device.stop(), Err(efi::Status::DEVICE_ERROR));
        UNREGISTER_FAIL.set(false);
        assert_eq!(state.with(|device| NonNull::from(device.report_queue.as_ref())).unwrap(), queue);
        assert_eq!(closes.load(Ordering::SeqCst), 0);
        assert_eq!(device.stop(), Err(efi::Status::UNSUPPORTED));
        assert_eq!(state.with(|device| NonNull::from(device.report_queue.as_ref())).unwrap(), queue);
        assert_eq!(device.stop(), Ok(()));
        assert!(!device.driver.is_managing(test_handle(2)));
        assert_eq!(UNREGISTERS.get(), 3);
        assert_eq!(device.stop(), Err(efi::Status::NOT_FOUND));
    }

    #[test]
    fn queued_reports_reach_receivers_and_stop_drops_them() {
        let callback: Rc<RefCell<Option<EventNotifyCallback>>> = Rc::default();
        let saved = callback.clone();
        let closed = callback.clone();
        let drops = Arc::new(AtomicUsize::new(0));
        let device_ptr = Rc::new(RefCell::new(core::ptr::null_mut::<efi::protocols::driver_binding::Protocol>()));
        struct Receiver {
            drops: Arc<AtomicUsize>,
            binding: Rc<RefCell<*mut efi::protocols::driver_binding::Protocol>>,
        }
        impl HidReportReceiver for Receiver {
            fn receive_report(&mut self, report: &[u8], _hid_io: &dyn HidIo) {
                assert_eq!(report, &[0x10, 0x20]);
                let binding = *self.binding.borrow();
                // SAFETY: the installed binding remains live throughout report dispatch.
                let status = unsafe { ((*binding).stop)(binding, test_handle(2).as_raw(), 0, core::ptr::null_mut()) };
                assert_eq!(status, efi::Status::ACCESS_DENIED);
                self.drops.fetch_add(1, Ordering::SeqCst);
            }
        }
        impl Drop for Receiver {
            fn drop(&mut self) {
                self.drops.fetch_add(10, Ordering::SeqCst);
            }
        }
        let services = mock_services(|protocols, events, _| {
            protocols.expect_close_interface().never();
            events.expect_create_event().returning_st(move |tpl, notify| {
                assert_eq!(tpl, ServiceTpl::Callback);
                *saved.borrow_mut() = Some(notify);
                Ok(test_event(3))
            });
            events.expect_signal_event().returning(|_| Ok(()));
            events.expect_close_event().returning_st(move |_| {
                closed.borrow_mut().take();
                Ok(())
            });
        });
        let received = drops.clone();
        let receiver_device = device_ptr.clone();
        let protocol = hid_io_stub();
        protocol.register_report_callback = register;
        let receivers: Vec<Box<dyn HidReportReceiver>> =
            vec![Box::new(Receiver { drops: received, binding: receiver_device })];
        let device = TestDevice::new(services, protocol, receivers).unwrap();
        *device_ptr.borrow_mut() = device.binding;
        let mut report = [0x10u8, 0x20];
        // SAFETY: the registered queue context is live and report is valid for two bytes.
        unsafe { UefiHidIo::report_callback(2, report.as_mut_ptr().cast(), CONTEXT.get()) };
        device
            .driver
            .with_state(test_handle(2), |_| {
                callback.borrow_mut().as_mut().unwrap()(test_event(3));
            })
            .unwrap();
        assert_eq!(drops.load(Ordering::SeqCst), 0);
        callback.borrow_mut().as_mut().unwrap()(test_event(3));
        assert_eq!(drops.load(Ordering::SeqCst), 1);
        assert_eq!(device.stop(), Ok(()));
        assert_eq!(drops.load(Ordering::SeqCst), 11);
    }
}
