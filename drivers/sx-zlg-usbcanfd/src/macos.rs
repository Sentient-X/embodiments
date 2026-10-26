//! Configuring the adapter on macOS, through Apple's `IOUSBHost` framework.
//!
//! macOS leaves a vendor-class device that no driver matches unconfigured, so it has no
//! interface to claim. The legacy `IOUSBDeviceInterface::SetConfiguration` (what `nusb` calls)
//! fails on this adapter with `kIOReturnNotResponding`, and after one failed request its control
//! pipe stays broken until it is replugged. `IOUSBHostDevice`'s `configureWithValue:` succeeds,
//! as a normal user, on a freshly plugged adapter. So before `nusb` opens the device this module
//! looks in the I/O Registry (no USB traffic) for an interface below it, and only when there is
//! none opens it with `IOUSBHost`, selects its one configuration, and releases it. The device is
//! never reset.

#![allow(unsafe_code)]

use std::ffi::CStr;
use std::time::{Duration, Instant};

use objc2::AllocAnyThread;
use objc2_core_foundation::{CFDictionary, CFRetained};
use objc2_io_kit::{
    IOIteratorNext, IOObjectConformsTo, IOObjectRelease, IORegistryEntryGetChildIterator,
    IORegistryEntryIDMatching, IOServiceGetMatchingService, io_object_t, kIOMainPortDefault,
};
use objc2_io_usb_host::{IOUSBHostDevice, IOUSBHostObjectInitOptions};

/// How long the interfaces may take to appear after the configuration is selected: Apple
/// documents that they are not guaranteed to exist when `configure` returns.
const INTERFACES_APPEAR: Duration = Duration::from_secs(2);

/// What [`ensure_configured`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Configured {
    /// An interface was already registered: nothing was sent.
    Already,
    /// The device was unconfigured and `IOUSBHost` selected configuration 1.
    Now,
}

/// An I/O Registry object, released when dropped.
struct Object(io_object_t);

impl Drop for Object {
    fn drop(&mut self) {
        if self.0 != 0 {
            IOObjectRelease(self.0);
        }
    }
}

/// Make sure the device at registry entry `registry_id` has its configuration selected.
///
/// # Errors
///
/// Why the device could not be found, opened or configured, with the `IOReturn` code.
pub fn ensure_configured(registry_id: u64) -> Result<Configured, String> {
    let service = find(registry_id)?;
    if has_interface(&service) {
        return Ok(Configured::Already);
    }
    configure(&service)?;
    let deadline = Instant::now() + INTERFACES_APPEAR;
    while !has_interface(&service) {
        if Instant::now() >= deadline {
            return Err(format!(
                "configuration 1 was selected through IOUSBHost but no interface appeared within {INTERFACES_APPEAR:?}"
            ));
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    Ok(Configured::Now)
}

fn find(registry_id: u64) -> Result<Object, String> {
    // SAFETY: plain IOKit calls; the matching dictionary is consumed by the lookup, and the
    // service returned is owned by `Object`.
    let service = unsafe {
        let matching = IORegistryEntryIDMatching(registry_id)
            .ok_or_else(|| "IORegistryEntryIDMatching failed".to_owned())?;
        let matching: CFRetained<CFDictionary> = CFRetained::cast_unchecked(matching);
        IOServiceGetMatchingService(kIOMainPortDefault, Some(matching))
    };
    if service == 0 {
        return Err(format!(
            "registry entry {registry_id:#x} is no longer registered (unplugged?)"
        ));
    }
    Ok(Object(service))
}

/// Whether an `IOUSBHostInterface` is registered below the device: a configured device.
fn has_interface(service: &Object) -> bool {
    const PLANE: &CStr = c"IOService";
    const INTERFACE: &CStr = c"IOUSBHostInterface";
    let mut iterator = 0;
    // SAFETY: the plane name is a NUL-terminated string IOKit only reads; the iterator is
    // released by `Object`.
    let status = unsafe {
        IORegistryEntryGetChildIterator(
            service.0,
            PLANE.as_ptr().cast_mut().cast(),
            &raw mut iterator,
        )
    };
    if status != 0 {
        return false;
    }
    let iterator = Object(iterator);
    loop {
        let child = Object(IOIteratorNext(iterator.0));
        if child.0 == 0 {
            return false;
        }
        // SAFETY: as above, a NUL-terminated class name IOKit only reads.
        if unsafe { IOObjectConformsTo(child.0, INTERFACE.as_ptr().cast_mut().cast()) } {
            return true;
        }
    }
}

fn configure(service: &Object) -> Result<(), String> {
    let mut error = None;
    // SAFETY: `initWithIOService` takes its own reference to the service; no queue and no
    // interest handler are given, both documented as optional.
    let device = unsafe {
        IOUSBHostDevice::initWithIOService_options_queue_error_interestHandler(
            IOUSBHostDevice::alloc(),
            service.0,
            IOUSBHostObjectInitOptions::None,
            None,
            Some(&mut error),
            std::ptr::null_mut(),
        )
    };
    let Some(device) = device else {
        return Err(format!(
            "IOUSBHost could not open the device: {}",
            error.map_or_else(|| "no reason given".to_owned(), |error| describe(&error))
        ));
    };
    // SAFETY: the device object is valid until `destroy`, which releases the exclusive open.
    unsafe {
        let result = if device.configurationDescriptor().is_null() {
            device
                .configureWithValue_matchInterfaces_error(1, true)
                .map_err(|error| format!("IOUSBHost configure(1): {}", describe(&error)))
        } else {
            Ok(())
        };
        device.destroy();
        result
    }
}

fn describe(error: &objc2_foundation::NSError) -> String {
    // An IOReturn is a 32-bit code; NSError carries it sign-extended.
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let code = error.code() as u32;
    format!("{} (IOReturn {code:#010x})", error.localizedDescription())
}

/// Put the calling thread under Mach's `THREAD_TIME_CONSTRAINT_POLICY`: a loop of `period`
/// that needs up to `computation` of CPU within `constraint` of each wake-up. Without it macOS
/// coalesces a default thread's timers and wake-ups, oversleeping `thread::sleep` and condvar
/// timeouts by about 0.75 ms (p50) and adding milliseconds to the tail of a blocking receive;
/// with it the oversleep is about 10 µs (README, "Waiting on macOS"). The adapter's own I/O
/// thread and nusb's event thread take it when the adapter opens; a tool's control loop sets
/// it on its own thread (`sx-identify` does); `sx-can-bench --realtime` sets it on the bench
/// thread to measure the same.
///
/// # Errors
///
/// The `kern_return_t` the kernel refused the policy with.
pub fn time_constraint(
    period: Duration,
    computation: Duration,
    constraint: Duration,
) -> Result<(), i32> {
    // SAFETY: the calling thread's own port.
    let thread = unsafe { mach2::mach_init::mach_thread_self() };
    constrain(thread, period, computation, constraint)
}

/// [`time_constraint`] for every thread of this process named `name` (`pthread_setname_np`),
/// returning how many took it. nusb delivers every USB completion on its own thread,
/// `nusb-events`, which this crate does not create: at default priority an overloaded host parks
/// it for tens of milliseconds and every answer waits behind it (2026-10-26, the static upper
/// body's `Silent(<id>)` at load 30–100), whatever priority the threads around it run at.
///
/// # Errors
///
/// The `kern_return_t` of listing the threads, or of the first refused policy.
pub fn time_constraint_named(
    name: &str,
    period: Duration,
    computation: Duration,
    constraint: Duration,
) -> Result<usize, i32> {
    use mach2::mach_types::thread_act_array_t;
    use mach2::message::mach_msg_type_number_t;
    use mach2::task::task_threads;
    use mach2::traps::mach_task_self;
    let mut threads: thread_act_array_t = std::ptr::null_mut();
    let mut count: mach_msg_type_number_t = 0;
    // SAFETY: the kernel fills `threads` with `count` send rights it allocated in this task;
    // each right and the array are released below.
    let status = unsafe { task_threads(mach_task_self(), &raw mut threads, &raw mut count) };
    if status != 0 {
        return Err(status);
    }
    let mut taken = 0;
    let mut refused = None;
    for index in 0..count as usize {
        // SAFETY: `index` is below the count the kernel returned.
        let port = unsafe { *threads.add(index) };
        let mut buffer = [0 as libc::c_char; 64];
        // SAFETY: a thread of this task; the buffer's length is passed.
        let named = unsafe {
            let pthread = libc::pthread_from_mach_thread_np(port);
            pthread != 0
                && libc::pthread_getname_np(pthread, buffer.as_mut_ptr(), buffer.len()) == 0
        };
        // SAFETY: NUL-terminated by `pthread_getname_np` within the buffer.
        if named
            && unsafe { std::ffi::CStr::from_ptr(buffer.as_ptr()) }.to_bytes() == name.as_bytes()
        {
            match constrain(port, period, computation, constraint) {
                Ok(()) => taken += 1,
                Err(code) => refused = refused.or(Some(code)),
            }
        }
        // SAFETY: the send right `task_threads` gave this task.
        unsafe { mach2::mach_port::mach_port_deallocate(mach_task_self(), port) };
    }
    // SAFETY: the array `task_threads` allocated, of `count` ports.
    unsafe {
        mach2::vm::mach_vm_deallocate(
            mach_task_self(),
            threads as u64,
            u64::from(count) * std::mem::size_of::<mach2::mach_types::thread_act_t>() as u64,
        );
    }
    refused.map_or(Ok(taken), Err)
}

fn constrain(
    thread: mach2::mach_types::thread_act_t,
    period: Duration,
    computation: Duration,
    constraint: Duration,
) -> Result<(), i32> {
    use mach2::mach_time::{mach_timebase_info, mach_timebase_info as Timebase};
    use mach2::thread_policy::{
        THREAD_TIME_CONSTRAINT_POLICY, THREAD_TIME_CONSTRAINT_POLICY_COUNT, thread_policy_set,
        thread_time_constraint_policy,
    };
    let mut timebase = Timebase { numer: 0, denom: 0 };
    // SAFETY: plain Mach calls on a thread of this task with a policy struct of the size the
    // count names.
    unsafe {
        mach_timebase_info(&raw mut timebase);
        let ticks = |time: Duration| {
            let ns = u64::try_from(time.as_nanos()).unwrap_or(u64::MAX);
            u32::try_from(ns * u64::from(timebase.denom) / u64::from(timebase.numer.max(1)))
                .unwrap_or(u32::MAX)
        };
        let mut policy = thread_time_constraint_policy {
            period: ticks(period),
            computation: ticks(computation),
            constraint: ticks(constraint),
            preemptible: 1,
        };
        let status = thread_policy_set(
            thread,
            THREAD_TIME_CONSTRAINT_POLICY,
            (&raw mut policy).cast(),
            THREAD_TIME_CONSTRAINT_POLICY_COUNT,
        );
        if status == 0 { Ok(()) } else { Err(status) }
    }
}
