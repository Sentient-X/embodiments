//! Mach's `THREAD_TIME_CONSTRAINT_POLICY`: the crate's macOS system calls. Moved here from
//! `sx-zlg-usbcanfd` (embodiments #110, measured there: without it macOS coalesces a default
//! thread's timers and oversleeps `thread::sleep` and condvar timeouts by about 0.75 ms at p50;
//! with it by about 10 µs).

#![allow(unsafe_code)]

use std::time::Duration;

use crate::{Refused, Role};

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
pub fn elevate(role: Role) -> Result<(), Refused> {
    // SAFETY: the calling thread's own port.
    let thread = unsafe { mach2::mach_init::mach_thread_self() };
    constrain(thread, role).map_err(Refused::Denied)
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
pub fn elevate_named(name: &str, role: Role) -> Result<usize, Refused> {
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
        return Err(Refused::Denied(status));
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
        if named
            // SAFETY: NUL-terminated by `pthread_getname_np` within the buffer.
            && unsafe { std::ffi::CStr::from_ptr(buffer.as_ptr()) }.to_bytes() == name.as_bytes()
        {
            match constrain(port, role) {
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
    refused.map_or(Ok(taken), |code| Err(Refused::Denied(code)))
}

fn constrain(thread: mach2::mach_types::thread_act_t, role: Role) -> Result<(), i32> {
    use mach2::mach_time::{mach_timebase_info, mach_timebase_info as Timebase};
    use mach2::thread_policy::{
        THREAD_TIME_CONSTRAINT_POLICY, THREAD_TIME_CONSTRAINT_POLICY_COUNT, thread_policy_set,
        thread_time_constraint_policy,
    };
    let (period, computation, constraint) = crate::constraint(role);
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

pub fn is_elevated() -> bool {
    use mach2::thread_policy::{
        THREAD_TIME_CONSTRAINT_POLICY, THREAD_TIME_CONSTRAINT_POLICY_COUNT, thread_policy_get,
        thread_time_constraint_policy,
    };
    let mut policy = thread_time_constraint_policy {
        period: 0,
        computation: 0,
        constraint: 0,
        preemptible: 0,
    };
    let mut count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
    let mut default = 0;
    // SAFETY: the calling thread's own port and a policy struct of the size `count` names.
    let status = unsafe {
        thread_policy_get(
            mach2::mach_init::mach_thread_self(),
            THREAD_TIME_CONSTRAINT_POLICY,
            (&raw mut policy).cast(),
            &raw mut count,
            &raw mut default,
        )
    };
    status == 0 && default == 0
}
