use crate::prelude::*;

extern "C" {
    pub static mut TIMER_USEC: u64;
}

#[repr(C, align(1))]
#[derive(Clone, Copy)]
pub struct ts_struct_t {
    frac: u32,
    integer: u32,
}

#[repr(C)]
pub union ts_t {
    ts64: u64,
    ts32: ts_struct_t,
}

#[repr(C)]
pub struct pc_timer_t {
    ts: ts_t,
    flags: c_int, /* The flags are defined above. */
    pad: c_int,
    period: f64, /* This is used for large period timers to count
                 the microseconds and split the period. */

    callback: Option<extern "C" fn(p: *mut c_void)>,
    pub p: *mut c_void,

    prev: *mut pc_timer_t,
    next: *mut pc_timer_t,
}

extern "C" {
    pub fn timer_on_auto(timer: *mut pc_timer_t, period: f64);
    pub fn timer_add(
        timer: *mut pc_timer_t,
        callback: unsafe extern "C" fn(p: *mut c_void),
        p: *mut c_void,
        start_timer: c_int,
    );
    pub fn timer_stop(timer: *mut pc_timer_t);
    pub fn timer_enable(timer: *mut pc_timer_t);
}

#[inline]
pub unsafe fn timer_set_delay_u64(timer: *mut pc_timer_t, delay: u64) {
    (*timer).ts.ts64 = 0;
    (*timer).ts.ts32.integer = tsc as u32;
    (*timer).ts.ts64 += delay;

    timer_enable(timer);
}
