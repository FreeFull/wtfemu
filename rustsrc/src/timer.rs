use libc::*;

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

    callback: extern "C" fn(p: *mut c_void),
    p: *mut c_void,

    prev: *mut pc_timer_t,
    next: *mut pc_timer_t,
}

extern "C" {
    pub fn timer_on_auto(timer: *mut pc_timer_t, period: f64);
}
