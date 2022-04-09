use crate::prelude::*;

#[allow(non_camel_case_types)]
pub type mutex_t = *mut c_void;

extern "C" {
    pub fn thread_create_mutex() -> mutex_t;
    pub fn thread_test_mutex(_: mutex_t) -> c_int;
    pub fn thread_wait_mutex(_: mutex_t) -> c_int;
    pub fn thread_release_mutex(_: mutex_t) -> c_int;
    pub fn thread_close_mutex(_: mutex_t);
}
