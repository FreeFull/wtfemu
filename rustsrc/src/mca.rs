#![allow(non_camel_case_types, non_snake_case, dead_code)]

/* automatically generated by rust-bindgen 0.59.2 */

extern "C" {
    pub(crate) fn mca_init(nr_cards: libc::c_int);
}
extern "C" {
    pub(crate) fn mca_add(
        read: ::std::option::Option<
            unsafe extern "C" fn(addr: libc::c_int, priv_: *mut libc::c_void) -> u8,
        >,
        write: ::std::option::Option<
            unsafe extern "C" fn(addr: libc::c_int, val: u8, priv_: *mut libc::c_void),
        >,
        feedb: ::std::option::Option<unsafe extern "C" fn(priv_: *mut libc::c_void) -> u8>,
        reset: ::std::option::Option<unsafe extern "C" fn(priv_: *mut libc::c_void)>,
        priv_: *mut libc::c_void,
    );
}
extern "C" {
    pub(crate) fn mca_set_index(index: libc::c_int);
}
extern "C" {
    pub(crate) fn mca_read(port: u16) -> u8;
}
extern "C" {
    pub(crate) fn mca_write(port: u16, val: u8);
}
extern "C" {
    pub(crate) fn mca_feedb() -> u8;
}
extern "C" {
    pub(crate) fn mca_reset();
}
extern "C" {
    pub(crate) fn ps2_cache_clean();
}
