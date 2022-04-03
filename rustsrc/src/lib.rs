use libc::*;

#[no_mangle]
pub unsafe extern "C" fn rusttest() {
    puts(b"Hello from Rust!".as_ptr() as *const _);
}
