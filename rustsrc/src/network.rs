use std::mem::size_of;

use crate::device::*;
use crate::thread::*;
use crate::timer::*;
use libc::*;

// network.h

/* Network provider types. */
const NET_TYPE_NONE: c_int = 0; /* networking disabled */
const NET_TYPE_PCAP: c_int = 1; /* use the (Win)Pcap API */
const NET_TYPE_SLIRP: c_int = 2; /* use the SLiRP port forwarder */

/* Supported network cards. */
const NONE: c_int = 0;
const NE1000: c_int = 1;
const NE2000: c_int = 2;
const RTL8019AS: c_int = 3;
const RTL8029AS: c_int = 4;

type NETRXCB = extern "C" fn(*mut c_void, *mut u8, c_int) -> c_int;
type NETWAITCB = extern "C" fn(*mut c_void) -> c_int;
type NETSETLINKSTATE = extern "C" fn(*mut c_void) -> c_int;

#[repr(C)]
pub struct netpkt_t {
    r#priv: *mut c_void,
    data: [u8; 65536], /* Maximum length + 1 to round up to the nearest power of 2. */
    len: c_int,
    prev: *mut netpkt_t,
    next: *mut netpkt_t,
}

#[repr(C)]
struct netcard_t {
    device: *const device_t,
    r#priv: *mut c_void,
    poll: extern "C" fn(*mut c_void) -> c_int,
    rx: NETRXCB,
    wait: NETWAITCB,
    set_link_state: NETSETLINKSTATE,
}

#[repr(C)]
struct netdev_t {
    device: [c_char; 128],
    description: [c_char; 128],
}

extern "C" {
    static nic_do_log: c_int;
    static network_ndev: c_int;
    static network_rx_pause: c_int;
    static network_devs: [netdev_t; 32];

    fn network_init();
    fn network_attach(_: *mut c_void, _: *mut u8, _: NETRXCB, _: NETWAITCB, _: NETSETLINKSTATE);
    fn network_close();
    fn network_reset();
    fn network_available() -> c_int;
    fn network_tx(_: *mut u8, _: c_int);
    fn network_tx_queue_check() -> c_int;

    fn net_pcap_prepare(_: *mut netdev_t) -> c_int;
    fn net_pcap_init() -> c_int;
    fn net_pcap_reset(_: *const netcard_t, _: *mut u8) -> c_int;
    fn net_pcap_close();
    fn net_pcap_in(_: *mut u8, _: c_int);

    fn net_slirp_init() -> c_int;
    fn net_slirp_reset(_: *const netcard_t, _: *mut u8) -> c_int;
    fn net_slirp_close();
    fn net_slirp_in(_: *mut u8, _: c_int);

    fn network_dev_to_id(_: *mut c_char) -> c_int;
    fn network_card_available(_: c_int) -> c_int;
    fn network_card_has_config(_: c_int) -> c_int;
    fn network_card_get_internal_name(_: c_int) -> *mut c_char;
    fn network_card_get_from_internal_name(_: *mut c_char) -> c_int;
    fn network_card_getdevice(_: c_int) -> *const device_t;

    fn network_set_wait(wait: c_int);
    fn network_get_wait() -> c_int;

    fn network_timer_stop();

}

// network.c

extern "C" {
    static net_device_none: device_t;
    static net_cards: [netcard_t; 20];

    // Globals
    static network_type: c_int;
    static network_card: c_int;
    static network_host: [c_char; 522];
    static network_tx_pause: c_int;

    // Locals
    static net_wait: std::sync::atomic::AtomicI32; // atomic
    static network_mutex: *mut c_void;
    static network_mac: *mut u8;
    static network_timer_active: u8;
    static mut network_rx_queue_timer: pc_timer_t;
    static mut first_pkt: [*mut netpkt_t; 3];
    static mut last_pkt: [*mut netpkt_t; 3];
    static mut queued_pkt: netpkt_t;

}

#[no_mangle]
pub unsafe extern "C" fn network_wait(wait: u8) {
    if wait != 0 {
        thread_wait_mutex(network_mutex);
    } else {
        thread_release_mutex(network_mutex);
    }
}

#[no_mangle]
pub unsafe extern "C" fn network_queue_put(
    tx: c_int,
    r#priv: *mut c_void,
    data: *mut u8,
    len: c_int,
) {
    let tx = tx as usize;
    let temp = calloc(1, size_of::<netpkt_t>()) as *mut netpkt_t;
    (*temp).r#priv = r#priv;
    memcpy(
        (*temp).data.as_mut_ptr() as *mut _,
        data as *mut _,
        len as size_t,
    );
    (*temp).len = len;
    (*temp).prev = last_pkt[tx];
    (*temp).next = std::ptr::null_mut();

    if !last_pkt[tx].is_null() {
        (*last_pkt[tx]).next = temp;
    }
    last_pkt[tx] = temp;

    if first_pkt[tx].is_null() {
        first_pkt[tx] = temp;
    }
}

#[no_mangle]
pub unsafe extern "C" fn network_queue_get(tx: c_int, pkt: *mut netpkt_t) {
    let tx = tx as usize;
    let temp = first_pkt[tx as usize];

    if temp.is_null() {
        memset(pkt as *mut _, 0x00, size_of::<netpkt_t>());
        return;
    }

    memcpy(pkt as *mut _, temp as *mut _, size_of::<netpkt_t>());

    first_pkt[tx] = (*temp).next;
    free(temp as *mut _);

    if first_pkt[tx].is_null() {
        last_pkt[tx] = std::ptr::null_mut();
    }
}

#[no_mangle]
pub unsafe extern "C" fn network_queue_transmit(tx: c_int) {
    let tx = tx as usize;
    let temp = first_pkt[tx];

    if temp.is_null() {
        return;
    }

    if (*temp).len > 0 {
        // todo: Figure out how to do network logging
        //network_dump_packet(temp);
        /* Why on earth is this not a function pointer?! */
        match network_type {
            NET_TYPE_PCAP => {
                net_pcap_in((*temp).data.as_mut_ptr(), (*temp).len);
            }

            NET_TYPE_SLIRP => {
                net_slirp_in((*temp).data.as_mut_ptr(), (*temp).len);
            }
            _ => {}
        }
    }

    first_pkt[tx] = (*temp).next;
    free(temp as *mut _);

    if first_pkt[tx].is_null() {
        last_pkt[tx] = std::ptr::null_mut();
    }
}

#[no_mangle]
pub unsafe extern "C" fn network_queue_copy(dest: c_int, src: c_int) {
    let dest = dest as usize;
    let src = src as usize;

    let temp = first_pkt[src];

    if temp.is_null() {
        return;
    }

    let temp2 = calloc(1, size_of::<netpkt_t>()) as *mut netpkt_t;
    (*temp2).r#priv = (*temp).r#priv;
    memcpy(
        (*temp2).data.as_mut_ptr() as *mut _,
        (*temp).data.as_mut_ptr() as *mut _,
        (*temp).len as size_t,
    );
    (*temp2).len = (*temp).len;
    (*temp2).prev = last_pkt[dest];
    (*temp2).next = std::ptr::null_mut();

    if !last_pkt[dest].is_null() {
        (*last_pkt[dest]).next = temp2;
    }
    last_pkt[dest] = temp2;

    if first_pkt[dest].is_null() {
        first_pkt[dest] = temp2;
    }

    first_pkt[src] = (*temp).next;
    free(temp as *mut _);

    if first_pkt[src].is_null() {
        last_pkt[src] = std::ptr::null_mut();
    }
}

#[no_mangle]
pub unsafe extern "C" fn network_queue_clear(tx: c_int) {
    let tx = tx as usize;
    let mut temp = first_pkt[tx];
    let mut temp2;

    if temp.is_null() {
        return;
    }

    while !temp.is_null() {
        temp2 = (*temp).next;
        free(temp as *mut _);
        temp = temp2;
    }

    first_pkt[tx] = std::ptr::null_mut();
    last_pkt[tx] = std::ptr::null_mut();
}

#[no_mangle]
pub unsafe extern "C" fn network_rx_queue(_priv: *mut c_void) {
    let mut ret: c_int = 1;

    if network_rx_pause != 0 || thread_test_mutex(network_mutex) == 0 {
        timer_on_auto(
            &mut network_rx_queue_timer as *mut _,
            0.762939453125 * 2.0 * 128.0,
        );
        return;
    }

    if queued_pkt.len == 0 {
        network_queue_get(0, &mut queued_pkt as *mut _);
    }
    if queued_pkt.len > 0 {
        //network_dump_packet(&queued_pkt);
        ret = (net_cards[network_card as usize].rx)(
            queued_pkt.r#priv,
            queued_pkt.data.as_mut_ptr(),
            queued_pkt.len,
        );
    }
    timer_on_auto(
        &mut network_rx_queue_timer as *mut _,
        0.762939453125
            * 2.0
            * (if queued_pkt.len >= 128 {
                queued_pkt.len as f64
            } else {
                128.0
            }),
    );
    if ret != 0 {
        queued_pkt.len = 0;
    }

    /* Transmission. */
    network_queue_copy(1, 2);

    network_wait(0);
}
