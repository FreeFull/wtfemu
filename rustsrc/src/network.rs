use std::{
    mem::size_of,
    ptr,
    sync::atomic::{AtomicI32, Ordering},
};

use crate::prelude::*;
use const_zero::const_zero;
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

impl netpkt_t {
    const fn empty() -> Self {
        netpkt_t {
            r#priv: ptr::null_mut(),
            data: [0; 65536],
            len: 0,
            prev: ptr::null_mut(),
            next: ptr::null_mut(),
        }
    }
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
    static mut nic_do_log: c_int;
    static mut network_ndev: c_int;
    static mut network_rx_pause: c_int;
    static mut network_devs: [netdev_t; 32];

    fn net_pcap_prepare(_: *mut netdev_t) -> c_int;
    fn net_pcap_init() -> c_int;
    fn net_pcap_reset(_: *const netcard_t, _: *mut u8) -> c_int;
    fn net_pcap_close();
    fn net_pcap_in(_: *mut u8, _: c_int);

    fn net_slirp_init() -> c_int;
    fn net_slirp_reset(_: *const netcard_t, _: *mut u8) -> c_int;
    fn net_slirp_close();
    fn net_slirp_in(_: *mut u8, _: c_int);
}

// network.c

extern "C" {
    static mut net_cards: [netcard_t; 20];

    // Globals
    static mut network_type: c_int;
    static mut network_card: c_int;
    static mut network_host: [c_char; 522];
    static mut network_tx_pause: c_int;
}

// Locals
static mut net_wait: AtomicI32 = AtomicI32::new(0);
static mut network_mutex: *mut c_void = ptr::null_mut();
static mut network_mac: *mut u8 = ptr::null_mut();
static mut network_timer_active: u8 = 0;
static mut network_rx_queue_timer: pc_timer_t = unsafe { const_zero!(pc_timer_t) };
static mut first_pkt: [*mut netpkt_t; 3] = [ptr::null_mut(); 3];
static mut last_pkt: [*mut netpkt_t; 3] = [ptr::null_mut(); 3];
static mut queued_pkt: netpkt_t = netpkt_t::empty();

/*
#ifdef ENABLE_NETWORK_LOG
int network_do_log = ENABLE_NETWORK_LOG;
static FILE *network_dump = NULL;
static mutex_t *network_dump_mutex;


static void
network_log(const char *fmt, ...)
{
    va_list ap;

    if (network_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}


static void
network_dump_packet(netpkt_t *pkt)
{
    if (!network_dump)
        return;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct {
        uint32_t ts_sec, ts_usec, incl_len, orig_len;
    } pcap_packet_hdr = {
        tv.tv_sec, tv.tv_usec, pkt->len, pkt->len
    };

    if (network_dump_mutex)
        thread_wait_mutex(network_dump_mutex);

    size_t written;
    if ((written = fwrite(&pcap_packet_hdr, 1, sizeof(pcap_packet_hdr), network_dump)) < sizeof(pcap_packet_hdr)) {
        network_log("NETWORK: failed to write dump packet header\n");
        fseek(network_dump, -written, SEEK_CUR);
    } else {
        if ((written = fwrite(pkt->data, 1, pkt->len, network_dump)) < pkt->len) {
                network_log("NETWORK: failed to write dump packet data\n");
                fseek(network_dump, -written - sizeof(pcap_packet_hdr), SEEK_CUR);
        }
        fflush(network_dump);
    }

    if (network_dump_mutex)
        thread_release_mutex(network_dump_mutex);
}
#else
#define network_log(fmt, ...)
#define network_dump_packet(pkt)
#endif
*/

#[no_mangle]
pub unsafe extern "C" fn network_wait(wait: u8) {
    if wait != 0 {
        thread_wait_mutex(network_mutex);
    } else {
        thread_release_mutex(network_mutex);
    }
}

/*
 * Initialize the configured network cards.
 *
 * This function gets called only once, from the System
 * Platform initialization code (currently in pc.c) to
 * set our local stuff to a known state.
 */
#[no_mangle]
pub unsafe extern "C" fn network_init() {
    let i;

    /* Initialize to a known state. */
    network_type = NET_TYPE_NONE;
    network_card = 0;

    /* Create a first device entry that's always there, as needed by UI. */
    strcpy(
        network_devs[0].device.as_mut_ptr() as *mut _,
        b"none\0".as_ptr() as *const _,
    );
    strcpy(
        network_devs[0].description.as_mut_ptr() as *mut _,
        b"None\0".as_ptr() as *const _,
    );
    network_ndev = 1;

    /* Initialize the Pcap system module, if present. */
    i = net_pcap_prepare(&mut network_devs[network_ndev as usize] as *mut _);
    if (i > 0) {
        network_ndev += i;
    }
    /* // todo
    #ifdef ENABLE_NETWORK_LOG
        /* Start packet dump. */
        network_dump = fopen("network.pcap", "wb");

        struct {
            uint32_t magic_number;
            uint16_t version_major, version_minor;
            int32_t         thiszone;
            uint32_t sigfigs, snaplen, network;
        } pcap_hdr = {
            0xa1b2c3d4,
            2, 4,
            0,
            0, 65535, 1
        };
        fwrite(&pcap_hdr, sizeof(pcap_hdr), 1, network_dump);
        fflush(network_dump);
    #endif
    */
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
    (*temp).next = ptr::null_mut();

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
        last_pkt[tx] = ptr::null_mut();
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
        last_pkt[tx] = ptr::null_mut();
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
    (*temp2).next = ptr::null_mut();

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
        last_pkt[src] = ptr::null_mut();
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

    first_pkt[tx] = ptr::null_mut();
    last_pkt[tx] = ptr::null_mut();
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

/*
 * Attach a network card to the system.
 *
 * This function is called by a hardware driver ("card") after it has
 * finished initializing itself, to link itself to the platform support
 * modules.
 */
#[no_mangle]
pub unsafe extern "C" fn network_attach(
    dev: *mut c_void,
    mac: *mut u8,
    rx: NETRXCB,
    wait: NETWAITCB,
    set_link_state: NETSETLINKSTATE,
) {
    if network_card == 0 {
        return;
    }

    /* Save the card's info. */
    net_cards[network_card as usize].r#priv = dev;
    net_cards[network_card as usize].rx = rx;
    net_cards[network_card as usize].wait = wait;
    net_cards[network_card as usize].set_link_state = set_link_state;
    network_mac = mac;

    network_set_wait(0);

    /* Activate the platform module. */
    match network_type {
        NET_TYPE_PCAP => {
            net_pcap_reset(&net_cards[network_card as usize], network_mac);
        }

        NET_TYPE_SLIRP => {
            net_slirp_reset(&net_cards[network_card as usize], network_mac);
        }

        _ => {}
    }

    first_pkt[0] = ptr::null_mut();
    first_pkt[1] = ptr::null_mut();
    first_pkt[2] = ptr::null_mut();
    last_pkt[0] = ptr::null_mut();
    last_pkt[1] = ptr::null_mut();
    last_pkt[2] = ptr::null_mut();
    memset(
        &mut queued_pkt as *mut _ as *mut _,
        0x00,
        size_of::<netpkt_t>(),
    );
    memset(
        &mut network_rx_queue_timer as *mut _ as *mut _,
        0x00,
        size_of::<pc_timer_t>(),
    );
    timer_add(
        &mut network_rx_queue_timer as *mut _,
        network_rx_queue,
        ptr::null_mut(),
        0,
    );
    /* 10 mbps. */
    timer_on_auto(&mut network_rx_queue_timer as *mut _, 0.762939453125 * 2.0);
    network_timer_active = 1;
}

/* Stop the network timer. */
#[no_mangle]
unsafe extern "C" fn network_timer_stop() {
    if network_timer_active != 0 {
        timer_stop(&mut network_rx_queue_timer);
        memset(
            &mut network_rx_queue_timer as *mut _ as *mut _,
            0x00,
            size_of::<pc_timer_t>(),
        );
        network_timer_active = 0;
    }
}

/* Stop any network activity. */
#[no_mangle]
pub unsafe extern "C" fn network_close() {
    network_timer_stop();

    /* If already closed, do nothing. */
    if network_mutex.is_null() {
        return;
    }

    /* Force-close the PCAP module. */
    net_pcap_close();

    /* Force-close the SLIRP module. */
    net_slirp_close();

    /* Close the network thread mutex. */
    thread_close_mutex(network_mutex);
    network_mutex = ptr::null_mut();
    network_mac = ptr::null_mut();
    /*
    #ifdef ENABLE_NETWORK_LOG
        thread_close_mutex(network_dump_mutex);
        network_dump_mutex = NULL;
    #endif
    */
    /* Here is where we clear the queues. */
    network_queue_clear(0);
    network_queue_clear(1);

    //network_log("NETWORK: closed.\n");
}

/*
 * Reset the network card(s).
 *
 * This function is called each time the system is reset,
 * either a hard reset (including power-up) or a soft reset
 * including C-A-D reset.)  It is responsible for connecting
 * everything together.
 */
#[no_mangle]
pub unsafe extern "C" fn network_reset() {
    let mut i = -1;

    //network_log("NETWORK: reset (type=%d, card=%d)\n",
    //                            network_type, network_card);

    ui_sb_update_icon(SB_NETWORK, 0);

    /* Just in case.. */
    network_close();

    /* If no active card, we're done. */
    if (network_type == NET_TYPE_NONE) || (network_card == 0) {
        return;
    }

    network_mutex = thread_create_mutex();
    /*#ifdef ENABLE_NETWORK_LOG
        network_dump_mutex = thread_create_mutex();
    #endif*/

    /* Initialize the platform module. */
    match (network_type) {
        NET_TYPE_PCAP => {
            i = net_pcap_init();
        }

        NET_TYPE_SLIRP => {
            i = net_slirp_init();
        }

        _ => {}
    }

    if i < 0 {
        /* Tell user we can't do this (at the moment.) */
        ui_msgbox_header(MBX_ERROR, IDS_2093 as *mut _, IDS_2129 as *mut _);

        // FIXME: we should ask in the dialog if they want to
        //          reconfigure or quit, and throw them into the
        //          Settings dialog if yes.

        /* Disable network. */
        network_type = NET_TYPE_NONE;

        return;
    }

    /*network_log("NETWORK: set up for %s, card='%s'\n",
    (network_type==NET_TYPE_SLIRP)?"SLiRP":"Pcap",
                    net_cards[network_card].name);*/

    /* Add the (new?) card to the I/O system. */
    if !net_cards[network_card as usize].device.is_null() {
        /*network_log(
            "NETWORK: adding device '%s'\n",
            net_cards[network_card].name,
        );*/
        device_add(net_cards[network_card as usize].device);
    }
}

/* Queue a packet for transmission to one of the network providers. */
#[no_mangle]
pub unsafe extern "C" fn network_tx(bufp: *mut u8, len: c_int) {
    ui_sb_update_icon(SB_NETWORK, 1);

    network_queue_put(2, ptr::null_mut(), bufp, len);

    ui_sb_update_icon(SB_NETWORK, 0);
}

/* Actually transmit the packet. */
#[no_mangle]
pub unsafe extern "C" fn network_tx_queue_check() -> c_int {
    if ((first_pkt[1].is_null()) && (last_pkt[1].is_null())) {
        return 0;
    }

    if (network_tx_pause != 0) {
        return 1;
    }

    network_queue_transmit(1);
    return 1;
}

#[no_mangle]
pub unsafe extern "C" fn network_dev_to_id(devname: *mut c_char) -> c_int {
    for i in 0..network_ndev as usize {
        if (strcmp(network_devs[i].device.as_mut_ptr() as *mut c_char, devname) == 0) {
            return (i as c_int);
        }
    }

    /* If no match found, assume "none". */
    return (0);
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_available() -> c_int {
    if ((network_type == NET_TYPE_NONE) || (network_card == 0)) {
        return (0);
    }

    return (1);
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_available(card: c_int) -> c_int {
    if (!net_cards[card as usize].device.is_null()) {
        return (device_available(net_cards[card as usize].device));
    }
    return (1);
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_getdevice(card: c_int) -> *const device_t {
    return (net_cards[card as usize].device);
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_has_config(card: c_int) -> c_int {
    if net_cards[card as usize].device.is_null() {
        return (0);
    }

    return (if !(*net_cards[card as usize].device).config.is_null() {
        1
    } else {
        0
    });
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_get_internal_name(card: c_int) -> *mut c_char {
    return device_get_internal_name(net_cards[card as usize].device);
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_get_from_internal_name(s: *mut c_char) -> c_int {
    let mut c = 0;

    while (!net_cards[c].device.is_null()) {
        if (strcmp((*net_cards[c].device).internal_name as *mut c_char, s) == 0) {
            return (c as c_int);
        }
        c += 1;
    }

    return 0;
}

#[no_mangle]
pub unsafe extern "C" fn network_set_wait(wait: c_int) {
    net_wait.store(wait, Ordering::SeqCst);
}

#[no_mangle]
pub unsafe extern "C" fn network_get_wait() -> c_int {
    let ret;

    ret = net_wait.load(Ordering::SeqCst);
    return ret;
}
