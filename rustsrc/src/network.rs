#![allow(non_upper_case_globals)]

use std::{
    ptr,
    sync::{
        atomic::{AtomicI32, AtomicPtr, AtomicU8, Ordering},
        Arc, Mutex,
    },
};

use crate::prelude::*;
use const_zero::const_zero;
use crossbeam_channel::{Receiver, Sender};
use once_cell::sync::Lazy;

pub mod net_slirp;
pub use net_slirp::*;
pub mod net_dp8390;
pub mod net_ne2000;

// network.h

/* Network provider types. */
const NET_TYPE_NONE: c_int = 0; /* networking disabled */
const NET_TYPE_PCAP: c_int = 1; /* use the (Win)Pcap API */
const NET_TYPE_SLIRP: c_int = 2; /* use the SLiRP port forwarder */

type NETRXCB = Option<unsafe extern "C" fn(*mut c_void, *mut u8, c_int) -> c_int>;
type NETWAITCB = Option<unsafe extern "C" fn(*mut c_void) -> c_int>;
type NETSETLINKSTATE = Option<unsafe extern "C" fn(*mut c_void) -> c_int>;

pub struct Packet {
    private: *mut c_void,
    data: Vec<u8>, /* Maximum length + 1 to round up to the nearest power of 2. */
}

impl Packet {
    pub fn new(private: *mut c_void, data: &[u8]) -> Self {
        Packet {
            private,
            data: Vec::from(data),
        }
    }

    fn empty() -> Self {
        Packet {
            private: ptr::null_mut(),
            data: vec![],
        }
    }
}

unsafe impl Send for Packet {}
impl std::ops::Deref for Packet {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.data[..]
    }
}

#[repr(C)]
pub struct netcard_t {
    device: *const device_t,
    r#priv: *mut c_void,
    poll: Option<extern "C" fn(*mut c_void) -> c_int>,
    rx: NETRXCB,
    wait: NETWAITCB,
    set_link_state: NETSETLINKSTATE,
}

unsafe impl Send for netcard_t {}

const poll_none: Option<extern "C" fn(*mut c_void) -> c_int> = None;
const rx_none: NETRXCB = None;
const wait_none: NETWAITCB = None;
const set_link_state_none: NETSETLINKSTATE = None;

impl netcard_t {
    pub fn new(device: *const device_t) -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(netcard_t {
            device,
            r#priv: ptr::null_mut(),
            poll: poll_none,
            rx: rx_none,
            wait: wait_none,
            set_link_state: set_link_state_none,
        }))
    }

    pub unsafe fn set_link_state(&self) -> bool {
        if let Some(set_link_state) = self.set_link_state {
            set_link_state(self.r#priv) != 0
        } else {
            false
        }
    }

    pub unsafe fn wait(&self) -> bool {
        if let Some(wait) = self.wait {
            wait(self.r#priv) != 0
        } else {
            false
        }
    }
}

#[repr(C)]
pub struct netdev_t {
    device: [c_char; 128],
    description: [c_char; 128],
}

extern "C" {
    fn net_pcap_prepare(_: *mut netdev_t) -> c_int;
    fn net_pcap_init() -> c_int;
    fn net_pcap_reset(_: *const netcard_t, _: *mut u8) -> c_int;
    fn net_pcap_close();
    fn net_pcap_in(_: *mut u8, _: c_int);
}

// network.c

const net_none_device: device_t = device_t {
    name: b"None\0".as_ptr() as *const _,
    internal_name: b"none\0".as_ptr() as *const _,
    flags: 0,
    local: NET_TYPE_NONE as _,
    init: None,
    close: None,
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: ptr::null(),
};

extern "C" {
    pub static threec503_device: device_t;
    pub static pcnet_am79c960_device: device_t;
    pub static pcnet_am79c961_device: device_t;
    pub static ne1000_device: device_t;
    pub static ne2000_device: device_t;
    pub static pcnet_am79c960_eb_device: device_t;
    pub static rtl8019as_device: device_t;
    pub static wd8003e_device: device_t;
    pub static wd8003eb_device: device_t;
    pub static wd8013ebt_device: device_t;
    pub static plip_device: device_t;
    pub static ethernext_mc_device: device_t;
    pub static wd8003eta_device: device_t;
    pub static wd8003ea_device: device_t;
    pub static pcnet_am79c973_device: device_t;
    pub static pcnet_am79c970a_device: device_t;
    pub static rtl8029as_device: device_t;
    pub static pcnet_am79c960_vlb_device: device_t;
}

static net_cards: Lazy<[Arc<Mutex<netcard_t>>; 20]> = Lazy::new(|| {
    [
        // clang-format off
        netcard_t::new(&net_none_device),
        netcard_t::new(unsafe { &threec503_device }),
        netcard_t::new(unsafe { &pcnet_am79c960_device }),
        netcard_t::new(unsafe { &pcnet_am79c961_device }),
        netcard_t::new(unsafe { &ne1000_device }),
        netcard_t::new(unsafe { &ne2000_device }),
        netcard_t::new(unsafe { &pcnet_am79c960_eb_device }),
        netcard_t::new(unsafe { &rtl8019as_device }),
        netcard_t::new(unsafe { &wd8003e_device }),
        netcard_t::new(unsafe { &wd8003eb_device }),
        netcard_t::new(unsafe { &wd8013ebt_device }),
        netcard_t::new(unsafe { &plip_device }),
        netcard_t::new(unsafe { &ethernext_mc_device }),
        netcard_t::new(unsafe { &wd8003eta_device }),
        netcard_t::new(unsafe { &wd8003ea_device }),
        netcard_t::new(unsafe { &pcnet_am79c973_device }),
        netcard_t::new(unsafe { &pcnet_am79c970a_device }),
        netcard_t::new(unsafe { &rtl8029as_device }),
        netcard_t::new(unsafe { &pcnet_am79c960_vlb_device }),
        netcard_t::new(ptr::null()), // clang-format off
    ]
});

// Globals
#[no_mangle]
pub static network_type: AtomicI32 = AtomicI32::new(0);
#[no_mangle]
pub static network_ndev: AtomicI32 = AtomicI32::new(0);
#[no_mangle]
pub static network_card: AtomicI32 = AtomicI32::new(0);
#[no_mangle]
pub static mut network_host: [c_char; 522] = [0; 522]; // Not used here, but used in C
#[no_mangle]
pub static mut network_devs: [netdev_t; 32] = unsafe { const_zero!([netdev_t; 32]) };
#[no_mangle]
pub static network_rx_pause: AtomicI32 = AtomicI32::new(0);

// Locals
struct NetState {
    network_mutex: AtomicPtr<c_void>,
    network_mac: Mutex<[u8; 6]>,
    network_timer_active: AtomicU8,
    network_rx_queue_timer: *mut pc_timer_t,
    network_type: Mutex<NetType>,
    rx_channel: (Sender<Packet>, Receiver<Packet>),
    tx_channel: (Sender<Packet>, Receiver<Packet>),
    queued_pkt: Mutex<Packet>,
}
unsafe impl Send for NetState {}
unsafe impl Sync for NetState {}
impl Drop for NetState {
    fn drop(&mut self) {
        let timer = std::mem::replace(&mut self.network_rx_queue_timer, ptr::null_mut());
        if !timer.is_null() {
            let _ = unsafe { Box::from_raw(timer) };
        }
    }
}

enum NetType {
    None,
    Slirp(SlirpThread),
    Pcap,
}

static state: Lazy<NetState> = Lazy::new(|| NetState {
    network_mutex: AtomicPtr::new(ptr::null_mut()),
    network_mac: Mutex::new([0; 6]),
    network_timer_active: AtomicU8::new(0),
    network_rx_queue_timer: unsafe { Box::into_raw(Box::new(const_zero!(pc_timer_t))) },
    network_type: Mutex::new(NetType::None),
    rx_channel: crossbeam_channel::unbounded(),
    tx_channel: crossbeam_channel::unbounded(),
    queued_pkt: Mutex::new(Packet::empty()),
});

#[no_mangle]
pub unsafe extern "C" fn network_wait(wait: u8) {
    let network_mutex = &state.network_mutex;
    if wait != 0 {
        thread_wait_mutex(network_mutex.load(Ordering::SeqCst));
    } else {
        thread_release_mutex(network_mutex.load(Ordering::SeqCst));
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
    network_type.store(NET_TYPE_NONE, Ordering::SeqCst);
    network_card.store(0, Ordering::SeqCst);

    /* Create a first device entry that's always there, as needed by UI. */
    strcpy(
        network_devs[0].device.as_mut_ptr() as *mut _,
        b"none\0".as_ptr() as *const _,
    );
    strcpy(
        network_devs[0].description.as_mut_ptr() as *mut _,
        b"None\0".as_ptr() as *const _,
    );
    network_ndev.store(1, Ordering::SeqCst);

    /* Initialize the Pcap system module, if present. */
    i = net_pcap_prepare(&mut network_devs[network_ndev.load(Ordering::SeqCst) as usize] as *mut _);
    if i > 0 {
        network_ndev.fetch_add(i, Ordering::SeqCst);
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
    data: *const u8,
    len: c_int,
) {
    let data = std::slice::from_raw_parts(data, len as usize);
    let temp = Packet::new(r#priv, data);
    network_queue_put_safe(tx, temp);
}

pub fn network_queue_put_safe(tx: c_int, pkt: Packet) {
    match tx {
        0 => {
            let _ = state.rx_channel.0.send(pkt);
        }
        1 => {
            let _ = state.tx_channel.0.send(pkt);
        }
        _ => {
            panic!("network_queue_put: Invalid tx value: {}", tx);
        }
    }
}

fn network_queue_get(tx: c_int, pkt: &mut Packet) {
    let temp = match tx {
        0 => state.rx_channel.1.try_recv(),
        1 => state.tx_channel.1.try_recv(),
        _ => panic!("network_queue_get: invalid tx value: {}", tx),
    };

    if temp.is_err() {
        *pkt = Packet::empty();
        return;
    }
    let temp = temp.unwrap();
    *pkt = temp;
}

unsafe fn network_queue_transmit() {
    match &*state.network_type.lock().unwrap() {
        NetType::Pcap => {
            let mut temp = match state.tx_channel.1.try_recv() {
                Ok(temp) => temp,
                _ => return,
            };
            if temp.len() > 0 {
                net_pcap_in(temp.data.as_mut_ptr(), temp.len() as _);
            }
        }
        NetType::Slirp(thread) => {
            thread.wake().unwrap();
        }
        NetType::None => {}
    }
}

unsafe fn network_queue_clear(tx: c_int) {
    let receiver = match tx {
        0 => &state.rx_channel.1,
        1 => &state.tx_channel.1,
        _ => panic!("network_queue_clear: Invalid tx value: {}", tx),
    };
    while let Ok(_) = receiver.try_recv() {}
}

unsafe extern "C" fn network_rx_queue(_priv: *mut c_void) {
    let mut ret: c_int = 1;

    if network_rx_pause.load(Ordering::SeqCst) != 0
        || thread_test_mutex(state.network_mutex.load(Ordering::SeqCst)) == 0
    {
        timer_on_auto(state.network_rx_queue_timer, 0.762939453125 * 2.0 * 128.0);
        return;
    }
    let mut pkt = match state.queued_pkt.try_lock() {
        Ok(pkt) => pkt,
        Err(_) => return,
    };
    if pkt.len() == 0 {
        network_queue_get(0, &mut pkt);
    }
    if pkt.len() > 0 {
        ret = net_cards[network_card.load(Ordering::SeqCst) as usize]
            .lock()
            .unwrap()
            .rx
            .expect("network_rx_queue: Null rx function")(
            pkt.private,
            pkt.data.as_mut_ptr(),
            pkt.len() as _,
        );
    }
    timer_on_auto(
        state.network_rx_queue_timer,
        0.762939453125
            * 2.0
            * (if pkt.len() >= 128 {
                pkt.len() as f64
            } else {
                128.0
            }),
    );
    if ret != 0 {
        pkt.data = vec![];
    }

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
    let card_index = network_card.load(Ordering::SeqCst);
    if card_index == 0 {
        return;
    }

    let card = &net_cards[card_index as usize];

    /* Save the card's info. */
    let mut net_card = card.lock().unwrap();
    net_card.r#priv = dev;
    net_card.rx = rx;
    net_card.wait = wait;
    net_card.set_link_state = set_link_state;
    let mut network_mac = state.network_mac.lock().unwrap();
    ptr::copy_nonoverlapping(mac, network_mac.as_mut_ptr(), 6);

    /* Activate the platform module. */
    match network_type.load(Ordering::SeqCst) {
        NET_TYPE_PCAP => {
            // todo: Fix when rewriting pcap
            *state.network_type.lock().unwrap() = NetType::Pcap;
            net_pcap_reset(&*net_card, network_mac.as_mut_ptr());
        }

        NET_TYPE_SLIRP => {
            *state.network_type.lock().unwrap() = NetType::Slirp(SlirpThread::reset(
                card.clone(),
                *network_mac,
                state.tx_channel.1.clone(),
            ));
        }

        _ => {
            *state.network_type.lock().unwrap() = NetType::None;
        }
    }
    drop(network_mac);

    state
        .queued_pkt
        .lock()
        .map(|mut pkt| *pkt = Packet::empty())
        .unwrap();
    ptr::write_bytes(state.network_rx_queue_timer, 0x00, 1);
    timer_add(
        state.network_rx_queue_timer,
        network_rx_queue,
        ptr::null_mut(),
        0,
    );
    /* 10 mbps. */
    timer_on_auto(state.network_rx_queue_timer, 0.762939453125 * 2.0);
    state.network_timer_active.store(1, Ordering::SeqCst);
}

/* Stop the network timer. */
#[no_mangle]
unsafe extern "C" fn network_timer_stop() {
    if state.network_timer_active.load(Ordering::SeqCst) != 0 {
        timer_stop(state.network_rx_queue_timer);
        ptr::write_bytes(state.network_rx_queue_timer, 0x00, 1);
        state.network_timer_active.store(0, Ordering::SeqCst);
    }
}

/* Stop any network activity. */
#[no_mangle]
pub unsafe extern "C" fn network_close() {
    network_timer_stop();

    /* If already closed, do nothing. */
    if state.network_mutex.load(Ordering::SeqCst).is_null() {
        return;
    }

    let net_type = std::mem::replace(&mut *state.network_type.lock().unwrap(), NetType::None);
    match net_type {
        NetType::Pcap => {
            net_pcap_close();
        }
        NetType::Slirp(thread) => {
            thread.close();
        }
        NetType::None => {}
    }

    /* Close the network thread mutex. */
    let mutex = state.network_mutex.swap(ptr::null_mut(), Ordering::SeqCst);
    thread_close_mutex(mutex);
    *state.network_mac.lock().unwrap() = [0; 6];
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
    let card = network_card.load(Ordering::SeqCst);

    /* If no active card, we're done. */
    if (network_type.load(Ordering::SeqCst) == NET_TYPE_NONE) || (card == 0) {
        return;
    }

    state
        .network_mutex
        .store(thread_create_mutex(), Ordering::SeqCst);
    /*#ifdef ENABLE_NETWORK_LOG
        network_dump_mutex = thread_create_mutex();
    #endif*/

    /* Initialize the platform module. */
    match network_type.load(Ordering::SeqCst) {
        NET_TYPE_PCAP => {
            i = net_pcap_init();
        }

        NET_TYPE_SLIRP => {
            i = 0;
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
        network_type.store(NET_TYPE_NONE, Ordering::SeqCst);

        return;
    }

    /*network_log("NETWORK: set up for %s, card='%s'\n",
    (network_type==NET_TYPE_SLIRP)?"SLiRP":"Pcap",
                    net_cards[network_card].name);*/

    /* Add the (new?) card to the I/O system. */
    let device = net_cards[card as usize].lock().unwrap().device;
    if !device.is_null() {
        /*network_log(
            "NETWORK: adding device '%s'\n",
            net_cards[network_card].name,
        );*/
        device_add(device);
    }
}

/* Queue a packet for transmission to one of the network providers. */
#[no_mangle]
pub unsafe extern "C" fn network_tx(bufp: *mut u8, len: c_int) {
    ui_sb_update_icon(SB_NETWORK, 1);

    network_queue_put(1, ptr::null_mut(), bufp, len);
    match &*state.network_type.lock().unwrap() {
        NetType::None => {}
        NetType::Slirp(thread) => thread.wake().unwrap(),
        NetType::Pcap => {}
    }

    ui_sb_update_icon(SB_NETWORK, 0);
}

/* Actually transmit the packet. */
#[no_mangle]
pub unsafe extern "C" fn network_tx_queue_check() -> c_int {
    if state.tx_channel.1.is_empty() {
        return 0;
    }

    network_queue_transmit();
    return 1;
}

#[no_mangle]
pub unsafe extern "C" fn network_dev_to_id(devname: *mut c_char) -> c_int {
    for i in 0..network_ndev.load(Ordering::SeqCst) as usize {
        if strcmp(network_devs[i].device.as_ptr() as *const c_char, devname) == 0 {
            return i as c_int;
        }
    }

    /* If no match found, assume "none". */
    return 0;
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_available() -> c_int {
    if (network_type.load(Ordering::SeqCst) == NET_TYPE_NONE)
        || (network_card.load(Ordering::SeqCst) == 0)
    {
        return 0;
    }

    return 1;
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_available(card: c_int) -> c_int {
    let device = net_cards[card as usize].lock().unwrap().device;
    if !device.is_null() {
        return device_available(device);
    }
    return 1;
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_getdevice(card: c_int) -> *const device_t {
    return net_cards[card as usize].lock().unwrap().device;
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_has_config(card: c_int) -> c_int {
    let device = net_cards[card as usize].lock().unwrap().device;
    if device.is_null() {
        return 0;
    }

    return if device_has_config(device) != 0 { 1 } else { 0 };
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_get_internal_name(card: c_int) -> *mut c_char {
    return device_get_internal_name(net_cards[card as usize].lock().unwrap().device);
}

/* UI */
#[no_mangle]
pub unsafe extern "C" fn network_card_get_from_internal_name(s: *mut c_char) -> c_int {
    let mut c = 0;

    loop {
        let device = net_cards[c].lock().unwrap().device;
        if device.is_null() {
            break;
        }
        if strcmp((*device).internal_name as *mut c_char, s) == 0 {
            return c as c_int;
        }
        c += 1;
    }

    return 0;
}

#[no_mangle]
pub extern "C" fn network_get_wait() -> c_int {
    return 0;
}
