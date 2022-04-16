use std::{
    io,
    net::{Ipv4Addr, Ipv6Addr},
    os::unix::prelude::*,
};

use super::*;

use const_zero::const_zero;
use crossbeam_channel::unbounded;
use epoll_rs::{Epoll, EpollEvent, Opts, OwnedRawFd};
use ipnetwork::{Ipv4Network, Ipv6Network};
use libslirp::{Handler, Opt, OptIpv4, OptIpv6, OptTftp, PollEvents};
use log::{log, Level};

struct SlirpHandler {
    card: Arc<Mutex<netcard_t>>,
    mac: [u8; 6],
}

impl Handler for SlirpHandler {
    type Timer = pc_timer_t;

    fn clock_get_ns(&mut self) -> i64 {
        let usec = unsafe { TIMER_USEC };
        if usec != 0 {
            (unsafe { tsc } / (usec / 1000)) as i64
        } else {
            0
        }
    }

    fn send_packet(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let card = self.card.lock().unwrap();
        let set_link_state;
        let wait;
        unsafe {
            set_link_state = card.set_link_state();
            wait = card.wait();
        }
        if !set_link_state && !wait {
            let received_mac = &buf[6..12];
            if received_mac != self.mac {
                // Note: network_queue_put_safe always succeeds
                network_queue_put_safe(0, Packet::new(card.r#priv, buf));
            }
            //println!("Packet received, len {}", buf.len());
            Ok(buf.len())
        } else {
            // Card isn't ready, ignore the packet
            println!("Packet dropped, {set_link_state} {wait}");
            Ok(0)
        }
    }

    fn register_poll_fd(&mut self, _fd: std::os::unix::prelude::RawFd) {
        println!("register_poll_fd: {}", _fd);
    }

    fn unregister_poll_fd(&mut self, _fd: std::os::unix::prelude::RawFd) {
        println!("unregister_poll_fd: {}", _fd);
    }

    fn guest_error(&mut self, msg: &str) {
        log!(Level::Error, "SLIRP guest error: {}", msg);
    }

    fn notify(&mut self) {
        println!("notify");
    }

    fn timer_new(&mut self, func: Box<dyn FnMut()>) -> Box<Self::Timer> {
        let mut timer = Box::new(unsafe { const_zero!(pc_timer_t) });
        let func = Box::into_raw(Box::new(func));
        extern "C" fn callback(data: *mut c_void) {
            let func = data as *mut Box<dyn FnMut()>;
            unsafe {
                let func = &mut *(*func);
                func();
            }
        }
        unsafe {
            timer_add(&mut *timer, callback, func as *mut _, 0);
        }
        timer
    }

    fn timer_mod(&mut self, timer: &mut Box<Self::Timer>, expire_time: i64) {
        unsafe {
            timer_set_delay_u64(&mut **timer, expire_time as _);
        }
    }

    fn timer_free(&mut self, mut timer: Box<Self::Timer>) {
        unsafe {
            let func = timer.p as *mut Box<dyn FnOnce()>;
            timer_stop(&mut *timer);
            // Deallocate the closure
            drop(Box::from_raw(func));
        }
    }
}

enum Command {
    Close,
}

pub struct SlirpThread {
    commands_in: Sender<Command>,
    waker: Waker,
}

impl Drop for SlirpThread {
    fn drop(&mut self) {
        let _ = self.commands_in.send(Command::Close);
        let _ = self.wake();
    }
}

impl SlirpThread {
    pub fn reset(
        card: Arc<Mutex<netcard_t>>,
        mac: [u8; 6],
        tx_receiver: crossbeam_channel::Receiver<Packet>,
    ) -> Self {
        let (command_sender, command_receiver) = unbounded();
        let epoll = Epoll::new().unwrap();
        let waker = Waker::new().unwrap();
        let _waker = waker.clone();

        std::thread::spawn(move || {
            let opt = Opt {
                restrict: false,
                mtu: 1500,
                disable_host_loopback: false,
                hostname: None,
                dns_suffixes: Vec::new(),
                domainname: None,
                ipv4: OptIpv4 {
                    disable: false,
                    net: Ipv4Network::new(Ipv4Addr::new(10, 0, 2, 0), 24).unwrap(),
                    host: Ipv4Addr::new(10, 0, 2, 2),
                    dhcp_start: Ipv4Addr::new(10, 0, 2, 15),
                    dns: Ipv4Addr::new(10, 0, 2, 3),
                },
                ipv6: OptIpv6 {
                    disable: true,
                    net6: Ipv6Network::new(Ipv6Addr::UNSPECIFIED, 0).unwrap(),
                    host: Ipv6Addr::UNSPECIFIED,
                    dns: Ipv6Addr::UNSPECIFIED,
                },
                tftp: OptTftp {
                    name: None,
                    root: None,
                    bootfile: None,
                },
            };
            let handler = SlirpHandler { card, mac };
            let context = libslirp::Context::new_with_opt(&opt, handler);
            // todo: Implement port forwarding config
            let mut timeout = u32::MAX;
            let mut pollfds = Vec::new();
            let mut events = vec![EpollEvent::zeroed(); 1024];
            let waker_token = epoll.add(waker, Opts::IN).unwrap();
            loop {
                context.pollfds_fill(&mut timeout, |raw_fd, poll_events| {
                    let index = pollfds.len();
                    let token = epoll
                        .add(SlirpFd(raw_fd), to_interest(poll_events))
                        .unwrap();
                    pollfds.push((token, poll_events, PollEvents::empty()));
                    index as i32
                });
                let result = epoll.wait(&mut events);
                let err = result.is_err();
                if err {
                    println!("Epoll err: {:?}", result);
                }
                if let Ok(events) = result {
                    for event in events {
                        let interests = event.events;
                        if event.fd() == waker_token.fd() && interests.contains(Opts::IN) {
                            waker_token.file().clear();
                            for command in command_receiver.try_iter() {
                                match command {
                                    Command::Close => return,
                                }
                            }
                            for pkt in tx_receiver.try_iter() {
                                context.input(&pkt);
                            }
                        } else {
                            // todo: Quadratic behaviour, figure out a fix later
                            for (token, wanted, received) in &mut pollfds {
                                if event.fd() == token.fd() {
                                    *received |= *wanted & to_pollevents(interests);
                                }
                            }
                        }
                    }
                }
                context.pollfds_poll(err, |index| pollfds[index as usize].2);
                for (token, _, _) in pollfds.drain(..) {
                    let _ = epoll.remove(token);
                }
            }
        });
        return SlirpThread {
            commands_in: command_sender,
            waker: _waker,
        };
    }
    pub fn close(self) {
        drop(self);
    }
    pub fn wake(&self) -> io::Result<()> {
        self.waker.wake()
    }
}

fn to_interest(events: PollEvents) -> Opts {
    let mut interest = Opts::empty();
    if events.has_in() {
        interest |= Opts::IN;
    }
    if events.has_out() {
        interest |= Opts::OUT;
    }
    if events.has_pri() {
        interest |= Opts::PRI;
    }
    if events.has_err() {
        interest |= Opts::ERR;
    }
    if events.has_hup() {
        interest |= Opts::HUP;
    }
    interest
}

fn to_pollevents(interests: Opts) -> PollEvents {
    let mut pollevents = PollEvents::empty();
    if interests.contains(Opts::IN) {
        pollevents |= PollEvents::poll_in();
    }
    if interests.contains(Opts::OUT) {
        pollevents |= PollEvents::poll_out();
    }
    if interests.contains(Opts::PRI) {
        pollevents |= PollEvents::poll_pri();
    }
    if interests.contains(Opts::ERR) {
        pollevents |= PollEvents::poll_err();
    }
    if interests.contains(Opts::HUP) {
        pollevents |= PollEvents::poll_hup();
    }
    pollevents
}

struct SlirpFd(RawFd);
impl AsRawFd for SlirpFd {
    fn as_raw_fd(&self) -> RawFd {
        self.0
    }
}
impl IntoRawFd for SlirpFd {
    fn into_raw_fd(self) -> i32 {
        self.0
    }
}
impl FromRawFd for SlirpFd {
    unsafe fn from_raw_fd(fd: RawFd) -> Self {
        SlirpFd(fd)
    }
}
impl OwnedRawFd for SlirpFd {}

#[derive(Clone)]
struct Waker(Arc<EventFdInner>);
impl Waker {
    fn new() -> io::Result<Self> {
        let raw_fd = unsafe { eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK) };
        if raw_fd == -1 {
            Err(io::Error::last_os_error())
        } else {
            Ok(Waker(Arc::new(EventFdInner(raw_fd))))
        }
    }

    fn wake(&self) -> io::Result<()> {
        unsafe {
            let written = write(self.0 .0, &1u64 as *const _ as *const _, 8);
            if written == -1 {
                Err(io::Error::last_os_error())
            } else {
                Ok(())
            }
        }
    }

    fn clear(&self) {
        unsafe {
            let buf = &mut 0u64;
            let _ = read(self.0 .0, buf as *mut _ as *mut _, 8);
        }
    }
}
impl AsRawFd for Waker {
    fn as_raw_fd(&self) -> RawFd {
        self.0 .0
    }
}
impl IntoRawFd for Waker {
    fn into_raw_fd(self) -> i32 {
        self.0 .0
    }
}
impl FromRawFd for Waker {
    unsafe fn from_raw_fd(fd: RawFd) -> Self {
        Waker(Arc::new(EventFdInner(fd)))
    }
}
impl OwnedRawFd for Waker {}
struct EventFdInner(RawFd);
impl Drop for EventFdInner {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.0);
        }
    }
}
