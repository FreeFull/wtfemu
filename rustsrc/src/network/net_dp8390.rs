#![allow(non_snake_case)]

use std::{
    mem::{size_of, size_of_val},
    ptr,
};

/*
* 86Box	A hypervisor and IBM PC system emulator that specializes in
*		running old operating systems and software designed for IBM
*		PC systems and compatibles from 1981 through fairly recent
*		system designs based on the PCI bus.
*
*		Header of the emulation of the DP8390 Network Interface
*		Controller used by the WD family, NE1000/NE2000 family, and
*		3Com 3C503 NIC's.
*
*
*
* Authors:	Miran Grca, <mgrca8@gmail.com>
*		Bochs project,
*
*		Copyright 2016-2018 Miran Grca.
*		Copyright 2008-2018 Bochs project.
*/

pub const DP8390_FLAG_EVEN_MAC: u8 = 0x01;
pub const DP8390_FLAG_CHECK_CR: u8 = 0x02;
pub const DP8390_FLAG_CLEAR_IRQ: u8 = 0x04;

/* Command Register - 00h read/write */
#[repr(C)]
pub struct CR_t {
    pub stop: c_int,      /* STP - Software Reset command */
    pub start: c_int,     /* START - start the NIC */
    pub tx_packet: c_int, /* TXP - initiate packet transmission */
    pub rdma_cmd: u8,     /* RD0,RD1,RD2 - Remote DMA command */
    pub pgsel: u8,        /* PS0,PS1 - Page select */
}

/* Interrupt Status Register - 07h read/write */

#[repr(C)]
pub struct ISR_t {
    pub pkt_rx: c_int,    /* PRX - packet received with no errors */
    pub pkt_tx: c_int,    /* PTX - packet txed with no errors */
    pub rx_err: c_int,    /* RXE - packet rxed with 1 or more errors */
    pub tx_err: c_int,    /* TXE - packet txed   "  " "    "    " */
    pub overwrite: c_int, /* OVW - rx buffer resources exhausted */
    pub cnt_oflow: c_int, /* CNT - network tally counter MSB's set */
    pub rdma_done: c_int, /* RDC - remote DMA complete */
    pub reset: c_int,     /* RST - reset status */
}

/* Interrupt Mask Register - 0fh write */
#[repr(C)]
pub struct IMR_t {
    pub rx_inte: c_int,    /* PRXE - packet rx interrupt enable */
    pub tx_inte: c_int,    /* PTXE - packet tx interrput enable */
    pub rxerr_inte: c_int, /* RXEE - rx error interrupt enable */
    pub txerr_inte: c_int, /* TXEE - tx error interrupt enable */
    pub overw_inte: c_int, /* OVWE - overwrite warn int enable */
    pub cofl_inte: c_int,  /* CNTE - counter o'flow int enable */
    pub rdma_inte: c_int,  /* RDCE - remote DMA complete int enable */
    pub reserved: c_int,   /* D7 - reserved */
}

/* Data Configuration Register - 0eh write */
#[repr(C)]
pub struct DCR_t {
    pub wdsize: c_int,   /* WTS - 8/16-bit select */
    pub endian: c_int,   /* BOS - byte-order select */
    pub longaddr: c_int, /* LAS - long-address select */
    pub r#loop: c_int,   /* LS  - loopback select */
    pub auto_rx: c_int,  /* AR  - auto-remove rx pkts with remote DMA */
    pub fifo_size: u8,   /* FT0,FT1 - fifo threshold */
}

/* Transmit Configuration Register - 0dh write */
#[repr(C)]
pub struct TCR_t {
    pub crc_disable: c_int, /* CRC - inhibit tx CRC */
    pub loop_cntl: u8,      /* LB0,LB1 - loopback control */
    pub ext_stoptx: c_int,  /* ATD - allow tx disable by external mcast */
    pub coll_prio: c_int,   /* OFST - backoff algorithm select */
    pub reserved: u8,       /* D5,D6,D7 - reserved */
}

/* Transmit Status Register - 04h read */
#[repr(C)]
pub struct TSR_t {
    pub tx_ok: c_int,      /* PTX - tx complete without error */
    pub reserved: c_int,   /*  D1 - reserved */
    pub collided: c_int,   /* COL - tx collided >= 1 times */
    pub aborted: c_int,    /* ABT - aborted due to excessive collisions */
    pub no_carrier: c_int, /* CRS - carrier-sense lost */
    pub fifo_ur: c_int,    /* FU  - FIFO underrun */
    pub cd_hbeat: c_int,   /* CDH - no tx cd-heartbeat from transceiver */
    pub ow_coll: c_int,    /* OWC - out-of-window collision */
}

/* Receive Configuration Register - 0ch write */
#[repr(C)]
pub struct RCR_t {
    pub errors_ok: c_int, /* SEP - accept pkts with rx errors */
    pub runts_ok: c_int,  /* AR  - accept < 64-byte runts */
    pub broadcast: c_int, /* AB  - accept eth broadcast address */
    pub multicast: c_int, /* AM  - check mcast hash array */
    pub promisc: c_int,   /* PRO - accept all packets */
    pub monitor: c_int,   /* MON - check pkts, but don't rx */
    pub reserved: u8,     /* D6,D7 - reserved */
}

/* Receive Status Register - 0ch read */
#[repr(C)]
pub struct RSR_t {
    pub rx_ok: c_int,       /* PRX - rx complete without error */
    pub bad_crc: c_int,     /* CRC - Bad CRC detected */
    pub bad_falign: c_int,  /* FAE - frame alignment error */
    pub fifo_or: c_int,     /* FO  - FIFO overrun */
    pub rx_missed: c_int,   /* MPA - missed packet error */
    pub rx_mbit: c_int,     /* PHY - unicast or mcast/bcast address match */
    pub rx_disabled: c_int, /* DIS - set when in monitor mode */
    pub deferred: c_int,    /* DFR - collision active */
}
#[repr(C)]
pub struct dp8390_t {
    /* Page 0 */
    pub CR: CR_t,
    pub ISR: ISR_t,
    pub IMR: IMR_t,
    pub DCR: DCR_t,
    pub TCR: TCR_t,
    pub TSR: TSR_t,
    pub RCR: RCR_t,
    pub RSR: RSR_t,

    pub local_dma: u16,    /* 01,02h read ; current local DMA addr */
    pub page_start: u8,    /* 01h write ; page start regr */
    pub page_stop: u8,     /* 02h write ; page stop regr */
    pub bound_ptr: u8,     /* 03h read/write ; boundary pointer */
    pub tx_page_start: u8, /* 04h write ; transmit page start reg */
    pub num_coll: u8,      /* 05h read  ; number-of-collisions reg */
    pub tx_bytes: u16,     /* 05,06h write ; transmit byte-count reg */
    pub fifo: u8,          /* 06h read  ; FIFO */
    pub remote_dma: u16,   /* 08,09h read ; current remote DMA addr */
    pub remote_start: u16, /* 08,09h write ; remote start address reg */
    pub remote_bytes: u16, /* 0a,0bh write ; remote byte-count reg */
    pub tallycnt_0: u8,    /* 0dh read  ; tally ctr 0 (frame align errs) */
    pub tallycnt_1: u8,    /* 0eh read  ; tally ctr 1 (CRC errors) */
    pub tallycnt_2: u8,    /* 0fh read  ; tally ctr 2 (missed pkt errs) */

    /* Page 1 */

    /*   Command Register 00h (repeated) */
    pub physaddr: [u8; 6], /* 01-06h read/write ; MAC address */
    pub curr_page: u8,     /* 07h read/write ; current page register */
    pub mchash: [u8; 8],   /* 08-0fh read/write ; multicast hash array */

    /* Page 2  - diagnostic use only */

    /*   Command Register 00h (repeated) */

    /*   Page Start Register 01h read  (repeated)
    *   Page Stop Register  02h read  (repeated)
    *   Current Local DMA Address 01,02h write (repeated)
    *   Transmit Page start address 04h read (repeated)
    *   Receive Configuration Register 0ch read (repeated)
    *   Transmit Configuration Register 0dh read (repeated)
    *   Data Configuration Register 0eh read (repeated)
    *   Interrupt Mask Register 0fh read (repeated)
    */
    pub rempkt_ptr: u8,   /* 03h read/write ; rmt next-pkt ptr */
    pub localpkt_ptr: u8, /* 05h read/write ; lcl next-pkt ptr */
    pub address_cnt: u16, /* 06,07h read/write ; address cter */

    /* Page 3  - should never be modified. */

    /* DP8390 memory */
    pub mem: *mut u8, /* on-chip packet memory */

    pub macaddr: [u8; 32], /* ASIC ROM'd MAC address, even bytes */
    pub macaddr_size: u8,  /* Defaults to 16 but can be 32 */
    pub flags: u8,         /* Flags affecting some behaviors. */
    pub id0: u8,           /* 0x50 for the Realtek NIC's, otherwise
                           0xFF. */
    pub id1: u8, /* 0x70 for the RTL8019AS, 0x43 for the
                 RTL8029AS, otherwise 0xFF. */
    pub mem_size: c_int,
    pub mem_start: c_int,
    pub mem_end: c_int,

    pub tx_timer_index: c_int,
    pub tx_timer_active: c_int,

    pub r#priv: *mut c_void,

    pub interrupt: Option<unsafe extern "C" fn(r#priv: *mut c_void, set: c_int)>,
}

/*
* 86Box	A hypervisor and IBM PC system emulator that specializes in
*		running old operating systems and software designed for IBM
*		PC systems and compatibles from 1981 through fairly recent
*		system designs based on the PCI bus.
*
*		Emulation of the DP8390 Network Interface Controller used by
*		the WD family, NE1000/NE2000 family, and 3Com 3C503 NIC's.
*
*
*
* Authors:	Miran Grca, <mgrca8@gmail.com>
*		Bochs project,
*
*		Copyright 2016-2018 Miran Grca.
*		Copyright 2008-2018 Bochs project.
*/
use crate::prelude::*;
use log::*;

/*
* Return the 6-bit index into the multicast
* table. Stolen unashamedly from FreeBSD's if_ed.c
*/
unsafe extern "C" fn mcast_index(dst: *const c_void) -> c_int {
    const POLYNOMIAL: u32 = 0x04c11db6;
    let mut crc = 0xffffffffu32;
    let mut carry: u32;
    let mut b: u8;
    let ep: [u8; 6] = *{ dst as *const [u8; 6] };

    for i in 0..6 {
        b = ep[i];
        for _ in 0..8 {
            carry = (crc >> 31) ^ (b as u32 & 1);
            crc <<= 1;
            b >>= 1;
            if carry != 0 {
                crc = (crc ^ POLYNOMIAL) | carry;
            }
        }
    }
    return (crc >> 26) as i32;
}

/*
* Access the 32K private RAM.
*
* The NE2000 memory is accessed through the data port of the
* ASIC (offset 0) after setting up a remote-DMA transfer.
* Both byte and word accesses are allowed.
* The first 16 bytes contain the MAC address at even locations,
* and there is 16K of buffer memory starting at 16K.
*/
#[no_mangle]
pub unsafe extern "C" fn dp8390_chipmem_read(dev: *mut dp8390_t, addr: u32, len: c_uint) -> u32 {
    let mut retval: u32 = 0;

    trace!("DP8390: Chipmem Read Address={:x}", addr);
    let mut addr = addr as usize;

    /* ROM'd MAC address */
    for i in 0..len {
        if (addr >= (*dev).mem_start as usize) && (addr < (*dev).mem_end as usize) {
            retval |= (*((*dev).mem.offset(addr as isize - (*dev).mem_start as isize)) as u32)
                << (i << 3);
        } else if addr < (*dev).macaddr_size as usize {
            retval |= ((*dev).macaddr[(addr & ((*dev).macaddr_size - 1) as usize) as usize] as u32)
                << (i << 3);
        } else {
            trace!("DP8390: out-of-bounds chipmem read, {:x}", addr);
            retval |= 0xff << (i << 3);
        }
        addr += 1;
    }

    return retval;
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_chipmem_write(
    dev: *mut dp8390_t,
    mut addr: u32,
    mut val: u32,
    len: c_uint,
) {
    trace!("DP8390: Chipmem Write Address={:x}", addr);

    for _ in 0..len {
        if (addr < (*dev).mem_start as u32) || (addr >= (*dev).mem_end as u32) {
            warn!("DP8390: out-of-bounds chipmem write, {:x}", addr);
            return;
        }

        *(*dev).mem.offset(addr as isize - (*dev).mem_start as isize) = (val & 0xFF) as u8;
        val >>= 8;
        addr += 1;
    }
}

/* Routines for handling reads/writes to the Command Register. */
#[no_mangle]
pub unsafe extern "C" fn dp8390_read_cr(dev: *mut dp8390_t) -> u32 {
    let retval: u32 = (((*dev).CR.pgsel as u32 & 0x03) << 6)
        | (((*dev).CR.rdma_cmd as u32 & 0x07) << 3)
        | (((*dev).CR.tx_packet as u32) << 2)
        | (((*dev).CR.start as u32) << 1)
        | ((*dev).CR.stop as u32);
    trace!("DP8390: read CR returns {:x}", retval);

    return retval;
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_write_cr(dev: *mut dp8390_t, mut val: u32) {
    trace!("DP8390: wrote {:x} to CR", val);

    /* Validate remote-DMA */
    if (val & 0x38) == 0x00 {
        debug!("DP8390: CR write - invalid rDMA value 0");
        val |= 0x20; /* dma_cmd == 4 is a safe default */
    }

    /* Check for s/w reset */
    if (val & 0x01) != 0 {
        (*dev).ISR.reset = 1;
        (*dev).CR.stop = 1;
    } else {
        (*dev).CR.stop = 0;
    }

    (*dev).CR.rdma_cmd = ((val & 0x38) >> 3) as u8;

    /* If start command issued, the RST bit in the ISR */
    /* must be cleared */
    if (val & 0x02) != 0 && (*dev).CR.start == 0 {
        (*dev).ISR.reset = 0;
    }

    (*dev).CR.start = ((val & 0x02) == 0x02) as i32;
    (*dev).CR.pgsel = ((val & 0xc0) >> 6) as u8;

    /* Check for send-packet command */
    if (*dev).CR.rdma_cmd == 3 {
        /* Set up DMA read from receive ring */
        (*dev).remote_start = (*dev).bound_ptr as u16 * 256;
        (*dev).remote_dma = (*dev).bound_ptr as u16 * 256;
        (*dev).remote_bytes = dp8390_chipmem_read(dev, (*dev).bound_ptr as u32 * 256 + 2, 2) as u16;
        trace!(
            "DP8390: sending buffer {:x} length {}",
            (*dev).remote_start,
            (*dev).remote_bytes
        );
    }

    /* Check for start-tx */
    if (val & 0x04) != 0 && (*dev).TCR.loop_cntl != 0 {
        // Not sure why this check is being done twice...
        if (*dev).TCR.loop_cntl != 0 {
            dp8390_rx_common(
                dev as _,
                (*dev)
                    .mem
                    .offset(((*dev).tx_page_start as isize * 256) - (*dev).mem_start as isize),
                (*dev).tx_bytes as i32,
            );
        }
    } else if (val & 0x04) != 0 {
        if (*dev).CR.stop != 0
            || ((*dev).CR.start == 0 && ((*dev).flags & DP8390_FLAG_CHECK_CR) != 0)
        {
            if (*dev).tx_bytes == 0
            /* njh@bandsman.co.uk */
            {
                return;
            } /* Solaris9 probe */
            debug!("DP8390: CR write - tx start, dev in reset");
        }

        if (*dev).tx_bytes == 0 {
            debug!("DP8390: CR write - tx start, tx bytes == 0");
        }

        /* Send the packet to the system driver */
        (*dev).CR.tx_packet = 1;

        network_tx(
            (*dev)
                .mem
                .offset(((*dev).tx_page_start as isize * 256) - (*dev).mem_start as isize),
            (*dev).tx_bytes as i32,
        );

        /* some more debug */
        if ((*dev).tx_timer_active) != 0 {
            debug!("DP8390: CR write, tx timer still active");
        }

        dp8390_tx(dev, val);
    }

    /* Linux probes for an interrupt by setting up a remote-DMA read
     * of 0 bytes with remote-DMA completion interrupts enabled.
     * Detect this here */
    if ((*dev).CR.rdma_cmd == 0x01) && (*dev).CR.start != 0 && ((*dev).remote_bytes == 0) {
        (*dev).ISR.rdma_done = 1;
        if (*dev).IMR.rdma_inte != 0 && (*dev).interrupt.is_some() {
            (*dev).interrupt.unwrap()((*dev).r#priv, 1);
            if ((*dev).flags & DP8390_FLAG_CLEAR_IRQ) != 0 {
                (*dev).interrupt.unwrap()((*dev).r#priv, 0);
            }
        }
    }
}

unsafe extern "C" fn dp8390_tx(dev: *mut dp8390_t, _val: u32) {
    (*dev).CR.tx_packet = 0;
    (*dev).TSR.tx_ok = 1;
    (*dev).ISR.pkt_tx = 1;

    /* Generate an interrupt if not masked */
    if (*dev).IMR.tx_inte != 0 && (*dev).interrupt.is_some() {
        (*dev).interrupt.unwrap()((*dev).r#priv, 1);
    }
    (*dev).tx_timer_active = 0;
}

/*
* Called by the platform-specific code when an Ethernet frame
* has been received. The destination address is tested to see
* if it should be accepted, and if the RX ring has enough room,
* it is copied into it and the receive process is updated.
*/
unsafe extern "C" fn dp8390_rx_common(
    r#priv: *mut c_void,
    buf: *mut u8,
    mut io_len: c_int,
) -> c_int {
    let dev: *mut dp8390_t = r#priv as _;
    const bcast_addr: [u8; 6] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff];
    let mut pkthdr: [u8; 4] = [0; 4];
    let mut startptr: *mut u8;
    let pages: c_int;
    let avail: c_int;
    let idx: c_int;
    let mut nextpage: c_int;
    let endbytes: c_int;

    if io_len != 60 {
        trace!("{}: rx_frame with length {}", "dp8390", io_len);
    }

    if ((*dev).CR.stop != 0) || ((*dev).page_start == 0) {
        return 0;
    }

    /*
     * Add the pkt header + CRC to the length, and work
     * out how many 256-byte pages the frame would occupy.
     */
    pages = (io_len + size_of_val(&pkthdr) as i32 + size_of::<u32>() as i32 + 255) / 256;
    if (*dev).curr_page < (*dev).bound_ptr {
        avail = (*dev).bound_ptr as i32 - (*dev).curr_page as i32;
    } else {
        avail = ((*dev).page_stop as i32 - (*dev).page_start as i32)
            - ((*dev).curr_page as i32 - (*dev).bound_ptr as i32);
    }

    /*
     * Avoid getting into a buffer overflow condition by
     * not attempting to do partial receives. The emulation
     * to handle this condition seems particularly painful.
     */
    if (avail < pages) || (avail == pages) {
        warn!("DP8390: no space");

        return 0;
    }

    if (io_len < 40/*60*/) && (*dev).RCR.runts_ok == 0 {
        debug!("DP8390: rejected small packet, length {}", io_len);

        return 1;
    }

    /* Some computers don't care... */
    if io_len < 60 {
        io_len = 60;
    }
    {
        let buf = *(buf as *mut [u8; 12]);
        debug!(
            "DP8390: RX {:x}:{:x}:{:x}:{:x}:{:x}:{:x} > {:x}:{:x}:{:x}:{:x}:{:x}:{:x} len {}",
            buf[6],
            buf[7],
            buf[8],
            buf[9],
            buf[10],
            buf[11],
            buf[0],
            buf[1],
            buf[2],
            buf[3],
            buf[4],
            buf[5],
            io_len
        );
    }
    /* Do address filtering if not in promiscuous mode. */
    if (*dev).RCR.promisc == 0 {
        /* If this is a broadcast frame.. */
        if (memcmp(buf as _, bcast_addr.as_ptr() as _, 6)) == 0 {
            /* Broadcast not enabled, we're done. */
            if (*dev).RCR.broadcast == 0 {
                debug!("DP8390: RX BC disabled");
                return 1;
            }
        }
        /* If this is a multicast frame.. */
        else if (*buf & 0x01) != 0 {
            /* Multicast not enabled, we're done. */
            if ((*dev).RCR.multicast) == 0 {
                debug!("DP8390: RX MC disabled");
                return 1;
            }

            /* Are we listening to this multicast address? */
            idx = mcast_index(buf as _);
            if ((*dev).mchash[idx as usize >> 3] & (1 << (idx as usize & 0x7))) != 0 {
                debug!("DP8390: RX MC not listed");
                return 1;
            }
        }
        /* Unicast, must be for us.. */
        else if (memcmp(buf as _, (*dev).physaddr.as_ptr() as _, 6)) != 0 {
            return 1;
        }
    } else {
        debug!("DP8390: RX promiscuous receive");
    }

    nextpage = (*dev).curr_page as i32 + pages;
    if nextpage >= (*dev).page_stop as i32 {
        nextpage -= (*dev).page_stop as i32 - (*dev).page_start as i32;
    }

    /* Set up packet header. */
    pkthdr[0] = 0x01; /* RXOK - packet is OK */
    if (*buf & 0x01) != 0 {
        pkthdr[0] |= 0x20; /* MULTICAST packet */
    }
    pkthdr[1] = nextpage as u8; /* ptr to next packet */
    pkthdr[2] = ((io_len + size_of_val(&pkthdr) as i32) & 0xff) as u8; /* length-low */
    pkthdr[3] = ((io_len + size_of_val(&pkthdr) as i32) >> 8) as u8; /* length-hi */
    trace!(
        "DP8390: RX pkthdr [{:x} {:x} {:x} {:x}]",
        pkthdr[0],
        pkthdr[1],
        pkthdr[2],
        pkthdr[3]
    );

    /* Copy into buffer, update curpage, and signal interrupt if config'd */
    startptr = (*dev)
        .mem
        .offset(((*dev).curr_page as isize * 256) as isize - (*dev).mem_start as isize);
    memcpy(startptr as _, pkthdr.as_ptr() as _, size_of_val(&pkthdr));
    if (nextpage > (*dev).curr_page as i32)
        || (((*dev).curr_page as i32 + pages) == (*dev).page_stop as i32)
    {
        memcpy(
            startptr.offset(size_of_val(&pkthdr) as isize) as _,
            buf as _,
            io_len as usize,
        );
    } else {
        endbytes = ((*dev).page_stop as i32 - (*dev).curr_page as i32) * 256;
        memcpy(
            startptr.offset(size_of_val(&pkthdr) as isize) as _,
            buf as _,
            endbytes as usize - size_of_val(&pkthdr),
        );
        startptr = (*dev)
            .mem
            .offset(((*dev).page_start as isize * 256) as isize - (*dev).mem_start as isize);
        memcpy(
            startptr as _,
            buf.offset(endbytes as isize - size_of_val(&pkthdr) as isize) as _,
            io_len as usize + 8 - endbytes as usize,
        );
    }
    (*dev).curr_page = nextpage as u8;

    (*dev).RSR.rx_ok = 1;
    (*dev).RSR.rx_mbit = if (*buf & 0x01) != 0 { 1 } else { 0 };
    (*dev).ISR.pkt_rx = 1;

    if (*dev).IMR.rx_inte != 0 && (*dev).interrupt.is_some() {
        (*dev).interrupt.unwrap()((*dev).r#priv, 1);
    }

    return 1;
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_rx(r#priv: *mut c_void, buf: *mut u8, io_len: c_int) -> c_int {
    let dev: *mut dp8390_t = r#priv as _;

    if ((*dev).DCR.r#loop == 0) || ((*dev).TCR.loop_cntl != 0) {
        return 0;
    }

    return dp8390_rx_common(r#priv, buf, io_len);
}

/* Handle reads/writes to the 'zeroth' page of the DS8390 register file. */
#[no_mangle]
pub unsafe extern "C" fn dp8390_page0_read(dev: *mut dp8390_t, off: u32, len: c_uint) -> u32 {
    let mut retval: u8 = 0;

    if len > 1 {
        /* encountered with win98 hardware probe */
        warn!(
            "DP8390: bad length! Page0 read from register 0x{:x}, len={}",
            off, len
        );
        return retval as u32;
    }

    match off {
        0x01 =>
        /* CLDA0 */
        {
            retval = ((*dev).local_dma & 0xff) as u8;
        }

        0x02 =>
        /* CLDA1 */
        {
            retval = ((*dev).local_dma >> 8) as u8;
        }

        0x03 =>
        /* BNRY */
        {
            retval = (*dev).bound_ptr;
        }

        0x04 =>
        /* TSR */
        {
            retval = (((*dev).TSR.ow_coll << 7)
                | ((*dev).TSR.cd_hbeat << 6)
                | ((*dev).TSR.fifo_ur << 5)
                | ((*dev).TSR.no_carrier << 4)
                | ((*dev).TSR.aborted << 3)
                | ((*dev).TSR.collided << 2)
                | ((*dev).TSR.tx_ok)) as u8;
        }

        0x05 =>
        /* NCR */
        {
            retval = (*dev).num_coll;
        }

        0x06 =>
        /* FIFO */
        {
            /* reading FIFO is only valid in loopback mode */
            debug!("DP8390: reading FIFO not supported yet");
            retval = (*dev).fifo;
        }

        0x07 =>
        /* ISR */
        {
            retval = (((*dev).ISR.reset << 7)
                | ((*dev).ISR.rdma_done << 6)
                | ((*dev).ISR.cnt_oflow << 5)
                | ((*dev).ISR.overwrite << 4)
                | ((*dev).ISR.tx_err << 3)
                | ((*dev).ISR.rx_err << 2)
                | ((*dev).ISR.pkt_tx << 1)
                | ((*dev).ISR.pkt_rx)) as u8;
        }

        0x08 =>
        /* CRDA0 */
        {
            retval = ((*dev).remote_dma & 0xff) as u8;
        }

        0x09 =>
        /* CRDA1 */
        {
            retval = ((*dev).remote_dma >> 8) as u8;
        }

        0x0a =>
        /* reserved / RTL8029ID0 */
        {
            retval = (*dev).id0;
        }

        0x0b =>
        /* reserved / RTL8029ID1 */
        {
            retval = (*dev).id1;
        }

        0x0c =>
        /* RSR */
        {
            retval = (((*dev).RSR.deferred << 7)
                | ((*dev).RSR.rx_disabled << 6)
                | ((*dev).RSR.rx_mbit << 5)
                | ((*dev).RSR.rx_missed << 4)
                | ((*dev).RSR.fifo_or << 3)
                | ((*dev).RSR.bad_falign << 2)
                | ((*dev).RSR.bad_crc << 1)
                | ((*dev).RSR.rx_ok)) as u8;
        }

        0x0d =>
        /* CNTR0 */
        {
            retval = (*dev).tallycnt_0;
        }

        0x0e =>
        /* CNTR1 */
        {
            retval = (*dev).tallycnt_1;
        }

        0x0f =>
        /* CNTR2 */
        {
            retval = (*dev).tallycnt_2;
        }

        _ => {
            warn!("DP8390: Page0 register 0x{:x} out of range", off);
        }
    }

    trace!(
        "DP8390: Page0 read from register 0x{:x}, value=0x{:x}",
        off,
        retval
    );

    return retval as u32;
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_page0_write(
    dev: *mut dp8390_t,
    off: u32,
    mut val: u32,
    _len: c_uint,
) {
    let val2: u8;

    trace!(
        "DP839: Page0 write to register 0x{:x}, value=0x{:x}",
        off,
        val
    );

    match off {
        0x01 =>
        /* PSTART */
        {
            (*dev).page_start = val as u8;
            trace!("DP8390: Starting RAM address: {:x}", val << 8);
        }

        0x02 =>
        /* PSTOP */
        {
            (*dev).page_stop = val as u8;
            trace!("DP8390: Stopping RAM address: {:x}", val << 8);
        }

        0x03 =>
        /* BNRY */
        {
            (*dev).bound_ptr = val as u8;
        }

        0x04 =>
        /* TPSR */
        {
            (*dev).tx_page_start = val as u8;
        }

        0x05 =>
        /* TBCR0 */
        {
            /* Clear out low byte and re-insert */
            (*dev).tx_bytes &= 0xff00;
            (*dev).tx_bytes |= (val & 0xff) as u16;
        }

        0x06 =>
        /* TBCR1 */
        {
            /* Clear out high byte and re-insert */
            (*dev).tx_bytes &= 0x00ff;
            (*dev).tx_bytes |= ((val & 0xff) << 8) as u16;
        }

        0x07 =>
        /* ISR */
        {
            val &= 0x7f; /* clear RST bit - status-only bit */
            /* All other values are cleared iff the ISR bit is 1 */
            (*dev).ISR.pkt_rx &= !(((val & 0x01) == 0x01) as c_int);
            (*dev).ISR.pkt_tx &= !(((val & 0x02) == 0x02) as c_int);
            (*dev).ISR.rx_err &= !(((val & 0x04) == 0x04) as c_int);
            (*dev).ISR.tx_err &= !(((val & 0x08) == 0x08) as c_int);
            (*dev).ISR.overwrite &= !(((val & 0x10) == 0x10) as c_int);
            (*dev).ISR.cnt_oflow &= !(((val & 0x20) == 0x20) as c_int);
            (*dev).ISR.rdma_done &= !(((val & 0x40) == 0x40) as c_int);
            val = (((*dev).ISR.rdma_done << 6)
                | ((*dev).ISR.cnt_oflow << 5)
                | ((*dev).ISR.overwrite << 4)
                | ((*dev).ISR.tx_err << 3)
                | ((*dev).ISR.rx_err << 2)
                | ((*dev).ISR.pkt_tx << 1)
                | ((*dev).ISR.pkt_rx)) as u32;
            val &= (((*dev).IMR.rdma_inte << 6)
                | ((*dev).IMR.cofl_inte << 5)
                | ((*dev).IMR.overw_inte << 4)
                | ((*dev).IMR.txerr_inte << 3)
                | ((*dev).IMR.rxerr_inte << 2)
                | ((*dev).IMR.tx_inte << 1)
                | ((*dev).IMR.rx_inte)) as u32;
            if (val == 0x00) && (*dev).interrupt.is_some() {
                (*dev).interrupt.unwrap()((*dev).r#priv, 0);
            }
        }

        0x08 =>
        /* RSAR0 */
        {
            /* Clear out low byte and re-insert */
            (*dev).remote_start &= 0xff00;
            (*dev).remote_start |= (val & 0xff) as u16;
            (*dev).remote_dma = (*dev).remote_start;
        }

        0x09 =>
        /* RSAR1 */
        {
            /* Clear out high byte and re-insert */
            (*dev).remote_start &= 0x00ff;
            (*dev).remote_start |= ((val & 0xff) << 8) as u16;
            (*dev).remote_dma = (*dev).remote_start;
        }

        0x0a =>
        /* RBCR0 */
        {
            /* Clear out low byte and re-insert */
            (*dev).remote_bytes &= 0xff00;
            (*dev).remote_bytes |= (val & 0xff) as u16;
        }

        0x0b =>
        /* RBCR1 */
        {
            /* Clear out high byte and re-insert */
            (*dev).remote_bytes &= 0x00ff;
            (*dev).remote_bytes |= ((val & 0xff) << 8) as u16;
        }

        0x0c =>
        /* RCR */
        {
            /* Check if the reserved bits are set */
            if (val & 0xc0) != 0 {
                debug!("DP8390: RCR write, reserved bits set");
            }

            /* Set all other bit-fields */
            (*dev).RCR.errors_ok = ((val & 0x01) == 0x01) as i32;
            (*dev).RCR.runts_ok = ((val & 0x02) == 0x02) as i32;
            (*dev).RCR.broadcast = ((val & 0x04) == 0x04) as i32;
            (*dev).RCR.multicast = ((val & 0x08) == 0x08) as i32;
            (*dev).RCR.promisc = ((val & 0x10) == 0x10) as i32;
            (*dev).RCR.monitor = ((val & 0x20) == 0x20) as i32;

            /* Monitor bit is a little suspicious... */
            if (val & 0x20) != 0 {
                debug!("DP8390: RCR write, monitor bit set!");
            }
        }

        0x0d =>
        /* TCR */
        {
            /* Check reserved bits */
            if (val & 0xe0) != 0 {
                debug!("DP8390: TCR write, reserved bits set");
            }

            /* Test loop mode (not supported) */
            if (val & 0x06) != 0 {
                (*dev).TCR.loop_cntl = ((val & 0x6) >> 1) as u8;
                debug!(
                    "DP8390: TCR write, loop mode {} not supported",
                    (*dev).TCR.loop_cntl
                );
            } else {
                (*dev).TCR.loop_cntl = 0;
            }

            /* Inhibit-CRC not supported. */
            if (val & 0x01) != 0 {
                debug!("DP8390: TCR write, inhibit-CRC not supported");
            }

            /* Auto-transmit disable very suspicious */
            if (val & 0x08) != 0 {
                debug!("DP8390: TCR write, auto transmit disable not supported");
            }

            /* Allow collision-offset to be set, although not used */
            (*dev).TCR.coll_prio = ((val & 0x08) == 0x08) as i32;
        }

        0x0e =>
        /* DCR */
        {
            /* the loopback mode is not suppported yet */
            if (val & 0x08) == 0 {
                debug!("DP8390: DCR write, loopback mode selected");
            }

            /* It is questionable to set longaddr and auto_rx, since
             * they are not supported on the NE2000. Print a warning
             * and continue. */
            if (val & 0x04) != 0 {
                debug!("DP8390: DCR write - LAS set ???");
            }
            if (val & 0x10) != 0 {
                debug!("DP8390: DCR write - AR set ???");
            }

            /* Set other values. */
            (*dev).DCR.wdsize = ((val & 0x01) == 0x01) as i32;
            (*dev).DCR.endian = ((val & 0x02) == 0x02) as i32;
            (*dev).DCR.longaddr = ((val & 0x04) == 0x04) as i32; /* illegal ? */
            (*dev).DCR.r#loop = ((val & 0x08) == 0x08) as i32;
            (*dev).DCR.auto_rx = ((val & 0x10) == 0x10) as i32; /* also illegal ? */
            (*dev).DCR.fifo_size = ((val & 0x50) >> 5) as u8;
        }

        0x0f =>
        /* IMR */
        {
            /* Check for reserved bit */
            if (val & 0x80) != 0 {
                debug!("DP8390: IMR write, reserved bit set");
            }

            /* Set other values */
            (*dev).IMR.rx_inte = ((val & 0x01) == 0x01) as i32;
            (*dev).IMR.tx_inte = ((val & 0x02) == 0x02) as i32;
            (*dev).IMR.rxerr_inte = ((val & 0x04) == 0x04) as i32;
            (*dev).IMR.txerr_inte = ((val & 0x08) == 0x08) as i32;
            (*dev).IMR.overw_inte = ((val & 0x10) == 0x10) as i32;
            (*dev).IMR.cofl_inte = ((val & 0x20) == 0x20) as i32;
            (*dev).IMR.rdma_inte = ((val & 0x40) == 0x40) as i32;
            val2 = (((*dev).ISR.rdma_done << 6)
                | ((*dev).ISR.cnt_oflow << 5)
                | ((*dev).ISR.overwrite << 4)
                | ((*dev).ISR.tx_err << 3)
                | ((*dev).ISR.rx_err << 2)
                | ((*dev).ISR.pkt_tx << 1)
                | ((*dev).ISR.pkt_rx)) as u8;
            if (*dev).interrupt.is_some() {
                if ((val & val2 as u32) & 0x7f) == 0 {
                    (*dev).interrupt.unwrap()((*dev).r#priv, 0);
                } else {
                    (*dev).interrupt.unwrap()((*dev).r#priv, 1);
                }
            }
        }

        _ => {
            warn!("DP8390: Page0 write, bad register 0x{:x}", off);
        }
    }
}

/* Handle reads/writes to the first page of the DS8390 register file. */
#[no_mangle]
pub unsafe extern "C" fn dp8390_page1_read(dev: *mut dp8390_t, off: u32, len: c_uint) -> u32 {
    trace!("DP8390: Page1 read from register 0x{:x}, len={}", off, len);

    match off {
        0x01 | 0x02 | 0x03 | 0x04 | 0x05 | 0x06 => {
            /* PAR0-5 */
            return ((*dev).physaddr[off as usize - 1]) as u32;
        }

        0x07 => {
            /* CURR */
            trace!("DP8390: returning current page: 0x{:x}", ((*dev).curr_page));
            return ((*dev).curr_page) as u32;
        }

        0x08 | 0x09 | 0x0a | 0x0b | 0x0c | 0x0d | 0x0e | 0x0f => {
            /* MAR0-7 */
            return ((*dev).mchash[off as usize - 8]) as u32;
        }

        _ => {
            warn!("DP8390: Page1 read register 0x{:x} out of range", off);
            return 0;
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_page1_write(dev: *mut dp8390_t, off: u32, val: u32, len: c_uint) {
    trace!(
        "DP8390: Page1 write to register 0x{:x}, len={}, value=0x{:x}",
        off,
        len,
        val
    );

    match off {
        0x01 | 0x02 | 0x03 | 0x04 | 0x05 | 0x06 => {
            /* PAR0-5 */
            (*dev).physaddr[off as usize - 1] = val as u8;
            if off == 6 {
                debug!(
                    "DP8390: Physical address set to {:x}:{:x}:{:x}:{:x}:{:x}:{:x}",
                    (*dev).physaddr[0],
                    (*dev).physaddr[1],
                    (*dev).physaddr[2],
                    (*dev).physaddr[3],
                    (*dev).physaddr[4],
                    (*dev).physaddr[5]
                );
            }
        }

        0x07 => {
            /* CURR */
            (*dev).curr_page = val as u8;
        }

        0x08 | 0x09 | 0x0a | 0x0b | 0x0c | 0x0d | 0x0e | 0x0f => {
            /* MAR0-7 */
            (*dev).mchash[off as usize - 8] = val as u8;
        }

        _ => {
            warn!("DP8390: Page1 write register 0x{:x} out of range", off);
        }
    }
}

/* Handle reads/writes to the second page of the DS8390 register file. */
#[no_mangle]
pub unsafe extern "C" fn dp8390_page2_read(dev: *mut dp8390_t, off: u32, len: c_uint) -> u32 {
    trace!("DP8390: Page2 read from register 0x{:x}, len={}", off, len);

    match off {
        0x01 =>
        /* PSTART */
        {
            return ((*dev).page_start) as u32
        }

        0x02 =>
        /* PSTOP */
        {
            return ((*dev).page_stop) as u32
        }

        0x03 =>
        /* Remote Next-packet pointer */
        {
            return ((*dev).rempkt_ptr) as u32
        }

        0x04 =>
        /* TPSR */
        {
            return ((*dev).tx_page_start) as u32
        }

        0x05 =>
        /* Local Next-packet pointer */
        {
            return ((*dev).localpkt_ptr) as u32
        }

        0x06 =>
        /* Address counter (upper) */
        {
            return ((*dev).address_cnt >> 8) as u32
        }

        0x07 =>
        /* Address counter (lower) */
        {
            return ((*dev).address_cnt & 0xff) as u32
        }

        0x08 | 0x09 | 0x0a | 0x0b => {
            /* Reserved */
            warn!("DP8390: reserved Page2 read - register 0x{:x}", off);
            return 0xff;
        }

        0x0c =>
        /* RCR */
        {
            return (((*dev).RCR.monitor << 5)
                | ((*dev).RCR.promisc << 4)
                | ((*dev).RCR.multicast << 3)
                | ((*dev).RCR.broadcast << 2)
                | ((*dev).RCR.runts_ok << 1)
                | ((*dev).RCR.errors_ok)) as u32
        }

        0x0d =>
        /* TCR */
        {
            return (((*dev).TCR.coll_prio << 4)
                | ((*dev).TCR.ext_stoptx << 3)
                | (((*dev).TCR.loop_cntl as i32 & 0x3) << 1)
                | ((*dev).TCR.crc_disable)) as u32
        }

        0x0e =>
        /* DCR */
        {
            return ((((*dev).DCR.fifo_size as i32 & 0x3) << 5)
                | ((*dev).DCR.auto_rx << 4)
                | ((*dev).DCR.r#loop << 3)
                | ((*dev).DCR.longaddr << 2)
                | ((*dev).DCR.endian << 1)
                | ((*dev).DCR.wdsize)) as u32
        }

        0x0f =>
        /* IMR */
        {
            return (((*dev).IMR.rdma_inte << 6)
                | ((*dev).IMR.cofl_inte << 5)
                | ((*dev).IMR.overw_inte << 4)
                | ((*dev).IMR.txerr_inte << 3)
                | ((*dev).IMR.rxerr_inte << 2)
                | ((*dev).IMR.tx_inte << 1)
                | ((*dev).IMR.rx_inte)) as u32
        }

        _ => {
            warn!("DP8390: Page2 register 0x{:x} out of range", off);
        }
    }

    return 0;
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_page2_write(dev: *mut dp8390_t, off: u32, val: u32, len: c_uint) {
    /* Maybe all writes here should be BX_PANIC()'d, since they
    affect internal operation, but let them through for now
    and print a warning. */
    warn!(
        "DP8390: Page2 write to register 0x{:x}, len={}, value=0x{:x}",
        off, len, val
    );

    match off {
        0x01 =>
        /* CLDA0 */
        {
            /* Clear out low byte and re-insert */
            (*dev).local_dma &= 0xff00;
            (*dev).local_dma |= (val & 0xff) as u16;
        }

        0x02 =>
        /* CLDA1 */
        {
            /* Clear out high byte and re-insert */
            (*dev).local_dma &= 0x00ff;
            (*dev).local_dma |= ((val & 0xff) << 8) as u16;
        }

        0x03 =>
        /* Remote Next-pkt pointer */
        {
            (*dev).rempkt_ptr = val as u8;
        }

        0x04 => {
            debug!("DP8390: Page 2 write to reserved register 0x04");
        }

        0x05 =>
        /* Local Next-packet pointer */
        {
            (*dev).localpkt_ptr = val as u8;
        }

        0x06 =>
        /* Address counter (upper) */
        {
            /* Clear out high byte and re-insert */
            (*dev).address_cnt &= 0x00ff;
            (*dev).address_cnt |= ((val & 0xff) << 8) as u16;
        }

        0x07 =>
        /* Address counter (lower) */
        {
            /* Clear out low byte and re-insert */
            (*dev).address_cnt &= 0xff00;
            (*dev).address_cnt |= (val & 0xff) as u16;
        }

        0x08 | 0x09 | 0x0a | 0x0b | 0x0c | 0x0d | 0x0e | 0x0f => {
            warn!("DP8390: Page2 write to reserved register 0x{:x}", off);
        }

        _ => {
            warn!("DP8390: Page2 write, illegal register 0x{:x}", off);
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_set_defaults(dev: *mut dp8390_t, flags: u8) {
    (*dev).macaddr_size = if (flags & DP8390_FLAG_EVEN_MAC as u8) != 0 {
        32
    } else {
        16
    };

    (*dev).flags = flags;
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_mem_alloc(dev: *mut dp8390_t, start: u32, size: u32) {
    (*dev).mem = calloc(size as usize, size_of::<u8>()) as *mut u8;
    memset((*dev).mem as _, 0, size as usize * size_of::<u8>());
    (*dev).mem_start = start as i32;
    (*dev).mem_end = start as i32 + size as i32;
    (*dev).mem_size = size as i32;
    trace!(
        "DP8390: Mapped {} bytes of memory at address {:x} in the address space",
        size,
        start
    );
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_set_id(dev: *mut dp8390_t, id0: u8, id1: u8) {
    (*dev).id0 = id0;
    (*dev).id1 = id1;
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_reset(dev: *mut dp8390_t) {
    let max: c_int;
    let mut shift: c_int = 0;

    if ((*dev).flags & DP8390_FLAG_EVEN_MAC as u8) != 0 {
        shift = 1;
    }

    max = 16 << shift;

    /* Initialize the MAC address area by doubling the physical address */
    for i in 0..max {
        if i < (6 << shift) {
            (*dev).macaddr[i as usize] = (*dev).physaddr[(i as usize) >> shift as usize];
        } else
        /* Signature */
        {
            (*dev).macaddr[i as usize] = 0x57;
        }
    }

    /* Zero out registers and memory */
    memset(&mut (*dev).CR as *mut _ as _, 0x00, size_of_val(&(*dev).CR));
    memset(
        &mut (*dev).ISR as *mut _ as _,
        0x00,
        size_of_val(&(*dev).ISR),
    );
    memset(
        &mut (*dev).IMR as *mut _ as _,
        0x00,
        size_of_val(&(*dev).IMR),
    );
    memset(
        &mut (*dev).DCR as *mut _ as _,
        0x00,
        size_of_val(&(*dev).DCR),
    );
    memset(
        &mut (*dev).TCR as *mut _ as _,
        0x00,
        size_of_val(&(*dev).TCR),
    );
    memset(
        &mut (*dev).TSR as *mut _ as _,
        0x00,
        size_of_val(&(*dev).TSR),
    );
    memset(
        &mut (*dev).RSR as *mut _ as _,
        0x00,
        size_of_val(&(*dev).RSR),
    );
    (*dev).tx_timer_active = 0;
    (*dev).local_dma = 0;
    (*dev).page_start = 0;
    (*dev).page_stop = 0;
    (*dev).bound_ptr = 0;
    (*dev).tx_page_start = 0;
    (*dev).num_coll = 0;
    (*dev).tx_bytes = 0;
    (*dev).fifo = 0;
    (*dev).remote_dma = 0;
    (*dev).remote_start = 0;

    (*dev).remote_bytes = 0;

    (*dev).tallycnt_0 = 0;
    (*dev).tallycnt_1 = 0;
    (*dev).tallycnt_2 = 0;

    (*dev).curr_page = 0;

    (*dev).rempkt_ptr = 0;
    (*dev).localpkt_ptr = 0;
    (*dev).address_cnt = 0;

    memset((*dev).mem as _, 0x00, (*dev).mem_size as usize);

    /* Set power-up conditions */
    (*dev).CR.stop = 1;
    (*dev).CR.rdma_cmd = 4;
    (*dev).ISR.reset = 1;
    (*dev).DCR.longaddr = 1;

    if (*dev).interrupt.is_some() {
        (*dev).interrupt.unwrap()((*dev).r#priv, 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn dp8390_soft_reset(dev: *mut dp8390_t) {
    memset(
        &mut ((*dev).ISR) as *mut _ as _,
        0x00,
        size_of_val(&(*dev).ISR),
    );
    (*dev).ISR.reset = 1;
}

unsafe extern "C" fn dp8390_init(_info: *const device_t) -> *mut c_void {
    let dp8390 = malloc(size_of::<dp8390_t>()) as *mut dp8390_t;
    memset(dp8390 as _, 0, size_of::<dp8390_t>());

    /* Set values assuming WORD and only the clear IRQ flag -
    - the NIC can then call dp8390_set_defaults() again to
    change that. */
    dp8390_set_defaults(dp8390, DP8390_FLAG_CLEAR_IRQ as u8);

    /* Set the two registers some NIC's use as ID bytes,
    to their default values of 0xFF. */
    dp8390_set_id(dp8390, 0xff, 0xff);

    return dp8390 as _;
}

unsafe extern "C" fn dp8390_close(r#priv: *mut c_void) {
    let dp8390: *mut dp8390_t = r#priv as _;

    /* Make sure the platform layer is shut down. */
    network_close();

    if !(dp8390).is_null() {
        if !((*dp8390).mem).is_null() {
            free((*dp8390).mem as _);
        }

        free(dp8390 as _);
    }
}

#[no_mangle]
pub static dp8390_device: device_t = device_t {
    name: b"DP8390 Network Interface Controller\0".as_ptr() as _,
    internal_name: b"dp8390\0".as_ptr() as _,
    flags: 0,
    local: 0,
    init: Some(dp8390_init),
    close: Some(dp8390_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: ptr::null(),
};
