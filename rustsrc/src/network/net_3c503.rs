/*
 * 86Box	An emulator of (mostly) x86-based PC systems and devices,
 *		using the ISA, EISA, VLB, MCA, and PCI system buses,
 *		roughly spanning the era between 1981 and 1995.
 *
 *		This file is part of the 86Box Project.
 *
 *		Implementation of the following network controllers:
 *			- 3Com Etherlink II 3c503 (ISA 8-bit).
 *
 *
 *
 * Based on	@(#)3c503.cpp Carl (MAME)
 *
 * Authors:	TheCollector1995, <mariogplayer@gmail.com>
 *		Miran Grca, <mgrca8@gmail.com>
 *		Fred N. van Kempen, <decwiz@yahoo.com>
 *		Carl, <unknown e-mail address>
 *
 *		Copyright 2018 TheCollector1995.
 *		Copyright 2018 Miran Grca.
 *		Copyright 2017,2018 Fred N. van Kempen.
 *		Portions Copyright (C) 2018  MAME Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free  Software  Foundation; either  version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is  distributed in the hope that it will be useful, but
 * WITHOUT   ANY  WARRANTY;  without  even   the  implied  warranty  of
 * MERCHANTABILITY  or FITNESS  FOR A PARTICULAR  PURPOSE. See  the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 *
 *   Free Software Foundation, Inc.
 *   59 Temple Place - Suite 330
 *   Boston, MA 02111-1307
 *   USA.
 */
use super::net_dp8390::*;
use crate::{mem::*, prelude::*, random::*};
use log::*;
use std::{
    mem::{size_of, size_of_val},
    ptr,
};

#[repr(C)]
struct threec503_regs_t {
    pstr: u8,
    pspr: u8,
    dqtr: u8,
    bcfr: u8,
    pcfr: u8,
    gacfr: u8,
    ctrl: u8,
    streg: u8,
    idcfr: u8,
    da: u16,
    vptr: u32,
    rfmsb: u8,
    rflsb: u8,
}

#[repr(C)]
struct threec503_t {
    dp8390: *mut dp8390_t,
    ram_mapping: mem_mapping_t,
    base_address: u32,
    base_irq: c_int,
    bios_addr: u32,
    maclocal: [u8; 6], /* configured MAC (local) address */
    regs: threec503_regs_t,
    dma_channel: c_int,
}

unsafe extern "C" fn threec503_interrupt(r#priv: *mut c_void, set: c_int) {
    let dev: *mut threec503_t = r#priv as _;

    match (*dev).base_irq {
        2 => {
            (*dev).regs.idcfr = 0x10;
        }

        3 => {
            (*dev).regs.idcfr = 0x20;
        }

        4 => {
            (*dev).regs.idcfr = 0x40;
        }

        5 => {
            (*dev).regs.idcfr = 0x80;
        }
        _ => {}
    }

    if (set) != 0 {
        picint(1 << (*dev).base_irq);
    } else {
        picintc(1 << (*dev).base_irq);
    }
}

unsafe extern "C" fn threec503_ram_write(addr: u32, val: u8, r#priv: *mut c_void) {
    let dev: *mut threec503_t = r#priv as _;

    if (addr & 0x3fff) >= 0x2000 {
        return;
    }

    *(*(*dev).dp8390).mem.offset(addr as isize & 0x1fff) = val;
}

unsafe extern "C" fn threec503_ram_read(addr: u32, r#priv: *mut c_void) -> u8 {
    let dev: *mut threec503_t = r#priv as _;

    if (addr & 0x3fff) >= 0x2000 {
        return 0xff;
    }

    return *(*(*dev).dp8390).mem.offset(addr as isize & 0x1fff);
}

unsafe extern "C" fn threec503_set_drq(dev: *mut threec503_t) {
    match (*dev).dma_channel {
        1 => {
            (*dev).regs.idcfr = 1;
        }

        2 => {
            (*dev).regs.idcfr = 2;
        }

        3 => {
            (*dev).regs.idcfr = 4;
        }
        _ => {}
    }
}

/* reset - restore state to power-up, cancelling all i/o */
unsafe extern "C" fn threec503_reset(r#priv: *mut c_void) {
    let dev: *mut threec503_t = r#priv as _;

    trace!("3Com503: reset");

    dp8390_reset((*dev).dp8390);

    memset(
        &mut (*dev).regs as *mut _ as _,
        0,
        size_of_val(&(*dev).regs),
    );

    (*dev).regs.ctrl = 0x0a;
}

unsafe extern "C" fn threec503_nic_lo_read(addr: u16, r#priv: *mut c_void) -> u8 {
    let dev: *mut threec503_t = r#priv as _;
    let mut retval: u8 = 0;
    let off: u32 = addr as u32 - (*dev).base_address;

    match ((*dev).regs.ctrl >> 2) & 3 {
        0x00 => {
            trace!("Read offset={:x}", off);
            if off == 0x00 {
                retval = dp8390_read_cr((*dev).dp8390) as u8;
            } else {
                match (*(*dev).dp8390).CR.pgsel {
                    0x00 => {
                        retval = dp8390_page0_read((*dev).dp8390, off, 1) as u8;
                    }

                    0x01 => {
                        retval = dp8390_page1_read((*dev).dp8390, off, 1) as u8;
                    }

                    0x02 => {
                        retval = dp8390_page2_read((*dev).dp8390, off, 1) as u8;
                    }

                    0x03 => {
                        retval = 0xff;
                    }

                    _ => {}
                }
            }
        }

        0x01 => {
            retval = (*(*dev).dp8390).macaddr[off as usize];
        }

        0x02 => {
            retval = (*(*dev).dp8390).macaddr[off as usize + 0x10];
        }

        0x03 => {
            retval = 0xff;
        }

        _ => {}
    }

    return retval;
}

unsafe extern "C" fn threec503_nic_lo_write(addr: u16, val: u8, r#priv: *mut c_void) {
    let dev: *mut threec503_t = r#priv as _;
    let off: u32 = addr as u32 - (*dev).base_address;
    let val = val as u32;

    match ((*dev).regs.ctrl >> 2) & 3 {
        0x00 => {
            /* The high 16 bytes of i/o space are for the ne2000 asic -
            the low 16 bytes are for the DS8390, with the current
            page being selected by the PS0,PS1 registers in the
            command register */
            if off == 0x00 {
                dp8390_write_cr((*dev).dp8390, val);
            } else {
                match (*(*dev).dp8390).CR.pgsel {
                    0x00 => {
                        dp8390_page0_write((*dev).dp8390, off, val, 1);
                    }

                    0x01 => {
                        dp8390_page1_write((*dev).dp8390, off, val, 1);
                    }

                    0x02 => {
                        dp8390_page2_write((*dev).dp8390, off, val, 1);
                    }

                    0x03 => {}
                    _ => {}
                }
            }
        }

        0x01 | 0x02 | 0x03 => {}
        _ => {}
    }

    trace!("3Com503: write addr {:x}, value {:x}", addr, val);
}

unsafe extern "C" fn threec503_nic_hi_read(addr: u16, r#priv: *mut c_void) -> u8 {
    let dev: *mut threec503_t = r#priv as _;

    trace!("3Com503: Read GA address={:x}", addr);

    match addr & 0x0f {
        0x00 => {
            return (*dev).regs.pstr;
        }

        0x01 => {
            return (*dev).regs.pspr;
        }

        0x02 => {
            return (*dev).regs.dqtr;
        }

        0x03 => {
            match (*dev).base_address {
                0x310 => {
                    (*dev).regs.bcfr = 0x40;
                }

                0x330 => {
                    (*dev).regs.bcfr = 0x20;
                }

                0x350 => {
                    (*dev).regs.bcfr = 0x10;
                }

                0x250 => {
                    (*dev).regs.bcfr = 0x08;
                }

                0x280 => {
                    (*dev).regs.bcfr = 0x04;
                }

                0x2a0 => {
                    (*dev).regs.bcfr = 0x02;
                }

                0x2e0 => {
                    (*dev).regs.bcfr = 0x01;
                }

                0x300 | _ => {
                    (*dev).regs.bcfr = 0x80;
                }
            }

            return (*dev).regs.bcfr;
        }

        0x04 => {
            match (*dev).bios_addr {
                0xdc000 => {
                    (*dev).regs.pcfr = 0x80;
                }

                0xd8000 => {
                    (*dev).regs.pcfr = 0x40;
                }

                0xcc000 => {
                    (*dev).regs.pcfr = 0x20;
                }

                0xc8000 => {
                    (*dev).regs.pcfr = 0x10;
                }

                _ => {}
            }

            return (*dev).regs.pcfr;
        }

        0x05 => {
            return (*dev).regs.gacfr;
        }

        0x06 => {
            return (*dev).regs.ctrl;
        }

        0x07 => {
            return (*dev).regs.streg;
        }

        0x08 => {
            return (*dev).regs.idcfr;
        }

        0x09 => {
            return ((*dev).regs.da >> 8) as u8;
        }

        0x0a => {
            return ((*dev).regs.da & 0xff) as u8;
        }

        0x0b => {
            return (((*dev).regs.vptr >> 12) & 0xff) as u8;
        }

        0x0c => {
            return (((*dev).regs.vptr >> 4) & 0xff) as u8;
        }

        0x0d => {
            return (((*dev).regs.vptr & 0x0f) << 4) as u8;
        }

        0x0e | 0x0f => {
            if ((*dev).regs.ctrl & 0x80) == 0 {
                return 0xff;
            }

            threec503_set_drq(dev);
            let ret = dp8390_chipmem_read((*dev).dp8390, (*dev).regs.da as u32, 1);
            (*dev).regs.da += 1;
            return ret as u8;
        }

        _ => {}
    }

    return 0;
}

unsafe extern "C" fn threec503_nic_hi_write(addr: u16, val: u8, r#priv: *mut c_void) {
    let dev: *mut threec503_t = r#priv as _;

    trace!("3Com503: Write GA address={:x}, val={:x}", addr, val);

    match addr & 0x0f {
        0x00 => {
            (*dev).regs.pstr = val;
        }

        0x01 => {
            (*dev).regs.pspr = val;
        }

        0x02 => {
            (*dev).regs.dqtr = val;
        }

        0x05 => {
            if ((*dev).regs.gacfr & 0x0f) != (val & 0x0f) {
                match val & 0x0f {
                    0 => {
                        /*ROM mapping*/
                        /* FIXME: Implement this when a BIOS is found/generated. */
                        mem_mapping_disable(&mut (*dev).ram_mapping);
                    }

                    9 => {
                        /*RAM mapping*/
                        mem_mapping_enable(&mut (*dev).ram_mapping);
                    }

                    _ => {
                        /*No ROM mapping*/
                        mem_mapping_disable(&mut (*dev).ram_mapping);
                    }
                }
            }

            if (val & 0x80) == 0 {
                threec503_interrupt(dev as _, 1);
            } else {
                threec503_interrupt(dev as _, 0);
            }

            (*dev).regs.gacfr = val;
        }

        0x06 => {
            if (val & 1) != 0 {
                threec503_reset(dev as _);
                (*(*dev).dp8390).ISR.reset = 1;
                (*dev).regs.ctrl = 0x0b;
                return;
            }

            if (val & 0x80) != ((*dev).regs.ctrl & 0x80) {
                if (val & 0x80) != 0 {
                    (*dev).regs.streg |= 0x88;
                } else {
                    (*dev).regs.streg &= !0x88;
                }
                (*dev).regs.streg &= !0x10;
            }
            (*dev).regs.ctrl = val;
        }

        0x08 => {
            match val & 0xf0 {
                0x00 | 0x10 | 0x20 | 0x40 | 0x80 => {
                    (*dev).regs.idcfr = ((*dev).regs.idcfr & 0x0f) | (val & 0xf0);
                }

                _ => {
                    warn!("Trying to set multiple IRQs: {:x}", val);
                }
            }

            match val & 0x0f {
                0x00 | 0x01 | 0x02 | 0x04 => {
                    (*dev).regs.idcfr = ((*dev).regs.idcfr & 0xf0) | (val & 0x0f);
                }

                0x08 => {}

                _ => {
                    warn!("Trying to set multiple DMA channels: {:x}", val);
                }
            }
        }

        0x09 => {
            (*dev).regs.da = ((val as u16) << 8) | ((*dev).regs.da & 0xff);
        }

        0x0a => {
            (*dev).regs.da = ((*dev).regs.da & 0xff00) | val as u16;
        }

        0x0b => {
            (*dev).regs.vptr = ((val as u32) << 12) | ((*dev).regs.vptr & 0xfff);
        }

        0x0c => {
            (*dev).regs.vptr = ((val as u32) << 4) | ((*dev).regs.vptr & 0xff00f);
        }

        0x0d => {
            (*dev).regs.vptr = ((val as u32) << 4) | ((*dev).regs.vptr & 0xffff0);
        }

        0x0e | 0x0f => {
            if ((*dev).regs.ctrl & 0x80) == 0 {
                return;
            }

            threec503_set_drq(dev);

            dp8390_chipmem_write((*dev).dp8390, (*dev).regs.da as u32, val as u32, 1);
            (*dev).regs.da += 1;
        }

        _ => {}
    }
}

unsafe extern "C" fn threec503_nic_ioset(dev: *mut threec503_t, addr: u16) {
    io_sethandler(
        addr,
        0x10,
        Some(threec503_nic_lo_read),
        None,
        None,
        Some(threec503_nic_lo_write),
        None,
        None,
        dev as _,
    );

    io_sethandler(
        addr + 0x400,
        0x10,
        Some(threec503_nic_hi_read),
        None,
        None,
        Some(threec503_nic_hi_write),
        None,
        None,
        dev as _,
    );
}

unsafe extern "C" fn threec503_nic_init(_info: *const device_t) -> *mut c_void {
    let mut mac: u32;
    let dev: *mut threec503_t = malloc(size_of::<threec503_t>()) as _;
    memset(dev as _, 0x00, size_of::<threec503_t>());
    (*dev).maclocal[0] = 0x02; /* 02:60:8C (3Com OID) */
    (*dev).maclocal[1] = 0x60;
    (*dev).maclocal[2] = 0x8C;

    (*dev).base_address = device_get_config_hex16(b"base\0".as_ptr() as _) as u32;
    (*dev).base_irq = device_get_config_int(b"irq\0".as_ptr() as _);
    (*dev).dma_channel = device_get_config_int(b"dma\0".as_ptr() as _);
    (*dev).bios_addr = device_get_config_hex20(b"bios_addr\0".as_ptr() as _) as u32;

    /* See if we have a local MAC address configured. */
    mac = device_get_config_mac(b"mac\0".as_ptr() as _, -1) as u32;

    /*
     * Make this device known to the I/O system.
     * PnP and PCI devices start with address spaces inactive.
     */
    threec503_nic_ioset(dev, (*dev).base_address as u16);

    /* Set up our BIA. */
    if (mac & 0xff000000) != 0 {
        /* Generate new local MAC. */
        (*dev).maclocal[3] = random_generate();
        (*dev).maclocal[4] = random_generate();
        (*dev).maclocal[5] = random_generate();
        mac = ((*dev).maclocal[3] as u32) << 16;
        mac |= ((*dev).maclocal[4] as u32) << 8;
        mac |= (*dev).maclocal[5] as u32;
        device_set_config_mac(b"mac\0".as_ptr() as _, mac as i32);
    } else {
        (*dev).maclocal[3] = ((mac >> 16) & 0xff) as u8;
        (*dev).maclocal[4] = ((mac >> 8) & 0xff) as u8;
        (*dev).maclocal[5] = (mac & 0xff) as u8;
    }

    (*dev).dp8390 = device_add(&dp8390_device) as _;
    (*(*dev).dp8390).r#priv = dev as _;
    (*(*dev).dp8390).interrupt = Some(threec503_interrupt);
    dp8390_set_defaults((*dev).dp8390, DP8390_FLAG_CHECK_CR | DP8390_FLAG_CLEAR_IRQ);
    dp8390_mem_alloc((*dev).dp8390, 0x2000, 0x2000);

    (*(*dev).dp8390).physaddr.copy_from_slice(&(*dev).maclocal);

    trace!(
        "I/O={:x}, IRQ={}, MAC={:x}:{:x}:{:x}:{:x}:{:x}:{:x}",
        (*dev).base_address,
        (*dev).base_irq,
        (*(*dev).dp8390).physaddr[0],
        (*(*dev).dp8390).physaddr[1],
        (*(*dev).dp8390).physaddr[2],
        (*(*dev).dp8390).physaddr[3],
        (*(*dev).dp8390).physaddr[4],
        (*(*dev).dp8390).physaddr[5]
    );

    /* Reset the board. */
    threec503_reset(dev as _);

    /* Map this system into the memory map. */
    mem_mapping_add(
        &mut (*dev).ram_mapping,
        (*dev).bios_addr,
        0x4000,
        Some(threec503_ram_read),
        None,
        None,
        Some(threec503_ram_write),
        None,
        None,
        ptr::null_mut(),
        MEM_MAPPING_EXTERNAL,
        dev as _,
    );
    // mem_mapping_disable(&(*dev).ram_mapping);
    (*dev).regs.gacfr = 0x09; /* Start with RAM mapping enabled. */

    /* Attach ourselves to the network module. */
    network_attach(
        (*dev).dp8390 as _,
        (*(*dev).dp8390).physaddr.as_mut_ptr(),
        Some(dp8390_rx),
        None,
        None,
    );

    return dev as _;
}

unsafe extern "C" fn threec503_nic_close(r#priv: *mut c_void) {
    let dev: *mut threec503_t = r#priv as _;

    trace!("3Com503: closed");

    free(dev as _);
}

#[no_mangle]
pub static threec503_config: [device_config_t; 6] = [
    device_config_t {
        name: b"base\0".as_ptr() as _,
        description: b"Address\0".as_ptr() as _,
        r#type: CONFIG_HEX16,
        default_string: b"\0".as_ptr() as _,
        default_int: 0x300,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"0x250\0".as_ptr() as _,
                value: 0x250,
            },
            device_config_selection_t {
                description: b"0x280\0".as_ptr() as _,
                value: 0x280,
            },
            device_config_selection_t {
                description: b"0x2a0\0".as_ptr() as _,
                value: 0x2a0,
            },
            device_config_selection_t {
                description: b"0x2e0\0".as_ptr() as _,
                value: 0x2e0,
            },
            device_config_selection_t {
                description: b"0x300\0".as_ptr() as _,
                value: 0x300,
            },
            device_config_selection_t {
                description: b"0x310\0".as_ptr() as _,
                value: 0x310,
            },
            device_config_selection_t {
                description: b"0x330\0".as_ptr() as _,
                value: 0x330,
            },
            device_config_selection_t {
                description: b"0x350\0".as_ptr() as _,
                value: 0x350,
            },
            device_config_selection_t {
                description: b"\0".as_ptr() as _,
                value: 0,
            },
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
        ],
    },
    device_config_t {
        name: b"irq\0".as_ptr() as _,
        description: b"IRQ\0".as_ptr() as _,
        r#type: CONFIG_SELECTION,
        default_string: b"\0".as_ptr() as _,
        default_int: 3,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"IRQ 2\0".as_ptr() as _,
                value: 2,
            },
            device_config_selection_t {
                description: b"IRQ 3\0".as_ptr() as _,
                value: 3,
            },
            device_config_selection_t {
                description: b"IRQ 4\0".as_ptr() as _,
                value: 4,
            },
            device_config_selection_t {
                description: b"IRQ 5\0".as_ptr() as _,
                value: 5,
            },
            device_config_selection_t {
                description: b"\0".as_ptr() as _,
                value: 0,
            },
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
        ],
    },
    device_config_t {
        name: b"dma\0".as_ptr() as _,
        description: b"DMA\0".as_ptr() as _,
        r#type: CONFIG_SELECTION,
        default_string: b"\0".as_ptr() as _,
        default_int: 3,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"DMA 1\0".as_ptr() as _,
                value: 1,
            },
            device_config_selection_t {
                description: b"DMA 2\0".as_ptr() as _,
                value: 2,
            },
            device_config_selection_t {
                description: b"DMA 3\0".as_ptr() as _,
                value: 3,
            },
            device_config_selection_t {
                description: b"\0".as_ptr() as _,
                value: 0,
            },
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
        ],
    },
    device_config_t {
        name: b"mac\0".as_ptr() as _,
        description: b"MAC Address\0".as_ptr() as _,
        r#type: CONFIG_MAC,
        default_string: b"\0".as_ptr() as _,
        default_int: -1,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"\0".as_ptr() as _,
                value: 0,
            },
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
        ],
    },
    device_config_t {
        name: b"bios_addr\0".as_ptr() as _,
        description: b"BIOS address\0".as_ptr() as _,
        r#type: CONFIG_HEX20,
        default_string: b"\0".as_ptr() as _,
        default_int: 0xCC000,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"DC00\0".as_ptr() as _,
                value: 0xDC000,
            },
            device_config_selection_t {
                description: b"D800\0".as_ptr() as _,
                value: 0xD8000,
            },
            device_config_selection_t {
                description: b"C800\0".as_ptr() as _,
                value: 0xC8000,
            },
            device_config_selection_t {
                description: b"CC00\0".as_ptr() as _,
                value: 0xCC000,
            },
            device_config_selection_t {
                description: b"\0".as_ptr() as _,
                value: 0,
            },
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
            device_config_selection_t::empty(),
        ],
    },
    device_config_t {
        name: b"\0".as_ptr() as _,
        description: b"\0".as_ptr() as _,
        r#type: CONFIG_END,
        ..device_config_t::empty()
    },
];

#[no_mangle]
pub static threec503_device: device_t = device_t {
    name: b"3Com EtherLink II\0".as_ptr() as _,
    internal_name: b"3c503\0".as_ptr() as _,
    flags: DEVICE_ISA as u32,
    local: 0,
    init: Some(threec503_nic_init),
    close: Some(threec503_nic_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: threec503_config.as_ptr(),
};
