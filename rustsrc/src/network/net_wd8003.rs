/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Implementation of the following network controllers:
 *			- SMC/WD 8003E (ISA 8-bit);
 *			- SMC/WD 8013EBT (ISA 16-bit);
 *			- SMC/WD 8013EP/A (MCA).
 *
 *
 *
 * Authors:	Fred N. van Kempen, <decwiz@yahoo.com>
 *		TheCollector1995, <mariogplayer@gmail.com>
 *		Miran Grca, <mgrca8@gmail.com>
 *		Peter Grehan, <grehan@iprg.nokia.com>
 *
 *		Copyright 2017,2018 Fred N. van Kempen.
 *		Copyright 2016-2018 Miran Grca.
 *		Portions Copyright (C) 2002  MandrakeSoft S.A.
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

const WD8003E: c_int = 1; /* WD8003E   :  8-bit ISA, no  interface chip */
const WD8003EB: c_int = 2; /* WD8003EB  :  8-bit ISA, 5x3 interface chip */
const WD8013EBT: c_int = 3; /* WD8013EBT : 16-bit ISA, no  interface chip */
const WD8003ETA: c_int = 4; /* WD8003ET/A: 16-bit MCA, no  interface chip */
const WD8003EA: c_int = 5; /* WD8003E/A : 16-bit MCA, 5x3 interface chip */

/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Implementation of the following network controllers:
 *			- SMC/WD 8003E (ISA 8-bit);
 *			- SMC/WD 8013EBT (ISA 16-bit);
 *			- SMC/WD 8013EP/A (MCA).
 *
 *
 *
 * Authors:	Fred N. van Kempen, <decwiz@yahoo.com>
 *		TheCollector1995, <mariogplayer@gmail.com>
 *		Miran Grca, <mgrca8@gmail.com>
 *		Peter Grehan, <grehan@iprg.nokia.com>
 *
 *		Copyright 2017-2019 Fred N. van Kempen.
 *		Copyright 2016-2019 Miran Grca.
 *		Portions Copyright (C) 2002  MandrakeSoft S.A.
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

use std::{ffi::CStr, num::Wrapping, ptr};

use super::net_dp8390::*;
use crate::{mem::*, prelude::*, random::random_generate};
use const_zero::const_zero;
use log::*;

/* Board r#type codes in card ID */
const _WE_TYPE_WD8003: u8 = 0x01;
const _WE_TYPE_WD8003S: u8 = 0x02;
const WE_TYPE_WD8003E: u8 = 0x03;
const WE_TYPE_WD8013EBT: u8 = 0x05;
const _WE_TYPE_TOSHIBA1: u8 = 0x11; /* named PCETA1 */
const _WE_TYPE_TOSHIBA2: u8 = 0x12; /* named PCETA2 */
const _WE_TYPE_TOSHIBA3: u8 = 0x13; /* named PCETB */
const _WE_TYPE_TOSHIBA4: u8 = 0x14; /* named PCETC */
const _WE_TYPE_WD8003W: u8 = 0x24;
const WE_TYPE_WD8003EB: u8 = 0x25;
const _WE_TYPE_WD8013W: u8 = 0x26;
const _WE_TYPE_WD8013EP: u8 = 0x27;
const _WE_TYPE_WD8013WC: u8 = 0x28;
const _WE_TYPE_WD8013EPC: u8 = 0x29;
const _WE_TYPE_SMC8216T: u8 = 0x2a;
const _WE_TYPE_SMC8216C: u8 = 0x2b;
const _WE_TYPE_WD8013EBP: u8 = 0x2c;

const WE_ICR_16BIT_SLOT: u8 = 0x01;

const WE_MSR_ENABLE_RAM: u8 = 0x40;
const WE_MSR_SOFT_RESET: u8 = 0x80;

const WE_IRR_ENABLE_IRQ: u8 = 0x80;

const _WE_ID_ETHERNET: u8 = 0x01;
const WE_ID_SOFT_CONFIG: u8 = 0x20;
const WE_ID_EXTRA_RAM: u8 = 0x40;
const WE_ID_BUS_MCA: u8 = 0x80;

struct NetCard {
    dp8390: *mut dp8390_t,
    ram_mapping: mem_mapping_t,
    ram_addr: u32,
    ram_size: u32,
    maclocal: [u8; 6], /* configured MAC (local) address */
    bit16: u8,
    board: c_int,
    name: *const c_char,
    base_address: u32,
    irq: c_int,

    /* POS registers, MCA boards only */
    pos_regs: [u8; 8],

    /* Memory for WD cards*/
    msr: u8,  /* Memory Select Register (MSR) */
    icr: u8,  /* Interface Configuration Register (ICR) */
    irr: u8,  /* Interrupt Request Register (IRR) */
    laar: u8, /* LA Address Register (read by Windows 98!) */
    if_chip: u8,
    board_chip: u8,
}

static we_int_table: [c_int; 4] = [2, 3, 4, 7];

unsafe extern "C" fn wd_interrupt(r#priv: *mut c_void, set: c_int) {
    let dev: *mut NetCard = r#priv as _;

    if ((*dev).irr & WE_IRR_ENABLE_IRQ) == 0 {
        return;
    }

    if set != 0 {
        picint(1 << (*dev).irq);
    } else {
        picintc(1 << (*dev).irq);
    }
}

/* reset - restore state to power-up, cancelling all i/o */
unsafe extern "C" fn wd_reset(r#priv: *mut c_void) {
    let dev: *mut NetCard = r#priv as _;

    trace!("{:?}: reset", CStr::from_ptr((*dev).name));

    dp8390_reset((*dev).dp8390);
}

unsafe extern "C" fn wd_soft_reset(r#priv: *mut c_void) {
    let dev: *mut NetCard = r#priv as _;

    dp8390_soft_reset((*dev).dp8390);
}

unsafe extern "C" fn wd_ram_read(addr: u32, r#priv: *mut c_void) -> u8 {
    let dev: *mut NetCard = r#priv as _;

    trace!(
        "WD80x3: RAM Read: addr={:06x}, val={:02x}",
        addr & ((*dev).ram_size - 1),
        *(*(*dev).dp8390)
            .mem
            .offset((addr & ((*dev).ram_size - 1)) as isize)
    );
    return *(*(*dev).dp8390)
        .mem
        .offset((addr & ((*dev).ram_size - 1)) as isize);
}

unsafe extern "C" fn wd_ram_write(addr: u32, val: u8, r#priv: *mut c_void) {
    let dev: *mut NetCard = r#priv as _;

    *(*(*dev).dp8390)
        .mem
        .offset((addr & ((*dev).ram_size - 1)) as isize) = val;
    trace!(
        "WD80x3: RAM Write: addr={:06x}, val={:02x}",
        addr & ((*dev).ram_size - 1),
        val
    );
}

unsafe extern "C" fn wd_get_irq_index(dev: *mut NetCard) -> c_int {
    let mut irq = 255;

    for i in 0..4 {
        if we_int_table[i] == (*dev).irq {
            irq = i as c_int;
        }
    }

    if irq != 255 {
        return (irq & 0x03) << 5;
    } else {
        return 0;
    }
}

unsafe extern "C" fn wd_smc_read(dev: *mut NetCard, mut off: u32) -> u32 {
    let mut retval: u32 = 0;
    let mut checksum: Wrapping<u8> = Wrapping(0);

    if (*dev).board == WD8003E {
        off |= 0x08;
    }

    match off {
        0x00 => {
            if ((*dev).board_chip & WE_ID_BUS_MCA) != 0 {
                retval = ((*dev).msr as u32 & 0xc0) | (((*dev).ram_addr >> 13) & 0x3f);
            } else {
                retval = (*dev).msr as u32;
            }
        }

        0x01 => {
            if ((*dev).board_chip & WE_ID_SOFT_CONFIG) != 0 {
                retval = (*dev).icr as u32;
            } else {
                retval = ((*dev).icr & WE_ICR_16BIT_SLOT) as u32;
            }
        }

        0x04 => {
            if ((*dev).board_chip & WE_ID_SOFT_CONFIG) != 0 {
                retval = ((*dev).irr as u32 & 0x9f) | wd_get_irq_index(dev) as u32;
            }
        }

        0x05 => {
            if ((*dev).board_chip & WE_ID_SOFT_CONFIG) != 0 {
                retval = (*dev).laar as u32;
            }
        }

        0x07 => {
            if ((*dev).board_chip & WE_ID_SOFT_CONFIG) != 0 {
                retval = (*dev).if_chip as u32;
            }
        }

        0x08 => {
            retval = (*(*dev).dp8390).physaddr[0] as u32;
        }

        0x09 => {
            retval = (*(*dev).dp8390).physaddr[1] as u32;
        }

        0x0a => {
            retval = (*(*dev).dp8390).physaddr[2] as u32;
        }

        0x0b => {
            retval = (*(*dev).dp8390).physaddr[3] as u32;
        }

        0x0c => {
            retval = (*(*dev).dp8390).physaddr[4] as u32;
        }

        0x0d => {
            retval = (*(*dev).dp8390).physaddr[5] as u32;
        }

        0x0e => {
            retval = (*dev).board_chip as u32;
        }

        0x0f => {
            /*This has to return the byte that adds up to 0xFF*/
            for byte in (*(*dev).dp8390).physaddr {
                checksum += byte;
            }
            checksum += (*dev).board_chip;

            retval = 0xff - (checksum.0 as u32);
        }

        _ => {}
    }

    trace!(
        "{:?}: ASIC read addr=0x{:02x}, value=0x{:04x}",
        CStr::from_ptr((*dev).name),
        off,
        retval
    );

    return retval;
}

unsafe extern "C" fn wd_set_ram(dev: *mut NetCard) {
    let mut a13: u32;

    if ((*dev).board_chip & 0xa0) == 0x20 {
        a13 = (*dev).msr as u32 & 0x3f;
        a13 <<= 13;

        (*dev).ram_addr = a13 | (1 << 19);
        mem_mapping_set_addr(&mut (*dev).ram_mapping, (*dev).ram_addr, (*dev).ram_size);
        trace!(
            "{:?}: RAM address set to {:08X}",
            CStr::from_ptr((*dev).name),
            (*dev).ram_addr
        );
    }

    if ((*dev).msr & WE_MSR_ENABLE_RAM) != 0 {
        mem_mapping_enable(&mut (*dev).ram_mapping);
        trace!("{:?}: RAM now enabled", CStr::from_ptr((*dev).name));
    } else {
        mem_mapping_disable(&mut (*dev).ram_mapping);
        trace!("{:?}: RAM now disabled", CStr::from_ptr((*dev).name));
    }
}

unsafe extern "C" fn wd_smc_write(dev: *mut NetCard, off: u32, val: u32) {
    let old: u8;

    trace!(
        "{:?}: ASIC write addr=0x{:02x}, value=0x{:04x}",
        CStr::from_ptr((*dev).name),
        off,
        val
    );

    if off != 0 && ((*dev).board == WD8003E) {
        return;
    }

    match off {
        /* Bits 0-5: Bits 13-18 of memory address (writable?):
              Windows 98 requires this to be preloaded with the initial
              addresss to work correctly;
        Bit 6: Enable memory if set;
        Bit 7: Software reset if set. */
        0x00 => {
            /* WD Control register */
            old = (*dev).msr;

            if (old & WE_MSR_SOFT_RESET) == 0 && (val & WE_MSR_SOFT_RESET as u32) != 0 {
                wd_soft_reset(dev as _);
                trace!("WD80x3: Soft reset");
            }

            if ((*dev).board_chip & 0xa0) == 0x20 {
                (*dev).msr = val as u8;
            } else {
                (*dev).msr = ((*dev).msr & 0x3f) | (val as u8 & 0xc0);
            }

            if (old & 0x7f) != ((val as u8) & 0x7f) {
                wd_set_ram(dev);
                trace!(
                    "WD80x3: Memory now {}abled (addr = {:08X})",
                    if (val as u8 & WE_MSR_ENABLE_RAM) != 0 {
                        "en"
                    } else {
                        "dis"
                    },
                    (*dev).ram_addr
                );
            }
        }

        /* Bit 1: 0 = 8-bit slot, 1 = 16-bit slot;
        Bit 3: 0 = 8k RAM, 1 = 32k RAM (only on revision < 2). */
        0x01 => {
            if ((*dev).bit16 & 2) != 0 {
                (*dev).icr = ((*dev).icr & WE_ICR_16BIT_SLOT) | (val as u8 & WE_ICR_16BIT_SLOT);
            } else {
                (*dev).icr = val as u8;
            }
        }

        /* Bit 5: Bit 0 of encoded IRQ;
        Bit 6: Bit 1 of encoded IRQ;
        Bit 7: Enable interrupts. */
        0x04 => {
            if ((*dev).board_chip & WE_ID_SOFT_CONFIG) != 0 {
                (*dev).irr = ((*dev).irr & 0xe0) | (val as u8 & 0x1f);
            }
        }

        /* Bits 0-4: Bits 19-23 of memory address (writable?):
              Windows 98 requires this to be preloaded with the initial
              addresss to work correctly;
        Bit 5: Enable software interrupt;
        Bit 6: Enable 16-bit RAM for LAN if set;
        Bit 7: Enable 16-bit RAM for host if set. */
        0x05 => {
            if ((*dev).board_chip & WE_ID_SOFT_CONFIG) != 0 {
                (*dev).laar = val as u8;
            }
        }

        /* Bits 0-4: Chip ID;
        Bit 5: Software configuration is supported if present;
        Bit 6: 0 = 16k RAM, 1 = 32k RAM. */
        0x07 => {
            if ((*dev).board_chip & WE_ID_SOFT_CONFIG) != 0 {
                (*dev).if_chip = val as u8;
            }
        }

        _ => {
            /* This is invalid, but happens under win95 device detection:
            maybe some clone cards implement writing for some other
            registers? */
            debug!(
                "{:?}: ASIC write invalid address {:04x}, ignoring",
                CStr::from_ptr((*dev).name),
                off
            );
        }
    }
}

unsafe extern "C" fn wd_read(addr: u16, dev: *mut NetCard, len: c_int) -> u8 {
    let mut retval: u8 = 0;
    let off: u32 = addr as u32 - (*dev).base_address;

    trace!("{:?}: read addr {:04x}", CStr::from_ptr((*dev).name), addr);

    if off == 0x10 {
        retval = dp8390_read_cr((*dev).dp8390) as u8;
    } else if off <= 0x0f {
        retval = wd_smc_read(dev, off) as u8;
    } else {
        match (*(*dev).dp8390).CR.pgsel {
            0x00 => {
                retval = dp8390_page0_read((*dev).dp8390, off - 0x10, len as u32) as u8;
            }
            0x01 => {
                retval = dp8390_page1_read((*dev).dp8390, off - 0x10, len as u32) as u8;
            }
            0x02 => {
                retval = dp8390_page2_read((*dev).dp8390, off - 0x10, len as u32) as u8;
            }
            _ => {
                debug!(
                    "{:?}: unknown value of pgsel in read - {}",
                    CStr::from_ptr((*dev).name),
                    (*(*dev).dp8390).CR.pgsel
                );
            }
        }
    }

    return retval;
}

unsafe extern "C" fn wd_readb(addr: u16, r#priv: *mut c_void) -> u8 {
    let dev: *mut NetCard = r#priv as _;

    return wd_read(addr, dev, 1);
}

unsafe extern "C" fn wd_readw(addr: u16, r#priv: *mut c_void) -> u16 {
    let dev: *mut NetCard = r#priv as _;

    return (wd_read(addr, dev, 2)) as u16;
}

unsafe extern "C" fn wd_write(addr: u16, val: u8, r#priv: *mut c_void, len: c_uint) {
    let dev: *mut NetCard = r#priv as _;
    let off: u32 = addr as u32 - (*dev).base_address;

    trace!(
        "{:?}: write addr {:04x}, value {:02x}",
        CStr::from_ptr((*dev).name),
        addr,
        val
    );

    if off == 0x10 {
        dp8390_write_cr((*dev).dp8390, val as u32);
    } else if off <= 0x0f {
        wd_smc_write(dev, off, val as u32);
    } else {
        match (*(*dev).dp8390).CR.pgsel {
            0x00 => {
                dp8390_page0_write((*dev).dp8390, off - 0x10, val as u32, len);
            }
            0x01 => {
                dp8390_page1_write((*dev).dp8390, off - 0x10, val as u32, len);
            }
            _ => {
                debug!(
                    "{:?}: unknown value of pgsel in write - {}\n",
                    CStr::from_ptr((*dev).name),
                    (*(*dev).dp8390).CR.pgsel
                );
            }
        }
    }
}

unsafe extern "C" fn wd_writeb(addr: u16, val: u8, r#priv: *mut c_void) {
    wd_write(addr, val, r#priv, 1);
}

unsafe extern "C" fn wd_writew(addr: u16, val: u16, r#priv: *mut c_void) {
    wd_write(addr, (val & 0xFF) as u8, r#priv, 2);
}

unsafe extern "C" fn wd_io_set(dev: *mut NetCard, addr: u16) {
    if ((*dev).bit16 & 1) != 0 {
        io_sethandler(
            addr,
            0x20,
            Some(wd_readb),
            Some(wd_readw),
            None,
            Some(wd_writeb),
            Some(wd_writew),
            None,
            dev as _,
        );
    } else {
        io_sethandler(
            addr,
            0x20,
            Some(wd_readb),
            None,
            None,
            Some(wd_writeb),
            None,
            None,
            dev as _,
        );
    }
}

unsafe extern "C" fn wd_io_remove(dev: *mut NetCard, addr: u16) {
    if ((*dev).bit16 & 1) != 0 {
        io_removehandler(
            addr,
            0x20,
            Some(wd_readb),
            Some(wd_readw),
            None,
            Some(wd_writeb),
            Some(wd_writew),
            None,
            dev as _,
        );
    } else {
        io_removehandler(
            addr,
            0x20,
            Some(wd_readb),
            None,
            None,
            Some(wd_writeb),
            None,
            None,
            dev as _,
        );
    }
}

unsafe extern "C" fn wd_mca_read(port: c_int, r#priv: *mut c_void) -> u8 {
    let dev: *mut NetCard = r#priv as _;

    return (*dev).pos_regs[port as usize & 7];
}

const MCA_6FC0_IRQS: [i8; 4] = [3, 4, 10, 15];

unsafe extern "C" fn wd_mca_write(port: c_int, val: u8, r#priv: *mut c_void) {
    let dev: *mut NetCard = r#priv as _;
    let irq: [i8; 4] = MCA_6FC0_IRQS;

    /* MCA does not write registers below 0x0100. */
    if port < 0x0102 {
        return;
    }

    /* Save the MCA register value. */
    (*dev).pos_regs[port as usize & 7] = val;

    /*
     * The PS/2 Model 80 BIOS always enables a card if it finds one,
     * even if no resources were assigned yet (because we only added
     * the card, but have not run AutoConfig yet...)
     *
     * So, remove current address, if any.
     */
    if ((*dev).base_address) != 0 {
        wd_io_remove(dev, (*dev).base_address as u16);
    }

    (*dev).base_address = ((*dev).pos_regs[2] as u32 & 0xfe) << 4;
    (*dev).ram_addr = ((*dev).pos_regs[3] as u32 & 0xfc) << 12;
    (*dev).irq = irq[(*dev).pos_regs[5] as usize & 0x02] as i32;

    /* Initialize the device if fully configured. */
    /* Register (new) I/O handler. */
    if ((*dev).pos_regs[2] & 0x01) != 0 {
        wd_io_set(dev, (*dev).base_address as u16);
    }

    mem_mapping_set_addr(&mut (*dev).ram_mapping, (*dev).ram_addr, (*dev).ram_size);

    mem_mapping_disable(&mut (*dev).ram_mapping);
    if ((*dev).msr & WE_MSR_ENABLE_RAM) != 0 && ((*dev).pos_regs[2] & 0x01) != 0 {
        mem_mapping_enable(&mut (*dev).ram_mapping);
    }

    trace!(
        "{:?}: attached IO=0x{:08X} IRQ={}, RAM addr=0x{:06x}",
        CStr::from_ptr((*dev).name),
        (*dev).base_address,
        (*dev).irq,
        (*dev).ram_addr
    );
}

unsafe extern "C" fn wd_mca_feedb(_priv: *mut c_void) -> u8 {
    return 1;
}

unsafe extern "C" fn wd_init(info: *const device_t) -> *mut c_void {
    let mut mac: u32;
    let dev: *mut NetCard = Box::into_raw(Box::new(const_zero!(NetCard)));
    (*dev).name = (*info).name;
    (*dev).board = (*info).local as i32;

    (*dev).maclocal[0] = 0x00; /* 00:00:C0 (WD/SMC OID) */
    (*dev).maclocal[1] = 0x00;
    (*dev).maclocal[2] = 0xC0;

    /* See if we have a local MAC address configured. */
    mac = device_get_config_mac(b"mac\0".as_ptr() as _, -1) as u32;

    /* Set up our BIA. */
    if (mac & 0xff000000) != 0 {
        /* Generate new local MAC. */
        (*dev).maclocal[3] = random_generate();
        (*dev).maclocal[4] = random_generate();
        (*dev).maclocal[5] = random_generate();
        mac = ((*dev).maclocal[3] as u32) << 16;
        mac |= ((*dev).maclocal[4] as u32) << 8;
        mac |= (*dev).maclocal[5] as u32;
        device_set_config_mac(b"mac\0".as_ptr() as _, mac as c_int);
    } else {
        (*dev).maclocal[3] = ((mac >> 16) & 0xff) as u8;
        (*dev).maclocal[4] = ((mac >> 8) & 0xff) as u8;
        (*dev).maclocal[5] = (mac & 0xff) as u8;
    }

    if ((*dev).board == WD8003ETA) || ((*dev).board == WD8003EA) {
        mca_add(
            Some(wd_mca_read),
            Some(wd_mca_write),
            Some(wd_mca_feedb),
            None,
            dev as _,
        );
    } else {
        (*dev).base_address = device_get_config_hex16(b"base\0".as_ptr() as _) as u32;
        (*dev).irq = device_get_config_int(b"irq\0".as_ptr() as _);
        (*dev).ram_addr = device_get_config_hex20(b"ram_addr\0".as_ptr() as _) as u32;
    }

    (*dev).dp8390 = device_add(&dp8390_device) as _;
    (*(*dev).dp8390).r#priv = dev as _;
    (*(*dev).dp8390).interrupt = Some(wd_interrupt);
    dp8390_set_defaults((*dev).dp8390, DP8390_FLAG_CHECK_CR | DP8390_FLAG_CLEAR_IRQ);

    match (*dev).board {
        /* Ethernet, ISA, no interface chip, RAM 8k */
        WD8003E => {
            (*dev).board_chip = WE_TYPE_WD8003E;
            (*dev).ram_size = 0x2000;
        }

        /* Ethernet, ISA, 5x3 interface chip, RAM 8k or 32k */
        WD8003EB => {
            (*dev).board_chip = WE_TYPE_WD8003EB;
            (*dev).if_chip = 1;
            (*dev).ram_size = device_get_config_int(b"ram_size\0".as_ptr() as _) as u32;
            if (*dev).ram_size == 0x8000 {
                (*dev).board_chip |= WE_ID_EXTRA_RAM;
            }

            /* Bit A19 is implicit 1. */
            (*dev).msr |= (((*dev).ram_addr >> 13) & 0x3f) as u8;
        }

        /* Ethernet, ISA, no interface chip, RAM 8k or 32k (8-bit slot) / 16k or 64k (16-bit slot) */
        WD8013EBT => {
            (*dev).board_chip = WE_TYPE_WD8013EBT;
            (*dev).ram_size = device_get_config_int(b"ram_size\0".as_ptr() as _) as u32;
            if (*dev).ram_size == 0x10000 {
                (*dev).board_chip |= WE_ID_EXTRA_RAM;
            }

            (*dev).bit16 = 2;
            if (is286) != 0 {
                (*dev).bit16 |= 1;
            } else {
                (*dev).bit16 |= 0;
                if (*dev).irq == 9 {
                    (*dev).irq = 2;
                }
                (*dev).ram_size >>= 1; /* Half the RAM when in 8-bit slot. */
            }
        }

        /* Ethernet, MCA, 5x3 interface chip, RAM 16k */
        WD8003EA => {
            (*dev).board_chip = WE_ID_SOFT_CONFIG;
        }
        /* Ethernet, MCA, no interface chip, RAM 16k */
        WD8003ETA => {
            (*dev).board_chip |= 0x05 | WE_ID_BUS_MCA;
            (*dev).ram_size = 0x4000;
            (*dev).pos_regs[0] = 0xC0;
            (*dev).pos_regs[1] = 0x6F;
            (*dev).bit16 = 3;
        }

        _ => {}
    }

    (*dev).irr |= WE_IRR_ENABLE_IRQ;
    (*dev).icr |= (*dev).bit16 & 0x01;

    dp8390_mem_alloc((*dev).dp8390, 0x0000, (*dev).ram_size);

    if ((*dev).base_address) != 0 {
        wd_io_set(dev, (*dev).base_address as u16);
    }

    (*(*dev).dp8390).physaddr.copy_from_slice(&(*dev).maclocal);

    trace!(
        "{:?}: I/O={:04x}, IRQ={}, MAC={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        CStr::from_ptr((*dev).name),
        (*dev).base_address,
        (*dev).irq,
        (*(*dev).dp8390).physaddr[0],
        (*(*dev).dp8390).physaddr[1],
        (*(*dev).dp8390).physaddr[2],
        (*(*dev).dp8390).physaddr[3],
        (*(*dev).dp8390).physaddr[4],
        (*(*dev).dp8390).physaddr[5]
    );

    /* Reset the board. */
    wd_reset(dev as _);

    /* Map this system into the memory map. */
    mem_mapping_add(
        &mut (*dev).ram_mapping,
        (*dev).ram_addr,
        (*dev).ram_size,
        Some(wd_ram_read),
        None,
        None,
        Some(wd_ram_write),
        None,
        None,
        ptr::null_mut(),
        MEM_MAPPING_EXTERNAL,
        dev as _,
    );

    mem_mapping_disable(&mut (*dev).ram_mapping);

    /* Attach ourselves to the network module. */
    network_attach(
        (*dev).dp8390 as _,
        (*(*dev).dp8390).physaddr.as_mut_ptr(),
        Some(dp8390_rx),
        None,
        None,
    );

    if ((*dev).board_chip & WE_ID_BUS_MCA) == 0 {
        trace!(
            "{:?}: attached IO=0x{:X} IRQ={}, RAM addr=0x{:06x}",
            CStr::from_ptr((*dev).name),
            (*dev).base_address,
            (*dev).irq,
            (*dev).ram_addr
        );
    }

    return dev as _;
}

unsafe extern "C" fn wd_close(r#priv: *mut c_void) {
    let dev: *mut NetCard = r#priv as _;

    trace!("{:?}: closed", CStr::from_ptr((*dev).name));

    drop(Box::from_raw(dev));
}

#[no_mangle]
static wd8003_config: [device_config_t; 5] = [
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
                description: b"0x240\0".as_ptr() as _,
                value: 0x240,
            },
            device_config_selection_t {
                description: b"0x280\0".as_ptr() as _,
                value: 0x280,
            },
            device_config_selection_t {
                description: b"0x300\0".as_ptr() as _,
                value: 0x300,
            },
            device_config_selection_t {
                description: b"0x380\0".as_ptr() as _,
                value: 0x380,
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
                description: b"IRQ 5\0".as_ptr() as _,
                value: 5,
            },
            device_config_selection_t {
                description: b"IRQ 7\0".as_ptr() as _,
                value: 7,
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
        name: b"ram_addr\0".as_ptr() as _,
        description: b"RAM address\0".as_ptr() as _,
        r#type: CONFIG_HEX20,
        default_string: b"\0".as_ptr() as _,
        default_int: 0xD0000,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"C800\0".as_ptr() as _,
                value: 0xC8000,
            },
            device_config_selection_t {
                description: b"CC00\0".as_ptr() as _,
                value: 0xCC000,
            },
            device_config_selection_t {
                description: b"D000\0".as_ptr() as _,
                value: 0xD0000,
            },
            device_config_selection_t {
                description: b"D400\0".as_ptr() as _,
                value: 0xD4000,
            },
            device_config_selection_t {
                description: b"D800\0".as_ptr() as _,
                value: 0xD8000,
            },
            device_config_selection_t {
                description: b"DC00\0".as_ptr() as _,
                value: 0xDC000,
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
        ],
    },
    device_config_t {
        name: b"mac\0".as_ptr() as _,
        description: b"MAC Address\0".as_ptr() as _,
        r#type: CONFIG_MAC,
        default_string: b"\0".as_ptr() as _,
        default_int: -1,
        ..device_config_t::empty()
    },
    device_config_t {
        name: b"\0".as_ptr() as _,
        description: b"\0".as_ptr() as _,
        r#type: CONFIG_END,
        ..device_config_t::empty()
    },
];

static wd8003eb_config: [device_config_t; 6] = [
    device_config_t {
        name: b"base\0".as_ptr() as _,
        description: b"Address\0".as_ptr() as _,
        r#type: CONFIG_HEX16,
        default_string: b"\0".as_ptr() as _,
        default_int: 0x280,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"0x200\0".as_ptr() as _,
                value: 0x200,
            },
            device_config_selection_t {
                description: b"0x220\0".as_ptr() as _,
                value: 0x220,
            },
            device_config_selection_t {
                description: b"0x240\0".as_ptr() as _,
                value: 0x240,
            },
            device_config_selection_t {
                description: b"0x260\0".as_ptr() as _,
                value: 0x260,
            },
            device_config_selection_t {
                description: b"0x280\0".as_ptr() as _,
                value: 0x280,
            },
            device_config_selection_t {
                description: b"0x2A0\0".as_ptr() as _,
                value: 0x2A0,
            },
            device_config_selection_t {
                description: b"0x2C0\0".as_ptr() as _,
                value: 0x2C0,
            },
            device_config_selection_t {
                description: b"0x300\0".as_ptr() as _,
                value: 0x300,
            },
            device_config_selection_t {
                description: b"0x340\0".as_ptr() as _,
                value: 0x340,
            },
            device_config_selection_t {
                description: b"0x380\0".as_ptr() as _,
                value: 0x380,
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
                description: b"IRQ 2/9\0".as_ptr() as _,
                value: 9,
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
                description: b"IRQ 7\0".as_ptr() as _,
                value: 7,
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
        name: b"ram_addr\0".as_ptr() as _,
        description: b"RAM address\0".as_ptr() as _,
        r#type: CONFIG_HEX20,
        default_string: b"\0".as_ptr() as _,
        default_int: 0xD0000,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"C000\0".as_ptr() as _,
                value: 0xC0000,
            },
            device_config_selection_t {
                description: b"C400\0".as_ptr() as _,
                value: 0xC4000,
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
                description: b"D000\0".as_ptr() as _,
                value: 0xD0000,
            },
            device_config_selection_t {
                description: b"D400\0".as_ptr() as _,
                value: 0xD4000,
            },
            device_config_selection_t {
                description: b"D800\0".as_ptr() as _,
                value: 0xD8000,
            },
            device_config_selection_t {
                description: b"DC00\0".as_ptr() as _,
                value: 0xDC000,
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
        name: b"ram_size\0".as_ptr() as _,
        description: b"RAM size\0".as_ptr() as _,
        r#type: CONFIG_SELECTION,
        default_string: b"\0".as_ptr() as _,
        default_int: 8192,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"8 kB\0".as_ptr() as _,
                value: 8192,
            },
            device_config_selection_t {
                description: b"32 kB\0".as_ptr() as _,
                value: 32768,
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
            device_config_selection_t::empty(),
        ],
    },
    device_config_t {
        name: b"mac\0".as_ptr() as _,
        description: b"MAC Address\0".as_ptr() as _,
        r#type: CONFIG_MAC,
        default_string: b"\0".as_ptr() as _,
        default_int: -1,
        ..device_config_t::empty()
    },
    device_config_t {
        name: b"\0".as_ptr() as _,
        description: b"\0".as_ptr() as _,
        r#type: CONFIG_END,
        ..device_config_t::empty()
    },
];

/* WD8013EBT configuration and defaults set according to this site:
http://wwwstacknl/~marcolz/network/wd80x3html#WD8013EBT */
static wd8013_config: [device_config_t; 6] = [
    device_config_t {
        name: b"base\0".as_ptr() as _,
        description: b"Address\0".as_ptr() as _,
        r#type: CONFIG_HEX16,
        default_string: b"\0".as_ptr() as _,
        default_int: 0x280,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"0x200\0".as_ptr() as _,
                value: 0x200,
            },
            device_config_selection_t {
                description: b"0x220\0".as_ptr() as _,
                value: 0x220,
            },
            device_config_selection_t {
                description: b"0x240\0".as_ptr() as _,
                value: 0x240,
            },
            device_config_selection_t {
                description: b"0x260\0".as_ptr() as _,
                value: 0x260,
            },
            device_config_selection_t {
                description: b"0x280\0".as_ptr() as _,
                value: 0x280,
            },
            device_config_selection_t {
                description: b"0x2A0\0".as_ptr() as _,
                value: 0x2A0,
            },
            device_config_selection_t {
                description: b"0x2C0\0".as_ptr() as _,
                value: 0x2C0,
            },
            device_config_selection_t {
                description: b"0x300\0".as_ptr() as _,
                value: 0x300,
            },
            device_config_selection_t {
                description: b"0x340\0".as_ptr() as _,
                value: 0x340,
            },
            device_config_selection_t {
                description: b"0x380\0".as_ptr() as _,
                value: 0x380,
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
                description: b"IRQ 2/9\0".as_ptr() as _,
                value: 9,
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
                description: b"IRQ 7\0".as_ptr() as _,
                value: 7,
            },
            device_config_selection_t {
                description: b"IRQ 10\0".as_ptr() as _,
                value: 10,
            },
            device_config_selection_t {
                description: b"IRQ 11\0".as_ptr() as _,
                value: 11,
            },
            device_config_selection_t {
                description: b"IRQ 15\0".as_ptr() as _,
                value: 15,
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
        name: b"ram_addr\0".as_ptr() as _,
        description: b"RAM address\0".as_ptr() as _,
        r#type: CONFIG_HEX20,
        default_string: b"\0".as_ptr() as _,
        default_int: 0xD0000,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"C000\0".as_ptr() as _,
                value: 0xC0000,
            },
            device_config_selection_t {
                description: b"C400\0".as_ptr() as _,
                value: 0xC4000,
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
                description: b"D000\0".as_ptr() as _,
                value: 0xD0000,
            },
            device_config_selection_t {
                description: b"D400\0".as_ptr() as _,
                value: 0xD4000,
            },
            device_config_selection_t {
                description: b"D800\0".as_ptr() as _,
                value: 0xD8000,
            },
            device_config_selection_t {
                description: b"DC00\0".as_ptr() as _,
                value: 0xDC000,
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
        name: b"ram_size\0".as_ptr() as _,
        description: b"RAM size\0".as_ptr() as _,
        r#type: CONFIG_SELECTION,
        default_string: b"\0".as_ptr() as _,
        default_int: 16384,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"16 kB\0".as_ptr() as _,
                value: 16384,
            },
            device_config_selection_t {
                description: b"64 kB\0".as_ptr() as _,
                value: 65536,
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
            device_config_selection_t::empty(),
        ],
    },
    device_config_t {
        name: b"mac\0".as_ptr() as _,
        description: b"MAC Address\0".as_ptr() as _,
        r#type: CONFIG_MAC,
        default_string: b"\0".as_ptr() as _,
        default_int: -1,
        ..device_config_t::empty()
    },
    device_config_t {
        name: b"\0".as_ptr() as _,
        description: b"\0".as_ptr() as _,
        r#type: CONFIG_END,
        ..device_config_t::empty()
    },
];

static mca_mac_config: [device_config_t; 2] = [
    device_config_t {
        name: b"mac\0".as_ptr() as _,
        description: b"MAC Address\0".as_ptr() as _,
        r#type: CONFIG_MAC,
        default_string: b"\0".as_ptr() as _,
        default_int: -1,
        ..device_config_t::empty()
    },
    device_config_t {
        name: b"\0".as_ptr() as _,
        description: b"\0".as_ptr() as _,
        r#type: CONFIG_END,
        ..device_config_t::empty()
    },
];

#[no_mangle]
pub static wd8003e_device: device_t = device_t {
    name: b"Western Digital WD8003E\0".as_ptr() as _,
    internal_name: b"wd8003e\0".as_ptr() as _,
    flags: DEVICE_ISA as u32,
    local: WD8003E as u32,
    init: Some(wd_init),
    close: Some(wd_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: wd8003_config.as_ptr(),
};
#[no_mangle]
pub static wd8003eb_device: device_t = device_t {
    name: b"Western Digital WD8003EB\0".as_ptr() as _,
    internal_name: b"wd8003eb\0".as_ptr() as _,
    flags: DEVICE_ISA as u32,
    local: WD8003EB as u32,
    init: Some(wd_init),
    close: Some(wd_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: wd8003eb_config.as_ptr(),
};
#[no_mangle]
pub static wd8013ebt_device: device_t = device_t {
    name: b"Western Digital WD8013EBT\0".as_ptr() as _,
    internal_name: b"wd8013ebt\0".as_ptr() as _,
    flags: DEVICE_ISA as u32,
    local: WD8013EBT as u32,
    init: Some(wd_init),
    close: Some(wd_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: wd8013_config.as_ptr(),
};
#[no_mangle]
pub static wd8003eta_device: device_t = device_t {
    name: b"Western Digital WD8003ET/A\0".as_ptr() as _,
    internal_name: b"wd8003eta\0".as_ptr() as _,
    flags: DEVICE_MCA as u32,
    local: WD8003ETA as u32,
    init: Some(wd_init),
    close: Some(wd_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: mca_mac_config.as_ptr(),
};
#[no_mangle]
pub static wd8003ea_device: device_t = device_t {
    name: b"Western Digital WD8003E/A\0".as_ptr() as _,
    internal_name: b"wd8003ea\0".as_ptr() as _,
    flags: DEVICE_MCA as u32,
    local: WD8003EA as u32,
    init: Some(wd_init),
    close: Some(wd_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: mca_mac_config.as_ptr(),
};
