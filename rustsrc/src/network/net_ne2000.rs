#![allow(non_camel_case_types)]
/*
 * VARCem	Virtual ARchaeological Computer EMulator.
 *		An emulator of (mostly) x86-based PC systems and devices,
 *		using the ISA,EISA,VLB,MCA  and PCI system buses, roughly
 *		spanning the era between 1981 and 1995.
 *
 *		This file is part of the VARCem Project.
 *
 *		Definitions for the NE2000 ethernet controller.
 *
 *
 *
 * Authors:	Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *		Copyright 2017,2018 Fred N. van Kempen.
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

use std::{ffi::CStr, ptr};

use super::net_dp8390::*;
use crate::{prelude::*, random::*};
use const_zero::const_zero;
use log::*;

const _NE2K_NONE: i32 = 0;
const NE2K_NE1000: i32 = 1; /* 8-bit ISA NE1000 */
const NE2K_NE2000: i32 = 2; /* 16-bit ISA NE2000 */
const NE2K_ETHERNEXT_MC: i32 = 3; /* 16-bit MCA EtherNext/MC */
const NE2K_RTL8019AS: i32 = 4; /* 16-bit ISA PnP Realtek 8019AS */
const NE2K_RTL8029AS: i32 = 5; /* 32-bit PCI Realtek 8029AS */

/*
 * VARCem	Virtual ARchaeological Computer EMulator.
 *		An emulator of (mostly) x86-based PC systems and devices,
 *		using the ISA,EISA,VLB,MCA  and PCI system buses, roughly
 *		spanning the era between 1981 and 1995.
 *
 *		This file is part of the VARCem Project.
 *
 *		Implementation of the following network controllers:
 *			- Novell NE1000 (ISA 8-bit);
 *			- Novell NE2000 (ISA 16-bit);
 *			- Novell NE/2 compatible (NetWorth Inc. Ethernext/MC) (MCA 16-bit);
 *			- Realtek RTL8019AS (ISA 16-bit, PnP);
 *			- Realtek RTL8029AS (PCI).
 *
 *
 *
 * Based on	@(#)ne2k.cc v1.56.2.1 2004/02/02 22:37:22 cbothamy
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

/*
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/mca.h>
#include <86box/pci.h>
#include <86box/pic.h>
#include <86box/random.h>
#include <86box/device.h>
#include <86box/network.h>
#include <86box/net_dp8390.h>
#include <86box/net_ne2000.h>
#include <86box/bswap.h>
#include <86box/isapnp.h>
*/

use crate::rom::*;

/* ROM BIOS file paths. */
static ROM_PATH_NE2000: &[u8] = b"roms/network/ne2000/ne2000.rom\0";
static ROM_PATH_RTL8019: &[u8] = b"roms/network/rtl8019as/rtl8019as.rom\0";
static ROM_PATH_RTL8029: &[u8] = b"roms/network/rtl8029as/rtl8029as.rom\0";

/* PCI info. */
const PCI_VENDID: u16 = 0x10ec; /* Realtek, Inc */
const PCI_DEVID: u16 = 0x8029; /* RTL8029AS */
const PCI_REGSIZE: usize = 256; /* size of PCI space */

const rtl8019as_pnp_rom: [u8; 75] = [
    0x4a, 0x8c, 0x80, 0x19, 0x00, 0x00, 0x00, 0x00,
    0x00, /* RTL8019, dummy checksum (filled in by isapnp_add_card) */
    0x0a, 0x10, 0x10, /* PnP version 1.0, vendor version 1.0 */
    0x82, 0x22, 0x00, b'R', b'E', b'A', b'L', b'T', b'E', b'K', b' ', b'P', b'L', b'U', b'G', b' ',
    b'&', b' ', b'P', b'L', b'A', b'Y', b' ', b'E', b'T', b'H', b'E', b'R', b'N', b'E', b'T', b' ',
    b'C', b'A', b'R', b'D', 0x00, /* ANSI identifier */
    0x16, 0x4a, 0x8c, 0x80, 0x19, 0x02, 0x00, /* logical device RTL8019 */
    0x1c, 0x41, 0xd0, 0x80, 0xd6, /* compatible device PNP80D6 */
    0x47, 0x00, 0x20, 0x02, 0x80, 0x03, 0x20,
    0x20, /* I/O 0x220-0x380, decodes 10-bit, 32-byte alignment, 32 addresses */
    0x23, 0x38, 0x9e, 0x01, /* IRQ 3/4/5/9/10/11/12/15, high true edge sensitive */
    0x79, 0x00, /* end tag, dummy checksum (filled in by isapnp_add_card) */
];

struct nic_t {
    dp8390: *mut dp8390_t,
    name: *const c_char,
    board: c_int,
    is_pci: bool,
    is_8bit: bool,
    base_address: u32,
    base_irq: c_int,
    bios_addr: u32,
    bios_size: u32,
    bios_mask: u32,
    card: c_int, /* PCI card slot */
    has_bios: bool,
    pci_bar: [bar_t; 2],
    pci_regs: [u8; PCI_REGSIZE],
    eeprom: [u8; 128], /* for RTL8029AS */
    bios_rom: rom_t,
    pnp_card: *mut c_void,
    pnp_csnsav: u8,
    maclocal: [u8; 6], /* configured MAC (local) address */

    /* RTL8019AS/RTL8029AS registers */
    config0: u8,
    config2: u8,
    config3: u8,
    _9346cr: u8,

    /* POS registers, MCA boards only */
    pos_regs: [u8; 8],
}

unsafe extern "C" fn nic_interrupt(r#priv: *mut c_void, set: c_int) {
    let dev: *mut nic_t = r#priv as _;

    if (*dev).is_pci {
        if (set) != 0 {
            pci_set_irq((*dev).card as u8, PCI_INTA);
        } else {
            pci_clear_irq((*dev).card as u8, PCI_INTA);
        }
    } else {
        if (set) != 0 {
            picint(1 << (*dev).base_irq);
        } else {
            picintc(1 << (*dev).base_irq);
        }
    }
}

/* reset - restore state to power-up, cancelling all i/o */
unsafe extern "C" fn nic_reset(r#priv: *mut c_void) {
    let dev: *mut nic_t = r#priv as _;

    trace!("{:?}: reset", CStr::from_ptr((*dev).name));

    dp8390_reset((*dev).dp8390);
}

unsafe extern "C" fn nic_soft_reset(r#priv: *mut c_void) {
    let dev: *mut nic_t = r#priv as _;

    dp8390_soft_reset((*dev).dp8390);
}

/*
 * Access the ASIC I/O space.
 *
 * This is the high 16 bytes of i/o space (the lower 16 bytes
 * is for the DS8390). Only two locations are used: offset 0,
 * which is used for data transfer, and offset 0x0f, which is
 * used to reset the device.
 *
 * The data transfer port is used to as 'external' DMA to the
 * DS8390. The chip has to have the DMA registers set up, and
 * after that, insw/outsw instructions can be used to move
 * the appropriate number of bytes to/from the device.
 */
unsafe fn asic_read(dev: *mut nic_t, off: u32, len: c_uint) -> u32 {
    let mut retval: u32 = 0;

    match off {
        0x00 => {
            /* Data register */
            /* A read remote-DMA command must have been issued,
            and the source-address and length registers must
            have been initialised. */
            if len > (*(*dev).dp8390).remote_bytes as c_uint {
                warn!(
                    "{:?}: DMA read underrun iolen={} remote_bytes={}",
                    CStr::from_ptr((*dev).name),
                    len,
                    (*(*dev).dp8390).remote_bytes
                );
            }

            trace!(
                "{:?}: DMA read: addr={:x} remote_bytes={}",
                CStr::from_ptr((*dev).name),
                (*(*dev).dp8390).remote_dma,
                (*(*dev).dp8390).remote_bytes
            );
            retval = dp8390_chipmem_read((*dev).dp8390, (*(*dev).dp8390).remote_dma as u32, len);

            /* The 8390 bumps the address and decreases the byte count
            by the selected word size after every access, not by
            the amount of data requested by the host (io_len). */
            if len == 4 {
                (*(*dev).dp8390).remote_dma += len as u16;
            } else {
                (*(*dev).dp8390).remote_dma += ((*(*dev).dp8390).DCR.wdsize + 1) as u16;
            }

            if (*(*dev).dp8390).remote_dma == ((*(*dev).dp8390).page_stop as u16) << 8 {
                (*(*dev).dp8390).remote_dma = ((*(*dev).dp8390).page_start as u16) << 8;
            }

            /* keep s.remote_bytes from underflowing */
            if (*(*dev).dp8390).remote_bytes > (*(*dev).dp8390).DCR.wdsize as u16 {
                if len == 4 {
                    (*(*dev).dp8390).remote_bytes -= len as u16;
                } else {
                    (*(*dev).dp8390).remote_bytes -= ((*(*dev).dp8390).DCR.wdsize + 1) as u16;
                }
            } else {
                (*(*dev).dp8390).remote_bytes = 0;
            }

            /* If all bytes have been written, signal remote-DMA complete */
            if (*(*dev).dp8390).remote_bytes == 0 {
                (*(*dev).dp8390).ISR.rdma_done = 1;
                if ((*(*dev).dp8390).IMR.rdma_inte) != 0 {
                    nic_interrupt(dev as _, 1);
                }
            }
        }

        0x0f =>
        /* Reset register */
        {
            nic_soft_reset(dev as _)
        }

        _ => warn!(
            "{:?}: ASIC read invalid address {:x}",
            CStr::from_ptr((*dev).name),
            off
        ),
    }

    return retval;
}

unsafe fn asic_write(dev: *mut nic_t, off: u32, val: u32, len: c_uint) {
    trace!(
        "{:?}: ASIC write addr=0x{:x}, value=0x{:x}",
        CStr::from_ptr((*dev).name),
        off,
        val
    );

    loop {
        match off {
            0x00 => {
                /* Data register - see asic_read for a description */
                if (len > 1) && ((*(*dev).dp8390).DCR.wdsize == 0) {
                    trace!(
                        "{:?}: DMA write length {} on byte mode operation",
                        CStr::from_ptr((*dev).name),
                        len
                    );
                    break;
                }
                if (*(*dev).dp8390).remote_bytes == 0 {
                    trace!("{:?}: DMA write, byte count 0", CStr::from_ptr((*dev).name));
                }

                dp8390_chipmem_write((*dev).dp8390, (*(*dev).dp8390).remote_dma as u32, val, len);
                if len == 4 {
                    (*(*dev).dp8390).remote_dma += len as u16;
                } else {
                    (*(*dev).dp8390).remote_dma += ((*(*dev).dp8390).DCR.wdsize + 1) as u16;
                }

                if (*(*dev).dp8390).remote_dma == ((*(*dev).dp8390).page_stop as u16) << 8 {
                    (*(*dev).dp8390).remote_dma = ((*(*dev).dp8390).page_start as u16) << 8;
                }

                if len == 4 {
                    (*(*dev).dp8390).remote_bytes -= len as u16;
                } else {
                    (*(*dev).dp8390).remote_bytes -= ((*(*dev).dp8390).DCR.wdsize + 1) as u16;
                }

                if (*(*dev).dp8390).remote_bytes > (*(*dev).dp8390).mem_size as u16 {
                    (*(*dev).dp8390).remote_bytes = 0;
                }

                /* If all bytes have been written, signal remote-DMA complete */
                if (*(*dev).dp8390).remote_bytes == 0 {
                    (*(*dev).dp8390).ISR.rdma_done = 1;
                    if ((*(*dev).dp8390).IMR.rdma_inte) != 0 {
                        nic_interrupt(dev as _, 1);
                    }
                }
            }

            0x0f => { /* Reset register */
                /* end of reset pulse */
            }

            _ => {
                /* this is invalid, but happens under win95 device detection */
                warn!(
                    "{:?}: ASIC write invalid address {:x}, ignoring",
                    CStr::from_ptr((*dev).name),
                    off
                );
            }
        }
        break;
    }
}

/* Writes to this page are illegal. */
unsafe fn page3_read(dev: *mut nic_t, off: u32, _len: c_uint) -> u32 {
    if (*dev).board >= NE2K_RTL8019AS {
        match off {
            0x1 => {
                /* 9346CR */
                return ((*dev)._9346cr) as u32;
            }

            0x3 => {
                /* CONFIG0 */
                return 0x00; /* Cable not BNC */
            }

            0x5 => {
                /* CONFIG2 */
                return ((*dev).config2 & 0xe0) as u32;
            }

            0x6 => {
                /* CONFIG3 */
                return ((*dev).config3 & 0x46) as u32;
            }

            0x8 => {
                /* CSNSAV */
                return if (*dev).board == NE2K_RTL8019AS {
                    (*dev).pnp_csnsav as u32
                } else {
                    0x00
                };
            }

            0xe => {
                /* 8029ASID0 */
                if (*dev).board == NE2K_RTL8029AS {
                    return 0x29;
                }
            }

            0xf => {
                /* 8029ASID1 */
                if (*dev).board == NE2K_RTL8029AS {
                    return 0x80;
                }
            }

            _ => {}
        }
    }

    trace!(
        "{:?}: Page3 read register 0x{:x} attempted",
        CStr::from_ptr((*dev).name),
        off,
    );
    return 0x00;
}

unsafe fn page3_write(dev: *mut nic_t, off: u32, val: u32, len: c_uint) {
    if (*dev).board >= NE2K_RTL8019AS {
        trace!(
            "{:?}: Page2 write to register 0x{:x}, len={}, value=0x{:x}",
            CStr::from_ptr((*dev).name),
            off,
            len,
            val
        );

        match off {
            0x01 => {
                /* 9346CR */
                (*dev)._9346cr = (val & 0xfe) as u8;
            }

            0x05 => {
                /* CONFIG2 */
                (*dev).config2 = (val & 0xe0) as u8;
            }

            0x06 => {
                /* CONFIG3 */
                (*dev).config3 = (val & 0x46) as u8;
            }

            0x09 => { /* HLTCLK  */ }

            _ => {
                debug!(
                    "{:?}: Page3 write to reserved register 0x{:x}",
                    CStr::from_ptr((*dev).name),
                    off,
                );
            }
        }
    } else {
        trace!(
            "{:?}: Page3 write register 0x{:x} attempted",
            CStr::from_ptr((*dev).name),
            off,
        );
    }
}

unsafe fn nic_read(dev: *mut nic_t, addr: u32, len: c_uint) -> u32 {
    let mut retval = 0;
    let off: c_int = addr as c_int - (*dev).base_address as c_int;

    trace!(
        "{:?}: read addr {}, len {}",
        CStr::from_ptr((*dev).name),
        addr,
        len
    );

    if off >= 0x10 {
        retval = asic_read(dev, off as u32 - 0x10, len);
    } else if off == 0x00 {
        retval = dp8390_read_cr((*dev).dp8390);
    } else {
        match (*(*dev).dp8390).CR.pgsel {
            0x00 => {
                retval = dp8390_page0_read((*dev).dp8390, off as u32, len);
            }
            0x01 => {
                retval = dp8390_page1_read((*dev).dp8390, off as u32, len);
            }
            0x02 => {
                retval = dp8390_page2_read((*dev).dp8390, off as u32, len);
            }
            0x03 => {
                retval = page3_read(dev, off as u32, len);
            }
            _ => {
                warn!(
                    "{:?}: unknown value of pgsel in read - {}",
                    CStr::from_ptr((*dev).name),
                    (*(*dev).dp8390).CR.pgsel
                );
            }
        }
    }

    return retval;
}

unsafe extern "C" fn nic_readb(addr: u16, r#priv: *mut c_void) -> u8 {
    return (nic_read(r#priv as _, addr as u32, 1)) as u8;
}

unsafe extern "C" fn nic_readw(addr: u16, r#priv: *mut c_void) -> u16 {
    return (nic_read(r#priv as _, addr as u32, 2)) as u16;
}

unsafe extern "C" fn nic_readl(addr: u16, r#priv: *mut c_void) -> u32 {
    return nic_read(r#priv as _, addr as u32, 4);
}

unsafe fn nic_write(dev: *mut nic_t, addr: u32, val: u32, len: c_uint) {
    let off: c_int = addr as c_int - (*dev).base_address as c_int;

    trace!(
        "{:?}: write addr {}, value {} len {}",
        CStr::from_ptr((*dev).name),
        addr,
        val,
        len
    );

    /* The high 16 bytes of i/o space are for the ne2000 asic -
    the low 16 bytes are for the DS8390, with the current
    page being selected by the PS0,PS1 registers in the
    command register */
    if off >= 0x10 {
        asic_write(dev, off as u32 - 0x10, val, len);
    } else if off == 0x00 {
        dp8390_write_cr((*dev).dp8390, val);
    } else {
        match (*(*dev).dp8390).CR.pgsel {
            0x00 => {
                dp8390_page0_write((*dev).dp8390, off as u32, val, len);
            }
            0x01 => {
                dp8390_page1_write((*dev).dp8390, off as u32, val, len);
            }
            0x02 => {
                dp8390_page2_write((*dev).dp8390, off as u32, val, len);
            }
            0x03 => {
                page3_write(dev, off as u32, val, len);
            }
            _ => {
                warn!(
                    "{:?}: unknown value of pgsel in write - {}",
                    CStr::from_ptr((*dev).name),
                    (*(*dev).dp8390).CR.pgsel
                );
            }
        }
    }
}

unsafe extern "C" fn nic_writeb(addr: u16, val: u8, r#priv: *mut c_void) {
    nic_write(r#priv as _, addr as u32, val as u32, 1);
}

unsafe extern "C" fn nic_writew(addr: u16, val: u16, r#priv: *mut c_void) {
    nic_write(r#priv as _, addr as u32, val as u32, 2);
}

unsafe extern "C" fn nic_writel(addr: u16, val: u32, r#priv: *mut c_void) {
    nic_write(r#priv as _, addr as u32, val, 4);
}

unsafe extern "C" fn nic_pnp_config_changed(
    ld: u8,
    config: *mut isapnp_device_config_t,
    r#priv: *mut c_void,
) {
    if (ld) != 0 {
        return;
    }

    let dev: *mut nic_t = r#priv as _;

    if ((*dev).base_address) != 0 {
        nic_ioremove(dev, (*dev).base_address as u16);
        (*dev).base_address = 0;
    }

    (*dev).base_address = (*config).io[0].base as u32;
    (*dev).base_irq = (*config).irq[0].irq() as i32;

    if (*config).activate != 0 && ((*dev).base_address != ISAPNP_IO_DISABLED) {
        nic_ioset(dev, (*dev).base_address as u16);
    }
}

unsafe extern "C" fn nic_pnp_csn_changed(csn: u8, r#priv: *mut c_void) {
    let dev: *mut nic_t = r#priv as _;

    (*dev).pnp_csnsav = csn;
}

unsafe extern "C" fn nic_pnp_read_vendor_reg(ld: u8, reg: u8, r#priv: *mut c_void) -> u8 {
    if ld != 0 {
        return 0x00;
    }

    let dev: *mut nic_t = r#priv as _;

    match reg {
        0xF0 => return (*dev).config0,

        0xF2 => return (*dev).config2,

        0xF3 => return (*dev).config3,

        0xF5 => return (*dev).pnp_csnsav,
        _ => {}
    }

    return 0x00;
}

unsafe extern "C" fn nic_pnp_write_vendor_reg(ld: u8, reg: u8, val: u8, r#priv: *mut c_void) {
    let dev: *mut nic_t = r#priv as _;

    if (ld == 0) && (reg == 0xf6) && (val & 0x04) != 0 {
        let csn: u8 = (*dev).pnp_csnsav;
        isapnp_set_csn((*dev).pnp_card, 0);
        (*dev).pnp_csnsav = csn;
    }
}

unsafe fn nic_ioset(dev: *mut nic_t, addr: u16) {
    if (*dev).is_pci {
        io_sethandler(
            addr,
            32,
            Some(nic_readb),
            Some(nic_readw),
            Some(nic_readl),
            Some(nic_writeb),
            Some(nic_writew),
            Some(nic_writel),
            dev as _,
        );
    } else {
        io_sethandler(
            addr,
            16,
            Some(nic_readb),
            None,
            None,
            Some(nic_writeb),
            None,
            None,
            dev as _,
        );
        if (*dev).is_8bit {
            io_sethandler(
                addr + 16,
                16,
                Some(nic_readb),
                None,
                None,
                Some(nic_writeb),
                None,
                None,
                dev as _,
            );
        } else {
            io_sethandler(
                addr + 16,
                16,
                Some(nic_readb),
                Some(nic_readw),
                None,
                Some(nic_writeb),
                Some(nic_writew),
                None,
                dev as _,
            );
        }
    }
}

unsafe fn nic_ioremove(dev: *mut nic_t, addr: u16) {
    if (*dev).is_pci {
        io_removehandler(
            addr,
            32,
            Some(nic_readb),
            Some(nic_readw),
            Some(nic_readl),
            Some(nic_writeb),
            Some(nic_writew),
            Some(nic_writel),
            dev as _,
        );
    } else {
        io_removehandler(
            addr,
            16,
            Some(nic_readb),
            None,
            None,
            Some(nic_writeb),
            None,
            None,
            dev as _,
        );
        if (*dev).is_8bit {
            io_removehandler(
                addr + 16,
                16,
                Some(nic_readb),
                None,
                None,
                Some(nic_writeb),
                None,
                None,
                dev as _,
            );
        } else {
            io_removehandler(
                addr + 16,
                16,
                Some(nic_readb),
                Some(nic_readw),
                None,
                Some(nic_writeb),
                Some(nic_writew),
                None,
                dev as _,
            );
        }
    }
}

unsafe fn nic_update_bios(dev: *mut nic_t) {
    let mut reg_bios_enable: c_int = 1;

    if !(*dev).has_bios {
        return;
    }

    if (*dev).is_pci {
        reg_bios_enable = (*dev).pci_bar[1].addr_regs.as_ref()[0] as i32 & 0x01;
    }

    /* PCI BIOS stuff, just enable_disable. */
    if (reg_bios_enable) != 0 {
        mem_mapping_set_addr(
            &mut (*dev).bios_rom.mapping,
            (*dev).bios_addr,
            (*dev).bios_size,
        );
        trace!(
            "{:?}: BIOS now at: {:x}",
            CStr::from_ptr((*dev).name),
            (*dev).bios_addr
        );
    } else {
        trace!("{:?}: BIOS disabled", CStr::from_ptr((*dev).name));
        mem_mapping_disable(&mut (*dev).bios_rom.mapping);
    }
}

unsafe extern "C" fn nic_pci_read(func: c_int, addr: c_int, r#priv: *mut c_void) -> u8 {
    let dev: *mut nic_t = r#priv as _;
    let mut ret: u8 = 0x00;
    let addr = addr as usize;
    match addr {
	 0x00|			/* PCI_VID_LO */
	 0x01 =>			/* PCI_VID_HI */
		ret = (*dev).pci_regs[addr],

	 0x02|			/* PCI_DID_LO */
	 0x03 =>			/* PCI_DID_HI */
		ret = (*dev).pci_regs[addr],

	 0x04|			/* PCI_COMMAND_LO */
	 0x05 =>			/* PCI_COMMAND_HI */
		ret = (*dev).pci_regs[addr],

	 0x06|			/* PCI_STATUS_LO */
	 0x07 =>			/* PCI_STATUS_HI */
		ret = (*dev).pci_regs[addr],

	 0x08 =>			/* PCI_REVID */
		ret = 0x00,		/* Rev. 00 */
	 0x09 =>			/* PCI_PIFR */
		ret = 0x00,		/* Rev. 00 */

	 0x0A =>			/* PCI_SCR */
		ret = (*dev).pci_regs[addr],

	 0x0B =>			/* PCI_BCR */
		ret = (*dev).pci_regs[addr],

	 0x10 =>			/* PCI_BAR 7:5 */
		ret = ((*dev).pci_bar[0].addr_regs.as_ref()[0] & 0xe0) | 0x01,
	 0x11 =>			/* PCI_BAR 15:8 */
		ret = (*dev).pci_bar[0].addr_regs.as_ref()[1],
	 0x12 =>			/* PCI_BAR 23:16 */
		ret = (*dev).pci_bar[0].addr_regs.as_ref()[2],
	 0x13 =>			/* PCI_BAR 31:24 */
		ret = (*dev).pci_bar[0].addr_regs.as_ref()[3],

	 0x2C|			/* PCI_SVID_LO */
	 0x2D =>			/* PCI_SVID_HI */
		ret = (*dev).pci_regs[addr],

	 0x2E|			/* PCI_SID_LO */
	 0x2F =>			/* PCI_SID_HI */
		ret = (*dev).pci_regs[addr],

	 0x30 =>			/* PCI_ROMBAR */
		ret = (*dev).pci_bar[1].addr_regs.as_ref()[0] & 0x01,
	 0x31 =>			/* PCI_ROMBAR 15:11 */
		ret = (*dev).pci_bar[1].addr_regs.as_ref()[1] & 0x80,
	 0x32 =>			/* PCI_ROMBAR 23:16 */
		ret = (*dev).pci_bar[1].addr_regs.as_ref()[2],
	 0x33 =>			/* PCI_ROMBAR 31:24 */
		ret = (*dev).pci_bar[1].addr_regs.as_ref()[3],

	 0x3C =>			/* PCI_ILR */
		ret = (*dev).pci_regs[addr],

	 0x3D =>			/* PCI_IPR */
		ret = (*dev).pci_regs[addr],
    _ => {}
    }

    trace!(
        "{:?}: PCI_Read({}, {:x}) = {:x}",
        CStr::from_ptr((*dev).name),
        func,
        addr,
        ret
    );

    return ret;
}

unsafe extern "C" fn nic_pci_write(func: c_int, addr: c_int, mut val: u8, r#priv: *mut c_void) {
    let dev: *mut nic_t = r#priv as _;
    let valxor: u8;

    trace!(
        "{:?}: PCI_Write({}, {:x}, {:x})",
        CStr::from_ptr((*dev).name),
        func,
        addr,
        val
    );

    let addr = addr as usize;
    match addr {
        0x04 => {
            /* PCI_COMMAND_LO */
            valxor = (val & 0x03) ^ (*dev).pci_regs[addr];
            if (valxor & PCI_COMMAND_IO as u8) != 0 {
                nic_ioremove(dev, (*dev).base_address as u16);
                if ((*dev).base_address != 0) && (val & PCI_COMMAND_IO as u8) != 0 {
                    nic_ioset(dev, (*dev).base_address as u16);
                }
            }
            (*dev).pci_regs[addr] = val & 0x03;
        }

        0x10 | 0x11 | 0x12 | 0x13 => {
            /* PCI_BAR */
            if addr == 0x10 {
                val &= 0xe0; /* 0xe0 acc to RTL DS */
                val |= 0x01; /* re-enable IOIN bit */
            }
            /* Remove old I/O. */
            nic_ioremove(dev, (*dev).base_address as u16);

            /* Set new I/O as per PCI request. */
            (*dev).pci_bar[0].addr_regs.as_mut()[addr & 3] = val;

            /* Then let's calculate the new I/O base. */
            (*dev).base_address = *(*dev).pci_bar[0].addr.as_ref() & 0xffe0;

            /* Log the new base. */
            trace!(
                "{:?}: PCI: new I/O base is {:x}",
                CStr::from_ptr((*dev).name),
                (*dev).base_address
            );
            /* We're done, so get out of the here. */
            if ((*dev).pci_regs[4] & PCI_COMMAND_IO as u8) != 0 {
                if (*dev).base_address != 0 {
                    nic_ioset(dev, (*dev).base_address as u16);
                }
            }
        }

        0x30 | 0x31 | 0x32 | 0x33 => {
            /* PCI_ROMBAR */
            (*dev).pci_bar[1].addr_regs.as_mut()[addr & 3] = val;
            /* (*dev).pci_bar[1].addr_regs[1] &= (*dev).bios_mask; */
            *(*dev).pci_bar[1].addr.as_mut() &= 0xffff8001;
            (*dev).bios_addr = *(*dev).pci_bar[1].addr.as_ref() & 0xffff8000;
            nic_update_bios(dev);
            return;
        }

        0x3C => {
            /* PCI_ILR */
            trace!("{:?}: IRQ now: {}", CStr::from_ptr((*dev).name), val);
            (*dev).base_irq = val as i32;
            (*dev).pci_regs[addr] = (*dev).base_irq as u8;
            return;
        }
        _ => {}
    }
}

unsafe fn nic_rom_init(dev: *mut nic_t, s: *mut c_char) {
    let temp: i64;
    let f: *mut FILE;

    if s.is_null() {
        return;
    }

    if (*dev).bios_addr == 0 {
        return;
    }

    if {
        f = rom_fopen(s, b"rb\0".as_ptr() as *mut _);
        !f.is_null()
    } {
        libc::fseek(f, 0, SEEK_END);
        temp = ftell(f);
        fclose(f);
        if temp == -1 {
            panic!("net_ne2000: ftell failed");
        }
        (*dev).bios_size = 0x10000;
        if temp <= 0x8000 {
            (*dev).bios_size = 0x8000;
        }
        if temp <= 0x4000 {
            (*dev).bios_size = 0x4000;
        }
        if temp <= 0x2000 {
            (*dev).bios_size = 0x2000;
        }
        (*dev).bios_mask = ((*dev).bios_size >> 8) & 0xff;
        (*dev).bios_mask = (0x100 - (*dev).bios_mask) & 0xff;
    } else {
        (*dev).bios_addr = 0x00000;
        (*dev).bios_size = 0;
        return;
    }

    /* Create a memory mapping for the space. */
    rom_init(
        &mut (*dev).bios_rom,
        s,
        (*dev).bios_addr,
        (*dev).bios_size as i32,
        (*dev).bios_size as i32 - 1,
        0,
        MEM_MAPPING_EXTERNAL,
    );

    trace!(
        "{:?}: BIOS configured at {:x} (size {})",
        CStr::from_ptr((*dev).name),
        (*dev).bios_addr,
        (*dev).bios_size
    );
}

unsafe extern "C" fn nic_mca_read(port: c_int, r#priv: *mut c_void) -> u8 {
    let dev: *mut nic_t = r#priv as _;

    return (*dev).pos_regs[port as usize & 7];
}

const MCA_611F_IO_PORTS: &[u16] = &[0x300, 0x340, 0x320, 0x360, 0x1300, 0x1340, 0x1320, 0x1360];

const MCA_611F_IRQS: &[i8] = &[2, 3, 4, 5, 10, 11, 12, 15];

unsafe extern "C" fn nic_mca_write(port: c_int, val: u8, r#priv: *mut c_void) {
    let dev: *mut nic_t = r#priv as _;
    let base = MCA_611F_IO_PORTS;
    let irq = MCA_611F_IRQS;

    /* MCA does not write registers below 0x0100. */
    if port < 0x0102 {
        return;
    }

    /* Save the MCA register value. */
    (*dev).pos_regs[port as usize & 7] = val;

    nic_ioremove(dev, (*dev).base_address as u16);

    /* This is always necessary so that the old handler doesn't remain. */
    /* Get the new assigned I/O base address. */
    (*dev).base_address = base[((*dev).pos_regs[2] as usize & 0xE0) >> 4] as u32;

    /* Save the new IRQ values. */
    (*dev).base_irq = irq[((*dev).pos_regs[2] as usize & 0xE) >> 1] as i32;

    (*dev).bios_addr = 0x0000;
    (*dev).has_bios = false;

    /*
     * The PS/2 Model 80 BIOS always enables a card if it finds one,
     * even if no resources were assigned yet (because we only added
     * the card, but have not run AutoConfig yet...)
     *
     * So, remove current address, if any.
     */

    /* Initialize the device if fully configured. */
    if ((*dev).pos_regs[2] & 0x01) != 0 {
        /* Card enabled; register (new) I/O handler. */

        nic_ioset(dev, (*dev).base_address as u16);

        nic_reset(dev as _);

        trace!(
            "EtherNext/MC: Port={:x}, IRQ={}",
            (*dev).base_address,
            (*dev).base_irq,
        );
    }
}

unsafe extern "C" fn nic_mca_feedb(r#priv: *mut c_void) -> u8 {
    let dev: *mut nic_t = r#priv as _;

    return (*dev).pos_regs[2] & 0x01;
}

unsafe extern "C" fn nic_init(info: *const device_t) -> *mut c_void {
    let mut mac: u32;
    let mut nic_rom: *const c_char = ptr::null();
    let mut dev = Box::new(const_zero!(nic_t));
    dev.name = (*info).name;
    dev.board = (*info).local as i32;

    if dev.board >= NE2K_RTL8019AS {
        dev.base_address = 0x340;
        dev.base_irq = 12;
        if dev.board == NE2K_RTL8029AS {
            dev.bios_addr = 0xD0000;
            dev.has_bios = device_get_config_int(b"bios\0".as_ptr() as _) != 0;
        } else {
            dev.bios_addr = 0x00000;
            dev.has_bios = false;
        }
    } else {
        if dev.board != NE2K_ETHERNEXT_MC {
            dev.base_address = device_get_config_hex16(b"base\0".as_ptr() as _) as u32;
            dev.base_irq = device_get_config_int(b"irq\0".as_ptr() as _);
            if dev.board == NE2K_NE2000 {
                dev.bios_addr = device_get_config_hex20(b"bios_addr\0".as_ptr() as _) as u32;
                dev.has_bios = dev.bios_addr != 0;
            } else {
                dev.bios_addr = 0x00000;
                dev.has_bios = false;
            }
        } else {
            mca_add(
                Some(nic_mca_read),
                Some(nic_mca_write),
                Some(nic_mca_feedb),
                None,
                &mut *dev as *mut _ as _,
            );
        }
    }

    /* See if we have a local MAC address configured. */
    mac = device_get_config_mac(b"mac\0".as_ptr() as _, -1) as u32;

    /* Set up our BIA. */
    if (mac & 0xff000000) != 0 {
        /* Generate new local MAC. */
        dev.maclocal[3] = random_generate();
        dev.maclocal[4] = random_generate();
        dev.maclocal[5] = random_generate();
        mac = (dev.maclocal[3] as u32) << 16;
        mac |= (dev.maclocal[4] as u32) << 8;
        mac |= (dev.maclocal[5]) as u32;
        device_set_config_mac(b"mac\0".as_ptr() as _, mac as i32);
    } else {
        dev.maclocal[3] = ((mac >> 16) & 0xff) as u8;
        dev.maclocal[4] = ((mac >> 8) & 0xff) as u8;
        dev.maclocal[5] = (mac & 0xff) as u8;
    }

    dev.dp8390 = device_add(&dp8390_device) as _;
    (*dev.dp8390).r#priv = &mut *dev as *mut _ as _;
    (*dev.dp8390).interrupt = Some(nic_interrupt);

    match dev.board {
        NE2K_NE1000 => {
            dev.maclocal[0] = 0x00; /* 00:00:D8 (Novell OID) */
            dev.maclocal[1] = 0x00;
            dev.maclocal[2] = 0xD8;
            dev.is_8bit = true;
            nic_rom = ptr::null_mut();
            dp8390_set_defaults(
                dev.dp8390,
                DP8390_FLAG_CHECK_CR as u8 | DP8390_FLAG_CLEAR_IRQ as u8,
            );
            dp8390_mem_alloc(dev.dp8390, 0x2000, 0x2000);
        }

        NE2K_NE2000 => {
            dev.maclocal[0] = 0x00; /* 00:00:D8 (Novell OID) */
            dev.maclocal[1] = 0x00;
            dev.maclocal[2] = 0xD8;
            nic_rom = ROM_PATH_NE2000.as_ptr() as _;
            dp8390_set_defaults(
                dev.dp8390,
                DP8390_FLAG_EVEN_MAC as u8
                    | DP8390_FLAG_CHECK_CR as u8
                    | DP8390_FLAG_CLEAR_IRQ as u8,
            );
            dp8390_mem_alloc(dev.dp8390, 0x4000, 0x4000);
        }

        NE2K_ETHERNEXT_MC => {
            dev.maclocal[0] = 0x00; /* 00:00:D8 (Networth Inc. OID) */
            dev.maclocal[1] = 0x00;
            dev.maclocal[2] = 0x79;
            dev.pos_regs[0] = 0x1F;
            dev.pos_regs[1] = 0x61;
            nic_rom = ptr::null_mut();
            dp8390_set_defaults(
                dev.dp8390,
                DP8390_FLAG_EVEN_MAC as u8
                    | DP8390_FLAG_CHECK_CR as u8
                    | DP8390_FLAG_CLEAR_IRQ as u8,
            );
            dp8390_mem_alloc(dev.dp8390, 0x4000, 0x4000);
        }

        NE2K_RTL8019AS | NE2K_RTL8029AS => {
            dev.is_pci = dev.board == NE2K_RTL8029AS;
            dev.maclocal[0] = 0x00; /* 00:E0:4C (Realtek OID) */
            dev.maclocal[1] = 0xE0;
            dev.maclocal[2] = 0x4C;
            nic_rom = if dev.board == NE2K_RTL8019AS {
                ROM_PATH_RTL8019.as_ptr() as _
            } else {
                ROM_PATH_RTL8029.as_ptr() as _
            };
            if dev.is_pci {
                dp8390_set_defaults(dev.dp8390, DP8390_FLAG_EVEN_MAC);
            } else {
                dp8390_set_defaults(dev.dp8390, DP8390_FLAG_EVEN_MAC | DP8390_FLAG_CLEAR_IRQ);
            }
            dp8390_set_id(
                dev.dp8390,
                0x50,
                if dev.board == NE2K_RTL8019AS {
                    0x70
                } else {
                    0x43
                },
            );
            dp8390_mem_alloc(dev.dp8390, 0x4000, 0x8000);
        }
        board => {
            warn!("net_ne2000: Unknown dev->board value {}", board);
        }
    }

    (*dev.dp8390).physaddr.copy_from_slice(&dev.maclocal);

    trace!(
        "{:?}: I/O={:x}, IRQ={}, MAC={:x}:{:x}:{:x}:{:x}:{:x}:{:x}",
        CStr::from_ptr(dev.name),
        dev.base_address,
        dev.base_irq,
        (*dev.dp8390).physaddr[0],
        (*dev.dp8390).physaddr[1],
        (*dev.dp8390).physaddr[2],
        (*dev.dp8390).physaddr[3],
        (*dev.dp8390).physaddr[4],
        (*dev.dp8390).physaddr[5],
    );

    /*
     * Make this device known to the I/O system.
     * PnP and PCI devices start with address spaces inactive.
     */
    if dev.board < NE2K_RTL8019AS && dev.board != NE2K_ETHERNEXT_MC {
        nic_ioset(&mut *dev, dev.base_address as u16);
    }

    /* Set up our BIOS ROM space, if any. */
    nic_rom_init(&mut *dev, nic_rom as *mut _);

    if dev.board >= NE2K_RTL8019AS {
        if dev.is_pci {
            /*
             * Configure the PCI space registers.
             *
             * We do this here, so the I/O routines are generic.
             */
            dev.pci_regs.fill(0);

            dev.pci_regs[0x00] = (PCI_VENDID & 0xff) as u8;
            dev.pci_regs[0x01] = (PCI_VENDID >> 8) as u8;
            dev.pci_regs[0x02] = (PCI_DEVID & 0xff) as u8;
            dev.pci_regs[0x03] = (PCI_DEVID >> 8) as u8;

            dev.pci_regs[0x04] = 0x03; /* IOEN */
            dev.pci_regs[0x05] = 0x00;
            dev.pci_regs[0x07] = 0x02; /* DST0, medium devsel */

            dev.pci_regs[0x09] = 0x00; /* PIFR */

            dev.pci_regs[0x0B] = 0x02; /* BCR: Network Controller */
            dev.pci_regs[0x0A] = 0x00; /* SCR: Ethernet */

            dev.pci_regs[0x2C] = (PCI_VENDID & 0xff) as u8;
            dev.pci_regs[0x2D] = (PCI_VENDID >> 8) as u8;
            dev.pci_regs[0x2E] = (PCI_DEVID & 0xff) as u8;
            dev.pci_regs[0x2F] = (PCI_DEVID >> 8) as u8;

            dev.pci_regs[0x3D] = PCI_INTA; /* PCI_IPR */

            /* Enable our address space in PCI. */
            dev.pci_bar[0].addr_regs.as_mut()[0] = 0x01;

            /* Enable our BIOS space in PCI, if needed. */
            if dev.bios_addr > 0 {
                *dev.pci_bar[1].addr.as_mut() = 0xFFFF8000;
                dev.pci_bar[1].addr_regs.as_mut()[1] = dev.bios_mask as u8;
            } else {
                *dev.pci_bar[1].addr.as_mut() = 0;
                dev.bios_size = 0;
            }

            mem_mapping_disable(&mut dev.bios_rom.mapping);

            /* Add device to the PCI bus, keep its slot number. */
            dev.card = pci_add_card(
                PCI_ADD_NORMAL as u8,
                Some(nic_pci_read),
                Some(nic_pci_write),
                &mut *dev as *mut _ as _,
            ) as i32;
        }

        /* Initialize the RTL8029 EEPROM. */
        dev.eeprom.fill(0);

        if dev.board == NE2K_RTL8029AS {
            dev.eeprom[0x02..0x08].copy_from_slice(&dev.maclocal);
            dev.eeprom[0x76] = (PCI_DEVID & 0xff) as u8;
            dev.eeprom[0x7A] = (PCI_DEVID & 0xff) as u8;
            dev.eeprom[0x7E] = (PCI_DEVID & 0xff) as u8;
            dev.eeprom[0x77] = (PCI_DEVID >> 8) as u8;
            dev.eeprom[0x7B] = (PCI_DEVID >> 8) as u8;
            dev.eeprom[0x7F] = (PCI_DEVID >> 8) as u8;
            dev.eeprom[0x78] = (PCI_VENDID & 0xff) as u8;
            dev.eeprom[0x7C] = (PCI_VENDID & 0xff) as u8;
            dev.eeprom[0x79] = (PCI_VENDID >> 8) as u8;
            dev.eeprom[0x7D] = (PCI_VENDID >> 8) as u8;
        } else {
            dev.eeprom[0x12..rtl8019as_pnp_rom.len() + 0x12].copy_from_slice(&rtl8019as_pnp_rom);

            dev.pnp_card = isapnp_add_card(
                dev.eeprom.as_mut_ptr().offset(0x12) as _,
                rtl8019as_pnp_rom.len() as u16,
                Some(nic_pnp_config_changed),
                Some(nic_pnp_csn_changed),
                Some(nic_pnp_read_vendor_reg),
                Some(nic_pnp_write_vendor_reg),
                &mut *dev as *mut _ as _,
            );
        }
    }

    if dev.board != NE2K_ETHERNEXT_MC {
        /* Reset the board. */
        nic_reset(&mut *dev as *mut _ as _);
    }

    /* Attach ourselves to the network module. */
    network_attach(
        dev.dp8390 as _,
        (*dev.dp8390).physaddr.as_mut_ptr(),
        Some(dp8390_rx),
        None,
        None,
    );

    trace!(
        "{:?}: {} attached IO=0x{:x} IRQ={}",
        CStr::from_ptr(dev.name),
        if dev.is_pci { "PCI" } else { "ISA" },
        dev.base_address,
        dev.base_irq
    );

    return Box::into_raw(dev) as _;
}

unsafe extern "C" fn nic_close(r#priv: *mut c_void) {
    let dev = Box::from_raw(r#priv as *mut nic_t);

    trace!("{:?}: closed", CStr::from_ptr(dev.name));

    drop(dev);
}

static ne1000_config: [device_config_t; 4] = [
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
                description: b"0x280\0".as_ptr() as _,
                value: 0x280,
            },
            device_config_selection_t {
                description: b"0x300\0".as_ptr() as _,
                value: 0x300,
            },
            device_config_selection_t {
                description: b"0x320\0".as_ptr() as _,
                value: 0x320,
            },
            device_config_selection_t {
                description: b"0x340\0".as_ptr() as _,
                value: 0x340,
            },
            device_config_selection_t {
                description: b"0x360\0".as_ptr() as _,
                value: 0x360,
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
                description: b"IRQ 10\0".as_ptr() as _,
                value: 10,
            },
            device_config_selection_t {
                description: b"IRQ 11\0".as_ptr() as _,
                value: 11,
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

static ne2000_config: [device_config_t; 5] = [
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
                description: b"0x280\0".as_ptr() as _,
                value: 0x280,
            },
            device_config_selection_t {
                description: b"0x300\0".as_ptr() as _,
                value: 0x300,
            },
            device_config_selection_t {
                description: b"0x320\0".as_ptr() as _,
                value: 0x320,
            },
            device_config_selection_t {
                description: b"0x340\0".as_ptr() as _,
                value: 0x340,
            },
            device_config_selection_t {
                description: b"0x360\0".as_ptr() as _,
                value: 0x360,
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
        ],
    },
    device_config_t {
        name: "irq\0".as_ptr() as _,
        description: "IRQ\0".as_ptr() as _,
        r#type: CONFIG_SELECTION,
        default_string: "\0".as_ptr() as _,
        default_int: 10,
        file_filter: "\0".as_ptr() as _,
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
                description: b"IRQ 10\0".as_ptr() as _,
                value: 10,
            },
            device_config_selection_t {
                description: b"IRQ 11\0".as_ptr() as _,
                value: 11,
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
        name: b"bios_addr\0".as_ptr() as _,
        description: b"BIOS address\0".as_ptr() as _,
        r#type: CONFIG_HEX20,
        default_string: b"\0".as_ptr() as _,
        default_int: 0,
        file_filter: b"\0".as_ptr() as _,
        spinner: device_config_spinner_t {
            min: 0,
            max: 0,
            step: 0,
        },
        selection: [
            device_config_selection_t {
                description: b"Disabled\0".as_ptr() as _,
                value: 0x00000,
            },
            device_config_selection_t {
                description: b"D000\0".as_ptr() as _,
                value: 0xD0000,
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

static rtl8019as_config: [device_config_t; 2] = [
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

static rtl8029as_config: [device_config_t; 3] = [
    device_config_t {
        name: b"bios\0".as_ptr() as _,
        description: b"Enable BIOS\0".as_ptr() as _,
        r#type: CONFIG_BINARY,
        default_string: b"\0".as_ptr() as _,
        default_int: 0,
        ..device_config_t::empty()
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
pub static ne1000_device: device_t = device_t {
    name: b"Novell NE1000\0".as_ptr() as _,
    internal_name: b"ne1k\0".as_ptr() as _,
    flags: DEVICE_ISA as u32,
    local: NE2K_NE1000 as u32,
    init: Some(nic_init),
    close: Some(nic_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: ne1000_config.as_ptr(),
};

#[no_mangle]
pub static ne2000_device: device_t = device_t {
    name: b"Novell NE2000\0".as_ptr() as _,
    internal_name: b"ne2k\0".as_ptr() as _,
    flags: DEVICE_ISA as u32 | DEVICE_AT as u32,
    local: NE2K_NE2000 as u32,
    init: Some(nic_init),
    close: Some(nic_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: ne2000_config.as_ptr(),
};

#[no_mangle]
pub static ethernext_mc_device: device_t = device_t {
    name: b"NetWorth EtherNext/MC\0".as_ptr() as _,
    internal_name: b"ethernextmc\0".as_ptr() as _,
    flags: DEVICE_MCA as u32,
    local: NE2K_ETHERNEXT_MC as u32,
    init: Some(nic_init),
    close: Some(nic_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: mca_mac_config.as_ptr(),
};

#[no_mangle]
pub static rtl8019as_device: device_t = device_t {
    name: b"Realtek RTL8019AS\0".as_ptr() as _,
    internal_name: b"ne2kpnp\0".as_ptr() as _,
    flags: DEVICE_ISA as u32 | DEVICE_AT as u32,
    local: NE2K_RTL8019AS as u32,
    init: Some(nic_init),
    close: Some(nic_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: rtl8019as_config.as_ptr(),
};

#[no_mangle]
pub static rtl8029as_device: device_t = device_t {
    name: b"Realtek RTL8029AS\0".as_ptr() as _,
    internal_name: b"ne2kpci\0".as_ptr() as _,
    flags: DEVICE_PCI as u32,
    local: NE2K_RTL8029AS as u32,
    init: Some(nic_init),
    close: Some(nic_close),
    reset: None,
    device_t_union: device_t_union { available: None },
    speed_changed: None,
    force_redraw: None,
    config: rtl8029as_config.as_ptr(),
};
