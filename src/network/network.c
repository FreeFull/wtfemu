/*
 * VARCem        Virtual ARchaeological Computer EMulator.
 *                An emulator of (mostly) x86-based PC systems and devices,
 *                using the ISA,EISA,VLB,MCA  and PCI system buses, roughly
 *                spanning the era between 1981 and 1995.
 *
 *                This file is part of the VARCem Project.
 *
 *                Implementation of the network module.
 *
 * NOTE                The definition of the netcard_t is currently not optimal;
 *                it should be malloc'ed and then linked to the NETCARD def.
 *                Will be done later.
 *
 *
 *
 * Author:        Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *                Copyright 2017-2019 Fred N. van Kempen.
 *
 *                Redistribution and  use  in source  and binary forms, with
 *                or  without modification, are permitted  provided that the
 *                following conditions are met:
 *
 *                1. Redistributions of  source  code must retain the entire
 *                   above notice, this list of conditions and the following
 *                   disclaimer.
 *
 *                2. Redistributions in binary form must reproduce the above
 *                   copyright  notice,  this list  of  conditions  and  the
 *                   following disclaimer in  the documentation and/or other
 *                   materials provided with the distribution.
 *
 *                3. Neither the  name of the copyright holder nor the names
 *                   of  its  contributors may be used to endorse or promote
 *                   products  derived from  this  software without specific
 *                   prior written permission.
 *
 * THIS SOFTWARE  IS  PROVIDED BY THE  COPYRIGHT  HOLDERS AND CONTRIBUTORS
 * "AS IS" AND  ANY EXPRESS  OR  IMPLIED  WARRANTIES,  INCLUDING, BUT  NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE  ARE  DISCLAIMED. IN  NO  EVENT  SHALL THE COPYRIGHT
 * HOLDER OR  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL,  EXEMPLARY,  OR  CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES;  LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON  ANY
 * THEORY OF  LIABILITY, WHETHER IN  CONTRACT, STRICT  LIABILITY, OR  TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING  IN ANY  WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdarg.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <time.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/plat.h>
#include <86box/ui.h>
#include <86box/network.h>
#include <86box/net_3c503.h>
#include <86box/net_ne2000.h>
#include <86box/net_pcnet.h>
#include <86box/net_plip.h>
#include <86box/net_wd8003.h>


const device_t net_none_device = {
    .name = "None",
    .internal_name = "none",
    .flags = 0,
    .local = NET_TYPE_NONE,
    .init = NULL,
    .close = NULL,
    .reset = NULL,
    { .available = NULL },
    .speed_changed = NULL,
    .force_redraw = NULL,
    .config = NULL
};


netcard_t net_cards[] = {
// clang-format off
    { &net_none_device,           NULL },
    { &threec503_device,          NULL },
    { &pcnet_am79c960_device,     NULL },
    { &pcnet_am79c961_device,     NULL },
    { &ne1000_device,             NULL },
    { &ne2000_device,             NULL },
    { &pcnet_am79c960_eb_device,  NULL },
    { &rtl8019as_device,          NULL },
    { &wd8003e_device,            NULL },
    { &wd8003eb_device,           NULL },
    { &wd8013ebt_device,          NULL },
    { &plip_device,               NULL },
    { &ethernext_mc_device,       NULL },
    { &wd8003eta_device,          NULL },
    { &wd8003ea_device,           NULL },
    { &pcnet_am79c973_device,     NULL },
    { &pcnet_am79c970a_device,    NULL },
    { &rtl8029as_device,          NULL },
    { &pcnet_am79c960_vlb_device, NULL },
    { NULL,                       NULL }
// clang-format off
};


/* Global variables. */
int                network_type;
int                network_ndev;
int                network_card;
char                network_host[522];
netdev_t        network_devs[32];
int                network_rx_pause = 0;