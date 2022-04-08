use libc::*;

// device.h

pub const CONFIG_END: c_int = -1;
pub const CONFIG_STRING: c_int = 0;
pub const CONFIG_INT: c_int = 1;
pub const CONFIG_BINARY: c_int = 2;
pub const CONFIG_SELECTION: c_int = 3;
pub const CONFIG_MIDI_OUT: c_int = 4;
pub const CONFIG_FNAME: c_int = 5;
pub const CONFIG_SPINNER: c_int = 6;
pub const CONFIG_HEX16: c_int = 7;
pub const CONFIG_HEX20: c_int = 8;
pub const CONFIG_MAC: c_int = 9;
pub const CONFIG_MIDI_IN: c_int = 10;

pub const DEVICE_NOT_WORKING: c_int = 1; /* does not currently work correctly and will be disabled in a release build */
pub const DEVICE_PCJR: c_int = 2; /* requires an IBM PCjr */
pub const DEVICE_AT: c_int = 4; /* requires an AT-compatible system */
pub const DEVICE_PS2: c_int = 8; /* requires a PS/1 or PS/2 system */
pub const DEVICE_ISA: c_int = 0x10; /* requires the ISA bus */
pub const DEVICE_CBUS: c_int = 0x20; /* requires the C-BUS bus */
pub const DEVICE_MCA: c_int = 0x40; /* requires the MCA bus */
pub const DEVICE_EISA: c_int = 0x80; /* requires the EISA bus */
pub const DEVICE_VLB: c_int = 0x100; /* requires the PCI bus */
pub const DEVICE_PCI: c_int = 0x200; /* requires the VLB bus */
pub const DEVICE_AGP: c_int = 0x400; /* requires the AGP bus */
pub const DEVICE_AC97: c_int = 0x800; /* requires the AC'97 bus */
pub const DEVICE_COM: c_int = 0x1000; /* requires a serial port */
pub const DEVICE_LPT: c_int = 0x2000; /* requires a parallel port */

#[repr(C)]
pub struct device_config_selection_t {
    description: *const c_char,
    value: c_int,
}
#[repr(C)]

pub struct device_config_spinner_t {
    min: i16,
    max: i16,
    step: i16,
}
#[repr(C)]

pub struct device_config_t {
    name: *const c_char,
    description: *const c_char,
    r#type: c_int,
    default_string: *const c_char,
    default_int: c_int,
    file_filter: *const c_char,
    spinner: device_config_spinner_t,
    selection: [device_config_selection_t; 16],
}

#[repr(C)]
pub union device_t_union {
    available: extern "C" fn() -> c_int,
    poll: extern "C" fn(x: c_int, y: c_int, z: c_int, b: c_int, r#priv: *mut c_void) -> c_int,
    register_pci_slot: extern "C" fn(
        device: c_int,
        r#type: c_int,
        inta: c_int,
        intb: c_int,
        intc: c_int,
        intd: c_int,
        r#priv: *mut c_void,
    ),
}

#[repr(C)]
pub struct device_t {
    pub name: *const c_char,
    pub internal_name: *const c_char,
    pub flags: u32, /* system flags */
    pub local: u32, /* flags local to device */

    pub init: extern "C" fn(*const device_t) -> *mut c_void,
    pub close: extern "C" fn(r#priv: *mut c_void),
    pub reset: extern "C" fn(r#priv: *mut c_void),

    pub device_t_union: device_t_union,

    pub speed_changed: extern "C" fn(r#priv: *mut c_void),
    pub force_redraw: extern "C" fn(r#priv: *mut c_void),

    pub config: *const device_config_t,
}

#[repr(C)]
pub struct device_context_t {
    dev: *const device_t,
    name: [c_char; 2048],
}

extern "C" {

    pub fn device_init();
    pub fn device_set_context(c: *mut device_context_t, d: *const device_t, inst: c_int);
    pub fn device_context(d: *const device_t);
    pub fn device_context_inst(d: *const device_t, inst: c_int);
    pub fn device_context_restore();
    pub fn device_add(d: *const device_t) -> *mut c_void;
    pub fn device_add_ex(d: *const device_t, r#priv: *mut c_void);
    pub fn device_add_inst(d: *const device_t, inst: c_int) -> *mut c_void;
    pub fn device_add_inst_ex(d: *const device_t, r#priv: *mut c_void, inst: c_int);
    pub fn device_cadd(d: *const device_t, cd: *const device_t) -> *mut c_void;
    pub fn device_cadd_ex(d: *const device_t, cd: *const device_t, r#priv: *mut c_void);
    pub fn device_cadd_inst(d: *const device_t, cd: *const device_t, inst: c_int) -> *mut c_void;
    pub fn device_cadd_inst_ex(
        d: *const device_t,
        cd: *const device_t,
        r#priv: *mut c_void,
        inst: c_int,
    );
    pub fn device_close_all();
    pub fn device_reset_all();
    pub fn device_reset_all_pci();
    pub fn device_get_priv(d: *const device_t) -> *mut c_void;
    pub fn device_available(d: *const device_t) -> c_int;
    pub fn device_poll(d: *const device_t, x: c_int, y: c_int, z: c_int, b: c_int) -> c_int;
    pub fn device_register_pci_slot(
        d: *const device_t,
        device: c_int,
        r#type: c_int,
        inta: c_int,
        intb: c_int,
        intc: c_int,
        intd: c_int,
    );
    pub fn device_speed_changed();
    pub fn device_force_redraw();
    pub fn device_get_name(d: *const device_t, bus: c_int, name: *mut c_char);
    pub fn device_has_config(d: *const device_t) -> c_int;

    pub fn device_is_valid(d: *const device_t, m: c_int) -> c_int;
    pub fn device_get_config_int(name: *const c_char) -> c_int;
    pub fn device_get_config_int_ex(s: *const c_char, dflt_int: c_int) -> c_int;
    pub fn device_get_config_hex16(name: *const c_char) -> c_int;
    pub fn device_get_config_hex20(name: *const c_char) -> c_int;
    pub fn device_get_config_mac(name: *const c_char, dflt_int: c_int) -> c_int;
    pub fn device_set_config_int(s: *const c_char, val: c_int);
    pub fn device_set_config_hex16(s: *const c_char, val: c_int);
    pub fn device_set_config_hex20(s: *const c_char, val: c_int);
    pub fn device_set_config_mac(s: *const c_char, val: c_int);
    pub fn device_get_config_string(name: *const c_char) -> *const c_char;

    pub fn device_get_internal_name(d: *const device_t) -> *mut c_char;

    pub fn machine_get_config_int(s: *mut c_char) -> c_int;
    pub fn machine_get_config_string(s: *mut c_char) -> *mut c_char;

}
