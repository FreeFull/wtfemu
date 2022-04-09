use crate::prelude::*;

// ui.h

pub const MBX_INFO: c_int = 1;
pub const MBX_ERROR: c_int = 2;
pub const MBX_QUESTION: c_int = 3;
pub const MBX_QUESTION_YN: c_int = 4;
pub const MBX_QUESTION_OK: c_int = 8;
pub const MBX_QMARK: c_int = 0x10;
pub const MBX_WARNING: c_int = 0x20;
pub const MBX_FATAL: c_int = 0x40;
pub const MBX_ANSI: c_int = 0x80;
pub const MBX_LINKS: c_int = 0x100;
pub const MBX_DONTASK: c_int = 0x200;

extern "C" {
    pub fn ui_msgbox(flags: c_int, message: *mut c_void) -> c_int;
    pub fn ui_msgbox_header(flags: c_int, header: *mut c_void, message: *mut c_void) -> c_int;
    pub fn ui_msgbox_ex(
        flags: c_int,
        header: *mut c_void,
        message: *mut c_void,
        btn1: *mut c_void,
        btn2: *mut c_void,
        btn3: *mut c_void,
    ) -> c_int;

    pub fn ui_check_menu_item(id: c_int, checked: c_int);
}

pub const SB_ICON_WIDTH: c_int = 24;
pub const SB_CASSETTE: c_int = 0x00;
pub const SB_CARTRIDGE: c_int = 0x10;
pub const SB_FLOPPY: c_int = 0x20;
pub const SB_CDROM: c_int = 0x30;
pub const SB_ZIP: c_int = 0x40;
pub const SB_MO: c_int = 0x50;
pub const SB_HDD: c_int = 0x60;
pub const SB_NETWORK: c_int = 0x70;
pub const SB_SOUND: c_int = 0x80;
pub const SB_TEXT: c_int = 0x90;

extern "C" {
    pub fn ui_sb_update_icon(tag: c_int, val: c_int);
}
