use crate::hardware::apic::APIC_MAP_TARGET_BASE;

// Timers
const APIC_OFFSET_TIMER_LVT: usize = 0x320;

const APIC_OFFSET_TIMER_INITIAL: usize = 0x380;
const APIC_OFFSET_TIMER_CURRENT: usize = 0x390;
const APIC_OFFSET_TIMER_DIVIDE: usize = 0x3E0;

#[repr(u8)]
#[derive(Debug)]
pub enum APICTimerDividerOption {
    DivideBy1 = 0b1011,
    DivideBy2 = 0b0000,
    DivideBy4 = 0b0001,
    DivideBy8 = 0b0010,
    DivideBy16 = 0b0011,
    DivideBy32 = 0b1000,
    DivideBy64 = 0b1001,
    DivideBy128 = 0b1010,
}

impl From<u8> for APICTimerDividerOption {
    fn from(val: u8) -> Self {
        use APICTimerDividerOption::*;
        match val {
            0b1011 => DivideBy1,
            0b0000 => DivideBy2,
            0b0001 => DivideBy4,
            0b0010 => DivideBy8,
            0b0011 => DivideBy16,
            0b1000 => DivideBy32,
            0b1001 => DivideBy64,
            0b1010 => DivideBy128,
            _ => panic!("invalid value")
        }
    }
}

pub fn read_divider() -> APICTimerDividerOption {
    let read = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_DIVIDE) as *const u32;
    let read = unsafe {
        *read
    };

    ((read & 0b1011) as u8).into()
}

pub fn set_divider(opt: APICTimerDividerOption) {
    let read = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_DIVIDE) as *mut u32;
    unsafe {
        *read = opt as u8 as u32;
    }
}

pub fn read_current_value() -> u32 {
    let read = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_CURRENT) as *const u32;
    unsafe {
        *read
    }
}

pub fn initial_value() -> u32 {
    let read = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_INITIAL) as *const u32;
    unsafe {
        *read
    }
}

pub fn set_initial_value(val: u32) {
    let read = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_INITIAL) as *mut u32;
    unsafe {
        *read = val
    };
}

#[repr(u8)]
pub enum APICTimerMode {
    OneShot = 0b00,
    Periodic = 0b01,
    Deadline = 0b10
}

pub fn get_timer_lvt() -> u32 {
    let lol = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_LVT) as *mut u32;
    unsafe { *lol }
}

pub fn set_timer_lvt(vector: u8, mode: APICTimerMode, masked: bool) {
    let value: u32 = vector as u32 | (mode as u32) << 17 | (if masked { 1u32 } else { 0u32 }) << 16 ;
    let lol = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_LVT) as *mut u32;
    unsafe { *lol = value}
}
