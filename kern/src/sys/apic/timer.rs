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

#[repr(u8)]
pub enum APICTimerMode {
    OneShot = 0b00,
    Periodic = 0b01,
    Deadline = 0b10
}


// pub fn read_current_value() -> u32 {
//     let read = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_CURRENT) as *const u32;
//     unsafe {
//         *read
//     }
// }

// pub fn initial_value() -> u32 {
//     let read = (APIC_MAP_TARGET_BASE + APIC_OFFSET_TIMER_INITIAL) as *const u32;
//     unsafe {
//         *read
//     }
// }


