use x86_64::instructions::port::{PortWriteOnly, Port};
use x86_64::instructions::interrupts::without_interrupts;
use spin::Mutex;
use bitflags::_core::time::Duration;
use crate::interrupts::{PICS, InterruptIndex};

const PIT_CH0: u16 = 0x40;
const PIT_CH1: u16 = 0x41;
const PIT_CH2: u16 = 0x42;
const PIT_CMD: u16 = 0x43;

pub static GLOBAL_PIT: Mutex<PIT> = Mutex::new(PIT::new());

pub struct PIT {
    command: PortWriteOnly<u8>,
    channel0: Port<u8>,
    spin_flag: bool,
}

/*

Bits         Usage
 6 and 7      Select channel :
                 0 0 = Channel 0
                 0 1 = Channel 1
                 1 0 = Channel 2
                 1 1 = Read-back command (8254 only)
 4 and 5      Access mode :
                 0 0 = Latch count value command
                 0 1 = Access mode: lobyte only
                 1 0 = Access mode: hibyte only
                 1 1 = Access mode: lobyte/hibyte
 1 to 3       Operating mode :
                 0 0 0 = Mode 0 (interrupt on terminal count)
                 0 0 1 = Mode 1 (hardware re-triggerable one-shot)
                 0 1 0 = Mode 2 (rate generator)
                 0 1 1 = Mode 3 (square wave generator)
                 1 0 0 = Mode 4 (software triggered strobe)
                 1 0 1 = Mode 5 (hardware triggered strobe)
                 1 1 0 = Mode 2 (rate generator, same as 010b)
                 1 1 1 = Mode 3 (square wave generator, same as 011b)
 0            BCD/Binary mode: 0 = 16-bit binary, 1 = four-digit BCD

*/



impl PIT {
    const fn new() -> PIT {
        PIT {
            command: PortWriteOnly::new(PIT_CMD),
            channel0: Port::new(PIT_CH0),
            spin_flag: false,
        }
    }

    pub fn sw_oneshot(&mut self, value: u16) {
        without_interrupts(|| {
            unsafe {
                self.command.write(0b00111000); // Mode 4, ch0, lo/hi, binary
                self.channel0.write(value as u8); // LoByte
                self.channel0.write((value >> 8) as u8); // HiByte
            }
        });
    }

    pub fn interrupt(&mut self) {
        self.spin_flag = true;
    }
}

const MAX_SINGLE_WAIT_DURATION: Duration = Duration::from_millis(50);

// 1 tick is ~838 ns;
// 65535 tick -> 54918330
fn spin_wait_internal(ns: u32) {
    let ticks = ns / 838;
    if ticks < 1 {
        return;
    }
    assert!(ticks <= 65535);
    PICS.lock().mask_interrupt(InterruptIndex::Timer as u8);
    without_interrupts(||{
        GLOBAL_PIT.lock().spin_flag = false;
        GLOBAL_PIT.lock().sw_oneshot(ticks as u16);
    });
    PICS.lock().unmask_interrupt(InterruptIndex::Timer as u8);
    loop {
        let current_flag = without_interrupts(|| {
            GLOBAL_PIT.lock().spin_flag
        });
        if current_flag {
            break;
        }
        x86_64::instructions::hlt();
    }
}

pub fn spin_wait(d: Duration) {
    let mut d_remain = d;
    while d_remain.as_nanos() > 0 {
        let delay = if d_remain > MAX_SINGLE_WAIT_DURATION {
            MAX_SINGLE_WAIT_DURATION
        } else {
            d_remain
        };
        spin_wait_internal(delay.as_nanos() as u32);
        d_remain -= delay;
    }
}