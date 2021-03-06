use x86_64::instructions::port::{PortWriteOnly, Port};
use x86_64::instructions::interrupts::without_interrupts;
use spin::{Mutex, RwLock};
use core::time::Duration;
use crate::interrupts::{PICS, InterruptIndex};
use core::sync::atomic::{AtomicU64, Ordering};

const PIT_CH0: u16 = 0x40;
const PIT_CH1: u16 = 0x41;
const PIT_CH2: u16 = 0x42;
const PIT_CMD: u16 = 0x43;

pub static GLOBAL_PIT: RwLock<PIT> = RwLock::new(PIT::new());
pub static SYSTEM_TIME: AtomicU64 = AtomicU64::new(0);

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum PITMode {
    TerminalCount = 0x0,
    HWOneShot = 0x1,
    RateGenerator = 0x2,
    SquareWave = 0x3,
    SWStrobe = 0x4,
    HWStrobe = 0x5,
}

pub struct PIT {
    command: PortWriteOnly<u8>,
    channel0: Port<u8>,
    mode: PITMode,
    interval: Duration,
    time: Duration,
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
            mode: PITMode::TerminalCount,
            interval: Duration::from_nanos(0),
            time: Duration::from_nanos(0),
        }
    }

    fn setup(&mut self, interval: Duration, mode: PITMode) {
        let value = interval.as_nanos() / 838;
        assert!(value < 65535);
        assert!(value >= 1);
        let value = value as u16;
        without_interrupts(|| {
            self.mode = mode;
            self.interval = interval;
            unsafe {
                self.command.write(0b00110000 | ((mode as u8) << 1)); // Mode 4, ch0, lo/hi, binary
                self.channel0.write(value as u8); // LoByte
                self.channel0.write((value >> 8) as u8); // HiByte
            }
        });
    }

    fn read(&mut self) -> u16 {
        without_interrupts(|| {
            let mut val: u16;
            unsafe {
                self.command.write(0b0);
                val = self.channel0.read() as u16;
                val |= (self.channel0.read() as u16) << 8;
            }
            val
        })
    }

    pub fn start_clock(&mut self) {
        self.setup(crate::config::SYSTEM_TIME_RESOLUTION, PITMode::SquareWave);
    }

    pub fn interrupt(&self) {
        SYSTEM_TIME.fetch_add(self.interval.as_millis() as u64, Ordering::Acquire);
    }

    pub fn current_time() -> Duration {
        Duration::from_millis(SYSTEM_TIME.load(Ordering::Relaxed))
    }
}

const MAX_SINGLE_WAIT_DURATION: Duration = Duration::from_millis(50);

// 1 tick is ~838 ns;
// 65535 tick -> 54918330
fn spin_wait_internal(d: Duration) {
    without_interrupts(||{
        let mut pit = GLOBAL_PIT.write();
        pit.setup(d, PITMode::SWStrobe);
        let value = pit.read();
        loop {
            let new_val = pit.read();
            if new_val > value {
                break;
            }
        }
    });
}

pub fn spin_wait(d: Duration) {
    let mut d_remain = d;
    while d_remain.as_nanos() > 0 {
        let delay = if d_remain > MAX_SINGLE_WAIT_DURATION {
            MAX_SINGLE_WAIT_DURATION
        } else {
            d_remain
        };
        spin_wait_internal(delay);
        d_remain -= delay;
    }
}