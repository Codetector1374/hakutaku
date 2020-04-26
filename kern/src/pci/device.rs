use spin::Mutex;
use x86_64::instructions::port::{PortWriteOnly, Port};
use crate::pci::{PCIError, PCIDeviceInfo};
use x86_64::instructions::interrupts::without_interrupts;
use crate::pci::class::{HeaderType, PCIDeviceClass};
use x86_64::PhysAddr;

pub mod xhci;

const CONFIG_ADDR: u16 = 0x0CF8;
const CONFIG_DATA: u16 = 0x0CFC;

const CONFIG_BAR_BASE_REG: u8 = 0x04;

const PORTS: Mutex<PCIPorts> = Mutex::new(PCIPorts::new());

struct PCIPorts {
    addr: PortWriteOnly<u32>,
    data_port: Port<u32>,
}

impl PCIPorts {
    const fn new() -> PCIPorts {
        PCIPorts {
            addr: PortWriteOnly::new(CONFIG_ADDR),
            data_port: Port::new(CONFIG_DATA),
        }
    }
}

#[derive(Debug)]
pub struct PCIDevice {
    pub bus: u8,
    pub device_number: u8,
    pub func: u8,
}

impl PCIDevice {
    /// Slot number range from 0 to 31
    /// Func number range from 0 to 7
    pub fn new_unchecked(bus: u8, device_number: u8, func: u8) -> Result<PCIDevice, PCIError> {
        if device_number > 31 {
            Err(PCIError::SlotNumber)
        } else if func > 7 {
            Err(PCIError::FuncNumber)
        } else {
            Ok(PCIDevice {
                bus,
                device_number,
                func,
            })
        }
    }

    pub fn into_info(self) -> Option<PCIDeviceInfo> {
        match self.vendor_id() {
            Ok(_) => {
                let r_class = self.device_info();
                let class = PCIDeviceClass::from(r_class.0, r_class.1, r_class.2);
                let header = self.header_type();
                Some(PCIDeviceInfo {
                    device: self,
                    class,
                    rev: r_class.3,
                    header_type: header,
                })
            },
            _ => {
                None
            }
        }
    }

    /// Caller Guarantees that reg is within range 0-63
    pub fn get_addr(&self, reg: u8) -> u32 {
        0x8000_0000 // Enable Bit
            | (self.bus as u32) << 16 // Bus
            | ((self.device_number & 0x1F) as u32) << 11
            | ((self.func & 0x7) as u32) << 8
            | ((reg << 2) & 0xFC) as u32
    }

    /// Register number must be in range 0-63
    pub fn read_config_dword(&self, register_number: u8) -> Result<u32, PCIError> {
        if register_number > 63 {
            Err(PCIError::RegisterNumber)
        } else {
            Ok(without_interrupts(|| {
                unsafe {
                    PORTS.lock().addr.write(self.get_addr(register_number));
                    PORTS.lock().data_port.read()
                }
            }))
        }
    }

    pub fn write_config_word(&mut self, register_number: u8, value: u32) {
        without_interrupts(|| {
            unsafe {
                PORTS.lock().addr.write(self.get_addr(register_number));
                PORTS.lock().data_port.write(value);
            }
        })
    }

    pub fn base_mmio_address(&self, bar: u8) -> Option<PhysAddr> {
        /* Okay this is really complicated.
         * 1) Read the bar register
         * 2) Check bit 0, 1: I/O Space Mapped, 0: Memory Mapped
         * 3) If Memory: Check bit [2:1] => 00 : Must map below 4G
         *                               => 01 : Must below 1Meg
         *                               => 10 : Anywhere 64bit
         *        (Make sure you read the next bar to get the higher 32bits)
         * 4) The real memory address, you need to align by masking off
         *    bottom 4 bits. For real IO address, mask the bottom 2 bits off.
        */
        let bar0 = self.read_config_bar_register(bar);
        if bar0 & 0x1 != 0 {
            // IOSpace
            return None;
        } else {
            return match (bar0 >> 1) & 0b11 {
                0b10 => {
                    Some(PhysAddr::new((bar0 as u64) & !0b1111u64))
                },
                _ => {
                    if bar >= 5 {
                        panic!("Can't 64 bit on BAR5");
                    }
                    let bar1 = self.read_config_bar_register(bar + 1);
                    let mut phy = (bar1 as u64) << 32;
                    phy |= bar0 as u64;
                    Some(PhysAddr::new(phy & !0b1111u64))
                }
            }
        }

    }

    pub fn read_config_bar_register(&self, no: u8) -> u32 {
        self.read_config_dword(CONFIG_BAR_BASE_REG + no as u8).expect("die")
    }

    pub fn irq_line(&self) -> u8 {
        self.read_config_dword(0xF).expect("die") as u8
    }

    pub fn address_space_size(&mut self) -> usize {
        let old_value = self.read_config_bar_register(0);
        self.write_config_word(CONFIG_BAR_BASE_REG, 0xFFFF_FFFF);
        let new_val = self.read_config_bar_register(0);
        self.write_config_word(CONFIG_BAR_BASE_REG, old_value);
        (!new_val) as usize
    }

    pub fn vendor_id(&self) -> Result<u16, PCIError> {
        let config_word = self.read_config_dword(0).expect("0 should be valid");
        if (config_word >> 16) == 0xFFFF {
            Err(PCIError::InvalidDevice)
        } else {
            Ok(config_word as u16)
        }
    }

    pub fn device_id(&self) -> Result<u16, PCIError> {
        let config_word = self.read_config_dword(0).expect("0 should be valid");
        if (config_word >> 16) == 0xFFFF {
            Err(PCIError::InvalidDevice)
        } else {
            Ok((config_word >> 16) as u16)
        }
    }

    /// This function returns a quad of Class, SubClass, Prog IF, Revision ID
    /// Note: executing this on an invalid device is UB
    pub fn device_info(&self) -> (u8, u8, u8, u8) {
        let config_word = self.read_config_dword(2).unwrap();
        let cc = (config_word >> 24) as u8;
        let subclass = (config_word >> 16) as u8;
        let progif = (config_word >> 8) as u8;
        let rev = config_word as u8;
        (cc, subclass, progif, rev)
    }

    pub fn header_type(&self) -> HeaderType {
        let config_word = self.read_config_dword(3).unwrap();
        HeaderType::from((config_word >> 16) as u8)
    }

    /// The caller should check if device is a PCI Bridge
    pub(super) fn secondary_bus_number(&self) -> u8 {
        let read = self.read_config_dword(6).unwrap();
        (read >> 8) as u8
    }
}
