use spin::Mutex;
use x86_64::instructions::port::{PortWriteOnly, Port};
use crate::pci::PCIError;
use x86_64::instructions::interrupts::without_interrupts;
use crate::pci::class::HeaderType;

const CONFIG_ADDR: u16 = 0x0CF8;
const CONFIG_DATA: u16 = 0x0CFC;

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
