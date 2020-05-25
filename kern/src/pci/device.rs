use spin::Mutex;
use x86_64::instructions::port::{PortWriteOnly, Port};
use crate::pci::{PCIError, PCIDeviceInfo, PCICapability};
use x86_64::instructions::interrupts::without_interrupts;
use crate::pci::class::{HeaderType, PCIDeviceClass};
use x86_64::PhysAddr;
use alloc::vec::Vec;
use alloc::string::String;

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

    fn get_config_addr(bus: u8, dev: u8, func: u8, offset: u8) -> u32 {
        0x8000_0000 // Enable Bit
            | (bus as u32) << 16 // Bus
            | ((dev & 0x1F) as u32) << 11
            | ((func & 0x7) as u32) << 8
            | ((offset << 2) & 0xFC) as u32
    }

    pub fn read_config_dword(&mut self, bus: u8, dev: u8, func: u8, offset: u8) -> u32 {
        without_interrupts(|| {
            unsafe {
                self.addr.write(Self::get_config_addr(bus, dev, func, offset));
                self.data_port.read()
            }
        })
    }

    pub fn write_config_dword(&mut self, bus: u8, dev: u8, func: u8, offset: u8, value: u32) {
        without_interrupts(|| {
            unsafe {
                self.addr.write(Self::get_config_addr(bus, dev, func, offset));
                self.data_port.write(value);
            }
        })
    }
}

#[derive(Debug, Clone)]
pub struct PCIDevice {
    pub bus: u8,
    pub device_number: u8,
    pub func: u8,
    pub info: PCIDeviceInfo,
}


impl PCIDevice {
    /// Slot number range from 0 to 31
    /// Func number range from 0 to 7
    pub fn new(bus: u8, dev: u8, func: u8) -> Option<PCIDevice> {
        if dev < 32 && func < 8 {
            let id_vid = PORTS.lock().read_config_dword(bus, dev, func, 0);
            let vid = id_vid as u16;
            let id = (id_vid >> 16) as u16;
            if vid == 0xFFFF {
                return None; // Device is empty
            }

            // Class Code & Rev
            let class_rev = PORTS.lock().read_config_dword(bus, dev, func, 2);
            let class = (class_rev >> 8) as u32;
            let rev = class_rev as u8;

            // HeaderType
            let header_type = (PORTS.lock().read_config_dword(bus, dev, func, 3) >> 16) as u8;

            let info = PCIDeviceInfo::new(class, rev, vid, id, header_type);

            let mut dev = PCIDevice {
                bus,
                device_number: dev,
                func,
                info,
            };
            dev.populate_capability();
            Some(dev)
        } else {
            None
        }
    }

    pub fn get_int_line(&self) -> u8 {
        let word = self.read_config_word(0x3c);
        word as u8
    }

    pub fn bus_location_str(&self) -> String {
        format!("{:04x}:{:02x}:{:02x}", self.bus, self.device_number, self.func)
    }

    fn populate_capability(&mut self) {
        let status = self.read_status();
        let ext_cap = (status >> 4) & 0x1 == 1;
        trace!("[PCI] has Capability List: {}", ext_cap);
        if ext_cap {
            let cap_offset = self.read_config_dword_dep(13) as u8;
            trace!("[PCI] Cap Offset {} ({})", cap_offset, cap_offset / 4);
            let mut next_offset = cap_offset;
            loop {
                if next_offset == 0 {
                    break;
                }
                assert_eq!(next_offset % 4, 0, "register alignment");
                let word = self.read_config_dword_dep(next_offset / 4);
                let capid = word as u8;
                let cap = PCICapability::new(capid, next_offset);
                trace!("[PCI] Detected Capability {:?}", &cap.id);
                self.info.capabilities.push(cap);
                next_offset = (word >> 8) as u8;
            }
        }
    }

    pub fn read_status(&self) -> u16 {
        (self.read_config_dword_dep(1) >> 16) as u16
    }

    pub fn read_config_dword(&self, offset: u8) -> u32 {
        PORTS.lock().read_config_dword(self.bus, self.device_number, self.func, offset / 4)
    }

    pub fn read_config_word(&self, offset: u8) -> u16 {
        let val = self.read_config_dword(offset);
        if offset % 4 >= 2 {
            (val >> 16) as u16
        } else {
            val as u16
        }
    }

    pub fn write_config_dword(&mut self, offset: u8, value: u32) {
        PORTS.lock().write_config_dword(self.bus, self.device_number, self.func, offset / 4, value);
    }

    pub fn write_config_word(&mut self, offset: u8, val: u16) {
        let old = self.read_config_dword(offset);
        let new = if offset % 4 >= 2 {
            (old & 0x00FF) | ((val as u32) << 16)
        } else {
            (old & 0xFF00) | val as u32
        };
        self.write_config_dword(offset, new);
    }

    /// Register number must be in range 0-63
    #[deprecated]
    pub fn read_config_dword_dep(&self, register_number: u8) -> u32 {
        self.read_config_dword(register_number * 4)
    }

    pub fn write_config_dword_dep(&mut self, register_number: u8, value: u32) {
        self.write_config_dword(register_number * 4, value);
    }

    pub fn base_mmio_address(&self, bar: u8) -> Option<PhysAddr> {
        /* Okay this is complicated.
         * 1) Read the bar register
         * 2) Check bit 0, 1: I/O Space Mapped, 0: Memory Mapped
         * 3) If Memory: Check bit [2:1] => 00 : Must map below 4G
         *                               => 01 : Must below 1Meg
         *        (Make sure you read the next bar to get the higher 32bits)
         * 4) The real memory address, you need to align by masking off
         *    bottom 4 bits. For real IO address, mask the bottom 2 bits off.
        */
        let bar0 = self.read_config_bar_register(bar);
        if bar0 & 0x1 != 0 {
            // IOSpace
            None
        } else {
            match (bar0 >> 1) & 0b11 {
                0b10 => {
                    Some(PhysAddr::new((bar0 as u64) & !0b1111u64))
                }
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
        self.read_config_dword_dep(CONFIG_BAR_BASE_REG + no as u8)
    }

    pub fn irq_line(&self) -> u8 {
        self.read_config_dword_dep(0xF) as u8
    }

    pub fn address_space_size(&mut self) -> usize {
        let old_value = self.read_config_bar_register(0);
        self.write_config_dword_dep(CONFIG_BAR_BASE_REG, 0xFFFF_FFFF);
        let new_val = self.read_config_bar_register(0);
        self.write_config_dword_dep(CONFIG_BAR_BASE_REG, old_value);
        (!new_val) as usize
    }

    /// This function returns a quad of Class, SubClass, Prog IF, Revision ID
    /// Note: executing this on an invalid device is UB
    pub fn device_info(&self) -> (u8, u8, u8, u8) {
        let config_word = self.read_config_dword_dep(2);
        let cc = (config_word >> 24) as u8;
        let subclass = (config_word >> 16) as u8;
        let progif = (config_word >> 8) as u8;
        let rev = config_word as u8;
        (cc, subclass, progif, rev)
    }

    pub fn header_type(&self) -> HeaderType {
        let config_word = self.read_config_word(super::consts::CONF_HEADER_TYPE_OFFSET);
        HeaderType::from(config_word as u8)
    }

    /// The caller should check if device is a PCI Bridge
    pub(super) fn secondary_bus_number(&self) -> u8 {
        let read = self.read_config_word(super::consts::CONF_SECONDARY_BUS_OFFSET);
        debug!("Secondary Raw: {:#x}", read);
        (read >> 8) as u8
    }

    pub(super) fn set_secondary_bus_number(&mut self, n: u8) {
        let read = self.read_config_word(super::consts::CONF_SECONDARY_BUS_OFFSET) & 0x00FF;
        self.write_config_word(super::consts::CONF_SECONDARY_BUS_OFFSET, read | (n as u16) << 8);
    }
}

impl PartialEq for PCIDevice {
    fn eq(&self, other: &Self) -> bool {
        self.bus == other.bus
            && self.device_number == other.device_number
            && self.func == other.func
    }
}
