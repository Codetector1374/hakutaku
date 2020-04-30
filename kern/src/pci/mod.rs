use crate::pci::device::PCIDevice;
use crate::pci::class::{PCIDeviceClass, HeaderType};
use alloc::vec::Vec;
use alloc::alloc::handle_alloc_error;
use spin::Mutex;

pub mod device;
pub mod class;

pub static GLOBAL_PCI: Mutex<PCIController> = Mutex::new(PCIController {});

pub struct PCIController {}

#[derive(Debug)]
pub enum PCIError {
    SlotNumber,
    FuncNumber,
    RegisterNumber,
    InvalidDevice,
}

#[derive(Debug, Copy, Clone)]
pub enum PCICapabilityID {
    PowerManagement,
    AGP,
    VPD,
    SlotID,
    MSI,
    CompactPCIHotSwap,
    PCIX,
    HyperTrasnport,
    Vendor,
    Debug,
    CompactPCICentralResourceCtrl,
    PCIHotPlug,
    PCIBridgeSubsystemVendorID,
    AGP8x,
    SecureDevice,
    PCIExpress,
    MSIX,
    Unknown(u8),
}

impl From<u8> for PCICapabilityID {
    fn from(id: u8) -> Self {
        use PCICapabilityID::*;
        match id {
            0x1 => PowerManagement,
            0x2 => AGP,
            0x3 => VPD,
            0x4 => SlotID,
            0x5 => MSI,
            0x6 => CompactPCIHotSwap,
            0x7 => PCIX,
            0x8 => HyperTrasnport,
            0x9 => Vendor,
            0xa => Debug,
            0xb => CompactPCICentralResourceCtrl,
            0xc => PCIHotPlug,
            0xd => PCIBridgeSubsystemVendorID,
            0xe => AGP8x,
            0xf => SecureDevice,
            0x10 => PCIExpress,
            0x11 => MSIX,
            _ => Unknown(id),
        }
    }
}

#[derive(Debug)]
pub struct PCICapability {
    pub id: PCICapabilityID,
    /// Byte based address
    pub addr: u8,
}

impl PCICapability {
    pub fn new(id: u8, addr: u8) -> PCICapability {
        PCICapability {
            id: PCICapabilityID::from(id),
            addr,
        }
    }
}

#[derive(Debug)]
pub struct PCIDeviceInfo {
    pub class: PCIDeviceClass,
    pub rev: u8,
    pub vendor_id: u16,
    pub device_id: u16,
    pub header_type: HeaderType,
    pub capabilities: Vec<PCICapability>,
}

impl PCIDeviceInfo {
    pub fn new(class_code: u32, rev: u8, vendor_id: u16, device_id: u16, header_type: u8) -> PCIDeviceInfo {
        let class = PCIDeviceClass::from(class_code);
        PCIDeviceInfo {
            class,
            rev,
            vendor_id,
            device_id,
            header_type: HeaderType::from(header_type),
            capabilities: Vec::default(),
        }
    }
}

impl PCIController {
    pub fn enumerate_pci_bus(&self) -> Vec<PCIDevice> {
        let mut bus = Vec::<PCIDevice>::with_capacity(16);
        self.enumerate_bus(0, &mut bus);
        bus
    }

    fn enumerate_bus(&self, bus: u8, vec: &mut Vec<PCIDevice>) {
        for device_id in 0..32 {
            self.check_device(bus, device_id, vec);
        }
    }

    fn check_device(&self, bus: u8, device: u8, vec: &mut Vec<PCIDevice>) {
        if self.check_function(bus, device, 0, vec) {
            for func in 1..8 {
                self.check_function(bus, device, func, vec);
            }
        }
    }

    /// Returns: isMultiFunction
    fn check_function(&self, bus: u8, device: u8, func: u8, vec: &mut Vec<PCIDevice>) -> bool {
        let dev = PCIDevice::new(bus, device, func);
        match dev {
            Some(d) => {
                let mf = d.info.header_type.is_multi_function();
                vec.push(d);
                mf
            }
            None => false
        }
    }
}

