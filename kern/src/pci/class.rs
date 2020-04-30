use crate::pci::class::PCIDeviceClass::{Other, SerialBusController};
use crate::pci::class::PCISerialBusController::{FireWire, ACCESSBus, SSA, USBController, FibreChannel};
use crate::pci::class::PCISerialBusUSB::{UHCI, OHCI, EHCI, XHCI, Device};
use cpuio::UnsafePort;
use crate::pci::class::HeaderType::{RegularDevice, PCIBridge, OtherHeaderType};


#[derive(Debug, Clone, Copy)]
pub enum HeaderType {
    RegularDevice(u8),
    PCIBridge(u8),
    OtherHeaderType(u8)
}

impl HeaderType {
    pub fn is_multi_function(&self) -> bool {
        match self {
            RegularDevice(a) => {
                a & 0x80 != 0
            },
            PCIBridge(a) => {
                a & 0x80 != 0
            },
            OtherHeaderType(a) => {
                a & 0x80 != 0
            },
        }
    }
}

impl From<u8> for HeaderType {
    fn from(header: u8) -> Self {
        match header & 0x7F {
            0x0 => RegularDevice(header),
            0x1 => PCIBridge(header),
            _ => OtherHeaderType(header)
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum PCIDeviceClass {
    Unclassified(u8, u8),
    MassStorageController(PCIClassMassStorage),
    NetworkController(u8, u8),
    SerialBusController(PCISerialBusController),
    Other(u8, u8, u8)
}

impl PCIDeviceClass {
    pub fn from(class_group: u32) -> PCIDeviceClass {
        let class = (class_group >> 16) as u8;
        let sub = (class_group >> 8) as u8;
        let progif = class_group as u8;
        match class {
            0x0C => SerialBusController(PCISerialBusController::from(sub, progif)),
            _ => Other(class, sub, progif)
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum PCIClassMassStorage {
    /// Prog If
    IDEController(u8),
    FloppyDiskController,
    IPIBusController,
    RAIDController,
    ATAController(u8),
    SATA(u8),
    SAS(u8),
    NonVolatileMemory(u8),
    Other(u8),
    Unknown(u8, u8)
}

#[derive(Debug, Clone, Copy)]
pub enum PCIClassNetworkController {
    Ethernet,
    TokenRing,
    FDDI,
    ATM,
    ISDN,
    WorldFip,
    PICMG,
    Infiniband,
    Fabric,
    Other(u8, u8)
}

#[derive(Debug, Clone, Copy)]
pub enum PCIClassDisplayController {
    VGACompatible(u8),
    XGA,
    NonVGACompatible3D,
    Other(u8, u8)
}

#[derive(Debug, Clone, Copy)]
pub enum PCISerialBusController {
    FireWire(u8),
    ACCESSBus,
    SSA,
    USBController(PCISerialBusUSB),
    FibreChannel,
    SMBus,
    InfiniBand,
    IPMIInterface(u8),
    CAN,
    Other(u8, u8)
}

impl PCISerialBusController {
    pub fn from(sub: u8, progif: u8) -> PCISerialBusController{
        match sub {
            0x0 => FireWire(progif),
            0x1 => ACCESSBus,
            0x2 => SSA,
            0x3 => USBController(progif.into()),
            0x4 => FibreChannel,
            _ => PCISerialBusController::Other(sub, progif)
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum PCISerialBusUSB {
    UHCI,
    OHCI,
    EHCI,
    XHCI,
    Other(u8),
    Device
}

impl From<u8> for PCISerialBusUSB {
    fn from(progif: u8) -> Self {
        match progif {
            0x0 => UHCI,
            0x10 => OHCI,
            0x20 => EHCI,
            0x30 => XHCI,
            0xFE => Device,
            _ => PCISerialBusUSB::Other(progif)
        }
    }
}