use super::class::HeaderType::*;
use cpuio::UnsafePort;

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
    MassStorageController(PCIClassMassStorageClass),
    NetworkController(u8, u8),
    SerialBusController(PCISerialBusControllerClass),
    BridgeDevice(PCIBridgeDeviceClass),
    SimpleCommunicationController(PCISimpleCommunicationControllerClass),
    Other(u8, u8, u8)
}

impl PCIDeviceClass {
    pub fn from(class_group: u32) -> PCIDeviceClass {
        use self::PCIDeviceClass::*;

        let class = (class_group >> 16) as u8;
        let sub = (class_group >> 8) as u8;
        let progif = class_group as u8;
        match class {
            0x01 => MassStorageController(PCIClassMassStorageClass::from(sub, progif)),
            0x02 => NetworkController(sub, progif),
            0x06 => BridgeDevice(PCIBridgeDeviceClass::from(sub, progif)),
            0x07 => SimpleCommunicationController(PCISimpleCommunicationControllerClass::from(sub, progif)),
            0x0C => SerialBusController(PCISerialBusControllerClass::from(sub, progif)),
            _ => Other(class, sub, progif)
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum PCIBridgeDeviceClass {
    HostBridge,
    ISABridge,
    PCItoPCIBridge(u8),
    PCItoPCIHostBridge(u8),
    Other(u8, u8)
}

impl PCIBridgeDeviceClass {
    pub fn from(sub: u8, progif: u8) -> Self {
        use self::PCIBridgeDeviceClass::*;
        match sub {
            0x0 => HostBridge,
            0x1 => ISABridge,
            0x4 => PCItoPCIBridge(progif),
            0x9 => PCItoPCIHostBridge(progif),
            _ => Other(sub, progif),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum PCIClassMassStorageClass {
    /// Prog If
    IDEController(u8),
    FloppyDiskController,
    IPIBusController,
    RAIDController,
    ATAController(u8),
    SATA(PCIClassMassStroageSATA),
    SAS(u8),
    NonVolatileMemory(u8),
    Other(u8),
    Unknown(u8, u8)
}

#[derive(Debug, Clone, Copy)]
pub enum PCIClassMassStroageSATA {
    AHCI,
    Other(u8)
}

impl PCIClassMassStroageSATA {
    pub fn from(prog_if: u8) -> PCIClassMassStroageSATA {
        match prog_if {
            0x1 => PCIClassMassStroageSATA::AHCI,
            _ => PCIClassMassStroageSATA::Other(prog_if),
        }
    }
}

impl PCIClassMassStorageClass {
    pub fn from(sub: u8, prog_if: u8) -> PCIClassMassStorageClass {
        match sub {
            0x06 => PCIClassMassStorageClass::SATA(PCIClassMassStroageSATA::from(prog_if)),
            _ => PCIClassMassStorageClass::Unknown(sub, prog_if),
        }
    }
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
pub enum PCISerialBusControllerClass {
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

impl PCISerialBusControllerClass {
    pub fn from(sub: u8, progif: u8) -> PCISerialBusControllerClass {
        use self::PCISerialBusControllerClass::*;

        match sub {
            0x0 => FireWire(progif),
            0x1 => ACCESSBus,
            0x2 => SSA,
            0x3 => USBController(progif.into()),
            0x4 => FibreChannel,
            _ => PCISerialBusControllerClass::Other(sub, progif)
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
        use self::PCISerialBusUSB::*;

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

#[derive(Debug, Copy, Clone)]
pub enum PCISimpleCommunicationControllerClass {
    SerialController(u8),
    ParallelController(u8),
    MultiPortSerial,
    Modem(u8),
    GPIB,
    SmartCard,
    Other(u8),
    Unknown(u8, u8)
}

impl PCISimpleCommunicationControllerClass {
    pub fn from(sub: u8, progif: u8) -> Self {
        use self::PCISimpleCommunicationControllerClass::*;
        match sub {
            0x0 => SerialController(progif),
            0x1 => ParallelController(progif),
            0x2 => MultiPortSerial,
            0x3 => Modem(progif),
            0x4 => GPIB,
            0x5 => SmartCard,
            0x80 => Other(progif),
            _ => Unknown(sub, progif)
        }
    }
}