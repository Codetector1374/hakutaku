use crate::PAGE_TABLE;
use crate::pci::device::PCIDevice;
use crate::pci::class::*;
use x86_64::PhysAddr;
use x86_64::instructions::interrupts::without_interrupts;
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use crate::pci::class::PCIClassMassStroageSATA::AHCI;
use x86_64::structures::paging::{Mapper, Size4KiB, Page, PhysFrame, PageTableFlags};
use volatile::{ReadOnly, Volatile, WriteOnly};
use kernel_api::syscall::sleep;
use bitflags::_core::time::Duration;

const AHCI_MEMORY_REGION_SIZE: usize = 0x1100;

pub struct AHCIController {
    dev: PCIDevice,
    physical_base_addr: PhysAddr,
    regs: Option<&'static mut AHCIRegisters>,
    ports: [AHCIHBAPortStatus; 32],
}

#[repr(C)]
struct AHCIRegisters {
    generic_control: AHCIGenericHostControl,
    _res0: [u8;116],
    _vendor: [u8; 96],
    ports: [AHCIHBAPort; 32],
}

const_assert_size!(AHCIRegisters, 0x1100);

#[repr(C)]
#[allow(non_snake_case)]
struct AHCIHBAPort {
    /// Command List Base Address (Host -> Device)
    CommandListBase: Volatile<PhysAddr>,
    /// FIS Base Address (Device -> Host)
    FISBase: Volatile<PhysAddr>,
    /// Interrupt Status
    IS: Volatile<u32>,
    /// Interrupt Enable
    IE: Volatile<u32>,
    /// Command and Status
    CMD: Volatile<u32>,
    _res: u32,
    /// Task File Data
    TFD: Volatile<u32>,
    /// Signature
    SIG: Volatile<u32>,
    /// SATA Status (SCR0)
    SSTS: Volatile<u32>,
    /// SATA Control (SCR2)
    SCTL: Volatile<u32>,
    /// SATA Error (SCR1)
    SERR: Volatile<u32>,
    /// SATA Active
    SACT: Volatile<u32>,
    /// Command Issue
    CI: Volatile<u32>,
    SNTF: Volatile<u32>,
    /// FIS Based Switching Control
    FBS: Volatile<u32>,
    /// Device Sleep
    DEVSLP: Volatile<u32>,
    _res2: [u32; 10],
    _vendor: [u8; 16],
}

const_assert_size!(AHCIHBAPort, 0x80);

#[repr(C)]
#[allow(non_snake_case)]
struct AHCIGenericHostControl {
    CAP: ReadOnly<u32>, // Host(HBA) Capabilities
    GHC: Volatile<u32>, // Global HBA Control
    IS: Volatile<u32>,
    PI: Volatile<u32>,
    VS: ReadOnly<u32>,
    CCC_CTL: Volatile<u32>,
    CCC_PORTS: Volatile<u32>,
    EM_LOC: Volatile<u32>,
    EM_CTL: Volatile<u32>,
    CAP2: ReadOnly<u32>,
    BOHC: Volatile<u32>,
}
const_assert_size!(AHCIGenericHostControl, 0x2c);

#[derive(Debug, Copy, Clone)]
pub struct AHCIHBAPortStatus {
    enabled: bool,
    device: AHCIHBAPortDevice,
    sata_speed: u8,
    powerstate: AHCIHBAPortPowerState,
}

impl Default for AHCIHBAPortStatus {
    fn default() -> Self {
        AHCIHBAPortStatus {
            enabled: false,
            device: AHCIHBAPortDevice::NoDevice,
            sata_speed: 0,
            powerstate: AHCIHBAPortPowerState::Offline,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum AHCIHBAPortPowerState {
    NoDevice,
    PresentWithoutCommunication,
    PresentWithCommunication,
    Offline,
}

impl From<u8> for AHCIHBAPortPowerState {
    fn from(val: u8) -> Self {
        match val {
            0x1 => AHCIHBAPortPowerState::PresentWithoutCommunication,
            0x3 => AHCIHBAPortPowerState::PresentWithCommunication,
            0x4 => AHCIHBAPortPowerState::Offline,
            _ => AHCIHBAPortPowerState::NoDevice,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum AHCIHBAPortDevice {
    NoDevice,
    SATA,
    SATAPI,
    Unknown(u32)
}

impl From<u32> for AHCIHBAPortDevice {
    fn from(val: u32) -> Self {
        match val {
            0x0000_0101 => AHCIHBAPortDevice::SATA,
            0xEB14_0101 => AHCIHBAPortDevice::SATAPI,
            _ => AHCIHBAPortDevice::Unknown(val)
        }
    }
}

impl AHCIController {
    pub fn create_from_device(dev: PCIDevice) -> Option<AHCIController> {
        if let PCIDeviceClass::MassStorageController(PCIClassMassStorage::SATA(PCIClassMassStroageSATA::AHCI)) = dev.info.class {
            let mut controller = AHCIController {
                dev,
                physical_base_addr: PhysAddr::new(0),
                regs: None,
                ports: [AHCIHBAPortStatus::default(); 32]
            };
            controller.internal_initialize();
            return Some(controller);
        }
        debug!("[AHCI] create_from_device: not AHCI device, {:?}", dev.info.class);
        None
    }

    fn transfer_control(&mut self) {
        let regs = self.regs.as_mut().expect("");
        let current_status = regs.generic_control.BOHC.read();
        if current_status & 0x1 == 1 {
            debug!("[AHCI] Transfering Control from BIOS");
            regs.generic_control.BOHC.write(0b10);
            loop {
                if regs.generic_control.BOHC.read() & 0x2 != 0 {
                    break;
                }
                sleep(Duration::from_micros(1));
            }
        }
        debug!("[AHCI] Device is OS owned");
    }

    fn internal_initialize(&mut self) {
        use crate::memory::frame_allocator::FrameAllocWrapper;
        // bar 5
        let bar5 = self.dev.read_config_bar_register(5);
        debug!("[AHCI] bar5 = 0x{:x}", bar5);
        self.physical_base_addr = PhysAddr::new(bar5 as u64);

        // Mapping PA => VA
        /*
        This code handles the allocation of VA -> PA mapping.
        1) The PA is aligned down to a page boundary (4k)
        2) The offset + net size of region forms the "size"
        3) This is used for alloc, which aligned up to 4K, give us the number of pages
         */
        let base_offset = self.physical_base_addr.as_u64() as usize & (4096 - 1);
        let alloc_base = self.physical_base_addr.align_down(4096u64);
        let alloc_size = base_offset + AHCI_MEMORY_REGION_SIZE;
        debug!("[AHCI] PA Offset: 0x{:x} alloc_req_size: {}", base_offset, alloc_size);
        let mut fallocw = FrameAllocWrapper {};
        let va_root = without_interrupts(|| {
            let (va, size) = GMMIO_ALLOC.lock().allocate(alloc_size);
            trace!("[AHCI] Allocated {} bytes VA starting: 0x{:016x}", size, va.as_u64());
            for offset in (0..size).step_by(4096) {
                let paddr = alloc_base + offset;
                let vaddr = va + offset;
                trace!("[XHCI] Mapping offset: {}, va: {:?} pa: {:?}", offset, vaddr, paddr);
                unsafe {
                    PAGE_TABLE.lock().map_to(
                        Page::<Size4KiB>::from_start_address(vaddr).expect("va_align"),
                        PhysFrame::<Size4KiB>::from_start_address(paddr).expect("pa_align"),
                        PageTableFlags::WRITABLE | PageTableFlags::NO_CACHE | PageTableFlags::PRESENT,
                        &mut fallocw
                    ).expect("mapped").flush();
                }
            }
            va
        });
        let regs_va = va_root + base_offset;
        self.regs = Some(unsafe { &mut *(regs_va.as_mut_ptr())});

        self.transfer_control();

        let ports = self.port_count();
        let ver = self.version();
        debug!("[AHCI] Version: {:x}.{:x}", ver.0, ver.1);
        debug!("[AHCI] Controller has {} ports", ports);
        debug!("[AHCI] Ports Map: {:032b}", self.port_implemented_map());
        self.port_scan();
    }

    pub fn port_scan(&mut self) {
        let map = self.port_implemented_map();
        let regs = self.regs.as_ref().expect("");
        for i in 0..32usize {
            if map & (1 << i) as u32 != 0 {
                let status = regs.ports[i].SSTS.read();
                let dev_detect = (status & 0xF) as u8;
                let dev_spd = (status >> 4 & 0xF) as u8;
                let dev_pm = (status >> 8) as u8;
                if dev_detect != 0 {
                    if dev_pm != 1 {
                        debug!("[AHCI] Device on Port {} is in PM State {}. Skipping", i, dev_pm);
                        continue;
                    }
                    debug!("[AHCI] Device on Port {}: detect: {}, spd: {}", i, dev_detect, dev_spd);
                    let device_sig = regs.ports[i].SIG.read();
                    self.ports[i].device = device_sig.into();
                    debug!("[AHCI] Device on Port {}: {:?}", i, self.ports[i].device);
                } else {
                    self.ports[i].device = AHCIHBAPortDevice::NoDevice;
                }
                self.ports[i].enabled = true;
                self.ports[i].powerstate = dev_pm.into();
                self.ports[i].sata_speed = dev_spd;
            } else {
                self.ports[i].enabled = false
            }
        }
    }

    pub fn port_count(&self) -> u8 {
        let cap = self.regs.as_ref().expect("initialized").generic_control.CAP.read();
        return ((cap & 0b11111) as u8) + 1;
    }

    pub fn port_implemented_map(&self) -> u32 {
        self.regs.as_ref().expect("initialized").generic_control.PI.read()
    }

    pub fn version(&self) -> (u16, u16) {
        let vers = self.regs.as_ref().expect("").generic_control.VS.read();
        ((vers >> 16) as u16, vers as u16)
    }
}