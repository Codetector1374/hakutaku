use crate::PAGE_TABLE;
use x86_64::{PhysAddr, VirtAddr};
use x86_64::instructions::interrupts::without_interrupts;
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use crate::device::pci::class::PCIClassMassStroageSATA::AHCI;
use x86_64::structures::paging::{Mapper, Size4KiB, Page, PhysFrame, PageTableFlags, MapperAllSizes};
use volatile::{ReadOnly, Volatile, WriteOnly};
use kernel_api::syscall::sleep;
use core::time::Duration;
use alloc::boxed::Box;
use crate::device::ahci::structures::{CommandList, ReceivedFIS, CommandTable, FISType, FISRegH2D, AHCIPortCommStructures};
use crate::hardware::pit::PIT;
use super::consts::*;
use core::cmp::min;
use crate::device::ahci::device::{AHCISATADevice, AHCIDevice};
use alloc::vec::Vec;
use spin::{Mutex, RwLock};
use alloc::sync::Arc;
use pc_keyboard::KeyCode::Mute;
use core::ops::{Deref, DerefMut};
use core::borrow::BorrowMut;
use crate::device::pci::device::PCIDevice;
use crate::device::pci::class::{PCIClassMassStorage, PCIClassMassStroageSATA, PCIDeviceClass};

const AHCI_MEMORY_REGION_SIZE: usize = 0x1100;

macro_rules! write_flush {
    ($var: expr, $val: expr) => {{
    $var.write($val); $var.read();
    }};
}

pub struct AHCIController {
    pub dev: PCIDevice,
    physical_base_addr: PhysAddr,
    regs: Option<&'static mut AHCIRegisters>,
    ports: [AHCIHBAPortStatus; 32],
    operation_structures: [Option<Arc<Mutex<AHCIPortCommStructures>>>; 32],
    capability: u32,
    port_map: u32,
    n_port: u32,
    link_map: u32,
}

#[repr(C)]
struct AHCIRegisters {
    generic_control: AHCIGenericHostControl,
    // 11 DWs 44 Bytes
    _res0: [u8; 116],
    _vendor: [u8; 96],
    ports: [AHCIHBAPort; 32],
}

const_assert_size!(AHCIRegisters, 0x1100);

#[repr(C)]
#[allow(non_snake_case)]
pub(super) struct AHCIHBAPort {
    /// Command List Base Address (Host -> Device)
    CommandListBase_L: Volatile<u32>,
    CommandListBase_H: Volatile<u32>,
    /// FIS Base Address (Device -> Host)
    FISBase_L: Volatile<u32>,
    FISBase_H: Volatile<u32>,
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

impl AHCIHBAPort {
    pub fn find_free_command_slot(&self) -> Option<u8> {
        let val = self.CI.read();
        for i in 0..32u8 {
            if (val >> i) & 0x1 == 0 {
                return Some(i);
            }
        }
        None
    }

    pub fn set_command_list_base(&mut self, addr: PhysAddr) {
        write_flush!(self.CommandListBase_L, addr.as_u64() as u32);
        write_flush!(self.CommandListBase_H, (addr.as_u64() >> 32) as u32);
    }

    pub fn set_fis_base(&mut self, addr: PhysAddr) {
        write_flush!(self.FISBase_L, addr.as_u64() as u32);
        write_flush!(self.FISBase_H, (addr.as_u64() >> 32) as u32);
    }
}

#[repr(C)]
#[allow(non_snake_case)]
struct AHCIGenericHostControl {
    CAP: Volatile<u32>,
    // Host(HBA) Capabilities
    GHC: Volatile<u32>,
    // Global HBA Control
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
    number: u8,
    enabled: bool,
    device: AHCIHBAPortDevice,
    sata_speed: u8,
    powerstate: AHCIHBAPortPowerState,
}

impl Default for AHCIHBAPortStatus {
    fn default() -> Self {
        AHCIHBAPortStatus {
            number: 0,
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
    Unknown(u32),
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
                regs: Default::default(),
                ports: Default::default(),
                operation_structures: Default::default(),
                capability: Default::default(),
                port_map: Default::default(),
                n_port: 0,
                link_map: 0,
            };
            controller.internal_initialize();
            return Some(controller);
        }
        warn!("[AHCI] create_from_device: not AHCI device, {:?}", dev.info.class);
        None
    }

    pub(super) fn get_port_registers(&self, port: u8) -> Option<Arc<Mutex<AHCIPortCommStructures>>> {
        match &self.operation_structures[port as usize] {
            Some(s) => Some(s.clone()),
            _ => None,
        }
    }

    fn internal_initialize(&mut self) {
        use crate::memory::frame_allocator::FrameAllocWrapper;
        // bar 5
        let bar5 = self.dev.read_config_bar_register(5);
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
        trace!("[AHCI] PA Offset: 0x{:x} alloc_req_size: {}", base_offset, alloc_size);
        let mut fallocw = FrameAllocWrapper {};
        let va_root = without_interrupts(|| {
            let (va, size) = GMMIO_ALLOC.lock().allocate(alloc_size);
            trace!("[AHCI] Allocated {} bytes VA starting: 0x{:016x}", size, va.as_u64());
            for offset in (0..size).step_by(4096) {
                let paddr = alloc_base + offset;
                let vaddr = va + offset;
                trace!("[XHCI] Mapping offset: {}, va: {:?} pa: {:?}", offset, vaddr, paddr);
                unsafe {
                    PAGE_TABLE.write().map_to(
                        Page::<Size4KiB>::from_start_address(vaddr).expect("va_align"),
                        PhysFrame::<Size4KiB>::from_start_address(paddr).expect("pa_align"),
                        PageTableFlags::WRITABLE | PageTableFlags::WRITE_THROUGH | PageTableFlags::PRESENT,
                        &mut fallocw,
                    ).expect("mapped").flush();
                }
            }
            va
        });
        let regs_va = va_root + base_offset;
        self.regs = Some(unsafe { &mut *(regs_va.as_mut_ptr()) });
        let regs = self.regs.as_mut().expect("thing");

        // Step 1: Save the existing HostCap register before reset.
        let mut cap_save = regs.generic_control.CAP.read();
        cap_save &= CAP_SPM | CAP_SMPS;
        cap_save |= CAP_SSS;

        // Step 2: Reset Controller
        self.controller_reset().expect("reset failure");

        let regs = self.regs.as_mut().expect("");
        // Enable AHCI Mode
        regs.generic_control.GHC.write(GHC_AHCIEnable);
        // Restore HostCap
        regs.generic_control.CAP.write(cap_save);
        // profit?? http://10.45.1.22/uboot/latest/source/drivers/ata/ahci.c#L194
        regs.generic_control.PI.write(0xf);

        // Update information
        self.capability = regs.generic_control.CAP.read();
        self.port_map = regs.generic_control.PI.read();
        self.n_port = self.port_count() as u32; // Lowest 6 bits
        let ver = self.version();
        trace!("[AHCI] Version: {:x}.{:x}", ver.0, ver.1);
        trace!("[AHCI] Controller has {} ports", self.n_port);
        trace!("[AHCI] Portmap: {:032b}", self.port_map);
        trace!("[AHCI] cap: 0x{:x}", self.capability);

        // Initialize all ports
        for port_number in 0..32u8 {
            if self.port_map >> port_number & 0x1 == 0 {
                continue;
            }
            self.initialize_port(port_number);
        }

        // Enable IRQ?
        let regs = self.regs.as_mut().expect("");
        let tmp = regs.generic_control.GHC.read();
        regs.generic_control.GHC.write(tmp | GHC_InterruptEnable);

        let word = self.dev.read_config_word(crate::device::pci::consts::CONF_COMMAND_OFFSET);
        self.dev.write_config_word(crate::device::pci::consts::CONF_COMMAND_OFFSET,
                                   word | crate::device::pci::consts::PCI_COMMAND_MASTER);

        self.start_ports();
    }

    fn controller_reset(&mut self) -> Result<(), ()> {
        let regs = self.regs.as_mut().expect("");
        let tmp = regs.generic_control.GHC.read();
        // Check if a reset is already in progress
        if tmp & GHC_HostReset == 0 {
            regs.generic_control.GHC.write(tmp | 0x1); // HBA Reset
        }

        // HBA *SHOULD* reset within 1 second
        let timeout_target = PIT::current_time() + AHCIHBAResetTimeout;
        while regs.generic_control.GHC.read() & 0x1 == 1 {
            if PIT::current_time() > timeout_target {
                error!("[AHCI] HBA Reset timeout");
                return Err(());
            }
            sleep(Duration::from_millis(1)).expect("slept");
        }
        trace!("[AHCI] Controller Reset Complete");
        Ok(())
    }

    fn initialize_port(&mut self, port: u8) {
        // Step1: Stop if it is active
        let regs = self.regs.as_mut().expect(concat!(file!(), line!()));
        let port_reg = &mut regs.ports[port as usize];

        let flags = PxCMD_ST | PxCMD_CMD_Running | PxCMD_FIS_RxEn | PxCMD_FIS_Running;
        let tmp = port_reg.CMD.read();
        if tmp & flags != 0 {
            trace!("[AHCI] Port {} is active, stopping", port);
            write_flush!(port_reg.CMD,tmp & !flags);
            sleep(Duration::from_millis(500)).expect("slept");
        }
        /* Add the spinup command to whatever mode bits may
		 * already be on in the command register.
		 */
        let tmp = port_reg.CMD.read();
        write_flush!(port_reg.CMD, tmp | PxCMD_SpinUp);

        if Self::sata_link_up(port_reg).is_err() {
            return;
        }

        // Error Clear
        port_reg.SERR.write(port_reg.SERR.read());

        // Spinup Drive
        let timeout_target = PIT::current_time() + AHCIDeviceSpinupTimeout;
        let spinup_begin = PIT::current_time();
        loop {
            let tfd = port_reg.TFD.read();
            if tfd & (PxTFD_BSY | PxTFD_DRQ) == 0 {
                break;
            }
            sleep(Duration::from_millis(1)).expect("slept");
            let tmp = port_reg.SSTS.read() & PxSSTS_DETMask;
            if tmp == PxSSTS_DET_Ready {
                break;
            }
            if PIT::current_time() > timeout_target {
                error!("[AHCI] Device spinup timeout");
            }
        }

        let tmp = port_reg.SSTS.read() & PxSSTS_DETMask;
        if tmp == PxSSTS_DET_ComInit {
            debug!("[AHCI] Device Present, no communication");
        }

        debug!("[AHCI] Spinup time: {:?}", PIT::current_time() - spinup_begin);

        let tmp = port_reg.SERR.read();
        if tmp != 0 {
            warn!("[AHCI] Error: 0x{:x}", tmp);
            port_reg.SERR.write(tmp);
        }


        // Clear Interrupt if any
        let tmp = port_reg.IS.read();
        trace!("[AHCI] Interrupt Status: {:#x}", tmp);
        if tmp != 0 {
            port_reg.IS.write(tmp);
        }
        regs.generic_control.IS.write(0x1 << port);

        let tmp = port_reg.SSTS.read();
        trace!("[AHCI] Port {} status: {:#x}", port, tmp);
        if tmp & PxSSTS_DETMask == PxSSTS_DET_Ready {
            self.link_map |= 0x1 << port;
        }
    }

    fn sata_link_up(port: &mut AHCIHBAPort) -> Result<(), ()> {
        let target = PIT::current_time() + AHCIPortLinkUpTimeout;
        loop {
            let val = port.SSTS.read();
            if val & PxSSTS_DETMask == PxSSTS_DET_Ready {
                return Ok(());
            }
            if PIT::current_time() > target {
                return Err(());
            }
            sleep(Duration::from_millis(1)).expect("");
        }
    }

    fn start_ports(&mut self) {
        trace!("[AHCI] Starting Ports: {:#x}", self.link_map);
        for port_number in 0..32u8 {
            if self.link_map >> port_number & 0x1 == 0 {
                continue;
            }
            self.start_port(port_number);
        }
    }

    fn start_port(&mut self, port: u8) {
        let regs = self.regs.as_mut().expect("");
        let port_reg = &mut regs.ports[port as usize];
        trace!("[AHCI] Starting Port: {}", port);
        let port_sts = port_reg.SSTS.read();
        trace!("[AHCI] Port: {} status: {:#x}", port, port_sts);
        if port_sts & PxSSTS_DETMask != PxSSTS_DET_Ready {
            warn!("[AHCI] Port {} has no link", port);
            return;
        }

        self.operation_structures[port as usize].replace(Arc::new(Mutex::new(
            AHCIPortCommStructures {
                port_reg: unsafe { &mut *(port_reg as *mut AHCIHBAPort) },
                command_list: Default::default(),
                receive_fis: Default::default(),
                command_tables: Default::default(),
            }
        )));
        let mut op_struct_lock = self.operation_structures[port as usize].as_ref().expect("").lock();

        // Setup Lists, FISes
        let ptr_cmd_list = &op_struct_lock.command_list as *const CommandList;
        // Translate VA => PA
        let cmd_list_pa = without_interrupts(|| {
            PAGE_TABLE.read().translate_addr(VirtAddr::new(ptr_cmd_list as u64)).expect("has physical mapping")
        });
        assert!(cmd_list_pa.is_aligned(1024u64), "CLB Alignment");
        regs.ports[port as usize].set_command_list_base(cmd_list_pa);

        let ptr_fis = &op_struct_lock.receive_fis as *const ReceivedFIS;
        let fis_pa = without_interrupts(|| {
            PAGE_TABLE.read().translate_addr(VirtAddr::new(ptr_fis as u64)).expect("mapped")
        });
        assert!(fis_pa.is_aligned(256u64), "CLB Alignment");
        regs.ports[port as usize].set_fis_base(fis_pa);

        let port_reg = &mut regs.ports[port as usize];
        // Initialize each command in the CMDList
        for idx in 0..op_struct_lock.command_list.commands.len() {
            let cmd_va_ptr = op_struct_lock.command_tables[idx].as_ref() as *const CommandTable;
            let cmd_tbl_pa = without_interrupts(|| {
                PAGE_TABLE.read().translate_addr(VirtAddr::new(cmd_va_ptr as u64)).expect("mapped")
            });
            let cmd_header = &mut op_struct_lock.command_list.commands[idx];
            cmd_header.prdtl = 8;
            cmd_header.ctba = cmd_tbl_pa;
        }

        // Actually Start Port
        write_flush!(port_reg.CMD, PxCMD_ICC_ACTIVE |
        PxCMD_FIS_RxEn | PxCMD_PowerOn | PxCMD_SpinUp | PxCMD_ST
        );

        let timeout_target = PIT::current_time() + AHCIDeviceSpinupTimeout;
        loop {
            if port_reg.TFD.read() & PxTFD_BSY == 0 {
                break;
            }

            if PIT::current_time() > timeout_target {
                error!("Start Device on port {} Spinup Timeout", port);
            }
        }
        trace!("[AHCI] Port {} started", port);
    }

    fn test(&mut self, port: u8) {
        use pretty_hex::*;
        let mut buf = [0u16; 256];
        let mut lock = self.operation_structures[port as usize].as_ref().expect("").lock();
        let count = Self::read_sector(lock.deref_mut(), 0, &mut buf).expect("");
        let buf2: [u8; 512] = unsafe { core::mem::transmute(buf) };
        println!("size: {} Last Two Bytes {:x} {:x}", count, buf2[510], buf2[511]);
        // println!("First Sector: \n{:?}", buf2.as_ref().hex_dump());
    }

    pub(super) fn read_sector(op_structure_lock: &mut AHCIPortCommStructures, sector: u64, buf: &mut [u16]) -> Result<usize, ()> {
        // let mut op_structure_lock = self.operation_structures[port as usize].as_ref().expect("").lock();
        op_structure_lock.port_reg.IS.write(!0); // Clear All Interrupts
        let slot = op_structure_lock.port_reg.find_free_command_slot().expect("free slot");
        let cmd_header = &mut op_structure_lock.command_list.commands[slot as usize];
        // H2D FIS is 5 DWORDs
        cmd_header.flags = (core::mem::size_of::<FISRegH2D>() / 4) as u16; // 5 DWORDs
        cmd_header.prdtl = 1;

        {
            let cmd_tbl = op_structure_lock.command_tables[slot as usize].as_mut();
            *cmd_tbl = CommandTable::default(); // zero table
            cmd_tbl.prtd_entry[0].flags = 0x1 << 31 | 511; // TODO Fixed sector size
            let bufpa = without_interrupts(|| {
                PAGE_TABLE.read().translate_addr(VirtAddr::new(buf.as_mut_ptr() as u64)).expect("mapped")
            });
            cmd_tbl.prtd_entry[0].dba = bufpa;

            cmd_tbl.cfis.fis_type = FISType::RegH2d;
            cmd_tbl.cfis.flags = 0x1 << 7;
            cmd_tbl.cfis.command = 0x25; // Read DMA (Non Ext: 256 sectors max)
            cmd_tbl.cfis.set_lba(sector);
            cmd_tbl.cfis.device = 1 << 6; // LBA Mode
            cmd_tbl.cfis.count = 1;
        }

        let timeout_target = PIT::current_time() + Duration::from_secs(5);
        while op_structure_lock.port_reg.TFD.read() & 0b10001000 != 0 {
            sleep(Duration::from_micros(1)).expect("slept");
            if PIT::current_time() > timeout_target {
                error!("[AHCI] timeout waiting for device to idle");
                error!("[AHCI] TFD Status: {:032b}", op_structure_lock.port_reg.TFD.read());
                return Err(());
            }
        }

        write_flush!(op_structure_lock.port_reg.CI, 1u32 << slot);
        let timeout_target = PIT::current_time() + Duration::from_secs(5);
        loop {
            if op_structure_lock.port_reg.CI.read() >> slot as u32 & 0x1 == 0 {
                break;
            }
            if op_structure_lock.port_reg.IS.read() >> 30 & 0x1 == 1 {
                error!("[AHCI] Disk Read Error Detected, slot {}", slot);
                return Err(());
            }
            if PIT::current_time() >= timeout_target {
                error!("[AHCI] Read timeout");
                return Err(());
            }
        }
        if op_structure_lock.port_reg.IS.read() >> 30 & 0x1 == 1 {
            error!("[AHCI] Disk Read Error Detected on, slot {}", slot);
            return Err(());
        }
        return Ok(min(512, buf.len() * 2));
    }

    pub fn port_scan(&mut self) -> Vec<Box<dyn AHCIDevice + Send + Sync>> {
        let map = self.port_implemented_map();
        let regs = self.regs.as_ref().expect("");
        let mut devices = Vec::new();
        for i in 0..32usize {
            self.ports[i].number = i as u8;
            if map & (1 << i) as u32 != 0 {
                let status = regs.ports[i].SSTS.read();
                let dev_detect = (status & 0xF) as u8;
                let dev_spd = (status >> 4 & 0xF) as u8;
                let dev_pm = (status >> 8) as u8;
                self.ports[i].enabled = true;
                self.ports[i].powerstate = dev_pm.into();
                self.ports[i].sata_speed = dev_spd;
                if dev_detect as u32 == PxSSTS_DET_Ready {
                    if dev_pm != 1 {
                        debug!("[AHCI] Device on Port {} is in PM State {}. Skipping", i, dev_pm);
                        continue;
                    }
                    let device_sig = regs.ports[i].SIG.read();
                    self.ports[i].device = device_sig.into();
                    match self.create_device(i as u8, &self.ports[i].device) {
                        Some(d) => devices.push(d),
                        _ => {}
                    }
                } else {
                    self.ports[i].device = AHCIHBAPortDevice::NoDevice;
                }
            } else {
                self.ports[i].enabled = false
            }
        }
        devices
    }

    /// Creates a AHCIDevice from HBAPortDevice.
    /// Returns None when the device is invalid or
    fn create_device(&self, port: u8, sig: &AHCIHBAPortDevice) -> Option<Box<dyn AHCIDevice + Send + Sync>> {
        match sig {
            AHCIHBAPortDevice::SATA => Some(Box::new(AHCISATADevice::create(port))),
            _ => None
        }
    }

    pub fn port_count(&self) -> u8 {
        let cap = self.regs.as_ref().expect("initialized").generic_control.CAP.read();
        return ((cap & 0b11111) as u8) + 1;
    }

    fn command_slot_count(&self) -> u8 {
        let cap = self.regs.as_ref().expect("initialized").generic_control.CAP.read();
        return ((cap >> 8 & 0b11111) as u8) + 1;
    }

    pub fn port_implemented_map(&self) -> u32 {
        self.regs.as_ref().expect("initialized").generic_control.PI.read()
    }

    pub fn version(&self) -> (u16, u16) {
        let vers = self.regs.as_ref().expect("").generic_control.VS.read();
        ((vers >> 16) as u16, vers as u16)
    }
}