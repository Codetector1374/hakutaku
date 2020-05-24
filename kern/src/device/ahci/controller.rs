use crate::PAGE_TABLE;
use crate::pci::device::PCIDevice;
use crate::pci::class::*;
use x86_64::{PhysAddr, VirtAddr};
use x86_64::instructions::interrupts::without_interrupts;
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use crate::pci::class::PCIClassMassStroageSATA::AHCI;
use x86_64::structures::paging::{Mapper, Size4KiB, Page, PhysFrame, PageTableFlags, MapperAllSizes};
use volatile::{ReadOnly, Volatile, WriteOnly};
use kernel_api::syscall::sleep;
use core::time::Duration;
use alloc::boxed::Box;
use crate::device::ahci::structures::{CommandList, ReceivedFIS, CommandTableList, CommandTable, FISType};
use crate::hardware::pit::PIT;
use super::consts::*;

const AHCI_MEMORY_REGION_SIZE: usize = 0x1100;

pub struct AHCIController {
    dev: PCIDevice,
    physical_base_addr: PhysAddr,
    regs: Option<&'static mut AHCIRegisters>,
    ports: [AHCIHBAPortStatus; 32],
    command_lists: [Option<Box<CommandList>>; 32],
    fis_list: [Option<Box<ReceivedFIS>>; 32],
    cmd_tables: [CommandTableList; 32],
}

#[repr(C)]
struct AHCIRegisters {
    generic_control: AHCIGenericHostControl, // 11 DWs 44 Bytes
    _res0: [u8; 116],
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
                regs: None,
                ports: [AHCIHBAPortStatus::default(); 32],
                command_lists: [None; 32],
                fis_list: [None; 32],
                cmd_tables: Default::default(),
            };
            controller.internal_initialize();
            return Some(controller);
        }
        debug!("[AHCI] create_from_device: not AHCI device, {:?}", dev.info.class);
        None
    }

    fn transfer_control(&mut self) {
        let regs = self.regs.as_mut().expect("");
        if regs.generic_control.CAP2.read() & 0x1 == 0{
            warn!("[AHCI] BIOS Hand Off is Not Supported");
           return;
        }
        let current_status = regs.generic_control.BOHC.read();
        if current_status & 0x1 == 1 {
            debug!("[AHCI] Transfering Control from BIOS");
            regs.generic_control.BOHC.write(0b10);
            loop {
                if regs.generic_control.BOHC.read() & 0x2 != 0 {
                    break;
                }
                sleep(Duration::from_micros(1)).expect("sleep");
            }
        }
        debug!("[AHCI] Device is OS owned: 0x{:x}", regs.generic_control.BOHC.read());
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
                    PAGE_TABLE.write().map_to(
                        Page::<Size4KiB>::from_start_address(vaddr).expect("va_align"),
                        PhysFrame::<Size4KiB>::from_start_address(paddr).expect("pa_align"),
                        PageTableFlags::WRITABLE | PageTableFlags::NO_CACHE | PageTableFlags::PRESENT,
                        &mut fallocw,
                    ).expect("mapped").flush();
                }
            }
            va
        });
        let regs_va = va_root + base_offset;
        self.regs = Some(unsafe { &mut *(regs_va.as_mut_ptr()) });
        let regs = self.regs.as_mut().expect("thing");

        // Removed since nobody does this.
        // self.transfer_control();

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

        // Intel needful-ness?
        if self.dev.info.vendor_id == crate::pci::ids::VID_INTEL {
            debug!("[AHCI] Intel Controller Detected");
            let tmp = self.dev.read_config_word(0x92);
            self.dev.write_config_word(0x92, tmp | 0xF);
        }


        let regs = self.regs.as_mut().expect("some");
        if regs.generic_control.GHC.read() >> 31 & 0x1 != 1 {
            debug!("[AHCI] Legacy Mode detected");
            regs.generic_control.GHC.write(1 << 31);
        }

        let ports = self.port_count();
        let ver = self.version();
        debug!("[AHCI] Version: {:x}.{:x}", ver.0, ver.1);
        debug!("[AHCI] Controller has {} ports", ports);
        debug!("[AHCI] Ports Map: {:032b}", self.port_implemented_map());
        debug!("[AHCI] Controller support {} command slots", self.command_slot_count());
        self.port_scan();
        let sata_port = self.ports.iter().find(|p| {
            if p.enabled {
                if let AHCIHBAPortDevice::SATA = p.device {
                    return true;
                }
            }
            false
        });
        if sata_port.is_some() {
            let sata_port_num = sata_port.unwrap().number;
            debug!("[AHCI] found SATA device on port {}", sata_port_num);
            self.setup_port(sata_port_num);
            self.test(sata_port_num);
        }
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
            sleep(Duration::from_millis(1));
        }
        debug!("[AHCI] Controller Reset Complete");
        Ok(())
    }

    fn reset_port(&mut self, port: u8) {
        let regs = self.regs.as_mut().expect("");
        regs.ports[port as usize].SCTL.write(0x701);
        sleep(Duration::from_millis(2)).expect("slept");
        regs.ports[port as usize].SCTL.write(0x700);
        while regs.ports[port as usize].SSTS.read() & 0xF != 0x3 {
            sleep(Duration::from_millis(1)).expect("slept");
        }
        debug!("[AHCI] Device Reset");
    }

    pub fn setup_port(&mut self, port: u8) {
        self.stop_port_cmd(port);
        self.reset_port(port);
        let regs = self.regs.as_mut().expect("");
        debug!("[AHCI] Command Engine Stopped");
        self.command_lists[port as usize].replace(Box::new(CommandList::default()));
        let ptr_cmd_list = self.command_lists[port as usize].as_ref().expect("some").as_ref() as *const CommandList;
        // Translate VA => PA
        let cmd_list_pa = without_interrupts(|| {
            PAGE_TABLE.read().translate_addr(VirtAddr::new(ptr_cmd_list as u64)).expect("has physical mapping")
        });
        regs.ports[port as usize].CommandListBase.write(cmd_list_pa);

        self.fis_list[port as usize].replace(Box::new(ReceivedFIS::default()));
        let ptr_fis = self.fis_list[port as usize].as_ref().expect("").as_ref() as *const ReceivedFIS;
        let fis_pa = without_interrupts(|| {
            PAGE_TABLE.read().translate_addr(VirtAddr::new(ptr_fis as u64)).expect("mapped")
        });
        regs.ports[port as usize].FISBase.write(fis_pa);


        // Initialize each command in the CMDList
        for (idx, cmd_header) in self.command_lists[port as usize].as_mut().expect("thing").commands.iter_mut().enumerate() {
            cmd_header.prdtl = 8;
            self.cmd_tables[port as usize].cmd_tables[idx].replace(Box::new(CommandTable::default()));
            let cmd_va_ptr = self.cmd_tables[port as usize].cmd_tables[idx].as_ref().expect("thing").as_ref() as *const CommandTable;
            let cmd_tbl_pa = without_interrupts(|| {
                PAGE_TABLE.read().translate_addr(VirtAddr::new(cmd_va_ptr as u64)).expect("mapped")
            });
            cmd_header.ctba = cmd_tbl_pa;
        }

        self.start_port_cmd(port);
        debug!("[AHCI] Command Table Initialized");
    }

    fn test(&mut self, port: u8) {
        use pretty_hex::*;
        let mut buf = [0u16; 256];
        self.read_sector(port, 0, &mut buf);
        let buf2: [u8; 512] = unsafe { core::mem::transmute(buf) };
        println!("Last Two Bytes {:x} {:x}", buf2[510], buf2[511]);
        // println!("First Sector: \n{:?}", buf2.as_ref().hex_dump());
    }

    fn read_sector(&mut self, port: u8, sector: u64, buf: &mut [u16]) {
        let hba_port = &mut self.regs.as_mut().expect("").ports[port as usize];
        hba_port.IS.write(!0); // Clear All Interrupts
        let slot = hba_port.find_free_command_slot().expect("free slot");
        debug!("[AHCI] read_sector: free slot: {}", slot);
        let cmd_header = &mut self.command_lists[port as usize].as_mut().expect("").commands[slot as usize];
        // D2H FIS is 4 DWORDs
        cmd_header.flags = 0x1 << 10 | 0x4; // 4 DWORDs
        cmd_header.prdtl = 1;

        let cmd_tbl = self.cmd_tables[port as usize].cmd_tables[slot as usize].as_mut().expect("lol").as_mut();
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

        let timeout_target = PIT::current_time() + Duration::from_secs(5);
        while hba_port.TFD.read() & 0b10001000 != 0 {
            sleep(Duration::from_micros(1)).expect("slept");
            trace!("[AHCI] Waiting on ctlr to idle");
            if PIT::current_time() > timeout_target {
                error!("[AHCI] timeout waiting for device to idle");
                error!("[AHCI] TFD Status: {:032b}", hba_port.TFD.read());
                panic!("AHCI Timeout");
            }
        }

        hba_port.CI.write(0x1u32 << slot as u32);
        debug!("[AHCI] Issued Read Command");
        let timeout_target = PIT::current_time() + Duration::from_secs(5);
        loop {
            if hba_port.CI.read() >> slot as u32 & 0x1 == 0 {
                break;
            }
            if hba_port.IS.read() >> 30 & 0x1 == 1 {
                error!("[AHCI] Disk Read Error Detected on Port {}, slot {}", port, slot);
                return;
            }
            if PIT::current_time() >= timeout_target {
                error!("[AHCI] Read timeout");
                return;
            }
        }
        if hba_port.IS.read() >> 30 & 0x1 == 1 {
            error!("[AHCI] Disk Read Error Detected on Port {}, slot {}", port, slot);
            return;
        }
        debug!("[AHCI] Read Complete");
    }

    /// Start Command Engine on port
    fn start_port_cmd(&mut self, port: u8) {
        use super::consts::*;
        let regs = self.regs.as_mut().expect("");
        // Waiting for CR to clear
        while regs.ports[port as usize].CMD.read() >> 15 & 0x1 == 1 {
            sleep(Duration::from_micros(1)).expect("slept");
        }
        regs.ports[port as usize].CMD.write(
            PxCMD_ST | PxCMD_SpinUp | PxCMD_PowerOn |
                PxCMD_FIS_RxEn | PxCMD_ICC_ACTIVE
        );
        let value = regs.ports[port as usize].CMD.read();
        debug!("[AHCI] CMD Port Value: {:032b}", value);
        // Before we write ST, let's do a CLO if the thing is busy
        if regs.ports[port as usize].TFD.read() >> 7 & 0x1 == 1 {
            debug!("[AHCI] Port {} busy before start", port);
            let value = regs.ports[port as usize].CMD.read();
            regs.ports[port as usize].CMD.write(value | 01 << 3);
            while regs.ports[port as usize].CMD.read() >> 3 & 0x1 == 1 {}
        }
        debug!("[AHCI] Port {} starting...", port);
        // while regs.ports[port as usize].CMD.read() & (PxCMD_CMD_Running | PxCMD_FIS_Running) != PxCMD_CMD_Running | PxCMD_FIS_Running {
        //     sleep(Duration::from_millis(1)).expect("slept");
        // }
        debug!("[AHCI] Port {} start: {:032b}", port, regs.ports[port as usize].CMD.read());
    }

    /// Stop Command Engine on port
    fn stop_port_cmd(&mut self, port: u8) {
        let regs = self.regs.as_mut().expect("");
        let val = regs.ports[port as usize].CMD.read();
        // Clear bit 0 (Start)
        regs.ports[port as usize].CMD.write(val & (!0b1));
        while regs.ports[port as usize].CMD.read() >> 15 & 0x1 != 0 {
            sleep(Duration::from_micros(1)).expect("slept");
        }
        // Clear bit 4
        let val = regs.ports[port as usize].CMD.read();
        regs.ports[port as usize].CMD.write(val & (!0b10000));
        while regs.ports[port as usize].CMD.read() >> 14 & 0x1 != 0 {
            sleep(Duration::from_micros(1)).expect("slept");
        }
    }

    pub fn port_scan(&mut self) {
        let map = self.port_implemented_map();
        let regs = self.regs.as_ref().expect("");
        for i in 0..32usize {
            self.ports[i].number = i as u8;
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
                    debug!("[AHCI] Device on Port {}: {:x?}", i, self.ports[i].device);
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