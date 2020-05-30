use crate::device::pci::device::PCIDevice;
use x86_64::{PhysAddr, VirtAddr};
use volatile::{Volatile, ReadOnly};
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use crate::PAGE_TABLE;
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::structures::paging::{Mapper, Page, Size4KiB, PhysFrame, PageTableFlags, MapperAllSizes};
use crate::memory::frame_allocator::FrameAllocWrapper;
use alloc::boxed::Box;
use core::fmt::{Debug, Formatter};
use alloc::vec::Vec;
use spin::Mutex;
use kernel_api::syscall::sleep;
use core::time::Duration;
use core::ops::{DerefMut, Deref};
use crate::interrupts::InterruptIndex;
use self::port::*;
use self::datastructure::*;
use self::registers::*;
use self::consts::*;
use crate::device::pci::PCIController;
use crate::device::pci::class::{PCIDeviceClass, PCISerialBusController, PCISerialBusUSB};
use crate::device::usb::xhci::extended_capability::*;
use crate::hardware::pit::PIT;

pub mod extended_capability;
pub mod port;
pub mod consts;
mod datastructure;
pub mod registers;

#[derive(Debug)]
pub struct XHCI {
    pci_device: PCIDevice,
    mmio_base: PhysAddr,
    mmio_virt_base: VirtAddr,
    mmio_size: usize,
    capability_regs: &'static mut XHCICapabilityRegisters,
    operational_regs: &'static mut XHCIOperationalRegisters,
    info: XHCIInfo,
    ports: Mutex<Vec<XHCIPortGroup>>,
}

#[derive(Default)]
pub struct XHCIInfo {
    max_slot: u8,
    /// This field also double as the num of ports,
    /// since the port number starts from 1
    max_port: u8,
    device_context_baa: Option<Box<DeviceContextBaseAddressArray>>,
    command_ring: Option<XHCIRing>,
    event_ring: Option<Box<XHCIRingSegment>>,
    event_ring_table: Option<Box<EventRingSegmentTable>>,
}

impl Debug for XHCIInfo {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("XHCIInfo{{ }}"))
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct XHCICapabilityRegisters {
    length_and_ver: ReadOnly<u32>,
    hcs_params: [ReadOnly<u32>; 3],
    hcc_param1: ReadOnly<u32>,
    doorbell_offset: ReadOnly<u32>,
    rts_offset: ReadOnly<u32>,
}

#[derive(Debug)]
#[repr(C)]
pub struct XHCIOperationalRegisters {
    command: Volatile<u32>,
    status: Volatile<u32>,
    /// Page size is 2^(n+12), n being the value read
    page_size: ReadOnly<u32>,
    _res1: [u32; 2],
    dnctlr: Volatile<u32>,
    /// 63:6 pointer | 5:4 res | Command Ring Running
    /// | Command Abort | Command Stop | RingCycleState
    command_ring_control: Volatile<u64>,
    _res2: [u32; 4],
    device_context_base_addr_array_ptr: Volatile<u64>,
    config: Volatile<u32>,
}

impl XHCIOperationalRegisters {
    pub fn as_ptr(&mut self) -> *mut Self {
        self as *mut Self
    }

    pub fn get_port_operational_register(&self, port: u8) -> &'static mut XHCIPortOperationalRegisters {
        unsafe { &mut *(((self as *const Self as usize) + 0x400 + 0x10 * (port as usize - 1)) as *mut XHCIPortOperationalRegisters) }
    }
}

// impl From<PCIDevice> for XHCI {
//     / The caller is to ensure the passed in device is a XHCI device
// fn from(mut dev: PCIDevice) -> Self {
// }
// }

#[derive(Debug)]
pub enum XHCIError {
    NoLegacyTag,
    UnexpectedOwnership,
}

impl XHCI {
    pub fn create_from_device(mut dev: PCIDevice) -> Option<XHCI> {
        if let PCIDeviceClass::SerialBusController(PCISerialBusController::USBController(PCISerialBusUSB::XHCI)) = &dev.info.class {
            // Step1: Enable Bus Master
            let mut tmp = dev.read_config_word(crate::device::pci::consts::CONF_COMMAND_OFFSET);
            tmp |= crate::device::pci::consts::PCI_COMMAND_MASTER;
            dev.write_config_word(crate::device::pci::consts::CONF_COMMAND_OFFSET, tmp);

            // Step2: Setup Interrupt Lines
            let interrupt_number = (InterruptIndex::XHCI).as_offset() as u32;
            dev.write_config_dword_dep(0xF, interrupt_number | 0x1 << 8);
            let interrupt = dev.read_config_dword_dep(0xF);
            let int_line = (interrupt >> 8) as u8;
            let int_num = interrupt as u8;
            debug!("[XHCI] Interrupt Line: {}, Interrupt Number: {}", int_line, int_num);

            // Step3: Setup MMIO Registers
            let size = Self::xhci_address_space_detect(&mut dev);
            let base = dev.base_mmio_address(0).expect("xHCI can't be MMIO");
            let base_offset = base.as_u64() as usize & (4096 - 1);
            let alloc_base = base.align_down(4096u64);
            trace!("[XHCI] Address: {:?}, size: {}, offset:{}", base, size, base_offset);
            let va_root = without_interrupts(|| {
                let (va, size) = GMMIO_ALLOC.lock().allocate(size + base_offset);
                trace!("[XHCI] Allocated: {} bytes MMIO Space, starting: {:?}", size, va);
                let mut fallocw = FrameAllocWrapper {};
                for offset in (0..size).step_by(4096) {
                    let paddr = alloc_base + offset;
                    let vaddr = va + offset;
                    trace!("[XHCI] Mapping offset: {}, va: {:?} pa: {:?}", offset, vaddr, paddr);
                    unsafe {
                        PAGE_TABLE.write().map_to(
                            Page::<Size4KiB>::from_start_address(vaddr).expect("Unaligned VA"),
                            PhysFrame::<Size4KiB>::from_start_address(paddr).expect("Unaligned PA"),
                            PageTableFlags::WRITABLE | PageTableFlags::WRITE_THROUGH | PageTableFlags::PRESENT,
                            &mut fallocw,
                        ).expect("Unable to map").flush();
                    }
                }
                va
            });
            let mmio_vbase = va_root + base_offset;
            let cap_regs = unsafe { &mut *(mmio_vbase.as_mut_ptr() as *mut XHCICapabilityRegisters) };
            debug!("[XHCI] Cap Len: {:x}", cap_regs.length_and_ver.read());
            let operation_offset = (cap_regs.length_and_ver.read() & 0xFF) as u8;
            let op_regs = unsafe { &mut *((mmio_vbase.as_u64() + operation_offset as u64) as *mut XHCIOperationalRegisters) };
            let mut controller = XHCI {
                pci_device: dev,
                mmio_base: base,
                mmio_virt_base: mmio_vbase,
                mmio_size: size,
                capability_regs: cap_regs,
                operational_regs: op_regs,
                info: Default::default(),
                ports: Mutex::new(Default::default()),
            };
            controller.internal_initialize().expect("");
            // controller.setup_controller();
            // return Some(controller);
            return None;
        }
        debug!("[XHCI] No XHCI Controller Found");
        None
    }

    fn xhci_address_space_detect(dev: &mut PCIDevice) -> usize {
        let old_value = dev.read_config_bar_register(0);
        dev.write_config_bar_register(0, 0xFFFF_FFFF);
        let new_val = dev.read_config_bar_register(0);
        dev.write_config_bar_register(0, old_value);
        (!new_val) as usize
    }

    fn get_runtime_interrupt_register(&self, offset: u8) -> &'static mut InterrupterRegisters {
        let base_ptr = self.capability_regs as *const XHCICapabilityRegisters as u64;
        let runtime_offset = self.capability_regs.rts_offset.read() as u64;
        unsafe { &mut *((base_ptr + runtime_offset + 0x20 + (offset as u64) * 0x20) as *mut InterrupterRegisters) }
    }

    fn get_doorbell_regster(&self, offset: u8) -> &'static mut DoorBellRegister {
        let base_ptr = self.capability_regs as *const XHCICapabilityRegisters as u64;
        let doorbell_offset = self.capability_regs.doorbell_offset.read() as u64;
        unsafe { &mut *((base_ptr + doorbell_offset + (offset as u64 * 4)) as *mut DoorBellRegister) }
    }

    fn internal_initialize(&mut self) -> Result<(), ()> {
        // Step 0: Transfer Ownership
        self.transfer_ownership().expect("ownership");
        // Step 1: RESET
        self.reset()?;

        // Step 2: Populate info from HCSParams1
        let hcsparams1 = self.capability_regs.hcs_params[0].read();
        self.info.max_port = (hcsparams1 & CAP_HCCPARAMS1_MAX_PORT_MASK >> CAP_HCCPARAMS1_MAX_PORT_SHIFT) as u8;
        self.info.max_slot = (hcsparams1 & CAP_HCCPARAMS1_SLOTS_MASK) as u8;

        // Step 3: Setup opRegs->config
        let mut tmp = self.operational_regs.config.read();
        tmp = (tmp & (!CAP_HCCPARAMS1_SLOTS_MASK)) | (self.info.max_slot as u32);
        self.operational_regs.config.write(tmp);
        debug!("[XHCI] OR->config <= {} slots", self.info.max_slot);
        debug!("[XHCI] nmbrPorts = {}", self.info.max_port);

        // Step 4: Initialize Memory Structures
        self.initialize_memory_structures()?;


        // Populate Slots
        // for t in self.extended_capability() {
        //     match t {
        //         ExtendedCapabilityTag::SupportedProtocol { major, minor: _, port_offset, port_count } => {
        //             let mut ports = self.ports.lock();
        //             for port in 0..port_count {
        //                 let usbport = XHCIPort::new(port + port_offset);
        //                 if major == 3 {
        //                     if ports.len() > port as usize {
        //                         let pg = ports.get_mut(port as usize).expect("has portgroup");
        //                         assert!(pg.usb3_port.is_none());
        //                         pg.usb3_port = Some(usbport);
        //                     } else {
        //                         assert_eq!(ports.len(), port as usize);
        //                         let mut pg = XHCIPortGroup::default();
        //                         pg.usb3_port = Some(usbport);
        //                         ports.push(pg);
        //                     }
        //                 } else {
        //                     if ports.len() > port as usize {
        //                         let pg = ports.get_mut(port as usize).expect("has portgroup");
        //                         assert!(pg.usb2_port.is_none());
        //                         pg.usb2_port = Some(usbport);
        //                     } else {
        //                         assert_eq!(ports.len(), port as usize);
        //                         let mut pg = XHCIPortGroup::default();
        //                         pg.usb2_port = Some(usbport);
        //                         ports.push(pg);
        //                     }
        //                 }
        //             }
        //         }
        //         _ => {}
        //     }
        // }
        Ok(())
    }

    fn initialize_memory_structures(&mut self) -> Result<(), ()> {
        // Step 1: Setup Device Context Base Address Array
        if self.info.device_context_baa.is_none() {
            self.info.device_context_baa = Some(Box::new(Default::default()));
            let dcbaa_ptr = self.info.device_context_baa.as_deref_mut().expect("has thing") as *const DeviceContextBaseAddressArray;
            let dcbaa_va = VirtAddr::new(dcbaa_ptr as u64);
            let dcbaa_pa = without_interrupts(|| {
                PAGE_TABLE.read().translate_addr(dcbaa_va).expect("Mapped")
            });
            assert!(dcbaa_pa.is_aligned(2048u64), "DCBAA Alignment");
            self.operational_regs.device_context_base_addr_array_ptr.write(dcbaa_pa.as_u64());
        } else {
            panic!("[XHCI] dcbaa already setup");
        }
        trace!("[XHCI] DCBAA Setup complete");

        // Setup Command Ring
        if self.info.command_ring.is_none() {
            self.info.command_ring = Some(XHCIRing::new_with_capacity(1, true));
            let crcr_va = VirtAddr::from_ptr(
                self.info.command_ring.as_ref().expect("thing").segments[0].as_ref() as *const XHCIRingSegment);
            let crcr_pa = without_interrupts(|| {
                PAGE_TABLE.read().translate_addr(crcr_va).expect("Unmapped")
            });
            debug!("[XHCI] CRCR initial {:x}", self.operational_regs.command_ring_control.read());
            let tmp = self.operational_regs.command_ring_control.read();
            if tmp & OP_CRCR_CRR_MASK == 0 {
                let cyc_state = self.info.command_ring.as_ref().expect("").cycle_state as u64;
                assert_eq!(crcr_pa.as_u64() & 0b111111, 0, "alignment");
                let val64 = (tmp & OP_CRCR_RES_MASK) |
                    (crcr_pa.as_u64() & OP_CRCR_CRPTR_MASK) |
                    (cyc_state & OP_CRCR_CS_MASK);
                self.operational_regs.command_ring_control.write(val64);
            } else {
                panic!("[XHCI] CrCr is Running");
            }
        } else {
            panic!("[XHCI] CrCr already setup");
        }
        trace!("[XHCI] CRCR Setup complete");


        // Zero device notification
        self.operational_regs.dnctlr.write(0x0);
        Ok(())
    }

    pub fn hcc_flags(&mut self) -> u32 {
        self.capability_regs.hcc_param1.read()
    }

    pub fn max_ports(&self) -> u8 {
        self.info.max_port
    }

    pub fn has_device(&self, p: &XHCIPort) -> bool {
        let port = p.port_id;
        if port > 0 {
            let port_op = self.operational_regs.get_port_operational_register(port);
            let base_val = port_op.portsc.read();
            base_val & 0x1 != 0
        } else {
            false
        }
    }

    pub fn poll_interrupts(&mut self) {
        let current_status = self.operational_regs.status.read();
        if current_status & 0x4 != 0 {
            debug!("[XHCI] Int: Host Error");
        }
        if current_status & 0x8 != 0 {
            debug!("[XHCI] Int: Event Interrupt");
        }
        if current_status & 0x10 != 0 {
            debug!("[XHCI] Int: Port Interrupt");
        }
        if self.operational_regs.status.read() & 0x1 << 3 != 0 {
            debug!("[XHCI] has interrupt");
            self.operational_regs.status.write(0x1 << 3); // Clear Interrupt
            if self.get_runtime_interrupt_register(0).pending() {
                let int0_rs = self.get_runtime_interrupt_register(0);
                debug!("[XHCI] Has Pending Interrupt on Ch0 (CMDRing): 0x{:x}", int0_rs.flags.read());
                // self.info.event_ring.unwrap().
                int0_rs.flags.write(int0_rs.flags.read() | 0x1); // Clear Interrupt
                let trb = unsafe { self.info.event_ring.as_ref().unwrap().trbs[0].normal };
                debug!("[XHCI] Event: {:x?}", trb);
            }
        }
    }

    pub fn poll_ports(&mut self) {
        let poll_port = |p: &mut XHCIPort| {
            let port_op = self.operational_regs.get_port_operational_register(p.port_id);
            let connected = port_op.portsc.read() & 0x1 == 1;
            use XHCIPortStatus::*;
            match p.status {
                XHCIPortStatus::Disconnected => {
                    if connected {
                        // Connect
                        debug!("Connecting device on port: {}", p.port_id);
                        p.status = Active;
                    }
                }
                _ => {
                    if !connected {
                        debug!("Disconnecting device on port: {}", p.port_id);
                        p.status = Disconnected;
                    }
                }
            }
        };

        for port_group in self.ports.lock().iter_mut() {
            if let Some(usb3) = &mut port_group.usb3_port {
                poll_port(usb3);
                if let XHCIPortStatus::Disconnected = XHCIPortStatus::Disconnected {} else {
                    continue;
                }
            }
            if let Some(usb2) = &mut port_group.usb2_port {
                poll_port(usb2);
            }
        }
    }

    pub fn halt_controller(&mut self) -> Result<(), ()> {
        if self.operational_regs.status.read() & OP_STS_HLT_MASK == 0 {
            debug!("[XHCI] Halting Controller");
            let mut tmp = self.operational_regs.command.read();
            tmp &= !OP_CMD_RUN_STOP_MASK;
            self.operational_regs.command.write(tmp);

            let wait_target = PIT::current_time() + HALT_TIMEOUT;
            loop {
                if self.operational_regs.status.read() & OP_STS_HLT_MASK == 1 {
                    break;
                }
                if PIT::current_time() > wait_target {
                    error!("[XHCI] Timedout while halting controller on bus: {}",
                           self.pci_device.bus_location_str());
                    return Err(());
                }
                sleep(Duration::from_millis(1)).expect("slept");
            };
        }
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), ()>{
        debug!("[XHCI] Resetting Controller!");
        // Step 1: Halt if needs to
        self.halt_controller()?;

        let mut tmp = self.operational_regs.command.read();
        tmp |= OP_CMD_RESET_MASK;
        self.operational_regs.command.write(tmp);

        let wait_target = PIT::current_time() + RESET_TIMEOUT;
        loop {
            if (self.operational_regs.command.read() & OP_CMD_RESET_MASK == 0) &&
                (self.operational_regs.status.read() & OP_STS_CNR_MASK == 0) {
                break;
            }
            if PIT::current_time() > wait_target {
                error!("[XHCI] Timedout while resetting controller on bus: {}",
                       self.pci_device.bus_location_str());
                return Err(());
            }
            sleep(Duration::from_millis(1)).expect("slept");
        }
        Ok(())
    }

    pub fn setup_controller(&mut self) {


        // Setup Event Ring & Table
        self.info.event_ring_table = Some(Box::new(EventRingSegmentTable::default()));
        self.info.event_ring = Some(Box::new(XHCIRingSegment::default()));
        // Event Ring Table: Setup the first entry
        let dequeue_va = self.info.event_ring.as_deref().unwrap() as *const XHCIRingSegment as u64;
        let dequeue_pa = without_interrupts(|| {
            PAGE_TABLE.read().translate_addr(VirtAddr::new(dequeue_va)).expect("has mapping")
        });
        self.info.event_ring_table.as_mut().expect("table").segments[0].addr = dequeue_pa;
        self.info.event_ring_table.as_mut().expect("table").segments[0].segment_size = 256;

        // Interrupt Registers
        let int_regs = self.get_runtime_interrupt_register(0);
        // Load Event Ring Dequeue Pointer Register
        int_regs.event_ring_deque_ptr.write(dequeue_pa.as_u64() | 0b1000);

        // Load Table
        int_regs.event_ring_table_size.write(1); // Set Table Size
        // Event Ring Segment Table
        let erst_va = VirtAddr::new(self.info.event_ring_table.as_deref().expect("lol") as *const EventRingSegmentTable as u64);
        let erst_pa = without_interrupts(|| {
            PAGE_TABLE.read().translate_addr(erst_va).expect("mapped")
        });
        int_regs.event_ring_seg_table_ptr.write(erst_pa);
        int_regs.flags.write(0x1 << 1); // Enable Interrupt




        debug!("[XHCI] Starting Controller...");
        // HostErrEnable | Interrupt EN | Host Rest | Run/Stop
        self.operational_regs.command.write(0b1101);
        debug!("[XHCI] Controller Status: {:x}", self.operational_regs.status.read());

        let cmd_ring_status = self.operational_regs.command_ring_control.read();
        debug!("[XHCI] CRCR Value: {:x}", cmd_ring_status);
    }

    pub fn send_nop(&mut self) {
        // let cmd_ring = self.info.command_ring.as_mut().expect("uninitialized");
        // debug!("[XHCI] Sending NOOP on index {}", self.info.command_ring_enqueue_ptr);
        // let target = &mut cmd_ring.trbs[self.info.command_ring_enqueue_ptr];
        // *target = NormalTRB::new_noop().into();
        // self.info.command_ring_enqueue_ptr += 1;
        // let target = &mut cmd_ring.trbs[self.info.command_ring_enqueue_ptr];
        // *target = NormalTRB::end_queue().into();
        // self.get_doorbell_regster(0).reg.write(0);
        unimplemented!();
    }

    pub fn extended_capability(&self) -> ExtendedCapabilityTags {
        // The higher 16bits specify the offset from base,
        // unit is **DWORD**
        let hcc1val = self.capability_regs.hcc_param1.read();
        let cap_list_offset = (hcc1val >> 14) & !0b11u32; // LSH 2 so it's in bytes
        let cap_list_base: VirtAddr = self.mmio_virt_base + cap_list_offset as u64;
        ExtendedCapabilityTags::get(cap_list_base.as_u64() as usize)
    }

    /// This function transfer the ownership of the controller from BIOS
    pub fn transfer_ownership(&mut self) -> Result<(), XHCIError> {
        let tags = self.extended_capability().find(|t| {
            match t {
                ExtendedCapabilityTag::USBLegacySupport { head: _, bios_own: _, os_own: _ } => true,
                _ => false
            }
        });
        match tags {
            Some(t) => {
                if let ExtendedCapabilityTag::USBLegacySupport { head, bios_own, os_own } = t {
                    if os_own && bios_own {
                        return Err(XHCIError::UnexpectedOwnership);
                    }
                    if os_own {
                        return Ok(());
                    }
                    let write_ptr = (head as *mut u8).wrapping_offset(3);
                    let read_ptr = (head as *const u8).wrapping_offset(2);
                    unsafe { write_ptr.write_volatile(0x1) }; // Claiming Ownership
                    // Now wait
                    // TODO implement timeout
                    while unsafe { read_ptr.read_volatile() } & 0x1 > 1 {}
                    return Ok(());
                }
                panic!("wrong tag type found");
            }
            _ => Ok(())
        }
    }
}
