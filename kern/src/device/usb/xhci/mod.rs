use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt::{Debug, Formatter};
use core::ops::{Deref, DerefMut};
use core::time::Duration;

use kernel_api::syscall::sleep;
use spin::{Mutex, RwLock};
use volatile::{ReadOnly, Volatile};
use x86_64::{PhysAddr, VirtAddr};
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::structures::paging::{Mapper, MapperAllSizes, Page, PageTableFlags, PhysFrame, Size4KiB};

use crate::device::pci::class::{PCIDeviceClass, PCISerialBusController, PCISerialBusUSB};
use crate::device::pci::device::PCIDevice;
use crate::device::pci::PCIController;
use crate::device::usb::xhci::extended_capability::*;
use crate::hardware::pit::PIT;
use crate::interrupts::InterruptIndex;
use crate::memory::frame_allocator::FrameAllocWrapper;
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use crate::{PAGE_TABLE, SCHEDULER};

use self::consts::*;
use self::datastructure::*;
use self::port::*;
use self::registers::*;
use hashbrown::HashMap;
use alloc::sync::Arc;
use crate::device::ahci::structures::CommandTable;
use x86_64::instructions::tlb::flush_all;
use core::cmp::max;
use core::option::Option::Some;
use crate::hardware::keyboard::blocking_get_char;
use pretty_hex::PrettyHex;
use alloc::string::String;
use alloc::borrow::ToOwned;
use crate::device::usb::descriptor::{USBDeviceDescriptor, USBConfigurationDescriptor, USBInterfaceDescriptor, USBEndpointDescriptor, USBInterfaceDescriptorSet, USBConfigurationDescriptorSet};
use crate::device::usb::error::USBError;

pub mod extended_capability;
pub mod port;
pub mod consts;
mod datastructure;
pub mod registers;

pub struct XHCI {
    pci_device: PCIDevice,
    mmio_base: PhysAddr,
    mmio_virt_base: VirtAddr,
    mmio_size: usize,
    regs: Mutex<XHCIRegisters>,
    info: RwLock<XHCIInfo>,
    pub ports: HashMap<u8, Arc<Mutex<XHCIPort>>>,
}

pub struct XHCIRegisters {
    capability_regs: &'static mut XHCICapabilityRegisters,
    operational_regs: &'static mut XHCIOperationalRegisters,
    doorbell_offset: u32,
    runtime_offset: u32,
}

impl XHCIRegisters {
    fn get_runtime_interrupt_register(&mut self, offset: u8) -> &'static mut InterrupterRegisters {
        let base_ptr = self.capability_regs as *const XHCICapabilityRegisters as u64;
        unsafe {
            &mut *((base_ptr + (self.runtime_offset as u64)
                + 0x20 + (offset as u64) * 0x20) as *mut InterrupterRegisters)
        }
    }

    fn get_doorbell_regster(&mut self, offset: u8) -> &'static mut DoorBellRegister {
        let base_ptr = self.capability_regs as *const XHCICapabilityRegisters as u64;
        unsafe {
            &mut *((base_ptr + (self.doorbell_offset as u64) +
                (offset as u64 * 4)) as *mut DoorBellRegister)
        }
    }
}

pub struct XHCIInfo {
    max_slot: u8,
    /// This field also double as the num of ports,
    /// since the port number starts from 1
    max_port: u8,
    page_size: u32,
    big_context: bool,
    device_context_baa: Option<Box<DeviceContextBaseAddressArray>>,
    device_contexts: [Option<Box<DeviceContextArray>>; 255],
    transfer_rings: [Option<XHCIRing>; 255],
    command_ring: Option<XHCIRing>,
    event_ring: Option<XHCIRing>,
    event_ring_table: Option<Box<EventRingSegmentTable>>,
    scratchpads: Option<Box<ScratchPadBufferArray>>,
}

impl Default for XHCIInfo {
    fn default() -> Self {
        Self {
            max_slot: 0,
            max_port: 0,
            page_size: 0,
            big_context: false,
            device_context_baa: None,
            device_contexts: [None; 255],
            transfer_rings: [None; 255],
            command_ring: None,
            event_ring: None,
            event_ring_table: None,
            scratchpads: None,
        }
    }
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
    hcc_param2: ReadOnly<u32>,
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
        assert_ne!(port, 0, "port can't be zero");
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
            trace!("[XHCI] Interrupt Line: {}, Interrupt Number: {}", int_line, int_num);

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
            trace!("[XHCI] Cap Len: {:x}", cap_regs.length_and_ver.read());
            let operation_offset = (cap_regs.length_and_ver.read() & 0xFF) as u8;
            let op_regs = unsafe { &mut *((mmio_vbase.as_u64() + operation_offset as u64) as *mut XHCIOperationalRegisters) };
            let doorbell_offset = cap_regs.doorbell_offset.read() & CAP_DBOFFSET_MASK;
            let runtime_offset = cap_regs.rts_offset.read() & CAP_RTSOFFSET_MASK;
            let regs = XHCIRegisters {
                capability_regs: cap_regs,
                operational_regs: op_regs,
                doorbell_offset,
                runtime_offset,
            };
            let mut controller = XHCI {
                pci_device: dev,
                mmio_base: base,
                mmio_virt_base: mmio_vbase,
                mmio_size: size,
                regs: Mutex::new(regs),
                info: Default::default(),
                ports: Default::default(),
            };
            controller.intel_ehci_xhci_handoff();
            controller.internal_initialize().expect("");
            return Some(controller);
        }
        error!("[XHCI] No XHCI Controller Found");
        None
    }

    /// Special Function to handle the intel controller
    /// where a mux is used to switch ports from EHCI to
    /// xHCI.
    fn intel_ehci_xhci_handoff(&mut self) {
        if self.pci_device.info.vendor_id == crate::device::pci::consts::VID_INTEL &&
            (
                self.pci_device.info.device_id == 0x8c31
            ) {
            debug!("[XHCI] Intel Controller Detected: Dev: {:#x}. EHCI Handoff", self.pci_device.info.device_id);
            let ports = self.pci_device.read_config_dword(USB_INTEL_USB3PRM);
            debug!("[XHCI] [Intel] Configurable Ports to SS: {:#x}", ports);
            // Enable Super Speed on Ports that supports it
            self.pci_device.write_config_dword(USB_INTEL_USB3_PSSEN, ports);

            let usb2_prm = self.pci_device.read_config_dword(USB_INTEL_USB2PRM);
            debug!("[XHCI] [Intel] Configurable USB2 xHCI handoff: {:#x}", usb2_prm);
            // TODO!!! REMOVE & PORTS, this is a hack for me to keep usb2 functional
            self.pci_device.write_config_dword(USB_INTEL_XUSB2PR, usb2_prm & ports);
        }
    }

    fn initialize_memory_structures(&mut self) -> Result<(), ()> {
        // Step 1: Setup Device Context Base Address Array
        let mut info = self.info.write();
        if info.device_context_baa.is_none() {
            info.device_context_baa = Some(Box::new(Default::default()));
            let dcbaa_ptr = info.device_context_baa.as_deref_mut().expect("has thing") as *const DeviceContextBaseAddressArray;
            let dcbaa_va = VirtAddr::new(dcbaa_ptr as u64);
            let dcbaa_pa = without_interrupts(|| {
                PAGE_TABLE.read().translate_addr(dcbaa_va).expect("Mapped")
            });
            assert!(dcbaa_pa.is_aligned(2048u64), "DCBAA Alignment");
            self.regs.lock().operational_regs.device_context_base_addr_array_ptr.write(dcbaa_pa.as_u64());
        } else {
            panic!("[XHCI] dcbaa already setup");
        }
        trace!("[XHCI] DCBAA Setup complete");

        // Step 2: Setup Command Ring (CRCR)
        if info.command_ring.is_none() {
            info.command_ring = Some(XHCIRing::new_with_capacity(1, true));
            let crcr_va = VirtAddr::from_ptr(
                info.command_ring.as_ref().expect("thing").segments[0].as_ref() as *const XHCIRingSegment);
            let crcr_pa = without_interrupts(|| {
                PAGE_TABLE.read().translate_addr(crcr_va).expect("Unmapped")
            });
            trace!("[XHCI] CRCR initial {:x}", self.regs.lock().operational_regs.command_ring_control.read());
            let tmp = self.regs.lock().operational_regs.command_ring_control.read();
            if tmp & OP_CRCR_CRR_MASK == 0 {
                let cyc_state = info.command_ring.as_ref().expect("").cycle_state as u64;
                assert_eq!(crcr_pa.as_u64() & 0b111111, 0, "alignment");
                let val64 = (tmp & OP_CRCR_RES_MASK) |
                    (crcr_pa.as_u64() & OP_CRCR_CRPTR_MASK) |
                    (cyc_state & OP_CRCR_CS_MASK);
                self.regs.lock().operational_regs.command_ring_control.write(val64);
            } else {
                panic!("[XHCI] CrCr is Running");
            }
        } else {
            panic!("[XHCI] CrCr already setup");
        }
        trace!("[XHCI] CRCR Setup complete");

        // Setup Event Ring
        info.event_ring = Some(XHCIRing::new_with_capacity(EVENT_RING_NUM_SEGMENTS, false));
        info.event_ring_table = Some(Box::new(EventRingSegmentTable::default()));
        info.event_ring_table.as_mut().expect("").segment_count = EVENT_RING_NUM_SEGMENTS;

        for idx in 0..EVENT_RING_NUM_SEGMENTS {
            let va = VirtAddr::from_ptr(
                info.event_ring.as_ref().expect("").
                    segments[idx].deref() as *const XHCIRingSegment
            );
            let pa = without_interrupts(|| {
                use crate::PAGE_TABLE;
                PAGE_TABLE.read().translate_addr(va).expect("")
            });
            let ent = info.event_ring_table.as_mut().expect("");
            ent.segments[idx].segment_size = TRBS_PER_SEGMENT as u32;
            assert_eq!(pa.as_u64() & 0b11_1111, 0, "alignment");
            ent.segments[idx].addr = pa;
        }
        // Update Interrupter 0 Dequeu Pointer
        let dequeue_ptr_va = VirtAddr::from_ptr(
            &info.event_ring.as_ref().expect("").segments[0].trbs[0] as *const TRB
        );
        let dequeu_ptr_pa = without_interrupts(|| {
            use crate::PAGE_TABLE;
            PAGE_TABLE.read().translate_addr(dequeue_ptr_va).expect("")
        });
        self.regs.lock().get_runtime_interrupt_register(0).event_ring_deque_ptr.
            write(dequeu_ptr_pa.as_u64() & INT_ERDP_DEQUEUE_PTR_MASK);
        // set ERST table register with correct count
        let mut tmp = self.regs.lock().get_runtime_interrupt_register(0).event_ring_table_size.read();
        tmp &= !INT_ERSTSZ_TABLE_SIZE_MASK;
        tmp |= (EVENT_RING_NUM_SEGMENTS as u32) & INT_ERSTSZ_TABLE_SIZE_MASK;
        self.regs.lock().get_runtime_interrupt_register(0).event_ring_table_size.write(tmp);
        // Setup Event Ring Segment Table Pointer
        let erst_va = VirtAddr::from_ptr(
            info.event_ring_table.as_ref().expect("").
                deref() as *const EventRingSegmentTable
        );
        let erst_pa = without_interrupts(|| {
            crate::PAGE_TABLE.read().translate_addr(erst_va).expect("")
        });
        self.regs.lock().get_runtime_interrupt_register(0).event_ring_seg_table_ptr.write(erst_pa);

        // Setup Scratchpad Registers
        let tmp = self.regs.lock().capability_regs.hcs_params[2].read();
        let mut num_sp = (tmp & CAP_HCSPARAMS2_MAX_SCRATCH_H_MSAK)
            >> CAP_HCSPARAMS2_MAX_SCRATCH_H_SHIFT;
        num_sp <<= 5;
        num_sp |= (tmp & CAP_HCSPARAMS2_MAX_SCRATCH_L_MSAK) >> CAP_HCSPARAMS2_MAX_SCRATCH_L_SHIFT;
        if num_sp > 0 {
            info.scratchpads = Some(Box::new(
                ScratchPadBufferArray::new_with_capacity(num_sp as usize, info.page_size as usize)
            ));
            info.device_context_baa.as_mut().expect("").entries[0] = without_interrupts(|| {
                crate::PAGE_TABLE.read().translate_addr(VirtAddr::from_ptr(
                    info.scratchpads.as_ref().expect("").as_ref() as *const ScratchPadBufferArray
                )).expect("mapped")
            });
        }


        // Zero device notification
        self.regs.lock().operational_regs.dnctlr.write(0x0);
        Ok(())
    }

    fn internal_initialize(&mut self) -> Result<(), ()> {
        // Step 0: Transfer Ownership
        self.transfer_ownership().expect("ownership");
        // Step 1: RESET
        self.reset()?;

        // Step 2: Populate info from HCSParams1
        {
            let mut info = self.info.write();
            let hcsparams1 = self.regs.lock().capability_regs.hcs_params[0].read();
            debug!("[XHCI] HCS1: {:#x}", hcsparams1);
            info.max_port = ((hcsparams1 & CAP_HCSPARAMS1_MAX_PORT_MASK) >> CAP_HCSPARAMS1_MAX_PORT_SHIFT) as u8;
            info.max_slot = (hcsparams1 & CAP_HCSPARAMS1_SLOTS_MASK) as u8;
            info.page_size = self.regs.lock().operational_regs.page_size.read() << 12;
            debug!("[XHCI] PageSize = {}", info.page_size);
            if info.page_size > 4096 {
                error!("[XHCI] PageSize > 4096 not supported");
                return Err(());
            }
            info.big_context = self.regs.lock().capability_regs.hcc_param1.read() & CAP_HCCPARAMS1_CSZ_MASK != 0;
            debug!("[XHCI] controller use {} bytes context", if info.big_context { 64 } else { 32 });
            if info.big_context {
                error!("[XHCI] 64 bytes context not supported yet");
            }

            let hccparams1 = self.regs.lock().capability_regs.hcc_param1.read();
            if hccparams1 & CAP_HCCPARAMS1_PORT_POWER_CTRL_MASK != 0 {
                info!("[XHCI] Controller Support Power Power");
            }
        }

        // Step 3: Setup opRegs->config
        {
            let info = self.info.read();
            let mut tmp = self.regs.lock().operational_regs.config.read();
            tmp = (tmp & (!CAP_HCSPARAMS1_SLOTS_MASK)) | (info.max_slot as u32);
            self.regs.lock().operational_regs.config.write(tmp);
            debug!("[XHCI] nmbrSlot= {}, nmbrPorts = {}", info.max_slot, info.max_port);
        }

        // Step 4: Initialize Memory Structures
        self.initialize_memory_structures()?;

        // Step 4.5: setup slot / port structures
        self.setup_port_structures();

        // Step 5: Start the Controller !
        self.start()?;

        // Clear Interrupt Ctrl / Pending
        self.regs.lock()
            .get_runtime_interrupt_register(0)
            .iman.write(INT_IRQ_FLAG_INT_PENDING_MASK);
        self.regs.lock().get_runtime_interrupt_register(0).imod.write(0);

        let ver = (self.regs.lock().capability_regs.length_and_ver.read() &
            CAP_HC_VERSION_MASK) >> CAP_HC_VERSION_SHIFT;
        debug!("[XHCI] Controller with version {:04x}", ver);

        Ok(())
    }

    pub fn halt(&mut self) -> Result<(), ()> {
        if self.regs.lock().operational_regs.status.read() & OP_STS_HLT_MASK == 0 {
            debug!("[XHCI] Halting Controller");
            let mut tmp = self.regs.lock().operational_regs.command.read();
            tmp &= !OP_CMD_RUN_STOP_MASK;
            self.regs.lock().operational_regs.command.write(tmp);

            let wait_target = PIT::current_time() + HALT_TIMEOUT;
            loop {
                if self.regs.lock().operational_regs.status.read() & OP_STS_HLT_MASK == 1 {
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

    fn start(&mut self) -> Result<(), ()> {
        debug!("[XHCI] Starting the controller");

        let mut tmp = self.regs.lock().operational_regs.command.read();
        tmp |= OP_CMD_RUN_STOP_MASK | OP_CMD_HSERR_EN_MASK;
        tmp &= !OP_CMD_INT_EN_MASK; // DISABLE INTERRUPT
        self.regs.lock().operational_regs.command.write(tmp);

        let wait_target = PIT::current_time() + HALT_TIMEOUT;
        loop {
            if self.regs.lock().operational_regs.status.read() & OP_STS_HLT_MASK == 0 {
                break;
            }
            if PIT::current_time() > wait_target {
                error!("[XHCI] Timedout while starting controller on bus: {}",
                       self.pci_device.bus_location_str());
                return Err(());
            }
            sleep(Duration::from_millis(1)).expect("slept");
        };

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), ()> {
        debug!("[XHCI] Resetting Controller!");
        // Step 1: Halt if needs to
        self.halt()?;

        let mut tmp = self.regs.lock().operational_regs.command.read();
        tmp |= OP_CMD_RESET_MASK;
        self.regs.lock().operational_regs.command.write(tmp);

        let wait_target = PIT::current_time() + RESET_TIMEOUT;
        loop {
            let cmd = self.regs.lock().operational_regs.command.read();
            let sts = self.regs.lock().operational_regs.status.read();
            if (cmd & OP_CMD_RESET_MASK == 0) &&
                (sts & OP_STS_CNR_MASK == 0) {
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

    fn setup_port_structures(&mut self) {
        for t in 1..=self.info.read().max_port {
            // TODO: [MultiXHCI] remove the 0 hardcoded controller
            self.ports.insert(t, Arc::new(Mutex::new(XHCIPort::new(0, t))));
        }
        debug!("[XHCI] Controller has {} ports {} slots", self.info.read().max_port, self.info.read().max_slot);

        // Pairs: [begin, end)
        let mut usb2ports: Option<(u8, u8)> = None;
        let mut usb3ports: Option<(u8, u8)> = None;
        for t in self.extended_capability() {
            match t {
                ExtendedCapabilityTag::SupportedProtocol { major, minor: _, port_offset: pg_base, port_count } => {
                    if major == 3 {
                        usb3ports = Some((pg_base, pg_base + port_count));
                    } else {
                        usb2ports = Some((pg_base, pg_base + port_count));
                    }
                }
                _ => {}
            }
        }
        if usb2ports.is_some() & usb3ports.is_some() {
            without_interrupts(|| {
                let usb2ports = usb2ports.expect("");
                let usb3ports = usb3ports.expect("");
                for usb2 in usb2ports.0..usb2ports.1 {
                    let mut port2 = self.ports.get(&usb2).expect("port?").lock();
                    port2.port_type = XHCIPortGeneration::USB2;
                    let offset = usb2 - usb2ports.0;
                    let usb3 = usb3ports.0 + offset;
                    if usb3 < usb3ports.1 {
                        let mut port3 = self.ports.get(&usb3).expect("matching port").lock();
                        port3.port_type = XHCIPortGeneration::USB3;
                        port3.matching_port = Some(usb2);
                        port2.matching_port = Some(usb3);
                    }
                }
            });
        }
    }

    fn xhci_address_space_detect(dev: &mut PCIDevice) -> usize {
        let old_value = dev.read_config_bar_register(0);
        dev.write_config_bar_register(0, 0xFFFF_FFFF);
        let new_val = dev.read_config_bar_register(0);
        dev.write_config_bar_register(0, old_value);
        (!new_val) as usize
    }

    pub fn has_device(&self, p: &XHCIPort) -> bool {
        let port = p.port_id;
        if port > 0 {
            let port_op = self.regs.lock().operational_regs.get_port_operational_register(port);
            let base_val = port_op.portsc.read();
            base_val & 0x1 != 0
        } else {
            false
        }
    }

    /// Must be called without interrupt
    pub fn handle_interrupt(&self) {
        let current_status = self.regs.lock().operational_regs.status.read();
        let mut response = 0;
        if current_status & OP_STS_HOST_ERR_MASK != 0 {
            response |= OP_STS_HOST_ERR_MASK;
            debug!("[XHCI] Int: Host Error");
        }
        if current_status & OP_STS_EVNT_PENDING_MASK != 0 {
            response |= OP_STS_EVNT_PENDING_MASK;
            debug!("[XHCI] Int: Event Interrupt");
            if self.regs.lock().get_runtime_interrupt_register(0).pending() {
                let int0_rs = self.regs.lock().get_runtime_interrupt_register(0);
                int0_rs.iman.write(int0_rs.iman.read() |
                    INT_IRQ_FLAG_INT_PENDING_MASK); // Clear Interrupt
                let (trb, er_deq_0, er_deq_ptr) = {
                    let mut info = self.info.write();
                    let er = info.event_ring.as_mut().expect("");
                    (er.pop(false), er.dequeue.0 as u64, er.dequeue_pointer())
                };
                if let Some(trb) = trb {
                    let tmp = er_deq_ptr.as_u64() | INT_ERDP_BUSY_MASK |
                        (INT_ERDP_DESI_MASK & er_deq_0);
                    int0_rs.event_ring_deque_ptr.write(tmp);
                    self.handle_event_trb(trb);
                } else {
                    warn!("[XHCI] Event Ring is Empty when interrupt");
                }
            }
        }
        if current_status & OP_STS_PORT_PENDING_MASK != 0 {
            response |= OP_STS_PORT_PENDING_MASK;
            for port in 1..=self.info.read().max_port {
                self.poll_port_status(port);
            }
            debug!("[XHCI] Int: Port Interrupt");
        }

        if current_status & (OP_STS_EVNT_PENDING_MASK | OP_STS_PORT_PENDING_MASK | OP_STS_HOST_ERR_MASK) == 0 {
            warn!("[XHCI] Interrupt without anything, checking command ring registers");
            let reg = self.regs.lock().get_runtime_interrupt_register(0);
            reg.iman.write(reg.iman.read());
        }

        match self.regs.try_lock() {
            None => {
                debug!("[XHCI] Failed to lock regs");
            }
            Some(mut g) => {
                g.operational_regs.status.write(response); // Clear Interrupt
            }
        }
    }

    fn handle_event_trb(&self, trb: TRB) {
        let tmp: TRBType = trb.into();
        use TRBType::*;
        match tmp {
            CommandCompletion(trb) => {
                let ptr = trb.trb_pointer;
                debug!("[XHCI] Command @ {:#x} completed", ptr);
            }
            PortStatusChange(trb) => {
                self.poll_port_status(trb.port_id);
            }
            Unknown(trb) => {
                debug!("[XHCI] unhandled trb with type: {}", trb.get_type());
            }
            _ => {
                warn!("[XHCI] Unhandled Event TRB Type")
            }
        }
    }

    fn poll_event_ring_trb(&self) -> Option<TRBType> {
        let timeout = Duration::from_millis(1000) + PIT::current_time();
        loop {
            let pop = self.info.write().event_ring.as_mut().expect("").pop(false);
            match pop {
                Some(trb) => {
                    let tmp = self.info.read().event_ring.as_ref().expect("").dequeue_pointer().as_u64() |
                        INT_ERDP_BUSY_MASK | (INT_ERDP_DESI_MASK & self.info.read().event_ring.as_ref().expect("").dequeue.0 as u64);
                    self.regs.lock().get_runtime_interrupt_register(0).event_ring_deque_ptr.write(tmp);
                    let lol = TRBType::from(trb);
                    return Some(lol);
                }
                None => {}
            }
            if PIT::current_time() > timeout {
                return None;
            }
            sleep(Duration::from_millis(10)).expect("slept");
        }
    }

    fn wait_command_complete(&self, ptr: u64) -> Option<CommandCompletionTRB> {
        // TODO update this code to use interrupt notification system
        self.regs.lock().get_doorbell_regster(0).reg.write(0);
        loop {
            let trb = self.poll_event_ring_trb()?;
            match trb {
                TRBType::CommandCompletion(c) => {
                    if c.trb_pointer == ptr {
                        return Some(c);
                    }
                }
                _ => {}
            }
        }
    }

    fn is_port_connected(&self, port_id: u8) -> bool {
        let port_op = self.regs.lock().operational_regs.get_port_operational_register(port_id);
        let mut port_status = port_op.portsc.read();
        if port_status & OP_PORT_STATUS_POWER_MASK == 0 {
            trace!("[XHCI] Port {} not powered. Powering On", port_id);
            let tmp = (port_status & !OP_PORT_STATUS_PED_MASK) | OP_PORT_STATUS_POWER_MASK;
            port_op.portsc.write(tmp);
            while port_op.portsc.read() & OP_PORT_STATUS_POWER_MASK == 0 {}
            sleep(Duration::from_millis(20)).expect("");
            port_status = port_op.portsc.read();
            trace!("[XHCI] port {} powerup complete", port_id);
        }
        let mut tmp = port_status & !OP_PORT_STATUS_PED_MASK; // Mask off this bit, writing a 1 will disable device
        tmp |= OP_PORT_STATUS_OCC_MASK | OP_PORT_STATUS_CSC_MASK;
        port_op.portsc.write(tmp);

        // Check if a port is in it's ready state
        let ready_mask = OP_PORT_STATUS_CCS_MASK;
        port_op.portsc.read() & ready_mask == ready_mask
    }

    // TODO: return Result
    fn reset_port(&self, port_id: u8) {
        let port_op = self.regs.lock().operational_regs.get_port_operational_register(port_id);
        let port_status = port_op.portsc.read();
        let mut tmp = port_status & !OP_PORT_STATUS_PED_MASK; // Mask off this bit, writing a 1 will disable device
        tmp |= OP_PORT_STATUS_RESET_MASK | OP_PORT_STATUS_PRC_MASK;
        port_op.portsc.write(tmp);
        let timeout = PORT_RESET_TIMEOUT + PIT::current_time();
        loop {
            sleep(Duration::from_millis(200)).expect("");
            let val = port_op.portsc.read();
            if val & OP_PORT_STATUS_RESET_MASK == 0 {
                if val & OP_PORT_STATUS_PRC_MASK != 0 {
                    let mut tmp = val & !OP_PORT_STATUS_PED_MASK; // Mask off this bit, writing a 1 will disable device
                    tmp |= OP_PORT_STATUS_PRC_MASK;
                    port_op.portsc.write(tmp);
                    break;
                }
                debug!("[XHCI] Port reset but no change");
            }
            if PIT::current_time() > timeout {
                error!("Failed to reset port {}", port_id);
                return;
            }
        }
        trace!("[XHCI] port {} reset", port_id);
    }

    fn setup_slot(&self, slot: u8, port_id: u8, max_packet_size: u16, block_cmd: bool) {
        let slot = slot as usize;
        let mut dev_ctx = Box::new(DeviceContextArray::default());
        let ctx_ptr = without_interrupts(|| {
            crate::PAGE_TABLE.read().translate_addr(
                VirtAddr::from_ptr(dev_ctx.deref() as *const DeviceContextArray)
            ).expect("")
        });

        let transfer_ring = XHCIRing::new_with_capacity(1, true);
        let transfer_ring_ptr = without_interrupts(|| {
            crate::PAGE_TABLE.read().translate_addr(
                VirtAddr::from_ptr(transfer_ring.segments[0].as_ref() as *const XHCIRingSegment)
            ).expect("")
        }).as_u64();
        assert_eq!(transfer_ring_ptr & 0b1111, 0, "alignment");
        trace!("[XHCI] Setting Transfer Ring Pointer to {:#x}", transfer_ring_ptr);

        // Setup Slot Context
        let portsc = self.regs.lock().operational_regs.get_port_operational_register(port_id).portsc.read();
        let speed = ((portsc & OP_PORT_STATUS_SPEED_MASK) >> OP_PORT_STATUS_SPEED_SHIFT) as u8;
        dev_ctx.slot.dword1.set_speed(speed);
        dev_ctx.slot.dword1.set_context_entries(1); // TODO Maybe not hardcode 1?
        dev_ctx.slot.root_hub_port_number = port_id;

        // Setup first EP Slot
        let epctx = &mut dev_ctx.endpoint[0];
        epctx.set_lsa_bit(); // Disable Streams
        epctx.set_cerr(3); // Max value (2 bit only)
        epctx.set_ep_type(EP_TYPE_CONTROL_BIDIR);
        if max_packet_size == 0 {
            epctx.max_packet_size = match speed {
                0 => {
                    error!("[XHCI] unknown device speed on port {}", port_id);
                    64
                }
                OP_PORT_STATUS_SPEED_LOW => 8,
                OP_PORT_STATUS_SPEED_FULL |
                OP_PORT_STATUS_SPEED_HIGH => 64,
                _ => 512,
            };
        } else {
            epctx.max_packet_size = max_packet_size;
        }
        epctx.average_trb_len = 8;
        epctx.dequeu_pointer = transfer_ring_ptr | 0x1; // Cycle Bit
        trace!("[XHCI] speed after reset, {}, {:x}", speed, portsc);

        let mut input_ctx = Box::new(InputContext {
            input: Default::default(),
            slot: dev_ctx.slot.clone(),
            endpoint: dev_ctx.endpoint.clone(),
        });
        input_ctx.input[1] = 0b11;
        let input_ctx_ptr = without_interrupts(|| {
            crate::PAGE_TABLE.read().translate_addr(
                VirtAddr::from_ptr(input_ctx.deref() as *const InputContext)
            ).expect("")
        });
        // Activate Entry
        self.info.write().device_contexts[slot - 1] = Some(dev_ctx);
        self.info.write().device_context_baa.as_mut().expect("").entries[slot] = ctx_ptr;
        self.info.write().transfer_rings[slot - 1] = Some(transfer_ring);


        let ptr = self.info.write().command_ring.as_mut().expect("").push(
            TRB { command: CommandTRB::address_device(slot as u8, input_ctx_ptr, block_cmd) }
        ).as_u64();
        self.wait_command_complete(ptr).expect("command_complete");
        core::mem::drop(input_ctx); // Don't early drop
    }

    fn send_control_command(&self, slot_id: u8, request_type: u8, request: u8,
                            value: u16, index: u16, length: u16,
                            write_to_usb: Option<&[u8]>, mut read_from_usb: Option<&mut [u8]>)
                            -> Result<usize, USBError>
    {
        let setup_trt = if write_to_usb.is_none() && read_from_usb.is_none() {
            0u8
        } else if write_to_usb.is_some() && read_from_usb.is_none() {
            2u8
        } else if read_from_usb.is_some() && write_to_usb.is_none() {
            3u8
        } else {
            return Err(USBError::InvalidArgument);
        };
        let mut setup = SetupStageTRB {
            request_type,
            request,
            value,
            index,
            length,
            int_target_trb_length: Default::default(),
            metadata: Default::default(),
        };
        setup.metadata.set_imm(true);
        setup.metadata.set_trb_type(TRB_TYPE_SETUP as u8);
        setup.metadata.set_trt(setup_trt);
        setup.int_target_trb_length.set_trb_length(8); // Always 8: Section 6.4.1.2.1, Table 6-25
        self.info.write().transfer_rings[slot_id as usize - 1].as_mut()
            .expect("").push(TRB { setup });

        if write_to_usb.is_some() || read_from_usb.is_some() {
            // Data TRB
            let mut data = DataStageTRB::default();
            data.buffer = if write_to_usb.is_some() {
                pt_translate!(VirtAddr::from_ptr(write_to_usb.as_ref().unwrap().as_ptr()))
            } else {
                pt_translate!(VirtAddr::from_ptr(read_from_usb.as_mut().unwrap().as_mut_ptr()))
            };
            data.params.set_transfer_size(
                if write_to_usb.is_some() {
                    write_to_usb.as_ref().unwrap().len() as u32
                } else {
                    read_from_usb.as_ref().unwrap().len() as u32
                });
            data.meta.set_read(read_from_usb.is_some());
            data.meta.set_trb_type(TRB_TYPE_DATA as u8);
            data.meta.set_eval_next(true);
            data.meta.set_chain(true);
            self.info.write().transfer_rings[slot_id as usize - 1].as_mut()
                .expect("").push(TRB { data }).as_u64();
        }
        // Event Data TRB
        let mut evnt_data = EventDataTRB::default();
        evnt_data.meta.set_trb_type(TRB_TYPE_EVENT_DATA as u8);
        self.info.write().transfer_rings[slot_id as usize - 1].as_mut()
            .expect("").push(TRB { event_data: evnt_data });
        // Status TRB
        let mut status_stage = StatusStageTRB::default();
        status_stage.meta.set_trb_type(TRB_TYPE_STATUS as u8);
        status_stage.meta.set_ioc(true);
        self.info.write().transfer_rings[slot_id as usize - 1].as_mut()
            .expect("").push(TRB { status_stage });

        self.regs.lock().get_doorbell_regster(slot_id).reg.write(1); // CTRL EP DB is 1
        loop {
            let result = self.poll_event_ring_trb();
            if let Some(trb) = result {
                match trb {
                    TRBType::TransferEvent(t) => {
                        let bytes_remain = t.status.get_bytes_remain() as usize;
                        let bytes_requested = if write_to_usb.is_some() {
                            write_to_usb.unwrap().len()
                        } else if read_from_usb.is_some() {
                            read_from_usb.unwrap().len()
                        } else {
                            0
                        };
                        return Ok(bytes_requested - bytes_remain);
                    }
                    _ => {
                        trace!("[XHCI] Unexp TRB: {:?}", &trb);
                    }
                }
            } else {
                error!("[XHCI] Poll TRB timedout");
                return Err(USBError::ControlEndpointTimeout);
            }
        }
    }

    fn fetch_descriptor(&self, slot_id: u8, desc_type: u8, desc_index: u8,
                        w_index: u16, buf: &mut [u8]) -> Result<usize, USBError>
    {
        self.send_control_command(slot_id, 0x80, REQUEST_GET_DESCRIPTOR,
                                  ((desc_type as u16) << 8) | (desc_index as u16),
                                  w_index, buf.len() as u16, None, Some(buf),
        )
    }

    fn fetch_device_descriptor(&self, slot_id: u8) -> Result<USBDeviceDescriptor, USBError> {
        let mut buf2 = [0u8; 18];
        self.fetch_descriptor(slot_id, DESCRIPTOR_TYPE_DEVICE, 0, 0, &mut buf2)?;
        Ok(unsafe { core::mem::transmute(buf2) })
    }

    fn into_type<T: Sized>(buf: &[u8]) -> T {
        let mut thing: T = unsafe { core::mem::zeroed() };
        {
            let tmp_slice = unsafe {
                core::slice::from_raw_parts_mut(
                    &mut thing as *mut T as *mut u8,
                    core::mem::size_of::<T>(),
                )
            };
            assert_eq!(tmp_slice.len(), buf.len(), "Unexpected size");
            tmp_slice.copy_from_slice(&buf);
        }
        thing
    }

    fn fetch_configuration_descriptor(&self, slot_id: u8) -> Result<USBConfigurationDescriptorSet, USBError> {
        use pretty_hex::*;
        let mut config_descriptor = [0u8; 9];
        self.fetch_descriptor(slot_id, DESCRIPTOR_TYPE_CONFIGURATION, 0, 0, &mut config_descriptor)?;
        let config: USBConfigurationDescriptor = unsafe { core::mem::transmute(config_descriptor) };
        let mut buf2 = vec![0u8; config.get_total_length() as usize];
        self.fetch_descriptor(slot_id, DESCRIPTOR_TYPE_CONFIGURATION, 0, 0, &mut buf2)?;
        let mut current_index = core::mem::size_of::<USBConfigurationDescriptor>();
        println!("FULL CONFIG DESC: {:#?}", buf2[0..].as_ref().hex_dump());
        let mut interfaces: Vec<USBInterfaceDescriptorSet> = Default::default();
        let mut interface_set: Option<USBInterfaceDescriptorSet> = None;
        loop {
            if current_index + 2 > buf2.len() {
                if current_index != buf2.len() {
                    warn!("[USB] Descriptor not fully fetched");
                }
                break;
            }
            let desc_size = buf2[current_index] as usize;
            if desc_size == 0 {
                break;
            }
            let desc_type = buf2[current_index + 1];
            match desc_type {
                DESCRIPTOR_TYPE_INTERFACE => {
                    if interface_set.is_some() {
                        interfaces.push(interface_set.unwrap());
                    }
                    let desc: USBInterfaceDescriptor = Self::into_type(&buf2[current_index..current_index + desc_size]);
                    debug!("[USB] IF Descriptor: {:?}", &desc);
                    interface_set = Some(USBInterfaceDescriptorSet::new(desc));
                }
                DESCRIPTOR_TYPE_ENDPOINT => {
                    let desc: USBEndpointDescriptor = Self::into_type(&buf2[current_index..current_index + desc_size]);
                    debug!("[USB] EP Descriptor: {:?}", &desc);
                    match &mut interface_set {
                        Some(ifset) => {
                            ifset.endpoints.push(desc);
                        }
                        _ => {
                            error!("[USB] EP Descriptor without IF");
                        }
                    }
                }
                _ => {
                    debug!("[USB] Unexpected descriptor type: {}", desc_type);
                }
            }
            current_index += desc_size;
        }
        Ok(USBConfigurationDescriptorSet { config, ifsets: interfaces })
    }

    fn fetch_string_descriptor(&self, slot: u8, index: u8, lang: u16) -> Result<String, USBError> {
        let mut buf = [0u8; 1];
        self.fetch_descriptor(slot as u8, DESCRIPTOR_TYPE_STRING,
                              index, lang, &mut buf)?;
        if buf[0] == 0 {
            return Err(USBError::DescriptorNotAvailable);
        }
        let mut buf2 = vec![0u8; buf[0] as usize];
        self.fetch_descriptor(slot as u8, DESCRIPTOR_TYPE_STRING,
                              index, lang, &mut buf2)?;
        assert_eq!(buf2[1], DESCRIPTOR_TYPE_STRING);
        let buf2: Vec<u16> = buf2.chunks_exact(2)
            .map(|l| { u16::from_ne_bytes([l[0], l[1]]) }).collect();
        Ok(String::from_utf16_lossy(&buf2[1..]))
    }

    /// returns Ok(slot_id)
    fn setup_new_device(&self, port_id: u8) -> Result<u8, USBError> {
        self.reset_port(port_id);
        let slot = self.send_slot_enable()? as usize;
        assert_ne!(slot, 0, "invalid slot 0 received");
        self.setup_slot(slot as u8, port_id, 0, true);
        let mut buf = [0u8; 8];
        self.fetch_descriptor(slot as u8, DESCRIPTOR_TYPE_DEVICE,
                              0, 0, &mut buf)?;
        self.reset_port(port_id);
        self.setup_slot(slot as u8, port_id, 0, false);
        let desc = self.fetch_device_descriptor(slot as u8)?;
        // println!("Device Descriptor: {:#?}", desc);
        let mut buf = [0u8; 2];
        self.fetch_descriptor(slot as u8, DESCRIPTOR_TYPE_STRING,
                              0, 0, &mut buf)?;
        assert_eq!(buf[1], DESCRIPTOR_TYPE_STRING, "Descriptor is not STRING");
        assert!(buf[0] >= 4, "has language");
        let mut buf2 = vec![0u8; buf[0] as usize];
        self.fetch_descriptor(slot as u8, DESCRIPTOR_TYPE_STRING,
                              0, 0, &mut buf2)?;
        let lang = buf2[2] as u16 | ((buf2[3] as u16) << 8);
        {
            // Display things
            let mfg = self.fetch_string_descriptor(slot as u8, desc.manufacturer_index, lang)?;
            let prd = self.fetch_string_descriptor(slot as u8, desc.product_index, lang)?;
            let ser = if desc.serial_index != 0 {
                self.fetch_string_descriptor(slot as u8, desc.serial_index, lang)?
            } else {
                String::from("")
            };
            debug!("[XHCI] New device: \nMFG: {}\nPrd:{}\nSerial:{}", mfg, prd, ser);
        }
        let configs = self.fetch_configuration_descriptor(slot as u8)?;
        let config_val = configs.config.config_val;
        debug!("[USB] Applying Config {}", config_val);
        self.send_control_command(slot as u8, 0x0, REQUEST_SET_CONFIGURATION,
                                  config_val as u16, 0, 0, None, None)?;

        debug!("[XHCI] Completed setup port {} on slot {}", port_id, slot);
        return Ok(slot as u8);
    }

    fn poll_port_status(&self, port_id: u8) {
        let ready = self.is_port_connected(port_id);
        let port_wrapper = self.ports.get(&port_id).cloned().expect("thing");
        let mut port = port_wrapper.lock();
        match port.status {
            XHCIPortStatus::Active |
            XHCIPortStatus::InactiveUSB3CompanionPort => {
                if !ready {
                    port.status = XHCIPortStatus::Disconnected;
                    // Perform Disconnection etc
                    debug!("[XHCI] Port {} disconnected", port_id);
                }
            }
            XHCIPortStatus::Failed => {
                if !ready {
                    port.status = XHCIPortStatus::Disconnected;
                    debug!("[XHCI] Port {} disconnected", port_id);
                }
            }
            _ => {
                if ready {
                    port.status = XHCIPortStatus::Connected;
                    if let XHCIPortGeneration::USB2 = port.port_type {
                        if let Some(usb3) = port.matching_port {
                            if self.is_port_connected(usb3) {
                                // Don't activate usb2 if usb3 present;
                                port.status = XHCIPortStatus::InactiveUSB3CompanionPort;
                                debug!("[XHCI] Marking port {} as disabled companion for {}", port_id, usb3);
                                return;
                            }
                        }
                    }
                    debug!("[XHCI] Port {} connected", port_id);
                    // TODO Maybe read speed for usb3 here before reset
                    match self.setup_new_device(port_id) {
                        Ok(slot) => {
                            port.status = XHCIPortStatus::Active;
                            port.slot = slot;
                        }
                        Err(_) => {
                            warn!("[XHCI] Failed to enable device on port {}", port_id);
                            port.status = XHCIPortStatus::Failed;
                        }
                    }

                    // Reset Port
                } else {
                    port.status = XHCIPortStatus::Disconnected;
                }
            }
        }
    }

    /// @Ok(opened_slot: u8)
    pub fn send_slot_enable(&self) -> Result<u8, USBError> {
        let cmd = CommandTRB::enable_slot();
        let ptr = self.info.try_write().expect("lock fuck").command_ring.as_mut()
            .expect("no cmd ring found").push(cmd.into()).as_u64();
        trace!("[XHCI] Sending Slot EN");
        match self.wait_command_complete(ptr) {
            Some(trb) => Ok(trb.slot),
            _ => Err(USBError::CommandTimeout)
        }
    }

    pub fn poll_ports(&self) {
        let max_port = self.info.read().max_port;
        for port in 1..=max_port {
            self.poll_port_status(port);
        }
    }


    pub fn send_nop(&self) {
        let ptr = {
            let mut info = self.info.write();
            let cmd_ring = info.command_ring.as_mut().expect("uninitialized");
            debug!("[XHCI] Sending NOOP on index {:?}", cmd_ring.enqueue);
            cmd_ring.push(CommandTRB::noop().into()).as_u64()
        };
        self.wait_command_complete(ptr).expect("thing");
        debug!("NoOP Complete at {:#x}", ptr);
    }

    pub fn extended_capability(&self) -> ExtendedCapabilityTags {
        // The higher 16bits specify the offset from base,
        // unit is **DWORD**
        let hcc1val = self.regs.lock().capability_regs.hcc_param1.read();
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
