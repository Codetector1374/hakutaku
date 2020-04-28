use crate::pci::device::PCIDevice;
use x86_64::{PhysAddr, VirtAddr};
use volatile::{Volatile, ReadOnly};
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use crate::PAGE_TABLE;
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::structures::paging::{Mapper, Page, Size4KiB, PhysFrame, PageTableFlags, MapperAllSizes};
use crate::memory::frame_allocator::FrameAllocWrapper;
use core::ops::Add;
use crate::pci::device::xhci::extended_capability::{ExtendedCapabilityTags, ExtendedCapabilityTag};
use alloc::boxed::Box;
use crate::pci::device::xhci::datastructure::{DeviceContextBaseAddressArray, CommandRingSegment};
use bitflags::_core::fmt::{Debug, Formatter};
use alloc::vec::Vec;
use crate::pci::device::xhci::port::XHCIPortGroup;

pub mod extended_capability;
pub mod port;
mod datastructure;

#[derive(Debug)]
pub struct XHCI {
    pci_device: PCIDevice,
    mmio_base: PhysAddr,
    mmio_virt_base: VirtAddr,
    mmio_size: usize,
    capability_regs: &'static mut XHCICapabilityRegisters,
    operational_regs: &'static mut XHCIOperationalRegisters,
    info: XHCIInfo,
    ports: Vec<XHCIPortGroup>,
}

#[derive(Default)]
pub struct XHCIInfo {
    max_slot: u8,
    max_interrupt: u16,
    /// This field also double as the num of ports,
    /// since the port number starts from 1
    max_port: u8,
    device_context_baa: Option<Box<DeviceContextBaseAddressArray>>,
    usb_cr: Option<Box<CommandRingSegment>>,
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
    device_notification: Volatile<u32>,
    /// 63:6 pointer | 5:4 res | Command Ring Running
    /// | Command Abort | Command Stop | RingCycleState
    command_ring_control: Volatile<u64>,
    _res2: [u32; 4],
    device_context_base_addr_array_ptr: Volatile<u64>,
    config: Volatile<u32>,
    port_status_ctrl: Volatile<u32>,
}

impl XHCIOperationalRegisters {
    pub fn as_ptr(&mut self) -> *mut Self {
        self as *mut Self
    }

    pub fn get_port_base(&self, port: u8) -> usize {
        (self as *const Self as usize) + 0x400 + 0x10 * (port as usize - 1)
    }
}

impl From<PCIDevice> for XHCI {
    /// The caller is to ensure the passed in device is a XHCI device
    fn from(mut dev: PCIDevice) -> Self {
        // Bar0
        let size = dev.address_space_size();
        let base = dev.base_mmio_address(0).expect("xHCI is MMIO");
        let base_offset = base.as_u64() as usize & (4096 - 1);
        let alloc_base = base.align_down(4096u64);
        trace!("XHCI Address: {:?}, size: {}, offset:{}", base, size, base_offset);
        let va_root = without_interrupts(|| {
            let (va, size) = GMMIO_ALLOC.lock().allocate(size);
            trace!("Allocated: {} bytes MMIO Space, starting: {:?}", size, va);
            let mut fallocw = FrameAllocWrapper {};
            for offset in (0..size).step_by(4096) {
                let paddr = alloc_base + offset;
                let vaddr = va + offset;
                trace!("Mapping offset: {}, va: {:?} pa: {:?}", offset, vaddr, paddr);
                unsafe {
                    PAGE_TABLE.lock().map_to(
                        Page::<Size4KiB>::from_start_address(vaddr).expect("Unaligned VA"),
                        PhysFrame::<Size4KiB>::from_start_address(paddr).expect("Unaligned PA"),
                        PageTableFlags::WRITABLE | PageTableFlags::PRESENT,
                        &mut fallocw,
                    ).expect("Unable to map").flush();
                }
            }
            va
        });
        let mmio_vbase = va_root + base_offset;
        without_interrupts(|| {
            let base = PAGE_TABLE.lock().translate_addr(mmio_vbase).expect("phy");
            info!("MMIO Phy Base: {:?}", base);
        });
        let cap_regs = unsafe { &mut *(mmio_vbase.as_mut_ptr() as *mut XHCICapabilityRegisters) };
        let operation_offset = (cap_regs.length_and_ver.read() & 0xFF) as u8;
        let op_regs = unsafe { &mut *((mmio_vbase.as_u64() + operation_offset as u64) as *mut XHCIOperationalRegisters) };
        XHCI {
            pci_device: dev,
            mmio_base: base,
            mmio_virt_base: mmio_vbase,
            mmio_size: size,
            capability_regs: cap_regs,
            operational_regs: op_regs,
            info: Default::default(),
            ports: Default::default(),
        }.internal_initialize()
    }
}

bitflags! {
    pub struct HCCFlags: u32 {
        const ADDR64BIT = 0x1 << 0;
        const BNC = 0x1 << 1;
        /// 64 bit context size if true
        const CONTEXTSIZE = 0x1 << 2;
        const PORTPOWERCTRL = 0x1 << 3;
        const PORTINDC = 0x1 << 4;
        const LIGHTRST = 0x1 << 5;
        const LTC = 0x1 << 6;
        const NOSSID = 0x1 << 7;
        const PARSEALLEVNT = 0x1 << 8;
        const STOP = 0x1 << 9;
        // not fully listed
    }
}

#[derive(Debug)]
pub enum XHCIError {
    NoLegacyTag,
    UnexpectedOwnership,
}

impl XHCI {
    fn internal_initialize(mut self) -> Self {
        // Populate info from HCSParams1
        let hcsparams1 = self.capability_regs.hcs_params[0].read();
        self.info.max_port = (hcsparams1 >> 24) as u8;
        self.info.max_interrupt= (hcsparams1 >> 8) as u16;
        self.info.max_slot = hcsparams1 as u8;

        self
    }

    pub fn hcc_flags(&mut self) -> HCCFlags {
        let val = self.capability_regs.hcc_param1.read();
        HCCFlags::from_bits_truncate(val)
    }

    pub fn max_ports(&self) -> u8 {
        self.info.max_port
    }

    pub fn has_device(&self, port: u8) -> bool {
        if port > 0 {
            let port_base = self.operational_regs.get_port_base(port);
            let base_val = unsafe { *(port_base as *const u32) };
            base_val & 0x1 != 0
        } else {
            false
        }
    }

    pub fn setup_controller(&mut self) {
        if self.info.device_context_baa.is_none() {
            self.info.device_context_baa = Some(Box::new(Default::default()));
            let dcbaa_ptr = self.info.device_context_baa.as_deref_mut().expect("has thing") as *const DeviceContextBaseAddressArray;
            let dcbaa_va = VirtAddr::new(dcbaa_ptr as u64);
            let dcbaa_pa = without_interrupts(|| {
                PAGE_TABLE.lock().translate_addr(dcbaa_va).expect("Mapped")
            });
            trace!("[XHCI] dcbaaPA: {:?}", dcbaa_pa);
            if dcbaa_pa.as_u64() & 0b11_1111 != 0 {
                panic!("Alignment issue");
            }
            // let ptr = &self.operational_regs.device_context_base_addr_array_ptr as *const Volatile<u64> as usize;
            // let trans = PAGE_TABLE.lock().translate(VirtAddr::new(ptr as u64));
            // trace!("PTR: 0x{:X}, {:#?}", ptr, trans);
            self.operational_regs.device_context_base_addr_array_ptr.write(dcbaa_pa.as_u64());
        } else {
            panic!("[XHCI] dcbaa already setup");
        }
        trace!("[XHCI] dcbaa done");

        // Setup Ctrl Ring Ctrl Register
        if self.info.usb_cr.is_none() {
            self.info.usb_cr = Some(Box::new(Default::default()));
            let crcr_ptr = self.info.usb_cr.as_deref().expect("thing") as *const CommandRingSegment;
            let crcr_va = VirtAddr::new(crcr_ptr as u64);
            let crcr_pa = without_interrupts(|| {
                PAGE_TABLE.lock().translate_addr(crcr_va).expect("Mapped")
            });
            if self.operational_regs.command_ring_control.read() & 0b1000 == 0 {
                self.operational_regs.command_ring_control.write(crcr_pa.as_u64() | 0x1u64);
            } else {
                panic!("[XHCI] CrCr Running");
            }
        } else {
            panic!("[XHCI] CrCr already setup");
        }
        trace!("[XHCI] CrCr setup");

        self.operational_regs.config.write(self.info.max_slot as u32);
        debug!("[XHCI] Setup USB Config with {} ports", self.info.max_slot);

        // Hardcode value: Only N1 is valid. Refer to xhci manual 5.4.4
        self.operational_regs.device_notification.write(0x2);

        debug!("Starting Controller...");
        // HostErrEnable | Interrupt EN | Host Rest | Run/Stop
        self.operational_regs.command.write(0b1101);

    }

    pub fn extended_capability(&mut self) -> ExtendedCapabilityTags {
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
                ExtendedCapabilityTag::USBLegacySupport{ head:_, bios_own:_, os_own :_} => true,
                _ => false
            }
        });
        match tags {
            Some(t) => {
                if let ExtendedCapabilityTag::USBLegacySupport { head, bios_own, os_own } = t {
                    if !bios_own || os_own {
                        return Err(XHCIError::UnexpectedOwnership);
                    }
                    let write_ptr = (head as *mut u8).wrapping_offset(3);
                    let read_ptr = (head as *const u8).wrapping_offset(2);
                    unsafe { write_ptr.write_volatile(0x1) }; // Claiming Ownership
                    // Now wait
                    // TODO implement timeout
                    while unsafe { read_ptr.read_volatile() } & 0x1 > 1 {}
                    return Ok(())
                }
                panic!("wrong tag type found");
            },
            _ => Err(XHCIError::NoLegacyTag)
        }
    }
}
