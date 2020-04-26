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

pub mod extended_capability;

#[derive(Debug)]
pub struct XHCI {
    pub pci_device: PCIDevice,
    pub mmio_base: PhysAddr,
    pub mmio_virt_base: VirtAddr,
    pub mmio_size: usize,
    pub capability_regs: &'static mut XHCICapabilityRegister,
}

#[derive(Debug)]
#[repr(C)]
pub struct XHCICapabilityRegister {
    pub length_and_ver: ReadOnly<u32>,
    pub hcs_params: [ReadOnly<u32>; 3],
    pub hcc_param1: ReadOnly<u32>,
    pub doorbell_offset: ReadOnly<u32>,
    pub rts_offset: ReadOnly<u32>,
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
                trace!("Mapping offset: {}", offset);
                let paddr = alloc_base + offset;
                let vaddr = va + offset;
                unsafe {
                    PAGE_TABLE.lock().map_to(
                        Page::<Size4KiB>::from_start_address(vaddr).expect("Unaligned VA"),
                        PhysFrame::from_start_address(paddr).expect("Unaligned PA"),
                        PageTableFlags::WRITABLE | PageTableFlags::PRESENT ,
                        &mut fallocw,
                    ).expect("Unable to map")
                }.flush();
            }
            va
        });
        let mmio_vbase = va_root + base_offset;
        without_interrupts(|| {
            let base = PAGE_TABLE.lock().translate_addr(mmio_vbase).expect("phy");
            info!("MMIO Phy Base: {:?}", base);
        });
        XHCI {
            pci_device: dev,
            mmio_base: base,
            mmio_virt_base: mmio_vbase,
            mmio_size: size,
            capability_regs: unsafe { &mut *(mmio_vbase.as_mut_ptr()) },
        }
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
    pub fn hcc_flags(&mut self) -> HCCFlags {
        let val = self.capability_regs.hcc_param1.read();
        HCCFlags::from_bits_truncate(val)
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
