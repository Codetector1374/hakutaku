use crate::pci::device::PCIDevice;
use x86_64::{PhysAddr, VirtAddr};
use volatile::{Volatile, ReadOnly};
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use crate::PAGE_TABLE;
use x86_64::instructions::interrupts::without_interrupts;
use x86_64::structures::paging::{Mapper, Page, Size4KiB, PhysFrame, PageTableFlags, MapperAllSizes};
use crate::memory::frame_allocator::FrameAllocWrapper;
use core::ops::Add;
use crate::pci::device::xhci::extended_capability::ExtendedCapabilityTags;

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

impl XHCI {
    pub fn cap_list(&mut self) {
        // The higher 16bits specify the offset from base,
        // unit is **DWORD**
        let hcc1val = self.capability_regs.hcc_param1.read();
        if hcc1val & 0x1 == 1 {
            debug!("[XHCI] controller supports 64 bits!");
        }
        let cap_list_offset = (hcc1val >> 14) & !0b11u32; // LSH 2 so it's in bytes
        debug!("caplist offset {:X}", cap_list_offset);
        let cap_list_base: VirtAddr = self.mmio_virt_base + cap_list_offset as u64;
        let val = unsafe {
            (cap_list_base.as_ptr() as *const u32).read_volatile()
        };

        debug!("[XHCI] thing: {:X}", val);

        let mut tags = ExtendedCapabilityTags::get(cap_list_base.as_u64() as usize);

        for (id,tag) in &mut tags.enumerate() {
            debug!("[XHCI] tag{}: {:?}", id, tag);
        }
    }
}
