use crate::PAGE_TABLE;
use crate::pci::device::PCIDevice;
use crate::pci::class::*;
use x86_64::PhysAddr;
use x86_64::instructions::interrupts::without_interrupts;
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use crate::pci::class::PCIClassMassStroageSATA::AHCI;
use x86_64::structures::paging::{Mapper, Size4KiB, Page, PhysFrame, PageTableFlags};
use volatile::{ReadOnly, Volatile, WriteOnly};

const AHCI_MEMORY_REGION_SIZE: usize = 0x1100;

pub struct AHCIController {
    dev: PCIDevice,
    physical_base_addr: PhysAddr,
    regs: Option<&'static mut AHCIRegisters>,
}

#[repr(C)]
struct AHCIRegisters {
    generic_control: AHCIGenericHostControl,
}

#[repr(C)]
#[allow(non_snake_case)]
struct AHCIGenericHostControl {
    CAP: ReadOnly<u32>, // Host(HBA) Capabilities
    GHC: Volatile<u32>, // Global HBA Control

}

impl AHCIController {
    pub fn create_from_device(dev: PCIDevice) -> Option<AHCIController> {
        if let PCIDeviceClass::MassStorageController(PCIClassMassStorage::SATA(PCIClassMassStroageSATA::AHCI)) = dev.info.class {
            let mut controller = AHCIController {
                dev,
                physical_base_addr: PhysAddr::new(0),
                regs: None,
            };
            controller.internal_initialize();
            return Some(controller);
        }
        debug!("[AHCI] create_from_device: not AHCI device, {:?}", dev.info.class);
        None
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

        let ports = self.port_count();
        debug!("[AHCI] Controller has {} ports", ports);

    }

    pub fn port_count(&self) -> u8 {
        let cap = self.regs.as_ref().expect("initialized").generic_control.CAP.read();
        return ((cap & 0b11111) as u8) + 1;
    }
}