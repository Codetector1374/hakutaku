use x86_64::VirtAddr;
use spin::Mutex;
use crate::memory::paging::{VMALLOC_BASE, VMALLOC_TOP};

pub static VMALLOC: Mutex<MMIOAllocator> = Mutex::new(
    MMIOAllocator::new(
        VirtAddr::new_truncate(VMALLOC_BASE),
        VirtAddr::new_truncate(VMALLOC_TOP)
    ));

pub struct MMIOAllocator {
    base_addr: VirtAddr,
    current_addr: VirtAddr,
    top_addr: VirtAddr,
}

impl MMIOAllocator {
    pub const fn new(base: VirtAddr, top: VirtAddr) -> MMIOAllocator {
        MMIOAllocator {
            base_addr: base,
            current_addr: base,
            top_addr: top,
        }
    }

    /// size will be round up to 4096
    pub fn allocate(&mut self, req_size: usize) -> (VirtAddr, usize) {
        let num_of_pages= (req_size + 4095) / 4096;
        let alloc_size = num_of_pages * 4096;
        assert!(self.current_addr.is_aligned(4096u64), "VMALLOC Current Alignment");
        let alloc_addr = self.current_addr;
        self.current_addr += alloc_size;
        assert!(alloc_size >= req_size, "VMALLOC Allocated less than requested");
        (alloc_addr, alloc_size)
    }

}