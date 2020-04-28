use x86_64::VirtAddr;
use spin::Mutex;

// TODO Remove Hard Coded Value

pub const MMIO_BASE: u64 = 0xFFFF_FE00_0000_0000;

pub static GMMIO_ALLOC: Mutex<MMIOAllocator> = Mutex::new(
    MMIOAllocator::new(
        VirtAddr::new_truncate(0xFFFF_FEF0_0000_0000),
        VirtAddr::new_truncate(MMIO_BASE)
    ));

pub struct MMIOAllocator {
    top_address: VirtAddr,
    current_addr: VirtAddr,
    limit_addr: VirtAddr,
}

impl MMIOAllocator {
    pub const fn new(top: VirtAddr, limit: VirtAddr) -> MMIOAllocator {
        MMIOAllocator {
            top_address: top,
            current_addr: top,
            limit_addr: limit,
        }
    }

    /// size will be round up to 4096
    pub fn allocate(&mut self, size: usize) -> (VirtAddr, usize) {
        let num_of_pages= (size + 4095) / 4096;
        let size = num_of_pages * 4096;
        self.current_addr = VirtAddr::new(self.current_addr.as_u64() - size as u64);
        (self.current_addr,size)
    }

}