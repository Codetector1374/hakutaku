use acpi::{AcpiHandler, PhysicalMapping};
use x86_64::{PhysAddr, VirtAddr};
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use core::ptr::NonNull;
use x86_64::structures::paging::{Mapper, Size4KiB, Page, PhysFrame, PageTableFlags};
use core::ops::Add;
use crate::memory::frame_allocator::FrameAllocWrapper;

pub struct KernACPIHandler;

impl AcpiHandler for KernACPIHandler {
    unsafe fn map_physical_region<T>(&mut self, physical_address: usize, size: usize) -> PhysicalMapping<T> {
        use crate::PAGE_TABLE;

        let pa_start = PhysAddr::new(physical_address as u64).align_down(4096u64);
        let pa_end: PhysAddr = PhysAddr::new((physical_address + size) as u64).align_down(4096u64);
        let num_page = pa_end - pa_start + 1;

        let va_offset = VirtAddr::new(physical_address as u64 - pa_start.as_u64());

        let (va, alloc_size) = GMMIO_ALLOC.lock().allocate((num_page * 4096) as usize);

        let mut falloc = FrameAllocWrapper {};
        for page in 0..num_page {
            PAGE_TABLE.write().map_to(
                Page::<Size4KiB>::from_start_address(va.add(page * 4096)).expect("va align"),
                PhysFrame::<Size4KiB>::from_start_address(pa_start.add(page * 4096)).expect("pa align"),
                PageTableFlags::PRESENT | PageTableFlags::WRITABLE, &mut falloc,
            ).expect("failed to map").flush();
        }

        PhysicalMapping {
            physical_start: (pa_start + va_offset.as_u64()).as_u64() as usize,
            virtual_start: NonNull::new((va + va_offset.as_u64()).as_mut_ptr()).expect("ptr"),
            region_length: size,
            mapped_length: alloc_size,
        }
    }

    // We do not unmap regions
    fn unmap_physical_region<T>(&mut self, _region: PhysicalMapping<T>) {}
}