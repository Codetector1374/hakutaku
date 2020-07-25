use acpi::{AcpiHandler, PhysicalMapping};
use x86_64::{PhysAddr, VirtAddr};
use crate::memory::mmio_bump_allocator::GMMIO_ALLOC;
use core::ptr::NonNull;
use x86_64::structures::paging::{Mapper, Size4KiB, Page, PhysFrame, PageTableFlags};
use core::ops::Add;
use crate::memory::frame_allocator::FrameAllocWrapper;
use crate::memory::paging::PHYSMAP_BASE;

pub struct KernACPIHandler;

impl AcpiHandler for KernACPIHandler {
    unsafe fn map_physical_region<T>(&mut self, physical_address: usize, size: usize) -> PhysicalMapping<T> {
        PhysicalMapping {
            physical_start: physical_address,
            virtual_start: NonNull::new((PHYSMAP_BASE as usize + physical_address) as *mut T).unwrap(),
            region_length: size,
            mapped_length: size,
        }
    }

    // We do not unmap regions
    fn unmap_physical_region<T>(&mut self, _region: PhysicalMapping<T>) {}
}