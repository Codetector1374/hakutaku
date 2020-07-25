/*
========================================================================================================================
    Start addr    |   Offset   |     End addr     |  Size   | VM area description
========================================================================================================================
                  |            |                  |         |
 0000000000000000 |    0       | 00007fffffffffff |  128 TB | user-space virtual memory, different per mm
__________________|____________|__________________|_________|___________________________________________________________
                                                            |
                                                            | Kernel-space virtual memory, shared between all processes:
____________________________________________________________|___________________________________________________________
                  |            |                  |         |
 ffff800000000000 | -128    TB | ffffbfffffffffff |   64 TB | physical memory map
 ffffc00000000000 |  -64    TB | ffffdfffffffffff |   32 TB | vmalloc/ioremap space (vmalloc_base)
 ffffe00000000000 |  -32    TB | ffffefffffffffff |    1 TB | ... unused hole
 ffffea0000000000 |  -22    TB | ffffeaffffffffff |    1 TB | virtual memory map (vmemmap_base)
 ffffeb0000000000 |  -21    TB | ffffebffffffffff |    1 TB | ... unused hole
 ffffffff80000000 |   -2    GB | ffffffff9fffffff |  512 MB | kernel text mapping, mapped to physical address 0
 ffffffffa0000000 |   -1.5  GB | ffffffffbfffffff |  512 MB | kernel heap
__________________|____________|__________________|_________|___________________________________________________________
 */


use spin::{RwLock, Mutex};
use x86_64::structures::paging::PageTable;
use alloc::boxed::Box;
use x86_64::PhysAddr;

extern "C" {
    static mut __kernel_pdps: u64;
}

pub static KERNEL_PML4_TABLE: Mutex<Option<Box<PageTable>>> = Mutex::new(None);

pub const PHYSMAP_BASE: u64     = 0xFFFF8000_00000000;
pub const KERNEL_TEXT_BASE: u64 = 0xFFFFFFFF_80000000;
pub const KERNEL_HEAP_BASE: u64 = 0xFFFFFFFF_a0000000;

lazy_static! {
    pub static ref KERNEL_PDPS: RwLock<Box<[PageTable; 256]>> = {
        RwLock::new(unsafe {
            Box::from_raw(&__kernel_pdps as *const u64 as *mut [PageTable; 256])
        })
    };
}
