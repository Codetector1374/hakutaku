use lazy_static::lazy_static;
use x86_64::VirtAddr;
use x86_64::structures::tss::TaskStateSegment;
use x86_64::structures::gdt::{GlobalDescriptorTable, Descriptor, SegmentSelector};
use x86_64::instructions::segmentation::set_cs;
use x86_64::instructions::tables::load_tss;
use alloc::boxed::Box;

pub const DOUBLE_FAULT_IST_INDEX: usize = 0;

struct Selectors {
    code_selector: SegmentSelector,
    user_cs: SegmentSelector,
    user_ds: SegmentSelector,
    tss_selector: SegmentSelector,
}

pub struct GDTInfo {
    gdt: Box<GlobalDescriptorTable>,
    selectors: Selectors,
}

impl GDTInfo {
    pub unsafe fn get_static_gdt(&self) -> &'static GlobalDescriptorTable {
         & *(self.gdt.as_ref() as *const GlobalDescriptorTable)
    }

    pub unsafe fn load(&self) {
        self.get_static_gdt().load();
        set_cs(self.selectors.code_selector);
        load_tss(self.selectors.tss_selector);
    }
}

pub fn create_gdt(tss: &'static TaskStateSegment) -> GDTInfo {
    let mut gdt = Box::new(GlobalDescriptorTable::new());
    let code_selector = gdt.add_entry(Descriptor::kernel_code_segment());
    let user_cs = gdt.add_entry(Descriptor::user_code_segment());
    let user_ds = gdt.add_entry(Descriptor::user_data_segment());
    let tss_selector = gdt.add_entry(Descriptor::tss_segment(tss));
    GDTInfo {
        gdt,
        selectors: Selectors {
            code_selector,
            user_cs,
            user_ds,
            tss_selector,
        },
    }
}

/* ============================ TSS ============================================================= */

pub struct TSSInfo {
    stack: Box<[u8; 1024]>,
    tss: Box<TaskStateSegment>,
}

pub fn create_tss() -> TSSInfo {
    let stack = Box::new([0u8; 1024]);
    let mut tss = Box::new(TaskStateSegment::new());
    tss.interrupt_stack_table[DOUBLE_FAULT_IST_INDEX] =
        (VirtAddr::from_ptr(stack.as_ptr()) + stack.len() as u64 - 1u64).align_down(8u64);
    TSSInfo {
        stack,
        tss,
    }
}

impl TSSInfo {
    pub fn get_tss_ptr(&self) -> &'static TaskStateSegment {
        unsafe { &*(self.tss.as_ref() as *const TaskStateSegment) }
    }
}