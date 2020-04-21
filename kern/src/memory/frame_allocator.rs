use x86_64::PhysAddr;
use x86_64::structures::paging::{FrameAllocator, PhysFrame, Size4KiB};
use crate::memory::is_aligned;
use stack_vec::StackVec;
use core::borrow::BorrowMut;
use crate::FRAME_ALLOC;

const FRAME_SIZE: usize = 4096;

#[derive(Copy, Clone)]
pub struct MemorySegment {
    start: usize,
    size: usize,
    current: usize,
}

impl Default for MemorySegment {
    fn default() -> Self {
        MemorySegment {
            start: 0,
            size: 0,
            current: 0,
        }
    }
}

impl MemorySegment {
    pub fn new(start: usize, size: usize) -> Result<MemorySegment, ()> {
        if is_aligned(start, FRAME_SIZE) {
            Ok(MemorySegment {
                start,
                size,
                current: start,
            })
        } else {
            Err(())
        }
    }

    pub fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        if self.size - (self.current - self.start) >= 4096 { // 4KB
            let val = PhysFrame::<Size4KiB>::from_start_address(PhysAddr::new(self.current as u64)).expect("Alignment");
            self.current += 4096;
            Some(val)
        } else {
            None
        }
    }
}

pub struct SegmentFrameAllocator {
    segments: [MemorySegment; 16],
    count: usize,
}

impl SegmentFrameAllocator {
    pub fn new() -> SegmentFrameAllocator {
        let mut alloc = SegmentFrameAllocator {
            segments: [MemorySegment::default(); 16],
            count: 0,
        };
        alloc
    }

    pub fn add_segment(&mut self, segment: MemorySegment) {
        if self.count < 16 {
            self.segments[self.count] = segment;
            self.count += 1;
        }
    }

    pub fn total_space(&self) -> usize {
        let mut size = 0usize;
        for seg in self.segments[0..self.count].iter() {
            size += seg.size;
        }
        size
    }

    pub fn free_space(&self) -> usize {
        let mut size = 0usize;
        for seg in self.segments[0..self.count].iter() {
            size += seg.size - (seg.current - seg.start);
        }
        size
    }
}

unsafe impl FrameAllocator<Size4KiB> for SegmentFrameAllocator {
    fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        for s in self.segments[0..self.count].as_mut().iter_mut() {
            if let Some(frame) = s.allocate_frame() {
                return Some(frame);
            }
        }
        None
    }
}

pub struct FrameAllocWrapper;

unsafe impl FrameAllocator<Size4KiB> for FrameAllocWrapper {
    fn allocate_frame(&mut self) -> Option<PhysFrame<Size4KiB>> {
        FRAME_ALLOC.lock().allocate_frame()
    }
}