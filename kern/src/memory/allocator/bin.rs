use core::alloc::Layout;
use core::fmt;
use core::fmt::{Debug, Formatter, Error};
use core::ptr;

use crate::memory::allocator::linked_list::LinkedList;
use crate::memory::allocator::LocalAlloc;
use crate::memory::*;

/// A simple allocator that allocates based on size classes.
///   bin 0 (2^3 bytes)    : handles allocations in (0, 2^3]
///   bin 1 (2^4 bytes)    : handles allocations in (2^3, 2^4]
///   ...
///   bin 29 (2^22 bytes): handles allocations in (2^31, 2^32]
///   
///   map_to_bin(size) -> k
///   

pub struct Allocator {
    bins: [LinkedList; 30],
    block_start: usize,
    block_current: usize,
    block_end: usize,
}

impl Allocator {
    /// Creates a new bin allocator that will allocate memory from the region
    /// starting at address `start` and ending at address `end`.
    pub fn new(start: usize, end: usize) -> Allocator {
        Allocator {
            bins: [LinkedList::new(); 30],
            block_start: start,
            block_current: start,
            block_end: end,
        }
    }
}

fn get_bin_size(bin_number: usize) -> usize {
    1usize << (bin_number + 3)
}

// TODO Optimize
fn get_bin_number(size: usize) -> usize {
    match size {
        0x0..=0x8 => 0,
        0x9..=0x10 => 1,
        0x11..=0x20 => 2,
        0x21..=0x40 => 3,
        0x41..=0x80 => 4,
        0x81..=0x100 => 5,
        0x101..=0x200 => 6,
        0x201..=0x400 => 7,
        0x401..=0x800 => 8,
        0x801..=0x1000 => 9,
        0x1001..=0x2000 => 10,
        0x2001..=0x4000 => 11,
        0x4001..=0x8000 => 12,
        0x8001..=0x10000 => 13,
        0x10001..=0x20000 => 14,
        0x20001..=0x40000 => 15,
        0x40001..=0x80000 => 16,
        0x80001..=0x100000 => 17,
        0x100001..=0x200000 => 18,
        0x200001..=0x400000 => 19,
        0x400001..=0x800000 => 20,
        0x800001..=0x1000000 => 21,
        0x1000001..=0x2000000 => 22,
        0x2000001..=0x4000000 => 23,
        0x4000001..=0x8000000 => 24,
        0x8000001..=0x10000000 => 25,
        0x10000001..=0x20000000 => 26,
        0x20000001..=0x40000000 => 27,
        0x40000001..=0x80000000 => 28,
        0x80000001..=0x100000000 => 29,
        _ => 30,
    }
}

impl Allocator {
    unsafe fn alloc_from_block(&mut self, layout: Layout, bin_number: usize) -> *mut u8 {
        let aligned
            = align_up(self.block_current, layout.align());
        let target = aligned.saturating_add(get_bin_size(bin_number));
        if target - get_bin_size(bin_number) != aligned ||
            target > self.block_end {
            return core::ptr::null_mut();
        }
        self.block_current = target;

        return aligned as *mut u8;
    }

    unsafe fn alloc_from_bin(&mut self, layout: Layout, bin_number: usize) -> *mut u8 {
        let bin: &mut LinkedList = &mut self.bins[bin_number];
        let align_check = layout.align() - 1;
        for chunk in bin.iter_mut() {
            if (chunk.value() as usize) & align_check == 0 {
                // Aligned
                let rtn = chunk.pop() as *mut u8;
                assert!((rtn as usize) >= self.block_start);
                assert!((rtn as usize) <= self.block_end);
                return rtn;
            }
        };
        core::ptr::null_mut()
    }

    unsafe fn break_up_chunk(&mut self, chunk_start: usize, chunk_size: usize, bin_number: usize) {
        let part_size = get_bin_size(bin_number);
        if part_size == 0 {
            panic!("FUCK");
        }
        assert_eq!(0, chunk_size%part_size);
        for i in 0..(chunk_size/part_size) {
            self.bins[bin_number].push((chunk_start + (i*part_size)) as *mut usize);
        }
    }
}

impl LocalAlloc for Allocator {
    /// Allocates memory. Returns a pointer meeting the size and alignment
    /// properties of `layout.size()` and `layout.align()`.
    ///
    /// If this method returns an `Ok(addr)`, `addr` will be non-null address
    /// pointing to a block of storage suitable for holding an instance of
    /// `layout`. In particular, the block will be at least `layout.size()`
    /// bytes large and will be aligned to `layout.align()`. The returned block
    /// of storage may or may not have its contents initialized or zeroed.
    ///
    /// # Safety
    ///
    /// The _caller_ must ensure that `layout.size() > 0` and that
    /// `layout.align()` is a power of two. Parameters not meeting these
    /// conditions may result in undefined behavior.
    ///
    /// # Errors
    ///
    /// Returning null pointer (`core::ptr::null_mut`)
    /// indicates that either memory is exhausted
    /// or `layout` does not meet this allocator's
    /// size or alignment constraints.
    unsafe fn alloc(&mut self, layout: Layout) -> *mut u8 {
        let bin_number = get_bin_number(layout.size());
        if bin_number >= 30 {
            return core::ptr::null_mut();
        }

        let align_check = layout.align() - 1;
        let alloc_from_bin = self.alloc_from_bin(layout, bin_number);
        if alloc_from_bin != core::ptr::null_mut() {
            return alloc_from_bin;
        }

        let alloc_from_block = self.alloc_from_block(layout, bin_number);

        if alloc_from_block != core::ptr::null_mut() {
            return alloc_from_block;
        };

        // Time to break blocks
        let alloc_bin_size = get_bin_size(bin_number);
        for bin_num in (bin_number + 1)..29 {
            let bin_size = get_bin_size(bin_num);
            let big_bin = &mut self.bins[bin_num];
            for chunk_in_bin in big_bin.iter_mut() {
                assert_eq!(bin_size%alloc_bin_size, 0);
                let chunk_base = chunk_in_bin.value() as usize;
                for subchunk_num in 0..(bin_size/alloc_bin_size) {
                    if (chunk_base + (subchunk_num * alloc_bin_size)) & align_check == 0 {
                        // Break this block
                        self.break_up_chunk(chunk_in_bin.pop() as usize, bin_size, bin_num);
                        let bin_alloc_after_break = self.alloc_from_bin(layout, bin_num);
                        assert!(!bin_alloc_after_break.is_null());
                        return bin_alloc_after_break;
                    }
                }
            }
        }

        core::ptr::null_mut()
    }

    /// Deallocates the memory referenced by `ptr`.
    ///
    /// # Safety
    ///
    /// The _caller_ must ensure the following:
    ///
    ///   * `ptr` must denote a block of memory currently allocated via this
    ///     allocator
    ///   * `layout` must properly represent the original layout used in the
    ///     allocation call that returned `ptr`
    ///
    /// Parameters not meeting these conditions may result in undefined
    /// behavior.
    unsafe fn dealloc(&mut self, ptr: *mut u8, layout: Layout) {
        let bin_num = get_bin_number(layout.size());
        self.bins[bin_num].push(ptr as *mut usize);
    }
}

impl Debug for Allocator {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), Error> {
        f.write_fmt(format_args!("Bin Allocator:
        BlockStart: {}
        BlockCurrent: {}
        BlockEnd: {}
        Bins::", self.block_start, self.block_current, self.block_end))?;
        for (i, bin) in self.bins.iter().enumerate() {
            f.write_fmt(format_args!("\
              Bin [{}] chunk size: [{}]:
                {:?}\n", i, get_bin_size(i), bin))?;
        }
        Ok(())
    }
}
