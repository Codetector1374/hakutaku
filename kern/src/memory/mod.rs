pub mod frame_allocator;
pub mod paging;
pub mod allocator;


// Utils


/// Align `addr` downwards to the nearest multiple of `align`.
///
/// The returned usize is always <= `addr.`
///
/// # Panics
///
/// Panics if `align` is not a power of 2.
pub fn align_down(addr: usize, align: usize) -> usize {
    if align == 0 || align & (align - 1) != 0 {
        panic!("alignment {} is not a power of two", align);
    }
    let rtn = addr & !(align - 1);
    return rtn;
}

/// Align `addr` upwards to the nearest multiple of `align`.
///
/// The returned `usize` is always >= `addr.`
///
/// # Panics
///
/// Panics if `align` is not a power of 2
/// or aligning up overflows the address.
pub fn align_up(addr: usize, align: usize) -> usize {
    if align == 0 || align & (align - 1) != 0 {
        panic!("alignment {} is not a power of two", align);
    }
    if (addr & (align - 1)) == 0 {
        addr
    } else {
        let new = align_down(addr + align, align);
        if new <= addr {
            panic!("Align up causes overflow");
        }
        new
    }
}

pub fn is_aligned(addr: usize, align: usize) -> bool {
    if align == 0 || align & (align - 1) != 0 {
        panic!("alignment {} is not a power of two", align);
    }
    return addr & (align - 1) == 0;
}
