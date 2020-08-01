
#[macro_export]
macro_rules! const_assert {
    ($cond:expr) => {
        // Causes overflow if condition is false
        let _ = [(); 0 - (!($cond) as usize)];
    };
    ($($xs:expr),+) => {
        $crate::const_assert!($($xs)&&+);
    };
    ($($xs:expr);+ $(;)*) => {
        $crate::const_assert!($($xs),+);
    };
}

#[macro_export]
macro_rules! const_assert_eq {
    ($x:expr, $($xs:expr),+) => {
        const _: () = { $crate::const_assert!($($x == $xs),+); ()};
    }
}

#[macro_export]
macro_rules! const_assert_size {
    ($struct:ident, $size:expr) => {
        $crate::const_assert_eq!(core::mem::size_of::<$struct>(), ($size));
    }
}

#[macro_export]
macro_rules! pt_translate {
    ($va: expr) => {
        x86_64::instructions::interrupts::without_interrupts(|| {
            use crate::x86_64::structures::paging::MapperAllSizes;
            crate::PAGE_TABLE.read().translate_addr(
                $va
            ).expect("")
        })
    };
}

