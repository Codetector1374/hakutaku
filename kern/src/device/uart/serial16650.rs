use x86_64::instructions::port::Port;
use x86_64::structures::paging::{OffsetPageTable, Mapper, Page, Size4KiB, PhysFrame, PageTableFlags};
use x86_64::{VirtAddr, PhysAddr};

use crate::device::uart::serial16650::Serial16650Base::{MMIO, PMIO};
use crate::device::uart::{UART, SERIAL_PORTS};
use crate::device::pci::device::PCIDevice;
use crate::memory::mmio_bump_allocator::VMALLOC;
use crate::memory::frame_allocator::FrameAllocWrapper;
use x86_64::instructions::interrupts::without_interrupts;
use alloc::sync::Arc;
use spin::Mutex;
use core_io::{Write, Read};

const OFFSET_DATA: u16 = 0;
const OFFSET_INTEN: u16 = 1;
const OFFSET_DIV_LB: u16 = 0;
const OFFSET_DIV_HB: u16 = 1;
const OFFSET_FCR: u16 = 2;
const OFFSET_LCR: u16 = 3;
const OFFSET_MCR: u16 = 4;
const OFFSET_LSR: u16 = 5;
const OFFSET_MSR: u16 = 6;
const OFFSET_SCRATCH: u16 = 7;

pub const COM1_BASE_ADDR: u16 = 0x3F8;

enum Serial16650Base {
    MMIO(VirtAddr),
    PMIO(u16),
}

pub struct Serial16650 {
    base_addr: Serial16650Base
}

impl Serial16650 {
    pub fn new_from_port(base: u16) -> Self {
        Self {
            base_addr: PMIO(base)
        }
    }

    pub fn new_from_mmio(mmio_base: VirtAddr) -> Self {
        Self {
            base_addr: MMIO(mmio_base)
        }
    }

    fn initialize(&mut self) {
        self.write_register(OFFSET_INTEN, 0); // Disables ALL interrupts
        self.write_register(OFFSET_LCR, 0x3); // 8 + No Parity + 1 Stop
        self.write_register(OFFSET_FCR, 0xC7); // FIFO Enable & Reset
    }

    fn read_register(&self, offset: u16) -> u8 {
        unsafe {
            match self.base_addr {
                PMIO(base) => {
                    x86_64::instructions::port::Port::new(base + OFFSET_SCRATCH).read()
                }
                MMIO(base) => {
                    *((base + 0x280u64 + ((offset as u64) * 4)).as_u64() as *mut u32) as u8
                }
            }
        }
    }

    fn write_register(&mut self, offset: u16, value: u8) {
        unsafe {
            match self.base_addr {
                PMIO(base) => {
                    x86_64::instructions::port::Port::new(base + OFFSET_SCRATCH).write(value);
                }
                MMIO(base) => {
                    *((base + 0x280u64 + ((offset as u64) * 4)).as_u64() as *mut u32) = value as u32
                }
            }
        }
    }

    pub fn verify(&mut self) -> bool {
        const MAGIC_VALUE: u8 = 0xAA;
        self.write_register(OFFSET_SCRATCH, MAGIC_VALUE);
        self.read_register(OFFSET_SCRATCH) == MAGIC_VALUE
    }
}

impl Write for Serial16650 {
    fn write(&mut self, buf: &[u8]) -> core_io::Result<usize> {
        for b in buf {
            self.write_byte(*b)
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> core_io::Result<()> {
        while self.read_register(OFFSET_LSR) & 0x40 == 0 {}
        Ok(())
    }
}

impl Read for Serial16650 {
    fn read(&mut self, buf: &mut [u8]) -> core_io::Result<usize> {
        let mut total = 0usize;
        while self.has_data() {
            buf[total] = self.read_byte().unwrap();
            total += 1;
        }
        Ok(total)
    }
}

impl core::fmt::Write for Serial16650 {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.as_bytes() {
            if *b == '\n' as u8 {
                self.write_byte('\r' as u8);
            }
            self.write_byte(*b);
        }
        Ok(())
    }
}

impl UART for Serial16650 {
    fn set_baudrate(&mut self, baud: u32) {
        let factor = match baud {
            115200 => 1u16,
            _ => 1u16, // Default
        };

        without_interrupts(|| {
            let mut lcr_tmp = self.read_register(OFFSET_LCR);
            lcr_tmp |= 0x80;

            self.write_register(OFFSET_LCR, lcr_tmp);
            self.write_register(OFFSET_DIV_LB, factor as u8);
            self.write_register(OFFSET_DIV_HB, (factor >> 8) as u8);

            lcr_tmp &= !0x80u8;
            self.write_register(OFFSET_LCR, lcr_tmp);

        })
    }

    fn write_byte(&mut self, byte: u8) {
        while self.read_register(OFFSET_LSR) & 0x20 == 0 {}
        self.write_register(OFFSET_DATA, byte);
    }

    fn read_byte(&mut self) -> Option<u8> {
        if self.has_data() {
            return Some(self.read_register(OFFSET_DATA))
        }
        None
    }

    fn has_data(&self) -> bool {
        self.read_register(OFFSET_LSR) & 0x1 != 0
    }
}

pub fn pci_load_16650_serial(dev: PCIDevice) {
    let (vmalloc_va, vmalloc_size) = VMALLOC.lock().allocate(10);
    debug!("[UART] VMALLOC returned ({:?}, {})", vmalloc_va, vmalloc_size);
    let mmio_pa = PhysAddr::new(dev.read_config_bar_register(1) as u64);
    let mmio_pa_page_boundary = mmio_pa.align_down(4096u64);
    debug!("[UART] 16650 PA at {:#x}", mmio_pa);

    let mut falloc = FrameAllocWrapper {};
    unsafe {
        crate::PAGE_TABLE.write().map_to(
            Page::<Size4KiB>::from_start_address(vmalloc_va).unwrap(),
            PhysFrame::<Size4KiB>::from_start_address(mmio_pa_page_boundary).unwrap(),
            PageTableFlags::PRESENT | PageTableFlags::NO_CACHE | PageTableFlags::WRITABLE,
            &mut falloc,
        ).expect("unable to map").flush();
    };

    let mmio_offset = (mmio_pa - mmio_pa_page_boundary.as_u64()).as_u64();
    let mmio_base = vmalloc_va + mmio_offset;

    let mut uart = Serial16650::new_from_mmio(mmio_base);

    if !uart.verify() {
        warn!("[UART] 16650 Driver Failed Verification.");
        return
    }
    info!("[UART] 16650 Driver Loaded and Verified.");
    uart.set_baudrate(115200);
    uart.initialize();

    SERIAL_PORTS.write().register_port(Arc::new(Mutex::new(uart)));
}
