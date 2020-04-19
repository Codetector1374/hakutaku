#![no_std]
#![feature(global_asm)]
#![feature(ptr_internals)]
#![feature(asm)]
#![feature(panic_info_message)]
#![feature(abi_x86_interrupt)]
#![allow(unused_imports, dead_code)]

use core::fmt::Write;
use crate::interrupts::PICS;

extern crate cpuio;
extern crate spin;
extern crate multiboot2;
extern crate x86_64;
extern crate pc_keyboard;

#[macro_use]
pub mod vga_buffer;
pub mod init;
pub mod hardware;
pub mod interrupts;
pub mod gdt;

fn kern_init() {

    println!("Re-Initializing GDT in Rust");
    gdt::init();
    println!("Setting Exception Handlers");
    interrupts::init_idt();
    println!("Success!");
    x86_64::instructions::interrupts::int3();
    println!("Continue Execution after int3!");
    println!("Initializing PICs");
    unsafe {
        PICS.lock().initialize();
    };
    x86_64::instructions::interrupts::enable();
}

#[no_mangle]
pub extern "C" fn kinit(multiboot_ptr: usize) -> !{
    println!("Hello World \n Multiboot: 0x{:08X}", multiboot_ptr);
    let boot_info = unsafe { multiboot2::load(multiboot_ptr) };

    let mem_tags = boot_info.memory_map_tag().expect("No Mem Tags");
    println!("==== BEGIN MEM =====");
    for (id, m) in mem_tags.memory_areas().enumerate() {
        println!("Chunk {}: 0x{:08X} - 0x{:08X} - {:?}", id, m.start_address(), m.size(), m.typ());
    }
    println!("===== END MEM ======");
    let elf_sections_tag = boot_info.elf_sections_tag()
        .expect("Elf-sections tag required");

    println!("==== BEGIN KERN INFO =====");
    let elf_sections_tag = boot_info.elf_sections_tag().expect("ELF is required");
    let kernel_start = elf_sections_tag.sections().map(|s| s.start_address())
        .min().unwrap();
    let kernel_end = elf_sections_tag.sections().map(|s| s.start_address() + s.size())
        .max().unwrap();
    let multiboot_start = boot_info.start_address();
    let multiboot_end = multiboot_start + (boot_info.total_size() as usize);
    println!("Kern: 0x{:08X} -> 0x{:08X}", kernel_start, kernel_end);
    println!("MB: 0x{:08X} -> 0x{:08X}", multiboot_start, multiboot_end);

    

    kern_init();

    unsafe {
        let ptr = 0xb0bacafe as *mut u32;
        *ptr = 0xdead;
    }

    loop {
        unsafe {
            asm!("hlt" :::: "volatile");
        }
    }
}
