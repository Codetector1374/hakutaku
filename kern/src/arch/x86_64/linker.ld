ENTRY(_start)

SECTIONS {
    . = 1M;

    .boot :
    {
        KEEP(*(.multiboot_header))
        KEEP(*(.text_init .text_init.*))
    }

    .rodata.init : {
        *(.rodata.init)
    }

    .bss_init :
    {
        *(.bss_init)
    }

    . = 0xFFFF800000000000;

    __kernel_start = .;
    .kernel : AT ( 0x200000 )
    {
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.data.rel.ro.local*) *(.data.rel.ro .data.rel.ro.*)
        *(*)
    }
    . = ALIGN(4K);

    __kernel_end = .;
}
