ENTRY(_start)

SECTIONS {
    . = 1M;

    .boot :
    {
        KEEP(*(.multiboot_header))
    }

    .text :
    {
        KEEP(*(.text.init))
        *(.text .text.*)
    }

    .rodata : {
        *(.rodata .rodata.*)
    }

    .data.rel.ro : {
        *(.data.rel.ro.local*) *(.data.rel.ro .data.rel.ro.*)
    }
}
