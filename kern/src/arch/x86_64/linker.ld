ENTRY(_start)

SECTIONS {
    . = 1M;

    .boot :
    {
        KEEP(*(.multiboot_header))
        KEEP(*(.text_init .text_init.*))
        *(.rodata.init)
        *(.bss_init)
    }

    . = 0xFFFFFFFF80000000 + LOADADDR(.boot) + SIZEOF(.boot); /* -2 GiB */

    __kernel_start = .;
    .kernel : AT (LOADADDR(.boot) + SIZEOF(.boot))
    {
        *(.kstack)
        . = ALIGN(1M);

        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.data.rel.ro.local*) *(.data.rel.ro .data.rel.ro.*)
        *(*)

        . = ALIGN(4K);
        __kernel_pdps = . ;
        . += (4096 * 256);
    }

    __kernel_end = .;

}
