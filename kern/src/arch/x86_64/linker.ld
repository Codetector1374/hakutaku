ENTRY(_start)

SECTIONS {
    .multiboot : {
        KEEP(*(.multiboot_header))
    }
    . = 0x8000;

    __smp_start = .;
    .smp : {
        KEEP(*(.smp .smp.*))
    }
    __smp_end = .;

    . = 1M;

    .boot :
    {
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
