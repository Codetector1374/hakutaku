section .multiboot_header
header_start:
    dd 0xe85250d6                ; magic number (multiboot 2)
    dd 0 ; Arch: x86
    dd header_end - header_start ; Length
    dd 0x100000000 - (0xe85250d6 + 0 + (header_end - header_start))

; ==============
; END TAG
; ==============
    dw 0    ; type
    dw 0    ; flags
    dd 8    ; size
header_end:
