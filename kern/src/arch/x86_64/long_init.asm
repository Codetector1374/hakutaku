global long_mode_start
extern kinit

section .text_init.long
bits 64
long_mode_start:
    mov ax, 0
    mov ss, ax
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    ; GOTO RUST KERNEL
    mov rax, kinit
    jmp rax
    hlt