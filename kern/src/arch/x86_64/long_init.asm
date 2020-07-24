global long_mode_start
extern kinit

section .kstack
align 16
resb 1024 * 16
kstack_top:

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
    mov rsp, kstack_top
    mov rax, kinit
    jmp rax
    hlt