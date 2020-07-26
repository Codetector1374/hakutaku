global _long_mode_start
global _ap_long_mode_start
global __ap_stack_top

extern kinit
extern ap_entry

section .kstack
align 16
resb 4096 * 4
kstack_top:

section .text_init.long
bits 64

align 8
__ap_stack_top:
    resb 8

_long_mode_start:
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


_ap_long_mode_start:
    mov ax, 0
    mov ss, ax
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    ; GOTO RUST
    mov rsp, [__ap_stack_top]
    mov rax, ap_entry
    jmp rax
    hlt


.loop:
    mov byte [0xb8001], 2
    jmp _ap_long_mode_start.loop