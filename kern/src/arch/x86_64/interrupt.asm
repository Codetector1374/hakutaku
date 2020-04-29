section .text
bits 64

global apic_timer
global syscall_handler
global restore_context_wrapper
extern handle_context_switch

apic_timer:
    push r15
    mov r15, 1
    jmp save_context
syscall_handler:
    push r15
    mov r15, 0x80
    jmp save_context


save_context:
    push r14
    push r13
    push r12
    push r11
    push r10
    push r9
    push r8
    push rbp
    push rdi
    push rsi
    push rbx
    push rdx
    push rcx
    push rax
    ; Setup Call
    mov rsi, r15
    mov rdi, rsp
    call handle_context_switch

restore_context:
    pop rax
    pop rcx
    pop rdx
    pop rbx
    pop rsi
    pop rdi
    pop rbp
    pop r8
    pop r9
    pop r10
    pop r11
    pop r12
    pop r13
    pop r14
    pop r15
    iretq

restore_context_wrapper:
    add rsp, 8
    jmp restore_context