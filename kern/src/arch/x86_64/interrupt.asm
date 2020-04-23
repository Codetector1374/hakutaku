section .text
bits 64

global save_context
extern handle_context_switch

save_context:
    push r15
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
    mov rdi, rsp
    call handle_context_switch

restore_cotext:
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