bits 64
section .text.usertest

global user_mode_test
global user_mode_test_end

user_mode_test:
    mov rax, 1
    mov rdi, 1000
    int 0x80
    jmp user_mode_test
user_mode_test_end: