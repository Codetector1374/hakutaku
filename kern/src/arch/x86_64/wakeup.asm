extern _ap_start

bits 16
section .smp

core_wakeup:
    cli                     ; Disable interrupts, we want to be alone

    xor ax, ax
    mov ds, ax              ; Set DS-register to 0 - used by lgdt

    lgdt [gdt_desc]         ; Load the GDT descriptor

    mov eax, cr0            ; Copy the contents of CR0 into EAX
    or eax, 1               ; Set bit 0
    mov cr0, eax            ; Copy the contents of EAX into CR0

    jmp 08h:smp_protected_entry

gdt:                    ; Address for the GDT
gdt_null:               ; Null Segment
        dd 0
        dd 0

gdt_code:               ; Code segment, read/execute, nonconforming
        dw 0xFFFF
        dw 0
        db 0
        db 10011010b
        db 11001111b
        db 0

gdt_data:               ; Data segment, read/write, expand down
        dw 0xFFFF
        dw 0
        db 0
        db 10010010b
        db 11001111b
        db 0

gdt_end:                ; Used to calculate the size of the GDT

gdt_desc:                       ; The GDT descriptor
        dw gdt_end - gdt - 1    ; Limit (size)
        dd gdt                  ; Address of the GDT

bits 32
section .smp.protected

smp_protected_entry:
    mov ax, 0x10
    mov ds, ax
    mov ss, ax

    jmp _ap_start
