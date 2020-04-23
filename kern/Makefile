arch ?= x86_64
kernel := build/kernel-$(arch).bin
target ?= $(arch)-unknown-none
iso := build/os-$(arch).iso
rust_os := target/$(target)/release/libkern.a
DEVICE ?= /run/media/$(shell whoami)/C8A9-B957
QEMU_ARGS ?= -no-reboot -cdrom $(iso)

linker_script := src/arch/$(arch)/linker.ld
grub_cfg := src/arch/$(arch)/grub.cfg
assembly_source_files := $(wildcard src/arch/$(arch)/*.asm)
assembly_object_files := $(patsubst src/arch/$(arch)/%.asm, \
	build/arch/$(arch)/%.o, $(assembly_source_files))

.PHONY: all clean run iso kernel

all: $(iso)

copy: $(kernel)
	@cp $(grub_cfg) $(DEVICE)/boot/grub/grub.cfg
	cp $(kernel) $(DEVICE)/boot/kernel.bin

kernel:
	@RUST_TARGET_PATH=$(shell pwd) cargo xbuild --release --target x86_64-unknown-none

clean:
	@cargo clean
	@rm -rf build

run: $(iso)
	@qemu-system-x86_64 $(QEMU_ARGS)

gdb: $(iso)
	@qemu-system-x86_64 -s -S $(QEMU_ARGS)

iso: $(iso)

$(iso): $(kernel) $(grub_cfg)
	@mkdir -p build/isofiles/boot/grub
	@cp $(kernel) build/isofiles/boot/kernel.bin
	@cp $(grub_cfg) build/isofiles/boot/grub
	@grub-mkrescue -o $(iso) build/isofiles
	@rm -r build/isofiles

$(kernel): kernel $(assembly_object_files) $(linker_script) $(rust_os)
	@ld -n -T $(linker_script) -o $(kernel) \
	$(assembly_object_files) $(rust_os)

# compile assembly files
build/arch/$(arch)/%.o: src/arch/$(arch)/%.asm
	@mkdir -p $(shell dirname $@)
	@nasm -felf64 $< -o $@
