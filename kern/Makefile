arch ?= x86_64
kernel := build/kernel-$(arch).bin
kernel_elf := build/kernel-$(arch).elf
target ?= $(arch)-unknown-none
iso := build/os-$(arch).iso
rust_os := target/$(target)/release/libkern.a
DEVICE ?= /run/media/$(shell whoami)/C8A9-B957
QEMU ?= qemu-system-x86_64
QEMU_ARGS ?= -M q35 -no-reboot -cdrom $(iso) \
						 -m 1G \
 						 -smp cpus=4 \
						 -serial stdio

linker_script := src/arch/$(arch)/linker.ld
grub_cfg := src/arch/$(arch)/grub.cfg
assembly_source_files := $(wildcard src/arch/$(arch)/*.asm)
assembly_object_files := $(patsubst src/arch/$(arch)/%.asm, \
	build/arch/$(arch)/%.o, $(assembly_source_files))

.PHONY: all clean run iso kernel

all: $(iso)

bochs: $(iso)
	bochs -q

net: $(kernel)
	cp $(kernel) /srv/tftp/kernel.bin

copy: $(kernel)
	@cp $(grub_cfg) $(DEVICE)/boot/grub/grub.cfg
	cp $(kernel) $(DEVICE)/boot/kernel.bin
	sync

kernel:
	cargo xbuild --release --target x86_64-unknown-none.json

clean:
	@cargo clean
	@rm -rf build

usb: $(iso)
	@$(QEMU) $(QEMU_ARGS) -drive id=my_usb_disk,file=./ext/demo_usb.img,if=none \
				    -device usb-storage,drive=my_usb_disk,id=usb0

run: $(iso)
	@$(QEMU) $(QEMU_ARGS)

gdb: $(iso)
	@$(QEMU) -s -S $(QEMU_ARGS)

iso: $(iso)

$(iso): $(kernel) $(grub_cfg)
	@mkdir -p build/isofiles/boot/grub
	@cp $(kernel) build/isofiles/boot/kernel.bin
	@cp $(grub_cfg) build/isofiles/boot/grub
	@grub-mkrescue -o $(iso) build/isofiles
	@rm -r build/isofiles

$(kernel): $(kernel_elf)
	cp $(kernel_elf) $(kernel)

$(kernel_elf): kernel $(assembly_object_files) $(linker_script) $(rust_os)
	@ld -n -T $(linker_script) -o $(kernel_elf) \
	$(assembly_object_files) $(rust_os)

# compile assembly files
build/arch/$(arch)/%.o: src/arch/$(arch)/%.asm
	@mkdir -p $(shell dirname $@)
	@nasm -felf64 -Wno-other $< -o $@
