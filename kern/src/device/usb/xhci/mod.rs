use crate::device::pci::device::PCIDevice;
use crate::device::pci::class::{PCISerialBusControllerClass, PCISerialBusUSB, PCIDeviceClass};
use x86_64::instructions::interrupts::without_interrupts;
use crate::memory::mmio_bump_allocator::VMALLOC;
use x86_64::structures::paging::{Page, Size4KiB, PhysFrame, PageTableFlags, Mapper};
use core::time::Duration;
use crate::hardware::pit::PIT;
use kernel_api::syscall::sleep;
use x86_64::VirtAddr;
use crate::device::usb::xhci::consts::*;
use core::alloc::Layout;
use xhci::{FlushType, Xhci, XhciWrapper};
use usb_host::USBHost;
use alloc::sync::Arc;
use spin::Mutex;

pub mod consts;
static XHCI_HAL: XhciHAL = XhciHAL();

struct XhciHAL();

impl xhci::HAL for XhciHAL {
    fn current_time(&self) -> Duration {
        PIT::current_time()
    }

    fn sleep(&self, dur: Duration) {
        sleep(dur).unwrap();
    }

    fn memory_barrier(&self) {}

    fn translate_addr(&self, addr: u64) -> u64 {
        pt_translate!(VirtAddr::new(addr)).as_u64()
    }

    fn flush_cache(&self, _addr: u64, _len: u64, _flush: FlushType) {
    }
}

impl usb_host::HAL2 for XhciHAL{
    fn sleep(dur: Duration) {
        sleep(dur).expect("slept???");
    }

    fn current_time() -> Duration {
        PIT::current_time()
    }
}

fn xhci_address_space_detect(dev: &mut PCIDevice) -> usize {
    let old_value = dev.read_config_bar_register(0);
    dev.write_config_bar_register(0, 0xFFFF_FFFF);
    let new_val = dev.read_config_bar_register(0) & 0xFFFFF000;
    dev.write_config_bar_register(0, 0xFFFF_FFFF);
    dev.write_config_bar_register(0, old_value);
    (!new_val) as usize
}

pub fn create_from_device(id: u64, mut dev: PCIDevice) {
    if let PCIDeviceClass::SerialBusController(PCISerialBusControllerClass::USBController(PCISerialBusUSB::XHCI)) = &dev.info.class {
        debug!("XHCI Device on: {:04x}:{:02x}:{:x} -> [{:04x}]:[{:04x}]",
               dev.bus, dev.device_number, dev.func, dev.info.vendor_id, dev.info.device_id);
        // Step1: Enable Bus Master
        let mut tmp = dev.read_config_word(crate::device::pci::consts::CONF_COMMAND_OFFSET);
        tmp |= crate::device::pci::consts::PCI_COMMAND_MASTER;
        dev.write_config_word(crate::device::pci::consts::CONF_COMMAND_OFFSET, tmp);

        intel_ehci_xhci_handoff(&mut dev);

        // Step2: Setup Interrupt Lines
        // let interrupt_number = (InterruptIndex::XHCI).as_offset() as u32;
        // dev.write_config_dword_dep(0xF, interrupt_number);
        // let interrupt = dev.read_config_dword_dep(0xF);
        // let int_line = (interrupt >> 8) as u8;
        // let int_num = interrupt as u8;
        // trace!("[XHCI] Interrupt Line: {}, Interrupt Number: {}", int_line, int_num);

        // Step3: Setup MMIO Registers
        let size = xhci_address_space_detect(&mut dev);
        info!("XHCI Size: {}", size);
        let base = dev.base_mmio_address(0).expect("xHCI can't be MMIO");
        let base_offset = base.as_u64() as usize & (4096 - 1);
        let alloc_base = base.align_down(4096u64);
        debug!("[XHCI] Address: {:?}, size: {}, offset:{}", base, size, base_offset);
        let va_root = without_interrupts(|| {
            let (va, size) = VMALLOC.lock().allocate(size + base_offset);
            debug!("[XHCI] Allocated: {} bytes MMIO Space, starting: {:?}", size, va);
            let mut fallocw = crate::memory::frame_allocator::FrameAllocWrapper{};
            for offset in (0..size).step_by(4096) {
                let paddr = alloc_base + offset;
                let vaddr = va + offset;
                debug!("[XHCI] Mapping offset: {}, va: {:?} pa: {:?}", offset, vaddr, paddr);
                unsafe {
                    crate::PAGE_TABLE.write().map_to(
                        Page::<Size4KiB>::from_start_address(vaddr).expect("Unaligned VA"),
                        PhysFrame::<Size4KiB>::from_start_address(paddr).expect("Unaligned PA"),
                        PageTableFlags::WRITABLE | PageTableFlags::WRITE_THROUGH | PageTableFlags::PRESENT,
                        &mut fallocw,
                    ).expect("Unable to map").flush();
                }
            }
            va
        });

        let mmio_vbase = va_root + base_offset;
        let xhci = Xhci::new(mmio_vbase.as_u64(), &XHCI_HAL);
        let xhci_controller = Arc::new(XhciWrapper(Mutex::new(xhci)));
        let mut usbhost = USBHost::<XhciHAL>::new();
        let root_device = usbhost.attach_root_hub(xhci_controller);
        USBHost::<XhciHAL>::setup_new_device(root_device);


    }
    error!("[XHCI] No XHCI Controller Found");
}

/// Special Function to handle the intel controller
/// where a mux is used to switch ports from EHCI to
/// xHCI.
fn intel_ehci_xhci_handoff(pci_device: &mut PCIDevice) {
    if pci_device.info.vendor_id == crate::device::pci::consts::VID_INTEL {
        debug!("[XHCI] Intel Controller Detected: Dev: {:#x}. EHCI Handoff", pci_device.info.device_id);
        let ports = pci_device.read_config_dword(USB_INTEL_USB3PRM);
        debug!("[XHCI] [Intel] Configurable Ports to SS: {:#x}", ports);
        // Enable Super Speed on Ports that supports it
        pci_device.write_config_dword(USB_INTEL_USB3_PSSEN, ports);

        let usb2_prm = pci_device.read_config_dword(USB_INTEL_USB2PRM);
        debug!("[XHCI] [Intel] Configurable USB2 xHCI handoff: {:#x}", usb2_prm);
        // TODO!!! REMOVE & PORTS, this is a hack for me to keep usb2 functional
        pci_device.write_config_dword(USB_INTEL_XUSB2PR, usb2_prm);
    }
}
