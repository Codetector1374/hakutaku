pub mod interrupt;
pub mod xhci;

use spin::{Mutex, RwLock};
use crate::device::pci::GLOBAL_PCI;
use crate::device::pci::class::PCISerialBusUSB;
use crate::device::pci::device::PCIDevice;
use alloc::vec::Vec;
use alloc::sync::Arc;
// use crate::device::usb::xhci::port::XHCIPort;
use x86_64::instructions::interrupts::without_interrupts;
use core::sync::atomic::{AtomicU64, Ordering};
use crate::memory::mmio_bump_allocator::VMALLOC;
use usb_host::USBHost;
use core::time::Duration;
use kernel_api::syscall::sleep;
use crate::sys::pit::PIT;
use usb_host::traits::USBHostController;
use usb_host::consts::USBSpeed;

pub static G_USB: USBSystem = USBSystem(Mutex::new(None));

pub struct USBHAL();
impl usb_host::HAL2 for USBHAL {
    fn sleep(dur: Duration) {
        sleep(dur).expect("failed in sleep syscall");
    }

    fn current_time() -> Duration {
        PIT::current_time()
    }
}

pub struct USBSystem(Mutex<Option<USBHost<USBHAL>>>);

impl USBSystem {
    pub fn initialize(&self) {
        let mut x = self.0.lock();
        if x.is_some() {
            panic!("USB Double Initialization triggered");
        }
        x.replace(USBHost::new());
    }

    pub fn setup_controller(&self, controller: Arc<dyn USBHostController>, speed: USBSpeed) {
        self.0.lock().as_mut().expect("USB Uninitialized").attach_root_hub(controller, speed);
    }
}