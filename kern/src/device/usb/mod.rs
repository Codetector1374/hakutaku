pub mod interrupt;
pub mod xhci;

use spin::{Mutex, RwLock};
use crate::device::pci::GLOBAL_PCI;
use crate::device::pci::class::PCISerialBusUSB;
use crate::device::pci::device::PCIDevice;
use alloc::vec::Vec;
use alloc::sync::Arc;
use x86_64::instructions::interrupts::without_interrupts;
use core::sync::atomic::{AtomicU64, Ordering};
use crate::memory::mmio_bump_allocator::VMALLOC;
use usb_host::{USBHost, HostCallbacks, USBResult};
use core::time::Duration;
use kernel_api::syscall::sleep;
use crate::sys::pit::PIT;
use usb_host::traits::USBHostController;
use usb_host::consts::USBSpeed;
use usb_host::structs::USBDevice;
use crate::device::usb::xhci::load_from_device;

struct USBHostCallback();

impl HostCallbacks<USBHAL> for USBHostCallback {
    fn new_device(&self, _host: &Arc<USBHost<USBHAL>>, device: &Arc<RwLock<USBDevice>>) -> USBResult<()> {
        debug!("[kUSB] Got new device: {:?}", &device.read().ddesc);
        Ok(())
    }
}

lazy_static! {
    pub static ref G_USB: USBSystem = USBSystem(Arc::new(USBHost::new(Arc::new(USBHostCallback()))));
}
pub struct USBSystem(Arc<USBHost<USBHAL>>);

pub struct USBHAL();
impl usb_host::UsbHAL for USBHAL {
    fn sleep(dur: Duration) {
        sleep(dur).expect("failed in sleep syscall");
    }

    fn current_time() -> Duration {
        PIT::current_time()
    }
}

impl USBSystem {
    pub fn setup_controller(&self, controller: Arc<dyn USBHostController>, speed: USBSpeed) {
        self.0.attach_root_hub(controller, speed);
    }

    pub fn setup_new_device(&self, dev: Arc<RwLock<USBDevice>>) {
        USBHost::<USBHAL>::setup_new_device(&self.0, dev).expect("USB Failed to load");
    }
}