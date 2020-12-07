pub mod interrupt;
pub mod xhci;

use spin::{Mutex, RwLock};
use crate::device::pci::GLOBAL_PCI;
use crate::device::pci::class::PCISerialBusUSB;
use crate::device::pci::device::PCIDevice;
use alloc::vec::Vec;
use alloc::sync::Arc;
use alloc::string::String;
use x86_64::instructions::interrupts::without_interrupts;
use core::sync::atomic::{AtomicU64, Ordering};
use crate::memory::mmio_bump_allocator::VMALLOC;
use usb_host::{USBHost, HostCallbacks, USBResult, USBErrorKind};
use core::time::Duration;
use kernel_api::syscall::sleep;
use crate::sys::pit::PIT;
use usb_host::traits::USBHostController;
use usb_host::consts::USBSpeed;
use usb_host::structs::{USBDevice, DeviceState};
use crate::device::usb::xhci::load_from_device;
use usb_host::drivers::hub::HubDriver;
use usb_host::drivers::keyboard::{HIDKeyboard, HIDKeyboardCallback};
use crate::sys::stdin::STD_IN;
use usb_host::drivers::mass_storage::{MassStorageDriver, MSDCallback, TransparentSCSI, SimpleBlockDevice};
use crate::storage::block::device::BlockDevice;
use core_io::error::ErrorKind;

struct USBHostCallback();

struct USBKeyboardCallback();

impl HIDKeyboardCallback for USBKeyboardCallback {
    fn key_down(ascii: u8) {
        STD_IN.insert(ascii);
    }
}

struct MassFSHook();

impl MSDCallback for MassFSHook {
    fn on_new_scsi(scsi: TransparentSCSI) -> USBResult<()> {
        let wrapper = USBSCSIWrapper(Mutex::new(scsi));
        let str = crate::storage::block::G_BLOCK_DEV_MGR.write().register_root_device(Arc::new(wrapper));
        debug!("USB MSD Attached as {}", str);
        Ok(())
    }
}

struct USBSCSIWrapper(Mutex<TransparentSCSI>);

impl BlockDevice for USBSCSIWrapper {
    fn sector_size(&self) -> u64 {
        self.0.lock().sector_size()
    }


    fn read_sector(&self, sector: u64, buf: &mut [u8]) -> core_io::Result<usize> {
        match self.0.lock().read_sector(sector, buf) {
            Ok(num) => Ok(num),
            Err(e) => {
                warn!("USB MSD ReadError: {:?}", e);
                Err(core_io::error::Error::from(ErrorKind::Other))
            }
        }
    }

    fn write_sector(&self, sector: u64, buf: &[u8]) -> core_io::Result<usize> {
        unimplemented!()
    }
}


impl HostCallbacks<USBHAL> for USBHostCallback {
    fn new_device(&self, host: &Arc<USBHost<USBHAL>>, device: &Arc<RwLock<USBDevice>>) -> USBResult<()> {
        use usb_host::consts::*;
        type X = USBHost<USBHAL>;

        let (device_desc, configuration) = {
            let d = device.read();
            (d.ddesc.clone(), d.config_desc.as_ref().ok_or(USBErrorKind::InvalidArgument.msg("expected config descriptor"))?.clone())
        };

        let mfg = X::fetch_string_descriptor(device, device_desc.iManufacturer, 0x409).unwrap_or(String::from("(no manufacturer name)"));
        let prd = X::fetch_string_descriptor(device, device_desc.iProduct, 0x409).unwrap_or(String::from("(no product name)"));
        let serial = X::fetch_string_descriptor(device, device_desc.iSerialNumber, 0x409).unwrap_or(String::from("(no serial number)"));
        debug!("[XHCI] New device:\n  MFG: {}\n  Prd:{}\n  Serial:{}", mfg, prd, serial);

        for interface in &configuration.ifsets {
            if interface.interface.bAlternateSetting != 0 {
                debug!("Skipping non-default altSetting Interface");
                continue;
            }

            if let Err(e) = HubDriver::<USBHAL>::probe(host, &device, interface) {
                error!("failed to probe hub: {:?}", e);
            }
            {
                let d = device.read();
                if matches!(d.device_state, DeviceState::Owned(_)) {
                    break;
                }
            }
            if let Err(e) = MassStorageDriver::<USBHAL, MassFSHook>::probe(&device, interface) {
                error!("failed to probe msd: {:?}", e);
            }
            {
                let d = device.read();
                if matches!(d.device_state, DeviceState::Owned(_)) {
                    break;
                }
            }

            if let Err(e) = HIDKeyboard::<USBHAL, USBKeyboardCallback>::probe(&device, interface) {
                error!("failed to probe hidkbd: {:?}", e);
            }
            {
                let d = device.read();
                if matches!(d.device_state, DeviceState::Owned(_)) {
                    break;
                }
            }
            // match interface.interface.bInterfaceClass {
            //     CLASS_CODE_HID => {
            //         if let Err(e) = HIDKeyboard::<XHCIHal>::probe(&device, interface) {
            //             error!("failed to probe hid: {:?}", e);
            //         }
            //     }
            //     _ => {}
            // }
        }

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