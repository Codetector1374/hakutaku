use crate::device::usb::device::USBDevice;
use alloc::string::String;
use crate::device::usb::descriptor::USBDeviceDescriptor;

pub struct USBXHCIDevice {
    pub dev_descriptor: USBDeviceDescriptor,
    pub manufacture: String,
    pub product: String,
    pub serial: String,

    pub control_slot: u8,
    pub xhci_port: u8,
    pub controller_id: u64,

    pub system_id: u64,
}

impl USBDevice for USBXHCIDevice {
    fn device_descriptor(&self) -> &USBDeviceDescriptor {
        &self.dev_descriptor
    }

    fn manufacture_string(&self) -> &str {
        &self.manufacture
    }

    fn product_string(&self) -> &str {
        &self.product
    }

    fn serial_string(&self) -> &str {
        &self.serial
    }

    fn bus(&self) -> u8 {
        self.controller_id as u8
    }

    fn device(&self) -> u8 {
        self.xhci_port
    }

    fn system_device_id(&self) -> u64 {
        self.system_id
    }
}