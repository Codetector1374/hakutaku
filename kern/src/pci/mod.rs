use crate::pci::device::PCIDevice;
use crate::pci::class::{PCIDeviceClass, HeaderType};
use alloc::vec::Vec;
use alloc::alloc::handle_alloc_error;

pub mod device;
pub mod class;

#[derive(Debug)]
pub enum PCIError {
    SlotNumber,
    FuncNumber,
    RegisterNumber,
    InvalidDevice,
}

#[derive(Debug)]
pub struct PCIDeviceInfo {
    pub device: PCIDevice,
    pub class: PCIDeviceClass,
    pub rev: u8,
    pub header_type: HeaderType,
}

pub fn enumerate_pci_bus() -> Vec<PCIDeviceInfo> {
    let mut bus = Vec::<PCIDeviceInfo>::with_capacity(16);
    enumerate_bus(0, &mut bus);
    bus
}

fn enumerate_bus(bus: u8, vec: &mut Vec<PCIDeviceInfo>) {
    for device_id in 0..32 {
        check_device(bus, device_id, vec);
    }
}

fn check_device(bus: u8, device: u8, vec: &mut Vec<PCIDeviceInfo>) {
    if check_function(bus, device, 0, vec) {
        for func in 1..8 {
            check_function(bus, device, func, vec);
        }
    }
}

/// Returns: isMultiFunction
fn check_function(bus: u8, device: u8, func: u8, vec: &mut Vec<PCIDeviceInfo>) -> bool {
    let dev = PCIDevice::new_unchecked(bus, device, func).expect("err");
    match dev.into_info() {
        Some(info) => {
            let mf = info.header_type.is_multi_function();
            if let HeaderType::PCIBridge(_) = info.header_type {
                let num = info.device.secondary_bus_number();
                vec.push(info);
                enumerate_bus(num, vec);
            } else {
                vec.push(info);
            }
            mf
        }
        _ => { false }
    }
}