use alloc::boxed::Box;
use core::fmt::{Debug, Formatter};
use crate::storage::block::device::{RootBlockDevice, BlockDevice};
use crate::device::ahci::{AHCI, G_AHCI};
use spin::Mutex;
use crate::device::ahci::controller::AHCIHBAPort;
use x86_64::registers::rflags::read;
use alloc::sync::Arc;
use crate::device::ahci::structures::AHCIPortCommStructures;

#[derive(Debug, Copy, Clone)]
pub enum AHCIDeviceType {
    SATA,
    SATAPI,
}

pub trait AHCIDevice {
    fn port(&self) -> u8;

    fn device_type(&self) -> AHCIDeviceType;
}

pub struct AHCIAttachedDevice {
    device: Box<dyn AHCIDevice + Send + Sync>,
    registers: Arc<Mutex<AHCIPortCommStructures>>,
    controller: usize,
    port: u8,
}

impl Debug for AHCIAttachedDevice {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "AHCIAttachedDevice: type[{:?}] on {}:{}",
               self.device.device_type(), self.controller, self.port)
    }
}

impl AHCIAttachedDevice {
    pub fn create(controller: usize, port: u8, device: Box<dyn AHCIDevice + Send + Sync>) -> AHCIAttachedDevice {
        let read_lock = G_AHCI.controllers.read();
        AHCIAttachedDevice {
            device,
            registers: read_lock[controller].get_port_registers(port).expect("initialized"),
            controller,
            port,
        }
    }
}

pub struct AHCISATADevice {
    pub(super) port: u8,
}

impl AHCISATADevice {
    pub fn create(port: u8) -> AHCISATADevice {
        AHCISATADevice {
            port,
        }
    }
}

impl AHCIDevice for AHCISATADevice {
    fn port(&self) -> u8 {
        self.port
    }

    fn device_type(&self) -> AHCIDeviceType {
        AHCIDeviceType::SATA
    }
}

pub struct AHCIBlockDevice {
    controller: usize,
    port: u8,
}

impl AHCIBlockDevice {
    pub fn create(controller: usize, port: u8) -> AHCIBlockDevice {
        AHCIBlockDevice {
            controller,
            port,
        }
    }
}

impl RootBlockDevice for AHCIBlockDevice {}

impl BlockDevice for AHCIBlockDevice {
    fn read_sector(&self, sector: u64, buf: &mut [u8]) -> core_io::Result<usize> {
        panic!("Unsupported");
    }

    fn write_sector(&mut self, sector: u64, buf: &[u8]) -> core_io::Result<usize> {
        panic!("Unsupported");
    }
}