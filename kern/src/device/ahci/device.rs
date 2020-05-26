use alloc::boxed::Box;
use core::fmt::{Debug, Formatter};
use crate::storage::block::device::{RootBlockDevice, BlockDevice};
use crate::device::ahci::{AHCI, G_AHCI};
use spin::Mutex;
use crate::device::ahci::controller::{AHCIHBAPort, AHCIController};
use x86_64::registers::rflags::read;
use alloc::sync::Arc;
use crate::device::ahci::structures::AHCIPortCommStructures;
use core::borrow::BorrowMut;
use core::ops::DerefMut;
use core::cmp::min;
use core_io::error::ErrorKind;

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
    comm_struct: Arc<Mutex<AHCIPortCommStructures>>,
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
            comm_struct: read_lock[controller].get_port_registers(port).expect("initialized"),
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
    dev: Arc<AHCIAttachedDevice>
}

impl From<Arc<AHCIAttachedDevice>> for AHCIBlockDevice {
    fn from(dev: Arc<AHCIAttachedDevice>) -> Self {
        Self {
            dev,
        }
    }
}

impl RootBlockDevice for AHCIBlockDevice {}

impl BlockDevice for AHCIBlockDevice {
    fn read_sector(&self, sector: u64, buf: &mut [u8]) -> core_io::Result<usize> {
        let mut port_comm = self.dev.comm_struct.lock();
        let mut alt_buffer = [0u16; 256];
        if AHCIController::read_sector(port_comm.deref_mut(), sector, &mut alt_buffer).is_err() {
            return Err(ErrorKind::Other.into())
        }
        let alt_buffer: [u8; 512] = unsafe { core::mem::transmute(alt_buffer) };
        let len = min(512, buf.len());
        buf[..len].copy_from_slice(&alt_buffer[..len]);
        Ok(min(512, buf.len()))
    }

    fn write_sector(&mut self, _sector: u64, _buf: &[u8]) -> core_io::Result<usize> {
        panic!("Unsupported");
    }
}