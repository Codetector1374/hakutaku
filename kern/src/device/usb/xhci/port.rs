use volatile::Volatile;
use alloc::vec::Vec;

#[repr(C)]
pub struct XHCIPortOperationalRegisters {
    /// Port Status & Ctrl
    pub portsc: Volatile<u32>,
    /// Port Power Management Status & Ctrl
    pub portpmsc: Volatile<u32>,
    /// Port Link Info
    pub portli: Volatile<u32>,
    /// Hardware LMP Control
    pub porthwlpmc: Volatile<u32>,
}

#[derive(Debug, Copy, Clone)]
pub enum XHCIPortGeneration {
    Unknown,
    USB2,
    USB3,
}

#[derive(Debug, Copy, Clone)]
pub enum XHCIPortStatus {
    Disconnected,
    Connected,
    Active,
    InactiveUSB3CompanionPort,
    Failed,
}

#[derive(Debug)]
pub struct XHCIPort {
    pub controller_id: usize,
    pub port_id: u8,
    pub slot: u8,
    pub matching_port: Option<u8>,
    pub port_type: XHCIPortGeneration,
    pub active_eps: Vec<u8>,
    pub status: XHCIPortStatus,
}

impl XHCIPort {
    pub fn new(controller: usize, port: u8) -> Self {
        Self {
            controller_id: controller,
            port_id: port,
            slot: 0,
            matching_port: None,
            port_type: XHCIPortGeneration::Unknown,
            active_eps: Default::default(),
            status: XHCIPortStatus::Disconnected
        }
    }

    pub fn reset(&mut self) {
        self.slot = 0;
        self.active_eps.clear();
        self.status = XHCIPortStatus::Disconnected;
    }
}