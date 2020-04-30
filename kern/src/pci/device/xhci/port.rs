use volatile::Volatile;

#[derive(Debug, Default)]
pub struct XHCIPortGroup {
    pub usb3_port: Option<XHCIPort>,
    pub usb2_port: Option<XHCIPort>,
}

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
pub enum XHCIPortStatus {
    Connected,
    Active,
    Disconnected
}

#[derive(Debug)]
pub struct XHCIPort {
    pub port_id: u8,
    pub status: XHCIPortStatus,
}

impl XHCIPort {
    pub fn new(id: u8) -> XHCIPort {
        XHCIPort {
            port_id: id,
            status: XHCIPortStatus::Disconnected,
        }
    }
}