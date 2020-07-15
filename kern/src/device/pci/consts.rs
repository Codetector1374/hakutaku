// PCI Config Space
pub const CONF_COMMAND_OFFSET: u8 = 0x4;
pub const CONF_HEADER_TYPE_OFFSET: u8 = 0x0e;
pub const CONF_SECONDARY_BUS_OFFSET: u8 = 0x18;
pub const CONF_CAPABILITY_PTR_OFFSET: u8 = 0x34;
pub const CONF_INTERRUPT_OFFSET: u8 = 0x3C;

pub const PCI_COMMAND_MASTER: u16 = 1 << 2;


// Vendor IDs
pub const VID_INTEL: u16 = 0x8086;