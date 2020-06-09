/* ------ USB Request bmRequestType --------------- */
pub const USB_REQUEST_TYPE_DIR_DEVICE_TO_HOST: u8 = 0x1 << 7;
pub const USB_REQUEST_TYPE_DIR_HOST_TO_DEVICE: u8 = 0x0 << 7;

pub const USB_REQUEST_TYPE_TYPE_STANDARD: u8 = 0x00 << 5;
pub const USB_REQUEST_TYPE_TYPE_CLASS: u8 = 0x01 << 5;
pub const USB_REQUEST_TYPE_TYPE_VENDOR: u8 = 0x02 << 5;

pub const USB_REQUEST_TYPE_RECP_DEVICE: u8 = 0x00;
pub const USB_REQUEST_TYPE_RECP_INTERFACE: u8 = 0x01;
pub const USB_REQUEST_TYPE_RECP_ENDPOINT: u8 = 0x02;
pub const USB_REQUEST_TYPE_RECP_OTHER: u8 = 0x03;
