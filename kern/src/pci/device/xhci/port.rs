#[derive(Debug)]
pub struct XHCIPortGroup {
    usb3_port: Option<XHCIPort>,
    usb2_port: Option<XHCIPort>,
}

#[derive(Debug, Copy, Clone)]
pub struct XHCIPort {
    port_id: u8,
}