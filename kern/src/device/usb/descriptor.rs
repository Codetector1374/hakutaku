#[derive(Debug)]
#[repr(C)]
pub struct USBDeviceDescriptor {
    length: u8,
    pub descriptor_type: u8,
    /// Version in bcd Major/Minor
    pub version: u16,
    pub class: u8,
    pub sub_class: u8,
    pub protocol: u8,
    max_packet_size: u8,
    pub vid: u16,
    pub pid: u16,
    /// BCD Device Release Version
    pub dev_version: u16,
    pub manufacturer_index: u8,
    pub product_index: u8,
    pub serial_index: u8,
    pub config_count: u8,
}
const_assert_size!(USBDeviceDescriptor, 18);

#[derive(Debug)]
#[repr(C)]
pub struct USBConfigurationDescriptor {
    length: u8,
    pub descriptor_type: u8,
    pub total_length: u16,
    pub num_interfaces: u8,
    pub config_val: u8,
    pub config_string: u8,
    pub attrs: u8,
    pub max_power: u8,
}