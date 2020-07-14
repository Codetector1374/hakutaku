use alloc::vec::Vec;

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


// pub struct USBConfigurationDescriptorSet {
//     pub config: USBConfigurationDescriptor,
//     pub ifsets: Vec<USBInterfaceDescriptorSet>,
// }
//
// // NOT A Descriptor
// pub struct USBInterfaceDescriptorSet {
//     pub interface: USBInterfaceDescriptor,
//     pub endpoints: Vec<USBEndpointDescriptor>,
// }
//
// impl USBInterfaceDescriptorSet {
//     pub fn new(ifdesc: USBInterfaceDescriptor) -> Self {
//         Self {
//             interface: ifdesc,
//             endpoints: Default::default()
//         }
//     }
// }

#[derive(Debug)]
#[repr(C)]
pub struct USBInterfaceDescriptor {
    length: u8,
    pub descriptor_type: u8,
    pub num_if: u8,
    pub alt_set: u8,
    pub num_ep: u8,
    pub class: u8,
    pub sub_class: u8,
    pub protocol: u8,
    pub ifstr_index: u8,
}
const_assert_size!(USBInterfaceDescriptor, 9);

#[repr(C)]
#[derive(Debug)]
pub struct USBEndpointDescriptor {
    length: u8,
    pub descriptor_type: u8,
    pub address: u8,
    pub attr: u8,
    max_packet_size: [u8; 2],
    pub interval: u8,
}
const_assert_size!(USBEndpointDescriptor, 7);

impl USBEndpointDescriptor {
    pub fn get_max_packet_size(&self) -> u16 {
        u16::from_le_bytes(self.max_packet_size)
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct USBConfigurationDescriptor {
    length: u8,
    pub descriptor_type: u8,
    total_length: [u8; 2],
    pub num_interfaces: u8,
    pub config_val: u8,
    pub config_string: u8,
    pub attrs: u8,
    pub max_power: u8,
}
const_assert_size!(USBConfigurationDescriptor, 9);

impl USBConfigurationDescriptor {
    pub fn get_total_length(&self) -> u16 {
        u16::from_le_bytes(self.total_length)
    }
}
