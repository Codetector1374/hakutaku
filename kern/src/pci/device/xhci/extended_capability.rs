use volatile::{ReadOnly, Volatile};
use core::fmt::{Debug, Formatter};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RawExtCapTag {
    base_ptr: *const u32,
    id: u8,
    next: u8,
}

impl From<*const u32> for RawExtCapTag {
    fn from(base_ptr: *const u32) -> Self {
        let base = unsafe { & *(base_ptr as *const Volatile<u32>) };
        let header = base.read();
        let id = (header & 0xFF) as u8;
        let next = ((header >> 8)& 0xFF) as u8;
        RawExtCapTag {
            base_ptr,
            id,
            next,
        }
    }
}

impl RawExtCapTag {
    pub fn next(&self) -> Option<RawExtCapTag> {
        if self.next != 0 {
            Some(RawExtCapTag::from(self.base_ptr.wrapping_add(self.next as usize)))
        } else {
            None
        }
    }
}

pub struct ExtendedCapabilityTags {
    ptr: Option<RawExtCapTag>
}

impl ExtendedCapabilityTags {
    /// Ensure base_ptr points to valid tag start
    pub fn get(base_ptr: usize) -> ExtendedCapabilityTags {
        ExtendedCapabilityTags {
            ptr: Some(RawExtCapTag::from(base_ptr as *const u32)),
        }
    }
}

impl Iterator for ExtendedCapabilityTags {
    type Item = ExtendedCapabilityTag;

    fn next(&mut self) -> Option<Self::Item> {
        match self.ptr {
            Some(p) => {
                let this = ExtendedCapabilityTag::from(&p);
                self.ptr = p.next();
                Some(this)
            },
            _ => { None }
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ExtendedCapabilityTag {
    USBLegacySupport {
        head: *mut u32,
        bios_own: bool,
        os_own: bool,
    },
    SupportedProtocol {
        major: u8,
        minor: u8,
        port_offset: u8,
        port_count: u8,
    },
    ExtendedPM,
    IOVirt,
    MessageInterrupt,
    LocalMemory,
    USBDebug,
    ExtendedMessageInterrupt,
    Unknown{ id: u8, next: u8, head: *const u32 }
}

// impl Debug for ExtendedCapabilityTag {
//     fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
//         match self {
//             ExtendedCapabilityTag::Unknown{id, next, header_ptr} => {
//                 f.write_fmt(format_args!("Unknown: id: {}, next: {}, at: 0x{:x}", id, next, *header_ptr as usize))
//             },
//         }
//     }
// }

impl From<&RawExtCapTag> for ExtendedCapabilityTag {
    fn from(t: &RawExtCapTag) -> Self {
        match t.id {
            0x1 => {
                let head = unsafe {t.base_ptr.read_volatile()};
                let bios_own = (head >> 16 & 0x1) == 1;
                let os_own = (head >> 24 & 0x1) == 1;
                ExtendedCapabilityTag::USBLegacySupport {
                    head: t.base_ptr as *mut u32,
                    bios_own,
                    os_own,
                }
            },
            0x2 => {
                // Read Major / Minor
                let head = unsafe {
                    t.base_ptr.read_volatile()
                };
                let major = ((head >> 24) & 0xF) as u8;
                let minor = ((head >> 16) & 0xFF) as u8;
                let offset8h = unsafe {
                    t.base_ptr.wrapping_offset(2).read_volatile()
                };
                let port_offset = offset8h as u8;
                let port_count = (offset8h >> 8) as u8;
                ExtendedCapabilityTag::SupportedProtocol {
                    major,
                    minor,
                    port_offset,
                    port_count,
                }
            },
            _ => {
                ExtendedCapabilityTag::Unknown {
                    head: t.base_ptr,
                    id: t.id,
                    next: t.next
                }
            }
        }
    }
}