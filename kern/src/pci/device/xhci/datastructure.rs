#[repr(C, align(2048))]
pub struct DeviceContextBaseAddressArray {
    array: [u8; 2048]
}
const_assert_size!(DeviceContextBaseAddressArray, 2048);

impl Default for DeviceContextBaseAddressArray {
    fn default() -> Self {
        DeviceContextBaseAddressArray {
            array: [0u8; 2048],
        }
    }
}

#[repr(C, align(4096))]
pub struct CommandRingSegment {
    trbs: [ControlTRB; 256],
}
const_assert_size!(CommandRingSegment, 4096);

impl Default for CommandRingSegment {
    fn default() -> Self {
        CommandRingSegment {
            trbs: [Default::default(); 256],
        }
    }
}

#[repr(C)]
#[derive(Default, Copy, Clone)]
pub struct SetupStageTRB {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
    /// Interrupt Target: (31:22), TRBLength (16:0)
    int_target_trb_length: u32,
    /// Transfer Type: (17:16)
    /// > 0: No Data, 1: Reserved, 2: Out Data, 3: In Data
    /// Transfer Type (15:10)
    /// > Should be set to Setup Stage TRB Type (XHCI Table 6-91)
    /// immData (6:6)
    /// Interrupt On Complete (5:5)
    /// Cycle Bit (0:0)
    metadata: u32
}
const_assert_size!(SetupStageTRB, 16);

#[repr(C)]
#[derive(Copy, Clone)]
pub union ControlTRB {
    setup: SetupStageTRB,
}
const_assert_size!(ControlTRB, 16);

impl Default for ControlTRB {
    fn default() -> Self {
        ControlTRB {
            setup: Default::default(),
        }
    }
}






