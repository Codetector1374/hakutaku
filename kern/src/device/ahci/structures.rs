use volatile::Volatile;
use alloc::boxed::Box;
use x86_64::PhysAddr;

#[repr(u8)]
pub enum FISType {
    FisTypeRegH2d = 0x27,	// Register FIS - host to device
    FisTypeRegD2h = 0x34,	// Register FIS - device to host
    FisTypeDmaAct = 0x39,	// DMA activate FIS - device to host
    FisTypeDmaSetup = 0x41,	// DMA setup FIS - bidirectional
    FisTypeData = 0x46,	// Data FIS - bidirectional
    FisTypeBist = 0x58,	// BIST activate FIS - bidirectional
    FisTypePioSetup = 0x5F,	// PIO setup FIS - device to host
    FisTypeDevBits = 0xA1,	// Set device bits FIS - device to host
}

#[repr(C)]
pub struct FISRegH2D {
    fis_type: FISType,
}

#[repr(C, align(256))]
pub struct ReceivedFIS {
    /// DMA Setup FIS
    dsfis: [Volatile<u8>; 0x1c],
    _res0: [u8; 0x20 - 0x1c],
    /// PIO Setup FIS
    psfis: [Volatile<u8>; 0x34 - 0x20],
    _res1: [u8; 0x40 - 0x34],
    /// Device to Host (D2H) Register FIS
    rfis: [Volatile<u8>; 0x54 - 0x40],
    _res2: [u8; 0x58 - 0x54],
    /// Set Device Bits FIS
    sdbfis: [Volatile<u8>; 0x60 - 0x58],
    /// Unknown FIS
    ufis: [u8; 0xa0 - 0x60],
    _res3: [u8; 0x100 - 0xa0],
}
const_assert_size!(ReceivedFIS, 256);
impl Default for ReceivedFIS {
    fn default() -> Self {
        unsafe {
            core::mem::zeroed()
        }
    }
}

#[repr(C)]
#[derive(Default)]
pub struct CommandList {
    pub commands: [CommandHeader; 32]
}
const_assert_size!(CommandList, 1024);

#[repr(C)]
#[derive(Copy, Clone)]
pub struct CommandHeader {
    // DW1
    pub flags: u16,
    pub prdtl: u16,
    // DW2
    /// Physical Region Descriptor Bytes Count
    pub prdbc: u32,
    // DW3, DW4
    /// Command Table Base Address (128 Align)
    pub ctba: PhysAddr,
    _res0: [u32; 4],
}
const_assert_size!(CommandHeader, 32);

impl Default for CommandHeader {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

#[derive(Default)]
pub struct CommandTableList {
    pub cmd_tables: [Option<Box<CommandTable>>; 32],
}

#[repr(C, align(128))]
pub struct CommandTable {
    /// Command FIS
    pub cfis: [u8; 64],
    /// ATA/ATAPI Command
    pub acmd: [u8; 16],
    pub _res0: [u8; 48],
    // TODO Might Change to support multiple
    pub prtd_entry: [PRDTEntry; 8],
}

impl Default for CommandTable {
    fn default() -> Self {
        unsafe {
            core::mem::zeroed()
        }
    }
}

#[repr(C)]
pub struct PRDTEntry {
    /// Data Base Address
    pub dba: u64,
    _res0: u32,
    /// 31: interrupt enable, 22:0 byte count (Zero base)
    pub flags: u32,
}