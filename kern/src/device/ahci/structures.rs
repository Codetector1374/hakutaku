use volatile::Volatile;
use alloc::boxed::Box;
use x86_64::PhysAddr;
use x86_64::instructions::cache::wbinvd;

#[repr(u8)]
pub enum FISType {
    Invalid = 0x0,
    RegH2d = 0x27,	// Register FIS - host to device
    RegD2h = 0x34,	// Register FIS - device to host
    DmaAct = 0x39,	// DMA activate FIS - device to host
    DmaSetup = 0x41,	// DMA setup FIS - bidirectional
    Data = 0x46,	// Data FIS - bidirectional
    Bist = 0x58,	// BIST activate FIS - bidirectional
    PioSetup = 0x5F,	// PIO setup FIS - device to host
    DevBits = 0xA1,	// Set device bits FIS - device to host
}

#[repr(C)]
pub struct FISRegH2D {
    // DW1
    pub fis_type: FISType,
    pub flags: u8,
    pub command: u8,
    pub feature_l: u8,
    //DW2
    lba0: u8,
    lba1: u8,
    lba2: u8,
    pub device: u8,

    //DW3
    lba3: u8,
    lba4: u8,
    lba5: u8,
    pub feature_h: u8,

    // DW4
    pub count: u16,
    pub icc: u8,
    pub ctrl: u8,
    // DW5
    _res0: u32
}
const_assert_size!(FISRegH2D, 20);

impl FISRegH2D {
    pub fn set_lba(&mut self, lba: u64) {
        assert!(lba < 1<<48, "LBA is limited to 48 bits");
        self.lba0 = (lba >> 0) as u8;
        self.lba1 = (lba >> 8) as u8;
        self.lba2 = (lba >> 16) as u8;
        self.lba3 = (lba >> 24) as u8;
        self.lba4 = (lba >> 32) as u8;
        self.lba5 = (lba >> 40) as u8;
    }
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

#[repr(C,align(1024))]
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
    pub cfis: FISRegH2D,
    _res: [u8; 64-core::mem::size_of::<FISRegH2D>()],
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
    pub dba: PhysAddr,
    _res0: u32,
    /// 31: interrupt enable, 22:0 byte count (Zero base)
    pub flags: u32,
}