use core::fmt;
use core::fmt::{Write, Debug, Formatter};
use crate::storage::block::device::BlockDevice;
use crate::storage::partition::{Partition, PartitionType};


#[repr(C)]
#[derive(Copy, Clone)]
pub struct CHS {
    _phantom: [u8; 3],
}

impl Default for CHS {
    fn default() -> Self {
        CHS {
            _phantom: [0u8; 3],
        }
    }
}

impl Debug for CHS {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!("Head: {}, Cylinder: {}, Sector: {}",
                                 self._phantom[0],
                                 (((self._phantom[1] as u16) << 8u16) & 0b110000000) | self._phantom[2] as u16,
                                 self._phantom[1] & 0b111111u8
        ))
    }
}
const_assert_size!(CHS, 3);

#[repr(C, packed)]
#[derive(Default)]
pub struct PartitionEntry {
    pub boot_indicator: u8,
    _chs_start: CHS,
    pub partition_type: u8,
    _chs_end: CHS,
    pub offset: u32,
    // TODO Assume Endian
    pub total_sectors: u32,
}

impl Into<Partition> for &PartitionEntry {
    fn into(self) -> Partition {
        Partition::new(self.offset as u64, self.total_sectors as u64, PartitionType::MBR(self.partition_type))
    }
}

impl PartitionEntry {
    pub fn is_fat32(&self) -> bool {
        self.partition_type == 0xB || self.partition_type == 0xC
    }
}

impl Debug for PartitionEntry {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "Partition: {{ offset: {}, length: {}, type: {:#x} }}", self.offset, self.total_sectors, self.partition_type)
    }
}

const_assert_size!(PartitionEntry, 16);

/// The master boot record (MBR).
#[repr(C, packed)]
pub struct MasterBootRecord {
    code: [u8; 436],
    pub disk_id: [u8; 10],
    pub parts: [PartitionEntry; 4],
    magic_number: [u8; 2], // 0x55 0xAA (As array so works with bid endian)
}

impl Debug for MasterBootRecord {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!("MBR Entry:
            Part1: {:?}
            Part2: {:?}
            Part3: {:?}
            Part4: {:?}
            MagicNumber: {:02X}{:02X}",
                                 self.parts[0], self.parts[1],
                                 self.parts[2], self.parts[3],
                                 self.magic_number[0], self.magic_number[1]))
    }
}

impl Default for MasterBootRecord {
    fn default() -> Self {
        MasterBootRecord {
            code: [0u8; 436],
            disk_id: [0u8; 10],
            parts: Default::default(),
            magic_number: [0u8; 2],
        }
    }
}

const_assert_size!(MasterBootRecord, 512);

#[derive(Debug)]
pub enum Error {
    /// There was an I/O error while reading the MBR.
    Io(core_io::error::Error),
    /// Partiion `.0` (0-indexed) contains an invalid or unknown boot indicator.
    UnknownBootIndicator(u8),
    /// The MBR magic signature was invalid.
    BadSignature,
}

fn check_boot_indicator(entry: &PartitionEntry, id: u8) -> Result<(), Error> {
    if entry.boot_indicator == 0x0 || entry.boot_indicator == 0x80 {
        Ok(())
    } else {
        Err(Error::UnknownBootIndicator(id))
    }
}

impl MasterBootRecord {
    /// Reads and returns the master boot record (MBR) from `device`.
    ///
    /// # Errors
    ///
    /// Returns `BadSignature` if the MBR contains an invalid magic signature.
    /// Returns `UnknownBootIndicator(n)` if partition `n` contains an invalid
    /// boot indicator. Returns `Io(err)` if the I/O error `err` occured while
    /// reading the MBR.
    pub fn from<T: BlockDevice + ?Sized>(device: &T) -> Result<MasterBootRecord, Error> {
        let sector_size = device.sector_size();
        let mut mbr: MasterBootRecord = Default::default();
        let buffer = unsafe {
            core::slice::from_raw_parts_mut(
                ((&mut mbr) as *mut MasterBootRecord) as *mut u8,
                core::mem::size_of::<MasterBootRecord>(),
            )
        };
        let read_size: Result<usize, core_io::error::Error> = device.read_sector(0, buffer);
        if read_size.is_err() {
            return Err(Error::Io(read_size.unwrap_err()));
        }
        let read_size = read_size.unwrap();
        if read_size != sector_size as usize {
            panic!("reading from a sector produced {} bytes instead of {}", read_size, sector_size);
        }
        if mbr.magic_number[0] == 0x55 && mbr.magic_number[1] == 0xAA {
            for (i, part) in mbr.parts.iter().enumerate() {
               check_boot_indicator(part, i as u8)?;
            }
            Ok(mbr)
        } else {
            Err(Error::BadSignature)
        }
    }
}
