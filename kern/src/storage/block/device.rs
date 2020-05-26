
pub trait RootBlockDevice : BlockDevice {
}

pub trait BlockDevice {
    /// Sector size in bytes.
    fn sector_size(&self) -> u64 { 512 } // 512 is default

    fn read_sector(&self, sector: u64, buf: &mut [u8]) -> core_io::Result<usize>;

    fn write_sector(&mut self, sector: u64, buf: &[u8]) -> core_io::Result<usize>;
}