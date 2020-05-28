use core::ops::Deref;
use crate::storage::block::device::BlockDevice;
use crate::storage::partition::mbr::MasterBootRecord;

pub mod mbr;

pub fn scan_for_partitions<'a, T: AsRef<dyn BlockDevice + 'a>>(dev: T) { // Check for MBR
    MasterBootRecord::from(dev);
}