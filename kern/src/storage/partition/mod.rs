use core::ops::Deref;
use crate::storage::block::device::BlockDevice;
use crate::storage::partition::mbr::{MasterBootRecord, PartitionEntry};
use alloc::sync::Arc;
use alloc::vec::Vec;

pub mod mbr;

#[derive(Clone, Debug)]
pub struct Partition {
    /// In units of Sector
    pub (super) offset: u64,
    /// In units of Sector
    pub (super) size: u64,
    pub (super) format: PartitionType,
}

impl Partition {
    pub fn new(offset: u64, size: u64, format: PartitionType) -> Self {
        Self {
            offset,
            size,
            format
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum PartitionType {
    GUID([u8; 16]),
    MBR(u8)
}

pub fn scan_for_partitions<T: BlockDevice + ?Sized>(dev: &T) -> Vec<Partition> { // Check for MBR
    let mut vec = Vec::new();
    let mbr = MasterBootRecord::from(dev).ok();
    if let Some(mbr) = mbr {
        for p in mbr.parts.iter() {
            if p.offset > 0 && p.total_sectors > 0 && p.partition_type != 0 {
                vec.push(p.into());
            }
        }

    }
    vec
}