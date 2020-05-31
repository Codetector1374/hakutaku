use spin::RwLock;
use alloc::vec::Vec;
use alloc::boxed::Box;
use crate::storage::block::device::BlockDevice;
use hashbrown::{HashMap, HashSet};
use alloc::string::String;
use alloc::sync::Arc;
use crate::storage::partition::{scan_for_partitions, Partition};

pub mod device;
lazy_static! {
    pub static ref G_BLOCK_DEV_MGR: RwLock<BlockDeviceManager> = {
        RwLock::new(BlockDeviceManager {
            children_map: HashMap::new(),
            devices: HashMap::new(),
            next_device: 0,
        })
    };
}

pub struct BlockDeviceManager {
    pub children_map: HashMap<String, Vec<String>>,
    pub devices: HashMap<String, Arc<dyn BlockDevice + Sync + Send>>,
    next_device: u32,
}

pub struct PartitionBlockDevice {
    parent: Arc<dyn BlockDevice + Send + Sync>,
    part: Partition,
}

impl BlockDevice for PartitionBlockDevice {
    fn sector_size(&self) -> u64 {
        self.parent.sector_size()
    }

    fn read_sector(&self, sector: u64, buf: &mut [u8]) -> core_io::Result<usize> {
        if sector >= self.part.size {
            return Err(core_io::error::ErrorKind::UnexpectedEof.into())
        }
        self.parent.read_sector(self.part.offset + sector, buf)
    }

    fn write_sector(&self, sector: u64, buf: &[u8]) -> core_io::Result<usize> {
        if sector >= self.part.size {
            return Err(core_io::error::ErrorKind::UnexpectedEof.into())
        }
        self.parent.write_sector(sector + self.part.offset, buf)
    }
}

impl PartitionBlockDevice {
    pub fn new(part: Partition, parent: Arc<dyn BlockDevice + Send + Sync>) -> PartitionBlockDevice {
        Self {
            parent,
            part
        }
    }
}

impl BlockDeviceManager {
    /// Returns the registered device name
    pub fn register_root_device(&mut self, dev: Arc<dyn BlockDevice + Sync + Send>) -> String {
        let device_id = self.next_device;
        self.next_device += 1;
        let device_name = format!("blk{}", device_id);
        trace!("[BD] Registering device: {}", device_name);
        self.children_map.insert(device_name.clone(), Default::default());
        self.devices.insert(device_name.clone(), dev.clone().into());
        let parts = scan_for_partitions(dev.as_ref());
        for p in parts {
            let child_dev = PartitionBlockDevice::new(p.clone(), dev.clone());
            let name = self.register_device(&device_name, Arc::new(child_dev));
        }
        device_name
    }

    fn register_device(&mut self, root: &String, dev: Arc<dyn BlockDevice + Sync + Send>) -> String {
        match self.children_map.get_mut(root) {
            Some(vec) => {
                let id = vec.len();
                let name = format!("{}p{}", root, id);
                if self.devices.insert(name.clone(), dev).is_some() {
                    panic!("WHAT?");
                }
                vec.push(name.clone());
                name
            },
            _ => {
                panic!("DIE");
            }
        }
    }

    pub fn list_devices(&self) -> Vec<String> {
        let mut a:Vec<String> = self.devices.keys().cloned().collect();
        a.sort();
        a
    }
}