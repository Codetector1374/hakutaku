use spin::RwLock;
use alloc::vec::Vec;
use alloc::boxed::Box;
use crate::storage::block::device::{RootBlockDevice, BlockDevice};
use hashbrown::HashMap;
use alloc::string::String;
use alloc::sync::Arc;
use crate::storage::partition::scan_for_partitions;

pub mod device;
lazy_static! {
    pub static ref G_BLOCK_DEV_MGR: RwLock<BlockDeviceManager> = {
        RwLock::new(BlockDeviceManager {
            root_devices: HashMap::new(),
            devices: HashMap::new(),
            next_device: 0,
        })
    };
}

pub struct BlockDeviceManager {
    pub root_devices: HashMap<String, Arc<dyn RootBlockDevice + Sync + Send>>,
    pub devices: HashMap<String, Arc<dyn BlockDevice + Sync + Send>>,
    next_device: u32,
}

impl BlockDeviceManager {
    /// Returns the registered device name
    pub fn register_device(&mut self, dev: Arc<dyn RootBlockDevice + Sync + Send>) -> String {
        let device_id = self.next_device;
        self.next_device += 1;
        let device_name = format!("blk{}", device_id);
        debug!("[BD] Registering device: {}", device_name);
        self.root_devices.insert(device_name.clone(), dev.clone());
        scan_for_partitions(dev);
        device_name
    }
}