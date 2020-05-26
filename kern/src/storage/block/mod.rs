use spin::RwLock;
use alloc::vec::Vec;
use alloc::boxed::Box;
use crate::storage::block::device::{RootBlockDevice, BlockDevice};
use hashbrown::HashMap;
use alloc::string::String;

pub mod device;
lazy_static! {
    pub static ref G_BLOCK_DEV_MGR: RwLock<BlockDeviceManager> = {
        RwLock::new(BlockDeviceManager {
            devices: HashMap::new(),
            next_device: 0,
        })
    };
}

pub struct BlockDeviceManager {
    pub devices: HashMap<String, Box<dyn RootBlockDevice + Sync + Send>>,
    next_device: u32,
}

impl BlockDeviceManager {
    /// Returns the registered device name
    pub fn register_device(&mut self, dev: Box<dyn RootBlockDevice + Sync + Send>) -> String {
        let device_id = self.next_device;
        self.next_device += 1;
        let device_name = format!("blk{}", device_id);
        debug!("[BD] Registering device: {}", device_name);
        self.devices.insert(device_name.clone(), dev);
        device_name
    }
}