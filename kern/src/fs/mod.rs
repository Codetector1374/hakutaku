//! fs/mod.rs: File System

use alloc::vec::Vec;

pub mod traits;
pub mod path;

pub struct VirtualFileSystem {
    pub mounts: Vec<VFSMount>
}

pub struct VFSMount {
    // path: ,
}