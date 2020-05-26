use crate::process::process::Id;
use crate::hardware::apic::GLOBAL_APIC;
use hashbrown::HashMap;
use core::fmt::{Debug, Formatter};

#[derive(Default)]
pub struct Processors {
    cpus: HashMap<u8, LocalCPU>,
}

impl Debug for Processors {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "Current Processes:\n{:?}", self.cpus)
    }
}

pub struct LocalCPU {
    pub current_pid: Option<Id>,
    apic_id: u8,
}

impl Debug for LocalCPU {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "[CPU {}, Running: {:?}]", self.apic_id, self.current_pid)
    }
}

impl LocalCPU {
    pub fn new(cpuid: u8) -> LocalCPU {
        LocalCPU {
            apic_id: cpuid,
            current_pid: None
        }
    }
}

impl Processors {
    pub fn current_cpu(&mut self) -> &mut LocalCPU {
        let cpuid = GLOBAL_APIC.lock().apic_id();
        if !self.cpus.contains_key(&cpuid) {
            self.cpus.insert(cpuid, LocalCPU::new(cpuid));
        }
        self.cpus.get_mut(&cpuid).expect("has cpu")
    }
}