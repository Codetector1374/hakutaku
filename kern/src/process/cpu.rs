use crate::process::process::{Id, Process};
use crate::sys::apic::GLOBAL_APIC;
use hashbrown::HashMap;
use core::fmt::{Debug, Formatter};
use crate::ACPI;
use crate::process::scheduler::idle_process;

pub struct Processors {
    cpus: HashMap<u8, LocalCPU>,
}

impl Default for Processors {
    fn default() -> Self {
        let mut procs = Self {
            cpus: Default::default(),
        };

        if let Some(acpi) = ACPI.read().as_ref() {
            let bsp = acpi.boot_processor.as_ref().expect("no bsp?");
            procs.cpus.insert(bsp.local_apic_id, LocalCPU::new(bsp.local_apic_id, bsp.processor_uid));

            for ap in acpi.application_processors.iter() {
                use acpi::ProcessorState;
                if let ProcessorState::Disabled = ap.state  {
                } else {
                    procs.cpus.insert(ap.local_apic_id, LocalCPU::new(ap.local_apic_id, ap.processor_uid));
                }
            }
        }
        procs
    }
}

impl Debug for Processors {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "Current Processes:\n{:#?}", self.cpus)
    }
}

pub struct LocalCPU {
    pub current_pid: Option<Id>,
    pub idle_task: Process,
    pub proc_id: u8,
    pub apic_id: u8,
}

impl Debug for LocalCPU {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "[CPUID: {}, LAPIC: {},  Running: {:?}]", self.proc_id ,self.apic_id, self.current_pid)
    }
}

impl LocalCPU {
    pub fn new(apic_id: u8, cpuid: u8) -> LocalCPU {
        LocalCPU {
            apic_id,
            idle_task: Process::new_idle(idle_process as u64),
            proc_id: cpuid,
            current_pid: None
        }
    }
}

impl Processors {
    pub fn current_cpu(&mut self) -> &mut LocalCPU {
        let cpuid = GLOBAL_APIC.read().apic_id();
        if !self.cpus.contains_key(&cpuid) {
            panic!("current cpu is not registered?");
        }
        self.cpus.get_mut(&cpuid).expect("has cpu")
    }
}