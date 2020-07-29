use log::{LevelFilter, Metadata, Record, Level};
use crate::device::uart::SERIAL_PORTS;

struct KernelLogger;

static LOGGER: KernelLogger = KernelLogger;

impl log::Log for KernelLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            // Record.target
            if record.level() < Level::Trace {
                println!("[{}] {}", record.level(), record.args());
            }
            for p in SERIAL_PORTS.read().ports.iter() {
                write!(p.lock(), "[{}][{}]: {}\n", record.level(), record.target(), record.args()).unwrap();
            }
        }
    }

    fn flush(&self) {}
}

pub unsafe fn init_logger() {
    log::set_logger(&LOGGER)
        .map(|()| {
            log::set_max_level(if let Some(_) = option_env!("VERBOSE_BUILD") {
                LevelFilter::Trace
            } else {
                LevelFilter::Trace
            })
        })
        .expect("Failed to initialize the logger");
}
