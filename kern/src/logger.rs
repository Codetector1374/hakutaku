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
            println!("[{}] {}", record.level(), record.args());
            match SERIAL_PORTS.try_read() {
                Some(r_handele) => {
                    if let Some(p) = r_handele.ports.values().find(|_|{true}) {
                        match p.try_lock() {
                            Some(mut p) => {
                                write!(p, "[{:?}][{}]: {}\n", &record.level(), record.target(), record.args()).unwrap();
                            },
                            _ => {}
                        }
                    }
                },
                _ => {}
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
                LevelFilter::Debug
            })
        })
        .expect("Failed to initialize the logger");
}
