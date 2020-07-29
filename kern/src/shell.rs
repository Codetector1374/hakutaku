use alloc::vec::Vec;
use alloc::string::String;
use core::fmt::Write;
use core::str;
use crate::hardware::keyboard::{blocking_get_char, GLOB_KEYBOARD};
use crate::process::scheduler::GlobalScheduler;
use crate::{SCHEDULER, ACPI};
use crate::hardware::pit::PIT;
use kernel_api::syscall::sleep;
use core::time::Duration;
use crate::device::usb::G_USB;
use x86_64::instructions::interrupts::without_interrupts;
use crate::device::ahci::G_AHCI;
use crate::hardware::apic::GLOBAL_APIC;
use crate::device::pci::class::PCIDeviceClass;
use crate::device::uart::serial16650::{Serial16650, COM1_BASE_ADDR};
use crate::device::uart::UART;

/// Error type for `Command` parse failures.
#[derive(Debug)]
enum Error {
    Empty,
    TooManyArgs,
}

pub struct Shell {
}

/// A structure representing a single shell command.
struct Command {
    args: Vec<String>,
}

impl Command {
    /// Parse a command from a string `s` using `buf` as storage for the
    /// arguments.
    ///
    /// # Errors
    ///
    /// If `s` contains no arguments, returns `Error::Empty`. If there are more
    /// arguments than `buf` can hold, returns `Error::TooManyArgs`.
    fn parse(s: &str) -> Result<Command, Error> {
        let mut args: Vec<String> = Vec::new();
        for arg in s.split(' ').filter(|a| !a.is_empty()) {
            args.push(String::from(arg));
        }

        if args.is_empty() {
            return Err(Error::Empty);
        }

        Ok(Command { args })
    }

    /// Returns this command's path. This is equivalent to the first argument.
    fn path(&self) -> &str {
        &self.args[0]
    }
}


impl Shell {
    pub fn new() -> Shell {
        Shell{
        }
    }
    /// Starts a shell using `prefix` as the prefix for each line. This function
    /// never returns.
    pub fn shell(&mut self, prefix: &str) {
        let mut input_vec: Vec<u8> = Vec::new();
        loop {
            input_vec.truncate(0);
            {
                print!("{}", prefix);
                loop {
                    let chr = blocking_get_char();
                    match chr {
                        b'\r' | b'\n' => { // \r \n
                            break;
                        }
                        3 => { // EXT (Ctrl-C)
                            print!("\n");
                            return;
                        }
                        32..=126 => {
                            input_vec.push(chr);
                            print!("{}", chr as char);
                        }
                        8 | 127 => {
                            if input_vec.is_empty() {
                                print!("\u{7}"); // Ring Bell
                            } else {
                                input_vec.truncate(input_vec.len() - 1);
                                print!("\u{8} \u{8}"); // Erase 1 char
                            }
                        }
                        _ => {
                            print!("\u{7}"); // Ring Bell
                        } // Bell
                    };
                };
                print!("\n")
            }
            let user_command_input = str::from_utf8(&input_vec).unwrap();
            let cmd_result =
                Command::parse(&user_command_input);
            match cmd_result {
                Ok(cmd) => {
                    let exec_result = self.process_command(&cmd);
                    match exec_result {
                        Ok(val) => {
                            if val >= 0 {
                                continue;
                            } else {
                                return;
                            }
                        }
                        Err(_) => {
                            println!("unknown command: {}", cmd.path());
                        }
                    }
                }
                Err(e) => {
                    match e {
                        Error::TooManyArgs => {
                            println!("error: too many arguments");
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    fn process_command(&mut self, command: &Command) -> Result<i8, ()> {
        match command.path() {
            "reboot" => {
                let zero = 0u64;
                unsafe { asm!("mov cr3, {0}", in(reg) zero) };
                Ok(0)
            },
            "echo" => {
                for (i, v) in command.args.iter().enumerate() {
                    if i == 0 {
                        continue;
                    }
                    print!("{} ", v);
                }
                print!("\n");
                Ok(0)
            },
            "lsahci" => {
                for dev in G_AHCI.attached_devices.read().iter() {
                    println!("{:?}", dev);
                }
                Ok(0)
            },
            "lsblk" => {
                use crate::storage::block::G_BLOCK_DEV_MGR;
                for name in G_BLOCK_DEV_MGR.read().list_devices() {
                    println!("device: {}", name);
                }
                Ok(0)
            },
            "help" => {
                print!("You call out for help.");
                for _ in 0..3 {
                    sleep(Duration::from_secs(1)).expect("sleep()");
                    print!(".")
                }
                println!("\nBut no one answers.");
                Ok(0)
            }
            "u" => {
                let repeat = if command.args.len() > 1 {
                    command.args[1].parse::<u32>().unwrap_or_else(|_|{1})
                } else {
                    1
                };
                for _ in 0..repeat {
                    G_USB.xhci.read().get(0).expect("LOL").send_nop();
                    sleep(Duration::from_millis(10)).expect("");
                }
                Ok(0)
            },
            "p" => {
                without_interrupts(|| {
                    G_USB.xhci.read().get(0).expect("LOL").poll_ports();
                });
                Ok(0)
            }
            "sleep" => {
                if command.args.len() > 1 {
                    let sec = command.args[1].parse::<u64>().unwrap_or_else(|_| {0});
                    if sec == 0 {
                        println!("Unable to parse time");
                        return Ok(1)
                    } else {
                        let time = sleep(Duration::from_secs(sec)).expect("sleep works");
                        println!("slept for {:?}", time);
                    }
                } else {
                    println!("Usage: sleep <seconds>")
                }
                Ok(0)
            },
            "date" => {
                let time = PIT::current_time();
                println!("Since boot: {:?}", time);
                Ok(0)
            },
            "ps" => {
                println!("  PID  |   STATE  ");
                println!("================================================================================");
                for p in SCHEDULER.summary() {
                    println!("  {:03}  | {}", p.pid, p.state)
                }
                Ok(0)
            }
            "lspci" => {
                use crate::device::pci::GLOBAL_PCI;
                let scan = command.args.len() >= 2 && command.args[1].eq("-s");
                let devs = without_interrupts(||{
                    debug!("Locking PCI Controller");
                    if scan {
                        GLOBAL_PCI.lock().scan_pci_bus();
                    }
                    GLOBAL_PCI.lock().enumerate_pci_bus()
                });
                for dev in &devs {
                    match dev.info.class {
                        PCIDeviceClass::Other(_c,_s,_i) => {},
                        PCIDeviceClass::BridgeDevice(_) => {},
                        _ => {
                            println!("PCI: {:04x}:{:02x}:{:02x}=> {:?}", dev.bus, dev.device_number, dev.func, dev.info.class);
                        }
                    }
                }
                Ok(0)
            },
            "uart" => {
                let mut serial = Serial16650::new_from_port(COM1_BASE_ADDR);
                serial.write(0x69);
                if serial.verify() {
                    println!("Real Serail")
                }
                Ok(0)
            },
            "rdblk" => {
                use crate::storage::block::G_BLOCK_DEV_MGR;
                use pretty_hex::*;
                if command.args.len() >= 3 {
                    let dev = &command.args[1];
                    let sec = command.args[2].parse::<u64>().unwrap_or_else(|_| {0});
                    match G_BLOCK_DEV_MGR.read().devices.get(dev) {
                        Some(blk_dev) => {
                            let mut buf = [0u8; 512];
                            blk_dev.read_sector(sec, &mut buf).ok().expect("thing");
                            println!("First Half: \n{:?}", buf[..256].as_ref().hex_dump());
                            println!("press any key to continue...");
                            blocking_get_char();
                            println!("Second Half: \n{:?}", buf[256..].as_ref().hex_dump());
                        }
                        _ => {
                            println!("{} not found", dev);
                        }
                    }
                    Ok(0)
                } else {
                    println!("rdblk [device] [sector]");
                    Ok(-1)
                }
            },
            "lsusb" => {
                for dev in G_USB.devices.read().iter() {
                    println!("Bus {:03} Device {:03}: {:04x}:{:04x} {} {}",
                             dev.bus(), dev.device(),
                             dev.device_descriptor().vid, dev.device_descriptor().pid,
                             dev.manufacture_string(), dev.product_string()
                    );
                }
                Ok(0)
            },
            "exit" => {
                Ok(-1)
            },
            _ => Err(())
        }
    }
}

