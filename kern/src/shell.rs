use alloc::vec::Vec;
use alloc::string::String;
use core::fmt::Write;
use core::str;
use crate::hardware::keyboard::blocking_get_char;
use crate::pci::device::PCIDevice;
use crate::pci::*;
use crate::pci::class::*;
use crate::pci::device::xhci::XHCI;
use crate::process::scheduler::GlobalScheduler;
use crate::SCHEDULER;
use crate::hardware::pit::PIT;
use kernel_api::syscall::sleep;
use bitflags::_core::time::Duration;
use crate::device::usb::G_USB;
use x86_64::instructions::interrupts::without_interrupts;

/// Error type for `Command` parse failures.
#[derive(Debug)]
enum Error {
    Empty,
    TooManyArgs,
}

pub struct Shell {
    xhci: Option<XHCI>,
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
            xhci: None,
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
            "sys" => {
                unsafe {
                    asm!("
                    mov rax, 40
                    int 0x80
                    ":::"rax":"volatile", "intel");
                }
                Ok(0)
            },
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
                SCHEDULER.critical(|s| {
                    println!("{:#?}", s.cpus);
                });
                Ok(0)
            }
            "u" => {
                without_interrupts(||{
                    G_USB.xhci.lock().as_mut().unwrap().send_nop();
                });
                Ok(0)
            },
            "lspci" => {
                let scan = command.args.len() >= 2 && command.args[1].eq("-s");
                let devs = without_interrupts(||{
                    debug!("Locking PCI Controller");
                    if scan {
                        GLOBAL_PCI.lock().scan_pci_bus();
                    }
                    GLOBAL_PCI.lock().enumerate_pci_bus()
                });
                for dev in &devs {
                    println!("PCI: {:04x}:{:02x}:{:02x}=> {:?}", dev.bus, dev.device_number, dev.func, dev.info.class);
                }
                Ok(0)
            }
            "exit" => {
                Ok(-1)
            },
            _ => Err(())
        }
    }
}

