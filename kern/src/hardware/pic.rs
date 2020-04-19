use cpuio::{UnsafePort, Port};

const NUM_INT_EACH_PIC: u8 = 8;

const END_OF_INTERRUPT: u8 = 0x20;

// PIC CONSTS
const ICW1_ICW4: u8 = 0x01;       /* ICW4 (not) needed */
const ICW1_SINGLE: u8 = 0x02;        /* Single (cascade) mode */
const ICW1_INTERVAL4: u8 = 0x04;        /* Call address interval 4 (8) */
const ICW1_LEVEL: u8 = 0x08;    /* Level triggered (edge) mode */
const ICW1_INIT: u8 = 0x10;  /* Initialization - required! */

const ICW4_8086: u8 = 0x01;          /* 8086/88 (MCS-80/85) mode */
const ICW4_AUTO: u8 = 0x02;          /* Auto (normal) EOI */
const ICW4_BUF_SLAVE: u8 = 0x08;     /* Buffered mode/slave */
const ICW4_BUF_MASTER: u8 = 0x0C;     /* Buffered mode/master */
const ICW4_SFNM: u8 = 0x10;        /* Special fully nested (not) */


const PIC1_CMD_PORT: u16 = 0x20;
const PIC2_CMD_PORT: u16 = 0xA0;

pub struct Pic {
    offset: u8,
    command: UnsafePort<u8>,
    data: UnsafePort<u8>,
}

impl Pic {
    fn handles_interrupt(&self, int_id: u8) -> bool {
        self.offset <= int_id && int_id < self.offset + NUM_INT_EACH_PIC
    }

    unsafe fn end_of_interrupt(&mut self) {
        self.command.write(END_OF_INTERRUPT);
    }
}

pub struct ChainedPics {
    pics: [Pic; 2],
}

impl ChainedPics {
    pub const unsafe fn new(offset1: u8, offset2: u8) -> ChainedPics {
        ChainedPics {
            pics: [
                Pic {
                    offset: offset1,
                    command: UnsafePort::new(PIC1_CMD_PORT),
                    data: UnsafePort::new(PIC1_CMD_PORT + 1), // CMD + 1 is Data
                },
                Pic {
                    offset: offset2,
                    command: UnsafePort::new(PIC2_CMD_PORT),
                    data: UnsafePort::new(PIC2_CMD_PORT + 1),
                },
            ]
        }
    }


    pub unsafe fn initialize(&mut self) {
        /* Port 0x80 is used for 'checkpoints' during POST. */
        /* The Linux kernel seems to think it is free for use :-/ */
        let mut wait_port: Port<u8> = Port::new(0x80);
        let mut wait = || { wait_port.write(0); };

        let saved_mask1 = self.pics[0].data.read();
        let saved_mask2 = self.pics[1].data.read();

        // Init Logic
        self.pics[0].command.write(ICW1_INIT | ICW1_ICW4);
        wait();
        self.pics[1].command.write(ICW1_INIT | ICW1_ICW4);
        wait();

        self.pics[0].data.write(self.pics[0].offset);
        wait();
        self.pics[1].data.write(self.pics[1].offset);
        wait();

        self.pics[0].data.write(4);
        wait();
        self.pics[1].data.write(2);
        wait();

        self.pics[0].data.write(ICW4_8086);
        wait();
        self.pics[1].data.write(ICW4_8086);
        wait();

        self.pics[0].data.write(saved_mask1);
        self.pics[1].data.write(saved_mask2);
    }

    pub fn handles_interrupt(&self, int_id: u8) -> bool {
        self.pics.iter().any(|p| p.handles_interrupt(int_id))

    }

    pub unsafe fn notify_end_of_interrupt(&mut self, interrupt_id: u8) {
        if self.handles_interrupt(interrupt_id) {
            if self.pics[1].handles_interrupt(interrupt_id) {
                self.pics[1].end_of_interrupt();
            }
            self.pics[0].end_of_interrupt();
        }
    }
}
