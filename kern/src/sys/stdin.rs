use spin::Mutex;
use alloc::collections::VecDeque;
use core::sync::atomic::{AtomicUsize, Ordering};
use core::sync::atomic::Ordering::Acquire;
use crate::structure::mpmc_queue::MpmcQueue;

const MAX_BUFFER_CHARACTER: usize = 32;

// TODO: Evaulate safty of this code. But this works for now LOL
lazy_static! {
pub static ref STD_IN: StandardInput = StandardInput {
    queue: MpmcQueue::new(MAX_BUFFER_CHARACTER),
};
}

pub struct StandardInput {
    queue: MpmcQueue<u8>
}

impl StandardInput {
    pub fn insert(&self, char: u8) {
        self.queue.enqueue(char);
    }

    pub fn pop(&self) -> Option<u8> {
        self.queue.dequeue()
    }

    pub fn has_data(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn blocking_get_char(&self) -> u8 {
        loop {
            match self.pop() {
                Some(thing) => return thing,
                _ => {}
            }
        }
    }
}