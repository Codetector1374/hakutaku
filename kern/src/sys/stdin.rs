use spin::Mutex;
use alloc::collections::VecDeque;
use core::sync::atomic::{AtomicUsize, Ordering};
use core::sync::atomic::Ordering::Acquire;

const MAX_BUFFER_CHARACTER: usize = 32;

// TODO: Evaulate safty of this code. But this works for now LOL
pub static STD_IN: StandardInput = StandardInput {
    buffer: [0u8; MAX_BUFFER_CHARACTER],
    enqueue: AtomicUsize::new(0),
    safe_todequeue: AtomicUsize::new(0),
    dequeue: AtomicUsize::new(0),
};

pub struct StandardInput {
    buffer: [u8; MAX_BUFFER_CHARACTER],
    enqueue: AtomicUsize,
    safe_todequeue: AtomicUsize,
    dequeue: AtomicUsize,
}

impl StandardInput {
    pub fn insert(&self, char: u8) {
        let enq = self.enqueue.load(Ordering::Relaxed);
        let next_enq = (enq + 1) % MAX_BUFFER_CHARACTER;
        if next_enq != self.safe_todequeue.load(Ordering::SeqCst) {
            if self.enqueue.compare_and_swap(enq, next_enq, Ordering::AcqRel) == enq {
                unsafe { *(&self.buffer[enq] as *const u8 as *mut u8) = char };
                self.safe_todequeue.store(next_enq, Ordering::Release);
                return;
            }
        }
    }

    pub fn pop(&self) -> Option<u8> {
        let deq = self.dequeue.load(Ordering::SeqCst);
        if deq != self.safe_todequeue.load(Ordering::SeqCst) {
            let val = self.buffer[deq];
            match self.dequeue.compare_exchange(deq, (deq + 1) % MAX_BUFFER_CHARACTER, Ordering::AcqRel, Acquire) {
                Ok(_) => Some(val),
                _ => None
            }
        } else {
            None
        }
    }

    pub fn has_data(&self) -> bool {
        self.dequeue.load(Ordering::Relaxed) != self.enqueue.load(Ordering::Relaxed)
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