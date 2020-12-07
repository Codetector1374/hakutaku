use core::sync::atomic::{AtomicUsize, Ordering, AtomicU16};
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::sync::atomic::Ordering::{Release, Acquire, Relaxed};

struct QueueCell<T> {
    sequence: AtomicUsize,
    data: Option<T>,
}

pub struct MpmcQueue<T> {
    enqueue: AtomicUsize,
    dequeue: AtomicUsize,
    buffer: Box<[QueueCell<T>]>,
    count: AtomicUsize,
    cap_mask: usize,
}

const IN_USE: u16 = 0x100;

impl<T> MpmcQueue<T> {
    pub fn new(capacity: usize) -> MpmcQueue<T> {
        assert_eq!((capacity - 1) & capacity, 0, "Only power of 2 capacity is supported");
        let mut vec = Vec::<QueueCell<T>>::with_capacity(capacity);
        for i in 0..capacity {
            vec.push(QueueCell { sequence: AtomicUsize::new(i), data: Option::None });
        }
        MpmcQueue {
            enqueue: AtomicUsize::new(0),
            dequeue: AtomicUsize::new(0),
            buffer: vec.into_boxed_slice(),
            cap_mask: capacity - 1,
            count: AtomicUsize::new(0),
        }
    }

    pub fn enqueue(&self, item: T) -> bool {
        let mut pos = self.enqueue.load(Ordering::Relaxed);
        loop {
            /* Load Sequence Number
                1) If sequence number == pos, then we found an empty cell. CAS it and if it works then LOL
                2) If seq num < pos, this is the case when we wrapped around. So the queue is full
                3) If seq > pos, someone else have "stole" the slot while we are doing needfuls. Restart
             */
            let seq = self.buffer[pos & self.cap_mask].sequence.load(Ordering::Acquire);
            if seq == pos { // 1)
                // Here we can use relaxed because we don't really care when does this happen, just when it happens
                // the value is still pos.
                if self.enqueue.compare_and_swap(pos, pos + 1, Ordering::Relaxed) == pos {
                    // This is safe because we have exclusive access to this cell
                    unsafe {
                        (*(&self.buffer[pos & self.cap_mask] as *const QueueCell<T> as *mut QueueCell<T>)).data = Some(item);
                    }
                    // This effectively marks the cell contains data. Aka lose exclusive access
                    self.buffer[pos & self.cap_mask].sequence.store(pos + 1, Release);
                    self.count.fetch_add(1, Relaxed);
                    return true;
                }
                // If the CAS failed, we need to try again
            } else if seq < pos { // 2)
                return false; // Full
            } else {
                pos = self.enqueue.load(Relaxed); // Relaxed load because only the CAS on this thing matters
            }
        }
    }

    pub fn dequeue(&self) -> Option<T> {
        let mut pos = self.dequeue.load(Ordering::Relaxed);
        loop {
            /* Load Sequence Number
                1) If sequence number == (pos + 1), then we found the cell. CAS it and if it works then LOL
                2) If seq num < (pos + 1), this is the case we have exhausted the queue
                3) If seq > (pos + 1), someone else have "stole" the slot while we are doing needfuls. Restart
             */
            let seq = self.buffer[pos & self.cap_mask].sequence.load(Ordering::Acquire);
            if seq == (pos + 1) { // 1)
                // Here we can use relaxed because we don't really care when does this happen, just when it happens
                // the value is still pos.
                if self.dequeue.compare_and_swap(pos, pos + 1, Ordering::Relaxed) == pos {
                    // This is safe because we have exclusive access to this cell
                    let data = unsafe {
                        (*(&self.buffer[pos & self.cap_mask] as *const QueueCell<T> as *mut QueueCell<T>)).data.take()
                    };
                    // This effectively marks the cell contains data. Aka lose exclusive access
                    self.buffer[pos & self.cap_mask].sequence.store(pos + 1 + self.cap_mask, Release);
                    self.count.fetch_sub(1, Relaxed);
                    return data;
                }
                // If the CAS failed, we need to try again
            } else if seq < (pos + 1) { // 2)
                return None; // Full
            } else {
                pos = self.enqueue.load(Relaxed); // Relaxed load because only the CAS on this thing matters
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.count.load(Acquire) == 0
    }
}
