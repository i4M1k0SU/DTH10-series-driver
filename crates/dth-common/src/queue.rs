//! [`TsQueue`]: the bounded queue a tuner's reader thread fills with TS and
//! a consumer (the daemon's connection, the command's output) drains.

use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use crate::LockExt;

/// Upper bound on queued bytes before the oldest chunks are dropped.
const QUEUE_LIMIT_BYTES: usize = 32 * 1024 * 1024;

struct QueueInner {
    chunks: VecDeque<Vec<u8>>,
    bytes: usize,
    /// Bytes thrown away because the consumer did not keep up (the queue is
    /// bounded; the oldest chunks go first).
    dropped: u64,
}

/// Thread-safe queue of 188-byte-aligned TS chunks.
#[derive(Clone)]
pub struct TsQueue {
    inner: Arc<(Mutex<QueueInner>, Condvar)>,
}

impl Default for TsQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl TsQueue {
    pub fn new() -> Self {
        TsQueue {
            inner: Arc::new((Mutex::new(QueueInner { chunks: VecDeque::new(), bytes: 0, dropped: 0 }), Condvar::new())),
        }
    }

    pub fn push(&self, chunk: Vec<u8>) {
        let (m, cv) = &*self.inner;
        let mut q = m.locked();
        q.bytes += chunk.len();
        q.chunks.push_back(chunk);
        while q.bytes > QUEUE_LIMIT_BYTES {
            if let Some(old) = q.chunks.pop_front() {
                q.bytes -= old.len();
                q.dropped += old.len() as u64;
            }
        }
        cv.notify_all();
    }

    /// Bytes dropped so far because the consumer did not keep up.
    pub fn dropped_bytes(&self) -> u64 {
        self.inner.0.locked().dropped
    }

    pub fn pop(&self) -> Option<Vec<u8>> {
        let (m, _) = &*self.inner;
        let mut q = m.locked();
        let c = q.chunks.pop_front()?;
        q.bytes -= c.len();
        Some(c)
    }

    pub fn len(&self) -> usize {
        self.inner.0.locked().chunks.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn purge(&self) {
        let mut q = self.inner.0.locked();
        q.chunks.clear();
        q.bytes = 0;
    }

    /// Wait until a chunk is available. `timeout_ms == 0` means "do not wait".
    pub fn wait(&self, timeout_ms: u32) -> bool {
        let (m, cv) = &*self.inner;
        let q = m.locked();
        if !q.chunks.is_empty() {
            return true;
        }
        if timeout_ms == 0 {
            return false;
        }
        let (q, _) = cv.wait_timeout(q, Duration::from_millis(timeout_ms as u64)).unwrap();
        !q.chunks.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queue_wait_and_limit() {
        let q = TsQueue::new();
        assert!(!q.wait(0));
        q.push(vec![0x47; 188]);
        assert!(q.wait(0));
        assert_eq!(q.pop().map(|c| c.len()), Some(188));
        assert!(q.pop().is_none());
        for _ in 0..40 {
            q.push(vec![0; 1024 * 1024]);
        }
        assert!(q.len() <= 32);
    }
}
