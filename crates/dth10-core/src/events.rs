//! Interrupt-endpoint event notifications.
//!
//! Each interrupt transfer (512 bytes) carries one event:
//!
//! ```text
//! u16 hdr0    : byte1 bit0 → filter #0, bit1 → filter #1 (both possible)
//! u16 class   : 0x0000 or 0xCBDA
//! u16 code
//! u16 len     : payload length
//! u8  payload[len] at +8
//! ```
//!
//! The events are CA/PBDA related (smart-card status, EMM PID, LNB
//! over-current); none of them signals lock or TS discontinuity, so a plain
//! TS receiver may ignore them.

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Event {
    /// Bit0 = filter #0, bit1 = filter #1.
    pub targets: u8,
    pub class: u16,
    pub code: u16,
    pub payload: Vec<u8>,
}

/// Known event codes.
pub mod code {
    /// class 0x0000: LNB over-current check request (model "h70"); the driver
    /// re-reads wIndex 0x1010 and switches the LNB supply off if bit0 of byte
    /// 12 is clear.
    pub const LNB_CHECK: (u16, u16) = (0x0000, 0x1011);
    /// class 0xCBDA: smart-card (B-CAS) status changed.
    pub const CA_SMART_CARD_STATUS: (u16, u16) = (0xCBDA, 0xCACD);
    /// class 0xCBDA: PBDA event pending (no payload).
    pub const PBDA_EVENT_PENDING: (u16, u16) = (0xCBDA, 0xBDDE);
    /// class 0xCBDA: PBDA event with payload (tuner event + record).
    pub const PBDA_EVENT_RECORD: (u16, u16) = (0xCBDA, 0xCA02);
    /// class 0xCBDA: register an EMM PID hook (payload = u16 LE PID).
    pub const EMM_PID: (u16, u16) = (0xCBDA, 0xBCED);
}

impl Event {
    /// Parse one interrupt transfer. Returns `None` for truncated buffers.
    pub fn parse(buf: &[u8]) -> Option<Event> {
        if buf.len() < 8 {
            return None;
        }
        let targets = buf[1] & 0x03;
        let class = u16::from_le_bytes([buf[2], buf[3]]);
        let code = u16::from_le_bytes([buf[4], buf[5]]);
        let len = u16::from_le_bytes([buf[6], buf[7]]) as usize;
        let end = (8 + len).min(buf.len());
        Some(Event { targets, class, code, payload: buf[8..end].to_vec() })
    }

    pub fn kind(&self) -> (u16, u16) {
        (self.class, self.code)
    }

    /// EMM PID carried by an [`code::EMM_PID`] event.
    pub fn emm_pid(&self) -> Option<u16> {
        if self.kind() != code::EMM_PID || self.payload.len() < 2 {
            return None;
        }
        Some(u16::from_le_bytes([self.payload[0], self.payload[1]]) & 0x1FFF)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_event() {
        let mut b = vec![0u8; 512];
        b[1] = 0x01;
        b[2..4].copy_from_slice(&0xCBDAu16.to_le_bytes());
        b[4..6].copy_from_slice(&0xBCEDu16.to_le_bytes());
        b[6..8].copy_from_slice(&2u16.to_le_bytes());
        b[8..10].copy_from_slice(&0x0001u16.to_le_bytes());
        let e = Event::parse(&b).unwrap();
        assert_eq!(e.targets, 1);
        assert_eq!(e.kind(), code::EMM_PID);
        assert_eq!(e.emm_pid(), Some(1));
    }
}
