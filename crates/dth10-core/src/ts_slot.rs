//! The slots of the bulk TS pipe: every bulk IN transfer is a whole number
//! of 256-byte slots, each carrying one 188-byte TS packet. Where the packet
//! sits in its slot depends on the model ([`SlotLayout::for_product`]).

/// Size of one slot of the bulk TS pipe.
pub const SLOT: usize = 256;
/// Size of a TS packet.
pub const TS_PACKET: usize = 188;

/// How a TS packet sits in its 256-byte slot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SlotLayout {
    /// The packet is the first 188 bytes of the slot; the remaining 68 bytes
    /// are padding. Every model but the DT-H10/U7.
    #[default]
    Plain,
    /// The DT-H10/U7: the slot starts with a 4-byte header and the packet
    /// follows it, split in two when header byte 1 (`L`) is not 0. With
    /// `L` = 0 the packet is the 188 bytes at +4. Otherwise it is the `L` + 4
    /// bytes at +4 followed by the 184 − `L` bytes at +8 + `L` rounded up to
    /// a multiple of 4: the rest of the packet is stored 4-byte aligned (`L`
    /// looks like the size of the adaptation field). This is how the vendor
    /// driver of the U7 cuts the packets out of the slots. It reads nothing
    /// else of the header. `L` = 184 comes out the same as `L` = 0; for
    /// larger values the vendor driver's copy is not defined, and the 188
    /// bytes at +4 are taken here.
    Split,
}

impl SlotLayout {
    /// The layout of the model with USB product ID `pid`.
    pub fn for_product(pid: u16) -> Self {
        if pid == crate::PID_DTH10_U7 { SlotLayout::Split } else { SlotLayout::Plain }
    }

    /// The TS packet of `slot`. [`SlotLayout::Plain`] borrows it from the
    /// slot; [`SlotLayout::Split`] puts it together in `scratch`.
    pub fn packet<'a>(self, slot: &'a [u8; SLOT], scratch: &'a mut [u8; TS_PACKET]) -> &'a [u8; TS_PACKET] {
        match self {
            SlotLayout::Plain => slot.first_chunk().unwrap(),
            SlotLayout::Split => {
                let l = slot[1] as usize;
                if l == 0 || l >= TS_PACKET - 4 {
                    scratch.copy_from_slice(&slot[4..4 + TS_PACKET]);
                } else {
                    let head = l + 4;
                    let rest = 8 + l.next_multiple_of(4);
                    scratch[..head].copy_from_slice(&slot[4..4 + head]);
                    scratch[head..].copy_from_slice(&slot[rest..rest + TS_PACKET - head]);
                }
                scratch
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A test packet: sync byte, then bytes counting up from 1.
    fn packet() -> [u8; TS_PACKET] {
        let mut p = [0u8; TS_PACKET];
        p[0] = 0x47;
        for (i, b) in p.iter_mut().enumerate().skip(1) {
            *b = i as u8;
        }
        p
    }

    /// The U7 slot of `pkt` with header byte 1 = `l`: header, the first
    /// `l` + 4 bytes, padding up to the next 4-byte boundary, the rest;
    /// padding bytes are 0xEE.
    fn split_slot(pkt: &[u8; TS_PACKET], l: u8) -> [u8; SLOT] {
        let mut s = [0xEEu8; SLOT];
        s[..4].copy_from_slice(&[0xA5, l, 0x5A, 0xC3]);
        let l = l as usize;
        if l == 0 {
            s[4..4 + TS_PACKET].copy_from_slice(pkt);
        } else {
            s[4..8 + l].copy_from_slice(&pkt[..4 + l]);
            let rest = 8 + l.next_multiple_of(4);
            s[rest..rest + 184 - l].copy_from_slice(&pkt[4 + l..]);
        }
        s
    }

    #[test]
    fn layout_by_product() {
        assert_eq!(SlotLayout::for_product(0x017B), SlotLayout::Split);
        for pid in [0x015F, 0x016B, 0x017C, 0x0199, 0x01FF, 0x0202, 0x1234] {
            assert_eq!(SlotLayout::for_product(pid), SlotLayout::Plain, "0x{pid:04x}");
        }
        assert_eq!(SlotLayout::default(), SlotLayout::Plain);
    }

    #[test]
    fn plain_is_the_first_188_bytes() {
        let mut slot = [0xEEu8; SLOT];
        slot[..TS_PACKET].copy_from_slice(&packet());
        let mut scratch = [0u8; TS_PACKET];
        assert_eq!(SlotLayout::Plain.packet(&slot, &mut scratch), &packet());
        assert_eq!(scratch, [0u8; TS_PACKET]);
    }

    #[test]
    fn split_without_header_length() {
        let p = packet();
        let mut scratch = [0u8; TS_PACKET];
        assert_eq!(SlotLayout::Split.packet(&split_slot(&p, 0), &mut scratch), &p);
        // Only byte 1 of the header counts.
        let mut s = split_slot(&p, 0);
        s[0] = 0xFF;
        s[2] = 0xFF;
        s[3] = 0xFF;
        assert_eq!(SlotLayout::Split.packet(&s, &mut scratch), &p);
    }

    #[test]
    fn split_every_header_length() {
        let p = packet();
        for l in 1..=183u8 {
            let mut scratch = [0u8; TS_PACKET];
            assert_eq!(SlotLayout::Split.packet(&split_slot(&p, l), &mut scratch), &p, "L = {l}");
        }
    }

    #[test]
    fn split_offsets_pinned() {
        // L = 1: 5 bytes at +4, the other 183 from +12 (8 + 4).
        let mut s = [0u8; SLOT];
        s[1] = 1;
        s[4..9].copy_from_slice(&[0x47, 1, 2, 3, 4]);
        s[9..12].fill(0xEE);
        for i in 0..183 {
            s[12 + i] = 0x10 + (i % 0x50) as u8;
        }
        let mut scratch = [0u8; TS_PACKET];
        let out = SlotLayout::Split.packet(&s, &mut scratch);
        assert_eq!(&out[..5], &[0x47, 1, 2, 3, 4]);
        assert_eq!(out[5], 0x10);
        assert_eq!(out[187], 0x10 + 182 % 0x50);
        assert!(!out.contains(&0xEE));
        // L = 4: no padding, the rest follows at +12 directly.
        let mut s = [0x11u8; SLOT];
        s[1] = 4;
        s[12] = 0x22;
        s[192] = 0x33; // first byte past the packet
        let out = SlotLayout::Split.packet(&s, &mut scratch);
        assert_eq!(out[8], 0x22);
        assert_eq!(out[187], 0x11);
        assert!(!out.contains(&0x33));
        // L = 183: 187 bytes at +4, the last byte at +8 + 184 = +192.
        let mut s = [0x11u8; SLOT];
        s[1] = 183;
        s[191] = 0x44; // padding
        s[192] = 0x55;
        let out = SlotLayout::Split.packet(&s, &mut scratch);
        assert_eq!(&out[183..], &[0x11, 0x11, 0x11, 0x11, 0x55]);
    }

    #[test]
    fn split_edge_header_lengths() {
        // 184 (nothing after the head) and anything larger: the 188 bytes at +4.
        let mut s = [0u8; SLOT];
        for (i, b) in s.iter_mut().enumerate() {
            *b = i as u8;
        }
        for l in [184u8, 185, 200, 255] {
            s[1] = l;
            let mut scratch = [0u8; TS_PACKET];
            let out = SlotLayout::Split.packet(&s, &mut scratch);
            assert_eq!(out[0], 4, "L = {l}");
            assert_eq!(&out[1..], &s[5..4 + TS_PACKET], "L = {l}");
        }
    }
}
