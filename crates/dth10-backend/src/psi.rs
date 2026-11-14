//! Minimal PAT/PMT collector, used to learn a channel's programs and PIDs
//! for the device-side descrambling mode (program PID list, 0xCBDA/0x1000).

use std::collections::{BTreeMap, HashMap};

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Program {
    pub number: u16,
    pub pmt_pid: u16,
    pub pcr_pid: u16,
    pub ecm_pids: Vec<u16>,
    pub es_pids: Vec<u16>,
}

impl Program {
    /// PIDs to hand to the device: PMT, ECMs, PCR and all elementary streams.
    pub fn pid_list(&self) -> Vec<u16> {
        let mut v = vec![self.pmt_pid];
        v.extend(&self.ecm_pids);
        if self.pcr_pid != 0x1FFF && !v.contains(&self.pcr_pid) {
            v.push(self.pcr_pid);
        }
        for &p in &self.es_pids {
            if !v.contains(&p) {
                v.push(p);
            }
        }
        v
    }
}

#[derive(Default)]
pub struct PsiCollector {
    sections: HashMap<u16, Vec<u8>>,
    pat: BTreeMap<u16, u16>,
    pmts: BTreeMap<u16, Program>,
}

impl PsiCollector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed 188-byte-aligned TS data.
    pub fn feed(&mut self, data: &[u8]) {
        for p in data.as_chunks::<188>().0 {
            if p[0] != 0x47 || p[1] & 0x80 != 0 {
                continue;
            }
            let pid = ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
            let afc = (p[3] >> 4) & 3;
            if afc & 1 == 0 {
                continue;
            }
            let wanted = pid == 0 || self.pat.values().any(|&pmt| pmt == pid);
            if !wanted {
                continue;
            }
            let mut off = 4;
            if afc & 2 != 0 {
                off += 1 + p[4] as usize;
            }
            if off >= 188 {
                continue;
            }
            let payload = &p[off..];
            let pusi = p[1] & 0x40 != 0;
            let buf = self.sections.entry(pid).or_default();
            if pusi {
                let ptr = payload[0] as usize;
                if !buf.is_empty() {
                    let done = std::mem::take(buf);
                    self.handle_section(pid, &done);
                }
                if ptr < payload.len() {
                    self.sections.entry(pid).or_default().extend_from_slice(&payload[1 + ptr..]);
                }
            } else if !buf.is_empty() {
                buf.extend_from_slice(payload);
            }
            // Complete sections can be handled as soon as their length is known.
            let buf = self.sections.entry(pid).or_default();
            if buf.len() >= 3 {
                let slen = (((buf[1] & 0x0F) as usize) << 8) | buf[2] as usize;
                if buf.len() >= 3 + slen {
                    let done = std::mem::take(buf);
                    self.handle_section(pid, &done[..3 + slen]);
                }
            }
        }
    }

    fn handle_section(&mut self, pid: u16, sec: &[u8]) {
        if sec.len() < 12 {
            return;
        }
        let tid = sec[0];
        let slen = (((sec[1] & 0x0F) as usize) << 8) | sec[2] as usize;
        if sec.len() < 3 + slen || slen < 9 {
            return;
        }
        let body = &sec[8..3 + slen - 4];
        if pid == 0 && tid == 0 {
            for e in body.as_chunks::<4>().0 {
                let prog = ((e[0] as u16) << 8) | e[1] as u16;
                let ppid = ((e[2] as u16 & 0x1F) << 8) | e[3] as u16;
                if prog != 0 {
                    self.pat.insert(prog, ppid);
                }
            }
        } else if tid == 2 && self.pat.values().any(|&pmt| pmt == pid) {
            let number = ((sec[3] as u16) << 8) | sec[4] as u16;
            let pcr_pid = ((sec[8] as u16 & 0x1F) << 8) | sec[9] as u16;
            let pilen = (((sec[10] & 0x0F) as usize) << 8) | sec[11] as usize;
            let end = 3 + slen - 4;
            if 12 + pilen > end {
                return;
            }
            let mut prog = Program { number, pmt_pid: pid, pcr_pid, ..Default::default() };
            collect_ca_pids(&sec[12..12 + pilen], &mut prog.ecm_pids);
            let mut j = 12 + pilen;
            while j + 5 <= end {
                let epid = ((sec[j + 1] as u16 & 0x1F) << 8) | sec[j + 2] as u16;
                let eilen = (((sec[j + 3] & 0x0F) as usize) << 8) | sec[j + 4] as usize;
                prog.es_pids.push(epid);
                if j + 5 + eilen <= end {
                    collect_ca_pids(&sec[j + 5..j + 5 + eilen], &mut prog.ecm_pids);
                }
                j += 5 + eilen;
            }
            self.pmts.insert(number, prog);
        }
    }

    /// All programs of the PAT have a parsed PMT.
    pub fn complete(&self) -> bool {
        !self.pat.is_empty() && self.pat.keys().all(|n| self.pmts.contains_key(n))
    }

    pub fn programs(&self) -> Vec<Program> {
        self.pmts.values().cloned().collect()
    }
}

fn collect_ca_pids(descriptors: &[u8], out: &mut Vec<u16>) {
    let mut i = 0;
    while i + 2 <= descriptors.len() {
        let tag = descriptors[i];
        let len = descriptors[i + 1] as usize;
        if tag == 0x09 && len >= 4 && i + 2 + len <= descriptors.len() {
            let pid = ((descriptors[i + 4] as u16 & 0x1F) << 8) | descriptors[i + 5] as u16;
            if pid != 0x1FFF && !out.contains(&pid) {
                out.push(pid);
            }
        }
        i += 2 + len;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(pid: u16, pusi: bool, payload: &[u8]) -> Vec<u8> {
        let mut p = vec![0xFFu8; 188];
        p[0] = 0x47;
        p[1] = ((pid >> 8) as u8 & 0x1F) | if pusi { 0x40 } else { 0 };
        p[2] = pid as u8;
        p[3] = 0x10;
        p[4] = 0; // pointer
        p[5..5 + payload.len()].copy_from_slice(payload);
        p
    }

    fn with_crc(mut sec: Vec<u8>) -> Vec<u8> {
        let len = sec.len() - 3 + 4;
        sec[1] = 0xB0 | ((len >> 8) as u8 & 0x0F);
        sec[2] = len as u8;
        sec.extend_from_slice(&[0, 0, 0, 0]);
        sec
    }

    #[test]
    fn parses_pat_and_pmt() {
        let pat = with_crc(vec![0x00, 0, 0, 0x7F, 0xE0, 0xC1, 0, 0, 0x04, 0x00, 0xE1, 0xF0]);
        let pmt = with_crc(vec![
            0x02, 0, 0, 0x04, 0x00, 0xC1, 0, 0, 0xE1, 0xFF, 0xF0, 0x06, 0x09, 0x04, 0x00, 0x05, 0xE9, 0x01, 0x02, 0xE1,
            0x00, 0xF0, 0x00, 0x0F, 0xE1, 0x10, 0xF0, 0x00,
        ]);
        let mut c = PsiCollector::new();
        c.feed(&packet(0, true, &pat));
        c.feed(&packet(0x1F0, true, &pmt));
        assert!(c.complete());
        let p = &c.programs()[0];
        assert_eq!(p.number, 0x400);
        assert_eq!(p.pcr_pid, 0x1FF);
        assert_eq!(p.ecm_pids, vec![0x901]);
        assert_eq!(p.es_pids, vec![0x100, 0x110]);
        assert_eq!(p.pid_list(), vec![0x1F0, 0x901, 0x1FF, 0x100, 0x110]);
    }
}
