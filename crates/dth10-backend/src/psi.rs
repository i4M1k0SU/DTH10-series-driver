//! Minimal PAT/PMT collector, used to learn a channel's programs and PIDs
//! for the device-side descrambling mode (program PID list, 0xCBDA/0x1000).

use std::collections::{BTreeMap, HashMap};

use dth_common::ts::{CA_DESCRIPTOR_TAG, NULL_PID, PAT_PID, PAT_TABLE_ID, PMT_TABLE_ID, SYNC_BYTE, TS_PACKET};

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
        if self.pcr_pid != NULL_PID && !v.contains(&self.pcr_pid) {
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

/// Collects the PAT and PMTs of one TS.
///
/// Right after a tune the stream can still carry packets of the TS selected
/// before (in the device or the bulk pipeline), so the PAT is tied to its
/// `transport_stream_id`: with an expected TSID, PATs of any other TSID are
/// ignored; without one, a PAT of a TSID other than the one held replaces it
/// and drops the PMTs collected so far (the newer TS wins). A PMT counts only
/// when the current PAT lists its program on the PID it arrived on.
#[derive(Default)]
pub struct PsiCollector {
    sections: HashMap<u16, Vec<u8>>,
    expected_tsid: Option<u16>,
    /// `transport_stream_id` of the PAT held in `pat`.
    tsid: Option<u16>,
    pat: BTreeMap<u16, u16>,
    pmts: BTreeMap<u16, Program>,
    stale_pats: u32,
}

impl PsiCollector {
    /// `expected_tsid`: the TSID selected for this stream, when known (ISDB-S).
    pub fn new(expected_tsid: Option<u16>) -> Self {
        Self { expected_tsid, ..Self::default() }
    }

    /// `transport_stream_id` of the PAT the programs come from.
    pub fn tsid(&self) -> Option<u16> {
        self.tsid
    }

    /// PAT sections ignored or replaced because they carried another TSID.
    pub fn stale_pats(&self) -> u32 {
        self.stale_pats
    }

    /// Feed 188-byte-aligned TS data.
    pub fn feed(&mut self, data: &[u8]) {
        for p in data.as_chunks::<TS_PACKET>().0 {
            if p[0] != SYNC_BYTE || p[1] & 0x80 != 0 {
                continue;
            }
            let pid = ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
            let afc = (p[3] >> 4) & 3;
            if afc & 1 == 0 {
                continue;
            }
            let wanted = pid == PAT_PID || self.pat.values().any(|&pmt| pmt == pid);
            if !wanted {
                continue;
            }
            let mut off = 4;
            if afc & 2 != 0 {
                off += 1 + p[4] as usize;
            }
            if off >= TS_PACKET {
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
        if pid == PAT_PID && tid == PAT_TABLE_ID {
            let tsid = ((sec[3] as u16) << 8) | sec[4] as u16;
            if self.expected_tsid.is_some_and(|want| want != tsid) {
                self.stale_pats += 1;
                return;
            }
            if self.tsid != Some(tsid) {
                if self.tsid.is_some() {
                    // Another TS took over: what was learned belongs to the old one.
                    self.stale_pats += 1;
                    self.pat.clear();
                    self.pmts.clear();
                    self.sections.retain(|&p, _| p == PAT_PID);
                }
                self.tsid = Some(tsid);
            }
            for e in body.as_chunks::<4>().0 {
                let prog = ((e[0] as u16) << 8) | e[1] as u16;
                let ppid = ((e[2] as u16 & 0x1F) << 8) | e[3] as u16;
                if prog != 0 {
                    self.pat.insert(prog, ppid);
                }
            }
        } else if tid == PMT_TABLE_ID {
            let number = ((sec[3] as u16) << 8) | sec[4] as u16;
            if self.pat.get(&number) != Some(&pid) {
                // A program of another TS (stale packets on a shared PMT PID).
                return;
            }
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

    /// All programs of the current PAT have a parsed PMT.
    pub fn complete(&self) -> bool {
        !self.pat.is_empty() && self.pat.keys().all(|n| self.pmts.contains_key(n))
    }

    /// The programs of the current PAT that have a parsed PMT.
    pub fn programs(&self) -> Vec<Program> {
        self.pmts.values().filter(|p| self.pat.get(&p.number) == Some(&p.pmt_pid)).cloned().collect()
    }
}

fn collect_ca_pids(descriptors: &[u8], out: &mut Vec<u16>) {
    let mut i = 0;
    while i + 2 <= descriptors.len() {
        let tag = descriptors[i];
        let len = descriptors[i + 1] as usize;
        if tag == CA_DESCRIPTOR_TAG && len >= 4 && i + 2 + len <= descriptors.len() {
            let pid = ((descriptors[i + 4] as u16 & 0x1F) << 8) | descriptors[i + 5] as u16;
            if pid != NULL_PID && !out.contains(&pid) {
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
        let mut c = PsiCollector::new(None);
        c.feed(&packet(0, true, &pat));
        c.feed(&packet(0x1F0, true, &pmt));
        assert!(c.complete());
        assert_eq!(c.tsid(), Some(0x7FE0));
        let p = &c.programs()[0];
        assert_eq!(p.number, 0x400);
        assert_eq!(p.pcr_pid, 0x1FF);
        assert_eq!(p.ecm_pids, vec![0x901]);
        assert_eq!(p.es_pids, vec![0x100, 0x110]);
        assert_eq!(p.pid_list(), vec![0x1F0, 0x901, 0x1FF, 0x100, 0x110]);
    }

    /// PAT section of `tsid` listing `(program, PMT PID)` pairs.
    fn pat(tsid: u16, programs: &[(u16, u16)]) -> Vec<u8> {
        let mut sec = vec![PAT_TABLE_ID, 0, 0, (tsid >> 8) as u8, tsid as u8, 0xC1, 0, 0];
        for &(prog, pmt_pid) in programs {
            sec.extend_from_slice(&[(prog >> 8) as u8, prog as u8, 0xE0 | (pmt_pid >> 8) as u8, pmt_pid as u8]);
        }
        with_crc(sec)
    }

    /// PMT section of `prog` with one elementary stream `es_pid`, also the PCR PID.
    fn pmt(prog: u16, es_pid: u16) -> Vec<u8> {
        let (hi, lo) = (0xE0 | (es_pid >> 8) as u8, es_pid as u8);
        let mut sec = vec![PMT_TABLE_ID, 0, 0, (prog >> 8) as u8, prog as u8, 0xC1, 0, 0, hi, lo, 0xF0, 0x00];
        sec.extend_from_slice(&[0x02, hi, lo, 0xF0, 0x00]);
        with_crc(sec)
    }

    fn numbers(c: &PsiCollector) -> Vec<u16> {
        c.programs().iter().map(|p| p.number).collect()
    }

    #[test]
    fn ignores_pat_of_another_tsid() {
        let mut c = PsiCollector::new(Some(0x4012));
        // The TS selected before is still in the pipeline.
        c.feed(&packet(PAT_PID, true, &pat(0x4011, &[(161, 0x1F0), (162, 0x1F1)])));
        c.feed(&packet(0x1F0, true, &pmt(161, 0x100)));
        assert!(!c.complete());
        assert!(c.programs().is_empty());
        assert_eq!(c.tsid(), None);
        c.feed(&packet(PAT_PID, true, &pat(0x4012, &[(171, 0x1F0)])));
        c.feed(&packet(0x1F1, true, &pmt(162, 0x110)));
        c.feed(&packet(0x1F0, true, &pmt(171, 0x120)));
        assert!(c.complete());
        assert_eq!(c.tsid(), Some(0x4012));
        assert_eq!(numbers(&c), vec![171]);
        assert_eq!(c.programs()[0].es_pids, vec![0x120]);
        // Late packets of the old TS change nothing.
        c.feed(&packet(PAT_PID, true, &pat(0x4011, &[(161, 0x1F0)])));
        c.feed(&packet(0x1F0, true, &pmt(161, 0x100)));
        assert_eq!(numbers(&c), vec![171]);
        assert_eq!(c.stale_pats(), 2);
    }

    #[test]
    fn ignores_stale_pmt_on_a_shared_pmt_pid() {
        let mut c = PsiCollector::new(Some(0x4012));
        c.feed(&packet(PAT_PID, true, &pat(0x4012, &[(171, 0x1F0), (172, 0x1F1)])));
        // PMTs of the old TS's programs on the PIDs the new PAT uses.
        c.feed(&packet(0x1F0, true, &pmt(161, 0x100)));
        c.feed(&packet(0x1F1, true, &pmt(162, 0x110)));
        assert!(!c.complete());
        assert!(c.programs().is_empty());
        c.feed(&packet(0x1F0, true, &pmt(171, 0x120)));
        assert!(!c.complete());
        // A program of the current PAT on the wrong PMT PID does not count either.
        c.feed(&packet(0x1F0, true, &pmt(172, 0x130)));
        assert!(!c.complete());
        c.feed(&packet(0x1F1, true, &pmt(172, 0x130)));
        assert!(c.complete());
        assert_eq!(numbers(&c), vec![171, 172]);
    }

    #[test]
    fn newer_tsid_replaces_the_pat_without_an_expected_one() {
        // ISDB-T: no TSID is selected, so the TS seen last wins.
        let mut c = PsiCollector::new(None);
        c.feed(&packet(PAT_PID, true, &pat(0x7FE0, &[(0x400, 0x1F0), (0x401, 0x1F1)])));
        c.feed(&packet(0x1F0, true, &pmt(0x400, 0x100)));
        assert_eq!(numbers(&c), vec![0x400]);
        c.feed(&packet(PAT_PID, true, &pat(0x7FE1, &[(0x408, 0x1F0)])));
        assert_eq!(c.tsid(), Some(0x7FE1));
        assert!(!c.complete());
        assert!(c.programs().is_empty());
        c.feed(&packet(0x1F1, true, &pmt(0x401, 0x110)));
        c.feed(&packet(0x1F0, true, &pmt(0x400, 0x100)));
        assert!(!c.complete());
        c.feed(&packet(0x1F0, true, &pmt(0x408, 0x180)));
        assert!(c.complete());
        assert_eq!(numbers(&c), vec![0x408]);
        // The same TSID again keeps what was learned.
        c.feed(&packet(PAT_PID, true, &pat(0x7FE1, &[(0x408, 0x1F0)])));
        assert!(c.complete());
        assert_eq!(c.stale_pats(), 1);
    }
}
