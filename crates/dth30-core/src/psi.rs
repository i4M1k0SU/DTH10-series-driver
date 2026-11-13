//! Just enough PSI to open a Program stream: PAT and PMT sections, their
//! assembly from TS packets of the Raw stream, and the [`ProgramSpec`] built
//! from them (docs/26 6-stream-payloads.md §4.1, 5-usermode.md §3.3).
//!
//! The vendor's filter builds the same record from the PAT and PMT: the
//! service ID, the PMT PID, the PCR PID, the CA_PID of the program-level CA
//! descriptor (tag 0x09) as the ECM PID, and the elementary streams with
//! their stream types.
//!
//! Self-contained on purpose (this crate does not depend on the DTH10
//! tuner crate).

use crate::vixs::stream::{self, EsEntry};
use crate::{Error, Result};

/// PID of the PAT.
pub const PAT_PID: u16 = 0x0000;
/// TS packet sizes the Raw stream may carry (204 on TOSHIBA kits, see
/// [`crate::model::KitKind::ts_packet_204`]).
pub const PACKET_188: usize = 188;
pub const PACKET_204: usize = 204;
/// Descriptor tag of the CA descriptor.
pub const CA_DESCRIPTOR: u8 = 0x09;

/// CRC-32 of MPEG-2 sections (polynomial 0x04C11DB7, no reflection, initial
/// value all ones). A section including its CRC sums to 0.
pub fn crc32_mpeg2(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= u32::from(b) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 { crc << 1 ^ 0x04C1_1DB7 } else { crc << 1 };
        }
    }
    crc
}

/// Check the long-form section header of `table_id` and the CRC; return the
/// section without the CRC.
fn long_section(sec: &[u8], table_id: u8) -> Result<&[u8]> {
    if sec.len() < 12 {
        return Err(Error::Protocol("PSI section too short"));
    }
    if sec[0] != table_id {
        return Err(Error::Protocol("unexpected PSI table ID"));
    }
    if sec[1] & 0x80 == 0 {
        return Err(Error::Protocol("PSI section without syntax indicator"));
    }
    let len = 3 + (usize::from(sec[1] & 0x0F) << 8 | usize::from(sec[2]));
    if len < 12 || len > sec.len() {
        return Err(Error::Protocol("PSI section length out of range"));
    }
    if crc32_mpeg2(&sec[..len]) != 0 {
        return Err(Error::Protocol("PSI section CRC mismatch"));
    }
    Ok(&sec[..len - 4])
}

fn pid13(hi: u8, lo: u8) -> u16 {
    u16::from(hi & 0x1F) << 8 | u16::from(lo)
}

fn len12(hi: u8, lo: u8) -> usize {
    usize::from(hi & 0x0F) << 8 | usize::from(lo)
}

/// One PAT entry. Program number 0 points at the NIT.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PatEntry {
    pub program_number: u16,
    pub pid: u16,
}

/// The entries of a PAT section (table ID 0x00, CRC checked).
pub fn parse_pat(sec: &[u8]) -> Result<Vec<PatEntry>> {
    let s = long_section(sec, 0x00)?;
    let (entries, _) = s[8..].as_chunks::<4>();
    Ok(entries
        .iter()
        .map(|c| PatEntry { program_number: u16::from_be_bytes([c[0], c[1]]), pid: pid13(c[2], c[3]) })
        .collect())
}

/// The PMT PID of service `sid`, or of the first service when `sid` is
/// `None` (the vendor's filter takes the first one unless told otherwise).
pub fn select_service(pat: &[PatEntry], sid: Option<u16>) -> Option<PatEntry> {
    pat.iter().copied().filter(|e| e.program_number != 0).find(|e| sid.is_none_or(|s| s == e.program_number))
}

/// A CA descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaDescriptor {
    pub ca_system_id: u16,
    pub ca_pid: u16,
}

/// The CA descriptors in a descriptor loop (malformed tails are ignored).
fn ca_descriptors(mut d: &[u8]) -> Vec<CaDescriptor> {
    let mut v = Vec::new();
    while d.len() >= 2 {
        let (tag, len) = (d[0], usize::from(d[1]));
        if 2 + len > d.len() {
            break;
        }
        if tag == CA_DESCRIPTOR && len >= 4 {
            v.push(CaDescriptor { ca_system_id: u16::from_be_bytes([d[2], d[3]]), ca_pid: pid13(d[4], d[5]) });
        }
        d = &d[2 + len..];
    }
    v
}

/// One elementary stream of a PMT.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PmtStream {
    pub stream_type: u8,
    pub pid: u16,
    /// CA descriptors in the stream's own descriptor loop.
    pub ca: Vec<CaDescriptor>,
}

/// A PMT section.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pmt {
    pub program_number: u16,
    pub pcr_pid: u16,
    /// Program-level CA descriptors.
    pub ca: Vec<CaDescriptor>,
    pub streams: Vec<PmtStream>,
}

/// Parse a PMT section (table ID 0x02, CRC checked).
pub fn parse_pmt(sec: &[u8]) -> Result<Pmt> {
    let s = long_section(sec, 0x02)?;
    let program_number = u16::from_be_bytes([s[3], s[4]]);
    let pcr_pid = pid13(s[8], s[9]);
    let pil = len12(s[10], s[11]);
    let mut p = 12 + pil;
    if p > s.len() {
        return Err(Error::Protocol("PMT program info runs past the section"));
    }
    let ca = ca_descriptors(&s[12..p]);
    let mut streams = Vec::new();
    while p + 5 <= s.len() {
        let eil = len12(s[p + 3], s[p + 4]);
        let end = p + 5 + eil;
        if end > s.len() {
            return Err(Error::Protocol("PMT stream info runs past the section"));
        }
        streams.push(PmtStream {
            stream_type: s[p],
            pid: pid13(s[p + 1], s[p + 2]),
            ca: ca_descriptors(&s[p + 5..end]),
        });
        p = end;
    }
    Ok(Pmt { program_number, pcr_pid, ca, streams })
}

/// One elementary stream of a [`ProgramSpec`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EsSpec {
    pub pid: u16,
    pub stream_type: u8,
    /// CA_PID of the stream's own CA descriptor, if any.
    pub ecm_pid: Option<u16>,
}

/// The service a Program stream is opened for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProgramSpec {
    pub sid: u16,
    pub pmt_pid: u16,
    pub pcr_pid: u16,
    /// CA_PID of the first program-level CA descriptor (the ECM PID).
    pub ecm_pid: Option<u16>,
    pub es: Vec<EsSpec>,
}

impl ProgramSpec {
    /// The record for the PMT of a service found at `pmt_pid`.
    pub fn from_pmt(pmt_pid: u16, pmt: &Pmt) -> Self {
        Self {
            sid: pmt.program_number,
            pmt_pid,
            pcr_pid: pmt.pcr_pid,
            ecm_pid: pmt.ca.first().map(|c| c.ca_pid),
            es: pmt
                .streams
                .iter()
                .map(|s| EsSpec { pid: s.pid, stream_type: s.stream_type, ecm_pid: s.ca.first().map(|c| c.ca_pid) })
                .collect(),
        }
    }

    /// From a PAT section and the PMT section of the chosen service; fails
    /// when the PMT is not the one the PAT names for `sid` (or for the first
    /// service when `sid` is `None`).
    pub fn from_sections(pat: &[u8], pmt: &[u8], sid: Option<u16>) -> Result<Self> {
        let entry = select_service(&parse_pat(pat)?, sid).ok_or(Error::Protocol("service not in the PAT"))?;
        let pmt = parse_pmt(pmt)?;
        if pmt.program_number != entry.program_number {
            return Err(Error::Protocol("PMT of another service"));
        }
        Ok(Self::from_pmt(entry.pid, &pmt))
    }

    /// The streams as the Program open takes them: cut at the first PID 0
    /// and at 24 streams, each with the program-level ECM PID as the
    /// vendor's filter sends it, or the stream's own one when the program
    /// has none, or 0.
    pub fn es_entries(&self) -> Vec<EsEntry> {
        let v: Vec<EsEntry> = self
            .es
            .iter()
            .map(|e| EsEntry {
                pid: e.pid,
                stream_type: e.stream_type,
                ecm_pid: self.ecm_pid.or(e.ecm_pid).unwrap_or(0),
            })
            .collect();
        let n = stream::limit_es(&v).len();
        v[..n].to_vec()
    }
}

/// The packet size and the offset of the first sync byte: at least three
/// (up to four) sync bytes have to line up in `data` (188 is tried first).
pub fn detect_packet_size(data: &[u8]) -> Option<(usize, usize)> {
    for size in [PACKET_188, PACKET_204] {
        for off in 0..size {
            let n = data.len().saturating_sub(off).div_ceil(size).min(4);
            if n >= 3 && (0..n).all(|i| data[off + i * size] == 0x47) {
                return Some((off, size));
            }
        }
    }
    None
}

/// Assembles the sections of one PID from TS packets.
#[derive(Debug, Clone)]
pub struct SectionCollector {
    pid: u16,
    buf: Vec<u8>,
    active: bool,
    cc: Option<u8>,
}

impl SectionCollector {
    pub fn new(pid: u16) -> Self {
        Self { pid, buf: Vec::new(), active: false, cc: None }
    }

    pub fn pid(&self) -> u16 {
        self.pid
    }

    fn reset(&mut self) {
        self.buf.clear();
        self.active = false;
    }

    /// Feed one TS packet (188 bytes or more; bytes past 188, such as RS
    /// parity, are ignored). Returns the sections it completed. Packets of
    /// other PIDs, scrambled ones and duplicates are skipped; a continuity
    /// gap drops the partial section.
    pub fn push(&mut self, pkt: &[u8]) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        if pkt.len() < PACKET_188 || pkt[0] != 0x47 || pid13(pkt[1], pkt[2]) != self.pid {
            return out;
        }
        if pkt[1] & 0x80 != 0 || pkt[3] & 0xC0 != 0 {
            self.reset();
            return out;
        }
        let afc = pkt[3] >> 4 & 3;
        if afc & 1 == 0 {
            return out;
        }
        let cc = pkt[3] & 0x0F;
        match self.cc {
            Some(prev) if prev == cc => return out,
            Some(prev) if (prev + 1) & 0x0F != cc => self.reset(),
            _ => {}
        }
        self.cc = Some(cc);
        let start = if afc & 2 != 0 { 5 + usize::from(pkt[4]) } else { 4 };
        if start >= PACKET_188 {
            return out;
        }
        let payload = &pkt[start..PACKET_188];
        if pkt[1] & 0x40 != 0 {
            let ptr = usize::from(payload[0]);
            if 1 + ptr > payload.len() {
                self.reset();
                return out;
            }
            if self.active {
                self.buf.extend_from_slice(&payload[1..1 + ptr]);
                self.drain(&mut out);
            }
            self.buf.clear();
            self.buf.extend_from_slice(&payload[1 + ptr..]);
            self.active = true;
        } else if self.active {
            self.buf.extend_from_slice(payload);
        }
        self.drain(&mut out);
        out
    }

    fn drain(&mut self, out: &mut Vec<Vec<u8>>) {
        while self.active {
            if self.buf.first() == Some(&0xFF) {
                // Stuffing up to the end of the packet.
                self.reset();
                break;
            }
            if self.buf.len() < 3 {
                break;
            }
            let len = 3 + len12(self.buf[1], self.buf[2]);
            if self.buf.len() < len {
                break;
            }
            out.push(self.buf.drain(..len).collect());
            if self.buf.is_empty() {
                // The next section starts with a new payload unit.
                self.active = false;
            }
        }
    }
}

/// Finds a service's [`ProgramSpec`] in the Raw stream: feed it the data
/// as read from the endpoint until [`ProgramScanner::push`] returns the
/// record. Packets may be split across reads.
#[derive(Debug, Clone)]
pub struct ProgramScanner {
    sid: Option<u16>,
    packet: Option<usize>,
    carry: Vec<u8>,
    pat: SectionCollector,
    pmt: Option<SectionCollector>,
}

impl ProgramScanner {
    /// Look for service `sid`, or the first service of the PAT.
    pub fn new(sid: Option<u16>) -> Self {
        Self { sid, packet: None, carry: Vec::new(), pat: SectionCollector::new(PAT_PID), pmt: None }
    }

    /// The packet size found, once known.
    pub fn packet_size(&self) -> Option<usize> {
        self.packet
    }

    /// Feed the next bytes of the Raw stream; returns the record once the
    /// service's PMT has been seen.
    pub fn push(&mut self, data: &[u8]) -> Option<ProgramSpec> {
        self.carry.extend_from_slice(data);
        loop {
            let size = match self.packet {
                Some(s) => s,
                None => match detect_packet_size(&self.carry) {
                    Some((off, s)) => {
                        self.carry.drain(..off);
                        self.packet = Some(s);
                        s
                    }
                    None => {
                        // Keep only what a later detection can still use.
                        let keep = 4 * PACKET_204;
                        if self.carry.len() > keep {
                            self.carry.drain(..self.carry.len() - keep);
                        }
                        return None;
                    }
                },
            };
            let mut pos = 0;
            while pos + size <= self.carry.len() && self.carry[pos] == 0x47 {
                let pkt = self.carry[pos..pos + size].to_vec();
                pos += size;
                if let Some(p) = self.packet_in(&pkt) {
                    self.carry.drain(..pos);
                    return Some(p);
                }
            }
            if pos + size <= self.carry.len() {
                // Lost sync: look for it again past this byte.
                self.packet = None;
                self.carry.drain(..pos + 1);
                continue;
            }
            self.carry.drain(..pos);
            return None;
        }
    }

    fn packet_in(&mut self, pkt: &[u8]) -> Option<ProgramSpec> {
        for sec in self.pat.push(pkt) {
            let Ok(pat) = parse_pat(&sec) else { continue };
            if let Some(e) = select_service(&pat, self.sid)
                && self.pmt.as_ref().is_none_or(|c| c.pid() != e.pid)
            {
                self.pmt = Some(SectionCollector::new(e.pid));
            }
        }
        let want = self.sid;
        let c = self.pmt.as_mut()?;
        let pmt_pid = c.pid();
        for sec in c.push(pkt) {
            let Ok(pmt) = parse_pmt(&sec) else { continue };
            if want.is_none_or(|s| s == pmt.program_number) {
                return Some(ProgramSpec::from_pmt(pmt_pid, &pmt));
            }
        }
        None
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A long-form section with a valid CRC.
    pub(crate) fn section(table_id: u8, ext: u16, body: &[u8]) -> Vec<u8> {
        let len = 5 + body.len() + 4;
        let mut s = vec![table_id, 0xB0 | (len >> 8) as u8, len as u8];
        s.extend_from_slice(&ext.to_be_bytes());
        s.extend_from_slice(&[0xC1, 0, 0]);
        s.extend_from_slice(body);
        let crc = crc32_mpeg2(&s);
        s.extend_from_slice(&crc.to_be_bytes());
        s
    }

    pub(crate) fn pat(entries: &[(u16, u16)]) -> Vec<u8> {
        let mut b = Vec::new();
        for &(n, pid) in entries {
            b.extend_from_slice(&n.to_be_bytes());
            b.extend_from_slice(&(0xE000 | pid).to_be_bytes());
        }
        section(0x00, 0x7FE0, &b)
    }

    fn ca(pid: u16) -> Vec<u8> {
        let mut d = vec![0x09, 4, 0x00, 0x05];
        d.extend_from_slice(&(0xE000 | pid).to_be_bytes());
        d
    }

    /// A PMT: (stream_type, pid, ES-level ECM PID).
    pub(crate) fn pmt(sid: u16, pcr: u16, ecm: Option<u16>, es: &[(u8, u16, Option<u16>)]) -> Vec<u8> {
        let mut b = (0xE000 | pcr).to_be_bytes().to_vec();
        // An unrelated descriptor before the CA descriptor.
        let mut info = vec![0xC1, 1, 0x84];
        if let Some(e) = ecm {
            info.extend(ca(e));
        }
        b.extend_from_slice(&(0xF000 | info.len() as u16).to_be_bytes());
        b.extend(info);
        for &(st, pid, e) in es {
            let d = e.map_or_default(ca);
            b.push(st);
            b.extend_from_slice(&(0xE000 | pid).to_be_bytes());
            b.extend_from_slice(&(0xF000 | d.len() as u16).to_be_bytes());
            b.extend(d);
        }
        section(0x02, sid, &b)
    }

    /// TS packets of `size` carrying `sec` on `pid`, starting at `cc`.
    pub(crate) fn packets(pid: u16, sec: &[u8], cc: u8, size: usize) -> Vec<u8> {
        let mut out = Vec::new();
        let mut data = vec![0u8];
        data.extend_from_slice(sec);
        for (i, chunk) in data.chunks(184).enumerate() {
            let mut p = vec![0x47, (pid >> 8) as u8 | if i == 0 { 0x40 } else { 0 }, pid as u8];
            p.push(0x10 | ((cc + i as u8) & 0x0F));
            p.extend_from_slice(chunk);
            p.resize(188, 0xFF);
            p.resize(size, 0xAA);
            out.extend(p);
        }
        out
    }

    #[test]
    fn crc() {
        assert_eq!(crc32_mpeg2(b"123456789"), 0x0376_E6E7);
        let s = pat(&[(0, 0x10)]);
        assert_eq!(crc32_mpeg2(&s), 0);
    }

    #[test]
    fn pat_parsing() {
        let s = pat(&[(0, 0x10), (0x0408, 0x1F0), (0x0409, 0x1F1)]);
        let p = parse_pat(&s).unwrap();
        assert_eq!(p.len(), 3);
        assert_eq!(p[1], PatEntry { program_number: 0x408, pid: 0x1F0 });
        assert_eq!(select_service(&p, None), Some(PatEntry { program_number: 0x408, pid: 0x1F0 }));
        assert_eq!(select_service(&p, Some(0x409)).unwrap().pid, 0x1F1);
        assert_eq!(select_service(&p, Some(0x40A)), None);
        let mut bad = s.clone();
        bad[9] ^= 1;
        assert!(parse_pat(&bad).is_err());
        assert!(parse_pat(&pmt(1, 0x100, None, &[])).is_err());
        assert!(parse_pat(&s[..10]).is_err());
    }

    #[test]
    fn pmt_parsing() {
        let s =
            pmt(0x408, 0x1FF, Some(0x1F01), &[(0x02, 0x100, None), (0x0F, 0x110, Some(0x1F02)), (0x06, 0x130, None)]);
        let p = parse_pmt(&s).unwrap();
        assert_eq!(p.program_number, 0x408);
        assert_eq!(p.pcr_pid, 0x1FF);
        assert_eq!(p.ca, [CaDescriptor { ca_system_id: 5, ca_pid: 0x1F01 }]);
        assert_eq!(p.streams.len(), 3);
        assert_eq!(p.streams[1].pid, 0x110);
        assert_eq!(p.streams[1].stream_type, 0x0F);
        assert_eq!(p.streams[1].ca[0].ca_pid, 0x1F02);
        assert!(p.streams[0].ca.is_empty());
    }

    #[test]
    fn program_spec_from_sections() {
        let pat = pat(&[(0, 0x10), (0x0408, 0x1F0)]);
        let pmt = pmt(0x408, 0x1FF, Some(0x1F01), &[(0x02, 0x100, None), (0x0F, 0x110, Some(0x1F02))]);
        let spec = ProgramSpec::from_sections(&pat, &pmt, None).unwrap();
        assert_eq!(
            spec,
            ProgramSpec {
                sid: 0x408,
                pmt_pid: 0x1F0,
                pcr_pid: 0x1FF,
                ecm_pid: Some(0x1F01),
                es: vec![
                    EsSpec { pid: 0x100, stream_type: 2, ecm_pid: None },
                    EsSpec { pid: 0x110, stream_type: 0x0F, ecm_pid: Some(0x1F02) },
                ],
            }
        );
        // The program-level ECM PID goes with every stream.
        let e = spec.es_entries();
        assert_eq!(e[0], EsEntry { pid: 0x100, stream_type: 2, ecm_pid: 0x1F01 });
        assert_eq!(e[1].ecm_pid, 0x1F01);
        // Without one, the stream's own, else 0.
        let s2 = ProgramSpec { ecm_pid: None, ..spec.clone() };
        assert_eq!(s2.es_entries().iter().map(|e| e.ecm_pid).collect::<Vec<_>>(), [0, 0x1F02]);
        assert!(ProgramSpec::from_sections(&pat, &pmt, Some(0x409)).is_err());
        let other = super::tests::pmt(0x409, 0x1FF, None, &[]);
        assert!(ProgramSpec::from_sections(&pat, &other, None).is_err());
    }

    #[test]
    fn sections_from_packets() {
        let long = pmt(0x408, 0x1FF, None, &(0..60).map(|i| (6u8, 0x200 + i, None)).collect::<Vec<_>>());
        assert!(long.len() > 184);
        let ts = packets(0x1F0, &long, 0xE, 188);
        let mut c = SectionCollector::new(0x1F0);
        let mut got = Vec::new();
        for p in ts.chunks(188) {
            got.extend(c.push(p));
        }
        assert_eq!(got, vec![long.clone()]);
        assert_eq!(parse_pmt(&got[0]).unwrap().streams.len(), 60);

        // A gap in the continuity counter drops the partial section.
        let pk: Vec<&[u8]> = ts.chunks(188).collect();
        assert_eq!(pk.len(), 2);
        let mut c = SectionCollector::new(0x1F0);
        assert!(c.push(pk[0]).is_empty());
        let mut gap = pk[1].to_vec();
        gap[3] = 0x10 | ((gap[3] + 1) & 0x0F);
        assert!(c.push(&gap).is_empty());
        // A duplicate is ignored, other PIDs too.
        let mut c = SectionCollector::new(0x1F0);
        assert!(c.push(pk[0]).is_empty());
        assert!(c.push(pk[0]).is_empty());
        assert!(c.push(&packets(0x1F1, &long, 0, 188)[..188]).is_empty());
        assert_eq!(c.push(pk[1]), vec![long.clone()]);
        // Scrambled packets are dropped.
        let mut c = SectionCollector::new(0x1F0);
        let mut scr = pk[0].to_vec();
        scr[3] |= 0x80;
        assert!(c.push(&scr).is_empty());
        assert!(c.push(pk[1]).is_empty());

        // Two short sections in one packet, then stuffing.
        let a = pat(&[(1, 0x100)]);
        let b = pat(&[(2, 0x101)]);
        let mut both = a.clone();
        both.extend(&b);
        let mut c = SectionCollector::new(0);
        assert_eq!(c.push(&packets(0, &both, 0, 188)), vec![a, b]);
    }

    #[test]
    fn packet_size_detection() {
        let ts = packets(0, &pat(&[(1, 0x100)]), 0, 204).repeat(4);
        assert_eq!(detect_packet_size(&ts), Some((0, 204)));
        let ts = packets(0, &pat(&[(1, 0x100)]), 0, 188).repeat(4);
        assert_eq!(detect_packet_size(&ts[5..]), Some((183, 188)));
        assert_eq!(detect_packet_size(&[0u8; 1000]), None);
    }

    #[test]
    fn scanner_finds_the_service() {
        for size in [188, 204] {
            let mut ts = packets(0x1FF, &[0; 200], 0, size); // unrelated
            ts.extend(packets(0, &pat(&[(0, 0x10), (0x408, 0x1F0), (0x409, 0x1F1)]), 0, size));
            ts.extend(packets(0x1F0, &pmt(0x408, 0x1FF, Some(0x1F01), &[(2, 0x100, None)]), 0, size));
            ts.extend(packets(0x1F1, &pmt(0x409, 0x1FE, None, &[(0x1B, 0x181, None)]), 0, size));
            for sid in [None, Some(0x409)] {
                let mut s = ProgramScanner::new(sid);
                // Fed in pieces that split packets, starting mid-packet.
                let mut found = None;
                for chunk in ts[7..].chunks(1000) {
                    if let Some(p) = s.push(chunk) {
                        found = Some(p);
                        break;
                    }
                }
                let p = found.unwrap_or_else(|| panic!("size {size}, sid {sid:?}"));
                assert_eq!(s.packet_size(), Some(size));
                match sid {
                    None => assert_eq!((p.sid, p.pmt_pid, p.ecm_pid), (0x408, 0x1F0, Some(0x1F01))),
                    Some(_) => assert_eq!((p.sid, p.pmt_pid, p.pcr_pid), (0x409, 0x1F1, 0x1FE)),
                }
            }
        }
    }
}
