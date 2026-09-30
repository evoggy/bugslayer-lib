// The capture stream: 512-byte self-describing blocks, identical on the FX2's
// EP6 and the RP2350's own bulk IN. Layout: rp2350/src/block.h in
// bugslayer-deck-firmware; semantics: docs/protocol.md.

pub const BLOCK_SIZE: usize = 512;
pub const HEADER_SIZE: usize = 28;
pub const PAYLOAD_MAX: usize = BLOCK_SIZE - HEADER_SIZE;
pub const MAGIC: u32 = 0x594C_5342; // "BSLY"

pub const SESSION: u8 = 0;
pub const SAMPLES: u8 = 1;
pub const OVERRUN: u8 = 2;
pub const END: u8 = 5;

pub const ENC_RAW16: u8 = 1;

/// GP16..31 in bit order, as wired on rev A.
pub const CHANNEL_NAMES: [&str; 16] = [
    "IO_1", "IO_2", "IO_3", "IO_4", "MISO", "OW", "SCK", "MOSI", "WKUP", "N_IO_1", "TX2", "RX2",
    "TX1", "RX1", "SDA", "SCL",
];

fn u16_at(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes(b[o..o + 2].try_into().unwrap())
}
fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}
fn u64_at(b: &[u8], o: usize) -> u64 {
    u64::from_le_bytes(b[o..o + 8].try_into().unwrap())
}
fn text(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

pub struct Block<'a> {
    pub magic: u32,
    pub kind: u8,
    pub stream: u8,
    pub session: u32,
    pub seq: u32,
    pub sample: u64,
    pub payload: &'a [u8],
}

impl<'a> Block<'a> {
    pub fn parse(raw: &'a [u8]) -> Block<'a> {
        let plen = (u16_at(raw, 24) as usize).min(PAYLOAD_MAX);
        Block {
            magic: u32_at(raw, 0),
            kind: raw[5],
            stream: raw[6],
            session: u32_at(raw, 8),
            seq: u32_at(raw, 12),
            sample: u64_at(raw, 16),
            payload: &raw[HEADER_SIZE..HEADER_SIZE + plen],
        }
    }
}

pub fn type_name(t: u8) -> String {
    match t {
        0 => "SESSION".into(),
        1 => "SAMPLES".into(),
        2 => "OVERRUN".into(),
        3 => "IDLE".into(),
        4 => "EVENT".into(),
        5 => "END".into(),
        n => format!("type {}", n),
    }
}

// Wire-format structs carry every field, used or not yet.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct StreamDesc {
    pub id: u8,
    pub encoding: u8,
    pub first_pin: u8,
    pub n_pins: u8,
    pub rate_num: u32,
    pub rate_den: u32,
    pub source: String,
}

impl StreamDesc {
    /// Samples per second; 0 for event streams (sck8).
    pub fn rate(&self) -> f64 {
        if self.rate_den == 0 {
            0.0
        } else {
            self.rate_num as f64 / self.rate_den as f64
        }
    }
    pub fn unit(&self) -> usize {
        if self.encoding == ENC_RAW16 {
            2
        } else {
            1
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct SessionInfo {
    pub timebase_hz: u32,
    pub arm_time_us: u64,
    pub fw: String,
    pub serial: String,
    pub hw: String,
    pub streams: Vec<StreamDesc>,
}

fn parse_session(p: &[u8]) -> Option<SessionInfo> {
    if p.len() < 64 {
        return None;
    }
    let n = p[56] as usize;
    let mut streams = Vec::new();
    for i in 0..n {
        let o = 64 + 20 * i;
        if o + 20 > p.len() {
            return None;
        }
        streams.push(StreamDesc {
            id: p[o],
            encoding: p[o + 1],
            first_pin: p[o + 2],
            n_pins: p[o + 3],
            rate_num: u32_at(p, o + 4),
            rate_den: u32_at(p, o + 8),
            source: text(&p[o + 12..o + 20]),
        });
    }
    Some(SessionInfo {
        timebase_hz: u32_at(p, 0),
        arm_time_us: u64_at(p, 8),
        fw: text(&p[16..32]),
        serial: text(&p[32..48]),
        hw: text(&p[48..56]),
        streams,
    })
}

#[derive(Debug, Clone)]
pub struct EndInfo {
    pub blocks: u32,
    pub overruns: u32,
    pub lost: u64,
    pub totals: Vec<u64>,
}

/// What a verified block means for the consumer.
#[allow(dead_code)]
pub enum Event<'a> {
    Session(&'a SessionInfo),
    Samples { stream: u8, first: u64, data: &'a [u8] },
    Overrun { stream: u8, first: u64, lost: u64 },
    End(&'a EndInfo),
}

pub struct StreamState {
    pub desc: StreamDesc,
    pub next_sample: u64,
    pub overruns: Vec<(u64, u64)>,
}

/// Checks one session's blocks as they arrive, per docs/protocol.md: SESSION
/// first, no sequence gaps, sample indices continuous except where an OVERRUN
/// says samples were lost, END totals matching what was received, and for the
/// synthetic counter source every sample equal to its own index.
pub struct Verifier {
    pub session: u32,
    pub info: Option<SessionInfo>,
    pub streams: Vec<StreamState>,
    pub end: Option<EndInfo>,
    pub blocks: u32,
    pub stale: u64,
    pub errors: Vec<String>,
    pub error_count: usize,
    next_seq: u32,
    counter_bad: u64,
}

impl Verifier {
    pub fn new(session: u32) -> Verifier {
        Verifier {
            session,
            info: None,
            streams: Vec::new(),
            end: None,
            blocks: 0,
            stale: 0,
            errors: Vec::new(),
            error_count: 0,
            next_seq: 0,
            counter_bad: 0,
        }
    }

    fn err(&mut self, msg: String) {
        self.error_count += 1;
        if self.errors.len() < 20 {
            self.errors.push(msg);
        }
    }

    pub fn lost(&self) -> u64 {
        self.streams.iter().flat_map(|s| &s.overruns).map(|o| o.1).sum()
    }
    pub fn overruns(&self) -> usize {
        self.streams.iter().map(|s| s.overruns.len()).sum()
    }

    /// A run of whole blocks.
    pub fn feed(&mut self, buf: &[u8], out: &mut impl FnMut(Event)) {
        for raw in buf.chunks_exact(BLOCK_SIZE) {
            self.feed_block(raw, out);
        }
    }

    fn feed_block(&mut self, raw: &[u8], out: &mut impl FnMut(Event)) {
        let b = Block::parse(raw);
        if b.magic != MAGIC {
            if self.blocks == 0 {
                // Residue from before this session: the stage 1 test, or an
                // aborted session's tail.
                self.stale += 1;
            } else {
                self.err(format!("bad magic 0x{:08x}", b.magic));
            }
            return;
        }
        if b.session != self.session {
            self.stale += 1; // an earlier session's tail; expected
            return;
        }
        self.blocks += 1;
        if b.seq != self.next_seq {
            self.err(format!("seq gap: expected {}, got {}", self.next_seq, b.seq));
        }
        self.next_seq = b.seq.wrapping_add(1);
        if self.blocks == 1 && b.kind != SESSION {
            self.err(format!("first block is {}, not SESSION", type_name(b.kind)));
        }

        match b.kind {
            SESSION => match parse_session(b.payload) {
                Some(info) => {
                    self.streams = info
                        .streams
                        .iter()
                        .map(|d| StreamState { desc: d.clone(), next_sample: 0, overruns: Vec::new() })
                        .collect();
                    self.info = Some(info);
                    out(Event::Session(self.info.as_ref().unwrap()));
                }
                None => self.err("malformed SESSION block".into()),
            },
            SAMPLES | OVERRUN => {
                let sid = b.stream as usize;
                if sid >= self.streams.len() {
                    self.err(format!("{} for unknown stream {}", type_name(b.kind), sid));
                    return;
                }
                let expected = self.streams[sid].next_sample;
                if b.sample != expected {
                    self.err(format!(
                        "stream {}: {} at {}, expected {}",
                        sid,
                        type_name(b.kind),
                        b.sample,
                        expected
                    ));
                }
                if b.kind == SAMPLES {
                    let st = &mut self.streams[sid];
                    let unit = st.desc.unit();
                    let data = &b.payload[..b.payload.len() / unit * unit];
                    st.next_sample = b.sample + (data.len() / unit) as u64;
                    if sid == 0 && st.desc.source == "counter" && unit == 2 {
                        for (i, s) in data.chunks_exact(2).enumerate() {
                            let v = u16::from_le_bytes([s[0], s[1]]);
                            if v != (b.sample + i as u64) as u16 {
                                self.counter_bad += 1;
                            }
                        }
                    }
                    out(Event::Samples { stream: b.stream, first: b.sample, data });
                } else {
                    let lost = if b.payload.len() >= 8 { u64_at(b.payload, 0) } else { 0 };
                    let st = &mut self.streams[sid];
                    st.overruns.push((b.sample, lost));
                    st.next_sample = b.sample + lost;
                    out(Event::Overrun { stream: b.stream, first: b.sample, lost });
                }
            }
            END => {
                let p = b.payload;
                if p.len() < 16 {
                    self.err("malformed END block".into());
                    return;
                }
                let mut totals = vec![b.sample];
                if p.len() >= 24 {
                    let n = p[16] as usize;
                    totals = (0..n).filter(|i| 24 + 8 * i + 8 <= p.len()).map(|i| u64_at(p, 24 + 8 * i)).collect();
                }
                let end = EndInfo { blocks: u32_at(p, 0), overruns: u32_at(p, 4), lost: u64_at(p, 8), totals };
                for (sid, st) in self.streams.iter().enumerate() {
                    if let Some(&t) = end.totals.get(sid) {
                        if t != st.next_sample {
                            let msg = format!("END says stream {} has {} samples, it reached {}", sid, t, st.next_sample);
                            self.errors.push(msg);
                            self.error_count += 1;
                        }
                    }
                }
                if end.blocks != self.blocks {
                    self.err(format!("END says {} blocks, received {}", end.blocks, self.blocks));
                }
                if end.overruns as usize != self.overruns() || end.lost != self.lost() {
                    self.err("END overrun totals do not match the OVERRUN blocks".into());
                }
                self.end = Some(end);
                out(Event::End(self.end.as_ref().unwrap()));
            }
            _ => {} // IDLE / EVENT: reserved
        }
    }

    /// Final checks once the stream is over.
    pub fn finish(&mut self) {
        if self.end.is_none() {
            self.err("no END block".into());
        }
        if self.counter_bad > 0 {
            let n = self.counter_bad;
            self.err(format!("{} samples do not match the synthetic counter", n));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block(kind: u8, stream: u8, session: u32, seq: u32, sample: u64, payload: &[u8]) -> Vec<u8> {
        let mut b = vec![0u8; BLOCK_SIZE];
        b[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        b[5] = kind;
        b[6] = stream;
        b[8..12].copy_from_slice(&session.to_le_bytes());
        b[12..16].copy_from_slice(&seq.to_le_bytes());
        b[16..24].copy_from_slice(&sample.to_le_bytes());
        b[24..26].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        b[28..28 + payload.len()].copy_from_slice(payload);
        b
    }

    fn session_payload() -> Vec<u8> {
        let mut p = vec![0u8; 84];
        p[0..4].copy_from_slice(&150_000_000u32.to_le_bytes());
        p[16..21].copy_from_slice(b"0.1.0");
        p[56] = 1;
        p[64] = 0;
        p[65] = ENC_RAW16;
        p[66] = 16;
        p[67] = 16;
        p[68..72].copy_from_slice(&1000u32.to_le_bytes());
        p[72..76].copy_from_slice(&1u32.to_le_bytes());
        p[76..83].copy_from_slice(b"counter");
        p
    }

    fn end_payload(blocks: u32, overruns: u32, lost: u64, total: u64) -> Vec<u8> {
        let mut p = vec![0u8; 32];
        p[0..4].copy_from_slice(&blocks.to_le_bytes());
        p[4..8].copy_from_slice(&overruns.to_le_bytes());
        p[8..16].copy_from_slice(&lost.to_le_bytes());
        p[16] = 1;
        p[24..32].copy_from_slice(&total.to_le_bytes());
        p
    }

    fn counter(first: u64, n: usize) -> Vec<u8> {
        (0..n).flat_map(|i| ((first + i as u64) as u16).to_le_bytes()).collect()
    }

    #[test]
    fn clean_session_passes() {
        let mut buf = vec![0xAAu8; BLOCK_SIZE]; // residue before the session
        buf.extend(block(SESSION, 0, 7, 0, 0, &session_payload()));
        buf.extend(block(SAMPLES, 0, 7, 1, 0, &counter(0, 242)));
        buf.extend(block(OVERRUN, 0, 7, 2, 242, &100u64.to_le_bytes()));
        buf.extend(block(SAMPLES, 0, 7, 3, 342, &counter(342, 242)));
        buf.extend(block(END, 0, 7, 4, 584, &end_payload(5, 1, 100, 584)));
        let mut v = Verifier::new(7);
        let mut samples = 0;
        v.feed(&buf, &mut |e| {
            if let Event::Samples { data, .. } = e {
                samples += data.len() / 2
            }
        });
        v.finish();
        assert!(v.errors.is_empty(), "{:?}", v.errors);
        assert_eq!(v.stale, 1);
        assert_eq!(samples, 484);
        assert_eq!(v.lost(), 100);
    }

    #[test]
    fn gap_and_bad_counter_fail() {
        let mut buf = Vec::new();
        buf.extend(block(SESSION, 0, 9, 0, 0, &session_payload()));
        buf.extend(block(0x42, 0, 1, 0, 0, &[])); // another session: stale
        let mut bad = counter(0, 242);
        bad[10] ^= 1;
        buf.extend(block(SAMPLES, 0, 9, 2, 0, &bad)); // seq 1 missing
        let mut v = Verifier::new(9);
        v.feed(&buf, &mut |_| {});
        v.finish();
        assert_eq!(v.stale, 1);
        assert!(v.errors.iter().any(|e| e.contains("seq gap")));
        assert!(v.errors.iter().any(|e| e.contains("synthetic counter")));
        assert!(v.errors.iter().any(|e| e.contains("no END")));
    }
}
