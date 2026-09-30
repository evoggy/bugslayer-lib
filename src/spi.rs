// SPI from the sck8 stream (stage 4 in bugslayer-deck-firmware). The raw16
// stream sits behind the Raw16 trait, so a front end that keeps only edges
// (bugslayer-ui) can time it too.
//
// The sck8 stream holds GP16-23 at every rising SCK edge: bits 0-3 IO_1..IO_4
// (the CS candidates), 4 MISO, 5 OW, 6 SCK, 7 MOSI. Bit 6 is 1 in every edge
// byte, so a byte with bit 6 clear is a marker the deck inserts when CS changed
// since the previous edge: that is what separates two transactions back to
// back on the same CS. The stream has exact bits but no time. The raw16 stream
// has time: its CS windows, in order, are the sck8 stream's transactions. When
// raw16 also resolves SCK (a few samples per SCK period) it counts each
// window's edges too, which gives an independent decode to check sck8 against.

use std::collections::BTreeMap;

use crate::stream::CHANNEL_NAMES;

const IO_MASK: u8 = 0x0F;
const MISO_BIT: u32 = 4;
const SCK_BIT: u32 = 6;
const MOSI_BIT: u32 = 7;

/// MSB-first bytes from bits; a trailing partial byte is dropped.
fn bits_to_bytes(bits: impl Iterator<Item = bool>) -> Vec<u8> {
    let bits: Vec<bool> = bits.collect();
    bits.chunks_exact(8).map(|c| c.iter().fold(0u8, |acc, &b| acc << 1 | b as u8)).collect()
}

pub fn cs_name(pattern: u8, idle: u8) -> String {
    let diff = (pattern ^ idle) & IO_MASK;
    let names: Vec<&str> = (0..4).filter(|i| diff >> i & 1 == 1).map(|i| CHANNEL_NAMES[i]).collect();
    if names.is_empty() {
        "none".into()
    } else {
        names.join("+")
    }
}

pub struct Window {
    pub t: f64,
    pub t_end: f64,
    pub pattern: u8,
    /// Indices of the raw16 samples at each rising SCK edge inside the window.
    pub rises: Vec<usize>,
}

/// The raw16 stream, as a slice or as something that stores it otherwise
/// (bugslayer-ui keeps only edges).
pub trait Raw16 {
    fn len(&self) -> u64;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn word(&self, i: u64) -> u16;
    /// Sample indices in `lo..hi` where a bit in `mask` differs from the
    /// sample before, sorted.
    fn changes(&self, mask: u16, lo: u64, hi: u64) -> Vec<u64>;
}

impl Raw16 for Vec<u16> {
    fn len(&self) -> u64 {
        Vec::len(self) as u64
    }
    fn word(&self, i: u64) -> u16 {
        self[i as usize]
    }
    fn changes(&self, mask: u16, lo: u64, hi: u64) -> Vec<u64> {
        (lo.max(1)..hi.min(Raw16::len(self)))
            .filter(|&i| (self[i as usize] ^ self[i as usize - 1]) & mask != 0)
            .collect()
    }
}

/// CS windows in the raw16 stream: runs where IO_1..4 differ from idle. Idle
/// is the commonest pattern by time, not the first: a capture can start inside
/// a transaction.
fn raw16_windows(raw: &dyn Raw16, rate: f64) -> (Vec<Window>, u8) {
    let n = raw.len();
    if n == 0 {
        return (Vec::new(), 0);
    }
    // Runs of one CS pattern: (start, end, pattern).
    let mut runs = Vec::new();
    let mut start = 0u64;
    for c in raw.changes(IO_MASK as u16, 1, n).into_iter().chain([n]) {
        runs.push((start, c, raw.word(start) as u8 & IO_MASK));
        start = c;
    }
    let mut hist = [0u64; 16];
    for &(a, b, p) in &runs {
        hist[p as usize] += b - a;
    }
    let idle = (0..16).max_by_key(|&i| hist[i]).unwrap_or(0) as u8;
    let out = runs
        .into_iter()
        .filter(|r| r.2 != idle)
        .map(|(a, b, p)| {
            let rises = raw
                .changes(1 << SCK_BIT, a + 1, b)
                .into_iter()
                .filter(|&i| raw.word(i) >> SCK_BIT & 1 == 1)
                .map(|i| i as usize)
                .collect();
            Window { t: a as f64 / rate, t_end: b as f64 / rate, pattern: p, rises }
        })
        .collect();
    (out, idle)
}

/// Split the sck8 stream at markers and at CS pattern changes: (pattern, edge
/// bytes) per segment. A marker follows the first edge after the CS change, so
/// it starts a segment at the edge just before it.
fn sck8_segments(b: &[u8]) -> Vec<(u8, Vec<u8>)> {
    let mut segs: Vec<(u8, Vec<u8>)> = Vec::new();
    for &x in b {
        if x >> SCK_BIT & 1 == 0 {
            if let Some((p, e)) = segs.last_mut() {
                if e.len() > 1 {
                    let first = e.pop().unwrap();
                    let p = *p;
                    segs.push((p, vec![first]));
                }
            }
            continue;
        }
        let p = x & IO_MASK;
        match segs.last_mut() {
            Some((sp, e)) if *sp == p => e.push(x),
            _ => segs.push((p, vec![x])),
        }
    }
    segs
}

pub struct Txn {
    pub t: Option<f64>,
    pub dur: Option<f64>,
    pub cs: String,
    pub edges: usize,
    pub mosi: Vec<u8>,
    pub miso: Vec<u8>,
    window: Option<usize>,
}

pub struct Decoded {
    pub txns: Vec<Txn>,
    pub notes: Vec<String>,
    /// Transactions also decoded from raw16, and how many of those differ.
    pub checked: usize,
    pub mismatched: usize,
}

/// Transactions from the sck8 stream, timed and cross-checked by raw16 when
/// given.
pub fn decode(sck8: &[u8], raw: Option<(&dyn Raw16, f64)>) -> Decoded {
    let mut notes = Vec::new();
    let segs = sck8_segments(sck8);
    let (windows, idle) = match raw {
        Some((r, rate)) if rate > 0.0 => {
            let (w, i) = raw16_windows(r, rate);
            (w, Some(i))
        }
        _ => (Vec::new(), None),
    };
    let active: Vec<usize> = (0..segs.len()).filter(|&i| idle.is_none_or(|id| segs[i].0 != id)).collect();

    // Windows with SCK activity are the ones sck8 can see; raw16 may miss SCK
    // entirely when it is too slow, so fall back to all windows.
    let mut timed: BTreeMap<usize, usize> = BTreeMap::new();
    if !windows.is_empty() {
        let seen: Vec<usize> = (0..windows.len()).filter(|&i| !windows[i].rises.is_empty()).collect();
        let all: Vec<usize> = (0..windows.len()).collect();
        let matched = [&seen, &all].into_iter().find(|cand| {
            cand.len() == active.len() && cand.iter().zip(&active).all(|(&w, &s)| windows[w].pattern == segs[s].0)
        });
        match matched {
            Some(cand) => timed = active.iter().copied().zip(cand.iter().copied()).collect(),
            None => notes.push(format!(
                "{} sck8 transactions vs {} raw16 CS windows with SCK ({} in all): not timed",
                active.len(),
                seen.len(),
                windows.len()
            )),
        }
    }

    let txns: Vec<Txn> = segs
        .iter()
        .enumerate()
        .map(|(i, (p, e))| {
            let w = timed.get(&i).copied();
            Txn {
                t: w.map(|w| windows[w].t),
                dur: w.map(|w| windows[w].t_end - windows[w].t),
                cs: idle.map_or_else(|| format!("0x{:x}", p), |id| cs_name(*p, id)),
                edges: e.len(),
                mosi: bits_to_bytes(e.iter().map(|x| x >> MOSI_BIT & 1 == 1)),
                miso: bits_to_bytes(e.iter().map(|x| x >> MISO_BIT & 1 == 1)),
                window: w,
            }
        })
        .collect();

    // For transactions whose raw16 window resolved every SCK edge, decode the
    // same bytes from raw16 alone and compare.
    let (mut checked, mut mismatched) = (0, 0);
    if let Some((r, _)) = raw {
        for t in &txns {
            let Some(w) = t.window.map(|w| &windows[w]) else { continue };
            if w.rises.len() != t.edges {
                continue;
            }
            let words: Vec<u16> = w.rises.iter().map(|&i| r.word(i as u64)).collect();
            let mosi = bits_to_bytes(words.iter().map(|x| x >> MOSI_BIT & 1 == 1));
            let miso = bits_to_bytes(words.iter().map(|x| x >> MISO_BIT & 1 == 1));
            checked += 1;
            if mosi != t.mosi || miso != t.miso {
                mismatched += 1;
            }
        }
    }
    Decoded { txns, notes, checked, mismatched }
}

fn hex(b: &[u8]) -> String {
    let s: Vec<String> = b.iter().take(16).map(|x| format!("{:02X}", x)).collect();
    format!("{}{}", s.join(" "), if b.len() > 16 { " ..." } else { "" })
}

pub fn format_txn(t: &Txn) -> String {
    let when = t.t.map_or("          ? s".to_string(), |t| format!("{:11.6} s", t));
    let dur = t.dur.map_or(" ".repeat(11), |d| format!("{:8.1} us", d * 1e6));
    format!(
        "{} {}  {:<6} {:5} clk  MOSI {}\n{:44}MISO {}",
        when,
        dur,
        t.cs,
        t.edges,
        hex(&t.mosi),
        "",
        hex(&t.miso)
    )
}

/// Transactions per chip select, e.g. [("IO_1", 9060)].
pub fn per_cs(d: &Decoded) -> Vec<(String, usize)> {
    let mut by_cs: BTreeMap<&str, usize> = BTreeMap::new();
    for t in &d.txns {
        *by_cs.entry(&t.cs).or_default() += 1;
    }
    by_cs.into_iter().map(|(k, n)| (k.to_string(), n)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Eight edges: one byte out and one in, on the CS pattern given (IO_3
    /// active low is 0xB, idle 0xF).
    fn edges(mosi: u8, miso: u8, cs_pattern: u8) -> Vec<u8> {
        (0..8)
            .map(|i| {
                let o = mosi >> (7 - i) & 1;
                let m = miso >> (7 - i) & 1;
                o << MOSI_BIT | 1 << SCK_BIT | m << MISO_BIT | cs_pattern
            })
            .collect()
    }

    #[test]
    fn segments_split_at_markers() {
        // Two transactions back to back on IO_3: the marker (bit 6 clear)
        // comes after the second transaction's first edge.
        let mut s = edges(0xA5, 0x3C, 0xB);
        let second = edges(0x81, 0x02, 0xB);
        s.push(second[0]);
        s.push(0x0B);
        s.extend(&second[1..]);
        let d = decode(&s, None);
        assert_eq!(d.txns.len(), 2);
        assert_eq!(d.txns[0].mosi, vec![0xA5]);
        assert_eq!(d.txns[0].miso, vec![0x3C]);
        assert_eq!(d.txns[1].mosi, vec![0x81]);
    }

    #[test]
    fn raw16_times_and_checks() {
        // raw16 at 4 samples per SCK period: idle, then CS on IO_3 for 8 clocks.
        let mut raw: Vec<u16> = vec![0xF; 10];
        let (mosi, miso) = (0xA5u8, 0x3Cu8);
        for i in 0..8 {
            let o = (mosi >> (7 - i) & 1) as u16;
            let m = (miso >> (7 - i) & 1) as u16;
            let base = 0xB | o << MOSI_BIT | m << MISO_BIT;
            raw.extend([base, base, base | 1 << SCK_BIT, base | 1 << SCK_BIT]);
        }
        raw.extend(vec![0xF; 100]);
        let d = decode(&edges(mosi, miso, 0xB), Some((&raw, 1e6)));
        assert_eq!(d.txns.len(), 1);
        assert_eq!(d.txns[0].cs, "IO_3");
        assert!((d.txns[0].t.unwrap() - 10e-6).abs() < 1e-9);
        assert_eq!((d.checked, d.mismatched), (1, 0));
    }
}
