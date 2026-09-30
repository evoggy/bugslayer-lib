// sigrok session files (.sr, v2): a zip of `version`, INI `metadata` and raw
// `logic-1-N` chunks. PulseView and sigrok-cli open them, and
// libsigrokdecode's decoders run on them unchanged.
//
// The writer streams: samples go into the zip in 4 MB chunks as they arrive,
// so a recording is bounded by disk, not RAM. The sck8 stream, if any, rides
// along as one more member that sigrok ignores and `bsly decode spi` reads.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::Path;

use anyhow::{bail, Context, Result};
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

pub const SCK8_MEMBER: &str = "bugslayer-sck8";
const CHUNK: usize = 4 * 1024 * 1024;
const UNIT: usize = 2;

pub fn samplerate_string(hz: u64) -> String {
    if hz.is_multiple_of(1_000_000) {
        format!("{} MHz", hz / 1_000_000)
    } else if hz.is_multiple_of(1_000) {
        format!("{} kHz", hz / 1_000)
    } else {
        format!("{} Hz", hz)
    }
}

fn parse_samplerate(s: &str) -> Option<u64> {
    let mut it = s.split_whitespace();
    let num: f64 = it.next()?.parse().ok()?;
    let mult = match it.next().unwrap_or("Hz") {
        "Hz" => 1.0,
        "kHz" => 1e3,
        "MHz" => 1e6,
        "GHz" => 1e9,
        _ => return None,
    };
    Some((num * mult).round() as u64)
}

pub struct SrWriter {
    zip: ZipWriter<BufWriter<File>>,
    buf: Vec<u8>,
    chunks: usize,
    last: [u8; UNIT],
    pub samples: u64,
}

impl SrWriter {
    pub fn create(path: &Path, rate_hz: u64, names: &[&str]) -> Result<SrWriter> {
        let file = File::create(path).with_context(|| format!("creating {}", path.display()))?;
        let mut zip = ZipWriter::new(BufWriter::new(file));
        let opts = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
        zip.start_file("version", opts)?;
        zip.write_all(b"2")?;
        let mut meta = String::from("[global]\nsigrok version=0.5.2\n\n[device 1]\n");
        meta += "capturefile=logic-1\n";
        meta += &format!("total probes={}\n", names.len());
        meta += &format!("samplerate={}\n", samplerate_string(rate_hz));
        meta += "total analog=0\n";
        for (i, n) in names.iter().enumerate() {
            meta += &format!("probe{}={}\n", i + 1, n);
        }
        meta += &format!("unitsize={}\n", UNIT);
        zip.start_file("metadata", opts)?;
        zip.write_all(meta.as_bytes())?;
        Ok(SrWriter { zip, buf: Vec::with_capacity(CHUNK), chunks: 0, last: [0; UNIT], samples: 0 })
    }

    fn flush_chunk(&mut self) -> Result<()> {
        if self.buf.is_empty() {
            return Ok(());
        }
        self.chunks += 1;
        // Fast deflate: logic data compresses well even at level 1, and the
        // writer has to keep up with 35 MB/s.
        let opts = SimpleFileOptions::default()
            .compression_method(CompressionMethod::Deflated)
            .compression_level(Some(1))
            .large_file(false);
        self.zip.start_file(format!("logic-1-{}", self.chunks), opts)?;
        self.zip.write_all(&self.buf)?;
        self.buf.clear();
        Ok(())
    }

    /// Raw16 samples, little-endian.
    pub fn push(&mut self, mut data: &[u8]) -> Result<()> {
        if data.len() >= UNIT {
            self.last.copy_from_slice(&data[data.len() - UNIT..]);
        }
        self.samples += (data.len() / UNIT) as u64;
        while !data.is_empty() {
            let n = (CHUNK - self.buf.len()).min(data.len());
            self.buf.extend_from_slice(&data[..n]);
            data = &data[n..];
            if self.buf.len() == CHUNK {
                self.flush_chunk()?;
            }
        }
        Ok(())
    }

    /// An overrun gap: `.sr` cannot mark one, so hold the last value.
    pub fn hold(&mut self, n: u64) -> Result<()> {
        let fill: Vec<u8> = self.last.iter().copied().cycle().take(64 * 1024 * UNIT).collect();
        let mut left = n;
        while left > 0 {
            let k = left.min((fill.len() / UNIT) as u64);
            let last = self.last;
            self.push(&fill[..k as usize * UNIT])?;
            self.last = last;
            left -= k;
        }
        Ok(())
    }

    pub fn finish(mut self, sck8: Option<&[u8]>) -> Result<()> {
        self.flush_chunk()?;
        if self.chunks == 0 {
            // sigrok wants at least one logic chunk.
            self.zip.start_file("logic-1-1", SimpleFileOptions::default())?;
        }
        if let Some(b) = sck8 {
            let opts = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
            self.zip.start_file(SCK8_MEMBER, opts)?;
            self.zip.write_all(b)?;
        }
        self.zip.finish()?.flush()?;
        Ok(())
    }
}

pub struct SrFile {
    pub raw16: Vec<u16>,
    pub rate_hz: u64,
    pub sck8: Option<Vec<u8>>,
}

/// A .sr with 16 channels at unitsize 2, as `bsly capture` writes it.
pub fn read(path: &Path) -> Result<SrFile> {
    let mut bytes = Vec::new();
    let (rate_hz, sck8) = read_stream(path, |b| bytes.extend_from_slice(b))?;
    let raw16 = bytes.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
    Ok(SrFile { raw16, rate_hz, sck8 })
}

/// As `read`, handing the samples (little-endian raw16 bytes, whole words) to
/// `f` one logic chunk at a time instead of collecting them. Returns the
/// sample rate and the sck8 member.
pub fn read_stream(path: &Path, mut chunk: impl FnMut(&[u8])) -> Result<(u64, Option<Vec<u8>>)> {
    let f = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    let mut z = ZipArchive::new(f).with_context(|| format!("{} is not a sigrok session", path.display()))?;
    let mut meta = String::new();
    z.by_name("metadata")?.read_to_string(&mut meta)?;
    let mut in_dev = false;
    let (mut capturefile, mut rate, mut unit) = (String::from("logic-1"), None, 1usize);
    for line in meta.lines() {
        let line = line.trim();
        if line.starts_with('[') {
            in_dev = line == "[device 1]";
        } else if in_dev {
            if let Some((k, v)) = line.split_once('=') {
                match k.trim() {
                    "capturefile" => capturefile = v.trim().to_string(),
                    "samplerate" => rate = parse_samplerate(v.trim()),
                    "unitsize" => unit = v.trim().parse().unwrap_or(1),
                    _ => {}
                }
            }
        }
    }
    if unit != 2 {
        bail!("{}: unitsize {} (bsly writes 2)", path.display(), unit);
    }
    let rate_hz = rate.with_context(|| format!("{}: no samplerate in metadata", path.display()))?;
    let prefix = format!("{}-", capturefile);
    let mut members: Vec<(usize, String)> = z
        .file_names()
        .filter_map(|n| n.strip_prefix(&prefix).and_then(|i| i.parse().ok()).map(|i| (i, n.to_string())))
        .collect();
    members.sort();
    let mut bytes = Vec::new();
    for (_, name) in &members {
        z.by_name(name)?.read_to_end(&mut bytes)?;
        let whole = bytes.len() / UNIT * UNIT;
        chunk(&bytes[..whole]);
        bytes.drain(..whole);
    }
    let sck8 = match z.by_name(SCK8_MEMBER) {
        Ok(mut m) => {
            let mut b = Vec::new();
            m.read_to_end(&mut b)?;
            Some(b)
        }
        Err(_) => None,
    };
    Ok((rate_hz, sck8))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_with_hold() {
        let dir = std::env::temp_dir().join(format!("bsly-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.sr");
        let mut w = SrWriter::create(&path, 250_000, &["a", "b"]).unwrap();
        w.push(&[1, 0, 2, 0, 3, 0]).unwrap();
        w.hold(3).unwrap();
        w.push(&[4, 0]).unwrap();
        w.finish(Some(&[0x40, 0x41])).unwrap();
        let f = read(&path).unwrap();
        assert_eq!(f.raw16, vec![1, 2, 3, 3, 3, 3, 4]);
        assert_eq!(f.rate_hz, 250_000);
        assert_eq!(f.sck8.as_deref(), Some(&[0x40u8, 0x41][..]));
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
