// An 8N1 UART receiver on sampled levels, one per line: feed it samples of
// one bit of the raw16 stream and it returns the bytes as they complete.

enum State {
    Idle,
    /// In a frame that started (falling edge) at sample `start`; `k` is the
    /// next bit to sample: 0 start, 1..=8 data, 9 stop.
    Frame { start: u64, k: u8, byte: u8, next: u64 },
}

/// One line's 8N1 receiver.
pub struct Rx {
    /// Bit of the raw16 word (index into CHANNEL_NAMES).
    pub bit: usize,
    spb: f64,
    prev: bool,
    state: State,
    /// Bytes received, and frames whose stop bit was low.
    pub bytes: u64,
    pub framing: u64,
}

impl Rx {
    /// A receiver for `bit`, at `spb` samples per bit.
    pub fn new(bit: usize, spb: f64) -> Rx {
        // Wait for the line to be seen idle (high) before the first start bit.
        Rx { bit, spb, prev: false, state: State::Idle, bytes: 0, framing: 0 }
    }

    fn at(&self, start: u64, k: u8) -> u64 {
        start + ((k as f64 + 0.5) * self.spb) as u64
    }

    /// Feed the sample with index `i`; returns a received byte.
    pub fn push(&mut self, i: u64, level: bool) -> Option<u8> {
        let mut out = None;
        match self.state {
            State::Idle => {
                if self.prev && !level {
                    self.state = State::Frame { start: i, k: 0, byte: 0, next: self.at(i, 0) };
                }
            }
            State::Frame { start, k, byte, next } if i >= next => match k {
                0 if level => self.state = State::Idle, // a glitch, not a start bit
                0 => self.state = State::Frame { start, k: 1, byte, next: self.at(start, 1) },
                1..=8 => {
                    let byte = byte | (level as u8) << (k - 1);
                    self.state = State::Frame { start, k: k + 1, byte, next: self.at(start, k + 1) };
                }
                _ => {
                    if level {
                        self.bytes += 1;
                        out = Some(byte);
                    } else {
                        self.framing += 1; // wrong baud, or a break
                    }
                    self.state = State::Idle;
                }
            },
            State::Frame { .. } => {}
        }
        self.prev = level;
        out
    }

    /// Samples were lost: drop any frame in flight.
    pub fn gap(&mut self) {
        self.state = State::Idle;
        self.prev = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Samples of `bytes` sent 8N1 at `spb` samples per bit, idle before and after.
    fn waveform(bytes: &[u8], spb: usize) -> Vec<bool> {
        let mut w = vec![true; 3 * spb];
        for &b in bytes {
            let bits = std::iter::once(false).chain((0..8).map(|i| b >> i & 1 == 1)).chain(std::iter::once(true));
            for bit in bits {
                w.extend(std::iter::repeat_n(bit, spb));
            }
        }
        w.extend(vec![true; 3 * spb]);
        w
    }

    #[test]
    fn decodes_8n1() {
        let msg = b"ESP-ROM:esp32s3\r\n\x00\xff";
        let mut rx = Rx::new(0, 17.36); // 2 Msps at 115200
        let w = waveform(msg, 17);
        let got: Vec<u8> = w.iter().enumerate().filter_map(|(i, &l)| rx.push(i as u64, l)).collect();
        assert_eq!(got, msg);
        assert_eq!(rx.framing, 0);
    }

    #[test]
    fn wrong_baud_gives_framing_errors() {
        let mut rx = Rx::new(0, 17.36 / 4.0);
        let w = waveform(b"\x00\x00\x00", 17);
        for (i, &l) in w.iter().enumerate() {
            rx.push(i as u64, l);
        }
        assert!(rx.framing > 0);
    }
}
