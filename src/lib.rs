//! Host-side library for the Bugslayer deck, shared by `bscli` (bugslayer-cli)
//! and bugslayer-ui.
//!
//! Nothing here prints or prompts. Where a front end has to decide something
//! halfway (power the port? which of several decks?), the library either
//! returns that question as a value or an [`Error`] variant, or takes a
//! callback for progress it would otherwise have printed.

pub mod bus;
pub mod deckctrl;
pub mod device;
pub mod error;
pub mod github;
pub mod pipe;
pub mod sigrok;
pub mod spi;
pub mod stream;
pub mod swo;
pub mod uart;
pub mod update;

pub use error::Error;

/// A byte: `0x44`, `68` or `0b1000100`.
pub fn parse_byte(s: &str) -> Result<u8, String> {
    let t = s.trim();
    let r = if let Some(h) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u8::from_str_radix(h, 16)
    } else if let Some(b) = t.strip_prefix("0b") {
        u8::from_str_radix(b, 2)
    } else {
        t.parse()
    };
    r.map_err(|_| format!("'{}' is not a byte (e.g. 0x44)", s))
}

/// Hex bytes, with or without spaces/commas/colons/0x: `1900`, `"19 00"`, `0x19,0x00`.
pub fn parse_hex(s: &str) -> Result<Vec<u8>, String> {
    let cleaned: String =
        s.split([' ', ',', ':']).map(|t| t.trim_start_matches("0x").trim_start_matches("0X")).collect();
    if cleaned.is_empty() || !cleaned.len().is_multiple_of(2) || !cleaned.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(format!("'{}' is not hex bytes (e.g. 1900 or \"19 00\")", s));
    }
    Ok((0..cleaned.len() / 2).map(|i| u8::from_str_radix(&cleaned[2 * i..2 * i + 2], 16).unwrap()).collect())
}

/// Classic hexdump lines, addresses starting at `base`.
pub fn hexdump(base: u32, data: &[u8]) -> Vec<String> {
    data.chunks(16)
        .enumerate()
        .map(|(i, c)| {
            let hex: Vec<String> = c.iter().map(|b| format!("{:02x}", b)).collect();
            let asc: String = c.iter().map(|&b| if (0x20..0x7f).contains(&b) { b as char } else { '.' }).collect();
            format!("{:04x}  {:<48} {}", base as usize + 16 * i, hex.join(" "), asc)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_bytes_and_hex() {
        assert_eq!(parse_byte("0x44"), Ok(0x44));
        assert_eq!(parse_byte("68"), Ok(68));
        assert!(parse_byte("0x144").is_err());
        assert_eq!(parse_hex("1900").unwrap(), vec![0x19, 0x00]);
        assert_eq!(parse_hex("0x19, 0x00 ab").unwrap(), vec![0x19, 0x00, 0xab]);
        assert!(parse_hex("190").is_err());
        assert!(parse_hex("zz").is_err());
    }

    #[test]
    fn dumps() {
        let d = hexdump(0x20, b"WiFi camera deck");
        assert_eq!(d, vec!["0020  57 69 46 69 20 63 61 6d 65 72 61 20 64 65 63 6b  WiFi camera deck"]);
    }
}
