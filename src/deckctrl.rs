// DeckCtrl: the STM32C011 deck controller on newer decks, spoken over the
// expansion-port I2C bus exactly as the Crazyflie does it
// (crazyflie-firmware src/deck/backends/deck_backend_deckctrl.c; register map
// in deck-ctrl-firmware README.md).
//
// Every controller starts at the discovery address. Enumeration resets them
// all, then repeatedly: put the unassigned ones in listening mode, read a CPU
// ID from the discovery address (the lowest ID wins bus arbitration, the rest
// back off), and move the winner to its own address.

use std::time::Duration;

use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};

use crate::bus::{Bus, XferError};
use crate::error::Error;

pub const ADDR_RESET: u8 = 0x41;
pub const ADDR_LISTEN: u8 = 0x42;
pub const ADDR_DISCOVERY: u8 = 0x43;
pub const ADDR_FIRST: u8 = 0x44;
/// 0x44..0x4F.
pub const MAX_DECKS: usize = 12;

pub const REG_INFO: u16 = 0x0000;
pub const INFO_LEN: usize = 0x20;
pub const REG_GPIO_DIR: u16 = 0x1000;
pub const REG_GPIO_VALUE: u16 = 0x1002;
pub const REG_ADDRESS: u16 = 0x1800;
pub const REG_CPU_ID: u16 = 0x1900;
pub const CPU_ID_LEN: usize = 12;

/// DeckCtrl GPIO index -> STM32C011 pin (deck-ctrl-firmware Core/Src/module/gpio.c).
pub const GPIO_PINS: [&str; 13] =
    ["PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA11", "PA12", "PC14", "PC15"];

fn reg(r: u16) -> [u8; 2] {
    r.to_be_bytes() // register address MSB first
}

pub fn read_reg(bus: &mut Bus, addr: u8, r: u16, n: usize) -> Result<Vec<u8>> {
    // The firmware moves at most 512 bytes per transfer.
    let mut out = Vec::with_capacity(n);
    while out.len() < n {
        let k = (n - out.len()).min(512);
        out.extend(bus.xfer_ok(addr, &reg(r + out.len() as u16), k)?);
    }
    Ok(out)
}

pub fn write_reg(bus: &mut Bus, addr: u8, r: u16, data: &[u8]) -> Result<()> {
    for (i, chunk) in data.chunks(500).enumerate() {
        let mut w = reg(r + (i * 500) as u16).to_vec();
        w.extend_from_slice(chunk);
        bus.xfer_ok(addr, &w, 0)?;
    }
    Ok(())
}

#[derive(Debug, Clone)]
pub struct Info {
    pub fw_major: u8,
    pub fw_minor: u8,
    pub vid: u8,
    pub pid: u8,
    pub rev: char,
    pub name: String,
    /// (year, month, day), None when unset (0 or 0xFF).
    pub manufactured: Option<(u16, u8, u8)>,
    pub magic_ok: bool,
    pub checksum_ok: bool,
}

impl Info {
    pub fn parse(raw: &[u8]) -> Info {
        let name_end = raw[7..22].iter().position(|&b| b == 0).unwrap_or(15);
        let (y, m, d) = (raw[0x16], raw[0x17], raw[0x18]);
        let bad = |v: u8| v == 0 || v == 0xFF;
        Info {
            fw_major: raw[2],
            fw_minor: raw[3],
            vid: raw[4],
            pid: raw[5],
            rev: raw[6] as char,
            name: String::from_utf8_lossy(&raw[7..7 + name_end]).into_owned(),
            manufactured: (!(bad(y) || bad(m) || bad(d))).then_some((2000 + y as u16, m, d)),
            magic_ok: raw[0] == 0xBC && raw[1] == 0xDC,
            checksum_ok: raw.iter().fold(0u8, |a, &b| a.wrapping_add(b)) == 0,
        }
    }

    pub fn problems(&self) -> Vec<&'static str> {
        let mut p = Vec::new();
        if !self.magic_ok {
            p.push("bad magic");
        }
        if !self.checksum_ok {
            p.push("bad checksum");
        }
        if self.manufactured.is_none() {
            p.push("no production date (a Crazyflie rejects it)");
        }
        p
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct Found {
    pub addr: u8,
    pub cpu_id: String,
    pub name: String,
    pub rev: String,
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{:02X}", x)).collect()
}

/// Reset every controller to its power-on state. False if none answered.
pub fn reset_all(bus: &mut Bus) -> Result<bool> {
    let answered = bus.xfer(ADDR_RESET, &reg(0), 2)?.is_ok();
    if answered {
        // It is a hardware reset of the controller MCU.
        std::thread::sleep(Duration::from_millis(20));
    }
    Ok(answered)
}

/// Discover every controller and give each its own address. Resets them all
/// first, so any GPIO state they held is lost.
pub fn enumerate(bus: &mut Bus) -> Result<Vec<(Found, Option<Info>)>> {
    reset_all(bus)?;
    let mut found = Vec::new();
    for i in 0..MAX_DECKS {
        if bus.xfer(ADDR_LISTEN, &reg(0), 2)?.is_err() {
            break; // nobody left unassigned
        }
        let cpu = match bus.xfer(ADDR_DISCOVERY, &reg(REG_CPU_ID), CPU_ID_LEN)? {
            Ok(c) => c,
            Err(_) => break,
        };
        let addr = ADDR_FIRST + i as u8;
        let mut w = reg(REG_ADDRESS).to_vec();
        w.push(addr);
        bus.xfer_ok(ADDR_DISCOVERY, &w, 0)?;
        let info = match bus.xfer(addr, &reg(REG_INFO), INFO_LEN)? {
            Ok(raw) => Some(Info::parse(&raw)),
            Err(XferError::Nak | XferError::Timeout) => None,
        };
        let (name, rev) = info.as_ref().map_or(("?".into(), "?".into()), |i| (i.name.clone(), i.rev.to_string()));
        found.push((Found { addr, cpu_id: hex(&cpu), name, rev }, info));
    }
    Ok(found)
}

/// Whether the controller at `f.addr` is still the one we assigned there.
fn still_there(bus: &mut Bus, f: &Found) -> Result<bool> {
    Ok(match bus.xfer(f.addr, &reg(REG_CPU_ID), CPU_ID_LEN)? {
        Ok(cpu) => hex(&cpu) == f.cpu_id,
        Err(_) => false,
    })
}

/// The enumerated decks: from `cache` when every one still answers with its
/// CPU ID, else by enumerating again (which resets them all; `enumerating` is
/// called first, as that takes a moment and loses the decks' GPIO state).
/// The flag says whether they were enumerated afresh.
pub fn decks(bus: &mut Bus, cache: &[Found], rescan: bool, enumerating: impl FnOnce()) -> Result<(Vec<Found>, bool)> {
    if !rescan && !cache.is_empty() {
        let mut ok = true;
        for f in cache {
            if !still_there(bus, f)? {
                ok = false;
                break;
            }
        }
        // Also nobody new waiting at the discovery address.
        if ok && bus.xfer(ADDR_DISCOVERY, &reg(REG_CPU_ID), CPU_ID_LEN)?.is_err() {
            return Ok((cache.to_vec(), false));
        }
    }
    enumerating();
    Ok((enumerate(bus)?.into_iter().map(|(f, _)| f).collect(), true))
}

/// Pick one deck: by address (`0x44`), index (`0`), name or CPU ID prefix.
/// None when several answered and `sel` does not say which: the front end asks.
pub fn pick<'a>(decks: &'a [Found], sel: Option<&str>) -> Result<Option<&'a Found>> {
    if decks.is_empty() {
        bail!(Error::NotFound("no deck controller answered on the bus".into()));
    }
    if let Some(s) = sel {
        let by_num = crate::parse_byte(s).ok().and_then(|n| {
            if (n as usize) < decks.len() && n < ADDR_FIRST {
                decks.get(n as usize)
            } else {
                decks.iter().find(|d| d.addr == n)
            }
        });
        let low = s.to_lowercase();
        let hit = by_num
            .or_else(|| decks.iter().find(|d| d.name.to_lowercase() == low))
            .or_else(|| decks.iter().find(|d| d.name.to_lowercase().starts_with(&low)))
            .or_else(|| decks.iter().find(|d| d.cpu_id.to_lowercase().starts_with(&low)));
        return hit.map(Some).ok_or_else(|| Error::NotFound(format!("no deck controller matches '{}'", s)).into());
    }
    Ok((decks.len() == 1).then(|| &decks[0]))
}

/// Direction and value registers, one bit per GPIO.
pub fn gpio_read(bus: &mut Bus, addr: u8) -> Result<(u16, u16)> {
    let r = read_reg(bus, addr, REG_GPIO_DIR, 4)?;
    Ok((u16::from_le_bytes([r[0], r[1]]), u16::from_le_bytes([r[2], r[3]])))
}

pub fn gpio_write_dir(bus: &mut Bus, addr: u8, dir: u16) -> Result<()> {
    write_reg(bus, addr, REG_GPIO_DIR, &dir.to_le_bytes())
}

pub fn gpio_write_value(bus: &mut Bus, addr: u8, value: u16) -> Result<()> {
    write_reg(bus, addr, REG_GPIO_VALUE, &value.to_le_bytes())
}

/// `3`, `0,4,12`, `0-3,9`, or `all`.
pub fn parse_pins(s: &str) -> Result<u16, String> {
    if s.eq_ignore_ascii_case("all") {
        return Ok((1 << GPIO_PINS.len()) - 1);
    }
    let mut mask = 0u16;
    for part in s.split(',').map(str::trim).filter(|p| !p.is_empty()) {
        let one = |t: &str| -> Result<usize, String> {
            let t = t.trim();
            let n = GPIO_PINS
                .iter()
                .position(|p| p.eq_ignore_ascii_case(t))
                .or_else(|| t.parse().ok())
                .ok_or(format!("'{}' is not a GPIO (0-12 or a pin name like PA4)", t))?;
            if n >= GPIO_PINS.len() {
                return Err(format!("GPIO {} out of range (0-{})", n, GPIO_PINS.len() - 1));
            }
            Ok(n)
        };
        let (a, b) = match part.split_once('-') {
            Some((a, b)) => (one(a)?, one(b)?),
            None => (one(part)?, one(part)?),
        };
        for i in a.min(b)..=a.max(b) {
            mask |= 1 << i;
        }
    }
    if mask == 0 {
        return Err("no GPIOs given".into());
    }
    Ok(mask)
}

/// Per GPIO: (index, pin name, output, high).
pub fn gpio_rows(dir: u16, value: u16) -> Vec<(usize, &'static str, bool, bool)> {
    GPIO_PINS.iter().enumerate().map(|(i, &pin)| (i, pin, dir >> i & 1 == 1, value >> i & 1 == 1)).collect()
}

/// Set GPIOs: the level first, then the direction, so a pin turned into an
/// output comes up at the level it was given. Writes only what changes.
pub fn gpio_set(bus: &mut Bus, addr: u8, old: (u16, u16), new: (u16, u16)) -> Result<()> {
    if new.1 != old.1 {
        gpio_write_value(bus, addr, new.1)?;
    }
    if new.0 != old.0 {
        gpio_write_dir(bus, addr, new.0)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pin_lists() {
        assert_eq!(parse_pins("3"), Ok(1 << 3));
        assert_eq!(parse_pins("0-2,12"), Ok(0b1_0000_0000_0111));
        assert_eq!(parse_pins("pa4,PC15"), Ok(1 << 4 | 1 << 12));
        assert_eq!(parse_pins("all"), Ok(0x1FFF));
        assert!(parse_pins("13").is_err());
        assert!(parse_pins("").is_err());
    }

    #[test]
    fn info_page() {
        // The WiFi camera deck config from deck-ctrl-firmware/configs/cam.yaml.
        let mut raw = vec![0u8; 32];
        raw[..7].copy_from_slice(&[0xBC, 0xDC, 0, 1, 0xBC, 0x21, b'A']);
        // 15-byte field: cfcli stores 14 characters + NUL.
        raw[7..21].copy_from_slice(&b"WiFi camera deck"[..14]);
        raw[0x16..0x19].copy_from_slice(&[26, 1, 1]);
        let sum = raw[..31].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        raw[31] = 0u8.wrapping_sub(sum);
        let i = Info::parse(&raw);
        assert!(i.magic_ok && i.checksum_ok);
        assert_eq!(i.name, "WiFi camera de");
        assert_eq!((i.vid, i.pid, i.rev), (0xBC, 0x21, 'A'));
        assert_eq!(i.manufactured, Some((2026, 1, 1)));
        assert!(i.problems().is_empty());
    }
}
