// The expansion-port I2C bus, with the deck as master.
//
// The firmware only moves bytes (`i2c on|off|xfer|recover`); what is safe to
// do with the bus is decided here. With a Crazyflie in the stack its STM32 is
// the master and the deck must stay off the bus. Standalone, the deck has to
// power the decks (VCC) and pull the bus up itself.

use std::time::Duration;

use anyhow::{bail, Context, Result};

use crate::device::{self, Control};
use crate::error::Error;

pub struct BusOptions {
    pub rate: u32,
    /// Drive the bus even though a Crazyflie seems to own it.
    pub force: bool,
    /// Switch VCC (and VCOM) on when the port is unpowered. Without it an
    /// unpowered port is `Error::Unpowered`, for the front end to ask.
    pub power: bool,
}

/// Why a transfer did not complete.
#[derive(Debug, PartialEq, Eq)]
pub enum XferError {
    Nak,
    Timeout,
}

/// Something the library did on the caller's behalf, for it to report:
/// (`"power"` or `"bus"`, what).
pub type Note = (&'static str, String);

pub struct Bus<'a> {
    pub ctl: &'a mut Control,
    /// What `open` switched on.
    pub notes: Vec<Note>,
}

/// What `stat` says about power and the bus.
#[derive(Debug, Clone, Copy)]
pub struct PortState {
    pub cf_vcc: bool,
    pub vcc_en: bool,
    pub vcom_en: bool,
    pub pull: bool,
}

impl PortState {
    /// VCC on the port that the deck did not switch on: a Crazyflie powers it,
    /// so its STM32 is the bus master and TX1/TX2 are its outputs.
    pub fn crazyflie(&self) -> bool {
        self.cf_vcc && !self.vcc_en
    }

    /// Standalone and without VCC or VCOM: `power_standalone` would switch
    /// something on.
    pub fn needs_power(&self) -> bool {
        !self.crazyflie() && (!self.cf_vcc || !self.vcom_en)
    }

    /// Standalone with no VCC at all: switching on powers the decks, which is
    /// worth asking about (VCOM alone is not).
    pub fn unpowered(&self) -> bool {
        !self.cf_vcc
    }
}

pub fn port_state(ctl: &mut Control) -> Result<PortState> {
    let mut kv = std::collections::BTreeMap::new();
    for r in ctl.query("stat")? {
        kv.extend(device::kv(&r));
    }
    if !kv.contains_key("vcc_en") {
        bail!(Error::Rejected(
            "the deck firmware is too old for I2C (needs rp2350 0.5.0: `i2c`, and power state in `stat`)".into()
        ));
    }
    let on = |k: &str| kv.get(k).map(String::as_str) == Some("1");
    Ok(PortState { cf_vcc: on("cf_vcc"), vcc_en: on("vcc_en"), vcom_en: on("vcom_en"), pull: on("pull") })
}

/// Standalone (no Crazyflie on the port): make sure both VCC and VCOM are on.
/// The firmware refuses to drive pins high without VCOM, because decks that run
/// from it (Lighthouse) would be back-powered through them. An unpowered port
/// is `Error::Unpowered` unless `power`.
pub fn power_standalone(ctl: &mut Control, st: &PortState, power: bool) -> Result<Vec<Note>> {
    let mut notes = Vec::new();
    if !st.cf_vcc {
        if !power {
            bail!(Error::Unpowered("the expansion port is unpowered".into()));
        }
        ctl.expect_ok("pwr vcc on")?;
        ctl.expect_ok("pwr vcom on")?;
        notes.push(("power", "VCC and VCOM switched on".into()));
    } else if !st.vcom_en {
        // VCC was switched on by hand, without VCOM.
        ctl.expect_ok("pwr vcom on")?;
        notes.push(("power", "VCOM switched on".into()));
    } else {
        return Ok(notes);
    }
    // Deck controllers need a moment after power-on before they answer.
    std::thread::sleep(Duration::from_millis(150));
    Ok(notes)
}

impl<'a> Bus<'a> {
    /// Make the bus usable: refuse if a Crazyflie is its master, power the
    /// decks and pull the bus up when standalone, and start the I2C master.
    pub fn open(ctl: &'a mut Control, opts: &BusOptions) -> Result<Bus<'a>> {
        let st = port_state(ctl)?;
        let crazyflie = st.crazyflie();
        if crazyflie && !opts.force {
            bail!(Error::Rejected(
                "a Crazyflie powers the expansion port, so it is the I2C master there; \
                 remove it, or pass --force to drive the bus anyway"
                    .into()
            ));
        }
        let mut notes = Vec::new();
        if !crazyflie {
            notes = power_standalone(ctl, &st, opts.power)?;
        }
        if !crazyflie && !st.pull {
            ctl.expect_ok("pull on")?;
            notes.push(("bus", "I2C pull-ups on".into()));
        }
        ctl.expect_ok(&format!("i2c on {}", opts.rate))?;
        Ok(Bus { ctl, notes })
    }

    /// Write `w`, then read `n` bytes with a repeated START.
    pub fn xfer(&mut self, addr: u8, w: &[u8], n: usize) -> Result<std::result::Result<Vec<u8>, XferError>> {
        let hex = if w.is_empty() { "-".to_string() } else { w.iter().map(|b| format!("{:02x}", b)).collect() };
        let reply = self.ctl.command(&format!("i2c xfer 0x{:02x} {} {}", addr, hex, n))?;
        match reply.as_str() {
            "err i2c nak" => return Ok(Err(XferError::Nak)),
            "err i2c timeout" => return Ok(Err(XferError::Timeout)),
            _ => {}
        }
        let Some(data) = reply.strip_prefix("i2c ok data=") else {
            bail!(Error::Rejected(format!("i2c xfer: {}", reply.strip_prefix("err ").unwrap_or(&reply))));
        };
        let bytes = (0..data.len() / 2)
            .map(|i| u8::from_str_radix(&data[2 * i..2 * i + 2], 16))
            .collect::<std::result::Result<Vec<u8>, _>>()
            .with_context(|| format!("bad data in '{}'", reply))?;
        if bytes.len() != n {
            bail!("i2c xfer: asked for {} bytes, got {}", n, bytes.len());
        }
        Ok(Ok(bytes))
    }

    /// Like `xfer`, but a NAK or timeout is an error.
    pub fn xfer_ok(&mut self, addr: u8, w: &[u8], n: usize) -> Result<Vec<u8>> {
        match self.xfer(addr, w, n)? {
            Ok(d) => Ok(d),
            Err(XferError::Nak) => bail!(Error::NotFound(format!("nothing acknowledged at 0x{:02x}", addr))),
            Err(XferError::Timeout) => bail!(Error::Timeout(format!(
                "I2C transfer to 0x{:02x} (bus held low? try `bscli i2c recover`)",
                addr
            ))),
        }
    }

    /// The addresses 0x08..0x77 that acknowledge, skipping the DeckCtrl
    /// reset/listen addresses, which have side effects.
    pub fn scan(&mut self) -> Result<Vec<u8>> {
        let mut found = Vec::new();
        for addr in 0x08u8..0x78 {
            if addr == crate::deckctrl::ADDR_RESET || addr == crate::deckctrl::ADDR_LISTEN {
                continue;
            }
            if self.xfer(addr, &[], 1)?.is_ok() {
                found.push(addr);
            }
        }
        Ok(found)
    }
}

impl Drop for Bus<'_> {
    /// Off the bus again: the pins go back to Hi-Z inputs.
    fn drop(&mut self) {
        let _ = self.ctl.command("i2c off");
    }
}

/// What an address that acknowledged on a scan is likely to be.
pub fn address_hint(addr: u8) -> &'static str {
    match addr {
        crate::deckctrl::ADDR_DISCOVERY => "DeckCtrl discovery address (unenumerated deck controller)",
        0x44..=0x4f => "DeckCtrl assigned range",
        0x50..=0x57 => "EEPROM range",
        _ => "",
    }
}

/// Clock SCL until a stuck device lets go of SDA. True when SDA is released.
pub fn recover(ctl: &mut Control) -> Result<bool> {
    let reply = ctl.expect_ok("i2c recover")?;
    Ok(device::kv(&reply).get("sda").map(String::as_str) == Some("1"))
}
