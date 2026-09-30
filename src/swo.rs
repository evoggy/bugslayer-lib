// SWO viewer: read the probe's SWO serial port (P1 = the Crazyflie's STM32)
// and decode the ITM packets on it.
//
// The RP2040 probe receives SWO on UART1 and forwards the raw bytes on its
// "SWO ACM0" CDC port, so the host sets the baud rate and decodes ITM here.
// The target's TPIU must be set up for NRZ at the same rate (the Crazyflie
// firmware does it with CONFIG_DEBUG_PRINT_ON_SWO).
//
// Only SWD1 (P1) and SWD2 (P5) have an SWO line; both are UART1 RX, so one at
// a time. DAP vendor commands 0x81/0x82 bind ACM0 to SWO1/SWO2.
//
// The STM32 only drives SWO (PB3, JTDO/TRACESWO) while its SWJ-DP is in SWD
// mode. It powers up in JTAG mode, and OpenOCD switches it back to JTAG when it
// exits, so SWO goes silent after every debug session. The keeper thread
// switches it to SWD through the port's DAP interface at start, whenever a
// debugger releases that port, and when SWO goes quiet (a session can be too
// short to see).

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use nusb::transfer::{Bulk, In, Out};
use nusb::{DeviceInfo, MaybeFuture};

use crate::device::{PID_PROBE, VID};
use crate::error::Error;

/// The probe's CMSIS-DAP interfaces: interface n is port SWDn.
const DAP_ITFS: u8 = 4;
/// "SWO ACM0": the CDC port the selected SWO is routed to.
const SWO_CDC_ITF: u8 = 4;
/// DAP vendor command binding ACM0 to SWO1; +1 for SWO2.
const ROUTE_ACM0_SWO1: u8 = 0x81;
/// How often the keeper checks whether a debugger has released the port.
const KEEPER_POLL: Duration = Duration::from_millis(100);
/// A debug session too short to see between polls also leaves the target in
/// JTAG mode, so switch again after this long without SWO data...
const KEEPER_SILENCE: Duration = Duration::from_secs(1);
/// ...but not more often than this while the target stays quiet.
const KEEPER_RETRY: Duration = Duration::from_secs(2);

/// One decoded ITM packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Packet {
    /// Software source: a write of `len` bytes to stimulus port `port`.
    Stimulus { port: u8, len: u8, value: u32 },
    /// Hardware source (DWT): discriminator `id`.
    Hardware { id: u8, len: u8, value: u32 },
    /// The ITM dropped packets.
    Overflow,
    /// Timestamp and extension packets (not used here).
    Other,
    /// A header byte that is no valid packet (lost sync or wrong baud rate).
    Invalid(u8),
}

#[derive(Debug)]
enum State {
    Header,
    /// Collecting a source packet's payload.
    Payload { hw: bool, addr: u8, len: u8, got: u8, value: u32 },
    /// Skipping continuation bytes (bit 7 set) of a timestamp/extension.
    Continuation,
}

/// Byte-at-a-time ITM packet decoder (ARMv7-M ARM, appendix D4).
pub struct Itm {
    state: State,
    zeros: u32,
}

impl Default for Itm {
    fn default() -> Itm {
        Itm::new()
    }
}

impl Itm {
    pub fn new() -> Itm {
        Itm { state: State::Header, zeros: 0 }
    }

    pub fn push(&mut self, b: u8) -> Option<Packet> {
        match self.state {
            State::Payload { hw, addr, len, got, value } => {
                let value = value | (b as u32) << (8 * got);
                if got + 1 == len {
                    self.state = State::Header;
                    return Some(if hw {
                        Packet::Hardware { id: addr, len, value }
                    } else {
                        Packet::Stimulus { port: addr, len, value }
                    });
                }
                self.state = State::Payload { hw, addr, len, got: got + 1, value };
                None
            }
            State::Continuation => {
                if b & 0x80 == 0 {
                    self.state = State::Header;
                }
                None
            }
            State::Header => {
                let zeros = std::mem::replace(&mut self.zeros, 0);
                match b {
                    // Synchronization: at least 47 zero bits then a one.
                    0x00 => {
                        self.zeros = zeros + 1;
                        None
                    }
                    0x80 if zeros >= 5 => None,
                    0x70 => Some(Packet::Overflow),
                    _ if b & 0x03 != 0 => {
                        let len = [0, 1, 2, 4][(b & 0x03) as usize];
                        self.state = State::Payload { hw: b & 0x04 != 0, addr: b >> 3, len, got: 0, value: 0 };
                        None
                    }
                    // Local timestamp (xxxx0000), extension (xxxx1x00) and
                    // global timestamp (10x10100) headers.
                    _ if b & 0x0F == 0x00 || b & 0x0B == 0x08 || b == 0x94 || b == 0xB4 => {
                        if b & 0x80 != 0 {
                            self.state = State::Continuation;
                        }
                        Some(Packet::Other)
                    }
                    _ => Some(Packet::Invalid(b)),
                }
            }
        }
    }
}

/// The probe's SWO serial port for this deck.
pub fn swo_port(probe: &DeviceInfo) -> Result<String> {
    let serial = probe.serial_number().unwrap_or("");
    let ports = serialport::available_ports().unwrap_or_default();
    ports
        .into_iter()
        .find(|p| match &p.port_type {
            serialport::SerialPortType::UsbPort(u) => {
                u.vid == VID
                    && u.pid == PID_PROBE
                    && u.serial_number.as_deref() == Some(serial)
                    && u.interface == Some(SWO_CDC_ITF)
            }
            _ => false,
        })
        .map(|p| p.port_name)
        .ok_or_else(|| {
            Error::Connection(format!(
                "the probe {} has no SWO serial port (is the cdc_acm driver bound?)",
                serial
            ))
            .into()
        })
}

/// Where port SWDn is on USB.
pub fn port_name(swd: u8) -> &'static str {
    match swd {
        1 => "SWD1 (P1)",
        _ => "SWD2 (P5)",
    }
}

/// One CMSIS-DAP command/response on an open DAP interface.
pub fn dap(intf: &nusb::Interface, cmd: &[u8]) -> Result<Vec<u8>> {
    let timeout = Duration::from_millis(500);
    // Interface n has OUT endpoint 0x04 + 2n and IN endpoint 0x85 + 2n.
    let n = intf.interface_number();
    let mut out = intf.endpoint::<Bulk, Out>(0x04 + 2 * n)?;
    out.transfer_blocking(cmd.to_vec().into(), timeout).status?;
    let mut inp = intf.endpoint::<Bulk, In>(0x85 + 2 * n)?;
    let c = inp.transfer_blocking(inp.allocate(512), timeout);
    c.status?;
    let r = c.buffer[..c.actual_len].to_vec();
    if r.first() != Some(&cmd[0]) {
        bail!("CMSIS-DAP command 0x{:02x} got reply {:02x?}", cmd[0], r);
    }
    Ok(r)
}

/// Put the target's SWJ-DP in SWD mode and read DPIDR. Leaves the target
/// running and the probe's pins parked.
pub fn switch_to_swd(intf: &nusb::Interface) -> Result<u32> {
    let r = dap(intf, &[0x02, 1])?; // DAP_Connect, SWD
    if r.get(1) != Some(&1) {
        bail!("the probe refused to connect in SWD mode");
    }
    let result = (|| {
        dap(intf, &[0x11, 0x40, 0x42, 0x0F, 0x00])?; // DAP_SWJ_Clock 1 MHz
        let reset = [0x12, 56, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        dap(intf, &reset)?; // line reset
        dap(intf, &[0x12, 16, 0x9E, 0xE7])?; // JTAG-to-SWD
        dap(intf, &reset)?;
        dap(intf, &[0x12, 8, 0x00])?; // idle
        let r = dap(intf, &[0x05, 0, 1, 0x02])?; // DAP_Transfer: read DPIDR
        if r.len() < 7 || r[1] != 1 || r[2] != 1 {
            bail!("the target did not answer on SWD (ack {:?})", r.get(2));
        }
        Ok(u32::from_le_bytes([r[3], r[4], r[5], r[6]]))
    })();
    let _ = dap(intf, &[0x03]); // DAP_Disconnect: parks the pins
    result
}

/// Route port `swd`'s SWO to ACM0, through that port's DAP interface or,
/// when a debugger holds it, any free one.
pub fn route(probe: &DeviceInfo, swd: u8) -> Result<()> {
    let dev = probe.open().wait().context("opening the probe")?;
    let order = std::iter::once(swd).chain((0..DAP_ITFS).filter(|&i| i != swd));
    for itf in order {
        if let Ok(intf) = dev.claim_interface(itf).wait() {
            dap(&intf, &[ROUTE_ACM0_SWO1 + swd - 1])?;
            return Ok(());
        }
    }
    bail!(Error::Connection("every CMSIS-DAP interface of the probe is in use".into()))
}

/// What the keeper thread has to say.
#[derive(Debug, Clone)]
pub enum KeeperEvent {
    /// The probe could not be opened: the target is not switched to SWD.
    CannotOpen(String),
    /// A debugger has the port's DAP interface.
    DebuggerBusy,
    /// The debugger let go: switching the target back to SWD.
    DebuggerReleased,
    SwitchFailed(String),
    /// Claiming the interface failed for another reason: the keeper stops.
    CannotClaim(String),
}

/// Keeps the target in SWD mode: at start, whenever a debugger that held the
/// port (so could have left it in JTAG mode) lets go of it, and when SWO goes
/// quiet. `last_rx` is when SWO data last arrived, in ms since `t0`. Runs until
/// `stop`; meant for a thread of its own.
pub fn keeper(
    probe: DeviceInfo,
    swd: u8,
    stop: Arc<AtomicBool>,
    t0: Instant,
    last_rx: Arc<AtomicU64>,
    mut event: impl FnMut(KeeperEvent),
) {
    let dev = match probe.open().wait() {
        Ok(d) => d,
        Err(e) => return event(KeeperEvent::CannotOpen(e.to_string())),
    };
    let mut need_switch = true;
    let mut busy = false;
    let mut last_switch = t0;
    while !stop.load(Ordering::Relaxed) {
        let quiet = t0.elapsed().saturating_sub(Duration::from_millis(last_rx.load(Ordering::Relaxed)));
        if quiet > KEEPER_SILENCE && last_switch.elapsed() > KEEPER_RETRY {
            need_switch = true;
        }
        match dev.claim_interface(swd).wait() {
            Ok(intf) => {
                if busy {
                    event(KeeperEvent::DebuggerReleased);
                }
                busy = false;
                if need_switch {
                    last_switch = Instant::now();
                    match switch_to_swd(&intf) {
                        Ok(_) => need_switch = false,
                        Err(e) => event(KeeperEvent::SwitchFailed(format!("{:#}", e))),
                    }
                }
            }
            Err(e) if e.kind() == nusb::ErrorKind::Busy => {
                if !busy {
                    event(KeeperEvent::DebuggerBusy);
                }
                busy = true;
                need_switch = true;
            }
            Err(e) => return event(KeeperEvent::CannotClaim(e.to_string())),
        }
        std::thread::sleep(KEEPER_POLL);
    }
}

/// Packet counts for a summary.
#[derive(Debug, Default, Clone, Copy)]
pub struct Counts {
    pub bytes: u64,
    pub stimulus: u64,
    pub hardware: u64,
    pub overflow: u64,
    pub invalid: u64,
}

impl Counts {
    /// Count one decoded packet. Joining a live stream mid-packet looks like
    /// invalid headers until the first packet decodes, so those don't count.
    pub fn packet(&mut self, p: &Packet) {
        match p {
            Packet::Stimulus { .. } => self.stimulus += 1,
            Packet::Hardware { .. } => self.hardware += 1,
            Packet::Overflow => self.overflow += 1,
            Packet::Invalid(_) if self.stimulus + self.hardware > 0 => self.invalid += 1,
            Packet::Invalid(_) | Packet::Other => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode(bytes: &[u8]) -> Vec<Packet> {
        let mut itm = Itm::new();
        bytes.iter().filter_map(|&b| itm.push(b)).collect()
    }

    #[test]
    fn stimulus_packets_of_each_size() {
        assert_eq!(
            decode(&[0x01, b'H', 0x0A, 0x34, 0x12, 0x0B, 0x78, 0x56, 0x34, 0x12]),
            vec![
                Packet::Stimulus { port: 0, len: 1, value: b'H' as u32 },
                Packet::Stimulus { port: 1, len: 2, value: 0x1234 },
                Packet::Stimulus { port: 1, len: 4, value: 0x1234_5678 },
            ]
        );
    }

    #[test]
    fn sync_overflow_and_hardware() {
        assert_eq!(
            decode(&[0, 0, 0, 0, 0, 0x80, 0x70, 0x0F, 1, 2, 3, 4, 0x01, b'x']),
            vec![
                Packet::Overflow,
                Packet::Hardware { id: 1, len: 4, value: 0x0403_0201 },
                Packet::Stimulus { port: 0, len: 1, value: b'x' as u32 },
            ]
        );
    }

    #[test]
    fn timestamps_skip_their_continuation_bytes() {
        // Local timestamp with two continuation bytes, then a 1-byte write
        // whose payload (0x80) must not be taken for a header.
        assert_eq!(
            decode(&[0xC0, 0x85, 0x01, 0x01, 0x80]),
            vec![Packet::Other, Packet::Stimulus { port: 0, len: 1, value: 0x80 }]
        );
    }

    #[test]
    fn invalid_header() {
        assert_eq!(decode(&[0x04]), vec![Packet::Invalid(0x04)]);
    }
}
