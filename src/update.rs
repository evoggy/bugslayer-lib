// Firmware updates: compare the deck's firmware with the latest GitHub releases
// and install new UF2s through the chips' USB bootloaders.
//
// The RP2350 reboots into its bootloader on `bootsel`, the probe (RP2040) on
// CMSIS-DAP vendor command 0x9F. Firmware older than that needs the button:
// SW2 held while plugging USB in, which puts *both* chips in their bootloaders.
// Either way the chip shows up as a USB drive and the UF2 is copied onto it.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use nusb::{DeviceInfo, MaybeFuture};

use crate::device::{self, Control, Deck};
use crate::error::Error;
use crate::github::{self, Release};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    Rp2350,
    Probe,
}

impl Chip {
    pub fn name(self) -> &'static str {
        match self {
            Chip::Rp2350 => "rp2350",
            Chip::Probe => "probe",
        }
    }
    pub fn repo(self) -> &'static str {
        match self {
            Chip::Rp2350 => "bugslayer-deck-firmware",
            Chip::Probe => "bugslayer-probe-firmware",
        }
    }
    pub fn asset_prefix(self) -> &'static str {
        match self {
            Chip::Rp2350 => "bugslayer-rp2350-",
            Chip::Probe => "bugslayer-probe-",
        }
    }
    /// `Board-ID:` in the bootloader drive's INFO_UF2.TXT, and its volume label.
    pub fn board_id(self) -> &'static str {
        match self {
            Chip::Rp2350 => "RP2350",
            Chip::Probe => "RPI-RP2",
        }
    }
}

/// The RP2350 firmware version, from `ver`.
pub fn rp2350_version(deck: &Deck) -> Result<String> {
    let mut ctl = Control::open(deck, false)?;
    let ver = device::kv(&ctl.command("ver")?);
    ver.get("rp2350").cloned().ok_or_else(|| anyhow::anyhow!("no rp2350= in the `ver` reply"))
}

/// One CMSIS-DAP command on whichever of the probe's interfaces is free.
pub fn probe_command(probe: &DeviceInfo, cmd: &[u8]) -> Result<Vec<u8>> {
    let dev = probe.open().wait().context("opening the probe")?;
    for itf in 0..4 {
        if let Ok(intf) = dev.claim_interface(itf).wait() {
            return crate::swo::dap(&intf, cmd);
        }
    }
    bail!(Error::Connection("every CMSIS-DAP interface of the probe is in use".into()))
}

/// DAP_Info ID 9, the product firmware version. Empty on firmware from before
/// the split into bugslayer-probe-firmware.
pub fn probe_version(probe: &DeviceInfo) -> Result<Option<String>> {
    let r = probe_command(probe, &[0x00, 0x09])?;
    let len = *r.get(1).unwrap_or(&0) as usize;
    if len == 0 || r.len() < 2 + len {
        return Ok(None);
    }
    Ok(Some(String::from_utf8_lossy(&r[2..2 + len]).trim_end_matches('\0').to_string()))
}

/// A `git describe` build between releases: `0.8.0-3-gabc1234[-dirty]`.
pub fn is_dev_build(v: &str) -> bool {
    v.contains("-g") || v.ends_with("-dirty")
}

/// Whether `latest` should replace `installed`.
pub fn outdated(installed: Option<&str>, latest: &semver::Version) -> bool {
    match installed.map(semver::Version::parse) {
        None => true,
        Some(Err(_)) => true,
        Some(Ok(v)) => v < *latest && !is_dev_build(installed.unwrap()),
    }
}

pub fn describe(installed: Option<&str>) -> String {
    match installed {
        None => "unknown (older than the first release)".into(),
        Some(v) if is_dev_build(v) => format!("{} (development build)", v),
        Some(v) => v.to_string(),
    }
}

/// What a chip runs, what GitHub has, and whether it should be updated.
#[derive(Debug, Clone)]
pub struct Status {
    pub chip: Chip,
    /// None when the chip cannot say (probe firmware from before versions
    /// were reported) or is not reachable.
    pub installed: Option<String>,
    /// The release to compare with: `tag` if given, else the latest (None when
    /// the repository has no release yet).
    pub release: Option<Release>,
    /// The release is newer than what is installed.
    pub stale: bool,
    /// The chip is not connected (no deck, or no probe on its hub).
    pub missing: bool,
}

/// The installed version of `chip`, if the deck can say.
pub fn installed(deck: Option<&Deck>, chip: Chip) -> Option<String> {
    match chip {
        Chip::Rp2350 => deck.and_then(|d| rp2350_version(d).ok()),
        Chip::Probe => deck.and_then(|d| d.probe.as_ref()).and_then(|p| probe_version(p).ok().flatten()),
    }
}

/// Compare `chip` with its release on GitHub (`tag`, or the latest one).
pub fn status(deck: Option<&Deck>, chip: Chip, tag: Option<&str>, prerelease: bool) -> Result<Status> {
    let have = installed(deck, chip);
    let release = match tag {
        Some(t) => Some(github::release_by_tag(chip.repo(), t)?),
        None => github::latest(chip.repo(), prerelease)?,
    };
    let stale = match &release {
        Some(r) => outdated(have.as_deref(), &r.version().context("release tag is not a version")?),
        None => false,
    };
    let missing = deck.is_none() || (chip == Chip::Probe && deck.and_then(|d| d.probe.as_ref()).is_none());
    Ok(Status { chip, installed: have, release, stale, missing })
}

/// The release's UF2 for `chip`: (asset name, size), then its bytes.
pub fn uf2_asset(chip: Chip, release: &Release) -> Result<&github::Asset> {
    release.asset(chip.asset_prefix(), ".uf2").ok_or_else(|| {
        Error::NotFound(format!("a {}*.uf2 in release {}", chip.asset_prefix(), release.tag_name)).into()
    })
}

pub fn download(chip: Chip, release: &Release) -> Result<Vec<u8>> {
    github::download(chip.repo(), uf2_asset(chip, release)?)
}

/// Progress from `install` that a front end shows.
#[derive(Debug, Clone)]
pub enum Note {
    /// The chip cannot reboot into its bootloader by itself: the user has to
    /// unplug the deck's USB, hold SW2, plug USB back in, then release SW2.
    /// `install` waits up to 3 minutes for the drive.
    PressButton(Chip),
    /// Copying the UF2 onto this bootloader drive.
    Copying(Chip, PathBuf),
}

/// Whether the RP2350 is still in its bootloader after the installs (the SW2
/// button puts both chips there): the deck's USB needs replugging.
pub fn rp2350_in_bootloader() -> bool {
    find_drive(Chip::Rp2350).is_some()
}

/// Get `chip` into its bootloader, copy the UF2 and wait for the drive to go.
pub fn install(deck: Option<&Deck>, chip: Chip, uf2: &[u8], mut note: impl FnMut(Note)) -> Result<()> {
    let drive = match find_drive(chip) {
        Some(d) => d, // already in the bootloader (e.g. after the button)
        None => {
            let asked = match (chip, deck) {
                (Chip::Rp2350, Some(d)) => {
                    let mut ctl = Control::open(d, false)?;
                    ctl.command("bootsel")?.starts_with("bootsel ok")
                }
                (Chip::Probe, Some(Deck { probe: Some(p), .. })) => probe_command(p, &[0x9F]).is_ok(),
                _ => false,
            };
            let rebooted = if asked { wait_for_drive(chip, Duration::from_secs(8)) } else { None };
            match rebooted {
                Some(d) => d,
                None => {
                    // Firmware from before the reboot commands. Nothing to
                    // type, only a button to press, so this works in scripts too.
                    note(Note::PressButton(chip));
                    wait_for_drive(chip, Duration::from_secs(180))
                        .ok_or_else(|| Error::Timeout(format!("no {} drive appeared", chip.board_id())))?
                }
            }
        }
    };

    note(Note::Copying(chip, drive.clone()));
    let target = drive.join(format!("bugslayer-{}.uf2", chip.name()));
    let copied = (|| -> std::io::Result<()> {
        let mut f = std::fs::File::create(&target)?;
        f.write_all(uf2)?;
        f.sync_all()
    })();
    // The chip reboots as soon as the last block lands, so the drive can vanish
    // before sync_all returns: only an error while the drive is still there counts.
    let t0 = Instant::now();
    while drive.join("INFO_UF2.TXT").exists() {
        if t0.elapsed() > Duration::from_secs(20) {
            copied.with_context(|| format!("writing {}", target.display()))?;
            bail!(Error::Timeout(format!("{} is still there after the copy", drive.display())));
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    Ok(())
}

/// After the chip restarts: check that it runs `version` (a release tag;
/// anything else, e.g. a file name, is only checked for coming back). Returns
/// the version it runs.
pub fn verify(serial: Option<&str>, probe_serial: Option<&str>, chip: Chip, version: &str) -> Result<String> {
    let want = version;
    let is_tag = semver::Version::parse(version).is_ok();
    let t0 = Instant::now();
    loop {
        let got = match chip {
            Chip::Rp2350 => device::list_decks()?
                .into_iter()
                .find(|d| serial.is_none_or(|s| d.serial == s))
                .filter(|d| d.port.is_some())
                .and_then(|d| rp2350_version(&d).ok()),
            // Found directly: the RP2350 may still be in its bootloader.
            Chip::Probe => device::list_devices()?
                .into_iter()
                .find(|d| {
                    d.vendor_id() == device::VID
                        && d.product_id() == device::PID_PROBE
                        && probe_serial.is_none_or(|s| d.serial_number() == Some(s))
                })
                .and_then(|p| probe_version(&p).ok().flatten()),
        };
        match got {
            Some(v) if v == want || !is_tag => return Ok(v),
            Some(v) if t0.elapsed() > Duration::from_secs(5) => {
                bail!(Error::Rejected(format!("{} came back with {}, not {}", chip.name(), v, want)))
            }
            _ if t0.elapsed() > Duration::from_secs(20) => {
                bail!(Error::Timeout(format!("{} did not come back on USB within 20 s", chip.name())))
            }
            _ => std::thread::sleep(Duration::from_millis(300)),
        }
    }
}

pub fn wait_for_drive(chip: Chip, timeout: Duration) -> Option<PathBuf> {
    let t0 = Instant::now();
    let mut mount_tried = false;
    while t0.elapsed() < timeout {
        if let Some(d) = find_drive(chip) {
            return Some(d);
        }
        // Linux without an automounter: ask udisks, as a desktop would.
        if !mount_tried && cfg!(target_os = "linux") {
            let dev = Path::new("/dev/disk/by-label").join(chip.board_id());
            if dev.exists() {
                mount_tried = true;
                let _ = std::process::Command::new("udisksctl")
                    .args(["mount", "--no-user-interaction", "-b"])
                    .arg(&dev)
                    .output();
            }
        }
        std::thread::sleep(Duration::from_millis(250));
    }
    None
}

/// A mounted UF2 bootloader drive whose INFO_UF2.TXT names `chip`'s board.
pub fn find_drive(chip: Chip) -> Option<PathBuf> {
    let want = format!("Board-ID: {}", chip.board_id());
    mount_points().into_iter().find(|m| {
        std::fs::read_to_string(m.join("INFO_UF2.TXT"))
            .map(|t| t.lines().any(|l| l.trim() == want))
            .unwrap_or(false)
    })
}

fn mount_points() -> Vec<PathBuf> {
    if cfg!(target_os = "linux") {
        // Field 2 of /proc/mounts, with spaces escaped as \040.
        std::fs::read_to_string("/proc/mounts")
            .unwrap_or_default()
            .lines()
            .filter_map(|l| l.split(' ').nth(1))
            .map(|p| PathBuf::from(p.replace("\\040", " ")))
            .collect()
    } else if cfg!(target_os = "macos") {
        std::fs::read_dir("/Volumes").map(|r| r.flatten().map(|e| e.path()).collect()).unwrap_or_default()
    } else {
        ('D'..='Z').map(|c| PathBuf::from(format!("{}:\\", c))).collect()
    }
}
