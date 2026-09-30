// The capture stream's USB pipe: read the block stream from the RP2350's
// vendor bulk IN (Full Speed, ~380 ksps) or the FX2's EP6 (High Speed,
// ~17 Msps) on a thread of its own with a queue of transfers always in flight,
// so whatever consumes it never stalls the pipe: at 35 MB/s the FX2's FIFO
// covers only ~60 us.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::Duration;

use anyhow::{Context, Result};
use nusb::transfer::{Bulk, In, TransferError};
use nusb::MaybeFuture;

use crate::stream::BLOCK_SIZE;

/// The RP2350's own Full-Speed sink tops out around 768 kB/s.
pub const USB_MAX_RATE: u32 = 380_000;

const USB_ITF: u8 = 2;
const USB_EP: u8 = 0x83;
const FX2_ITF: u8 = 0;
const FX2_EP: u8 = 0x86;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sink {
    Usb,
    Fx2,
}

impl Sink {
    pub fn name(self) -> &'static str {
        match self {
            Sink::Usb => "usb",
            Sink::Fx2 => "fx2",
        }
    }
}


/// What the reader thread delivers: whole blocks, or a transfer error.
pub enum Msg {
    Data(Vec<u8>),
    Error(String),
}

/// A running reader: set `stop`, then join `thread` for the byte count of
/// short packets it dropped (an earlier session's PKTEND flush).
pub struct Reader {
    pub stop: Arc<AtomicBool>,
    pub thread: std::thread::JoinHandle<u64>,
    pub rx: mpsc::Receiver<Msg>,
}

/// Start reading `ep` of `itf`. Returns once the transfers are queued.
pub fn start_reader(info: &nusb::DeviceInfo, sink: Sink) -> Result<Reader> {
    let (itf, ep, n_xfers, xfer_size) = match sink {
        // One block per transfer: at Full Speed a block is 8 packets, so a
        // transfer completes after every block and nothing waits in a
        // part-filled one.
        Sink::Usb => (USB_ITF, USB_EP, 32, BLOCK_SIZE),
        // One packet is one block. The deck ends a transfer with a
        // zero-length packet whenever it goes quiet, so no timeout is needed
        // (cancelling a part-filled High Speed transfer can lose packets).
        Sink::Fx2 => (FX2_ITF, FX2_EP, 64, 32 * BLOCK_SIZE),
    };
    let dev = info.open().wait().context("opening the capture USB device")?;
    let intf = dev.claim_interface(itf).wait().context("claiming the capture interface")?;
    let mut ep = intf.endpoint::<Bulk, In>(ep).context("opening the capture endpoint")?;
    let stop = Arc::new(AtomicBool::new(false));
    let (tx, rx) = mpsc::channel();
    for _ in 0..n_xfers {
        let b = ep.allocate(xfer_size);
        ep.submit(b);
    }
    let stop2 = stop.clone();
    let thread = std::thread::spawn(move || {
        let _intf = intf; // keep the interface claimed while reading
        let mut carry: Vec<u8> = Vec::new();
        let mut short: u64 = 0;
        let mut handle = |data: &[u8], tx: &mpsc::Sender<Msg>| match sink {
            Sink::Fx2 => {
                // A transfer is whole blocks plus at most one short packet:
                // the deck's PKTEND flush of an earlier session's partial
                // block. It is never part of this session; drop it.
                let n = data.len() / BLOCK_SIZE * BLOCK_SIZE;
                short += (data.len() - n) as u64;
                if n > 0 {
                    let _ = tx.send(Msg::Data(data[..n].to_vec()));
                }
            }
            Sink::Usb => {
                carry.extend_from_slice(data);
                let n = carry.len() / BLOCK_SIZE * BLOCK_SIZE;
                if n > 0 {
                    let _ = tx.send(Msg::Data(carry.drain(..n).collect()));
                }
            }
        };
        while !stop2.load(Ordering::Relaxed) {
            let Some(c) = ep.wait_next_complete(Duration::from_millis(100)) else { continue };
            match c.status {
                Ok(()) => handle(&c.buffer[..c.actual_len], &tx),
                Err(TransferError::Disconnected) => {
                    let _ = tx.send(Msg::Error("the capture device disconnected".into()));
                    return short;
                }
                Err(e) => {
                    let _ = tx.send(Msg::Error(format!("transfer error: {}", e)));
                }
            }
            let mut b = c.buffer;
            b.clear();
            ep.submit(b);
        }
        ep.cancel_all();
        while ep.pending() > 0 {
            if ep.wait_next_complete(Duration::from_secs(1)).is_none() {
                break;
            }
        }
        short
    });
    Ok(Reader { stop, thread, rx })
}
