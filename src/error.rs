use std::fmt;

/// Classified failures, so a front end can react by kind (bscli maps them to
/// its exit codes). The message is best-effort and can change.
#[derive(Debug)]
pub enum Error {
    Connection(String),
    NotFound(String),
    /// The deck answered `err ...`, or the request is not allowed.
    Rejected(String),
    Timeout(String),
    Verification(String),
    /// The expansion port has no power and the caller did not allow switching
    /// it on: ask, then call again with power allowed.
    Unpowered(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Connection(s) => write!(f, "connection error: {}", s),
            Error::NotFound(s) => write!(f, "not found: {}", s),
            Error::Rejected(s) => write!(f, "the deck rejected the command: {}", s),
            Error::Timeout(s) => write!(f, "timeout: {}", s),
            Error::Verification(s) => write!(f, "verification failed: {}", s),
            Error::Unpowered(s) => write!(f, "unpowered: {}", s),
        }
    }
}

impl std::error::Error for Error {}

/// The library error in `err`'s chain, if any.
pub fn find(err: &anyhow::Error) -> Option<&Error> {
    err.downcast_ref::<Error>().or_else(|| err.chain().find_map(|c| c.downcast_ref::<Error>()))
}

/// Extra guidance for failures with a known way out.
pub fn hint(err: &anyhow::Error) -> Option<&'static str> {
    for cause in err.chain() {
        if let Some(e) = cause.downcast_ref::<nusb::Error>() {
            if e.kind() == nusb::ErrorKind::PermissionDenied {
                return Some(
                    "no access to the deck's USB devices; install the udev rules from \
                     bugslayer-deck-firmware/host/99-bugslayer-deck.rules",
                );
            }
            if e.kind() == nusb::ErrorKind::Busy {
                return Some("another program (bsly.py? PulseView?) has the interface claimed");
            }
        }
    }
    None
}
