# bugslayer-lib

Host-side Rust library for the Bugslayer deck (crate `bugslayer`), shared by
[bsly](../bugslayer-cli) and [bugslayer-ui](../bugslayer-ui). It speaks the
v0 protocol of bugslayer-deck-firmware (`docs/protocol.md` there).

Nothing here prints or prompts. Where a front end has to decide something
halfway, the library returns the question: `device::DeckChoice::Several`
(which deck?), `deckctrl::pick` returning `None` (which controller?),
`Error::Unpowered` (switch the port on?). Progress it would have printed goes
to a callback (`update::Note`, `swo::KeeperEvent`) or comes back as notes
(`bus::Bus::notes`).

| Module | What |
|---|---|
| `device` | Finding decks (DB11/DB12/DB13 pairing), the ASCII control channel, UART bridge and expansion-port USB lookup |
| `stream` | Capture block format and the session verifier |
| `pipe` | The capture stream's USB reader thread (RP2350 bulk IN or FX2 EP6) |
| `sigrok` | Streaming `.sr` writer and reader |
| `spi` | sck8 SPI decode, timed from raw16 CS windows (raw16 behind the `Raw16` trait) |
| `uart` | 8N1 receiver on sampled levels |
| `bus` | Expansion-port I2C with the deck as master; port power rules |
| `deckctrl` | DeckCtrl enumeration, info page, registers, GPIOs |
| `swo` | ITM decoder, probe SWO routing, the SWD-mode keeper |
| `github`, `update` | Firmware releases on GitHub and UF2 install through the USB bootloaders |
| `error` | `Error` kinds, `hint` for common USB failures |

```
cargo test
```
