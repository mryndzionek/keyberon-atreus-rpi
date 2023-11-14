# `keyberon-atreus-rpi`

[![Rust](https://github.com/mryndzionek/keyberon-atreus-rpi/actions/workflows/rust.yml/badge.svg)](https://github.com/mryndzionek/keyberon-atreus-rpi/actions/workflows/rust.yml)

> [Keyberon](https://github.com/TeXitoi/keyberon) configuration on RP2040
for my Atreus-like keyboards, using [`probe-run`] + [`defmt`] + [`flip-link`].
STM32F1 (Bluepill) version [here](https://github.com/mryndzionek/keyberon-atreus).

### Build

```
cargo run --release --bin keyberon-atreus-rpi
```

[`probe-run`]: https://crates.io/crates/probe-run
[`defmt`]: https://github.com/knurling-rs/defmt
[`flip-link`]: https://github.com/knurling-rs/flip-link

