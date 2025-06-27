# ADS1220

A `#![no_std]` Rust driver library for interacting with TI [ADS1220](https://www.ti.com/product/ADS1220) Delta-Sigma ADC chip.

## Cargo Features

All features are disabled by default.

- `defmt` - Implements `defmt::Format` for most public types so they can be printed using `defmt::info!()` and relatives
- `embedded-hal-async` - Provides async implementations of all the ADS1220 functions
