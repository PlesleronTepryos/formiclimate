# Changelog

## 0.3.0 - 2026.05.17

### Added

- `page!` macro: static page layout specification with compile-time cursor tracking and overflow
  detection; see the macro's documentation for supported commands; can be used to create a new page
  or rewrite an existing one in-place
- `utils.rs`: `f32_to_bytes`, `u16_to_bytes`, `i16_to_bytes` number formatting functions
- Fleshed out `display::PageData`: `blank()`, `new()`, `clear()`, `write_byte()`, `write_bytes()`,
  `into_data()`, `end_line_pos()`, `next_line_pos()` methods
- `README.md`: added [LLM Use Disclosure](/README.md#llm-use-disclosure)

### Changed

- `Display` is now double-buffered instead of needing to be sent an entire new page by value every
  frame
- `Day::name` and `Month::name` now return `&[u8; 9]` instead of `&str`
- `Day::abbrev` and `Month::abbrev` now return `&[u8; 3]` instead of `&str`
- `Date::suffix` now returns `&[u8; 2]` instead of `&[u8]`
- `ControllerConfig::diapause_status` now returns `&[u8; 11]` instead of `&str`;
  several status labels renamed for clarity

### Removed

- `PageBuilder` struct (replaced by `page!`)

## 0.2.2 - 2026.05.09

### Fixed

- Missing exit string for `ControllerConfig`

## 0.2.1 - 2026.05.08

### Added

- Added changelog

### Changed

- Reworked macros for ergonomics and factored them into dedicated `codegen` module
- Assorted minor refactoring and optimization

## 0.2.0 - 2026.05.07

Note: changes listed here likely incomplete

### Added

- `encoder.rs`: input handling for rotary encoder w/ push button
- `utils.rs`: assorted free functions that were prevously scattered across the crate
- Interactive UI (using encoder w/ button)
- The user can now edit configuration fields while the device is running
- New manual control mode
- Dedicated `Relay` abstraction with fault detection/correction
- Methods for `PageBuilder` to display larger integers

### Changed

- Reworked temperature control logic to be less janky and more efficient overall
- Renamed `pwm` module to `control`
- Heavy optimization across the board, primarily to keep binary size low
- Refactored `Sensorium` to mitigate monomorphization bloat

#### Hardware Changes

- Overhauled coolant loop and rebuilt heat exchanger to run the coolant in direct contact with the evaporator

### Removed

- Removed evaporator temperature sensor as there is no longer a heat exchange block to monitor
- Removed all instances of floating point division, instead using `utils::recip`

### Fixed

- A lot of bugs

## 0.1.1 - 2026.05.15

### Fixed

- Mishandling of 12th hour of day in target temperature calculation
- Compressor relay would not reliably switch on, leading to control logic locking up

## <=0.1.0

Changes not recorded beyond this point, sorry
