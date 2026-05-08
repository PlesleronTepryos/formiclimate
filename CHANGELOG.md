# Changelog

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
