//! Abstractions for relay-controlled devices

use arduino_hal::port::{
    mode::{Io, Output},
    Pin, PinOps,
};

/// A relay's powered state
pub enum RelayState {
    /// Off/unpowered
    Inactive,

    /// On/powered
    Active,
}

/// A relay controlled by a digital output pin
pub struct Relay<PIN> {
    pin: Pin<Output, PIN>,
    state: RelayState,
}

impl<PIN> Relay<PIN>
where
    PIN: PinOps,
{
    /// Create a new reay in an inactive state
    pub fn new<MODE>(pin: Pin<MODE, PIN>) -> Self
    where
        MODE: Io,
    {
        Self {
            pin: pin.into_output(),
            state: RelayState::Inactive,
        }
    }

    /// Whether the relay is currently active
    pub const fn is_active(&self) -> bool {
        matches!(self.state, RelayState::Active)
    }

    /// Activate the relay
    pub fn activate(&mut self) {
        self.pin.set_high();
        self.state = RelayState::Active;
    }

    /// Deactivate the relay
    pub fn deactivate(&mut self) {
        self.pin.set_low();
        self.state = RelayState::Inactive;
    }
}
