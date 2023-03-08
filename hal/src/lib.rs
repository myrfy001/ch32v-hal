#![no_std]

pub mod prelude;

pub mod rcc;
pub mod afio;
pub mod gpio;

#[cfg(feature = "device-selected")]
use embedded_hal as hal;

#[cfg(feature = "ch32v307")]
/// Re-export of the [svd2rust](https://crates.io/crates/svd2rust) auto-generated API for the ch32v307 peripherals.
pub use ch32v3 as pac; 


mod sealed {
    pub trait Sealed {}
}

pub(crate) use sealed::Sealed;