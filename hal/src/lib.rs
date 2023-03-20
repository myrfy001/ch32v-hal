#![no_std]

pub mod prelude;

#[cfg(feature = "device-selected")]
pub mod rcc;
#[cfg(feature = "device-selected")]
pub mod afio;
#[cfg(feature = "device-selected")]
pub mod gpio;
#[cfg(feature = "device-selected")]
pub mod serial;
#[cfg(feature = "device-selected")]
pub mod dma;
#[cfg(feature = "device-selected")]
pub mod timer;
#[cfg(feature = "device-selected")]
pub mod time;

#[cfg(feature = "device-selected")]
use embedded_hal as hal;

#[cfg(feature = "ch32v307")]
/// Re-export of the [svd2rust](https://crates.io/crates/svd2rust) auto-generated API for the ch32v307 peripherals.
pub use ch32v3::ch32v30x as pac; 


mod sealed {
    pub trait Sealed {}
}

pub(crate) use sealed::Sealed;