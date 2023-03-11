pub use fugit::ExtU32 as _fugit_DurationExtU32;
pub use fugit::RateExtU32 as _fugit_RateExtU32;

pub use crate::rcc::RccExt as _ch32v_hal_rcc_RccExt;
pub use crate::afio::AfioExt as _stm32_hal_afio_AfioExt;
pub use crate::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
pub use crate::hal::digital::v2::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
pub use crate::hal::digital::v2::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use crate::hal::prelude::*;