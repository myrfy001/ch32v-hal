//! # Alternate Function I/Os
use crate::pac::{afio, AFIO, RCC};

use crate::rcc::{Enable, Reset};

use crate::gpio::{
    Debugger, Floating, Input, PA15, {PB3, PB4},
};

pub trait AfioExt {
    fn constrain(self) -> Parts;
}

impl AfioExt for AFIO {
    fn constrain(self) -> Parts {
        let rcc = unsafe { &(*RCC::ptr()) };
        AFIO::enable(rcc);
        AFIO::reset(rcc);

        Parts {
            ecr: ECR { _0: () },
            pcfr: PCFR {
                _0: (),
                jtag_enabled: true,
            },
            exticr1: EXTICR1 { _0: () },
            exticr2: EXTICR2 { _0: () },
            exticr3: EXTICR3 { _0: () },
            exticr4: EXTICR4 { _0: () },
            pcfr2: PCFR2 { _0: () },
        }
    }
}

/// HAL wrapper around the AFIO registers
///
/// Aquired by calling [constrain](trait.AfioExt.html#constrain) on the [AFIO
/// registers](../pac/struct.AFIO.html)
///
/// ```rust
/// let p = pac::Peripherals::take().unwrap();
/// let mut rcc = p.RCC.constrain();
/// let mut afio = p.AFIO.constrain();
pub struct Parts {
    pub ecr: ECR,
    pub pcfr: PCFR,
    pub exticr1: EXTICR1,
    pub exticr2: EXTICR2,
    pub exticr3: EXTICR3,
    pub exticr4: EXTICR4,
    pub pcfr2: PCFR2,
}

pub struct ECR {
    _0: (),
}

impl ECR {
    pub fn ecr(&mut self) -> &afio::ECR {
        unsafe { &(*AFIO::ptr()).ecr }
    }
}

/// AF remap and debug I/O configuration register (MAPR)
///
/// Aquired through the [Parts](struct.Parts.html) struct.
///
/// ```rust
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcc = dp.RCC.constrain();
/// let mut afio = dp.AFIO.constrain();
/// function_using_mapr(&mut afio.mapr);
/// ```
pub struct PCFR {
    _0: (),
    jtag_enabled: bool,
}

impl PCFR {
    fn pcfr(&mut self) -> &afio::PCFR {
        unsafe { &(*AFIO::ptr()).pcfr }
    }

    pub fn modify_mapr<F>(&mut self, mod_fn: F)
    where
        F: for<'w> FnOnce(&afio::pcfr::R, &'w mut afio::pcfr::W) -> &'w mut afio::pcfr::W,
    {
        let debug_bits = if self.jtag_enabled { 0b000 } else { 0b010 };
        self.pcfr()
            .modify(unsafe { |r, w| mod_fn(r, w).swcfg().bits(debug_bits) });
    }

    /// Disables the JTAG to free up pa15, pb3 and pb4 for normal use
    #[allow(clippy::redundant_field_names, clippy::type_complexity)]
    pub fn disable_jtag(
        &mut self,
        pa15: PA15<Debugger>,
        pb3: PB3<Debugger>,
        pb4: PB4<Debugger>,
    ) -> (
        PA15<Input<Floating>>,
        PB3<Input<Floating>>,
        PB4<Input<Floating>>,
    ) {
        self.jtag_enabled = false;
        // Avoid duplicating swj_cfg write code
        self.modify_mapr(|_, w| w);

        // NOTE(unsafe) The pins are now in the good state.
        unsafe { (pa15.activate(), pb3.activate(), pb4.activate()) }
    }
}

pub struct EXTICR1 {
    _0: (),
}

impl EXTICR1 {
    pub fn exticr1(&mut self) -> &afio::EXTICR1 {
        unsafe { &(*AFIO::ptr()).exticr1 }
    }
}

pub struct EXTICR2 {
    _0: (),
}

impl EXTICR2 {
    pub fn exticr2(&mut self) -> &afio::EXTICR2 {
        unsafe { &(*AFIO::ptr()).exticr2 }
    }
}

pub struct EXTICR3 {
    _0: (),
}

impl EXTICR3 {
    pub fn exticr3(&mut self) -> &afio::EXTICR3 {
        unsafe { &(*AFIO::ptr()).exticr3 }
    }
}

pub struct EXTICR4 {
    _0: (),
}

impl EXTICR4 {
    pub fn exticr4(&mut self) -> &afio::EXTICR4 {
        unsafe { &(*AFIO::ptr()).exticr4 }
    }
}

pub struct PCFR2 {
    _0: (),
}

impl PCFR2 {
    pub fn pcfr2(&mut self) -> &afio::PCFR2 {
        unsafe { &(*AFIO::ptr()).pcfr2 }
    }
}
