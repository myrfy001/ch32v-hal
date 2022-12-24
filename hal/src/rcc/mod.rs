//! Clock configuration.
//!
//! This module provides functionality to configure the RCC to generate the requested clocks.
//!
//! # Example
//!
//! ```
//! let dp = pac::Peripherals::take().unwrap();
//! let rcc = dp.RCC.constrain();
//! let clocks = rcc
//!     .cfgr
//!     .use_hse(8.MHz())
//!     .sysclk(168.MHz())
//!     .pclk1(24.MHz())
//!     .freeze();
//! ```


// TODO ADD Support For I2S


use crate::pac::{rcc, RCC};

use fugit::HertzU32 as Hertz;
use fugit::RateExtU32;


use pll::MainPll;



mod pll;

mod enable;
mod consts;
use crate::pac::rcc::RegisterBlock as RccRB;

/// Bus associated to peripheral
pub trait RccBus: crate::Sealed {
    /// Bus type;
    type Bus;
}

/// Enable/disable peripheral
pub trait Enable: RccBus {
    fn enable(rcc: &RccRB);
    fn disable(rcc: &RccRB);
}

/// Low power enable/disable peripheral
pub trait LPEnable: RccBus {
    fn low_power_enable(rcc: &RccRB);
    fn low_power_disable(rcc: &RccRB);
}

/// Reset peripheral
pub trait Reset: RccBus {
    fn reset(rcc: &RccRB);
}

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

/// Frequency on bus that peripheral is connected in
pub trait BusClock {
    /// Calculates frequency depending on `Clock` state
    fn clock(clocks: &Clocks) -> Hertz;
}

/// Frequency on bus that timer is connected in
pub trait BusTimerClock {
    /// Calculates base frequency of timer depending on `Clock` state
    fn timer_clock(clocks: &Clocks) -> Hertz;
}

impl<T> BusClock for T
where
    T: RccBus,
    T::Bus: BusClock,
{
    fn clock(clocks: &Clocks) -> Hertz {
        T::Bus::clock(clocks)
    }
}

impl<T> BusTimerClock for T
where
    T: RccBus,
    T::Bus: BusTimerClock,
{
    fn timer_clock(clocks: &Clocks) -> Hertz {
        T::Bus::timer_clock(clocks)
    }
}

/// AMBA High-performance Bus 1 (AHB1) registers
pub struct AHB1 {
    _0: (),
}

impl AHB1 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::AHBPCENR {
        &rcc.ahbpcenr
    }

    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::AHBRSTR {
        &rcc.ahbrstr
    }
}


/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::APB1PCENR {
        &rcc.apb1pcenr
    }
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::APB1PRSTR {
        &rcc.apb1prstr
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    #[inline(always)]
    fn enr(rcc: &RccRB) -> &rcc::APB2PCENR {
        &rcc.apb2pcenr
    }
    #[inline(always)]
    fn rstr(rcc: &RccRB) -> &rcc::APB2PRSTR {
        &rcc.apb2prstr
    }
}

impl BusClock for AHB1 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}


impl BusClock for APB1 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.pclk1
    }
}

impl BusClock for APB2 {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.pclk2
    }
}

impl BusTimerClock for APB1 {
    fn timer_clock(clocks: &Clocks) -> Hertz {
        let pclk_mul = if clocks.ppre1 == 1 { 1 } else { 2 };
        Hertz::from_raw(clocks.pclk1.raw() * pclk_mul)
    }
}

impl BusTimerClock for APB2 {
    fn timer_clock(clocks: &Clocks) -> Hertz {
        let pclk_mul = if clocks.ppre2 == 1 { 1 } else { 2 };
        Hertz::from_raw(clocks.pclk2.raw() * pclk_mul)
    }
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {}
    }
}


// TODO use xxx.unwrap_or when it is stable as const fn
macro_rules! const_option_unwrap_or {
    ($val:expr, $default:expr) => {
        if let Some(t) = $val {
            t
        } else {
            $default
        }
    };
}

// TODO use xxx.unwrap_or when it is stable as const fn
macro_rules! const_option_unwrap {
    ($val:expr) => {
        if let Some(t) = $val {
            t
        } else {
            panic!();
        }
    };
}

/// Constrained RCC peripheral
/// TODO Can we remove this empty struct?
pub struct Rcc {}

/// Built-in high speed clock frequency
pub const HSI: u32 = 8_000_000; // Hz

#[cfg(any(
    feature = "ch32v307",
))]
/// Minimum system clock frequency
// TODO Check the following freq from user manual! 
pub const SYSCLK_MIN: u32 = 24_000_000;


#[cfg(feature = "ch32v307")]
/// Maximum system clock frequency
pub const SYSCLK_MAX: u32 = 144_000_000;


/// Maximum APB2 peripheral clock frequency
pub const PCLK2_MAX: u32 = SYSCLK_MAX;

/// Maximum APB1 peripheral clock frequency
pub const PCLK1_MAX: u32 = PCLK2_MAX;

pub struct ClockCalculator {
    hse: Option<u32>,
    hse_bypass: bool,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,

    pll_cfg: Option<MainPll>,
    ahb_prediv_bits: u8,
    apb1_prediv_bits: u8,
    apb2_prediv_bits: u8,
}

impl ClockCalculator {

    pub const fn new() -> Self {
        Self {
            hse: None,
            hse_bypass: false,
            hclk: None,
            pclk1: None,
            pclk2: None,
            sysclk: None,

            pll_cfg: None,
            ahb_prediv_bits: 0,
            apb1_prediv_bits: 0,
            apb2_prediv_bits: 0,
        }
    }

    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    pub const fn use_hse(mut self, freq: Hertz) -> Self {
        self.hse = Some(freq.raw());
        self
    }

    /// Bypasses the high-speed external oscillator and uses an external clock input on the OSC_IN
    /// pin.
    ///
    /// For this configuration, the OSC_IN pin should be connected to a clock source with a
    /// frequency specified in the call to use_hse(), and the OSC_OUT pin should not be connected.
    ///
    /// This function has no effect unless use_hse() is also called.
    pub const fn bypass_hse_oscillator(self) -> Self {
        Self {
            hse_bypass: true,
            ..self
        }
    }

    pub const fn hclk(mut self, freq: Hertz) -> Self {
        self.hclk = Some(freq.raw());
        self
    }

    pub const fn pclk1(mut self, freq: Hertz) -> Self {
        self.pclk1 = Some(freq.raw());
        self
    }

    pub const fn pclk2(mut self, freq: Hertz) -> Self {
        self.pclk2 = Some(freq.raw());
        self
    }

    pub const fn sysclk(mut self, freq: Hertz) -> Self {
        self.sysclk = Some(freq.raw());
        self
    }


    
    /// Calculate the hardware setting bits.
    /// ### Note: 
    /// This is a const function and it's designed to be calculate during compiling
    /// ### Panic: 
    /// will panic if can't find a solution for the desired clock frequency.
    pub const fn freeze(self) -> Self {
        self.freeze_internal()
    }

    

    const fn freeze_internal(mut self) -> Self {

        
        let pllsrcclk = const_option_unwrap_or!(self.hse, HSI);

        if let None = self.sysclk {
            self.sysclk = Some(pllsrcclk);
        }

        let sysclk = const_option_unwrap!(self.sysclk);


        let main_pll = const_option_unwrap!(MainPll::calc_fast_setup(pllsrcclk, self.hse.is_some(), sysclk));
        
        // TODO support bypass pll by setting this to None
        self.pll_cfg = Some(main_pll);
        



        let hclk = const_option_unwrap_or!(self.hclk, sysclk);
        let hpre_bits = if hclk == sysclk {
            0b0000
        } else {
            let (t,_) = const_option_unwrap!(consts::calc_pll_param(sysclk, hclk, &consts::AHB_PREDIV_TABLE, &consts::NO_OP_DIV_MUL_FACTOR));
            t.cfg_bit
        };
        self.ahb_prediv_bits = hpre_bits as u8;


        let pclk1 = const_option_unwrap_or!(self.pclk1, hclk);
        
        let ppre1_bits = if pclk1 == hclk {
            0b000
        } else {
            let (t,_) = const_option_unwrap!(consts::calc_pll_param(hclk, pclk1, &consts::APB_PREDIV_TABLE, &consts::NO_OP_DIV_MUL_FACTOR));
            t.cfg_bit
        };
        self.apb1_prediv_bits = ppre1_bits as u8;


        let pclk2 = const_option_unwrap_or!(self.pclk2, hclk);

        let ppre2_bits = if pclk2 == hclk {
            0b000
        } else {
            let (t,_) = const_option_unwrap!(consts::calc_pll_param(hclk, pclk2, &consts::APB_PREDIV_TABLE, &consts::NO_OP_DIV_MUL_FACTOR));
            t.cfg_bit
        };
        self.apb2_prediv_bits = ppre2_bits as u8;

        self
    }
}

impl Rcc {
    /// Initialises the hardware according to CFGR state returning a Clocks instance.
    pub fn do_setup(self, calculator: &ClockCalculator) -> Clocks {

        let rcc = unsafe { &*RCC::ptr() };


        if calculator.hse.is_some() {
            // enable HSE and wait for it to be ready
            rcc.ctlr.modify(|_, w| {
                if calculator.hse_bypass {
                    w.hsebyp().set_bit();
                }
                w.hseon().set_bit()
            });
            while rcc.ctlr.read().hserdy().bit_is_clear() {}
        }

        if let Some(pll_cfg) = &calculator.pll_cfg {
            pll_cfg.fast_setup();

            // Enable PLL
            rcc.ctlr.modify(|_, w| w.pllon().set_bit());

            // Wait for PLL to stabilise
            while rcc.ctlr.read().pllrdy().bit_is_clear() {}
        }

        // Set scaling factors
        rcc.cfgr0.modify(|_, w| unsafe {
            w.ppre2()
                .bits(calculator.apb2_prediv_bits)
                .ppre1()
                .bits(calculator.apb1_prediv_bits)
                .hpre()
                .bits(calculator.ahb_prediv_bits)
        });

        // TODO: Check the following line, is it true on qingke-v4 core?
        // Wait for the new prescalers to kick in
        // "The clocks are divided with the new prescaler factor from 1 to 16 AHB cycles after write");
        unsafe {riscv::asm::delay(16)};

        // Select system clock source
        rcc.cfgr0.modify(|_, w| {
            unsafe{
                w.sw().bits(if calculator.pll_cfg.is_some() {
                    0b10  // PLL
                } else if calculator.hse.is_some() {
                    0b01  // HSE
                } else {
                    0b00  // HSI
                })
            }
        });

        let sysclk = calculator.sysclk.unwrap();
        let hclk = calculator.hclk.unwrap_or(sysclk);
        let pclk1 = calculator.pclk1.unwrap_or(hclk);
        let pclk2 = calculator.pclk2.unwrap_or(hclk);

        let clocks = Clocks {
            hclk: hclk.Hz(),
            pclk1: pclk1.Hz(),
            pclk2: pclk2.Hz(),
            ppre1: (pclk1 / hclk) as u8,
            ppre2: (pclk2 / hclk) as u8,
            sysclk: sysclk.Hz(),
        };

        clocks
    }
}



/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
}

impl Clocks {
    /// Returns the frequency of the AHB1
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the prescaler of the APB1
    pub fn ppre1(&self) -> u8 {
        self.ppre1
    }

    /// Returns the prescaler of the APB2
    pub fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }


}

