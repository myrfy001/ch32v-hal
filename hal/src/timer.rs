/*!
  # Timer

  ## Alternate function remapping

  This is a list of the remap settings you can use to assign pins to PWM channels
  and the QEI peripherals

  ### TIM1

  Not available on STM32F101.

  | Channel | Tim1NoRemap | Tim1FullRemap |
  |:---:|:-----------:|:-------------:|
  | CH1 |     PA8     |       PE9     |
  | CH2 |     PA9     |       PE11    |
  | CH3 |     PA10    |       PE13    |
  | CH4 |     PA11    |       PE14    |

  ### TIM2

  | Channel | Tim2NoRemap | Tim2PartialRemap1 | Tim2PartialRemap2 | Tim2FullRemap |
  |:---:|:-----------:|:-----------------:|:-----------------:|:-------------:|
  | CH1 |     PA0     |        PA15       |        PA0        |      PA15     |
  | CH2 |     PA1     |        PB3        |        PA1        |      PB3      |
  | CH3 |     PA2     |        PA2        |        PB10       |      PB10     |
  | CH4 |     PA3     |        PA3        |        PB11       |      PB11     |

  ### TIM3

  | Channel | Tim3NoRemap | Tim3PartialRemap | Tim3FullRemap |
  |:---:|:-----------:|:----------------:|:-------------:|
  | CH1 |     PA6     |        PB4       |      PC6      |
  | CH2 |     PA7     |        PB5       |      PC7      |
  | CH3 |     PB0     |        PB0       |      PC8      |
  | CH4 |     PB1     |        PB1       |      PC9      |

  ### TIM4

  Not available on low density devices.

  | Channel | Tim4NoRemap | Tim4Remap |
  |:---:|:-----------:|:---------:|
  | CH1 |     PB6     |    PD12   |
  | CH2 |     PB7     |    PD13   |
  | CH3 |     PB8     |    PD14   |
  | CH4 |     PB9     |    PD15   |
*/
#![allow(non_upper_case_globals)]


use crate::pac::{self, RCC};

// TODO: Support This Later
//use crate::pac::DBGMCU as DBG,;



use crate::rcc::{self, Clocks};
use core::convert::TryFrom;
// TODO: Support This Later
use qingke::peripheral::{STK, stk::StkClkSource};

use crate::time::Hertz;

#[cfg(feature = "rtic")]
pub mod monotonic;
#[cfg(feature = "rtic")]
pub use monotonic::*;
pub mod pwm_input;
pub use pwm_input::*;
pub(crate) mod pins;
pub use pins::*;
pub mod delay;
pub use delay::*;
pub mod counter;
pub use counter::*;
pub mod pwm;
pub use pwm::*;

mod hal_02;

/// Timer wrapper
pub struct Timer<TIM> {
    pub(crate) tim: TIM,
    pub(crate) clk: Hertz,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    C1 = 0,
    C2 = 1,
    C3 = 2,
    C4 = 3,
}

/// Interrupt events
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SysEvent {
    /// [Timer] timed out / count down ended
    Update,
}

bitflags::bitflags! {
    pub struct Event: u32 {
        const Update  = 1 << 0;
        const C1 = 1 << 1;
        const C2 = 1 << 2;
        const C3 = 1 << 3;
        const C4 = 1 << 4;
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Error {
    /// Timer is disabled
    Disabled,
    WrongAutoReload,
}

pub trait TimerExt: Sized {
    /// Non-blocking [Counter] with custom fixed precision
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> Counter<Self, FREQ>;
    /// Non-blocking [Counter] with fixed precision of 1 ms (1 kHz sampling)
    ///
    /// Can wait from 2 ms to 65 sec for 16-bit timer and from 2 ms to 49 days for 32-bit timer.
    ///
    /// NOTE: don't use this if your system frequency more than 65 MHz
    fn counter_ms(self, clocks: &Clocks) -> CounterMs<Self> {
        self.counter::<1_000>(clocks)
    }
    /// Non-blocking [Counter] with fixed precision of 1 μs (1 MHz sampling)
    ///
    /// Can wait from 2 μs to 65 ms for 16-bit timer and from 2 μs to 71 min for 32-bit timer.
    fn counter_us(self, clocks: &Clocks) -> CounterUs<Self> {
        self.counter::<1_000_000>(clocks)
    }
    /// Non-blocking [Counter] with dynamic precision which uses `Hertz` as Duration units
    fn counter_hz(self, clocks: &Clocks) -> CounterHz<Self>;

    /// Blocking [Delay] with custom fixed precision
    fn delay<const FREQ: u32>(self, clocks: &Clocks) -> Delay<Self, FREQ>;
    /// Blocking [Delay] with fixed precision of 1 ms (1 kHz sampling)
    ///
    /// Can wait from 2 ms to 49 days.
    ///
    /// NOTE: don't use this if your system frequency more than 65 MHz
    fn delay_ms(self, clocks: &Clocks) -> DelayMs<Self> {
        self.delay::<1_000>(clocks)
    }
    /// Blocking [Delay] with fixed precision of 1 μs (1 MHz sampling)
    ///
    /// Can wait from 2 μs to 71 min.
    fn delay_us(self, clocks: &Clocks) -> DelayUs<Self> {
        self.delay::<1_000_000>(clocks)
    }
}

impl<TIM: Instance> TimerExt for TIM {
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> Counter<Self, FREQ> {
        FTimer::new(self, clocks).counter()
    }
    fn counter_hz(self, clocks: &Clocks) -> CounterHz<Self> {
        Timer::new(self, clocks).counter_hz()
    }
    fn delay<const FREQ: u32>(self, clocks: &Clocks) -> Delay<Self, FREQ> {
        FTimer::new(self, clocks).delay()
    }
}

pub trait SysTimerExt: Sized {
    /// Creates timer which takes [Hertz] as Duration
    fn counter_hz(self, clocks: &Clocks) -> SysCounterHz;

    /// Creates timer with custom precision (core frequency recommended is known)
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> SysCounter<FREQ>;
    /// Creates timer with precision of 1 μs (1 MHz sampling)
    fn counter_us(self, clocks: &Clocks) -> SysCounterUs {
        self.counter::<1_000_000>(clocks)
    }
    /// Blocking [Delay] with custom precision
    fn delay(self, clocks: &Clocks) -> SysDelay;
}

impl SysTimerExt for STK {
    fn counter_hz(self, clocks: &Clocks) -> SysCounterHz {
        Timer::syst_hclk(self, clocks).counter_hz()
    }
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> SysCounter<FREQ> {
        Timer::syst_hclk(self, clocks).counter()
    }
    fn delay(self, clocks: &Clocks) -> SysDelay {
        Timer::syst_hclk(self, clocks).delay()
    }
}



impl Timer<STK> {
    /// Initialize SysTick timer and set it frequency to `HCLK`
    pub fn syst_hclk(mut tim: STK, clocks: &Clocks) -> Self {
        tim.set_clock_source(StkClkSource::Hclk);
        Self {
            tim,
            clk: clocks.hclk(),
        }
    }

    /// Initialize SysTick timer and set it frequency to `HCLK / 8`
    pub fn syst_hclk_div_8(mut tim: STK, clocks: &Clocks) -> Self {
        tim.set_clock_source(StkClkSource::HclkDiv8);
        Self {
            tim,
            clk: clocks.hclk() / 8,
        }
    }

    // pub fn configure(&mut self, clocks: &Clocks) {
    //     self.tim.set_clock_source(SystClkSource::Core);
    //     self.clk = clocks.hclk();
    // }

    // pub fn configure_external(&mut self, clocks: &Clocks) {
    //     self.tim.set_clock_source(SystClkSource::External);
    //     self.clk = clocks.hclk() / 8;
    // }

    pub fn release(self) -> STK {
        self.tim
    }

    /// Starts listening for an `event`
    pub fn listen(&mut self, event: SysEvent) {
        match event {
            SysEvent::Update => self.tim.enable_interrupt(),
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: SysEvent) {
        match event {
            SysEvent::Update => self.tim.disable_interrupt(),
        }
    }

    /// Resets the counter
    pub fn reset(&mut self) {
        // According to the Cortex-M3 Generic User Guide, the interrupt request is only generated
        // when the counter goes from 1 to 0, so writing zero should not trigger an interrupt
        riscv::interrupt::free(|| {
            self.tim.clear_current_non_atomic();
        });
        
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Ocm {
    Frozen = 0,
    ActiveOnMatch = 1,
    InactiveOnMatch = 2,
    Toggle = 3,
    ForceInactive = 4,
    ForceActive = 5,
    PwmMode1 = 6,
    PwmMode2 = 7,
}

mod sealed {
    use super::{Channel, Event, Ocm};

    // TODO: Support this when DBG function is ready.
    // use super::DBG;

    pub trait General {
        type Width: Into<u32> + From<u16>;
        fn max_auto_reload() -> u32;
        unsafe fn set_auto_reload_unchecked(&mut self, arr: u32);
        fn set_auto_reload(&mut self, arr: u32) -> Result<(), super::Error>;
        fn read_auto_reload() -> u32;
        fn enable_preload(&mut self, b: bool);
        fn enable_counter(&mut self);
        fn disable_counter(&mut self);
        fn is_counter_enabled(&self) -> bool;
        fn reset_counter(&mut self);
        fn set_prescaler(&mut self, psc: u16);
        fn read_prescaler(&self) -> u16;
        fn trigger_update(&mut self);
        fn clear_interrupt_flag(&mut self, event: Event);
        fn listen_interrupt(&mut self, event: Event, b: bool);
        fn get_interrupt_flag(&self) -> Event;
        fn read_count(&self) -> Self::Width;
        fn start_one_pulse(&mut self);
        fn cr1_reset(&mut self);
        // TODO: Support this when DBG function is ready.
        // fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool);
    }

    pub trait WithPwm: General {
        const CH_NUMBER: u8;
        fn read_cc_value(channel: u8) -> u32;
        fn set_cc_value(channel: u8, value: u32);
        fn preload_output_channel_in_mode(&mut self, channel: Channel, mode: Ocm);
        fn start_pwm(&mut self);
        fn enable_channel(channel: u8, b: bool);
    }

    pub trait MasterTimer: General {
        type Mms;
        fn master_mode(&mut self, mode: Self::Mms);
    }
}
pub(crate) use sealed::{General, MasterTimer, WithPwm};

pub trait Instance:
    crate::Sealed + rcc::Enable + rcc::Reset + rcc::BusTimerClock + General
{
}

macro_rules! hal {
    ($($TIM:ty: [
        $Timer:ident,
        $bits:ty,
        $dbg_timX_stop:ident,
        $(c: ($cnum:ident $(, $aoe:ident)?),)?
        $(m: $timbase:ident,)?
    ],)+) => {
        $(
            impl Instance for $TIM { }
            pub type $Timer = Timer<$TIM>;

            impl General for $TIM {
                type Width = $bits;

                #[inline(always)]
                fn max_auto_reload() -> u32 {
                    <$bits>::MAX as u32
                }
                #[inline(always)]
                unsafe fn set_auto_reload_unchecked(&mut self, arr: u32) {
                    self.atrlr.write(|w| w.bits(arr))
                }
                #[inline(always)]
                fn set_auto_reload(&mut self, arr: u32) -> Result<(), Error> {
                    // Note: Make it impossible to set the ARR value to 0, since this
                    // would cause an infinite loop.
                    if arr > 0 && arr <= Self::max_auto_reload() {
                        Ok(unsafe { self.set_auto_reload_unchecked(arr) })
                    } else {
                        Err(Error::WrongAutoReload)
                    }
                }
                #[inline(always)]
                fn read_auto_reload() -> u32 {
                    let tim = unsafe { &*<$TIM>::ptr() };
                    tim.atrlr.read().bits()
                }
                #[inline(always)]
                fn enable_preload(&mut self, b: bool) {
                    self.ctlr1.modify(|_, w| w.arpe().bit(b));
                }
                #[inline(always)]
                fn enable_counter(&mut self) {
                    self.ctlr1.modify(|_, w| w.cen().set_bit());
                }
                #[inline(always)]
                fn disable_counter(&mut self) {
                    self.ctlr1.modify(|_, w| w.cen().clear_bit());
                }
                #[inline(always)]
                fn is_counter_enabled(&self) -> bool {
                    self.ctlr1.read().cen().bit()
                }
                #[inline(always)]
                fn reset_counter(&mut self) {
                    self.cnt.reset();
                }
                #[inline(always)]
                fn set_prescaler(&mut self, psc: u16) {
                    self.psc.write(|w| unsafe {w.psc().bits(psc)} );
                }
                #[inline(always)]
                fn read_prescaler(&self) -> u16 {
                    self.psc.read().psc().bits()
                }
                #[inline(always)]
                fn trigger_update(&mut self) {
                    // Sets the URS bit to prevent an interrupt from being triggered by
                    // the UG bit
                    self.ctlr1.modify(|_, w| w.urs().set_bit());
                    self.swevgr.write(|w| w.ug().set_bit());
                    self.ctlr1.modify(|_, w| w.urs().clear_bit());
                }
                #[inline(always)]
                fn clear_interrupt_flag(&mut self, event: Event) {
                    self.swevgr.write(|w| unsafe { w.bits(0xffff & !event.bits()) });
                }
                #[inline(always)]
                fn listen_interrupt(&mut self, event: Event, b: bool) {
                    if b {
                        self.dmaintenr.modify(|r, w| unsafe { w.bits(r.bits() | event.bits()) });
                    } else {
                        self.dmaintenr.modify(|r, w| unsafe { w.bits(r.bits() & !event.bits()) });
                    }
                }
                #[inline(always)]
                fn get_interrupt_flag(&self) -> Event {
                    Event::from_bits_truncate(self.intfr.read().bits())
                }
                #[inline(always)]
                fn read_count(&self) -> Self::Width {
                    self.cnt.read().bits() as Self::Width
                }
                #[inline(always)]
                fn start_one_pulse(&mut self) {
                    self.ctlr1.write(|w| unsafe { w.bits(1 << 3) }.cen().set_bit());
                }
                #[inline(always)]
                fn cr1_reset(&mut self) {
                    self.ctlr1.reset();
                }

                // TODO: Support this when DBG function is ready.
                // #[inline(always)]
                // fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
                //     dbg.cr.modify(|_, w| w.$dbg_timX_stop().bit(state));
                // }
            }
            $(with_pwm!($TIM: $cnum $(, $aoe)?);)?

            // TODO Enable the following code when upstream PAC add the `MMS_A` Enum type

            // $(impl MasterTimer for $TIM {
            //     type Mms = pac::$timbase::ctlr2::MMS_A;
            //     fn master_mode(&mut self, mode: Self::Mms) {
            //         self.ctlr2.modify(|_,w| w.mms().variant(mode));
            //     }
            // })?
        )+
    }
}

macro_rules! with_pwm {
    ($TIM:ty: CH1) => {
        impl WithPwm for $TIM {
            const CH_NUMBER: u8 = 1;

            #[inline(always)]
            fn read_cc_value(channel: u8) -> u32 {
                let tim = unsafe { &*<$TIM>::ptr() };
                if channel < Self::CH_NUMBER {
                    tim.ccr[channel as usize].read().bits()
                } else {
                    0
                }
            }

            #[inline(always)]
            fn set_cc_value(channel: u8, value: u32) {
                let tim = unsafe { &*<$TIM>::ptr() };
                #[allow(unused_unsafe)]
                if channel < Self::CH_NUMBER {
                    tim.ccr[channel as usize].write(|w| unsafe { w.bits(value) })
                }
            }

            #[inline(always)]
            fn preload_output_channel_in_mode(&mut self, channel: Channel, mode: Ocm) {
                match channel {
                    Channel::C1 => {
                        self.ccmr1_output()
                        .modify(|_, w| w.oc1pe().set_bit().oc1m().bits(mode as _) );
                    }
                    _ => {},
                }
            }

            #[inline(always)]
            fn start_pwm(&mut self) {
                self.cr1.write(|w| w.cen().set_bit());
            }

            #[inline(always)]
            fn enable_channel(c: u8, b: bool) {
                let tim = unsafe { &*<$TIM>::ptr() };
                if c < Self::CH_NUMBER {
                    unsafe { bb::write(&tim.ccer, c*4, b); }
                }
            }
        }
    };
    ($TIM:ty: CH2) => {
        impl WithPwm for $TIM {
            const CH_NUMBER: u8 = 2;

            #[inline(always)]
            fn read_cc_value(channel: u8) -> u32 {
                let tim = unsafe { &*<$TIM>::ptr() };
                if channel < Self::CH_NUMBER {
                    tim.ccr[channel as usize].read().bits()
                } else {
                    0
                }
            }

            #[inline(always)]
            fn set_cc_value(channel: u8, value: u32) {
                let tim = unsafe { &*<$TIM>::ptr() };
                #[allow(unused_unsafe)]
                if channel < Self::CH_NUMBER {
                    tim.ccr[channel as usize].write(|w| unsafe { w.bits(value) })
                }
            }

            #[inline(always)]
            fn preload_output_channel_in_mode(&mut self, channel: Channel, mode: Ocm) {
                match channel {
                    Channel::C1 => {
                        self.ccmr1_output()
                        .modify(|_, w| w.oc1pe().set_bit().oc1m().bits(mode as _) );
                    }
                    Channel::C2 => {
                        self.ccmr1_output()
                        .modify(|_, w| w.oc2pe().set_bit().oc2m().bits(mode as _) );
                    }
                    _ => {},
                }
            }

            #[inline(always)]
            fn start_pwm(&mut self) {
                self.cr1.write(|w| w.cen().set_bit());
            }

            #[inline(always)]
            fn enable_channel(c: u8, b: bool) {
                let tim = unsafe { &*<$TIM>::ptr() };
                if c < Self::CH_NUMBER {
                    unsafe { bb::write(&tim.ccer, c*4, b); }
                }
            }
        }
    };
    ($TIM:ty: CH4 $(, $aoe:ident)?) => {
        impl WithPwm for $TIM {
            const CH_NUMBER: u8 = 4;

            #[inline(always)]
            fn read_cc_value(channel: u8) -> u32 {
                
                get_compare_channel!($TIM, channel).read().bits()
            }

            #[inline(always)]
            fn set_cc_value(channel: u8, value: u32) {
                get_compare_channel!($TIM, channel).write(|w| unsafe { w.bits(value) })
            }

            #[inline(always)]
            fn preload_output_channel_in_mode(&mut self, channel: Channel, mode: Ocm) {
                match channel {
                    Channel::C1 => {
                        self.chctlr1_output()
                        .modify(|_, w| unsafe{w.oc1pe().set_bit().oc1m().bits(mode as _)} );
                    }
                    Channel::C2 => {
                        self.chctlr1_output()
                        .modify(|_, w| unsafe{w.oc2pe().set_bit().oc2m().bits(mode as _)} );
                    }
                    Channel::C3 => {
                        self.chctlr2_output()
                        .modify(|_, w| unsafe{w.oc3pe().set_bit().oc3m().bits(mode as _)} );
                    }
                    Channel::C4 => {
                        self.chctlr2_output()
                        .modify(|_, w| unsafe{w.oc4pe().set_bit().oc4m().bits(mode as _)} );
                    }
                }
            }

            #[inline(always)]
            fn start_pwm(&mut self) {
                $(let $aoe = self.bdtr.modify(|_, w| w.aoe().set_bit());)?
                self.ctlr1.write(|w| w.cen().set_bit());
            }

            #[inline(always)]
            fn enable_channel(c: u8, b: bool) {
                let tim = unsafe { &*<$TIM>::ptr() };
                if c < Self::CH_NUMBER {
                    tim.ccer.modify(|r, w| unsafe {
                        w.bits(
                            if b {
                                r.bits() | 1<<(4*c)
                            } else {
                                r.bits() & (!(1<<(4*c)))
                            }
                        )
                    } );
                }
            }
        }
    }
}


macro_rules! get_compare_channel {
    ($TIM:ty, $ch: expr) => {
        unsafe{
            let tim =  &*<$TIM>::ptr() ;
            &*(&tim.ch1cvr as *const _ as *const pac::tim1::CH1CVR).offset($ch as isize)
        }
    };
}

impl<TIM: Instance> Timer<TIM> {
    /// Initialize timer
    pub fn new(tim: TIM, clocks: &Clocks) -> Self {
        unsafe {
            //NOTE(unsafe) this reference will only be used for atomic writes with no side effects
            let rcc = &(*RCC::ptr());
            // Enable and reset the timer peripheral
            TIM::enable(rcc);
            TIM::reset(rcc);
        }

        Self {
            clk: TIM::timer_clock(clocks),
            tim,
        }
    }

    pub fn configure(&mut self, clocks: &Clocks) {
        self.clk = TIM::timer_clock(clocks);
    }

    pub fn counter_hz(self) -> CounterHz<TIM> {
        CounterHz(self)
    }

    pub fn release(self) -> TIM {
        self.tim
    }

    /// Starts listening for an `event`
    ///
    /// Note, you will also have to enable the TIM2 interrupt in the NVIC to start
    /// receiving events.
    pub fn listen(&mut self, event: Event) {
        self.tim.listen_interrupt(event, true);
    }

    /// Clears interrupt associated with `event`.
    ///
    /// If the interrupt is not cleared, it will immediately retrigger after
    /// the ISR has finished.
    pub fn clear_interrupt(&mut self, event: Event) {
        self.tim.clear_interrupt_flag(event);
    }

    pub fn get_interrupt(&mut self) -> Event {
        self.tim.get_interrupt_flag()
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        self.tim.listen_interrupt(event, false);
    }

    // TODO: Support this when DBG function is ready.
    // /// Stopping timer in debug mode can cause troubles when sampling the signal
    // pub fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
    //     self.tim.stop_in_debug(dbg, state);
    // }
}

impl<TIM: Instance + MasterTimer> Timer<TIM> {
    pub fn set_master_mode(&mut self, mode: TIM::Mms) {
        self.tim.master_mode(mode)
    }
}

/// Timer wrapper for fixed precision timers.
///
/// Uses `fugit::TimerDurationU32` for most of operations
pub struct FTimer<TIM, const FREQ: u32> {
    tim: TIM,
}

/// `FTimer` with precision of 1 μs (1 MHz sampling)
pub type FTimerUs<TIM> = FTimer<TIM, 1_000_000>;

/// `FTimer` with precision of 1 ms (1 kHz sampling)
///
/// NOTE: don't use this if your system frequency more than 65 MHz
pub type FTimerMs<TIM> = FTimer<TIM, 1_000>;

impl<TIM: Instance, const FREQ: u32> FTimer<TIM, FREQ> {
    /// Initialize timer
    pub fn new(tim: TIM, clocks: &Clocks) -> Self {
        unsafe {
            //NOTE(unsafe) this reference will only be used for atomic writes with no side effects
            let rcc = &(*RCC::ptr());
            // Enable and reset the timer peripheral
            TIM::enable(rcc);
            TIM::reset(rcc);
        }

        let mut t = Self { tim };
        t.configure(clocks);
        t
    }

    /// Calculate prescaler depending on `Clocks` state
    pub fn configure(&mut self, clocks: &Clocks) {
        let clk = TIM::timer_clock(clocks);
        assert!(clk.raw() % FREQ == 0);
        let psc = clk.raw() / FREQ;
        self.tim.set_prescaler(u16::try_from(psc - 1).unwrap());
    }

    /// Creates `Counter` that imlements [embedded_hal::timer::CountDown]
    pub fn counter(self) -> Counter<TIM, FREQ> {
        Counter(self)
    }

    /// Creates `Delay` that imlements [embedded_hal::blocking::delay] traits
    pub fn delay(self) -> Delay<TIM, FREQ> {
        Delay(self)
    }

    /// Releases the TIM peripheral
    pub fn release(self) -> TIM {
        self.tim
    }

    /// Starts listening for an `event`
    ///
    /// Note, you will also have to enable the TIM2 interrupt in the NVIC to start
    /// receiving events.
    pub fn listen(&mut self, event: Event) {
        self.tim.listen_interrupt(event, true);
    }

    /// Clears interrupt associated with `event`.
    ///
    /// If the interrupt is not cleared, it will immediately retrigger after
    /// the ISR has finished.
    pub fn clear_interrupt(&mut self, event: Event) {
        self.tim.clear_interrupt_flag(event);
    }

    pub fn get_interrupt(&mut self) -> Event {
        self.tim.get_interrupt_flag()
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        self.tim.listen_interrupt(event, false);
    }

    // TODO: Support this when DBG function is ready.
    // /// Stopping timer in debug mode can cause troubles when sampling the signal
    // pub fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
    //     self.tim.stop_in_debug(dbg, state);
    // }
}

impl<TIM: Instance + MasterTimer, const FREQ: u32> FTimer<TIM, FREQ> {
    pub fn set_master_mode(&mut self, mode: TIM::Mms) {
        self.tim.master_mode(mode)
    }
}

#[inline(always)]
const fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u32) {
    let ticks = clock / freq;
    let psc = (ticks - 1) / (1 << 16);
    let arr = ticks / (psc + 1) - 1;
    (psc as u16, arr)
}




// It seems that all ch32v chips have TIM1 and TIM2
hal!(
    pac::TIM1: [Timer1, u16, dbg_tim1_stop, c: (CH4, _aoe), m: tim1,],
    pac::TIM2: [Timer2, u16, dbg_tim2_stop, c: (CH4), m: tim2,],
);

#[cfg(feature="tim_adtm_8_9_10")]
hal!(
    pac::TIM8: [Timer8, u16, dbg_tim8_stop, c: (CH4, _aoe), m: tim1,],
    // TODO TIM9、TIM10
    
);

#[cfg(feature="tim_gptm_3")]
hal! {
    pac::TIM3: [Timer3, u16, dbg_tim3_stop, c: (CH4), m: tim2,],
}

#[cfg(feature="tim_gptm_4_5")]
hal! {
    pac::TIM4: [Timer4, u16, dbg_tim4_stop, c: (CH4), m: tim2,],
    pac::TIM5: [Timer5, u16, dbg_tim5_stop, c: (CH4), m: tim2,],
    
}

#[cfg(feature="tim_bctm_6_7")]
hal! {
    pac::TIM6: [Timer6, u16, dbg_tim6_stop, m: tim6,],
    pac::TIM7: [Timer7, u16, dbg_tim7_stop, m: tim6,],
}
