//! Testing PWM output for custom pin combinations
//! 
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use core::panic::PanicInfo;
use riscv::asm;
use riscv_rt::entry;
use fugit::HertzU32;

use ch32v_hal::{self, pac, prelude::*, timer::Timer};


#[panic_handler]
fn my_panic_handler(_panic: &PanicInfo) -> !{
    loop{}
}

#[entry]
fn main() -> ! {


    const clock_calc: ch32v_hal::rcc::ClockCalculator = ch32v_hal::rcc::ClockCalculator::new()
    .use_hse( HertzU32::MHz(8))
    .sysclk(HertzU32::MHz(8 as u32))
    .pclk1(HertzU32::MHz(8 as u32))
    .pclk2(HertzU32::MHz(8 as u32))
    .freeze();

    let peripherals = pac::Peripherals::take().unwrap();
    let clocks = peripherals.RCC.constrain().do_setup(&clock_calc);


    let mut afio = peripherals.AFIO.constrain();

    let gpioa = peripherals.GPIOA.split();
    let mut gpiob = peripherals.GPIOB.split();

    let (_pa15, _pb3, pb4) = afio.pcfr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // TIM3
    let p0 = pb4.into_alternate_push_pull();
    let p1 = gpiob.pb5.into_alternate_push_pull();

    let pwm = Timer::new(peripherals.TIM3, &clocks).pwm_hz((p0, p1), &mut afio.pcfr, 1.kHz());

    let max = pwm.get_max_duty();

    let mut pwm_channels = pwm.split();

    // Enable the individual channels
    pwm_channels.0.enable();
    pwm_channels.1.enable();


    pwm_channels.0.set_duty(max /2);
    pwm_channels.1.set_duty(max / 4);

    loop {}
}
