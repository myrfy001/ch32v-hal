#![no_std]
#![no_main]

use core::panic::PanicInfo;
use riscv::asm;
use riscv_rt::entry;

use ch32v_hal::{self, pac, prelude::*};
use fugit::HertzU32;




#[panic_handler]
fn my_panic_handler(_panic: &PanicInfo) -> !{
    loop{}
}


fn delay() {
    for _ in 0..10000 {
        unsafe {asm::nop()};
    }
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

    let x = unsafe{pac::Peripherals::steal()};
    x.RCC.apb2pcenr.modify(|_,w| w.iopden().set_bit());
    x.GPIOD.cfghr.modify(|_, w| unsafe{w.mode14().bits(0x03).cnf14().bits(0x01)});




    loop {
        x.GPIOD.outdr.modify(|_,w| w.odr14().set_bit());

        delay();

        x.GPIOD.outdr.modify(|_,w| w.odr14().clear_bit());

        delay();

        
    }
}