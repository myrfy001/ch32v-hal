#![no_std]
#![no_main]

use core::panic::PanicInfo;
use riscv::asm;
use riscv_rt::entry;

use ch32v_hal::{self, pac, prelude::*, serial::{Config, Serial}};
use fugit::HertzU32;

use nb::block;
use unwrap_infallible::UnwrapInfallible;




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
    let mut led = peripherals.GPIOD.split().pd13.into_push_pull_output();

    let mut gpioa = peripherals.GPIOA.split();
    // USART1 on Pins A9 and A10
    let pin_tx = gpioa.pa9.into_alternate_push_pull();
    let pin_rx = gpioa.pa10;

    // Prepare the alternate function I/O registers
    let mut afio = peripherals.AFIO.constrain();

    // Create an interface struct for USART1 with 9600 Baud
    let serial = Serial::new(
        peripherals.USART1,
        (pin_tx, pin_rx),
        &mut afio.pcfr,
        Config::default()
            .baudrate(9600.bps())
            .wordlength_9bits()
            .parity_none(),
        &clocks,
    );

    // Separate into tx and rx channels
    let (mut tx, mut rx) = serial.split();

    // Write data (9 bits) to the USART.
    // Depending on the configuration, only the lower 7, 8, or 9 bits are used.
    block!(tx.write_u16(0x1FF)).unwrap_infallible();

    // Write 'R' (8 bits) to the USART
    block!(tx.write(b'R')).unwrap_infallible();

    // Receive a data (9 bits) from the USART and store it in "received"
    let received = block!(rx.read_u16()).unwrap();

    // Receive a data (8 bits) from the USART and store it in "received"
    let received = block!(rx.read()).unwrap();

    loop{};

}