#![no_std]
#![no_main]

use panic_halt as _;

use gd32vf103xx_hal as hal;
use gd32vf103xx_hal::prelude::*;
use hal::pac;
use longan_nano::sprintln;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();

    let mut afio = dp.AFIO.constrain(&mut rcu);

    let gpioa = dp.GPIOA.split(&mut rcu);
    longan_nano::stdout::configure(
        dp.USART0,
        gpioa.pa9,
        gpioa.pa10,
        115_200.bps(),
        &mut afio,
        &mut rcu,
    );

    sprintln!("Hello, world");

    loop {}
}
