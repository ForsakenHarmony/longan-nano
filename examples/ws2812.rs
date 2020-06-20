#![no_std]
#![no_main]

use embedded_hal::blocking::delay::DelayMs;
use gd32vf103xx_hal::delay::McycleDelay;
use gd32vf103xx_hal::pac;
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::spi::{Spi, MODE_0};
use gd32vf103xx_hal::timer::Timer;
#[allow(unused_imports)]
use panic_halt;
use riscv_rt::entry;
use smart_leds::{SmartLedsWrite, RGB8};
//use ws2812_spi as ws2812;
use ws2812_timer_delay as ws2812;

//use crate::ws2812::prerendered::{Timing, Ws2812};
use crate::ws2812::Ws2812;
//
#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(24.mhz())
        .freeze();

    let gpiob = dp.GPIOB.split(&mut rcu);

    //	rcu.clocks.timer0()

    //	let sck = gpiob.pb13.into_alternate_push_pull();
    //	let miso = gpiob.pb14.into_floating_input();
    //	let mosi = gpiob.pb15.into_alternate_push_pull();
    //	let spi = Spi::spi1(dp.SPI1, (sck, miso, mosi), MODE_0, 3.mhz(), &mut rcu);

    let mut timer = Timer::timer1(dp.TIMER1, 3.mhz(), &mut rcu);
    let mut delay = McycleDelay::new(&rcu.clocks);

    const MAX: usize = 39;
    const COLOR1: RGB8 = RGB8 {
        r: 0x00,
        g: 0xc3 / 5,
        b: 0x36 / 5,
    };
    const COLOR2: RGB8 = RGB8 {
        r: 0x00,
        g: 0x24 / 5,
        b: 0xb0 / 5,
    };
    let mut data: [RGB8; MAX] = [(0, 0, 0).into(); MAX];
    let mut main = 0;
    let mut render_data = [0; MAX * 3 * 5];
    //	let mut ws = Ws2812::new(spi, Timing::new(3_000_000).unwrap(), &mut render_data);
    //	let mut ws = Ws2812::new(spi);

    let mosi = gpiob.pb15.into_push_pull_output();
    let mut ws = Ws2812::new(timer, mosi);

    let mut up = true;

    loop {
        //		if main < MAX {
        //			data[main] = COLOR1;
        ////			data[main+1] = COLOR1;
        ////			data[main+2] = COLOR1;
        //			main += 1;
        //		}
        for (i, rgb) in data.iter_mut().enumerate() {
            let distance = (main as i32 - i as i32).abs() as u8;
            let c1 = (
                COLOR1.r as u32 * (MAX as u32 - distance as u32) / MAX as u32,
                COLOR1.g as u32 * (MAX as u32 - distance as u32) / MAX as u32,
                COLOR1.b as u32 * (MAX as u32 - distance as u32) / MAX as u32,
            );
            let c2 = (
                COLOR2.r as u32 * distance as u32 / MAX as u32,
                COLOR2.g as u32 * distance as u32 / MAX as u32,
                COLOR2.b as u32 * distance as u32 / MAX as u32,
            );
            let ct = (
                (c1.0 + c2.0) as u8,
                (c1.1 + c2.1) as u8,
                (c1.2 + c2.2) as u8,
            )
                .into();
            *rgb = ct;
        }
        if up {
            if main == MAX - 1 {
                up = false;
                main -= 2;
            }
            main += 1;
        } else {
            if main == 0 {
                up = true;
                main += 2;
            }
            main -= 1;
        }
        //		data[2] = COLOR2;
        //		data[2] = COLOR2;
        //		data[2] = COLOR2;
        ws.write(data.iter().cloned()).unwrap_or_default();
        delay.delay_ms(100);
        //		data[2] = COLOR2;
    }

    //loop {
    //	continue;
    //}
}
