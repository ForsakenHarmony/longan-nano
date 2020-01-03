#![no_std]
#![no_main]

#[allow(unused)]
use panic_halt;

use ws2812_spi as ws2812;

use crate::ws2812::prerendered::{Timing, Ws2812};

use smart_leds::{SmartLedsWrite, RGB8};

use riscv_rt::entry;
use gd32vf103xx_hal::pac as pac;
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::spi::{Spi, MODE_0};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::delay::DelayMs;


struct Delay;

impl DelayMs<u16> for Delay {
	fn delay_ms(&mut self, ms: u16) {
		let dt = (ms as u64) * 1000;
		let t0 = riscv::register::mcycle::read64();
		loop {
			let t = riscv::register::mcycle::read64();
			if (t - t0) > dt {
				break;
			}
		}
	}
}

#[entry]
fn main() -> ! {
	let dp = pac::Peripherals::take().unwrap();

	// Configure clocks
	let rcu = dp.RCU.constrain();
	let clocks = rcu.cctl.ext_hf_clock(8.mhz()).sysclk(108.mhz()).freeze();

	let gpioa = dp.GPIOA.split();
	let gpiob = dp.GPIOB.split();

	let sck = gpiob.pb13.into_alternate_push_pull();
	let miso = gpiob.pb14.into_floating_input();
	let mosi = gpiob.pb15.into_alternate_push_pull();
	let spi = Spi::spi1(dp.SPI1, (sck, miso, mosi), MODE_0, 4.mhz(), &clocks);

	//let dc = gpiob.pb0.into_push_pull_output();
	//let rst = gpiob.pb1.into_push_pull_output();
	//let mut cs = gpiob.pb2.into_push_pull_output();
	//cs.set_low().unwrap();

	let mut delay = Delay;
  
	// Configure SPI with 3Mhz rate
//	let spi = Spi::spi1(
//		p.SPI1,
//		(sck, miso, mosi),
//		ws2812::MODE,
//		3_000_000.hz(),
//		&mut rcc,
//	);
	const MAX: usize = 8;
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
	let mut ws = Ws2812::new(spi, Timing::new(4_000_000).unwrap(), &mut render_data);
	let mut up = true;
	
	loop {
		for i in 0..MAX {
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
			).into();
			data[i] = ct;
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
		ws.write(data.iter().cloned()).unwrap();
		delay.delay_ms(100u16);
	}
	
	//loop {
	//	continue;
	//}
}

