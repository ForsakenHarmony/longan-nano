#![no_std]
#![no_main]
// #![feature(const_generics)]
// #![feature(const_generic_impls_guard)]

// use panic_halt as _;

// use core::f32::consts::PI;
use embedded_graphics::fonts::{Font6x8};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
// use embedded_graphics::primitives::Rectangle;
use gd32vf103xx_hal::pac;
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::timer::Timer;
// use gd32vf103xx_hal::spi::{Spi, MODE_0};
// use hub75::Hub75;
use riscv_rt::{entry, DefaultExceptionHandler};
use gd32vf103xx_hal::delay::McycleDelay;
// use lazy_static::lazy_static;
use core::{f32, fmt};
// use longan_nano::led::{rgb, Led, RED};
use longan_nano::sprintln;

// use riscv_rt::pre_init;
use core::panic::PanicInfo;
use core::sync::atomic::{Ordering};
use core::sync::atomic;
// use gd32vf103xx_hal::time::Hertz;
use embedded_graphics::primitives::Rectangle;
// use core::fmt::Alignment::Left;
// use core::cell::UnsafeCell;

const WIDTH: usize = 8;
const HEIGHT: usize = 8;

const PI: f32 = 3.14159265358979323846264338327950288_f32;

// static mut LEDS: AtomicPtr<Option<&mut RED>> = AtomicPtr::new(&mut None);

// impl<T: fmt::Debug> fmt::Debug for [T; 64]
// {
// 	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
// 		fmt::Debug::fmt(&&self[..], f)
// 	}
// }

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
	sprintln!("{}", info);
	// unsafe {
	// 	let leds = LEDS.get_mut().as_ref();
	// 	leds.and_then(|o| o.map(|leds| leds.on()));
	// }
	loop {
		atomic::compiler_fence(Ordering::SeqCst);
	}
}

// #[no_mangle]
// extern "C" fn __truncdfsf2(x: f64) -> f64 {
// 	let x1p120 = f64::from_bits(0x4770000000000000); // 0x1p120f === 2 ^ 120
//
// 	let mut i: u64 = x.to_bits();
// 	let mut e: i64 = (i >> 52 & 0x7ff) as i64 - 0x3ff + 12;
// 	let m: u64;
//
// 	if e >= 52 + 12 {
// 		return x;
// 	}
// 	if e < 12 {
// 		e = 1;
// 	}
// 	m = -1i64 as u64 >> e;
// 	if (i & m) == 0 {
// 		return x;
// 	}
// 	unsafe {
// 		::core::ptr::read_volatile(&(x + x1p120));
// 	}
// 	// force_eval!(x + x1p120);
// 	i &= !m;
// 	f64::from_bits(i)
// }

#[export_name = "UserTimer"]
fn user_timer() {
	sprintln!("Timer");
}

#[export_name = "TimeOut"]
fn time_out() {
	sprintln!("Timer");
}

#[export_name = "DefaultHandler"]
fn default_handler() {
	sprintln!("Interrupt");
}

#[export_name = "ExceptionHandler"]
fn exception_handler(trap_frame: &riscv_rt::TrapFrame) -> ! {
	sprintln!("Crash");
	DefaultExceptionHandler(trap_frame);
}

#[inline(never)]
pub unsafe extern "C" fn __read32(_default: usize, addr: usize) -> u32 {
	let ptr = addr as *const u32;
	ptr.read_volatile()
}

#[export_name = "trap_handler"]
fn trap_handler() {
	use riscv::register::{mcause, mepc, mtval, mcause::{Trap, Exception}};
	let ld_insn_addr = __read32 as *const () as usize;

	let mcause = mcause::read();
	let mepc = mepc::read();

	if mepc == ld_insn_addr && mcause.cause() == Trap::Exception(Exception::LoadFault) {
		mepc::write(mepc + 2);
		return;
	}

	sprintln!("trap!");
	sprintln!("mcause={:08x} mepc={:08x} mtval={:08x} addr={:08x}", mcause.bits(), mepc, mtval::read(), ld_insn_addr);

	loop {}
}


// static mut MATRIX: dyn Hub75 =

// #[interrupt]
// fn timer() {
// 	sprintln!("Timer");
// }

#[entry]
fn main() -> ! {
	let dp = pac::Peripherals::take().unwrap();

	// Configure clocks
	// let rcu = dp.RCU.constrain();
	let mut rcu = dp
		.RCU
		.configure()
		.ext_hf_clock(8.mhz())
		.sysclk(108.mhz())
		.freeze();
	let mut afio = dp.AFIO.constrain(&mut rcu);
	// let clocks = rcu.cctl.ext_hf_clock(8.mhz()).sysclk(108.mhz()).freeze();

	let gpioa = dp.GPIOA.split(&mut rcu);
	let gpiob = dp.GPIOB.split(&mut rcu);
	// let gpioc = dp.GPIOC.split(&mut rcu);
	// let (_, _, _, _, pb4) =
	// afio.disable_jtag(gpioa.pa13, gpioa.pa14, gpioa.pa15, gpiob.pb3, gpiob.pb4);

	// let (mut red, mut green, mut blue) = rgb(gpioc.pc13, gpioa.pa1, gpioa.pa2);
	// let leds = unsafe {
	// 	LEDS.store(&mut Some(&mut red), Ordering::SeqCst);
	// 	LEDS.get_mut().as_ref().unwrap().unwrap()
	// };

	longan_nano::stdout::configure(
		dp.USART0,
		gpioa.pa9,
		gpioa.pa10,
		115_200.bps(),
		&mut afio,
		&mut rcu,
	);

	let _timer0 = Timer::timer0(dp.TIMER0, 250.hz(), &mut rcu);
	let _timer1 = Timer::timer1(dp.TIMER1, 250.hz(), &mut rcu);
	let _timer2 = Timer::timer2(dp.TIMER2, 250.hz(), &mut rcu);
	let _timer3 = Timer::timer3(dp.TIMER3, 250.hz(), &mut rcu);
	let _timer4 = Timer::timer4(dp.TIMER4, 250.hz(), &mut rcu);
	let _timer5 = Timer::timer5(dp.TIMER5, 250.hz(), &mut rcu);
	let _timer6 = Timer::timer6(dp.TIMER6, 250.hz(), &mut rcu);

	// let sck = gpioa.pa5.into_alternate_push_pull();
	// let miso = gpioa.pa6.into_floating_input();
	// let mosi = gpioa.pa7.into_alternate_push_pull();
	// let spi0 = Spi::spi0(dp.SPI0, (sck, miso, mosi), MODE_0, 50.mhz(), &clocks);

	// let dc = gpiob.pb0.into_push_pull_output();
	// let rst = gpiob.pb1.into_push_pull_output();
	// let mut cs = gpiob.pb2.into_push_pull_output();
	// cs.set_low().unwrap();


	let mut r1 = gpiob.pb0.into_push_pull_output();
	let mut r2 = gpiob.pb5.into_push_pull_output();
	let g1 = gpiob.pb6.into_push_pull_output();
	let g2 = gpiob.pb7.into_push_pull_output();
	let mut b1 = gpiob.pb8.into_push_pull_output();
	let mut b2 = gpiob.pb9.into_push_pull_output();

	let mut a = gpiob.pb12.into_push_pull_output();
	let mut b = gpiob.pb13.into_push_pull_output();
	let mut c = gpiob.pb14.into_push_pull_output();
	let mut d = gpiob.pb15.into_push_pull_output();

	let mut clk = gpiob.pb1.into_push_pull_output();
	let mut oe = gpiob.pb10.into_push_pull_output();
	let mut lat = gpiob.pb11.into_push_pull_output();

	let mut matrix = Hub75::new((r1, g1, b1, r2, g2, b2, a, b, c, d, clk, lat, oe), 1);

	let mut delay = McycleDelay::new(&rcu.clocks);

	// let mut data = [HSV::default(); WIDTH * HEIGHT];
	// let mut matrix_data = [[HSV::default(); HEIGHT]; WIDTH];

	// matrix.clear();
	// matrix.draw(
	// 	Rectangle::new(
	// 		Coord::new(0, 0),
	// 		Coord::new(WIDTH as i32 - 1, HEIGHT as i32 - 1)
	// 	).fill(Some(Rgb565::from(0u16))));
	// // let t = Font6x8::render_str(" Hello Rust! ")
	// // 	.fill(Some(Rgb565::from((0,0xff,0))))
	// // 	.translate(Coord::new(2, 2));
	// matrix.draw(
	// 	Rectangle::new(
	// 		Coord::new(16, 16),
	// 		Coord::new(31, 31)
	// 	).fill(Some(Rgb565::from((0xff, 0xff, 0xff)))));
	// matrix.draw(
	// 	Rectangle::new(
	// 		Coord::new(0, 0),
	// 		Coord::new(8, 8)
	// 	).fill(Some(Rgb565::from((0xff, 0x0, 0xff)))));
	// matrix.draw(t);
	// matrix.output(&mut delay);
	// delay.delay_ms(100);

	// let mut sin: u8 = 0;
	// let mut hue: u8 = 0;
	//
	// // const CYCLE_LENGTH: usize = 16;
	//
	// let mut x_pos: f32 = 0.0;
	// let mut y_pos: f32 = 0.0;

	sprintln!("Before loop");
	// matrix.data.iter().for_each(|row| {
	// 	sprintln!("{:?}", &row[..]);
	// });

	// println!("yes");

	// oe.set_low().unwrap();
	//
	// for count in 0..1 {
	// 	r1.set_high().unwrap();
	// 	r2.set_high().unwrap();
	// 	for _ in 0..32 {
	// 		clk.set_high().unwrap();
	// 		clk.set_low().unwrap();
	// 	}
	// 	r1.set_low().unwrap();
	// 	r2.set_low().unwrap();
	// 	b1.set_high().unwrap();
	// 	b2.set_high().unwrap();
	// 	for _ in 0..32 {
	// 		clk.set_high().unwrap();
	// 		clk.set_low().unwrap();
	// 	}
	// 	b1.set_low().unwrap();
	// 	b2.set_low().unwrap();
	//
	// 	oe.set_high().unwrap();
	//
	// 	delay.delay_us(2);
	// 	lat.set_low().unwrap();
	// 	delay.delay_us(2);
	// 	lat.set_high().unwrap();
	//
	// 	// Select row
	// 	if count & 1 != 0 {
	// 		a.set_high().ok();
	// 	} else {
	// 		a.set_low().ok();
	// 	}
	// 	if count & 2 != 0 {
	// 		b.set_high().ok();
	// 	} else {
	// 		b.set_low().ok();
	// 	}
	// 	if count & 4 != 0 {
	// 		c.set_high().ok();
	// 	} else {
	// 		c.set_low().ok();
	// 	}
	// 	if count & 8 != 0 {
	// 		d.set_high().ok();
	// 	} else {
	// 		d.set_low().ok();
	// 	}
	//
	// 	delay.delay_us(2);
	// 	oe.set_low().unwrap();
	// }
	// oe.set_high().unwrap();
	// oe.set_low().unwrap();

	let mut thing: f32 = 0.0;
	loop {
		matrix.clear();
		let left = thing as i32;
		let left = if left < 0 { 0 } else { left };
		let right = thing as i32 + 3;
		let right = if right > 63 { 63 } else { right };
		matrix.draw(
			Rectangle::new(
				Coord::new(left, 0),
				Coord::new(right, 3)
			).fill(Some(Rgb565::from((0xff, 0x0, 0x00)))));
		matrix.draw(
			Rectangle::new(
				Coord::new(63 - right, 4),
				Coord::new(63 - left, 7)
			).fill(Some(Rgb565::from((0x00, 0xff, 0x00)))));
		matrix.draw(
			Rectangle::new(
				Coord::new(left, 8),
				Coord::new(right, 11)
			).fill(Some(Rgb565::from((0x00, 0x0, 0xff)))));
		matrix.draw(
			Rectangle::new(
				Coord::new(63 - right, 12),
				Coord::new(63 - left, 15)
			).fill(Some(Rgb565::from((0xff, 0xff, 0xff)))));
		thing += 0.05;
		if thing > 64.0 {
			thing = -3.0;
		}


		// oe.set_low().unwrap();
		// for count in 0..4 {
		// 	r1.set_high().unwrap();
		// 	r2.set_high().unwrap();
		// 	for _ in 0..16 {
		// 		clk.set_high().unwrap();
		// 		clk.set_low().unwrap();
		// 	}
		// 	r1.set_low().unwrap();
		// 	r2.set_low().unwrap();
		// 	b1.set_high().unwrap();
		// 	b2.set_high().unwrap();
		// 	for _ in 0..16 {
		// 		clk.set_high().unwrap();
		// 		clk.set_low().unwrap();
		// 	}
		// 	b1.set_low().unwrap();
		// 	b2.set_low().unwrap();
		//
		// 	oe.set_high().unwrap();
		//
		// 	// delay.delay_us(2);
		// 	lat.set_low().unwrap();
		// 	// delay.delay_us(2);
		// 	lat.set_high().unwrap();
		//
		// 	// Select row
		// 	if count & 1 != 0 {
		// 		a.set_high().ok();
		// 	} else {
		// 		a.set_low().ok();
		// 	}
		// 	if count & 2 != 0 {
		// 		b.set_high().ok();
		// 	} else {
		// 		b.set_low().ok();
		// 	}
		// 	if count & 4 != 0 {
		// 		c.set_high().ok();
		// 	} else {
		// 		c.set_low().ok();
		// 	}
		// 	if count & 8 != 0 {
		// 		d.set_high().ok();
		// 	} else {
		// 		d.set_low().ok();
		// 	}
		//
		// 	delay.delay_us(2);
		// 	oe.set_low().unwrap();
		// 	delay.delay_ms(1);
		// }
		// // delay.delay_us(2);
		// oe.set_high().unwrap();
		// b.set_high().ok();

		// sprintln!("1");
		// // matrix.output(&mut delay);
		// // // delay.delay_ms(100);
		// // // matrix.output(&mut delay);
		// // // delay.delay_ms(200);
		// sin = sin + 1;
		// hue = hue + 2;
		// //
		// //
		// // // sprintln!("Before sin");
		// //
		//
		// sprintln!("2");
		// let thing = sin as f32 / 255.0 * 2.0 * PI;
		// x_pos += libm::sinf(thing);
		// y_pos += libm::cosf(thing);
		// //
		// // // sprintln!("After sin");
		// //
		// sprintln!("3");
		// for x in 0..WIDTH {
		// 	// sprintln!("Loomp");
		// 	for y in 0..HEIGHT {
		// 		// let noise_val= (noise((x as f32 + x_pos as f32) / 4.0, (y as f32 + y_pos as f32) / 4.0, 0.0) * 255.0) as u8;
		// 		let noise_val = (noise((x as f32 + x_pos as f32 / 5.0) / 4.0, (y as f32) / 4.0, y_pos as f32 / 512.0) * 255.0) as u8;
		// 		let colour = HSV::new(noise_val + hue, noise_val, 255 /* noise_val */);
		// 		matrix_data[x][y] = colour;
		// 		// sprintln!("{:?} {:?}",	colour, hsv2rgb_rainbow(colour));
		// 	}
		// }
		// sprintln!("4");
		// //
		// // // matrix_to_strip(&matrix_data, &mut data);
		// //
		// // // ws.write(brightness(data.iter().cloned().map(|i| i.into()), 25)).unwrap();
		// // // delay.delay_ms(32u8);
		// //
		// // matrix.draw(matrix_data.iter().map(|row| row.iter().map(|pixel|)).flatten());
		// // // sprintln!("Send data");
		// matrix.draw(MatrixIter::new(&matrix_data)/*.filter(|p| {
		// 	// sprintln!("{:?}", p);
		// 	true
		// })*/);
		// sprintln!("5");
		// // sprintln!("Draw data");
		matrix.output(&mut delay);
		sprintln!("Loop");
		// // delay.delay_ms(100);
	}
}

// #[exception]
// fn HardFault(ef: &ExceptionFrame) -> ! {
// 	panic!("{:#?}", ef);
// }

struct MatrixIter<'a> {
	x: usize,
	y: usize,
	matrix: &'a [[HSV; HEIGHT]; WIDTH],
}

impl<'a> MatrixIter<'a> {
	fn new(matrix: &'a [[HSV; HEIGHT]; WIDTH]) -> MatrixIter {
		MatrixIter {
			x: 0,
			y: 0,
			matrix,
		}
	}
}

impl Iterator for MatrixIter<'_> {
	type Item = Pixel<Rgb565>;

	fn next(&mut self) -> Option<Self::Item> {
		// sprintln!("Next");
		if self.x >= WIDTH {
			self.y += 1;
			self.x = 0;
		}
		if self.y >= HEIGHT {
			return None;
		}

		// sprintln!("Inner {}:{}", self.x, self.y);
		let pixel = Pixel(UnsignedCoord::new(self.x as u32, self.y as u32), hsv2rgb_rainbow(self.matrix[self.x][self.y]));

		self.x += 1;

		Some(pixel)
	}
}


// fn matrix_to_strip(matrix: &[[HSV; HEIGHT]; WIDTH], data: &mut [HSV; WIDTH * HEIGHT]) {
// 	for i in 0..WIDTH * HEIGHT {
// 		let mut y = i % HEIGHT;
// 		let x = (i - y) / HEIGHT;
// 		if x % 2 == 0 {
// 			y = HEIGHT - y - 1;
// 		}
//
// 		data[i] = matrix[x][y];
// 	}
// }

#[derive(Copy, Clone, Debug)]
struct HSV {
	hue: u8,
	saturation: u8,
	value: u8,
}

impl HSV {
	fn new(hue: u8, saturation: u8, value: u8) -> Self {
		HSV {
			hue,
			saturation,
			value,
		}
	}
}

impl Into<Rgb565> for HSV {
	fn into(self) -> Rgb565 {
		hsv2rgb_rainbow(self)
	}
}

impl Default for HSV {
	fn default() -> Self {
		HSV::new(0, 0, 0)
	}
}

// from fastled
fn scale8(i: u8, scale: u8) -> u8 {
	(((i as u16) * (1 + scale as u16)) >> 8) as u8
}

// from fastled
fn scale8_video(i: u8, scale: u8) -> u8 {
	(((i as usize * scale as usize) >> 8) + if i > 0 && scale > 0 { 1 } else { 0 }) as u8
}

// from fastled
fn hsv2rgb_rainbow(hsv: HSV) -> Rgb565 {
	const K255: u8 = 255;
	const K171: u8 = 171;
	const K170: u8 = 170;
	const K85: u8 = 85;

	// Yellow has a higher inherent brightness than
	// any other color; 'pure' yellow is perceived to
	// be 93% as bright as white.  In order to make
	// yellow appear the correct relative brightness,
	// it has to be rendered brighter than all other
	// colors.
	// Level Y1 is a moderate boost, the default.
	// Level Y2 is a strong boost.
	const Y1: bool = true;
	const Y2: bool = false;

	// G2: Whether to divide all greens by two.
	// Depends GREATLY on your particular LEDs
	const G2: bool = false;

	// GSCALE: what to scale green down by.
	// Depends GREATLY on your particular LEDs
	const GSCALE: u8 = 0;

	let hue: u8 = hsv.hue;
	let sat: u8 = hsv.saturation;
	let mut val: u8 = hsv.value;

	let offset: u8 = hue & 0x1F; // 0..31

	// offset8 = offset * 8
	let mut offset8: u8 = offset;
	{
		offset8 <<= 3;
	}

	let third: u8 = scale8(offset8, (256u16 / 3) as u8); // max = 85

	let mut r = 0;
	let mut g = 0;
	let mut b = 0;

	if hue & 0x80 == 0 {
		// 0XX
		if hue & 0x40 == 0 {
			// 00X
			//section 0-1
			if hue & 0x20 == 0 {
				// 000
				//case 0: // R -> O
				r = K255 - third;
				g = third;
				b = 0;
			} else {
				// 001
				//case 1: // O -> Y
				if Y1 {
					r = K171;
					g = K85 + third;
					b = 0;
				}
				if Y2 {
					r = K170 + third;
					//uint8_t twothirds = (third << 1);
					let twothirds = scale8(offset8, ((256 * 2) / 3) as u8); // max=170
					g = K85 + twothirds;
					b = 0;
				}
			}
		} else {
			//01X
			// section 2-3
			if hue & 0x20 == 0 {
				// 010
				//case 2: // Y -> G
				if Y1 {
					//uint8_t twothirds = (third << 1);
					let twothirds = scale8(offset8, ((256 * 2) / 3) as u8); // max=170
					r = K171 - twothirds;
					g = K170 + third;
					b = 0;
				}
				if Y2 {
					r = K255 - offset8;
					g = K255;
					b = 0;
				}
			} else {
				// 011
				// case 3: // G -> A
				r = 0;
				g = K255 - third;
				b = third;
			}
		}
	} else {
		// section 4-7
		// 1XX
		if hue & 0x40 == 0 {
			// 10X
			if hue & 0x20 == 0 {
				// 100
				//case 4: // A -> B
				r = 0;
				//uint8_t twothirds = (third << 1);
				let twothirds = scale8(offset8, ((256 * 2) / 3) as u8); // max=170
				g = K171 - twothirds; //K170?
				b = K85 + twothirds;
			} else {
				// 101
				//case 5: // B -> P
				r = third;
				g = 0;

				b = K255 - third;
			}
		} else {
			if hue & 0x20 == 0 {
				// 110
				//case 6: // P -- K
				r = K85 + third;
				g = 0;

				b = K171 - third;
			} else {
				// 111
				//case 7: // K -> R
				r = K170 + third;
				g = 0;

				b = K85 - third;
			}
		}
	}

	// This is one of the good places to scale the green down,
	// although the client can scale green down as well.
	if G2 {
		g = g >> 1;
	}
	if GSCALE > 0 {
		g = scale8_video(g, GSCALE);
	}

	// Scale down colors if we're desaturated at all
	// and add the brightness_floor to r, g, and b.
	if sat != 255 {
		if sat == 0 {
			r = 255;
			b = 255;
			g = 255;
		} else {
			//nscale8x3_video( r, g, b, sat);
			if r > 0 { r = scale8(r, sat) }
			if g > 0 { g = scale8(g, sat) }
			if b > 0 { b = scale8(b, sat) }

			let mut desat = 255 - sat;
			desat = scale8(desat, desat);

			let brightness_floor = desat;
			r += brightness_floor;
			g += brightness_floor;
			b += brightness_floor;
		}
	}

	// Now scale everything down if we're at value < 255.
	if val != 255 {
		val = scale8_video(val, val);
		if val == 0 {
			r = 0;
			g = 0;
			b = 0;
		} else {
			if r > 0 { r = scale8(r, val) }
			if g > 0 { g = scale8(g, val) }
			if b > 0 { b = scale8(b, val) }
		}
	}

	Rgb565::from((
		r,
		g,
		b,
	))
}

const GRAD3: [[i32; 3]; 12] = [[1, 1, 0], [-1, 1, 0], [1, -1, 0], [-1, -1, 0], [1, 0, 1], [-1, 0, 1], [1, 0, -1], [-1, 0, -1], [0, 1, 1], [0, -1, 1], [0, 1, -1], [0, -1, -1]];

// const P: [usize; 256] = [151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180];

const PERM: [usize; 512] = [151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180, 151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180];

// lazy_static! {
//  static ref PERM: [usize; 512] = {
//     let mut perm = [0; 512];
//     for i in 0..512 {
//       perm[i] = P[i & 255];
//     }
//     perm
//  };
// }

fn fastfloor(x: f32) -> i32 {
	if x > 0.0 {
		x as i32
	} else {
		(x - 1.0) as i32
	}
}

fn dot(g: [i32; 3], x: f32, y: f32, z: f32) -> f32 {
	return g[0] as f32 * x + g[1] as f32 * y + g[2] as f32 * z;
}

// 3D simplex noise
pub fn noise(xin: f32, yin: f32, zin: f32) -> f32 {
	// Noise contributions from the four corners
	let n0;
	let n1;
	let n2;
	let n3;

	// Skew the input space to determine which simplex cell we're in
	let f3 = 1.0 / 3.0;
	let s = (xin + yin + zin) * f3; // Very nice and simple skew factor for 3D
	let i = fastfloor(xin + s) as f32;
	let j = fastfloor(yin + s) as f32;
	let k = fastfloor(zin + s) as f32;

	let g3 = 1.0 / 6.0; // Very nice and simple unskew factor, too
	let t = (i + j + k) * g3;
	let x0_ = i - t; // Unskew the cell origin back to (x,y,z) space
	let y0_ = j - t;
	let z0_ = k - t;
	let x0 = xin - x0_; // The x,y,z distances from the cell origin
	let y0 = yin - y0_;
	let z0 = zin - z0_;

	// For the 3D case, the simplex shape is a slightly irregular tetrahedron.
	// Determine which simplex we are in.
	// Offsets for second corner of simplex in (i,j,k) coords
	let i1;
	let j1;
	let k1;
	// Offsets for third corner of simplex in (i,j,k) coords
	let i2;
	let j2;
	let k2;

	if x0 >= y0 {
		if y0 >= z0 {
			// X Y Z order
			i1 = 1;
			j1 = 0;
			k1 = 0;
			i2 = 1;
			j2 = 1;
			k2 = 0;
		} else if x0 >= z0 {
			// X Z Y order
			i1 = 1;
			j1 = 0;
			k1 = 0;
			i2 = 1;
			j2 = 0;
			k2 = 1;
		} else {
			// Z X Y order
			i1 = 0;
			j1 = 0;
			k1 = 1;
			i2 = 1;
			j2 = 0;
			k2 = 1;
		}
		// x0<y0
	} else {
		// Z Y X order
		if y0 < z0 {
			i1 = 0;
			j1 = 0;
			k1 = 1;
			i2 = 0;
			j2 = 1;
			k2 = 1;
		}
		// Y Z X order
		else if x0 < z0 {
			i1 = 0;
			j1 = 1;
			k1 = 0;
			i2 = 0;
			j2 = 1;
			k2 = 1;
		}
		// Y X Z order
		else {
			i1 = 0;
			j1 = 1;
			k1 = 0;
			i2 = 1;
			j2 = 1;
			k2 = 0;
		}
	}

	// A step of (1,0,0) in (i,j,k) means a step of (1-c,-c,-c) in (x,y,z),
	// a step of (0,1,0) in (i,j,k) means a step of (-c,1-c,-c) in (x,y,z), and
	// a step of (0,0,1) in (i,j,k) means a step of (-c,-c,1-c) in (x,y,z), where
	// c = 1/6.
	let x1 = x0 - i1 as f32 + g3; // Offsets for second corner in (x,y,z) coords
	let y1 = y0 - j1 as f32 + g3;
	let z1 = z0 - k1 as f32 + g3;
	let x2 = x0 - i2 as f32 + 2.0 * g3; // Offsets for third corner in (x,y,z) coords
	let y2 = y0 - j2 as f32 + 2.0 * g3;
	let z2 = z0 - k2 as f32 + 2.0 * g3;
	let x3 = x0 - 1.0 + 3.0 * g3; // Offsets for last corner in (x,y,z) coords
	let y3 = y0 - 1.0 + 3.0 * g3;
	let z3 = z0 - 1.0 + 3.0 * g3;

	// Work out the hashed gradient indices of the four simplex corners
	let ii = i as usize & 255;
	let jj = j as usize & 255;
	let kk = k as usize & 255;
	let gi0 = PERM[ii + PERM[jj + PERM[kk]]] % 12;
	let gi1 = PERM[ii + i1 + PERM[jj + j1 + PERM[kk + k1]]] % 12;
	let gi2 = PERM[ii + i2 + PERM[jj + j2 + PERM[kk + k2]]] % 12;
	let gi3 = PERM[ii + 1 + PERM[jj + 1 + PERM[kk + 1]]] % 12;

	// Calculate the contribution from the four corners
	let mut t0 = 0.5 - x0 * x0 - y0 * y0 - z0 * z0;
	if t0 < 0.0 {
		n0 = 0.0;
	} else {
		t0 *= t0;
		n0 = t0 * t0 * dot(GRAD3[gi0], x0, y0, z0);
	}
	let mut t1 = 0.5 - x1 * x1 - y1 * y1 - z1 * z1;
	if t1 < 0.0 {
		n1 = 0.0;
	} else {
		t1 *= t1;
		n1 = t1 * t1 * dot(GRAD3[gi1], x1, y1, z1);
	}
	let mut t2 = 0.5 - x2 * x2 - y2 * y2 - z2 * z2;
	if t2 < 0.0 {
		n2 = 0.0;
	} else {
		t2 *= t2;
		n2 = t2 * t2 * dot(GRAD3[gi2], x2, y2, z2);
	}
	let mut t3 = 0.5 - x3 * x3 - y3 * y3 - z3 * z3;
	if t3 < 0.0 {
		n3 = 0.0;
	} else {
		t3 *= t3;
		n3 = t3 * t3 * dot(GRAD3[gi3], x3, y3, z3);
	}

	// Add contributions from each corner to get the final noise value.
	// The result is scaled to stay just inside [-1,1]
	return 32.0 * (n0 + n1 + n2 + n3);
}

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::OutputPin;
// Inspired by
// - https://github.com/polyfloyd/ledcat/blob/master/src/device/hub75.rs
// - https://github.com/mmou/led-marquee/blob/8c88531a6938edff6db829ca21c15304515874ea/src/hub.rs
// - https://github.com/adafruit/RGB-matrix-Panel/blob/master/RGBmatrixPanel.cpp
// - https://www.mikrocontroller.net/topic/452187 (sorry, german only)

/// # Theory of Operation
/// This display is essentially split in half, with the top 16 rows being
/// controlled by one set of shift registers (r1, g1, b1) and the botton 16
/// rows by another set (r2, g2, b2). So, the best way to update it is to
/// show one of the botton and top rows in tandem. The row (between 0-15) is then
/// selected by the A, B, C, D pins, which are just, as one might expect, the bits 0 to 3.
///
/// The display doesn't really do brightness, so we have to do it ourselves, by
/// rendering the same frame multiple times, with some pixels being turned of if
/// they are darker (pwm)

pub struct Hub75<PINS> {
	//       r1, g1, b1, r2, g2, b2, column, row
	data: [[(u8, u8, u8, u8, u8, u8); 64]; 16],
	brightness_step: u8,
	brightness_count: u8,
	pins: PINS,
}

/// A trait, so that it's easier to reason about the pins
/// Implemented for a tuple `(r1, g1, b1, r2, g2, b2, a, b, c, d, clk, lat, oe)`
/// with every element implementing `OutputPin`
pub trait Outputs {
	type R1: OutputPin;
	type G1: OutputPin;
	type B1: OutputPin;
	type R2: OutputPin;
	type G2: OutputPin;
	type B2: OutputPin;
	type A: OutputPin;
	type B: OutputPin;
	type C: OutputPin;
	type D: OutputPin;
	type CLK: OutputPin;
	type LAT: OutputPin;
	type OE: OutputPin;
	fn r1(&mut self) -> &mut Self::R1;
	fn g1(&mut self) -> &mut Self::G1;
	fn b1(&mut self) -> &mut Self::B1;
	fn r2(&mut self) -> &mut Self::R2;
	fn g2(&mut self) -> &mut Self::G2;
	fn b2(&mut self) -> &mut Self::B2;
	fn a(&mut self) -> &mut Self::A;
	fn b(&mut self) -> &mut Self::B;
	fn c(&mut self) -> &mut Self::C;
	fn d(&mut self) -> &mut Self::D;
	fn clk(&mut self) -> &mut Self::CLK;
	fn lat(&mut self) -> &mut Self::LAT;
	fn oe(&mut self) -> &mut Self::OE;
}

impl<
	R1: OutputPin,
	G1: OutputPin,
	B1: OutputPin,
	R2: OutputPin,
	G2: OutputPin,
	B2: OutputPin,
	A: OutputPin,
	B: OutputPin,
	C: OutputPin,
	D: OutputPin,
	CLK: OutputPin,
	LAT: OutputPin,
	OE: OutputPin,
> Outputs for (R1, G1, B1, R2, G2, B2, A, B, C, D, CLK, LAT, OE)
{
	type R1 = R1;
	type G1 = G1;
	type B1 = B1;
	type R2 = R2;
	type G2 = G2;
	type B2 = B2;
	type A = A;
	type B = B;
	type C = C;
	type D = D;
	type CLK = CLK;
	type LAT = LAT;
	type OE = OE;
	fn r1(&mut self) -> &mut R1 {
		&mut self.0
	}
	fn g1(&mut self) -> &mut G1 {
		&mut self.1
	}
	fn b1(&mut self) -> &mut B1 {
		&mut self.2
	}
	fn r2(&mut self) -> &mut R2 {
		&mut self.3
	}
	fn g2(&mut self) -> &mut G2 {
		&mut self.4
	}
	fn b2(&mut self) -> &mut B2 {
		&mut self.5
	}
	fn a(&mut self) -> &mut A {
		&mut self.6
	}
	fn b(&mut self) -> &mut B {
		&mut self.7
	}
	fn c(&mut self) -> &mut C {
		&mut self.8
	}
	fn d(&mut self) -> &mut D {
		&mut self.9
	}
	fn clk(&mut self) -> &mut CLK {
		&mut self.10
	}
	fn lat(&mut self) -> &mut LAT {
		&mut self.11
	}
	fn oe(&mut self) -> &mut OE {
		&mut self.12
	}
}

impl<PINS: Outputs> Hub75<PINS> {
	/// Create a new hub instance
	///
	/// Takes an implementation of the Outputs trait,
	/// using a tuple `(r1, g1, b1, r2, g2, b2, a, b, c, d, clk, lat, oe)`,
	/// with every member implementing `OutputPin` is usually the right choice.
	///
	/// `brightness_bits` provides the number of brightness_bits for each color (1-8).
	/// More bits allow for much more colors, especially in combination with the gamma correction,
	/// but each extra bit doubles the time `output` will take. This might lead to noticable flicker.
	///
	/// 3-4 bits are usually a good choice.
	pub fn new(pins: PINS, brightness_bits: u8) -> Self {
		assert!(brightness_bits < 9 && brightness_bits > 0);
		let data = [[(0, 0, 0, 0, 0, 0); 64]; 16];
		let brightness_step = 1 << (8 - brightness_bits);
		let brightness_count = ((1 << brightness_bits as u16) - 1) as u8;
		Self {
			data,
			brightness_step,
			brightness_count,
			pins,
		}
	}

	/// Output the buffer to the display
	///
	/// Takes some time and should be called quite often, otherwise the output
	/// will flicker
	pub fn output<DELAY: DelayUs<u8>>(&mut self, delay: &mut DELAY) {
		// Enable the output
		// The previous last row will continue to display
		// self.pins.oe().set_low().ok();
		// PWM cycle
		for mut brightness in 0..self.brightness_count {
			brightness = (brightness + 1).saturating_mul(self.brightness_step);
			for (count, row) in self.data.iter().enumerate() {

				self.pins.oe().set_low().ok();
				for element in row.iter() {
					if element.0 >= brightness {
						self.pins.r1().set_high().ok();
					} else {
						self.pins.r1().set_low().ok();
					}
					if element.1 >= brightness {
						self.pins.g1().set_high().ok();
					} else {
						self.pins.g1().set_low().ok();
					}
					if element.2 >= brightness {
						self.pins.b1().set_high().ok();
					} else {
						self.pins.b1().set_low().ok();
					}
					if element.3 >= brightness {
						self.pins.r2().set_high().ok();
					} else {
						self.pins.r2().set_low().ok();
					}
					if element.4 >= brightness {
						self.pins.g2().set_high().ok();
					} else {
						self.pins.g2().set_low().ok();
					}
					if element.5 >= brightness {
						self.pins.b2().set_high().ok();
					} else {
						self.pins.b2().set_low().ok();
					}
					// delay.delay_us(10);
					self.pins.clk().set_high().ok();
					// delay.delay_us(10);
					self.pins.clk().set_low().ok();
					// delay.delay_us(10);
				}
				self.pins.oe().set_high().ok();

				self.pins.lat().set_high().ok();
				self.pins.lat().set_low().ok();
				// delay.delay_us(5);
				// Select row
				// delay.delay_us(2);
				if count & 1 != 0 {
					self.pins.a().set_high().ok();
				} else {
					self.pins.a().set_low().ok();
				}
				// delay.delay_us(1);
				if count & 2 != 0 {
					self.pins.b().set_high().ok();
				} else {
					self.pins.b().set_low().ok();
				}
				// delay.delay_us(1);
				if count & 4 != 0 {
					self.pins.c().set_high().ok();
				} else {
					self.pins.c().set_low().ok();
				}
				// delay.delay_us(1);
				if count & 8 != 0 {
					self.pins.d().set_high().ok();
				} else {
					self.pins.d().set_low().ok();
				}
				// Prevents ghosting, no idea why
				// delay.delay_us(1);
				// delay.delay_us(2);
				// delay.delay_us(50);

			}
		}
		// Disable the output
		// Prevents one row from being much brighter than the others
		// self.pins.oe().set_high().ok();
	}
	/// Clear the output
	///
	/// It's a bit faster than using the embedded_graphics interface
	/// to do the same
	pub fn clear(&mut self) {
		for row in self.data.iter_mut() {
			for e in row.iter_mut() {
				e.0 = 0;
				e.1 = 0;
				e.2 = 0;
				e.3 = 0;
				e.4 = 0;
				e.5 = 0;
			}
		}
	}
}

use embedded_graphics::{
	drawable::{Dimensions, Pixel},
	// pixelcolor::Rgb565,
	Drawing, SizedDrawing,
};

impl<PINS: Outputs> Drawing<Rgb565> for Hub75<PINS> {
	fn draw<T>(&mut self, item_pixels: T)
		where
			T: IntoIterator<Item = Pixel<Rgb565>>,
	{
		// This table remaps linear input values
		// (the numbers we’d like to use; e.g. 127 = half brightness)
		// to nonlinear gamma-corrected output values
		// (numbers producing the desired effect on the LED;
		// e.g. 36 = half brightness).
		const GAMMA8: [u8; 256] = [
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4,
			4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11,
			12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22,
			22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37,
			38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58,
			59, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85,
			86, 87, 89, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
			115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142, 144,
			146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175, 177, 180,
			182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213, 215, 218, 220,
			223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255,
		];
		for Pixel(coord, color) in item_pixels {
			let row = coord[1] % 16;
			let data = &mut self.data[row as usize][coord[0] as usize];
			if coord[1] >= 16 {
				data.3 = GAMMA8[color.r() as usize];
				data.4 = GAMMA8[color.g() as usize];
				data.5 = GAMMA8[color.b() as usize];
			} else {
				data.0 = GAMMA8[color.r() as usize];
				data.1 = GAMMA8[color.g() as usize];
				data.2 = GAMMA8[color.b() as usize];
			}
		}
	}
}

// TODO Does it make sense to include this?
impl<PINS: Outputs> SizedDrawing<Rgb565> for Hub75<PINS> {
	fn draw_sized<T>(&mut self, item_pixels: T)
		where
			T: IntoIterator<Item = Pixel<Rgb565>> + Dimensions,
	{
		self.draw(item_pixels);
	}
}
