#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use panic_probe as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    flash::FlashExt,
    pac::{self, interrupt},
    prelude::*,
    rcc::{Config, RccExt},
    time::Hertz,
    timer::{Event, Remap, Tim2PartialRemap1, Timer},
};

static NUMB_OVERFLOW: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    defmt::println!("STM32F103C8 Timer Channel Pin Remap");

    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();

    let rcc = dp.RCC.constrain();

    let clock_config = Config::default()
        .use_hse(Hertz::MHz(8))
        .sysclk(Hertz::MHz(72))
        .hclk(Hertz::MHz(72))
        .pclk1(Hertz::MHz(36));

    let mut clocks = rcc.freeze(clock_config, &mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut clocks);
    let gpiob = dp.GPIOB.split(&mut clocks);

    let mut afio = dp.AFIO.constrain(&mut clocks);

    let (pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let pa15 = pa15.into_pull_down_input(&mut gpioa.crh);

    Tim2PartialRemap1::remap(&mut afio.mapr);

    let timer2 = Timer::new(dp.TIM2, &mut clocks);

    unsafe {
        pac::NVIC::unmask(interrupt::TIM2);
    }

    let mut counter = timer2.counter_hz();

    counter.listen(Event::Update);

    let timer_register = unsafe { &*pac::TIM2::ptr() };

    timer_register.ccmr1_input().modify(|_, w| w.cc1s().ti1());
    timer_register.ccmr1_input().modify(|_, w| w.cc2s().ti1());

    timer_register
        .ccmr1_input()
        .modify(|_, w| w.ic1f().fck_int_n8());
    timer_register
        .ccmr1_input()
        .modify(|_, w| w.ic2f().fck_int_n8());

    timer_register.ccer().modify(|_, w| w.cc1e().set_bit());
    timer_register.ccer().modify(|_, w| w.cc2e().set_bit());

    timer_register.ccer().modify(|_, w| w.cc1p().clear_bit());
    timer_register.ccer().modify(|_, w| w.cc2p().set_bit());

    counter.start_raw(7199, 65535);

    let mut start_press_tick = 0u16;

    let mut is_pressed = false;

    loop {
        if counter.get_interrupt().contains(Event::C1) {
            let captured_tick = timer_register.ccr1().read().ccr().bits() as u16;

            let pin_is_high = pa15.is_high();

            if pin_is_high && !is_pressed {
                start_press_tick = captured_tick;
                NUMB_OVERFLOW.store(0, Ordering::SeqCst);

                is_pressed = true;
                defmt::println!("--- Button Pressed ---");
                defmt::println!("start: {}", start_press_tick);
            }
            counter.clear_interrupt(Event::C1);
        }

        if counter.get_interrupt().contains(Event::C2) {
            let captured_tick = timer_register.ccr2().read().ccr().bits() as u16;

            let pin_is_high = pa15.is_high();
            if !pin_is_high && is_pressed {
                let total_overflow = NUMB_OVERFLOW.load(Ordering::SeqCst);

                let total_tick =
                    (total_overflow * 65536) + captured_tick as u32 - start_press_tick as u32;

                defmt::println!(
                    "end: {} | total_overflow: {}",
                    captured_tick,
                    total_overflow
                );
                defmt::println!("press long: {} ms", total_tick as f32 / 10f32);
                defmt::println!("----------------------");
                is_pressed = false;
            }

            counter.clear_interrupt(Event::C2);
        }
    }
}

#[interrupt]
fn TIM2() {
    let timer_register = unsafe { &*pac::TIM2::ptr() };

    // check update interrupt flag
    if timer_register.sr().read().uif().bit_is_set() {
        // Increment overflow count
        NUMB_OVERFLOW.fetch_add(1, Ordering::SeqCst);
        // Clear update interrupt flag
        timer_register.sr().modify(|_, w| w.uif().clear_bit());
    }
}
