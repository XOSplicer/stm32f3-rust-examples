// This code is part of an article on using DMA with Embedded Rust
//
// See: https://flowdsp.io/blog/stm32f3-02-dac-dma/

#![no_std]
#![no_main]

#[macro_use]
extern crate lazy_static;

#[allow(unused_extern_crates)]
extern crate panic_itm;

use core::cell::RefCell;

use cortex_m;
use cortex_m::{iprintln, interrupt::Mutex};
use cortex_m_rt::entry;

use stm32f3::stm32f303;
use stm32f303::interrupt;

use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::delay::Delay;


use atomic::Atomic;

mod wavetable;
use wavetable::WaveTableSampler;


// - leds ---------------------------------------------------------------------

pub fn init_leds(dp: &stm32f303::Peripherals) {
    // enable GPIOE clock
    let rcc = &dp.RCC;
    rcc.ahbenr.modify(|_, w| w.iopeen().set_bit());

    // configure LED pins
    let gpioe = &dp.GPIOE;
    gpioe.moder.modify(|_, w| w.moder8().output()     // LED: LD4  on pin PE08 (blue)
                               .moder9().output()     // LED: LD3  on pin PE09 (red)
                               .moder10().output()    // LED: LD5  on pin PE10 (orange)
                               .moder11().output()    // LED: LD7  on pin PE11 (green)
                               .moder12().output()    // LED: LD9  on pin PE12 (blue)
                               .moder13().output()    // LED: LD10 on pin PE13 (red)
                               .moder14().output()    // LED: LD8  on pin PE14 (orange)
                               .moder15().output());  // LED: LD6  on pin PE15 (green)
}


// - 2. tim2 ------------------------------------------------------------------

pub fn init_tim2(dp: &stm32f303::Peripherals) {
    // enable TIM2 clock
    let rcc = &dp.RCC;
    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());

    // calculate timer frequency
    let sysclk = 8_000_000;       // the stmf32f3 discovery board CPU runs at 8Mhz by defaBult
    let fs = 44_100;              // we want an audio sampling rate of 44.1KHz
    let arr = sysclk / fs;        // value to use for auto reload register (arr)

    // configure TIM2
    let tim2 = &dp.TIM2;
    tim2.cr2.write(|w| w.mms().update());       // trigger interrupt when counter reaches arr value
    tim2.arr.write(|w| w.arr().bits(arr));      // timer period (sysclk / fs)
    tim2.cr1.modify(|_, w| w.cen().enabled());  // enable TIM2
}


// - 3. dac1 ------------------------------------------------------------------

pub fn init_dac1(dp: &stm32f303::Peripherals) {
    // enable GPIOA and DAC clocks
    let rcc = &dp.RCC;
    rcc.ahbenr.modify(|_, w| w.iopaen().set_bit());
    rcc.apb1enr.modify(|_, w| w.dacen().set_bit());

    // configure PA04, PA05 (DAC_OUT1 & DAC_OUT2) as analog, floating
    let gpioa = &dp.GPIOA;
    gpioa.moder.modify(|_, w| w.moder4().analog()
                               .moder5().analog());
    gpioa.pupdr.modify(|_, w| w.pupdr4().floating()
                               .pupdr5().floating());

    // configure DAC
    let dac = &dp.DAC;
    dac.cr.write(|w| w.boff1().disabled()     // disable dac output buffer for channel 1
                      .boff2().disabled()     // disable dac output buffer for channel 2
                      .ten1().enabled()       // enable trigger for channel 1
                      .ten2().enabled()       // enable trigger for channel 2
                      .tsel1().tim2_trgo()    // set trigger for channel 1 to TIM2
                      .tsel2().tim2_trgo());  // set trigger for channel 2 to TIM2

    // enable DAC
    dac.cr.modify(|_, w| w.en1().enabled()    // enable dac channel 1
                          .en2().enabled());  // enable dac channel 2
}


// - 4. dma -------------------------------------------------------------------

lazy_static! {
    static ref MUTEX_DMA2:  Mutex<RefCell<Option<stm32f303::DMA2>>>  = Mutex::new(RefCell::new(None));
    static ref MUTEX_GPIOE: Mutex<RefCell<Option<stm32f303::GPIOE>>> = Mutex::new(RefCell::new(None));
}

const DMA_LENGTH:usize = 64;
static mut DMA_BUFFER: [u32; DMA_LENGTH] = [0; DMA_LENGTH];


pub fn init_dma2(cp: &mut cortex_m::peripheral::Peripherals, dp: &stm32f303::Peripherals) {
    // enable DMA2 clock
    let rcc = &dp.RCC;
    rcc.ahbenr.modify(|_, w| w.dma2en().set_bit());

    // dma parameters
    let ma = unsafe { DMA_BUFFER.as_ptr() } as usize as u32;
    let pa = 0x40007420; // Dual DAC 12-bit right-aligned data holding register (DHR12RD)
    let ndt = DMA_LENGTH as u16;

    // configure DMA2 channel 3
    let dma2 = &dp.DMA2;
    dma2.cmar3.write(|w| w.ma().bits(ma));     // source memory address
    dma2.cpar3.write(|w| w.pa().bits(pa));     // destination peripheral address
    dma2.cndtr3.write(|w| w.ndt().bits(ndt));  // number of items to transfer
    dma2.ccr3.write(|w| {
        w.dir().from_memory()   // source is memory
         .mem2mem().disabled()  // disable memory to memory transfer
         .minc().enabled()      // increment memory address every transfer
         .pinc().disabled()     // don't increment peripheral address every transfer
         .msize().bit32()       // memory word size is 32 bits
         .psize().bit32()       // peripheral word size is 32 bits
         .circ().enabled()      // dma mode is circular
         .pl().high()           // set dma priority to high
         .teie().enabled()      // trigger an interrupt if an error occurs
         .tcie().enabled()      // trigger an interrupt when transfer is complete
         .htie().enabled()      // trigger an interrupt when half the transfer is complete
    });

    // enable DMA interrupt
    let nvic = &mut cp.NVIC;
    nvic.enable(stm32f303::Interrupt::DMA2_CH3);

    // enable DMA transfers for DAC
    let dac = &dp.DAC;
    dac.cr.modify(|_, w| w.dmaen1().enabled());
}


// - 5. dma interrupt handler -------------------------------------------------

#[interrupt]
fn DMA2_CH3() {
    enum State { HT, TC, Error, Unknown };

    // determine dma state
    let state = cortex_m::interrupt::free(|cs| {
        let refcell = MUTEX_DMA2.borrow(cs).borrow();
        let dma2 = match refcell.as_ref() { None => return State::Error, Some(v) => v };

        // cache interrupt status before clearing interrupt flag
        let isr = dma2.isr.read();

        // clear interrupt flag and return dma state
        if isr.tcif3().is_complete() {
            dma2.ifcr.write(|w| w.ctcif3().clear());
            return State::TC;
        } else if isr.htif3().is_half() {
            dma2.ifcr.write(|w| w.chtif3().clear());
            return State::HT;
        } else if isr.teif3().is_error() {
            dma2.ifcr.write(|w| w.cteif3().clear());
            return State::Error;
        }
        return State::Unknown;
    });

    // invoke audio callback
    match state {
        State::HT => audio_callback(unsafe { &mut DMA_BUFFER }, DMA_LENGTH / 2, 0),
        State::TC => audio_callback(unsafe { &mut DMA_BUFFER }, DMA_LENGTH / 2, 1),
        _ => (),
    }

    // set leds to state
    cortex_m::interrupt::free(|cs| {
        let refcell = MUTEX_GPIOE.borrow(cs).borrow();
        let gpioe = match refcell.as_ref() { None => return, Some(v) => v };
        match state {
            State::HT => gpioe.odr.modify(|_, w| w.odr9().low().odr11().high().odr15().low()),
            State::TC => gpioe.odr.modify(|_, w| w.odr9().low().odr11().low().odr15().high()),
            State::Error => gpioe.odr.modify(|_, w| w.odr9().high().odr11().low().odr15().low()),
            State::Unknown => gpioe.odr.modify(|_, w| w.odr9().low().odr11().low().odr15().low()),
        }
    });
}


// - 6. audio callback --------------------------------------------------------


lazy_static! {
    static ref SAMPLER_1: WaveTableSampler = WaveTableSampler::new(&wavetable::SIN, 44_100., 440.);
}


fn audio_callback(buffer: &mut [u32; DMA_LENGTH], length: usize, offset: usize) {
    for t in 0..length {
        let channel_1 = SAMPLER_1.sample() as u32;
        let frame = t + (offset * length);
        buffer[frame] = (0 << 16) + channel_1;
    }

}


// - 7. main ------------------------------------------------------------------

#[entry]
fn main() -> ! {
    assert!(Atomic::<f32>::is_lock_free());

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f303::Peripherals::take().unwrap();

    // initialize peripherals
    init_leds(&dp);
    init_tim2(&dp);
    init_dac1(&dp);
    init_dma2(&mut cp, &dp);

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, clocks);
    let dma_move = dp.DMA2;
    let gpioe_move = dp.GPIOE;

    let mut itm = cp.ITM;
    iprintln!(&mut itm.stim[0], "synth-stm32f3 initialized peripherals");



    // wrap shared peripherals
    cortex_m::interrupt::free(|cs| {
        MUTEX_DMA2.borrow(cs).replace(Some(dma_move));
        MUTEX_GPIOE.borrow(cs).replace(Some(gpioe_move));
    });



    // enable DMA to start transfer
    cortex_m::interrupt::free(|cs| {
        let refcell = MUTEX_DMA2.borrow(cs).borrow();
        let dma2 = refcell.as_ref().unwrap();
        dma2.ccr3.modify(|_, w| w.en().enabled());
    });

    // enter main loop
    loop {
        delay.delay_ms(1000_u32);
        SAMPLER_1.set_tone_frequency(440.);
        delay.delay_ms(1000_u32);
        SAMPLER_1.set_tone_frequency(880.);
    }
}
