#![no_std]
#![no_main]

use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, Timer0Pwm};
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    /*
     * For examples (and inspiration), head to
     *
     *     https://github.com/Rahix/avr-hal/tree/main/examples
     *
     * NOTE: Not all examples were ported to all boards!  There is a good chance though, that code
     * for a different board can be adapted for yours.  The Arduino Uno currently has the most
     * examples available.
     */

    let mut led = pins.d13.into_output();

    let mut timer2 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale64);


    let mut d5 = pins.d5.into_output().into_pwm(&mut timer2);
    
    d5.set_duty(128);
    d5.enable();

    loop {
        led.toggle();
        arduino_hal::delay_ms(1000);
    }
}
