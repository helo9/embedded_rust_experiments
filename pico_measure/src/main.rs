//! # Pico Measure Experiment
//!
//! Measure some voltages using the ADC at pins GP26_A0, GP27_A1 and GP28_A2
//! and write the values out via usb serial.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use panic_halt as _;

use rp_pico::entry;
use rp_pico::hal::pac;
use rp_pico::hal;

use embedded_hal_0_2::{adc::OneShot, digital::v2::ToggleableOutputPin};
use embedded_hal_0_2::timer::CountDown;

use fugit::ExtU32;

// usb related
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use pico_measure_transport::{Measurement, pack};

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut count_down = timer.count_down();

    #[cfg(feature = "rp2040-e5")]
    {
        let sio = hal::Sio::new(pac.SIO);
        let _pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
    }

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    // Enable adc
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    // Enable the temperature sensor
    let mut temperature_sensor = adc.take_temp_sensor().unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    
    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    count_down.start(500.millis());

    loop {

        if count_down.wait().is_ok() {
            led_pin.toggle().unwrap();
            let temperature_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();

            let measurment = Measurement {
                millis: 0u32,
                measurement: temperature_adc_counts,
                sensor_id: 0u8,
            };
            
            // Serialize the measurement to a compact binary format using Postcard
            match pack(&measurment) {
                Ok(serialized) => {
                    // Send the serialized data over USB
                    match serial.write(&serialized) {
                        Ok(_) => {
                            // Successfully sent the data
                        }
                        Err(_e) => {
                            // Didn't work..
                        }
                    }
                }
                Err(_e) => {
                    // Didn't work as well :(
                }
            }

            count_down.start(500.millis());
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }

    }
}

// End of file