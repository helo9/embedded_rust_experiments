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
use rp_pico::hal;
use rp_pico::hal::clocks::ClocksManager;
use rp_pico::hal::{clocks, pac, usb};

use embedded_hal_0_2::timer::CountDown;
use embedded_hal_0_2::{adc::OneShot, digital::v2::ToggleableOutputPin};

use fugit::ExtU32;

// usb related
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use pico_measure_transport::{pack, Measurement, MeasurementUnit};

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core_peripherals = cortex_m::Peripherals::take().unwrap();

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

    setup_systick(&mut core_peripherals.SYST);

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut countdown = timer.count_down();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    countdown.start(500.millis());

    loop {
        if countdown.wait().is_ok() {
            led_pin.toggle().unwrap();
            let temperature_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();

            let measurment = Measurement {
                millis: get_ticks_since_startup(),
                measurement: temperature_adc_counts,
                unit: MeasurementUnit::Counts,
                sensor_id: 0u8,
            };

            // Serialize the measurement to a compact binary format using Postcard
            match pack::<32>(&measurment) {
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

            countdown.start(500.millis());
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

// Setup SysTick to trigger an interrupt every 1 ms (125 MHz system clock)
fn setup_systick(systck: &mut pac::SYST) {
    const TICKS_PER_MS: u32 = 125_000; // SysTick frequency: 125 MHz / 1000 = 125000 ticks per millisecond
    systck.set_reload(TICKS_PER_MS - 1); // Set reload to 1ms
    systck.clear_current(); // Clear current value
    systck.enable_counter(); // Enable the counter
                             //systck.enable_interrupt(); // Enable the interrupt (optional)
}

// Get the number of ticks since startup
fn get_ticks_since_startup() -> u32 {
    let systick = unsafe { &*pac::SYST::PTR }; // Access the SysTick register
    systick.cvr.read() // Read the current value (ticks since startup)
}
