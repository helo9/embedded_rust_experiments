//! # Pico Measure Experiment
//!
//! Measure some voltages using the ADC at pins GP26_A0, GP27_A1 and GP28_A2
//! and write the values out via usb serial.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use fugit::MillisDurationU32;
use panic_halt as _;

use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::clocks::ClocksManager;
use rp_pico::hal::Adc;
use rp_pico::hal::Timer;
use rp_pico::hal::Watchdog;
use rp_pico::hal::{clocks, pac, usb};

use embedded_hal_0_2::timer::CountDown;
use embedded_hal_0_2::{adc::OneShot, digital::v2::ToggleableOutputPin};

use fugit::{Duration, ExtU32};
use rp_pico::Pins;
// usb related
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use pico_measure_transport::{pack, Measurement, MeasurementUnit};

struct Board {
    core: pac::CorePeripherals,
    watchdog: Watchdog,
    pins: Pins,
    timer: Timer,
    adc: Adc,
    usb_bus: UsbBusAllocator<hal::usb::UsbBus>,
}

impl Board {
    pub fn take() -> Option<Self> {
        Some(Self::new(
            pac::Peripherals::take()?,
            pac::CorePeripherals::take()?,
        ))
    }

    fn new(mut pac: pac::Peripherals, core: pac::CorePeripherals) -> Self {
        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

        let sio = hal::Sio::new(pac.SIO);

        let pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

        let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        ));

        Board {
            core,
            watchdog,
            pins,
            timer,
            adc,
            usb_bus,
        }
    }
}

#[entry]
fn main() -> ! {
    let mut board = Board::take().unwrap();

    setup_systick(&mut board.core.SYST);

    let mut countdown = board.timer.count_down();

    // Enable the temperature sensor
    let mut temperature_sensor = board.adc.take_temp_sensor().unwrap();

    // Set the LED to be an output
    let mut led_pin = board.pins.led.into_push_pull_output();

    let mut usb_serial = SerialPort::new(&board.usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&board.usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    countdown.start(500.millis());

    loop {
        if countdown.wait().is_ok() {
            countdown.start(500.millis());
            led_pin.toggle().unwrap();
            let temperature_adc_counts: u16 = board.adc.read(&mut temperature_sensor).unwrap();

            let millis = get_ticks_since_startup();

            let measurment = Measurement {
                millis,
                measurement: temperature_adc_counts,
                unit: MeasurementUnit::Counts,
                sensor_id: 0u8,
            };

            // Serialize the measurement to a compact binary format using Postcard
            match pack::<32>(&measurment) {
                Ok(serialized) => {
                    // Send the serialized data over USB
                    match usb_serial.write(&serialized) {
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
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut usb_serial]) {
            let mut buf = [0u8; 64];
            match usb_serial.read(&mut buf) {
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
                        match usb_serial.write(wr_ptr) {
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
