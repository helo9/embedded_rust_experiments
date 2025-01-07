//! # Pico Measure Experiment
//!
//! Measure some voltages using the ADC at pins GP26_A0, GP27_A1 and GP28_A2
//! and write the values out via usb serial.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use cortex_m::interrupt;
use cortex_m_rt::exception;
use dht_sensor::dht11::Reading;
use embedded_hal_0_2::digital::v2::OutputPin;
use panic_halt as _;

use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::adc::AdcPin;
use rp_pico::hal::Adc;
use rp_pico::hal::Clock;
use rp_pico::hal::Timer;
use rp_pico::hal::Watchdog;
use rp_pico::hal::{clocks, pac, usb};

use embedded_hal_0_2::timer::CountDown;
use embedded_hal_0_2::{adc::OneShot, digital::v2::ToggleableOutputPin};

use fugit::{MillisDurationU32, ExtU32};

use rp_pico::Pins;

// usb related

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

// dht11 humidity sensor

use dht_sensor::{dht11, DhtReading};

// custom data transport

use pico_measure_transport::{pack, Measurement, MeasuredQuantity, MeasurementGroup};

struct Board {
    pins: Pins,
    timer: Timer,
    adc: Adc,
    usb_bus: UsbBusAllocator<hal::usb::UsbBus>,
    delay: cortex_m::delay::Delay,
}

impl Board {
    pub fn take() -> Option<Self> {
        Some(Self::new(
            pac::Peripherals::take()?,
            pac::CorePeripherals::take()?,
        ))
    }

    fn new(mut pac: pac::Peripherals, mut core: pac::CorePeripherals) -> Self {
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

        const TICKS_PER_MS: u32 = 125_000; // SysTick frequency: 125 MHz / 1000 = 125000 ticks per millisecond
        core.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        core.SYST.set_reload(TICKS_PER_MS - 1); // Set reload to 1ms
        core.SYST.clear_current(); // Clear current value
        core.SYST.enable_counter(); // Enable the counter
        core.SYST.enable_interrupt();

        let sio = hal::Sio::new(pac.SIO);

        let pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

        let adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        ));

        let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        Board {
            pins,
            timer,
            adc,
            usb_bus,
            delay
        }
    }
}

const FACTOR: f32 = 105.6f32 / 5.6f32 * 3.3f32 / 4096.0f32;
static MILLIS: portable_atomic::AtomicU32 = portable_atomic::AtomicU32::new(0);

#[entry]
fn main() -> ! {
    let mut board = Board::take().unwrap();

    let mut countdown = board.timer.count_down();

    // Enable the temperature sensor
    let mut temperature_sensor = board.adc.take_temp_sensor().unwrap();
    let mut adc_pin1 = AdcPin::new(board.pins.gpio26.into_floating_input()).unwrap();
    let mut adc_pin2 = AdcPin::new(board.pins.gpio27.into_floating_input()).unwrap();
    let mut adc_pin3 = AdcPin::new(board.pins.gpio28.into_floating_input()).unwrap();

    // Set the LED to be an output

    let mut led_pin = board.pins.led.into_push_pull_output();

    // DHT sensor
    let mut dht11_pin: hal::gpio::InOutPin<hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionNull, hal::gpio::PullDown>> = hal::gpio::InOutPin::new(board.pins.gpio0);
    let _ = dht11_pin.set_high();

    // USB Device setup

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
            let temperature_adc_counts: u32 = board.adc.read(&mut temperature_sensor).unwrap();
            let voltage1_counts: u32 = board.adc.read(&mut adc_pin1).unwrap();
            let voltage2_counts: u32 = board.adc.read(&mut adc_pin2).unwrap();
            let voltage3_counts: u32 = board.adc.read(&mut adc_pin3).unwrap();

            let millis = MILLIS.load(portable_atomic::Ordering::Relaxed);

            let (temp2, rel_humidity) = match dht11::Reading::read(&mut board.delay, &mut dht11_pin) {
                Ok(r) => (Some(r.temperature), Some(r.relative_humidity)),
                Err(_) => (None, None)
            };

            let measurment = MeasurementGroup {
                millis,
                measurements: [
                    Some(Measurement {
                        value:temperature_from_cnts(temperature_adc_counts),
                        sensor_id: 0u8
                    }),
                    Some(Measurement {
                        value:MeasuredQuantity::Volt(FACTOR * (voltage1_counts as f32)),
                        sensor_id: 26u8
                    }),
                    Some(Measurement {
                        value:MeasuredQuantity::Volt(FACTOR * (voltage2_counts as f32)),
                        sensor_id: 27u8
                    }),
                    Some(Measurement { value: MeasuredQuantity::Celsius(20f32), sensor_id: 1 }),
                    Some(Measurement {
                        value: MeasuredQuantity::Volt(FACTOR * (voltage3_counts as f32)),
                        sensor_id: 28u8
                    }),
                    temp2.map(|t| Measurement { value: MeasuredQuantity::Celsius(t as f32), sensor_id: 1}),
                    rel_humidity.map(|rh| Measurement{ value: MeasuredQuantity::RelativeHumidity(rh as u16), sensor_id: 2 }),
                    None
                ]
            };

            // Serialize the measurement to a compact binary format using Postcard
            match pack::<256>(&measurment) {
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

#[exception]
fn SysTick() {
    MILLIS.add(1, portable_atomic::Ordering::Relaxed);
}

fn temperature_from_cnts(cnts: u32) -> MeasuredQuantity {
    
    let measured_volts = 3.3 / 4095.0 * (cnts as f32);

    let temperature = 27.0 - ( measured_volts - 0.706) / 0.001721;

    MeasuredQuantity::Celsius(temperature)
}