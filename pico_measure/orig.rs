//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_serial::defmt_serial;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use panic_probe as _;
use static_cell::StaticCell;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

static USB: StaticCell<UsbBusAllocator<bsp::hal::usb::UsbBus>> = StaticCell::new();
static SERIAL: StaticCell<SerialPort<'_, bsp::hal::usb::UsbBus>> = StaticCell::new();

#[entry]
fn main() -> ! {
    //info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    led_pin.set_high().unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);    

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(0x02)  // CDC (Communication Device Class)
        .build();

    
    /*loop {
        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    } */

    // Run the USB device loop
    loop {

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

        // Send data if required (e.g., periodic message)
        led_pin.toggle().ok();

        // Small delay to prevent overloading the USB bus
        delay.delay_ms(1000);
    }
}

// End of file
