//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{
    digital::OutputPin,
    spi::{SpiBus, MODE_0},
};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    gpio::FunctionSpi,
    pac,
    sio::Sio,
    spi::Spi,
    watchdog::Watchdog,
};

pub enum DRV8711Register {
    CTRL = 0x00,
    TORQUE = 0x01,
    OFF = 0x02,
    BLANK = 0x03,
    DECAY = 0x04,
    STALL = 0x05,
    DRIVE = 0x06,
    STATUS = 0x07,
}

pub enum Drv8711DecayMode {
    Slow = 0b000,
    SlowIncMixedDec = 0b001,
    Fast = 0b010,
    Mixed = 0b011,
    SlowIncAutoMixedDec = 0b100,
    AutoMixed = 0b101,
}

pub enum Drv8711StepMode {
    MicroStep256,
    MicroStep128,
    MicroStep64,
    MicroStep32,
    MicroStep16,
    MicroStep8,
    MicroStep4,
    MicroStep2,
    MicroStep1,
}

pub struct Drv8711<SPI, CS> {
    spi: SPI,
    cs: CS,
    ctrl: u16,
    torque: u16,
    off: u16,
    blank: u16,
    decay: u16,
    stall: u16,
    drive: u16,
}

impl<SPI, CS> Drv8711<SPI, CS>
where
    SPI: SpiBus<u16>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs_pin: CS) -> Self {
        Drv8711 {
            spi: spi,
            cs: cs_pin,
            ctrl: 0xC10,
            torque: 0x1FF,
            off: 0x030,
            blank: 0x80,
            decay: 0x110,
            stall: 0x40,
            drive: 0xA59,
        }
    }

    pub fn set_decay_mode(&mut self, decay_mode: Drv8711DecayMode) -> Result<(), SPI::Error> {
        self.decay &= 0xFu16;
        self.decay |= ((decay_mode as u8 & 0b111u8) as u16) << 8;

        self.write_reg(DRV8711Register::DECAY as u8, self.decay)?;

        Ok(())
    }

    pub fn set_step_mode(&mut self, step_mode: Drv8711StepMode) -> Result<(), SPI::Error> {
        // Pick 1/4 micro-step by default.
        let sm = match step_mode {
            Drv8711StepMode::MicroStep1 => 0b0000u8,
            Drv8711StepMode::MicroStep2 => 0b0001u8,
            Drv8711StepMode::MicroStep4 => 0b0010u8,
            Drv8711StepMode::MicroStep8 => 0b0011u8,
            Drv8711StepMode::MicroStep16 => 0b0100u8,
            Drv8711StepMode::MicroStep32 => 0b0101u8,
            Drv8711StepMode::MicroStep64 => 0b0110u8,
            Drv8711StepMode::MicroStep128 => 0b0111u8,
            Drv8711StepMode::MicroStep256 => 0b1000u8,
            _ => 0b0010u8,
        };

        self.ctrl &= 0b111110000111;
        self.ctrl |= (sm as u16) << 3;

        self.write_reg(DRV8711Register::CTRL as u8, self.ctrl)?;

        Ok(())
    }

    pub fn set_milliamps(&mut self, mut current: u16) -> Result<(), SPI::Error> {

        if current > 8000 {
            current = 8000;
        }

        let mut torque_bits: u16 = ((768 as u32 * current as u32) / 6875) as u16;
        let mut isgain_bits: u8 = 0b11;

        while torque_bits > 0xFF {
            torque_bits >>= 1;
            isgain_bits -= 1;
        }

        self.ctrl = (self.ctrl & 0b110011111111) | ((isgain_bits as u16) << 8);
        self.torque =  (self.torque & 0b111100000000) | torque_bits;

        info!("Got new current, using 0b{:03b} for gain and {} for torque", isgain_bits, torque_bits);

        self.write_reg(DRV8711Register::CTRL as u8, self.ctrl)?;
        self.write_reg(DRV8711Register::TORQUE as u8, self.torque)?;

        Ok(())
    }

    pub fn write_defaults(&mut self) -> Result<(), SPI::Error> {

        const DEFAULT_CTRL: u16 = 0xC10;
        self.write_reg(DRV8711Register::CTRL as u8, DEFAULT_CTRL)?;

        const DEFAULT_TORQUE: u16 = 0x1FF;
        self.write_reg(DRV8711Register::TORQUE as u8, DEFAULT_TORQUE)?;

        const DEFAULT_OFF: u16 = 0x030;
        self.write_reg(DRV8711Register::OFF as u8, DEFAULT_OFF)?;

        const DEFAULT_BLANK: u16 = 0x080;
        self.write_reg(DRV8711Register::BLANK as u8, DEFAULT_BLANK)?;

        const DEFAULT_DECAY: u16 = 0x110;
        self.write_reg(DRV8711Register::DECAY as u8, DEFAULT_DECAY)?;

        const DEFAULT_DRIVE: u16 = 0xA59;
        self.write_reg(DRV8711Register::DRIVE as u8, DEFAULT_DRIVE)?;

        const DEFAULT_STALL: u16 = 0x040;
        self.write_reg(DRV8711Register::STALL as u8, DEFAULT_STALL)?;

        Ok(())
    }

    pub fn enable(&mut self) -> Result<(), SPI::Error> {
        self.ctrl |= 0b1;

        self.write_reg(DRV8711Register::CTRL as u8, self.ctrl)?;

        Ok(())
    }

    pub fn step(&mut self) -> Result<(), SPI::Error> {
        self.write_reg(DRV8711Register::CTRL as u8, self.ctrl | (1 << 2))?;

        Ok(())
    }

    fn write_reg(&mut self, address: u8, value: u16) -> Result<(), SPI::Error> {
        // Read/write bit and register address are the first 4 bits of the first
        // byte; data is in the remaining 4 bits of the first byte combined with
        // the second byte (12 bits total).

        info!("Writing Register: addres {}, value {}", address, value);

        let command = (((address & 0b111) as u16) << 12) | (value & 0xFFF);

        info!("writing command {:016b}", command);

        self.cs.set_high().unwrap();

        self.spi.write(&[command])?;

        self.cs.set_low().unwrap();

        Ok(())
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
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

    let sclk = pins.gpio18.into_function::<FunctionSpi>();
    let sdo = pins.gpio19.into_function::<FunctionSpi>();
    let sdi = pins.gpio16.into_function::<FunctionSpi>();

    let spi_device = pac.SPI0;
    let spi_pin_layout = (sdo, sdi, sclk);

    let spi = Spi::<_, _, _, 16>::new(spi_device, spi_pin_layout).init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        1_000_000u32.Hz(),
        MODE_0,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    let spi_cs_pin = pins.gpio17.into_push_pull_output();

    let mut drv8711 = Drv8711::new(spi, spi_cs_pin);

    drv8711.write_defaults().unwrap();
    drv8711.set_milliamps(5500).unwrap();
    drv8711.enable().unwrap();

    info!("Stepper driver enabled!");

    loop {
        drv8711.step().unwrap();
        led_pin.set_high().unwrap();
        delay.delay_ms(2);

        drv8711.step().unwrap();
        led_pin.set_low().unwrap();
        delay.delay_ms(2);
    }
}

// End of file
