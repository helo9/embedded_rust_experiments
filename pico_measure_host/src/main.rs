use std::fmt;
use std::time::Duration;
use std::io::{self, Read};

use pico_measure_transport::{Measurement, unpack};

struct PicoTransErr(pico_measure_transport::Error);

impl fmt::Display for PicoTransErr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            pico_measure_transport::Error::InvalidInput(_) => write!(f, "Invalid input"),
            pico_measure_transport::Error::TooFewBufferElements => write!(f, "Too few buffer elements"),
        }
    }
}

fn main() -> io::Result<()> {
    // Open the USB serial port (change as needed for your OS)
    let port_name = "/dev/ttyACM0";  // Replace with your serial port
    let baudrate = 115200;

    let mut port = serialport::new(port_name, baudrate)
        .timeout(Duration::from_millis(700))
        .open()?;

    let mut buffer = [0u8; 128];
    let mut buffer_cursor = 0usize;

    println!("Initialization complete");

    loop {
        
        match port.read(&mut buffer[buffer_cursor..]) {
            Ok(n) => {
                buffer_cursor += n;
                if buffer_cursor >= buffer.len() {
                    buffer_cursor = 0usize;
                }
                println!("was reading {} bytes, we have {}", n, buffer_cursor);
            },
            Err(e) => {
                eprintln!("Error reading from serial port: {}", e);
                continue;
            }
        }

        match unpack(&buffer[0..buffer_cursor]) {
            Ok(measurement) => {
                println!("Received Measurement ({}): {:?}", buffer_cursor, measurement);
                buffer_cursor = 0;
            },
            Err(pico_measure_transport::Error::InvalidInput(_)) => buffer_cursor = 0,
            _ => {}
        }
    }
}