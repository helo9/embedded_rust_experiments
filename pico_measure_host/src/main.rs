use std::time::Duration;
use std::io::{self, Read};

use pico_measure_transport::{Measurement, unpack};

fn main() -> io::Result<()> {
    // Open the USB serial port (change as needed for your OS)
    let port_name = "/dev/ttyACM0";  // Replace with your serial port
    let baudrate = 115200;

    let mut port = serialport::new(port_name, baudrate)
        .timeout(Duration::from_millis(100))
        .open()?;

    let mut buffer: [u8;8] = [0u8; 8];

    println!("Starting operation?");

    loop {

        // Read bytes from the serial port
        match port.read_exact(&mut buffer) {
            Ok(_) => {
                // Deserialize the data using Postcard
                match unpack(&buffer) {
                    Ok(measurement) => {
                        println!("Received Measurement: {:?}", measurement);
                    }
                    Err(e) => {
                        eprintln!("Failed to deserialize data: {}", e);
                    }
                }
            }
            Err(e) => {
                eprintln!("Error reading from serial port: {}", e);
            }
        }
    }
}