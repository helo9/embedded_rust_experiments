use chrono::{Datelike, Local, Timelike};
use csv::Writer;
use std::fmt::{self, Debug};
use std::fs::File;
use std::io::Read;
use std::thread::sleep;
use std::time::Duration;

use pico_measure_transport::{unpack, MeasuredValue, Measurement, MeasurementGroup};

struct PicoTransErr(pico_measure_transport::Error);

impl fmt::Display for PicoTransErr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            pico_measure_transport::Error::InvalidInput(_) => write!(f, "Invalid input"),
            pico_measure_transport::Error::TooFewBufferElements => {
                write!(f, "Too few buffer elements")
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Open the USB serial port (change as needed for your OS)
    let port_name = "/dev/ttyACM0"; // Replace with your serial port
    let baudrate = 115200;

    loop {
        let now = Local::now();

        let logpath = format!(
            "{}{:02}{:02}_{:02}-{:02}_pico_measure.csv",
            now.year(),
            now.month(),
            now.day(),
            now.hour(),
            now.minute()
        );

        match record_measurements(port_name, baudrate, &logpath) {
            Ok(v) => {}
            Err(e) => break (Err(e)),
        }
    }
}

fn record_measurements(
    port_name: &str,
    baudrate: u32,
    logpath: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let csv_file = File::create(logpath)?;

    let mut wtr = Writer::from_writer(csv_file);

    let mut port = serialport::new(port_name, baudrate)
        .timeout(Duration::from_millis(700))
        .open()?;

    let mut buffer = [0u8; 256];
    let mut buffer_cursor = 0usize;

    let mut serial_errors = 0u16;

    println!("Initialization complete");

    while serial_errors <= 50 {
        if let Ok(bytes_read) = port.read(&mut buffer[buffer_cursor..]) {
            buffer_cursor += bytes_read;

            if buffer_cursor >= buffer.len() {
                buffer_cursor = 0usize;
            }
        } else {
            eprintln!("Error reading from serial port");
            serial_errors += 1;
        }

        match unpack(&buffer[0..buffer_cursor]) {
            Ok(measurement) => {
                println!(
                    "Received Measurement ({}): {:?}",
                    buffer_cursor, measurement
                );

                write_csv_row(&mut wtr, &measurement)?;

                // reset receive buffer
                buffer_cursor = 0;
            }
            Err(pico_measure_transport::Error::TooFewBufferElements) => {
                // we still hope for valid data
            }
            Err(_) => {
                // give up
                buffer_cursor = 0;

                // increase error counter
                serial_errors += 1;
            }
        }
    }

    Ok(())
}

fn write_csv_row(
    writer: &mut csv::Writer<File>,
    mgroup: &MeasurementGroup,
) -> Result<(), Box<dyn std::error::Error>> {

    writer.write_field(mgroup.millis.to_string())?;

    let measurements: Vec<&Measurement> = mgroup
        .measurements
        .iter()
        .filter_map(|x| x.as_ref())
        .collect();

    for measurement in measurements {
        match measurement.value {
            MeasuredValue::Volt(v) => writer.write_field(v.to_string())?,
            MeasuredValue::Ampere(i) => writer.write_field(i.to_string())?,
            MeasuredValue::Counts(c) => writer.write_field(c.to_string())?,
            MeasuredValue::Celsius(t) => writer.write_field(t.to_string())?,
        }
    }

    writer.write_record(None::<&[u8]>)?;

    writer.flush()?;

    Ok(())
}
