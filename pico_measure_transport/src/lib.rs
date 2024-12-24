#![no_std]
use core::ops::Deref;
use core::result::Result::{Ok, Err};
use serde::{Serialize, Deserialize};
use postcard::{to_slice, from_bytes};
use heapless::Vec;

pub const START_BYTE: u8 = 0xAA;

#[derive(Debug)]
pub enum Error {
    InvalidInput(Option<postcard::Error>),
    TooFewBufferElements,
}

impl From<postcard::Error> for Error {
    fn from(err: postcard::Error) -> Error {
        match err {
            postcard::Error::DeserializeUnexpectedEnd => Self::TooFewBufferElements,
            _ => Self::InvalidInput(Some(err)),
        }
    }
}

#[derive(Deserialize, Serialize, Debug, PartialEq)]
pub enum MeasurementUnit {
    Volt,
    Celsius,
    Ampere,
    Counts
}

#[derive(Deserialize, Serialize, Debug, PartialEq)]
pub struct Measurement {
    pub millis: u32,
    pub measurement: u16,
    pub unit: MeasurementUnit,
    pub sensor_id: u8,
}

pub fn unpack (buffer: &[u8]) -> Result<Measurement, Error>  {

    if buffer.len() < 2 {
        return Err(Error::TooFewBufferElements);
    }

    if buffer[0] != START_BYTE {
        return Err(Error::InvalidInput(None));
    }

    let data_len = buffer[1] as usize;

    if data_len > buffer.len() {
        return Err(Error::TooFewBufferElements);
    }

    let measurement = from_bytes(&buffer[2..data_len])?;

    Ok(measurement)
}

pub fn pack <const N: usize> (measurement: &Measurement) -> Result<Vec<u8,N>, Error> {
    let mut buffer: [u8; N] = [0; N];

    buffer[0] = START_BYTE;

    let data_bytes = {
        let used_buffer = to_slice(measurement, &mut buffer[2..])?;
        used_buffer.len() as u8 + 2u8
    };

    buffer[1] = data_bytes + 2u8;
    

    let mut vec = Vec::<u8, N>::new();
    vec.extend_from_slice(&buffer[..=buffer[1] as usize]).unwrap(); // TODO: add error handling

    Ok(vec)
}

#[cfg(test)]
mod tests {
    use core::{u16, u32, u8};

    use libc_print::std_name::{println, eprintln, dbg};

    use super::*;

    #[test]
    fn it_works() {
        let measurement = Measurement {
            millis: u32::MAX,
            measurement: u16::MAX,
            sensor_id: u8::MAX
        };

        let buf = pack::<32>(&measurement).unwrap();

        println!("{:#?} ({})", buf.as_slice(), buf.len());

        let deserialized = unpack(buf.deref()).unwrap();

        assert_eq!(deserialized, measurement);
    }
}
