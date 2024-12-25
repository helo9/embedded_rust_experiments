#![no_std]
use core::ops::Deref as _;
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
pub enum MeasuredValue {
    Volt(f32),
    Celsius(f32),
    Ampere(f32),
    Counts(u32)
}

#[derive(Deserialize, Serialize, Debug, PartialEq)]
pub struct Measurement {
    pub sensor_id: u8,
    pub value: MeasuredValue,
}

#[derive(Deserialize, Serialize, Debug, PartialEq)]
pub struct MeasurementGroup {
    pub millis: u32,
    pub measurements: [Option<Measurement>; 8],
}

pub fn unpack (buffer: &[u8]) -> Result<MeasurementGroup, Error>  {

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

pub fn pack <const N: usize> (measurement: &MeasurementGroup) -> Result<Vec<u8,N>, Error> {
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
    use core::{u32, u8};

    use libc_print::std_name::println;

    use super::*;

    #[test]
    fn it_works() {
        let measurement = MeasurementGroup {
            millis: u32::MAX,
            measurements: [
                Some(Measurement{
                    value: MeasuredValue::Volt(f32::MAX),
                    sensor_id: u8::MAX
                }),
                None, None, None, None, None, None, None
            ],
        };

        let buf = pack::<32>(&measurement).unwrap();

        println!("{:#?} ({})", buf.as_slice(), buf.len());

        let deserialized = unpack(buf.deref()).unwrap();

        assert_eq!(deserialized, measurement);
    }
}
