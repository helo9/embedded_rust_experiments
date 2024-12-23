#![no_std]
use core::ops::Deref;
use serde::{Serialize, Deserialize};
use postcard::{to_vec, from_bytes, Error};
use heapless::Vec;

#[derive(Deserialize, Serialize, Debug, PartialEq)]
pub struct Measurement {
    pub millis: u32,
    pub measurement: u16,
    pub sensor_id: u8,
}

pub fn unpack (buffer: &[u8]) -> Result<Measurement, Error>  {
    let measurement = from_bytes(buffer)?;

    Ok(measurement)
}

pub fn pack (measurement: &Measurement) -> Result<Vec<u8,8>, Error> {
    let buf: Vec<u8,8> = to_vec::<Measurement, 8>(measurement)?;

    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let measurement = Measurement {
            millis: 123u32,
            measurement: 895u16,
            sensor_id: 1u8
        };

        let buf = pack(&measurement).unwrap();
        let deserialized = unpack(buf.deref()).unwrap();

        assert_eq!(deserialized, measurement);
    }
}
