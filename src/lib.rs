#![no_std]

use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
    digital::v2::InputPin,
};

pub mod config;

use config::{ConfigRegister, R, W};

pub struct ADS1115<I2C, P> {
    i2c: I2C,
    address: Address,
    alrt_rdy: P,
}

impl<I2C, P, E> ConfigRegister<E> for ADS1115<I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    P: InputPin,
{
    fn modify_config<F>(&mut self, f: F) -> Result<(), E>
    where
        for<'w> F: FnOnce(&R, &'w mut W) -> &'w mut W,
    {
        let r = self.read_config()?;
        let mut w = W::from(&r);
        f(&r, &mut w);
        self.i2c.write(
            self.address as u8,
            &[Register::Config as u8, w.msb(), w.lsb()],
        )?;
        Ok(())
    }

    fn read_config<'w>(&mut self) -> Result<R, E> {
        let mut buffer = [0u8; 2];
        self.i2c
            .write_read(self.address as u8, &[Register::Config as u8], &mut buffer)?;
        Ok(R::new(((buffer[0] as u16) << 8) + buffer[1] as u16))
    }

    fn write_config<F>(&mut self, f: F) -> Result<(), E>
    where
        F: FnOnce(&mut W) -> &mut W,
    {
        let mut w = W::reset_value();
        f(&mut w);
        self.i2c.write(
            self.address as u8,
            &[Register::Config as u8, w.msb(), w.lsb()],
        )?;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Address {
    Test = 0x1,
}

#[repr(u8)]
pub enum Register {
    Conversion = 0x0,
    Config = 0x1,
    LoThresh = 0x2,
    HiThresh = 0x3,
}
