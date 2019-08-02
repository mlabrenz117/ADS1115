#![no_std]
use core::marker::PhantomData;

use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
    digital::v2::InputPin,
};

pub mod config;

use config::{ConfigRegister, R, W};

pub struct ADS1115<MODE, I2C, P> {
    i2c: I2C,
    alrt_rdy: P,
    addr: Address,
    _mode: PhantomData<MODE>,
}

pub struct SingleShot;
pub struct Continuous;

impl<MODE, I2C, P, E> ADS1115<MODE, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    P: InputPin,
{
    pub fn new(i2c: I2C, alrt_rdy: P, addr: Address) -> ADS1115<SingleShot, I2C, P> {
        ADS1115 {
            i2c: i2c,
            alrt_rdy,
            addr,
            _mode: PhantomData,
        }
    }

    pub fn free(mut self) -> Result<(I2C, P), E> {
        self.reset_config()?;
        Ok((self.i2c, self.alrt_rdy))
    }
}

impl<I2C, P, E> ADS1115<SingleShot, I2C, P> where I2C: WriteRead<Error = E> + Write<Error = E> {}

impl<I2C, P, E, MODE> ConfigRegister<E> for ADS1115<MODE, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    fn modify_config<F>(&mut self, f: F) -> Result<(), E>
    where
        for<'w> F: FnOnce(&R, &'w mut W) -> &'w mut W,
    {
        let r = self.read_config()?;
        let mut w = W::from(&r);
        f(&r, &mut w);
        self.i2c
            .write(self.addr as u8, &[Register::Config as u8, w.msb(), w.lsb()])?;
        Ok(())
    }

    fn read_config<'w>(&mut self) -> Result<R, E> {
        let mut buffer = [0u8; 2];
        self.i2c
            .write_read(self.addr as u8, &[Register::Config as u8], &mut buffer)?;
        Ok(R::new(((buffer[0] as u16) << 8) + buffer[1] as u16))
    }

    fn write_config<F>(&mut self, f: F) -> Result<(), E>
    where
        F: FnOnce(&mut W) -> &mut W,
    {
        let mut w = W::reset_value();
        f(&mut w);
        self.i2c
            .write(self.addr as u8, &[Register::Config as u8, w.msb(), w.lsb()])?;
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
