#![no_std]
use core::marker::PhantomData;

use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
    digital::v2::InputPin,
};

mod config;

pub use config::{
    Bit, Channel, CompLat, CompPol, CompQue, ComparatorMode, ConversionMode, DataRate, Gain,
};

use config::{ConfigRegister, R, W};

pub struct ADS1115<MODE, I2C, P> {
    i2c: I2C,
    alrt_rdy: P,
    addr_pin: AddressPin,
    _mode: PhantomData<MODE>,
}

pub type Address = u8;
pub struct SingleShot;
pub struct Continuous;

/// P should be Input < PullUp >
impl<MODE, I2C, P, E> ADS1115<MODE, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    pub fn new(i2c: I2C, alrt_rdy: P, addr_pin: AddressPin) -> ADS1115<SingleShot, I2C, P> {
        ADS1115 {
            i2c: i2c,
            alrt_rdy,
            addr_pin,
            _mode: PhantomData,
        }
    }

    /// Reset all ADS1115 devices on given i2c bus, issues command 0x06.
    pub unsafe fn reset_general_call(mut self) -> Result<ADS1115<SingleShot, I2C, P>, E> {
        self.i2c.write(self.addr_pin as Address, &[0x00, 0x06])?;
        Ok(ADS1115 {
            i2c: self.i2c,
            alrt_rdy: self.alrt_rdy,
            addr_pin: self.addr_pin,
            _mode: PhantomData,
        })
    }

    /// Reset ADS1115 device at the addr_piness assosiated with this driver instance.
    pub fn reset(mut self) -> Result<ADS1115<SingleShot, I2C, P>, E> {
        self.reset_config()?;
        Ok(ADS1115 {
            i2c: self.i2c,
            alrt_rdy: self.alrt_rdy,
            addr_pin: self.addr_pin,
            _mode: PhantomData,
        })
    }

    pub fn free(mut self) -> Result<(I2C, P), E> {
        self.reset_config()?;
        Ok((self.i2c, self.alrt_rdy))
    }

    pub fn addr_pin(&self) -> (AddressPin, Address) {
        (self.addr_pin, self.addr_pin as Address)
    }

    pub fn last_conversion_result(&mut self) -> Result<i16, E> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(
            self.addr_pin as Address,
            &[Register::Conversion as u8],
            &mut buffer,
        )?;
        Ok((buffer[0] as i16) << 8 + buffer[1] as i16)
    }
}

impl<MODE, I2C, P, E> ADS1115<MODE, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    P: InputPin,
{
    /// Configures ALERT/RDY pin as a conversion ready pin.
    /// It does this by setting the MSB of the high threshold register to '1' and the MSB
    /// of the low threshold register to '0'. COMP_POL and COMP_QUE bits will be set to '0'.
    ///
    /// Note: ALERT/RDY pin requires a pull up resistor.
    pub fn set_conversion_ready_pin_mode() -> Result<(), E> {
        unimplemented!()
    }
}

impl<I2C, P, E> ADS1115<SingleShot, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    pub fn read(&mut self, channel: Channel) -> Result<i16, E> {
        self.modify_config(|_, w| w.mux().set_multiplexer_config(channel).os().set_bit())?;
        while self.read_config()?.os() == Bit::Clear {}
        self.last_conversion_result()
    }

    pub fn into_continuous_mode(mut self) -> Result<ADS1115<Continuous, I2C, P>, E> {
        while self.read_config()?.os() == Bit::Set {}
        self.modify_config(|_, w| w.mode().set_mode(ConversionMode::Continuous))?;
        Ok(ADS1115 {
            i2c: self.i2c,
            alrt_rdy: self.alrt_rdy,
            addr_pin: self.addr_pin,
            _mode: PhantomData,
        })
    }
}

impl<I2C, P, E> ADS1115<Continuous, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    pub fn begin_conversion(&mut self, channel: Channel) -> Result<(), E> {
        Ok(())
    }

    pub fn into_single_shot_mode(mut self) -> Result<ADS1115<SingleShot, I2C, P>, E> {
        while self.read_config()?.os() == Bit::Set {}
        self.modify_config(|_, w| w.mode().set_mode(ConversionMode::SingleShot))?;
        Ok(ADS1115 {
            i2c: self.i2c,
            alrt_rdy: self.alrt_rdy,
            addr_pin: self.addr_pin,
            _mode: PhantomData,
        })
    }
}

impl<MODE, I2C, P, E> ConfigRegister<E> for ADS1115<MODE, I2C, P>
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
        self.i2c.write(
            self.addr_pin as Address,
            &[Register::Config as u8, w.msb(), w.lsb()],
        )?;
        Ok(())
    }

    fn read_config<'w>(&mut self) -> Result<R, E> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(
            self.addr_pin as Address,
            &[Register::Config as u8],
            &mut buffer,
        )?;
        Ok(R::new(((buffer[0] as u16) << 8) + buffer[1] as u16))
    }

    fn write_config<F>(&mut self, f: F) -> Result<(), E>
    where
        F: FnOnce(&mut W) -> &mut W,
    {
        let mut w = W::reset_value();
        f(&mut w);
        self.i2c.write(
            self.addr_pin as Address,
            &[Register::Config as u8, w.msb(), w.lsb()],
        )?;
        Ok(())
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum AddressPin {
    Ground = 0b1001000,
    VDD = 0b1001001,
    SDA = 0b1001010,
    SCL = 0b1001011,
}

#[repr(u8)]
pub(crate) enum Register {
    Conversion = 0x0,
    Config = 0x1,
    LoThresh = 0x2,
    HiThresh = 0x3,
}
