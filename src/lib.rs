#![no_std]
use core::convert::From;
use core::fmt::Debug;
use core::marker::PhantomData;

use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
    digital::v2::InputPin,
};

#[cfg(not(feature = "direct-access"))]
mod registers;

#[cfg(feature = "direct-access")]
pub mod registers;

pub use registers::{
    Bit, Channel, CompAssertions, CompLat, CompPol, ComparatorMode, ConversionMode, DataRate, Gain,
};

use registers::{CompQue, Register, Registers, R, W};

pub enum ADS1115Error<E> {
    AlertRdyPinUnconfigured,
    ThresholdError,
    I2CError(E),
}

impl<E> From<E> for ADS1115Error<E> {
    fn from(e: E) -> Self {
        ADS1115Error::I2CError(e)
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

pub struct ADS1115<Mode, PinMode, I2C, P> {
    i2c: I2C,
    alrt_rdy: Option<P>,
    addr_pin: AddressPin,
    _mode: PhantomData<Mode>,
    _p_mode: PhantomData<PinMode>,
}

pub type Address = u8;

pub struct SingleShot;
pub struct Continuous;
pub struct PinDisabled;
pub struct CompAlrtPin;
pub struct ConversReadyPin;

macro_rules! change_state {
    ($id:ident) => {
        ADS1115 {
            i2c: $id.i2c,
            alrt_rdy: $id.alrt_rdy,
            addr_pin: $id.addr_pin,
            _mode: PhantomData,
            _p_mode: PhantomData,
        }
    };
}

impl<Mode, PinMode, I2C, P, E> Registers<E> for ADS1115<Mode, PinMode, I2C, P>
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

    fn read_conversion_register(&mut self) -> Result<i16, E> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(
            self.addr_pin as Address,
            &[Register::Conversion as u8],
            &mut buffer,
        )?;
        Ok(((buffer[0] as u16) << 8) as i16 + buffer[1] as i16)
    }

    fn write_lo_thresh(&mut self, value: i16) -> Result<(), E> {
        let msb = ((value as u16 & 0xff00) >> 8) as u8;
        let lsb = (value & 0x00ff) as u8;
        self.i2c.write(
            self.addr_pin as Address,
            &[Register::LoThresh as u8, msb, lsb],
        )?;
        Ok(())
    }

    fn write_hi_thresh(&mut self, value: i16) -> Result<(), E> {
        let msb = ((value as u16 & 0xff00) >> 8) as u8;
        let lsb = (value & 0x00ff) as u8;
        self.i2c.write(
            self.addr_pin as Address,
            &[Register::HiThresh as u8, msb, lsb],
        )?;
        Ok(())
    }

    fn read_lo_thresh(&mut self) -> Result<i16, E> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(
            self.addr_pin as Address,
            &[Register::LoThresh as u8],
            &mut buffer,
        )?;
        Ok(((buffer[0] as u16) << 8) as i16 + buffer[1] as i16)
    }

    fn read_hi_thresh(&mut self) -> Result<i16, E> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(
            self.addr_pin as Address,
            &[Register::HiThresh as u8],
            &mut buffer,
        )?;
        Ok(((buffer[0] as u16) << 8) as i16 + buffer[1] as i16)
    }
}

pub fn new<I2C: WriteRead<Error = E> + Write<Error = E>, E, P: InputPin>(
    i2c: I2C,
    addr_pin: AddressPin,
) -> ADS1115<SingleShot, PinDisabled, I2C, P> {
    ADS1115 {
        i2c: i2c,
        alrt_rdy: None,
        addr_pin,
        _mode: PhantomData,
        _p_mode: PhantomData,
    }
}

/// P should be Input < PullUp >
impl<Mode, PinMode, I2C, P, E> ADS1115<Mode, PinMode, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    P: InputPin,
{
    /// Reset all ADS1115 devices on given i2c bus, issues command 0x06.
    pub unsafe fn reset_general_call(
        mut self,
    ) -> Result<ADS1115<SingleShot, PinDisabled, I2C, P>, ADS1115Error<E>> {
        self.i2c.write(0x00, &[0x06])?;
        Ok(change_state!(self))
    }

    /// Reset ADS1115 device at the address assosiated with this driver instance.
    pub fn reset(mut self) -> Result<ADS1115<SingleShot, PinDisabled, I2C, P>, ADS1115Error<E>> {
        self.reset_config()?;
        Ok(change_state!(self))
    }

    pub fn free(mut self) -> Result<(I2C, Option<P>), ADS1115Error<E>> {
        self.reset_config()?;
        Ok((self.i2c, self.alrt_rdy))
    }

    pub fn addr_pin(&self) -> (AddressPin, Address) {
        (self.addr_pin, self.addr_pin as Address)
    }

    pub fn last_conversion_result(&mut self) -> Result<i16, ADS1115Error<E>> {
        Ok(self.read_conversion_register()?)
    }

    /// Sets up pin to act as alert_rdy pin.
    ///
    /// Note, per the manufature's datasheet, the alert/rdy
    /// pin MUST be pulled high/low with a pull up/down resistor (1 kOhm - 10 kOhm)
    pub unsafe fn set_alert_rdy_pin(&mut self, pin: P) {
        self.alrt_rdy = Some(pin);
    }

    /// Configures ALERT/RDY pin as a conversion ready pin.
    /// It does this by setting the MSB of the high threshold register to '1' and the MSB
    /// of the low threshold register to '0'. COMP_POL and COMP_QUE bits will be set to '0'.
    ///
    /// When in this mode the ALERT/RDY pin outputs the OS bit in single-shot mode and pulses
    /// when in continuous conversion mode.
    pub fn enable_conversion_ready_pin(
        mut self,
    ) -> Result<ADS1115<Mode, ConversReadyPin, I2C, P>, ADS1115Error<E>> {
        if self.alrt_rdy.is_none() {
            return Err(ADS1115Error::AlertRdyPinUnconfigured);
        }
        self.write_hi_thresh(-0x8000)?;
        self.write_lo_thresh(0x7FFF)?;
        self.modify_config(|_, w| {
            w.comp_pol()
                .set_polarity(CompPol::ActiveLow)
                .comp_que()
                .set_comparator_queue(CompQue::AssertAfterOne)
        })?;
        Ok(change_state!(self))
    }

    /// Requires ALERT/RDY pin to be set up with the set_alert_rdy_pin() function.
    pub fn enable_comparator(
        mut self,
    ) -> Result<ADS1115<Mode, CompAlrtPin, I2C, P>, ADS1115Error<E>> {
        if self.alrt_rdy.is_none() {
            return Err(ADS1115Error::AlertRdyPinUnconfigured);
        }
        self.modify_config(|_, w| w.comp_que().set_comparator_queue(CompQue::AssertAfterTwo))?;
        Ok(change_state!(self))
    }

    pub fn get_gain(&mut self) -> Result<Gain, ADS1115Error<E>> {
        Ok(self.read_config()?.pga())
    }

    pub fn set_gain(&mut self, gain: Gain) -> Result<(), ADS1115Error<E>> {
        self.modify_config(|_, w| w.pga().set_gain(gain))?;
        Ok(())
    }

    pub fn get_conversion_mode(&mut self) -> Result<ConversionMode, ADS1115Error<E>> {
        Ok(self.read_config()?.mode())
    }

    pub fn get_multiplexer_config(&mut self) -> Result<Channel, ADS1115Error<E>> {
        Ok(self.read_config()?.mux())
    }

    pub fn is_conversion_ready(&mut self) -> Result<bool, ADS1115Error<E>> {
        Ok(self.read_config()?.os() == Bit::Set)
    }
}

impl<PinMode, I2C, P, E> ADS1115<SingleShot, PinMode, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    P: InputPin,
{
    pub fn read_channel(&mut self, channel: Channel) -> Result<i16, ADS1115Error<E>> {
        self.modify_config(|_, w| w.mux().set_multiplexer_config(channel).os().set_bit())?;
        while self.read_config()?.os() == Bit::Clear {}
        self.last_conversion_result()
    }

    pub fn trigger_channel_conversion(&mut self, channel: Channel) -> Result<(), ADS1115Error<E>> {
        self.modify_config(|_, w| w.mux().set_multiplexer_config(channel).os().set_bit())?;
        Ok(())
    }

    pub fn read(&mut self) -> Result<i16, ADS1115Error<E>> {
        self.modify_config(|_, w| w.os().set_bit())?;
        while self.read_config()?.os() == Bit::Clear {}
        self.last_conversion_result()
    }

    pub fn trigger_conversion(&mut self) -> Result<(), ADS1115Error<E>> {
        self.modify_config(|_, w| w.os().set_bit())?;
        Ok(())
    }

    pub fn into_continuous_mode(
        self,
    ) -> Result<ADS1115<Continuous, PinMode, I2C, P>, ADS1115Error<E>> {
        Ok(change_state!(self))
    }
}

impl<PinMode, I2C, P, E> ADS1115<Continuous, PinMode, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    P: InputPin,
{
    pub fn begin_conversion(&mut self, channel: Channel) -> Result<(), ADS1115Error<E>> {
        while self.read_config()?.os() == Bit::Set {}
        self.modify_config(|_, w| {
            w.mux()
                .set_multiplexer_config(channel)
                .mode()
                .set_mode(ConversionMode::Continuous)
        })?;
        Ok(())
    }

    pub fn stop_conversion(&mut self) -> Result<(), ADS1115Error<E>> {
        self.modify_config(|_, w| w.mode().set_mode(ConversionMode::SingleShot))?;
        Ok(())
    }

    pub fn get_data_rate(&mut self) -> Result<DataRate, ADS1115Error<E>> {
        Ok(self.read_config()?.dr())
    }

    pub fn set_data_rate(&mut self, rate: DataRate) -> Result<(), ADS1115Error<E>> {
        self.modify_config(|_, w| w.dr().set_data_rate(rate))?;
        Ok(())
    }

    pub fn into_single_shot_mode(
        mut self,
    ) -> Result<ADS1115<SingleShot, PinMode, I2C, P>, ADS1115Error<E>> {
        while self.read_config()?.os() == Bit::Set {}
        self.modify_config(|_, w| w.mode().set_mode(ConversionMode::SingleShot))?;
        Ok(change_state!(self))
    }
}

impl<Mode, I2C, P, E> ADS1115<Mode, CompAlrtPin, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    P: InputPin,
{
    pub fn set_lo_thresh(&mut self, value: i16) -> Result<(), ADS1115Error<E>> {
        let curr_hi = self.read_hi_thresh()?;
        if value >= curr_hi {
            return Err(ADS1115Error::ThresholdError);
        }
        self.write_lo_thresh(value)?;
        Ok(())
    }

    pub fn get_lo_thresh(&mut self) -> Result<i16, ADS1115Error<E>> {
        Ok(self.read_lo_thresh()?)
    }

    pub fn set_hi_thresh(&mut self, value: i16) -> Result<(), ADS1115Error<E>> {
        let curr_lo = self.read_lo_thresh()?;
        if value <= curr_lo {
            return Err(ADS1115Error::ThresholdError);
        }
        self.write_hi_thresh(value)?;
        Ok(())
    }

    pub fn get_hi_thresh(&mut self) -> Result<i16, ADS1115Error<E>> {
        Ok(self.read_hi_thresh()?)
    }

    pub fn get_latching_mode(&mut self) -> Result<CompLat, ADS1115Error<E>> {
        Ok(self.read_config()?.comp_lat())
    }

    pub fn set_latching_mode(&mut self, latch: CompLat) -> Result<(), ADS1115Error<E>> {
        self.modify_config(|_, w| w.comp_lat().set_latching_mode(latch))?;
        Ok(())
    }

    pub fn get_comparator_mode(&mut self) -> Result<ComparatorMode, ADS1115Error<E>> {
        Ok(self.read_config()?.comp_mode())
    }

    pub fn get_comparator_queue(&mut self) -> Result<CompQue, ADS1115Error<E>> {
        Ok(self.read_config()?.comp_que())
    }

    pub fn get_comparator_polarity(&mut self) -> Result<CompPol, ADS1115Error<E>> {
        Ok(self.read_config()?.comp_pol())
    }

    pub fn set_comparator_polarity(&mut self, polarity: CompPol) -> Result<(), ADS1115Error<E>> {
        self.modify_config(|_, w| w.comp_pol().set_polarity(polarity))?;
        Ok(())
    }

    pub fn set_comparator_mode(&mut self, mode: ComparatorMode) -> Result<(), ADS1115Error<E>> {
        self.modify_config(|_, w| w.comp_mode().set_mode(mode))?;
        Ok(())
    }

    pub fn disable_comparator(
        mut self,
    ) -> Result<ADS1115<Mode, PinDisabled, I2C, P>, ADS1115Error<E>> {
        self.modify_config(|_, w| w.comp_que().set_comparator_queue(CompQue::Disabled))?;
        Ok(change_state!(self))
    }
}

impl<Mode, I2C, P, E> ADS1115<Mode, ConversReadyPin, I2C, P>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    P: InputPin,
{
    pub fn disable_coversion_ready_pin(
        mut self,
    ) -> Result<ADS1115<Mode, PinDisabled, I2C, P>, ADS1115Error<E>> {
        self.write_hi_thresh(0x7FFF)?;
        self.write_lo_thresh(-0x8000)?;
        self.modify_config(|_, w| w.comp_que().set_comparator_queue(CompQue::Disabled))?;
        Ok(change_state!(self))
    }
}
