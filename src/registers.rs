#![allow(dead_code)]

use core::convert::TryFrom;

#[repr(u8)]
pub enum Register {
    Conversion = 0x0,
    Config = 0x1,
    LoThresh = 0x2,
    HiThresh = 0x3,
}
pub trait Registers<E> {
    fn modify_config<F>(&mut self, f: F) -> Result<(), E>
    where
        for<'w> F: FnOnce(&R, &'w mut W) -> &'w mut W;

    fn read_config<'w>(&mut self) -> Result<R, E>;

    fn write_config<F>(&mut self, f: F) -> Result<(), E>
    where
        F: FnOnce(&mut W) -> &mut W;

    fn reset_config(&mut self) -> Result<(), E> {
        self.write_config(|w| w)
    }

    fn write_lo_thresh(&mut self, value: i16) -> Result<(), E>;

    fn read_lo_thresh(&mut self) -> Result<i16, E>;

    fn write_hi_thresh(&mut self, value: i16) -> Result<(), E>;

    fn read_hi_thresh(&mut self) -> Result<i16, E>;

    fn read_conversion_register(&mut self) -> Result<i16, E>;
}

pub struct R {
    bits: u16,
}

pub struct W {
    bits: u16,
}

impl R {
    pub fn new(bits: u16) -> Self {
        R { bits }
    }

    pub fn bits(&self) -> u16 {
        self.bits
    }

    pub fn os(&self) -> Bit {
        Bit::from({
            const MASK: bool = true;
            const OFFSET: u8 = 15;
            ((self.bits >> OFFSET) & MASK as u16) != 0
        })
    }

    pub fn mux(&self) -> Channel {
        const MASK: u16 = 0x7000;
        Channel::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
    }

    pub fn pga(&self) -> Gain {
        const MASK: u16 = 0x0E00;
        Gain::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
    }

    pub fn mode(&self) -> ConversionMode {
        const MASK: u16 = 0x0100;
        ConversionMode::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
    }

    pub fn dr(&self) -> DataRate {
        const MASK: u16 = 0x00E0;
        DataRate::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
    }

    pub fn comp_mode(&self) -> ComparatorMode {
        const MASK: u16 = 0x0010;
        ComparatorMode::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
    }

    pub fn comp_pol(&self) -> CompPol {
        const MASK: u16 = 0x0008;
        CompPol::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
    }

    pub fn comp_lat(&self) -> CompLat {
        const MASK: u16 = 0x0004;
        CompLat::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
    }

    pub fn comp_que(&self) -> CompQue {
        const MASK: u16 = 0x0003;
        CompQue::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
    }
}

impl W {
    pub fn reset_value() -> W {
        W { bits: 0x8583 }
    }

    pub unsafe fn bits(&mut self, bits: u16) -> &mut Self {
        self.bits = bits;
        self
    }

    pub fn os(&mut self) -> _OS {
        _OS { w: self }
    }

    pub fn mux(&mut self) -> _Channel {
        _Channel { w: self }
    }

    pub fn pga(&mut self) -> _Pga {
        _Pga { w: self }
    }

    pub fn mode(&mut self) -> _Mode {
        _Mode { w: self }
    }

    pub fn dr(&mut self) -> _Dr {
        _Dr { w: self }
    }

    pub fn comp_mode(&mut self) -> _CompMode {
        _CompMode { w: self }
    }

    pub fn comp_pol(&mut self) -> _CompPol {
        _CompPol { w: self }
    }

    pub fn comp_lat(&mut self) -> _CompLat {
        _CompLat { w: self }
    }

    pub fn comp_que(&mut self) -> _CompQue {
        _CompQue { w: self }
    }

    pub fn msb(&self) -> u8 {
        (self.bits >> 8) as u8
    }

    pub fn lsb(&self) -> u8 {
        const MASK: u16 = 0x00ff;
        (self.bits & MASK) as u8
    }
}

impl From<&R> for W {
    fn from(r: &R) -> W {
        W { bits: r.bits }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Bit {
    Set,
    Clear,
}

impl From<bool> for Bit {
    fn from(b: bool) -> Bit {
        match b {
            true => Bit::Set,
            false => Bit::Clear,
        }
    }
}

impl Bit {
    fn to_u16(&self) -> u16 {
        match self {
            Bit::Set => 1,
            Bit::Clear => 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Channel {
    Diff0_1,
    Diff0_3,
    Diff1_3,
    Diff2_3,
    Single1,
    Single2,
    Single3,
    Single4,
}

impl Channel {
    fn to_u16(&self) -> u16 {
        match self {
            Channel::Diff0_1 => 0b000,
            Channel::Diff0_3 => 0b001,
            Channel::Diff1_3 => 0b010,
            Channel::Diff2_3 => 0b011,
            Channel::Single1 => 0b100,
            Channel::Single2 => 0b101,
            Channel::Single3 => 0b110,
            Channel::Single4 => 0b111,
        }
    }
}

impl TryFrom<u16> for Channel {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0000 => Ok(Channel::Diff0_1),
            0x1000 => Ok(Channel::Diff0_3),
            0x2000 => Ok(Channel::Diff1_3),
            0x3000 => Ok(Channel::Diff2_3),
            0x4000 => Ok(Channel::Single1),
            0x5000 => Ok(Channel::Single2),
            0x6000 => Ok(Channel::Single3),
            0x7000 => Ok(Channel::Single4),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Gain {
    TwoThirds,
    One,
    Two,
    Four,
    Eight,
    Sixteen,
}

impl Gain {
    fn to_u16(&self) -> u16 {
        match self {
            Gain::TwoThirds => 0b000,
            Gain::One => 0b001,
            Gain::Two => 0b010,
            Gain::Four => 0b011,
            Gain::Eight => 0b100,
            Gain::Sixteen => 0b101,
        }
    }
}

impl TryFrom<u16> for Gain {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0000 => Ok(Gain::TwoThirds),
            0x0200 => Ok(Gain::One),
            0x0400 => Ok(Gain::Two),
            0x0600 => Ok(Gain::Four),
            0x0800 => Ok(Gain::Eight),
            0x0A00 => Ok(Gain::Sixteen),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ConversionMode {
    SingleShot,
    Continuous,
}

impl ConversionMode {
    fn to_u16(&self) -> u16 {
        match self {
            ConversionMode::SingleShot => 0b1,
            ConversionMode::Continuous => 0b0,
        }
    }
}

impl TryFrom<u16> for ConversionMode {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0100 => Ok(ConversionMode::SingleShot),
            0x0000 => Ok(ConversionMode::Continuous),
            _ => Err(()),
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum DataRate {
    SPS_8,
    SPS_16,
    SPS_32,
    SPS_64,
    SPS_128,
    SPS_250,
    SPS_475,
    SPS_860,
}

impl DataRate {
    fn to_u16(&self) -> u16 {
        match self {
            DataRate::SPS_8 => 0b000,
            DataRate::SPS_16 => 0b001,
            DataRate::SPS_32 => 0b010,
            DataRate::SPS_64 => 0b011,
            DataRate::SPS_128 => 0b100,
            DataRate::SPS_250 => 0b101,
            DataRate::SPS_475 => 0b110,
            DataRate::SPS_860 => 0b111,
        }
    }
}

impl TryFrom<u16> for DataRate {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0000 => Ok(DataRate::SPS_8),
            0x0020 => Ok(DataRate::SPS_16),
            0x0040 => Ok(DataRate::SPS_32),
            0x0060 => Ok(DataRate::SPS_64),
            0x0080 => Ok(DataRate::SPS_128),
            0x00A0 => Ok(DataRate::SPS_250),
            0x00C0 => Ok(DataRate::SPS_475),
            0x00E0 => Ok(DataRate::SPS_860),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ComparatorMode {
    Traditional,
    Window,
}

impl ComparatorMode {
    fn to_u16(&self) -> u16 {
        match self {
            ComparatorMode::Traditional => 0b0,
            ComparatorMode::Window => 0b1,
        }
    }
}

impl TryFrom<u16> for ComparatorMode {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0000 => Ok(ComparatorMode::Traditional),
            0x0010 => Ok(ComparatorMode::Window),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CompPol {
    ActiveHigh,
    ActiveLow,
}

impl CompPol {
    fn to_u16(&self) -> u16 {
        match self {
            CompPol::ActiveLow => 0b0,
            CompPol::ActiveHigh => 0b1,
        }
    }
}

impl TryFrom<u16> for CompPol {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0000 => Ok(CompPol::ActiveLow),
            0x0008 => Ok(CompPol::ActiveHigh),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CompLat {
    NonLatching,
    Latching,
}

impl CompLat {
    fn to_u16(&self) -> u16 {
        match self {
            CompLat::NonLatching => 0b0,
            CompLat::Latching => 0b1,
        }
    }
}

impl TryFrom<u16> for CompLat {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0000 => Ok(CompLat::NonLatching),
            0x0004 => Ok(CompLat::Latching),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CompAssertions {
    AssertAfterOne,
    AssertAfterTwo,
    AssertAfterFour,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CompQue {
    Disabled,
    AssertAfterOne,
    AssertAfterTwo,
    AssertAfterFour,
}

impl CompQue {
    fn to_u16(&self) -> u16 {
        match self {
            CompQue::AssertAfterOne => 0b00,
            CompQue::AssertAfterTwo => 0b01,
            CompQue::AssertAfterFour => 0b10,
            CompQue::Disabled => 0b11,
        }
    }
}

impl TryFrom<u16> for CompQue {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0000 => Ok(CompQue::AssertAfterOne),
            0x0001 => Ok(CompQue::AssertAfterTwo),
            0x0002 => Ok(CompQue::AssertAfterFour),
            0x0003 => Ok(CompQue::Disabled),
            _ => Err(()),
        }
    }
}

macro_rules! set_w_value {
    ($self:ident, $value:ident; $mask:expr; $offset:expr) => {{
        const MASK: u16 = $mask;
        const OFFSET: u8 = $offset;
        $self.w.bits &= !(MASK << OFFSET);
        $self.w.bits |= ($value.to_u16() & MASK) << OFFSET;
        $self.w
    }};
}

pub struct _OS<'a> {
    w: &'a mut W,
}

impl<'a> _OS<'a> {
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }

    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }

    pub fn bit(self, value: bool) -> &'a mut W {
        let value = Bit::from(value);
        set_w_value!(self, value; 0b1; 15)
        //const MASK: bool = true;
        //const OFFSET: u8 = 15;
        //self.w.bits &= !((MASK as u16) << OFFSET);
        //self.w.bits |= ((value & MASK) as u16) << OFFSET;
        //self.w
    }
}

pub struct _Channel<'a> {
    w: &'a mut W,
}

impl<'a> _Channel<'a> {
    pub fn set_multiplexer_config(self, config: Channel) -> &'a mut W {
        set_w_value!(self, config; 0b111; 12)
    }
}

pub struct _Pga<'a> {
    w: &'a mut W,
}

impl<'a> _Pga<'a> {
    pub fn set_gain(self, gain: Gain) -> &'a mut W {
        set_w_value!(self, gain; 0b111; 9)
        //const MASK: u16 = 0x0E00;
        //self.w.bits = (!MASK & self.w.bits) + gain.to_u16();
        //self.w
    }
}

pub struct _Mode<'a> {
    w: &'a mut W,
}

impl<'a> _Mode<'a> {
    pub fn set_mode(self, mode: ConversionMode) -> &'a mut W {
        set_w_value!(self, mode; 0b1; 8)
        //const MASK: u16 = 0x0100;
        //self.w.bits = (!MASK & self.w.bits) + mode.to_u16();
        //self.w
    }
}

pub struct _Dr<'a> {
    w: &'a mut W,
}

impl<'a> _Dr<'a> {
    pub fn set_data_rate(self, data_rate: DataRate) -> &'a mut W {
        set_w_value!(self, data_rate; 0b111; 5)
        //const MASK: u16 = 0x00E0;
        //self.w.bits = (!MASK & self.w.bits) + data_rate.to_u16();
        //self.w
    }
}

pub struct _CompMode<'a> {
    w: &'a mut W,
}

impl<'a> _CompMode<'a> {
    pub fn set_mode(self, comparator_mode: ComparatorMode) -> &'a mut W {
        set_w_value!(self, comparator_mode; 0b1; 4)
        //const MASK: u16 = 0x0010;
        //self.w.bits = (!MASK & self.w.bits) + comparator_mode.to_u16();
        //self.w
    }
}

pub struct _CompPol<'a> {
    w: &'a mut W,
}

impl<'a> _CompPol<'a> {
    pub fn set_polarity(self, polarity: CompPol) -> &'a mut W {
        set_w_value!(self, polarity; 0b1; 3)
        //const MASK: u16 = 0x0008;
        //self.w.bits = (!MASK & self.w.bits) + polarity.to_u16();
        //self.w
    }
}

pub struct _CompLat<'a> {
    w: &'a mut W,
}

impl<'a> _CompLat<'a> {
    pub fn set_latching_mode(self, latching_mode: CompLat) -> &'a mut W {
        set_w_value!(self, latching_mode; 0b1; 2)
        //const MASK: u16 = 0x0008;
        //self.w.bits = (!MASK & self.w.bits) + latching_mode.to_u16();
        //self.w
    }
}

pub struct _CompQue<'a> {
    w: &'a mut W,
}

impl<'a> _CompQue<'a> {
    pub fn set_comparator_queue(self, mode: CompQue) -> &'a mut W {
        set_w_value!(self, mode; 0b11; 0)
        //const MASK: u16 = 0x0003;
        //self.w.bits = (!MASK & self.w.bits) + mode.to_u16();
        //self.w
    }
}

#[cfg(test)]
mod tests {
    use super::W;
    #[test]
    fn it_works() {
        let w = W::reset_value();
        assert_eq!(w.lsb(), 0x83);
        assert_eq!(w.msb(), 0x85);
    }
}
