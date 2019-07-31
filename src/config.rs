use core::convert::TryFrom;

pub trait ConfigRegister<E> {
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

    pub fn mux(&self) -> Mux {
        const MASK: u16 = 0x7000;
        Mux::try_from(self.bits() & MASK).unwrap_or_else(|_| unreachable!())
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

    pub fn mux(&mut self) -> _Mux {
        _Mux { w: self }
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

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Mux {
    Diff0_1,
    Diff0_3,
    Diff1_3,
    Diff2_3,
    Single1,
    Single2,
    Single3,
    Single4,
}

impl Mux {
    fn to_u16(&self) -> u16 {
        match self {
            Mux::Diff0_1 => 0x0000,
            Mux::Diff0_3 => 0x1000,
            Mux::Diff1_3 => 0x2000,
            Mux::Diff2_3 => 0x3000,
            Mux::Single1 => 0x4000,
            Mux::Single2 => 0x5000,
            Mux::Single3 => 0x6000,
            Mux::Single4 => 0x7000,
        }
    }
}

impl TryFrom<u16> for Mux {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, ()> {
        match value {
            0x0000 => Ok(Mux::Diff0_1),
            0x1000 => Ok(Mux::Diff0_3),
            0x2000 => Ok(Mux::Diff1_3),
            0x3000 => Ok(Mux::Diff2_3),
            0x4000 => Ok(Mux::Single1),
            0x5000 => Ok(Mux::Single2),
            0x6000 => Ok(Mux::Single3),
            0x7000 => Ok(Mux::Single4),
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
            Gain::TwoThirds => 0x0000,
            Gain::One => 0x0200,
            Gain::Two => 0x0400,
            Gain::Four => 0x0600,
            Gain::Eight => 0x0800,
            Gain::Sixteen => 0x0A00,
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
            ConversionMode::SingleShot => 0x0100,
            ConversionMode::Continuous => 0x0000,
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
            DataRate::SPS_8 => 0x0000,
            DataRate::SPS_16 => 0x0020,
            DataRate::SPS_32 => 0x0040,
            DataRate::SPS_64 => 0x0060,
            DataRate::SPS_128 => 0x0080,
            DataRate::SPS_250 => 0x00A0,
            DataRate::SPS_475 => 0x00C0,
            DataRate::SPS_860 => 0x00E0,
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
            ComparatorMode::Traditional => 0x0000,
            ComparatorMode::Window => 0x0010,
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
            CompPol::ActiveLow => 0x0000,
            CompPol::ActiveHigh => 0x0008,
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
            CompLat::NonLatching => 0x0000,
            CompLat::Latching => 0x0004,
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
pub enum CompQue {
    Disabled,
    AssertAfterOne,
    AssertAfterTwo,
    AssertAfterFour,
}

impl CompQue {
    fn to_u16(&self) -> u16 {
        match self {
            CompQue::AssertAfterOne => 0x0000,
            CompQue::AssertAfterTwo => 0x0001,
            CompQue::AssertAfterFour => 0x0002,
            CompQue::Disabled => 0x0004,
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
        const MASK: bool = true;
        const OFFSET: u8 = 15;
        self.w.bits &= !((MASK as u16) << OFFSET);
        self.w.bits |= ((value & MASK) as u16) << OFFSET;
        self.w
    }
}

pub struct _Mux<'a> {
    w: &'a mut W,
}

impl<'a> _Mux<'a> {
    pub fn set_multiplexer_config(self, config: Mux) -> &'a mut W {
        const MASK: u16 = 0x7000;
        self.w.bits = (!MASK & self.w.bits) + config.to_u16();
        self.w
    }
}

pub struct _Pga<'a> {
    w: &'a mut W,
}

impl<'a> _Pga<'a> {
    pub fn set_gain(self, gain: Gain) -> &'a mut W {
        const MASK: u16 = 0x0E00;
        self.w.bits = (!MASK & self.w.bits) + gain.to_u16();
        self.w
    }
}

pub struct _Mode<'a> {
    w: &'a mut W,
}

impl<'a> _Mode<'a> {
    pub fn set_mode(self, mode: ConversionMode) -> &'a mut W {
        const MASK: u16 = 0x0100;
        self.w.bits = (!MASK & self.w.bits) + mode.to_u16();
        self.w
    }
}

pub struct _Dr<'a> {
    w: &'a mut W,
}

impl<'a> _Dr<'a> {
    pub fn set_data_rate(self, data_rate: DataRate) -> &'a mut W {
        const MASK: u16 = 0x00E0;
        self.w.bits = (!MASK & self.w.bits) + data_rate.to_u16();
        self.w
    }
}

pub struct _CompMode<'a> {
    w: &'a mut W,
}

impl<'a> _CompMode<'a> {
    pub fn set_mode(self, comparator_mode: ComparatorMode) -> &'a mut W {
        const MASK: u16 = 0x0010;
        self.w.bits = (!MASK & self.w.bits) + comparator_mode.to_u16();
        self.w
    }
}

pub struct _CompPol<'a> {
    w: &'a mut W,
}

impl<'a> _CompPol<'a> {
    pub fn set_polarity(self, polarity: CompPol) -> &'a mut W {
        const MASK: u16 = 0x0008;
        self.w.bits = (!MASK & self.w.bits) + polarity.to_u16();
        self.w
    }
}

pub struct _CompLat<'a> {
    w: &'a mut W,
}

impl<'a> _CompLat<'a> {
    pub fn set_latching_mode(self, latching_mode: CompLat) -> &'a mut W {
        const MASK: u16 = 0x0008;
        self.w.bits = (!MASK & self.w.bits) + latching_mode.to_u16();
        self.w
    }
}

pub struct _CompQue<'a> {
    w: &'a mut W,
}

impl<'a> _CompQue<'a> {
    pub fn set_comparater_queue(self, mode: CompQue) -> &'a mut W {
        const MASK: u16 = 0x0003;
        self.w.bits = (!MASK & self.w.bits) + mode.to_u16();
        self.w
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
