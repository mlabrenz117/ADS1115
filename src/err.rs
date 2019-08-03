use core::convert::From;

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
