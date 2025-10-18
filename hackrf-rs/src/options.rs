use crate::RfPathFilter;

#[derive(Debug, Clone, Copy)]
pub enum CenterFreq {
    Auto { freq_hz: u64 },
    Explicit { if_freq_hz: u64, lo_freq_hz: u64, filter_path: RfPathFilter }
}

impl core::convert::From<u64> for CenterFreq {
    fn from(freq_hz: u64) -> Self { Self::Auto { freq_hz } }
}

#[derive(Debug, Clone, Copy)]
pub enum SampleRate {
    Auto { freq_hz: f64 },
    Explicit { freq_hz: u32, divider: u32 }
}

impl core::convert::From<f64> for SampleRate {
    fn from(freq_hz: f64) -> Self { Self::Auto { freq_hz } }
}

#[derive(Debug, Clone, Copy)]
pub struct RxOptions {
    pub center_freq: CenterFreq,
    pub sample_rate: SampleRate,
    pub bandwidth: u32,
    pub enable_amp: bool,
    pub enable_bias_tee: bool,
    pub lna_gain: u32,
    pub vga_gain: u32,
}

impl RxOptions {
    /// Convert these options to the corresponding `TxOptions`, with given `txvga_gain`
    pub fn to_tx(&self, txvga_gain: u32) -> TxOptions {
        let RxOptions {
            center_freq, sample_rate, bandwidth,
            enable_amp, enable_bias_tee, ..
        } = *self;
        TxOptions {
            center_freq, sample_rate, bandwidth, enable_amp,
            enable_bias_tee, txvga_gain,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TxOptions {
    pub center_freq: CenterFreq,
    pub sample_rate: SampleRate,
    pub bandwidth: u32,
    pub enable_amp: bool,
    pub enable_bias_tee: bool,
    pub txvga_gain: u32,
}

impl TxOptions {
    /// Convert these options to the corresponding `RxOptions`, with given lna/vga gain
    pub fn to_rx(&self, lna_gain: u32, vga_gain: u32) -> RxOptions {
        let TxOptions {
            center_freq, sample_rate, bandwidth,
            enable_amp, enable_bias_tee, ..
        } = *self;
        RxOptions {
            center_freq, sample_rate, bandwidth, enable_amp,
            enable_bias_tee, lna_gain, vga_gain,
        }
    }
}


// pub struct RxOptionsBuilder {
//     pub center_freq: Option<CenterFreq>,
//     pub sampling_rate: Option<SampleRate>,
//     pub bandwidth: Option<u32>,
//     pub enable_amp: bool,
//     pub lna_gain_db: u32,
//     pub vga_gain_db: u32,
//     pub enable_bias_tee: bool,
// }
