use std::collections::VecDeque;
use core::ops::ControlFlow;
use hackrf_rs::{
    options::RxOptions,
    Hackrf,
};
use num_complex::Complex64;

const BIT_RATE: f64 = 4800.0;
const FREQ_LO: f64 = 433_600_000.0;
const FREQ_HI: f64 = 433_664_000.0;

/// Command received from or sent to the radio
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RadioCommand {
    pub device_id: u64,
    pub channel_id: u8,
    pub counter: u8,
    pub command: u8,
    pub intensity: u8,
}

struct Crc16XModem {
    state: u16,
    bit: u8,
}

impl Crc16XModem {
    const POLY: u16 = 0x1021;
    const CHECK: u16 = 0x35e0; // 0x09ba;
    const MAKE: u16 = 0x097f;
    pub fn new() -> Self {
        Self { state: 0, bit: 0 }
    }
    pub fn reset(&mut self) {
        self.state = 0;
        self.bit = 0;
    }
    pub fn write(&mut self, data: &[u8]) {
        assert!(self.bit == 0, "trying to write a byte in the middle fo the bit");
        // https://crccalc.com/?crc=010000&method=CRC-16&datatype=hex&outtype=hex
        // initial value is determined by matching the existing CRC
        let mut crc = self.state;
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ Self::POLY;
                } else {
                    crc = crc << 1
                }
            }
        }
        self.state = crc;
    }
    // pub fn feed(&mut self, bit: bool) {
    //     let mut crc = self.state;
    //     crc ^= (bit as u16) << 15;
    //     if crc & 0x8000 != 0 {
    //         crc = (crc << 1) ^ Self::POLY;
    //     } else {
    //         crc = crc << 1
    //     }
    //     self.state = crc;
    //     self.bit = (self.bit + 1) & 7;
    // }
    pub fn make_crc(&self) -> u16 {
        self.state ^ Self::MAKE
    }
    pub fn check_crc(&self) -> bool {
        self.state ^ Self::CHECK == 0
    }
}

fn crc16_xmodem(data: &[u8]) -> u16 {
    let mut crc = Crc16XModem::new();
    crc.write(data);
    crc.make_crc()
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Command {
    Idle,
    Quit,
    Vibrate(u8),
    Shock(u8),
    Beep(u8),
}

impl Command {
    const SHOCK  : u8 = 0b01110001;
    const VIBRATE: u8 = 0b01110010;
    const BEEP   : u8 = 0b01110011;
    pub fn command_bytes(&self) -> [u8; 2] {
        match self {
            Command::Idle | Command::Quit => [0, 0],
            Command::Vibrate(level) => [Self::VIBRATE, *level],
            Command::Shock(level) => [Self::SHOCK, *level],
            Command::Beep(level) => [Self::BEEP, *level],
        }
    }
}

/// Modulator for generating FSK-modulated signals
pub struct Modulator {
    command: Command,
    center_freq: f64,
    sample_rate: f64,
    freq_lo: f64,
    freq_hi: f64,
    bit_rate: f64,
    device_id: u64,
    channel_id: u8,
    sequence_number: u8,
    phase: f64,
    current_sample: usize,
    buffer: Vec<u8>,
}

impl Modulator {
    pub fn new(center_freq: f64, sample_rate: f64, device_id: u64, channel_id: u8) -> Self {
        Self {
            command: Command::Idle, center_freq, sample_rate,
            freq_lo: FREQ_LO, freq_hi: FREQ_HI,
            device_id, channel_id,
            bit_rate: 4800.0,
            sequence_number: 0,
            phase: 0.0, current_sample: 0,
            buffer: vec![0; 148],
        }
    }
    
    pub fn set_command(&mut self, command: Command) {
        self.command = command.clone();
        self.buffer.fill(0);
        self.current_sample = 0;
        self.phase = 0.0;
        if matches!(command, Command::Idle | Command::Quit) {
            return;
        }
        const PREAMBLE_BYTES: usize = 133;
        self.buffer[..PREAMBLE_BYTES].fill(0xaa);
        let data_bits = &mut self.buffer[PREAMBLE_BYTES..][..14];
        data_bits[..8].copy_from_slice(&self.device_id.to_be_bytes());
        data_bits[8] = self.channel_id;
        data_bits[9] = self.sequence_number;
        data_bits[10..12].copy_from_slice(&command.command_bytes());
        let crc = crc16_xmodem(&data_bits[..12]);
        data_bits[12..14].copy_from_slice(&crc.to_be_bytes());
        self.sequence_number = self.sequence_number.wrapping_add(1);
    }
    
    pub fn fill(&mut self, buffer: &mut[u8]) -> usize {
        if matches!(self.command, Command::Idle | Command::Quit) {
            buffer.fill(0);
            return buffer.len();
        }
        use core::f64::consts::TAU;
        let samples_per_bit = (self.sample_rate / self.bit_rate) as usize;

        let omega_lo = TAU * (self.freq_lo - self.center_freq) / self.sample_rate;
        let omega_hi = TAU * (self.freq_hi - self.center_freq) / self.sample_rate;
        let (iqs, _whatever) = buffer.as_chunks_mut::<2>();
        for (ii, iq) in iqs.iter_mut().enumerate() {
            let current_bit = self.current_sample / samples_per_bit;
            let byte_idx = current_bit >> 3;
            let bit_idx = 7 - (current_bit & 7);
            if byte_idx >= self.buffer.len() {
                self.set_command(Command::Idle);
                self.fill(&mut buffer[ii*2..]);
                break;
            }
            let bit = (self.buffer[byte_idx] >> bit_idx) & 1;
            let omega = if bit == 0 { omega_lo } else { omega_hi };
            self.phase = (self.phase + omega).rem_euclid(TAU);
            let (sin, cos) = self.phase.sin_cos();
            iq[0] = ((cos + 1.0) * 127.5) as u8;
            iq[1] = ((sin + 1.0) * 127.5) as u8;
            self.current_sample += 1;
        }

        buffer.len()
    }

    pub fn get_command(&self) -> &Command {
        &self.command
    }
}

/// Demodulator for receiving and decoding FSK-modulated signals
pub struct Demodulator<F> {
    callback: F,
    center_freq: f64,
    sample_rate: f64,
    freq_lo: f64,
    freq_hi: f64,
    // bit_rate: f64,
    // State for FSK demodulation
    phase_lo: f64,
    phase_hi: f64,
    // State for bit synchronization
    samples_per_bit: usize,
    bit_buffer: VecDeque<u8>,
    lo_accumulator: Complex64,
    hi_accumulator: Complex64,
    // State for packet detection
    // packet_bits: Vec<u8>,
    // preamble: usize,
    crc: Crc16XModem,
    current_run: (Option<u8>, usize),
}

impl<F> Demodulator<F>
    where F: FnMut(RadioCommand) + Send + 'static
{
    const LPF: f64 = 1.0 / 100.0;
    const THRESHOLD: f64 = 0.5;
    pub fn new(
        center_freq: f64, 
        sample_rate: f64, 
        callback: F,
    ) -> Self {
        let bit_rate = BIT_RATE;
        let samples_per_bit = (sample_rate / bit_rate) as usize;
        
        Self {
            callback,
            center_freq,
            sample_rate,
            freq_lo: FREQ_LO,
            freq_hi: FREQ_HI,
            // bit_rate,
            phase_lo: 0.0,
            phase_hi: 0.0,
            samples_per_bit,
            bit_buffer: VecDeque::new(),
            lo_accumulator: Complex64::ZERO,
            hi_accumulator: Complex64::ZERO,
            // packet_bits: Vec::new(),
            // preamble: 0,
            crc: Crc16XModem::new(),
            current_run: (None, 0),
        }
    }

    fn feed_bit(&mut self, current_bit: Option<u8>) {
        let (prev_bit, run) = self.current_run;
        if prev_bit != current_bit {
            let length = (run + self.samples_per_bit / 2) / self.samples_per_bit;
            // eprintln!("Bit change: {:?} for {}", prev_bit, length);
            if length != 0 {
                match prev_bit {
                    Some(bit) => {
                        for _ii in 0..length {
                            self.bit_buffer.push_back(bit);
                        }
                    },
                    None => {
                        self.try_decode_packet();
                        self.bit_buffer.clear();
                        self.crc.reset();
                    },
                }
            }
            self.current_run = (current_bit, 0);
        } else {
            self.current_run = (prev_bit, run + 1);
        }
    }
    /// Process incoming IQ samples and extract radio commands
    pub fn process_samples(&mut self, samples: &[u8]) {
        use core::f64::consts::TAU;
        
        let omega_lo = TAU * (self.freq_lo - self.center_freq) / self.sample_rate;
        let omega_hi = TAU * (self.freq_hi - self.center_freq) / self.sample_rate;
        
        // Process IQ samples in pairs
        let samples = samples.chunks_exact(2);
        
        for iq in samples {
            // Convert from u8 to normalized complex number
            let i = (iq[0] as f64 / 127.5) - 1.0;
            let q = (iq[1] as f64 / 127.5) - 1.0;
            
            // Demodulate by mixing with reference frequencies
            self.phase_lo = (self.phase_lo + omega_lo).rem_euclid(TAU);
            let (sin_lo, cos_lo) = self.phase_lo.sin_cos();
            let lo_i = i * cos_lo + q * sin_lo;
            let lo_q = q * cos_lo - i * sin_lo;
            let lo = Complex64::new(lo_i, lo_q);

            self.phase_hi = (self.phase_hi + omega_hi).rem_euclid(TAU);
            let (sin_hi, cos_hi) = self.phase_hi.sin_cos();
            let hi_i = i * cos_hi + q * sin_hi;
            let hi_q = q * cos_hi - i * sin_hi;
            let hi = Complex64::new(hi_i, hi_q);
            
            // low-pass filter the magnitudes
            self.lo_accumulator = (1.0 - Self::LPF) * self.lo_accumulator + Self::LPF * lo;
            self.hi_accumulator = (1.0 - Self::LPF) * self.hi_accumulator + Self::LPF * hi;
            let lo_magnitude = self.lo_accumulator.norm() > Self::THRESHOLD;
            let hi_magnitude = self.hi_accumulator.norm() > Self::THRESHOLD;
            let current_bit =
                if      lo_magnitude && !hi_magnitude { Some(0) }
                else if hi_magnitude && !lo_magnitude { Some(1) }
                else    { None };
            self.feed_bit(current_bit);
        }
        self.try_decode_packet();
    }

    /// Try to detect and decode a packet from the bit buffer
    fn try_decode_packet(&mut self) {
        const PACKET_BYTES: usize = 14;
        if self.bit_buffer.len() < PACKET_BYTES * 8 {
            return;
        }
        for window in self.bit_buffer.as_slices().0.windows(PACKET_BYTES * 8) {
            let mut data_bytes = [0u8; PACKET_BYTES];
            for (i, chunk) in window.chunks(8).enumerate() {
                let mut byte = 0u8;
                for (j, &bit) in chunk.iter().enumerate() {
                    byte |= (bit as u8) << (7 - j);
                }
                data_bytes[i] = byte;
            }
            let mut crc = Crc16XModem::new();
            crc.write(&data_bytes);
            if !crc.check_crc() {
                continue;
            }
            let radio_command = RadioCommand {
                device_id: u64::from_be_bytes(data_bytes[0..8].try_into().unwrap()),
                channel_id: data_bytes[8],
                counter: data_bytes[9],
                command: data_bytes[10],
                intensity: data_bytes[11],
            };
            (self.callback)(radio_command);
        }
        let to_drain = self.bit_buffer.len() - 112;
        self.bit_buffer.drain(0..to_drain);
    }

    /// Start receiving and demodulating
    pub fn start_rx(mut self, mut hackrf: Hackrf, rx_options: RxOptions) -> Result<(), Box<dyn std::error::Error>> {
        let (done_tx, done_rx) = std::sync::mpsc::channel();
        let (terminate_tx, terminate_rx) = std::sync::mpsc::channel();
        ctrlc::set_handler(move || {
            let _ = terminate_tx.send(());
            let _ = done_tx.send(());
        })?;
        hackrf.start_rx(rx_options, move |samples| {
            if terminate_rx.try_recv().is_ok() {
                return ControlFlow::Break(());
            }
            self.process_samples(samples);
            ControlFlow::Continue(())
        })?;
        done_rx.recv()?;
        hackrf.stop_rx()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    // captured from a real JugBow device
    const TEST_PACKET: [u8; 14] = [
        0b00111010, 0b10010111,
        0b10110010, 0b01011001,
        0b10010101, 0b01111111,
        0b00011010, 0b00100111,
        0b10100101, 0b11000010,
        0b01110010, 0b00000001,
        0b11110100, 0b01100010,
    ];

    // test CRC16-XMODEM calculation
    #[test]
    fn test_crc16_xmodem_calc() {
        let mut crc = Crc16XModem::new();
        crc.write(&TEST_PACKET[..12]);
        let expected_crc = u16::from_be_bytes(TEST_PACKET[12..14].try_into().unwrap());
        assert_eq!(crc.make_crc(), expected_crc);
    }

    // test CRC16-XMODEM checking
    #[test]
    fn test_crc16_xmodem_check() {
        let mut crc = Crc16XModem::new();
        crc.write(&TEST_PACKET);
        assert!(crc.check_crc());
    }

    #[test]
    fn test_modulator_basic() {
        let mut modulator = Modulator::new(433_500_000.0, 10_000_000.0, 0x3a97b259957f1a27, 0xa5);
        assert_eq!(modulator.get_command(), &Command::Idle);
        
        modulator.set_command(Command::Vibrate(5));
        assert_eq!(modulator.get_command(), &Command::Vibrate(5));
    }

    #[test]
    fn test_modulator_encode() {
        let mut modulator = Modulator::new(433_500_000.0, 10_000_000.0, 0x3a97b259957f1a27, 0xa5);
        modulator.sequence_number = 0b11000010;
        modulator.set_command(Command::Vibrate(1));
        const PREAMBLE_BYTES: usize = 133;
        const PACKET_BYTES: usize = 14;
        assert_eq!(
            &modulator.buffer[PREAMBLE_BYTES..][..PACKET_BYTES],
            &TEST_PACKET
        );
    }

    #[test]
    fn test_round_trip_encode_decode() {
        // Create a modulator with known parameters
        let device_id = 0x3a97b259957f1a27;
        let channel_id = 0xa5;
        let mut modulator = Modulator::new(433_500_000.0, 10_000_000.0, device_id, channel_id);
        
        // Set a command
        modulator.set_command(Command::Vibrate(5));
        
        // Generate samples
        let sample_count = (10_000_000.0 / 4800.0 * (1180.0 + 8.0)) as usize * 2;
        let mut samples = vec![0u8; sample_count];
        modulator.fill(&mut samples);
        
        // Create a demodulator and channel
        let (tx, rx) = mpsc::channel();
        let mut demodulator = Demodulator::new(433_500_000.0, 10_000_000.0, tx);
        
        // Process the generated samples
        demodulator.process_samples(&samples);

        // Check if we received the command
        if let Ok(radio_command) = rx.try_recv() {
            assert_eq!(radio_command.device_id, device_id);
            assert_eq!(radio_command.channel_id, channel_id);
            assert_eq!(radio_command.command, Command::VIBRATE);
            assert_eq!(radio_command.intensity, 5);
        } else {
            panic!("Expected to receive a radio command");
        }
    }

    #[test]
    fn test_radio_command_creation() {
        let cmd = RadioCommand {
            device_id: 0x1234567890abcdef,
            channel_id: 0x42,
            counter: 0x10,
            command: 0x71,
            intensity: 0x05,
        };
        
        assert_eq!(cmd.device_id, 0x1234567890abcdef);
        assert_eq!(cmd.channel_id, 0x42);
        assert_eq!(cmd.counter, 0x10);
        assert_eq!(cmd.command, 0x71);
        assert_eq!(cmd.intensity, 0x05);
    }
}
