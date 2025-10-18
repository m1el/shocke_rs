use std::sync::mpsc::Sender;
use core::ops::ControlFlow;
use hackrf_rs::{
    options::RxOptions,
    Hackrf,
};

/// Command received from or sent to the radio
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RadioCommand {
    pub device_id: u64,
    pub channel_id: u8,
    pub counter: u8,
    pub command: u8,
    pub intensity: u8,
}

/// CRC-16/XMODEM implementation for packet validation
fn crc16_xmodem(data: &[u8]) -> u16 {
    // https://crccalc.com/?crc=010000&method=CRC-16&datatype=hex&outtype=hex
    // initial value is determined by matching the existing CRC
    let mut crc = 0x3be7;
    let poly = 0x1021;
    let xor = 0; // 0x097f
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc = crc << 1
            }
        }
    }
    crc ^ xor
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
    counter: u8,
    phase: f64,
    current_sample: usize,
    buffer: Vec<u8>,
}

impl Modulator {
    pub fn new(center_freq: f64, sample_rate: f64, device_id: u64, channel_id: u8) -> Self {
        Self {
            command: Command::Idle, center_freq, sample_rate,
            freq_lo: 433_600_000.0, freq_hi: 433_664_000.0,
            device_id, channel_id,
            bit_rate: 4800.0,
            counter: 0b11000010,
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
        data_bits[9] = self.counter;
        data_bits[10..12].copy_from_slice(&command.command_bytes());
        let crc = crc16_xmodem(&data_bits[..12]);
        data_bits[12..14].copy_from_slice(&crc.to_be_bytes());
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
        for iq in iqs.iter_mut() {
            let current_bit = self.current_sample / samples_per_bit;
            let byte_idx = current_bit >> 3;
            let bit_idx = 7 - (current_bit & 7);
            if byte_idx >= self.buffer.len() {
                self.set_command(Command::Idle);
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
pub struct Demodulator {
    center_freq: f64,
    sample_rate: f64,
    freq_lo: f64,
    freq_hi: f64,
    bit_rate: f64,
    sender: Sender<RadioCommand>,
    // State for FSK demodulation
    phase_lo: f64,
    phase_hi: f64,
    // State for bit synchronization
    samples_per_bit: usize,
    bit_buffer: Vec<bool>,
    lo_accumulator: f64,
    hi_accumulator: f64,
    sample_count: usize,
    // State for packet detection
    packet_bits: Vec<u8>,
    in_packet: bool,
}

impl Demodulator {
    pub fn new(
        center_freq: f64, 
        sample_rate: f64, 
        sender: Sender<RadioCommand>
    ) -> Self {
        let bit_rate = 4800.0;
        let samples_per_bit = (sample_rate / bit_rate) as usize;
        
        Self {
            center_freq,
            sample_rate,
            freq_lo: 433_600_000.0,
            freq_hi: 433_664_000.0,
            bit_rate,
            sender,
            phase_lo: 0.0,
            phase_hi: 0.0,
            samples_per_bit,
            bit_buffer: Vec::new(),
            lo_accumulator: 0.0,
            hi_accumulator: 0.0,
            sample_count: 0,
            packet_bits: Vec::new(),
            in_packet: false,
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
            let lo_magnitude = (lo_i * lo_i + lo_q * lo_q).sqrt();
            
            self.phase_hi = (self.phase_hi + omega_hi).rem_euclid(TAU);
            let (sin_hi, cos_hi) = self.phase_hi.sin_cos();
            let hi_i = i * cos_hi + q * sin_hi;
            let hi_q = q * cos_hi - i * sin_hi;
            let hi_magnitude = (hi_i * hi_i + hi_q * hi_q).sqrt();
            
            // Accumulate energy for bit decision
            self.lo_accumulator += lo_magnitude;
            self.hi_accumulator += hi_magnitude;
            self.sample_count += 1;
            
            // Make bit decision at bit boundaries
            if self.sample_count >= self.samples_per_bit {
                let bit = self.hi_accumulator > self.lo_accumulator;
                self.bit_buffer.push(bit);
                
                // Reset accumulators
                self.lo_accumulator = 0.0;
                self.hi_accumulator = 0.0;
                self.sample_count = 0;
                
                // Try to detect and decode packets
                self.try_decode_packet();
            }
        }
    }

    /// Try to detect and decode a packet from the bit buffer
    fn try_decode_packet(&mut self) {
        // Look for preamble pattern (1064 bits of alternating 0xAA)
        const PREAMBLE_BITS: usize = 1064;
        const DATA_BITS: usize = 112;
        const PACKET_BITS: usize = PREAMBLE_BITS + DATA_BITS;
        
        if self.bit_buffer.len() < PACKET_BITS {
            return;
        }
        
        // Check if we have a valid preamble at the current position
        if !self.in_packet {
            // Look for preamble pattern - try both phases (starting with 0 or 1)
            let mut best_alternating_count = 0;
            
            for phase in [false, true] {
                let mut alternating_count = 0;
                for i in 0..PREAMBLE_BITS.min(self.bit_buffer.len()) {
                    let expected = ((i & 1) == 1) ^ phase;
                    if self.bit_buffer[i] == expected {
                        alternating_count += 1;
                    } else {
                        break;
                    }
                }
                if alternating_count > best_alternating_count {
                    best_alternating_count = alternating_count;
                }
            }
            
            // Require at least 90% match for preamble
            if best_alternating_count >= (PREAMBLE_BITS * 9 / 10) {
                self.in_packet = true;
            } else {
                // Remove one bit and try again
                if self.bit_buffer.len() > PACKET_BITS * 2 {
                    self.bit_buffer.drain(..1);
                }
                return;
            }
        }
        
        if self.in_packet && self.bit_buffer.len() >= PACKET_BITS {
            // Extract data bits
            let data_bits = &self.bit_buffer[PREAMBLE_BITS..PACKET_BITS];
            
            // Convert bits to bytes
            let mut data_bytes = [0u8; 14];
            for (byte_idx, byte) in data_bytes.iter_mut().enumerate() {
                for bit_idx in 0..8 {
                    if data_bits[byte_idx * 8 + bit_idx] {
                        *byte |= 1 << (7 - bit_idx);
                    }
                }
            }
            
            // Verify CRC
            let crc_received = u16::from_be_bytes([data_bytes[12], data_bytes[13]]);
            let crc_calculated = crc16_xmodem(&data_bytes[..12]);
            
            if crc_received == crc_calculated {
                // Valid packet! Extract fields
                let device_id = u64::from_be_bytes([
                    data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3],
                    data_bytes[4], data_bytes[5], data_bytes[6], data_bytes[7],
                ]);
                let channel_id = data_bytes[8];
                let counter = data_bytes[9];
                let command = data_bytes[10];
                let intensity = data_bytes[11];
                
                let radio_command = RadioCommand {
                    device_id,
                    channel_id,
                    counter,
                    command,
                    intensity,
                };
                
                // Send to channel (ignore errors if receiver is dropped)
                let _ = self.sender.send(radio_command);
            }
            
            // Clear processed bits
            self.bit_buffer.drain(..PACKET_BITS);
            self.in_packet = false;
        }
    }

    /// Start receiving and demodulating
    pub fn start_rx(mut self, mut hackrf: Hackrf, rx_options: RxOptions) -> Result<(), Box<dyn std::error::Error>> {
        hackrf.start_rx(rx_options, move |samples| {
            self.process_samples(samples);
            ControlFlow::Continue(())
        })?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    #[test]
    fn test_crc16_xmodem() {
        // Test with known data
        let data = [0x01, 0x00, 0x00];
        let crc = crc16_xmodem(&data);
        // This is a simple smoke test to ensure the function works
        assert!(crc != 0);
    }

    #[test]
    fn test_modulator_basic() {
        let mut modulator = Modulator::new(433_500_000.0, 10_000_000.0, 0x3a97b259957f1a27, 0xa5);
        assert_eq!(modulator.get_command(), &Command::Idle);
        
        modulator.set_command(Command::Vibrate(5));
        assert_eq!(modulator.get_command(), &Command::Vibrate(5));
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
        let sample_count = (10_000_000.0 / 4800.0 * 1180.0) as usize * 2;
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
