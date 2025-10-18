use std::sync::mpsc;
use std::collections::HashMap;
use core::ops::ControlFlow;
use hackrf_rs::{
    options::RxOptions,
    DeviceList, Hackrf,
};

// Copy the necessary structs and functions from jugbow_modem
// since we can't directly use the module in examples without a lib.rs

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
    let mut crc = 0x3be7;
    let poly = 0x1021;
    let xor = 0;
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

/// Demodulator for receiving and decoding FSK-modulated signals
pub struct Demodulator {
    center_freq: f64,
    sample_rate: f64,
    freq_lo: f64,
    freq_hi: f64,
    bit_rate: f64,
    sender: mpsc::Sender<RadioCommand>,
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
    in_packet: bool,
}

impl Demodulator {
    pub fn new(
        center_freq: f64, 
        sample_rate: f64, 
        sender: mpsc::Sender<RadioCommand>
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
            let mut best_phase = false;
            
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
                    best_phase = phase;
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
}

type LazyResult<T> = Result<T, Box<dyn core::error::Error>>;

fn command_name(command: u8) -> &'static str {
    match command {
        0b01110001 => "SHOCK",
        0b01110010 => "VIBRATE",
        0b01110011 => "BEEP",
        _ => "UNKNOWN",
    }
}

fn main() -> LazyResult<()> {
    println!("Remote Capture Tool - listening for radio commands...");
    println!("Available devices:");
    for device in DeviceList::new()?.iter() {
        println!("{device:?}");
    }

    let mut hackrf = Hackrf::open_first()?;
    let options = RxOptions {
        center_freq: 433_500_000_u64.into(),
        sample_rate: 10_000_000_f64.into(),
        bandwidth: 3_500_000,
        enable_amp: true,
        enable_bias_tee: false,
        lna_gain: 32,
        vga_gain: 32,
    };

    let (tx, rx) = mpsc::channel();
    let mut demodulator = Demodulator::new(433_500_000.0, 10_000_000.0, tx);
    
    // Track unique device/channel combinations
    let mut seen_remotes: HashMap<(u64, u8), RadioCommand> = HashMap::new();

    // Spawn a thread to process received commands
    let print_thread = std::thread::spawn(move || {
        while let Ok(cmd) = rx.recv() {
            let key = (cmd.device_id, cmd.channel_id);
            
            // Print command details
            println!("\n=== Radio Command Received ===");
            println!("  Device ID:   0x{:016x}", cmd.device_id);
            println!("  Channel ID:  0x{:02x}", cmd.channel_id);
            println!("  Counter:     0x{:02x}", cmd.counter);
            println!("  Command:     {} (0x{:02x})", command_name(cmd.command), cmd.command);
            println!("  Intensity:   {}", cmd.intensity);
            
            // Track if this is a new remote
            if !seen_remotes.contains_key(&key) {
                println!("  >>> NEW REMOTE DETECTED! <<<");
                seen_remotes.insert(key, cmd.clone());
                
                println!("\n=== Configuration for shockers.toml ===");
                println!("[[shocker]]");
                println!("name = \"Remote {:016x}\"", cmd.device_id);
                println!("uuid = \"<generate-your-uuid>\"");
                println!("type = \"jugbow\"");
                println!("device_id = 0x{:016x}", cmd.device_id);
                println!("channel_id = 0x{:02x}", cmd.channel_id);
                println!();
            }
        }
    });

    // Start receiving
    let (done_tx, done_rx) = mpsc::sync_channel(1);
    hackrf.start_rx(options, move |samples| {
        demodulator.process_samples(samples);
        ControlFlow::Continue(())
    })?;

    // Ctrl-C handler
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl-C, stopping...");
        done_tx.send(()).unwrap();
    }).expect("Error setting Ctrl-C handler");

    done_rx.recv()?;
    println!("Stopping receiver...");
    hackrf.stop_rx()?;
    
    drop(print_thread);
    
    Ok(())
}
