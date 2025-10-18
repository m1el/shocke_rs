use std::sync::mpsc::SendError;
use std::io::BufRead;
use core::ops::ControlFlow;
use hackrf_rs::{
    options::TxOptions,
    DeviceList, Hackrf,
};

type LazyResult<T> = Result<T, Box<dyn core::error::Error>>;

#[derive(Clone, Debug, PartialEq, Eq)]
enum Command {
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
    fn command_bytes(&self) -> [u8; 2] {
        match self {
            Command::Idle | Command::Quit => [0, 0],
            Command::Vibrate(level) => [Self::VIBRATE, *level],
            Command::Shock(level) => [Self::SHOCK, *level],
            Command::Beep(level) => [Self::BEEP, *level],
        }
    }
}

struct State {
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

impl State {
    fn new(center_freq: f64, sample_rate: f64) -> Self {
        Self {
            command: Command::Idle, center_freq, sample_rate,
            freq_lo: 433_600_000.0, freq_hi: 433_664_000.0,
            device_id: 0x3a97b259957f1a27, channel_id: 0xa5,
            bit_rate: 4800.0,
            counter: 0b11000010,
            phase: 0.0, current_sample: 0,
            buffer: vec![0; 148],
        }
    }
    fn set_command(&mut self, command: Command) {
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
        // eprintln!("{:?}", command.command_bytes());
        data_bits[10..12].copy_from_slice(&command.command_bytes());
        let crc = crc16_xmodem(&data_bits[..12]);
        data_bits[12..14].copy_from_slice(&crc.to_be_bytes());
        // for byte in data_bits {            
        //     eprintln!("{:08b}", byte);
        // }
    }
    fn fill(&mut self, buffer: &mut[u8]) -> usize {
        if matches!(self.command, Command::Idle | Command::Quit) {
            buffer.fill(0);
            return buffer.len();
        }
        // let start = std::time::Instant::now();
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
            // if self.current_sample > current_bit * samples_per_bit {
            //     current_bit += 1;
            // }
        }

        // eprintln!("elapsed: {:?}", start.elapsed());
        buffer.len()
    }
}

fn main() -> LazyResult<()> {
    // let mut current_state = State::new(433_500_000.0, 10_000_000.0);
    // current_state.set_command(Command::Vibrate(2));
    // let samples = (10_000_000_f64 / 4790_f64 * 1180_f64) as usize * 2;
    // let mut buffer = vec![0; samples];
    // eprintln!("fill rv: {}", current_state.fill(&mut buffer));
    // std::fs::write("generated.bin", buffer)?;
    // if true { return Ok(()); }

    println!("Available devices:");
    for device in DeviceList::new()?.iter() {
        println!("{device:?}");
    }

    const RESET: bool = false;
    const RESET_MILLIS: u64 = 600;
    if RESET {
        let mut hackrf = Hackrf::open_first()?;
        hackrf.reset()?;
        std::thread::sleep(std::time::Duration::from_millis(RESET_MILLIS));
    }

    let mut hackrf = Hackrf::open_first()?;

    // hackrf.reset()?;
    let options = TxOptions {
        center_freq: 433_500_000_u64.into(),
        sample_rate: 10_000_000_f64.into(),
        bandwidth: 3_500_000,
        enable_amp: false,
        enable_bias_tee: false,
        txvga_gain: 40,
    };

    let (command_tx, command_rx) = std::sync::mpsc::sync_channel(1);
    let (done_tx, done_rx) = std::sync::mpsc::sync_channel(1);
    let done2_tx = done_tx.clone();

    // hackrf transmit loop
    let mut current_state = State::new(433_500_000.0, 10_000_000.0);
    hackrf.start_tx(options, move |samples| {
        while let Ok(command) = command_rx.try_recv() {
            if command == Command::Quit {
                eprintln!("quitting: {command:?}");
                return ControlFlow::Break(0);
            }
            if current_state.command == Command::Idle {
                eprintln!("new command: {command:?}");
                current_state.set_command(command);
            } else {
                eprintln!("previous command in progress, ignoring command: {command:?}...");
            }
        }
        let written = current_state.fill(samples);
        ControlFlow::Continue(written)
    }, move |_success| {
        done_tx.send(()).unwrap();
    })?;

    // Ctrl-C handler
    let terminate_tx = command_tx.clone();
    ctrlc::set_handler(move || {
        use core::sync::atomic::{AtomicU32, Ordering::Relaxed};
        static CTRL_C_PRESSED: AtomicU32 = AtomicU32::new(0);
        match CTRL_C_PRESSED.fetch_add(1, Relaxed) {
            0 => {
                eprintln!("ctrl-c pressed, sending quit command...");
                terminate_tx.send(Command::Quit).unwrap();
            },
            _ => {
                eprintln!("ctrl-c already pressed, force quitting...");
                done2_tx.send(()).unwrap();
                std::process::exit(1);
            },
        }        
    }).expect("Error setting Ctrl-C handler");

    // stdin handler
    let stdin_tx = command_tx.clone();
    std::thread::spawn::<_, Result<(), SendError<_>>>(move || {
        // control loop
        let mut cmd = String::new();
        let stdin = std::io::stdin();
        let mut handle = stdin.lock();

        loop {
            cmd.clear();
            if handle.read_line(&mut cmd).is_err() {
                cmd.clear();
                cmd.push('q');
            }
            let cmd = cmd.trim();
            let chr = if cmd.is_empty() { b'q' } else { cmd.as_bytes()[0] };
            if !chr.is_ascii() { continue; }
            let rest: Result<usize, _> = cmd[1..].trim().parse();
            let command = match (chr, rest) {
                (b'q', _) => Command::Quit,
                (b'v', Ok(rest)) => Command::Vibrate(rest as u8),
                (b'v', Err(_))   => Command::Vibrate(1),
                (b's', Ok(rest)) => Command::Shock(rest as u8),
                (b's', Err(_))   => Command::Shock(1),
                (b'b', Ok(rest)) => Command::Beep(rest as u8),
                (b'b', Err(_))   => Command::Beep(0xf1),
                _ => Command::Idle,
            };
            let quit = command == Command::Quit;
            stdin_tx.send(command)?;
            if quit {
                break;
            }
        }

        Ok(())
    });

    done_rx.recv()?;
    println!("done");
    hackrf.stop_tx()?;
    Ok(())
}
