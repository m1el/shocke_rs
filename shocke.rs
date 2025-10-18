use std::sync::mpsc::{SyncSender, SendError};
use std::io::BufRead;
use core::ops::ControlFlow;
use hackrf_rs::{
    options::{RxOptions, TxOptions},
    DeviceList, Hackrf,
};
use serde::Deserialize;

mod jugbow_modem;
use jugbow_modem::{Modulator, Command};

type LazyResult<T> = Result<T, Box<dyn core::error::Error>>;

#[derive(Debug, Deserialize)]
struct ShockerConfig {
    name: String,
    uuid: String,
    #[serde(rename = "type")]
    shocker_type: String,
    device_id: u64,
    channel_id: u8,
}

#[derive(Debug, Deserialize)]
struct Config {
    shocker: Vec<ShockerConfig>,
}

fn load_config() -> LazyResult<Config> {
    let config_path = "shockers.toml";
    let config_content = std::fs::read_to_string(config_path)?;
    let config: Config = toml::from_str(&config_content)?;
    Ok(config)
}

fn listen() -> LazyResult<()> {
    let demodulator = jugbow_modem::Demodulator::new(
        433_500_000.0, 10_000_000.0, move |radio| {
            println!("Received data: {:x?}", radio);
        },
    );
    let hackrf = Hackrf::open_first()?;
    let rx_options = RxOptions {
        center_freq: 433_500_000_u64.into(),
        sample_rate: 10_000_000_f64.into(),
        bandwidth: 3_500_000,
        enable_amp: false,
        enable_bias_tee: false,
        lna_gain: 40,
        vga_gain: 20,
    };
    demodulator.start_rx(hackrf, rx_options)?;
    Ok(())
}

fn ctrlc_handler(terminate_tx: SyncSender<Command>, done2_tx: SyncSender<()>) {
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
}

fn stdin_loop(stdin_tx: SyncSender<Command>) -> Result<(), SendError<Command>> {
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
        // ctrl-d pressed
        if cmd.is_empty() {
            cmd.push('q');
        }
        // empty line = vibrate once
        if cmd.trim().is_empty() {
            cmd.clear();
            cmd.push('v');
        }
        let cmd = cmd.trim();
        let chr = cmd.as_bytes()[0];
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
}

fn control() -> LazyResult<()> {
    // Load shocker configurations
    let config = load_config()?;
    
    if config.shocker.is_empty() {
        return Err("No shockers configured in shockers.toml".into());
    }
    
    // Use the first shocker for now
    let shocker = &config.shocker[0];
    println!("Using shocker: {} ({})", shocker.name, shocker.uuid);
    println!("  Type: {}", shocker.shocker_type);
    println!("  Device ID: 0x{:016x}", shocker.device_id);
    println!("  Channel ID: 0x{:02x}", shocker.channel_id);

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
    let mut current_state = Modulator::new(433_500_000.0, 10_000_000.0, shocker.device_id, shocker.channel_id);
    hackrf.start_tx(options, move |samples| {
        while let Ok(command) = command_rx.try_recv() {
            if command == Command::Quit {
                eprintln!("quitting: {command:?}");
                return ControlFlow::Break(0);
            }
            if *current_state.get_command() == Command::Idle {
                eprintln!("new command: {command:?}");
                current_state.set_command(command);
            } else {
                eprintln!("previous command in progress, ignoring command: {command:?}...");
            }
        }
        if *current_state.get_command() == Command::Quit {
        }
        let written = current_state.fill(samples);
        ControlFlow::Continue(written)
    }, move |_success| {
        done_tx.send(()).unwrap();
    })?;

    // Ctrl-C handler
    ctrlc_handler(command_tx.clone(), done2_tx);

    // stdin handler
    let stdin_tx = command_tx.clone();
    std::thread::spawn(move || stdin_loop(stdin_tx));

    done_rx.recv()?;
    println!("done");
    hackrf.stop_tx()?;
    Ok(())
}

fn main() -> LazyResult<()> {
    let args: Vec<String> = std::env::args().collect();
    if args[1..] == ["listen"] {
        listen()
    } else {
        control()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_config() {
        let config = load_config().expect("Failed to load config");
        assert!(!config.shocker.is_empty(), "Config should have at least one shocker");
        
        let shocker = &config.shocker[0];
        assert_eq!(shocker.name, "The First One");
        assert_eq!(shocker.uuid, "0199e439-c376-797d-92f5-6b0f077ecf53");
        assert_eq!(shocker.shocker_type, "jugbow");
        assert_eq!(shocker.device_id, 0x3a97b259957f1a27);
        assert_eq!(shocker.channel_id, 0xa5);
    }

    #[test]
    fn test_modulator_with_config() {
        let config = load_config().expect("Failed to load config");
        let shocker = &config.shocker[0];
        
        let state = Modulator::new(433_500_000.0, 10_000_000.0, shocker.device_id, shocker.channel_id);
        assert_eq!(*state.get_command(), Command::Idle);
    }
}
