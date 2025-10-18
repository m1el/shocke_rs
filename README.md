# shocke_rs

A Rust-based shocker control system using HackRF.

## Features

- **Transmit Mode**: Control shockers using radio commands
- **Capture Mode**: Capture and decode remote control signals to extract device IDs and channel IDs

## Configuration

Create a `shockers.toml` file in the same directory as the binary to configure your shockers.

### Example Configuration

```toml
[[shocker]]
name = "The First One"
uuid = "0199e439-c376-797d-92f5-6b0f077ecf53"
type = "jugbow"
device_id = 0x3a97b259957f1a27
channel_id = 0xa5
```

### Configuration Fields

- `name`: A human-readable name for the shocker
- `uuid`: A unique identifier for the shocker
- `type`: The type/model of the shocker (e.g., "jugbow")
- `device_id`: The device ID in hexadecimal format (64-bit)
- `channel_id`: The channel ID in hexadecimal format (8-bit)

You can configure multiple shockers by adding additional `[[shocker]]` sections. The application will use the first shocker in the configuration.

## Usage

### Capturing Remote IDs

To capture device IDs and channel IDs from your remote control:

```bash
cargo run --example capture
```

The capture tool will:
1. Listen for radio signals on 433.5 MHz
2. Demodulate and decode FSK-modulated packets
3. Display captured device information including device_id and channel_id
4. Print configuration snippets ready to paste into `shockers.toml`

Press your remote control buttons while the capture tool is running. When a valid packet is received, you'll see output like:

```
=== Radio Command Received ===
  Device ID:   0x3a97b259957f1a27
  Channel ID:  0xa5
  Counter:     0xc2
  Command:     VIBRATE (0x72)
  Intensity:   5
  >>> NEW REMOTE DETECTED! <<<

=== Configuration for shockers.toml ===
[[shocker]]
name = "Remote 3a97b259957f1a27"
uuid = "<generate-your-uuid>"
type = "jugbow"
device_id = 0x3a97b259957f1a27
channel_id = 0xa5
```

### Transmit Mode

Run the application with:

```bash
cargo run
```

The application will load the shocker configuration from `shockers.toml` and display which shocker it's using.

## Architecture

The project is organized into the following modules:

- **jugbow_modem**: Core FSK modulation/demodulation module
  - `Modulator`: Generates FSK-modulated signals for transmission
  - `Demodulator`: Receives and decodes FSK-modulated signals
  - `RadioCommand`: Data structure for radio commands
  - `Command`: Enum for command types (Shock, Vibrate, Beep)

## Requirements

- Rust toolchain
- HackRF hardware
- libhackrf library

### Installing Dependencies

On Ubuntu/Debian:
```bash
sudo apt install libhackrf-dev libhackrf0
```

On macOS with Homebrew:
```bash
brew install hackrf
```

## Development

### Running Tests

```bash
cargo test
```

### Building

```bash
cargo build
```

