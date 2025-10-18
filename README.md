# shocke_rs

A Rust-based shocker control system using HackRF.

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

Run the application with:

```bash
cargo run
```

The application will load the shocker configuration from `shockers.toml` and display which shocker it's using.

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
