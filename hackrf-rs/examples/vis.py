import numpy as np
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser(description='Visualize FFT Waterfall of HackRF IQ data')
    parser.add_argument('file_path', type=str, help='Path to the raw IQ data file')
    parser.add_argument('center_freq', type=float, help='Center frequency in Hz')
    parser.add_argument('sample_rate', type=float, help='Sample rate in Hz')
    parser.add_argument('--fft-size', type=int, default=1024, help='FFT size (default: 1024)')
    parser.add_argument('--overlap', type=float, default=0.5, help='Overlap fraction (default: 0.5)')
    parser.add_argument('--cut', type=int, default=-1, help='cut n samples')
    args = parser.parse_args()

    # Read the binary file
    with open(args.file_path, 'rb') as f:
        raw_data = f.read()

    if args.cut > 0:
        raw_data = raw_data[:args.cut * 2]

    # Ensure even number of bytes
    if len(raw_data) % 2 != 0:
        raw_data = raw_data[:-1]  # Truncate last byte if odd
    # Convert to signed 8-bit integers
    iq_data = np.frombuffer(raw_data, dtype=np.int8)

    # Separate I and Q
    i_samples = iq_data[0::2]
    q_samples = iq_data[1::2]

    # Create complex IQ data
    iq_complex = i_samples + 1j * q_samples

    # Parameters for spectrogram
    fft_size = args.fft_size
    hop_size = int(fft_size * (1 - args.overlap))
    num_segments = (len(iq_complex) - fft_size) // hop_size + 1

    # Initialize spectrogram array
    spectrogram = np.zeros((num_segments, fft_size))

    # Compute FFT for each window
    for i in range(num_segments):
        start_idx = i * hop_size
        end_idx = start_idx + fft_size
        window = iq_complex[start_idx:end_idx]
        
        # Apply window (Hann) for better spectral leakage
        window *= np.hanning(fft_size)
        
        # FFT
        fft_result = np.fft.fft(window)
        fft_shifted = np.fft.fftshift(fft_result)
        magnitude = np.abs(fft_shifted)
        
        # Convert to dB
        spectrogram[i, :] = 20 * np.log10(magnitude + 1e-10)

    # Generate frequency axis
    freq_axis = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1 / args.sample_rate)) + args.center_freq

    # Time axis (approximate, in seconds)
    time_axis = np.arange(num_segments) * (hop_size / args.sample_rate)

    # Plot the waterfall
    plt.figure(figsize=(12, 7))
    plt.pcolormesh(freq_axis, time_axis, spectrogram, shading='gouraud', cmap='viridis')
    plt.colorbar(label='Magnitude (dB)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Time (s)')
    plt.title('FFT Waterfall Spectrogram of IQ Data')
    plt.ylim(0, time_axis[-1])  # Time starts from top
    plt.gca().invert_yaxis()  # Waterfall flows down
    plt.show()

if __name__ == '__main__':
    main()