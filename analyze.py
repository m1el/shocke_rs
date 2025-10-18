from math import ceil
import matplotlib.pyplot as plt
import numpy as np

tau = 2.0 * np.pi

center_freq    = 433_500_000  # Hz
signal_freq_lo = 433_600_000
signal_freq_hi = 433_664_000
sample_rate    =  10_000_000

FAKE = False

def get_samples():
    if FAKE:
        omega = tau * (signal_freq - center_freq) / sample_rate
        time = 1
        nsamples = sample_rate * time
        samples = np.arange(0, nsamples)
        samples = np.exp(1j * omega * samples)
        return samples
    else:
        # return np.load('generated.bin',allow_pickle=True)
        return np.load('samples.bin',allow_pickle=True)


def adjust_freq(samples, signal_freq):
    omega = tau * (signal_freq - center_freq) / sample_rate
    spiral = np.arange(0, len(samples))
    spiral = np.exp(-1j * omega * spiral)
    return samples * spiral

def downsample(samples, ratio):
    nsamples = (len(samples) // ratio) * ratio
    return samples[:nsamples] \
        .reshape(-1, ratio) \
        .mean(axis=1)


def demodulate(samples):
    samples_lo = adjust_freq(samples, signal_freq_lo)
    samples_lo = np.abs(downsample(samples_lo, 250))
    samples_hi = adjust_freq(samples, signal_freq_hi)
    samples_hi = np.abs(downsample(samples_hi, 250))
    difference = samples_hi - samples_lo
    plt.figure(0)
    plt.plot(samples_lo)
    plt.plot(samples_hi)
    plt.xlabel("Samples")
    plt.ylabel("Amplitude")
    plt.legend(["low", "high"])
    plt.show()

    packets = []
    bits = []
    threshold = 0.5
    bit_rate = 16.5 / 2 # 66000
    current_run = [None, 0]
    def consume_bit(bit):
        nonlocal bits, current_run
        if current_run[0] == bit:
            current_run[1] += 1
            return
        prev_bit, run = current_run
        current_run = [bit, 1]
        length = round(run / bit_rate)
        if length == 0:
            return
        if prev_bit == None:
            if len(bits) > 0:
                packets.append(bits)
                bits = []
        else:
            for _ii in range(length):
                bits.append(prev_bit)

    for sample in difference:
        if abs(sample) < threshold:
            bit = None
        elif sample > 0.0:
            bit = 1
        else:
            bit = 0
        consume_bit(bit)

    prev_bit, run = current_run
    if prev_bit != None:
        length = round(run / bit_rate)
        if length > 0:
            for _ii in range(length):
                bits.append(prev_bit)
    if len(bits) > 0:
        packets.append(bits)

    return packets

# from binascii import hexlify
# packet = construct_packet('VIBRATE', 1)
# print(hexlify(packet))
# for c in packet:
#     print(f'{c:08b}')
samples = get_samples()
packets = demodulate(samples)
print([''.join(str(b) for b in p) for p in packets])
print([len(p) for p in packets])

# plt.figure(0)
# plt.plot(np.abs(samples_lo))
# plt.plot(np.abs(samples_hi))
# plt.xlabel("Samples")
# plt.ylabel("Amplitude")
# plt.legend(["low", "high"])
# plt.show()

'''measure_freq = 100_000

plt.figure(0)
plt.plot(downsampled)
plt.xlabel("Samples")
plt.ylabel("Amplitude")
plt.show()
'''