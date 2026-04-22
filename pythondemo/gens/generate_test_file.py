#!/usr/bin/env python3
import argparse
import numpy as np
import wave

SAMPLE_RATE = 44100
DEFAULT_FREQ_FILE = "testfreq.csv"


def load_frequencies(filepath):
    freqs = []
    with open(filepath, "r") as f:
        for line in f:
            for val in line.strip().split(","):
                val = val.strip()
                if not val:
                    continue
                try:
                    freqs.append(float(val))
                except ValueError:
                    continue
    return freqs


def generate_tone(frequencies, duration, sample_rate=SAMPLE_RATE):
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    signal = np.zeros_like(t)
    for freq in frequencies:
        signal += np.sin(2 * np.pi * freq * t)
    if len(frequencies) > 0:
        signal /= len(frequencies)
    # Normalize to 16-bit range
    signal = np.clip(signal, -1.0, 1.0)
    signal = (signal * 32767).astype(np.int16)
    return signal


def write_wav(filename, signal, sample_rate=SAMPLE_RATE):
    with wave.open(filename, "w") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(signal.tobytes())


def main():
    parser = argparse.ArgumentParser(
        description="Generate a WAV file with multiple simultaneous frequencies"
    )
    parser.add_argument(
        "--seconds", type=float, default=5.0, help="Duration in seconds (default: 5)"
    )
    parser.add_argument(
        "--freq",
        type=float,
        nargs="+",
        help="Up to 10 frequencies to play (overrides testfreq.csv)",
    )
    parser.add_argument(
        "-o", "--output", default="test_tone.wav", help="Output filename (default: test_tone.wav)"
    )
    args = parser.parse_args()

    if args.freq:
        if len(args.freq) > 10:
            parser.error("--freq accepts up to 10 frequencies")
        frequencies = args.freq
    else:
        frequencies = load_frequencies(DEFAULT_FREQ_FILE)

    if not frequencies:
        print("No frequencies provided.")
        return

    print(f"Generating {args.seconds}s tone with {len(frequencies)} frequencies:")
    for f in frequencies:
        print(f"  {f} Hz")

    signal = generate_tone(frequencies, args.seconds)
    write_wav(args.output, signal)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
