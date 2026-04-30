#!/usr/bin/env python3
"""
USFMH Demo Host Script

Companion script that runs on a laptop and communicates with the ESP32-S3
over serial. Handles:
  1. Playing test tones through laptop speakers
  2. Collecting user responses (y/n)
  3. Receiving and playing back processed audio from the ESP32
  4. Optionally uploading a WAV file to SPIFFS before the test

Usage:
  python demo.py --port /dev/ttyUSB0
  python demo.py --port /dev/ttyUSB0 --upload my_audio.wav
    python demo.py --port /dev/ttyUSB0 --sample 12
  python demo.py --port COM3 --baud 115200
"""

import argparse
import time
import wave
from pathlib import Path

import numpy as np
import serial
import sounddevice as sd


TONE_AMPLITUDE = 0.3       # Volume for test tones (0.0 - 1.0)
PLAYBACK_SAMPLE_RATE = 44100  # Sample rate for tone generation
TONE_FADE_DURATION_S = 0.020  # fade-in / fade-out applied to test tones to avoid clicks
FIR_SAMPLE_RATE = 48000
FIR_FILTER_ORDER = 500     # legacy — kept for filters.bin / MATLAB compatibility

# Firmware FFT-filterbank parameters (must match audio_processor.c)
FFT_N = 16384
FFT_XOVER_BINS = 2.0       # raised-cosine transition width in bins at each band edge

# Serial / boot timings for the USB Serial-JTAG link on ESP32-S3
SERIAL_DEFAULT_BAUD = 115200
RESET_DTR_HOLD_S = 0.1
RESET_RTS_HOLD_S = 0.5
BOOT_BANNER_TIMEOUT_S = 15.0
PROTOCOL_LOOP_TIMEOUT_S = 120.0

# Filesystem paths (relative to repo root). The "spiffs_data" directory name is
# kept for historical reasons — the firmware now reads input.wav from the SD
# card, but the host script still stages the prepared WAV here as a working copy.
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_WAV_PATH = REPO_ROOT / "spiffs_data" / "input.wav"
DUMP_DIR = REPO_ROOT / "dumps"

# Visual reference frequencies overlaid on the spectrum plots.
PLOT_REFERENCE_FREQS = (392.0, 1975.33, 4698.67, 5301.33)

# Band definitions matching filter_bank_coeffs.h (32-band FIR bank used by firmware)
FILTER_BANK_MIN_FREQS = np.array([
    33.50, 41.50, 51.00, 62.50, 77.00, 95.00, 117.50, 145.00,
    178.50, 220.00, 271.50, 335.00, 413.00, 509.00, 627.50, 773.50,
    953.00, 1174.00, 1446.00, 1781.50, 2195.50, 2705.00, 3333.00, 4107.00,
    5060.00, 6234.50, 7681.00, 8739.50, 9722.50, 12722.50, 15000.00, 17500.00,
], dtype=np.float64)

FILTER_BANK_MAX_FREQS = np.array([
    41.50, 51.00, 62.50, 77.00, 95.00, 117.50, 145.00, 178.50,
    220.00, 271.50, 335.00, 413.00, 509.00, 627.50, 773.50, 953.00,
    1174.00, 1446.00, 1781.50, 2195.50, 2705.00, 3333.00, 4107.00, 5060.00,
    6234.50, 7681.00, 8739.50, 9722.50, 12722.50, 15000.00, 17500.00, 20000.00,
], dtype=np.float64)

# True midpoints of each band: (min + max) / 2
# Computed rather than copied from FILTER_REFERENCE_FREQS to avoid the
# duplicate-endpoint bug that placed bands 29-31 centers at their edges.
FILTER_BANK_CENTER_FREQS = (FILTER_BANK_MIN_FREQS + FILTER_BANK_MAX_FREQS) * 0.5

FILTER_REFERENCE_FREQS = np.array([
    30.0, 37.0, 46.0, 56.0, 69.0, 85.0, 105.0, 130.0,
    160.0, 197.0, 243.0, 300.0, 370.0, 456.0, 562.0, 693.0,
    854.0, 1052.0, 1296.0, 1596.0, 1967.0, 2424.0, 2986.0, 3680.0,
    4534.0, 5586.0, 6883.0, 8479.0, 9000.0, 10445.0, 15000.0,
    15000.0, 20000.0, 20000.0,
], dtype=np.float64)

TEST_SAMPLE_FREQS = np.array([
    10.0, 13.0, 17.0, 23.0, 30.0, 40.0, 52.0, 69.0, 91.0,
    120.0, 159.0, 211.0, 280.0, 371.0, 491.0, 651.0, 862.0,
    1142.0, 1512.0, 2001.0, 2649.0, 3507.0, 4644.0, 6149.0,
    8141.0, 10776.0, 14263.0,
], dtype=np.float64)


def build_filter_layout() -> tuple[np.ndarray, np.ndarray]:
    """Return the FIR band edges and center frequencies used by the firmware."""
    edges = (FILTER_REFERENCE_FREQS[:-1] + FILTER_REFERENCE_FREQS[1:]) * 0.5
    # Use true midpoints of each band so normalization in design_bandpass_fir
    # is anchored at the passband center, not at a band edge.
    # (FILTER_REFERENCE_FREQS has duplicate endpoint values that would place
    # the center of the last few bands at their edges rather than their middle.)
    centers = (edges[:-1] + edges[1:]) * 0.5
    return edges, centers


# ─────────────────────────────────────────────────────────────────────────────
# FFT-filterbank helpers — mirror of firmware audio_processor.c
# ─────────────────────────────────────────────────────────────────────────────

def fft_band_weight(freq_hz: np.ndarray | float, fmin: float, fmax: float,
                    delta_hz: float) -> np.ndarray | float:
    """Raised-cosine band window matching band_weight() in audio_processor.c.

    Returns 1.0 inside [fmin+Δ/2, fmax-Δ/2], 0.0 outside [fmin-Δ/2, fmax+Δ/2],
    and half-cosine transition at each edge. Adjacent bands with matched edges
    sum to 1.0 (partition of unity)."""
    f = np.asarray(freq_hz, dtype=np.float64)
    half = delta_hz * 0.5
    rising = 0.5 * (1.0 - np.cos(np.pi * (f - (fmin - half)) / delta_hz))
    falling = 0.5 * (1.0 + np.cos(np.pi * (f - (fmax - half)) / delta_hz))
    out = np.zeros_like(f)
    # Pass band (flat 1.0)
    flat = (f >= fmin + half) & (f <= fmax - half)
    # Rising edge at fmin
    rise = (f > fmin - half) & (f < fmin + half)
    # Falling edge at fmax
    fall = (f > fmax - half) & (f < fmax + half)
    out[flat] = 1.0
    out[rise] = rising[rise]
    out[fall] = falling[fall]
    return out


def fft_composite_mask(gains: np.ndarray,
                       mins: np.ndarray = FILTER_BANK_MIN_FREQS,
                       maxs: np.ndarray = FILTER_BANK_MAX_FREQS,
                       sample_rate: int = FIR_SAMPLE_RATE,
                       fft_n: int = FFT_N) -> np.ndarray:
    """Build the real-valued composite magnitude mask H_mag[k] for k = 0..N/2,
    mirroring build_mask() in audio_processor.c for the positive half.

    gains: linear gain per band (0 = muted). One per band in FILTER_BANK_*."""
    bin_hz = sample_rate / fft_n
    delta_hz = FFT_XOVER_BINS * bin_hz
    k = np.arange(fft_n // 2 + 1)
    freqs = k * bin_hz
    H = np.zeros_like(freqs)
    for j in range(len(mins)):
        if gains[j] == 0.0:
            continue
        H += gains[j] * fft_band_weight(freqs, float(mins[j]), float(maxs[j]), delta_hz)
    return H


def design_bandpass_fir(low_hz: float, high_hz: float, center_hz: float,
                        sample_rate: int = FIR_SAMPLE_RATE,
                        order: int = FIR_FILTER_ORDER) -> np.ndarray:
    """Legacy windowed-sinc FIR bandpass designer.

    No longer used by the firmware runtime (which uses an FFT filterbank), but
    still used by export_filter_matrix() to write filters.bin / MATLAB-style
    coefficient files."""
    tap_count = order + 1
    midpoint = order / 2.0
    n = np.arange(tap_count, dtype=np.float64)
    k = n - midpoint

    coeffs = np.empty(tap_count, dtype=np.float64)
    zero_mask = np.isclose(k, 0.0)
    coeffs[zero_mask] = 2.0 * (high_hz - low_hz) / sample_rate

    non_zero = ~zero_mask
    coeffs[non_zero] = (
        np.sin(2.0 * np.pi * high_hz * k[non_zero] / sample_rate)
        - np.sin(2.0 * np.pi * low_hz * k[non_zero] / sample_rate)
    ) / (np.pi * k[non_zero])

    window = np.kaiser(tap_count, 15.0)
    coeffs *= window

    response = np.sum(coeffs * np.exp(-1j * 2.0 * np.pi * center_hz * n / sample_rate))
    magnitude = abs(response)
    if magnitude > 1.0e-9:
        coeffs /= magnitude

    return coeffs.astype(np.float32)


def export_filter_matrix(output_path: Path,
                         sample_rate: int = FIR_SAMPLE_RATE,
                         order: int = FIR_FILTER_ORDER):
    """Write the FIR bank coefficients in a text format that is easy to paste into a matrix."""
    edges, centers = build_filter_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="ascii") as handle:
        handle.write(f"Filter sample rate: {sample_rate} Hz\n")
        handle.write(f"Filter order: {order}\n\n")

        matrix_rows = []
        for index, center_hz in enumerate(centers, start=1):
            low_hz = float(edges[index - 1])
            high_hz = float(edges[index])
            coeffs = design_bandpass_fir(low_hz, high_hz, float(center_hz), sample_rate, order)
            matrix_rows.append(coeffs)

            handle.write(
                f"Filter {index}: fmin = {low_hz:.2f} Hz, "
                f"fmax = {high_hz:.2f} Hz, fcenter = {center_hz:.0f} Hz\n"
            )
            for coeff in coeffs:
                handle.write(f"{coeff:.16f}\n")
            handle.write("\n\n")

        handle.write("Matrix form:\n")
        handle.write("[\n")
        for coeffs in matrix_rows:
            row = " ".join(f"{coeff:.16f}" for coeff in coeffs)
            handle.write(f"  {row}\n")
        handle.write("]\n")


def read_wav_as_mono_float(path: Path) -> tuple[np.ndarray, int]:
    """Read a PCM WAV file and return mono float32 samples in [-1.0, 1.0]."""
    with wave.open(str(path), "rb") as wav_file:
        channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        sample_rate = wav_file.getframerate()
        frame_count = wav_file.getnframes()
        pcm = wav_file.readframes(frame_count)

    if sample_width == 1:
        audio = (np.frombuffer(pcm, dtype=np.uint8).astype(np.float32) - 128.0) / 128.0
    elif sample_width == 2:
        audio = np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0
    elif sample_width == 3:
        raw = np.frombuffer(pcm, dtype=np.uint8).reshape(-1, 3)
        signed = (
            raw[:, 0].astype(np.int32)
            | (raw[:, 1].astype(np.int32) << 8)
            | (raw[:, 2].astype(np.int32) << 16)
        )
        sign_mask = 1 << 23
        audio = ((signed ^ sign_mask) - sign_mask).astype(np.float32) / 8388608.0
    elif sample_width == 4:
        audio = np.frombuffer(pcm, dtype="<i4").astype(np.float32) / 2147483648.0
    else:
        raise ValueError(f"Unsupported WAV sample width: {sample_width} bytes")

    if channels > 1:
        audio = audio.reshape(-1, channels).mean(axis=1)

    return audio.astype(np.float32), sample_rate


def resample_audio(audio: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    """Resample audio with linear interpolation to avoid extra dependencies."""
    if src_rate == dst_rate or len(audio) == 0:
        return audio.astype(np.float32, copy=True)

    duration = len(audio) / src_rate
    output_length = max(1, int(round(duration * dst_rate)))
    src_positions = np.arange(len(audio), dtype=np.float64)
    dst_positions = np.arange(output_length, dtype=np.float64) * src_rate / dst_rate
    dst_positions = np.clip(dst_positions, 0.0, max(len(audio) - 1, 0))
    return np.interp(dst_positions, src_positions, audio).astype(np.float32)


def write_mono_wav(path: Path, audio: np.ndarray, sample_rate: int):
    """Write mono float audio to a 16-bit PCM WAV file."""
    clipped = np.clip(audio, -1.0, 1.0)
    pcm16 = np.round(clipped * 32767.0).astype("<i2")

    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(pcm16.tobytes())


def prepare_wav_for_spiffs(input_path: Path, output_path: Path,
                           sample_rate: int = FIR_SAMPLE_RATE):
    """Convert an input WAV into the 16-bit mono 48 kHz SPIFFS-friendly format."""
    audio, original_rate = read_wav_as_mono_float(input_path)
    resampled = resample_audio(audio, original_rate, sample_rate)
    write_mono_wav(output_path, resampled, sample_rate)

    duration = len(resampled) / sample_rate if len(resampled) else 0.0
    print(f"Prepared {output_path} from {input_path}")
    print(f"  Original: {len(audio)} mono samples @ {original_rate} Hz")
    print(f"  Output:   {len(resampled)} mono samples @ {sample_rate} Hz ({duration:.2f}s)")


def generate_tone(freq_hz: float, duration_ms: int, sample_rate: int = PLAYBACK_SAMPLE_RATE) -> np.ndarray:
    """Generate a sine wave tone with fade-in/fade-out to avoid clicks."""
    duration_s = duration_ms / 1000.0
    t = np.linspace(0, duration_s, int(sample_rate * duration_s), endpoint=False)
    tone = TONE_AMPLITUDE * np.sin(2 * np.pi * freq_hz * t).astype(np.float32)

    fade_samples = int(TONE_FADE_DURATION_S * sample_rate)
    if fade_samples > 0 and len(tone) > 2 * fade_samples:
        fade_in = np.linspace(0, 1, fade_samples, dtype=np.float32)
        fade_out = np.linspace(1, 0, fade_samples, dtype=np.float32)
        tone[:fade_samples] *= fade_in
        tone[-fade_samples:] *= fade_out

    return tone


def play_tone(freq_hz: float, duration_ms: int):
    """Play a sine wave tone through laptop speakers."""
    tone = generate_tone(freq_hz, duration_ms)
    sd.play(tone, samplerate=PLAYBACK_SAMPLE_RATE, blocking=True)


def read_line(ser: serial.Serial, timeout: float = 30.0) -> str:
    """Read a line from serial, filtering out ESP-IDF log messages."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if ser.in_waiting > 0:
                raw = ser.readline()
                try:
                    line = raw.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue

                # Skip ESP-IDF log lines (start with I, W, E, D followed by parenthesis)
                if line and len(line) > 3 and line[0] in "IWED" and line[1] == " " and "(" in line[:20]:
                    print(f"  [ESP32] {line}")
                    continue

                if line:
                    return line
            else:
                time.sleep(0.01)
        except serial.SerialException:
            time.sleep(0.1)

    return ""


def handle_hearing_test(ser: serial.Serial):
    """
    Handle the hearing test phase.
    Reads TONE commands from ESP32, plays tones, asks user, sends response.
    Returns when a non-TONE message is received (passed back to caller).
    """
    print("\n===== HEARING TEST =====")
    print("You will hear tones at different frequencies.")
    print("Type 'y' if you can hear the tone, 'n' if you cannot.\n")

    while True:
        line = read_line(ser, timeout=60)
        if not line:
            print("Timeout waiting for ESP32")
            return ""

        if line.startswith("TONE "):
            parts = line.split()
            if len(parts) >= 3:
                freq = float(parts[1])
                duration = int(parts[2])
                print(f"Playing tone: {freq:.0f} Hz ({duration} ms)...")

                play_tone(freq, duration)

                while True:
                    response = input("  Can you hear it? (y/n): ").strip().lower()
                    if response in ("y", "n"):
                        break
                    print("  Please enter 'y' or 'n'")

                ser.write(f"{response}\n".encode("utf-8"))
                ser.flush()
        else:
            # Non-TONE line — return it for the caller to handle
            return line


def handle_audio_stream(ser: serial.Serial, sample_rate: int, num_samples: int):
    """Receive raw int16 PCM data from the ESP32 and play it through laptop speakers."""
    print(f"\nReceiving {num_samples} samples at {sample_rate} Hz...")

    total_bytes = num_samples * 2  # int16 = 2 bytes
    received = b""

    while len(received) < total_bytes:
        chunk = ser.read(min(4096, total_bytes - len(received)))
        if not chunk:
            print("Warning: timeout receiving audio data")
            break
        received += chunk
        progress = len(received) / total_bytes * 100
        print(f"\r  Receiving: {progress:.1f}%", end="", flush=True)

    print(f"\r  Received {len(received)} / {total_bytes} bytes")

    # Convert int16 to float32
    actual_samples = len(received) // 2
    audio = np.frombuffer(received[:actual_samples * 2], dtype=np.int16).astype(np.float32) / 32768.0

    print(f"Playing processed audio ({actual_samples} samples, {actual_samples / sample_rate:.1f}s)...")
    sd.play(audio, samplerate=sample_rate, blocking=True)
    print("Playback complete.")

    return audio


def plot_results(filtered_audio: np.ndarray, sample_rate: int, original_wav_path: Path):
    """
    Plot three panels matching the MATLAB poster figure:
      1. Filter Arrangement  — frequency response of all 32 firmware bands
      2. Filtered Spectrum   — FFT magnitude of the audio received from ESP32
      3. Original Spectrum   — FFT magnitude of the source WAV file
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: matplotlib is required for --plot.")
        print("  pip install matplotlib")
        return

    # ── Load original audio at the same sample rate ───────────────────────
    if original_wav_path.exists():
        orig_audio, orig_sr = read_wav_as_mono_float(original_wav_path)
        orig_audio = resample_audio(orig_audio, orig_sr, sample_rate)
    else:
        print(f"Warning: original WAV not found at {original_wav_path}, using silence.")
        orig_audio = np.zeros_like(filtered_audio)

    # ── Full-signal FFTs (mirrors MATLAB's abs(fft(sig))) ─────────────────
    L_orig = len(orig_audio)
    orig_fft = np.abs(np.fft.fft(orig_audio))
    freq_orig = np.arange(L_orig) * (sample_rate / L_orig)

    L_filt = len(filtered_audio)
    filt_fft = np.abs(np.fft.fft(filtered_audio))
    freq_filt = np.arange(L_filt) * (sample_rate / L_filt)

    # Only positive frequencies up to 20 kHz
    pos_orig = (freq_orig > 0) & (freq_orig <= 20000)
    pos_filt = (freq_filt > 0) & (freq_filt <= 20000)

    print("Computing filter frequency responses...")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.canvas.manager.set_window_title("USFMH Filter Analysis")

    # ── Subplot 1: Filter Arrangement ─────────────────────────────────────
    # FFT-filterbank: show each band's raised-cosine weight (all at unity gain)
    # and the composite mask. This matches the firmware's audio_processor.c.
    ax1 = axes[0]
    bin_hz = sample_rate / FFT_N
    delta_hz = FFT_XOVER_BINS * bin_hz
    freqs_dense = np.logspace(np.log10(20), np.log10(sample_rate / 2), 4096)
    for i in range(len(FILTER_BANK_MIN_FREQS)):
        w = fft_band_weight(freqs_dense, float(FILTER_BANK_MIN_FREQS[i]),
                            float(FILTER_BANK_MAX_FREQS[i]), delta_hz)
        mag_db = 20.0 * np.log10(w + 1e-12)
        ax1.semilogx(freqs_dense, mag_db, linewidth=0.8, alpha=0.6)

    unity = np.ones(len(FILTER_BANK_MIN_FREQS))
    Hmag = fft_composite_mask(unity, sample_rate=sample_rate)
    kfreqs = np.arange(len(Hmag)) * bin_hz
    ax1.semilogx(kfreqs, 20.0 * np.log10(Hmag + 1e-12),
                 color="black", linewidth=1.5, label="Composite mask (all bands unity)")
    ax1.axhline(-20, color="red", linestyle="--", linewidth=1.0, label="-20 dB")
    ax1.set_xlim([30, 10000])
    ax1.set_ylim([-140, 20])
    ax1.set_xlabel("Frequency in Hz")
    ax1.set_ylabel("Magnitude in dB")
    ax1.set_title(f"FFT Filterbank Arrangement (N={FFT_N}, {bin_hz:.2f} Hz/bin)")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(True, which="both", alpha=0.3)

    # ── Subplot 2: Filtered Spectrum ───────────────────────────────────────
    ax2 = axes[1]
    ax2.semilogx(freq_filt[pos_filt], filt_fft[pos_filt], linewidth=1.5)
    for rf in PLOT_REFERENCE_FREQS:
        ax2.axvline(rf, color="red", linestyle="--", linewidth=0.8, alpha=0.6)
    ax2.set_xlim([30, 20000])
    ax2.set_xlabel("Frequency in Hz")
    ax2.set_ylabel("Amplitude")
    ax2.set_title("Filtered Spectrum")
    ax2.grid(True, which="both", alpha=0.3)

    # ── Subplot 3: Original Spectrum ───────────────────────────────────────
    ax3 = axes[2]
    ax3.semilogx(freq_orig[pos_orig], orig_fft[pos_orig], linewidth=1.5)
    for rf in PLOT_REFERENCE_FREQS:
        ax3.axvline(rf, color="red", linestyle="--", linewidth=0.8, alpha=0.6)
    ax3.set_xlim([30, 20000])
    ax3.set_xlabel("Frequency in Hz")
    ax3.set_ylabel("Amplitude")
    ax3.set_title("Original Spectrum")
    ax3.grid(True, which="both", alpha=0.3)

    # Match y-axes of the two spectrum plots so they're directly comparable
    y_max = max(ax2.get_ylim()[1], ax3.get_ylim()[1])
    ax2.set_ylim([0, y_max])
    ax3.set_ylim([0, y_max])

    plt.tight_layout()
    plt.show()


def dump_diagnostics(filtered_audio: np.ndarray, sample_rate: int,
                     original_wav_path: Path, sample_label: str):
    """
    Write all meaningful diagnostic data to a text file for offline analysis.

    Includes: original/filtered time-domain samples, their FFT magnitudes,
    filter bank frequency responses, and per-band statistics.
    """
    DUMP_DIR.mkdir(exist_ok=True)
    dump_path = DUMP_DIR / f"dump_sample_{sample_label}.txt"

    # ── Load original audio ──────────────────────────────────────────────
    if original_wav_path.exists():
        orig_audio, orig_sr = read_wav_as_mono_float(original_wav_path)
        orig_audio = resample_audio(orig_audio, orig_sr, sample_rate)
    else:
        orig_audio = np.zeros_like(filtered_audio)

    # ── FFTs ─────────────────────────────────────────────────────────────
    L_orig = len(orig_audio)
    orig_fft = np.abs(np.fft.fft(orig_audio))
    freq_orig = np.arange(L_orig) * (sample_rate / L_orig)

    L_filt = len(filtered_audio)
    filt_fft = np.abs(np.fft.fft(filtered_audio))
    freq_filt = np.arange(L_filt) * (sample_rate / L_filt)

    # Positive frequencies only (up to Nyquist)
    half_orig = L_orig // 2
    half_filt = L_filt // 2

    with open(dump_path, "w") as f:
        f.write(f"USFMH Diagnostic Dump\n")
        f.write(f"Sample mode: {sample_label}\n")
        f.write(f"Sample rate: {sample_rate} Hz\n")
        f.write(f"Original samples: {L_orig}\n")
        f.write(f"Filtered samples: {L_filt}\n")
        f.write(f"Firmware DSP: FFT filterbank, N={FFT_N}, bin={sample_rate/FFT_N:.2f} Hz\n")
        f.write(f"Number of bands: {len(FILTER_BANK_MIN_FREQS)}\n\n")

        # ── Original audio stats ─────────────────────────────────────────
        f.write("=" * 70 + "\n")
        f.write("ORIGINAL AUDIO STATISTICS\n")
        f.write("=" * 70 + "\n")
        f.write(f"  min: {orig_audio.min():.6f}\n")
        f.write(f"  max: {orig_audio.max():.6f}\n")
        f.write(f"  mean: {orig_audio.mean():.6f}\n")
        f.write(f"  rms: {np.sqrt(np.mean(orig_audio**2)):.6f}\n\n")

        # ── Filtered audio stats ─────────────────────────────────────────
        f.write("=" * 70 + "\n")
        f.write("FILTERED AUDIO STATISTICS\n")
        f.write("=" * 70 + "\n")
        f.write(f"  min: {filtered_audio.min():.6f}\n")
        f.write(f"  max: {filtered_audio.max():.6f}\n")
        f.write(f"  mean: {filtered_audio.mean():.6f}\n")
        f.write(f"  rms: {np.sqrt(np.mean(filtered_audio**2)):.6f}\n\n")

        # ── Top 20 FFT peaks (original) ──────────────────────────────────
        f.write("=" * 70 + "\n")
        f.write("ORIGINAL FFT — TOP 20 PEAKS (positive freqs, up to 20 kHz)\n")
        f.write("=" * 70 + "\n")
        mask_orig = freq_orig[:half_orig] <= 20000
        orig_pos = orig_fft[:half_orig][mask_orig]
        freq_pos = freq_orig[:half_orig][mask_orig]
        top_idx = np.argsort(orig_pos)[::-1][:20]
        for rank, idx in enumerate(top_idx, 1):
            f.write(f"  {rank:2d}. {freq_pos[idx]:10.2f} Hz  amplitude={orig_pos[idx]:.4f}\n")
        f.write("\n")

        # ── Top 20 FFT peaks (filtered) ──────────────────────────────────
        f.write("=" * 70 + "\n")
        f.write("FILTERED FFT — TOP 20 PEAKS (positive freqs, up to 20 kHz)\n")
        f.write("=" * 70 + "\n")
        mask_filt = freq_filt[:half_filt] <= 20000
        filt_pos = filt_fft[:half_filt][mask_filt]
        freq_filt_pos = freq_filt[:half_filt][mask_filt]
        top_idx_f = np.argsort(filt_pos)[::-1][:20]
        for rank, idx in enumerate(top_idx_f, 1):
            f.write(f"  {rank:2d}. {freq_filt_pos[idx]:10.2f} Hz  amplitude={filt_pos[idx]:.4f}\n")
        f.write("\n")

        # ── Per-band analysis ────────────────────────────────────────────
        # Peak dB / Gain@ctr are the response of the FFT composite mask
        # (assuming all bands active at unity gain) — the actual firmware
        # filter shape. Orig/Filt energy come from the signal FFTs.
        f.write("=" * 70 + "\n")
        f.write("PER-BAND FILTER ANALYSIS (FFT composite mask, all-bands-unity)\n")
        f.write(f"{'Band':>4}  {'fmin':>10}  {'fmax':>10}  {'fcenter':>10}  "
                f"{'Peak dB':>8}  {'Gain@ctr':>9}  "
                f"{'Orig energy':>12}  {'Filt energy':>12}  {'Ratio':>8}\n")
        f.write("-" * 100 + "\n")

        bin_hz = sample_rate / FFT_N
        Hmag = fft_composite_mask(np.ones(len(FILTER_BANK_MIN_FREQS)),
                                  sample_rate=sample_rate)
        mask_freqs = np.arange(len(Hmag)) * bin_hz

        for i in range(len(FILTER_BANK_MIN_FREQS)):
            lo = FILTER_BANK_MIN_FREQS[i]
            hi = FILTER_BANK_MAX_FREQS[i]
            ctr = FILTER_BANK_CENTER_FREQS[i]

            band_bin = (mask_freqs >= lo) & (mask_freqs <= hi)
            peak_val = np.max(Hmag[band_bin]) if np.any(band_bin) else 0.0
            peak_db = 20.0 * np.log10(peak_val + 1e-12)

            ctr_idx = np.argmin(np.abs(mask_freqs - ctr))
            gain_at_ctr = 20.0 * np.log10(Hmag[ctr_idx] + 1e-12)

            band_mask_o = (freq_orig[:half_orig] >= lo) & (freq_orig[:half_orig] <= hi)
            band_mask_f = (freq_filt[:half_filt] >= lo) & (freq_filt[:half_filt] <= hi)
            orig_energy = np.sum(orig_fft[:half_orig][band_mask_o] ** 2)
            filt_energy = np.sum(filt_fft[:half_filt][band_mask_f] ** 2)
            ratio = filt_energy / orig_energy if orig_energy > 1e-12 else 0.0

            f.write(f"{i:4d}  {lo:10.2f}  {hi:10.2f}  {ctr:10.2f}  "
                    f"{peak_db:8.2f}  {gain_at_ctr:9.2f}  "
                    f"{orig_energy:12.2f}  {filt_energy:12.2f}  {ratio:8.4f}\n")
        f.write("\n")

        # ── Original FFT (positive freqs, decimated to ~2000 points) ─────
        f.write("=" * 70 + "\n")
        f.write("ORIGINAL FFT DATA (freq_hz, amplitude) — positive freqs <= 20 kHz\n")
        f.write("=" * 70 + "\n")
        step = max(1, len(freq_pos) // 2000)
        for i in range(0, len(freq_pos), step):
            f.write(f"{freq_pos[i]:.2f}\t{orig_pos[i]:.6f}\n")
        f.write("\n")

        # ── Filtered FFT (positive freqs, decimated) ─────────────────────
        f.write("=" * 70 + "\n")
        f.write("FILTERED FFT DATA (freq_hz, amplitude) — positive freqs <= 20 kHz\n")
        f.write("=" * 70 + "\n")
        step = max(1, len(freq_filt_pos) // 2000)
        for i in range(0, len(freq_filt_pos), step):
            f.write(f"{freq_filt_pos[i]:.2f}\t{filt_pos[i]:.6f}\n")
        f.write("\n")

        # ── Time-domain samples (first 200 + last 200) ───────────────────
        f.write("=" * 70 + "\n")
        f.write("TIME-DOMAIN SAMPLES (first 200 original vs filtered)\n")
        f.write("=" * 70 + "\n")
        n_show = min(200, L_orig, L_filt)
        for i in range(n_show):
            o = orig_audio[i] if i < L_orig else 0.0
            fl = filtered_audio[i] if i < L_filt else 0.0
            f.write(f"{i}\t{o:.8f}\t{fl:.8f}\n")

    print(f"Diagnostic dump written to: {dump_path}")
    print(f"  File size: {dump_path.stat().st_size / 1024:.1f} KB")


def upload_wav_to_spiffs(port: str, baud: int, wav_path: str):
    """
    Print instructions for uploading a WAV file to the ESP32's SPIFFS partition.

    Note: the production firmware reads input.wav from the SD card, not from
    SPIFFS. This helper is kept as a fallback for boards without SD wiring.
    """
    print(f"\nTo upload '{wav_path}' to SPIFFS, run:")
    print(f"  1. Create SPIFFS image:")
    print(f"     mkdir -p spiffs_data && cp {wav_path} spiffs_data/input.wav")
    print(f"     python -m spiffsgen 1048576 spiffs_data spiffs.bin")
    print(f"  2. Flash SPIFFS partition:")
    print(f"     esptool.py --port {port} --baud {baud} write_flash 0x110000 spiffs.bin")
    print(f"\nOr use: idf.py -p {port} spiffsgen")
    print()


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser for the demo host script."""
    parser = argparse.ArgumentParser(description="USFMH Demo Host Script")
    parser.add_argument("--port", help="Serial port (e.g., /dev/ttyUSB0, COM3)")
    parser.add_argument("--baud", type=int, default=SERIAL_DEFAULT_BAUD,
                        help=f"Baud rate (default: {SERIAL_DEFAULT_BAUD})")
    parser.add_argument("--upload", metavar="WAV_FILE",
                        help="Show instructions to upload a WAV file to SPIFFS before the test")
    parser.add_argument("--prepare-wav", metavar="WAV_FILE",
                        help="Convert a WAV file to 16-bit mono 48 kHz for SPIFFS")
    parser.add_argument("--prepared-output", type=Path, default=DEFAULT_INPUT_WAV_PATH,
                        help=f"Output path for --prepare-wav (default: {DEFAULT_INPUT_WAV_PATH})")
    parser.add_argument("--export-filter-matrix", type=Path, metavar="TXT_FILE",
                        help="Write the FIR coefficient bank to a text file")
    parser.add_argument("--filter-sample-rate", type=int, default=FIR_SAMPLE_RATE,
                        help="Sample rate to use for offline FIR export/prep helpers")
    parser.add_argument("--sample", type=str,
                        help=(
                            "Debug mode: select one hearing-test sample index or 'all' for all active DSP bands "
                            f"(0-{len(TEST_SAMPLE_FREQS) - 1})"
                        ))
    parser.add_argument("--plot", action="store_true",
                        help="After processing, show filter arrangement and spectrum plots")
    parser.add_argument("--dump", action="store_true",
                        help="Dump diagnostic data (FFTs, per-band stats, samples) to dumps/ directory")
    return parser


def parse_sample_arg(parser: argparse.ArgumentParser, raw: str | None) -> tuple[int | None, bool]:
    """Validate the `--sample` flag. Returns (sample_index, sample_all)."""
    if raw is None:
        return None, False
    if raw.lower() == "all":
        return None, True
    try:
        idx = int(raw)
    except ValueError:
        parser.error(f"--sample must be 'all' or in range 0-{len(TEST_SAMPLE_FREQS) - 1}")
    if not (0 <= idx < len(TEST_SAMPLE_FREQS)):
        parser.error(f"--sample must be in range 0-{len(TEST_SAMPLE_FREQS) - 1}")
    return idx, False


def _wait_for_boot_banner(ser: serial.Serial, timeout_s: float) -> str:
    """Drain serial until the firmware prints its 'Waiting for host' banner.
    Returns the accumulated boot log (whether or not the banner was seen)."""
    boot_log = ""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                chunk = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
                boot_log += chunk
                if "Waiting for host" in boot_log:
                    return boot_log
            except Exception:
                pass
        time.sleep(0.1)
    return boot_log


def connect_and_boot(port: str, baud: int) -> serial.Serial | None:
    """Open the serial port, pulse DTR/RTS to reset the board, and wait for
    the firmware boot banner. Returns the open Serial on success, or None
    after closing it on failure (so the caller can simply check for None)."""
    print(f"Connecting to ESP32 on {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)

    print("Resetting ESP32...")
    ser.dtr = False
    ser.rts = True
    time.sleep(RESET_DTR_HOLD_S)
    ser.rts = False
    time.sleep(RESET_RTS_HOLD_S)

    # Drain any boot ROM / bootloader output before waiting for the banner
    if ser.in_waiting:
        ser.read(ser.in_waiting)

    print("Waiting for ESP32 boot (press RST if nothing happens)...")
    boot_log = _wait_for_boot_banner(ser, BOOT_BANNER_TIMEOUT_S)

    if "Waiting for host" not in boot_log:
        # DTR/RTS reset isn't reliable on USB Serial/JTAG — fall back to manual RST
        print("DTR/RTS reset didn't trigger boot. Press RST on the board now...")
        boot_log = _wait_for_boot_banner(ser, BOOT_BANNER_TIMEOUT_S)

    if "Waiting for host" not in boot_log:
        print("ERROR: Timed out waiting for ESP32 boot banner.")
        print("       Make sure firmware is flashed and the USB cable is connected.")
        if boot_log.strip():
            print(f"       Received: {boot_log[:200]}")
        ser.close()
        return None

    time.sleep(0.2)
    return ser


def send_handshake(ser: serial.Serial, sample_index: int | None, sample_all: bool):
    """Send the appropriate READY handshake based on debug-sample selection."""
    if sample_index is None and not sample_all:
        print("Sending READY handshake...")
        ser.write(b"READY\n")
    elif sample_all:
        print("Sending READY handshake with debug sample all (all DSP bands active)...")
        ser.write(b"READY SAMPLE ALL\n")
    else:
        selected_hz = TEST_SAMPLE_FREQS[sample_index]
        print(
            f"Sending READY handshake with debug sample {sample_index} "
            f"({selected_hz:.0f} Hz)..."
        )
        ser.write(f"READY SAMPLE {sample_index}\n".encode("utf-8"))
    ser.flush()


def run_protocol_loop(ser: serial.Serial, args: argparse.Namespace,
                      sample_index: int | None, sample_all: bool):
    """Drive the host side of the TONE / AUDIO / DONE protocol once the
    READY handshake has been sent. Plays tones, collects user responses,
    receives the processed audio stream, and triggers --plot/--dump on
    completion."""
    while True:
        line = read_line(ser, timeout=PROTOCOL_LOOP_TIMEOUT_S)
        if not line:
            continue

        if line.startswith("TONE "):
            parts = line.split()
            if len(parts) >= 3:
                freq = float(parts[1])
                duration = int(parts[2])

                if sample_index is not None:
                    target_hz = TEST_SAMPLE_FREQS[sample_index]
                    response = "y" if abs(freq - target_hz) < 0.5 else "n"
                    print(
                        f"Auto-response for debug sample mode: {freq:.0f} Hz -> {response}"
                    )
                    ser.write(f"{response}\n".encode("utf-8"))
                    ser.flush()
                    continue

                print("\n===== HEARING TEST =====")
                print("You will hear tones at different frequencies.")
                print("Type 'y' if you can hear the tone, 'n' if you cannot.\n")

                print(f"Playing tone: {freq:.0f} Hz ({duration} ms)...")
                play_tone(freq, duration)

                while True:
                    response = input("  Can you hear it? (y/n): ").strip().lower()
                    if response in ("y", "n"):
                        break
                    print("  Please enter 'y' or 'n'")

                ser.write(f"{response}\n".encode("utf-8"))
                ser.flush()

                # Continue handling the remaining TONE messages
                remaining = handle_hearing_test(ser)
                if remaining:
                    line = remaining
                else:
                    continue

        if line.startswith("AUDIO "):
            parts = line.split()
            if len(parts) >= 3:
                sample_rate = int(parts[1])
                num_samples = int(parts[2])
                audio = handle_audio_stream(ser, sample_rate, num_samples)

                # Wait for DONE
                done_line = read_line(ser, timeout=30)
                if done_line == "DONE":
                    print("\n===== COMPLETE =====")
                    print("Audio processing pipeline finished successfully.")
                else:
                    print(f"Expected DONE, got: {done_line}")

                # Build sample label for filenames
                if sample_all:
                    sample_label = "a"
                elif sample_index is not None:
                    sample_label = str(sample_index)
                else:
                    sample_label = "hearing"

                if args.dump:
                    dump_diagnostics(audio, sample_rate, DEFAULT_INPUT_WAV_PATH, sample_label)
                if args.plot:
                    plot_results(audio, sample_rate, DEFAULT_INPUT_WAV_PATH)
                return

        elif line == "DONE":
            print("\n===== COMPLETE =====")
            return


def run_offline_helpers(args: argparse.Namespace) -> bool:
    """Run any of the non-serial helper modes the user selected on the CLI.
    Returns True if at least one offline helper ran (used by main() to decide
    whether --port is still required)."""
    ran_any = False
    if args.prepare_wav:
        prepare_wav_for_spiffs(Path(args.prepare_wav), args.prepared_output, args.filter_sample_rate)
        ran_any = True

    if args.export_filter_matrix:
        export_filter_matrix(args.export_filter_matrix, args.filter_sample_rate, FIR_FILTER_ORDER)
        print(f"Wrote filter matrix to {args.export_filter_matrix}")
        ran_any = True

    if args.upload:
        upload_wav_to_spiffs(args.port, args.baud, args.upload)
        input("Press Enter after uploading the WAV file to continue...")
        ran_any = True

    return ran_any


def main():
    """CLI entry point. Parses args, runs any offline helpers, then (if a port
    is supplied) connects to the ESP32 and drives the host side of the
    hearing-test + audio-streaming protocol."""
    parser = build_arg_parser()
    args = parser.parse_args()

    sample_index, sample_all = parse_sample_arg(parser, args.sample)

    ran_offline = run_offline_helpers(args)

    if not args.port:
        if ran_offline:
            return
        parser.error("--port is required unless you are only using offline helpers")

    ser = connect_and_boot(args.port, args.baud)
    if ser is None:
        return

    try:
        send_handshake(ser, sample_index, sample_all)
        run_protocol_loop(ser, args, sample_index, sample_all)
    finally:
        ser.close()
        print("Connection closed.")


if __name__ == "__main__":
    main()
