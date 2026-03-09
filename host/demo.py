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
  python demo.py --port COM3 --baud 115200
"""

import argparse
import struct
import sys
import time

import numpy as np
import serial
import sounddevice as sd


TONE_AMPLITUDE = 0.3       # Volume for test tones (0.0 - 1.0)
PLAYBACK_SAMPLE_RATE = 44100  # Sample rate for tone generation


def generate_tone(freq_hz: float, duration_ms: int, sample_rate: int = PLAYBACK_SAMPLE_RATE) -> np.ndarray:
    """Generate a sine wave tone with fade-in/fade-out to avoid clicks."""
    duration_s = duration_ms / 1000.0
    t = np.linspace(0, duration_s, int(sample_rate * duration_s), endpoint=False)
    tone = TONE_AMPLITUDE * np.sin(2 * np.pi * freq_hz * t).astype(np.float32)

    # Apply 20ms fade-in and fade-out
    fade_samples = int(0.02 * sample_rate)
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


def upload_wav_to_spiffs(port: str, baud: int, wav_path: str):
    """
    Upload a WAV file to the ESP32's SPIFFS partition.
    Uses esptool/parttool. This is a simplified helper — for production,
    use idf.py spiffsgen or the partition tool.
    """
    print(f"\nTo upload '{wav_path}' to SPIFFS, run:")
    print(f"  1. Create SPIFFS image:")
    print(f"     mkdir -p spiffs_data && cp {wav_path} spiffs_data/input.wav")
    print(f"     python -m spiffsgen 1048576 spiffs_data spiffs.bin")
    print(f"  2. Flash SPIFFS partition:")
    print(f"     esptool.py --port {port} --baud {baud} write_flash 0x110000 spiffs.bin")
    print(f"\nOr use: idf.py -p {port} spiffsgen")
    print()


def main():
    parser = argparse.ArgumentParser(description="USFMH Demo Host Script")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0, COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--upload", metavar="WAV_FILE",
                        help="Show instructions to upload a WAV file to SPIFFS before the test")
    args = parser.parse_args()

    if args.upload:
        upload_wav_to_spiffs(args.port, args.baud, args.upload)
        input("Press Enter after uploading the WAV file to continue...")

    print(f"Connecting to ESP32 on {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=1)

    # Reset the ESP32 via DTR/RTS toggle
    print("Resetting ESP32...")
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False
    time.sleep(1.5)  # Wait for boot

    # Flush any boot messages
    while ser.in_waiting:
        ser.read(ser.in_waiting)
        time.sleep(0.1)

    # Send handshake
    print("Sending READY handshake...")
    ser.write(b"READY\n")
    ser.flush()

    # Main protocol loop
    while True:
        line = read_line(ser, timeout=60)
        if not line:
            continue

        if line.startswith("TONE "):
            # First TONE message — enter hearing test handler
            # Re-inject this line by processing it here
            parts = line.split()
            if len(parts) >= 3:
                freq = float(parts[1])
                duration = int(parts[2])

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

                # Continue handling more TONE messages
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
                break

        elif line == "DONE":
            print("\n===== COMPLETE =====")
            break

    ser.close()
    print("Connection closed.")


if __name__ == "__main__":
    main()
