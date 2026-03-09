![Python](https://img.shields.io/badge/Python-3.8+-blue?logo=python)
![C](https://img.shields.io/badge/C-ESP-IDF-green?logo=c)
![ESP32](https://img.shields.io/badge/ESP32-S3-orange?logo=espressif)

# USFMH — User Calibrated Frequency Mapping Headset

**Senior Project — Spring 2026, University of New Hampshire**

## Overview

USFMH is an embedded audio processing system that personalizes headphone audio based on an individual's hearing profile. It runs on an **ESP32-S3** and works in three phases:

1. **Hearing Test** — A binary-search-based tone test across 27 logarithmically spaced frequency bands (10 Hz – 14,263 Hz) determines which frequencies the user can and cannot hear.
2. **EQ Processing** — A 27-band parametric equalizer (peaking biquad filters) boosts frequencies the user has difficulty hearing, applied in real time to a WAV file stored on SPIFFS.
3. **Playback** — Processed audio is streamed back to a host computer over serial for playback.

A companion **Python host script** runs on a laptop to play test tones through speakers, collect user responses, and play back the processed result.

## Hardware

- ESP32-S3 development board (8 MB flash, octal PSRAM)
- USB serial connection to host computer

## Project Structure

```
├── CMakeLists.txt            # Top-level ESP-IDF project file
├── partitions.csv            # Custom partition table (NVS, app, SPIFFS)
├── sdkconfig.defaults        # SDK configuration defaults
├── run.sh                    # One-click build, flash & run script (fish)
├── data.md                   # Test frequency and band-edge reference data
├── USFMH/main/
│   ├── main.c                # Application entry point (app_main)
│   ├── hearing_test.c/h      # Serial-based hearing test (binary search)
│   ├── audio_processor.c/h   # 27-band biquad EQ engine
│   └── wav_utils.c/h         # WAV file read/write helpers
├── host/
│   ├── demo.py               # Python host companion script
│   └── requirements.txt      # Python dependencies
└── spiffs_data/
    └── input.wav             # Audio file to process (flashed to SPIFFS)
```

## Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/) (v5.x recommended)
- Python 3.8+
- A WAV file placed in `spiffs_data/input.wav` (16-bit mono)

## Quick Start

### 1. Build, flash & monitor

The included `run.sh` script handles everything in one step:

```bash
./run.sh --port /dev/ttyACM0
```

Flags:
- `--port <device>` — Serial port (auto-detected if omitted)
- `--skip-build` — Skip the build step
- `--skip-flash-spiffs` — Skip re-flashing the SPIFFS partition

Or manually:

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

### 2. Run the host script

In a separate terminal:

```bash
cd host
pip install -r requirements.txt
python demo.py --port /dev/ttyACM0 --baud 921600
```

The script will:
1. Handshake with the ESP32 (`READY`)
2. Play tones and prompt you to respond `y`/`n`
3. Receive and play back the EQ-processed audio

## Serial Protocol

| Direction | Message | Description |
|-----------|---------|-------------|
| Host → ESP32 | `READY\n` | Handshake to start the test |
| ESP32 → Host | `TONE <freq_hz> <duration_ms>\n` | Request to play a test tone |
| Host → ESP32 | `y\n` or `n\n` | User heard / didn't hear the tone |
| ESP32 → Host | `AUDIO <sample_rate> <num_samples>\n` | Header before streaming PCM data |
| ESP32 → Host | *raw int16 PCM bytes* | Processed audio payload |
| ESP32 → Host | `DONE\n` | Processing complete |

