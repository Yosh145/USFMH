# USFMH — User Calibrated Frequency Mapping Headset

**Senior Project — Spring 2026, University of New Hampshire**

## Overview

The User Calibrated Frequency Mapping Headset is an embedded audio processing system that personalizes headphone audio based on an individual's hearing profile. It runs on an **ESP32-S3** and works in three phases:

1. **Hearing Test** — A binary-search-based tone test across 27 logarithmically spaced frequency bands (10 Hz – 14,263 Hz) determines which frequencies the user can and cannot hear.
2. **EQ Processing** — A MATLAB-style 32-band FIR filter bank boosts frequencies the user has difficulty hearing, applied to a WAV file stored on SPIFFS.
3. **Playback** — Processed audio is streamed back to a host computer over serial for playback.

A companion **Python host script** runs on a laptop to play test tones through speakers, collect user responses, and play back the processed result.

## Hardware

- ESP32-S3 development board (8 MB flash, octal PSRAM)
- microSD card (FAT32) for input WAV and filter coefficients
- USB serial connection to host computer

The pinout used by this firmware is captured in [`ESP32S3_Pinout.png`](ESP32S3_Pinout.png).
The full bill of materials for the final speaker build is in [`Senior Project Parts.pdf`](Senior%20Project%20Parts.pdf).

## Project Structure

```
├── CMakeLists.txt            # Top-level ESP-IDF project file
├── partitions.csv            # Custom partition table (NVS, app, SPIFFS)
├── sdkconfig.defaults        # SDK configuration defaults
├── run.sh                    # One-click build, flash & run script (fish)
├── data.md                   # Hearing-test frequency and band-edge reference data
├── USFMH/main/
│   ├── main.c                # Application entry point (app_main)
│   ├── sd_card.c/h           # SD card mount + filesystem helpers
│   ├── coeff_loader.c/h      # Load FIR bank from /sdcard/filters.bin
│   ├── hearing_test.c/h      # Serial-based hearing test (binary search)
│   ├── audio_processor.c/h   # 32-band FFT-filterbank EQ engine
│   ├── wav_utils.c/h         # WAV file read/write helpers
│   └── filter_bank_coeffs.h  # Legacy baked-in coefficients (not compiled in)
├── pythondemo/
│   ├── demo.py               # Python host script and offline FIR/WAV prep helpers
│   ├── requirements.txt      # Python dependencies
│   └── gens/                 # Filter/test-file generation scripts
├── MATLAB/
│   ├── Senior_Project_test_script6.m  # Reference 32-band FIR design
│   ├── matrix.txt            # MATLAB-generated coefficient matrix
│   └── README.md             # MATLAB workflow notes
├── archive/debug_dumps/      # Reference serial-capture dumps (--dump output)
└── spiffs_data/
    └── input.wav             # Working copy of the audio (host-side staging area)
```

### Architecture (3 layers)

1. **MATLAB reference** ([`MATLAB/`](MATLAB/)) — ground-truth FIR design (5001-tap filters). The script writes `matrix.txt`, which the Python tooling re-formats for the firmware.
2. **Python host** ([`pythondemo/demo.py`](pythondemo/demo.py)) — designs the 501-tap firmware filters, plays test tones, drives the hearing test over serial, receives processed audio, optionally plots/dumps diagnostics.
3. **ESP32-S3 firmware** ([`USFMH/main/`](USFMH/main/)) — at runtime, mounts the SD card, loads `filters.bin` into PSRAM, runs the binary-search hearing test over serial, then applies an FFT-based composite-mask filterbank to the WAV in `/sdcard/input.wav` and streams the result back to the host.

Coefficient flow:
```
MATLAB matrix.txt  ──►  pythondemo/gens/generate_filter_bin.py  ──►  filters.bin  ──►  /sdcard/filters.bin (boot-time load)
```

## Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/) (v5.x recommended)
- Python 3.8+
- A WAV file placed in `spiffs_data/input.wav` (16-bit mono)

## Quick Start

Until the SD card arrives, the project uses the existing SPIFFS partition for audio input and output. The host script can now both prepare a WAV for SPIFFS and export the FIR coefficient matrix used by the firmware.

### 0. Prepare the input WAV and optional filter matrix

```bash
cd pythondemo
python demo.py --prepare-wav /path/to/source.wav
python demo.py --export-filter-matrix filter_matrix.txt
```

The first command writes a 16-bit mono 48 kHz file to `../spiffs_data/input.wav` by default. The second writes the full 32-band coefficient list plus a matrix-form block that is easy to paste into MATLAB, Python, or C arrays. After updating `filter_matrix.txt`, regenerate `USFMH/main/filter_bank_coeffs.h` with `python pythondemo/gens/generate_filter_header.py` before rebuilding the firmware.

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
cd pythondemo
pip install -r requirements.txt
python demo.py --port /dev/ttyACM0 --baud 921600
```

The script will:
1. Handshake with the ESP32 (`READY`)
2. Play tones and prompt you to respond `y`/`n`
3. Receive and play back the FIR-processed audio

# List of Softwares/Hardwares Used
![Espressif](https://img.shields.io/badge/espressif-E7352C.svg?style=for-the-badge&logo=espressif&logoColor=white)
![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white)
![Drawio](https://img.shields.io/badge/drawio-%23F08705?style=for-the-badge&logo=diagrams.net&logoColor=white)
![KiCad](https://img.shields.io/badge/kicad-%23314CB0.svg?style=for-the-badge&logo=kicad&logoColor=white)
![Google Drive](https://img.shields.io/badge/Google%20Drive-4285F4?style=for-the-badge&logo=googledrive&logoColor=white)

AI Help from:
![Claude](https://img.shields.io/badge/Claude-D97757?style=for-the-badge&logo=claude&logoColor=white)
![ChatGPT](https://img.shields.io/badge/chatGPT-74aa9c?style=for-the-badge&logo=openai&logoColor=white)
![Google Gemini](https://img.shields.io/badge/google%20gemini-8E75B2?style=for-the-badge&logo=google%20gemini&logoColor=white)

## Serial Protocol

| Direction | Message | Description |
|-----------|---------|-------------|
| Host → ESP32 | `READY\n` | Handshake to start the test |
| Host → ESP32 | `READY SAMPLE <N>\n` | Debug mode: skip hearing test, mark only band `N` as heard |
| Host → ESP32 | `READY SAMPLE ALL\n` | Debug mode: skip hearing test, treat all bands as heard |
| ESP32 → Host | `TONE <freq_hz> <duration_ms>\n` | Request to play a test tone |
| Host → ESP32 | `y\n` or `n\n` | User heard / didn't hear the tone |
| ESP32 → Host | `AUDIO <sample_rate> <num_samples>\n` | Header before streaming PCM data |
| ESP32 → Host | *raw int16 PCM bytes* | Processed audio payload |
| ESP32 → Host | `DONE\n` | Processing complete |

## Build & runtime troubleshooting

- **ESP-IDF version**: built and tested with ESP-IDF v5.x. Earlier versions are unlikely to work because the FFT path uses `esp_dsp` from `dependencies.lock`.
- **USB CDC port permissions (Linux)**: if `/dev/ttyACM0` shows up but you get `Permission denied`, add yourself to the `dialout` group (or `uucp` on some distros) and re-login.
- **DTR/RTS reset doesn't fire**: the USB Serial-JTAG bridge on the ESP32-S3 doesn't always honor DTR/RTS. The host script will fall back to asking you to press the **RST** button — that's expected.
- **SD card formatting**: must be **FAT32**. Copy `filters.bin` (generated by `pythondemo/gens/generate_filter_bin.py`) and `input.wav` (16-bit mono 48 kHz, generated by `python pythondemo/demo.py --prepare-wav <source>`) to the card root before booting.
- **Sample rate**: the firmware enforces 48 kHz input as of [USFMH/main/audio_processor.c](USFMH/main/audio_processor.c). Anything else fails fast with `ESP_ERR_INVALID_ARG`.
- **PSRAM allocation failures**: the FFT workbuffers need ~256 KB of PSRAM. If allocation fails, confirm `CONFIG_SPIRAM_*` is set in [USFMH/sdkconfig.defaults](USFMH/sdkconfig.defaults) and that the board actually has octal PSRAM.

## Handoff notes (for whoever picks this up next)

Known limitations / reasonable next steps:
- **Sample rate is locked to 48 kHz** — the FFT bin layout is hardcoded around it. Generalizing requires re-deriving `FFT_N` for the lowest band.
- **Batch processing only** — the firmware reads the full WAV, processes it, then streams it back. A streaming/real-time variant would require restructuring [audio_processor.c](USFMH/main/audio_processor.c) around partial blocks.
- **Mono only** — the WAV utils and FFT path assume one channel.
- **Filter regeneration is two-step** — `MATLAB/matrix.txt` → `filters.bin` is decoupled from the firmware build. A `idf.py` build hook could close that loop.
- **OLED UI is unfinished** — there was an earlier `runoled.sh` / `oled_test/` experiment that was removed during cleanup. A small status display would be a natural addition for a standalone (non-laptop) build.
- **No automated tests** — the Python helpers in `pythondemo/demo.py` are good candidates for unit coverage of `fft_band_weight`, `design_bandpass_fir`, and `prepare_wav_for_spiffs`.

