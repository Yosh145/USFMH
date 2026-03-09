#!/usr/bin/env fish
#
# USFMH One-Click Build, Flash & Run
#
# Usage:  ./run.sh [--port /dev/ttyACM0] [--skip-build] [--skip-flash-spiffs]
#

set SCRIPT_DIR (cd (dirname (status filename)); and pwd)
set HOST_DIR "$SCRIPT_DIR/host"
set SPIFFS_DIR "$SCRIPT_DIR/spiffs_data"
set SPIFFS_BIN "$SCRIPT_DIR/spiffs.bin"
set SPIFFS_SIZE 2293760
set SPIFFS_OFFSET 0x1d0000

# Defaults
set PORT ""
set SKIP_BUILD false
set SKIP_SPIFFS false

# Parse arguments
set i 1
while test $i -le (count $argv)
    switch $argv[$i]
        case --port
            set i (math $i + 1)
            set PORT $argv[$i]
        case --skip-build
            set SKIP_BUILD true
        case --skip-flash-spiffs
            set SKIP_SPIFFS true
        case -h --help
            echo "Usage: ./run.sh [--port PORT] [--skip-build] [--skip-flash-spiffs]"
            echo ""
            echo "  --port PORT            Serial port (auto-detected if not specified)"
            echo "  --skip-build           Skip firmware build step"
            echo "  --skip-flash-spiffs    Skip SPIFFS flash (WAV file already on device)"
            exit 0
        case '*'
            echo "Unknown option: $argv[$i]"
            exit 1
    end
    set i (math $i + 1)
end

# ── Source ESP-IDF environment ──────────────────────────────────────────────
echo "━━━ Sourcing ESP-IDF environment ━━━"
source "$HOME/esp/esp-idf/export.fish"
echo "  IDF_PATH: $IDF_PATH"

# ── Auto-detect serial port ────────────────────────────────────────────────
if test -z "$PORT"
    for p in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1
        if test -e "$p"
            set PORT "$p"
            break
        end
    end
    if test -z "$PORT"
        echo "ERROR: No serial port found. Connect your ESP32-S3 and retry."
        exit 1
    end
end
echo "  Using port: $PORT"

# ── Step 1: Build firmware ──────────────────────────────────────────────────
if test "$SKIP_BUILD" = false
    echo ""
    echo "━━━ Step 1: Building firmware ━━━"
    cd "$SCRIPT_DIR"
    idf.py build; or exit 1
    echo "  ✓ Build complete"
else
    echo ""
    echo "━━━ Step 1: Build skipped ━━━"
end

# ── Step 2: Flash firmware ──────────────────────────────────────────────────
echo ""
echo "━━━ Step 2: Flashing firmware ━━━"
cd "$SCRIPT_DIR"
idf.py -p "$PORT" flash; or exit 1
echo "  ✓ Firmware flashed"

# ── Step 3: Flash SPIFFS (if needed) ────────────────────────────────────────
if test "$SKIP_SPIFFS" = false
    echo ""
    echo "━━━ Step 3: Flashing SPIFFS (WAV file) ━━━"

    if not test -f "$SPIFFS_BIN"
        if not test -d "$SPIFFS_DIR"; or not test -f "$SPIFFS_DIR/input.wav"
            echo "ERROR: No SPIFFS image or WAV file found."
            echo "  Place a 16-bit mono WAV at: $SPIFFS_DIR/input.wav"
            echo "  Then re-run this script."
            exit 1
        end
        echo "  Generating SPIFFS image..."
        python "$IDF_PATH/components/spiffs/spiffsgen.py" "$SPIFFS_SIZE" "$SPIFFS_DIR" "$SPIFFS_BIN"; or exit 1
    end

    python -m esptool --chip esp32s3 --port "$PORT" --baud 460800 \
        write_flash "$SPIFFS_OFFSET" "$SPIFFS_BIN"; or exit 1
    echo "  ✓ SPIFFS flashed"
else
    echo ""
    echo "━━━ Step 3: SPIFFS flash skipped ━━━"
end

# ── Step 4: Install Python dependencies ─────────────────────────────────────
echo ""
echo "━━━ Step 4: Checking Python dependencies ━━━"
/usr/bin/python3 -m pip install -q -r "$HOST_DIR/requirements.txt" 2>/dev/null
or /usr/bin/python3 -m pip install --user -q -r "$HOST_DIR/requirements.txt"
echo "  ✓ Dependencies ready"

# ── Step 5: Run the hearing test ─────────────────────────────────────────────
echo ""
echo "━━━ Step 5: Starting hearing test ━━━"
echo "  The ESP32 will reboot and the test will begin."
echo "  You will hear tones — respond y/n for each."
echo ""

cd "$HOST_DIR"
/usr/bin/python3 demo.py --port "$PORT"
