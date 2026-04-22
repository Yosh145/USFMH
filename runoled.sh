#!/usr/bin/env fish
#
# OLED Test — Build, flash & monitor
#
# Usage:  ./runoled.sh [--port /dev/ttyACM0] [--skip-build] [-text "Hello World"]
#

set SCRIPT_DIR (cd (dirname (status filename)); and pwd)
set PROJECT_DIR "$SCRIPT_DIR/oled_test"

# Defaults
set PORT ""
set SKIP_BUILD false
set TICKER_TEXT ""

# Parse arguments
set i 1
while test $i -le (count $argv)
    switch $argv[$i]
        case --port
            set i (math $i + 1)
            set PORT $argv[$i]
        case --skip-build
            set SKIP_BUILD true
        case -text
            set i (math $i + 1)
            if test $i -gt (count $argv)
                echo "ERROR: -text requires a string argument"
                exit 1
            end
            set TICKER_TEXT $argv[$i]
        case -h --help
            echo "Usage: ./runoled.sh [--port PORT] [--skip-build] [-text \"your message\"]"
            echo ""
            echo "  --port PORT          Serial port (auto-detected if not specified)"
            echo "  --skip-build         Skip firmware build step"
            echo "  -text \"string\"       Scroll a news ticker with the given text"
            echo ""
            echo "Without -text, displays \"sup\" and a smiley face."
            exit 0
        case '*'
            echo "Unknown option: $argv[$i]"
            exit 1
    end
    set i (math $i + 1)
end

# ── Source ESP-IDF environment ──
echo "━━━ Sourcing ESP-IDF environment ━━━"
source "$HOME/esp/esp-idf/export.fish"

# ── Auto-detect serial port ──
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

# ── Build ──
if test "$SKIP_BUILD" = false
    echo ""
    echo "━━━ Building OLED test firmware ━━━"
    cd "$PROJECT_DIR"

    # Pass ticker text as a CMake define if provided
    if test -n "$TICKER_TEXT"
        echo "  Ticker text: \"$TICKER_TEXT\""
        idf.py -DTICKER_TEXT="$TICKER_TEXT" build; or exit 1
    else
        idf.py build; or exit 1
    end
    echo "  ✓ Build complete"
else
    echo ""
    echo "━━━ Build skipped ━━━"
end

# ── Flash & Monitor ──
echo ""
echo "━━━ Flashing & monitoring ━━━"
cd "$PROJECT_DIR"
idf.py -p "$PORT" flash monitor; or exit 1
