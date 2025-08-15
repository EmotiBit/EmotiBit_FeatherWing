#!/bin/bash

# Script to build EmotiBit FeatherWing main firmware using PlatformIO
# Usage: ./build.sh

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if PlatformIO is installed
if ! command -v pio &> /dev/null; then
    echo "Error: PlatformIO is not installed."
    echo "Install with: pip install platformio"
    exit 1
fi

echo "Building EmotiBit stock firmware..."
cd "$SCRIPT_DIR/EmotiBit_stock_firmware"
pio run

echo "Building EmotiBit stock firmware PPG 100Hz..."
cd "$SCRIPT_DIR/EmotiBit_stock_firmware_PPG_100Hz"
pio run

echo "All builds completed!"