#!/bin/bash
#
# VALOR - Record Simulation Script
# Records FlightGear visualization during VALOR autoland simulation
#
# Usage:
#   ./scripts/record_simulation.sh [scenario]
#   Default scenario: moderate
#

set -e

SCENARIO="${1:-moderate}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
VIDEO_FILE="results/videos/valor_${SCENARIO}_${TIMESTAMP}.mp4"

# Create videos directory
mkdir -p results/videos

echo "========================================"
echo "VALOR - Automated Recording"
echo "========================================"
echo "Scenario: $SCENARIO"
echo "Video will be saved to: $VIDEO_FILE"
echo ""
echo "IMPORTANT: macOS Screen Recording Permissions Required"
echo "If recording fails, grant screen recording permission:"
echo "  System Settings > Privacy & Security > Screen Recording"
echo "  Enable: Terminal (or iTerm2)"
echo "========================================"
echo ""

# Check if ffmpeg is available
if ! command -v ffmpeg &> /dev/null; then
    echo "Error: ffmpeg not found. Install with: brew install ffmpeg"
    exit 1
fi

# Start screen recording in background
# Recording display 1 (main screen) with no audio
echo "Starting screen recording..."
ffmpeg -f avfoundation -framerate 30 -i "1:none" -t 180 -vcodec libx264 -preset fast -crf 23 "$VIDEO_FILE" > results/videos/ffmpeg_output.log 2>&1 &
FFMPEG_PID=$!
echo "Screen recording started (PID: $FFMPEG_PID)"
sleep 3

# Start FlightGear
echo "Starting FlightGear..."
"/Applications/FlightGear.app/Contents/MacOS/FlightGear" \
    --aircraft=c172p \
    --airport=KSFO \
    --runway=28R \
    --lat=37.612607 \
    --lon=-122.296882 \
    --heading=280 \
    --altitude=968 \
    --generic=socket,in,50,localhost,5501,udp,valor_generic \
    --disable-random-objects \
    --disable-random-vegetation \
    --disable-clouds \
    --timeofday=noon \
    --visibility=15000 \
    --geometry=1920x1080 \
    > /dev/null 2>&1 &
FG_PID=$!
echo "FlightGear started (PID: $FG_PID)"

# Wait for FlightGear to initialize
echo "Waiting 15 seconds for FlightGear to initialize..."
sleep 15

# Run simulation
echo "Starting VALOR simulation..."
.venv/bin/python scripts/main.py --scenario "$SCENARIO"
SIMULATION_EXIT=$?

# Wait a few seconds after simulation completes
echo "Simulation complete. Capturing final 10 seconds..."
sleep 10

# Stop recording
echo "Stopping screen recording..."
kill -INT $FFMPEG_PID 2>/dev/null || true
sleep 3

# Stop FlightGear
echo "Stopping FlightGear..."
kill $FG_PID 2>/dev/null || true

echo ""
echo "========================================"
echo "Recording Complete!"
echo "========================================"

# Check if video was created
if [ -f "$VIDEO_FILE" ]; then
    VIDEO_SIZE=$(ls -lh "$VIDEO_FILE" | awk '{print $5}')
    echo "Video saved: $VIDEO_FILE"
    echo "File size: $VIDEO_SIZE"
    echo ""
    echo "To view: open $VIDEO_FILE"
else
    echo "WARNING: Video file not created!"
    echo "Check ffmpeg log: results/videos/ffmpeg_output.log"
    echo ""
    echo "If screen recording permission was denied:"
    echo "  1. Go to System Settings > Privacy & Security > Screen Recording"
    echo "  2. Enable your terminal application"
    echo "  3. Restart terminal and try again"
fi

exit $SIMULATION_EXIT
