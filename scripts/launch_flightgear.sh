#!/bin/bash
#
# VALOR - FlightGear Launch Script
# Launches FlightGear configured to receive FGNetFDM data from the simulation.
#
# Usage:
#   ./launch_flightgear.sh              # Launch with defaults
#   ./launch_flightgear.sh --external   # For external visualization
#

set -e

# Configuration
FG_PORT="${FG_PORT:-5501}"
FG_RATE="${FG_RATE:-50}"
AIRCRAFT="${FG_AIRCRAFT:-c172p}"

# KSFO Runway 28R approach position
# 3nm back from threshold on extended centerline
AIRPORT="KSFO"
RUNWAY="28R"
RUNWAY_HDG=280

# Threshold position
THR_LAT=37.6213
THR_LON=-122.3590

# Calculate initial position (3nm back on centerline)
DIST_NM=3.0
DIST_M=$(echo "$DIST_NM * 1852" | bc -l)

# Python for position calculation
INIT_POS=$(python3 << EOF
import math

thr_lat = $THR_LAT
thr_lon = $THR_LON
rwy_hdg = $RUNWAY_HDG
dist_m = $DIST_M
gs_deg = 3.0

# Earth radius
R = 6371000

# Convert to radians
lat_rad = math.radians(thr_lat)
lon_rad = math.radians(thr_lon)
back_bearing = math.radians((rwy_hdg + 180) % 360)

# Calculate new position
lat2 = math.asin(
    math.sin(lat_rad) * math.cos(dist_m / R) +
    math.cos(lat_rad) * math.sin(dist_m / R) * math.cos(back_bearing)
)
lon2 = lon_rad + math.atan2(
    math.sin(back_bearing) * math.sin(dist_m / R) * math.cos(lat_rad),
    math.cos(dist_m / R) - math.sin(lat_rad) * math.sin(lat2)
)

# Altitude on glideslope (3 deg)
alt_m = dist_m * math.tan(math.radians(gs_deg))
alt_ft = alt_m * 3.28084 + 13  # Add field elevation

print(f"{math.degrees(lat2):.6f} {math.degrees(lon2):.6f} {alt_ft:.0f}")
EOF
)

read INIT_LAT INIT_LON INIT_ALT <<< "$INIT_POS"

echo "=============================================="
echo "VALOR - FlightGear Visualization"
echo "=============================================="
echo "Airport: $AIRPORT Runway $RUNWAY"
echo "Initial position: ${INIT_LAT}°N, ${INIT_LON}°W"
echo "Initial altitude: ${INIT_ALT} ft MSL"
echo "Heading: ${RUNWAY_HDG}°"
echo ""
echo "Receiving FGNetFDM on port: $FG_PORT"
echo "=============================================="

# Find FlightGear executable
find_fgfs() {
    # Check environment variable
    if [ -n "$FGFS_BIN" ] && [ -x "$FGFS_BIN" ]; then
        echo "$FGFS_BIN"
        return 0
    fi

    # Check PATH
    if command -v fgfs &> /dev/null; then
        command -v fgfs
        return 0
    fi

    # macOS application bundles - check for both "fgfs" and "FlightGear" executables
    local mac_paths=(
        "/Applications/FlightGear.app/Contents/MacOS/FlightGear"
        "/Applications/FlightGear.app/Contents/MacOS/fgfs"
        "/Applications/FlightGear 2020.3.app/Contents/MacOS/fgfs"
        "/Applications/FlightGear-2020.3.app/Contents/MacOS/fgfs"
        "$HOME/Applications/FlightGear.app/Contents/MacOS/FlightGear"
        "$HOME/Applications/FlightGear.app/Contents/MacOS/fgfs"
    )

    for path in "${mac_paths[@]}"; do
        if [ -x "$path" ] && [ -f "$path" ]; then
            echo "$path"
            return 0
        fi
    done

    # Try mdfind on macOS - look specifically for FlightGear in app bundle
    if command -v mdfind &> /dev/null; then
        local found
        found=$(mdfind "kMDItemKind == 'Application'" 2>/dev/null | grep -i flightgear | head -n1)
        if [ -n "$found" ]; then
            # Check for executable inside the .app
            for exe_name in "FlightGear" "fgfs"; do
                local exe_path="$found/Contents/MacOS/$exe_name"
                if [ -x "$exe_path" ] && [ -f "$exe_path" ]; then
                    echo "$exe_path"
                    return 0
                fi
            done
        fi
    fi

    return 1
}

FGFS=$(find_fgfs) || {
    echo "ERROR: FlightGear executable not found!"
    echo ""
    echo "Please install FlightGear or set FGFS_BIN environment variable:"
    echo "  export FGFS_BIN=/path/to/fgfs"
    echo ""
    echo "On macOS, install from: https://www.flightgear.org/download/"
    exit 1
}

echo "Using FlightGear: $FGFS"
echo ""

# Build FlightGear command
FG_ARGS=(
    "--aircraft=$AIRCRAFT"
    "--airport=$AIRPORT"
    "--runway=$RUNWAY"
    "--lat=$INIT_LAT"
    "--lon=$INIT_LON"
    "--altitude=$INIT_ALT"
    "--heading=$RUNWAY_HDG"
    "--vc=65"
    "--glideslope=3"
    "--native-fdm=socket,in,$FG_RATE,,${FG_PORT},udp"
    "--fdm=external"
    "--disable-ai-traffic"
    "--disable-ai-models"
    "--disable-random-objects"
    "--disable-terrasync"
    "--fog-fastest"
    "--visibility=20000"
    "--timeofday=noon"
    "--prop:/sim/rendering/multi-sample-buffers=true"
    "--prop:/sim/rendering/multi-samples=4"
)

# Add prop for better visual
FG_ARGS+=(
    "--prop:/sim/current-view/view-number=0"
    "--prop:/sim/current-view/field-of-view=60"
)

echo "Starting FlightGear..."
echo "(Press Ctrl+C to stop)"
echo ""

# Run FlightGear
exec "$FGFS" "${FG_ARGS[@]}"
