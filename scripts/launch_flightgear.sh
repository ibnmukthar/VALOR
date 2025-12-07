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

# Read configuration from config.json to match simulation
CONFIG_FILE="${CONFIG_FILE:-data/config.json}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Python script to read config and compute initial position
INIT_POS=$(python3 << EOF
import json
import math
import os

config_path = os.path.join("$PROJECT_DIR", "$CONFIG_FILE")
with open(config_path, 'r') as f:
    config = json.load(f)

airport = config["airport"]
sim = config["simulation"]

thr_lat = airport["threshold_lat_deg"]
thr_lon = airport["threshold_lon_deg"]
rwy_hdg = airport["runway_heading_deg"]
field_elev = airport["elevation_ft"]

dist_nm = sim["initial_distance_nm"]
dist_m = dist_nm * 1852
gs_deg = sim["glideslope_deg"]

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

# Altitude on glideslope
alt_m = dist_m * math.tan(math.radians(gs_deg))
alt_ft = alt_m * 3.28084 + field_elev

# Output: lat lon alt airport runway heading
print(f"{math.degrees(lat2):.6f} {math.degrees(lon2):.6f} {alt_ft:.0f} {airport['icao']} {airport['runway_id']} {int(rwy_hdg)}")
EOF
)

read INIT_LAT INIT_LON INIT_ALT AIRPORT RUNWAY RUNWAY_HDG <<< "$INIT_POS"

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
