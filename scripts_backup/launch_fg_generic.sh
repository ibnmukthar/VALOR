#!/usr/bin/env bash
set -euo pipefail

# Launch FlightGear to visualize VALOR via native FDM (FGNetFDM) UDP input.
# Usage: scripts/launch_fg_generic.sh [port] [rate_hz]
# Defaults: port=5501, rate=60

PORT="${1:-5501}"
RATE="${2:-60}"

ROOT_DIR="$(cd "$(dirname "$0")"/.. && pwd)"

# We use FlightGear's native FDM binary protocol:
#   --fdm=null --native-fdm=socket,in,<rate>,127.0.0.1,<port>,udp
# JSBSim sends FGNetFDM packets to the same host/port from scripts/fg_sender.py.

# Initial viewpoint: start FlightGear where JSBSim puts the aircraft
# (2 NM back from threshold along extended centerline, on glideslope)
# KSFO Runway 28R: threshold at 37.6213, -122.359; runway heading 280 deg.
# 2 NM = 3704 m; heading 280 -> aircraft starts ESE of threshold.
# At lat 37.62: 1 deg lat ~ 111000 m, 1 deg lon ~ 88000 m
# dN = -3704 * cos(280) = -3704 * 0.1736 = -643 m = -0.0058 deg lat
# dE = -3704 * sin(280) = -3704 * (-0.9848) = +3648 m = +0.0414 deg lon
LAT=37.6155     # threshold (37.6213) - 0.0058
LON=-122.3176   # threshold (-122.359) + 0.0414
ALT_FT=650      # on 3 deg glideslope at 2 NM (~636 ft AGL + 13 ft elev)
HDG=280

# Resolve FlightGear executable
# Priority:
#  1. FGFS_BIN env var if provided
#  2. fgfs found in PATH
#  3. Standard macOS app bundle (FlightGear*.app) under /Applications or ~/Applications
#  4. mdfind (Spotlight) search as last resort

VERBOSE=${VERBOSE:-0}
log() { [ "$VERBOSE" = 1 ] && echo "[launch_fg] $*" >&2; }

FGFS_BIN="${FGFS_BIN:-}"
FGFS_APP="${FGFS_APP:-}"
if [ -n "$FGFS_BIN" ]; then
  log "Using FGFS_BIN env override: $FGFS_BIN"
elif [ -n "$FGFS_APP" ]; then
  # Allow user to specify the .app bundle directly
  CAND_DIR="$FGFS_APP/Contents/MacOS"
  if [ -d "$CAND_DIR" ]; then
    # Preferred order: fgfs, FlightGear, any executable containing fgfs substring
    for n in fgfs FlightGear flightgear; do
      if [ -x "$CAND_DIR/$n" ]; then
        FGFS_BIN="$CAND_DIR/$n"
        log "Resolved from FGFS_APP: $FGFS_BIN"
        break
      fi
    done
    if [ -z "$FGFS_BIN" ]; then
      # fallback: pick first executable file in directory
      while IFS= read -r f; do
        [ -x "$f" ] || continue
        FGFS_BIN="$f"; log "Using first executable in bundle: $FGFS_BIN"; break
      done < <(find "$CAND_DIR" -maxdepth 1 -type f 2>/dev/null)
    fi
  fi
elif command -v fgfs >/dev/null 2>&1; then
  FGFS_BIN="$(command -v fgfs)"
  log "Found fgfs on PATH: $FGFS_BIN"
else
  CANDIDATES=()
  # Common app bundle names (versioned or not)
  for base in /Applications ~/Applications; do
    for app in "$base"/FlightGear*.app; do
      [ -d "$app" ] || continue
      CANDIDATES+=("$app/Contents/MacOS/fgfs")
    done
  done
  # Deduplicate
  uniq_candidates=()
  for c in "${CANDIDATES[@]:-}"; do
    skip=0
    for u in "${uniq_candidates[@]:-}"; do [ "$c" = "$u" ] && skip=1 && break; done
    [ $skip -eq 0 ] && uniq_candidates+=("$c")
  done
  for c in "${uniq_candidates[@]:-}"; do
    if [ -e "$c" ]; then
      perm_info="$(ls -ld "$c" 2>/dev/null | awk '{print $1" "$3":"$4}')"
      log "Checking candidate: $c (exists perm=$perm_info)"
      if [ -x "$c" ]; then
        FGFS_BIN="$c"
        break
      else
        log "Candidate not executable: $c"
      fi
    else
      log "Candidate path does not exist: $c"
    fi
  done
  # If fgfs not executable but a 'FlightGear' binary exists, try that
  if [ -z "$FGFS_BIN" ]; then
    for base in /Applications ~/Applications; do
      for app in "$base"/FlightGear*.app; do
        [ -d "$app" ] || continue
        alt="$app/Contents/MacOS/FlightGear"
        if [ -x "$alt" ]; then
          log "Using alternate binary name: $alt"
          FGFS_BIN="$alt"
          break
        fi
      done
      [ -n "$FGFS_BIN" ] && break
    done
  fi
  if [ -z "$FGFS_BIN" ] && command -v mdfind >/dev/null 2>&1; then
    log "Attempting Spotlight search via mdfind"
    SPOT=$(mdfind "kMDItemCFBundleIdentifier == 'org.flightgear.FlightGear'" | head -n1)
    if [ -n "$SPOT" ] && [ -x "$SPOT/Contents/MacOS/fgfs" ]; then
      FGFS_BIN="$SPOT/Contents/MacOS/fgfs"
      log "Located via mdfind: $FGFS_BIN"
    fi
  fi
  # As last resort, try any plausible executable inside found bundle(s)
  if [ -z "$FGFS_BIN" ]; then
    for app in /Applications/FlightGear*.app ~/Applications/FlightGear*.app; do
      [ -d "$app/Contents/MacOS" ] || continue
      for n in fgfs FlightGear flightgear; do
        if [ -x "$app/Contents/MacOS/$n" ]; then
          FGFS_BIN="$app/Contents/MacOS/$n"; log "Fallback picked: $FGFS_BIN"; break
        fi
      done
      [ -n "$FGFS_BIN" ] && break
    done
  fi
fi

if [ -z "$FGFS_BIN" ]; then
  echo "Error: Could not locate FlightGear executable (fgfs)." >&2
  echo "Troubleshooting steps:" >&2
  echo "  1. Ensure FlightGear.app is in /Applications (or set FGFS_BIN)." >&2
  echo "  2. You can run: ls -1 /Applications | grep -i FlightGear" >&2
  echo "  3. Or launch once manually so Spotlight indexes it." >&2
  echo "  4. Then re-run with: VERBOSE=1 bash scripts/launch_fg_generic.sh" >&2
  if [ "$VERBOSE" = 1 ]; then
    for app in "/Applications/FlightGear.app" /Applications/FlightGear*.app ~/Applications/FlightGear*.app "$FGFS_APP"; do
      [ -d "$app/Contents/MacOS" ] || continue
      echo "[launch_fg] Listing $app/Contents/MacOS:" >&2
      ls -l "$app/Contents/MacOS" >&2 || true
    done
  fi
  exit 127
fi

echo "Using FlightGear binary: $FGFS_BIN" >&2

# Optional diagnostics:
#   DEBUG=1      -> bash xtrace
#   DRY_RUN=1    -> print command and exit
#   LOG_STDERR=1 -> tee stderr to results/logs/flightgear_stderr.log if directory exists
DEBUG=${DEBUG:-0}
DRY_RUN=${DRY_RUN:-0}
LOG_STDERR=${LOG_STDERR:-0}

[ "$DEBUG" = 1 ] && set -x

# Default to C172P for realistic visualization, but allow override via
# FG_AIRCRAFT for simpler external-FDM-friendly models (e.g. ufo).
AIRCRAFT="${FG_AIRCRAFT:-ufo}"

# Start at the airport (not explicit lat/lon) to ensure FG loads proper scenery.
CMD=("$FGFS_BIN"
  --aircraft="$AIRCRAFT"
  --fdm=null
  --native-fdm=socket,in,$RATE,127.0.0.1,$PORT,udp
  --airport=KSFO
  --runway=28R
  --offset-distance=2
  --offset-azimuth=180
  --altitude="$ALT_FT"
  --timeofday=noon
  --geometry=1280x720
  --disable-ai-traffic
  --prop:/sim/current-view/view-number=1
  --prop:/consumables/fuel/tank[0]/level-gal_us=20
  --prop:/consumables/fuel/tank[1]/level-gal_us=20
  --prop:/consumables/fuel/tank[0]/selected=true
  --prop:/consumables/fuel/tank[1]/selected=true
  --enable-fuel-freeze
)

if [ "$DRY_RUN" = 1 ]; then
  printf 'DRY_RUN FlightGear command:\n'
  printf '%q ' "${CMD[@]}"; echo
  exit 0
fi

if [ "$LOG_STDERR" = 1 ]; then
  LOG_DIR="$ROOT_DIR/results/logs"
  mkdir -p "$LOG_DIR"
  LOG_FILE="$LOG_DIR/flightgear_stderr.log"
  echo "Logging stderr to $LOG_FILE" >&2
  exec "${CMD[@]}" 2> >(tee "$LOG_FILE" >&2)
else
  exec "${CMD[@]}"
fi

