"""
VALOR - Wind Environment Model
Implements realistic wind profiles with shear, turbulence, and microburst effects.

Includes wind shear detection for go-around triggering.
"""

import math
import json
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
from enum import Enum

from simulation import FT_TO_M, M_TO_FT, KTS_TO_MPS, DEG_TO_RAD


@dataclass
class WindComponents:
    """Wind vector in NED frame (m/s)."""
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0

    @property
    def speed_mps(self) -> float:
        return math.sqrt(self.north**2 + self.east**2)

    @property
    def speed_kts(self) -> float:
        return self.speed_mps / KTS_TO_MPS

    @property
    def direction_deg(self) -> float:
        """Wind direction (where it's coming FROM)."""
        return (math.atan2(-self.east, -self.north) * 180 / math.pi) % 360


class WindShearType(Enum):
    """Types of wind shear conditions."""
    NONE = "none"
    HEADWIND_LOSS = "headwind_loss"
    TAILWIND_GAIN = "tailwind_gain"
    DOWNDRAFT = "downdraft"
    MICROBURST = "microburst"


@dataclass
class WindShearAlert:
    """Wind shear detection alert."""
    detected: bool = False
    shear_type: WindShearType = WindShearType.NONE
    severity: float = 0.0  # 0-1, 1 = severe
    airspeed_change_kts: float = 0.0
    vertical_speed_change_fpm: float = 0.0


class DrydenTurbulence:
    """
    Dryden turbulence model (MIL-F-8785C).

    Generates continuous turbulence using colored noise filters.
    """

    def __init__(self, sigma_u: float = 2.0, sigma_v: float = 2.0, sigma_w: float = 1.5,
                 scale_length: float = 200.0, seed: Optional[int] = None):
        """
        Initialize Dryden turbulence model.

        Args:
            sigma_u, sigma_v, sigma_w: RMS intensities (m/s)
            scale_length: Turbulence scale length (m)
            seed: Random seed for reproducibility
        """
        self.sigma_u = sigma_u
        self.sigma_v = sigma_v
        self.sigma_w = sigma_w
        self.L = scale_length

        self.rng = np.random.default_rng(seed)

        # Filter states
        self.u_state = 0.0
        self.v_state = 0.0
        self.w_state1 = 0.0
        self.w_state2 = 0.0

    def get_turbulence(self, airspeed_mps: float, altitude_agl_m: float,
                       dt: float) -> Tuple[float, float, float]:
        """
        Get turbulence components at current state.

        Args:
            airspeed_mps: True airspeed in m/s
            altitude_agl_m: Altitude AGL in meters
            dt: Time step in seconds

        Returns:
            Tuple of (u_turb, v_turb, w_turb) in m/s (body frame)
        """
        if airspeed_mps < 1.0 or dt <= 0:
            return 0.0, 0.0, 0.0

        # Altitude-dependent scale lengths (MIL-F-8785C)
        h = max(10.0, altitude_agl_m)
        Lu = Lv = self.L
        Lw = max(10.0, h / 2)

        # Time constants
        tau_u = Lu / airspeed_mps
        tau_v = Lv / airspeed_mps
        tau_w = Lw / airspeed_mps

        # White noise inputs
        nu = self.rng.standard_normal()
        nv = self.rng.standard_normal()
        nw = self.rng.standard_normal()

        # First-order filter for u and v (Markov process)
        # x(k+1) = a*x(k) + b*n(k)
        # where a = exp(-dt/tau), b = sigma*sqrt(1-a^2)

        # U component
        a_u = math.exp(-dt / tau_u)
        b_u = self.sigma_u * math.sqrt(1 - a_u**2)
        self.u_state = a_u * self.u_state + b_u * nu

        # V component
        a_v = math.exp(-dt / tau_v)
        b_v = self.sigma_v * math.sqrt(1 - a_v**2)
        self.v_state = a_v * self.v_state + b_v * nv

        # W component (second-order filter for more realistic response)
        # Simplified: use first-order with altitude-dependent scaling
        a_w = math.exp(-dt / tau_w)
        # Reduce vertical turbulence near ground
        ground_factor = min(1.0, h / 100.0)
        b_w = self.sigma_w * ground_factor * math.sqrt(1 - a_w**2)
        self.w_state1 = a_w * self.w_state1 + b_w * nw

        return self.u_state, self.v_state, self.w_state1

    def reset(self, seed: Optional[int] = None):
        """Reset filter states and optionally reseed."""
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.u_state = 0.0
        self.v_state = 0.0
        self.w_state1 = 0.0
        self.w_state2 = 0.0


class LogarithmicWindShear:
    """
    Logarithmic wind profile for boundary layer shear.

    U(z) = U_ref * ln(z/z0) / ln(z_ref/z0)
    """

    def __init__(self, reference_speed_mps: float, reference_height_m: float = 10.0,
                 roughness_length_m: float = 0.03, wind_direction_deg: float = 0.0):
        """
        Initialize logarithmic wind shear model.

        Args:
            reference_speed_mps: Wind speed at reference height
            reference_height_m: Reference height (typically 10m)
            roughness_length_m: Surface roughness length
            wind_direction_deg: Wind direction (where FROM)
        """
        self.u_ref = reference_speed_mps
        self.z_ref = reference_height_m
        self.z0 = roughness_length_m
        self.direction_rad = wind_direction_deg * DEG_TO_RAD

    def get_wind_speed(self, altitude_m: float) -> float:
        """Get wind speed at given altitude."""
        z = max(self.z0 + 0.1, altitude_m)
        return self.u_ref * math.log(z / self.z0) / math.log(self.z_ref / self.z0)

    def get_wind_ned(self, altitude_m: float) -> Tuple[float, float, float]:
        """Get wind components in NED frame."""
        speed = self.get_wind_speed(altitude_m)

        # Wind direction is where it comes FROM
        # So north wind blows TO the south
        north = -speed * math.cos(self.direction_rad)
        east = -speed * math.sin(self.direction_rad)

        return north, east, 0.0


class MicroburstModel:
    """
    Simplified microburst model.

    Models a cylindrical downdraft with horizontal outflow near ground.
    """

    def __init__(self, center_x: float = 0.0, center_y: float = 0.0,
                 radius_m: float = 500.0, max_downdraft_mps: float = 10.0,
                 outflow_height_m: float = 100.0):
        """
        Initialize microburst model.

        Args:
            center_x, center_y: Center position relative to runway threshold (m)
            radius_m: Radius of the microburst
            max_downdraft_mps: Maximum downdraft speed
            outflow_height_m: Height where outflow begins
        """
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius_m
        self.max_downdraft = max_downdraft_mps
        self.outflow_height = outflow_height_m

    def get_wind_ned(self, x: float, y: float, altitude_m: float
                     ) -> Tuple[float, float, float]:
        """
        Get microburst wind components.

        Args:
            x, y: Position relative to runway threshold (m)
            altitude_m: Altitude AGL

        Returns:
            Wind components (north, east, down) in m/s
        """
        # Distance from microburst center
        dx = x - self.center_x
        dy = y - self.center_y
        r = math.sqrt(dx**2 + dy**2)

        if r > self.radius * 2:
            return 0.0, 0.0, 0.0

        # Radial factor (bell-shaped profile)
        radial_factor = math.exp(-(r / self.radius)**2)

        if altitude_m > self.outflow_height:
            # Above outflow: primarily downdraft
            altitude_factor = 1.0
            w_down = self.max_downdraft * radial_factor * altitude_factor

            # Slight horizontal convergence above
            u_horiz = 0.0
            v_horiz = 0.0
        else:
            # Below outflow: downdraft transitions to horizontal outflow
            altitude_factor = altitude_m / self.outflow_height
            w_down = self.max_downdraft * radial_factor * altitude_factor

            # Horizontal outflow (radially outward)
            outflow_speed = self.max_downdraft * radial_factor * (1 - altitude_factor) * 1.5

            if r > 0.1:
                u_horiz = outflow_speed * (dx / r)
                v_horiz = outflow_speed * (dy / r)
            else:
                u_horiz = 0.0
                v_horiz = 0.0

        return u_horiz, v_horiz, w_down


class WindEnvironment:
    """
    Complete wind environment model combining base wind, shear, turbulence,
    and optional microburst.
    """

    def __init__(self, config_path: str = "data/config.json"):
        self.config = self._load_config(config_path)
        wind_cfg = self.config["wind"]

        # Base wind
        base_speed_mps = wind_cfg["base_speed_kts"] * KTS_TO_MPS
        base_direction = wind_cfg["base_direction_deg"]

        # Wind shear
        shear_cfg = wind_cfg.get("shear", {})
        if shear_cfg.get("enabled", False):
            self.shear = LogarithmicWindShear(
                reference_speed_mps=base_speed_mps,
                reference_height_m=shear_cfg.get("reference_height_m", 10.0),
                roughness_length_m=shear_cfg.get("surface_roughness_m", 0.03),
                wind_direction_deg=base_direction
            )
        else:
            self.shear = None
            self.base_north = -base_speed_mps * math.cos(base_direction * DEG_TO_RAD)
            self.base_east = -base_speed_mps * math.sin(base_direction * DEG_TO_RAD)

        # Turbulence
        turb_cfg = wind_cfg.get("turbulence", {})
        if turb_cfg.get("enabled", False):
            self.turbulence = DrydenTurbulence(
                sigma_u=turb_cfg.get("sigma_u_mps", 2.0),
                sigma_v=turb_cfg.get("sigma_v_mps", 2.0),
                sigma_w=turb_cfg.get("sigma_w_mps", 1.5),
                scale_length=turb_cfg.get("scale_length_m", 200.0),
                seed=turb_cfg.get("seed", None)
            )
        else:
            self.turbulence = None

        # Microburst
        mb_cfg = wind_cfg.get("microburst", {})
        if mb_cfg.get("enabled", False):
            self.microburst = MicroburstModel(
                center_x=mb_cfg.get("center_distance_nm", 0.5) * 1852,
                center_y=0,
                radius_m=mb_cfg.get("radius_nm", 0.25) * 1852,
                max_downdraft_mps=mb_cfg.get("max_downdraft_fpm", 1500) * 0.00508,
                outflow_height_m=100.0
            )
        else:
            self.microburst = None

        # Runway heading for crosswind calculation
        self.runway_heading_rad = self.config["airport"]["runway_heading_deg"] * DEG_TO_RAD

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, 'r') as f:
            return json.load(f)

    def get_wind(self, altitude_agl_m: float, x_m: float = 0.0, y_m: float = 0.0,
                 airspeed_mps: float = 30.0, dt: float = 0.01
                 ) -> WindComponents:
        """
        Get total wind at given position and altitude.

        Args:
            altitude_agl_m: Altitude above ground (meters)
            x_m, y_m: Position relative to threshold (meters)
            airspeed_mps: Aircraft true airspeed (for turbulence scaling)
            dt: Time step

        Returns:
            WindComponents with north, east, down components
        """
        # Base wind with shear
        if self.shear:
            wn, we, wd = self.shear.get_wind_ned(altitude_agl_m)
        else:
            wn, we, wd = self.base_north, self.base_east, 0.0

        # Add turbulence (in body frame, approximate as NED for simplicity)
        if self.turbulence:
            tu, tv, tw = self.turbulence.get_turbulence(airspeed_mps, altitude_agl_m, dt)
            wn += tu
            we += tv
            wd += tw

        # Add microburst
        if self.microburst:
            mn, me, md = self.microburst.get_wind_ned(x_m, y_m, altitude_agl_m)
            wn += mn
            we += me
            wd += md

        return WindComponents(north=wn, east=we, down=wd)

    def get_crosswind_component(self, wind: WindComponents) -> float:
        """
        Calculate crosswind component relative to runway (m/s).

        Positive = wind from right.
        """
        # Wind velocity in runway frame
        wind_speed = wind.speed_mps
        wind_dir_rad = math.atan2(wind.east, wind.north)  # Where wind is going TO

        # Crosswind is perpendicular to runway
        relative_dir = wind_dir_rad - self.runway_heading_rad
        crosswind = wind_speed * math.sin(relative_dir)

        return crosswind

    def get_headwind_component(self, wind: WindComponents) -> float:
        """
        Calculate headwind component relative to runway (m/s).

        Positive = headwind.
        """
        wind_speed = wind.speed_mps
        wind_dir_rad = math.atan2(wind.east, wind.north)

        relative_dir = wind_dir_rad - self.runway_heading_rad
        headwind = -wind_speed * math.cos(relative_dir)  # Negative because wind goes TO

        return headwind


class WindShearDetector:
    """
    Wind shear detection system based on aircraft state changes.

    Uses rate of change of airspeed and vertical speed to detect shear.
    """

    def __init__(self, config_path: str = "data/config.json"):
        self.config = self._load_config(config_path)

        shear_cfg = self.config.get("wind_shear", {}).get("detection", {})
        self.airspeed_threshold_kts = shear_cfg.get("airspeed_loss_threshold_kts", 15)
        self.sink_rate_threshold_fpm = shear_cfg.get("sink_rate_threshold_fpm", 1000)

        # State tracking
        self.prev_airspeed_kts = None
        self.prev_sink_rate_fpm = None
        self.prev_t = None

        # Averaging
        self.airspeed_changes = []
        self.sink_rate_changes = []
        self.window_sec = 3.0

        # Warmup period to ignore initialization transients
        self.warmup_sec = 10.0
        self.start_time = None

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, 'r') as f:
            return json.load(f)

    def update(self, t: float, airspeed_kts: float, sink_rate_fpm: float
               ) -> WindShearAlert:
        """
        Update detector with current state and check for wind shear.

        Args:
            t: Current simulation time
            airspeed_kts: Current indicated/calibrated airspeed
            sink_rate_fpm: Current sink rate (positive = descending)

        Returns:
            WindShearAlert with detection status
        """
        alert = WindShearAlert()

        # Initialize start time
        if self.start_time is None:
            self.start_time = t

        # During warmup, just track but don't alert
        in_warmup = (t - self.start_time) < self.warmup_sec

        if self.prev_t is None:
            self.prev_airspeed_kts = airspeed_kts
            self.prev_sink_rate_fpm = sink_rate_fpm
            self.prev_t = t
            return alert

        dt = t - self.prev_t
        if dt <= 0:
            return alert

        # Calculate rates of change
        airspeed_rate = (airspeed_kts - self.prev_airspeed_kts) / dt
        sink_rate_rate = (sink_rate_fpm - self.prev_sink_rate_fpm) / dt

        # Track changes over window
        self.airspeed_changes.append((t, airspeed_kts - self.prev_airspeed_kts))
        self.sink_rate_changes.append((t, sink_rate_fpm - self.prev_sink_rate_fpm))

        # Remove old entries
        self.airspeed_changes = [(ts, v) for ts, v in self.airspeed_changes
                                  if t - ts <= self.window_sec]
        self.sink_rate_changes = [(ts, v) for ts, v in self.sink_rate_changes
                                   if t - ts <= self.window_sec]

        # Calculate cumulative changes over window
        total_airspeed_change = sum(v for _, v in self.airspeed_changes)
        total_sink_change = sum(v for _, v in self.sink_rate_changes)

        # Detect shear conditions (only after warmup)
        if in_warmup:
            self.prev_airspeed_kts = airspeed_kts
            self.prev_sink_rate_fpm = sink_rate_fpm
            self.prev_t = t
            return alert

        # Airspeed loss (headwind shear / performance decreasing shear)
        if total_airspeed_change < -self.airspeed_threshold_kts:
            alert.detected = True
            alert.shear_type = WindShearType.HEADWIND_LOSS
            alert.airspeed_change_kts = total_airspeed_change
            alert.severity = min(1.0, abs(total_airspeed_change) /
                                 (self.airspeed_threshold_kts * 2))

        # Excessive sink rate
        if total_sink_change > self.sink_rate_threshold_fpm:
            alert.detected = True
            if alert.shear_type == WindShearType.HEADWIND_LOSS:
                alert.shear_type = WindShearType.MICROBURST
            else:
                alert.shear_type = WindShearType.DOWNDRAFT
            alert.vertical_speed_change_fpm = total_sink_change
            alert.severity = max(alert.severity,
                                 min(1.0, total_sink_change /
                                     (self.sink_rate_threshold_fpm * 2)))

        # Update previous values
        self.prev_airspeed_kts = airspeed_kts
        self.prev_sink_rate_fpm = sink_rate_fpm
        self.prev_t = t

        return alert

    def reset(self):
        """Reset detector state."""
        self.prev_airspeed_kts = None
        self.prev_sink_rate_fpm = None
        self.prev_t = None
        self.airspeed_changes = []
        self.sink_rate_changes = []
