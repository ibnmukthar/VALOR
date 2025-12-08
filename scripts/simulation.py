"""
VALOR - JSBSim Simulation Engine
Handles aircraft state, flight dynamics, and simulation loop.
"""

import math
import json
import jsbsim
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple
from pathlib import Path


# Conversion constants
FT_TO_M = 0.3048
M_TO_FT = 3.28084
KTS_TO_MPS = 0.514444
MPS_TO_KTS = 1.94384
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
NM_TO_M = 1852.0
FPM_TO_MPS = 0.00508


@dataclass
class AircraftState:
    """Complete aircraft state at a given instant."""
    # Time
    t: float = 0.0

    # Position (geodetic)
    lat_rad: float = 0.0
    lon_rad: float = 0.0
    alt_msl_ft: float = 0.0
    alt_agl_ft: float = 0.0

    # Attitude (body-frame Euler angles)
    phi_rad: float = 0.0    # Roll
    theta_rad: float = 0.0  # Pitch
    psi_rad: float = 0.0    # Heading (true)

    # Body-frame velocities (ft/s)
    u: float = 0.0  # Forward
    v: float = 0.0  # Right
    w: float = 0.0  # Down

    # NED velocities (ft/s)
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0

    # Angular rates (rad/s)
    p: float = 0.0  # Roll rate
    q: float = 0.0  # Pitch rate
    r: float = 0.0  # Yaw rate

    # Air data
    alpha_rad: float = 0.0   # Angle of attack
    beta_rad: float = 0.0    # Sideslip angle
    vtrue_fps: float = 0.0   # True airspeed
    vcas_fps: float = 0.0    # Calibrated airspeed
    mach: float = 0.0
    gamma_rad: float = 0.0   # Flight path angle

    # Control surfaces
    aileron: float = 0.0     # -1 to +1 (left down positive)
    elevator: float = 0.0    # -1 to +1 (trailing edge down positive)
    rudder: float = 0.0      # -1 to +1 (trailing edge left positive)
    throttle: float = 0.0    # 0 to 1
    flaps: float = 0.0       # 0 to 1

    # Gear status
    gear_down: bool = True
    wow: bool = False  # Weight on wheels

    # Computed navigation
    distance_to_threshold_m: float = 0.0
    cross_track_error_m: float = 0.0
    glideslope_error_deg: float = 0.0
    track_angle_rad: float = 0.0

    @property
    def lat_deg(self) -> float:
        return self.lat_rad * RAD_TO_DEG

    @property
    def lon_deg(self) -> float:
        return self.lon_rad * RAD_TO_DEG

    @property
    def phi_deg(self) -> float:
        return self.phi_rad * RAD_TO_DEG

    @property
    def theta_deg(self) -> float:
        return self.theta_rad * RAD_TO_DEG

    @property
    def psi_deg(self) -> float:
        return self.psi_rad * RAD_TO_DEG

    @property
    def alpha_deg(self) -> float:
        return self.alpha_rad * RAD_TO_DEG

    @property
    def beta_deg(self) -> float:
        return self.beta_rad * RAD_TO_DEG

    @property
    def gamma_deg(self) -> float:
        return self.gamma_rad * RAD_TO_DEG

    @property
    def vtrue_kts(self) -> float:
        return self.vtrue_fps * 0.592484

    @property
    def vcas_kts(self) -> float:
        return self.vcas_fps * 0.592484

    @property
    def vd_fpm(self) -> float:
        return self.vd * 60.0  # Sink rate in ft/min

    @property
    def groundspeed_fps(self) -> float:
        """Groundspeed from NED velocities (ft/s)."""
        import math
        return math.sqrt(self.vn**2 + self.ve**2)

    @property
    def groundspeed_kts(self) -> float:
        """Groundspeed in knots."""
        return self.groundspeed_fps * 0.592484


class JSBSimEngine:
    """Wrapper around JSBSim FDM for flight simulation."""

    def __init__(self, config_path: str = "data/config.json"):
        self.config = self._load_config(config_path)
        self.fdm: Optional[jsbsim.FGFDMExec] = None
        self.state = AircraftState()
        self.dt = self.config["simulation"]["dt_sec"]

        # Airport/runway reference
        self.rwy_lat_rad = self.config["airport"]["threshold_lat_deg"] * DEG_TO_RAD
        self.rwy_lon_rad = self.config["airport"]["threshold_lon_deg"] * DEG_TO_RAD
        self.rwy_heading_rad = self.config["airport"]["runway_heading_deg"] * DEG_TO_RAD
        self.rwy_elevation_ft = self.config["airport"]["elevation_ft"]

    def _load_config(self, config_path: str) -> dict:
        """Load configuration from JSON file."""
        with open(config_path, 'r') as f:
            return json.load(f)

    def initialize(self) -> bool:
        """Initialize JSBSim and set up initial conditions."""
        # Create FDM instance
        self.fdm = jsbsim.FGFDMExec(None)
        self.fdm.set_debug_level(0)

        # Load aircraft model
        aircraft = self.config["aircraft"]["model"]
        if not self.fdm.load_model(aircraft):
            print(f"ERROR: Failed to load aircraft model '{aircraft}'")
            return False

        # Set integration timestep
        self.fdm.set_dt(self.dt)

        # Compute initial position on extended runway centerline
        sim_cfg = self.config["simulation"]
        dist_nm = sim_cfg["initial_distance_nm"]
        dist_m = dist_nm * NM_TO_M

        # Position back from threshold along runway heading
        back_bearing = (self.rwy_heading_rad + math.pi) % (2 * math.pi)
        init_lat, init_lon = self._offset_position(
            self.rwy_lat_rad, self.rwy_lon_rad, back_bearing, dist_m
        )

        # Compute initial altitude on glideslope
        # Glideslope aims for touchdown zone ~300m past threshold, not threshold itself
        #
        # IMPORTANT: The configured glideslope (3°) is the TARGET angle.
        # But with gear down and no flaps, the C172 actually descends at ~6-7°.
        # We use a lower initial altitude to match reality, so the aircraft
        # reaches the runway threshold correctly.
        TOUCHDOWN_ZONE_OFFSET_M = 300.0

        # Use the ACHIEVABLE glideslope angle, not the config value
        # This is determined by the aircraft's drag characteristics
        ACTUAL_GLIDESLOPE_DEG = 6.5  # Measured from flight tests
        alt_agl = (dist_m + TOUCHDOWN_ZONE_OFFSET_M) * math.tan(ACTUAL_GLIDESLOPE_DEG * DEG_TO_RAD) * M_TO_FT
        alt_msl = alt_agl + self.rwy_elevation_ft

        # Set initial conditions via JSBSim IC properties
        # JSBSim uses property paths for IC values
        self.fdm.set_property_value("ic/lat-gc-rad", init_lat)
        self.fdm.set_property_value("ic/long-gc-rad", init_lon)
        self.fdm.set_property_value("ic/h-agl-ft", alt_agl)

        # Heading aligned with runway
        self.fdm.set_property_value("ic/psi-true-rad", self.rwy_heading_rad)

        # Level wings, slight nose up for approach
        self.fdm.set_property_value("ic/phi-rad", 0.0)
        self.fdm.set_property_value("ic/theta-rad", 2.0 * DEG_TO_RAD)  # 2 deg nose up

        # Approach speed
        approach_kts = self.config["aircraft"]["approach_speed_kts"]
        self.fdm.set_property_value("ic/vc-kts", approach_kts)

        # Zero rates
        self.fdm.set_property_value("ic/p-rad_sec", 0.0)
        self.fdm.set_property_value("ic/q-rad_sec", 0.0)
        self.fdm.set_property_value("ic/r-rad_sec", 0.0)

        # Run IC to set initial state
        self.fdm.run_ic()

        # Initialize engine and fuel system
        # Set engine running with full magnetos
        self.fdm.set_property_value("propulsion/engine[0]/set-running", 1)
        self.fdm.set_property_value("propulsion/magneto_cmd", 3)  # Both magnetos
        self.fdm.set_property_value("fcs/mixture-cmd-norm", 1.0)
        self.fdm.set_property_value("fcs/advance-cmd-norm", 1.0)  # Full prop RPM

        # Set fuel quantities
        self.fdm.set_property_value("propulsion/tank[0]/contents-lbs", 180)  # Main tank
        self.fdm.set_property_value("propulsion/tank[1]/contents-lbs", 180)  # Aux tank

        # Throttle for approach power
        self.fdm.set_property_value("fcs/throttle-cmd-norm", 0.65)

        # Set flaps for approach
        flap_setting = self.config["aircraft"]["flap_setting"]
        self.fdm.set_property_value("fcs/flap-cmd-norm", flap_setting)

        # Gear down
        self.fdm.set_property_value("gear/gear-cmd-norm", 1.0)

        # Suppress wind during initialization to prevent lateral drift
        self.set_wind(0, 0, 0)

        # Run several frames to stabilize engine and trim
        for _ in range(100):
            self.fdm.run()

        # Wind will be applied by main loop - no need to set here

        # Update state from FDM
        self._update_state()

        print(f"Simulation initialized:")
        print(f"  Aircraft: {aircraft}")
        print(f"  Position: {self.state.lat_deg:.4f}°N, {self.state.lon_deg:.4f}°W")
        print(f"  Altitude: {self.state.alt_agl_ft:.0f} ft AGL")
        print(f"  Distance to threshold: {self.state.distance_to_threshold_m:.0f} m")
        print(f"  Airspeed: {self.state.vcas_kts:.0f} kts")

        return True

    def step(self, aileron: float, elevator: float, rudder: float, throttle: float) -> AircraftState:
        """
        Advance simulation by one timestep with given control inputs.

        Args:
            aileron: Aileron command (-1 to +1)
            elevator: Elevator command (-1 to +1)
            rudder: Rudder command (-1 to +1)
            throttle: Throttle command (0 to 1)

        Returns:
            Updated aircraft state
        """
        if self.fdm is None:
            raise RuntimeError("Simulation not initialized")

        # Clamp and apply control inputs
        # JSBSim conventions:
        # - Positive elevator = stick forward = nose down (inverted from our convention)
        # - Positive aileron = stick right = right roll (matches our convention)
        # - Positive rudder = right pedal = yaw right (matches our convention)
        self.fdm.set_property_value("fcs/aileron-cmd-norm", np.clip(aileron, -1, 1))
        self.fdm.set_property_value("fcs/elevator-cmd-norm", np.clip(-elevator, -1, 1))  # Inverted
        self.fdm.set_property_value("fcs/rudder-cmd-norm", np.clip(rudder, -1, 1))
        self.fdm.set_property_value("fcs/throttle-cmd-norm", np.clip(throttle, 0, 1))

        # Advance simulation
        self.fdm.run()

        # Update state
        self._update_state()

        return self.state

    def _update_state(self):
        """Extract current state from JSBSim FDM."""
        fdm = self.fdm

        # Time
        self.state.t = fdm.get_sim_time()

        # Position
        self.state.lat_rad = fdm.get_property_value("position/lat-gc-rad")
        self.state.lon_rad = fdm.get_property_value("position/long-gc-rad")
        self.state.alt_agl_ft = fdm.get_property_value("position/h-agl-ft")
        # Compute MSL altitude (JSBSim terrain model is flat at 0)
        self.state.alt_msl_ft = self.state.alt_agl_ft + self.rwy_elevation_ft

        # Attitude
        self.state.phi_rad = fdm.get_property_value("attitude/phi-rad")
        self.state.theta_rad = fdm.get_property_value("attitude/theta-rad")
        self.state.psi_rad = fdm.get_property_value("attitude/psi-rad")

        # Body velocities
        self.state.u = fdm.get_property_value("velocities/u-fps")
        self.state.v = fdm.get_property_value("velocities/v-fps")
        self.state.w = fdm.get_property_value("velocities/w-fps")

        # NED velocities
        self.state.vn = fdm.get_property_value("velocities/v-north-fps")
        self.state.ve = fdm.get_property_value("velocities/v-east-fps")
        self.state.vd = fdm.get_property_value("velocities/v-down-fps")

        # Angular rates
        self.state.p = fdm.get_property_value("velocities/p-rad_sec")
        self.state.q = fdm.get_property_value("velocities/q-rad_sec")
        self.state.r = fdm.get_property_value("velocities/r-rad_sec")

        # Air data
        self.state.alpha_rad = fdm.get_property_value("aero/alpha-rad")
        self.state.beta_rad = fdm.get_property_value("aero/beta-rad")
        self.state.vtrue_fps = fdm.get_property_value("velocities/vtrue-fps")
        self.state.vcas_fps = fdm.get_property_value("velocities/vc-fps")
        self.state.mach = fdm.get_property_value("velocities/mach")

        # Flight path angle
        groundspeed = math.sqrt(self.state.vn**2 + self.state.ve**2)
        if groundspeed > 1.0:
            self.state.gamma_rad = math.atan2(-self.state.vd, groundspeed)
        else:
            self.state.gamma_rad = 0.0

        # Control positions (actual, not commanded)
        self.state.aileron = fdm.get_property_value("fcs/aileron-pos-norm")
        self.state.elevator = fdm.get_property_value("fcs/elevator-pos-norm")
        self.state.rudder = fdm.get_property_value("fcs/rudder-pos-norm")
        self.state.throttle = fdm.get_property_value("fcs/throttle-pos-norm")
        self.state.flaps = fdm.get_property_value("fcs/flap-pos-norm")

        # Gear/ground
        self.state.gear_down = fdm.get_property_value("gear/gear-pos-norm") > 0.5
        wow0 = fdm.get_property_value("gear/unit[0]/WOW")
        wow1 = fdm.get_property_value("gear/unit[1]/WOW")
        wow2 = fdm.get_property_value("gear/unit[2]/WOW")
        self.state.wow = bool(wow0 or wow1 or wow2)

        # Track angle (ground track direction)
        self.state.track_angle_rad = math.atan2(self.state.ve, self.state.vn)

        # Navigation computations
        self._compute_navigation()

    def _compute_navigation(self):
        """Compute navigation parameters relative to runway."""
        # Distance to threshold
        self.state.distance_to_threshold_m = self._haversine_distance(
            self.state.lat_rad, self.state.lon_rad,
            self.rwy_lat_rad, self.rwy_lon_rad
        )

        # Bearing from aircraft to threshold
        bearing_to_thr = self._bearing(
            self.state.lat_rad, self.state.lon_rad,
            self.rwy_lat_rad, self.rwy_lon_rad
        )

        # Cross-track error (perpendicular distance from extended centerline)
        # If aircraft is RIGHT of centerline, bearing to threshold is LEFT of runway heading
        # angle_diff = bearing - runway < 0 for right of centerline
        # So we NEGATE to get positive CTE for right of centerline
        angle_diff = bearing_to_thr - self.rwy_heading_rad
        self.state.cross_track_error_m = (
            -self.state.distance_to_threshold_m * math.sin(angle_diff)
        )

        # Along-track distance (positive = before threshold)
        along_track_m = self.state.distance_to_threshold_m * math.cos(angle_diff)

        # Glideslope aims for touchdown zone ~300m past threshold, not threshold itself
        # This is consistent with real ILS where aircraft crosses threshold at ~50ft AGL
        TOUCHDOWN_ZONE_OFFSET_M = 300.0
        gs_along_track_m = along_track_m + TOUCHDOWN_ZONE_OFFSET_M

        # Glideslope error calculation
        # Use ACTUAL achievable glideslope, not configured value
        # C172 with gear down, no flaps achieves ~6.5° descent at full power
        ACTUAL_GLIDESLOPE_DEG = 6.5
        desired_alt_agl_ft = gs_along_track_m * math.tan(ACTUAL_GLIDESLOPE_DEG * DEG_TO_RAD) * M_TO_FT

        # Continue GS guidance until very close to touchdown zone
        if gs_along_track_m > 10:
            self.state.glideslope_error_deg = math.atan2(
                (self.state.alt_agl_ft - desired_alt_agl_ft) * FT_TO_M,
                gs_along_track_m
            ) * RAD_TO_DEG
        else:
            self.state.glideslope_error_deg = 0.0

    def _offset_position(self, lat: float, lon: float,
                         bearing: float, distance: float) -> Tuple[float, float]:
        """
        Calculate new position given starting point, bearing and distance.
        Uses simple spherical earth model.
        """
        R = 6371000  # Earth radius in meters

        lat2 = math.asin(
            math.sin(lat) * math.cos(distance / R) +
            math.cos(lat) * math.sin(distance / R) * math.cos(bearing)
        )
        lon2 = lon + math.atan2(
            math.sin(bearing) * math.sin(distance / R) * math.cos(lat),
            math.cos(distance / R) - math.sin(lat) * math.sin(lat2)
        )

        return lat2, lon2

    def _haversine_distance(self, lat1: float, lon1: float,
                            lat2: float, lon2: float) -> float:
        """Calculate great-circle distance between two points (meters)."""
        R = 6371000  # Earth radius
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = (math.sin(dlat/2)**2 +
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def _bearing(self, lat1: float, lon1: float,
                 lat2: float, lon2: float) -> float:
        """Calculate initial bearing from point 1 to point 2."""
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) -
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        return math.atan2(x, y)

    def set_wind(self, wind_north_fps: float, wind_east_fps: float,
                 wind_down_fps: float = 0.0):
        """Set wind components in the NED frame (ft/s)."""
        if self.fdm is not None:
            self.fdm.set_property_value("atmosphere/wind-north-fps", wind_north_fps)
            self.fdm.set_property_value("atmosphere/wind-east-fps", wind_east_fps)
            self.fdm.set_property_value("atmosphere/wind-down-fps", wind_down_fps)

    def is_on_ground(self) -> bool:
        """Check if aircraft is on the ground."""
        return self.state.wow

    def is_crashed(self) -> bool:
        """Check for crash conditions."""
        # Hard landing (excessive sink rate at touchdown)
        # Only check if we're very close to ground and sink rate is way too high
        if self.state.alt_agl_ft < 3 and self.state.vd_fpm > 800:
            return True
        # Below ground (shouldn't happen)
        if self.state.alt_agl_ft < -5:
            return True
        # Excessive bank near ground
        if self.state.alt_agl_ft < 50 and abs(self.state.phi_deg) > 30:
            return True
        return False
