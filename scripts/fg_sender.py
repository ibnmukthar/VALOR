import math
import os
import socket
import struct
import time
from typing import Optional

FG_NET_FDM_VERSION = 24
FT2M = 0.3048


class FlightGearGenericSender:
    """
    UDP sender for FlightGear's native FDM (FGNetFDM) protocol.

    Packs JSBSim state into the FGNetFDM binary struct and sends it at
    a fixed rate to FlightGear's --native-fdm=socket input.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 5501, cfg: Optional[dict] = None):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dest = (str(host), int(port))
        self._elev_ft = 0.0
        if cfg and "airport" in cfg:
            try:
                self._elev_ft = float(cfg["airport"].get("elevation_ft", 0.0))
            except Exception:
                self._elev_ft = 0.0

        # Optional debug: freeze the pose sent to FlightGear at the first
        # call, so we can verify that a static short-final position renders
        # correctly. Enable via environment variable FG_FREEZE_FDM=1.
        self._freeze_pose = os.getenv("FG_FREEZE_FDM", "").lower() in ("1", "true", "yes", "on")
        self._frozen_pose = None

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass

    @staticmethod
    def _safe_get(fdm, prop: str, default: float = 0.0) -> float:
        try:
            return float(fdm[prop])
        except Exception:
            return float(default)

    def send(self, fdm, t_sec: float):
        # Latitude/Longitude (prefer explicit geodetic)
        try:
            lat_deg = float(fdm["position/lat-geod-deg"])  # geodetic
        except Exception:
            lat_deg = float(fdm["position/lat-gc-deg"])    # great-circle
        try:
            lon_deg = float(fdm["position/long-geod-deg"])  # geodetic
        except Exception:
            lon_deg = float(fdm["position/long-gc-deg"])     # great-circle

        # Altitudes
        h_agl_ft = self._safe_get(fdm, "position/h-agl-ft", 0.0)
        # JSBSim has no terrain model, so h-sl-ft often equals h-agl-ft (assumes flat earth at 0).
        # We ALWAYS compute MSL as AGL + field elevation to give FlightGear correct altitude.
        h_sl_ft = h_agl_ft + self._elev_ft

        # Attitude (deg)
        roll_deg = self._safe_get(fdm, "attitude/phi-deg", 0.0)
        pitch_deg = self._safe_get(fdm, "attitude/theta-deg", 0.0)
        heading_deg = self._safe_get(fdm, "attitude/psi-deg", 0.0)

        # Body velocities (ft/s)
        u_fps = self._safe_get(fdm, "velocities/u-fps", 0.0)
        v_fps = self._safe_get(fdm, "velocities/v-fps", 0.0)
        w_fps = self._safe_get(fdm, "velocities/w-fps", 0.0)

        # Body rates (rad/s)
        p = self._safe_get(fdm, "velocities/p-rad_sec", 0.0)
        q = self._safe_get(fdm, "velocities/q-rad_sec", 0.0)
        r = self._safe_get(fdm, "velocities/r-rad_sec", 0.0)

        # Aerodynamic angles (rad)
        alpha_rad = self._safe_get(fdm, "aero/alpha-rad", 0.0)
        beta_rad = self._safe_get(fdm, "aero/beta-rad", 0.0)

        # Airspeed and climb
        tas_fps = self._safe_get(fdm, "velocities/vt-fps", 0.0)
        climb_rate_fps = self._safe_get(fdm, "velocities/h-dot-fps", 0.0)

        # Inertial velocities in local-level frame (ft/s) if available
        v_north_fps = self._safe_get(fdm, "velocities/v-north-fps", 0.0)
        v_east_fps = self._safe_get(fdm, "velocities/v-east-fps", 0.0)
        v_down_fps = self._safe_get(fdm, "velocities/v-down-fps", 0.0)

        # Basic accelerations and warnings
        A_X_pilot = 0.0
        A_Y_pilot = 0.0
        A_Z_pilot = 0.0
        stall_warning = 0.0
        slip_deg = 0.0

        # Conversions to FGNetFDM units
        lon_rad = math.radians(lon_deg)
        lat_rad = math.radians(lat_deg)
        alt_m = h_sl_ft * FT2M
        agl_m = h_agl_ft * FT2M
        phi_rad = math.radians(roll_deg)
        theta_rad = math.radians(pitch_deg)
        psi_rad = math.radians(heading_deg)

        # Euler angle rates: approximate with body rates
        phidot = float(p)
        thetadot = float(q)
        psidot = float(r)

        # Optional debug: freeze pose so FG sees a static aircraft on
        # short final. When enabled, we capture the first pose and then
        # keep re-sending it with zero velocities and rates.
        if self._freeze_pose:
            if self._frozen_pose is None:
                self._frozen_pose = (lon_rad, lat_rad, alt_m, agl_m, phi_rad, theta_rad, psi_rad)
            else:
                lon_rad, lat_rad, alt_m, agl_m, phi_rad, theta_rad, psi_rad = self._frozen_pose
                tas_fps = 0.0
                climb_rate_fps = 0.0
                v_north_fps = 0.0
                v_east_fps = 0.0
                v_down_fps = 0.0
                u_fps = 0.0
                v_fps = 0.0
                w_fps = 0.0
                phidot = 0.0
                thetadot = 0.0
                psidot = 0.0

        # Engine / fuel / gear: values chosen to keep FG's internal systems happy
        num_engines = 1
        eng_state = [2, 0, 0, 0]  # 2 ~ running

        # Nominal C172-like engine parameters (purely cosmetic for FG)
        rpm = [2400.0, 0.0, 0.0, 0.0]
        fuel_flow = [8.0, 0.0, 0.0, 0.0]
        fuel_px = [5.0, 0.0, 0.0, 0.0]
        egt = [1400.0, 0.0, 0.0, 0.0]
        cht = [300.0, 0.0, 0.0, 0.0]
        mp_osi = [20.0, 0.0, 0.0, 0.0]
        tit = [1300.0, 0.0, 0.0, 0.0]
        oil_temp = [180.0, 0.0, 0.0, 0.0]
        oil_px = [60.0, 0.0, 0.0, 0.0]

        # Give FlightGear plenty of fuel so its internal scripts don't scream "Out of fuel!"
        num_tanks = 2
        fuel_quantity = [25.0, 25.0, 0.0, 0.0]

        num_wheels = 3
        wow = [0, 0, 0]
        gear_pos = [1.0, 1.0, 1.0]
        gear_steer = [0.0, 0.0, 0.0]
        gear_compression = [0.0, 0.0, 0.0]

        # Time / environment / control surfaces
        cur_time = int(time.time())
        warp = 0
        visibility_m = 50000.0

        elevator = 0.0
        elevator_trim_tab = 0.0
        left_flap = 0.0
        right_flap = 0.0
        left_aileron = 0.0
        right_aileron = 0.0
        rudder = 0.0
        nose_wheel = 0.0
        speedbrake = 0.0
        spoilers = 0.0

        # Clamp all float fields; sanitize NaN/inf to keep struct packing safe
        def _clamp(x, lo=-1e6, hi=1e6):
            try:
                v = float(x)
            except Exception:
                return 0.0
            if not math.isfinite(v):
                return 0.0
            if v < lo:
                return lo
            if v > hi:
                return hi
            return v

        values = [
            FG_NET_FDM_VERSION,
            0,
            _clamp(lon_rad),
            _clamp(lat_rad),
            _clamp(alt_m),
            _clamp(agl_m),
            _clamp(phi_rad),
            _clamp(theta_rad),
            _clamp(psi_rad),
            _clamp(alpha_rad),
            _clamp(beta_rad),
            _clamp(phidot),
            _clamp(thetadot),
            _clamp(psidot),
            _clamp(tas_fps),
            _clamp(climb_rate_fps),
            _clamp(v_north_fps),
            _clamp(v_east_fps),
            _clamp(v_down_fps),
            _clamp(u_fps),
            _clamp(v_fps),
            _clamp(w_fps),
            _clamp(A_X_pilot),
            _clamp(A_Y_pilot),
            _clamp(A_Z_pilot),
            _clamp(stall_warning),
            _clamp(slip_deg),
            num_engines,
            *eng_state,
            *[_clamp(x) for x in rpm],
            *[_clamp(x) for x in fuel_flow],
            *[_clamp(x) for x in fuel_px],
            *[_clamp(x) for x in egt],
            *[_clamp(x) for x in cht],
            *[_clamp(x) for x in mp_osi],
            *[_clamp(x) for x in tit],
            *[_clamp(x) for x in oil_temp],
            *[_clamp(x) for x in oil_px],
            num_tanks,
            *[_clamp(x) for x in fuel_quantity],
            num_wheels,
            *wow,
            *[_clamp(x) for x in gear_pos],
            *[_clamp(x) for x in gear_steer],
            *[_clamp(x) for x in gear_compression],
            cur_time,
            warp,
            _clamp(visibility_m),
            _clamp(elevator),
            _clamp(elevator_trim_tab),
            _clamp(left_flap),
            _clamp(right_flap),
            _clamp(left_aileron),
            _clamp(right_aileron),
            _clamp(rudder),
            _clamp(nose_wheel),
            _clamp(speedbrake),
            _clamp(spoilers),
        ]

        fmt = "!II3d22fI4I36fI4fI3I3f3f3fIif10f"

        try:
            packet = struct.pack(fmt, *values)
            self._sock.sendto(packet, self._dest)
        except Exception as e:
            # Keep FlightGear from freezing if we ever hit a bad value
            print(f"[FG_SENDER_PACK_ERROR] {e}")

