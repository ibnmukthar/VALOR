"""
VALOR - FlightGear Visualization Interface
Implements FGNetFDM protocol for accurate aircraft state transmission.
"""

import socket
import struct
import math
import json
from typing import Optional
from simulation import AircraftState, FT_TO_M, DEG_TO_RAD


class FGNetFDM:
    """
    FlightGear Network FDM (Flight Dynamics Model) protocol.

    This is the native binary protocol used by FlightGear to receive
    external flight dynamics data. The structure must exactly match
    FlightGear's expected format.

    Reference: FlightGear source code - src/Network/net_fdm.hxx
    """

    # FGNetFDM version - must match FlightGear expectations
    FG_NET_FDM_VERSION = 24

    def __init__(self, host: str = "127.0.0.1", port: int = 5501,
                 field_elevation_ft: float = 0.0):
        """
        Initialize FGNetFDM sender.

        Args:
            host: FlightGear host address
            port: UDP port FlightGear is listening on
            field_elevation_ft: Airport field elevation for MSL calculation
        """
        self.host = host
        self.port = port
        self.field_elevation_ft = field_elevation_ft
        self.socket: Optional[socket.socket] = None

    def connect(self):
        """Create UDP socket for sending."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"FlightGear sender ready: {self.host}:{self.port}")

    def close(self):
        """Close the socket."""
        if self.socket:
            self.socket.close()
            self.socket = None

    def send(self, state: AircraftState):
        """
        Pack and send aircraft state to FlightGear.

        The FGNetFDM structure (from FlightGear source):
        - Version and padding
        - Position: lon/lat (radians), altitude (meters)
        - Attitude: phi/theta/psi (radians)
        - Velocities: ground track, climb rate, etc.
        - Angular velocities: phidot, thetadot, psidot
        - Accelerations
        - Control surfaces
        - Engine data
        - Gear data
        - And more...
        """
        if not self.socket:
            self.connect()

        # Build the FGNetFDM packet
        packet = self._build_packet(state)

        try:
            self.socket.sendto(packet, (self.host, self.port))
        except Exception as e:
            print(f"FlightGear send error: {e}")

    def _build_packet(self, state: AircraftState) -> bytes:
        """
        Build the binary FGNetFDM packet.

        This matches the exact structure FlightGear expects.
        All multi-byte values are in network byte order (big-endian).
        """
        # Safe value helper
        def safe(x, default=0.0):
            if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
                return default
            return x

        # Calculate MSL altitude properly
        alt_msl_m = safe(state.alt_agl_ft + self.field_elevation_ft) * FT_TO_M
        alt_agl_m = safe(state.alt_agl_ft) * FT_TO_M

        # Convert velocities to proper units
        climb_rate_mps = -safe(state.vd) * FT_TO_M  # vd is positive down, climb_rate positive up
        vtrue_mps = safe(state.vtrue_fps) * FT_TO_M
        vcas_mps = safe(state.vcas_fps) * FT_TO_M

        # Body velocities in m/s
        u_mps = safe(state.u) * FT_TO_M
        v_mps = safe(state.v) * FT_TO_M
        w_mps = safe(state.w) * FT_TO_M

        # NED velocities in m/s
        vn_mps = safe(state.vn) * FT_TO_M
        ve_mps = safe(state.ve) * FT_TO_M
        vd_mps = safe(state.vd) * FT_TO_M

        # Ground track angle and speed
        groundspeed_mps = math.sqrt(vn_mps**2 + ve_mps**2)
        track_rad = math.atan2(ve_mps, vn_mps) if groundspeed_mps > 0.1 else state.psi_rad

        # Pack the FGNetFDM structure
        # Format: network byte order (big-endian)
        #
        # The structure:
        # uint32_t version (4 bytes)
        # uint32_t padding (4 bytes)
        # double longitude (8 bytes) - radians
        # double latitude (8 bytes) - radians
        # double altitude (8 bytes) - meters MSL
        # float agl (4 bytes) - meters
        # float phi (4 bytes) - radians (roll)
        # float theta (4 bytes) - radians (pitch)
        # float psi (4 bytes) - radians (heading)
        # float alpha (4 bytes) - radians
        # float beta (4 bytes) - radians
        # float phidot (4 bytes) - rad/s
        # float thetadot (4 bytes) - rad/s
        # float psidot (4 bytes) - rad/s
        # float vcas (4 bytes) - m/s
        # float climb_rate (4 bytes) - m/s (positive up)
        # float v_north (4 bytes) - m/s
        # float v_east (4 bytes) - m/s
        # float v_down (4 bytes) - m/s
        # float v_body_u (4 bytes) - m/s
        # float v_body_v (4 bytes) - m/s
        # float v_body_w (4 bytes) - m/s
        # float A_X_pilot (4 bytes) - m/s^2
        # float A_Y_pilot (4 bytes) - m/s^2
        # float A_Z_pilot (4 bytes) - m/s^2
        # float stall_warning (4 bytes) - 0-1
        # float slip_deg (4 bytes) - degrees
        # uint32_t num_engines (4 bytes)
        # uint32_t eng_state[4] (16 bytes)
        # float rpm[4] (16 bytes)
        # float fuel_flow[4] (16 bytes) - gal/hr
        # float fuel_px[4] (16 bytes) - psi
        # float egt[4] (16 bytes) - deg F
        # float cht[4] (16 bytes) - deg F
        # float mp_osi[4] (16 bytes) - manifold pressure (inHg)
        # float tit[4] (16 bytes) - turbine inlet temp
        # float oil_temp[4] (16 bytes) - deg F
        # float oil_px[4] (16 bytes) - psi
        # uint32_t num_tanks (4 bytes)
        # float fuel_quantity[4] (16 bytes) - gallons
        # uint32_t num_wheels (4 bytes)
        # uint32_t wow[3] (12 bytes) - weight on wheels
        # float gear_pos[3] (12 bytes) - 0-1
        # float gear_steer[3] (12 bytes) - degrees
        # float gear_compression[3] (12 bytes) - normalized
        # uint32_t cur_time (4 bytes) - epoch seconds
        # int32_t warp (4 bytes) - time warp
        # float visibility (4 bytes) - meters
        # float elevator (4 bytes) - normalized
        # float elevator_trim_tab (4 bytes) - normalized
        # float left_flap (4 bytes) - normalized
        # float right_flap (4 bytes) - normalized
        # float left_aileron (4 bytes) - normalized
        # float right_aileron (4 bytes) - normalized
        # float rudder (4 bytes) - normalized
        # float nose_wheel (4 bytes) - degrees
        # float speedbrake (4 bytes) - normalized
        # float spoilers (4 bytes) - normalized

        # Engine data (1 engine, remaining zeroed)
        num_engines = 1
        eng_state = [2, 0, 0, 0]  # 2 = running
        rpm = [2400.0, 0.0, 0.0, 0.0]
        fuel_flow = [8.0, 0.0, 0.0, 0.0]  # gal/hr
        fuel_px = [6.0, 0.0, 0.0, 0.0]
        egt = [1200.0, 0.0, 0.0, 0.0]
        cht = [400.0, 0.0, 0.0, 0.0]
        mp_osi = [24.0, 0.0, 0.0, 0.0]
        tit = [0.0, 0.0, 0.0, 0.0]
        oil_temp = [180.0, 0.0, 0.0, 0.0]
        oil_px = [60.0, 0.0, 0.0, 0.0]

        # Fuel tanks
        num_tanks = 2
        fuel_quantity = [25.0, 25.0, 0.0, 0.0]  # gallons

        # Gear data
        num_wheels = 3
        wow_state = [1 if state.wow else 0, 1 if state.wow else 0, 1 if state.wow else 0]
        gear_pos = [1.0, 1.0, 1.0]  # Fully extended
        gear_steer = [0.0, 0.0, 0.0]
        gear_compression = [0.1 if state.wow else 0.0] * 3

        # Time
        cur_time = int(state.t)
        warp = 0

        # Visibility
        visibility = 20000.0  # 20 km

        # Control surfaces (normalized -1 to 1)
        elevator = safe(state.elevator)
        elevator_trim = 0.0
        left_flap = safe(state.flaps)
        right_flap = safe(state.flaps)
        left_aileron = safe(state.aileron)
        right_aileron = -safe(state.aileron)  # Opposite deflection
        rudder = safe(state.rudder)
        nose_wheel = safe(state.rudder) * 10.0  # Linked to rudder, degrees
        speedbrake = 0.0
        spoilers = 0.0

        # Now pack everything
        # Format string for struct.pack
        # ! = network byte order (big-endian)
        # I = unsigned int (4 bytes)
        # d = double (8 bytes)
        # f = float (4 bytes)
        # i = signed int (4 bytes)

        fmt = "!II"          # version, padding
        fmt += "ddd"         # lon, lat, alt
        fmt += "f"           # agl
        fmt += "fff"         # phi, theta, psi
        fmt += "ff"          # alpha, beta
        fmt += "fff"         # phidot, thetadot, psidot
        fmt += "ff"          # vcas, climb_rate
        fmt += "fff"         # v_north, v_east, v_down
        fmt += "fff"         # v_body_u, v_body_v, v_body_w
        fmt += "fff"         # A_X, A_Y, A_Z
        fmt += "ff"          # stall_warning, slip_deg
        fmt += "I"           # num_engines
        fmt += "4I"          # eng_state[4]
        fmt += "4f"          # rpm[4]
        fmt += "4f"          # fuel_flow[4]
        fmt += "4f"          # fuel_px[4]
        fmt += "4f"          # egt[4]
        fmt += "4f"          # cht[4]
        fmt += "4f"          # mp_osi[4]
        fmt += "4f"          # tit[4]
        fmt += "4f"          # oil_temp[4]
        fmt += "4f"          # oil_px[4]
        fmt += "I"           # num_tanks
        fmt += "4f"          # fuel_quantity[4]
        fmt += "I"           # num_wheels
        fmt += "3I"          # wow[3]
        fmt += "3f"          # gear_pos[3]
        fmt += "3f"          # gear_steer[3]
        fmt += "3f"          # gear_compression[3]
        fmt += "Ii"          # cur_time, warp
        fmt += "f"           # visibility
        fmt += "ff"          # elevator, elevator_trim
        fmt += "ff"          # left_flap, right_flap
        fmt += "ff"          # left_aileron, right_aileron
        fmt += "f"           # rudder
        fmt += "f"           # nose_wheel
        fmt += "ff"          # speedbrake, spoilers

        packet = struct.pack(
            fmt,
            # Header
            self.FG_NET_FDM_VERSION,  # version
            0,                         # padding
            # Position
            safe(state.lon_rad),       # longitude (radians)
            safe(state.lat_rad),       # latitude (radians)
            alt_msl_m,                 # altitude MSL (meters)
            alt_agl_m,                 # AGL (meters)
            # Attitude
            safe(state.phi_rad),       # roll
            safe(state.theta_rad),     # pitch
            safe(state.psi_rad),       # heading
            # Aero angles
            safe(state.alpha_rad),     # alpha
            safe(state.beta_rad),      # beta
            # Angular rates
            safe(state.p),             # roll rate
            safe(state.q),             # pitch rate
            safe(state.r),             # yaw rate
            # Velocities
            vcas_mps,                  # calibrated airspeed
            climb_rate_mps,            # climb rate
            vn_mps,                    # north velocity
            ve_mps,                    # east velocity
            vd_mps,                    # down velocity
            u_mps,                     # body u
            v_mps,                     # body v
            w_mps,                     # body w
            # Accelerations (simplified)
            0.0, 0.0, -9.81,           # A_X, A_Y, A_Z
            # Warnings
            0.0,                       # stall warning
            safe(state.beta_rad) * 57.3,  # slip (degrees)
            # Engines
            num_engines,
            *eng_state,
            *rpm,
            *fuel_flow,
            *fuel_px,
            *egt,
            *cht,
            *mp_osi,
            *tit,
            *oil_temp,
            *oil_px,
            # Fuel
            num_tanks,
            *fuel_quantity,
            # Gear
            num_wheels,
            *wow_state,
            *gear_pos,
            *gear_steer,
            *gear_compression,
            # Time
            cur_time,
            warp,
            # Environment
            visibility,
            # Control surfaces
            elevator,
            elevator_trim,
            left_flap,
            right_flap,
            left_aileron,
            right_aileron,
            rudder,
            nose_wheel,
            speedbrake,
            spoilers,
        )

        return packet


def create_flightgear_sender(config_path: str = "data/config.json") -> FGNetFDM:
    """Create and configure a FlightGear sender from config file."""
    with open(config_path, 'r') as f:
        config = json.load(f)

    fg_cfg = config.get("flightgear", {})
    airport_cfg = config.get("airport", {})

    sender = FGNetFDM(
        host=fg_cfg.get("host", "127.0.0.1"),
        port=fg_cfg.get("port", 5501),
        field_elevation_ft=airport_cfg.get("elevation_ft", 0.0)
    )

    return sender
