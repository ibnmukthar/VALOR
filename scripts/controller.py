"""
VALOR - Autoland Controller System
Implements complete ILS autoland with crosswind handling using crab-to-sideslip technique.

Controller architecture:
1. Localizer (lateral) - Track runway centerline with wind correction
2. Glideslope (vertical) - Track 3° descent path with flare
3. Autothrottle - Maintain approach speed with wind compensation
4. Yaw control - Crab-to-sideslip transition near touchdown
"""

import math
import json
from dataclasses import dataclass, field
from typing import Tuple, Optional
from enum import Enum, auto

from simulation import AircraftState, DEG_TO_RAD, RAD_TO_DEG


class AutolandPhase(Enum):
    """Autoland phases."""
    APPROACH = auto()      # Normal approach, tracking LOC/GS
    FLARE = auto()         # Flare initiation, pitch up and throttle retard
    DECRAB = auto()        # Transition from crab to sideslip
    ROLLOUT = auto()       # On runway, deceleration
    GO_AROUND = auto()     # Rejected landing


@dataclass
class PIDState:
    """PID controller state."""
    integral: float = 0.0
    prev_error: float = 0.0
    prev_output: float = 0.0


class PIDController:
    """
    PID controller with anti-windup and rate limiting.
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_limit: float = 10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.state = PIDState()

    def compute(self, error: float, dt: float) -> float:
        """
        Compute PID output.

        Args:
            error: Current error (setpoint - process variable)
            dt: Time step in seconds

        Returns:
            Control output (clamped to limits)
        """
        if dt <= 0:
            return self.state.prev_output

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.state.integral += error * dt
        self.state.integral = max(-self.integral_limit,
                                   min(self.integral_limit, self.state.integral))
        i_term = self.ki * self.state.integral

        # Derivative term (on error, with filtering)
        d_term = 0.0
        if self.state.prev_error is not None:
            d_term = self.kd * (error - self.state.prev_error) / dt

        self.state.prev_error = error

        # Total output with saturation
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        self.state.prev_output = output
        return output

    def reset(self):
        """Reset controller state."""
        self.state = PIDState()


class AutolandController:
    """
    Complete autoland controller implementing:
    - Localizer tracking with wind correction
    - Glideslope tracking with flare
    - Autothrottle with wind compensation
    - Crab-to-sideslip transition for crosswind landing
    """

    def __init__(self, config_path: str = "data/config.json"):
        self.config = self._load_config(config_path)

        # Initialize phase
        self.phase = AutolandPhase.APPROACH

        # Extract configuration
        al_cfg = self.config["autoland"]

        # Localizer (lateral) controller - outputs bank angle command
        loc_cfg = al_cfg["localizer"]
        self.loc_controller = PIDController(
            kp=loc_cfg["kp"],
            ki=loc_cfg["ki"],
            kd=loc_cfg["kd"],
            output_min=-loc_cfg["max_bank_deg"],
            output_max=loc_cfg["max_bank_deg"]
        )
        self.max_bank_deg = loc_cfg["max_bank_deg"]

        # Glideslope (vertical) controller - outputs pitch rate command
        gs_cfg = al_cfg["glideslope"]
        self.gs_controller = PIDController(
            kp=gs_cfg["kp"],
            ki=gs_cfg["ki"],
            kd=gs_cfg["kd"],
            output_min=gs_cfg["min_pitch_deg"],
            output_max=gs_cfg["max_pitch_deg"]
        )

        # Roll rate controller (inner loop)
        self.roll_controller = PIDController(
            kp=2.0, ki=0.0, kd=0.1,
            output_min=-1.0, output_max=1.0
        )

        # Pitch rate controller (inner loop) - reduced gains for stability
        self.pitch_controller = PIDController(
            kp=0.5, ki=0.1, kd=0.2,
            output_min=-1.0, output_max=1.0
        )

        # Autothrottle
        at_cfg = al_cfg["autothrottle"]
        self.throttle_controller = PIDController(
            kp=at_cfg["kp"],
            ki=at_cfg["ki"],
            kd=at_cfg["kd"],
            output_min=at_cfg["min_throttle"],
            output_max=at_cfg["max_throttle"]
        )

        # Yaw controller for sideslip
        yaw_cfg = al_cfg["yaw_control"]
        self.yaw_controller = PIDController(
            kp=yaw_cfg["rudder_kp"],
            ki=0.0,
            kd=yaw_cfg["rudder_kd"],
            output_min=-1.0, output_max=1.0
        )

        # Flare parameters
        flare_cfg = al_cfg["flare"]
        self.flare_height_ft = flare_cfg["initiation_height_ft"]
        self.flare_target_sink_fpm = flare_cfg["target_sink_rate_fpm"]
        self.flare_pitch_rate = flare_cfg["pitch_rate_deg_per_sec"]
        self.throttle_retard_height_ft = flare_cfg["throttle_retard_height_ft"]

        # Decrab parameters
        self.decrab_height_ft = yaw_cfg["decrab_height_ft"]
        self.decrab_transition_ft = yaw_cfg["decrab_transition_ft"]
        self.max_sideslip_deg = yaw_cfg["max_sideslip_deg"]

        # Aircraft parameters
        ac_cfg = self.config["aircraft"]
        self.vref_kts = ac_cfg["vref_kts"]
        self.approach_speed_kts = ac_cfg["approach_speed_kts"]

        # Runway parameters
        airport_cfg = self.config["airport"]
        self.runway_heading_rad = airport_cfg["runway_heading_deg"] * DEG_TO_RAD

        # State tracking
        self.flare_start_pitch_deg = None
        self.touchdown_time = None

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, 'r') as f:
            return json.load(f)

    def compute(self, state: AircraftState, dt: float) -> Tuple[float, float, float, float]:
        """
        Compute control commands based on current state.

        Args:
            state: Current aircraft state
            dt: Time step in seconds

        Returns:
            Tuple of (aileron, elevator, rudder, throttle) commands
            Each normalized to appropriate range
        """
        # Phase transitions
        self._update_phase(state)

        if self.phase == AutolandPhase.GO_AROUND:
            return self._go_around_controls(state, dt)
        elif self.phase == AutolandPhase.ROLLOUT:
            return self._rollout_controls(state, dt)
        elif self.phase == AutolandPhase.FLARE:
            return self._flare_controls(state, dt)
        else:
            return self._approach_controls(state, dt)

    def _update_phase(self, state: AircraftState):
        """Update autoland phase based on state."""
        # Check for touchdown
        if state.wow and self.phase != AutolandPhase.ROLLOUT:
            self.phase = AutolandPhase.ROLLOUT
            self.touchdown_time = state.t
            return

        # Check for flare initiation
        if (self.phase == AutolandPhase.APPROACH and
                state.alt_agl_ft <= self.flare_height_ft):
            self.phase = AutolandPhase.FLARE
            self.flare_start_pitch_deg = state.theta_deg
            return

    def _approach_controls(self, state: AircraftState, dt: float
                           ) -> Tuple[float, float, float, float]:
        """
        Normal approach control - track localizer and glideslope.

        CRITICAL: On the "back side of the power curve" (approach):
        - PITCH controls SPEED (pitch down to accelerate, pitch up to slow)
        - THROTTLE controls FLIGHT PATH / ALTITUDE (more power = shallower descent)

        Speed is PRIORITY #1. A slow aircraft will sink no matter what.
        """
        # === LATERAL CONTROL (LOCALIZER) ===
        cte_m = state.cross_track_error_m

        # Desired ground track: runway heading + intercept angle based on CTE
        intercept_angle_deg = -cte_m * 0.3  # 100m → 30° intercept
        intercept_angle_deg = max(-30, min(30, intercept_angle_deg))

        desired_track_rad = self.runway_heading_rad + intercept_angle_deg * math.pi / 180
        track_error = self._normalize_angle(state.track_angle_rad - desired_track_rad)
        track_error_deg = track_error * RAD_TO_DEG

        bank_cmd_deg = -track_error_deg * 1.5
        bank_cmd_deg = max(-self.max_bank_deg, min(self.max_bank_deg, bank_cmd_deg))

        bank_error = bank_cmd_deg - state.phi_deg
        aileron = self.roll_controller.compute(bank_error, dt)

        # === SPEED-PRIORITY VERTICAL CONTROL ===
        #
        # The fundamental insight: An aircraft on approach that gets slow will sink
        # because it needs to increase angle of attack (more drag) to maintain lift.
        # This creates a vicious cycle - slow → high alpha → more drag → slower → stall.
        #
        # Solution: PITCH FOR SPEED, THROTTLE FOR FLIGHT PATH
        # - Too slow? Pitch DOWN to trade altitude for speed (accept going below GS)
        # - Too fast? Pitch UP to slow down
        # - On speed? Use throttle to control sink rate / glideslope tracking

        speed_kts = state.vcas_kts
        gs_error_deg = state.glideslope_error_deg
        sink_rate_fpm = state.vd_fpm  # Positive = descending

        # Target speed with margin above stall
        target_speed_kts = self.approach_speed_kts  # 70 kts from config
        speed_error = target_speed_kts - speed_kts  # Positive = too slow

        # === PITCH CONTROL: Maintain speed ===
        # C172 trimmed approach pitch is about 0° to -2° at 70 kts with partial power
        # We adjust pitch to maintain speed:
        # - Too slow: reduce pitch (nose down) to accelerate
        # - Too fast: increase pitch (nose up) to decelerate

        # Base pitch for 70kt approach (approximately -2° for 3° descent at 70kts)
        BASE_PITCH_DEG = -2.0

        # Speed correction: 1 kt slow = 0.5° more nose down
        # This is aggressive to prevent the slow-sink-stall cycle
        pitch_for_speed = -speed_error * 0.5

        target_pitch_deg = BASE_PITCH_DEG + pitch_for_speed

        # Hard limits to prevent extreme attitudes
        # Never pitch below -10° (excessive dive)
        # Never pitch above +5° (risk of stall on approach)
        target_pitch_deg = max(-10.0, min(5.0, target_pitch_deg))

        # Alpha protection: if alpha is getting high, don't let pitch increase further
        if state.alpha_deg > 10:
            # Approaching stall - reduce pitch aggressively
            target_pitch_deg = min(target_pitch_deg, -5.0)
        elif state.alpha_deg > 8:
            # Getting high - limit pitch
            target_pitch_deg = min(target_pitch_deg, 0.0)

        # Compute elevator command
        pitch_error = target_pitch_deg - state.theta_deg
        elevator = self.pitch_controller.compute(pitch_error, dt)

        # === THROTTLE CONTROL: Control flight path / glideslope ===
        # With speed maintained by pitch, we use throttle to control descent rate
        #
        # Target sink rate for 3° glideslope at 70 kts groundspeed:
        # sink_rate = groundspeed × tan(3°) = 70 × 1.68781 × 60 × 0.0524 = ~370 fpm

        TARGET_SINK_RATE_FPM = 370.0

        # Calculate sink error - positive means sinking too fast
        sink_error = sink_rate_fpm - TARGET_SINK_RATE_FPM

        # Base throttle for C172 in approach config (gear down)
        # This needs to be high enough to maintain shallow descent
        # In testing, 0.55 gave 765 fpm sink, we need less sink = more power
        # Rule of thumb: 60-70% power for level flight, slightly less for descent
        throttle_base = 0.65

        # Sink rate correction: More aggressive to actually control descent
        # 100 fpm too fast = +10% throttle (was 5%)
        throttle_for_sink = sink_error * 0.001

        # Glideslope correction: below GS = more power
        gs_correction = -gs_error_deg * 0.08  # 1° below = +8% throttle

        # Speed deficit adds power
        speed_correction = 0.0
        if speed_kts < target_speed_kts - 2:
            # Add power when slow
            slow_amount = target_speed_kts - speed_kts
            speed_correction = slow_amount * 0.03  # 10 kts slow = +30%

        throttle = throttle_base + throttle_for_sink + gs_correction + speed_correction

        # Clamp throttle
        throttle = max(0.4, min(1.0, throttle))

        # === YAW CONTROL (CRAB TO SIDESLIP TRANSITION) ===
        decrab_progress = self._compute_decrab_progress(state.alt_agl_ft)

        # More aggressive decrab below 50 ft for clear visual alignment
        if state.alt_agl_ft < 50:
            decrab_progress = min(1.0, decrab_progress * 1.5)

        # Coordinated flight (crab): zero sideslip
        crab_rudder = self.yaw_controller.compute(-state.beta_deg, dt)

        # Sideslip: align heading with runway - stronger gain for visible effect
        heading_error = self._normalize_angle(state.psi_rad - self.runway_heading_rad)
        heading_error_deg = heading_error * RAD_TO_DEG
        sideslip_rudder = heading_error_deg * 0.1  # Increased from 0.05
        sideslip_rudder = max(-1.0, min(1.0, sideslip_rudder))

        # Blend crab and sideslip rudder
        rudder = (1.0 - decrab_progress) * crab_rudder + decrab_progress * sideslip_rudder

        # Add yaw rate damper for Dutch roll suppression
        rudder += self._yaw_rate_damper(state.r)
        rudder = max(-1.0, min(1.0, rudder))

        return aileron, elevator, rudder, throttle

    def _flare_controls(self, state: AircraftState, dt: float
                        ) -> Tuple[float, float, float, float]:
        """
        Flare control - reduce sink rate for touchdown.

        Uses exponential pitch-up to arrest descent rate progressively.
        CRITICAL: Must maintain centerline tracking during flare to avoid landing off-runway.
        """
        alt = state.alt_agl_ft
        cte_m = state.cross_track_error_m

        # === LATERAL CONTROL ===
        # Maintain centerline tracking during flare - this is critical!
        # Use stronger corrections than before to prevent drift into water

        # Bank to correct CTE - stronger than approach but still gentle
        bank_cmd_deg = -cte_m * 0.5  # 0.5 deg bank per meter of CTE (was 0.1)
        bank_cmd_deg = max(-10, min(10, bank_cmd_deg))  # Allow up to 10 deg (was 5)

        # Roll control with full authority
        bank_error = bank_cmd_deg - state.phi_deg
        aileron = self.roll_controller.compute(bank_error, dt)  # Full authority (was 0.5)

        # === VERTICAL CONTROL (FLARE) ===
        # Exponential flare: pitch increases as altitude decreases
        # At flare height, pitch ~5 deg. At ground, pitch ~12 deg.
        # This gives an exponential decay in descent rate

        # Height ratio: 1 at flare start, 0 at ground
        h_ratio = max(0.0, min(1.0, alt / self.flare_height_ft))

        # Target pitch: increases as we get lower
        # Exponential curve: starts gentle, gets more aggressive
        target_pitch_deg = 4.0 + 10.0 * (1.0 - h_ratio) ** 1.5  # 4-14 deg

        # Add sink rate feedback to modulate flare pitch
        # Positive vd_fpm means descending (sink)
        # Target is ~200 fpm sink at touchdown
        sink_rate_error = (state.vd_fpm - self.flare_target_sink_fpm) / 500.0  # Normalized
        # If sinking too fast, increase pitch; if too slow, decrease pitch
        # Increased gain for better response in gusty conditions
        target_pitch_deg += sink_rate_error * 5.0  # Was 3.0 - now 5 deg per 500 fpm error

        # Full up elevator command to achieve flare - increased gain for gusty
        pitch_error = target_pitch_deg - state.theta_deg
        elevator = max(-1.0, min(1.0, pitch_error * 0.4))  # Was 0.25

        # === THROTTLE ===
        # Maintain more power to prevent stall in gusty conditions
        throttle = max(0.25, 0.5 * h_ratio)  # Was max(0.15, 0.4 * h_ratio)

        # === RUDDER ===
        # Align with runway AND correct for CTE
        heading_error = self._normalize_angle(state.psi_rad - self.runway_heading_rad)
        heading_error_deg = heading_error * RAD_TO_DEG

        # Add CTE correction to rudder - steer towards centerline
        # If aircraft is right of centerline (CTE > 0), need to turn left (negative rudder)
        cte_rudder = -cte_m * 0.02  # 0.02 rudder per meter of CTE

        rudder = heading_error_deg * 0.1 + cte_rudder
        rudder = max(-1.0, min(1.0, rudder))

        return aileron, elevator, rudder, throttle

    def _rollout_controls(self, state: AircraftState, dt: float
                          ) -> Tuple[float, float, float, float]:
        """
        Rollout control - maintain centerline and decelerate.
        """
        # Throttle to idle
        throttle = 0.0

        # Elevator slightly down for nosewheel authority
        elevator = 0.1

        # Track centerline with rudder - use DEGREES for proper scaling
        heading_error = self._normalize_angle(state.psi_rad - self.runway_heading_rad)
        heading_error_deg = heading_error * RAD_TO_DEG

        # Stronger steering: 10° heading error → full rudder, 10m CTE → 0.5 rudder
        rudder = heading_error_deg * 0.1 + state.cross_track_error_m * 0.05
        rudder = max(-1.0, min(1.0, rudder))

        # Wings level
        aileron = -state.phi_deg * 0.1

        return aileron, elevator, rudder, throttle

    def _go_around_controls(self, state: AircraftState, dt: float
                            ) -> Tuple[float, float, float, float]:
        """
        Go-around control - climb away.
        """
        # Full power
        throttle = 1.0

        # Pitch up to climb attitude
        target_pitch = 10.0
        elevator = self.pitch_controller.compute(target_pitch - state.theta_deg, dt)

        # Wings level
        aileron = self.roll_controller.compute(-state.phi_deg, dt)

        # Coordinated
        rudder = self.yaw_controller.compute(-state.beta_deg, dt)

        return aileron, elevator, rudder, throttle

    def _compute_decrab_progress(self, alt_ft: float) -> float:
        """
        Compute decrab transition progress (0 = full crab, 1 = full sideslip).
        """
        if alt_ft >= self.decrab_height_ft:
            return 0.0
        elif alt_ft <= (self.decrab_height_ft - self.decrab_transition_ft):
            return 1.0
        else:
            return (self.decrab_height_ft - alt_ft) / self.decrab_transition_ft

    def _yaw_rate_damper(self, r_rad_s: float) -> float:
        """
        Simple yaw rate damper for Dutch roll suppression.

        Only damps yaw RATE oscillations - does not try to control sideslip.
        This is the key to preventing Dutch roll without fighting the aircraft's
        natural crab angle in crosswind.

        δ_r = -K_r · r

        Args:
            r_rad_s: Yaw rate in rad/s (positive = nose right)

        Returns:
            Rudder correction (-1 to 1), limited to ±0.3
        """
        K_r = 0.5  # Moderate yaw rate damping gain
        r_deg_s = r_rad_s * RAD_TO_DEG

        # Compute damping rudder: oppose yaw rate
        damper_rudder = -K_r * r_deg_s / 10.0

        # Limit contribution to ±0.3 to prevent overpowering primary control
        return max(-0.3, min(0.3, damper_rudder))

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to -pi to pi."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def trigger_go_around(self):
        """Trigger go-around mode."""
        self.phase = AutolandPhase.GO_AROUND

    def get_phase(self) -> AutolandPhase:
        """Get current autoland phase."""
        return self.phase

    def reset(self):
        """Reset controller state."""
        self.phase = AutolandPhase.APPROACH
        self.flare_start_pitch_deg = None
        self.touchdown_time = None
        self.loc_controller.reset()
        self.gs_controller.reset()
        self.roll_controller.reset()
        self.pitch_controller.reset()
        self.throttle_controller.reset()
        self.yaw_controller.reset()
