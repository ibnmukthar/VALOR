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

        # Pitch rate controller (inner loop) - increased gains for flare authority
        self.pitch_controller = PIDController(
            kp=2.5, ki=0.5, kd=0.1,
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

        Uses crab angle to maintain ground track.
        """
        # === LATERAL CONTROL (LOCALIZER) ===
        # Use GROUND TRACK (not heading) for wind-corrected guidance
        cte_m = state.cross_track_error_m

        # Desired ground track: runway heading + intercept angle based on CTE
        # Positive CTE (right of centerline) → need track LEFT of runway
        # Max intercept angle: 30 degrees at 100m offset
        intercept_angle_deg = -cte_m * 0.3  # 100m → 30° intercept
        intercept_angle_deg = max(-30, min(30, intercept_angle_deg))

        desired_track_rad = self.runway_heading_rad + intercept_angle_deg * math.pi / 180

        # Track error: current ground track vs desired track
        # This automatically accounts for wind - we control where we're GOING not where we're POINTING
        track_error = self._normalize_angle(state.track_angle_rad - desired_track_rad)
        track_error_deg = track_error * RAD_TO_DEG

        # Bank command proportional to track error
        # 10° track error → 15° bank
        bank_cmd_deg = -track_error_deg * 1.5

        # Limit bank angle
        bank_cmd_deg = max(-self.max_bank_deg, min(self.max_bank_deg, bank_cmd_deg))

        # Inner loop: bank angle to aileron command
        bank_error = bank_cmd_deg - state.phi_deg
        aileron = self.roll_controller.compute(bank_error, dt)

        # === VERTICAL CONTROL (GLIDESLOPE) ===
        # Glideslope error to pitch command
        gs_error_deg = state.glideslope_error_deg
        pitch_cmd_deg = self.gs_controller.compute(-gs_error_deg, dt)

        # Add nominal approach pitch
        target_pitch_deg = 2.0 + pitch_cmd_deg  # 2 deg nose up baseline

        # Inner loop: pitch to elevator command
        pitch_error = target_pitch_deg - state.theta_deg
        elevator = self.pitch_controller.compute(pitch_error, dt)

        # === AUTOTHROTTLE ===
        speed_error = self.approach_speed_kts - state.vcas_kts
        throttle_base = 0.55  # Nominal approach throttle (gear and flaps down)
        throttle_correction = self.throttle_controller.compute(speed_error, dt)
        # More aggressive throttle response
        throttle_correction *= 2.0
        throttle = max(0.2, min(1.0, throttle_base + throttle_correction))

        # === YAW CONTROL (CRAB) ===
        # During approach, use crab - keep wings level, let heading differ from track
        # Rudder primarily for coordinated turns
        # Use beta (sideslip) to zero for coordinated flight
        rudder = self.yaw_controller.compute(-state.beta_deg, dt)

        return aileron, elevator, rudder, throttle

    def _flare_controls(self, state: AircraftState, dt: float
                        ) -> Tuple[float, float, float, float]:
        """
        Flare control - reduce sink rate for touchdown.

        Uses exponential pitch-up to arrest descent rate progressively.
        """
        alt = state.alt_agl_ft

        # === LATERAL CONTROL ===
        # Keep wings level during flare - gentle corrections only
        bank_cmd_deg = -state.cross_track_error_m * 0.1  # Very gentle
        bank_cmd_deg = max(-5, min(5, bank_cmd_deg))  # Strict limit

        # Roll towards wings level
        bank_error = bank_cmd_deg - state.phi_deg
        aileron = self.roll_controller.compute(bank_error * 0.5, dt)  # Reduced authority

        # === VERTICAL CONTROL (FLARE) ===
        # Exponential flare: pitch increases as altitude decreases
        # At flare height, pitch ~5 deg. At ground, pitch ~12 deg.
        # This gives an exponential decay in descent rate

        # Height ratio: 1 at flare start, 0 at ground
        h_ratio = max(0.0, min(1.0, alt / self.flare_height_ft))

        # Target pitch: increases as we get lower
        # Exponential curve: starts gentle, gets more aggressive
        target_pitch_deg = 4.0 + 10.0 * (1.0 - h_ratio) ** 1.5  # 4-14 deg

        # Full up elevator command to achieve flare
        pitch_error = target_pitch_deg - state.theta_deg
        elevator = max(-1.0, min(1.0, pitch_error * 0.25))  # High gain

        # === THROTTLE ===
        # Maintain some power to prevent stall, reduce progressively
        throttle = max(0.15, 0.4 * h_ratio)  # 40% at start, 15% at ground

        # === RUDDER ===
        # Align with runway (decrab)
        heading_error = self._normalize_angle(state.psi_rad - self.runway_heading_rad)
        heading_error_deg = heading_error * RAD_TO_DEG
        rudder = heading_error_deg * 0.03  # Gentle rudder
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

        # Track centerline with rudder
        heading_error = self._normalize_angle(state.psi_rad - self.runway_heading_rad)
        cte_correction = state.cross_track_error_m * 0.01
        rudder = heading_error * 2.0 + cte_correction
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
