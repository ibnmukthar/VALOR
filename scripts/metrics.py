"""
VALOR - Flight Metrics for Research Evaluation

Computes RMS lateral, RMS vertical, and max speed error as per research proposal.

Metrics computed:
- RMS_lat = √(1/T ∫ y(t)² dt)  - Lateral tracking accuracy (cross-track error)
- RMS_vert = √(1/T ∫ (γ-γ_ref)² dt) - Glidepath tracking accuracy
- ΔV_max = max|V(t) - V_ref| - Energy stability (max speed deviation)
"""

import math
from dataclasses import dataclass, field
from typing import Optional, List


@dataclass
class FlightMetrics:
    """
    Research metrics for autoland evaluation.

    These metrics quantify the controller's ability to:
    1. Track the runway centerline (lateral)
    2. Track the glideslope (vertical)
    3. Maintain stable airspeed (energy management)
    """

    v_ref_kts: float = 70.0       # Reference approach speed
    gamma_ref_deg: float = -3.0   # Reference glidepath (3° descent = -3°)

    # Accumulators for RMS calculations
    lateral_sq_sum: float = 0.0   # Sum of CTE² × dt
    vertical_sq_sum: float = 0.0  # Sum of γ_error² × dt
    max_speed_error_kts: float = 0.0  # Maximum |V - V_ref|

    # Time tracking
    total_time: float = 0.0
    sample_count: int = 0

    # Touchdown metrics
    touchdown_cte_m: Optional[float] = None
    touchdown_sink_fpm: Optional[float] = None
    touchdown_speed_kts: Optional[float] = None

    # Phase tracking
    flare_initiated: bool = False
    flare_time: Optional[float] = None

    def update(self, cte_m: float, gamma_deg: float, vcas_kts: float, dt: float):
        """
        Update metrics with current state.

        Args:
            cte_m: Cross-track error in meters (+ = right of centerline)
            gamma_deg: Flight path angle in degrees (- = descending)
            vcas_kts: Calibrated airspeed in knots
            dt: Time step in seconds
        """
        if dt <= 0:
            return

        # Lateral: cross-track error squared, time-weighted
        self.lateral_sq_sum += cte_m ** 2 * dt

        # Vertical: flight path angle error squared, time-weighted
        gamma_error = gamma_deg - self.gamma_ref_deg
        self.vertical_sq_sum += gamma_error ** 2 * dt

        # Speed: track maximum deviation from reference
        speed_error = abs(vcas_kts - self.v_ref_kts)
        self.max_speed_error_kts = max(self.max_speed_error_kts, speed_error)

        self.total_time += dt
        self.sample_count += 1

    def record_touchdown(self, cte_m: float, sink_fpm: float, speed_kts: float):
        """
        Record touchdown conditions.

        Args:
            cte_m: Cross-track error at touchdown
            sink_fpm: Sink rate at touchdown (positive = descending)
            speed_kts: Groundspeed at touchdown
        """
        self.touchdown_cte_m = cte_m
        self.touchdown_sink_fpm = sink_fpm
        self.touchdown_speed_kts = speed_kts

    def record_flare(self, time: float):
        """Record when flare was initiated."""
        if not self.flare_initiated:
            self.flare_initiated = True
            self.flare_time = time

    @property
    def rms_lateral_m(self) -> float:
        """
        RMS lateral deviation (meters).

        RMS_lat = √(1/T ∫ y(t)² dt)

        Lower is better. Typical targets:
        - CAT I: < 10m
        - CAT II: < 5m
        - CAT III: < 3m
        """
        if self.total_time <= 0:
            return 0.0
        return math.sqrt(self.lateral_sq_sum / self.total_time)

    @property
    def rms_vertical_deg(self) -> float:
        """
        RMS vertical (glidepath) error (degrees).

        RMS_vert = √(1/T ∫ (γ-γ_ref)² dt)

        Lower is better. Typical targets:
        - CAT I: < 0.5°
        - CAT II: < 0.3°
        - CAT III: < 0.2°
        """
        if self.total_time <= 0:
            return 0.0
        return math.sqrt(self.vertical_sq_sum / self.total_time)

    def summary(self) -> str:
        """Return formatted summary string for console output."""
        lines = [
            "=" * 50,
            "VALOR Flight Metrics Summary",
            "=" * 50,
            "",
            "Tracking Performance:",
            f"  RMS Lateral Error:    {self.rms_lateral_m:7.2f} m",
            f"  RMS Vertical Error:   {self.rms_vertical_deg:7.3f}°",
            f"  Max Speed Deviation:  {self.max_speed_error_kts:7.1f} kts",
            "",
        ]

        if self.touchdown_cte_m is not None:
            lines.extend([
                "Touchdown Conditions:",
                f"  Cross-track Error:    {self.touchdown_cte_m:7.2f} m",
                f"  Sink Rate:            {self.touchdown_sink_fpm:7.1f} fpm",
                f"  Speed:                {self.touchdown_speed_kts:7.1f} kts",
                "",
            ])

        lines.extend([
            "Statistics:",
            f"  Total Time:           {self.total_time:7.1f} s",
            f"  Sample Count:         {self.sample_count:7d}",
            "=" * 50,
        ])

        return "\n".join(lines)

    def to_dict(self) -> dict:
        """Export metrics as dictionary for logging/serialization."""
        return {
            "rms_lateral_m": self.rms_lateral_m,
            "rms_vertical_deg": self.rms_vertical_deg,
            "max_speed_error_kts": self.max_speed_error_kts,
            "touchdown_cte_m": self.touchdown_cte_m,
            "touchdown_sink_fpm": self.touchdown_sink_fpm,
            "touchdown_speed_kts": self.touchdown_speed_kts,
            "total_time_s": self.total_time,
            "sample_count": self.sample_count,
            "v_ref_kts": self.v_ref_kts,
            "gamma_ref_deg": self.gamma_ref_deg,
        }

    def reset(self):
        """Reset all accumulators for a new run."""
        self.lateral_sq_sum = 0.0
        self.vertical_sq_sum = 0.0
        self.max_speed_error_kts = 0.0
        self.total_time = 0.0
        self.sample_count = 0
        self.touchdown_cte_m = None
        self.touchdown_sink_fpm = None
        self.touchdown_speed_kts = None
        self.flare_initiated = False
        self.flare_time = None
