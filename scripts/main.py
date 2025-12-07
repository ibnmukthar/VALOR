#!/usr/bin/env python3
"""
VALOR - Vector-based Autonomous Landing & Orientation Regulator
Main simulation runner for crosswind autoland demonstration.

Usage:
    python main.py                    # Run with defaults
    python main.py --no-flightgear    # Run without FlightGear visualization
    python main.py --scenario gusty   # Run specific wind scenario
"""

import argparse
import json
import math
import time
import csv
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any

from simulation import JSBSimEngine, AircraftState, FT_TO_M, M_TO_FT, KTS_TO_MPS
from controller import AutolandController, AutolandPhase
from wind import WindEnvironment, WindShearDetector, WindShearType
from flightgear import FGNetFDM, create_flightgear_sender


class SimulationRunner:
    """
    Main simulation orchestrator.

    Coordinates JSBSim, controller, wind environment, and FlightGear visualization.
    """

    def __init__(self, config_path: str = "data/config.json",
                 use_flightgear: bool = True):
        self.config_path = config_path
        self.config = self._load_config()

        # Core components
        self.sim = JSBSimEngine(config_path)
        self.controller = AutolandController(config_path)
        self.wind_env = WindEnvironment(config_path)
        self.shear_detector = WindShearDetector(config_path)

        # FlightGear visualization
        self.use_flightgear = use_flightgear
        self.fg_sender: Optional[FGNetFDM] = None

        # Simulation parameters
        self.dt = self.config["simulation"]["dt_sec"]
        self.max_duration = self.config["simulation"]["max_duration_sec"]

        # Logging
        self.log_data = []
        self.log_rate_hz = self.config["logging"]["log_rate_hz"]

        # FG send rate tracking
        fg_cfg = self.config.get("flightgear", {})
        self.fg_rate_hz = fg_cfg.get("rate_hz", 50)
        self.fg_interval = 1.0 / self.fg_rate_hz
        self.last_fg_send_time = 0.0

    def _load_config(self) -> dict:
        with open(self.config_path, 'r') as f:
            return json.load(f)

    def initialize(self) -> bool:
        """Initialize all simulation components."""
        print("=" * 60)
        print("VALOR - Crosswind Autoland Simulation")
        print("=" * 60)

        # Initialize JSBSim
        if not self.sim.initialize():
            print("ERROR: Failed to initialize JSBSim")
            return False

        # Initialize FlightGear connection
        if self.use_flightgear:
            try:
                self.fg_sender = create_flightgear_sender(self.config_path)
                self.fg_sender.connect()
            except Exception as e:
                print(f"WARNING: FlightGear connection failed: {e}")
                print("Continuing without visualization...")
                self.fg_sender = None

        # Print wind conditions
        initial_wind = self.wind_env.get_wind(
            altitude_agl_m=self.sim.state.alt_agl_ft * FT_TO_M,
            airspeed_mps=self.sim.state.vtrue_fps * FT_TO_M
        )
        crosswind_kts = abs(self.wind_env.get_crosswind_component(initial_wind)) / KTS_TO_MPS
        headwind_kts = self.wind_env.get_headwind_component(initial_wind) / KTS_TO_MPS

        print(f"\nWind conditions:")
        print(f"  Speed: {initial_wind.speed_kts:.0f} kts from {initial_wind.direction_deg:.0f}°")
        print(f"  Crosswind: {crosswind_kts:.0f} kts")
        print(f"  Headwind: {headwind_kts:.0f} kts")
        print(f"\nAutoland controller active. Starting approach...")
        print("-" * 60)

        return True

    def run(self) -> Dict[str, Any]:
        """
        Run the complete simulation.

        Returns:
            Dictionary with simulation results
        """
        start_time = time.time()
        sim_start_time = 0.0

        state = self.sim.state
        touchdown_state: Optional[AircraftState] = None
        final_phase = AutolandPhase.APPROACH
        go_around_triggered = False

        step_count = 0
        log_interval = int(1.0 / (self.log_rate_hz * self.dt))

        # Real-time pacing for FlightGear visualization
        realtime_enabled = self.use_flightgear and self.fg_sender is not None
        realtime_factor = 1.0  # 1.0 = real-time, 2.0 = 2x speed
        last_realtime_check = time.time()
        last_sim_time_check = 0.0

        print(f"{'Time':>6} {'Alt':>6} {'Speed':>6} {'Phase':>12} {'CTE':>8} {'Status'}")
        print("-" * 60)
        if realtime_enabled:
            print("(Running in real-time for FlightGear visualization)")
        sys.stdout.flush()

        while state.t < self.max_duration:
            # Get current wind
            wind = self.wind_env.get_wind(
                altitude_agl_m=state.alt_agl_ft * FT_TO_M,
                x_m=state.distance_to_threshold_m,
                y_m=state.cross_track_error_m,
                airspeed_mps=state.vtrue_fps * FT_TO_M,
                dt=self.dt
            )

            # Set wind in simulation
            self.sim.set_wind(
                wind_north_fps=wind.north * M_TO_FT,
                wind_east_fps=wind.east * M_TO_FT,
                wind_down_fps=wind.down * M_TO_FT
            )

            # Check for wind shear
            shear_alert = self.shear_detector.update(
                t=state.t,
                airspeed_kts=state.vcas_kts,
                sink_rate_fpm=state.vd_fpm
            )

            # Go-around on severe shear
            go_around_cfg = self.config.get("wind_shear", {}).get("go_around", {})
            if (go_around_cfg.get("enabled", False) and
                    shear_alert.detected and
                    shear_alert.severity > 0.7 and
                    state.alt_agl_ft > go_around_cfg.get("min_altitude_ft", 200)):
                self.controller.trigger_go_around()
                go_around_triggered = True
                print(f"\n*** WIND SHEAR ALERT: {shear_alert.shear_type.value} ***")
                print(f"    Airspeed change: {shear_alert.airspeed_change_kts:+.0f} kts")
                print(f"    Triggering go-around!")

            # Compute control commands
            aileron, elevator, rudder, throttle = self.controller.compute(state, self.dt)

            # Advance simulation
            state = self.sim.step(aileron, elevator, rudder, throttle)

            # Send to FlightGear at reduced rate
            if self.fg_sender and (state.t - self.last_fg_send_time) >= self.fg_interval:
                try:
                    self.fg_sender.send(state)
                    self.last_fg_send_time = state.t
                except Exception as e:
                    pass  # Silently ignore FG errors

            # Real-time pacing - sync simulation time to wall clock
            if realtime_enabled:
                sim_elapsed = state.t - last_sim_time_check
                if sim_elapsed >= 0.1:  # Check every 0.1 sim seconds
                    wall_elapsed = time.time() - last_realtime_check
                    target_wall_time = sim_elapsed / realtime_factor
                    sleep_time = target_wall_time - wall_elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    last_realtime_check = time.time()
                    last_sim_time_check = state.t

            # Log data
            if step_count % log_interval == 0:
                self._log_state(state, wind, self.controller.get_phase())

            # Print status periodically
            if step_count % int(2.0 / self.dt) == 0:  # Every 2 seconds
                phase = self.controller.get_phase()
                status = ""
                if shear_alert.detected:
                    status = f"SHEAR: {shear_alert.shear_type.value}"

                print(f"{state.t:6.1f} {state.alt_agl_ft:6.0f} "
                      f"{state.vcas_kts:6.0f} {phase.name:>12} "
                      f"{state.cross_track_error_m:+8.1f} {status}")
                sys.stdout.flush()

            # Check termination conditions
            final_phase = self.controller.get_phase()

            if state.wow and not touchdown_state:
                touchdown_state = AircraftState(**state.__dict__)
                print(f"\n*** TOUCHDOWN at t={state.t:.1f}s ***")
                print(f"    Position: {state.lat_deg:.6f}°N, {state.lon_deg:.6f}°W")
                print(f"    Sink rate: {state.vd_fpm:.0f} fpm")
                print(f"    CTE: {state.cross_track_error_m:.1f} m")
                print(f"    Heading: {state.psi_deg:.1f}°")

            if final_phase == AutolandPhase.ROLLOUT and state.vcas_kts < 20:
                print("\n*** Rollout complete - aircraft stopped ***")
                break

            if final_phase == AutolandPhase.GO_AROUND and state.alt_agl_ft > 500:
                print("\n*** Go-around climb established ***")
                break

            if self.sim.is_crashed():
                print("\n*** CRASH DETECTED ***")
                break

            step_count += 1

        # Simulation complete
        elapsed = time.time() - start_time
        print("-" * 60)
        print(f"Simulation complete. Wall time: {elapsed:.1f}s, Sim time: {state.t:.1f}s")

        # Save log data
        self._save_log()

        # Compile results
        results = {
            "success": touchdown_state is not None and final_phase == AutolandPhase.ROLLOUT,
            "go_around": go_around_triggered,
            "final_phase": final_phase.name,
            "sim_time_sec": state.t,
            "wall_time_sec": elapsed,
        }

        if touchdown_state:
            results["touchdown"] = {
                "time_sec": touchdown_state.t,
                "lat_deg": touchdown_state.lat_deg,
                "lon_deg": touchdown_state.lon_deg,
                "sink_rate_fpm": touchdown_state.vd_fpm,
                "cross_track_error_m": touchdown_state.cross_track_error_m,
                "heading_deg": touchdown_state.psi_deg,
                "airspeed_kts": touchdown_state.vcas_kts,
            }

        return results

    def _log_state(self, state: AircraftState, wind, phase: AutolandPhase):
        """Record state for logging."""
        self.log_data.append({
            "t": state.t,
            "lat_deg": state.lat_deg,
            "lon_deg": state.lon_deg,
            "alt_agl_ft": state.alt_agl_ft,
            "alt_msl_ft": state.alt_msl_ft,
            "phi_deg": state.phi_deg,
            "theta_deg": state.theta_deg,
            "psi_deg": state.psi_deg,
            "vcas_kts": state.vcas_kts,
            "vtrue_kts": state.vtrue_kts,
            "alpha_deg": state.alpha_deg,
            "beta_deg": state.beta_deg,
            "gamma_deg": state.gamma_deg,
            "vd_fpm": state.vd_fpm,
            "cte_m": state.cross_track_error_m,
            "gs_error_deg": state.glideslope_error_deg,
            "dist_m": state.distance_to_threshold_m,
            "aileron": state.aileron,
            "elevator": state.elevator,
            "rudder": state.rudder,
            "throttle": state.throttle,
            "wind_n_mps": wind.north,
            "wind_e_mps": wind.east,
            "wind_d_mps": wind.down,
            "wind_speed_kts": wind.speed_kts,
            "phase": phase.name,
            "wow": int(state.wow),
        })

    def _save_log(self):
        """Save logged data to CSV."""
        if not self.log_data:
            return

        output_dir = Path(self.config["logging"]["output_dir"])
        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = output_dir / f"autoland_{timestamp}.csv"

        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.log_data[0].keys())
            writer.writeheader()
            writer.writerows(self.log_data)

        print(f"\nLog saved to: {filename}")

    def cleanup(self):
        """Clean up resources."""
        if self.fg_sender:
            self.fg_sender.close()


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="VALOR - Crosswind Autoland Simulation"
    )
    parser.add_argument(
        "--no-flightgear", "-n",
        action="store_true",
        help="Run without FlightGear visualization"
    )
    parser.add_argument(
        "--config", "-c",
        default="data/config.json",
        help="Path to configuration file"
    )
    parser.add_argument(
        "--scenario", "-s",
        choices=["calm", "moderate", "gusty", "severe", "microburst"],
        default="moderate",
        help="Wind scenario to simulate"
    )
    return parser.parse_args()


def configure_scenario(config_path: str, scenario: str):
    """Modify config for specific wind scenario."""
    with open(config_path, 'r') as f:
        config = json.load(f)

    if scenario == "calm":
        config["wind"]["base_speed_kts"] = 5
        config["wind"]["turbulence"]["enabled"] = False
        config["wind"]["microburst"]["enabled"] = False

    elif scenario == "moderate":
        config["wind"]["base_speed_kts"] = 15
        config["wind"]["base_direction_deg"] = 250  # 30° crosswind
        config["wind"]["turbulence"]["enabled"] = False
        config["wind"]["microburst"]["enabled"] = False

    elif scenario == "gusty":
        config["wind"]["base_speed_kts"] = 20
        config["wind"]["base_direction_deg"] = 240  # 40° crosswind
        config["wind"]["turbulence"]["enabled"] = True
        config["wind"]["turbulence"]["sigma_u_mps"] = 4.0
        config["wind"]["turbulence"]["sigma_v_mps"] = 4.0
        config["wind"]["turbulence"]["sigma_w_mps"] = 2.5
        config["wind"]["microburst"]["enabled"] = False

    elif scenario == "severe":
        config["wind"]["base_speed_kts"] = 25
        config["wind"]["base_direction_deg"] = 230  # Near max crosswind
        config["wind"]["turbulence"]["enabled"] = True
        config["wind"]["turbulence"]["sigma_u_mps"] = 5.0
        config["wind"]["turbulence"]["sigma_v_mps"] = 5.0
        config["wind"]["turbulence"]["sigma_w_mps"] = 3.0
        config["wind"]["microburst"]["enabled"] = False

    elif scenario == "microburst":
        config["wind"]["base_speed_kts"] = 15
        config["wind"]["turbulence"]["enabled"] = True
        config["wind"]["microburst"]["enabled"] = True
        config["wind"]["microburst"]["max_downdraft_fpm"] = 1500
        config["wind"]["microburst"]["max_headwind_change_kts"] = 30

    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)

    print(f"Configured scenario: {scenario}")


def main():
    """Main entry point."""
    args = parse_args()

    # Configure wind scenario
    configure_scenario(args.config, args.scenario)

    # Create and run simulation
    runner = SimulationRunner(
        config_path=args.config,
        use_flightgear=not args.no_flightgear
    )

    try:
        if not runner.initialize():
            print("Initialization failed")
            return 1

        results = runner.run()

        # Print summary
        print("\n" + "=" * 60)
        print("SIMULATION RESULTS")
        print("=" * 60)
        print(f"Outcome: {'SUCCESS' if results['success'] else 'INCOMPLETE'}")
        print(f"Final phase: {results['final_phase']}")

        if "touchdown" in results:
            td = results["touchdown"]
            print(f"\nTouchdown metrics:")
            print(f"  Time: {td['time_sec']:.1f} seconds")
            print(f"  Sink rate: {td['sink_rate_fpm']:.0f} ft/min")
            print(f"  Lateral deviation: {td['cross_track_error_m']:.1f} m")
            print(f"  Heading: {td['heading_deg']:.1f}°")
            print(f"  Speed: {td['airspeed_kts']:.0f} kts")

        if results.get('go_around'):
            print("\n*** GO-AROUND WAS TRIGGERED ***")

        return 0 if results['success'] else 1

    finally:
        runner.cleanup()


if __name__ == "__main__":
    exit(main())
