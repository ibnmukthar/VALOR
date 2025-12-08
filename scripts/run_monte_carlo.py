#!/usr/bin/env python3
"""
Monte Carlo simulation runner for VALOR touchdown dispersion analysis.

Runs multiple simulations with different turbulence seeds across scenarios
to generate data for the touchdown dispersion figure (Fig. Appendix).

Usage:
    python run_monte_carlo.py                    # Run all scenarios
    python run_monte_carlo.py --scenario severe  # Run single scenario
    python run_monte_carlo.py --runs 20          # 20 runs per scenario
"""

import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Any

# Add scripts directory to path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from main import SimulationRunner, configure_scenario


# Output directory
MONTE_CARLO_DIR = ROOT / "results" / "monte_carlo"


def run_single_simulation(config_path: str, scenario: str, seed: int) -> Dict[str, Any]:
    """
    Run a single simulation with specified scenario and seed.

    Returns dict with results or None if simulation failed.
    """
    # Load and modify config with seed
    with open(config_path, 'r') as f:
        config = json.load(f)

    config["wind"]["turbulence"]["seed"] = seed

    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)

    # Configure scenario (this also modifies config)
    configure_scenario(config_path, scenario)

    # Re-add seed after configure_scenario overwrites turbulence settings
    with open(config_path, 'r') as f:
        config = json.load(f)
    config["wind"]["turbulence"]["seed"] = seed
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)

    try:
        # Create runner without FlightGear
        runner = SimulationRunner(config_path, use_flightgear=False)

        if not runner.initialize():
            return None

        results = runner.run()
        runner.cleanup()

        return results

    except Exception as e:
        print(f"  ERROR: {e}")
        return None


def run_monte_carlo(
    scenarios: List[str],
    runs_per_scenario: int = 10,
    base_seed: int = 1000
) -> List[Dict[str, Any]]:
    """
    Run Monte Carlo simulations across scenarios.

    Args:
        scenarios: List of scenario names
        runs_per_scenario: Number of runs per scenario
        base_seed: Starting seed value

    Returns:
        List of result dictionaries
    """
    config_path = str(ROOT / "data" / "config.json")
    all_results = []

    total_runs = len(scenarios) * runs_per_scenario
    run_num = 0

    print("=" * 60)
    print("VALOR Monte Carlo Simulation")
    print("=" * 60)
    print(f"Scenarios: {scenarios}")
    print(f"Runs per scenario: {runs_per_scenario}")
    print(f"Total runs: {total_runs}")
    print("=" * 60)

    start_time = time.time()

    for scenario in scenarios:
        print(f"\n>>> Scenario: {scenario.upper()}")
        print("-" * 40)

        for i in range(runs_per_scenario):
            run_num += 1
            seed = base_seed + i

            print(f"  Run {i+1}/{runs_per_scenario} (seed={seed})...", end=" ", flush=True)

            results = run_single_simulation(config_path, scenario, seed)

            if results is None:
                print("FAILED")
                continue

            # Extract key metrics
            metrics = results.get("metrics", {})
            touchdown = results.get("touchdown", {})

            record = {
                "scenario": scenario,
                "seed": seed,
                "success": results.get("success", False),
                "touchdown_cte_m": touchdown.get("cross_track_error_m", float("nan")),
                "touchdown_sink_fpm": touchdown.get("sink_rate_fpm", float("nan")),
                "touchdown_speed_kts": touchdown.get("airspeed_kts", float("nan")),
                "rms_lateral_m": metrics.get("rms_lateral_m", float("nan")),
                "rms_vertical_deg": metrics.get("rms_vertical_deg", float("nan")),
                "max_speed_error_kts": metrics.get("max_speed_error_kts", float("nan")),
            }

            all_results.append(record)

            status = "OK" if record["success"] else "incomplete"
            cte = record["touchdown_cte_m"]
            print(f"{status}, CTE={cte:.1f}m" if not pd.isna(cte) else status)

    elapsed = time.time() - start_time
    print("\n" + "=" * 60)
    print(f"Monte Carlo complete: {len(all_results)}/{total_runs} successful")
    print(f"Total time: {elapsed:.1f}s ({elapsed/total_runs:.2f}s per run)")
    print("=" * 60)

    return all_results


def save_results(results: List[Dict[str, Any]], output_path: Path):
    """Save results to CSV."""
    if not results:
        print("No results to save")
        return

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    print(f"Results saved to: {output_path}")


def main():
    """Main entry point."""
    # Import pandas here to check availability
    global pd
    try:
        import pandas as pd
    except ImportError:
        import math
        class FakePD:
            @staticmethod
            def isna(x):
                return x != x  # NaN check
        pd = FakePD()

    parser = argparse.ArgumentParser(
        description="VALOR Monte Carlo simulation runner"
    )
    parser.add_argument(
        "--scenario", "-s",
        choices=["calm", "moderate", "gusty", "severe", "all"],
        default="all",
        help="Scenario to run (default: all)"
    )
    parser.add_argument(
        "--runs", "-r",
        type=int,
        default=10,
        help="Number of runs per scenario (default: 10)"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=1000,
        help="Base seed value (default: 1000)"
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default=MONTE_CARLO_DIR / "touchdown_summary.csv",
        help="Output CSV path"
    )
    args = parser.parse_args()

    # Determine scenarios
    if args.scenario == "all":
        scenarios = ["calm", "moderate", "gusty", "severe"]
    else:
        scenarios = [args.scenario]

    # Run Monte Carlo
    results = run_monte_carlo(
        scenarios=scenarios,
        runs_per_scenario=args.runs,
        base_seed=args.seed
    )

    # Save results
    save_results(results, args.output)

    return 0 if results else 1


if __name__ == "__main__":
    sys.exit(main())
