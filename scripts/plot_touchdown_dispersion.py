#!/usr/bin/env python3
"""
Generate Fig. Appendix (fig_touchdown_dispersion.pdf) for the VALOR report.

Creates a two-panel figure showing touchdown dispersion across scenarios:
- Top panel: Scatter plot of individual touchdown CTE values
- Bottom panel: Bar chart of mean touchdown CTE ± std dev per scenario

Usage:
    python plot_touchdown_dispersion.py                              # Use default CSV
    python plot_touchdown_dispersion.py --csv path/to/summary.csv    # Custom CSV
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# Directories
ROOT = Path(__file__).resolve().parents[1]
MONTE_CARLO_DIR = ROOT / "results" / "monte_carlo"
FIGS_DIR = ROOT / "results" / "figs"

# Scenario display order and colors
SCENARIO_ORDER = ["calm", "moderate", "gusty", "severe"]
SCENARIO_LABELS = {
    "calm": "Calm",
    "moderate": "Moderate",
    "gusty": "Gusty",
    "severe": "Severe"
}
SCENARIO_COLORS = {
    "calm": "#2ecc71",      # Green
    "moderate": "#3498db",  # Blue
    "gusty": "#f39c12",     # Orange
    "severe": "#e74c3c",    # Red
}


def load_data(csv_path: Path) -> pd.DataFrame:
    """Load Monte Carlo results from CSV."""
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} records from {csv_path.name}")

    # Filter to successful runs only
    df_success = df[df['success'] == True].copy()
    print(f"  Successful runs: {len(df_success)}")

    return df_success


def compute_statistics(df: pd.DataFrame) -> pd.DataFrame:
    """Compute per-scenario statistics."""
    stats = []
    for scenario in SCENARIO_ORDER:
        scenario_data = df[df['scenario'] == scenario]
        if len(scenario_data) == 0:
            continue

        cte = scenario_data['touchdown_cte_m'].values
        rms_lat = scenario_data['rms_lateral_m'].values

        stats.append({
            "scenario": scenario,
            "n_runs": len(scenario_data),
            "cte_mean": np.mean(np.abs(cte)),
            "cte_std": np.std(cte),
            "cte_min": np.min(cte),
            "cte_max": np.max(cte),
            "rms_lat_mean": np.mean(rms_lat),
            "rms_lat_std": np.std(rms_lat),
        })

    return pd.DataFrame(stats)


def plot_touchdown_dispersion(df: pd.DataFrame, output_path: Path):
    """
    Generate the two-panel touchdown dispersion figure.

    Top panel: Scatter/strip plot of individual touchdown CTE values
    Bottom panel: Bar chart of mean |CTE| ± std per scenario
    """
    # Compute statistics
    stats = compute_statistics(df)
    print("\nPer-scenario statistics:")
    print(stats.to_string(index=False))

    # Create figure
    fig, axes = plt.subplots(2, 1, figsize=(8, 8),
                              gridspec_kw={'height_ratios': [2, 1]})

    # =========================================================================
    # Top Panel: Scatter/Strip Plot
    # =========================================================================
    ax1 = axes[0]

    # Plot individual points for each scenario with jitter
    for i, scenario in enumerate(SCENARIO_ORDER):
        scenario_data = df[df['scenario'] == scenario]
        if len(scenario_data) == 0:
            continue

        cte = scenario_data['touchdown_cte_m'].values
        color = SCENARIO_COLORS[scenario]

        # Add horizontal jitter for visibility
        jitter = np.random.uniform(-0.15, 0.15, size=len(cte))
        x_positions = np.full_like(cte, i) + jitter

        ax1.scatter(x_positions, cte, c=color, s=60, alpha=0.7,
                    edgecolors='white', linewidths=0.5,
                    label=SCENARIO_LABELS[scenario])

        # Add mean marker
        mean_cte = np.mean(cte)
        ax1.scatter([i], [mean_cte], c=color, s=150, marker='D',
                    edgecolors='black', linewidths=1.5, zorder=5)

    # Formatting
    ax1.axhline(y=0, color='gray', linestyle='--', linewidth=0.8, alpha=0.5)
    ax1.set_xticks(range(len(SCENARIO_ORDER)))
    ax1.set_xticklabels([SCENARIO_LABELS[s] for s in SCENARIO_ORDER], fontsize=11)
    ax1.set_ylabel('Touchdown Cross-track Error [m]', fontsize=11)
    ax1.set_title('Touchdown Dispersion Across Wind Scenarios\n(10 Monte Carlo runs per scenario)',
                  fontsize=12, fontweight='bold')
    ax1.grid(True, axis='y', alpha=0.3)

    # Add legend explaining diamond = mean
    diamond = plt.Line2D([0], [0], marker='D', color='gray', markersize=8,
                         markerfacecolor='gray', markeredgecolor='black',
                         linestyle='None', label='Mean')
    circle = plt.Line2D([0], [0], marker='o', color='gray', markersize=8,
                        markerfacecolor='gray', markeredgecolor='white',
                        linestyle='None', label='Individual runs')
    ax1.legend(handles=[circle, diamond], loc='upper left', fontsize=9)

    # =========================================================================
    # Bottom Panel: Bar Chart of Mean |CTE| with Error Bars
    # =========================================================================
    ax2 = axes[1]

    x_pos = np.arange(len(stats))
    colors = [SCENARIO_COLORS[s] for s in stats['scenario']]

    bars = ax2.bar(x_pos, stats['cte_mean'], yerr=stats['cte_std'],
                   color=colors, alpha=0.8, capsize=5,
                   edgecolor='black', linewidth=0.8,
                   error_kw={'linewidth': 1.5, 'capthick': 1.5})

    # Add value labels on bars
    for i, (bar, mean, std) in enumerate(zip(bars, stats['cte_mean'], stats['cte_std'])):
        height = bar.get_height()
        ax2.annotate(f'{mean:.1f}±{std:.1f}',
                     xy=(bar.get_x() + bar.get_width()/2, height + std + 0.5),
                     ha='center', va='bottom', fontsize=9, fontweight='bold')

    # Formatting
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels([SCENARIO_LABELS[s] for s in stats['scenario']], fontsize=11)
    ax2.set_ylabel('Mean |Touchdown CTE| [m]', fontsize=11)
    ax2.set_xlabel('Wind Scenario', fontsize=11)
    ax2.grid(True, axis='y', alpha=0.3)

    # Set reasonable y-limit
    max_val = (stats['cte_mean'] + stats['cte_std']).max()
    ax2.set_ylim(0, max_val * 1.3)

    # =========================================================================
    # Final adjustments
    # =========================================================================
    plt.tight_layout()

    # Create output directory
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Save PDF
    fig.savefig(output_path, dpi=300, bbox_inches='tight', format='pdf')
    print(f"\nFigure saved to: {output_path}")

    # Save PNG preview
    png_path = output_path.with_suffix('.png')
    fig.savefig(png_path, dpi=150, bbox_inches='tight', format='png')
    print(f"Preview saved to: {png_path}")

    plt.close(fig)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Generate touchdown dispersion figure for VALOR report"
    )
    parser.add_argument(
        "--csv", "-c",
        type=Path,
        default=MONTE_CARLO_DIR / "touchdown_summary.csv",
        help="Path to Monte Carlo summary CSV"
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default=FIGS_DIR / "fig_touchdown_dispersion.pdf",
        help="Output PDF path"
    )
    args = parser.parse_args()

    # Check input file exists
    if not args.csv.exists():
        print(f"ERROR: CSV file not found: {args.csv}")
        print("Run run_monte_carlo.py first to generate data.")
        return 1

    # Load data
    df = load_data(args.csv)

    if len(df) == 0:
        print("ERROR: No successful runs in data")
        return 1

    # Generate plot
    plot_touchdown_dispersion(df, args.output)

    return 0


if __name__ == "__main__":
    sys.exit(main())
