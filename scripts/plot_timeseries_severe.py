#!/usr/bin/env python3
"""
Generate Fig. 3 (fig_timeseries_severe.pdf) for the VALOR report.

Time-series plot for severe crosswind scenario:
- Subplot 1: Cross-track error vs. time
- Subplot 2: Heading and runway course vs. time with highlighted de-crab region

Annotations for flare start and touchdown are included.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

# Add scripts directory to path for imports
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

# Directories
LOGS = ROOT / "results" / "logs"
FIGS = ROOT / "results" / "figs"

# Runway heading for KSFO 28R
RUNWAY_HEADING_DEG = 298.0

# De-crab altitude thresholds (from config)
DECRAB_START_FT = 100.0  # De-crab begins
DECRAB_END_FT = 50.0     # De-crab complete


def find_latest_csv(logs_dir: Path) -> Path:
    """Find the most recent CSV log file."""
    csvs = sorted(logs_dir.glob("autoland_*.csv"))
    if not csvs:
        raise FileNotFoundError(f"No CSV files found in {logs_dir}")
    return csvs[-1]


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-180, 180] range."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def find_event_indices(df: pd.DataFrame) -> dict:
    """
    Find indices for key events: flare start and touchdown.

    Returns dict with keys: 'flare_idx', 'touchdown_idx', and times.
    """
    events = {}

    # Find flare start (first row where phase == 'FLARE')
    flare_mask = df['phase'] == 'FLARE'
    if flare_mask.any():
        events['flare_idx'] = flare_mask.idxmax()
        events['flare_time'] = df.loc[events['flare_idx'], 't']
        events['flare_alt'] = df.loc[events['flare_idx'], 'alt_agl_ft']

    # Find touchdown (first row where wow == 1)
    touchdown_mask = df['wow'] == 1
    if touchdown_mask.any():
        events['touchdown_idx'] = touchdown_mask.idxmax()
        events['touchdown_time'] = df.loc[events['touchdown_idx'], 't']

    return events


def find_decrab_region(df: pd.DataFrame) -> tuple:
    """
    Find the time range where de-crab occurs (altitude between DECRAB_START and DECRAB_END).

    Returns (start_time, end_time) or (None, None) if not found.
    """
    # De-crab region: altitude between 100 ft and 50 ft AGL
    decrab_mask = (df['alt_agl_ft'] <= DECRAB_START_FT) & (df['alt_agl_ft'] >= DECRAB_END_FT)

    if not decrab_mask.any():
        return None, None

    decrab_indices = df.index[decrab_mask]
    start_time = df.loc[decrab_indices[0], 't']
    end_time = df.loc[decrab_indices[-1], 't']

    return start_time, end_time


def plot_timeseries_severe(csv_path: Path, output_path: Path):
    """
    Generate the two-panel time-series figure for severe crosswind scenario.

    Parameters:
        csv_path: Path to the CSV log file
        output_path: Path for output PDF file
    """
    # Load data
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} rows from {csv_path.name}")

    # Time array
    t = df['t'].values

    # Cross-track error (in meters)
    cte = df['cte_m'].values

    # Heading and runway course
    psi = df['psi_deg'].values
    runway_course = np.full_like(psi, RUNWAY_HEADING_DEG)

    # Compute crab angle (heading - runway heading), normalized
    crab_angle = np.array([normalize_angle(p - RUNWAY_HEADING_DEG) for p in psi])

    # Find key events
    events = find_event_indices(df)
    decrab_start, decrab_end = find_decrab_region(df)

    # Find when touchdown occurs to limit x-axis
    if 'touchdown_time' in events:
        # Add a bit of buffer after touchdown for rollout visibility
        t_max = min(events['touchdown_time'] + 3, t[-1])
    else:
        t_max = t[-1]

    # For clean visualization, start from when wind kicks in (after warmup ~3s)
    t_min = 3.0

    # Create figure with two subplots
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # Color scheme
    color_cte = '#1f77b4'        # Blue for CTE
    color_heading = '#2ca02c'    # Green for heading
    color_runway = '#d62728'     # Red for runway course
    color_decrab = '#ff7f0e'     # Orange for de-crab region
    color_flare = '#9467bd'      # Purple for flare
    color_td = '#8c564b'         # Brown for touchdown

    # =========================================================================
    # Subplot 1: Cross-track error vs. time
    # =========================================================================
    ax1 = axes[0]

    ax1.plot(t, cte, color=color_cte, linewidth=1.5, label='Cross-track error')
    ax1.axhline(y=0, color='gray', linestyle='--', linewidth=0.8, alpha=0.7)

    # Highlight de-crab region
    if decrab_start is not None and decrab_end is not None:
        ax1.axvspan(decrab_start, decrab_end, alpha=0.2, color=color_decrab,
                    label='De-crab region')

    # Annotate flare start
    if 'flare_time' in events:
        ax1.axvline(x=events['flare_time'], color=color_flare, linestyle=':',
                    linewidth=1.5, alpha=0.8)
        # Get CTE at flare for annotation position
        flare_cte = df.loc[events['flare_idx'], 'cte_m']
        ax1.annotate('Flare', xy=(events['flare_time'], flare_cte),
                     xytext=(events['flare_time'] - 3, flare_cte + 0.5),
                     fontsize=9, color=color_flare,
                     arrowprops=dict(arrowstyle='->', color=color_flare, lw=0.8))

    # Annotate touchdown
    if 'touchdown_time' in events:
        ax1.axvline(x=events['touchdown_time'], color=color_td, linestyle='--',
                    linewidth=1.5, alpha=0.8)
        td_cte = df.loc[events['touchdown_idx'], 'cte_m']
        ax1.annotate('Touchdown', xy=(events['touchdown_time'], td_cte),
                     xytext=(events['touchdown_time'] + 1, td_cte + 0.8),
                     fontsize=9, color=color_td,
                     arrowprops=dict(arrowstyle='->', color=color_td, lw=0.8))

    ax1.set_ylabel('Cross-track Error [m]', fontsize=11)
    ax1.set_xlim(t_min, t_max)

    # Set y-limits to focus on approach tracking (exclude large rollout excursions)
    # Find CTE range during approach/flare only
    approach_flare_mask = (df['phase'].isin(['APPROACH', 'FLARE'])) & (df['t'] >= t_min)
    if approach_flare_mask.any():
        cte_approach = df.loc[approach_flare_mask, 'cte_m'].values
        cte_max_vis = max(abs(cte_approach.min()), abs(cte_approach.max())) * 1.3
        cte_max_vis = max(cte_max_vis, 5)  # At least ±5 m range
        ax1.set_ylim(-cte_max_vis, cte_max_vis)

    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=9)
    ax1.set_title('Severe Crosswind Landing: Lateral Tracking and De-Crab', fontsize=12, fontweight='bold')

    # =========================================================================
    # Subplot 2: Heading and runway course vs. time
    # =========================================================================
    ax2 = axes[1]

    # Plot heading and runway course
    ax2.plot(t, psi, color=color_heading, linewidth=1.5, label='Aircraft heading')
    ax2.plot(t, runway_course, color=color_runway, linestyle='--', linewidth=1.5,
             label=f'Runway course ({RUNWAY_HEADING_DEG:.0f}°)')

    # Highlight de-crab region with shading
    if decrab_start is not None and decrab_end is not None:
        ax2.axvspan(decrab_start, decrab_end, alpha=0.2, color=color_decrab,
                    label='De-crab region')

        # Add annotation for de-crab
        mid_decrab = (decrab_start + decrab_end) / 2
        # Find heading at mid de-crab
        decrab_mask = (df['t'] >= decrab_start) & (df['t'] <= decrab_end)
        if decrab_mask.any():
            mid_idx = df.index[decrab_mask][len(df.index[decrab_mask])//2]
            mid_heading = df.loc[mid_idx, 'psi_deg']
            ax2.annotate('De-crab\ntransition',
                         xy=(mid_decrab, mid_heading),
                         xytext=(mid_decrab - 4, mid_heading + 8),
                         fontsize=9, color=color_decrab, ha='center',
                         arrowprops=dict(arrowstyle='->', color=color_decrab, lw=0.8))

    # Annotate flare start
    if 'flare_time' in events:
        ax2.axvline(x=events['flare_time'], color=color_flare, linestyle=':',
                    linewidth=1.5, alpha=0.8)

    # Annotate touchdown
    if 'touchdown_time' in events:
        ax2.axvline(x=events['touchdown_time'], color=color_td, linestyle='--',
                    linewidth=1.5, alpha=0.8)

    ax2.set_xlabel('Time [s]', fontsize=11)
    ax2.set_ylabel('Heading [°]', fontsize=11)
    ax2.set_xlim(t_min, t_max)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=9)

    # Adjust y-limits for heading plot to show reasonable range
    heading_min = min(psi.min(), RUNWAY_HEADING_DEG) - 5
    heading_max = max(psi.max(), RUNWAY_HEADING_DEG) + 5
    ax2.set_ylim(heading_min, heading_max)

    # =========================================================================
    # Add secondary y-axis for crab angle on subplot 2
    # =========================================================================
    ax2_twin = ax2.twinx()
    ax2_twin.plot(t, crab_angle, color='gray', linewidth=1.0, alpha=0.5,
                  linestyle='-.', label='Crab angle')
    ax2_twin.axhline(y=0, color='gray', linestyle=':', linewidth=0.5, alpha=0.5)
    ax2_twin.set_ylabel('Crab Angle [°]', fontsize=10, color='gray')
    ax2_twin.tick_params(axis='y', labelcolor='gray')

    # Set reasonable crab angle limits
    crab_min = min(crab_angle.min(), -5)
    crab_max = max(crab_angle.max(), 5)
    ax2_twin.set_ylim(crab_min - 2, crab_max + 2)

    # =========================================================================
    # Final adjustments
    # =========================================================================
    plt.tight_layout()

    # Create output directory if needed
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Save as PDF
    fig.savefig(output_path, dpi=300, bbox_inches='tight', format='pdf')
    print(f"Figure saved to: {output_path}")

    # Also save as PNG for quick preview
    png_path = output_path.with_suffix('.png')
    fig.savefig(png_path, dpi=150, bbox_inches='tight', format='png')
    print(f"Preview saved to: {png_path}")

    plt.close(fig)

    # Print summary statistics (for approach and flare only, excluding rollout)
    approach_flare_mask = df['phase'].isin(['APPROACH', 'FLARE'])
    cte_approach = df.loc[approach_flare_mask, 'cte_m'].values
    crab_approach = np.array([normalize_angle(p - RUNWAY_HEADING_DEG)
                              for p in df.loc[approach_flare_mask, 'psi_deg'].values])

    print("\n--- Summary Statistics (Approach + Flare only) ---")
    print(f"Cross-track error: min={cte_approach.min():.2f} m, max={cte_approach.max():.2f} m, "
          f"RMS={np.sqrt(np.mean(cte_approach**2)):.2f} m")
    print(f"Crab angle: min={crab_approach.min():.1f}°, max={crab_approach.max():.1f}°")
    if 'flare_time' in events:
        print(f"Flare initiated at t={events['flare_time']:.1f} s, "
              f"alt={events['flare_alt']:.0f} ft")
    if 'touchdown_time' in events:
        td_cte = df.loc[events['touchdown_idx'], 'cte_m']
        td_heading = df.loc[events['touchdown_idx'], 'psi_deg']
        print(f"Touchdown at t={events['touchdown_time']:.1f} s, "
              f"CTE={td_cte:.2f} m, heading={td_heading:.1f}°")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate Fig. 3 time-series plot for severe crosswind scenario"
    )
    parser.add_argument(
        "--csv", "-c",
        type=Path,
        help="Path to CSV log file (default: most recent in results/logs)"
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default=FIGS / "fig_timeseries_severe.pdf",
        help="Output PDF path"
    )
    args = parser.parse_args()

    # Find CSV file
    if args.csv:
        csv_path = args.csv
    else:
        csv_path = find_latest_csv(LOGS)

    if not csv_path.exists():
        print(f"ERROR: CSV file not found: {csv_path}")
        return 1

    # Generate plot
    plot_timeseries_severe(csv_path, args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
