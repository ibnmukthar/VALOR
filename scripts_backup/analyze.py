import json
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from metrics import compute_touchdown_metrics, robust_touchdown_index
from winds import Dryden3D

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "results" / "logs"
FIGS = ROOT / "results" / "figs"
CONFIG = ROOT / "data" / "config.json"


def ensure_dirs():
    FIGS.mkdir(parents=True, exist_ok=True)


def summarize_csv(path: Path) -> dict:
    df = pd.read_csv(path)
    m = compute_touchdown_metrics(df)
    # Lateral deviation at touchdown using robust WOW-aware index when present
    i_td = robust_touchdown_index(df) if m.get("touched_down") else None
    lat_dev_at_td = float("nan")
    if i_td is not None and "lat_dev_m" in df.columns:
        v = df.iloc[i_td]["lat_dev_m"]
        lat_dev_at_td = float(v) if pd.notna(v) else float("nan")
    # Fallback: last valid sample's lateral deviation if TD is invalid
    td_fallback = False
    if not np.isfinite(lat_dev_at_td):
        good = df[["lat_dev_m"]].applymap(np.isfinite).all(axis=1)
        if good.any():
            # Prefer the last finite sample at or before touchdown index if available
            if i_td is not None:
                finite_idxs = np.where(good.values & (np.arange(len(df)) <= i_td))[0]
                if len(finite_idxs):
                    i_fb = int(finite_idxs[-1])
                else:
                    i_fb = int(np.where(good.values)[0][-1])
            else:
                i_fb = int(np.where(good.values)[0][-1])
            lat_dev_at_td = float(df.iloc[i_fb]["lat_dev_m"]) if pd.notna(df.iloc[i_fb]["lat_dev_m"]) else float("nan")
            td_fallback = True
    m.update({
        "case": path.stem,
        "n_rows": len(df),
        "lat_dev_m_at_td": lat_dev_at_td,
        "td_fallback": td_fallback
    })
    return m


def plot_timeseries(df: pd.DataFrame, case: str):
    t = df["t"].values
    i_td = robust_touchdown_index(df)
    t_td = float(df["t"].iloc[i_td]) if i_td is not None else None
    # 1) Altitude and desired glideslope height error
    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax[0].plot(t, df["alt_ft"], label="Altitude AGL [ft]")
    ax[0].plot(t, df["h_des_ft"], '--', label="Desired GS height [ft]")
    if i_td is not None:
        ax[0].axvline(t_td, color='r', linestyle=':', alpha=0.8, label='TD (gate)')
    ax[0].set_ylabel("Height [ft]")
    ax[0].legend(); ax[0].grid(True)
    ax[1].plot(t, df["e_alt_ft"], label="Height error [ft]")
    if i_td is not None:
        ax[1].axvline(t_td, color='r', linestyle=':', alpha=0.8)
    ax[1].set_xlabel("Time [s]"); ax[1].set_ylabel("Error [ft]")
    ax[1].grid(True)
    fig.tight_layout(); fig.savefig(FIGS / f"{case}_altitude.png", dpi=150); plt.close(fig)

    # 2) Gamma tracking and airspeed
    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax[0].plot(t, df["gamma_deg"], label="Gamma [deg]")
    ax[0].plot(t, df["gamma_des_deg"], '--', label="Gamma des [deg]")
    if i_td is not None:
        ax[0].axvline(t_td, color='r', linestyle=':', alpha=0.8, label='TD (gate)')
    ax[0].legend(); ax[0].grid(True)
    ax[1].plot(t, df["va_mps"] * 1.94384, label="True airspeed [kt]")
    if i_td is not None:
        ax[1].axvline(t_td, color='r', linestyle=':', alpha=0.8)
    ax[1].set_xlabel("Time [s]"); ax[1].set_ylabel("Airspeed [kt]")
    ax[1].grid(True)
    fig.tight_layout(); fig.savefig(FIGS / f"{case}_gamma_speed.png", dpi=150); plt.close(fig)

    # 3) Lateral deviation and bank
    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax[0].plot(t, df["lat_dev_m"], label="Cross-track [m]")
    if i_td is not None:
        ax[0].axvline(t_td, color='r', linestyle=':', alpha=0.8, label='TD (gate)')
    ax[0].set_ylabel("Cross-track [m]"); ax[0].grid(True)
    ax[1].plot(t, df["phi_deg"], label="Bank [deg]")
    if i_td is not None:
        ax[1].axvline(t_td, color='r', linestyle=':', alpha=0.8)
    ax[1].set_xlabel("Time [s]"); ax[1].set_ylabel("Phi [deg]"); ax[1].grid(True)
    fig.tight_layout(); fig.savefig(FIGS / f"{case}_lateral.png", dpi=150); plt.close(fig)

    # 4) Control histories
    fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    ax[0].plot(t, df["aileron"], label="Aileron")
    if i_td is not None:
        ax[0].axvline(t_td, color='r', linestyle=':', alpha=0.8)
    ax[1].plot(t, df["elevator"], label="Elevator")
    if i_td is not None:
        ax[1].axvline(t_td, color='r', linestyle=':', alpha=0.8)
    ax[2].plot(t, df["throttle"], label="Throttle")
    if i_td is not None:
        ax[2].axvline(t_td, color='r', linestyle=':', alpha=0.8)
    for a in ax: a.grid(True)
    ax[-1].set_xlabel("Time [s]")
    fig.tight_layout(); fig.savefig(FIGS / f"{case}_controls.png", dpi=150); plt.close(fig)


def barplot_touchdown_lat_dev(summary: pd.DataFrame):
    # Bar plot of lateral deviation at touchdown across cases
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar(summary["case"], summary["lat_dev_m_at_td"])
    ax.set_ylabel("Cross-track at TD [m]")
    ax.set_xticklabels(summary["case"], rotation=25, ha='right')
    ax.grid(True, axis='y')
    fig.tight_layout(); fig.savefig(FIGS / "touchdown_xtrack_bar.png", dpi=150); plt.close(fig)


def dryden_psd_theory(freq, Va, L, sigma, component="u"):
    # Angular frequency along path
    omega = 2*np.pi*freq
    Omega = omega * L / max(Va, 1e-6)
    if component == "u":
        Su = (2 * sigma**2 * L / (np.pi * max(Va, 1e-6))) * (1.0 / (1.0 + Omega**2))
        return Su
    # lateral/vertical approximation
    Svw = (sigma**2 * L / (np.pi * max(Va, 1e-6))) * ((1 + 3*Omega**2) / (1 + Omega**2)**2)
    return Svw


def generate_dryden_psd_plot():
    # Load config for Dryden params
    cfg = json.loads(Path(CONFIG).read_text())
    d = cfg.get("wind", {}).get("dryden", {})
    Va = float(cfg["sim"].get("approach_tas_mps", 38.6))
    sig_u = float(d.get("sigma_u", 2.0)); sig_w = float(d.get("sigma_w", 1.0))
    Lu = float(d.get("Lu", 200.0)); Lw = float(d.get("Lw", 50.0))
    dt = float(cfg["sim"].get("dt", 0.01))

    N = 32768  # ~327 s at 0.01 s
    gen = Dryden3D(Va_mps=Va, sigma_u=sig_u, sigma_v=sig_u, sigma_w=sig_w, Lu=Lu, Lv=Lu, Lw=Lw, seed=11)
    x_u = np.zeros(N); x_w = np.zeros(N)
    for k in range(N):
        uvw = gen.step(dt)
        x_u[k] = uvw[0]
        x_w[k] = uvw[2]

    def psd(x, fs):
        X = np.fft.rfft(x * np.hanning(len(x)))
        Pxx = (1.0/(fs*len(x))) * (np.abs(X)**2)
        f = np.fft.rfftfreq(len(x), d=1.0/fs)
        # Smooth (moving average)
        win = 16
        Pxx_s = np.convolve(Pxx, np.ones(win)/win, mode='same')
        return f, Pxx_s

    fs = 1.0/dt
    fu, Pu = psd(x_u, fs)
    fw, Pw = psd(x_w, fs)

    Su = dryden_psd_theory(fu, Va, Lu, sig_u, component="u")
    Sw = dryden_psd_theory(fw, Va, Lw, sig_w, component="w")

    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    ax[0].loglog(fu[1:], Pu[1:], label="AR approx")
    ax[0].loglog(fu[1:], Su[1:], '--', label="Dryden theory")
    ax[0].set_title("Longitudinal u PSD"); ax[0].set_xlabel("Hz"); ax[0].set_ylabel("PSD")
    ax[0].legend(); ax[0].grid(True, which='both')
    ax[1].loglog(fw[1:], Pw[1:], label="AR approx")
    ax[1].loglog(fw[1:], Sw[1:], '--', label="Dryden theory")
    ax[1].set_title("Vertical w PSD"); ax[1].set_xlabel("Hz")
    ax[1].legend(); ax[1].grid(True, which='both')
    fig.tight_layout(); fig.savefig(FIGS / "dryden_psd.png", dpi=150); plt.close(fig)


def main():
    ensure_dirs()
    paths = sorted(LOGS.glob("*.csv"))
    if not paths:
        print("No CSV logs found. Run scripts/run_suite.py first.")
        return
    # Whitelist the current GA-based suite only
    allowed = {
        "baseline_0kt", "steady_10kt", "steady_20kt", "steady_30kt",
        "gust_20kt", "hybrid_dryden_20kt"
    }
    paths = [p for p in paths if p.stem in allowed]
    rows = []
    for p in paths:
        df = pd.read_csv(p)
        rows.append(summarize_csv(p))
        plot_timeseries(df, p.stem)
    summary = pd.DataFrame(rows)
    summary.to_csv(ROOT / "results" / "summary.csv", index=False)
    with pd.option_context("display.max_columns", None):
        print(summary)
    barplot_touchdown_lat_dev(summary)
    # Generate PSD validation figure
    generate_dryden_psd_plot()

if __name__ == "__main__":
    main()
