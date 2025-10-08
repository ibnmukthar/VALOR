"""
Runs VALOR scenarios and writes CSV logs to results/logs/.
Scenarios:
  - baseline_0kt
  - steady_10kt / steady_20kt / steady_30kt
  - gust_20kt (20 kt base + 1-cosine gust)
"""
import os, json, csv
from pathlib import Path

import numpy as np

try:
    import jsbsim
except Exception as e:
    raise SystemExit("JSBSim not found. Activate your venv and `pip install jsbsim`.") from e

from controller import PID
from winds import one_cosine_gust, kts_to_mps, mps_to_fps

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "data" / "config.json"
LOGS = ROOT / "results" / "logs"

def load_config():
    with open(CONFIG, "r") as f:
        return json.load(f)

def set_initial_short_final(fdm, cfg):
    ic = cfg["sim"]["init"]
    ap = cfg["airport"]
    fdm["ic/altitude-ft"]   = float(ic["agl_ft"] + ap["elevation_ft"])
    fdm["ic/alpha-deg"]     = 3.0
    fdm["ic/beta-deg"]      = 0.0
    fdm["ic/psi-true-deg"]  = float(ic["psi_deg"])   # align with runway heading
    fdm["ic/theta-deg"]     = float(ic["theta_deg"]) # shallow approach pitch
    fdm["ic/phi-deg"]       = float(ic["phi_deg"])   # wings level
    fdm["ic/vtrue-kts"]     = float(ic["vtrue_kts"])
    # Put near runway threshold (refine to actual runway coords later)
    fdm["ic/long-gc-deg"]   = float(ap["threshold_lon_deg"])
    fdm["ic/lat-gc-deg"]    = float(ap["threshold_lat_deg"])

def apply_wind(fdm, crosswind_mps):
    # Simple: constant crosswind as EAST component (NED), in ft/s
    fdm["atmosphere/wind-north-fps"] = 0.0
    fdm["atmosphere/wind-east-fps"]  = float(mps_to_fps(crosswind_mps))
    fdm["atmosphere/wind-down-fps"]  = 0.0

def run_case(case_name, crosswind_kts=0.0, gust=None, aircraft="c172p"):
    cfg = load_config()
    dt = cfg["sim"]["dt"]
    t_final = cfg["sim"]["t_final"]

    fdm = jsbsim.FGFDMExec(None)
    fdm.set_root_dir(jsbsim.get_default_root_dir())
    if not fdm.load_model(aircraft):
        raise RuntimeError(f"Failed to load aircraft model: {aircraft}")

    set_initial_short_final(fdm, cfg)
    fdm.run_ic()

    # Controllers (tune later)
    roll = PID(**cfg["controllers"]["roll_pid"])
    glide = PID(**cfg["controllers"]["glide_pid"])

    t = 0.0
    steps = int(t_final / dt)
    rows = []
    base_xwind_mps = kts_to_mps(crosswind_kts)

    for _ in range(steps):
        # Crosswind + optional gust (lateral)
        gust_mps = 0.0
        if gust:
            gust_mps = one_cosine_gust(t, gust["t0"], gust["duration"], kts_to_mps(gust["Umax_kts"]))
        apply_wind(fdm, base_xwind_mps + gust_mps)

        # Simple controls (placeholder — replace with tuned logic)
        phi_deg = fdm["attitude/phi-deg"]
        aileron = roll.step(0.0 - phi_deg, dt)

        theta_deg = fdm["attitude/theta-deg"]
        elevator = glide.step(3.0 - theta_deg, dt)

        throttle = 0.30
        rudder   = 0.00  # add de-crab logic later

        fdm["fcs/aileron-cmd-norm"]  = float(aileron)
        fdm["fcs/elevator-cmd-norm"] = float(elevator)
        fdm["fcs/rudder-cmd-norm"]   = float(rudder)
        fdm["fcs/throttle-cmd-norm"] = float(throttle)

        ok = fdm.run()
        if not ok:
            break

        rows.append({
            "t": t,
            "lat_deg": fdm["position/lat-gc-deg"],
            "lon_deg": fdm["position/long-gc-deg"],
            "alt_ft":  fdm["position/h-agl-ft"],
            "phi_deg": fdm["attitude/phi-deg"],
            "theta_deg": fdm["attitude/theta-deg"],
            "psi_deg": fdm["attitude/psi-deg"],
            "vn_fps":  fdm["velocities/v-north-fps"],
            "ve_fps":  fdm["velocities/v-east-fps"],
            "vd_fps":  fdm["velocities/v-down-fps"],
            "aileron": aileron,
            "elevator": elevator,
            "rudder": rudder,
            "throttle": throttle,
            "xwind_kts": crosswind_kts,
            "gust_mps": gust_mps
        })

        t += dt
        fdm.set_dt(dt)

    LOGS.mkdir(parents=True, exist_ok=True)
    out = LOGS / f"{case_name}.csv"
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"[OK] {case_name}: {len(rows)} rows -> {out}")
    return out

def main():
    cases = [
        ("baseline_0kt", 0.0, None),
        ("steady_10kt", 10.0, None),
        ("steady_20kt", 20.0, None),
        ("steady_30kt", 30.0, None),
        ("gust_20kt",  20.0, {"t0": 15.0, "duration": 6.0, "Umax_kts": 10.0}),
    ]
    for name, xw, gust in cases:
        run_case(name, xw, gust)

if __name__ == "__main__":
    main()