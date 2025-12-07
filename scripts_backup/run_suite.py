"""
Runs VALOR scenarios and writes CSV logs to results/logs/.
Scenarios:
  - baseline_0kt
  - steady_10kt / steady_20kt / steady_30kt
  - gust_20kt (20 kt base + 1-cosine gust)
  - hybrid_dryden_* (hybrid shear + Dryden turbulence)
"""
import os, json, csv, math
from pathlib import Path

import numpy as np

try:
    import jsbsim
except Exception as e:
    raise SystemExit("JSBSim not found. Activate your venv and `pip install jsbsim`.") from e

from controller import PID
from winds import (
    one_cosine_gust, kts_to_mps, mps_to_fps,
    Dryden3D, LogLayerShear, StabilityCorrectedShear, HybridWindModel
)
from fg_sender import FlightGearGenericSender


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "data" / "config.json"
LOGS = ROOT / "results" / "logs"

def load_config():
    with open(CONFIG, "r") as f:
        return json.load(f)

def set_initial_short_final(fdm, cfg):
    ic = cfg["sim"]["init"]
    ap = cfg["airport"]
    # Place ~N NM out on extended centerline (default 2 NM)
    nm = 1852.0
    initial_dist_nm = float(ic.get("initial_dist_nm", 2.0))
    dist_m = initial_dist_nm * nm
    hdg = math.radians(float(ap["runway_heading_deg"]))
    dN = -dist_m * math.cos(hdg)
    dE = -dist_m * math.sin(hdg)
    mlat, mlon = meters_per_deg(float(ap["threshold_lat_deg"]))
    lat0 = float(ap["threshold_lat_deg"]) + dN / mlat
    lon0 = float(ap["threshold_lon_deg"]) + dE / mlon

    # Altitude: optionally place on a nominal glideslope
    use_gs = bool(ic.get("use_glideslope_ic", True))
    if use_gs:
        gs_deg = float(ic.get("glideslope_deg", 3.0))
        agl_m = math.tan(math.radians(gs_deg)) * dist_m
        agl_ft = agl_m * 3.28084
    else:
        agl_ft = float(ic.get("agl_ft", 200.0))

    alt_msl_ft = float(agl_ft + ap["elevation_ft"])  # AGL + field elevation
    # Altitude (use multiple synonyms for robustness across aircraft definitions)
    for prop in [
        "ic/altitude-ft",    # common
        "ic/h-sl-ft",        # some models use sea-level altitude
        "ic/h-agl-ft"        # some models support direct AGL IC
    ]:
        try:
            fdm[prop] = alt_msl_ft if prop != "ic/h-agl-ft" else agl_ft
        except Exception:
            pass

    # Attitude and speed
    fdm["ic/alpha-deg"]     = 3.0
    fdm["ic/beta-deg"]      = 0.0
    fdm["ic/psi-true-deg"]  = float(ic["psi_deg"])   # align with runway heading
    fdm["ic/theta-deg"]     = float(ic["theta_deg"]) # shallow approach pitch
    fdm["ic/phi-deg"]       = float(ic["phi_deg"])   # wings level
    fdm["ic/vt-kts"]        = float(ic["vtrue_kts"])

    # Position (set a broad set of synonyms; different models bind different IC keys)
    for prop, val in [
        ("ic/lat-gc-deg",  lat0), ("ic/long-gc-deg", lon0),
        ("ic/lat-geod-deg", lat0), ("ic/long-geod-deg", lon0),
        ("ic/vrp-lat-gc-deg", lat0), ("ic/vrp-long-gc-deg", lon0),
    ]:
        try:
            fdm[prop] = val
        except Exception:
            pass

    # Keep initial conditions minimal to avoid over-constraining JSBSim IC solver.
    # We do not set gamma or explicit inertial/body velocities; vt-kts + attitude suffice.

# --- Local geodesy helpers (flat-earth approx) ---
def meters_per_deg(lat_deg: float):
    lat_rad = math.radians(lat_deg)
    m_per_deg_lat = 111132.92 - 559.82*math.cos(2*lat_rad) + 1.175*math.cos(4*lat_rad)
    m_per_deg_lon = (111412.84*math.cos(lat_rad) - 93.5*math.cos(3*lat_rad))
    return m_per_deg_lat, m_per_deg_lon

def en_from_latlon(lat_deg, lon_deg, lat0_deg, lon0_deg):
    mlat, mlon = meters_per_deg(lat0_deg)
    dn = (lat_deg - lat0_deg) * mlat
    de = (lon_deg - lon0_deg) * mlon
    return dn, de

def cross_track_m(lat_deg, lon_deg, lat0_deg, lon0_deg, runway_heading_deg):
    dn, de = en_from_latlon(lat_deg, lon_deg, lat0_deg, lon0_deg)
    # Unit along-runway (north,east)
    hr = math.radians(runway_heading_deg)
    en_along = (math.cos(hr), math.sin(hr))
    en_cross = (-en_along[1], en_along[0])  # +90 deg
    # Project displacement onto cross-track
    return dn*en_cross[0] + de*en_cross[1]

def along_track_m(lat_deg, lon_deg, lat0_deg, lon0_deg, runway_heading_deg):
    """
    Signed along-runway displacement (m). Positive in runway heading direction.
    """
    dn, de = en_from_latlon(lat_deg, lon_deg, lat0_deg, lon0_deg)
    hr = math.radians(runway_heading_deg)
    en_along = (math.cos(hr), math.sin(hr))
    return dn*en_along[0] + de*en_along[1]

# --- Wind application ---
def apply_wind_ned(fdm, north_mps, east_mps, down_mps):
    fdm["atmosphere/wind-north-fps"] = float(mps_to_fps(north_mps))
    fdm["atmosphere/wind-east-fps"]  = float(mps_to_fps(east_mps))
    fdm["atmosphere/wind-down-fps"]  = float(mps_to_fps(down_mps))


def make_wind_model(cfg, base_crosswind_kts: float):
    """Construct wind model per config. Falls back to constant crosswind."""
    wind_cfg = cfg.get("wind", {})
    model = wind_cfg.get("model", "constant")
    ap = cfg["airport"]
    if model == "hybrid":
        shear_cfg = wind_cfg.get("shear", {})
        shear_type = shear_cfg.get("type", "log").lower()
        if shear_type in ("mo", "monin_obukhov", "stability") or ("L_m" in shear_cfg or "L" in shear_cfg):
            shear = StabilityCorrectedShear(
                Uref_mps=kts_to_mps(float(base_crosswind_kts)),
                z_ref_m=float(shear_cfg.get("z_ref_m", 10.0)),
                z0_m=float(shear_cfg.get("z0_m", 0.1)),
                L_m=float(shear_cfg.get("L_m", float("inf"))),
                heading_deg=float(ap["runway_heading_deg"])
            )
        else:
            shear = LogLayerShear(
                Uref_mps=kts_to_mps(float(base_crosswind_kts)),
                z_ref_m=float(shear_cfg.get("z_ref_m", 10.0)),
                z0_m=float(shear_cfg.get("z0_m", 0.1)),
                heading_deg=float(ap["runway_heading_deg"])
            )
        d = wind_cfg.get("dryden", {})
        turb = Dryden3D(
            Va_mps=float(cfg["sim"].get("approach_tas_mps", 65.0)),
            sigma_u=float(d.get("sigma_u", 1.5)),
            sigma_v=float(d.get("sigma_v", 1.5)),
            sigma_w=float(d.get("sigma_w", 0.8)),
            Lu=float(d.get("Lu", 200.0)),
            Lv=float(d.get("Lv", 200.0)),
            Lw=float(d.get("Lw", 50.0)),
            seed=int(d.get("seed", 7))
        )
        return HybridWindModel(
            shear=shear, turb=turb,
            turb_decay_h_m=float(wind_cfg.get("turb_decay_h_m", 30.0)),
            use_crosswind_axis=True
        )
    # constant model
    class _Const:
        def __init__(self, east_mps): self.east = float(east_mps)
        def step_ned(self, dt, z): return 0.0, self.east, 0.0
    return _Const(kts_to_mps(base_crosswind_kts))


def run_case(case_name, crosswind_kts=0.0, gust=None, aircraft=None):
    cfg = load_config()
    dt = cfg["sim"]["dt"]
    t_final = cfg["sim"]["t_final"]

    # Select aircraft with fallback list for stability
    prefer = []
    if aircraft:
        prefer.append(str(aircraft))
    sim_ac = cfg["sim"].get("aircraft")
    if sim_ac:
        prefer.append(str(sim_ac))
    prefer += [str(x) for x in cfg["sim"].get("aircraft_candidates", [])]
    if not prefer:
        prefer = ["c172x", "c172p"]

    fdm = jsbsim.FGFDMExec(None)
    fdm.set_root_dir(jsbsim.get_default_root_dir())
    loaded = False
    for ac in prefer:
        try:
            if fdm.load_model(ac):
                loaded = True
                aircraft = ac
                break
        except Exception:
            continue
    if not loaded:
        raise RuntimeError(f"Failed to load any aircraft from candidates: {prefer}")

    set_initial_short_final(fdm, cfg)
    fdm.run_ic()
    # Ensure fixed integration timestep is set before the first run
    fdm.set_dt(dt)

    # Controllers
    roll = PID(**cfg["controllers"]["roll_pid"])
    glide = PID(**cfg["controllers"]["glide_pid"])  # flight-path angle control
    thrtl = PID(**cfg["controllers"]["throttle_pid"])  # autothrottle (speed hold)
    Vref_mps = float(cfg["sim"].get("approach_tas_mps", 65.0))

    # Wind model
    wind_model = make_wind_model(cfg, crosswind_kts)

    # FlightGear visualization (optional)
    viz_cfg = cfg.get("viz", {}).get("flightgear", {})
    fg_sender = None
    if bool(viz_cfg.get("enabled", False)):
        try:
            fg_sender = FlightGearGenericSender(
                viz_cfg.get("host", "127.0.0.1"), int(viz_cfg.get("port", 5501)), cfg
            )
        except Exception:
            fg_sender = None

    ap = cfg["airport"]
    lat0, lon0 = float(ap["threshold_lat_deg"]), float(ap["threshold_lon_deg"])
    rw_hdg = float(ap["runway_heading_deg"])

    # Prefer geodetic lat/lon if available
    def _latlon(fdm):
        # Prefer explicit geodetic properties; avoid ambiguous 'longitude-deg'
        try:
            lat = float(fdm["position/lat-geod-deg"])
        except Exception:
            lat = float(fdm["position/lat-gc-deg"])
        try:
            lon = float(fdm["position/long-geod-deg"])
        except Exception:
            lon = float(fdm["position/long-gc-deg"])
        return lat, lon

    t = 0.0
    steps = int(t_final / dt)
    # Do not enforce inertial/body velocities post-IC; let JSBSim resolve them from IC.

    # Initialize remaining along-runway distance using initial position
    lat_init, lon_init = _latlon(fdm)
    along_init_m = along_track_m(lat_init, lon_init, lat0, lon0, rw_hdg)
    s_rem_m = max(0.0, -along_init_m)
    # Initialize previous lat/lon for cross-track based lateral guidance
    lat_prev, lon_prev = lat_init, lon_init

    # Debug: print initial position for FlightGear troubleshooting
    h_agl_init = float(fdm["position/h-agl-ft"])
    h_sl_init = float(fdm["position/h-sl-ft"])
    print(f"  [FG_DEBUG] Init pos: lat={lat_init:.5f}, lon={lon_init:.5f}, h_agl={h_agl_init:.1f}ft, h_sl={h_sl_init:.1f}ft, dist_to_thr={s_rem_m:.0f}m")

    rows = []

    for _ in range(steps):
        # Wind field
        z_agl_m = float(fdm["position/h-agl-ft"]) * 0.3048
        n_mps, e_mps, d_mps = wind_model.step_ned(dt, z_agl_m)
        # Optional deterministic gust added on crosswind (east)
        gust_mps = 0.0
        if gust:
            gust_mps = one_cosine_gust(t, gust["t0"], gust["duration"], kts_to_mps(gust["Umax_kts"]))
            e_mps += gust_mps
        apply_wind_ned(fdm, n_mps, e_mps, d_mps)

        # Lateral: simple outer loop on cross-track -> desired bank, then track bank
        cte_m = cross_track_m(lat_prev, lon_prev, lat0, lon0, rw_hdg)  # y_lat (m)
        phi_des = float(np.clip(-0.003 * cte_m, -15.0, 15.0))
        phi_ref_deg = phi_des  # keep name for logging
        phi_deg = float(fdm["attitude/phi-deg"])  # roll angle
        aileron = roll.step(phi_ref_deg - phi_deg, dt)

        # Flight-path angle control using gamma = theta - alpha (deg). gamma_des is negative (descent).
        theta_deg = float(fdm["attitude/theta-deg"])
        alpha_deg = float(fdm["aero/alpha-deg"])
        gamma_deg = theta_deg - alpha_deg
        gamma_des_deg = -float(cfg["sim"]["init"].get("glideslope_deg", 3.0))
        e_gamma_deg = gamma_des_deg - gamma_deg
        elevator = glide.step(e_gamma_deg, dt)

        # Autothrottle: hold true airspeed using vt-fps (robust)
        tas_fps = float(fdm["velocities/vt-fps"])     # true airspeed, ft/s
        Va_mps = tas_fps * 0.3048
        throttle = float(np.clip(0.5 + 0.02 * (Vref_mps - Va_mps), 0.0, 1.0))

        rudder   = 0.00  # add de-crab logic later

        # Commands
        fdm["fcs/aileron-cmd-norm"]  = float(aileron)
        # Elevator command: positive PID output commands nose-up for c172p
        fdm["fcs/elevator-cmd-norm"] = float(elevator)
        fdm["fcs/rudder-cmd-norm"]   = float(rudder)
        fdm["fcs/throttle-cmd-norm"] = float(throttle)

        ok = fdm.run()
        if not ok:
            break

        # Stream state to FlightGear (if enabled)
        if fg_sender:
            try:
                fg_sender.send(fdm, t)
            except Exception as e:
                print(f"[FG_SENDER_ERROR] {e}")


        # Update geometry and derived metrics
        lat, lon = _latlon(fdm)
        lat_dev_m = cross_track_m(lat, lon, lat0, lon0, rw_hdg)
        along_m_log = along_track_m(lat, lon, lat0, lon0, rw_hdg)
        # Distance to threshold from lat/lon (diagnostic only)
        dist_to_thr_m_geo = max(0.0, -along_m_log)
        # Integrate remaining distance from inertial velocities for robustness
        vn_fps_log = fdm["velocities/v-north-fps"]
        ve_fps_log = fdm["velocities/v-east-fps"]
        along_rate_mps = (vn_fps_log * math.cos(math.radians(rw_hdg)) + ve_fps_log * math.sin(math.radians(rw_hdg))) * 0.3048
        s_rem_m = max(0.0, s_rem_m - along_rate_mps * dt)

        gs_deg = float(cfg["sim"]["init"].get("glideslope_deg", 3.0))
        h_des_ft = math.tan(math.radians(gs_deg)) * s_rem_m * 3.28084
        e_alt_ft = h_des_ft - fdm["position/h-agl-ft"]

        vd_fps_log = fdm["velocities/v-down-fps"]
        # Log gamma via theta-alpha (deg)
        theta_deg_log = float(fdm["attitude/theta-deg"])
        alpha_deg_log = float(fdm["aero/alpha-deg"])
        gamma_deg_log = theta_deg_log - alpha_deg_log
        gamma_des_deg = -float(cfg["sim"]["init"].get("glideslope_deg", 3.0))
        e_gamma_deg_log = gamma_des_deg - gamma_deg_log
        # Log true airspeed via vt-fps
        Vt_fps_log = float(fdm["velocities/vt-fps"])
        Va_mps_log = Vt_fps_log * 0.3048

        # WOW flags (robust read across typical unit indices)
        def _p(name, default=0.0):
            try:
                return float(fdm[name])
            except Exception:
                return float(default)
        _w0 = 1 if _p("gear/unit[0]/WOW", 0.0) > 0.5 else 0
        _w1 = 1 if _p("gear/unit[1]/WOW", 0.0) > 0.5 else 0
        _w2 = 1 if _p("gear/unit[2]/WOW", 0.0) > 0.5 else 0
        _wa = 1 if (_w0 or _w1 or _w2) else 0

        rows.append({
            "t": t,
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_ft":  fdm["position/h-agl-ft"],
            "phi_deg": fdm["attitude/phi-deg"],
            "theta_deg": fdm["attitude/theta-deg"],
            "psi_deg": fdm["attitude/psi-deg"],
            "vn_fps":  vn_fps_log,
            "ve_fps":  ve_fps_log,
            "vd_fps":  vd_fps_log,
            "va_mps":  Va_mps_log,
            "gamma_deg": gamma_deg_log,
            "gamma_des_deg": gamma_des_deg,
            "e_gamma_deg": e_gamma_deg_log,
            "phi_ref_deg": phi_ref_deg,
            "cte_m": cte_m,
            "aileron": aileron,
            "elevator": elevator,
            "rudder": rudder,
            "throttle": throttle,
            "xwind_kts": crosswind_kts,
            "gust_mps": gust_mps,
            "lat_dev_m": float(lat_dev_m),
            "dist_to_thr_m": float(dist_to_thr_m_geo),
            "dist_int_m": float(s_rem_m),
            "h_des_ft": float(h_des_ft),
            "e_alt_ft": float(e_alt_ft),
            "wow0": _w0,
            "wow1": _w1,
            "wow2": _w2,
            "wow_any": _wa
        })

        # Early termination: virtual threshold-plane gate to avoid ground physics
        tdg = cfg["sim"].get("td_gate", {})
        gate_alt_ft = float(tdg.get("alt_ft", 10.0))
        gate_dist_m = float(tdg.get("dist_m", 5.0))
        gate_tmin_s = float(tdg.get("t_min_s", 2.0))
        dist_for_gate = float(s_rem_m) if np.isfinite(s_rem_m) else float(dist_to_thr_m_geo)
        if (dist_for_gate <= gate_dist_m) and (fdm["position/h-agl-ft"] <= gate_alt_ft) and (t >= gate_tmin_s):
            break

        # Update previous lat/lon for cross-track guidance on next iteration
        lat_prev, lon_prev = lat, lon

        t += dt
        # Set dt for next integration step before calling run() again
        fdm.set_dt(dt)


    # Cleanup FG sender if used
    if fg_sender:
        try:
            fg_sender.close()
        except Exception:
            pass

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
        ("hybrid_dryden_20kt", 20.0, None),
    ]
    for name, xw, gust in cases:
        run_case(name, xw, gust)

if __name__ == "__main__":
    main()