import os, math, time
import numpy as np
import jsbsim

# --- Sim constants (tweak as you like) ---
DT = 0.01             # 100 Hz
SIM_TIME = 30.0       # seconds
XWIND_KT = 20.0       # crosswind component (from right), knots

# --- Helpers ---
def kt_to_mps(kts: float) -> float:
    return kts * 0.514444

def set_initial_short_final(fg):
    # Short final: 2 NM out, ~200 ft AGL, landing config-ish
    # Positioning is illustrative; adapt to your airport/runway later.
    fg['ic/altitude-ft'] = 200.0 + 13.0     # AGL + field elev approx
    fg['ic/alpha-deg']   = 3.0
    fg['ic/beta-deg']    = 0.0
    fg['ic/psi-true-deg']= 90.0             # runway 09 example
    fg['ic/theta-deg']   = 3.0
    fg['ic/phi-deg']     = 0.0
    fg['ic/vtrue-kts']   = 130.0
    # Place roughly 2 NM out on extended centerline (you’ll refine with real coords)
    fg['ic/long-gc-deg'] = -73.7800
    fg['ic/lat-gc-deg']  = 40.6400

def apply_simple_crosswind(fg, xwind_kts):
    # JSBSim has wind models; simplest is to set ambient wind NED components.
    # Assume runway heading ~090 deg; crosswind from right = southerly wind.
    # NED: (north, east, down) m/s
    v = kt_to_mps(xwind_kts)
    fg['atmosphere/wind-north-fps'] = 0.0
    fg['atmosphere/wind-east-fps']  = v * 3.28084  # m/s to ft/s
    fg['atmosphere/wind-down-fps']  = 0.0

def main():
    # Load default aircraft (C172 or your jet once you pick one)
    # JSBSim ships with sample aircraft under its data path.
    fdm = jsbsim.FGFDMExec(None)
    fdm.set_root_dir(jsbsim.get_default_root_dir())
    # Choose an aircraft present in your JSBSim data install; c172p is common:
    if not fdm.load_model("c172p"):
        raise RuntimeError("Failed to load aircraft model (c172p).")

    # Initial conditions & wind
    set_initial_short_final(fdm)
    fdm.run_ic()  # initialize
    apply_simple_crosswind(fdm, XWIND_KT)

    t = 0.0
    log = []

    while t < SIM_TIME:
        # Simple “hold” controls (placeholder: you’ll replace with PID/LQR)
        # Keep wings level; slight nose-up; throttle modest:
        fdm['fcs/aileron-cmd-norm']  = 0.0
        fdm['fcs/elevator-cmd-norm'] = -0.05
        fdm['fcs/rudder-cmd-norm']   = 0.0
        fdm['fcs/throttle-cmd-norm'] = 0.3

        ok = fdm.run()
        if not ok:
            break

        # Record a few states
        log.append({
            't': t,
            'lat_deg': fdm['position/lat-gc-deg'],
            'lon_deg': fdm['position/long-gc-deg'],
            'alt_ft':  fdm['position/h-agl-ft'],
            'phi_deg': fdm['attitude/phi-deg'],
            'theta_deg': fdm['attitude/theta-deg'],
            'psi_deg': fdm['attitude/psi-deg'],
            'vn_fps':  fdm['velocities/v-north-fps'],
            've_fps':  fdm['velocities/v-east-fps'],
            'vd_fps':  fdm['velocities/v-down-fps'],
        })

        t += DT
        fdm.set_dt(DT)

    print(f"Ran {len(log)} steps at {1/DT:.0f} Hz with {XWIND_KT} kt crosswind.")
    # Later: save to CSV and plot in matplotlib

if __name__ == "__main__":
    main()