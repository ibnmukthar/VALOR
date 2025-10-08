import numpy as np

def one_cosine_gust(t, t0, duration, Umax_mps):
    """
    1-cosine gust profile (m/s):
      starts at t0, lasts 'duration', peaks at Umax_mps.
    """
    if t < t0:
        return 0.0
    tau = (t - t0) / duration
    if tau >= 1.0:
        return 0.0
    return 0.5 * Umax_mps * (1 - np.cos(np.pi * tau))

def kts_to_mps(kts): return kts * 0.514444
def mps_to_fps(mps): return mps * 3.28084