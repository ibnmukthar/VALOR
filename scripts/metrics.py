import numpy as np
import pandas as pd

def touchdown_index(alt_series, thresh_ft=2.0):
    """Return first index where AGL altitude <= threshold (touchdown)."""
    idx = np.where(np.asarray(alt_series) <= thresh_ft)[0]
    return int(idx[0]) if len(idx) else None

def compute_touchdown_metrics(df: pd.DataFrame):
    """Compute a few key metrics at touchdown (if any)."""
    i = touchdown_index(df["alt_ft"])
    if i is None:
        return {"touched_down": False}
    r = df.iloc[i]
    return {
        "touched_down": True,
        "t": float(r["t"]),
        "alt_ft": float(r["alt_ft"]),
        "roll_deg": float(r["phi_deg"]),
        "sink_fps": float(r["vd_fps"]),    # positive down
        "lat_dev_m": float(r.get("lat_dev_m", float("nan")))
    }