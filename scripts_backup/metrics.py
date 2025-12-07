import numpy as np
import pandas as pd

def touchdown_index(alt_series, thresh_ft=2.0):
    """Legacy: first index where AGL altitude <= threshold (touchdown)."""
    idx = np.where(np.asarray(alt_series) <= thresh_ft)[0]
    return int(idx[0]) if len(idx) else None

def _wow_any_series(df: pd.DataFrame):
    """Return a boolean-like numpy array for any WOW contact if present; else None."""
    cols = [c for c in df.columns if c.lower() in ("wow_any", "wow0", "wow1", "wow2", "wow_nose", "wow_main_l", "wow_main_r")]
    if not cols:
        return None
    if "wow_any" in df.columns:
        return (df["wow_any"].values.astype(float) > 0.5)
    acc = np.zeros(len(df), dtype=bool)
    for c in cols:
        try:
            acc |= (df[c].values.astype(float) > 0.5)
        except Exception:
            pass
    return acc

def robust_touchdown_index(df: pd.DataFrame, alt_ft_thresh: float = 2.0, dist_m_thresh: float = 60.0, t_min: float = 2.0,
                            virtual_gate_alt_ft: float = 10.0, virtual_gate_m: float = 5.0):
    """
    Virtual threshold-plane gate with WOW-aware and robust fallbacks:
    Order of preference:
      1) Virtual threshold-plane crossing: dist<=virtual_gate_m AND alt<=virtual_gate_alt_ft AND t>=t_min
      2) WOW asserted with gating: t>=t_min and dist<=dist_m_thresh
      3) Runway threshold gate: first time dist<=dist_m_thresh with t>=max(t_min, 10s)
      4) Altitude-based fallback: alt<=alt_ft_thresh and dist<=dist_m_thresh and t>=t_min
      5) Last resort: index of minimum distance-to-threshold

    Notes:
      - Prefer integrated distance column 'dist_int_m' when present; fall back to 'dist_to_thr_m' (geo) otherwise.
    """
    # Choose best distance signal
    dist_col = None
    if "dist_int_m" in df.columns:
        dist_col = "dist_int_m"
    elif "dist_to_thr_m" in df.columns:
        dist_col = "dist_to_thr_m"

    # 1) Virtual threshold-plane crossing (use geo distance if available)
    if dist_col:
        cond = (df[dist_col].values <= float(virtual_gate_m))
        cond &= (df["alt_ft"].values <= float(virtual_gate_alt_ft))
        if "t" in df.columns:
            cond &= (df["t"].values >= t_min)
        idx = np.where(cond)[0]
        if len(idx):
            return int(idx[0])

    # 2) WOW-based with gating
    wow_any = _wow_any_series(df)
    if wow_any is not None:
        cond = wow_any
        if "t" in df.columns:
            cond &= (df["t"].values >= t_min)
        if dist_col:
            cond &= (df[dist_col].values <= dist_m_thresh)
        idx = np.where(cond)[0]
        if len(idx):
            return int(idx[0])

    # 3) Runway threshold gate (distance only)
    if dist_col:
        t_gate_min = max(t_min, 10.0)
        cond = (df[dist_col].values <= dist_m_thresh)
        if "t" in df.columns:
            cond &= (df["t"].values >= t_gate_min)
        idx = np.where(cond)[0]
        if len(idx):
            return int(idx[0])

    # 4) Altitude+distance fallback
    cond = (df["alt_ft"] <= alt_ft_thresh)
    if dist_col:
        cond &= (df[dist_col] <= dist_m_thresh)
    if "t" in df.columns:
        cond &= (df["t"] >= t_min)
    idx = np.where(cond.values)[0]
    if len(idx):
        return int(idx[0])

    # 5) Last-resort: minimum distance
    if dist_col and len(df):
        try:
            return int(df[dist_col].idxmin())
        except Exception:
            pass
    return None

def compute_touchdown_metrics(df: pd.DataFrame):
    """Compute a few key metrics at touchdown (if any)."""
    i = robust_touchdown_index(df)
    if i is None:
        # Fallback to legacy method if robust condition fails
        i = touchdown_index(df["alt_ft"]) if "alt_ft" in df.columns else None
    if i is None:
        return {"touched_down": False}
    r = df.iloc[i]
    return {
        "touched_down": True,
        "t": float(r.get("t", np.nan)),
        "alt_ft": float(r.get("alt_ft", np.nan)),
        "roll_deg": float(r.get("phi_deg", np.nan)),
        "sink_fps": float(r.get("vd_fps", np.nan)),    # positive down
        "lat_dev_m": float(r.get("lat_dev_m", float("nan")))
    }