import math
import numpy as np

# --- Simple deterministic gust ---
def one_cosine_gust(t, t0, duration, Umax_mps):
    """
    1-cosine gust profile (m/s):
      starts at t0, lasts 'duration', peaks at Umax_mps.
    """
    if t < t0:
        return 0.0
    tau = (t - t0) / max(duration, 1e-6)
    if tau >= 1.0:
        return 0.0
    return 0.5 * Umax_mps * (1 - np.cos(np.pi * tau))

# --- Conversions ---
def kts_to_mps(kts): return kts * 0.514444

def mps_to_fps(mps): return mps * 3.28084

# --- Helper math ---
def deg2rad(d): return d * np.pi / 180.0

def heading_to_unit_xy(heading_deg):
    """
    Convert heading (deg, clockwise from North) to unit vector in EN (x,y) frame.
    Returns (ex, ey) where ex is North, ey is East components.
    """
    r = deg2rad(heading_deg)
    # Heading 0 -> North (1,0), 90 -> East (0,1)
    return np.cos(r), np.sin(r)

# --- Dryden-like colored noise (compact AR(1)/AR(2) approximation) ---
class Dryden3D:
    """
    Compact Dryden-like turbulence generator.
    Uses AR filters with time constants tau = L / Va to shape white noise.
    Parameters are given as RMS intensities (sigma_*) and scale lengths (L_*), all in SI.
    """
    def __init__(self, Va_mps=65.0,
                 sigma_u=1.0, sigma_v=1.0, sigma_w=0.5,
                 Lu=200.0, Lv=200.0, Lw=50.0, seed: int | None = 1):
        self.Va = max(Va_mps, 1.0)
        self.sig = np.array([sigma_u, sigma_v, sigma_w], dtype=float)
        self.L = np.array([Lu, Lv, Lw], dtype=float)
        self.rng = np.random.default_rng(seed)
        # AR(1) states for u,v and AR(2) for w (to capture vertical spectrum better)
        self.x = np.zeros(3)
        self.w_state = np.zeros(2)

    def reset(self):
        self.x[:] = 0.0
        self.w_state[:] = 0.0

    def step(self, dt):
        dt = max(dt, 1e-4)
        # Time constants tau = L / Va
        tau = self.L / self.Va
        a = np.exp(-dt / np.maximum(tau, 1e-6))  # AR(1) coeffs in [0,1)
        # Drive white noise
        n = self.rng.standard_normal(3)
        # Scale so stationary variance ~ sigma^2
        # For AR(1): x_{k+1} = a x_k + b n, var = b^2 / (1-a^2)
        b = self.sig * np.sqrt(np.maximum(1 - a*a, 1e-6))
        self.x = a * self.x + b * n

        # Vertical channel: simple AR(2) for richer spectrum
        # y_{k+1} = c1 y_k + c2 y_{k-1} + g n
        # Choose c1,c2 to give lightly underdamped pole pair; tune by Lw
        wn = 1.0 / max(tau[2], 0.5)
        zeta = 0.6
        c1 = 2*np.exp(-zeta*wn*dt)*np.cos(wn*dt*np.sqrt(max(1-zeta*zeta, 1e-6)))
        c2 = -np.exp(-2*zeta*wn*dt)
        g = self.sig[2] * 0.3  # small drive; empirically stable
        yk, ykm1 = self.w_state
        y_next = c1*yk + c2*ykm1 + g*self.rng.standard_normal()
        self.w_state = np.array([y_next, yk])

        return np.array([self.x[0], self.x[1], self.w_state[0]], dtype=float)

# --- Log-layer wind shear aligned with runway heading ---
class LogLayerShear:
    """
    U(z) = U_ref * ln((z+z0)/z0) / ln((z_ref+z0)/z0), limited to z>=0.
    Direction is set by heading_deg (clockwise from North).
    """
    def __init__(self, Uref_mps=10.0, z_ref_m=10.0, z0_m=0.1, heading_deg=90.0):
        self.Uref = float(Uref_mps)
        self.z_ref = max(float(z_ref_m), 0.1)
        self.z0 = max(float(z0_m), 1e-3)
        self.heading = float(heading_deg)
        self.en = heading_to_unit_xy(self.heading)  # along-runway unit (north,east)
        # Crosswind is 90 deg to runway heading (to the right of heading)
        self.ec = heading_to_unit_xy(self.heading + 90.0)

    def speed_at(self, z_m: float) -> float:
        z = max(z_m, 0.0)
        num = math.log((z + self.z0) / self.z0)
        den = math.log((self.z_ref + self.z0) / self.z0)
        return self.Uref * (num / max(den, 1e-6))

    def vector_EN(self, z_m: float, use_crosswind=True):
        U = self.speed_at(z_m)
        ex_n, ex_e = (self.ec if use_crosswind else self.en)
        return np.array([U*ex_n, U*ex_e])

# --- Monin–Obukhov stability-corrected surface layer shear ---
class StabilityCorrectedShear:
    """
    U(z) = Uref * [ln(z/z0) - psi_m(z/L) + psi_m(z0/L)] / A
    where A = ln(z_ref/z0) - psi_m(z_ref/L) + psi_m(z0/L).
    If |L| is very large, reduces to neutral log layer.
    Direction set by heading_deg (clockwise from North).
    """
    def __init__(self, Uref_mps=10.0, z_ref_m=10.0, z0_m=0.1, L_m=float("inf"), heading_deg=90.0):
        self.Uref = float(Uref_mps)
        self.z_ref = max(float(z_ref_m), 0.1)
        self.z0 = max(float(z0_m), 1e-3)
        self.L = float(L_m)
        self.kappa = 0.4
        self.heading = float(heading_deg)
        self.en = heading_to_unit_xy(self.heading)
        self.ec = heading_to_unit_xy(self.heading + 90.0)
        # Precompute denominator A
        self._A = self._phi(self.z_ref) - self._phi(self.z0)
        if abs(self._A) < 1e-6:
            self._A = 1e-6

    def _psi_m(self, z):
        # Stability correction for momentum
        if not np.isfinite(self.L) or abs(self.L) > 1e8:
            return 0.0  # neutral
        if self.L > 0.0:  # stable
            return -5.0 * z / max(self.L, 1e-6)
        # unstable L < 0 (Businger-Dyer form)
        x = (1.0 - 16.0 * z / min(self.L, -1e-6)) ** 0.25
        return 2.0 * math.log((1 + x) / 2.0) + math.log((1 + x*x) / 2.0) - 2.0 * math.atan(x) + math.pi/2.0

    def _phi(self, z):
        zc = max(z, self.z0)
        return math.log(zc / self.z0) - self._psi_m(zc)

    def speed_at(self, z_m: float) -> float:
        z = max(z_m, 0.0)
        num = self._phi(z) - self._phi(self.z0)
        return self.Uref * (num / self._A)

    def vector_EN(self, z_m: float, use_crosswind=True):
        U = self.speed_at(z_m)
        ex_n, ex_e = (self.ec if use_crosswind else self.en)
        return np.array([U*ex_n, U*ex_e])


# --- Hybrid model: shear + Dryden, with near-ground attenuation ---
class HybridWindModel:
    def __init__(self, shear: LogLayerShear, turb: Dryden3D,
                 turb_decay_h_m=30.0, use_crosswind_axis=True):
        self.shear = shear
        self.turb = turb
        self.h = max(turb_decay_h_m, 1.0)
        self.use_cross = use_crosswind_axis

    def step_ned(self, dt, z_agl_m: float):
        # Base shear (EN -> NE components)
        en = self.shear.vector_EN(z_agl_m, use_crosswind=self.use_cross)
        # Turbulence with exponential decay near ground
        decay = 1.0 - np.exp(-max(z_agl_m, 0.0) / self.h)
        uvd = self.turb.step(dt) * decay  # (u,v,w) in body-aligned EN approx
        # Map EN -> NE (north,east), and w is vertical (down negative of vertical wind)
        north = en[0] + uvd[0]
        east  = en[1] + uvd[1]
        down  = -uvd[2]  # positive up gust -> negative down component
        return float(north), float(east), float(down)