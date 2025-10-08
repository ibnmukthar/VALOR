import numpy as np

def one_cosine_gust(t, t_start, duration, Umax):
    """ Simple 1-cosine gust profile (start at t_start, last duration seconds). """
    if t < t_start: return 0.0
    tau = (t - t_start) / duration
    if tau > 1.0: return 0.0
    return Umax * 0.5 * (1 - np.cos(np.pi * tau))