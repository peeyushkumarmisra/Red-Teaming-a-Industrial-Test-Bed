import numpy as np

QUINTIC_PEAK_VELOCITY = 1.875
QUINTIC_PEAK_ACCELERATION = 10.0 / np.sqrt(3.0)


def quintic(q0, q1, duration, t):
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    if duration <= 0.0 or t >= duration:
        return q1.copy(), np.zeros_like(q1), np.zeros_like(q1)
    s = max(t, 0.0) / duration
    d = q1 - q0
    position = q0 + d * (10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5)
    velocity = d * (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / duration
    acceleration = d * (60.0 * s - 180.0 * s**2 + 120.0 * s**3) / duration**2
    return position, velocity, acceleration


def move_duration(q0, q1, max_velocity, max_acceleration, minimum=0.0):
    d = np.abs(np.asarray(q1, dtype=float) - np.asarray(q0, dtype=float))
    by_velocity = QUINTIC_PEAK_VELOCITY * d / np.asarray(max_velocity, dtype=float)
    limit = np.asarray(max_acceleration, dtype=float)
    by_acceleration = np.sqrt(QUINTIC_PEAK_ACCELERATION * d / limit)
    return float(max(minimum, by_velocity.max(initial=0.0), by_acceleration.max(initial=0.0)))