"""Pure recovery math mirrored by rb_recovery_policy.h; no transport access."""
import numpy as np


def damping_gain(gravity, velocity, kd, cap):
    gravity, velocity, kd, cap = np.broadcast_arrays(gravity, velocity, kd, cap)
    estimate = gravity - kd * velocity
    bounded = kd.copy()
    mask = abs(estimate) > cap
    moving = mask & (abs(velocity) >= 1e-8)
    bounded[mask & ~moving] = 0.
    bounded[moving] = np.clip((gravity[moving] - np.copysign(cap[moving] * .98, estimate[moving])) /
                             velocity[moving], 0., kd[moving])
    return bounded


def filtered_velocity(previous, delta, dt):
    return previous + dt / (.04 + dt) * (delta / dt - previous)


class RecoverableControlError(RuntimeError):
    def __init__(self, message, details=None):
        self.details = details or []
        super().__init__(message)


class ArrivalTimeout(RecoverableControlError):
    pass


class FollowingError(RecoverableControlError):
    pass
