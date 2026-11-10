"""Offline compilation of the ten existing dances into one C2 joint trajectory.

Source plans are immutable. Only small entry/exit windows are replaced; gripper
pulses and the pivot's constrained body are never trimmed. No CAN imports here.
"""
import math
import numpy as np
from rs_record_playback import LOWER, UPPER
from rs_gravity_record import CAPS

MAX_TRIM_S = .45
BRIDGE_JERK_LIMIT = 20.  # rad/s^3, transition design budget, not a motor rating


def state(curve, t):
    q, v = curve.at(t)
    return np.asarray([q, v, curve.acceleration_at(t)], dtype=float)


class QuinticBridge:
    def __init__(self, left, right, duration):
        left, right = np.asarray(left, float), np.asarray(right, float)
        self.duration = float(duration)
        if (left.shape != (3, 7) or right.shape != (3, 7)
                or not np.isfinite([left, right]).all()
                or not math.isfinite(self.duration) or self.duration <= 0):
            raise ValueError('Invalid bridge boundary states or duration')
        T = self.duration
        b0, b1, b2 = left[0], T*left[1], .5*T*T*left[2]
        D, V, A = right[0]-b0-b1-b2, T*right[1]-b1-2*b2, T*T*right[2]-2*b2
        self.coefficients = np.array([b0, b1, b2, 10*D-4*V+.5*A,
                                      -15*D+7*V-A, 6*D-3*V+.5*A])
        self.start, self.end = left[0].copy(), right[0].copy()
        self.lower, self.upper = np.zeros(7), np.zeros(7)
        self.peak_velocity, self.peak_acceleration, self.peak_jerk = (np.zeros(7) for _ in range(3))
        for joint in range(7):
            polynomial = np.polynomial.Polynomial(self.coefficients[:, joint])
            for order in range(4):
                roots = polynomial.deriv(order+1).roots()
                points = [0., 1.] + [float(r.real) for r in roots
                                          if abs(r.imag) < 1e-8 and 0 < r.real < 1]
                values = polynomial.deriv(order)(points) / T**order
                if order == 0:
                    self.lower[joint], self.upper[joint] = min(values), max(values)
                else:
                    (self.peak_velocity, self.peak_acceleration, self.peak_jerk)[order-1][joint] = max(abs(values))

    def derivative(self, t, order):
        coefficients = np.polynomial.polynomial.polyder(self.coefficients, m=order)
        return np.polynomial.polynomial.polyval(np.clip(t/self.duration, 0., 1.), coefficients) / self.duration**order

    def at(self, t):
        return self.derivative(t, 0), self.derivative(t, 1)

    def acceleration_at(self, t):
        return self.derivative(t, 2)


class ClipWindow:
    def __init__(self, source, start, end):
        if not 0 <= start < end <= source.duration:
            raise ValueError('Invalid retained action window')
        self.source, self.offset, self.duration = source, start, end-start
        self.start, self.end = source.at(start)[0], source.at(end)[0]
        self.peak_velocity, self.peak_acceleration = source.peak_velocity, source.peak_acceleration

    def at(self, t):
        return self.source.at(self.offset+float(np.clip(t, 0, self.duration)))

    def acceleration_at(self, t):
        return self.source.acceleration_at(self.offset+float(np.clip(t, 0, self.duration)))


def trim_windows(plan):
    """Return maximum removable entry/exit seconds, protecting special phases."""
    entry = exit_ = min(MAX_TRIM_S, .1*plan.duration)
    if len(plan.stages) == 1:
        curve = plan.stages[0][1]
        if hasattr(curve, 'pulse_starts_s'):
            entry = min(entry, float(curve.pulse_starts_s[0])*.8)
            exit_ = min(exit_, (plan.duration-float(curve.pulse_starts_s[-1])-curve.pulse.duration)*.8)
        if hasattr(curve, 'dance_interval_s'):
            begin, end = curve.dance_interval_s
            entry, exit_ = min(entry, .4*begin), min(exit_, .4*(plan.duration-end))
    if min(entry, exit_) <= 0:
        raise ValueError('No removable entry/exit window')
    return entry, exit_


def check_bridge(curve, gravity, collision):
    if np.any(curve.lower < LOWER-1e-9) or np.any(curve.upper > UPPER+1e-9):
        raise ValueError('bridge joint bound exceeded')
    if (np.max(curve.peak_velocity[:6]) > 1.+1e-9
            or np.max(curve.peak_acceleration[:6]) > 2.+1e-9
            or np.max(curve.peak_jerk[:6]) > BRIDGE_JERK_LIMIT+1e-9):
        raise ValueError('bridge velocity/acceleration/jerk budget exceeded')
    # Pulses stay inside retained actions; transitions must keep J7 closed.
    if max(abs(curve.lower[6]), abs(curve.upper[6])) > 1e-8:
        raise ValueError('bridge would cut a gripper pulse')
    return audit_samples(curve, gravity, collision, step=.01)


def audit_samples(curve, gravity, collision, step=.02, extra_times=()):
    times = np.unique(np.r_[np.linspace(0, curve.duration, math.ceil(curve.duration/step)+1), extra_times])
    peak_gravity, peak_inverse = np.zeros(6), np.zeros(6)
    peak_v, peak_a = np.zeros(7), np.zeros(7)
    data = gravity.model.createData()
    open_samples = 0
    for t in times:
        q, v, a = state(curve, t)
        if not np.isfinite([q, v, a]).all():
            raise ValueError('non-finite trajectory')
        if np.any(q < LOWER-1e-8) or np.any(q > UPPER+1e-8):
            raise ValueError(f'joint limit at {t:.3f}s')
        if np.any(q[:6] < gravity.lower-1e-8) or np.any(q[:6] > gravity.upper+1e-8):
            raise ValueError(f'model joint limit at {t:.3f}s')
        tau = np.asarray(gravity(q[:6]))
        inverse = gravity.pin.rnea(gravity.model, data, q[:6], v[:6], a[:6])
        if not np.isfinite([tau, inverse]).all() or np.any(abs(tau) > CAPS[:6]) or np.any(abs(inverse) > CAPS[:6]):
            raise ValueError(f'model torque budget at {t:.3f}s')
        # Local collision model has no calibrated J7 motor-to-finger mapping.
        # For retained open-gripper motions this is explicitly a closed-finger
        # proxy, never presented as an open-gripper or environment guarantee.
        proxy = q.copy()
        open_samples += int(abs(proxy[6]) > 1e-8)
        proxy[6] = 0.
        if collision.check(proxy)['colliding']:
            raise ValueError(f'self-collision (closed-finger model) at {t:.3f}s')
        peak_gravity = np.maximum(peak_gravity, abs(tau))
        peak_inverse = np.maximum(peak_inverse, abs(inverse))
        peak_v, peak_a = np.maximum(peak_v, abs(v)), np.maximum(peak_a, abs(a))
    return dict(samples=len(times),check_period_s=step,gravity_peak_nm=peak_gravity.tolist(),
                inverse_dynamics_peak_nm=peak_inverse.tolist(),sampled_peak_velocity_rad_s=peak_v.tolist(),
                sampled_peak_acceleration_rad_s2=peak_a.tolist(),
                self_collision_scope='closed-finger URDF mesh model; sampled, not continuous collision detection',
                open_gripper_proxy_samples=open_samples,open_finger_geometry_checked=False,environment_checked=False)


def connect(left, right, gravity, collision):
    _, max_left = trim_windows(left)
    max_right, _ = trim_windows(right)
    failures = []
    # Try full windows first. Search both shorter and longer bridges; longer
    # time alone need not fix overshoot with nonzero boundary velocities.
    for fraction in (1., .75, .5):
        trim_left, trim_right = max_left*fraction, max_right*fraction
        begin, end = state(left, left.duration-trim_left), state(right, trim_right)
        for factor in (.85, 1., 1.25, 1.5, 2.):
            curve = QuinticBridge(begin, end, (trim_left+trim_right)*factor)
            try:
                checks = check_bridge(curve, gravity, collision)
            except ValueError as error:
                failures.append(str(error))
                continue
            report = dict(left_trim_s=trim_left,right_trim_s=trim_right,duration_s=curve.duration,
                          coefficients_normalized=curve.coefficients.tolist(),
                          left_state=begin.tolist(),right_state=end.tolist(),
                          peak_velocity_rad_s=curve.peak_velocity.tolist(),
                          peak_acceleration_rad_s2=curve.peak_acceleration.tolist(),
                          peak_jerk_rad_s3=curve.peak_jerk.tolist(),validation=checks)
            return curve, report
    raise ValueError('No feasible quintic connection: '+ '; '.join(sorted(set(failures))))


class SequencePlan:
    def __init__(self, numbers, plans, gravity, collision, cache=None):
        self.numbers = list(numbers)
        if not 1 <= len(numbers) <= 32 or any(n not in {f'{i:02}' for i in range(1,11)} for n in numbers):
            raise ValueError('请选择 1～32 个动作，编号仅限 01～10')
        cache = {} if cache is None else cache
        joins = []
        for left, right in zip(numbers, numbers[1:]):
            key = (left, right)
            if key not in cache:cache[key] = connect(plans[left], plans[right], gravity, collision)
            joins.append(cache[key])
        self.parts, self.layout, self.connections = [], [], []
        elapsed = 0.
        for i, number in enumerate(numbers):
            source = plans[number]
            start = joins[i-1][1]['right_trim_s'] if i else 0.
            end = source.duration-(joins[i][1]['left_trim_s'] if i < len(joins) else 0.)
            window = ClipWindow(source, start, end)
            self.parts.append(window)
            self.layout.append(dict(kind='action',number=number,step=i+1,start_s=elapsed,
                                    duration_s=window.duration,source_start_s=start,source_end_s=end))
            elapsed += window.duration
            if i < len(joins):
                bridge, report = joins[i]
                self.parts.append(bridge)
                self.layout.append(dict(kind='bridge',from_number=number,to_number=numbers[i+1],
                                        start_s=elapsed,duration_s=bridge.duration))
                self.connections.append(dict(from_number=number,to_number=numbers[i+1],start_s=elapsed,**report))
                elapsed += bridge.duration
        self.ends = np.cumsum([p.duration for p in self.parts])
        self.duration = float(self.ends[-1])
        self.start, self.end = self.parts[0].start.copy(), self.parts[-1].end.copy()
        self.peak_velocity = np.max([p.peak_velocity for p in self.parts], axis=0)
        self.peak_acceleration = np.max([p.peak_acceleration for p in self.parts], axis=0)
        self.velocity_limit, self.acceleration_limit = 1., 2.
        self.gripper_velocity_limit = max(plans[n].gripper_velocity_limit for n in numbers)
        self.gripper_acceleration_limit = max(plans[n].gripper_acceleration_limit for n in numbers)
        self.gripper_following_limit = max(getattr(plans[n], 'gripper_following_limit', 0.) for n in numbers)
        self.gripper_motion = any(plans[n].gripper_motion for n in numbers)
        if not self.gripper_motion:del self.gripper_following_limit
        # Session.execute_plan sees ONE segment, not one per action. Thus its
        # existing sampler uses a single monotonic clock with no internal settle.
        self.stages = [('continuous_sequence', self)]
        gaps = [np.max(abs(state(a, a.duration)-state(b, 0)), axis=1)
                for a, b in zip(self.parts, self.parts[1:])]
        self.seam_errors = np.max(gaps, axis=0) if gaps else np.zeros(3)
        if np.max(self.seam_errors) > 1e-7:
            raise ValueError('Sequence position/velocity/acceleration seam')
        if max(np.max(abs(state(self, t)[1:])) for t in (0., self.duration)) > 1e-7:
            raise ValueError('Sequence must start and end at rest')

    def locate(self, t):
        t = float(np.clip(t, 0., self.duration))
        i = min(int(np.searchsorted(self.ends, t, side='right')), len(self.parts)-1)
        return self.parts[i], t-(self.ends[i-1] if i else 0.)

    def at(self, t):
        curve, t = self.locate(t)
        return curve.at(t)

    def acceleration_at(self, t):
        curve, t = self.locate(t)
        return curve.acceleration_at(t)

    def gripper_profile_at(self, t):
        from rs_gravity_play import FOLLOW_TOLERANCE
        curve, _ = self.locate(t)
        if isinstance(curve, ClipWindow):
            source = curve.source
            return source.gripper_velocity_limit, getattr(source, 'gripper_following_limit', FOLLOW_TOLERANCE)
        return 1., FOLLOW_TOLERANCE

    def audit(self, gravity, collision):
        report = audit_samples(self, gravity, collision, extra_times=self.ends)
        if (np.max(self.peak_velocity[:6]) > self.velocity_limit+1e-8
                or np.max(self.peak_acceleration[:6]) > self.acceleration_limit+1e-8
                or self.peak_velocity[6] > self.gripper_velocity_limit+1e-8
                or self.peak_acceleration[6] > self.gripper_acceleration_limit+1e-8):
            raise ValueError('Sequence dynamic bounds exceeded')
        report.update(seam_max_errors_q_v_a=self.seam_errors.tolist(),
                      velocity_upper_bound_rad_s=self.peak_velocity.tolist(),
                      acceleration_upper_bound_rad_s2=self.peak_acceleration.tolist(),
                      bridge_jerk_budget_rad_s3=BRIDGE_JERK_LIMIT,
                      source_bodies_preserved=True,internal_settle_calls=0,
                      collision_identity=collision.identity)
        return report
