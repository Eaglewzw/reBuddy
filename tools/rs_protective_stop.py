"""Checked recording fault stop: brake once, latch position, keep serving CAN."""
import math
import queue
import time

import numpy as np

from rs_gravity_play import Controller


class ProtectiveStopRequested(RuntimeError):
    pass


def maintain_hold(backend, gravity, commands, stop, journal, verify):
    """Return only on explicit home/quit/release; invalid feedback raises.

    No enable, disable or automatic resumption. Caller owns final cleanup.
    Storage failures cannot interrupt holding. Feedback/control failures can.
    """
    def event(name, **values):
        try:
            journal.put(dict(event=name, monotonic_ns=time.monotonic_ns(), **values))
        except Exception:
            pass

    controller = Controller(gravity)
    started = time.monotonic()
    last_tick = started
    stable = None
    anchor = None
    brake_kd = None
    ramp_at = None
    announced = False
    stop[0] = None
    event('protective_stop_requested')
    print('正在制动保持；home/quit 回零退出，release 明确失能。', flush=True)
    while True:
        now = time.monotonic()
        if now-last_tick > .1:
            raise RuntimeError('protective hold control deadline exceeded')
        last_tick = now
        if stop[0]:
            stop[0] = None
            print('保持中；home/quit 回零退出，release 失能。', flush=True)
        try:
            word = commands.get_nowait().strip().lower()
        except queue.Empty:
            word = ''
        # Read and validate before any transition out of protective control.
        backend.heartbeat()
        q, begin, end = backend.read()
        verify()
        gain = 0. if anchor is None else min(1., (now-ramp_at)/.5)
        command = controller.recovery_command(q, end, anchor=anchor, gain=gain, brake_kd=brake_kd)
        if time.monotonic_ns()-begin > 100_000_000:
            raise RuntimeError('protective hold feedback expired before send')
        backend.send(command)
        event('protective_stop_command', phase='braking' if anchor is None else 'holding', **command)
        if anchor is None:
            still = np.max(abs(controller.velocity)) <= math.radians(1.)
            stable = (stable if stable is not None else now) if still else None
            if stable is not None and now-stable >= .5:
                anchor = q.copy()
                brake_kd = np.array(command['kd'])
                ramp_at = now
            elif now-started > 5.:
                raise RuntimeError('protective braking did not stop within 5s')
        elif gain >= 1. and not announced:
            announced = True
            event('protective_hold_entered', target_rad=anchor.tolist())
            print('已固定姿态，保持力矩；不会自动恢复录制。', flush=True)
        if word == 'release':
            return 'release'
        if word in ('home', 'quit'):
            # Only leave for homing after the braking and gain ramp complete.
            if announced:
                return 'home'
            print('尚在制动，请待固定姿态后再次输入 home/quit。', flush=True)
        elif word and word != 'eof':
            print('保持中；home/quit 回零退出，release 失能。', flush=True)
        time.sleep(.02)
