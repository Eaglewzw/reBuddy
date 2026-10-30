#!/usr/bin/env python3
"""Offline screen-only neutral bridges for the sparse imported turn.

No joint angles or robot trajectories are inferred. Exact source endpoints are
retained, with shared neutral bridges rather than a separate clip per pair.
"""
import argparse
from pathlib import Path
import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def load(path):
    frames = []
    with Image.open(path) as gif:
        for i in range(gif.n_frames):
            gif.seek(i)
            frames.append(np.array(gif.convert('RGB').resize((240, 240), getattr(Image, 'Resampling', Image).LANCZOS)))
    return frames


def morph(a, b, fractions):
    if np.array_equal(a, b):
        return [a.copy() for _ in fractions]
    gray_a, gray_b = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY), cv2.cvtColor(b, cv2.COLOR_RGB2GRAY)
    forward = cv2.calcOpticalFlowFarneback(gray_a, gray_b, None, .5, 5, 25, 6, 7, 1.5, 0)
    backward = cv2.calcOpticalFlowFarneback(gray_b, gray_a, None, .5, 5, 25, 6, 7, 1.5, 0)
    yy, xx = np.mgrid[:a.shape[0], :a.shape[1]].astype(np.float32)
    result = []
    for t in fractions:
        if t <= 0:
            result.append(a.copy()); continue
        if t >= 1:
            result.append(b.copy()); continue
        left = cv2.remap(a, xx-t*forward[..., 0], yy-t*forward[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        right = cv2.remap(b, xx-(1-t)*backward[..., 0], yy-(1-t)*backward[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        result.append(np.clip(left.astype(float)*(1-t)+right.astype(float)*t, 0, 255).astype(np.uint8))
    return result


def generate():
    folder = ROOT/'assets/gifs/dance'
    neutral = load(folder/'01_nod.gif')[0]
    turn = load(folder/'05_base_turn_complete_drawings.gif')
    # The imported sequence starts facing away from the shared neutral.
    # Its midpoint is the closest matching orientation. Rotate the cycle.
    start = len(turn)//2
    cycle = turn[start:] + turn[:start] + [turn[start]]
    keys = [neutral] + cycle + [neutral]
    times = [0.] + list(np.linspace(300., 2050., len(cycle))) + [2350.]
    samples = np.arange(48)*50.
    output = [None]*48
    for i in range(len(keys)-1):
        indices = [n for n, t in enumerate(samples) if times[i] <= t <= times[i+1]]
        fractions = [(samples[n]-times[i])/(times[i+1]-times[i]) for n in indices]
        if i in (0, len(keys)-2):
            fractions = [t*t*t*(10-15*t+6*t*t) for t in fractions]
        frames = morph(keys[i],keys[i+1],fractions) if i in (0,len(keys)-2) else [keys[i] if t<.5 else keys[i+1] for t in fractions]
        for n, image in zip(indices, frames):
            output[n] = image
    assert all(image is not None for image in output)
    assert np.array_equal(output[0], neutral) and np.array_equal(output[-1], neutral)
    return np.stack(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    np.savez_compressed(args.output, frames=generate())
