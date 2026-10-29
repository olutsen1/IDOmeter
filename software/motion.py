"""
Motion-artefact detection in the PPG.

A moving finger or a pressed sensor shows up in the pulse wave as a short
burst far larger than the usual beat-to-beat swing, or as the pulse almost
vanishing. Such stretches would otherwise look like deep breaths in the
PPG-derived breathing curve and disturb heart rate.

Method: the PPG is band-passed to 0.5–5 Hz (pulse and motion, without slow
drift); its local amplitude is the peak-to-peak range in a 1.5 s window (at
least one beat at >= 40 bpm). This is compared with the median local
amplitude over the surrounding 60 s. A stretch is flagged when it is more
than HIGH times larger, or less than LOW times smaller (signal lost). The
flag is widened by PAD_BEFORE / PAD_AFTER seconds to cover the settling.

LOW is deliberately small: after a deep breath the finger vessels constrict
and the pulse amplitude can drop to ~40 % for several seconds; that is
physiology, not an artefact, and must not be flagged.
"""
import numpy as np
from scipy.ndimage import maximum_filter1d, median_filter, minimum_filter1d
from scipy.signal import butter, sosfiltfilt

HIGH = 3.0
LOW = 0.15
WINDOW_S = 1.5
REF_S = 60
PAD_BEFORE = 1.5
PAD_AFTER = 2.5
_FS_OUT = 10
_SOS = butter(2, (0.5, 5.0), btype="band", fs=100, output="sos")


def detect(t, ir, fs=100):
    """Return a list of (start, end) times of motion artefacts.

    t, ir: PPG time stamps (s) and raw IR samples. NaN gaps are bridged.
    """
    t = np.asarray(t, float)
    x = np.asarray(ir, float)
    ok = np.isfinite(t) & np.isfinite(x)
    if ok.sum() < 10 * fs:
        return []
    t, x = t[ok], x[ok]
    y = sosfiltfilt(_SOS, x - x.mean())
    w = max(3, int(WINDOW_S * fs))
    amp = maximum_filter1d(y, w) - minimum_filter1d(y, w)
    # work at 10 Hz for the slow reference
    k = fs // _FS_OUT
    n = len(amp) // k * k
    a = amp[:n].reshape(-1, k).max(1)
    ta = t[:n].reshape(-1, k).mean(1)
    ref = median_filter(a, size=min(len(a) | 1, REF_S * _FS_OUT + 1), mode="nearest")
    bad = (a > HIGH * ref) | (a < LOW * ref)
    return _segments(ta, bad)


def _segments(ta, bad):
    segs = []
    i = 0
    while i < len(bad):
        if bad[i]:
            j = i
            while j + 1 < len(bad) and bad[j + 1]:
                j += 1
            s, e = ta[i] - PAD_BEFORE, ta[j] + PAD_AFTER
            if segs and s <= segs[-1][1]:
                segs[-1] = (segs[-1][0], max(e, segs[-1][1]))
            else:
                segs.append((s, e))
            i = j + 1
        else:
            i += 1
    return segs


def mask(times, segs):
    """True where a time falls inside any artefact segment."""
    times = np.asarray(times, float)
    m = np.zeros(len(times), bool)
    for s, e in segs:
        m |= (times >= s) & (times <= e)
    return m


def longest_clean(times, segs, t_from):
    """Longest stretch after t_from without artefacts, as a boolean selector."""
    times = np.asarray(times, float)
    good = (times >= t_from) & ~mask(times, segs)
    best, cur, best_end = 0, 0, -1
    for i, g in enumerate(good):
        cur = cur + 1 if g else 0
        if cur > best:
            best, best_end = cur, i
    sel = np.zeros(len(times), bool)
    if best:
        sel[best_end - best + 1:best_end + 1] = True
    return sel
