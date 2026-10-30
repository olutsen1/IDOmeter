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

A gentle movement can distort the pulse without making it larger (extra small
peaks, a deformed upstroke). So, when beat times are given, each beat's SHAPE
is also checked: the beat waveform (−0.35…+0.45 s around the peak, amplitude-
normalised) is correlated with the median shape of the surrounding beats.
A beat whose correlation is below SHAPE_MIN is flagged. Normalising the
amplitude keeps vasoconstriction (smaller but same-shaped beats) unflagged.

A change of finger pressure can shift the whole baseline in a fraction of a
second without changing the beat shape; the breathing curve would show it as
a breath. So the level of each beat's foot (diastolic end) is compared with
the beat two places earlier; a change of more than STEP_MAX pulse amplitudes
is flagged. On a real recording such a step measured 1.35 pulse amplitudes
while quiet breathing stayed below 0.56.
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
SHAPE_MIN = 0.75            # correlation with the typical beat shape
SHAPE_WIN = (-0.35, 0.45)   # s around the systolic peak
SHAPE_REF = 15              # beats on each side forming the typical shape
STEP_MAX = 1.0              # baseline change over two beats, in pulse amplitudes
_FS_OUT = 10
_SOS = butter(2, (0.5, 5.0), btype="band", fs=100, output="sos")


def beat_shapes(t, x_bp, beats, fs=100):
    """Correlation of each beat's shape with the median shape of its neighbours."""
    n0, n1 = int(SHAPE_WIN[0] * fs), int(SHAPE_WIN[1] * fs)
    idx = np.searchsorted(t, beats)
    segs, keep = [], []
    for i in idx:
        if i + n0 < 0 or i + n1 > len(x_bp):
            segs.append(None)
            continue
        w = x_bp[i + n0:i + n1]
        sd = w.std()
        segs.append((w - w.mean()) / sd if sd > 0 else None)
    corr = np.full(len(beats), np.nan)
    good = [k for k, w in enumerate(segs) if w is not None]
    for k in good:
        nb = [segs[j] for j in good if j != k and abs(j - k) <= SHAPE_REF]
        if len(nb) < 5:
            continue
        tmpl = np.median(nb, axis=0)
        corr[k] = float(np.corrcoef(segs[k], tmpl)[0, 1])
    return corr


def detect(t, ir, fs=100, beats=None):
    """Return a list of (start, end) times of motion artefacts.

    t, ir: PPG time stamps (s) and raw IR samples. NaN gaps are bridged.
    beats: systolic peak times (s); if given, beat shapes are checked too.
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
    if beats is not None and len(beats):
        b = np.asarray(beats, float)
        b = b[(b > t[0]) & (b < t[-1])]
        corr = beat_shapes(t, y, b, fs)
        for tb in b[np.isfinite(corr) & (corr < SHAPE_MIN)]:
            bad |= (ta >= tb + SHAPE_WIN[0]) & (ta <= tb + SHAPE_WIN[1])
        foot, amp = [], []
        for tb in b:
            s = x[(t >= tb - 0.45) & (t <= tb + 0.05)]
            foot.append(s.max() if len(s) > 5 else np.nan)
            amp.append(s.max() - s.min() if len(s) > 5 else np.nan)
        foot = np.array(foot)
        pa = np.nanmedian(amp) if len(amp) else np.nan
        if len(b) >= 3 and np.isfinite(pa) and pa > 0:
            step = np.abs(foot[2:] - foot[:-2]) / pa
            for i in np.where(step > STEP_MAX)[0]:
                bad |= (ta >= b[i]) & (ta <= b[i + 2])
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
