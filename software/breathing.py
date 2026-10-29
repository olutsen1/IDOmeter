"""
Breathing estimated from the PPG signal, for when there is no breathing sensor.

Breathing modulates the finger pulse wave in three ways: the baseline drifts
with each breath (respiratory-induced intensity variation, RIIV), the pulse
amplitude changes, and the heart rate speeds up on inhalation (respiratory
sinus arrhythmia). The baseline drift is usually the strongest in a finger
PPG, so it is used here: the raw IR signal is averaged down to 10 Hz and
band-passed to 0.13–0.6 Hz (8–36 breaths per minute).

The result shows the breathing rhythm, pauses and deep breaths. It is not a
volume measurement, and its direction (whether inhalation goes up or down)
depends on the person and sensor placement.
"""
import numpy as np
from scipy.signal import butter, sosfiltfilt, welch

OUT_FS = 10                 # Hz, output sample rate
BAND = (0.13, 0.6)          # Hz: 8–36 breaths per minute; the low edge stays above
                            # ~0.1 Hz vasomotor (Mayer) waves, which also move the PPG baseline
_SOS = butter(2, BAND, btype="band", fs=OUT_FS, output="sos")
MIN_SECONDS = 15            # shorter stretches are too short for the low-cut filter


def from_ppg(t, ir, fs=100):
    """Return (t_out, breathing) at OUT_FS, normalised to unit robust spread.

    t, ir: PPG time stamps (s) and raw IR samples (not inverted). Gaps (NaN)
    are bridged by interpolation. Returns empty arrays if there is too little data.
    """
    t = np.asarray(t, float)
    x = np.asarray(ir, float)
    ok = np.isfinite(t) & np.isfinite(x)
    if ok.sum() < MIN_SECONDS * fs:
        return np.array([]), np.array([])
    t, x = t[ok], x[ok]
    k = fs // OUT_FS
    n = len(x) // k * k
    xd = x[:n].reshape(-1, k).mean(1)
    td = t[:n].reshape(-1, k).mean(1)
    y = sosfiltfilt(_SOS, xd - xd.mean())
    spread = 1.4826 * np.median(np.abs(y - np.median(y)))
    if spread > 0:
        y = y / spread
    return td, y


def rate(y, fs=OUT_FS):
    """Dominant breathing rate in breaths/min and a 0–1 quality score.

    Quality is the share of in-band power within ±0.05 Hz of the peak; below
    about 0.3 the rhythm is not clear enough to trust the number.
    """
    y = np.asarray(y, float)
    if len(y) < MIN_SECONDS * fs:
        return float("nan"), 0.0
    f, p = welch(y, fs=fs, nperseg=min(len(y), 32 * fs))
    band = (f >= BAND[0]) & (f <= BAND[1])
    if not band.any() or p[band].sum() <= 0:
        return float("nan"), 0.0
    fb, pb = f[band], p[band]
    i = int(np.argmax(pb))
    near = np.abs(fb - fb[i]) <= 0.05
    return 60.0 * fb[i], float(pb[near].sum() / pb.sum())
