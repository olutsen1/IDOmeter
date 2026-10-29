"""
Breathing estimated from the PPG signal, for when there is no breathing sensor.

Breathing modulates the finger pulse wave in three ways: the baseline drifts
with each breath (respiratory-induced intensity variation, RIIV), the pulse
amplitude changes, and the heart rate speeds up on inhalation (respiratory
sinus arrhythmia). The baseline drift is usually the strongest in a finger
PPG, so it is used here. The pulse itself is much larger than this drift, so
it is removed first: a moving average exactly one beat long (applied twice)
cancels the heartbeat and its harmonics at any heart rate. The result is then
averaged down to 10 Hz and band-passed to 0.13–0.6 Hz (8–36 breaths per minute).

The result shows the breathing rhythm, pauses and deep breaths. It is not a
volume measurement, and its direction (whether inhalation goes up or down)
depends on the person and sensor placement.

The breathing RATE is counted breath by breath in three modulations — the
baseline above, the heart rate (respiratory sinus arrhythmia) and the pulse
amplitude — and fused (rate_fused). The heart-rate and amplitude modulations
are not disturbed by slow baseline drift, which made a single-signal estimate
stick near the lower band edge. Synthetic check (heart rate 50–70, breathing
8–24 /min, 60 s windows): mean error ~1 breath/min. Breathing slower than
~8 /min lies at the band edge and is overestimated.
"""
import numpy as np
from scipy.signal import butter, sosfiltfilt

from receiver_core import clean_ibi

HR_BAND = (0.6, 3.0)        # Hz: 36–180 bpm, used to find the beat length

OUT_FS = 10                 # Hz, output sample rate
BAND = (0.13, 0.6)          # Hz: 8–36 breaths per minute; the low edge stays above
                            # ~0.1 Hz vasomotor (Mayer) waves, which also move the PPG baseline
_SOS = butter(3, BAND, btype="band", fs=OUT_FS, output="sos")
MIN_SECONDS = 15            # shorter stretches are too short for the low-cut filter


def beat_length(x, fs):
    """Typical beat length in seconds, from the spectral peak in HR_BAND."""
    n = 1 << int(np.ceil(np.log2(max(len(x), 16 * fs))))
    spec = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x)), n))
    f = np.fft.rfftfreq(n, 1 / fs)
    band = (f >= HR_BAND[0]) & (f <= HR_BAND[1])
    return 1.0 / f[band][np.argmax(spec[band])]


def _cancel_heartbeat(x, fs, beat_s):
    """Moving average one beat long, twice: zeros at the heart rate and its harmonics."""
    n = max(3, int(round(beat_s * fs)))
    k = np.ones(n) / n
    pad = np.pad(x, (n, n), mode="edge")
    y = np.convolve(np.convolve(pad, k, mode="same"), k, mode="same")
    return y[n:-n]


def from_ppg(t, ir, fs=100, beat_s=None):
    """Return (t_out, breathing) at OUT_FS, normalised to unit robust spread.

    t, ir: PPG time stamps (s) and raw IR samples (not inverted). Gaps (NaN)
    are bridged by interpolation. beat_s: beat length in seconds if known
    (otherwise estimated). Returns empty arrays if there is too little data.
    """
    t = np.asarray(t, float)
    x = np.asarray(ir, float)
    ok = np.isfinite(t) & np.isfinite(x)
    if ok.sum() < MIN_SECONDS * fs:
        return np.array([]), np.array([])
    t, x = t[ok], x[ok]
    if beat_s is None or not np.isfinite(beat_s):
        beat_s = beat_length(x, fs)
    x = _cancel_heartbeat(x, fs, beat_s)
    k = fs // OUT_FS
    n = len(x) // k * k
    xd = x[:n].reshape(-1, k).mean(1)
    td = t[:n].reshape(-1, k).mean(1)
    y = sosfiltfilt(_SOS, xd - xd.mean())
    spread = 1.4826 * np.median(np.abs(y - np.median(y)))
    if spread > 0:
        y = y / spread
    return td, y


def rate_count(y, fs=OUT_FS):
    """Breathing rate by counting breaths in the curve, and a 0–1 regularity score.

    "Advanced counting" (Schäfer & Kratky, 2008): find the peaks and troughs,
    drop pairs whose height difference is below 30 % of the upper quartile of
    all differences (small wiggles, not breaths), repeat until stable; a breath
    is the time between two remaining peaks. The rate is 60 / median breath.
    Regularity is the share of breaths within ±30 % of the median.
    """
    y = np.asarray(y, float)
    if len(y) < MIN_SECONDS * fs:
        return float("nan"), 0.0
    d = np.diff(y)
    ext = np.where(np.sign(d[1:]) != np.sign(d[:-1]))[0] + 1      # local extrema
    if len(ext) < 4:
        return float("nan"), 0.0
    ext = list(ext)
    while len(ext) >= 4:
        h = np.abs(np.diff(y[ext]))
        thr = 0.3 * np.percentile(h, 75)
        i = int(np.argmin(h))
        if h[i] >= thr:
            break
        del ext[i:i + 2]                       # remove the smallest swing (one peak + one trough)
    ext = np.array(ext)
    if len(ext) < 4:
        return float("nan"), 0.0
    d2 = np.diff(y)
    peaks = [e for e in ext if 0 < e < len(y) - 1 and d2[e - 1] > 0 >= d2[e]]
    breaths = np.diff(peaks) / fs
    if len(breaths) < 2:
        return float("nan"), 0.0
    med = float(np.median(breaths))
    regular = float(np.mean(np.abs(breaths - med) <= 0.3 * med))
    return 60.0 / med, regular


def _beat_series(times, values, t_out):
    """A per-beat value resampled to t_out and band-passed to the breathing band."""
    times = np.asarray(times, float)
    values = np.asarray(values, float)
    ok = np.isfinite(times) & np.isfinite(values)
    if ok.sum() < 8:
        return None
    y = np.interp(t_out, times[ok], values[ok])
    return sosfiltfilt(_SOS, y - y.mean())


def modulations(t, ir, beats, fs=100):
    """The three breathing modulations of the PPG, on a common 10 Hz time grid.

    riiv: baseline (intensity) variation, from from_ppg();
    riav: pulse amplitude of each beat;
    rifv: heart rate of each beat (respiratory sinus arrhythmia).
    beats: systolic peak times (s), i.e. minima of the raw IR signal.
    Returns (t_out, {name: y}) with only the series that could be computed.
    """
    t_out, riiv = from_ppg(t, ir, fs)
    out = {}
    if not len(t_out):
        return t_out, out
    out["riiv"] = riiv
    t = np.asarray(t, float)
    x = np.asarray(ir, float)
    b = np.asarray(beats, float)
    b = b[(b > t[0] + 0.5) & (b < t[-1])]
    if len(b) < 8:
        return t_out, out
    ibi = np.diff(b)
    ok = clean_ibi(ibi)                      # misdetected beats would fake a "breath"
    rifv = _beat_series(b[1:][ok], 60.0 / ibi[ok], t_out)
    if rifv is not None:
        out["rifv"] = rifv
    amp = []
    for tb in b:
        seg = x[(t >= tb - 0.45) & (t <= tb + 0.05)]
        amp.append(seg.max() - seg.min() if len(seg) > 5 else np.nan)
    riav = _beat_series(b, amp, t_out)
    if riav is not None:
        out["riav"] = riav
    return t_out, out


def rate_fused(mods, fs=OUT_FS):
    """Breathing rate from the modulations ("smart fusion", Karlen et al., 2013).

    mods: {name: series} from modulations(). Heart-rate (rifv) and amplitude
    (riav) modulations are not affected by baseline drift and are used first:
    if both agree within 4 /min, their mean is returned with quality 1.0.
    Otherwise the baseline (riiv) joins as a tie-breaker and the median of
    the three is returned, with quality = share of estimates within 4 /min.
    """
    r = {k: rate_count(y, fs)[0] for k, y in mods.items()}
    r = {k: v for k, v in r.items() if np.isfinite(v)}
    if not r:
        return float("nan"), 0.0
    main = [r[k] for k in ("rifv", "riav") if k in r]
    if len(main) == 2 and abs(main[0] - main[1]) <= 4.0:
        return float(np.mean(main)), 1.0
    vals = np.array(list(r.values()))
    med = float(np.median(vals))
    return med, float(np.mean(np.abs(vals - med) <= 4.0)) if len(vals) > 1 else 0.5
