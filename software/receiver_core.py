"""
Receiver core: packet reception, clock alignment, buffers, markers, session
recording, heart-beat detection. No GUI: used by receiver.py (the window)
and by --headless mode (testing without a screen).
"""
import csv
import datetime as dt
import json
import os
import queue
import socket
import threading
import time

import numpy as np
from scipy.signal import butter, find_peaks, medfilt, sosfiltfilt

import i18n
import protocol as P
from i18n import tr
from clocksync import ClockSync

DEFAULT_CONFIG = {
    "listen_port": P.DATA_PORT,
    "sessions_dir": "sessions",
    "window_s": 30,
    "language": "en",
    # "markers": {"1": "Question", ...}  — leave out to use the language's defaults
    # Raw EDA conversion.
    #  relative: 0…100 rel. units of the ADC full scale; invert=true for the Grove
    #            GSR, whose output voltage falls as skin conductance rises
    #  linear:   µS = raw * gain + offset   (the simulator sends nS -> gain 0.001)
    #  table:    µS piecewise-linear over calibration points [[raw, µS], ...]
    #  none:     raw counts
    # With --sim, linear 0.001 is always used.
    "eda_calibration": {"type": "relative", "full_scale": 65520, "invert": True},
    "invert_ppg": True,
    # Breathing curve: "sensor" = the device's breathing channel, "ppg" = estimated
    # from the pulse wave (breathing.py), "auto" = sensor if it sends data, else ppg.
    "breathing_source": "auto",
}


def load_config(path="config.json"):
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            user = json.load(f)
        cfg.update(user)
    i18n.set_language(cfg.get("language", "en"))
    if not cfg.get("markers"):
        cfg["markers"] = dict(tr("markers"))
    return cfg


def eda_to_uS(raw, cal):
    raw = np.asarray(raw, float)
    t = cal.get("type", "none")
    if t == "linear":
        return raw * cal.get("gain", 1.0) + cal.get("offset", 0.0)
    if t == "table":
        pts = np.array(sorted(cal["points"]), float)
        return np.interp(raw, pts[:, 0], pts[:, 1])
    if t == "relative":
        v = 100.0 * raw / cal.get("full_scale", 65520)
        return 100.0 - v if cal.get("invert", True) else v
    return raw


def eda_is_uS(cal):
    return cal.get("type") in ("linear", "table")


def eda_units(cal):
    return tr("units_uS") if eda_is_uS(cal) else tr("units_rel")


# ---------------------------------------------------------------- buffers

class Ring:
    """Ring buffer of several synchronous columns."""

    def __init__(self, capacity, ncols):
        self.cap = capacity
        self.data = np.full((capacity, ncols), np.nan)
        self.n = 0          # total written

    def extend(self, rows):
        rows = np.atleast_2d(rows)
        for r in rows[-self.cap:]:
            self.data[self.n % self.cap] = r
            self.n += 1

    def tail(self):
        if self.n <= self.cap:
            return self.data[:self.n]
        i = self.n % self.cap
        return np.vstack((self.data[i:], self.data[:i]))


# ---------------------------------------------------------------- heart rate

_SOS = butter(2, [0.7, 3.5], btype="band", fs=P.BASE_RATE_HZ, output="sos")


def despike(x):
    """Remove 1–2-sample glitches from the PPG (5-point median), keeping NaN gaps.

    On the real device the raw PPG has 2-sample out-of-place values every few
    seconds (seen in both LED channels at once). A 50 ms median removes them
    and leaves the pulse shape and beat timing practically unchanged. Raw data
    are still recorded as received; this is applied before processing only.
    """
    x = np.asarray(x, float)
    ok = np.isfinite(x)
    if ok.sum() < 5:
        return x
    y = np.interp(np.arange(len(x)), np.where(ok)[0], x[ok])
    y = medfilt(y, 5)
    y[~ok] = np.nan
    return y


def detect_beats(t, ppg, fs=P.BASE_RATE_HZ):
    """Pulse-wave peak times. ppg must already have the right polarity (up = systole)."""
    ok = np.isfinite(ppg)
    if ok.sum() < 3 * fs:
        return np.array([])
    t, x = t[ok], ppg[ok]
    x = sosfiltfilt(_SOS, x - np.mean(x))
    prom = 0.4 * np.std(x)
    pk, _ = find_peaks(x, distance=int(0.33 * fs), prominence=prom)
    # refine the peak with a parabola
    out = []
    for p in pk:
        if 0 < p < len(x) - 1:
            y0, y1, y2 = x[p - 1], x[p], x[p + 1]
            den = y0 - 2 * y1 + y2
            d = 0.5 * (y0 - y2) / den if den != 0 else 0.0
            out.append(t[p] + d * (t[p + 1] - t[p]))
        else:
            out.append(t[p])
    return np.array(out)


# Beat-interval artefact rejection. A misdetected beat (finger moved, sensor
# pressed) produces an interval that jumps away from both of its neighbours,
# while physiological changes, even large ones (startle, slow deep breathing),
# change the interval gradually from one beat to the next. So each interval is
# compared with the mean of its two neighbours; the worst one is rejected if it
# differs by more than IBI_TOL, and the check repeats with the remaining beats
# until nothing exceeds the tolerance. Raw data are never changed: only the
# heart-rate series gets gaps.
IBI_RANGE = (0.33, 1.6)     # s: 37–180 bpm
IBI_TOL = 0.30
LIVE_WINDOW = 40            # intervals re-checked live; older decisions are final


def clean_ibi(ibi):
    """True for beat intervals kept, False for artefacts."""
    ibi = np.asarray(ibi, float)
    keep = (ibi >= IBI_RANGE[0]) & (ibi <= IBI_RANGE[1])
    while True:
        idx = np.where(keep)[0]
        if len(idx) < 3:
            break
        v = ibi[idx]
        prev = np.r_[np.nan, v[:-1]]
        nxt = np.r_[v[1:], np.nan]
        ref = np.nanmean(np.vstack((prev, nxt)), axis=0)
        dev = np.abs(v - ref) / ref
        j = int(np.nanargmax(dev))
        if dev[j] <= IBI_TOL:
            break
        keep[idx[j]] = False
    return keep


class LiveBeats:
    def __init__(self):
        self.beats = []        # beat times (app seconds)
        self.ibi = []          # (time of the closing beat, interval)
        self.final = []        # (t, bpm) decided for good; bpm NaN = rejected
        self.hr = []           # final + the recent window, re-checked on each update
        self.rejected = 0

    def _recheck(self):
        if len(self.ibi) > 2 * LIVE_WINDOW:            # freeze the oldest decisions
            old = self.ibi[:LIVE_WINDOW]
            keep = clean_ibi([v for _, v in self.ibi[:2 * LIVE_WINDOW]])[:LIVE_WINDOW]
            self.final += [(t, 60.0 / v if k else float("nan")) for (t, v), k in zip(old, keep)]
            self.rejected += int((~keep).sum())
            self.ibi = self.ibi[LIVE_WINDOW:]
        keep = clean_ibi([v for _, v in self.ibi])
        # the newest interval has no right-hand neighbour yet: shown, judged on the next beat
        self.hr = self.final + [(t, 60.0 / v if k else float("nan"))
                                for (t, v), k in zip(self.ibi, keep)]
        self.rejected_now = self.rejected + int((~keep).sum())

    def update(self, t, ppg, now_t):
        if len(t) < 300:
            return
        sel = t > now_t - 12
        bt = detect_beats(t[sel], ppg[sel])
        added = False
        for b in bt:
            if b > now_t - 1.5:           # the filter has not settled at the window edge
                continue
            if self.beats and b <= self.beats[-1] + 0.3:
                continue
            if self.beats:
                self.ibi.append((b, b - self.beats[-1]))
                added = True
            self.beats.append(b)
        if added:
            self._recheck()
        if len(self.beats) > 2000:
            self.beats = self.beats[-1000:]
            self.final = self.final[-1000:]


# ---------------------------------------------------------------- recording

class Recorder:
    def __init__(self, base_dir, cfg, app_t0_wall):
        stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.dir = os.path.join(base_dir, stamp)
        os.makedirs(self.dir, exist_ok=True)

        def w(name, header):
            f = open(os.path.join(self.dir, name), "w", newline="", encoding="utf-8")
            c = csv.writer(f)
            c.writerow(header)
            return f, c
        self.f_fast, self.fast = w("fast.csv", ["seq", "device_us", "t", "ppg_ir", "ppg_red"])
        self.f_slow, self.slow = w("slow.csv", ["seq", "device_us", "t", "eda_raw", "eda_uS", "resp"])
        self.f_pk, self.pk = w("packets.csv", ["seq", "t_end_device_us", "arrival_t", "flags"])
        self.f_mk, self.mk = w("markers.csv", ["t", "key", "label", "note"])
        self.files = [self.f_fast, self.f_slow, self.f_pk, self.f_mk]
        self.meta = {
            "format": "idometer-session-1",
            "started_wall": dt.datetime.now().astimezone().isoformat(),
            "app_t0_wall": app_t0_wall,
            "time_note": "t: seconds of the app clock (perf_counter since start); "
                         "device_us: device clock; packets.csv allows recomputing t",
            "base_rate_hz": P.BASE_RATE_HZ,
            "slow_rate_hz": P.SLOW_RATE_HZ,
            "config": cfg,
        }
        self._write_meta()
        self.last_flush = time.perf_counter()

    def _write_meta(self):
        with open(os.path.join(self.dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)

    def packet(self, pkt, arrival_t, t_fast, t_slow, eda_uS):
        self.pk.writerow([pkt.seq, pkt.t_end_us, f"{arrival_t:.6f}", pkt.flags])
        for us, t, a, b in zip(pkt.fast_times_us(), t_fast, pkt.ppg_ir, pkt.ppg_red):
            self.fast.writerow([pkt.seq, us, f"{t:.4f}", a, b])
        for us, t, e, u, r in zip(pkt.slow_times_us(), t_slow, pkt.eda, eda_uS, pkt.resp):
            self.slow.writerow([pkt.seq, us, f"{t:.4f}", e, f"{u:.4f}", r])
        if time.perf_counter() - self.last_flush > 1.0:
            self.flush()

    def marker(self, m):
        self.mk.writerow([f"{m['t']:.4f}", m["key"], m["label"], m.get("note", "")])
        self.f_mk.flush()

    def flush(self):
        for f in self.files:
            f.flush()
        self.last_flush = time.perf_counter()

    def close(self, stats):
        self.meta["stopped_wall"] = dt.datetime.now().astimezone().isoformat()
        self.meta["stats"] = stats
        self._write_meta()
        for f in self.files:
            f.close()


# ---------------------------------------------------------------- reception

class UdpListener(threading.Thread):
    def __init__(self, port, out_q, clock):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("0.0.0.0", port))
        self.q = out_q
        self.clock = clock
        self.peer = None
        self.hello = {}

    def run(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except OSError:
                return
            # timestamp on arrival, before any processing
            t = self.clock()
            self.q.put((t, data, addr))
            # reply "IDHI" so the device learns the computer's address
            # and switches from broadcast to sending only here
            last = self.hello.get(addr)
            if last is None or t - last > 2.0:
                self.hello[addr] = t
                try:
                    self.sock.sendto(b"IDHI", addr)
                except OSError:
                    pass


class Engine:
    """Everything except the window. Call poll() regularly (in the GUI, from a timer)."""

    def __init__(self, cfg, sim_feedback=None):
        self.cfg = cfg
        self.app_t0 = time.perf_counter()
        self.app_t0_wall = dt.datetime.now().astimezone().isoformat()
        self.q = queue.Queue()
        self.listener = UdpListener(cfg["listen_port"], self.q, self.now)
        self.listener.start()
        self.sync = ClockSync()
        cap_fast = P.BASE_RATE_HZ * 180
        cap_slow = P.SLOW_RATE_HZ * 180
        self.fast = Ring(cap_fast, 3)    # t, ppg_ir (for display), ppg_red
        self.slow = Ring(cap_slow, 3)    # t, eda_uS, resp
        self.beats = LiveBeats()
        self.markers = []
        self.recorder = None
        self.last_seq = None
        self.last_t_end = None
        self.packets = 0
        self.lost = 0
        self.bad = 0
        self.resets = 0
        self.last_rx = None
        self.last_flags = 0
        self.peer = None
        self._beat_tick = 0.0
        self.sim_feedback = sim_feedback   # (host, port) or None
        self._fb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if sim_feedback else None

    def now(self):
        return time.perf_counter() - self.app_t0

    # --- data
    def poll(self):
        n = 0
        while True:
            try:
                arrival, data, addr = self.q.get_nowait()
            except queue.Empty:
                break
            try:
                pkt = P.decode(data)
            except ValueError:
                self.bad += 1
                continue
            self.peer = addr
            self._handle(pkt, arrival)
            n += 1
        now = self.now()
        if now - self._beat_tick > 0.5:
            self._beat_tick = now
            tail = self.fast.tail()
            if len(tail):
                sel = tail[:, 0] > now - 15
                self.beats.update(tail[sel, 0], despike(tail[sel, 1]), now)
        return n

    def _handle(self, pkt, arrival):
        # device restart: its clock went backwards
        if self.last_t_end is not None and pkt.t0_us < self.last_t_end - 1_000_000:
            self.sync.reset()
            self.last_seq = None
            self.resets += 1
        if self.last_seq is not None:
            gap = (pkt.seq - self.last_seq) & 0xFFFFFFFF
            if gap == 0 or gap > 0x7FFFFFFF:      # duplicate or late packet
                return
            if gap > 1:
                self.lost += gap - 1
                # gap in the plot
                self.fast.extend([[np.nan] * 3])
                self.slow.extend([[np.nan] * 3])
        self.last_seq = pkt.seq
        self.last_t_end = pkt.t_end_us
        self.packets += 1
        self.last_rx = arrival
        self.last_flags = pkt.flags

        self.sync.add(pkt.t_end_us, arrival)
        tf = self.sync.to_host(pkt.fast_times_us())
        ts = self.sync.to_host(pkt.slow_times_us())
        eda = eda_to_uS(pkt.eda, self.cfg["eda_calibration"])
        sign = -1 if self.cfg.get("invert_ppg", True) else 1
        self.fast.extend(np.column_stack([tf, sign * pkt.ppg_ir, sign * pkt.ppg_red]))
        self.slow.extend(np.column_stack([ts, eda, pkt.resp / 1000.0]))
        if self.recorder:
            self.recorder.packet(pkt, arrival, tf, ts, eda)

    # --- markers
    def add_marker(self, key, label, note="", t=None):
        m = {"t": self.now() if t is None else t, "key": key, "label": label, "note": note}
        self.markers.append(m)
        if self.recorder:
            self.recorder.marker(m)
        if self._fb_sock:
            try:
                self._fb_sock.sendto(("MARK " + label).encode("utf-8"), self.sim_feedback)
            except OSError:
                pass
        return m

    # --- recording
    def start_recording(self):
        if self.recorder:
            return self.recorder.dir
        self.recorder = Recorder(self.cfg["sessions_dir"], self.cfg, self.app_t0_wall)
        self.recorder.meta["record_start_t"] = self.now()
        return self.recorder.dir

    def stop_recording(self):
        if not self.recorder:
            return None
        d = self.recorder.dir
        self.recorder.meta["record_stop_t"] = self.now()
        self.recorder.close(self.stats())
        self.recorder = None
        return d

    # --- status
    def stats(self):
        return {
            "packets": self.packets,
            "lost": self.lost,
            "bad": self.bad,
            "device_resets": self.resets,
            "drift_ppm": round(float(self.sync.drift_ppm), 1) if self.sync.ready else None,
            "beats_rejected": getattr(self.beats, "rejected_now", 0),
        }

    def connected(self):
        return self.last_rx is not None and self.now() - self.last_rx < 2.0
