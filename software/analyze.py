"""
IDometer post-session analysis.

    python analyze.py sessions\\2026-10-28_17-40-00
    python analyze.py sessions\\...  --truth sim_truth.csv     (simulator check)

What it does:
  1. Re-aligns device and computer clocks over the whole session, more precisely.
  2. Detects heart beats in the raw PPG and builds instantaneous heart rate.
  3. Cuts a −3…+12 s window around each marker and measures:
       * SCR, the phasic skin response: "trough → peak" amplitude and latency;
       * heart rate: dip (0.5–3 s) and rise (3–8 s) relative to 3 s before the marker.
  4. Plots:
       overview.png   — the whole session with markers;
       responses.png  — responses overlaid by marker type (thin lines = single
                        events, thick = mean).
  5. Writes responses.csv, a table with one row per marker.

With --truth it compares the findings with what the simulator planted and prints
clock-alignment accuracy, sensitivity and amplitude error.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

import breathing as BR
import protocol as P
from i18n import tr
from clocksync import fit_global
from receiver_core import clean_ibi, detect_beats

SCR_MIN_US = 0.05          # "response present" threshold, µS (typically 0.01–0.05)
EDA_WIN = (-3.0, 12.0)
HR_WIN = (-5.0, 12.0)


def load_session(d):
    with open(os.path.join(d, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    fast = np.loadtxt(os.path.join(d, "fast.csv"), delimiter=",", skiprows=1, ndmin=2)
    slow = np.loadtxt(os.path.join(d, "slow.csv"), delimiter=",", skiprows=1, ndmin=2)
    pk = np.loadtxt(os.path.join(d, "packets.csv"), delimiter=",", skiprows=1, ndmin=2)
    markers = []
    with open(os.path.join(d, "markers.csv"), encoding="utf-8") as f:
        for r in csv.DictReader(f):
            r["t"] = float(r["t"])
            markers.append(r)
    return meta, fast, slow, pk, markers


def scr_measure(t_rel, eda, thr=SCR_MIN_US):
    """Trough in 0.5–4 s, then a peak no later than 4 s after it. Returns (amplitude, latency)."""
    a = (t_rel >= 0.5) & (t_rel <= 4.0)
    if a.sum() < 3:
        return 0.0, np.nan
    ia = np.where(a)[0]
    i0 = ia[np.argmin(eda[ia])]
    # an SCR rises within 0.5–4 s; the limit rejects slow drift of the tonic level
    b = (t_rel > t_rel[i0]) & (t_rel <= min(t_rel[i0] + 4.0, 8.0))
    if b.sum() < 2:
        return 0.0, np.nan
    ib = np.where(b)[0]
    i1 = ib[np.argmax(eda[ib])]
    amp = eda[i1] - eda[i0]
    if amp < thr:
        return amp, np.nan
    # response onset: the last moment before the peak where the rise is still < 10 % of the amplitude
    seg = eda[i0:i1 + 1] - eda[i0]
    k = np.where(seg < 0.1 * amp)[0][-1]
    return amp, t_rel[i0 + k]


def lowpass_eda(x, fs=P.SLOW_RATE_HZ, fc=1.0):
    """1 Hz low-pass: removes ADC noise but keeps the SCR shape (rise ~1 s)."""
    from scipy.signal import butter, sosfiltfilt
    ok = np.isfinite(x)
    y = np.interp(np.arange(len(x)), np.where(ok)[0], x[ok])
    return sosfiltfilt(butter(2, fc, fs=fs, output="sos"), y)


def main():
    ap = argparse.ArgumentParser(description="IDometer session analysis")
    ap.add_argument("session")
    ap.add_argument("--truth", help="simulator sim_truth.csv for verification")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    from receiver_core import load_config
    load_config()   # sets the interface language from config.json

    meta, fast, slow, pk, markers = load_session(args.session)
    out = os.path.join(args.session, "analysis")
    os.makedirs(out, exist_ok=True)

    # 1. clocks
    to_host = fit_global(pk[:, 1], pk[:, 2])
    if np.any(np.diff(fast[:, 1]) < 0):
        print(tr("an_clock_back"))
    tf = to_host(fast[:, 1])
    ts = to_host(slow[:, 1])
    order = np.argsort(tf)
    tf, fast = tf[order], fast[order]
    order = np.argsort(ts)
    ts, slow = ts[order], slow[order]
    eda_raw = slow[:, 4]
    eda = lowpass_eda(eda_raw)
    from receiver_core import eda_is_uS, eda_units
    units = eda_units(meta["config"].get("eda_calibration", {}))
    # "response present" threshold: at least 5x the sensor noise (and at least 0.03 µS)
    resid = eda_raw - eda
    noise = 1.4826 * np.median(np.abs(resid - np.median(resid)))
    thr = max(5 * noise, 0.03 if eda_is_uS(meta["config"].get("eda_calibration", {})) else 0.0)
    resp = slow[:, 5] / 1000.0
    exp_pk = int(pk[-1, 0] - pk[0, 0] + 1)
    print(tr("an_session", path=args.session))
    print(tr("an_summary", dur=ts[-1] - ts[0], got=len(pk), exp=exp_pk, lost=exp_pk - len(pk),
             drift=getattr(to_host, "drift_ppm", 0)))

    # 2. heart rate
    sign = -1 if meta["config"].get("invert_ppg", True) else 1
    beats = detect_beats(tf, sign * fast[:, 3].astype(float))
    ibi = np.diff(beats)
    okb = clean_ibi(ibi)
    hr_t = beats[1:][okb]
    hr_v = 60.0 / ibi[okb]
    hr_plot = np.where(okb, 60.0 / np.where(ibi > 0, ibi, np.nan), np.nan)   # gaps at rejected beats
    print(tr("an_noise", noise=noise, thr=thr, units=units))
    print(tr("an_beats", n=len(beats), hr=np.median(hr_v)))
    n_rej = int((~okb).sum())
    print(tr("an_beats_rejected", n=n_rej, pct=100.0 * n_rej / max(len(okb), 1)))
    if n_rej > 0.1 * len(okb):
        print(tr("an_beats_poor"))

    # breathing: the device's sensor channel, or estimated from the PPG when there is none
    tb, mods = BR.modulations(tf, fast[:, 3].astype(float), beats)
    yb = mods.get("riiv", np.array([]))
    has_sensor = bool(np.nanmax(np.abs(resp)) > 0)
    src = meta["config"].get("breathing_source", "auto")
    use_ppg = src == "ppg" or (src == "auto" and not has_sensor)
    if use_ppg and len(tb):
        resp_t, resp_y, resp_label = tb, yb, tr("an_y_resp_ppg")
    else:
        resp_t, resp_y, resp_label = ts, resp, tr("an_y_resp")
    if len(tb):
        # median of 60 s windows, keeping only windows with a clear rhythm
        rates = []
        for i in range(0, len(yb) - 600 + 1, 300):
            r_, q_ = BR.rate_fused({k: v[i:i + 600] for k, v in mods.items()})
            if q_ >= 0.6:
                rates.append(r_)
        if rates:
            print(tr("an_breath_ppg", br=np.median(rates), n=len(rates)))
        if has_sensor:
            from scipy.signal import sosfiltfilt
            ref = np.interp(tb, ts, resp)
            ref = sosfiltfilt(BR._SOS, ref - ref.mean())
            k = slice(100, -100)
            print(tr("an_breath_check", r=np.corrcoef(yb[k], ref[k])[0, 1],
                     a=np.median(rates) if rates else float("nan"), b=BR.rate_count(ref)[0]))

    def hr_at(t):
        return np.interp(t, hr_t, hr_v, left=np.nan, right=np.nan)

    # 3. windows around markers
    eg = np.arange(EDA_WIN[0], EDA_WIN[1] + 1e-9, 1 / P.SLOW_RATE_HZ)
    hg = np.arange(HR_WIN[0], HR_WIN[1] + 1e-9, 0.25)
    rows, per_label = [], {}
    def measure(t0):
        e = np.interp(t0 + eg, ts, eda)
        e_rel = e - np.mean(e[(eg >= -2) & (eg < 0)])
        amp, lat = scr_measure(eg, e_rel, thr)
        h = hr_at(t0 + hg)
        with np.errstate(all="ignore"):
            hb = np.nanmean(h[(hg >= -3) & (hg < 0)])
            h_rel = h - hb
            dec = np.nanmean(h_rel[(hg >= 0.5) & (hg <= 3)])
            acc = np.nanmean(h_rel[(hg >= 3) & (hg <= 8)])
        return e_rel, h_rel, amp, lat, hb, dec, acc

    def inside(t0):
        return t0 + EDA_WIN[0] >= ts[0] and t0 + EDA_WIN[1] <= ts[-1]

    for m in markers:
        t0 = m["t"]
        if not inside(t0):
            continue
        e_rel, h_rel, amp, lat, hb, dec, acc = measure(t0)
        rows.append({"t": round(t0, 3), "label": m["label"], "note": m.get("note", ""),
                     "scr_amp": round(float(amp), 4),
                     "scr_latency_s": "" if np.isnan(lat) else round(float(lat), 2),
                     "scr_detected": int(not np.isnan(lat)),
                     "hr_baseline": round(float(hb), 1),
                     "hr_change_0.5-3s": round(float(dec), 1),
                     "hr_change_3-8s": round(float(acc), 1)})
        per_label.setdefault(m["label"], []).append((e_rel, h_rel))

    # Chance level: the same measurement at moments far from all markers.
    # Shows how often a "response" appears by itself (spontaneous SCRs, noise).
    mt_all = np.array([m["t"] for m in markers]) if markers else np.array([])
    cand = np.arange(ts[0] + 5, ts[-1] - 13, 0.5)
    if len(mt_all):
        cand = cand[np.min(np.abs(cand[:, None] - mt_all[None, :]), axis=1) > 10]
    rng = np.random.default_rng(0)
    cand = rng.choice(cand, size=min(200, len(cand)), replace=False) if len(cand) else cand
    chance = [measure(t)[3] for t in cand]
    chance_rate = (np.mean([not np.isnan(x) for x in chance]) if chance else np.nan)

    with open(os.path.join(out, "responses.csv"), "w", newline="", encoding="utf-8-sig") as f:
        if rows:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    print(tr("an_table_head", units=units))
    for lab, items in per_label.items():
        rs = [r for r in rows if r["label"] == lab]
        det = sum(r["scr_detected"] for r in rs)
        amps = [r["scr_amp"] for r in rs if r["scr_detected"]]
        dec = np.nanmean([r["hr_change_0.5-3s"] for r in rs])
        acc = np.nanmean([r["hr_change_3-8s"] for r in rs])
        print(f"  {lab:20s} {len(rs):3d}   {det:3d}/{len(rs):<3d}    "
              f"{(np.mean(amps) if amps else 0):10.3f}        {dec:+5.1f} / {acc:+5.1f}")

    if len(cand):
        print(tr("an_chance", n=len(cand), pct=100 * chance_rate))
    else:
        print(tr("an_chance_none"))

    # 4. check against the simulator
    if args.truth:
        check_truth(args.truth, to_host, markers, rows, beats)

    # 5. plots
    if not args.no_plots:
        plot_all(out, ts, eda, resp_t, resp_y, resp_label, beats[1:], hr_plot, markers, per_label, eg, hg, units)
        print(tr("an_outputs", path=out))


def check_truth(path, to_host, markers, rows, beats):
    truth = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["kind"] in ("response", "no_response"):
                truth.append(r)
    st = np.array([to_host(int(r["stim_device_us"])) for r in truth])
    mt = np.array([m["t"] for m in markers])
    errs, tp, fn, fp, tn, amp_err, lat_err = [], 0, 0, 0, 0, [], []
    by_t = {r["t"]: r for r in rows}
    for r, s in zip(truth, st):
        if not len(mt):
            break
        j = int(np.argmin(np.abs(mt - s)))
        if abs(mt[j] - s) > 0.5:
            continue
        errs.append(1000 * (s - mt[j]))
        row = by_t.get(round(mt[j], 3))
        if row is None:
            continue
        resp_true = r["kind"] == "response"
        det = bool(row["scr_detected"])
        tp += resp_true and det
        fn += resp_true and not det
        fp += (not resp_true) and det
        tn += (not resp_true) and not det
        if resp_true and det:
            amp_err.append(row["scr_amp"] - float(r["scr_amp_uS"]))
            onset = to_host(int(r["onset_device_us"])) - mt[j]
            lat_err.append(row["scr_latency_s"] - onset)
    errs = np.array(errs)
    print(tr("an_truth_head"))
    if len(errs):
        print(tr("an_truth_sync", mean=errs.mean(), max=np.abs(errs).max(), n=len(errs)))
    print(tr("an_truth_found", tp=tp, all=tp + fn, fp=fp, none=fp + tn))
    if amp_err:
        print(tr("an_truth_amp", mean=np.mean(amp_err), sd=np.std(amp_err)))
        print(tr("an_truth_lat", mean=np.mean(lat_err)))
    bpath = path.replace(".csv", "_beats.csv")
    if os.path.exists(bpath) and len(beats):
        tb = to_host(np.loadtxt(bpath, delimiter=",", skiprows=1, ndmin=1))
        tb = tb[(tb > beats[0] - 1) & (tb < beats[-1])]
        d = np.array([beats[np.argmin(np.abs(beats - b - 0.18))] - b for b in tb])
        good = np.abs(d - 0.18) < 0.15
        ibi_true = np.diff(tb)
        print(tr("an_truth_beats", planted=len(tb), found=len(beats), sd=1000 * np.std(d[good])))


def plot_all(out, ts, eda, resp_t, resp_y, resp_label, hr_t, hr_v, markers, per_label, eg, hg, units):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    labels = list(dict.fromkeys(m["label"] for m in markers))
    lc = {lab: colors[i % len(colors)] for i, lab in enumerate(labels)}
    t0 = ts[0]

    fig, ax = plt.subplots(3, 1, figsize=(16, 8), sharex=True,
                           gridspec_kw={"height_ratios": [3, 2, 1.5]})
    ax[0].plot((ts - t0) / 60, eda, lw=1, color="#1f77b4")
    ax[0].set_ylabel(tr("an_y_eda", units=units))
    ax[1].plot((hr_t - t0) / 60, hr_v, lw=0.8, color="#d62728")
    ax[1].set_ylabel(tr("an_y_hr"))
    ax[2].plot((resp_t - t0) / 60, resp_y, lw=0.6, color="#2ca02c")
    ax[2].set_ylabel(resp_label)
    ax[2].set_xlabel(tr("an_x_min"))
    seen = set()
    for m in markers:
        x = (m["t"] - t0) / 60
        for a in ax:
            a.axvline(x, color=lc[m["label"]], lw=0.8, alpha=0.7, ls="--")
        if m["label"] not in seen:
            ax[0].plot([], [], color=lc[m["label"]], ls="--", label=m["label"])
            seen.add(m["label"])
    if seen:
        ax[0].legend(loc="upper left", fontsize=8, ncol=4)
    for a in ax:
        a.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "overview.png"), dpi=110)
    plt.close(fig)

    if not per_label:
        return
    n = len(per_label)
    fig, ax = plt.subplots(n, 2, figsize=(12, 2.6 * n), squeeze=False)
    for i, (lab, items) in enumerate(per_label.items()):
        E = np.array([x[0] for x in items])
        H = np.array([x[1] for x in items])
        for e in E:
            ax[i, 0].plot(eg, e, color=lc[lab], lw=0.6, alpha=0.35)
        ax[i, 0].plot(eg, E.mean(0), color=lc[lab], lw=2.5)
        for h in H:
            ax[i, 1].plot(hg, h, color=lc[lab], lw=0.6, alpha=0.35)
        with np.errstate(all="ignore"):
            ax[i, 1].plot(hg, np.nanmean(H, 0), color=lc[lab], lw=2.5)
        ax[i, 0].set_ylabel(tr("an_y_delta_eda", label=lab, n=len(items), units=units))
        ax[i, 1].set_ylabel(tr("an_y_delta_hr"))
        for a in ax[i]:
            a.axvline(0, color="k", lw=1)
            a.axhline(0, color="#999", lw=0.5)
            a.axvspan(1, 4, color="#ffd54f", alpha=0.15)
            a.grid(alpha=0.2)
    ax[0, 0].set_title(tr("an_title_eda"))
    ax[0, 1].set_title(tr("an_title_hr"))
    ax[-1, 0].set_xlabel(tr("an_x_from_marker"))
    ax[-1, 1].set_xlabel(tr("an_x_from_marker"))
    fig.tight_layout()
    fig.savefig(os.path.join(out, "responses.png"), dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
