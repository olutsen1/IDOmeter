"""
IDometer — анализ сессии после записи.

    python analyze.py sessions\\2026-10-28_17-40-00
    python analyze.py sessions\\...  --truth sim_truth.csv     (проверка на симуляторе)

Что делает:
  1. Заново и точнее сводит часы устройства и компьютера по всей сессии.
  2. Находит удары пульса по сырому PPG и строит мгновенный пульс.
  3. Для каждой метки вырезает окно −3…+12 с и измеряет:
       * SCR — фазический ответ кожи: амплитуда «впадина → пик», латентность;
       * пульс — замедление (0,5–3 с) и ускорение (3–8 с) относительно 3 с до метки.
  4. Рисует:
       overview.png   — вся сессия с метками;
       responses.png  — наложенные отклики по типам меток (тонкие линии — отдельные
                        события, толстая — среднее).
  5. Пишет responses.csv — таблица по каждой метке.

С --truth сравнивает найденное с тем, что заложил симулятор, и печатает
точность синхронизации часов, чувствительность и ошибку амплитуды.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

import protocol as P
from clocksync import fit_global
from receiver_core import detect_beats

SCR_MIN_US = 0.05          # порог «ответ есть», мкСм (типично 0,01–0,05)
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
    """Впадина в 0,5–4 с, затем пик не позже 4 с после неё. Возвращает (амплитуда, латентность)."""
    a = (t_rel >= 0.5) & (t_rel <= 4.0)
    if a.sum() < 3:
        return 0.0, np.nan
    ia = np.where(a)[0]
    i0 = ia[np.argmin(eda[ia])]
    # подъём SCR занимает 0,5–4 с; ограничение отсекает медленный дрейф тонического уровня
    b = (t_rel > t_rel[i0]) & (t_rel <= min(t_rel[i0] + 4.0, 8.0))
    if b.sum() < 2:
        return 0.0, np.nan
    ib = np.where(b)[0]
    i1 = ib[np.argmax(eda[ib])]
    amp = eda[i1] - eda[i0]
    if amp < thr:
        return amp, np.nan
    # начало ответа: последний момент перед пиком, когда подъём ещё < 10 % амплитуды
    seg = eda[i0:i1 + 1] - eda[i0]
    k = np.where(seg < 0.1 * amp)[0][-1]
    return amp, t_rel[i0 + k]


def lowpass_eda(x, fs=P.SLOW_RATE_HZ, fc=1.0):
    """Сглаживание 1 Гц: убирает шум АЦП, но не трогает форму SCR (подъём ~1 с)."""
    from scipy.signal import butter, sosfiltfilt
    ok = np.isfinite(x)
    y = np.interp(np.arange(len(x)), np.where(ok)[0], x[ok])
    return sosfiltfilt(butter(2, fc, fs=fs, output="sos"), y)


def main():
    ap = argparse.ArgumentParser(description="Анализ сессии IDometer")
    ap.add_argument("session")
    ap.add_argument("--truth", help="sim_truth.csv симулятора для проверки")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    meta, fast, slow, pk, markers = load_session(args.session)
    out = os.path.join(args.session, "analysis")
    os.makedirs(out, exist_ok=True)

    # 1. часы
    to_host = fit_global(pk[:, 1], pk[:, 2])
    if np.any(np.diff(fast[:, 1]) < 0):
        print("Внимание: часы устройства шли назад (перезапуск?) — пересчёт времени может быть неточным.")
    tf = to_host(fast[:, 1])
    ts = to_host(slow[:, 1])
    order = np.argsort(tf)
    tf, fast = tf[order], fast[order]
    order = np.argsort(ts)
    ts, slow = ts[order], slow[order]
    eda_raw = slow[:, 4]
    eda = lowpass_eda(eda_raw)
    from receiver_core import eda_units
    units = eda_units(meta["config"].get("eda_calibration", {}))
    # порог «ответ есть»: не меньше 5 шумов датчика (и не меньше 0,03 мкСм)
    resid = eda_raw - eda
    noise = 1.4826 * np.median(np.abs(resid - np.median(resid)))
    thr = max(5 * noise, 0.03 if units == "мкСм" else 0.0)
    resp = slow[:, 5] / 1000.0
    exp_pk = int(pk[-1, 0] - pk[0, 0] + 1)
    print(f"Сессия: {args.session}")
    print(f"  длительность {ts[-1] - ts[0]:.0f} с, пакетов {len(pk)} из {exp_pk} "
          f"(потеряно {exp_pk - len(pk)}), дрейф часов устройства "
          f"{getattr(to_host, 'drift_ppm', 0):+.1f} ppm")

    # 2. пульс
    sign = -1 if meta["config"].get("invert_ppg", True) else 1
    beats = detect_beats(tf, sign * fast[:, 3].astype(float))
    ibi = np.diff(beats)
    okb = (ibi > 0.33) & (ibi < 1.6)
    hr_t = beats[1:][okb]
    hr_v = 60.0 / ibi[okb]
    print(f"  шум EDA {noise:.4f} {units}, порог ответа {thr:.3f} {units}")
    print(f"  ударов найдено {len(beats)}, средний пульс {np.median(hr_v):.0f} уд/мин")

    def hr_at(t):
        return np.interp(t, hr_t, hr_v, left=np.nan, right=np.nan)

    # 3. окна вокруг меток
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

    # Случайный уровень: те же измерения в моментах, далёких от всех меток.
    # Показывает, как часто «ответ» находится просто так (спонтанные SCR, шум).
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

    print(f"\n  Тип метки             n   SCR есть   ср.ампл., {units}   пульс 0,5–3 с / 3–8 с")
    for lab, items in per_label.items():
        rs = [r for r in rows if r["label"] == lab]
        det = sum(r["scr_detected"] for r in rs)
        amps = [r["scr_amp"] for r in rs if r["scr_detected"]]
        dec = np.nanmean([r["hr_change_0.5-3s"] for r in rs])
        acc = np.nanmean([r["hr_change_3-8s"] for r in rs])
        print(f"  {lab:20s} {len(rs):3d}   {det:3d}/{len(rs):<3d}    "
              f"{(np.mean(amps) if amps else 0):10.3f}        {dec:+5.1f} / {acc:+5.1f}")

    if len(cand):
        print(f"  Случайный уровень: в {len(cand)} окнах без меток «ответ» найден в "
              f"{100 * chance_rate:.0f}% — с этим и сравнивайте долю ответов на метки.")
    else:
        print("  Случайный уровень: нет достаточно длинных промежутков без меток.")

    # 4. сверка с симулятором
    if args.truth:
        check_truth(args.truth, to_host, markers, rows, beats)

    # 5. графики
    if not args.no_plots:
        plot_all(out, ts, eda, resp, hr_t, hr_v, markers, per_label, eg, hg, units)
        print(f"\nГрафики и таблица: {out}")


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
    print("\n=== Сверка с симулятором ===")
    if len(errs):
        print(f"  Синхронизация: момент метки vs момент, когда её получило «устройство»: "
              f"среднее {errs.mean():+.1f} мс, макс. |ошибка| {np.abs(errs).max():.1f} мс "
              f"(n={len(errs)})")
    print(f"  Заложенный ответ найден: {tp} из {tp + fn};  "
          f"ложных ответов при отсутствии заложенного: {fp} из {fp + tn}")
    if amp_err:
        print(f"  Амплитуда SCR: средняя ошибка {np.mean(amp_err):+.3f} мкСм, "
              f"разброс {np.std(amp_err):.3f} мкСм")
        print(f"  Латентность начала ответа: средняя ошибка {np.mean(lat_err):+.2f} с")
    bpath = path.replace(".csv", "_beats.csv")
    if os.path.exists(bpath) and len(beats):
        tb = to_host(np.loadtxt(bpath, delimiter=",", skiprows=1, ndmin=1))
        tb = tb[(tb > beats[0] - 1) & (tb < beats[-1])]
        d = np.array([beats[np.argmin(np.abs(beats - b - 0.18))] - b for b in tb])
        good = np.abs(d - 0.18) < 0.15
        ibi_true = np.diff(tb)
        print(f"  Удары: заложено {len(tb)}, найдено {len(beats)}; "
              f"разброс момента удара {1000 * np.std(d[good]):.1f} мс "
              f"(пик волны отстаёт от удара на ~180 мс по построению)")


def plot_all(out, ts, eda, resp, hr_t, hr_v, markers, per_label, eg, hg, units):
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
    ax[0].set_ylabel(f"EDA, {units}")
    ax[1].plot((hr_t - t0) / 60, hr_v, lw=0.8, color="#d62728")
    ax[1].set_ylabel("Пульс, уд/мин")
    ax[2].plot((ts - t0) / 60, resp, lw=0.6, color="#2ca02c")
    ax[2].set_ylabel("Дыхание")
    ax[2].set_xlabel("минуты")
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
        ax[i, 0].set_ylabel(f"{lab}\n(n={len(items)})\nΔ {units}")
        ax[i, 1].set_ylabel("Δ уд/мин")
        for a in ax[i]:
            a.axvline(0, color="k", lw=1)
            a.axhline(0, color="#999", lw=0.5)
            a.axvspan(1, 4, color="#ffd54f", alpha=0.15)
            a.grid(alpha=0.2)
    ax[0, 0].set_title("Кожная проводимость относительно 2 с до метки")
    ax[0, 1].set_title("Пульс относительно 3 с до метки")
    ax[-1, 0].set_xlabel("секунды от метки")
    ax[-1, 1].set_xlabel("секунды от метки")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "responses.png"), dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
