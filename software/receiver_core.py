"""
Ядро приёмника: приём пакетов, синхронизация часов, буферы, метки, запись сессии,
обнаружение ударов пульса. Без графического интерфейса — его использует
receiver.py (окно) и режим --headless (для проверки без экрана).
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
from scipy.signal import butter, find_peaks, sosfiltfilt

import protocol as P
from clocksync import ClockSync

DEFAULT_CONFIG = {
    "listen_port": P.DATA_PORT,
    "sessions_dir": "sessions",
    "window_s": 30,
    "markers": {
        "1": "Вопрос",
        "2": "Отражение",
        "3": "Эксперимент",
        "4": "Усиление",
        "5": "Самораскрытие",
        "6": "Пауза / тишина",
        "7": "Тема клиента",
        "8": "Прикосновение",
        "9": "Другое",
        "Space": "Событие",
        "B": "Базовая линия"
    },
    # Пересчёт сырого EDA.
    #  relative: 0…100 отн. ед. от полной шкалы АЦП; invert=true — для Grove GSR,
    #            у которого напряжение падает, когда проводимость кожи растёт
    #  linear:   мкСм = raw * gain + offset   (симулятор шлёт нС -> gain 0.001)
    #  table:    мкСм кусочно-линейно по точкам калибровки [[raw, мкСм], ...]
    #  none:     сырые отсчёты
    # При запуске с --sim всегда используется linear 0.001.
    "eda_calibration": {"type": "relative", "full_scale": 65520, "invert": True},
    "invert_ppg": True,
}


def load_config(path="config.json"):
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            user = json.load(f)
        cfg.update(user)
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


def eda_units(cal):
    return "мкСм" if cal.get("type") in ("linear", "table") else "отн. ед."


# ---------------------------------------------------------------- буферы

class Ring:
    """Кольцевой буфер нескольких синхронных колонок."""

    def __init__(self, capacity, ncols):
        self.cap = capacity
        self.data = np.full((capacity, ncols), np.nan)
        self.n = 0          # всего записано

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


# ---------------------------------------------------------------- пульс

_SOS = butter(2, [0.7, 3.5], btype="band", fs=P.BASE_RATE_HZ, output="sos")


def detect_beats(t, ppg, fs=P.BASE_RATE_HZ):
    """Моменты пиков пульсовой волны. ppg уже с правильной полярностью (вверх = систола)."""
    ok = np.isfinite(ppg)
    if ok.sum() < 3 * fs:
        return np.array([])
    t, x = t[ok], ppg[ok]
    x = sosfiltfilt(_SOS, x - np.mean(x))
    prom = 0.4 * np.std(x)
    pk, _ = find_peaks(x, distance=int(0.33 * fs), prominence=prom)
    # уточнение вершины параболой
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


class LiveBeats:
    def __init__(self):
        self.beats = []        # время ударов (сек приложения)
        self.hr = []           # (t, уд/мин)

    def update(self, t, ppg, now_t):
        if len(t) < 300:
            return
        sel = t > now_t - 12
        bt = detect_beats(t[sel], ppg[sel])
        for b in bt:
            if b > now_t - 1.5:           # у края окна фильтр ещё не устоялся
                continue
            if self.beats and b <= self.beats[-1] + 0.3:
                continue
            if self.beats:
                ibi = b - self.beats[-1]
                if 0.33 <= ibi <= 1.6:
                    self.hr.append((b, 60.0 / ibi))
            self.beats.append(b)
        if len(self.beats) > 2000:
            self.beats = self.beats[-1000:]
            self.hr = self.hr[-1000:]


# ---------------------------------------------------------------- запись

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
            "time_note": "t — секунды часов приложения (perf_counter от запуска); "
                         "device_us — часы устройства; packets.csv позволяет пересчитать t",
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


# ---------------------------------------------------------------- приём

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
            # метка времени ставится сразу при приходе, до любой обработки
            t = self.clock()
            self.q.put((t, data, addr))
            # отвечаем устройству «IDHI», чтобы оно узнало адрес компьютера
            # и перешло с рассылки всем на отправку только сюда
            last = self.hello.get(addr)
            if last is None or t - last > 2.0:
                self.hello[addr] = t
                try:
                    self.sock.sendto(b"IDHI", addr)
                except OSError:
                    pass


class Engine:
    """Всё, кроме окна. Вызывайте poll() регулярно (в окне — по таймеру)."""

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
        self.fast = Ring(cap_fast, 3)    # t, ppg_ir (для показа), ppg_red
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
        self.sim_feedback = sim_feedback   # (host, port) или None
        self._fb_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if sim_feedback else None

    def now(self):
        return time.perf_counter() - self.app_t0

    # --- данные
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
                self.beats.update(tail[:, 0], tail[:, 1], now)
        return n

    def _handle(self, pkt, arrival):
        # перезапуск устройства: часы ушли назад
        if self.last_t_end is not None and pkt.t0_us < self.last_t_end - 1_000_000:
            self.sync.reset()
            self.last_seq = None
            self.resets += 1
        if self.last_seq is not None:
            gap = (pkt.seq - self.last_seq) & 0xFFFFFFFF
            if gap == 0 or gap > 0x7FFFFFFF:      # дубль или опоздавший пакет
                return
            if gap > 1:
                self.lost += gap - 1
                # разрыв на графике
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

    # --- метки
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

    # --- запись
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

    # --- состояние
    def stats(self):
        return {
            "packets": self.packets,
            "lost": self.lost,
            "bad": self.bad,
            "device_resets": self.resets,
            "drift_ppm": round(float(self.sync.drift_ppm), 1) if self.sync.ready else None,
        }

    def connected(self):
        return self.last_rx is not None and self.now() - self.last_rx < 2.0
