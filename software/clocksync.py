"""
Перевод времени устройства во время компьютера.

Каждый пакет даёт пару (t_end устройства, момент прихода на компьютер).
Момент прихода = истинное время + задержка сети (всегда >= 0, иногда большая).
Поэтому берём не среднее, а НИЖНЮЮ огибающую: минимальное смещение в каждом
коротком интервале, и по этим минимумам строим прямую. Наклон прямой учитывает
дрейф кварца устройства (обычно десятки ppm), сдвиг — разницу часов.

Точность на практике: единицы миллисекунд даже при Wi-Fi с задержками
в сотни миллисекунд.
"""
from collections import OrderedDict

import numpy as np


def _fit_lower_envelope(d_s, off, iterations=2):
    """Линейная регрессия по минимумам с отбрасыванием выбросов вверх."""
    d_s = np.asarray(d_s, float)
    off = np.asarray(off, float)
    mask = np.ones(len(d_s), bool)
    ref = d_s.mean()
    a, b = float(np.min(off)), 0.0
    for _ in range(iterations + 1):
        if mask.sum() < 3:
            break
        b, a = np.polyfit(d_s[mask] - ref, off[mask], 1)
        resid = off - (a + b * (d_s - ref))
        mad = np.median(np.abs(resid[mask] - np.median(resid[mask]))) + 1e-6
        mask = resid < 3 * mad + 0.002
    return a, b, ref


class ClockSync:
    """Онлайн-оценка соответствия «время устройства → время компьютера» (секунды)."""

    def __init__(self, bucket_s=2.0, window_buckets=60):
        self.bucket_s = bucket_s
        self.window = window_buckets
        self.buckets = OrderedDict()   # ключ -> (d_s, offset)
        self._a = None
        self._b = 0.0
        self._ref = 0.0
        self.recent_offsets = []

    def reset(self):
        self.__init__(self.bucket_s, self.window)

    def add(self, d_end_us: int, host_s: float):
        d = d_end_us / 1e6
        off = host_s - d
        key = int(host_s // self.bucket_s)
        cur = self.buckets.get(key)
        if cur is None or off < cur[1]:
            self.buckets[key] = (d, off)
        while len(self.buckets) > self.window:
            self.buckets.popitem(last=False)
        self.recent_offsets.append(off)
        if len(self.recent_offsets) > 200:
            del self.recent_offsets[:100]
        self._refit()

    def _refit(self):
        vals = list(self.buckets.values())
        if len(vals) < 4:
            self._a = min(v[1] for v in vals)
            self._b = 0.0
            self._ref = 0.0
            return
        d = [v[0] for v in vals]
        o = [v[1] for v in vals]
        self._a, self._b, self._ref = _fit_lower_envelope(d, o)

    @property
    def ready(self):
        return self._a is not None

    def to_host(self, d_us):
        d = np.asarray(d_us, float) / 1e6
        return d + self._a + self._b * (d - self._ref)

    @property
    def drift_ppm(self):
        return -self._b * 1e6   # >0: часы устройства спешат

    def latency_ms(self):
        """Типичная задержка сверх минимальной (медиана), мс — показатель качества связи."""
        if not self.recent_offsets or self._a is None:
            return float("nan")
        return 1000 * (np.median(self.recent_offsets[-50:]) - min(self.recent_offsets[-50:]))


def fit_global(d_end_us, host_s, bucket_s=2.0):
    """Подгонка по всей сессии сразу (для анализа после записи). Возвращает функцию."""
    d_end_us = np.asarray(d_end_us, float)
    host_s = np.asarray(host_s, float)
    off = host_s - d_end_us / 1e6
    keys = np.floor(host_s / bucket_s).astype(np.int64)
    bd, bo = [], []
    for k in np.unique(keys):
        idx = np.where(keys == k)[0]
        j = idx[np.argmin(off[idx])]
        bd.append(d_end_us[j] / 1e6)
        bo.append(off[j])
    if len(bd) < 3:
        a0 = float(np.min(off))
        return lambda d_us: np.asarray(d_us, float) / 1e6 + a0
    a, b, ref = _fit_lower_envelope(bd, bo)

    def to_host(d_us):
        d = np.asarray(d_us, float) / 1e6
        return d + a + b * (d - ref)
    to_host.drift_ppm = -b * 1e6
    return to_host
