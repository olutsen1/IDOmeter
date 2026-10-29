"""
IDometer — протокол пакета данных, версия 1.

Один и тот же формат используют симулятор (simulator.py) и будущая прошивка ESP32.
Транспорт: UDP-датаграммы на порт приёмника (по умолчанию 5005).

Принцип: устройство НЕ знает реального времени. Оно нумерует пакеты и ставит
метку по своему внутреннему счётчику микросекунд. Перевод во время компьютера
делает приёмник (см. clocksync.py).

Структура пакета DATA (все числа little-endian):

  Заголовок, 20 байт:
    magic     2 байта  b'ID'
    version   uint8    1
    type      uint8    1 = DATA
    seq       uint32   номер пакета, растёт на 1, переполнение допустимо
    t0_us     uint64   время устройства (мкс) для ПЕРВОГО такта в пакете
    n_ticks   uint16   число тактов базовой частоты в пакете (кратно SLOW_DIV)
    flags     uint16   бит 0: палец на датчике PPG; бит 1: идёт запись на SD;
                       остальные зарезервированы

  Данные:
    ppg_ir    n_ticks           × uint32   (MAX30102, 18 бит)
    ppg_red   n_ticks           × uint32
    eda       n_ticks/SLOW_DIV  × int32    (сырые отсчёты АЦП или нС в симуляторе)
    resp      n_ticks/SLOW_DIV  × int32    (дыхание; 0, если канала нет)

  Такт k пакета имеет время устройства t0_us + k * TICK_US.
  Медленный отсчёт j относится к такту j * SLOW_DIV.
"""
import struct
from dataclasses import dataclass

import numpy as np

MAGIC = b"ID"
VERSION = 1
TYPE_DATA = 1

BASE_RATE_HZ = 100          # PPG
SLOW_DIV = 5                # EDA и дыхание: 100 / 5 = 20 Гц
SLOW_RATE_HZ = BASE_RATE_HZ // SLOW_DIV
TICK_US = 1_000_000 // BASE_RATE_HZ
DEFAULT_TICKS_PER_PACKET = 10   # 100 мс на пакет

FLAG_FINGER = 1 << 0
FLAG_SD_REC = 1 << 1

_HDR = struct.Struct("<2sBBIQHH")
HEADER_SIZE = _HDR.size     # 20


@dataclass
class DataPacket:
    seq: int
    t0_us: int
    n_ticks: int
    flags: int
    ppg_ir: np.ndarray
    ppg_red: np.ndarray
    eda: np.ndarray
    resp: np.ndarray

    @property
    def t_end_us(self) -> int:
        """Время устройства сразу после последнего такта (момент готовности пакета)."""
        return self.t0_us + self.n_ticks * TICK_US

    def fast_times_us(self) -> np.ndarray:
        return self.t0_us + np.arange(self.n_ticks, dtype=np.int64) * TICK_US

    def slow_times_us(self) -> np.ndarray:
        n = self.n_ticks // SLOW_DIV
        return self.t0_us + np.arange(n, dtype=np.int64) * SLOW_DIV * TICK_US


def encode(p: DataPacket) -> bytes:
    if p.n_ticks % SLOW_DIV:
        raise ValueError("n_ticks должно быть кратно SLOW_DIV")
    ns = p.n_ticks // SLOW_DIV
    hdr = _HDR.pack(MAGIC, VERSION, TYPE_DATA, p.seq & 0xFFFFFFFF,
                    p.t0_us, p.n_ticks, p.flags)
    return (hdr
            + np.asarray(p.ppg_ir, "<u4")[:p.n_ticks].tobytes()
            + np.asarray(p.ppg_red, "<u4")[:p.n_ticks].tobytes()
            + np.asarray(p.eda, "<i4")[:ns].tobytes()
            + np.asarray(p.resp, "<i4")[:ns].tobytes())


def decode(buf: bytes) -> DataPacket:
    if len(buf) < HEADER_SIZE:
        raise ValueError("короткий пакет")
    magic, ver, typ, seq, t0, n, flags = _HDR.unpack_from(buf, 0)
    if magic != MAGIC or ver != VERSION or typ != TYPE_DATA:
        raise ValueError("чужой или неподдерживаемый пакет")
    if n == 0 or n % SLOW_DIV:
        raise ValueError("неверное n_ticks")
    ns = n // SLOW_DIV
    need = HEADER_SIZE + 4 * (2 * n + 2 * ns)
    if len(buf) != need:
        raise ValueError(f"длина {len(buf)}, ожидалось {need}")
    o = HEADER_SIZE
    ir = np.frombuffer(buf, "<u4", n, o); o += 4 * n
    red = np.frombuffer(buf, "<u4", n, o); o += 4 * n
    eda = np.frombuffer(buf, "<i4", ns, o); o += 4 * ns
    resp = np.frombuffer(buf, "<i4", ns, o)
    return DataPacket(seq, t0, n, flags, ir.astype(np.int64), red.astype(np.int64),
                      eda.astype(np.int64), resp.astype(np.int64))


# Служебное сообщение приёмника симулятору: b"MARK <метка>" (UTF-8).
# Нужно только для проверки цепочки: симулятор «реагирует телом» на метки.
SIM_FEEDBACK_PORT = 5006
DATA_PORT = 5005
