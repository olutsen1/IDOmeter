"""
IDometer data packet protocol, version 1.

The same format is used by the simulator (simulator.py) and the ESP32 firmware.
Transport: UDP datagrams to the receiver port (default 5005).

Principle: the device does NOT know real time. It numbers the packets and
stamps them with its own microsecond counter. The receiver converts this to
computer time (see clocksync.py).

DATA packet layout (all numbers little-endian):

  Header, 20 bytes:
    magic     2 bytes  b'ID'
    version   uint8    1
    type      uint8    1 = DATA
    seq       uint32   packet number, increments by 1, wraps around
    t0_us     uint64   device time (µs) of the FIRST tick in the packet
    n_ticks   uint16   number of base-rate ticks in the packet (multiple of SLOW_DIV)
    flags     uint16   bit 0: finger on the PPG sensor; bit 1: SD recording on;
                       other bits reserved

  Data:
    ppg_ir    n_ticks           × uint32   (MAX30102, 18 bit)
    ppg_red   n_ticks           × uint32
    eda       n_ticks/SLOW_DIV  × int32    (raw ADC counts, or nS in the simulator)
    resp      n_ticks/SLOW_DIV  × int32    (breathing; 0 if there is no such channel)

  Tick k of a packet has device time t0_us + k * TICK_US.
  Slow sample j belongs to tick j * SLOW_DIV.
"""
import struct
from dataclasses import dataclass

import numpy as np

MAGIC = b"ID"
VERSION = 1
TYPE_DATA = 1

BASE_RATE_HZ = 100          # PPG
SLOW_DIV = 5                # EDA and breathing: 100 / 5 = 20 Hz
SLOW_RATE_HZ = BASE_RATE_HZ // SLOW_DIV
TICK_US = 1_000_000 // BASE_RATE_HZ
DEFAULT_TICKS_PER_PACKET = 10   # 100 ms per packet

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
        """Device time right after the last tick (when the packet is complete)."""
        return self.t0_us + self.n_ticks * TICK_US

    def fast_times_us(self) -> np.ndarray:
        return self.t0_us + np.arange(self.n_ticks, dtype=np.int64) * TICK_US

    def slow_times_us(self) -> np.ndarray:
        n = self.n_ticks // SLOW_DIV
        return self.t0_us + np.arange(n, dtype=np.int64) * SLOW_DIV * TICK_US


def encode(p: DataPacket) -> bytes:
    if p.n_ticks % SLOW_DIV:
        raise ValueError("n_ticks must be a multiple of SLOW_DIV")
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
        raise ValueError("packet too short")
    magic, ver, typ, seq, t0, n, flags = _HDR.unpack_from(buf, 0)
    if magic != MAGIC or ver != VERSION or typ != TYPE_DATA:
        raise ValueError("foreign or unsupported packet")
    if n == 0 or n % SLOW_DIV:
        raise ValueError("invalid n_ticks")
    ns = n // SLOW_DIV
    need = HEADER_SIZE + 4 * (2 * n + 2 * ns)
    if len(buf) != need:
        raise ValueError(f"length {len(buf)}, expected {need}")
    o = HEADER_SIZE
    ir = np.frombuffer(buf, "<u4", n, o); o += 4 * n
    red = np.frombuffer(buf, "<u4", n, o); o += 4 * n
    eda = np.frombuffer(buf, "<i4", ns, o); o += 4 * ns
    resp = np.frombuffer(buf, "<i4", ns, o)
    return DataPacket(seq, t0, n, flags, ir.astype(np.int64), red.astype(np.int64),
                      eda.astype(np.int64), resp.astype(np.int64))


# Control message from the receiver to the simulator: b"MARK <label>" (UTF-8).
# Used only for end-to-end testing: the simulator "responds bodily" to markers.
SIM_FEEDBACK_PORT = 5006
DATA_PORT = 5005
