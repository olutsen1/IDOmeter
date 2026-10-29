"""
IDometer — симулятор устройства («тело в коробке»).

Генерирует правдоподобные сигналы и шлёт их по протоколу v1 (protocol.py),
как это будет делать настоящее устройство:

  * кожная проводимость: медленный тонический уровень + фазические ответы (SCR),
    включая спонтанные, не связанные ни с какими событиями;
  * PPG (MAX30102, ИК и красный канал) с пульсом, дыхательной аритмией и шумом;
  * дыхание.

Имитирует и несовершенства реального устройства: собственные часы с дрейфом
и случайным начальным сдвигом, задержки и «заикания» Wi-Fi, потерю пакетов.

Главное: симулятор слушает метки приёмника (порт 5006) и с заданной
вероятностью «отвечает телом» — SCR через 1–2,5 с, замедление, затем
ускорение пульса, иногда глубокий вдох. Все заложенные ответы записываются
в sim_truth.csv в единицах времени УСТРОЙСТВА, чтобы анализ мог проверить,
находит ли он то, что было заложено.

Запуск:  python simulator.py
         python simulator.py --help
"""
import argparse
import csv
import heapq
import math
import queue
import random
import socket
import threading
import time

import numpy as np

import i18n
import protocol as P
from i18n import tr


# ---------------------------------------------------------------- физиология

def _bateman_peak(tr, td):
    xs = np.linspace(0, 20, 20001)
    return float(np.max(np.exp(-xs / td) - np.exp(-xs / tr)))


class Body:
    TAU_RISE = 0.75
    TAU_DECAY = 2.5

    def __init__(self, rng: random.Random, spont_per_min=2.0):
        self.rng = rng
        self.spont_rate = spont_per_min / 60.0
        self._norm = _bateman_peak(self.TAU_RISE, self.TAU_DECAY)
        self.tonic = 5.0
        self.tonic_target = 5.0
        self.scrs = []                  # (onset_s, amp_uS)
        self.hr_events = []             # onset_s
        self.sighs = []                 # onset_s
        self.resp_phase = 0.0
        self.resp_rate = 14.0
        self.hr_noise = 0.0
        self.beats = []
        self.next_beat = None
        self.new_beats = []             # для журнала
        self.t_last = None

    # --- события
    def add_scr(self, onset, amp):
        self.scrs.append((onset, amp))

    def add_hr_response(self, onset):
        self.hr_events.append(onset)

    def add_sigh(self, onset):
        self.sighs.append(onset)

    # --- компоненты
    def _bateman(self, x):
        if x <= 0:
            return 0.0
        return (math.exp(-x / self.TAU_DECAY) - math.exp(-x / self.TAU_RISE)) / self._norm

    def eda(self, t):
        v = self.tonic
        for on, a in self.scrs:
            v += a * self._bateman(t - on)
        return v

    def _hr(self, t):
        hr = 68.0 + self.hr_noise + 4.0 * math.sin(self.resp_phase)
        for on in self.hr_events:
            x = t - on
            hr += -3.0 * math.exp(-0.5 * ((x - 1.5) / 0.7) ** 2)
            hr += 6.0 * math.exp(-0.5 * ((x - 5.0) / 1.6) ** 2)
        return hr

    def _resp_amp(self, t):
        a = 1.0
        for on in self.sighs:
            a += 0.9 * math.exp(-0.5 * ((t - on - 1.5) / 1.2) ** 2)
        return a

    @staticmethod
    def _pulse(x):
        if x < 0 or x > 1.0:
            return 0.0
        return (math.exp(-0.5 * ((x - 0.18) / 0.055) ** 2)
                + 0.35 * math.exp(-0.5 * ((x - 0.42) / 0.07) ** 2))

    # --- шаг 10 мс
    def step(self, t, slow: bool, spont_cb=None):
        dt = 0.0 if self.t_last is None else t - self.t_last
        self.t_last = t
        # дыхание
        self.resp_rate += (14.0 - self.resp_rate) * dt / 30 + self.rng.gauss(0, 0.3) * math.sqrt(max(dt, 0))
        self.resp_phase += 2 * math.pi * self.resp_rate / 60.0 * dt
        # медленный шум пульса
        self.hr_noise += -self.hr_noise * dt / 20 + self.rng.gauss(0, 0.6) * math.sqrt(max(dt, 0))
        # удары
        if self.next_beat is None:
            self.next_beat = t
        while self.next_beat <= t:
            self.beats.append(self.next_beat)
            self.new_beats.append(self.next_beat)
            self.next_beat += 60.0 / self._hr(self.next_beat)
        self.beats = [b for b in self.beats if t - b < 1.2]
        pulse = sum(self._pulse(t - b) for b in self.beats)
        resp = self._resp_amp(t) * math.sin(self.resp_phase)
        ir = 120000 + 400 * resp - 1300 * pulse + self.rng.gauss(0, 25)
        red = 95000 + 300 * resp - 900 * pulse + self.rng.gauss(0, 25)

        eda = resp_out = None
        if slow:
            # тоника: медленное блуждание
            if self.rng.random() < 0.002:
                self.tonic_target = min(12, max(2, self.tonic_target + self.rng.gauss(0, 0.8)))
            self.tonic += (self.tonic_target - self.tonic) * 0.05 / 60 + self.rng.gauss(0, 0.002)
            # спонтанные SCR
            if self.rng.random() < self.spont_rate * 0.05:
                amp = self.rng.uniform(0.05, 0.25)
                self.add_scr(t, amp)
                if spont_cb:
                    spont_cb(t, amp)
            self.scrs = [(o, a) for o, a in self.scrs if t - o < 30]
            self.hr_events = [o for o in self.hr_events if t - o < 15]
            self.sighs = [o for o in self.sighs if t - o < 10]
            eda = self.eda(t) + self.rng.gauss(0, 0.008)
            resp_out = resp + self.rng.gauss(0, 0.02)
        return ir, red, eda, resp_out


# ---------------------------------------------------------------- «Wi-Fi»

class NetSender(threading.Thread):
    """Отправка с задержками, «заиканиями» и потерями. Порядок пакетов сохраняется."""

    def __init__(self, addr, rng, loss, jitter_ms, stall_every_s):
        super().__init__(daemon=True)
        self.addr = addr
        self.rng = rng
        self.loss = loss
        self.jitter = jitter_ms / 1000
        self.stall_every = stall_every_s
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.heap = []
        self.cv = threading.Condition()
        self.last_send = 0.0
        self.stall_until = 0.0
        self.counter = 0
        self.lost = 0

    def submit(self, data: bytes, ready_t: float):
        if self.rng.random() < self.loss:
            self.lost += 1
            return
        if self.stall_every > 0 and self.rng.random() < 0.1 / self.stall_every:
            self.stall_until = ready_t + self.rng.uniform(0.3, 1.2)
        delay = 0.002 + self.rng.expovariate(1 / self.jitter) if self.jitter > 0 else 0.002
        send_at = max(ready_t + delay, self.last_send, self.stall_until)
        self.last_send = send_at
        with self.cv:
            heapq.heappush(self.heap, (send_at, self.counter, data))
            self.counter += 1
            self.cv.notify()

    def run(self):
        while True:
            with self.cv:
                while not self.heap:
                    self.cv.wait()
                send_at, _, data = self.heap[0]
                wait = send_at - time.perf_counter()
                if wait > 0:
                    self.cv.wait(timeout=wait)
                    continue
                heapq.heappop(self.heap)
            try:
                self.sock.sendto(data, self.addr)
            except OSError:
                pass


# ---------------------------------------------------------------- основной цикл

def main():
    ap = argparse.ArgumentParser(description="IDometer device simulator")
    ap.add_argument("--host", default="127.0.0.1", help="receiver address")
    ap.add_argument("--port", type=int, default=P.DATA_PORT)
    ap.add_argument("--feedback-port", type=int, default=P.SIM_FEEDBACK_PORT,
                    help="port where the simulator listens for markers")
    ap.add_argument("--p-response", type=float, default=0.8,
                    help="probability of a bodily response to a marker")
    ap.add_argument("--habituation", type=float, default=0.9,
                    help="amplitude factor for each repeat of the same marker type")
    ap.add_argument("--spont", type=float, default=2.0, help="spontaneous SCRs per minute")
    ap.add_argument("--drift-ppm", type=float, default=40.0, help="device clock drift")
    ap.add_argument("--jitter-ms", type=float, default=10.0, help="mean network delay")
    ap.add_argument("--stall-every", type=float, default=40.0,
                    help="Wi-Fi stalls on average every N seconds (0 = never)")
    ap.add_argument("--loss", type=float, default=0.005, help="fraction of lost packets")
    ap.add_argument("--truth", default="sim_truth.csv", help="log of planted responses")
    ap.add_argument("--duration", type=float, default=0, help="stop after N seconds")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()
    i18n.init_from_config()

    rng = random.Random(args.seed)
    body = Body(rng, args.spont)
    net = NetSender((args.host, args.port), rng, args.loss, args.jitter_ms, args.stall_every)
    net.start()

    # часы устройства: свой начальный сдвиг и дрейф
    t_start = time.perf_counter()
    dev_offset = rng.uniform(5, 50)          # «устройство включено N секунд назад»
    k_drift = 1 + args.drift_ppm * 1e-6

    def dev_us(t):
        return int(round(((t - t_start) * k_drift + dev_offset) * 1e6))

    # журнал истины
    truth_f = open(args.truth, "w", newline="", encoding="utf-8")
    truth = csv.writer(truth_f)
    truth.writerow(["kind", "label", "stim_device_us", "onset_device_us", "scr_amp_uS",
                    "hr_response", "sigh"])
    beats_f = open(args.truth.replace(".csv", "_beats.csv"), "w", newline="")
    beats = csv.writer(beats_f)
    beats.writerow(["beat_device_us"])

    def on_spont(t, amp):
        truth.writerow(["spontaneous", "", "", dev_us(t), f"{amp:.4f}", 0, 0])

    # метки от приёмника
    marks = queue.Queue()

    def listen():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", args.feedback_port))
        while True:
            data, _ = s.recvfrom(1024)
            if data.startswith(b"MARK"):
                marks.put((time.perf_counter(), data[5:].decode("utf-8", "replace")))
    threading.Thread(target=listen, daemon=True).start()

    label_counts = {}
    n = P.DEFAULT_TICKS_PER_PACKET
    block = n / P.BASE_RATE_HZ
    seq = 0
    print(tr("sim_start", host=args.host, port=args.port, fb=args.feedback_port))
    print(tr("sim_params", drift=args.drift_ppm, loss=args.loss, truth=args.truth))
    print(tr("sim_stop_hint"))
    try:
        while True:
            t_block = t_start + seq * block
            t_ready = t_block + block
            now = time.perf_counter()
            if t_ready > now:
                time.sleep(t_ready - now)
            if args.duration and t_ready - t_start > args.duration:
                break

            # обработать метки, пришедшие к этому моменту
            while not marks.empty():
                t_stim, label = marks.get()
                cnt = label_counts.get(label, 0)
                label_counts[label] = cnt + 1
                if rng.random() < args.p_response:
                    onset = t_stim + rng.uniform(1.0, 2.5)
                    amp = rng.uniform(0.2, 0.8) * args.habituation ** cnt
                    sigh = rng.random() < 0.4
                    body.add_scr(onset, amp)
                    body.add_hr_response(t_stim)
                    if sigh:
                        body.add_sigh(t_stim)
                    truth.writerow(["response", label, dev_us(t_stim), dev_us(onset),
                                    f"{amp:.4f}", 1, int(sigh)])
                else:
                    truth.writerow(["no_response", label, dev_us(t_stim), "", "", 0, 0])
                truth_f.flush()

            ir = np.zeros(n, np.int64)
            red = np.zeros(n, np.int64)
            eda = []
            resp = []
            for i in range(n):
                t = t_block + i / P.BASE_RATE_HZ
                slow = (i % P.SLOW_DIV) == 0
                a, b, e, r = body.step(t, slow, on_spont)
                ir[i], red[i] = a, b
                if slow:
                    eda.append(round(e * 1000))       # нС
                    resp.append(round(r * 1000))
            for bt in body.new_beats:
                beats.writerow([dev_us(bt)])
            body.new_beats.clear()

            pkt = P.DataPacket(seq, dev_us(t_block), n, P.FLAG_FINGER,
                               ir, red, np.array(eda), np.array(resp))
            net.submit(P.encode(pkt), t_ready)
            seq += 1
            if seq % 100 == 0:
                truth_f.flush()
                beats_f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        truth_f.close()
        beats_f.close()
        print(tr("sim_done", n=seq, lost=net.lost))


if __name__ == "__main__":
    main()
