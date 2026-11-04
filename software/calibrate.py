"""
Resistor calibration of the skin-conductance channel, guided step by step.

The board runs the `board_check` sketch (tools/board_check), which prints the
GSR voltage from both ADCs twice a second over USB. This program reads that
output, tells you which resistor to connect, records each step by itself and
saves everything to calibration/<date_time>/:

    summary.csv   one row per step: resistor, conductance, mean and spread of both ADCs
    samples.csv   every reading with its step label
    board_log.txt the board's output exactly as received
    console.txt   what was shown on screen

Usage: python calibrate.py [--port COM5] [--lang uk]
Close the Arduino IDE Serial Monitor first: only one program can use the port.
"""
import argparse
import csv
import math
import os
import queue
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime

from i18n import init_from_config, set_language, tr

RESISTORS = [1_000_000, 470_000, 220_000, 100_000, 47_000]   # ohms, connected in this order
OPEN_START_S = 15       # electrodes open at the start
SETTLE_S = 2            # ignored after each change (contact settling)
HOLD_S = 12             # recorded per resistor
OPEN_BETWEEN_S = 8      # recorded with electrodes open after each resistor
UNSTABLE_MV = 5.0       # ADS1115 spread above this suggests a poor contact
KNOWN_USB = {0x1A86: "CH340/CH9102", 0x10C4: "CP210x", 0x303A: "Espressif USB", 0x0403: "FTDI"}

console = []


def say(text=""):
    print(text, flush=True)
    console.append(text)


def ask(prompt):
    console.append(prompt)
    try:
        ans = input(prompt)
    except EOFError:
        ans = "q"
    console.append(f"> {ans}")
    return ans.strip()


def ohms_text(r):
    return f"{r / 1e6:g} MΩ" if r >= 1e6 else f"{r / 1e3:g} kΩ"


# ---------------------------------------------------------------- serial port
def pick_port(requested):
    import serial.tools.list_ports
    if requested:
        return requested
    ports = list(serial.tools.list_ports.comports())
    likely = [p for p in ports if p.vid in KNOWN_USB]
    cands = likely or ports
    if not cands:
        say(tr("cal_no_port"))
        sys.exit(1)
    if len(cands) == 1:
        return cands[0].device
    say(tr("cal_choose_port"))
    for i, p in enumerate(cands, 1):
        say(f"  {i}. {p.device}  {p.description}")
    while True:
        a = ask(tr("cal_port_prompt", n=len(cands)))
        if a.isdigit() and 1 <= int(a) <= len(cands):
            return cands[int(a) - 1].device


def parse_line(line):
    """'  12.5 | 2104 | 2098.6 | +0.3 %' -> (12.5, 2104.0, 2098.6); ADS may be missing (nan)."""
    parts = line.split("|")
    if len(parts) < 3:
        return None
    try:
        t = float(parts[0])
        esp = float(parts[1])
    except ValueError:
        return None
    try:
        ads = float(parts[2])
    except ValueError:
        ads = math.nan
    return t, esp, ads


class Board:
    """Reads the board's lines in the background; keeps the parsed samples."""

    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.lines = []          # (host time, raw line)
        self.samples = queue.Queue()
        self.ads_found = None
        self.resets = 0
        self.stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        buf = b""
        while not self.stop:
            try:
                buf += self.ser.read(256)
            except Exception:
                break
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").rstrip("\r")
                now = time.monotonic()
                self.lines.append((now, line))
                if "IDometer board check" in line and self.ads_found is not None:
                    self.resets += 1
                if "ADS1115" in line and "(0x48)" in line:
                    self.ads_found = "found" in line and "NOT" not in line
                s = parse_line(line)
                if s:
                    self.samples.put((now,) + s)

    def drain(self):
        while not self.samples.empty():
            self.samples.get_nowait()

    def record(self, seconds, label, settle=0.0):
        """Collect samples for `seconds` after `settle`; shows a one-line live readout."""
        self.drain()
        t_end_settle = time.monotonic() + settle
        t_end = t_end_settle + seconds
        got = []
        while time.monotonic() < t_end:
            try:
                s = self.samples.get(timeout=0.3)
            except queue.Empty:
                continue
            left = max(0, t_end - time.monotonic())
            ads = "—" if math.isnan(s[3]) else f"{s[3]:.1f}"
            print(f"\r    {tr('cal_live', left=left, esp=s[2], ads=ads)}   ", end="", flush=True)
            if s[0] >= t_end_settle:
                got.append(s + (label,))
        print("\r" + " " * 78 + "\r", end="", flush=True)
        return got

    def close(self):
        self.stop = True
        time.sleep(0.3)
        self.ser.close()


def stats(rows):
    esp = [r[2] for r in rows]
    ads = [r[3] for r in rows if not math.isnan(r[3])]
    out = {"n": len(rows),
           "esp": statistics.fmean(esp) if esp else math.nan,
           "esp_sd": statistics.pstdev(esp) if len(esp) > 1 else math.nan,
           "ads": statistics.fmean(ads) if ads else math.nan,
           "ads_sd": statistics.pstdev(ads) if len(ads) > 1 else math.nan}
    return out


def show(st):
    ads = "—" if math.isnan(st["ads"]) else f"{st['ads']:.1f} ± {st['ads_sd']:.1f}"
    say("    " + tr("cal_result", esp=f"{st['esp']:.0f} ± {st['esp_sd']:.1f}", ads=ads, n=st["n"]))


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="IDometer resistor calibration")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0 (default: found automatically)")
    ap.add_argument("--lang", help="interface language (default: from config.json)")
    args = ap.parse_args()
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    init_from_config()
    if args.lang:
        set_language(args.lang)

    say(tr("cal_title"))
    try:
        import serial  # noqa: F401
    except ImportError:
        say(tr("cal_no_pyserial"))
        sys.exit(1)
    port = pick_port(args.port)
    try:
        board = Board(port)
    except Exception as e:
        say(tr("cal_port_busy", port=port, err=e))
        if isinstance(e, PermissionError) or "ermission" in str(e):
            say(tr("cal_linux_dialout"))
        sys.exit(1)
    say(tr("cal_connected", port=port))

    # opening the port restarts the board; wait for its first readings
    t0 = time.monotonic()
    while board.samples.empty() and time.monotonic() - t0 < 12:
        time.sleep(0.2)
    if board.samples.empty():
        say(tr("cal_no_data"))
        board.close()
        sys.exit(1)
    say(tr("cal_ads_ok") if board.ads_found else tr("cal_ads_missing"))

    folder = os.path.join("calibration", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    os.makedirs(folder, exist_ok=True)
    samples, summary = [], []

    say()
    say(tr("cal_intro", hold=HOLD_S, n=len(RESISTORS)))
    if ask(tr("cal_start_prompt")).lower() == "q":
        board.close()
        return

    say(tr("cal_open_start", s=OPEN_START_S))
    rows = board.record(OPEN_START_S, "open")
    samples += rows
    st = stats(rows)
    show(st)
    summary.append(("open", "", "", 0.0, st))

    i = 0
    while i < len(RESISTORS):
        r = RESISTORS[i]
        say()
        say(tr("cal_step", i=i + 1, n=len(RESISTORS), r=ohms_text(r), us=1e6 / r))
        a = ask(tr("cal_connect_prompt", r=ohms_text(r)))
        if a.lower() == "q":
            break
        if a.lower() == "s":
            say(tr("cal_skipped"))
            i += 1
            continue
        measured = ""
        if a:
            try:
                measured = float(a.replace(",", ".")) * 1000
            except ValueError:
                say(tr("cal_bad_number"))
                continue
        label = f"R{i + 1}"
        rows = board.record(HOLD_S, label, settle=SETTLE_S)
        st = stats(rows)
        show(st)
        unstable = (not math.isnan(st["ads_sd"]) and st["ads_sd"] > UNSTABLE_MV) or st["n"] < HOLD_S
        if unstable:
            say(tr("cal_unstable"))
            if ask(tr("cal_repeat_prompt")).lower() != "n":
                continue
        samples += rows
        r_used = measured or r
        summary.append((label, r, measured, 1e6 / r_used, st))

        ask(tr("cal_disconnect_prompt"))
        rows = board.record(OPEN_BETWEEN_S, f"open after {label}", settle=SETTLE_S)
        samples += rows
        st = stats(rows)
        show(st)
        summary.append((f"open after {label}", "", "", 0.0, st))
        i += 1

    if board.resets:
        say(tr("cal_resets", n=board.resets))
    board.close()

    # ---- save
    with open(os.path.join(folder, "summary.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["step", "nominal_ohm", "measured_ohm", "conductance_uS", "n",
                    "esp32_mV", "esp32_sd_mV", "ads1115_mV", "ads1115_sd_mV"])
        for label, nom, meas, us, st in summary:
            w.writerow([label, nom, meas, f"{us:.4f}", st["n"], f"{st['esp']:.1f}", f"{st['esp_sd']:.2f}",
                        f"{st['ads']:.2f}", f"{st['ads_sd']:.2f}"])
    with open(os.path.join(folder, "samples.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["host_s", "board_s", "esp32_mV", "ads1115_mV", "step"])
        h0 = samples[0][0] if samples else 0
        for s in samples:
            w.writerow([f"{s[0] - h0:.2f}", s[1], s[2], "" if math.isnan(s[3]) else s[3], s[4]])
    with open(os.path.join(folder, "board_log.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(line for _, line in board.lines) + "\n")

    say()
    say(tr("cal_table_head"))
    for label, nom, meas, us, st in summary:
        if nom:
            ads = "—" if math.isnan(st["ads"]) else f"{st['ads']:.1f}"
            say(f"  {ohms_text(meas or nom):>9}  {us:6.2f}  {st['esp']:7.0f}  {ads:>8}")
    say()
    say(tr("cal_saved", path=os.path.abspath(folder)))
    with open(os.path.join(folder, "console.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(console) + "\n")
    try:
        if sys.platform.startswith("win"):
            os.startfile(os.path.abspath(folder))
        elif sys.platform == "darwin":
            subprocess.Popen(["open", folder])
        else:
            subprocess.Popen(["xdg-open", folder], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


if __name__ == "__main__":
    main()
