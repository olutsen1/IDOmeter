# IDOmeter

An open-source instrument for watching the body's response to therapeutic
interventions in real time: skin conductance (EDA/GSR), pulse (PPG) and,
later, breathing. Built by a physician and Gestalt therapist for
phenomenological experiments: mark an intervention with one key press and
see what the body does in the following seconds.

> **Not a medical device.** IDOmeter is a research and educational tool. It is
> not intended for diagnosis or treatment and has not been certified for
> medical use. Use it only with the informed consent of the person wearing it.

## What's in the repository

| Folder | Contents |
|---|---|
| `software/` | Windows/Linux/macOS app (Python): live plots, intervention markers, session recording, response analysis, device simulator |
| `firmware/IDometer_fw/` | ESP32 firmware (TTGO T-Display) for Arduino IDE, ESP32 core 3.x |
| `hardware/` | wiring diagram and connection table of the prototype board |
| `legacy/` | the original 2023–2025 prototype firmware (MQTT + Raspberry Pi + Grafana) |

The interface is in English and Ukrainian; another language is one more text
file in `software/lang/` (see `software/README.md`). User guides are in English
and Ukrainian (`*_uk`).

## Hardware (prototype board IDometer V1.0)

- LilyGO TTGO T-Display (ESP32, 1.14" ST7789 screen)
- MAX30102 pulse oximeter module (I²C: SDA 21, SCL 22)
- Seeed Grove GSR sensor (analog, GPIO36)
- microSD module HW-125 (HSPI: SCLK 25, MISO 27, MOSI 26, CS 33) — optional backup recording

## How it works

- **The device does not know real time.** It samples on a hardware timer
  (PPG 100 Hz, EDA 20 Hz), numbers every packet and stamps it with its own
  microsecond counter. Packets go over Wi-Fi (UDP, port 5005).
- **The computer aligns the clocks** using the lower envelope of packet
  arrival delays, including crystal drift. Markers are set on the computer's
  clock, so signals and markers share one timeline with millisecond accuracy
  (≈3 ms in simulator tests, even with Wi-Fi jitter and packet loss).
- **No processing on the device.** Raw PPG and EDA are sent; heart beats,
  heart rate and skin conductance responses (SCR) are computed on the computer.
- **Breathing from the pulse wave** when there is no breathing sensor: the
  breathing rhythm is extracted from the PPG baseline (`software/breathing.py`).
- **Wi-Fi setup without recompiling:** the device remembers up to 5 networks;
  hold the right button at power-on (or wait 30 s without a known network)
  and connect a phone to `IDometer-setup`.
- **Analysis** cuts a −3…+12 s window around each marker, measures SCR
  amplitude/latency and heart-rate change, overlays responses by marker type,
  and reports a *chance level* — how often a "response" appears in windows
  without markers — so real effects can be told from spontaneous activity.

## Quick start (no hardware needed)

1. Install Python 3.11+ from python.org.
2. In `software/`, run `1_install.bat`, then `2_demo_with_simulator.bat`.
3. Press **R** to record, **1…9** to place markers. The simulator "responds"
   to markers like a body would.
4. `python analyze.py sessions\<folder> --truth sim_truth.csv`

User guide: `software/README.md` (Ukrainian: `README_uk.md`). Firmware flashing: `firmware/HOW_TO_FLASH.md` (`HOW_TO_FLASH_uk.md`).

## Protocol v1

UDP, little-endian. 20-byte header: `'ID'`, version=1, type=1, `uint32 seq`,
`uint64 t0_us`, `uint16 n_ticks`, `uint16 flags`; then `n_ticks × uint32` IR,
`n_ticks × uint32` red, `n_ticks/5 × int32` EDA, `n_ticks/5 × int32`
respiration. Details in `software/protocol.py`.

## Privacy

Session recordings are personal health data. They are stored only locally in
`software/sessions/`, which is excluded from git by `.gitignore`.

## License

GNU GPL v3 — see `LICENSE`.
