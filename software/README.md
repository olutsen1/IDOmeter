# IDOmeter app — user guide

Receives data from the device (or the simulator), shows live plots, lets you
mark interventions with key presses, records sessions and analyses responses
to markers. Everything runs on one computer: no Raspberry Pi, MQTT broker or
Grafana.

## Install (once)

1. Install Python 3.11 or newer from python.org.
2. Run `1_install.bat`.

## Try it without hardware

Run `2_demo_with_simulator.bat`. A simulator console and the app window open.
If Windows asks about network access for Python, allow it for private networks.

Press **R** to record, then place markers with **1…9**, **Space**, **B**.
The simulator responds like a body with 80 % probability: a rise in skin
conductance 1–2.5 s later, a heart-rate dip then rise, sometimes a deep breath.
It also produces spontaneous responses unrelated to markers, as a real body does.

## With the device

Flash the firmware (`../firmware/HOW_TO_FLASH.md`), then run
`3_receiver_real_device.bat`.

## Keys

| Key | Action |
|---|---|
| 1…9, Space, B | marker of that type |
| N | note: the time is taken when N is pressed; type the text, Enter |
| Esc | cancel the note |
| R | start / stop recording |
| F | freeze scrolling (then the plots can be panned with the mouse) |

Markers placed before pressing R are shown on the plots but not saved.

## Analysis

```
python analyze.py sessions\<folder>
python analyze.py sessions\<folder> --truth sim_truth.csv     (simulator check)
```

Results go to `sessions\<folder>\analysis`: `overview.png` (whole session),
`responses.png` (responses overlaid by marker type) and `responses.csv`.
The analysis also reports a **chance level**: how often a "response" is found
in windows without markers. Only a marker response rate clearly above it
suggests an effect.

## Language and marker names

Set `"language"` in `config.json` (`"en"`, `"ru"`). Texts live in `lang/`.
To add a language, copy `lang/en.json` to e.g. `lang/uk.json`, translate the
values (keep the `{placeholders}`), and set `"language": "uk"`.

Custom marker names: add to `config.json`
`"markers": {"1": "Question", "2": "Reflection", ...}`. Without it, the
selected language's defaults are used.

## EDA calibration

In `config.json`, `eda_calibration`:

- `{"type": "relative", "full_scale": 65520, "invert": true}` — default for the
  Grove GSR: 0–100 relative units, up = higher conductance;
- `{"type": "table", "points": [[raw, µS], ...]}` — after calibrating with resistors;
- `{"type": "linear", "gain": 0.001, "offset": 0}` — used automatically with the simulator.

## Files

| File | Purpose |
|---|---|
| `protocol.py` | packet format (the firmware implements the same) |
| `clocksync.py` | device ↔ computer clock alignment |
| `simulator.py` | device simulator (`python simulator.py --help`) |
| `receiver.py`, `receiver_core.py` | the app |
| `analyze.py` | session analysis |
| `i18n.py`, `lang/` | interface languages |
| `config.json` | settings |

A session is a folder with `fast.csv` (PPG 100 Hz), `slow.csv` (EDA and
breathing 20 Hz), `markers.csv`, `packets.csv` (for exact time recomputation)
and `meta.json`.
