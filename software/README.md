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
It also produces spontaneous responses unrelated to markers, as a real body does,
and about once every two minutes a finger movement (`--motion`, per minute).

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

Set `"language"` in `config.json` (`"en"`, `"uk"`). Texts live in `lang/`.
To add a language, copy `lang/en.json` to e.g. `lang/de.json`, translate the
values (keep the `{placeholders}`), and set `"language": "de"`.

Custom marker names: add to `config.json`
`"markers": {"1": "Question", "2": "Reflection", ...}`. Without it, the
selected language's defaults are used.

## Heart-rate artefacts

If the finger moves or the sensor is pressed, a beat can be misdetected and
the heart rate jumps for one point (e.g. 55 → 93 → 52). Such intervals are
rejected: each beat interval is compared with the mean of its two neighbours,
and the most deviating one is dropped if it differs by more than 30 %,
repeating until none does. Real changes, even large ones (a startle, ±20 bpm
swings during slow deep breathing), change gradually from beat to beat and are
kept; this was checked on synthetic cases. A rejected beat shows as a gap in the
heart-rate line, never as an invented value. The raw PPG is always recorded in
full, and the analysis prints how many intervals were rejected; above 10 % it
warns that the sensor contact was poor.

## Movement artefacts

A moving finger or a pressed sensor shows in the pulse wave as a short burst
far larger than usual (or as the pulse almost vanishing). Such stretches are
shaded grey on all plots and the status panel says "movement". Inside them the
heart rate and the PPG breathing curve are not shown, and they are left out of
the breathing rate and of the heart-rate response around markers. The skin
conductance is not affected. Raw data are recorded in full; the analysis
reports how many artefacts there were and how long, shades them in
`overview.png`, and adds `motion_s` (seconds of movement in each marker's
window) to `responses.csv`.

Detection: local pulse-wave range (1.5 s) above 3× or below 0.15× its median
over the surrounding minute, **or** a beat whose shape correlates below 0.75
with the typical shape of the surrounding beats (a gentle movement distorts
the pulse without making it larger); widened by 1.5 s before and 2.5 s after. The low
limit is kept small on purpose: after a deep breath the finger pulse normally
shrinks to ~40 % for several seconds, and that is physiology. Simulator check:
all movements detected; 5 minutes with 10 deep breaths and no movement gave no
false bands.

## Breathing without a sensor

If the device sends no breathing channel (the current board has none), the
Breathing plot shows breathing **estimated from the pulse wave** (labelled
"from PPG") and the status panel shows the breathing rate. Each breath shifts
the finger PPG baseline; `breathing.py` extracts that rhythm (8–36 breaths per
minute). It shows rhythm, pauses and deep breaths, not breathing volume, and
whether inhalation goes up or down depends on the person.

The breathing **rate** is counted breath by breath in three signals and
combined: the baseline shift, the heart rate (it rises on inhalation) and the
pulse amplitude. The last two are not disturbed by slow baseline drift. On
synthetic data the rate error is about 1 breath/min; breathing slower than
about 8/min is at the edge of the range and is overestimated. The rate is
shown as "—" when the three signals disagree.

`"breathing_source"` in `config.json`: `"auto"` (default: sensor if present,
otherwise PPG), `"sensor"` or `"ppg"`. The analysis reports the breathing rate
from PPG and, when a sensor is present, how well the two agree.

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
