"""Generate hardware/wiring.svg — module-level wiring of IDOmeter V1.0 + planned ADS1115."""
import sys

W, H = 1300, 870
out = []
A = out.append

COL = {"3V3": "#d32f2f", "5V": "#ef6c00", "GND": "#212121", "SDA": "#1565c0",
       "SCL": "#00897b", "SPI": "#6a1b9a", "SIG": "#c9a400"}

A(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
  'font-family="Helvetica, Arial, sans-serif" font-size="14">')
A('<rect width="100%" height="100%" fill="#ffffff"/>')
A('<text x="30" y="38" font-size="22" font-weight="bold" fill="#222">IDOmeter V1.0 — wiring</text>')
A('<text x="30" y="62" fill="#555">Module-level connections, traced from the PCB and firmware. '
  'Dashed: ADS1115 external ADC (planned).</text>')

# ---------------------------------------------------------------- T-Display
TX, TY, TW = 40, 110, 240
tpins = [("3V", "3V3"), ("G", "GND"), ("5V", "5V"), ("21", "SDA"), ("22", "SCL"),
         ("27", "SPI"), ("26", "SPI"), ("25", "SPI"), ("33", "SPI"), ("36", "SIG")]
tnames = {"3V": "3V3", "G": "GND", "5V": "5V (USB)", "21": "GPIO21  SDA", "22": "GPIO22  SCL",
          "27": "GPIO27  MISO", "26": "GPIO26  MOSI", "25": "GPIO25  SCK", "33": "GPIO33  CS",
          "36": "GPIO36  ADC1 (GSR)"}
step = 52
TH = step * len(tpins) + 40
A(f'<rect x="{TX}" y="{TY}" width="{TW}" height="{TH}" rx="8" fill="#eceff1" stroke="#455a64" stroke-width="2"/>')
A(f'<text x="{TX+12}" y="{TY+24}" font-weight="bold">LilyGO TTGO T-Display</text>')
A(f'<text x="{TX+12}" y="{TY+42}" fill="#555" font-size="12">ESP32 · 1.14″ screen · buttons 0/35</text>')
tp = {}
for i, (p, net) in enumerate(tpins):
    y = TY + 70 + i * step
    tp[p] = y
    A(f'<text x="{TX+TW-12}" y="{y+5}" text-anchor="end">{tnames[p]}</text>')
    A(f'<circle cx="{TX+TW}" cy="{y}" r="5" fill="{COL[net]}"/>')

# ---------------------------------------------------------------- rails
rails = {"3V3": 330, "GND": 360, "5V": 390, "SDA": 420, "SCL": 450}
rail_top, rail_bot = 120, 800
for net, x in rails.items():
    A(f'<line x1="{x}" y1="{rail_top}" x2="{x}" y2="{rail_bot}" stroke="{COL[net]}" stroke-width="3"/>')
    A(f'<text x="{x}" y="{rail_top-8}" text-anchor="middle" font-size="12" font-weight="bold" fill="{COL[net]}">{net}</text>')
dot = lambda x, y, c: A(f'<circle cx="{x}" cy="{y}" r="5" fill="{c}"/>')
for p, net in (("3V", "3V3"), ("G", "GND"), ("5V", "5V"), ("21", "SDA"), ("22", "SCL")):
    x = rails[net]
    A(f'<line x1="{TX+TW}" y1="{tp[p]}" x2="{x}" y2="{tp[p]}" stroke="{COL[net]}" stroke-width="3"/>')
    dot(x, tp[p], COL[net])

# ---------------------------------------------------------------- modules
MX, MW = 620, 300


def module(y, title, sub, pins, dashed=False, w=MW, x=MX):
    h = 50 + 34 * len(pins)
    dash = ' stroke-dasharray="8 6"' if dashed else ""
    fill = "#fafafa" if dashed else "#e3f2fd"
    A(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fill}" stroke="#37474f" stroke-width="2"{dash}/>')
    A(f'<text x="{x+12}" y="{y+22}" font-weight="bold">{title}</text>')
    A(f'<text x="{x+12}" y="{y+40}" fill="#555" font-size="12">{sub}</text>')
    pos = {}
    for i, (p, net) in enumerate(pins):
        py = y + 62 + i * 34
        pos[p] = py
        A(f'<text x="{x+14}" y="{py+5}">{p}</text>')
        if net:
            A(f'<circle cx="{x}" cy="{py}" r="5" fill="{COL[net]}"/>')
    return pos, h


def tap(net, py, dashed=False):
    x = rails[net]
    d = ' stroke-dasharray="8 6"' if dashed else ""
    A(f'<line x1="{x}" y1="{py}" x2="{MX}" y2="{py}" stroke="{COL[net]}" stroke-width="3"{d}/>')
    dot(x, py, COL[net])


# MAX30102
m, mh = module(100, "MAX30102 pulse sensor", "GY-MAX30102 · I²C address 0x57",
               [("VIN", "3V3"), ("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA"), ("INT  (not connected)", None)])
for p, net in (("VIN", "3V3"), ("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA")):
    tap(net, m[p])

# ADS1115 (planned)
a, ah = module(330, "ADS1115 16-bit ADC  (planned)", "I²C address 0x48 · VDD from 3V3 only",
               [("A0", None), ("VDD", "3V3"), ("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA"), ("ADDR → GND", None)],
               dashed=True)
for p, net in (("VDD", "3V3"), ("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA")):
    tap(net, a[p], dashed=True)

# Grove GSR, to the right
GX = 1000
g, gh = module(330, "Grove GSR v1.2", "white wire unused",
               [("SIG  (yellow)", "SIG"), ("VCC  (red)", "3V3"), ("GND  (black)", "GND"), ("Electrodes  (2-pin)", None)],
               w=250, x=GX)
# Grove power from rails: route above, along y = g pin, crossing the ADS box is avoided by going below
gy_vcc, gy_gnd, gy_sig = g["VCC  (red)"], g["GND  (black)"], g["SIG  (yellow)"]
for net, py in (("3V3", gy_vcc), ("GND", gy_gnd)):
    # rail -> down to a lane below ADS, then right, then up into Grove
    lane = 640 if net == "3V3" else 660
    vx = GX - 40 if net == "3V3" else GX - 20
    x = rails[net]
    A(f'<polyline points="{x},{lane} {vx},{lane} {vx},{py} {GX},{py}" fill="none" stroke="{COL[net]}" stroke-width="3"/>')
    dot(x, lane, COL[net])
# Grove SIG: today to GPIO36 (solid), planned to ADS A0 (dashed)
jx = GX - 60
A(f'<polyline points="{GX},{gy_sig} {jx},{gy_sig} {jx},620 {TX+TW+16},620 {TX+TW+16},{tp["36"]} {TX+TW},{tp["36"]}" '
  f'fill="none" stroke="{COL["SIG"]}" stroke-width="3"/>')
A(f'<line x1="{MX+MW}" y1="{a["A0"]}" x2="{jx}" y2="{gy_sig}" stroke="{COL["SIG"]}" stroke-width="3" stroke-dasharray="8 6"/>')
A(f'<circle cx="{MX+MW}" cy="{a["A0"]}" r="5" fill="{COL["SIG"]}"/>')
A(f'<circle cx="{jx}" cy="{gy_sig}" r="5" fill="{COL["SIG"]}"/>')

# SD module
s, sh = module(690, "microSD module HW-125", "SPI on HSPI pins · own 3.3 V regulator",
               [("VCC", "5V"), ("GND", "GND"), ("MISO / MOSI / SCK / CS", "SPI")])
for p, net in (("VCC", "5V"), ("GND", "GND")):
    tap(net, s[p])
# SPI bundle: from T-Display pins 27/26/25/33 to SD
sp = s["MISO / MOSI / SCK / CS"]
bx = 312
for p in ("27", "26", "25", "33"):
    A(f'<line x1="{TX+TW}" y1="{tp[p]}" x2="{bx}" y2="{tp[p]}" stroke="{COL["SPI"]}" stroke-width="3"/>')
A(f'<polyline points="{bx},{tp["27"]} {bx},{830} {560},{830} {560},{sp} {MX},{sp}" fill="none" stroke="{COL["SPI"]}" stroke-width="6"/>')
A(f'<text x="{bx+10}" y="{846}" fill="{COL["SPI"]}" font-size="12">4 SPI lines: 27→MISO, 26→MOSI, 25→SCK, 33→CS</text>')

# legend / notes
NX, NY = 1000, 710
notes = [
    "Notes",
    "• Grove GSR runs on 3.3 V: its output",
    "  stays within the ADC input range.",
    "• ADS1115: VDD on 3V3, never 5V.",
    "• A1–A3 free (A1: breathing, later).",
    "• GPIO36 can stay connected to SIG.",
    "• Lines crossing without a dot",
    "  are not connected.",
]
for i, t in enumerate(notes):
    A(f'<text x="{NX}" y="{NY+i*20}" {"font-weight=\"bold\"" if i == 0 else ""} fill="#333" font-size="13">{t}</text>')

A('</svg>')
open(sys.argv[1], "w", encoding="utf-8").write("\n".join(out))
