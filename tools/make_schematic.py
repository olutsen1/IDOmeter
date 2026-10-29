"""
Generate hardware/schematic.svg: schematic of the IDOmeter V1.0 board.

Connections use net labels (same name = same net), power symbols for
+3V3 / +5V / GND. The ADS1115 is drawn in grey as a planned part (DNP).

Requires schemdraw (pip install schemdraw).
Usage: python tools/make_schematic.py hardware/schematic.svg
"""
import sys

import schemdraw
import schemdraw.elements as elm

schemdraw.config(fontsize=12, unit=2.0)
GREY = "#8a8a8a"

STUB = 0.9


def net(d, anchor, name, side, color=None):
    """Short wire from a pin, ending in a net-label tag."""
    w = elm.Line().at(anchor).length(STUB)
    t = elm.Tag(width=max(1.5, 0.21 * len(name) + 0.5)).label(name, fontsize=11)
    if side == "L":
        w, t = w.left(), t.left()
    else:
        w, t = w.right(), t.right()
    if color:
        w, t = w.color(color), t.color(color)
    d += w
    d += t


PCOL = {"+3V3": "#c62828", "+5V": "#e65100", "GND": "#000000"}


def power(d, anchor, kind, side, color=None):
    """Power net as a coloured tag: '+3V3', '+5V' or 'GND'."""
    net(d, anchor, kind, side, color or PCOL[kind])


def nc(d, anchor):
    d += elm.NoConnect().at(anchor)


def ic(d, at, title, left, right, color=None, size=None):
    # schemdraw draws each side from the bottom up; callers list pins top-down
    pins = [elm.IcPin(name=n, side="L", anchorname=f"L_{n}") for n in reversed(left)]
    pins += [elm.IcPin(name=n, side="R", anchorname=f"R_{n}") for n in reversed(right)]
    u = elm.Ic(pins=pins, size=size, pinspacing=0.75, edgepadH=0.6).right().anchor("center").at(at).label(title, "top", fontsize=13)
    if color:
        u = u.color(color)
    d += u
    return u


with schemdraw.Drawing(show=False) as d:
    d.config(bgcolor="white")

    # ---------------------------------------------------------- U1 T-Display
    # Header pins listed from the end next to the USB connector.
    hdrA = ["3V", "G", "G ", "12", "13", "15", "2", "17", "22", "21", "G  ", "G   "]
    hdrB = ["5V", "G", "27", "26", "25", "33", "32", "39", "38", "37", "36", "3V"]
    u1 = ic(d, (0, 0), "U1  LilyGO TTGO T-Display (ESP32)", hdrA, hdrB, size=(5.2, 9.6))
    # header A (left)
    # header A carries only the I²C lines on this PCB; ground comes via header B
    for p in ("3V", "G", "G ", "12", "13", "15", "2", "17", "G  ", "G   "):
        nc(d, getattr(u1, f"L_{p}"))
    net(d, u1.L_22, "SCL", "L")
    net(d, u1.L_21, "SDA", "L")
    # header B (right)
    power(d, u1.R_5V, "+5V", "R")
    power(d, u1.R_G, "GND", "R")
    net(d, u1.R_27, "SD_MISO", "R")
    net(d, u1.R_26, "SD_MOSI", "R")
    net(d, u1.R_25, "SD_SCK", "R")
    net(d, u1.R_33, "SD_CS", "R")
    for p in ("32", "39", "38", "37"):
        nc(d, getattr(u1, f"R_{p}"))
    net(d, u1.R_36, "GSR_SIG", "R")
    power(d, u1.R_3V, "+3V3", "R")

    # ---------------------------------------------------------- U2 MAX30102
    u2 = ic(d, (-13.5, 4.2), "U2  MAX30102 module (GY-MAX30102)", [], ["VIN", "GND", "SCL", "SDA", "INT"],
            size=(3.6, 4.4))
    power(d, u2.R_VIN, "+3V3", "R")
    power(d, u2.R_GND, "GND", "R")
    net(d, u2.R_SCL, "SCL", "R")
    net(d, u2.R_SDA, "SDA", "R")
    nc(d, u2.R_INT)
    d += elm.Label().at((-13.5, 1.4)).label("on finger strap, 4-wire cable", fontsize=10, color="#555")

    # ---------------------------------------------------------- U4 microSD
    u4 = ic(d, (-13.5, -4.0), "U4  microSD module HW-125", [], ["VCC", "GND", "MISO", "MOSI", "SCK", "CS"],
            size=(3.6, 5.0))
    power(d, u4.R_VCC, "+5V", "R")
    power(d, u4.R_GND, "GND", "R")
    net(d, u4.R_MISO, "SD_MISO", "R")
    net(d, u4.R_MOSI, "SD_MOSI", "R")
    net(d, u4.R_SCK, "SD_SCK", "R")
    net(d, u4.R_CS, "SD_CS", "R")
    d += elm.Label().at((-13.5, -7.2)).label("own 3.3 V regulator", fontsize=10, color="#555")

    # ---------------------------------------------------------- U3 Grove GSR
    u3 = ic(d, (12.5, 4.4), "U3  Grove GSR v1.2", ["E1", "E2"],
            ["SIG (yellow)", "NC (white)", "VCC (red)", "GND (black)"], size=(4.4, 3.8))
    net(d, getattr(u3, "R_SIG (yellow)"), "GSR_SIG", "R")
    nc(d, getattr(u3, "R_NC (white)"))
    power(d, getattr(u3, "R_VCC (red)"), "+3V3", "R")
    power(d, getattr(u3, "R_GND (black)"), "GND", "R")
    for p in ("E1", "E2"):
        d += elm.Line().at(getattr(u3, f"L_{p}")).left().length(1.2)
        d += elm.Dot(open=True)
    d += elm.Label().at((8.6, 5.9)).label("finger electrodes", fontsize=10, color="#555")
    d += elm.Label().at((12.5, 1.9)).label("cable soldered to PCB", fontsize=10, color="#555")

    # ---------------------------------------------------------- U5 ADS1115 (planned)
    u5 = ic(d, (12.5, -4.6), "U5  ADS1115 module  (planned, DNP)", [],
            ["VDD", "GND", "SCL", "SDA", "ADDR", "ALRT", "A0", "A1", "A2", "A3"], color=GREY, size=(4.0, 7.6))
    power(d, u5.R_VDD, "+3V3", "R", GREY)
    power(d, u5.R_GND, "GND", "R", GREY)
    net(d, u5.R_SCL, "SCL", "R", GREY)
    net(d, u5.R_SDA, "SDA", "R", GREY)
    power(d, u5.R_ADDR, "GND", "R", GREY)
    nc(d, u5.R_ALRT)
    net(d, u5.R_A0, "GSR_SIG", "R", GREY)
    for p in ("A1", "A2", "A3"):
        nc(d, getattr(u5, f"R_{p}"))
    d += elm.Label().at((12.5, -9.2)).label("I²C 0x48 (ADDR = GND) · VDD 3.3 V only", fontsize=10, color=GREY)

    # ---------------------------------------------------------- notes
    notes = ("Notes\n"
             "• Same net label = connected.  ✕ = not connected.\n"
             "• U1 pins listed from the USB end of each header.\n"
             "• U1 on-board: USB-C (5 V), LiPo charger, buttons GPIO0 / GPIO35, ST7789 display.\n"
             "• I²C pull-ups are on the MAX30102 / ADS1115 modules, none on the PCB.\n"
             "• MAX30102 address 0x57, ADS1115 0x48: no conflict on the shared bus.")
    d += elm.Label().at((-16.5, -9.8)).label(notes, fontsize=11, halign="left", valign="top")
    d += elm.Label().at((0, 7.4)).label("IDOmeter V1.0 — schematic", fontsize=18)

    out = sys.argv[1] if len(sys.argv) > 1 else "schematic.svg"
    d.save(out, dpi=200) if out.endswith(".png") else d.save(out)
