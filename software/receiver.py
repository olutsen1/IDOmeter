"""
IDometer receiver: live plots, intervention markers, session recording.

Run:
    python receiver.py            — with the real device
    python receiver.py --sim      — with the simulator (markers are sent to the
                                    simulator so it can "respond bodily")

Keys (configured in config.json):
    1…9, Space, B    — place a marker of that type
    N                — start a note (marker time = moment N is pressed);
                       Enter — save, Esc — cancel
    R                — start / stop recording
    F                — freeze / resume plot scrolling
"""
import argparse
import random
import sys
import time

import numpy as np

import breathing as BR
import motion as MO
import protocol as P
from i18n import tr
from receiver_core import Engine, despike, eda_is_uS, eda_units, load_config

MARKER_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
                 "#42d4f4", "#f032e6", "#9a6324", "#808000", "#000075",
                 "#469990", "#800000"]


# ---------------------------------------------------------------- headless

def run_headless(engine, duration, auto_every):
    """Recording without a window; with auto_every>0 markers are placed automatically (testing)."""
    labels = [(k, v) for k, v in engine.cfg["markers"].items() if k in "123456789"]
    d = engine.start_recording()
    print(tr("hl_recording"), d)
    t_end = engine.now() + duration
    next_mark = engine.now() + 20 if auto_every else float("inf")
    rng = random.Random(1)
    while engine.now() < t_end:
        engine.poll()
        if engine.now() >= next_mark:
            k, v = rng.choice(labels[:3])
            engine.add_marker(k, v)
            next_mark = engine.now() + auto_every * rng.uniform(0.7, 1.3)
        time.sleep(0.02)
    engine.poll()
    print(tr("hl_stats"), engine.stats(), tr("hl_markers"), len(engine.markers))
    engine.stop_recording()
    return d


# ---------------------------------------------------------------- window

# ---- status panel (top of the side panel)
PANEL_CSS = """
QFrame#card { background: #f6f7f9; border: 1px solid #e2e5ea; border-radius: 8px; }
QLabel#caption { color: #6b7280; font-size: 9pt; }
QLabel#unit { color: #6b7280; font-size: 10pt; }
QLabel#small { color: #6b7280; font-size: 8.5pt; }
QLabel#conn { font-weight: 600; }
"""
DOT = {"ok": "#2e7d32", "bad": "#c62828", "idle": "#9aa0a6"}
CHIP = {"ok": ("#e6f4ea", "#1e6b30"), "bad": ("#fdecea", "#b3261e"),
        "warn": ("#fff4e5", "#a85b00"), "rec": ("#c62828", "#ffffff")}


def build_status_panel(vl, show_breath=True):
    """Connection card, heart-rate / breathing tiles and state chips.

    Returns a simple namespace with the widgets and setters used by refresh().
    """
    from types import SimpleNamespace

    from pyqtgraph.Qt import QtCore, QtWidgets
    ns = SimpleNamespace()

    def card():
        f = QtWidgets.QFrame()
        f.setObjectName("card")
        lay = QtWidgets.QVBoxLayout(f)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(2)
        return f, lay

    def label(text="", name=None):
        w = QtWidgets.QLabel(text)
        if name:
            w.setObjectName(name)
        return w

    # connection: coloured dot, state, device address; link quality and clock below
    c, cl = card()
    top = QtWidgets.QHBoxLayout()
    top.setSpacing(6)
    ns.dot = label("●")
    ns.conn = label("", "conn")
    ns.peer = label("", "small")
    top.addWidget(ns.dot)
    top.addWidget(ns.conn)
    top.addStretch(1)
    top.addWidget(ns.peer)
    cl.addLayout(top)
    ns.link = label("", "small")
    ns.sync = label("", "small")
    cl.addWidget(ns.link)
    cl.addWidget(ns.sync)
    vl.addWidget(c)

    # vital tiles; colours match the plot lines
    def tile(caption, unit, color):
        f, lay = card()
        lay.addWidget(label(caption, "caption"))
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        big = label("—", "big")
        big.setStyleSheet(f"color: {color}; font-size: 28pt; font-weight: 600;")
        row.addWidget(big)
        u = label(unit, "unit")
        u.setAlignment(QtCore.Qt.AlignmentFlag.AlignBottom)
        row.addWidget(u, 0, QtCore.Qt.AlignmentFlag.AlignBottom)
        row.addStretch(1)
        lay.addLayout(row)
        return f, big

    tiles = QtWidgets.QHBoxLayout()
    tiles.setSpacing(8)
    hr_tile, ns.hr = tile(tr("tile_hr"), tr("units_bpm"), "#d62728")
    ns.br_tile, ns.br = tile(tr("tile_breath"), tr("units_per_min"), "#2ca02c")
    ns.br_tile.setVisible(show_breath)
    tiles.addWidget(hr_tile, 1)
    tiles.addWidget(ns.br_tile, 1)
    vl.addLayout(tiles)

    # state chips: finger, movement, recording
    chips = QtWidgets.QHBoxLayout()
    chips.setSpacing(6)
    ns.finger, ns.motion, ns.rec = label("", "chip"), label("", "chip"), label("", "chip")
    for w in (ns.finger, ns.motion, ns.rec):
        chips.addWidget(w)
    chips.addStretch(1)
    ns.motion.setVisible(False)
    ns.rec.setVisible(False)
    vl.addLayout(chips)
    vl.addSpacing(4)

    def set_conn(kind, text, peer):
        ns.dot.setStyleSheet(f"color: {DOT[kind]}; font-size: 14pt;")
        ns.conn.setText(text)
        ns.peer.setText(peer)

    def set_chip(w, text, kind, visible=True):
        bg, fg = CHIP[kind]
        w.setText(text)
        w.setStyleSheet(f"background: {bg}; color: {fg}; border-radius: 10px; padding: 2px 9px; font-size: 9pt;")
        w.setVisible(visible)

    ns.set_conn, ns.set_chip = set_conn, set_chip
    set_conn("idle", "", "")
    return ns


def run_gui(engine):
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
    from scipy.signal import butter, sosfiltfilt

    sos_disp = butter(2, [0.5, 5.0], btype="band", fs=P.BASE_RATE_HZ, output="sos")
    sos_eda = butter(2, 1.5, fs=P.SLOW_RATE_HZ, output="sos")   # display only
    eda_min_span = 0.5 if eda_is_uS(engine.cfg["eda_calibration"]) else 2.0
    cfg = engine.cfg
    key_color = {k: MARKER_COLORS[i % len(MARKER_COLORS)]
                 for i, k in enumerate(cfg["markers"].keys())}
    key_color["N"] = "#555555"

    pg.setConfigOptions(antialias=True, background="w", foreground="#333")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    win = QtWidgets.QMainWindow()
    win.setWindowTitle("IDometer")
    win.resize(1400, 850)
    central = QtWidgets.QWidget()
    win.setCentralWidget(central)
    hl = QtWidgets.QHBoxLayout(central)

    # ---- plots
    glw = pg.GraphicsLayoutWidget()
    hl.addWidget(glw, 1)

    def mkplot(row, title, units, height):
        p = glw.addPlot(row=row, col=0)
        p.setLabel("left", title, units=units)
        p.getAxis("left").enableAutoSIPrefix(False)
        p.showGrid(x=True, y=True, alpha=0.2)
        p.setMouseEnabled(x=False, y=False)
        p.setMenuEnabled(False)
        p.hideButtons()
        glw.ci.layout.setRowStretchFactor(row, height)
        return p

    p_eda = mkplot(0, tr("plot_eda"), eda_units(cfg["eda_calibration"]), 4)
    p_hr = mkplot(1, tr("plot_hr"), tr("units_bpm"), 3)
    p_ppg = mkplot(2, "PPG", "", 2)
    p_resp = mkplot(3, tr("plot_resp"), "", 2)
    plots = [p_eda, p_hr, p_ppg, p_resp]
    for p in plots[1:]:
        p.setXLink(p_eda)
    for p in plots[:-1]:
        p.getAxis("bottom").setStyle(showValues=False)
    p_resp.setLabel("bottom", tr("plot_time"), units=tr("units_s"))
    p_ppg.getAxis("left").setStyle(showValues=False)
    p_resp.getAxis("left").setStyle(showValues=False)

    c_eda = p_eda.plot(pen=pg.mkPen("#1f77b4", width=2), connect="finite")
    c_hr = p_hr.plot(pen=pg.mkPen("#d62728", width=2), connect="finite", symbol="o", symbolSize=4,
                     symbolBrush="#d62728", symbolPen=None)
    c_ppg = p_ppg.plot(pen=pg.mkPen("#888", width=1), connect="finite")
    c_resp = p_resp.plot(pen=pg.mkPen("#2ca02c", width=2), connect="finite")

    # ---- side panel
    side = QtWidgets.QWidget()
    side.setFixedWidth(320)
    hl.addWidget(side)
    vl = QtWidgets.QVBoxLayout(side)

    side.setStyleSheet(PANEL_CSS)
    st = build_status_panel(vl, show_breath=True)

    rec_btn = QtWidgets.QPushButton(tr("btn_rec_start"))
    rec_btn.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    rec_btn.setMinimumHeight(40)
    vl.addWidget(rec_btn)
    rec_info = QtWidgets.QLabel("")
    rec_info.setWordWrap(True)
    vl.addWidget(rec_info)

    row = QtWidgets.QHBoxLayout()
    row.addWidget(QtWidgets.QLabel(tr("window_s")))
    win_spin = QtWidgets.QSpinBox()
    win_spin.setRange(10, 170)
    win_spin.setValue(int(cfg.get("window_s", 30)))
    win_spin.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
    row.addWidget(win_spin)
    freeze = QtWidgets.QCheckBox(tr("freeze"))
    freeze.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    row.addWidget(freeze)
    vl.addLayout(row)

    vl.addWidget(QtWidgets.QLabel(tr("markers_title")))
    grid = QtWidgets.QGridLayout()
    vl.addLayout(grid)

    note = QtWidgets.QLineEdit()
    # click-only focus: otherwise the field grabs focus at start and digit keys type text instead of markers
    note.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
    note.setPlaceholderText(tr("note_placeholder"))
    vl.addWidget(note)

    vl.addWidget(QtWidgets.QLabel(tr("log_title")))
    log = QtWidgets.QListWidget()
    log.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    vl.addWidget(log, 1)

    # ---- markers
    drawn = []

    def fmt_t(t):
        base = engine.recorder.meta["record_start_t"] if engine.recorder else 0.0
        s = max(0.0, t - base)
        return f"{int(s // 60):02d}:{s % 60:04.1f}"

    def draw_marker(m):
        color = key_color.get(m["key"], "#555")
        items = []
        for p in plots:
            ln = pg.InfiniteLine(pos=m["t"], angle=90,
                                 pen=pg.mkPen(color, width=1.5,
                                              style=QtCore.Qt.PenStyle.DashLine))
            p.addItem(ln, ignoreBounds=True)
            items.append((p, ln))
        txt = pg.TextItem(m["label"] if not m.get("note") else m["note"][:30],
                          color=color, anchor=(0, 0))
        p_eda.addItem(txt, ignoreBounds=True)   # labels must not affect autoscaling
        items.append((p_eda, txt))
        drawn.append((m, txt))
        text = f"{fmt_t(m['t'])}  {m['label']}"
        if m.get("note"):
            text += f": {m['note']}"
        it = QtWidgets.QListWidgetItem(text)
        it.setForeground(QtGui.QColor(color))
        log.insertItem(0, it)

    def add_marker(key, label):
        draw_marker(engine.add_marker(key, label))

    def key_name(k):
        return tr("key_space") if k == "Space" else k

    shortcuts = []
    for i, (k, label) in enumerate(cfg["markers"].items()):
        b = QtWidgets.QPushButton(f"{key_name(k)}  {label}")
        b.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        b.setStyleSheet(f"text-align:left; padding:4px; color:{key_color[k]};")
        b.clicked.connect(lambda _=False, k=k, l=label: add_marker(k, l))
        grid.addWidget(b, i // 2, i % 2)
        sc = QtGui.QShortcut(QtGui.QKeySequence(k), win)
        sc.activated.connect(lambda k=k, l=label: add_marker(k, l))
        shortcuts.append(sc)

    note_t = [None]

    def begin_note():
        note_t[0] = engine.now()
        note.setFocus()

    def commit_note():
        text = note.text().strip()
        t = note_t[0] if note_t[0] is not None else engine.now()
        if text:
            draw_marker(engine.add_marker("N", tr("marker_note"), text, t=t))
        note.clear()
        note_t[0] = None
        note.clearFocus()

    note.returnPressed.connect(commit_note)
    note.textEdited.connect(lambda _: note_t.__setitem__(0, note_t[0] or engine.now()))
    sc = QtGui.QShortcut(QtGui.QKeySequence("N"), win)
    sc.activated.connect(begin_note)
    shortcuts.append(sc)
    esc = QtGui.QShortcut(QtGui.QKeySequence("Esc"), note)
    esc.setContext(QtCore.Qt.ShortcutContext.WidgetShortcut)
    esc.activated.connect(lambda: (note.clear(), note.clearFocus(), note_t.__setitem__(0, None)))
    shortcuts.append(esc)

    # ---- recording
    def toggle_rec():
        if engine.recorder:
            d = engine.stop_recording()
            rec_btn.setText(tr("btn_rec_start"))
            rec_btn.setStyleSheet("")
            rec_info.setText(tr("rec_saved", path=d))
            sep = QtWidgets.QListWidgetItem(tr("log_rec_stopped"))
            sep.setForeground(QtGui.QColor("#999"))
            log.insertItem(0, sep)
        else:
            d = engine.start_recording()
            rec_btn.setText(tr("btn_rec_stop"))
            sep = QtWidgets.QListWidgetItem(tr("log_rec_started"))
            sep.setForeground(QtGui.QColor("#c62828"))
            log.insertItem(0, sep)
            rec_btn.setStyleSheet("background:#c62828; color:white; font-weight:bold;")
            rec_info.setText(tr("rec_folder", path=d))

    rec_btn.clicked.connect(toggle_rec)
    # when frozen, the plots can be panned with the mouse
    freeze.toggled.connect(lambda on: [p.setMouseEnabled(x=on, y=False) for p in plots])
    for k, fn in (("R", toggle_rec), ("F", lambda: freeze.toggle())):
        sc = QtGui.QShortcut(QtGui.QKeySequence(k), win)
        sc.activated.connect(fn)
        shortcuts.append(sc)

    # ---- motion artefacts: grey bands on all plots
    motion_items = []

    def draw_motion():
        key = tuple((round(a, 1), round(b, 1)) for a, b in state.get("motion", []))
        if key == state.get("motion_drawn"):
            return                                # unchanged: keep the existing bands
        state["motion_drawn"] = key
        for p, it in motion_items:
            p.removeItem(it)
        motion_items.clear()
        for s0, s1 in state.get("motion", []):
            for p in plots:
                it = pg.LinearRegionItem(values=(s0, s1), movable=False,
                                         brush=pg.mkBrush(120, 120, 120, 50),
                                         pen=pg.mkPen(None))
                it.setZValue(-10)
                p.addItem(it, ignoreBounds=True)
                motion_items.append((p, it))

    # ---- refresh
    state = {"pk": 0, "pk_t": time.perf_counter(), "rate": 0.0, "st": 0.0}
    DISPLAY_DELAY = 0.25

    def refresh():
        engine.poll()
        now = engine.now()
        W = win_spin.value()
        x1 = now - DISPLAY_DELAY
        x0 = x1 - W

        s = engine.slow.tail()
        if len(s):
            sel = s[:, 0] > x0 - 2
            te, ye = s[sel, 0], s[sel, 1].copy()
            ok = np.isfinite(ye)
            if ok.sum() > 20:
                yi = np.interp(np.arange(len(ye)), np.where(ok)[0], ye[ok])
                ye = sosfiltfilt(sos_eda, yi)
                ye[~ok] = np.nan
            c_eda.setData(te, ye)
            state["eda_y"] = ye
            resp = s[sel, 2]
            src = cfg.get("breathing_source", "auto")
            has_sensor = bool(np.nanmax(np.abs(resp)) > 0) if np.isfinite(resp).any() else False
            use_ppg = src == "ppg" or (src == "auto" and not has_sensor)
            if not use_ppg:
                c_resp.setData(s[sel, 0], resp)
            if use_ppg != state.get("resp_ppg"):
                state["resp_ppg"] = use_ppg
                p_resp.setLabel("left", tr("plot_resp_ppg") if use_ppg else tr("plot_resp"))
                if use_ppg:
                    c_resp.setData([], [])
        f = engine.fast.tail()
        if len(f):
            f = f[f[:, 0] > now - 95].copy()
            f[:, 1] = despike(f[:, 1])
        if len(f) > 50:
            sel = f[:, 0] > x0 - 2
            t, y = f[sel, 0], f[sel, 1]
            ok = np.isfinite(y) & np.isfinite(t)
            if ok.sum() > 30:
                yy = np.interp(np.arange(len(y)), np.where(ok)[0], y[ok])
                yy = sosfiltfilt(sos_disp, yy - yy.mean())
                yy[~ok] = np.nan
                c_ppg.setData(t, yy)
                state["ppg_y"] = (t, yy)
        # motion artefacts and PPG breathing: recomputed twice a second over the last 90 s
        if len(f) > 50 and time.perf_counter() - state.get("br_calc", 0) > 0.5:
            state["br_calc"] = time.perf_counter()
            sel = f[:, 0] > now - 90
            sign = -1 if cfg.get("invert_ppg", True) else 1
            raw_ir = sign * f[sel, 1]
            new = MO.detect(f[sel, 0], raw_ir, beats=engine.beats.beats)
            t_from = f[sel, 0][0] if sel.any() else now
            old = [sg for sg in state.get("motion", []) if sg[1] < t_from]
            state["motion"] = (old + new)[-200:]
            if state.get("resp_ppg"):
                tb, mods = BR.modulations(f[sel, 0], raw_ir, engine.beats.beats)
                if len(tb):
                    y = mods["riiv"].copy()
                    y[MO.mask(tb, new)] = np.nan          # a movement is not a breath
                    c_resp.setData(tb, y)
                    clean = MO.longest_clean(tb, new, tb[-1] - 60)
                    state["br_rate"] = (BR.rate_fused({k: v[clean] for k, v in mods.items()})
                                        if clean.sum() >= 300 else (float("nan"), 0.0))
            draw_motion()
        if engine.beats.hr:
            hr = np.array(engine.beats.hr)
            sel = hr[:, 0] > x0 - 5
            hv = hr[sel, 1].copy()
            hv[MO.mask(hr[sel, 0], state.get("motion", []))] = np.nan
            c_hr.setData(hr[sel, 0], hv)

        if not freeze.isChecked():
            p_eda.setXRange(x0, x1, padding=0)
            for p in (p_hr, p_resp):
                p.enableAutoRange(axis="y")
            # PPG scale from the clean signal only, so a movement burst does not squash the pulses
            if "ppg_y" in state:
                tp, yp = state["ppg_y"]
                vis = (tp > x0) & np.isfinite(yp) & ~MO.mask(tp, state.get("motion", []))
                if vis.sum() > 50:
                    lo, hi = np.percentile(yp[vis], (1, 99))
                    p_ppg.setYRange(lo - 0.15 * (hi - lo), hi + 0.15 * (hi - lo), padding=0)
            # EDA scale no narrower than eda_min_span, else ADC noise fills the whole plot
            ys = state.get("eda_y")
            if ys is not None and len(ys) and np.isfinite(ys).any():
                lo, hi = np.nanmin(ys), np.nanmax(ys)
                pad = max(eda_min_span - (hi - lo), 0) / 2 + 0.05 * (hi - lo)
                p_eda.setYRange(lo - pad, hi + pad, padding=0)

        # marker labels at the top edge of the EDA plot
        (_, _), (ylo, yhi) = p_eda.viewRange()
        # staggered in 3 rows so that nearby markers do not overlap
        for i, (m, txt) in enumerate(drawn):
            if x0 - 5 < m["t"] < x1 + 1:
                txt.setPos(m["t"], yhi - (i % 3) * 0.09 * (yhi - ylo))

        # status every half second
        if time.perf_counter() - state["st"] > 0.5:
            dtp = time.perf_counter() - state["pk_t"]
            state["rate"] = (engine.packets - state["pk"]) / dtp
            state["pk"], state["pk_t"] = engine.packets, time.perf_counter()
            state["st"] = time.perf_counter()
            if engine.connected():
                st.set_conn("ok", tr("st_connected"), engine.peer[0])
            elif engine.packets:
                st.set_conn("bad", tr("st_lost"), "")
            else:
                st.set_conn("idle", tr("st_waiting", port=cfg["listen_port"]), "")
            st.link.setText(tr("st_rate", rate=state["rate"], lost=engine.lost))
            if not engine.sync.ready:
                st.sync.setText("")
            elif len(engine.sync.buckets) < 30:     # under a minute: the slope is not reliable yet
                st.sync.setText(tr("st_sync_pending", lat=engine.sync.latency_ms()))
            else:
                st.sync.setText(tr("st_sync", drift=engine.sync.drift_ppm, lat=engine.sync.latency_ms()))
            good = [v for _, v in engine.beats.hr[-3:] if np.isfinite(v)]
            st.hr.setText(f"{good[-1]:.0f}" if good else "—")
            br, q = state.get("br_rate", (float("nan"), 0.0))
            st.br.setText(f"{br:.0f}" if np.isfinite(br) and q >= 0.6 else "—")
            st.br_tile.setVisible(bool(state.get("resp_ppg")))
            finger_on = bool(engine.last_flags & P.FLAG_FINGER)
            st.set_chip(st.finger, tr("st_finger") if finger_on else tr("st_no_finger"),
                        "ok" if finger_on else "bad")
            moving = any(s1 > now - 3 for _, s1 in state.get("motion", []))
            st.set_chip(st.motion, tr("st_motion"), "warn", visible=moving)
            if engine.recorder:
                el = now - engine.recorder.meta["record_start_t"]
                st.set_chip(st.rec, tr("st_recording", mm=int(el // 60), ss=int(el % 60)), "rec")
            else:
                st.rec.setVisible(False)

    def safe_refresh():
        # an error in one frame must not close the program
        try:
            refresh()
        except Exception:
            import traceback
            if not state.get("err_shown"):
                traceback.print_exc()
                state["err_shown"] = True

    timer = QtCore.QTimer()
    timer.timeout.connect(safe_refresh)
    timer.start(33)

    def on_quit():
        if engine.recorder:
            engine.stop_recording()
    app.aboutToQuit.connect(on_quit)

    win.show()
    app.exec()


def main():
    ap = argparse.ArgumentParser(description="IDometer receiver")
    ap.add_argument("--config", default="config.json")
    ap.add_argument("--sim", action="store_true",
                    help="send markers to the simulator so it can 'respond'")
    ap.add_argument("--sim-host", default="127.0.0.1")
    ap.add_argument("--headless", action="store_true", help="no window (for testing)")
    ap.add_argument("--duration", type=float, default=120)
    ap.add_argument("--auto-marks", type=float, default=0,
                    help="headless: place a marker roughly every N seconds")
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.sim:      # the simulator sends nanosiemens
        cfg["eda_calibration"] = {"type": "linear", "gain": 0.001, "offset": 0.0}
    fb = (args.sim_host, P.SIM_FEEDBACK_PORT) if args.sim else None
    engine = Engine(cfg, sim_feedback=fb)
    if args.headless:
        run_headless(engine, args.duration, args.auto_marks)
    else:
        run_gui(engine)


if __name__ == "__main__":
    main()
