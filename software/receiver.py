"""
IDometer — приёмник: живые графики, метки интервенций, запись сессии.

Запуск:
    python receiver.py            — работа с настоящим устройством
    python receiver.py --sim      — с симулятором (метки уходят симулятору,
                                    и он «отвечает телом»)

Клавиши (настраиваются в config.json):
    1…9, Пробел, B   — поставить метку соответствующего типа
    N                — начать заметку (время метки = момент нажатия N);
                       Enter — сохранить, Esc — отменить
    R                — старт / стоп записи
    F                — заморозить / продолжить прокрутку графиков
"""
import argparse
import random
import sys
import time

import numpy as np

import protocol as P
from receiver_core import Engine, eda_units, load_config

MARKER_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
                 "#42d4f4", "#f032e6", "#9a6324", "#808000", "#000075",
                 "#469990", "#800000"]


# ---------------------------------------------------------------- без окна

def run_headless(engine, duration, auto_every):
    """Запись без окна; при auto_every>0 метки ставятся автоматически (для проверки)."""
    labels = [(k, v) for k, v in engine.cfg["markers"].items() if k in "123456789"]
    d = engine.start_recording()
    print("Запись:", d)
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
    print("Статистика:", engine.stats(), "меток:", len(engine.markers))
    engine.stop_recording()
    return d


# ---------------------------------------------------------------- окно

def run_gui(engine):
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
    from scipy.signal import butter, sosfiltfilt

    sos_disp = butter(2, [0.5, 5.0], btype="band", fs=P.BASE_RATE_HZ, output="sos")
    sos_eda = butter(2, 1.5, fs=P.SLOW_RATE_HZ, output="sos")   # только для показа
    eda_min_span = 0.5 if eda_units(engine.cfg["eda_calibration"]) == "мкСм" else 2.0
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

    # ---- графики
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

    p_eda = mkplot(0, "Кожная проводимость", eda_units(cfg["eda_calibration"]), 4)
    p_hr = mkplot(1, "Пульс", "уд/мин", 3)
    p_ppg = mkplot(2, "PPG", "", 2)
    p_resp = mkplot(3, "Дыхание", "", 2)
    plots = [p_eda, p_hr, p_ppg, p_resp]
    for p in plots[1:]:
        p.setXLink(p_eda)
    for p in plots[:-1]:
        p.getAxis("bottom").setStyle(showValues=False)
    p_resp.setLabel("bottom", "время", units="с")
    p_ppg.getAxis("left").setStyle(showValues=False)
    p_resp.getAxis("left").setStyle(showValues=False)

    c_eda = p_eda.plot(pen=pg.mkPen("#1f77b4", width=2), connect="finite")
    c_hr = p_hr.plot(pen=pg.mkPen("#d62728", width=2), symbol="o", symbolSize=4,
                     symbolBrush="#d62728", symbolPen=None)
    c_ppg = p_ppg.plot(pen=pg.mkPen("#888", width=1), connect="finite")
    c_resp = p_resp.plot(pen=pg.mkPen("#2ca02c", width=2), connect="finite")

    # ---- правая панель
    side = QtWidgets.QWidget()
    side.setFixedWidth(320)
    hl.addWidget(side)
    vl = QtWidgets.QVBoxLayout(side)

    status = QtWidgets.QLabel()
    status.setWordWrap(True)
    status.setTextFormat(QtCore.Qt.TextFormat.RichText)
    vl.addWidget(status)

    rec_btn = QtWidgets.QPushButton("● Начать запись  (R)")
    rec_btn.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    rec_btn.setMinimumHeight(40)
    vl.addWidget(rec_btn)
    rec_info = QtWidgets.QLabel("")
    rec_info.setWordWrap(True)
    vl.addWidget(rec_info)

    row = QtWidgets.QHBoxLayout()
    row.addWidget(QtWidgets.QLabel("Окно, с:"))
    win_spin = QtWidgets.QSpinBox()
    win_spin.setRange(10, 170)
    win_spin.setValue(int(cfg.get("window_s", 30)))
    win_spin.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
    row.addWidget(win_spin)
    freeze = QtWidgets.QCheckBox("Заморозить (F)")
    freeze.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    row.addWidget(freeze)
    vl.addLayout(row)

    vl.addWidget(QtWidgets.QLabel("<b>Метки</b>"))
    grid = QtWidgets.QGridLayout()
    vl.addLayout(grid)

    note = QtWidgets.QLineEdit()
    # только по клику: иначе поле заберёт фокус при старте и цифры уйдут в текст, а не в метки
    note.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
    note.setPlaceholderText("N — заметка, Enter — сохранить")
    vl.addWidget(note)

    vl.addWidget(QtWidgets.QLabel("<b>Журнал меток</b>"))
    log = QtWidgets.QListWidget()
    log.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
    vl.addWidget(log, 1)

    # ---- метки
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
        p_eda.addItem(txt, ignoreBounds=True)   # подписи не должны влиять на масштаб
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
        return "Пробел" if k == "Space" else k

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
            draw_marker(engine.add_marker("N", "Заметка", text, t=t))
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

    # ---- запись
    def toggle_rec():
        if engine.recorder:
            d = engine.stop_recording()
            rec_btn.setText("● Начать запись  (R)")
            rec_btn.setStyleSheet("")
            rec_info.setText(f"Сохранено: {d}")
            sep = QtWidgets.QListWidgetItem("——— запись остановлена ———")
            sep.setForeground(QtGui.QColor("#999"))
            log.insertItem(0, sep)
        else:
            d = engine.start_recording()
            rec_btn.setText("■ Остановить запись  (R)")
            sep = QtWidgets.QListWidgetItem("——— запись начата, время от 00:00 ———")
            sep.setForeground(QtGui.QColor("#c62828"))
            log.insertItem(0, sep)
            rec_btn.setStyleSheet("background:#c62828; color:white; font-weight:bold;")
            rec_info.setText(f"Папка: {d}")

    rec_btn.clicked.connect(toggle_rec)
    # в замороженном режиме можно листать графики мышью
    freeze.toggled.connect(lambda on: [p.setMouseEnabled(x=on, y=False) for p in plots])
    for k, fn in (("R", toggle_rec), ("F", lambda: freeze.toggle())):
        sc = QtGui.QShortcut(QtGui.QKeySequence(k), win)
        sc.activated.connect(fn)
        shortcuts.append(sc)

    # ---- обновление
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
            c_resp.setData(s[sel, 0], s[sel, 2])
        f = engine.fast.tail()
        if len(f) > 50:
            sel = f[:, 0] > x0 - 2
            t, y = f[sel, 0], f[sel, 1]
            ok = np.isfinite(y) & np.isfinite(t)
            if ok.sum() > 30:
                yy = np.interp(np.arange(len(y)), np.where(ok)[0], y[ok])
                yy = sosfiltfilt(sos_disp, yy - yy.mean())
                yy[~ok] = np.nan
                c_ppg.setData(t, yy)
        if engine.beats.hr:
            hr = np.array(engine.beats.hr)
            sel = hr[:, 0] > x0 - 5
            c_hr.setData(hr[sel, 0], hr[sel, 1])

        if not freeze.isChecked():
            p_eda.setXRange(x0, x1, padding=0)
            for p in (p_hr, p_ppg, p_resp):
                p.enableAutoRange(axis="y")
            # шкала кожи не уже eda_min_span: иначе шум АЦП растягивается на весь экран
            ys = state.get("eda_y")
            if ys is not None and len(ys) and np.isfinite(ys).any():
                lo, hi = np.nanmin(ys), np.nanmax(ys)
                pad = max(eda_min_span - (hi - lo), 0) / 2 + 0.05 * (hi - lo)
                p_eda.setYRange(lo - pad, hi + pad, padding=0)

        # подписи меток — у верхнего края графика EDA
        (_, _), (ylo, yhi) = p_eda.viewRange()
        # лесенкой в 3 ряда, чтобы близкие метки не налезали друг на друга
        for i, (m, txt) in enumerate(drawn):
            if x0 - 5 < m["t"] < x1 + 1:
                txt.setPos(m["t"], yhi - (i % 3) * 0.09 * (yhi - ylo))

        # статус раз в полсекунды
        if time.perf_counter() - state["st"] > 0.5:
            dtp = time.perf_counter() - state["pk_t"]
            state["rate"] = (engine.packets - state["pk"]) / dtp
            state["pk"], state["pk_t"] = engine.packets, time.perf_counter()
            state["st"] = time.perf_counter()
            if engine.connected():
                conn = f"<span style='color:#2e7d32'>● связь есть</span> ({engine.peer[0]})"
            elif engine.packets:
                conn = "<span style='color:#c62828'>● связь потеряна</span>"
            else:
                conn = f"<span style='color:#888'>● жду данные на порту {cfg['listen_port']}</span>"
            finger = ("палец на датчике" if engine.last_flags & P.FLAG_FINGER
                      else "<span style='color:#c62828'>нет пальца на датчике</span>")
            hr_now = f"{engine.beats.hr[-1][1]:.0f}" if engine.beats.hr else "—"
            if not engine.sync.ready:
                sync = "—"
            elif len(engine.sync.buckets) < 30:     # меньше минуты — наклон ещё неточен
                sync = f"часы сводятся…, задержка {engine.sync.latency_ms():.0f} мс"
            else:
                sync = (f"дрейф часов {engine.sync.drift_ppm:+.0f} ppm, "
                        f"задержка {engine.sync.latency_ms():.0f} мс")
            rec = ""
            if engine.recorder:
                el = now - engine.recorder.meta["record_start_t"]
                rec = f"<br><b style='color:#c62828'>ЗАПИСЬ {int(el // 60):02d}:{int(el % 60):02d}</b>"
            status.setText(
                f"{conn}<br>{state['rate']:.1f} пакетов/с, потеряно {engine.lost}"
                f"<br>{finger}<br>пульс {hr_now} уд/мин<br>{sync}{rec}")

    def safe_refresh():
        # ошибка в одном кадре не должна закрывать программу
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
    ap = argparse.ArgumentParser(description="Приёмник IDometer")
    ap.add_argument("--config", default="config.json")
    ap.add_argument("--sim", action="store_true",
                    help="отправлять метки симулятору, чтобы он «отвечал»")
    ap.add_argument("--sim-host", default="127.0.0.1")
    ap.add_argument("--headless", action="store_true", help="без окна (для проверки)")
    ap.add_argument("--duration", type=float, default=120)
    ap.add_argument("--auto-marks", type=float, default=0,
                    help="в режиме без окна: ставить метку примерно раз в N секунд")
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.sim:      # симулятор шлёт наносименсы
        cfg["eda_calibration"] = {"type": "linear", "gain": 0.001, "offset": 0.0}
    fb = (args.sim_host, P.SIM_FEEDBACK_PORT) if args.sim else None
    engine = Engine(cfg, sim_feedback=fb)
    if args.headless:
        run_headless(engine, args.duration, args.auto_marks)
    else:
        run_gui(engine)


if __name__ == "__main__":
    main()
