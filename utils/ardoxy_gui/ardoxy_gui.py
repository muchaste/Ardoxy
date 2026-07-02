"""
ardoxy_gui.py  —  Ardoxy-OS companion GUI
Requires: pyserial, matplotlib  (see requirements.txt)
Run with:  python ardoxy_gui.py
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import queue
import time
import csv
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ── global state ───────────────────────────────────────────────────────────────
ser = None
serial_thread = None
msg_queue     = queue.Queue()
_dl_msg_queue = queue.Queue()   # dedicated queue for file listing / download
_dl_routing   = False           # when True, route file-transfer lines to _dl_msg_queue
data_rows = []          # list of dicts
running = False
paused = False
connected = False

# ── serial I/O ────────────────────────────────────────────────────────────────

def serial_reader():
    """Background thread: read lines from serial, push to msg_queue."""
    global connected
    while connected and ser and ser.is_open:
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                msg_queue.put(line)
        except Exception:
            break
    msg_queue.put("__DISCONNECTED__")


def send(line: str):
    """Send a single newline-terminated command."""
    if ser and ser.is_open:
        ser.write((line + "\n").encode("ascii"))


def send_and_ack(line: str, timeout: float = 2.0) -> bool:
    """Send a command and block until ACK:OK or ACK:ERR is received."""
    # Drain any stale messages before sending so we don't consume an old ACK.
    while not msg_queue.empty():
        try:
            msg_queue.get_nowait()
        except queue.Empty:
            break
    send(line)
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            resp = msg_queue.get(timeout=0.1)
            handle_line(resp)
            if resp.startswith("ACK:OK"):
                return True
            if resp.startswith("ACK:ERR"):
                return False
        except queue.Empty:
            pass
    return False


# ── GUI helpers ───────────────────────────────────────────────────────────────

def lbl(parent, text, **kw):
    return ttk.Label(parent, text=text, **kw)


def entry(parent, textvariable, width=10):
    return ttk.Entry(parent, textvariable=textvariable, width=width)


# ── tab 1: Connect ────────────────────────────────────────────────────────────

def build_connect_tab(nb):
    frame = ttk.Frame(nb, padding=16)
    nb.add(frame, text="Connect")

    lbl(frame, "Serial Port:").grid(row=0, column=0, sticky="w", padx=4, pady=6)
    port_var = tk.StringVar()
    port_cb = ttk.Combobox(frame, textvariable=port_var, width=18, state="readonly")
    port_cb.grid(row=0, column=1, sticky="w", padx=4)

    def refresh_ports():
        ports = [p.device for p in serial.tools.list_ports.comports()]
        port_cb["values"] = ports
        if ports:
            port_var.set(ports[0])

    ttk.Button(frame, text="Refresh", command=refresh_ports).grid(
        row=0, column=2, padx=4)

    lbl(frame, "Baud rate: 19200 (fixed)").grid(
        row=1, column=0, columnspan=3, sticky="w", padx=4, pady=2)

    status_var = tk.StringVar(value="Disconnected")
    lbl(frame, "Status:").grid(row=2, column=0, sticky="w", padx=4, pady=6)
    status_lbl = ttk.Label(frame, textvariable=status_var, foreground="red")
    status_lbl.grid(row=2, column=1, sticky="w", padx=4)

    arduino_state_var = tk.StringVar(value="—")
    lbl(frame, "Arduino state:").grid(row=3, column=0, sticky="w", padx=4)
    ttk.Label(frame, textvariable=arduino_state_var).grid(
        row=3, column=1, sticky="w", padx=4)

    conn_btn = ttk.Button(frame, text="Connect")
    conn_btn.grid(row=4, column=0, columnspan=2, pady=12, sticky="w", padx=4)

    msg_lbl = ttk.Label(frame, text="", foreground="grey", wraplength=360)
    msg_lbl.grid(row=5, column=0, columnspan=4, sticky="w", padx=4)

    def do_connect():
        global ser, serial_thread, connected
        port = port_var.get()
        if not port:
            messagebox.showerror("Error", "No port selected.")
            return
        try:
            ser = serial.Serial(port, 19200, timeout=1.0)
            time.sleep(2.0)           # let Arduino reset
            connected = True
            serial_thread = threading.Thread(target=serial_reader, daemon=True)
            serial_thread.start()
            status_var.set("Connected")
            status_lbl.configure(foreground="green")
            conn_btn.configure(text="Disconnect", command=do_disconnect)
            if configure_tab_ref:
                configure_tab_ref._send_btn.configure(state="normal")
            # ask state
            send("CMD:STATUS")
        except serial.SerialException as exc:
            messagebox.showerror("Connection failed", str(exc))

    def do_disconnect():
        global ser, connected
        connected = False
        if ser:
            try:
                send("CMD:STOP")
                ser.close()
            except Exception:
                pass
            ser = None
        status_var.set("Disconnected")
        status_lbl.configure(foreground="red")
        conn_btn.configure(text="Connect", command=do_connect)
        arduino_state_var.set("—")
        if configure_tab_ref:
            configure_tab_ref._send_btn.configure(state="disabled")

    conn_btn.configure(command=do_connect)
    refresh_ports()

    # expose for poll_queue and disconnect handler
    frame._arduino_state_var = arduino_state_var
    frame._msg_lbl = msg_lbl
    frame._status_var = status_var
    frame._status_lbl = status_lbl
    frame._conn_btn = conn_btn
    frame._do_connect = do_connect
    return frame


# ── tab 2: Configure ─────────────────────────────────────────────────────────

def build_configure_tab(nb):
    frame = ttk.Frame(nb, padding=16)
    nb.add(frame, text="Configure")

    # Mode
    lbl(frame, "Mode:").grid(row=0, column=0, sticky="w", pady=4)
    mode_var = tk.StringVar(value="MEASURE")
    mode_cb = ttk.Combobox(frame, textvariable=mode_var, width=16,
                           values=["MEASURE", "SETPOINT", "SEQUENCE"],
                           state="readonly")
    mode_cb.grid(row=0, column=1, sticky="w", padx=4)

    # Channels
    lbl(frame, "Channels:").grid(row=1, column=0, sticky="w", pady=4)
    nch_var = tk.IntVar(value=1)
    ttk.Spinbox(frame, from_=1, to=4, textvariable=nch_var, width=4).grid(
        row=1, column=1, sticky="w", padx=4)

    # Relay pins
    relay_frame = ttk.LabelFrame(frame, text="Relay pins (one per channel)", padding=8)
    relay_frame.grid(row=2, column=0, columnspan=4, sticky="ew", pady=6)
    relay_vars = [tk.StringVar(value=str(3 + i)) for i in range(4)]
    for i in range(4):
        lbl(relay_frame, f"CH{i+1}:").grid(row=0, column=i * 2, padx=(8, 0))
        entry(relay_frame, relay_vars[i], width=5).grid(row=0, column=i * 2 + 1, padx=(2, 8))

    # Sampling
    misc_frame = ttk.LabelFrame(frame, text="Timing", padding=8)
    misc_frame.grid(row=3, column=0, columnspan=4, sticky="ew", pady=4)
    interval_var = tk.StringVar(value="2000")
    duration_var = tk.StringVar(value="60")
    lbl(misc_frame, "Interval (ms):").grid(row=0, column=0, sticky="w")
    entry(misc_frame, interval_var).grid(row=0, column=1, padx=4)
    lbl(misc_frame, "Duration (min):").grid(row=0, column=2, sticky="w", padx=(16, 0))
    entry(misc_frame, duration_var).grid(row=0, column=3, padx=4)

    # ── SETPOINT panel ────────────────────────────────────────────────────────
    sp_frame = ttk.LabelFrame(frame, text="Setpoint control", padding=8)
    sp_var = tk.StringVar(value="30.0")
    kp_var = tk.StringVar(value="10.0")
    ki_var = tk.StringVar(value="1.0")
    kd_var = tk.StringVar(value="0.0")
    for col, (ltext, var) in enumerate([("Setpoint (%air):", sp_var),
                                        ("Kp:", kp_var),
                                        ("Ki:", ki_var),
                                        ("Kd:", kd_var)]):
        lbl(sp_frame, ltext).grid(row=0, column=col * 2, sticky="w", padx=(4, 0))
        entry(sp_frame, var, width=7).grid(row=0, column=col * 2 + 1, padx=(2, 8))

    # ── SEQUENCE panel ────────────────────────────────────────────────────────
    seq_outer = ttk.LabelFrame(frame, text="Sequence phases", padding=8)

    seq_kp_var = tk.StringVar(value="10.0")
    seq_ki_var = tk.StringVar(value="1.0")
    seq_kd_var = tk.StringVar(value="0.0")

    pid_row = ttk.Frame(seq_outer)
    pid_row.pack(fill="x", pady=(0, 6))
    for ltext, var in [("Kp:", seq_kp_var), ("Ki:", seq_ki_var), ("Kd:", seq_kd_var)]:
        lbl(pid_row, ltext).pack(side="left", padx=(4, 0))
        entry(pid_row, var, width=6).pack(side="left", padx=(2, 8))

    cols = ("Phase", "Setpoint (%air)", "Duration (min)", "Type (c/h/p)")
    phase_tree = ttk.Treeview(seq_outer, columns=cols, show="headings", height=6)
    for c in cols:
        phase_tree.heading(c, text=c)
        phase_tree.column(c, width=120, anchor="center")
    phase_tree.pack(fill="x")

    btn_row = ttk.Frame(seq_outer)
    btn_row.pack(fill="x", pady=4)

    def add_phase():
        n = len(phase_tree.get_children())
        phase_tree.insert("", "end", values=(n + 1, "30.0", "15", "h"))

    def remove_phase():
        sel = phase_tree.selection()
        if sel:
            phase_tree.delete(sel[0])
            # renumber
            for i, iid in enumerate(phase_tree.get_children()):
                vals = list(phase_tree.item(iid, "values"))
                vals[0] = i + 1
                phase_tree.item(iid, values=vals)

    def edit_phase(event):
        """In-place edit of double-clicked cell."""
        item = phase_tree.identify_row(event.y)
        col_id = phase_tree.identify_column(event.x)
        if not item or not col_id:
            return
        col_idx = int(col_id.lstrip("#")) - 1
        if col_idx == 0:
            return   # don't edit index
        x, y, w, h = phase_tree.bbox(item, col_id)
        vals = list(phase_tree.item(item, "values"))
        var = tk.StringVar(value=vals[col_idx])

        if col_idx == 3:
            # Phase type column: combobox with descriptive labels
            TYPE_OPTIONS = ["h — hold", "c — change", "p — pause"]
            type_map = {"h": "h — hold", "c": "c — change", "p": "p — pause"}
            var.set(type_map.get(vals[col_idx], vals[col_idx]))
            edit_widget = ttk.Combobox(phase_tree, textvariable=var,
                                       values=TYPE_OPTIONS, state="readonly", width=14)
            edit_widget.place(x=x, y=y, width=w, height=h)
            edit_widget.focus()

            def commit_type(e=None):
                vals[col_idx] = var.get()[0]  # store only the letter (h/c/p)
                phase_tree.item(item, values=vals)
                edit_widget.destroy()

            edit_widget.bind("<<ComboboxSelected>>", commit_type)
            edit_widget.bind("<FocusOut>", commit_type)
        else:
            edit_widget = ttk.Entry(phase_tree, textvariable=var, width=12)
            edit_widget.place(x=x, y=y, width=w, height=h)
            edit_widget.focus()

            def commit(e=None):
                vals[col_idx] = var.get()
                phase_tree.item(item, values=vals)
                edit_widget.destroy()

            edit_widget.bind("<Return>", commit)
            edit_widget.bind("<FocusOut>", commit)

    phase_tree.bind("<Double-1>", edit_phase)
    ttk.Button(btn_row, text="Add phase", command=add_phase).pack(side="left", padx=4)
    ttk.Button(btn_row, text="Remove selected", command=remove_phase).pack(side="left", padx=4)
    # Seed with two default phases
    phase_tree.insert("", "end", values=(1, "50.0", "15", "h"))
    phase_tree.insert("", "end", values=(2, "30.0", "15", "h"))

    # ── mode switching ────────────────────────────────────────────────────────
    def on_mode_change(*_):
        m = mode_var.get()
        sp_frame.grid_forget()
        seq_outer.grid_forget()
        if m == "SETPOINT":
            sp_frame.grid(row=5, column=0, columnspan=4, sticky="ew", pady=4)
        elif m == "SEQUENCE":
            seq_outer.grid(row=5, column=0, columnspan=4, sticky="ew", pady=4)

    mode_var.trace_add("write", on_mode_change)
    on_mode_change()

    send_btn = ttk.Button(frame, text="Send Config to Arduino", state="disabled")
    send_btn.grid(row=6, column=0, columnspan=2, pady=10, sticky="w")
    cfg_status_var = tk.StringVar(value="")
    ttk.Label(frame, textvariable=cfg_status_var, foreground="blue").grid(
        row=6, column=2, columnspan=2, sticky="w")

    def validate_and_send():
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected to Arduino.")
            return

        m = mode_var.get()
        nch = nch_var.get()
        interval = int(interval_var.get())
        duration = int(duration_var.get())
        relay_pins = [relay_vars[i].get() for i in range(nch)]

        # Validate minimum interval
        min_interval = (nch + 1) * 200
        if interval < min_interval:
            messagebox.showerror(
                "Invalid config",
                f"Interval must be ≥ {min_interval} ms for {nch} channel(s).")
            return

        # Validate no duplicate pins
        all_pins = relay_pins + [str(8), str(9)]   # RX, TX are 8, 9
        if len(set(all_pins)) != len(all_pins):
            messagebox.showerror("Invalid config", "Duplicate pin assignments detected.")
            return

        cfg_status_var.set("Sending…")
        frame.update_idletasks()

        cmd_count = [0]

        def ack(cmd):
            cmd_count[0] += 1
            cfg_status_var.set(f"Sending {cmd_count[0]}…")
            frame.update_idletasks()
            return send_and_ack(cmd)

        ok = True
        ok = ok and ack(f"CFG:MODE:{m}")
        ok = ok and ack(f"CFG:NCHANNELS:{nch}")
        for i in range(nch):
            ok = ok and ack(f"CFG:RELAY:{i}:{relay_pins[i]}")
        ok = ok and ack(f"CFG:INTERVAL:{interval}")
        ok = ok and ack(f"CFG:DURATION:{duration}")

        if m == "SETPOINT":
            ok = ok and ack(f"CFG:SETPOINT:{sp_var.get()}")
            ok = ok and ack(f"CFG:KP:{kp_var.get()}")
            ok = ok and ack(f"CFG:KI:{ki_var.get()}")
            ok = ok and ack(f"CFG:KD:{kd_var.get()}")

        elif m == "SEQUENCE":
            ok = ok and ack(f"CFG:KP:{seq_kp_var.get()}")
            ok = ok and ack(f"CFG:KI:{seq_ki_var.get()}")
            ok = ok and ack(f"CFG:KD:{seq_kd_var.get()}")
            rows = phase_tree.get_children()
            ok = ok and ack(f"CFG:NPHASES:{len(rows)}")
            for idx, iid in enumerate(rows):
                vals = phase_tree.item(iid, "values")
                ok = ok and ack(
                    f"CFG:PHASE:{idx}:{vals[1]}:{vals[2]}:{vals[3]}")

        cfg_status_var.set("Config sent ✓" if ok else "Config FAILED ✗")

    send_btn.configure(command=validate_and_send)
    frame._send_btn = send_btn
    return frame


# ── tab 3: Run & Monitor ──────────────────────────────────────────────────────

def build_run_tab(nb):
    frame = ttk.Frame(nb, padding=8)
    nb.add(frame, text="Run & Monitor")

    top_bar = ttk.Frame(frame)
    top_bar.pack(fill="x", pady=(0, 6))

    start_btn = ttk.Button(top_bar, text="▶  Start")
    start_btn.pack(side="left", padx=4)
    pause_btn = ttk.Button(top_bar, text="⏸  Pause", state="disabled")
    pause_btn.pack(side="left", padx=4)
    stop_btn = ttk.Button(top_bar, text="■  Stop", state="disabled")
    stop_btn.pack(side="left", padx=4)
    ttk.Button(top_bar, text="Save CSV", command=lambda: save_csv()).pack(
        side="left", padx=12)

    info_var = tk.StringVar(value="Idle")
    ttk.Label(top_bar, textvariable=info_var, foreground="grey").pack(
        side="left", padx=8)
    auto_scroll_var = tk.BooleanVar(value=True)
    ttk.Checkbutton(top_bar, text="Auto-scroll (600s)",
                    variable=auto_scroll_var).pack(side="right", padx=8)

    # ── chart ─────────────────────────────────────────────────────────────────
    fig = Figure(figsize=(9, 3.8), dpi=96, tight_layout=True)
    ax_do = fig.add_subplot(111)
    ax_temp = ax_do.twinx()
    ax_do.set_xlabel("Time (s)")
    ax_do.set_ylabel("DO (% air sat)", color="steelblue")
    ax_temp.set_ylabel("Temp (°C)", color="coral")
    ax_do.tick_params(axis="y", labelcolor="steelblue")
    ax_temp.tick_params(axis="y", labelcolor="coral")

    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().pack(fill="both", expand=True)

    # ── data table ────────────────────────────────────────────────────────────
    tbl_frame = ttk.Frame(frame)
    tbl_frame.pack(fill="x", pady=(4, 0))
    tbl_cols = ("time_s", "DO_ch1", "DO_ch2", "DO_ch3", "DO_ch4",
                "temp", "output", "setpoint", "phase", "type")
    tbl = ttk.Treeview(tbl_frame, columns=tbl_cols, show="headings", height=5)
    for c in tbl_cols:
        tbl.heading(c, text=c)
        tbl.column(c, width=72, anchor="center")
    vsb = ttk.Scrollbar(tbl_frame, orient="vertical", command=tbl.yview)
    tbl.configure(yscrollcommand=vsb.set)
    tbl.pack(side="left", fill="x", expand=True)
    vsb.pack(side="right", fill="y")

    # chart data buffers
    t_buf = []
    do_bufs = [[] for _ in range(4)]
    temp_buf = []
    sp_buf = []

    COLORS = ["steelblue", "darkorange", "green", "purple"]
    DO_LINES = [ax_do.plot([], [], color=COLORS[i], label=f"DO ch{i+1}")[0]
                for i in range(4)]
    TEMP_LINE, = ax_temp.plot([], [], color="coral", alpha=0.6, label="Temp")
    SP_LINE,   = ax_do.plot([], [], color="navy", linestyle="--", alpha=0.5,
                             label="Setpoint")
    ax_do.legend(loc="upper left", fontsize=7)

    def clear_chart():
        t_buf.clear()
        for buf in do_bufs:
            buf.clear()
        temp_buf.clear()
        sp_buf.clear()
        for child in tbl.get_children():
            tbl.delete(child)

    def parse_data_line(line: str) -> dict | None:
        """Parse DATA:<ms>,<do...>,<temp>,<output...>,<sp>,<phase>,<ptype>"""
        body = line[5:]       # strip "DATA:"
        parts = body.split(",")
        if len(parts) < 6:
            return None
        d = {}
        d["time_s"] = int(parts[0]) / 1000.0
        # unknown n_channels — scan for first clearly non-DO value
        # format: ms, do_1..do_n, temp, out_1..out_n, sp, phase, ptype
        # Use the known order: we can reconstruct from length.
        # Let nch = (len(parts) - 4) // 2
        nch = (len(parts) - 4) // 2
        nch = max(1, min(4, nch))
        idx = 1
        d["do"] = [float(parts[idx + i]) for i in range(nch)]
        idx += nch
        d["temp"] = float(parts[idx]);  idx += 1
        d["outputs"] = [float(parts[idx + i]) for i in range(nch)];  idx += nch
        d["setpoint"] = float(parts[idx]);  idx += 1
        d["phase"] = parts[idx];  idx += 1
        d["ptype"] = parts[idx].strip() if idx < len(parts) else "?"
        return d

    def update_chart_and_table(d: dict):
        t_buf.append(d["time_s"])
        for i, v in enumerate(d["do"]):
            do_bufs[i].append(v)
        temp_buf.append(d["temp"])
        sp_buf.append(d["setpoint"])

        for i, line in enumerate(DO_LINES):
            if i < len(d["do"]):
                line.set_data(t_buf, do_bufs[i])
            else:
                line.set_data([], [])
        TEMP_LINE.set_data(t_buf, temp_buf)
        SP_LINE.set_data(t_buf, sp_buf)

        ax_do.relim(); ax_do.autoscale_view()
        ax_temp.relim(); ax_temp.autoscale_view()
        if auto_scroll_var.get() and t_buf and t_buf[-1] > 600:
            ax_do.set_xlim(t_buf[-1] - 600, t_buf[-1] + 10)
        canvas.draw_idle()

        do_str = [f"{v:.2f}" for v in d["do"]]
        while len(do_str) < 4:
            do_str.append("")
        row_vals = (
            f"{d['time_s']:.1f}",
            *do_str,
            f"{d['temp']:.2f}",
            ", ".join(f"{v:.1f}" for v in d["outputs"]),
            f"{d['setpoint']:.1f}",
            d["phase"],
            d["ptype"]
        )
        tbl.insert("", "end", values=row_vals)
        # keep at most 200 rows visible
        children = tbl.get_children()
        if len(children) > 200:
            tbl.delete(children[0])
        tbl.yview_moveto(1.0)

        data_rows.append({
            "time_s": d["time_s"],
            **{f"DO_ch{i+1}": (d["do"][i] if i < len(d["do"]) else "")
               for i in range(4)},
            "temp": d["temp"],
            "outputs": ", ".join(f"{v:.1f}" for v in d["outputs"]),
            "setpoint": d["setpoint"],
            "phase": d["phase"],
            "ptype": d["ptype"]
        })

    def set_state_stopped(msg="Stopped"):
        global running, paused
        running = False
        paused = False
        start_btn.configure(text="▶  Start", command=do_start, state="normal")
        pause_btn.configure(state="disabled")
        stop_btn.configure(state="disabled")
        info_var.set(msg)

    def set_state_running():
        global running, paused
        running = True
        paused = False
        start_btn.configure(state="disabled")
        pause_btn.configure(state="normal")
        stop_btn.configure(state="normal")
        info_var.set("Running…")

    def set_state_paused():
        global running, paused
        running = False
        paused = True
        start_btn.configure(text="▶  Resume", command=do_resume, state="normal")
        pause_btn.configure(state="disabled")
        stop_btn.configure(state="normal")
        info_var.set("Paused")

    def do_start():
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        clear_chart()
        data_rows.clear()
        send("CMD:START")
        set_state_running()

    def do_resume():
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        send("CMD:RESUME")
        set_state_running()

    def do_pause():
        send("CMD:PAUSE")
        set_state_paused()

    def do_stop():
        send("CMD:STOP")
        set_state_stopped()

    start_btn.configure(command=do_start)
    pause_btn.configure(command=do_pause)
    stop_btn.configure(command=do_stop)

    def save_csv():
        if not data_rows:
            messagebox.showinfo("No data", "No data to save yet.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="Save data as CSV",
            initialfile=f"ardoxy_data_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        )
        if not path:
            return
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(data_rows[0].keys()))
            writer.writeheader()
            writer.writerows(data_rows)
        messagebox.showinfo("Saved", f"Data saved to:\n{path}")

    # expose callbacks for poll_queue
    frame._parse_data_line = parse_data_line
    frame._update_chart_and_table = update_chart_and_table
    frame._info_var = info_var
    frame._start_btn = start_btn
    frame._pause_btn = pause_btn
    frame._stop_btn = stop_btn
    frame._set_state_stopped = set_state_stopped
    frame._set_state_paused = set_state_paused
    return frame


# ── main queue polling ────────────────────────────────────────────────────────

connect_tab_ref = None
configure_tab_ref = None
run_tab_ref = None


def handle_line(line: str):
    """Process one line from the Arduino; called from poll_queue and send_and_ack."""
    global _dl_routing
    if _dl_routing and line.startswith(("FLINE:", "FILESTART:", "FILEEND:", "FILE:", "FILES_DONE")):
        _dl_msg_queue.put_nowait(line)
        return
    if line.startswith("DATA:"):
        if run_tab_ref:
            d = run_tab_ref._parse_data_line(line)
            if d:
                run_tab_ref._update_chart_and_table(d)
            elif hasattr(run_tab_ref, "_log"):
                run_tab_ref._log(line)

    elif line.startswith("STATUS:CH:"):
        if run_tab_ref and hasattr(run_tab_ref, "_sa_update_status"):
            run_tab_ref._sa_update_status(line)
        if run_tab_ref and hasattr(run_tab_ref, "_log"):
            run_tab_ref._log(line)

    elif line.startswith("STATUS:"):
        state_str = line[7:]
        if connect_tab_ref:
            connect_tab_ref._arduino_state_var.set(state_str)
        if run_tab_ref and hasattr(run_tab_ref, "_log"):
            run_tab_ref._log(line)

    elif line.startswith("MSG:"):
        msg = line[4:]
        if connect_tab_ref:
            connect_tab_ref._msg_lbl.configure(text=msg)
        if run_tab_ref and hasattr(run_tab_ref, "_log"):
            run_tab_ref._log(line)

    elif line == "DONE":
        global running, paused
        running = False
        paused = False
        if run_tab_ref:
            run_tab_ref._set_state_stopped("Finished")
        if run_tab_ref and hasattr(run_tab_ref, "_log"):
            run_tab_ref._log("DONE")

    elif line == "__DISCONNECTED__":
        global connected
        connected = False
        running = False
        paused = False
        if connect_tab_ref:
            connect_tab_ref._status_var.set("Disconnected")
            connect_tab_ref._status_lbl.configure(foreground="red")
            connect_tab_ref._conn_btn.configure(text="Connect",
                                                command=connect_tab_ref._do_connect)
        if configure_tab_ref:
            configure_tab_ref._send_btn.configure(state="disabled")
            if hasattr(configure_tab_ref, "_test_relay_reset"):
                configure_tab_ref._test_relay_reset()
        if run_tab_ref:
            run_tab_ref._set_state_stopped("Connection lost")


def poll_queue(root):
    try:
        while not msg_queue.empty():
            try:
                line = msg_queue.get_nowait()
                handle_line(line)
            except queue.Empty:
                break
    except Exception:
        pass
    root.after(100, poll_queue, root)


# ── UI factories ──────────────────────────────────────────────────────────────

def build_live_ui(root):
    """Build the Live Experiment UI (tethered to PC via USB serial)."""
    global connect_tab_ref, configure_tab_ref, run_tab_ref
    nb = ttk.Notebook(root)
    nb.pack(fill="both", expand=True, padx=6, pady=6)
    connect_tab_ref = build_connect_tab(nb)
    configure_tab_ref = build_configure_tab(nb)
    run_tab_ref = build_run_tab(nb)
    root.after(100, poll_queue, root)


def build_standalone_ui(root):
    """Build the Standalone Experiment UI (configure Arduino autonomously)."""
    global connect_tab_ref, configure_tab_ref, run_tab_ref

    nb = ttk.Notebook(root)
    nb.pack(fill="both", expand=True, padx=6, pady=6)

    # ── Tab 1: Connect (reused builder) ───────────────────────────────────────
    connect_tab_ref = build_connect_tab(nb)

    # ── Tab 2: Configure ──────────────────────────────────────────────────────
    cfg_outer = ttk.Frame(nb, padding=0)
    nb.add(cfg_outer, text="Configure")

    # Scrollable inner form
    _canvas = tk.Canvas(cfg_outer, borderwidth=0, highlightthickness=0)
    _vsb = ttk.Scrollbar(cfg_outer, orient="vertical", command=_canvas.yview)
    _canvas.configure(yscrollcommand=_vsb.set)
    _vsb.pack(side="right", fill="y")
    _canvas.pack(side="left", fill="both", expand=True)
    inner = ttk.Frame(_canvas, padding=10)
    _win = _canvas.create_window((0, 0), window=inner, anchor="nw")

    def _on_inner_cfg(e):
        _canvas.configure(scrollregion=_canvas.bbox("all"))
    def _on_canvas_cfg(e):
        _canvas.itemconfig(_win, width=e.width)
    inner.bind("<Configure>", _on_inner_cfg)
    _canvas.bind("<Configure>", _on_canvas_cfg)

    # mouse-wheel scroll
    def _on_wheel(e):
        _canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
    _canvas.bind_all("<MouseWheel>", _on_wheel)

    r = 0  # grid row counter

    # Sensors
    lbl(inner, "Sensors:").grid(row=r, column=0, sticky="w", pady=3)
    nsensors_var = tk.IntVar(value=1)
    _sf = ttk.Frame(inner)
    _sf.grid(row=r, column=1, columnspan=5, sticky="w")
    ttk.Radiobutton(_sf, text="1 FireSting",  variable=nsensors_var, value=1).pack(side="left")
    ttk.Radiobutton(_sf, text="2 FireStings", variable=nsensors_var, value=2).pack(side="left", padx=8)
    r += 1

    # Channels on sensor 1
    lbl(inner, "Channels on sensor 1 (Serial 1):").grid(row=r, column=0, sticky="w", pady=3)
    s1ch_var = tk.IntVar(value=1)
    s1ch_spin = ttk.Spinbox(inner, from_=1, to=4, textvariable=s1ch_var, width=4)
    s1ch_spin.grid(row=r, column=1, sticky="w", padx=4)
    r += 1

    # Channels on sensor 2
    lbl(inner, "Channels on sensor 2 (Serial 2):").grid(row=r, column=0, sticky="w", pady=3)
    s2ch_var = tk.IntVar(value=1)
    s2ch_spin = ttk.Spinbox(inner, from_=1, to=4, textvariable=s2ch_var, width=4)
    s2ch_spin.grid(row=r, column=1, sticky="w", padx=4)
    lbl(inner, "(only when 2 sensors)", foreground="grey").grid(
        row=r, column=2, sticky="w", padx=4)
    r += 1

    def get_nch():
        """Total channels = sensor-1 channels [+ sensor-2 channels if 2 sensors]."""
        return s1ch_var.get() + (s2ch_var.get() if nsensors_var.get() == 2 else 0)

    # Channel IDs  (2 rows × 4 cols)
    # ── Unified channel settings table (ID, Kp, Ki, Kd, relay — one row/channel) ──
    ch_settings_frame = ttk.LabelFrame(inner, text="Channel settings", padding=6)
    ch_settings_frame.grid(row=r, column=0, columnspan=6, sticky="ew", pady=4)
    for _col, _htxt in enumerate(("Ch", "Channel ID", "Kp", "Ki", "Kd", "Relay pin")):
        ttk.Label(ch_settings_frame, text=_htxt,
                  font=("", 9, "bold")).grid(row=0, column=_col, padx=6, pady=2, sticky="w")
    tank_vars       = [tk.StringVar(value=f"CH{i+1}") for i in range(8)]
    kp_vars         = [tk.StringVar(value="10.0") for _ in range(8)]
    ki_vars         = [tk.StringVar(value="0.0")  for _ in range(8)]
    kd_vars         = [tk.StringVar(value="0.0")  for _ in range(8)]
    _default_pins   = [23, 25, 27, 29, 31, 33, 35, 37]
    relay_vars      = [tk.StringVar(value=str(_default_pins[i])) for i in range(8)]
    _ch_row_widgets = []   # per-channel list of Entry widgets for enable/disable
    for i in range(8):
        ttk.Label(ch_settings_frame, text=str(i + 1)).grid(
            row=i + 1, column=0, padx=6, pady=2)
        _e_id  = ttk.Entry(ch_settings_frame, textvariable=tank_vars[i],  width=9)
        _e_kp  = ttk.Entry(ch_settings_frame, textvariable=kp_vars[i],   width=7)
        _e_ki  = ttk.Entry(ch_settings_frame, textvariable=ki_vars[i],   width=7)
        _e_kd  = ttk.Entry(ch_settings_frame, textvariable=kd_vars[i],   width=7)
        _e_rly = ttk.Entry(ch_settings_frame, textvariable=relay_vars[i], width=5)
        _e_id.grid( row=i + 1, column=1, padx=4, pady=2)
        _e_kp.grid( row=i + 1, column=2, padx=4, pady=2)
        _e_ki.grid( row=i + 1, column=3, padx=4, pady=2)
        _e_kd.grid( row=i + 1, column=4, padx=4, pady=2)
        _e_rly.grid(row=i + 1, column=5, padx=4, pady=2)
        _ch_row_widgets.append([_e_id, _e_kp, _e_ki, _e_kd, _e_rly])
    r += 1

    # Timing
    timing_frame = ttk.LabelFrame(inner, text="Timing", padding=6)
    timing_frame.grid(row=r, column=0, columnspan=6, sticky="ew", pady=4)
    interval_var = tk.StringVar(value="30000")
    ttk.Label(timing_frame, text="Interval (ms):").grid(row=0, column=0, sticky="w")
    ttk.Entry(timing_frame, textvariable=interval_var, width=10).grid(
        row=0, column=1, padx=4)
    r += 1

    # ── Relay Test ────────────────────────────────────────────────────────────
    relay_test_frame = ttk.LabelFrame(inner, text="Relay Test  (wiring verification)", padding=6)
    relay_test_frame.grid(row=r, column=0, columnspan=6, sticky="ew", pady=4)

    relay_test_state = [False] * 8   # False=closed, True=open
    relay_test_btns  = []

    def _test_btn_text(ch):
        s = "OPEN" if relay_test_state[ch] else "CLOSED"
        return f"{tank_vars[ch].get()}  pin {relay_vars[ch].get()}\n{s}"

    def _test_relay_toggle(ch):
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        new_st = not relay_test_state[ch]
        relay_test_state[ch] = new_st
        send(f"CMD:TESTPIN:{relay_vars[ch].get()}:{1 if new_st else 0}")
        relay_test_btns[ch].configure(
            text=_test_btn_text(ch),
            foreground="red" if new_st else "black")

    def _test_relay_all_off():
        for _ch in range(8):
            if relay_test_state[_ch] and connected and ser:
                send(f"CMD:TESTPIN:{relay_vars[_ch].get()}:0")
            relay_test_state[_ch] = False
        for _ch in range(8):
            relay_test_btns[_ch].configure(text=_test_btn_text(_ch), foreground="black")

    def _test_relay_reset():
        for _ch in range(8):
            relay_test_state[_ch] = False
        for _ch in range(8):
            relay_test_btns[_ch].configure(text=_test_btn_text(_ch), foreground="black")

    for _i in range(8):
        _b = tk.Button(relay_test_frame, text=_test_btn_text(_i),
                       command=lambda ch=_i: _test_relay_toggle(ch), width=16,
                       relief="raised", padx=4, pady=4)
        _b.grid(row=_i // 4, column=_i % 4, padx=4, pady=4)
        relay_test_btns.append(_b)

    ttk.Button(relay_test_frame, text="All Closed",
               command=_test_relay_all_off).grid(
        row=2, column=0, columnspan=2, padx=4, pady=(2, 6), sticky="w")
    ttk.Label(relay_test_frame,
              text="Relays return to CLOSED on Arduino reset or power cycle.",
              foreground="grey", font=("", 8)).grid(
        row=2, column=2, columnspan=2, sticky="w", padx=4)
    r += 1

    # ── Per-channel mode & setpoint configuration ─────────────────────────────
    _CH_SEQ_COLS = ("Phase", "Type", "Days", "Hours", "Minutes",
                    "SP / minSP", "maxSP", "peakHour")
    _CH_COL_W    = (46, 62, 46, 46, 58, 82, 62, 72)
    _TYPE_OPTS   = ["h — hold", "c — change", "d — daily cycle", "p — pause"]
    _TYPE_MAP    = {"h": "h — hold", "c": "c — change",
                    "d": "d — daily cycle", "p": "p — pause"}

    ch_cfg_outer = ttk.LabelFrame(
        inner, text="Channel mode & setpoints  (tab = channel)", padding=6)
    ch_cfg_outer.grid(row=r, column=0, columnspan=6, sticky="ew", pady=4)
    ch_nb = ttk.Notebook(ch_cfg_outer)
    ch_nb.pack(fill="both", expand=True)

    _seq_clipboard    = []   # shared copy/paste clipboard for sequence phases
    ch_mode_vars      = []
    ch_immediate_vars = []
    ch_start_y_vars   = []
    ch_start_mo_vars  = []
    ch_start_d_vars   = []
    ch_start_h_vars   = []
    ch_start_mi_vars  = []
    ch_sp_vars        = []
    ch_dur_vars       = []
    ch_phase_trees    = []

    for _ci in range(8):
        _tab_f = ttk.Frame(ch_nb, padding=8)
        ch_nb.add(_tab_f, text=f"CH{_ci + 1}")
        _rf = 0

        # Mode selector
        _mv = tk.StringVar(value="MEASURE")
        ch_mode_vars.append(_mv)
        ttk.Label(_tab_f, text="Mode:").grid(row=_rf, column=0, sticky="w", pady=2)
        ttk.Combobox(_tab_f, textvariable=_mv, width=12,
                     values=["MEASURE", "SETPOINT", "SEQUENCE"],
                     state="readonly").grid(row=_rf, column=1, sticky="w", padx=4)
        _rf += 1

        # Start immediately checkbox
        _imm_v = tk.BooleanVar(value=True)
        ch_immediate_vars.append(_imm_v)
        ttk.Checkbutton(_tab_f, text="Start immediately",
                        variable=_imm_v).grid(
            row=_rf, column=0, columnspan=2, sticky="w", pady=2)
        _rf += 1

        # Start datetime entries
        _now = time.localtime()
        _sy  = tk.StringVar(value=str(_now.tm_year))
        _smo = tk.StringVar(value=str(_now.tm_mon))
        _sd  = tk.StringVar(value=str(_now.tm_mday))
        _sh  = tk.StringVar(value=str(_now.tm_hour))
        _smi = tk.StringVar(value=str(_now.tm_min))
        ch_start_y_vars.append(_sy)
        ch_start_mo_vars.append(_smo)
        ch_start_d_vars.append(_sd)
        ch_start_h_vars.append(_sh)
        ch_start_mi_vars.append(_smi)
        _dt_f = ttk.Frame(_tab_f)
        _dt_f.grid(row=_rf, column=0, columnspan=6, sticky="w", pady=2)
        for _lt, _sv, _ew in [("Start: Y", _sy, 5), ("  M", _smo, 3),
                               ("  D", _sd, 3), ("    H", _sh, 3), ("  M", _smi, 3)]:
            ttk.Label(_dt_f, text=_lt).pack(side="left")
            ttk.Entry(_dt_f, textvariable=_sv, width=_ew).pack(side="left", padx=(2, 0))
        _rf += 1

        def _make_dt_toggle(_imm_v=_imm_v, _dt_f=_dt_f):
            def _toggle_dt(*_):
                _s = "disabled" if _imm_v.get() else "normal"
                for _w in _dt_f.winfo_children():
                    try:
                        _w.configure(state=_s)
                    except tk.TclError:
                        pass
            return _toggle_dt

        _tgl_dt = _make_dt_toggle()
        _imm_v.trace_add("write", _tgl_dt)
        _tgl_dt()

        # SETPOINT panel (hidden until mode=SETPOINT)
        _sp_pf = ttk.LabelFrame(_tab_f, text="Setpoint", padding=6)
        _sp_v  = tk.StringVar(value="30.0")
        _dur_v = tk.StringVar(value="1440")
        ch_sp_vars.append(_sp_v)
        ch_dur_vars.append(_dur_v)
        ttk.Label(_sp_pf, text="Setpoint (% air):").grid(row=0, column=0, sticky="w")
        ttk.Entry(_sp_pf, textvariable=_sp_v, width=8).grid(row=0, column=1, padx=4)
        ttk.Label(_sp_pf, text="Duration (min):").grid(
            row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Entry(_sp_pf, textvariable=_dur_v, width=7).grid(row=0, column=3, padx=4)

        # SEQUENCE panel (hidden until mode=SEQUENCE)
        _seq_pf = ttk.LabelFrame(
            _tab_f, text="Sequence phases  (double-click to edit)", padding=6)
        _pt = ttk.Treeview(_seq_pf, columns=_CH_SEQ_COLS, show="headings", height=5)
        for _cc, _ww in zip(_CH_SEQ_COLS, _CH_COL_W):
            _pt.heading(_cc, text=_cc)
            _pt.column(_cc, width=_ww, anchor="center")
        _pt.pack(fill="x", pady=(0, 2))
        ch_phase_trees.append(_pt)
        _pb = ttk.Frame(_seq_pf)
        _pb.pack(fill="x")

        def _make_ch_add(_pt=_pt):
            def _ch_add():
                n = len(_pt.get_children())
                _pt.insert("", "end", values=(n + 1, "h", "0", "1", "0", "30.0", "", ""))
            return _ch_add

        def _make_ch_remove(_pt=_pt):
            def _ch_remove():
                sel = _pt.selection()
                if sel:
                    _pt.delete(sel[0])
                    for _ii, _iid in enumerate(_pt.get_children()):
                        _v = list(_pt.item(_iid, "values"))
                        _v[0] = _ii + 1
                        _pt.item(_iid, values=_v)
            return _ch_remove

        def _make_ch_edit(_pt=_pt):
            def _ch_edit(event):
                item = _pt.identify_row(event.y)
                col_id = _pt.identify_column(event.x)
                if not item or not col_id:
                    return
                col_idx = int(col_id.lstrip("#")) - 1
                if col_idx == 0:
                    return
                x, y, w, h = _pt.bbox(item, col_id)
                vals = list(_pt.item(item, "values"))
                var = tk.StringVar(value=vals[col_idx])
                if col_idx == 1:
                    var.set(_TYPE_MAP.get(vals[col_idx], vals[col_idx]))
                    ew = ttk.Combobox(_pt, textvariable=var,
                                      values=_TYPE_OPTS, state="readonly", width=16)
                    ew.place(x=x, y=y, width=w + 60, height=h)
                    ew.focus()
                    def _ct(e=None, vals=vals, col_idx=col_idx, item=item, ew=ew):
                        vals[col_idx] = var.get()[0]
                        _pt.item(item, values=vals)
                        ew.destroy()
                    ew.bind("<<ComboboxSelected>>", _ct)
                    ew.bind("<FocusOut>", _ct)
                else:
                    ew = ttk.Entry(_pt, textvariable=var, width=10)
                    ew.place(x=x, y=y, width=w, height=h)
                    ew.focus()
                    def _ce(e=None, vals=vals, col_idx=col_idx, item=item, ew=ew):
                        vals[col_idx] = var.get()
                        _pt.item(item, values=vals)
                        ew.destroy()
                    ew.bind("<Return>", _ce)
                    ew.bind("<FocusOut>", _ce)
            return _ch_edit

        _pt.bind("<Double-1>", _make_ch_edit())
        def _make_ch_copy(_pt=_pt):
            def _ch_copy():
                _seq_clipboard.clear()
                for _iid in _pt.get_children():
                    _seq_clipboard.append(_pt.item(_iid, "values"))
            return _ch_copy

        def _make_ch_paste(_pt=_pt):
            def _ch_paste():
                if not _seq_clipboard:
                    messagebox.showinfo("Paste sequence", "Clipboard is empty.")
                    return
                for _iid in _pt.get_children():
                    _pt.delete(_iid)
                for _ii, _row in enumerate(_seq_clipboard):
                    _rv = list(_row)
                    _rv[0] = _ii + 1   # renumber
                    _pt.insert("", "end", values=_rv)
            return _ch_paste

        ttk.Button(_pb, text="Add phase",
                   command=_make_ch_add()).pack(side="left", padx=4)
        ttk.Button(_pb, text="Remove selected",
                   command=_make_ch_remove()).pack(side="left", padx=4)
        ttk.Button(_pb, text="Copy sequence",
                   command=_make_ch_copy()).pack(side="left", padx=4)
        ttk.Button(_pb, text="Paste sequence",
                   command=_make_ch_paste()).pack(side="left", padx=4)
        ttk.Label(_pb,
                  text="For 'd': SP/minSP=min DO, maxSP=max DO, peakHour=h of max  |  For 'c': SP=target, maxSP=start DO (required)",
                  foreground="grey", font=("", 8)).pack(side="left", padx=8)
        _pt.insert("", "end", values=(1, "c", "0", "1", "0", "15.0", "100.0", ""))
        _pt.insert("", "end", values=(2, "h", "0", "1", "0", "15.0", "", ""))

        # Show/hide mode-specific panels
        def _make_mode_toggle(_mv=_mv, _sp_pf=_sp_pf, _seq_pf=_seq_pf, _rf=_rf):
            def _ch_mode_chg(*_):
                _sp_pf.grid_forget()
                _seq_pf.grid_forget()
                _m = _mv.get()
                if _m == "SETPOINT":
                    _sp_pf.grid(row=_rf, column=0, columnspan=6, sticky="ew", pady=4)
                elif _m == "SEQUENCE":
                    _seq_pf.grid(row=_rf, column=0, columnspan=6, sticky="ew", pady=4)
            return _ch_mode_chg

        _ch_toggle = _make_mode_toggle()
        _mv.trace_add("write", _ch_toggle)
        _ch_toggle()

    r += 1

    # Update channel table rows + notebook tabs based on active channel count
    def _update_active_channels(*_):
        ns  = nsensors_var.get()
        nch = get_nch()
        s2ch_spin.configure(state="normal" if ns == 2 else "disabled")
        for _ii, _row_w in enumerate(_ch_row_widgets):
            _s = "normal" if _ii < nch else "disabled"
            for _w in _row_w:
                _w.configure(state=_s)
        for _ii in range(8):
            ch_nb.tab(_ii, state="normal" if _ii < nch else "disabled")
            relay_test_btns[_ii].configure(state="normal" if _ii < nch else "disabled")
        try:
            if int(ch_nb.index(ch_nb.select())) >= nch:
                ch_nb.select(nch - 1)
        except Exception:
            pass

    nsensors_var.trace_add("write", _update_active_channels)
    s1ch_var.trace_add("write", _update_active_channels)
    s2ch_var.trace_add("write", _update_active_channels)
    _update_active_channels()

    # Action row: Sync RTC + Send & Save Config
    cfg_status_var = tk.StringVar(value="")
    action_row = ttk.Frame(inner)
    action_row.grid(row=r, column=0, columnspan=6, sticky="ew", pady=10)

    def _sync_rtc():
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        t = time.localtime()
        cmd = (f"CMD:SETRTC:{t.tm_year}:{t.tm_mon}:{t.tm_mday}"
               f":{t.tm_hour}:{t.tm_min}:{t.tm_sec}")
        ok = send_and_ack(cmd)
        cfg_status_var.set("RTC synced ✓" if ok else "RTC sync FAILED ✗")

    ttk.Button(action_row, text="Sync RTC to PC clock",
               command=_sync_rtc).pack(side="left", padx=4)

    send_btn = ttk.Button(action_row, text="Send & Save Config", state="disabled")
    send_btn.pack(side="left", padx=8)
    ttk.Button(action_row, text="Export Config…",
               command=lambda: _export_config()).pack(side="left", padx=4)
    ttk.Button(action_row, text="Import Config…",
               command=lambda: _import_config()).pack(side="left", padx=4)
    ttk.Button(action_row, text="Read from Arduino…",
               command=lambda: _import_from_arduino()).pack(side="left", padx=4)
    ttk.Label(action_row, textvariable=cfg_status_var,
              foreground="blue").pack(side="left", padx=4)
    r += 1

    def _validate_and_send():
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected to Arduino.")
            return
        ns   = nsensors_var.get()
        nch  = get_nch()
        s1ch = s1ch_var.get()

        cfg_status_var.set("Sending…")
        inner.update_idletasks()

        _n = [0]
        def ack(cmd):
            _n[0] += 1
            cfg_status_var.set(f"Sending command {_n[0]}…")
            inner.update_idletasks()
            return send_and_ack(cmd)

        ok = True
        ok = ok and ack(f"CFG:NCHANNELS:{nch}")
        ok = ok and ack(f"CFG:SENSORS:{ns}")
        if ns == 2:
            ok = ok and ack(f"CFG:S1CHANNELS:{s1ch}")
        ok = ok and ack(f"CFG:INTERVAL:{interval_var.get()}")

        for i in range(nch):
            ok = ok and ack(f"CFG:TANKID:{i}:{tank_vars[i].get()}")
            ok = ok and ack(f"CFG:KP:{i}:{kp_vars[i].get()}")
            ok = ok and ack(f"CFG:KI:{i}:{ki_vars[i].get()}")
            ok = ok and ack(f"CFG:KD:{i}:{kd_vars[i].get()}")
            ok = ok and ack(f"CFG:RELAY:{i}:{relay_vars[i].get()}")

        for i in range(nch):
            m = ch_mode_vars[i].get()
            ok = ok and ack(f"CFG:CH:{i}:MODE:{m}")
            if ch_immediate_vars[i].get():
                ok = ok and ack(f"CFG:CH:{i}:START:0:0:0:0:0:0")
            else:
                y  = ch_start_y_vars[i].get()
                mo = ch_start_mo_vars[i].get()
                d  = ch_start_d_vars[i].get()
                h  = ch_start_h_vars[i].get()
                mi = ch_start_mi_vars[i].get()
                ok = ok and ack(f"CFG:CH:{i}:START:{y}:{mo}:{d}:{h}:{mi}:0")
            if m == "SETPOINT":
                ok = ok and ack(f"CFG:CH:{i}:SETPOINT:{ch_sp_vars[i].get()}")
                ok = ok and ack(f"CFG:CH:{i}:DURATION:{ch_dur_vars[i].get()}")
            elif m == "SEQUENCE":
                rows = ch_phase_trees[i].get_children()
                ok = ok and ack(f"CFG:CH:{i}:NPHASES:{len(rows)}")
                for idx, iid in enumerate(rows):
                    v = ch_phase_trees[i].item(iid, "values")
                    ptype = v[1]
                    d_v, h_v, mi_v, sp = v[2], v[3], v[4], v[5]
                    maxsp = v[6] if v[6] else "0"
                    peak  = v[7] if v[7] else "0"
                    if ptype == "d":
                        if not v[6] or not v[7]:
                            messagebox.showerror("Invalid config",
                                f"CH{i+1} phase {idx+1}: daily cycle requires maxSP and peakHour.")
                            cfg_status_var.set("Config FAILED \u2717")
                            return
                        if float(sp) >= float(maxsp):
                            messagebox.showerror("Invalid config",
                                f"CH{i+1} phase {idx+1}: daily cycle minSP ({sp}) must be < maxSP ({maxsp}).")
                            cfg_status_var.set("Config FAILED \u2717")
                            return
                        ok = ok and ack(
                            f"CFG:CH:{i}:PHASE:{idx}:{sp}:{d_v}:{h_v}:{mi_v}"
                            f":d:{sp}:{maxsp}:{peak}")
                    elif ptype == "c":
                        if not v[6] or float(v[6]) <= 0:
                            messagebox.showerror("Invalid config",
                                f"CH{i+1} phase {idx+1}: ramp phase requires startDO (maxSP field, must be > 0).")
                            cfg_status_var.set("Config FAILED \u2717")
                            return
                        ok = ok and ack(
                            f"CFG:CH:{i}:PHASE:{idx}:{sp}:{d_v}:{h_v}:{mi_v}:c:{maxsp}")
                    else:
                        ok = ok and ack(
                            f"CFG:CH:{i}:PHASE:{idx}:{sp}:{d_v}:{h_v}:{mi_v}:{ptype}")

        if ok:
            ok = ok and ack("CMD:SAVECONFIG")

        cfg_status_var.set(
            "Config sent & saved ✓" if ok else "FAILED — check serial log ✗")

    def _export_config():
        """Save current GUI configuration as CONFIG.TXT (key=value) to a PC file."""
        ns   = nsensors_var.get()
        nch  = get_nch()
        s1ch = s1ch_var.get()
        lines = []
        lines.append(f"NCHANNELS={nch}")
        lines.append(f"NSENSORS={ns}")
        if ns == 2:
            lines.append(f"S1CHANNELS={s1ch}")
        lines.append(f"INTERVAL={interval_var.get()}")
        for i in range(nch):
            lines.append(f"RELAY_{i}={relay_vars[i].get()}")
        for i in range(nch):
            lines.append(f"TANKID_{i}={tank_vars[i].get()}")
            lines.append(f"KP_{i}={kp_vars[i].get()}")
            lines.append(f"KI_{i}={ki_vars[i].get()}")
            lines.append(f"KD_{i}={kd_vars[i].get()}")
        _mode_int = {"MEASURE": 0, "SETPOINT": 1, "SEQUENCE": 2}
        for i in range(nch):
            m = ch_mode_vars[i].get()
            lines.append(f"CH_{i}_MODE={_mode_int[m]}")
            if ch_immediate_vars[i].get():
                lines.append(f"CH_{i}_START=0")
            else:
                try:
                    _t = time.struct_time((
                        int(ch_start_y_vars[i].get()),
                        int(ch_start_mo_vars[i].get()),
                        int(ch_start_d_vars[i].get()),
                        int(ch_start_h_vars[i].get()),
                        int(ch_start_mi_vars[i].get()),
                        0, 0, 0, -1))
                    lines.append(f"CH_{i}_START={int(time.mktime(_t))}")
                except ValueError:
                    lines.append(f"CH_{i}_START=0")
            if m == "SETPOINT":
                lines.append(f"CH_{i}_SETPOINT={ch_sp_vars[i].get()}")
                lines.append(f"CH_{i}_DUR_MIN={ch_dur_vars[i].get()}")
            elif m == "SEQUENCE":
                rows = ch_phase_trees[i].get_children()
                lines.append(f"CH_{i}_NPHASES={len(rows)}")
                for j, iid in enumerate(rows):
                    v = ch_phase_trees[i].item(iid, "values")
                    ptype = v[1]
                    dur_sec = (int(v[2]) * 86400 + int(v[3]) * 3600
                               + int(v[4]) * 60)
                    sp = v[5]
                    maxsp = v[6] if v[6] else "0"
                    peak  = v[7] if v[7] else "0"
                    if ptype == "d":
                        if not v[6] or not v[7]:
                            messagebox.showerror("Invalid config",
                                f"CH{i+1} phase {j+1}: daily cycle requires maxSP and peakHour.")
                            cfg_status_var.set("Export FAILED \u2717")
                            return
                        if float(sp) >= float(maxsp):
                            messagebox.showerror("Invalid config",
                                f"CH{i+1} phase {j+1}: daily cycle minSP ({sp}) must be < maxSP ({maxsp}).")
                            cfg_status_var.set("Export FAILED \u2717")
                            return
                        lines.append(
                            f"CH_{i}_PHASE_{j}={sp},{dur_sec},{ptype},{sp},{maxsp},{peak}")
                    elif ptype == "c":
                        if not v[6] or float(v[6]) <= 0:
                            messagebox.showerror("Invalid config",
                                f"CH{i+1} phase {j+1}: ramp phase requires startDO (maxSP field, must be > 0).")
                            cfg_status_var.set("Export FAILED \u2717")
                            return
                        lines.append(f"CH_{i}_PHASE_{j}={sp},{dur_sec},{ptype},{maxsp}")
                    else:
                        lines.append(f"CH_{i}_PHASE_{j}={sp},{dur_sec},{ptype}")
        path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Export CONFIG.TXT",
            initialfile="CONFIG.TXT"
        )
        if not path:
            return
        with open(path, "w", newline="\n") as f:
            f.write("\n".join(lines) + "\n")
        cfg_status_var.set("Config exported ✓")

    def _apply_config_dict(cfg):
        """Apply a key=value config dict to all GUI variables. Shared by all import sources."""
        _mode_str = {"0": "MEASURE", "1": "SETPOINT", "2": "SEQUENCE"}
        ns  = int(cfg.get("NSENSORS", "1"))
        nch = int(cfg.get("NCHANNELS", "1"))
        nsensors_var.set(ns)
        if ns == 2:
            s1ch = int(cfg.get("S1CHANNELS", str(nch)))
            s2ch = nch - s1ch
            s1ch_var.set(s1ch)
            s2ch_var.set(max(1, s2ch))
        else:
            s1ch_var.set(nch)
        if "INTERVAL" in cfg:
            interval_var.set(cfg["INTERVAL"])
        for i in range(nch):
            if f"RELAY_{i}" in cfg: relay_vars[i].set(cfg[f"RELAY_{i}"])
            if f"TANKID_{i}" in cfg: tank_vars[i].set(cfg[f"TANKID_{i}"])
            if f"KP_{i}" in cfg: kp_vars[i].set(cfg[f"KP_{i}"])
            if f"KI_{i}" in cfg: ki_vars[i].set(cfg[f"KI_{i}"])
            if f"KD_{i}" in cfg: kd_vars[i].set(cfg[f"KD_{i}"])
        for i in range(nch):
            m_str = _mode_str.get(cfg.get(f"CH_{i}_MODE", "0"), "MEASURE")
            ch_mode_vars[i].set(m_str)
            start_ts = int(cfg.get(f"CH_{i}_START", "0"))
            if start_ts == 0:
                ch_immediate_vars[i].set(True)
            else:
                ch_immediate_vars[i].set(False)
                _lt = time.localtime(start_ts)
                ch_start_y_vars[i].set(str(_lt.tm_year))
                ch_start_mo_vars[i].set(str(_lt.tm_mon))
                ch_start_d_vars[i].set(str(_lt.tm_mday))
                ch_start_h_vars[i].set(str(_lt.tm_hour))
                ch_start_mi_vars[i].set(str(_lt.tm_min))
            if m_str == "SETPOINT":
                if f"CH_{i}_SETPOINT" in cfg: ch_sp_vars[i].set(cfg[f"CH_{i}_SETPOINT"])
                if f"CH_{i}_DUR_MIN" in cfg: ch_dur_vars[i].set(cfg[f"CH_{i}_DUR_MIN"])
            elif m_str == "SEQUENCE":
                _pt = ch_phase_trees[i]
                _pt.delete(*_pt.get_children())
                n_phases = int(cfg.get(f"CH_{i}_NPHASES", "0"))
                for j in range(n_phases):
                    phase_val = cfg.get(f"CH_{i}_PHASE_{j}", "")
                    if not phase_val:
                        continue
                    parts = phase_val.split(",")
                    if len(parts) < 3:
                        continue
                    sp      = parts[0]
                    dur_sec = int(parts[1])
                    ptype   = parts[2]
                    days    = dur_sec // 86400
                    rem_    = dur_sec % 86400
                    hours   = rem_ // 3600
                    minutes = (rem_ % 3600) // 60
                    if ptype == "d" and len(parts) >= 6:
                        maxsp = parts[4]; peak = parts[5]
                    elif ptype == "c" and len(parts) >= 4:
                        maxsp = parts[3]; peak = ""  # start DO for 'c' phase
                    else:
                        maxsp = ""; peak = ""
                    _pt.insert("", "end",
                               values=(j + 1, ptype, days, hours, minutes,
                                       sp, maxsp, peak))

    def _import_config():
        """Load a previously exported CONFIG.TXT and populate all GUI variables."""
        path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Import CONFIG.TXT"
        )
        if not path:
            return
        cfg = {}
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or "=" not in line:
                    continue
                key, _, val = line.partition("=")
                cfg[key.strip()] = val.strip()
        _apply_config_dict(cfg)
        cfg_status_var.set("Config imported ✓")

    def _import_from_arduino():
        """Read current in-memory config from Arduino over serial and populate GUI."""
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        cfg_status_var.set("Reading config from Arduino…")
        inner.update_idletasks()
        while not msg_queue.empty():
            try: msg_queue.get_nowait()
            except queue.Empty: break
        send("CMD:READCONFIG")
        lines = []
        in_config = False
        deadline = time.time() + 5.0
        while time.time() < deadline:
            try:
                line = msg_queue.get(timeout=0.1)
                if line == "CONFIG_START":
                    in_config = True
                elif line == "CONFIG_END" and in_config:
                    break
                elif in_config and "=" in line:
                    lines.append(line)
            except queue.Empty:
                pass
        if not lines:
            cfg_status_var.set("No config received — check connection ✗")
            return
        cfg = {}
        for line in lines:
            key, _, val = line.partition("=")
            cfg[key.strip()] = val.strip()
        _apply_config_dict(cfg)
        cfg_status_var.set(f"Config imported from Arduino ✓  ({len(cfg)} keys)")

    send_btn.configure(command=_validate_and_send)
    cfg_outer._send_btn = send_btn
    cfg_outer._test_relay_reset = _test_relay_reset
    configure_tab_ref = cfg_outer

    # ── Tab 3: Run & Monitor ──────────────────────────────────────────────────
    run_frame = ttk.Frame(nb, padding=8)
    nb.add(run_frame, text="Run & Monitor")

    ctrl_bar = ttk.Frame(run_frame)
    ctrl_bar.pack(fill="x", pady=(0, 6))

    sa_start_btn   = ttk.Button(ctrl_bar, text="▶  Start")
    sa_start_btn.pack(side="left", padx=4)
    sa_recover_btn = ttk.Button(ctrl_bar, text="↺  Recover")
    sa_recover_btn.pack(side="left", padx=4)
    sa_pause_btn   = ttk.Button(ctrl_bar, text="⏸  Pause",  state="disabled")
    sa_pause_btn.pack(side="left", padx=4)
    sa_resume_btn  = ttk.Button(ctrl_bar, text="▶  Resume", state="disabled")
    sa_resume_btn.pack(side="left", padx=4)
    sa_stop_btn    = ttk.Button(ctrl_bar, text="■  Stop",   state="disabled")
    sa_stop_btn.pack(side="left", padx=4)

    sa_info_var = tk.StringVar(value="Idle")
    ttk.Label(ctrl_bar, textvariable=sa_info_var,
              foreground="grey").pack(side="left", padx=12)

    # Channel status overview
    _st_outer = ttk.LabelFrame(run_frame, text="Channel status", padding=4)
    _st_outer.pack(fill="x", pady=(0, 4))
    _ST_COLS = ("CH", "TankID", "Status", "Phase", "DO (% air)", "SP (% air)")
    _ST_W    = (32, 70, 150, 46, 80, 80)
    sa_status_tree = ttk.Treeview(_st_outer, columns=_ST_COLS,
                                  show="headings", height=4)
    for _sc, _sw in zip(_ST_COLS, _ST_W):
        sa_status_tree.heading(_sc, text=_sc)
        sa_status_tree.column(_sc, width=_sw, anchor="center")
    sa_status_tree.pack(fill="x")
    _sa_status_iids = []
    for _sii in range(8):
        _iid = sa_status_tree.insert("", "end",
                                     values=(_sii + 1, f"CH{_sii + 1}",
                                             "—", "—", "—", "—"))
        _sa_status_iids.append(_iid)

    def _sa_update_status(line: str):
        """Parse STATUS:CH:<i>:<status>:<phaseIdx>:<do>:<sp> and update table."""
        parts = line.split(":")
        if len(parts) < 7:
            return
        try:
            ch_i   = int(parts[2])
            status = parts[3]
            ph_idx = parts[4]
            do_val = parts[5]
            sp_val = parts[6].strip()
            tid    = tank_vars[ch_i].get() if ch_i < len(tank_vars) else f"CH{ch_i + 1}"
            if 0 <= ch_i < 8:
                sa_status_tree.item(_sa_status_iids[ch_i],
                                    values=(ch_i + 1, tid, status,
                                            ph_idx, do_val, sp_val))
        except (ValueError, IndexError):
            pass

    # \u2500\u2500 Download Log Files \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
    dl_frame = ttk.LabelFrame(run_frame, text="Download Log Files", padding=4)
    dl_frame.pack(fill="x", pady=(0, 4))

    dl_top = ttk.Frame(dl_frame)
    dl_top.pack(fill="x", pady=(0, 2))
    ttk.Button(dl_top, text="List Files",
               command=lambda: _dl_list_files()).pack(side="left", padx=4)
    ttk.Button(dl_top, text="Download Selected",
               command=lambda: _dl_download()).pack(side="left", padx=4)
    dl_status_var = tk.StringVar(value="Connect and click 'List Files' to begin")
    ttk.Label(dl_top, textvariable=dl_status_var,
              foreground="grey", font=("", 8)).pack(side="left", padx=8)

    dl_prog_var = tk.DoubleVar(value=0)
    ttk.Progressbar(dl_frame, variable=dl_prog_var, maximum=100).pack(
        fill="x", padx=4, pady=(0, 2))

    dl_cols = ("Filename", "Size", "Est. time")
    dl_tree = ttk.Treeview(dl_frame, columns=dl_cols, show="headings", height=3)
    for _dc, _dw in zip(dl_cols, (180, 80, 80)):
        dl_tree.heading(_dc, text=_dc)
        dl_tree.column(_dc, width=_dw, anchor="center")
    dl_tree.pack(fill="x", padx=4, pady=(0, 4))

    def _dl_list_files():
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        global _dl_routing
        dl_status_var.set("Listing files\u2026")
        dl_frame.update_idletasks()
        while not _dl_msg_queue.empty():
            try: _dl_msg_queue.get_nowait()
            except queue.Empty: break
        _dl_routing = True
        send("CMD:LISTFILES")
        for iid in dl_tree.get_children():
            dl_tree.delete(iid)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            try:
                line = _dl_msg_queue.get(timeout=0.2)
                if line == "FILES_DONE":
                    break
                elif line.startswith("FILE:"):
                    parts = line[5:].rsplit(":", 1)
                    if len(parts) == 2:
                        fname = parts[0]
                        try:
                            sz = int(parts[1])
                            sz_str  = f"{sz // 1024} KB"
                            est_s   = sz / 1920
                            est_str = f"~{int(est_s//60)}m {int(est_s%60)}s"
                        except ValueError:
                            sz_str = "?"; est_str = "?"
                        dl_tree.insert("", "end", values=(fname, sz_str, est_str))
            except queue.Empty:
                pass
        _dl_routing = False
        n = len(dl_tree.get_children())
        dl_status_var.set(f"{n} file(s) found" if n else "No .csv files on SD card")

    def _dl_download():
        sel = dl_tree.selection()
        if not sel:
            messagebox.showinfo("Download", "Select a file from the list first.")
            return
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        fname = dl_tree.item(sel[0], "values")[0]
        save_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="Save log file as",
            initialfile=fname
        )
        if not save_path:
            return

        def _worker():
            global _dl_routing
            try:
                while not _dl_msg_queue.empty():
                    try: _dl_msg_queue.get_nowait()
                    except queue.Empty: break
                _dl_routing = True
                send(f"CMD:SENDFILE:{fname}")
                total_bytes = 0
                deadline = time.time() + 5.0
                while time.time() < deadline:
                    try:
                        line = _dl_msg_queue.get(timeout=0.2)
                        if line.startswith("FILESTART:"):
                            try: total_bytes = int(line.rsplit(":", 1)[1])
                            except (ValueError, IndexError): pass
                            break
                        elif "ACK:ERR" in line:
                            dl_frame.after(0, lambda l=line: dl_status_var.set(f"Error: {l}"))
                            _dl_routing = False; return
                    except queue.Empty:
                        pass
                bytes_rx = 0; row_count = 0
                with open(save_path, "w", newline="\n") as out:
                    while True:
                        try:
                            line = _dl_msg_queue.get(timeout=15.0)
                            if line.startswith("FLINE:"):
                                content = line[6:]
                                out.write(content + "\n")
                                bytes_rx += len(content) + 1
                                row_count += 1
                                if total_bytes > 0:
                                    pct = min(100.0, bytes_rx / total_bytes * 100)
                                    dl_frame.after(0, lambda p=pct: dl_prog_var.set(p))
                                if row_count % 100 == 0:
                                    kb = bytes_rx // 1024
                                    dl_frame.after(0, lambda k=kb: dl_status_var.set(
                                        f"Downloading\u2026 {k} KB received"))
                            elif line.startswith("FILEEND:"):
                                break
                        except queue.Empty:
                            dl_frame.after(0, lambda: dl_status_var.set("Timeout \u2717"))
                            _dl_routing = False; return
                dl_frame.after(0, lambda: dl_prog_var.set(100))
                short = save_path.replace("\\", "/").split("/")[-1]
                dl_frame.after(0, lambda: dl_status_var.set(
                    f"Saved {row_count} rows \u2192 {short} \u2713"))
            except Exception as e:
                dl_frame.after(0, lambda: dl_status_var.set(f"Error: {e}"))
            finally:
                _dl_routing = False

        dl_prog_var.set(0)
        dl_status_var.set("Starting download\u2026")
        threading.Thread(target=_worker, daemon=True).start()

    # Serial log
    log_frame = ttk.LabelFrame(run_frame, text="Serial log", padding=4)
    log_frame.pack(fill="both", expand=True)
    log_text = tk.Text(log_frame, state="disabled", wrap="word",
                       height=8, font=("Courier New", 9))
    _log_scroll = ttk.Scrollbar(log_frame, orient="vertical",
                                command=log_text.yview)
    log_text.configure(yscrollcommand=_log_scroll.set)
    _log_scroll.pack(side="right", fill="y")
    log_text.pack(fill="both", expand=True)

    def _sa_log(msg):
        log_text.configure(state="normal")
        log_text.insert("end", msg + "\n")
        log_text.configure(state="disabled")
        log_text.yview_moveto(1.0)

    def _sa_set_running():
        sa_start_btn.configure(state="disabled")
        sa_recover_btn.configure(state="disabled")
        sa_pause_btn.configure(state="normal")
        sa_resume_btn.configure(state="disabled")
        sa_stop_btn.configure(state="normal")
        sa_info_var.set("Running…")

    def _sa_set_state_paused():
        sa_recover_btn.configure(state="disabled")
        sa_pause_btn.configure(state="disabled")
        sa_resume_btn.configure(state="normal")
        sa_info_var.set("Paused")

    def _sa_set_state_stopped(msg="Stopped"):
        sa_start_btn.configure(state="normal")
        sa_recover_btn.configure(state="normal")
        sa_pause_btn.configure(state="disabled")
        sa_resume_btn.configure(state="disabled")
        sa_stop_btn.configure(state="disabled")
        sa_info_var.set(msg)

    def _sa_do_start():
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        send("CMD:START")
        _sa_set_running()

    def _sa_do_pause():
        send("CMD:PAUSE")
        _sa_set_state_paused()

    def _sa_do_resume():
        send("CMD:RESUME")
        _sa_set_running()

    def _sa_do_stop():
        send("CMD:STOP")
        _sa_set_state_stopped()

    def _sa_do_recover():
        if not connected or not ser:
            messagebox.showerror("Error", "Not connected.")
            return
        send("CMD:RECOVER")
        _sa_set_running()

    sa_start_btn.configure(command=_sa_do_start)
    sa_recover_btn.configure(command=_sa_do_recover)
    sa_pause_btn.configure(command=_sa_do_pause)
    sa_resume_btn.configure(command=_sa_do_resume)
    sa_stop_btn.configure(command=_sa_do_stop)

    # Wire required attributes for handle_line()
    run_frame._parse_data_line      = lambda line: None   # log via _log; no chart
    run_frame._update_chart_and_table = lambda d: None
    run_frame._info_var             = sa_info_var
    run_frame._start_btn            = sa_start_btn
    run_frame._recover_btn          = sa_recover_btn
    run_frame._pause_btn            = sa_pause_btn
    run_frame._stop_btn             = sa_stop_btn
    run_frame._set_state_stopped    = _sa_set_state_stopped
    run_frame._set_state_paused     = _sa_set_state_paused
    run_frame._log                  = _sa_log
    run_frame._sa_update_status     = _sa_update_status
    run_tab_ref = run_frame

    root.after(100, poll_queue, root)


# ── main ──────────────────────────────────────────────────────────────────────

root = tk.Tk()
root.title("Ardoxy")
root.minsize(860, 680)


def _launch(mode: str):
    splash.destroy()
    root.title(f"Ardoxy \u2014 {mode}")
    if mode == "live":
        build_live_ui(root)
    else:
        build_standalone_ui(root)


# ── mode-selection splash ─────────────────────────────────────────────────────
splash = ttk.Frame(root, padding=40)
splash.pack(fill="both", expand=True)

ttk.Label(splash, text="Ardoxy", font=("", 32, "bold")).pack(pady=(80, 8))
ttk.Label(splash, text="Select experiment mode to continue:",
          font=("", 12), foreground="grey").pack(pady=(0, 48))

_btn_row = ttk.Frame(splash)
_btn_row.pack()
ttk.Button(_btn_row, text="Live Experiment",
           command=lambda: _launch("live"),
           width=26).pack(side="left", padx=24)
ttk.Button(_btn_row, text="Standalone Experiment",
           command=lambda: _launch("standalone"),
           width=26).pack(side="left", padx=24)

ttk.Label(splash,
          text="Live: Arduino connected to PC via USB.    "
               "Standalone: Arduino runs autonomously with SD card.",
          foreground="grey", font=("", 9)).pack(pady=(24, 0))

root.mainloop()
