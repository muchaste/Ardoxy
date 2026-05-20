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
msg_queue = queue.Queue()
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
    if line.startswith("DATA:"):
        if run_tab_ref:
            d = run_tab_ref._parse_data_line(line)
            if d:
                run_tab_ref._update_chart_and_table(d)

    elif line.startswith("STATUS:"):
        state_str = line[7:]
        if connect_tab_ref:
            connect_tab_ref._arduino_state_var.set(state_str)

    elif line.startswith("MSG:"):
        msg = line[4:]
        if connect_tab_ref:
            connect_tab_ref._msg_lbl.configure(text=msg)

    elif line == "DONE":
        global running, paused
        running = False
        paused = False
        if run_tab_ref:
            run_tab_ref._set_state_stopped("Finished")

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


# ── main ──────────────────────────────────────────────────────────────────────

root = tk.Tk()
root.title("Ardoxy-OS")
root.minsize(860, 680)

nb = ttk.Notebook(root)
nb.pack(fill="both", expand=True, padx=6, pady=6)

connect_tab_ref = build_connect_tab(nb)
configure_tab_ref = build_configure_tab(nb)
run_tab_ref = build_run_tab(nb)

root.after(100, poll_queue, root)
root.mainloop()
