#!/usr/bin/env python3
"""
FMA Force Sensor - Serial Reader + Live Plot + Recorder (Tkinter GUI)
====================================================================
Companion to the FMA sketch that prints ONE TIMESTAMPED VALUE PER LINE.

WIRE PROTOCOL (one line per sample)
    <millis>,<force>    e.g. "8557,0.000" - Arduino millis(), force in newtons
    <millis>,-1         sensor status fault / stale
    <millis>,-2         I2C read failed
    # ...               banner / info, shown in the status bar

Negative forces are clamped to 0.0 in software, so the plot and the
recorded CSV only contain values >= 0.

RECORDED CSV COLUMNS
    index       row number since Record was pressed (starts at 0)
    device_ms   raw Arduino millis() for this sample
    elapsed_ms  device time since the first recorded sample (starts at 0)
    host_time   PC date and time when this line ARRIVED, millisecond resolution
    force_N     force in newtons

COMMAND SENT TO THE ARDUINO (one line)
    RESET       reboot the board; setup() tares the sensor again, so the
                sensor must be unloaded. USB disconnects, so the GUI stops any
                recording, closes the port, waits for the same port to come
                back, reopens it and clears the plot. The start-up messages,
                including the new zero offset, are shown in the status bar.

TIME SOURCES
    The live plot and elapsed_ms use the Arduino timestamps, so Python does
    not need to know the sample period. host_time is the PC arrival time: a
    background thread stamps each line with time.perf_counter() as soon as
    its newline arrives, anchored once to the PC wall clock. It is later than
    the real sample by USB and operating-system delays, and it jitters.

    If the device time goes backwards (the board was reset), the time axis
    continues from the last value instead of jumping back, and the reset is
    counted.

Requirements:
    pip install pyserial matplotlib
    pip install openpyxl        # only if you save as .xlsx

A COM port can be opened by only ONE program at a time.
Close the Arduino Serial Monitor and Docklight before pressing Start.
"""

import csv
import os
import sys 
import queue
import threading
import time
from collections import deque
from datetime import datetime, timedelta

import serial
from serial.tools import list_ports

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk


# ----------------------------- configuration ------------------------------
DEFAULT_BAUD   = "115200"
BAUD_CHOICES   = ("9600", "19200", "38400", "57600",
                  "115200", "230400", "250000", "500000", "1000000")

PLOT_SECONDS      = 10
MAX_BUFFER_POINTS = 20000      # hard safety cap for the plot buffers
MAX_PLOT_POINTS   = 1500
POLL_MS           = 20         # how often the GUI drains the reader queue
REDRAW_MS         = 50
RATE_MS           = 500
BOOT_DELAY_MS     = 4000
RX_LIMIT          = 8192
FLUSH_SECONDS     = 1.0
SERIAL_TIMEOUT_S  = 0.1        # reader thread wakes at least this often
READER_JOIN_S     = 1.0
RESET_CLOSE_MS    = 300        # after sending RESET, wait before closing the port
RESET_WAIT_MS     = 2000       # first reconnect attempt after closing the port
RECONNECT_RETRY_MS  = 500
RECONNECT_TIMEOUT_S = 15.0
SHOW_TOOLBAR      = False
DEBUG_ECHO        = False

# --- negative-force handling ---
# Any force below this value is treated as zero. Set to 0.0 to clamp only
# strictly-negative values, or raise slightly (e.g. 0.005) to also swallow
# a bit of noise on the positive side.
CLAMP_BELOW_N   = 0.0
# --------------------------------------------------------------------------

CSV_HEADER = ["index", "device_ms", "elapsed_ms", "host_time", "force_N"]

def resource_path(relative_path):
    """Return correct path whether running as .py or PyInstaller .exe."""
    try:
        base_path = sys._MEIPASS   # PyInstaller --onefile temp folder
    except AttributeError:
        base_path = os.path.abspath(".")  # normal .py run
    return os.path.join(base_path, relative_path)


class ForceSensorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("FMA Force Sensor - v5 (2026_09)") 

        # --- window icon ---
        try:
             self.root.iconbitmap(resource_path("microforce.ico"))
        except tk.TclError:
            pass
    # -------------------

        self.ser = None
        self.port_map = {}

        # --- sample state ---
        self.sample_count = 0
        self.status_faults = 0
        self.i2c_faults = 0
        self.unparsed = 0
        self.board_resets = 0
        self.last_force = None

        # --- device time ---
        self.last_device_ms = None   # last raw millis() received
        self.dev_offset_ms = 0       # added after a reset to keep time continuous
        self.plot_t0_ms = None       # continuous device time at plot zero

        # --- plot buffers (trimmed by time, capped by MAX_BUFFER_POINTS) ---
        self.t_buffer = deque(maxlen=MAX_BUFFER_POINTS)
        self.f_buffer = deque(maxlen=MAX_BUFFER_POINTS)

        # --- rate measurement ---
        self.rate_mark_count = 0
        self.rate_mark_time = None
        self.measured_hz = 0.0

        # --- jobs ---
        self.poll_job = None
        self.redraw_job = None
        self.rate_job = None
        self.connect_job = None
        self.reconnect_job = None

        # --- board reset / reconnect ---
        self.reset_pending = None     # (port, baud) while a reset is in progress
        self.reconnect_deadline = 0.0
        self.keep_startup_lines = False  # True right after a reset: keep boot messages

        # --- reader thread ---
        self.rx_queue = queue.Queue()
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.skip_first_line = False

        # --- PC time anchors ---
        self.perf0 = None      # time.perf_counter() at streaming start
        self.wall0 = None      # datetime.now() at streaming start

        # --- recording state ---
        self.rec_file = None
        self.rec_writer = None
        self.rec_csv_path = None
        self.rec_target_path = None
        self.rec_rows = 0
        self.rec_last_flush = 0.0
        self.rec_armed_perf = None   # perf_counter() when Record was pressed
        self.rec_dev_t0 = None       # continuous device ms of first recorded row
        self.rec_first_sample = None # sample count at the first recorded row

        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------- UI -----------------------------------
    def _build_ui(self):
        bar = ttk.Frame(self.root, padding=(8, 6))
        bar.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(bar, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var,
                                       state="readonly", width=32)
        self.port_combo.pack(side=tk.LEFT, padx=(4, 10))

        ttk.Label(bar, text="Baud:").pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD)
        self.baud_combo = ttk.Combobox(bar, textvariable=self.baud_var,
                                       values=BAUD_CHOICES, width=9)
        self.baud_combo.pack(side=tk.LEFT, padx=(4, 10))

        self.refresh_btn = ttk.Button(bar, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=2)
        self.start_btn = ttk.Button(bar, text="Start", command=self.on_start)
        self.start_btn.pack(side=tk.LEFT, padx=2)
        self.stop_btn = ttk.Button(bar, text="Stop", command=self.on_stop,
                                   state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=2)
        self.clear_btn = ttk.Button(bar, text="Clear", command=self.on_clear)
        self.clear_btn.pack(side=tk.LEFT, padx=2)
        self.record_btn = ttk.Button(bar, text="Record", command=self.on_record)
        self.record_btn.pack(side=tk.LEFT, padx=2)
        self.reset_btn = ttk.Button(bar, text="Reset board",
                                    command=self.on_reset_board, state=tk.DISABLED)
        self.reset_btn.pack(side=tk.LEFT, padx=(12, 2))

        self.value_var = tk.StringVar(value="--- N")
        ttk.Label(bar, textvariable=self.value_var,
                  font=("Segoe UI", 16, "bold")).pack(side=tk.RIGHT)
        self.rec_var = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.rec_var, foreground="#b00000",
                  font=("Segoe UI", 9, "bold")).pack(side=tk.RIGHT, padx=(0, 14))

        bar2 = ttk.Frame(self.root, padding=(8, 0, 8, 6))
        bar2.pack(side=tk.TOP, fill=tk.X)

        self.rate_var = tk.StringVar(value="0 Hz")
        ttk.Label(bar2, textvariable=self.rate_var,
                  font=("Segoe UI", 10, "bold")).pack(side=tk.RIGHT)
        ttk.Label(bar2, text="Measured rate:").pack(side=tk.RIGHT, padx=(0, 4))

        self.fig = Figure(figsize=(10, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        (self.line,) = self.ax.plot([], [], lw=1.2, color="tab:blue")
        self.ax.axhline(0, color="gray", linestyle="--", linewidth=0.8)
        self.ax.set_xlabel("Device time (s)")
        self.ax.set_ylabel("Force (N)")
        self.ax.set_title("FMA Force Sensor - Live Reading")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, PLOT_SECONDS)
        self.ax.set_ylim(-0.05, 0.5)   # bottom slightly below zero
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        if SHOW_TOOLBAR:
            NavigationToolbar2Tk(self.canvas, self.root).update()

        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN,
                  anchor=tk.W, padding=(6, 3)).pack(side=tk.BOTTOM, fill=tk.X)

    # --------------------------- port handling ----------------------------
    def refresh_ports(self):
        previous = self.port_var.get()
        self.port_map.clear()
        labels = []
        for p in sorted(list_ports.comports(), key=lambda x: x.device):
            label = p.device
            self.port_map[label] = p.device
            labels.append(label)

        self.port_combo["values"] = labels
        if not labels:
            self.port_var.set("")
            self.status_var.set("No serial ports found. Plug the board in, then Refresh.")
            return

        if previous in labels:
            self.port_var.set(previous)
        else:
            ports = list_ports.comports()
            # Prefer an Arduino (USB vendor ID 0x2341), then any USB port.
            # Built-in ports such as COM1 have vid None, so they come last.
            preferred = next((p.device for p in ports if p.vid == 0x2341), None)
            if preferred is None:
                preferred = next((p.device for p in ports if p.vid is not None),
                                 labels[0])
            self.port_var.set(preferred)
        self.status_var.set("Found {} port(s).".format(len(labels)))

    # ---------------------------- connection ------------------------------
    def on_start(self):
        label = self.port_var.get()
        if not label:
            messagebox.showwarning("No port", "Select a COM port first.")
            return

        device = self.port_map.get(label, label)
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Bad baud rate",
                                 "Baud rate must be an integer, e.g. 115200.")
            return

        try:
            self.ser = serial.Serial(device, baud, timeout=SERIAL_TIMEOUT_S)
        except serial.SerialException as e:
            messagebox.showerror(
                "Could not open port",
                "Failed to open {} at {} baud.\n\n{}\n\n"
                "Usual causes: the Arduino IDE Serial Monitor or Docklight still "
                "has the port open, the board was unplugged, or the wrong port "
                "was selected.".format(device, baud, e),
            )
            self.ser = None
            return

        self._set_controls_running(True)
        self.status_var.set(
            "Opened {} @ {} baud. Waiting before reading...".format(device, baud)
        )
        self.connect_job = self.root.after(BOOT_DELAY_MS, self._begin_streaming)

    def _begin_streaming(self):
        self.connect_job = None
        if self.ser is None:
            return
        if self.keep_startup_lines:
            # Just after a reset the buffer holds only fresh boot messages
            # (banner, tare result), so keep them for the status bar.
            self.keep_startup_lines = False
        else:
            try:
                self.ser.reset_input_buffer()
            except (serial.SerialException, OSError) as e:
                self._fatal_serial_error(e)
                return

        # Anchor the PC wall clock to the high-resolution counter ONCE.
        # Every host_time = wall0 + (stamp - perf0).
        self.perf0 = time.perf_counter()
        self.wall0 = datetime.now()

        # Clearing the buffer can cut a line in half, so ignore the first one.
        self.skip_first_line = True

        self.rx_queue = queue.Queue()
        self.reader_stop = threading.Event()
        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            args=(self.ser, self.reader_stop, self.rx_queue),
            daemon=True,
        )
        self.reader_thread.start()

        self.rate_mark_count = self.sample_count
        self.rate_mark_time = time.monotonic()
        self.status_var.set("Reading...")
        self.poll_job = self.root.after(POLL_MS, self._poll)
        self.redraw_job = self.root.after(REDRAW_MS, self._redraw)
        self.rate_job = self.root.after(RATE_MS, self._update_rate)

    def _stop_reader(self):
        if self.reader_thread is not None:
            self.reader_stop.set()
            self.reader_thread.join(timeout=READER_JOIN_S)
            self.reader_thread = None

    def on_stop(self):
        for job in (self.poll_job, self.redraw_job, self.rate_job,
                    self.connect_job, self.reconnect_job):
            if job is not None:
                self.root.after_cancel(job)
        self.poll_job = self.redraw_job = self.rate_job = self.connect_job = None
        self.reconnect_job = None
        self.reset_pending = None      # Stop also cancels a reset in progress
        self.keep_startup_lines = False

        # Stop the reader first, then write any lines still in the queue,
        # so the last samples before Stop are not lost.
        self._stop_reader()
        self._drain_queue()

        if self.is_recording():
            self._stop_recording(announce=False)

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        self._set_controls_running(False)
        self.rate_var.set("0 Hz")
        self.status_var.set("Stopped. Port closed.")

    def _set_controls_running(self, running):
        state = tk.DISABLED if running else tk.NORMAL
        self.start_btn.config(state=state)
        self.refresh_btn.config(state=state)
        self.baud_combo.config(state=state if running else "normal")
        self.port_combo.config(state=tk.DISABLED if running else "readonly")
        self.stop_btn.config(state=tk.NORMAL if running else tk.DISABLED)
        self.reset_btn.config(state=tk.NORMAL if running else tk.DISABLED)

    def _fatal_serial_error(self, err):
        self.on_stop()
        messagebox.showerror("Serial error",
                             "The connection was lost:\n\n{}".format(err))

    # --------------------------- board commands ---------------------------
    def on_reset_board(self):
        if self.ser is None:
            messagebox.showwarning("Not connected", "Press Start first.")
            return

        message = ("The board will reboot and tare again. Do not touch the "
                   "sensor.\n\nThe USB connection will drop for a few seconds, "
                   "and the plot will be cleared.")
        if self.is_recording():
            message += "\n\nThe current recording will be stopped and saved."
        if not messagebox.askokcancel("Reset board", message):
            return

        if self.is_recording():
            self._stop_recording(announce=False)

        try:
            self.ser.write(b"RESET\n")
            self.ser.flush()
        except (serial.SerialException, OSError) as e:
            self._fatal_serial_error(e)
            return

        self.reset_pending = (self.ser.port, self.ser.baudrate)
        self.reset_btn.config(state=tk.DISABLED)
        self.status_var.set("Reset command sent...")
        # Give the Arduino time to receive the command before closing the port.
        self.reconnect_job = self.root.after(RESET_CLOSE_MS, self._reset_close_port)

    def _reset_close_port(self):
        # Can run from its timer OR from _poll (if USB dropped first).
        # Cancel the timer so it cannot run a second time later.
        if self.reconnect_job is not None:
            self.root.after_cancel(self.reconnect_job)
            self.reconnect_job = None
        pending = self.reset_pending
        if pending is None:
            return

        self.on_stop()                     # closes the port; clears reset_pending
        self.reset_pending = pending

        for widget in (self.start_btn, self.refresh_btn,
                       self.port_combo, self.baud_combo):
            widget.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)          # Stop cancels the reconnect

        self.reconnect_deadline = time.monotonic() + RECONNECT_TIMEOUT_S
        self.status_var.set(
            "Board is rebooting. Waiting for {} to come back... "
            "(Stop cancels)".format(pending[0]))
        self.reconnect_job = self.root.after(RESET_WAIT_MS, self._try_reconnect)

    def _try_reconnect(self):
        self.reconnect_job = None
        if self.reset_pending is None:
            return
        device, baud = self.reset_pending

        try:
            ser = serial.Serial(device, baud, timeout=SERIAL_TIMEOUT_S)
        except (serial.SerialException, OSError, ValueError):
            if time.monotonic() < self.reconnect_deadline:
                self.reconnect_job = self.root.after(RECONNECT_RETRY_MS,
                                                     self._try_reconnect)
                return
            self.reset_pending = None
            self._set_controls_running(False)
            self.refresh_ports()
            self.status_var.set("Reconnect failed.")
            messagebox.showerror(
                "Reconnect failed",
                "The board did not come back on {} within {:.0f} s.\n\n"
                "Press Refresh, check the port, then press Start. If the board "
                "is not responding, press its reset button."
                .format(device, RECONNECT_TIMEOUT_S))
            return

        self.reset_pending = None
        self.ser = ser
        self.keep_startup_lines = True     # fresh boot: show the tare result
        self.on_clear()
        self._reset_device_time()          # millis() restarted: not a surprise reset
        self._set_controls_running(True)
        self.status_var.set(
            "Reconnected to {}. Waiting for the board to tare...".format(device))
        self.connect_job = self.root.after(BOOT_DELAY_MS, self._begin_streaming)

    def _reset_device_time(self):
        self.last_device_ms = None
        self.dev_offset_ms = 0

    # ------------------------------ reading -------------------------------
    @staticmethod
    def _reader_loop(ser, stop_event, out_queue):
        """Runs in a background thread. Stamps each line on arrival.

        Only this thread touches the serial port while streaming. It never
        touches Tkinter; it only puts (kind, payload, stamp) into the queue.
        """
        pending = b""
        while not stop_event.is_set():
            try:
                # Returns at the first b"\n", after RX_LIMIT bytes, or when
                # SERIAL_TIMEOUT_S expires (possibly with a partial line).
                chunk = ser.read_until(b"\n", RX_LIMIT)
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                if not stop_event.is_set():
                    out_queue.put(("error", e, None))
                return

            if not chunk:
                continue

            stamp = time.perf_counter()     # taken right after the newline
            pending += chunk
            if pending.endswith(b"\n"):
                out_queue.put(("line", pending, stamp))
                pending = b""
            elif len(pending) > RX_LIMIT:
                out_queue.put(("overflow", None, None))
                pending = b""

    def _poll(self):
        if self.ser is None:
            return
        error = self._drain_queue()
        if error is not None:
            if self.reset_pending is not None:
                self._reset_close_port()     # expected: the board is rebooting
            else:
                self._fatal_serial_error(error)
            return
        self.poll_job = self.root.after(POLL_MS, self._poll)

    def _drain_queue(self):
        """Process every queued line. Returns the reader's exception, if any."""
        rows = []
        error = None
        while True:
            try:
                kind, payload, stamp = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "error":
                error = payload
                break
            if kind == "overflow":
                self.status_var.set(
                    "Receiving data but no line endings - check the baud rate."
                )
                continue

            if self.skip_first_line:
                self.skip_first_line = False
                continue

            text = payload.decode("ascii", errors="replace").strip()
            if DEBUG_ECHO:
                print("[rx +{:.3f} s] {!r}".format(stamp - self.perf0, text))

            result = self._handle_line(text)
            if result is None or not self.is_recording():
                continue
            if stamp < self.rec_armed_perf:
                continue            # arrived before Record was pressed

            sample, device_ms, dev_ms, force = result
            if self.rec_dev_t0 is None:
                self.rec_dev_t0 = dev_ms
                self.rec_first_sample = sample
            elapsed_ms = dev_ms - self.rec_dev_t0
            index = sample - self.rec_first_sample

            wall = self.wall0 + timedelta(seconds=stamp - self.perf0)
            host_time = wall.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            # Built explicitly so the order always matches CSV_HEADER.
            rows.append([index, device_ms, elapsed_ms, host_time, force])

        if rows:
            self._write_rows(rows)
        return error

    def _continuous_device_ms(self, device_ms):
        """Return device time that never goes backwards.

        If millis() goes backwards, the board was reset (or millis() wrapped
        after ~49.7 days). Shift the offset so time continues from the last
        value. The real length of the gap is unknown.
        """
        if self.last_device_ms is not None and device_ms < self.last_device_ms:
            self.board_resets += 1
            self.dev_offset_ms += self.last_device_ms - device_ms
            self.status_var.set(
                "Board reset detected (device time went backwards). "
                "Resets this session: {}".format(self.board_resets)
            )
        self.last_device_ms = device_ms
        return device_ms + self.dev_offset_ms

    def _handle_line(self, text):
        if not text:
            return None

        if text.startswith("#"):
            # Show Arduino info lines (tare result, errors) in the status bar.
            self.status_var.set("Arduino: " + text[1:].strip())
            return None

        parts = text.split(",")
        if len(parts) != 2:
            self.unparsed += 1
            return None

        try:
            device_ms = int(parts[0])
        except ValueError:
            self.unparsed += 1
            return None

        # Update device time for every valid line, including fault lines,
        # so a board reset is detected even while the sensor is faulting.
        dev_ms = self._continuous_device_ms(device_ms)

        value = parts[1].strip()
        if value == "-1":
            self.status_faults += 1
            return None
        if value == "-2":
            self.i2c_faults += 1
            return None

        try:
            force = float(value)
        except ValueError:
            self.unparsed += 1
            return None

        # --- negative clamp: anything below CLAMP_BELOW_N is treated as 0 ---
        if force < CLAMP_BELOW_N:
            force = 0.0

        return self._process_force(device_ms, dev_ms, force)

    def _process_force(self, device_ms, dev_ms, force):
        # No Python-side filtering. The Arduino already sent the value.
        # Negative values are already clamped in _handle_line().
        if self.plot_t0_ms is None:
            self.plot_t0_ms = dev_ms
        t = (dev_ms - self.plot_t0_ms) / 1000.0

        self.t_buffer.append(t)
        self.f_buffer.append(force)
        # Keep only the last PLOT_SECONDS of data in the plot buffers.
        while self.t_buffer and t - self.t_buffer[0] > PLOT_SECONDS:
            self.t_buffer.popleft()
            self.f_buffer.popleft()

        self.last_force = force
        sample = self.sample_count
        self.sample_count += 1

        return sample, device_ms, dev_ms, round(force, 5)

    # ----------------------------- recording ------------------------------
    def is_recording(self):
        return self.rec_file is not None

    def on_record(self):
        if self.is_recording():
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        suggested = "force_log_{}.csv".format(
            datetime.now().strftime("%Y%m%d_%H%M%S"))
        target = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save recording as",
            initialfile=suggested,
            defaultextension=".csv",
            filetypes=[("CSV file (recommended)", "*.csv"),
                       ("Excel workbook", "*.xlsx")],
        )
        if not target:
            return

        root_name, ext = os.path.splitext(target)
        csv_path = target if ext.lower() == ".csv" else root_name + ".csv"

        try:
            self.rec_file = open(csv_path, "w", newline="", encoding="utf-8")
            self.rec_writer = csv.writer(self.rec_file)
            self.rec_writer.writerow(CSV_HEADER)
        except OSError as e:
            self.rec_file = None
            self.rec_writer = None
            messagebox.showerror("Cannot write file",
                                 "Could not open the file for writing:\n\n{}".format(e))
            return

        self.rec_csv_path = csv_path
        self.rec_target_path = target
        self.rec_rows = 0
        self.rec_last_flush = time.monotonic()
        self.rec_armed_perf = time.perf_counter()
        self.rec_dev_t0 = None
        self.rec_first_sample = None

        self.record_btn.config(text="Stop Rec")
        self.rec_var.set("REC 0 rows")
        self.status_var.set("Recording to {}".format(os.path.basename(csv_path)))

        if self.ser is None:
            messagebox.showinfo(
                "Not connected",
                "Recording is armed, but the port is not open. Press Start to "
                "begin receiving data.",
            )

    def _write_rows(self, rows):
        try:
            self.rec_writer.writerows(rows)
        except (OSError, ValueError) as e:
            self._stop_recording(announce=False)
            messagebox.showerror("Recording stopped",
                                 "Writing failed:\n\n{}".format(e))
            return

        self.rec_rows += len(rows)
        now = time.monotonic()
        if now - self.rec_last_flush >= FLUSH_SECONDS:
            self.rec_file.flush()
            self.rec_last_flush = now
        self.rec_var.set("REC {} rows".format(self.rec_rows))

    def _stop_recording(self, announce=True):
        if not self.is_recording():
            return
        try:
            self.rec_file.flush()
            self.rec_file.close()
        except OSError:
            pass
        self.rec_file = None
        self.rec_writer = None

        self.record_btn.config(text="Record")
        self.rec_var.set("")

        final_path = self.rec_csv_path
        note = ""
        if self.rec_target_path.lower().endswith(".xlsx"):
            ok, result = self._convert_to_xlsx(self.rec_csv_path,
                                               self.rec_target_path)
            if ok:
                final_path = result
                note = "\n\nThe CSV was kept as a backup:\n{}".format(self.rec_csv_path)
            else:
                note = ("\n\nThe Excel conversion failed ({}), but your data is "
                        "safe in the CSV.".format(result))

        if announce:
            messagebox.showinfo(
                "Recording saved",
                "{} rows written to:\n{}{}\n\nDuring this session: "
                "{} status faults, {} I2C faults, {} unparsed lines, "
                "{} board resets."
                .format(self.rec_rows, final_path, note,
                        self.status_faults, self.i2c_faults, self.unparsed,
                        self.board_resets),
            )
        self.status_var.set("Recording saved: {} rows -> {}".format(
            self.rec_rows, os.path.basename(final_path)))

    @staticmethod
    def _convert_to_xlsx(csv_path, xlsx_path):
        try:
            from openpyxl import Workbook
        except ImportError:
            return False, "openpyxl is not installed"
        try:
            wb = Workbook()
            ws = wb.active
            ws.title = "Force data"
            with open(csv_path, newline="", encoding="utf-8") as f:
                for i, row in enumerate(csv.reader(f)):
                    if i == 0:
                        ws.append(row)
                        continue
                    typed = []
                    for value in row:
                        try:
                            typed.append(float(value) if "." in value else int(value))
                        except ValueError:
                            typed.append(value)
                    ws.append(typed)
            ws.freeze_panes = "A2"
            wb.save(xlsx_path)
            return True, xlsx_path
        except Exception as e:
            return False, str(e)

    # ------------------------- display / drawing --------------------------
    def _update_rate(self):
        now = time.monotonic()
        if self.rate_mark_time is not None:
            dt = now - self.rate_mark_time
            if dt > 0:
                self.measured_hz = (self.sample_count - self.rate_mark_count) / dt
                self.rate_var.set("{:.0f} Hz".format(self.measured_hz))
        self.rate_mark_count = self.sample_count
        self.rate_mark_time = now

        if self.ser is not None:
            self.rate_job = self.root.after(RATE_MS, self._update_rate)

    def _redraw(self):
        if self.last_force is not None:
            self.value_var.set("{:+.3f} N".format(self.last_force))

        if self.f_buffer:
            n = len(self.f_buffer)
            step = max(1, n // MAX_PLOT_POINTS)
            xs = list(self.t_buffer)[::step]
            ys = list(self.f_buffer)[::step]
            self.line.set_data(xs, ys)

            t_end = self.t_buffer[-1]
            self.ax.set_xlim(max(0.0, t_end - PLOT_SECONDS),
                             max(PLOT_SECONDS, t_end))

            # y-axis: always start at 0 (or below if there's real negative data,
            # but there won't be - we clamped them), grow to fit the data.
            y_max = max(ys) if ys else 0.1
            y_max = max(y_max, 0.1)            # always show at least 0..0.1
            self.ax.set_ylim(-0.02, y_max * 1.15)
            self.canvas.draw_idle()

        if self.ser is not None:
            self.redraw_job = self.root.after(REDRAW_MS, self._redraw)

    def on_clear(self):
        self.t_buffer.clear()
        self.f_buffer.clear()
        self.sample_count = 0
        self.status_faults = self.i2c_faults = self.unparsed = 0
        self.board_resets = 0
        self.plot_t0_ms = None
        self.last_force = None
        self.line.set_data([], [])
        self.ax.set_xlim(0, PLOT_SECONDS)
        self.ax.set_ylim(-0.02, 0.5)
        self.canvas.draw_idle()
        self.value_var.set("--- N")

    def on_close(self):
        if self.is_recording():
            if not messagebox.askokcancel(
                "Recording in progress",
                "A recording is still running. Close anyway? The file will be "
                "saved before exit.",
            ):
                return
        self.on_stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    ForceSensorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
