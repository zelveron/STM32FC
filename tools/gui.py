#!/usr/bin/env python3
"""
Live monitor for the STM32F407 BMP581 + BMI323 streamer.

Reads tagged CSV over USB CDC and shows:
  - BMI323 connection status
  - BMP581: pressure / temperature / altitude
  - BMI323: accel (g) and gyro (deg/s)

Usage:
    python3 tools/gui.py                 # auto-detect port
    python3 tools/gui.py --port /dev/ttyACM0
"""

import argparse
import glob
import os
import queue
import sys
import threading
import time

import serial
import tkinter as tk
from tkinter import ttk


def detect_port():
    for path in glob.glob("/dev/serial/by-id/*"):
        name = os.path.basename(path)
        if any(k in name for k in ("STMicroelectronics", "CDC", "F407", "stm32")):
            return os.path.realpath(path)
    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[0]
    return "/dev/ttyACM0"  # fallback; the reader keeps retrying until it appears


class MonitorApp:
    def __init__(self, root, port):
        self.root = root
        self.port = port
        self.q = queue.Queue()
        self._last_data = time.time()
        root.title("BMI323 + BMP581 Monitor")
        root.geometry("480x360")
        root.configure(bg="#1e1e1e")

        bg = "#1e1e1e"
        fg = "#e0e0e0"
        green = "#2ecc71"
        red = "#e74c3c"

        style = ttk.Style()
        style.configure("TLabel", background=bg, foreground=fg, font=("Helvetica", 12))
        style.configure("Header.TLabel", font=("Helvetica", 14, "bold"))
        style.configure("Value.TLabel", font=("Helvetica", 16, "bold"), foreground="#ffffff")
        style.configure("TFrame", background=bg)

        # --- BMI status header ---
        self.status_var = tk.StringVar(value="BMI323: waiting...")
        status_lbl = tk.Label(root, textvariable=self.status_var, font=("Helvetica", 14, "bold"),
                              bg=bg, fg=red)
        status_lbl.pack(pady=(12, 8))

        # --- BMP section ---
        bmp_frame = ttk.LabelFrame(root, text="BMP581 (pressure)")
        bmp_frame.pack(fill="x", padx=16, pady=6)

        self.pressure_var = tk.StringVar(value="-- hPa")
        self.temp_var = tk.StringVar(value="-- °C")
        self.alt_var = tk.StringVar(value="-- m")

        self._row(bmp_frame, "Pressure", self.pressure_var)
        self._row(bmp_frame, "Temperature", self.temp_var)
        self._row(bmp_frame, "Altitude", self.alt_var)

        # --- BMI section ---
        bmi_frame = ttk.LabelFrame(root, text="BMI323 (IMU)")
        bmi_frame.pack(fill="x", padx=16, pady=6)

        self.acc_var = tk.StringVar(value="--, --, -- g")
        self.gyr_var = tk.StringVar(value="--, --, -- dps")

        self._row(bmi_frame, "Accel", self.acc_var)
        self._row(bmi_frame, "Gyro", self.gyr_var)

        # --- footer ---
        self.footer_var = tk.StringVar(value=f"connecting to {port} ...")
        ttk.Label(root, textvariable=self.footer_var, foreground="#888888").pack(side="bottom", pady=6)

        self._connected = False
        self.status_lbl = status_lbl
        self.red = red
        self.green = green

        self.reader = threading.Thread(target=self._read_loop, args=(port,), daemon=True)
        self.reader.start()
        root.after(50, self._poll)

    def _row(self, parent, label, var):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=8, pady=3)
        ttk.Label(frame, text=label, width=14, anchor="w").pack(side="left")
        ttk.Label(frame, textvariable=var, style="Value.TLabel", anchor="e").pack(side="right", fill="x", expand=True)

    def _read_loop(self, port):
        """Reconnect-forever serial reader. Pushes (kind, payload) into the queue."""
        while True:
            try:
                ser = serial.Serial(port, 115200, timeout=1)
            except Exception as exc:
                self.q.put(("status", f"cannot open {port}: {exc}"))
                time.sleep(2)
                continue

            self.q.put(("status", f"connected: {port}"))
            try:
                ser.reset_input_buffer()
                for raw in ser:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        self.q.put(("line", line))
            except Exception as exc:
                self.q.put(("status", f"disconnected: {exc}"))
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
            time.sleep(1)

    def _poll(self):
        try:
            while True:
                kind, val = self.q.get_nowait()
                if kind == "line":
                    self._last_data = time.time()
                    self._handle_line(val)
                elif kind == "status":
                    self.footer_var.set(val)
        except queue.Empty:
            pass

        # Data watchdog: if the stream stalls, surface it instead of freezing.
        if time.time() - self._last_data > 3:
            self.footer_var.set("no data — reconnecting...")

        self.root.after(50, self._poll)

    def _handle_line(self, line):
        parts = line.split(",")
        if not parts:
            return
        tag = parts[0]

        if tag == "BMP" and len(parts) == 4:
            try:
                p = float(parts[1])
                t = float(parts[2])
                a = float(parts[3])
            except ValueError:
                return
            self.pressure_var.set(f"{p:.3f} hPa")
            self.temp_var.set(f"{t:.2f} °C")
            self.alt_var.set(f"{a:.2f} m")

        elif tag == "BMI" and len(parts) == 7:
            try:
                vals = [float(x) for x in parts[1:]]
            except ValueError:
                return
            self.acc_var.set(f"{vals[0]:+.4f}, {vals[1]:+.4f}, {vals[2]:+.4f} g")
            self.gyr_var.set(f"{vals[3]:+7.2f}, {vals[4]:+7.2f}, {vals[5]:+7.2f} dps")

        elif tag == "BMI_STATUS" and len(parts) == 2:
            self._connected = (parts[1] == "1")
            if self._connected:
                self.status_var.set("BMI323: CONNECTED")
                self.status_lbl.configure(fg=self.green)
            else:
                self.status_var.set("BMI323: NOT FOUND")
                self.status_lbl.configure(fg=self.red)


def main():
    ap = argparse.ArgumentParser(description="BMI323 + BMP581 monitor GUI")
    ap.add_argument("--port", "-p", default=None, help="serial port (default: auto-detect)")
    args = ap.parse_args()

    port = args.port or detect_port()

    root = tk.Tk()
    MonitorApp(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
