#!/usr/bin/env python3
"""
Live monitor for the STM32F407 BMP581 + BMI323 + uBlox streamer.

Reads tagged CSV over USB CDC and shows:
  - BMI323 connection status
  - BMP581: pressure / temperature / altitude
  - BMI323: accel (g) and gyro (deg/s)
  - Attitude: roll / pitch / yaw (complementary filter) + artificial horizon
  - uBlox GNSS: position / altitude / satellites / fix

Usage:
    python3 tools/gui.py                 # auto-detect port
    python3 tools/gui.py --port /dev/ttyACM0
"""

import argparse
import glob
import math
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
        root.title("BMI323 + BMP581 + GNSS Monitor")
        root.geometry("500x800")
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

        # --- ALS section ---
        als_frame = ttk.LabelFrame(root, text="ALS31300 (Hall)")
        als_frame.pack(fill="x", padx=16, pady=6)

        self.als_var = tk.StringVar(value="--, --, --")
        self.als_temp_var = tk.StringVar(value="-- °C")
        self.als_hdg_var = tk.StringVar(value="-- °")

        self._row(als_frame, "Field X,Y,Z", self.als_var)
        self._row(als_frame, "Temp", self.als_temp_var)
        self._row(als_frame, "Heading", self.als_hdg_var)

        # --- GPS section ---
        gps_frame = ttk.LabelFrame(root, text="uBlox GNSS (NMEA)")
        gps_frame.pack(fill="x", padx=16, pady=6)

        self.pos_var = tk.StringVar(value="--, --")
        self.gps_alt_var = tk.StringVar(value="-- m")
        self.sats_var = tk.StringVar(value="--")
        self.fix_var = tk.StringVar(value="--")
        self.gps_time_var = tk.StringVar(value="--")
        self.gps_speed_var = tk.StringVar(value="--")

        self._row(gps_frame, "Position", self.pos_var)
        self._row(gps_frame, "GPS Altitude", self.gps_alt_var)
        self._row(gps_frame, "Satellites", self.sats_var)
        self._row(gps_frame, "Fix", self.fix_var)
        self._row(gps_frame, "Time (UTC)", self.gps_time_var)
        self._row(gps_frame, "Speed", self.gps_speed_var)

        # --- Attitude section ---
        att_frame = ttk.LabelFrame(root, text="Attitude (complementary filter)")
        att_frame.pack(fill="x", padx=16, pady=6)

        self.roll_var = tk.StringVar(value="-- °")
        self.pitch_var = tk.StringVar(value="-- °")
        self.yaw_var = tk.StringVar(value="-- °")

        self._row(att_frame, "Roll", self.roll_var)
        self._row(att_frame, "Pitch", self.pitch_var)
        self._row(att_frame, "Yaw", self.yaw_var)

        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._yaw_deg = 0.0

        self.horizon = tk.Canvas(att_frame, width=240, height=150,
                                 bg="#000000", highlightthickness=0)
        self.horizon.pack(pady=6)
        self._draw_horizon()

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

    @staticmethod
    def _fix_label(fix):
        return {0: "No fix", 1: "GPS fix", 2: "DGPS", 4: "RTK fixed",
                5: "RTK float", 6: "Dead reckoning"}.get(fix, f"fix {fix}")

    def _draw_horizon(self):
        c = self.horizon
        c.delete("all")
        w = int(c["width"]); h = int(c["height"])
        cx, cy = w / 2.0, h / 2.0
        roll = self._roll_deg
        pitch = self._pitch_deg

        # Pitch: positive = nose up -> horizon drops (more sky visible).
        horizon_y = cy + pitch * 2.0   # pixels

        # Roll: rotate the horizon line about the canvas centre.
        rad = math.radians(roll)
        cosr, sinr = math.cos(rad), math.sin(rad)
        ext = w + h   # endpoints well outside the canvas

        def rot(x, y):
            dx, dy = x - cx, y - cy
            return (cx + dx * cosr - dy * sinr, cy + dx * sinr + dy * cosr)

        x0, y0 = rot(cx - ext, horizon_y)
        x1, y1 = rot(cx + ext, horizon_y)

        c.create_rectangle(0, 0, w, h, fill="#3b82f6", outline="")             # sky
        c.create_polygon(x0, y0, x1, y1, w + ext, h + ext, -ext, h + ext,
                         fill="#7a4b22", outline="")                            # ground
        c.create_line(x0, y0, x1, y1, fill="#ffffff", width=2)                  # horizon
        c.create_line(cx - 20, cy, cx + 20, cy, fill="#fbbf24", width=2)        # wings
        c.create_line(cx, cy - 12, cx, cy + 12, fill="#fbbf24", width=2)        # nose/tail
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#fbbf24", outline="")

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

        elif tag == "ALS" and len(parts) == 6:
            try:
                x = int(parts[1]); y = int(parts[2]); z = int(parts[3])
                t = float(parts[4]); hdg = float(parts[5])
            except ValueError:
                return
            self.als_var.set(f"{x:+d}, {y:+d}, {z:+d}")
            self.als_temp_var.set(f"{t:.1f} °C")
            self.als_hdg_var.set(f"{hdg:.1f} °")

        elif tag == "ATT" and len(parts) == 4:
            try:
                self._roll_deg = float(parts[1])
                self._pitch_deg = float(parts[2])
                self._yaw_deg = float(parts[3])
            except ValueError:
                return
            self.roll_var.set(f"{self._roll_deg:+.1f} °")
            self.pitch_var.set(f"{self._pitch_deg:+.1f} °")
            self.yaw_var.set(f"{self._yaw_deg:+.1f} °")
            self._draw_horizon()

        elif tag == "GPS" and len(parts) == 8:
            try:
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3])
                sats = int(parts[4])
                fix = int(parts[5])
                t = parts[6]
                spd = float(parts[7])
            except ValueError:
                return
            self.pos_var.set(f"{lat:.6f}, {lon:.6f}")
            self.gps_alt_var.set(f"{alt:.1f} m")
            self.sats_var.set(str(sats))
            self.fix_var.set(self._fix_label(fix))
            self.gps_time_var.set(t)
            self.gps_speed_var.set(f"{spd:.1f} km/h")

        elif tag == "GPS_STAT" and len(parts) == 5:
            try:
                fix = int(parts[1])
                sats = int(parts[2])
                t = parts[3]
                spd = float(parts[4])
            except ValueError:
                return
            self.sats_var.set(str(sats))
            self.fix_var.set(self._fix_label(fix))
            self.gps_time_var.set(t)
            self.gps_speed_var.set(f"{spd:.1f} km/h")

        elif tag == "BMI_STATUS" and len(parts) == 2:
            self._connected = (parts[1] == "1")
            if self._connected:
                self.status_var.set("BMI323: CONNECTED")
                self.status_lbl.configure(fg=self.green)
            else:
                self.status_var.set("BMI323: NOT FOUND")
                self.status_lbl.configure(fg=self.red)


def main():
    ap = argparse.ArgumentParser(description="BMI323 + BMP581 + GNSS monitor GUI")
    ap.add_argument("--port", "-p", default=None, help="serial port (default: auto-detect)")
    args = ap.parse_args()

    port = args.port or detect_port()

    root = tk.Tk()
    MonitorApp(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
