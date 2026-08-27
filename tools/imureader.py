#!/usr/bin/env python3
"""
Read BMI323 IMU data streamed by the STM32F407 over USB CDC.

The firmware prints CSV lines:
    time_ms,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,temp_c
(acc in g, gyr in deg/s, temp in deg C)

Usage:
    python3 tools/imureader.py                 # stream to console (auto-detect port)
    python3 tools/imureader.py -o data.csv     # also log to CSV file
    python3 tools/imureader.py -n 1000         # read 1000 samples then exit
    python3 tools/imureader.py --plot          # live plot (needs matplotlib)
    python3 tools/imureader.py --port /dev/ttyACM0
"""

import argparse
import csv
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed. Run: pip install pyserial")

DEFAULT_VID_PID = "0483:5740"  # STMicroelectronics Virtual COM Port


def detect_port():
    """Find the STM32 CDC port, preferring /dev/serial/by-id matches."""
    candidates = []

    # Symlinks in /dev/serial/by-id are the most reliable identifiers.
    for path in glob.glob("/dev/serial/by-id/*"):
        name = os.path.basename(path)
        if any(k in name for k in ("STMicroelectronics", "CDC", "F407", "stm32")):
            candidates.append(os.path.realpath(path))

    if candidates:
        return candidates[0]

    # Fallback: any ACM port.
    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[0]

    return None


def parse_line(line):
    """Parse a CSV data line. Returns list of floats or None."""
    line = line.strip()
    if not line or line.startswith("#") or line.startswith("STM32") or \
       line.startswith("format") or line.startswith("BMI323") or \
       line == "read_error":
        return None
    parts = line.split(",")
    if len(parts) != 8:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser(description="Read BMI323 IMU data from STM32F407")
    ap.add_argument("--port", "-p", default=None, help="serial port (default: auto-detect)")
    ap.add_argument("--baud", "-b", type=int, default=115200, help="baud rate (USB CDC ignores this)")
    ap.add_argument("--output", "-o", default=None, help="log data to CSV file")
    ap.add_argument("--count", "-n", type=int, default=0, help="number of samples (0 = infinite)")
    ap.add_argument("--plot", action="store_true", help="show a live plot")
    ap.add_argument("--quiet", "-q", action="store_true", help="suppress console output")
    args = ap.parse_args()

    port = args.port or detect_port()
    if not port:
        sys.exit("No STM32 CDC port found. Is the board plugged in and flashed?")

    print(f"Opening {port} @ {args.baud} ...", file=sys.stderr)
    ser = serial.Serial(port, args.baud, timeout=1)
    # Flush any stale bytes from a previous session.
    ser.reset_input_buffer()

    out_fh = None
    writer = None
    if args.output:
        out_fh = open(args.output, "w", newline="")
        writer = csv.writer(out_fh)
        writer.writerow(["time_ms", "acc_x", "acc_y", "acc_z",
                         "gyr_x", "gyr_y", "gyr_z", "temp_c"])

    plotter = None
    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            sys.exit("matplotlib is not installed. Run: pip install matplotlib")

        class LivePlot:
            def __init__(self):
                plt.ion()
                self.fig, (self.a1, self.a2) = plt.subplots(2, 1, sharex=True)
                self.fig.suptitle("BMI323 IMU")
                self.t, self.ax, self.ay, self.az = [], [], [], []
                self.gx, self.gy, self.gz = [], [], [], []
                self.lines1 = self.a1.plot([], [], [], [], [], [])[0]
                self.a1.set_ylabel("Accel (g)")
                self.a1.legend(["x", "y", "z"], loc="upper right")
                self.a2.set_ylabel("Gyro (dps)")
                self.a2.set_xlabel("time (ms)")

            def update(self, row):
                t, ax, ay, az, gx, gy, gz, _ = row
                self.t.append(t); self.ax.append(ax); self.ay.append(ay); self.az.append(az)
                self.gx.append(gx); self.gy.append(gy); self.gz.append(gz)
                self.t = self.t[-500:]
                self.ax = self.ax[-500:]; self.ay = self.ay[-500:]; self.az = self.az[-500:]
                self.gx = self.gx[-500:]; self.gy = self.gy[-500:]; self.gz = self.gz[-500:]
                self.a1.clear(); self.a2.clear()
                self.a1.plot(self.t, self.ax, self.t, self.ay, self.t, self.az)
                self.a2.plot(self.t, self.gx, self.t, self.gy, self.t, self.gz)
                self.a1.set_ylabel("Accel (g)"); self.a2.set_ylabel("Gyro (dps)")
                self.a2.set_xlabel("time (ms)")
                plt.pause(0.001)

        plotter = LivePlot()

    count = 0
    header_printed = False

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace")
            if not line:
                continue
            row = parse_line(line)
            if row is None:
                # Show firmware banner/status lines once so the user sees init result.
                stripped = line.strip()
                if stripped and not args.quiet and not header_printed:
                    print(f"[{stripped}]", file=sys.stderr)
                continue

            t, ax, ay, az, gx, gy, gz, temp = row

            if writer:
                writer.writerow(row)

            if plotter:
                plotter.update(row)

            if not args.quiet:
                if not header_printed:
                    print(f"{'time(ms)':>9} {'acc_x':>9} {'acc_y':>9} {'acc_z':>9} "
                          f"{'gyr_x':>9} {'gyr_y':>9} {'gyr_z':>9} {'tempC':>7}")
                    header_printed = True
                print(f"{t:9.0f} {ax:9.4f} {ay:9.4f} {az:9.4f} "
                      f"{gx:9.2f} {gy:9.2f} {gz:9.2f} {temp:7.2f}")

            count += 1
            if args.count and count >= args.count:
                break

    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
    finally:
        ser.close()
        if out_fh:
            out_fh.close()
            print(f"Logged {count} samples to {args.output}", file=sys.stderr)
        else:
            print(f"Read {count} samples.", file=sys.stderr)


if __name__ == "__main__":
    main()
