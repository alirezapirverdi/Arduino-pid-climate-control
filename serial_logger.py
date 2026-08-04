#!/usr/bin/env python3
"""
serial_logger.py — logs and live-plots the climate-control CSV telemetry.

Usage:
    pip install pyserial matplotlib
    python serial_logger.py --port /dev/ttyACM0 --baud 9600

The sketch prints one CSV line every ~2s when logging is ON (default):
    millis,tempC,humRH,setT,setH,Kp,Ki,Kd,heaterPWM,fan,humidifier,dehumid,fault

This script:
  1. Appends every valid line to a timestamped CSV file in ./logs/
  2. Live-plots temperature vs. setpoint and heater PWM output, which
     is exactly what you want for a step-response plot in a report.

Press Ctrl+C to stop; the CSV file is preserved.
"""

import argparse
import csv
import os
import time
from collections import deque
from datetime import datetime

import serial
import matplotlib.pyplot as plt

COLUMNS = [
    "millis", "tempC", "humRH", "setT", "setH",
    "Kp", "Ki", "Kd", "heaterPWM", "fan", "humidifier", "dehumid", "fault",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyACM0 or COM3")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--window", type=int, default=150,
                     help="number of most recent samples shown on the live plot")
    args = ap.parse_args()

    os.makedirs("logs", exist_ok=True)
    log_path = os.path.join(
        "logs", f"climate_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # allow the Uno to reset after the port opens

    t_buf, temp_buf, set_buf, pwm_buf = (deque(maxlen=args.window) for _ in range(4))

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))

    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        print(f"Logging to {log_path}. Ctrl+C to stop.")

        t0 = None
        try:
            while True:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw or raw.count(",") != len(COLUMNS) - 1:
                    continue  # skip banners/help text/partial lines
                parts = raw.split(",")
                try:
                    values = [float(p) for p in parts]
                except ValueError:
                    continue

                writer.writerow(parts)
                f.flush()

                ms = values[0]
                if t0 is None:
                    t0 = ms
                t_sec = (ms - t0) / 1000.0

                t_buf.append(t_sec)
                temp_buf.append(values[1])
                set_buf.append(values[3])
                pwm_buf.append(values[8])

                ax1.cla()
                ax1.plot(t_buf, temp_buf, label="Temperature (C)")
                ax1.plot(t_buf, set_buf, "--", label="Setpoint (C)")
                ax1.set_ylabel("Temp (C)")
                ax1.legend(loc="upper left")
                ax1.grid(True)

                ax2.cla()
                ax2.plot(t_buf, pwm_buf, color="tab:red", label="Heater PWM")
                ax2.set_ylabel("PWM (0-255)")
                ax2.set_xlabel("Time (s)")
                ax2.legend(loc="upper left")
                ax2.grid(True)

                plt.pause(0.01)
        except KeyboardInterrupt:
            print(f"\nStopped. Log saved at {log_path}")


if __name__ == "__main__":
    main()
