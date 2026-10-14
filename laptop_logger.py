#!/usr/bin/env python3
"""
============================================================
 Multi-Source Energy-Harvesting Autonomous Sensor Node
 Laptop Logger — The Copperbelt University  |  Group Seven
============================================================
 Reads telemetry from the Arduino over USB-UART and appends
 timestamped CSV records to a log file for analysis.

 Usage:
   python3 laptop_logger.py --port COM3 --baud 9600
   python3 laptop_logger.py --port /dev/ttyUSB0 --baud 9600

 CSV Output:
   pc_timestamp, device_id, sequence, temperature_C,
   humidity_pct, Vcap_mV, AC_Voltage_V, energy_mode
============================================================
"""

import serial
import serial.tools.list_ports
import csv
import argparse
import datetime
import os
import sys
import time


LOG_FILENAME = "sensor_log.csv"
CSV_FIELDS   = ["pc_timestamp", "device_id", "sequence",
                "temperature_C", "humidity_pct", "Vcap_mV",
                "AC_Voltage_V", "energy_mode"]


def list_ports():
    """Print all available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
    else:
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device}  —  {p.description}")


def run_logger(port: str, baud: int, log_file: str):
    """Main logging loop."""
    print(f"Connecting to {port} at {baud} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=2)
    except serial.SerialException as e:
        print(f"ERROR: Could not open port {port}: {e}")
        sys.exit(1)

    time.sleep(2)  # Allow Arduino to reset after connection

    # Open CSV file (append mode so data persists across runs)
    file_exists = os.path.exists(log_file)
    csvfile = open(log_file, "a", newline="")
    writer  = csv.DictWriter(csvfile, fieldnames=CSV_FIELDS)

    if not file_exists:
        writer.writeheader()

    print(f"Logging to: {log_file}")
    print("Press Ctrl+C to stop.\n")
    print(f"{'Timestamp':<26} | {'ID':<8} | {'Seq':>6} | {'Temp':>6} | {'Hum':>6} | {'Vcap':>6} | {'ACrms':>7} | Mode")
    print("-" * 92)

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()

            # Skip header / comment lines
            if not line or line.startswith("=") or line.startswith("pc_") or line.startswith("SYSTEM"):
                if line.startswith("SYSTEM"):
                    print(f"  [NODE STATUS] {line}")
                continue

            # Skip sensor error records
            if line.startswith("SENSOR_ERR"):
                ts = datetime.datetime.now().isoformat(timespec="seconds")
                print(f"  [SENSOR ERROR @ {ts}]  {line}")
                continue

            parts = line.split(",")
            if len(parts) != 7:
                continue  # Malformed line

            ts = datetime.datetime.now().isoformat(timespec="milliseconds")

            row = {
                "pc_timestamp": ts,
                "device_id":    parts[0].strip(),
                "sequence":     parts[1].strip(),
                "temperature_C":parts[2].strip(),
                "humidity_pct": parts[3].strip(),
                "Vcap_mV":      parts[4].strip(),
                "AC_Voltage_V": parts[5].strip(),
                "energy_mode":  parts[6].strip(),
            }
            writer.writerow(row)
            csvfile.flush()

            # Console output
            mode_color = {
                "LOW":       "\033[93m",   # Yellow
                "NORMAL":    "\033[92m",   # Green
                "HIGH":      "\033[96m",   # Cyan
                "DEEP_SLEEP":"\033[91m",   # Red
            }.get(row["energy_mode"], "\033[0m")
            reset = "\033[0m"

            print(
                f"  {ts:<26} | {row['device_id']:<8} | {row['sequence']:>6} | "
                f"{row['temperature_C']:>5}°C | {row['humidity_pct']:>5}% | "
                f"{row['Vcap_mV']:>5}mV | {row['AC_Voltage_V']:>5}V | "
                f"{mode_color}{row['energy_mode']}{reset}"
            )

    except KeyboardInterrupt:
        print("\n\nLogging stopped by user.")
    finally:
        ser.close()
        csvfile.close()
        print(f"Data saved to: {log_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Laptop logger for Multi-Source Energy-Harvesting Sensor Node"
    )
    parser.add_argument("--port",  type=str, default=None,
                        help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud",  type=int, default=9600,
                        help="Baud rate (default: 9600)")
    parser.add_argument("--log",   type=str, default=LOG_FILENAME,
                        help="Output CSV file path")
    parser.add_argument("--list",  action="store_true",
                        help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if args.port is None:
        print("ERROR: Please specify --port  (use --list to see available ports)")
        sys.exit(1)

    run_logger(args.port, args.baud, args.log)


if __name__ == "__main__":
    main()
