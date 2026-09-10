#!/usr/bin/env python3
"""
imu_uart.py

Reads the LSM6DSO32 CSV stream off the Nucleo's ST-LINK virtual COM port.

The firmware prints a header line, then one row per sample at 104 Hz:
    <ms_since_start>,<ax_mg>,<ay_mg>,<az_mg>,<gx_mdps>,<gy_mdps>,<gz_mdps>

Rows are printed as they arrive, and optionally appended to a CSV file.

Usage:
    python imu_uart.py --port COM5
    python imu_uart.py --port /dev/ttyACM0 --out run.csv
"""

import argparse
import csv
import serial
from serial.tools import list_ports

# ======================================================================
# Configuration
# ======================================================================

TIMEOUT_S = 2
MCU_HEADER = "t_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps"
COLUMNS = ("Time (s)", "ax (g)", "ay (g)", "az (g)","gx (dps)", "gy (dps)", "gz (dps)")
COL_W = 11

def print_header():
    """Print the column header and its rule."""
    rule = "=" * (len(COLUMNS) * (COL_W + 1) - 1)
    print(rule)
    print(" ".join(f"{name:>{COL_W}}" for name in COLUMNS))
    print(rule)
    

def scale_row(t_ms, ax, ay, az, gx, gy, gz):
    """Convert one raw sample to the units in COLUMNS: s, g, and dps."""
    return (
        t_ms / 1000.0,  # ms -> s
        ax / 1000.0,    # mg -> g
        ay / 1000.0,
        az / 1000.0,
        gx / 1000.0,    # mdps -> dps
        gy / 1000.0,
        gz / 1000.0,
    )


def format_row(values):
    """Format one scaled sample as a right-aligned row."""
    return " ".join(f"{v:>{COL_W}.3f}" for v in values)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("-p", "--port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="must match huart2.Init.BaudRate in Core/Src/usart.c")
    parser.add_argument("--out", help="append samples to this CSV file as well")

    args = parser.parse_args()

    if args.list:
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found")
        for port in ports:
            print(f"{port.device:<20} {port.description}")
        return

    if not args.port:
        parser.error("--port is required (use --list to find it)")

    # ======================================================================
    # Stream Samples
    # ======================================================================

    ser = serial.Serial(args.port, args.baud, timeout=TIMEOUT_S)

    out_file = open(args.out, "a", newline="") if args.out else None
    writer = None
    if out_file is not None:
        writer = csv.writer(out_file)
        if out_file.tell() == 0:
            writer.writerow(COLUMNS)

    print(f"Reading from {args.port} at {args.baud} baud. Ctrl-C to stop.")
    if args.out:
        print(f"Logging to {args.out}.")
    print()
    print_header()

    try:
        while True:
            line = ser.readline().decode("ascii", errors="ignore").strip()

            if not line or line == MCU_HEADER:
                continue    # timeout, or the MCU's own CSV header after a reset

            fields = line.split(",")
            if len(fields) != len(COLUMNS):
                continue    # partial line, e.g. connecting mid-sample

            try:
                t_ms, ax, ay, az, gx, gy, gz = (int(f) for f in fields)
            except ValueError:
                continue    # malformed line

            values = scale_row(t_ms, ax, ay, az, gx, gy, gz)
            print(format_row(values))

            if writer is not None:
                writer.writerow(f"{v:.3f}" for v in values)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if out_file is not None:
            out_file.close()


if __name__=="__main__":
    main()