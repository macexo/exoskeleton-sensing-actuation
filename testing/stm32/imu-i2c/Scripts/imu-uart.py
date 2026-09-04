#!/usr/bin/env python3
"""
imu-uart.py

UART data streamer for STM32 LSM6DSO32 sensor measurements.

Line format from the MCU: <ms_since_start>,<ax_mg>,<ay_mg>,<az_mg>,<gx_mdps>,<gy_mdps>,<gz_mdps>

The MCU sends a CSV header line once at reset and then streams samples freely,
so this script just reads until interrupted with Ctrl-C.
"""

import argparse
import serial
from serial.tools import list_ports

# ======================================================================
# Configuration
# ======================================================================

TIMEOUT_S = 2

MCU_HEADER = "t_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps"

COLUMNS = ("Time (s)", "ax (g)", "ay (g)", "az (g)",
           "gx (dps)", "gy (dps)", "gz (dps)")
COL_W = 11 # char width for each column

def print_header():
    """Print the column header and its rule."""
    rule = "=" * (len(COLUMNS) * (COL_W + 1) - 1)
    print(rule)
    print(" ".join(f"{name:>{COL_W}}" for name in COLUMNS))
    print(rule)


def format_row(t_ms, ax, ay, az, gx, gy, gz):
    """Format one raw sample as a right-aligned row in g and dps."""
    values = (
        t_ms / 1000.0,  # ms -> s
        ax / 1000.0,    # mg -> g
        ay / 1000.0,
        az / 1000.0,
        gx / 1000.0,    # mdps -> dps
        gy / 1000.0,
        gz / 1000.0,
    )
    return " ".join(f"{v:>{COL_W}.3f}" for v in values)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("-p", "--port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="must match huart2.Init.BaudRate in Core/Src/usart.c")

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

    print(f"Reading from {args.port} at {args.baud} baud. Ctrl-C to stop.\n")
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

            print(format_row(t_ms, ax, ay, az, gx, gy, gz))

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__=="__main__":
    main()