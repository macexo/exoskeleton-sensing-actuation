#!/usr/bin/env python3
"""
imu_can.py

Reads the LSM6DSO32 stream off the CAN bus through Linux SocketCAN.

The firmware sends a pair of 8-byte frames per sample at 104 Hz, little endian,
with the axes as raw signed sensor counts:
    0x065: <uint16 sequence>,<int16 ax>,<int16 ay>,<int16 az>
    0x066: <uint16 sequence>,<int16 gx>,<int16 gy>,<int16 gz>

Frames are paired by sequence. They carry no MCU timestamp, so Time (s) is the
host receive time since this script started. Missing counts the sample pairs
lost so far.

Rows are printed as they arrive, and optionally appended to a CSV file.

Usage:
    python3 imu_can.py --channel can1
    python3 imu_can.py --channel can1 --out run.csv
"""

import argparse
import csv
import socket
import struct
import time

# ======================================================================
# Configuration
# ======================================================================

TIMEOUT_S = 1
PAIR_EXPIRY_S = 0.5             # incomplete pairs older than this are discarded
MAX_PENDING = 64                # ceiling on incomplete pairs held at once
ACCEL_G_PER_COUNT = 0.000488    # must match the +/-16 g range in Core/Src/main.c
GYRO_DPS_PER_COUNT = 0.070      # must match the +/-2000 dps range in Core/Src/main.c
CAN_FRAME_FORMAT = "=IB3x8s"    # SocketCAN struct can_frame: ID, DLC, padding, data
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
PAYLOAD_FORMAT = "<Hhhh"        # sequence, x, y, z
FILTER_MASK = 0xC00007FF        # standard data frames only: no extended IDs or remote requests
ARPHRD_CAN = "280"              # /sys/class/net/<name>/type of a CAN interface
COLUMNS = ("Time (s)", "Seq", "ax (g)", "ay (g)", "az (g)",
           "gx (dps)", "gy (dps)", "gz (dps)", "Missing")
COL_W = 11

def print_header():
    """Print the column header and its rule."""
    rule = "=" * (len(COLUMNS) * (COL_W + 1) - 1)
    print(rule)
    print(" ".join(f"{name:>{COL_W}}" for name in COLUMNS))
    print(rule)


def scale_row(t_s, sequence, accel, gyro, missing):
    """Convert one raw sample pair to the units in COLUMNS: s, g, and dps."""
    return (
        t_s,
        sequence,
        accel[0] * ACCEL_G_PER_COUNT,   # counts -> g
        accel[1] * ACCEL_G_PER_COUNT,
        accel[2] * ACCEL_G_PER_COUNT,
        gyro[0] * GYRO_DPS_PER_COUNT,   # counts -> dps
        gyro[1] * GYRO_DPS_PER_COUNT,
        gyro[2] * GYRO_DPS_PER_COUNT,
        missing,
    )


def format_row(values):
    """Format one scaled sample as a right-aligned row."""
    return " ".join(f"{v:>{COL_W}.3f}" if isinstance(v, float) else f"{v:>{COL_W}}"
                    for v in values)


def can_interfaces():
    """Return the names of the network interfaces that are CAN devices."""
    names = []
    for _, name in socket.if_nameindex():
        with open(f"/sys/class/net/{name}/type") as f:
            if f.read().strip() == ARPHRD_CAN:
                names.append(name)
    return names


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="list CAN interfaces and exit")
    parser.add_argument("-c", "--channel", default="can0", help="CAN interface, e.g. can0")
    parser.add_argument("--accel-id", type=lambda s: int(s, 0), default=0x065,
                        help="must match IMU_ACCEL_CAN_ID in Core/Src/main.c")
    parser.add_argument("--gyro-id", type=lambda s: int(s, 0), default=0x066,
                        help="must match IMU_GYRO_CAN_ID in Core/Src/main.c")
    parser.add_argument("--out", help="append samples to this CSV file as well")

    args = parser.parse_args()

    if not hasattr(socket, "AF_CAN"):
        parser.error("SocketCAN requires Linux; run this script on the Raspberry Pi.")

    if args.list:
        names = can_interfaces()
        if not names:
            print("No CAN interfaces found")
        for name in names:
            print(name)
        return

    if args.accel_id == args.gyro_id or not all(0 <= n <= 0x7FF for n in (args.accel_id, args.gyro_id)):
        parser.error("Choose two distinct standard CAN IDs (0..0x7ff).")

    # ======================================================================
    # Stream Samples
    # ======================================================================

    bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    filters = b"".join(struct.pack("=II", n, FILTER_MASK) for n in (args.accel_id, args.gyro_id))
    bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
    try:
        bus.bind((args.channel,))
    except OSError as exc:
        parser.error(f"Cannot open {args.channel}: {exc}; configure it at 1000000 bit/s first.")
    bus.settimeout(TIMEOUT_S)

    out_file = open(args.out, "a", newline="") if args.out else None
    writer = None
    if out_file is not None:
        writer = csv.writer(out_file)
        if out_file.tell() == 0:
            writer.writerow(COLUMNS)

    print(f"Reading from {args.channel}. Ctrl-C to stop.")
    if args.out:
        print(f"Logging to {args.out}.")
    print()
    print_header()

    pending = {}
    last_sequence = None
    missing = 0
    start = time.monotonic()

    try:
        while True:
            try:
                frame = bus.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                pending.clear()     # bus went quiet, so half a pair will never complete
                continue

            if len(frame) != CAN_FRAME_SIZE:
                continue    # not a classic CAN frame

            can_id, dlc, payload = struct.unpack(CAN_FRAME_FORMAT, frame)
            if dlc != 8 or can_id not in (args.accel_id, args.gyro_id):
                continue

            sequence, x, y, z = struct.unpack(PAYLOAD_FORMAT, payload)
            now = time.monotonic()

            # Expire incomplete pairs so resets, packet loss and wrap cannot mix old data
            pending = {key: value for key, value in pending.items() if now - value[0] < PAIR_EXPIRY_S}
            pair = pending.setdefault(sequence, [now, {}])[1]
            pair[can_id] = (x, y, z)

            if len(pair) == 2:
                if last_sequence is not None:
                    gap = (sequence - last_sequence) & 0xFFFF
                    if 0 < gap < 0x8000:
                        missing += gap - 1
                last_sequence = sequence

                values = scale_row(now - start, sequence, pair[args.accel_id], pair[args.gyro_id], missing)
                print(format_row(values))

                if writer is not None:
                    # More decimals than the console: one accel count is under 1 mg
                    writer.writerow(f"{v:.6f}" if isinstance(v, float) else v for v in values)

                del pending[sequence]

            if len(pending) > MAX_PENDING:
                del pending[next(iter(pending))]

    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
        if out_file is not None:
            out_file.close()


if __name__ == "__main__":
    main()
