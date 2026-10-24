#!/usr/bin/env python3
"""Receive pcb-imu-can telemetry on Linux / Raspberry Pi SocketCAN (stdlib only)."""
import argparse
import socket
import struct
import time


def decode_payload(payload):
    """Return (sequence, x, y, z); axes remain signed raw sensor counts."""
    return struct.unpack('<Hhhh', payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel', default='can0')
    parser.add_argument('--accel-id', type=lambda s: int(s, 0), default=0x065)
    parser.add_argument('--gyro-id', type=lambda s: int(s, 0), default=0x066)
    args = parser.parse_args()
    if not hasattr(socket, 'AF_CAN'):
        parser.error('SocketCAN requires Linux; run this script on the Raspberry Pi.')
    if args.accel_id == args.gyro_id or not all(0 <= n <= 0x7ff for n in (args.accel_id, args.gyro_id)):
        parser.error('Choose two distinct standard CAN IDs (0..0x7ff).')

    pending = {}
    last_sequence = None
    missing = 0
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        # Match standard data frames only (exclude extended IDs and remote requests).
        filters = b''.join(struct.pack('=II', n, 0xC00007FF)
                           for n in (args.accel_id, args.gyro_id))
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        try:
            bus.bind((args.channel,))
        except OSError as exc:
            parser.error(f'Cannot open {args.channel}: {exc}; configure it at 500000 bit/s first.')
        bus.settimeout(1.0)
        print('host_monotonic_s,sequence,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,missing_pairs', flush=True)
        try:
            while True:
                try:
                    frame = bus.recv(16)
                except socket.timeout:
                    pending.clear()
                    continue
                if len(frame) != 16:
                    continue
                can_id, dlc, payload = struct.unpack('=IB3x8s', frame)
                if dlc != 8 or can_id not in (args.accel_id, args.gyro_id):
                    continue
                sequence, x, y, z = decode_payload(payload)
                now = time.monotonic()
                # Expire incomplete pairs so resets, packet loss and wrap cannot mix old data.
                pending = {key: value for key, value in pending.items() if now - value[0] < 0.5}
                pair = pending.setdefault(sequence, [now, {}])[1]
                pair[can_id] = (x, y, z)
                if len(pair) == 2:
                    accel = [v * 0.000488 for v in pair[args.accel_id]]
                    gyro = [v * 0.070 for v in pair[args.gyro_id]]
                    if last_sequence is not None:
                        gap = (sequence - last_sequence) & 0xffff
                        if 0 < gap < 0x8000:
                            missing += gap - 1
                    last_sequence = sequence
                    values = ','.join(f'{v:.6f}' for v in accel + gyro)
                    print(f'{now:.6f},{sequence},{values},{missing}', flush=True)
                    del pending[sequence]
                if len(pending) > 64:
                    del pending[next(iter(pending))]
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
