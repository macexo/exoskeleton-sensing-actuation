"""V3 telemetry helpers for the manual console.

Reuse the unchanged four-board checker's sequence pairing/scaling. Motor-status
scaling mirrors last year's apis/can/python/can_motor.py:parse_motor_status and
PR #4's can_send_motor_status, without importing the sibling repo at runtime.
"""

from collections import deque
from dataclasses import dataclass
import struct

import imu_canbus


@dataclass(frozen=True)
class IMUSample:
    sequence: int
    received_at: float
    raw_accel: tuple
    raw_gyro: tuple
    scaled: tuple

    def fields(self):
        fields = {"imu_sequence": self.sequence}
        fields.update(zip(("ax_raw", "ay_raw", "az_raw"), self.raw_accel))
        fields.update(zip(("gx_raw", "gy_raw", "gz_raw"), self.raw_gyro))
        fields.update(zip(("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"),
                          self.scaled))
        return fields


class IMUState:
    """Bounded per-joint V3 queue; no downsampling or old gyro-position layout."""

    def __init__(self, joint, queue_size=187):
        self.node = imu_canbus.Node(joint)
        self.queue = deque(maxlen=queue_size)

    def add_frame(self, kind, sequence, axes, received_at):
        # Expire even if this sequence already has one half; the baseline helper
        # only prunes when inserting a new sequence. Never join old/new halves.
        self.node.pending = {seq: entry for seq, entry in self.node.pending.items()
                             if received_at - entry[0] < imu_canbus.PAIR_EXPIRY_S}
        pair = self.node.add_frame(kind, sequence, axes, received_at)
        if pair is None:
            return None
        accel, gyro = pair
        sample = IMUSample(sequence, received_at, accel, gyro, imu_canbus.scale(accel, gyro))
        self.queue.append(sample)
        return sample


def decode_imu_id(raw_id):
    """Match exact standard Pi-directed V3 IDs, rejecting other destinations/sources."""
    for joint in imu_canbus.NODE_NAMES:
        accel, gyro = imu_canbus.imu_ids(joint)
        if raw_id == accel:
            return joint, imu_canbus.CAN_MSG_IMU_ACCEL
        if raw_id == gyro:
            return joint, imu_canbus.CAN_MSG_IMU_GYRO
    return None


def parse_motor_status(payload):
    """Last-year can_motor.parse_motor_status semantics, with explicit eight-byte validation."""
    if len(payload) != 8:
        raise ValueError("motor status payload must contain exactly eight bytes")
    position, speed, phase_current, temperature, fault = struct.unpack("<hhhbB", payload)
    return position / 10.0, speed * 10.0, phase_current / 100.0, temperature, fault
