#!/usr/bin/env python3
"""Manual Pi torque console, V3 IMU capture and motor-feedback CSV logger (SocketCAN).

Python 3.9+, standard library only. See motor_console_README.md for the protocol,
bench limits, integration procedure, and hardware checks still required.
"""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import math
import socket
import struct
import sys
import threading
import time

from console_telemetry import IMUState, decode_imu_id, parse_motor_status
import imu_canbus

JOINTS = {1: "left hip", 2: "right hip", 3: "left knee", 4: "right knee"}
# PR #4 assigns only right knee by default; other boards are IMU-only.
DEFAULT_MOTOR_MAP = {4: 107}
DOCUMENTED_MOTOR_MAP = {1: 104, 2: 105, 3: 106, 4: 107}  # UNVERIFIED optional attribution.
KT_NM_PER_A = 1.431             # Nominal 0.159 Nm/A * 9 reduction; not calibration.
BENCH_CURRENT_LIMIT_A = 5.0
TORQUE_LIMIT_NM = 7.155         # KT_NM_PER_A * BENCH_CURRENT_LIMIT_A
FIRMWARE_TIMEOUT_S = 0.200
REFRESH_S = 0.050
STALE_S = 0.500
STATUS_S = 1.0
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF
CAN_FRAME = struct.Struct("=IB3x8s")
FEEDBACK_FUNCTION = 0x29
SO_TIMESTAMP = getattr(socket, "SO_TIMESTAMP", 29)
SO_RXQ_OVFL = getattr(socket, "SO_RXQ_OVFL", 40)
ESTOP_REASON = 1

CSV_COLUMNS = (
    "timestamp_utc", "elapsed_s", "event", "raw_can_id", "arbitration_id",
    "extended", "dlc", "payload_hex", "joint", "joint_name", "motor_id",
    "requested_torque_nm", "sent_torque_nm", "feedback_state", "feedback_age_s",
    "position_deg", "speed_erpm", "phase_current_a", "driver_temperature_c",
    "fault_code", "feedback_source", "imu_sequence", "imu_pair_state",
    "imu_state", "imu_age_s", "ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw",
    "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "imu_pairs", "imu_missing",
    "imu_jumps", "imu_rate_hz", "timestamp_source", "rx_socket_drops", "detail",
)


class CommandError(ValueError):
    """An invalid operator command that does not terminate a healthy console."""


@dataclass(frozen=True)
class Frame:
    """Classic CAN frame; raw_id includes SocketCAN EFF/RTR/ERR flags."""

    raw_id: int
    data: bytes
    dlc: int
    is_fd: bool = False
    received_wall: float = None
    received_monotonic: float = None
    socket_drops: int = None

    @property
    def arbitration_id(self):
        return self.raw_id & (CAN_EFF_MASK if self.extended else CAN_SFF_MASK)

    @property
    def extended(self):
        return bool(self.raw_id & CAN_EFF_FLAG)


@dataclass(frozen=True)
class Feedback:
    """One actual received feedback sample, separate from requested torque."""

    joint: int
    motor_id: int
    received_at: float
    position_deg: float
    speed_erpm: int
    phase_current_a: float
    driver_temperature_c: int
    fault_code: int
    source: str = "forwarded"

    def fields(self):
        return {name: getattr(self, name) for name in (
            "position_deg", "speed_erpm", "phase_current_a", "driver_temperature_c",
            "fault_code",
        )}


def validate_motor_map(motor_map):
    """Validate optional attribution, without implying every joint has a motor."""
    if not motor_map or any(type(joint) is not int or joint not in JOINTS for joint in motor_map):
        raise ValueError("motor map must specify one or more distinct joints in 1..4")
    if any(type(driver) is not int or not 1 <= driver <= 255 for driver in motor_map.values()):
        raise ValueError("motor IDs must be integers in 1..255")
    if len(set(motor_map.values())) != len(motor_map):
        raise ValueError("motor IDs must be unique")
    return dict(motor_map)


def parse_motor_map(value):
    """Parse optional attribution such as --motor-map 4:107 (the PR default)."""
    try:
        pairs = [entry.split(":") for entry in value.split(",")]
        if any(len(pair) != 2 for pair in pairs):
            raise ValueError("expected joint:motor pairs")
        result = {int(joint): int(driver) for joint, driver in pairs}
        if len(result) != len(pairs):
            raise ValueError("each joint may appear only once")
        return validate_motor_map(result)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def torque_frame(joint, torque_nm):
    """Encode Pi source 0, TORQUE_CMD type 1; reject invalid/unsafe requests."""
    if type(joint) is not int or joint not in JOINTS:
        raise ValueError("joint must be 1, 2, 3, or 4")
    if not math.isfinite(torque_nm) or abs(torque_nm) > TORQUE_LIMIT_NM:
        raise ValueError(f"torque must be finite and within +/-{TORQUE_LIMIT_NM:.3f} Nm")
    raw = int(round(torque_nm * 1000.0))
    return Frame((1 << 7) | joint, struct.pack("<h", raw), 2)


def decode_feedback(frame, motor_map, received_at, source="forwarded"):
    """Decode one selected stream: PR #4 forwarded status or explicit raw function 0x29.

    Forwarded payload/scaling reuses last year's can_motor.parse_motor_status
    semantics. Exact identifiers and eight-byte classic data frames are required.
    Direct and forwarded streams are never silently mixed.
    """
    if (frame.is_fd or frame.raw_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)
            or frame.dlc != 8 or len(frame.data) != 8):
        return None
    if source == "forwarded":
        joint = next((joint for joint in JOINTS if frame.raw_id == (0x100 | (joint << 4))), None)
        if joint is None:
            return None
        driver = motor_map.get(joint)
        values = parse_motor_status(frame.data)
    elif source == "direct":
        if not frame.extended or (frame.raw_id & CAN_EFF_MASK) >> 8 != FEEDBACK_FUNCTION:
            return None
        driver = frame.raw_id & 0xFF
        joint = next((joint for joint, motor in motor_map.items() if motor == driver), None)
        if joint is None:
            return None
        position, speed, phase_current, temperature, fault = struct.unpack(">hhhbB", frame.data)
        values = position / 10.0, speed * 10.0, phase_current / 100.0, temperature, fault
    else:
        raise ValueError("feedback source must be forwarded or direct")
    return Feedback(joint, driver, received_at, *values, source=source)


def is_estop(frame):
    """Recognize the shared Pi software ESTOP format, including frame-type checks."""
    return (not frame.is_fd and frame.raw_id == 0 and frame.dlc == 1
            and len(frame.data) == 1)


class SocketCAN:
    """Small Linux classic SocketCAN transport; concurrent send and receive are supported."""

    def __init__(self, channel, motor_map, feedback_source="forwarded"):
        if not hasattr(socket, "AF_CAN"):
            raise OSError("SocketCAN requires Linux; tests can run on other operating systems")
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            # CAN_ERR_FLAG in a filter mask selects Linux error-only reception.
            standard_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK
            standard_ids = [identifier for joint in JOINTS for identifier in imu_canbus.imu_ids(joint)]
            standard_ids.append(0)
            if feedback_source == "forwarded":
                standard_ids.extend(0x100 | (joint << 4) for joint in JOINTS)
            elif feedback_source != "direct":
                raise ValueError("feedback source must be forwarded or direct")
            filters = [(identifier, standard_mask) for identifier in standard_ids]
            if feedback_source == "direct":
                mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK
                filters.extend((CAN_EFF_FLAG | (FEEDBACK_FUNCTION << 8) | driver, mask)
                               for driver in motor_map.values())
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                 b"".join(struct.pack("=II", *entry) for entry in filters))
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            for option in (SO_TIMESTAMP, SO_RXQ_OVFL):
                try:
                    self.sock.setsockopt(socket.SOL_SOCKET, option, 1)
                except OSError:
                    pass  # Optional kernel timestamp/drop metadata, as in imu_canbus.py.
            # Bound send/recv so a busy or silent bus cannot hold the stop lock indefinitely.
            self.sock.settimeout(0.020)
            self.sock.bind((channel,))
        except BaseException:
            self.sock.close()
            raise

    def send(self, frame):
        """Queue one frame; success is kernel acceptance, never proof of motor execution."""
        packet = CAN_FRAME.pack(frame.raw_id, frame.dlc, frame.data.ljust(8, b"\0"))
        if self.sock.send(packet) != CAN_FRAME.size:
            raise OSError("short SocketCAN send")

    def recv(self):
        """Receive a classic frame with optional kernel timestamp/drop metadata.

        Ancillary parsing follows imu_canbus.recv_frame, preserving raw ID flags
        and rejecting non-classic sizes instead of masking extended IDs to 11 bits.
        """
        try:
            packet, ancillary, _, _ = self.sock.recvmsg(72, socket.CMSG_SPACE(16) * 2)
        except socket.timeout:
            return None
        if len(packet) != CAN_FRAME.size:
            return None
        received_wall = None
        drops = None
        for level, kind, data in ancillary:
            if level != socket.SOL_SOCKET:
                continue
            if kind == SO_TIMESTAMP and len(data) in (8, 16):
                seconds, micros = struct.unpack("=qq" if len(data) == 16 else "=ii", data)
                received_wall = seconds + micros / 1e6
            elif kind == SO_RXQ_OVFL and len(data) >= 4:
                drops = struct.unpack("=I", data[:4])[0]
        observed_mono = time.monotonic()
        if received_wall is not None:
            observed_mono -= max(0.0, time.time() - received_wall)
        raw_id, dlc, data = CAN_FRAME.unpack(packet)
        return Frame(raw_id, data[:min(dlc, 8)], dlc, received_wall=received_wall,
                     received_monotonic=observed_mono, socket_drops=drops)

    def close(self):
        self.sock.close()


class CSVLog:
    """Thread-safe, immediately flushed event CSV; empty fields represent missing data."""

    def __init__(self, output):
        self.output = output
        self.lock = threading.Lock()
        self.started = time.monotonic()
        self.writer = csv.DictWriter(output, fieldnames=CSV_COLUMNS)
        self.writer.writeheader()
        output.flush()

    def write(self, event, frame=None, **fields):
        row = {"timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
               "elapsed_s": f"{time.monotonic() - self.started:.6f}", "event": event}
        if frame is not None:
            if frame.received_wall is not None:
                row["timestamp_utc"] = datetime.fromtimestamp(
                    frame.received_wall, timezone.utc).isoformat(timespec="microseconds")
            row.update(timestamp_source="kernel" if frame.received_wall is not None else "userspace",
                       rx_socket_drops="" if frame.socket_drops is None else frame.socket_drops)
            row.update(raw_can_id=f"0x{frame.raw_id:08X}",
                       arbitration_id=f"0x{frame.arbitration_id:X}",
                       extended=int(frame.extended), dlc=frame.dlc, payload_hex=frame.data.hex())
        row.update(fields)
        with self.lock:
            self.writer.writerow(row)
            self.output.flush()


class MotorConsole:
    """Coordinate requested commands, refresh, V3 IMU/motor telemetry, and fail-closed shutdown.

    The command lock covers both the active set and sends. Once stop acquires it,
    no earlier refresh can send a nonzero command after the stop frames.
    """

    def __init__(self, transport, logger, motor_map=None, refresh_s=REFRESH_S,
                 stale_s=STALE_S, clock=time.monotonic, report=print,
                 feedback_source="forwarded"):
        self.transport = transport
        self.logger = logger
        self.motor_map = validate_motor_map(DEFAULT_MOTOR_MAP if motor_map is None else motor_map)
        if feedback_source not in ("forwarded", "direct"):
            raise ValueError("feedback source must be forwarded or direct")
        self.feedback_source = feedback_source
        if not math.isfinite(refresh_s) or not 0.010 <= refresh_s <= 0.100:
            raise ValueError("refresh interval must be 0.010..0.100 s (firmware timeout 0.200 s)")
        if not math.isfinite(stale_s) or stale_s <= 0:
            raise ValueError("stale interval must be positive and finite")
        self.refresh_s = refresh_s
        self.stale_s = stale_s
        self.clock = clock
        self.report = report
        self.command_lock = threading.RLock()
        self.feedback_lock = threading.Lock()
        self.active = {}
        self.latest = {}
        self.imu = {joint: IMUState(joint) for joint in JOINTS}
        self.socket_drops = 0
        self.estopped = False
        self.shutdown = threading.Event()
        self.failure = None
        self.threads = []
        self.closed = False

    def identity(self, joint):
        return {"joint": joint, "joint_name": JOINTS[joint], "motor_id": self.motor_map.get(joint)}

    def _send_torque(self, joint, requested, event):
        frame = torque_frame(joint, requested)
        sent = struct.unpack("<h", frame.data)[0] / 1000.0
        try:
            self.transport.send(frame)
        except Exception as error:
            self.logger.write("send_failed", frame, **self.identity(joint),
                              requested_torque_nm=requested, detail=str(error))
            raise
        self.logger.write(event, frame, **self.identity(joint),
                          requested_torque_nm=requested, sent_torque_nm=sent)
        return sent

    def set_torque(self, joints, requested):
        """Send an immediate request and retain nonzero encoded commands for refresh."""
        try:
            for joint in joints:
                torque_frame(joint, requested)  # Validate before partial fan-out.
        except ValueError as error:
            raise CommandError(str(error)) from error
        with self.command_lock:
            if self.shutdown.is_set() or self.closed:
                raise RuntimeError("console is shutting down")
            if self.estopped:
                raise CommandError("software ESTOP is latched; reset boards and restart the console")
            try:
                for joint in joints:
                    sent = self._send_torque(joint, requested, "command")
                    if sent:
                        self.active[joint] = requested
                    else:
                        self.active.pop(joint, None)
                    self.report(f"joint {joint}: requested {requested:.6g} Nm, "
                                f"sent {sent:.3f} Nm (execution unconfirmed)")
            except BaseException:
                self.stop("command exception")
                raise

    def stop(self, reason="tstop", estop=False):
        """Cancel refresh atomically and attempt every zero even if one send/log fails.

        ESTOP is sent first when requested, then all four explicit zero requests.
        Returns error strings; never lets a failed CSV write prevent a stop attempt.
        """
        errors = []
        with self.command_lock:
            self.active.clear()
            if estop:
                self.estopped = True
                frame = Frame(0, bytes([ESTOP_REASON]), 1)
                try:
                    self.transport.send(frame)
                except Exception as error:
                    errors.append(f"ESTOP send: {error}")
                    event = "send_failed"
                else:
                    event = "estop"
                try:
                    self.logger.write(event, frame, detail=reason)
                except Exception as error:
                    errors.append(f"CSV: {error}")
            for joint in JOINTS:
                try:
                    self._send_torque(joint, 0.0, "stop")
                except Exception as error:
                    errors.append(f"joint {joint} stop: {error}")
            try:
                self.logger.write("commands_cleared", detail=reason)
            except Exception as error:
                errors.append(f"CSV: {error}")
        for error in errors:
            self.report(f"[STOP ATTEMPT FAILED] {error}")
        return errors

    def refresh_once(self):
        """Refresh only the active set; failures cancel all requests and attempt stop."""
        with self.command_lock:
            if self.shutdown.is_set():
                return
            try:
                for joint, requested in self.active.items():
                    self._send_torque(joint, requested, "refresh")
            except BaseException:
                self.stop("refresh exception")
                raise

    def process_frame(self, frame):
        """Fan out telemetry by type, following the familiar Pi listener structure."""
        if frame.socket_drops is not None and frame.socket_drops != self.socket_drops:
            self.socket_drops = frame.socket_drops
            self.report(f"[CAN] receive socket cumulative drops: {self.socket_drops}")
        received_at = (self.clock() if frame.received_monotonic is None
                       else frame.received_monotonic)
        if is_estop(frame):
            with self.command_lock:
                self.estopped = True
                self.active.clear()
                self.logger.write("estop_received", frame, detail=f"reason={frame.data[0]}")
                self.stop("received software ESTOP")
            return
        imu_id = decode_imu_id(frame.raw_id)
        if imu_id is not None and not frame.is_fd and frame.dlc == 8 and len(frame.data) == 8:
            joint, kind = imu_id
            sequence, *axes = struct.unpack(imu_canbus.PAYLOAD_FORMAT, frame.data)
            with self.feedback_lock:
                sample = self.imu[joint].add_frame(kind, sequence, tuple(axes), received_at)
            axis_names = (("ax_raw", "ay_raw", "az_raw") if kind == 3
                          else ("gx_raw", "gy_raw", "gz_raw"))
            self.logger.write("imu_accel" if kind == 3 else "imu_gyro", frame,
                              **self.identity(joint), imu_sequence=sequence,
                              imu_pair_state="paired" if sample else "waiting",
                              **dict(zip(axis_names, axes)))
            if sample is not None:
                self.logger.write("imu_pair", **self.identity(joint),
                                  imu_pair_state="paired", imu_state="received",
                                  imu_age_s=f"{max(0.0, self.clock() - received_at):.6f}",
                                  **sample.fields())
            return
        feedback = decode_feedback(frame, self.motor_map, received_at, self.feedback_source)
        if feedback is None:
            return
        with self.feedback_lock:
            self.latest[feedback.joint] = feedback
        self.logger.write("feedback", frame, **self.identity(feedback.joint),
                          feedback_state="received",
                          feedback_age_s=f"{max(0.0, self.clock() - received_at):.6f}",
                          feedback_source=feedback.source, **feedback.fields())

    def feedback_snapshot(self, joint):
        """Return state/age/sample; never manufacture a zero sample for an absent motor."""
        if joint not in JOINTS:
            raise ValueError("joint must be 1..4")
        with self.feedback_lock:
            feedback = self.latest.get(joint)
        if feedback is None:
            return "missing", None, None
        age = max(0.0, self.clock() - feedback.received_at)
        return ("stale" if age >= self.stale_s else "fresh"), age, feedback

    def imu_snapshot(self, joint):
        """Return a locked copy of the familiar per-joint IMU queue and counters."""
        with self.feedback_lock:
            state = self.imu[joint]
            samples = list(state.queue)
            node = state.node
            counters = {"imu_pairs": node.pairs, "imu_missing": node.missing,
                        "imu_jumps": node.jumps, "imu_rate_hz": node.mean_rate()}
        if not samples:
            return "missing", None, samples, counters
        age = max(0.0, self.clock() - samples[-1].received_at)
        return "stale" if age >= self.stale_s else "fresh", age, samples, counters

    def show_imu(self, joint, full_queue=False):
        """Familiar l<n>/q<n> queries, adapted to V3 sequence + raw XYZ payloads."""
        state, age, samples, counters = self.imu_snapshot(joint)
        prefix = f"joint {joint} ({JOINTS[joint]}) IMU"
        if not samples:
            self.report(f"{prefix}: MISSING; no complete V3 pair received")
            return
        self.report(f"{prefix}: {state.upper()}, age {age:.3f} s; "
                    f"pairs={counters['imu_pairs']}, missing={counters['imu_missing']}, "
                    f"jumps={counters['imu_jumps']}, mean={counters['imu_rate_hz']:.1f} Hz")
        for sample in samples if full_queue else samples[-1:]:
            ax, ay, az, gx, gy, gz = sample.scaled
            self.report(f"  last received seq={sample.sequence}; "
                        f"accel g=({ax:.6f}, {ay:.6f}, {az:.6f}); "
                        f"gyro dps=({gx:.3f}, {gy:.3f}, {gz:.3f})")

    def show_status(self):
        for joint in JOINTS:
            self.show_imu(joint)
            self.show_motor(joint)
        self.report(f"receive socket cumulative drops: {self.socket_drops}")

    def log_status(self):
        """Record missing/stale explicitly; decoded values occur only on actual RX rows."""
        for joint in JOINTS:
            state, age, _ = self.feedback_snapshot(joint)
            imu_state, imu_age, _, counters = self.imu_snapshot(joint)
            self.logger.write("status", **self.identity(joint), feedback_state=state,
                              feedback_source=self.feedback_source,
                              feedback_age_s="" if age is None else f"{age:.6f}",
                              imu_state=imu_state, imu_age_s="" if imu_age is None else f"{imu_age:.6f}",
                              rx_socket_drops=self.socket_drops, **counters)

    def show_motor(self, joint):
        state, age, feedback = self.feedback_snapshot(joint)
        driver = self.motor_map.get(joint)
        prefix = (f"joint {joint} ({JOINTS[joint]}), "
                  f"motor {driver if driver is not None else 'unassigned'}, {self.feedback_source}")
        if feedback is None:
            self.report(f"{prefix}: MISSING; no selected motor feedback received")
            return
        self.report(f"{prefix}: {state.upper()}, age {age:.3f} s; last received "
                    f"position {feedback.position_deg:.1f} deg, speed {feedback.speed_erpm} ERPM, "
                    f"phase current {feedback.phase_current_a:.2f} A, "
                    f"driver temperature {feedback.driver_temperature_c} C, "
                    f"fault {feedback.fault_code} (0x{feedback.fault_code:02X})")

    def _worker_failed(self, error):
        # Stop occurs here even if the foreground is still blocked in input().
        with self.command_lock:
            if self.failure is None:
                self.failure = error
            self.shutdown.set()
            self.stop(f"background exception: {error}")
        self.report(f"[FATAL] background worker failed: {error}; stop attempted; press Enter to exit")

    def _refresh_worker(self):
        try:
            while not self.shutdown.wait(self.refresh_s):
                self.refresh_once()
        except BaseException as error:
            self._worker_failed(error)

    def _receive_worker(self):
        next_status = self.clock()
        try:
            while not self.shutdown.is_set():
                frame = self.transport.recv()
                if frame is not None:
                    self.process_frame(frame)
                if self.clock() >= next_status:
                    self.log_status()
                    next_status = self.clock() + STATUS_S
        except BaseException as error:
            self._worker_failed(error)

    def start(self):
        """Start both workers after recording the initial missing feedback state."""
        self.log_status()
        for target, name in ((self._refresh_worker, "torque-refresh"),
                             (self._receive_worker, "motor-feedback")):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self.threads.append(thread)

    def close(self):
        """Prevent new sends, cancel refresh, attempt all-joint stop, then close transport."""
        with self.command_lock:
            if self.closed:
                return
            self.shutdown.set()
            errors = self.stop("console shutdown")
            self.closed = True
            if errors and self.failure is None:
                self.failure = RuntimeError("; ".join(errors))
        for thread in self.threads:
            if thread is not threading.current_thread():
                thread.join(timeout=1.0)
        self.transport.close()


HELP = f"""Commands:
  t <joint> <Nm>  request joint 1..4 torque (limit +/-{TORQUE_LIMIT_NM:.3f} Nm)
  t_all <Nm>     request all four joints; tall is an alias
  tstop          clear active requests and send zero to all joints
  estop          broadcast latched software ESTOP and clear active requests
  m1 .. m4       display latest selected motor feedback and its age
  l1 .. l4       display latest complete V3 IMU pair (g, degrees/s)
  q1 .. q4       display bounded queue of up to 187 complete V3 IMU pairs
  status         show all IMU/motor states, counts and receive socket drops
  help           show this help
  quit           clear active requests, attempt zero on all joints, and exit
Active requests refresh every 50 ms by default (PR #4 firmware expiry: 200 ms).
Sent means accepted by the local CAN socket; motor execution is unconfirmed.
"""


def handle_command(console, line):
    """Validate a complete command before changing requests. Return False on quit."""
    parts = line.strip().lower().split()
    if not parts:
        return True
    command = parts[0]
    if command == "t" and len(parts) == 3:
        try:
            joint = int(parts[1])
            requested = float(parts[2])
        except ValueError as error:
            raise CommandError("usage: t <joint 1..4> <finite Nm>") from error
        console.set_torque([joint], requested)
    elif command in ("t_all", "tall") and len(parts) == 2:
        try:
            requested = float(parts[1])
        except ValueError as error:
            raise CommandError("usage: t_all <finite Nm>") from error
        console.set_torque(list(JOINTS), requested)
    elif command == "tstop" and len(parts) == 1:
        if console.stop():
            raise OSError("one or more stop attempts failed")
        console.report("active requests cleared; zero requests sent to all joints")
    elif command == "estop" and len(parts) == 1:
        if console.stop("operator ESTOP", estop=True):
            raise OSError("one or more ESTOP/zero attempts failed")
        console.report("software ESTOP sent; active requests cleared; board reset required to unlatch")
    elif len(parts) == 1 and command in ("m1", "m2", "m3", "m4"):
        console.show_motor(int(command[1]))
    elif len(parts) == 1 and command in ("l1", "l2", "l3", "l4", "q1", "q2", "q3", "q4"):
        console.show_imu(int(command[1]), full_queue=command[0] == "q")
    elif command == "status" and len(parts) == 1:
        console.show_status()
    elif command in ("help", "h") and len(parts) == 1:
        console.report(HELP)
    elif command == "quit" and len(parts) == 1:
        return False
    else:
        raise CommandError("invalid command or arguments; type help")
    return True


def run_console(console, read_line=input):
    """Ensure EOF, Ctrl+C, foreground/background errors, and quit all attempt stop."""
    try:
        console.start()
        while not console.shutdown.is_set():
            try:
                line = read_line("> ")
            except (EOFError, KeyboardInterrupt):
                break
            if console.shutdown.is_set():
                break
            try:
                if not handle_command(console, line):
                    break
            except CommandError as error:
                console.report(f"[INPUT] {error}")
    finally:
        console.close()
    return 1 if console.failure else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can1", help="Linux SocketCAN interface (default can1)")
    parser.add_argument("--motor-map", type=parse_motor_map,
                        default=dict(DEFAULT_MOTOR_MAP), metavar="4:107",
                        help="optional joint:motor attribution; PR default only 4:107; verify physically")
    parser.add_argument("--feedback-source", choices=("forwarded", "direct"), default="forwarded",
                        help="forwarded standard MOTOR_STATUS (default) or raw extended function 0x29")
    parser.add_argument("--out", default="motor_log.csv", help="CSV path (created exclusively)")
    parser.add_argument("--refresh-ms", type=float, default=REFRESH_S * 1000,
                        help="Pi torque refresh period, 10..100 ms; default 50")
    parser.add_argument("--stale-ms", type=float, default=STALE_S * 1000,
                        help="age considered stale; default 500 ms")
    args = parser.parse_args(argv)
    if not math.isfinite(args.refresh_ms) or not 10 <= args.refresh_ms <= 100:
        parser.error("--refresh-ms must be finite and in 10..100 (firmware expiry 200 ms)")
    if not math.isfinite(args.stale_ms) or args.stale_ms <= 0:
        parser.error("--stale-ms must be positive and finite")
    transport = None
    try:
        with open(args.out, "x", newline="", encoding="utf-8") as output:
            logger = CSVLog(output)
            transport = SocketCAN(args.channel, args.motor_map, args.feedback_source)
            console = MotorConsole(transport, logger, args.motor_map, args.refresh_ms / 1000,
                                   args.stale_ms / 1000, feedback_source=args.feedback_source)
            logger.write("session", detail=f"channel={args.channel}; motor_map={args.motor_map}; "
                         f"refresh_ms={args.refresh_ms}; firmware_timeout_ms=200; local_refresh_ms=50; "
                         f"feedback_source={args.feedback_source}; motor map requires hardware verification")
            print(f"Manual torque console, {args.channel}, 1 Mbit/s (configure interface separately)")
            print(f"Feedback source: {args.feedback_source}; attribution {args.motor_map}; CSV {args.out}")
            print("PR #4 default: only node 4 -> motor 107; nodes 1-3 are IMU-only unless configured.")
            print("Motor attribution REQUIRES PHYSICAL VERIFICATION; sent requests do not prove enablement.")
            print(f"Nominal conversion {KT_NM_PER_A} Nm/A; bench limit +/-5 A = +/-7.155 Nm")
            print(HELP)
            return run_console(console)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f"[FATAL] {error}", file=sys.stderr)
        return 1
    finally:
        if transport is not None:
            transport.close()


if __name__ == "__main__":
    sys.exit(main())
