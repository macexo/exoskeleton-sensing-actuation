#!/usr/bin/env python3
"""
imu_canbus.py

Receives the LSM6DSO32 streams from up to four pcb-imu-canbus boards sharing one
CAN bus, through Linux SocketCAN, and reports the rate and sample loss per board.

Each board sends a pair of 8-byte frames per sample at 417 Hz, little endian,
with the axes as raw signed sensor counts. CAN IDs follow the joint controller
layout: bits [10:7] message type, [6:4] source node, [3:0] destination (0 = Pi).

    node  joint        accel  gyro
    1     left hip     0x190  0x210
    2     right hip    0x1A0  0x220
    3     left knee    0x1B0  0x230
    4     right knee   0x1C0  0x240

    accel: <uint16 sequence>,<int16 ax>,<int16 ay>,<int16 az>
    gyro:  <uint16 sequence>,<int16 gx>,<int16 gy>,<int16 gz>

Frames are paired per node by sequence. Four boards make ~1670 pairs/s, too many
to print, so the console shows a summary per node every second and --out writes
every pair to a CSV file. --show prints live rows for one node instead.

Time (s) is the kernel's receive time since the script started (not MCU sample
time). Missing counts the sample pairs a node lost. Jumps counts sequence breaks
too big to be loss: a board reset, or two boards built with the same NODE_ID.

On exit (Ctrl-C or --duration) a per-node result is printed. The exit status is
0 only if every node delivered data with loss at or under --max-loss and no jumps.

Usage:
    python3 imu_canbus.py --channel can1
    python3 imu_canbus.py --channel can1 --duration 300 --out run.csv
    python3 imu_canbus.py --channel can1 --nodes 1,2
    python3 imu_canbus.py --channel can1 --show 2
"""

import argparse
import csv
import socket
import struct
import sys
import time

# ======================================================================
# Configuration
# ======================================================================

SAMPLE_RATE_HZ = 417            # must match the ODR in Core/Src/main.c
ACCEL_G_PER_COUNT = 0.000488    # must match the +/-16 g range in Core/Src/main.c
GYRO_DPS_PER_COUNT = 0.070      # must match the +/-2000 dps range in Core/Src/main.c

CAN_MSG_IMU_ACCEL = 0x3         # CAN ID bits [10:7], as in Core/Src/main.c
CAN_MSG_IMU_GYRO = 0x4
CAN_NODE_PI = 0x0
NODE_NAMES = {1: "left hip", 2: "right hip", 3: "left knee", 4: "right knee"}

RECV_TIMEOUT_S = 0.1            # wake up this often even when the bus is quiet
STALE_S = 0.5                   # a node silent this long is reported as stale
PAIR_EXPIRY_S = 0.5             # incomplete pairs older than this are discarded
GAP_SLACK = 1.5                 # a sequence gap above elapsed * rate * slack + margin
GAP_MARGIN = 10                 #   is a jump (reset, duplicate NODE_ID), not lost samples
RCVBUF_BYTES = 1 << 20          # requested socket buffer; the kernel caps it at rmem_max

CAN_FRAME_FORMAT = "=IB3x8s"    # SocketCAN struct can_frame: ID, DLC, padding, data
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
PAYLOAD_FORMAT = "<Hhhh"        # sequence, x, y, z
FILTER_MASK = 0xC00007FF        # standard data frames only: no extended IDs or remote requests
ARPHRD_CAN = "280"              # /sys/class/net/<name>/type of a CAN interface
SO_TIMESTAMP = getattr(socket, "SO_TIMESTAMP", 29)  # Linux: kernel receive time per frame
SO_RXQ_OVFL = getattr(socket, "SO_RXQ_OVFL", 40)    # Linux: frames dropped by a full socket buffer

CSV_COLUMNS = ("Time (s)", "Node", "Seq", "ax (g)", "ay (g)", "az (g)",
               "gx (dps)", "gy (dps)", "gz (dps)", "Missing")
COL_W = 11


def build_id(msg_type, src, dest):
    """Build an 11-bit CAN ID: type in bits [10:7], source [6:4], destination [3:0]."""
    return (msg_type << 7) | (src << 4) | dest


def imu_ids(node):
    """Return the (accel, gyro) CAN IDs of a board built with NODE_ID = node."""
    return (build_id(CAN_MSG_IMU_ACCEL, node, CAN_NODE_PI),
            build_id(CAN_MSG_IMU_GYRO, node, CAN_NODE_PI))


def scale(accel, gyro):
    """Convert raw counts to (ax, ay, az) in g followed by (gx, gy, gz) in dps."""
    return (tuple(a * ACCEL_G_PER_COUNT for a in accel) +
            tuple(g * GYRO_DPS_PER_COUNT for g in gyro))


# ======================================================================
# Per-node pairing and loss accounting
# ======================================================================

class Node:
    """Pairs one board's accel and gyro frames and counts its lost samples."""

    def __init__(self, node_id):
        self.node_id = node_id
        self.name = NODE_NAMES[node_id]
        self.pending = {}           # sequence -> (first receive time, {msg_type: axes})
        self.last_sequence = None
        self.first_pair_time = None
        self.last_pair_time = None
        self.last_rx = None         # receive time of the newest frame, paired or not
        self.pairs = 0
        self.missing = 0
        self.jumps = 0
        self.window_pairs = 0       # pairs since the last console summary
        self.latest = None          # scaled values of the newest pair
        self.stale = False

    def add_frame(self, msg_type, sequence, axes, rx_time):
        """Store one frame. Returns (accel, gyro) raw counts once its pair completes, else None."""
        self.last_rx = rx_time

        if sequence not in self.pending:
            # Expire incomplete pairs so resets, loss and wrap cannot mix old data
            self.pending = {seq: entry for seq, entry in self.pending.items()
                            if rx_time - entry[0] < PAIR_EXPIRY_S}
            self.pending[sequence] = (rx_time, {})

        halves = self.pending[sequence][1]
        halves[msg_type] = axes
        if len(halves) < 2:
            return None

        del self.pending[sequence]
        self._count_pair(sequence, rx_time)
        accel = halves[CAN_MSG_IMU_ACCEL]
        gyro = halves[CAN_MSG_IMU_GYRO]
        self.latest = scale(accel, gyro)
        return accel, gyro

    def _count_pair(self, sequence, rx_time):
        """Update the loss counters for a completed pair."""
        if self.last_sequence is not None:
            gap = (sequence - self.last_sequence) & 0xFFFF
            # The board advances its sequence for every sample, sent or not, so a
            # gap of n means n - 1 lost pairs, as long as enough time passed for
            # them. A bigger gap, a repeat, or a step backwards is a jump.
            elapsed = max(rx_time - self.last_pair_time, 0.0)
            plausible = int(elapsed * SAMPLE_RATE_HZ * GAP_SLACK) + GAP_MARGIN
            if 0 < gap <= plausible:
                self.missing += gap - 1
            else:
                self.jumps += 1
        else:
            self.first_pair_time = rx_time

        self.last_sequence = sequence
        self.last_pair_time = rx_time
        self.pairs += 1
        self.window_pairs += 1

    def loss_percent(self):
        """Lost pairs as a percentage of the pairs the board produced while seen."""
        total = self.pairs + self.missing
        return 100.0 * self.missing / total if total else 0.0

    def mean_rate(self):
        """Average received pair rate over the span this node was seen, in Hz."""
        if self.pairs < 2 or self.last_pair_time <= self.first_pair_time:
            return 0.0
        return (self.pairs - 1) / (self.last_pair_time - self.first_pair_time)


# ======================================================================
# SocketCAN
# ======================================================================

def can_interfaces():
    """Return the names of the network interfaces that are CAN devices."""
    names = []
    for _, name in socket.if_nameindex():
        with open(f"/sys/class/net/{name}/type") as f:
            if f.read().strip() == ARPHRD_CAN:
                names.append(name)
    return names


def open_bus(channel, nodes):
    """Open a raw CAN socket that only passes the IMU frames of the given nodes.

    Also asks the kernel for a per-frame receive timestamp, so loss accounting
    stays right when the script falls behind and reads a backlog, and for the
    count of frames dropped because the socket buffer was full.
    Raises OSError if the interface cannot be opened.
    """
    bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    filters = b"".join(struct.pack("=II", can_id, FILTER_MASK)
                       for node in nodes for can_id in imu_ids(node))
    bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)

    for option, value in ((socket.SO_RCVBUF, RCVBUF_BYTES), (SO_TIMESTAMP, 1), (SO_RXQ_OVFL, 1)):
        try:
            bus.setsockopt(socket.SOL_SOCKET, option, value)
        except OSError:
            pass    # optional: without it the script falls back to time.time() and no drop count

    try:
        bus.bind((channel,))
    except OSError:
        bus.close()
        raise
    bus.settimeout(RECV_TIMEOUT_S)
    return bus


def recv_frame(bus, ancillary_size):
    """Receive one frame.

    Returns (can_id, dlc, payload, rx_time, drops), or None for a frame that is
    not classic CAN. rx_time is the kernel receive time when available, else
    now. drops is the socket's cumulative overflow count, or None when the
    kernel did not attach one (it only does once the count is non-zero).
    """
    data, ancdata, _, _ = bus.recvmsg(CAN_FRAME_SIZE, ancillary_size)
    rx_time = None
    drops = None

    for level, kind, cdata in ancdata:
        if level != socket.SOL_SOCKET:
            continue
        if kind == SO_TIMESTAMP and len(cdata) in (8, 16):
            # struct timeval: two 64-bit fields on a 64-bit OS, two 32-bit fields on a 32-bit one
            sec, usec = struct.unpack("=qq" if len(cdata) == 16 else "=ii", cdata)
            rx_time = sec + usec / 1e6
        elif kind == SO_RXQ_OVFL and len(cdata) >= 4:
            drops = struct.unpack("=I", cdata[:4])[0]

    if len(data) != CAN_FRAME_SIZE:
        return None

    can_id, dlc, payload = struct.unpack(CAN_FRAME_FORMAT, data)
    return can_id & 0x7FF, dlc, payload, rx_time if rx_time is not None else time.time(), drops


# ======================================================================
# Console output
# ======================================================================

def log(message):
    """Print an event line straight away, even when stdout is piped."""
    print(message, flush=True)


def format_row(values):
    """Format one CSV-style row as right-aligned console columns."""
    return " ".join(f"{v:>{COL_W}.3f}" if isinstance(v, float) else f"{v:>{COL_W}}"
                    for v in values)


def print_row_header():
    """Print the column header for --show rows."""
    rule = "=" * (len(CSV_COLUMNS) * (COL_W + 1) - 1)
    print(rule)
    print(" ".join(f"{name:>{COL_W}}" for name in CSV_COLUMNS))
    print(rule)


def node_status(node):
    """One word for the node's state in the console summary."""
    if node.last_rx is None:
        return "WAITING"
    return "STALE" if node.stale else "OK"


def print_summary(nodes, elapsed_s, window_s, socket_drops):
    """Print one block: rate, totals, loss and the newest values for every node."""
    print(f"--- {elapsed_s:8.1f} s --- socket drops: {socket_drops}")
    print(f"{'node':>4} {'joint':<10} {'rate Hz':>7} {'pairs':>9} {'missing':>8} "
          f"{'loss %':>7} {'jumps':>5} {'status':>7} "
          f"{'ax g':>7} {'ay g':>7} {'az g':>7} {'gx dps':>8} {'gy dps':>8} {'gz dps':>8}")
    for node in nodes.values():
        rate = node.window_pairs / window_s if window_s > 0 else 0.0
        node.window_pairs = 0
        line = (f"{node.node_id:>4} {node.name:<10} {rate:>7.1f} {node.pairs:>9} {node.missing:>8} "
                f"{node.loss_percent():>7.3f} {node.jumps:>5} {node_status(node):>7}")
        if node.latest is not None:
            ax, ay, az, gx, gy, gz = node.latest
            line += f" {ax:>7.3f} {ay:>7.3f} {az:>7.3f} {gx:>8.2f} {gy:>8.2f} {gz:>8.2f}"
        print(line)
    print(flush=True)


def print_report(nodes, elapsed_s, socket_drops, max_loss):
    """Print the per-node result of the run. Returns True if every node passed."""
    all_passed = True
    print()
    print(f"==== Result after {elapsed_s:.1f} s ====")
    print(f"{'node':>4} {'joint':<10} {'pairs':>9} {'rate Hz':>7} {'missing':>8} "
          f"{'loss %':>7} {'jumps':>5}  result")
    for node in nodes.values():
        if node.pairs == 0:
            result = "FAIL (no data)"
        elif node.loss_percent() > max_loss:
            result = f"FAIL (loss over {max_loss}%)"
        elif node.jumps:
            result = "FAIL (sequence jumps: reset or duplicate NODE_ID)"
        else:
            result = "PASS"
        all_passed = all_passed and result == "PASS"
        print(f"{node.node_id:>4} {node.name:<10} {node.pairs:>9} {node.mean_rate():>7.1f} "
              f"{node.missing:>8} {node.loss_percent():>7.3f} {node.jumps:>5}  {result}")

    print(f"socket drops: {socket_drops}", end="")
    if socket_drops:
        print("  (the Pi fell behind: some missing pairs were lost here, not on the bus)")
    else:
        print()
    return all_passed


# ======================================================================
# Main
# ======================================================================

def parse_nodes(text):
    """Parse a comma-separated node list such as '1,2,3,4'."""
    try:
        nodes = sorted({int(part) for part in text.split(",") if part.strip()})
    except ValueError:
        raise argparse.ArgumentTypeError("use node numbers separated by commas, e.g. 1,2,3,4")
    if not nodes or any(n not in NODE_NAMES for n in nodes):
        raise argparse.ArgumentTypeError("nodes must be in 1..4")
    return nodes


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="list CAN interfaces and exit")
    parser.add_argument("-c", "--channel", default="can1", help="CAN interface, e.g. can1")
    parser.add_argument("--nodes", type=parse_nodes, default=[1, 2, 3, 4],
                        help="nodes expected on the bus (default 1,2,3,4)")
    parser.add_argument("--out", help="append every sample pair to this CSV file")
    parser.add_argument("--duration", type=float, help="stop after this many seconds")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="seconds between console summaries (default 1)")
    parser.add_argument("--show", type=int, metavar="NODE",
                        help="print every row from this node instead of summaries")
    parser.add_argument("--max-loss", type=float, default=0.1,
                        help="highest loss %% a node may have and still pass (default 0.1)")

    args = parser.parse_args()

    if not hasattr(socket, "AF_CAN"):
        parser.error("SocketCAN requires Linux; run this script on the Raspberry Pi.")

    if args.list:
        names = can_interfaces()
        if not names:
            print("No CAN interfaces found")
        for name in names:
            print(name)
        return 0

    if args.show is not None and args.show not in args.nodes:
        parser.error("--show must name one of the --nodes")
    if args.interval <= 0:
        parser.error("--interval must be positive")

    nodes = {n: Node(n) for n in args.nodes}
    id_map = {}     # CAN ID -> (node number, message type)
    for n in args.nodes:
        accel_id, gyro_id = imu_ids(n)
        id_map[accel_id] = (n, CAN_MSG_IMU_ACCEL)
        id_map[gyro_id] = (n, CAN_MSG_IMU_GYRO)

    try:
        bus = open_bus(args.channel, args.nodes)
    except OSError as exc:
        parser.error(f"Cannot open {args.channel}: {exc}; configure it at 1000000 bit/s first.")
    ancillary_size = socket.CMSG_SPACE(16) + socket.CMSG_SPACE(4)

    out_file = open(args.out, "a", newline="") if args.out else None
    writer = None
    if out_file is not None:
        writer = csv.writer(out_file)
        if out_file.tell() == 0:
            writer.writerow(CSV_COLUMNS)

    print(f"Reading from {args.channel}. Ctrl-C to stop.")
    for n in args.nodes:
        accel_id, gyro_id = imu_ids(n)
        print(f"  node {n} ({NODE_NAMES[n]}): accel 0x{accel_id:03X}, gyro 0x{gyro_id:03X}")
    if args.out:
        print(f"Logging to {args.out}.")
    print()
    if args.show is not None:
        print_row_header()

    # ======================================================================
    # Stream Samples
    # ======================================================================

    start = time.time()
    last_summary = start
    socket_drops = 0

    try:
        while True:
            try:
                frame = recv_frame(bus, ancillary_size)
            except socket.timeout:
                frame = None
            except OSError as exc:
                # The interface going down (bus-off, cable, ip link) should not kill a long run
                log(f"[CAN] receive error: {exc}; retrying")
                time.sleep(0.5)
                frame = None

            if frame is not None:
                can_id, dlc, payload, rx_time, drops = frame
                if drops is not None:
                    socket_drops = drops
                key = id_map.get(can_id)

                if key is not None and dlc == 8:
                    node = nodes[key[0]]
                    if node.stale:
                        node.stale = False
                        log(f"[ALIVE] node {node.node_id} ({node.name}) traffic resumed")

                    sequence, x, y, z = struct.unpack(PAYLOAD_FORMAT, payload)
                    pair = node.add_frame(key[1], sequence, (x, y, z), rx_time)

                    if pair is not None:
                        if node.pairs == 1:
                            log(f"[NODE] first sample pair from node {node.node_id} ({node.name})")
                        values = ((rx_time - start, node.node_id, sequence) +
                                  scale(*pair) + (node.missing,))
                        if writer is not None:
                            # More decimals than the console: one accel count is under 1 mg
                            writer.writerow(f"{v:.6f}" if isinstance(v, float) else v
                                            for v in values)
                        if args.show == node.node_id:
                            print(format_row(values))

            now = time.time()

            for node in nodes.values():
                if node.last_rx is not None and not node.stale and now - node.last_rx >= STALE_S:
                    node.stale = True
                    node.pending.clear()    # half a pair will never complete now
                    log(f"[STALE] node {node.node_id} ({node.name}) silent for "
                        f"{now - node.last_rx:.1f} s")

            if args.show is None and now - last_summary >= args.interval:
                print_summary(nodes, now - start, now - last_summary, socket_drops)
                last_summary = now

            if args.duration is not None and now - start >= args.duration:
                break

    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
        if out_file is not None:
            out_file.close()

    passed = print_report(nodes, time.time() - start, socket_drops, args.max_loss)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
