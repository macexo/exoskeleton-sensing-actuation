# Joint controller bench test V1

Purpose: manually verify that motor commands, motor feedback and the V3 SPI IMU
work together over CAN. This is a bench bring-up tool, not the final controller.

Python 3.9+, standard library only; live operation needs Linux SocketCAN.
Keep `motor_console.py`, `console_telemetry.py` and the unchanged `imu_canbus.py`
together. No sibling-repository runtime dependency or ML model is required.

## Run

Configure CAN at 1 Mbit/s, then:

```bash
python3 motor_console.py --channel can1 --out bench_001.csv
```

| Command | Purpose |
| --- | --- |
| `t 4 0.1` | Request a small torque on joint 4 |
| `t_all 0.1` / `tall 0.1` | Request torque on all four nodes |
| `tstop` | Clear active requests and attempt zero on all joints |
| `estop` | Broadcast latched software ESTOP and clear active requests |
| `m1`–`m4` | Latest motor status and age |
| `l1`–`l4`, `q1`–`q4` | Latest IMU pair or recent paired samples |
| `status`, `help`, `quit` | Summary, command reference, stop and exit |

Active requests refresh every 50 ms while the prompt waits for input. Firmware
expires them after 200 ms without a valid Pi command. Requested torque is limited
to ±7.155 Nm (nominal 1.431 Nm/A, firmware clamp ±5 A); begin far below that limit.
A sent request does not confirm motor execution. Quit/EOF/Ctrl+C attempt stop.

## CAN receive format

IMU IDs also match the folder’s `can.png` and the unchanged SPI baseline
in this project. Matches Majock's sensing-actuation PR #4 at
`05b60dbe30a4c72d468401b182a04210aab36d83`:

- IMU accel `190/1A0/1B0/1C0`, gyro `210/220/230/240`: standard DLC 8,
  `<Hhhh` sequence + raw XYZ. Accel ×0.000488 g; gyro ×0.070 degrees/s.
  Nominal 417 Hz; halves are paired by joint and sequence.
- Default motor status `110/120/130/140`: standard DLC 8, little-endian
  `<hhhbB`: position /10 degrees, speed ×10 ERPM, phase current /100 A,
  driver temperature and raw fault code. This reuses last year's decoder format.
- Optional `--feedback-source direct` receives extended `(0x29 << 8) | driver`,
  big-endian. Latest firmware uses `0x29`, replacing the older PR's `0x09`.
- Torque requests are standard `081`–`084`; ESTOP is `000`, one reason byte.

Only node 4 → motor 107 is assigned by default. Nodes without `JOINT_MOTOR_ID`
are IMU-only. The optional `--motor-map` only labels feedback; it does not configure
or enable motors. Verify physical assignments before using all-joint commands.

The CSV contains command events, raw IDs/payloads, IMU sequences/axes, motor
position/speed/phase-current/temperature/fault, and missing/stale ages. Missing
feedback is not manufactured as zero. The unchanged checker supplies IMU pairing,
scaling and loss accounting.

## Bench check

Use an unloaded, secured motor with a second person at the physical power cutoff.
Start with zero torque, confirm IMU and motor status, then exercise small positive
and negative requests on one joint. Check targeting, `tstop`, latched ESTOP and
command expiry. Add motors one at a time, then test all-joint requests.

Capture the bus and run the existing IMU checker concurrently:

```bash
candump -L can1 > bench_raw.log
python3 imu_canbus.py --channel can1 --duration 300 --out imu_with_motors.csv
```

Confirm IMU continuity/rate/loss while motors are commanded, unchanged LEDs and
agreement between raw frames and CSV. Hardware validation is still pending.
