# SPI joint controller

STM32F446 joint firmware for the Sensing PCB. The LSM6DSO32 streams raw
accelerometer and gyro pairs over CAN. A board assigned an AK70-9 also accepts
manual torque commands and forwards motor feedback to the Pi.

## Hardware and firmware

| Function | Setting |
| --- | --- |
| MCU | STM32F446RET6, internal HSI + PLL, 180 MHz |
| CAN1 | PA11 RX, PA12 TX; 1 Mbit/s, normal mode, auto retransmission, auto bus-off recovery |
| SPI3 | PC10 SCK, PB4 MISO, PB5 MOSI; mode 3, 5.625 MHz |
| IMU CS | PB6, software controlled, idle high |
| IMU INT1 | PB1, EXTI1 rising edge, pull-down; gyro data-ready, pulsed |
| LED1 | PC13, active high |
| IMU | LSM6DSO32, +/-16 g, +/-2000 dps, both 417 Hz, interrupt driven |
| IMU filters | accel LPF2 at ODR/20 (~21 Hz), gyro LPF1 AGGRESSIVE (24.6 Hz) |

## Node IDs

| Node | Joint | `NODE_ID` constant | Accel ID | Gyro ID |
| --- | --- | --- | --- | --- |
| 1 | left hip | `CAN_NODE_LEFT_HIP` | `0x190` | `0x210` |
| 2 | right hip | `CAN_NODE_RIGHT_HIP` | `0x1A0` | `0x220` |
| 3 | left knee | `CAN_NODE_LEFT_KNEE` | `0x1B0` | `0x230` |
| 4 | right knee | `CAN_NODE_RIGHT_KNEE` | `0x1C0` | `0x240` |

Set the CubeIDE C preprocessor symbol `NODE_ID` per board. The default is
`CAN_NODE_RIGHT_HIP` (2). Confirm the physical motor address before flashing.
Right knee (4) defaults to motor 107, the pairing in the source controller.
Other joints default to IMU-only. Define `JOINT_MOTOR_ID` to enable an assigned
motor, or set it to 0 to build any board for IMU-only testing.

IDs are `type << 7 | node << 4 | dest`, the layout in the joint controller's
`can_common.h` (type 3 = IMU accel, 4 = IMU gyro, dest 0 = Pi). Only the IDs are
shared: the payload is this project's raw-count format, so the joint controller's
Pi scripts (`pi_can_buffer.py`) cannot decode these frames.

**Every board on the bus needs a different `NODE_ID`.** Two boards with the same ID
corrupt each other's frames. The Pi script reports that as sequence jumps.

## Wiring

```text
[ Pi CAN HAT ]--[ node 1 ]--[ node 2 ]--[ node 3 ]--[ node 4 ]
   120 ohm                                           120 ohm
```

- One linear bus with short stubs. Twist CAN_H with CAN_L, and run a common ground
  to every board.
- Terminate with 120 ohm at the **two physical ends only**: the Pi (many HATs have
  this on a jumper) and the last board. Remove any termination on the middle
  boards. With power off, CAN_H to CAN_L should measure about **60 ohm**. About
  120 ohm means one end is missing; about 40 ohm or less means too many are fitted.
- The Pi needs a CAN controller/transceiver (HAT or USB adapter); do not connect
  CAN_H/CAN_L directly to Pi GPIO.

## LED1

| Pattern | Meaning |
| --- | --- |
| `NODE_ID` slow flashes after reset, then off | Startup; count the flashes to confirm the node |
| Toggles every 500 ms | Sample pairs are being sent and ACKed |
| Solid on | A CAN send or IMU read failed, or no sample for 100 ms |
| Fast blink forever | Startup failure (`Error_Handler`) |

An ACK only means *some* other node accepted the frame. On a shared bus that can be
another IMU board, so a blinking LED does not prove the Pi is receiving. Use the
Pi script for that.

## Wire format

Unchanged from `pcb-imu-can`. Each sample is two **standard 11-bit data frames**,
DLC **8**, all little endian:

| Frame | Bytes 0..1 | Bytes 2..3 | Bytes 4..5 | Bytes 6..7 |
| --- | --- | --- | --- | --- |
| accel | uint16 sequence | int16 accel X | int16 accel Y | int16 accel Z |
| gyro | same sequence | int16 gyro X | int16 gyro Y | int16 gyro Z |

Raw counts: multiply acceleration by **0.000488** for g and angular velocity by
**0.070** for degrees/s. Each board's sequence starts at zero, wraps after 65535,
and advances for every captured sample even if sending failed.

Example, node 2, sequence 42, acceleration (0, 0, 2049), gyro (-1, 0, 1):

```text
1A0#2A00000000000108
220#2A00FFFF00000100
```

Bus load: 834 frames/s per board, about 3,340 frames/s for four, which is roughly
40-45% of a 1 Mbit/s bus. Accel frames outrank gyro frames, and lower node numbers
outrank higher ones, so if the bus is ever overloaded node 4's gyro frames are
the first to suffer.

## Receive on Raspberry Pi 5

Configure the interface at 1 Mbit/s (substitute your interface name):

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 listen-only off restart-ms 100
sudo ip link set can1 up
```

Raw check that all four nodes are talking (`can-utils`):

```bash
candump can1,190:7FF,1A0:7FF,1B0:7FF,1C0:7FF,210:7FF,220:7FF,230:7FF,240:7FF
```

Copy `Scripts/imu_canbus.py` to the Pi and run it with Python 3 (no pip packages):

```bash
python3 imu_canbus.py --list                                  # find the interface
python3 imu_canbus.py --channel can1                          # live summary, Ctrl-C for the result
python3 imu_canbus.py --channel can1 --duration 300 --out run.csv
python3 imu_canbus.py --channel can1 --nodes 1,2              # only some boards connected
python3 imu_canbus.py --channel can1 --show 3                 # every row from node 3
```

Every second the console shows, per node: receive rate (expect about 417 Hz),
pairs, missing pairs, loss %, sequence jumps, status (`OK`, `STALE`, `WAITING`)
and the newest values in g and dps. Events print as they happen:
`[NODE]` on a node's first pair, `[STALE]` after 0.5 s of silence, `[ALIVE]` when
it resumes.

On exit the script prints a result per node and exits 0 only if every node passed:

| Result | Meaning |
| --- | --- |
| `PASS` | Data received, loss at or under `--max-loss` (default 0.1%), no jumps |
| `FAIL (no data)` | Nothing from that node: power, wiring, or wrong `NODE_ID` |
| `FAIL (loss over ...)` | Samples were lost; see below |
| `FAIL (sequence jumps ...)` | Sequence broke: the board reset, or two boards share a `NODE_ID` |

`socket drops` counts frames the Pi itself dropped because the script fell behind,
from the kernel's socket overflow counter. If it is non-zero, some of the missing
pairs were lost on the Pi, not on the bus. Loss with zero socket drops is on the
bus or the board: check `ip -details -statistics link show can1` for bus errors and
each board's `can_pairs_dropped` and `can_last_esr`.

CSV columns: receive time in seconds since the script started (kernel receive
time, not MCU sample time), node, sequence, ax/ay/az in g, gx/gy/gz in degrees/s,
and that node's cumulative missing pairs. The CSV uses 6 decimal places.

## Test procedure

1. Flash each board with its own `NODE_ID` and count the startup flashes.
2. Connect one board at a time. Confirm its ID pair shows up in `candump` before
   adding the next.
3. With all four up, run `imu_canbus.py --duration 300 --out run.csv`.

## Joint-controller port

The source was `macexo/exoskeleton-embedded` at `6a25ef9`, under
`src/stm32/joint-controller/Core`. The first port commit imports
`can_common`, `can_motor`, and `can/ak70_9` headers and sources.
The controller loop ports torque-to-current conversion, periodic refresh,
latched e-stop, global reset, and temporary encoder zeroing on first feedback.
The application polls filtered CAN FIFOs with commands first and checks
whether the current and origin commands could be queued.

The board setup, SPI3 driver, interrupt-driven sampling, sequenced IMU
payloads, and LED patterns come from `pcb-imu-canbus`. The older controller
used I²C/DMA, embedded position in the gyro frame, and printed diagnostics
over UART. This project reports motor information on a dedicated CAN frame.

### Manual Pi contract

| Message | Identifier and payload |
| --- | --- |
| Torque | Standard `0x080 + NODE_ID`, DLC 2. Signed little-endian millinewton-metres. |
| E-stop | Standard type 0 broadcast (Pi ID `0x000`). Latches zero current until reset. |
| Global reset | Standard `0x067`. Motor-enabled boards request zero current and wait for their transmit mailboxes to drain before resetting. |
| Motor feedback | Extended `0x900 + JOINT_MOTOR_ID`, DLC 8. Queues temporary origin on first matching frame. |
| Motor status | Standard `0x100 + (NODE_ID << 4)`, DLC 8. Three signed little-endian int16 fields (0.1° position, 10 ERPM speed, 0.01 A current), signed °C in byte 6, fault code in byte 7. |

Current is torque divided by 1.431 Nm/A, limited to ±5 A. The STM32 refreshes
current every 50 ms. The Pi must repeat active torque commands, for example
every 50 ms. After 200 ms without a new command the STM32 requests zero.
A one-shot manual command therefore expires. E-stop and command FIFO overflow
latch the stop. Configure and verify the motor driver's own CAN timeout on
the bench because a disconnected bus cannot deliver a zero-current command.

Status is forwarded only on fresh matching feedback, up to 20 Hz during
steady operation. Fault-code changes bypass the rate limit. The IMU capture
script above decodes the raw IMU pairs. Use the Embedded Pi
`can_motor.parse_motor_status` decoder for the separate status frames.

### Motor bench verification (pending)

First use a CAN monitor with motor power disconnected. Check the assigned
address, current clamp, zero on command expiry, latched e-stop, and rejection
of torque after e-stop. Inject a matching status frame and a wrong-address
frame. Only the matching frame should produce telemetry. For right knee,
extended `0x96B` carrying `FF FB 00 02 FF 9C 28 01` should yield standard
`0x140` carrying `FB FF 02 00 9C FF 28 01` (−0.5°, 20 ERPM, −1 A, 40 °C,
fault 1). Then follow the repository's unloaded two-person bench procedure
and capture IMU rate and loss with motor traffic present.
