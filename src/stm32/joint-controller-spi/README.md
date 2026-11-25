# SPI joint controller

STM32F446 firmware for the Sensing PCB. Each board streams its LSM6DSO32 IMU over
CAN at 417 Hz. A board assigned an AK70-9 also takes torque commands from the Pi
and reports the motor's status back.

> [!WARNING]
> This firmware moves motors. Follow the
> [safety rules](../../../CONTRIBUTING.md#safety-rules-for-actuation-testing): a second
> person at the power cut-off, motors unloaded and clamped, ESTOP checked before any
> torque.

## Hardware

| Function | Setting |
| --- | --- |
| MCU | STM32F446RET6, HSI + PLL, 180 MHz |
| CAN1 | PA11 RX, PA12 TX; 1 Mbit/s, auto retransmission and bus-off recovery |
| IMU | LSM6DSO32 on SPI3 (PC10 SCK, PB4 MISO, PB5 MOSI, PB6 CS), INT1 on PB1; ±16 g, ±2000 dps, 417 Hz |
| LED1 | PC13, active high |
| Motor | CubeMars AK70-9 in servo mode |

## Per-board settings

Before flashing each board, edit the two marked lines in the **PER-BOARD SETTINGS**
block at the top of `Core/Src/main.c`:

```c
#define BOARD_NODE_ID          2U    /* <-- EDIT 1 of 2 */
#define BOARD_JOINT_MOTOR_ID   105   /* <-- EDIT 2 of 2, 0 = IMU only */
```

| Node | Joint | `BOARD_NODE_ID` | `BOARD_JOINT_MOTOR_ID` | IMU accel / gyro | Motor status |
| --- | --- | --- | --- | --- | --- |
| 1 | left hip | `1U` | `104` | `0x190` / `0x210` | `0x110` |
| 2 | right hip | `2U` | `105` | `0x1A0` / `0x220` | `0x120` |
| 3 | left knee | `3U` | `106` | `0x1B0` / `0x230` | `0x130` |
| 4 | right knee | `4U` | `107` | `0x1C0` / `0x240` | `0x140` |

Node IDs and motor IDs must each be unique on the bus. Confirm each motor's address
with the CubeMars tool before flashing. Preprocessor symbols `NODE_ID` and
`JOINT_MOTOR_ID` in a CubeIDE build configuration override both lines.

## Wiring

```text
[ Pi CAN HAT ]--[ node 1 ]--[ node 2 ]--[ node 3 ]--[ node 4 ]
   120 ohm                                           120 ohm
```

One linear bus, CAN_H and CAN_L twisted, common ground to every board. Terminate
only the two ends. With power off, CAN_H to CAN_L should read about **60 ohm**
(120 ohm: one end missing; 40 ohm or less: too many fitted).

## LED1

| Pattern | Meaning |
| --- | --- |
| `NODE_ID` slow flashes after reset | Startup; count them to confirm the node |
| Toggles every 500 ms | IMU pairs are being sent and ACKed |
| Solid on | CAN send or IMU read failed, or no sample for 100 ms |
| Fast blink forever | Startup failure (`Error_Handler`) |

A blinking LED only proves *some* node ACKed. Use the Pi scripts to confirm the Pi
receives.

## CAN messages

11-bit IDs are `type << 7 | node << 4 | dest` (dest 0 = Pi). Payloads are little
endian unless noted.

| Message | ID | DLC | Payload |
| --- | --- | --- | --- |
| ESTOP | `0x000` (any type-0 ID) | 1 | reason. Latches 0 A until reset |
| Global reset | `0x067` | any | Boards command 0 A, then reset (clears ESTOP) |
| Torque | `0x080 + node` | 2 | int16 Nm × 1000 |
| Motor status | `0x100 + 0x10·node` | 8 | int16 position (0.1°), int16 speed (10 ERPM), int16 current (0.01 A), int8 °C, uint8 fault |
| IMU accel | `0x180 + 0x10·node` | 8 | uint16 sequence, int16 ax, ay, az (× 0.000488 g) |
| IMU gyro | `0x200 + 0x10·node` | 8 | same sequence, int16 gx, gy, gz (× 0.070 dps) |
| AK70-9 feedback | extended `0x2900 + motor ID` | 8 | CubeMars servo protocol, big endian |

**Motor behaviour:**

- Torque is converted to current at 1.431 Nm/A and clamped to ±5 A (about ±7.2 Nm).
- The board refreshes the current every 50 ms. It requests 0 A if no torque command
  arrives for **200 ms**, so the Pi must keep repeating commands.
- ESTOP and a command FIFO overrun latch the stop until reset.
- Status is sent at up to 20 Hz, only on fresh feedback. A fault change is sent
  straight away.
- The encoder origin is zeroed on the first feedback after boot.
- Set the motor's own CAN timeout too: a broken bus can't deliver a 0 A command.

IMU alone is 40–45% of the bus with four boards. Keep each motor's feedback rate at
100–200 Hz. If the bus saturates, node 4's gyro frames are delayed first.

## Testing

### Terminals

| Terminal | Where | Used for |
| --- | --- | --- |
| **PC** | STM32CubeIDE | Edit IDs, build, flash, watch debugger counters |
| **T1** | Pi | CAN setup, `candump`, `cansend`, killing scripts |
| **T2** | Pi | `imu_canbus.py`: IMU rate and loss per node |
| **T3** | Pi | `motor_console.py`: torque commands and motor status |

T1–T3 can be SSH sessions or `tmux` panes. Copy the whole `Scripts/` folder to the
Pi: `motor_console.py` imports `console_telemetry.py`, which imports `imu_canbus.py`.
Python 3.9+, standard library only.

### 1. Prepare the motors (CubeMars tool, once)

Servo mode, CAN ID from the table above, feedback rate 100–200 Hz, and the motor's
own CAN timeout enabled. Label each motor with its ID.

### 2. Flash each board (PC)

1. **File → Open Projects from File System…** → this folder.
2. Set `BOARD_NODE_ID` and `BOARD_JOINT_MOTOR_ID` for the board.
3. **Project → Build All**, then **Run → Debug** over the board's SWD header.
4. Count the LED1 startup flashes, then label the board.
5. Repeat for every board.

### 3. Bring up CAN (T1)

```bash
sudo apt install can-utils            # first time only
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 listen-only off restart-ms 100
sudo ip link set can1 up
```

### 4. IMU only, motor power off

Connect one board at a time and check its frames appear before adding the next:

```bash
# T1
candump can1,190:7FF,1A0:7FF,1B0:7FF,1C0:7FF,210:7FF,220:7FF,230:7FF,240:7FF
```

With all boards connected, run a baseline:

```bash
# T2
python3 imu_canbus.py --channel can1 --duration 30
```

Expect about 417 Hz, 0 missing and 0 jumps for each node, and `PASS` for each node.
Add `--nodes 1,2` if only some boards are connected.

Optional, still without motor power: inject fake feedback for node 4's motor and
check the board forwards it as `0x140` carrying `FB FF 02 00 9C FF 28 01`
(−0.5°, 20 ERPM, −1 A, 40 °C, fault 1):

```bash
# T1
cansend can1 0000296B#FFFB0002FF9C2801
```

### 5. One motor on the bench

Start the terminals in this order, then power **one** motor:

```bash
# T1: raw log of the whole bus
candump -L can1 > bench_raw.log

# T2: IMU checker, until Ctrl-C
python3 imu_canbus.py --channel can1

# T3: motor console; --out must be a new file each run
python3 motor_console.py --channel can1 --out bench_001.csv --motor-map 1:104,2:105,3:106,4:107
```

`--motor-map` only labels feedback; without it the console assumes only node 4 has a
motor. Steps for node 4 (use the matching number for other joints):

| Step | Terminal and command | Check |
| --- | --- | --- |
| 1 | T3: `m4` | Status is fresh; turning the shaft by hand changes position |
| 2 | T3: `t 4 0` | Current near 0 A |
| 3 | T3: `estop`, then `t 4 0.1` | Motor must **not** move |
| 4 | T1: `cansend can1 067#` | Boards reset and the ESTOP latch clears |
| 5 | T3: `t 4 0.1`, then `m4` | Motor pushes gently, about 0.07 A |
| 6 | T3: `t 4 -0.1` | Pushes the other way |
| 7 | T3: `tstop` | Motor goes slack |
| 8 | T3: `t 4 0.1`, then T1: `pkill -9 -f motor_console.py` | Motor goes slack within **200 ms** (command timeout) |
| 9 | T3 (restarted with a new `--out`): `t 4 0.1`, `l4` | IMU sequence keeps increasing under torque |

Steps 4 and 8 need a free shell: stop the T1 `candump` first, or open another one.
Throughout, T2 should stay at about 417 Hz with 0 missing for that node. The reset in
step 4 adds sequence jumps on every node, which is expected; restart T2 afterwards.

`quit` and Ctrl-C in the console send 0 Nm before exiting, so only `pkill -9` tests
the firmware's command timeout.

### 6. Add the other motors

Power them one at a time and repeat steps 1, 5, 6 and 7 for each joint. Then:

```text
t_all 0.1
status
tstop
```

### 7. Five-minute run with all motors

```bash
# T2 (restarted after any reset)
python3 imu_canbus.py --channel can1 --duration 300 --out imu_with_motors.csv

# T3: keep torque applied for the whole run
t_all 0.1
```

T2 stops after 300 s and must print `PASS` for every node. Then `tstop` and `quit`
in T3, and Ctrl-C in T1. Keep `bench_raw.log`, `bench_00N.csv` and
`imu_with_motors.csv`.

### Console commands (T3)

| Command | Action |
| --- | --- |
| `t <joint> <Nm>` | Stream torque to one joint, refreshed every 50 ms |
| `t_all <Nm>` | Stream torque to all four joints |
| `tstop` | Clear requests and send 0 to all joints |
| `estop` | Broadcast latched ESTOP |
| `m1`–`m4` | Latest motor status and its age |
| `l1`–`l4`, `q1`–`q4` | Latest IMU pair, or the recent queue |
| `status`, `help`, `quit` | Summary, help, stop and exit |

More detail in [Scripts/motor_console_README.md](Scripts/motor_console_README.md).

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `FAIL (no data)` | Power, wiring, or wrong `BOARD_NODE_ID` |
| `FAIL (sequence jumps)` | The board reset (ESTOP reset, brown-out under motor current) or two boards share a node ID |
| No motor status | Motor ID doesn't match `BOARD_JOINT_MOTOR_ID`, motor not in servo mode, or unpowered |
| IMU loss only with motors running | T1: `ip -details -statistics link show can1` for bus errors. Non-zero `socket drops` in T2 means the Pi fell behind |
| `[FATAL] ... exists` in T3 | The `--out` file already exists; use a new name |

Debugger counters (PC):

| Counter | Non-zero means |
| --- | --- |
| `can_pairs_dropped` | Board couldn't send |
| `imu_read_errors` | SPI problem |
| `imu_drdy_overruns` | Main loop overloaded |
| `can_last_esr` | CAN error state |

## Origin

This project combines `testing/stm32/pcb-imu-canbus` (SPI IMU, CAN IDs, LED, Pi
checker) with exoskeleton-embedded's `src/stm32/joint-controller` at `6a25ef9`
(torque frame, AK70-9 current control, ESTOP, global reset). It adds:

- the 200 ms command timeout
- the forwarded motor status frame
- per-board motor IDs

It drops the old LSM6DS3TR I²C driver and the USART diagnostics. Motor position now
travels in the status frame instead of the gyro frame.
