# pcb-imu-can

Standalone STM32CubeIDE project based on `pcb-test-can`, adding the SPI IMU
transport from `pcb-test-imu`. Streams LSM6DSO32 acceleration and gyro data to a
Raspberry Pi 5 over classic CAN at **500 kbit/s**. This is a bench telemetry
protocol; it does not implement the suit's existing joint-control protocol.

## Hardware and firmware

| Function | Setting |
| --- | --- |
| MCU | STM32F446RET6, internal HSI + PLL, 180 MHz |
| CAN1 | PA11 RX, PA12 TX; 500 kbit/s, normal mode |
| SPI3 | PC10 SCK, PB4 MISO, PB5 MOSI; mode 3, 5.625 MHz |
| IMU CS | PB6, software controlled, idle high |
| LED1 | PC13, active high |
| IMU | LSM6DSO32, +/-16 g, +/-2000 dps, both 104 Hz, polling |

Use the SPI board revision matching these pins. Connect CAN_H, CAN_L and a
common ground to the Pi's CAN interface, with 120 ohm termination at each end.
The Pi needs a CAN controller/transceiver (HAT or USB adapter); do not connect
CAN_H/CAN_L directly to Pi GPIO. Use the same hardware that acknowledged
`pcb-test-can`, at the same 500 kbit/s rate and with listen-only disabled.

## Import, build and flash

1. In STM32CubeIDE, **File > Open Projects from File System**, select this folder.
2. Build the `pcb-imu-can` project and flash/debug that project.
3. Resume execution after the debugger stops at `main`.

No UART, SWO or debugger is required to stream data. LED1 toggles every 500 ms
while complete sample pairs are acknowledged. Solid on means a CAN send or IMU
read failed, or no fresh sample arrived for 100 ms. Fast blink means a startup
failure. A CAN ACK confirms a receiver controller accepted the frame on the bus,
not that a Pi application has decoded it.

Startup verifies WHO_AM_I == 0x6C and checks all sensor configuration writes.
Reset and CAN waits are bounded. Failed reads are never transmitted. CAN uses
single-attempt transmission (no automatic retransmission) and automatic bus-off
recovery. Dropped samples are not queued for later replay.

Debugger variables: `imu_whoami`, `imu_samples`, `imu_read_errors`,
`can_pairs_sent`, `can_pairs_dropped`, `can_last_esr`, `can_last_tsr`.
The two CAN snapshots describe the most recent completed/aborted attempt.

## Wire format

Each captured sample produces two **standard 11-bit data frames**, DLC **8**.
All multi-byte values use **little endian**:

| CAN ID | Bytes 0..1 | Bytes 2..3 | Bytes 4..5 | Bytes 6..7 |
| --- | --- | --- | --- | --- |
| 0x065 (101) | uint16 sequence | int16 accel X | int16 accel Y | int16 accel Z |
| 0x066 (102) | same sequence | int16 gyro X | int16 gyro Y | int16 gyro Z |

Axis values are raw two's-complement counts. Convert acceleration to **g** by
multiplying by **0.000488**, and angular velocity to **degrees/s** by **0.070**.
These factors match the configured ranges. This preserves sensor precision
without clipping gyro values into a 16-bit mdps field.

Sequence starts at zero, wraps after 65535 and increments for every captured
sample, even if CAN transmission fails. Pair only equal sequences; discard
incomplete pairs. The paired readings are polled sequentially, not hardware
timestamped. No MCU timestamp is transmitted. At 104 sample pairs/s the stream
is 208 frames/s. Give each additional board its own pair of IDs by changing
`IMU_ACCEL_CAN_ID` / `IMU_GYRO_CAN_ID` in `Core/Src/main.c`; the default 0x065
also overlaps the old fixed-payload CAN test, so stop that test on other nodes.

Example sequence 42: acceleration (0, 0, 2049), gyro (-1, 0, 1):

```text
065#2A00000000000108
066#2A00FFFF00000100
```

## Receive on Raspberry Pi 5

Assuming the Pi's CAN hardware/driver already exposes `can0`, configure its rate
while no other application relies on the interface:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 listen-only off restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

Raw inspection, if `can-utils` is installed:

```bash
candump can0,065:7FF,066:7FF
```

Copy `Scripts/imu_can.py` to the Pi and run it with Python 3 (no pip packages):

```bash
python3 imu_can.py --channel can0
python3 imu_can.py --channel can0 > imu.csv
```

Output columns are host monotonic receive time (not MCU sample time), sequence,
ax/ay/az in g, gx/gy/gz in degrees/s, and cumulative missing sample pairs.
The missing counter handles normal sequence wrap; MCU resets cannot be reliably
distinguished from packet loss without an additional boot identifier.
Use `--accel-id` and `--gyro-id` if firmware IDs are changed.

## Source maintenance

The `.ioc` records the combined CAN/SPI hardware settings; `Core/Src/main.c` is
the application source. Back it up before CubeMX regeneration: the custom main
is not maintained inside CubeMX USER CODE blocks. Build/flash through CubeIDE
does not regenerate it. Existing `pcb-test-can` and `pcb-test-imu` projects are
unchanged.
