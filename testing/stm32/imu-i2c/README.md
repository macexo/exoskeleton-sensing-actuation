# imu-i2c — LSM6DSO32 Bring-Up over I2C

Standalone STM32CubeIDE project that brings up an Adafruit LSM6DSO32 breakout on
I2C1 of a Nucleo-F446RE and streams accelerometer and gyroscope samples to the
host over the ST-LINK virtual COM port.

What the firmware does:

1. Checks `WHO_AM_I`, then software-resets the sensor and waits for the flag to clear.
2. Configures ±16 g / ±2000 dps, block data update on, both at 104 Hz.
3. Polls the accel and gyro data-ready flags and emits one CSV row per sample pair.
4. Blinks LD2 (PA5) at roughly 1 Hz — blinking means samples are still going out.

The I2C transport itself is the shared port from [`apis/imu`](../../../apis/imu);
this project only exercises it.

## Hardware

| Part | Notes |
| --- | --- |
| STM32 Nucleo-F446RE | I2C1 at 400 kHz (Fast mode), USART2 at 115200 8N1 |
| Adafruit LSM6DSO32 breakout | On-board 3.3 V regulator, level shifting, and 10 kΩ SCL/SDA pull-ups |

## Pin Connections

| Nucleo pin | Nucleo header | LSM6DSO32 breakout | Signal |
| --- | --- | --- | --- |
| 3V3 | CN6 pin 4 (or CN7 pin 16) | VIN | 3.3 V supply |
| GND | CN6 pin 6 (or CN7 pin 20) | GND | Ground |
| PB6 | CN10 pin 17 (Arduino D10) | SCL | I2C1_SCL |
| PB7 | CN7 pin 21 | SDA | I2C1_SDA |

Four wires total — no external pull-ups are needed, the breakout has its own.

Leave the breakout's `AD0`/`SDO` pin unconnected. It idles low, giving slave
address **0x6A** (`LSM6DSO32_I2C_ADD_L`, what `main.c` uses); tying it to 3V3
selects 0x6B and requires changing `imu_bus` to `LSM6DSO32_I2C_ADD_H`.

`CS` is unconnected as well — the breakout pulls it high, which is what keeps
the part in I2C mode. `INT1`/`INT2` are unused here; this project polls.

UART needs no wiring: PA2/PA3 are hard-wired to the on-board ST-LINK, so the
same USB cable that flashes the board carries the data stream.

## Build and Run

1. **STM32CubeIDE → File → Open Projects from File System…**, select this folder
   (the one with `.project` and `.ioc`), and import it. Import this folder, not
   the repository root.
2. Build with **Project → Build All**.
3. Connect the Nucleo over USB and flash with **Run → Debug**.
4. Close the debugger's serial view (only one program can hold the port), then
   run the Python script below.

If `imu_init()` traps in `Error_Handler()`, the sensor did not answer with the
expected `WHO_AM_I` — check the four wires and the address before anything else.

## Python Script

[`Scripts/imu_uart.py`](Scripts/imu_uart.py) reads the stream off the virtual COM
port and prints it as a table. The firmware sends integer mg and mdps; the script
converts to seconds, g, and dps on the way out.

```bash
pip install pyserial

python Scripts/imu_uart.py --list                  # find the port
python Scripts/imu_uart.py --port COM5             # Windows
python Scripts/imu_uart.py --port /dev/ttyACM0     # Linux
```

`--list` prints every serial port with its description — the Nucleo shows up as
an ST-LINK virtual COM port. `--baud` defaults to 115200 and only needs changing
if `huart2.Init.BaudRate` does. Stop with Ctrl-C.

```text
===================================================================================
   Time (s)      ax (g)      ay (g)      az (g)    gx (dps)    gy (dps)    gz (dps)
===================================================================================
      1.043      -0.012       0.035       1.002      -0.262       0.175      -0.087
      1.053      -0.009       0.033       0.998      -0.175       0.210      -0.052
```

At rest and flat, one accel axis should read close to ±1 g and the other two near
zero, with all three gyro axes near zero.

### Logging to CSV

`--out` writes the same rows to a file as it prints them:

```bash
python Scripts/imu_uart.py --port COM5 --out run.csv
```

The file holds the displayed values — seconds, g, and dps to three decimals —
under the same column headings. Runs append rather than overwrite, and the header
is written only when the file is new, so several runs can share one file.
