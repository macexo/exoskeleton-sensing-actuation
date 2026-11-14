# McMaster Exoskeleton — Sensing & Actuation

Embedded code for the sensing and actuation subsystem of the McMaster Exoskeleton.

This subsystem is the loop that makes the exoskeleton move: it reads the IMUs on each
joint, runs the control logic on an STM32, and commands the actuators over CAN at 1 Mbit/s.

> [!WARNING]
> This code drives motors on a device a person wears. Read the
> [safety rules](CONTRIBUTING.md#safety-rules-for-actuation-testing) before you flash
> anything that can move a joint.

## Hardware

| Role | Part | Interface | Notes |
| --- | --- | --- | --- |
| MCU (dev board) | STM32 Nucleo-F446RE | — | Cortex-M4F @ 180 MHz |
| MCU (custom PCB) | STM32F446RET6 on the exo PCB | CAN1 (PA11 RX, PA12 TX); SPI3 to the IMU | 180 MHz from internal HSI + PLL; LED1 on PC13 for status. Bring-up projects are `testing/stm32/pcb-*` |
| Host | Raspberry Pi 5 | CAN at 1 Mbit/s through a CAN HAT (`can1`) | Receives IMU telemetry from the PCB. |
| Actuator | CubeMars AK70-9 KV60 | CAN at 1 Mbit/s (29-bit extended IDs) | Servo mode + MIT impedance control |
| IMU | LSM6DSO32 | SPI or I2C | 6-axis; ±32 g range |

## Repository Structure

```text
exoskeleton-sensing-actuation/
├── apis/                     # Reusable hardware APIs, one folder per domain
│   └── <domain>/             # e.g. motor, imu, can
│       ├── inc/              # Public headers
│       ├── src/              # Implementation
│       ├── docs/             # Supporting notes and diagrams
│       ├── python/           # Host-side helpers for this API, where useful
│       └── <domain>-api.md   # API reference
├── src/                      # Fully working control loops — what runs on the exo
│   ├── stm32/                # Controller projects (STM32CubeIDE)
│   └── pi/                   # Raspberry Pi side (Python)
├── testing/                  # Standalone STM32 projects for bring-up and testing
│   ├── stm32/                # One CubeIDE project per peripheral or API under test
│   ├── pi/                   # Host- and Pi-side test scripts
│   └── discovery/            # Scratch projects for evaluating new parts
├── CONTRIBUTING.md           # Workflow, coding standards, safety rules
└── README.md
```

## Quick Start

```bash
git clone https://github.com/macexo/exoskeleton-sensing-actuation.git
cd exoskeleton-sensing-actuation
```

1. Open **STM32CubeIDE** → **File → Open Projects from File System…** and select a
   project folder under `testing/stm32/` or `src/stm32/` (the one containing `.project`
   and `.ioc`). Import the project folder, not the repository root — each project is
   standalone.
2. Build with **Project → Build All**. A fresh clone should compile with no warnings.
3. Connect the Nucleo over USB and flash with **Run → Debug**.
4. Run the matching Python script to talk to the board over UART and confirm it responds.

If you are new to the repo, start with a project under `testing/stm32/` — they are small,
self-contained, and exercise one peripheral each.

Full setup, prerequisites, and troubleshooting are in
[CONTRIBUTING.md](CONTRIBUTING.md#getting-started).

## Projects

| Project | Description | Status |
| --- | --- | --- |
| `apis/imu` | LSM6DSO32 IMU API — STM32 HAL SPI and I2C ports over ST's platform-independent vendor driver | Complete |
| `apis/motor` | AK70-9 motor API — servo-mode and MIT impedance control with feedback parsing over CAN bus | Complete |
| `testing/stm32/imu-i2c` | LSM6DSO32 bring-up over I2C — `WHO_AM_I` check, 104 Hz ODR, polled data-ready, samples streamed over UART | Complete |
| `testing/stm32/imu-spi` | LSM6DSO32 bring-up over SPI2 in 4-wire mode — same configuration and polled data-ready as the I2C project | Complete |
| `testing/stm32/pcb-test-led` | Custom STM32F446RE PCB smoke test — toggles LED1 (PC13) every 2 s to confirm the board boots and flashes | Complete |
| `testing/stm32/pcb-test-can` | PCB CAN1 transmit test — sends a fixed frame (ID `0x065`) every 250 ms at 1 Mbit/s; LED1 shows whether frames are ACKed | Complete |
| `testing/stm32/pcb-test-imu` | PCB LSM6DSO32 bring-up over SPI3 — `WHO_AM_I` check with fault hints, 104 Hz polled samples streamed as CSV over SWO | Complete |
| `testing/stm32/pcb-imu-can` | PCB IMU-over-CAN telemetry — 417 Hz interrupt-driven accel/gyro pairs on IDs `0x065`/`0x066` at 1 Mbit/s, with a Raspberry Pi 5 receiver script (`imu_can.py`) for live view and CSV logging | Complete |

Update a row the moment its code lands here, and add a row before you start a new project so nobody
duplicates your work.

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first PR. It covers the branch and
commit conventions, the C style used here, the CubeMX user-code rules that keep your work
from being deleted on regeneration, and the actuation safety rules.

## References

- [STM32F446RE reference manual (RM0390)](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [CubeMars AK70-9](https://www.cubemars.com/data/cms/202602/ak-series-prodcut-manual-v3-2-0-for-ak-3-0-robotic-actuator.pdf)
- [LSM6DSO32 datasheet](https://www.st.com/en/mems-and-sensors/lsm6dso32.html)
