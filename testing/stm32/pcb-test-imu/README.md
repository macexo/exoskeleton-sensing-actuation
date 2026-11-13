# pcb-test-imu

IMU test for the STM32F446RE PCB. The board brings up the LSM6DSO32 over SPI, checks its ID, then streams accelerometer and gyroscope samples as CSV over SWO. LED1 shows whether the stream is running.

## IMU setup

| Setting      | Value                                          |
|--------------|------------------------------------------------|
| Peripheral   | SPI3 (PC10 = SCK, PB4 = MISO, PB5 = MOSI)      |
| Chip select  | PB6 (`IMU_CS`), driven in software             |
| SPI mode     | Mode 3 (CPOL=1, CPHA=1), 5.625 MHz             |
| Accel range  | ±16 g                                          |
| Gyro range   | ±2000 dps                                      |
| Data rate    | 104 Hz (both)                                  |

The IMU_INT1 interrupt on PB1 is not used; the program polls the data-ready flags in the main loop.

## Startup check

At startup the program reads the WHO_AM_I register and prints the result over SWO whether it passes or not:

```
IMU WHO_AM_I: got 0x6C, want 0x6C (bus status 0)
```

If the value is wrong the program stops in `Error_Handler()`. The value points at the fault:

| WHO_AM_I        | Likely cause                                              |
|-----------------|-----------------------------------------------------------|
| `0x6C`          | OK                                                        |
| `0x00`          | MISO stuck low: not connected, or the sensor is not driving it |
| `0xFF`          | CS not reaching the sensor, or the sensor has no power    |
| `0xD8` / `0x36` | Clock settings (CPOL/CPHA) wrong                          |
| Anything else   | Wrong part, or a short on the bus                         |

## Output format

After the startup check, the program prints a header line, then one CSV line per sample:

```
t_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps
```

| Column              | Unit                         |
|---------------------|------------------------------|
| `t_ms`              | Milliseconds since boot      |
| `ax_mg` … `az_mg`   | Acceleration in milli-g      |
| `gx_mdps` … `gz_mdps` | Angular rate in milli-degrees per second |

Output goes to ITM stimulus port 0. If no debugger is reading SWO, the writes return immediately, so the board runs normally without one.

## LED1 status

| LED1                     | Meaning                                                          |
|--------------------------|------------------------------------------------------------------|
| Blinking about once a second | Samples are streaming                                        |
| Solid on                 | Error (`Error_Handler()`): wrong WHO_AM_I, or the reset did not complete |
| Off                      | Streaming has stopped                                            |

## Viewing the SWO output

SWV is enabled in `pcb-test-imu Debug.launch` with the core clock set to 180 MHz. If it is not picked up, tick it under Debug Configurations → Debugger → Serial Wire Viewer.

1. Start a debug session.
2. Open Window → Show View → SWV → SWV ITM Data Console.
3. Click the settings (wrench) icon and enable ITM stimulus port 0.
4. Press the red Start Trace button, then Resume.

Press Start Trace before Resume, or the WHO_AM_I line is missed.
