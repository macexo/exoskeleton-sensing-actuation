# pcb-test-can

CAN transmit test for the STM32F446RE PCB. The board sends one CAN frame every 250 ms and uses LED1 to show whether the frames are being acknowledged on the bus.

## CAN frame

| Setting   | Value                          |
|-----------|--------------------------------|
| Peripheral| CAN1 (PA11 = RX, PA12 = TX)    |
| Bit rate  | 500 kbit/s                     |
| ID        | Standard, 101 (0x065)          |
| DLC       | 8                              |
| Data      | `DE AD BE EF 00 00 00 00`      |
| Interval  | 250 ms                         |

## LED1 status

| LED1                     | Meaning                                                                 |
|--------------------------|-------------------------------------------------------------------------|
| Blinking every 250 ms    | Frames are being sent and acknowledged by another node                  |
| Solid on                 | Frames are not being acknowledged (no other node, wiring, or bit rate)  |
| Fast blink               | Startup error (`Error_Handler()`), e.g. `HAL_CAN_Start()` failed because the transceiver is unpowered or CAN RX is not idle |

## Bench setup

The board needs at least one other node on the bus to acknowledge its frames, e.g. a USB-CAN adapter set to 500 kbit/s. With nothing else connected, LED1 stays solid on. Make sure the bus is terminated (120 Ω at each end).

Received frames are not processed. No CAN filter is configured, so incoming frames are discarded.
