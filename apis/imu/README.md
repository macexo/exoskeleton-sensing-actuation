# LSM6DSO32 IMU Driver

STM32 HAL bus ports for the [LSM6DSO32](https://www.st.com/en/mems-and-sensors/lsm6dso32.html) 6-axis IMU, built on ST's [open-source, platform-independent C driver](https://github.com/STMicroelectronics/lsm6dso32-pid).

```
sensors/
├─ inc/
│  ├─ lsm6dso32_reg.h    ST vendor driver, unmodified
│  └─ lsm6dso32_port.h   ctx constructors + bus descriptors
└─ src/
   ├─ lsm6dso32_reg.c    ST vendor driver, unmodified
   ├─ lsm6dso32_port_spi.c
   └─ lsm6dso32_port_i2c.c
```

## Design

ST's driver (`lsm6dso32_reg.c/.h`) is portable because every call takes a `stmdev_ctx_t` carrying three function pointers — `read_reg`, `write_reg`, and an optional `mdelay` (millisecond delay, equivalent to `HAL_Delay`) — plus a generic `void *handle` for whatever platform state those functions need.

To port it, we only implement those functions — `lsm6dso32_port_spi.c` / `lsm6dso32_port_i2c.c`.

Each port exposes exactly one public function — `lsm6dso32_spi_ctx()` or `lsm6dso32_i2c_ctx()` — that builds and returns a fully populated `stmdev_ctx_t`, with our functions wired in as the pointers and `ctx.handle` pointing to a small bus-descriptor struct that bundles the HAL handle with the extra piece its bus needs:

```c
typedef struct { SPI_HandleTypeDef *hspi; GPIO_TypeDef *cs_port; uint16_t cs_pin; } lsm6dso32_spi_bus_t;
typedef struct { I2C_HandleTypeDef *hi2c; uint16_t addr8; }                        lsm6dso32_i2c_bus_t;
```

Both are declared in `lsm6dso32_port.h`, each guarded by `HAL_SPI_MODULE_ENABLED` / `HAL_I2C_MODULE_ENABLED` (set in `stm32f4xx_hal_conf.h` by CubeMX).

## Usage

```c
#include "lsm6dso32_port.h"

/* SPI */
static lsm6dso32_spi_bus_t imu_bus = { &hspi2, IMU_CS_GPIO_Port, IMU_CS_Pin };
stmdev_ctx_t imu = lsm6dso32_spi_ctx(&imu_bus);

/* I2C */
static lsm6dso32_i2c_bus_t imu_bus = { &hi2c1, LSM6DSO32_I2C_ADD_L };
stmdev_ctx_t imu = lsm6dso32_i2c_ctx(&imu_bus);
```

The bus descriptor **must have static or file-scope storage** — `stmdev_ctx_t` only stores a pointer to it, so a stack-local descriptor dangles the moment the constructor returns.
