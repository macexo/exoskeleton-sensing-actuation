/**
 * @file    lsm6dso32_port_i2c.c
 * @brief   I2C transport for the LSM6DSO32 driver (STM32 HAL).
 *
 * Bus requirements: 7-bit addressing, <= 400 kHz (Fast mode), external pull-ups on SCL/SDA.
 *
 * The slave address is selected by the SA0 pin: 
 * low -> LSM6DSO32_I2C_ADD_L, high -> LSM6DSO32_I2C_ADD_H
 */

#include "lsm6dso32_port.h"

#if defined(HAL_I2C_MODULE_ENABLED)

#define LSM6DSO32_I2C_TIMEOUT_MS  100

static int32_t lsm6dso32_i2c_write(void *handle, uint8_t reg,
                                    const uint8_t *buf, uint16_t len)
{
  lsm6dso32_i2c_bus_t *bus = (lsm6dso32_i2c_bus_t *)handle;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(bus->hi2c, bus->addr8,
                                                reg, I2C_MEMADD_SIZE_8BIT,
                                                (uint8_t *)buf, len,
                                                LSM6DSO32_I2C_TIMEOUT_MS);
  return (status == HAL_OK) ? 0 : -1;
}

static int32_t lsm6dso32_i2c_read(void *handle, uint8_t reg,
                                   uint8_t *buf, uint16_t len)
{
  lsm6dso32_i2c_bus_t *bus = (lsm6dso32_i2c_bus_t *)handle;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(bus->hi2c, bus->addr8,
                                               reg, I2C_MEMADD_SIZE_8BIT,
                                               buf, len,
                                               LSM6DSO32_I2C_TIMEOUT_MS);
  return (status == HAL_OK) ? 0 : -1;
}

static void lsm6dso32_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

stmdev_ctx_t lsm6dso32_i2c_ctx(lsm6dso32_i2c_bus_t *bus)
{
  stmdev_ctx_t ctx = {
    .write_reg = lsm6dso32_i2c_write,
    .read_reg  = lsm6dso32_i2c_read,
    .mdelay    = lsm6dso32_delay_ms,
    .handle    = bus,
  };
  
  return ctx;
}

#endif /* HAL_I2C_MODULE_ENABLED */
