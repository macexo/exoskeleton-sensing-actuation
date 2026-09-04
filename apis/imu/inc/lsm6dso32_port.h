/**
 * @file    lsm6dso32_port.h
 * @brief   STM32 HAL bus ports for the LSM6DSO32 driver (SPI and I2C).
 *
 * One port per transport, each guarded by the HAL module a project has
 * enabled (HAL_SPI_MODULE_ENABLED / HAL_I2C_MODULE_ENABLED, set in
 * stm32f4xx_hal_conf.h by CubeMX). Both ports can sit in the same build
 * unconditionally: whichever module a project has disabled compiles to
 * nothing, so the same source tree serves an SPI board, an I2C board, or
 * one that supports both by populate option.
 *
 * Each constructor takes a bus descriptor rather than hardcoding a
 * peripheral handle, so the same file works on any board and supports
 * more than one sensor instance (e.g. two IMUs on different buses, or two
 * on the same I2C bus at different SA0 addresses).
 *
 * The bus descriptor MUST have static or file-scope storage. stmdev_ctx_t
 * only stores a pointer to it (ctx.handle) -- a stack-local descriptor
 * would dangle the instant the constructor returns.
 *
 * Usage (SPI):
 *   static lsm6dso32_spi_bus_t imu_bus = { &hspi2, IMU_CS_GPIO_Port, IMU_CS_Pin };
 *   stmdev_ctx_t imu = lsm6dso32_spi_ctx(&imu_bus);
 *
 * Usage (I2C):
 *   static lsm6dso32_i2c_bus_t imu_bus = { &hi2c1, LSM6DSO32_I2C_ADD_L };
 *   stmdev_ctx_t imu = lsm6dso32_i2c_ctx(&imu_bus);
 *
 * Either way, allow the sensor's power-on time (see datasheet) before the
 * first register access -- that delay is board/timing policy, so it is
 * left to the caller rather than baked into the port.
 */

#ifndef LSM6DSO32_PORT_H
#define LSM6DSO32_PORT_H

#include "stm32f4xx_hal.h"
#include "lsm6dso32_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(HAL_SPI_MODULE_ENABLED)

/** SPI bus + chip-select pin for one LSM6DSO32. Must outlive the ctx. */
typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef       *cs_port;
  uint16_t            cs_pin;
} lsm6dso32_spi_bus_t;

/**
 * @brief  Build a stmdev_ctx_t for a sensor on the given SPI bus.
 *         Drives CS idle-high as a side effect.
 * @param  bus  Static/file-scope bus descriptor (kept by pointer, not copied).
 * @retval      Ready-to-use ctx; pass to any lsm6dso32_* driver call.
 */
stmdev_ctx_t lsm6dso32_spi_ctx(lsm6dso32_spi_bus_t *bus);

#endif /* HAL_SPI_MODULE_ENABLED */

#if defined(HAL_I2C_MODULE_ENABLED)

/** I2C bus + 8-bit slave address for one LSM6DSO32. Must outlive the ctx. */
typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t            addr8; /* LSM6DSO32_I2C_ADD_L or LSM6DSO32_I2C_ADD_H */
} lsm6dso32_i2c_bus_t;

/**
 * @brief  Build a stmdev_ctx_t for a sensor on the given I2C bus.
 * @param  bus  Static/file-scope bus descriptor (kept by pointer, not copied).
 * @retval      Ready-to-use ctx; pass to any lsm6dso32_* driver call.
 */
stmdev_ctx_t lsm6dso32_i2c_ctx(lsm6dso32_i2c_bus_t *bus);

#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSO32_PORT_H */
