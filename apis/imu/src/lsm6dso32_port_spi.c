/**
 * @file    lsm6dso32_port_spi.c
 * @brief   SPI transport for the LSM6DSO32 driver (STM32 HAL).
 *
 * Control byte: RW bit in the MSB, AD6:0 below it.
 *
 * Bus requirements: 4-wire SPI mode 3 (CPOL=1, CPHA=1), 8-bit frames, MSB
 * first, <= 10 MHz, with CS driven in software so it stays low across the
 * whole address + data transfer.
 */

#include "lsm6dso32_port.h"

#if defined(HAL_SPI_MODULE_ENABLED)

#define LSM6DSO32_SPI_TIMEOUT_MS  100
#define LSM6DSO32_SPI_READ_BIT    0x80U // MSB of the control byte: 1 = read 

static int32_t lsm6dso32_spi_write(void *handle, uint8_t reg,
                                    const uint8_t *buf, uint16_t len)
{
  lsm6dso32_spi_bus_t *bus = (lsm6dso32_spi_bus_t *)handle;
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(bus->hspi, &reg, 1, LSM6DSO32_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Transmit(bus->hspi, (uint8_t *)buf, len, LSM6DSO32_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_SET);

  return (status == HAL_OK) ? 0 : -1;
}

static int32_t lsm6dso32_spi_read(void *handle, uint8_t reg,
                                   uint8_t *buf, uint16_t len)
{
  lsm6dso32_spi_bus_t *bus = (lsm6dso32_spi_bus_t *)handle;
  HAL_StatusTypeDef status;

  reg |= LSM6DSO32_SPI_READ_BIT;

  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(bus->hspi, &reg, 1, LSM6DSO32_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(bus->hspi, buf, len, LSM6DSO32_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_SET);

  return (status == HAL_OK) ? 0 : -1;
}

static void lsm6dso32_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

stmdev_ctx_t lsm6dso32_spi_ctx(lsm6dso32_spi_bus_t *bus)
{
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_SET); // CS idles high

  stmdev_ctx_t ctx = {
    .write_reg = lsm6dso32_spi_write,
    .read_reg  = lsm6dso32_spi_read,
    .mdelay    = lsm6dso32_delay_ms,
    .handle    = bus,
  };
  return ctx;
}

#endif /* HAL_SPI_MODULE_ENABLED */
