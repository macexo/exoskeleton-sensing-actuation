/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dso32_reg.h"
#include "lsm6dso32_port.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_BOOT_TIME_MS     35  // Turn-on time
#define TX_BUF_SIZE          96  // Enough for one CSV line
#define LED_BLINK_DIV        52  // Samples per LD2 toggle (~1 Hz blink at 104 Hz ODR)
#define IMU_RESET_TIMEOUT_MS 100 // Ceiling on the software-reset flag poll

/* Set to 1, jumper PB5 (MOSI) to PB4 (MISO), and unplug the sensor to test
   the STM32 side of the bus on its own. Set back to 0 for normal operation. */
#define IMU_SPI_LOOPBACK_TEST 0

/* Set to 1 with the sensor wired up as normal. Reports what is on the far
   end of the MISO wire, then pulses CS so it can be caught on a meter. */
#define IMU_SPI_PIN_PROBE 0

/* Set to 1 to read WHO_AM_I forever at ~200 Hz, so a scope has a repeating
   transaction to trigger on. Never returns. Set back to 0 to run normally. */
#define IMU_SPI_SCOPE_LOOP 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static lsm6dso32_spi_bus_t imu_bus = { &hspi3, IMU_CS_GPIO_Port, IMU_CS_Pin };
static stmdev_ctx_t imu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void imu_init(void);
static void imu_stream(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if IMU_SPI_LOOPBACK_TEST
/**
  * @brief  Bus self-test with the sensor out of the picture: wire PB5 to PB4
  *         and every byte sent must come back identical. Passing puts the
  *         fault on the breakout or its wiring, failing on the STM32 side.
  * @retval None
  */
static void spi_loopback_test(void)
{
  /* Mixed and alternating bits to catch a stuck clock */
  static const uint8_t tx[] = { 0x6C, 0xA5, 0x5A, 0x00, 0xFF };
  uint8_t rx[sizeof(tx)] = { 0 };
  char line[TX_BUF_SIZE];

  /* CS stays high: nothing is listening, this only exercises the pins */
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)tx, rx,
                                                 sizeof(tx), 100);

  int n = snprintf(line, sizeof(line),
                   "loopback hal=%d sent %02X %02X %02X %02X %02X got %02X %02X %02X %02X %02X\r\n",
                   (int)st, tx[0], tx[1], tx[2], tx[3], tx[4],
                   rx[0], rx[1], rx[2], rx[3], rx[4]);
  HAL_UART_Transmit(&huart2, (uint8_t *)line, n, HAL_MAX_DELAY);

  const char *verdict = (memcmp(tx, rx, sizeof(tx)) == 0)
                        ? "loopback PASS: STM32 side is good, fault is the breakout or wiring\r\n"
                        : "loopback FAIL: fault is on the STM32 side, sensor is not involved\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)verdict, strlen(verdict), HAL_MAX_DELAY);
}
#endif

#if IMU_SPI_PIN_PROBE
/**
  * @brief  Read PB4 as an input under the internal pull-up and then the
  *         pull-down. An open wire follows the pull; one reaching the powered
  *         breakout is held by the board network. Separates "DO is not
  *         connected" from "DO is connected but never driven" -- both read
  *         back 0xFF at the SPI layer.
  * @retval Pin level with that pull applied
  */
static uint8_t probe_miso(uint32_t pull)
{
  GPIO_InitTypeDef g = {0};

  g.Pin   = GPIO_PIN_4;
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = pull;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  HAL_Delay(2);

  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}

static void spi_pin_probe(void)
{
  uint8_t pu = probe_miso(GPIO_PULLUP);
  uint8_t pd = probe_miso(GPIO_PULLDOWN);

  /* Hand PB4 back to SPI3 before anything tries a transfer */
  GPIO_InitTypeDef g = {0};

  g.Pin       = GPIO_PIN_4;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &g);

  const char *verdict;

  if (pu && !pd)
    verdict = "  -> DO wire is open, or the breakout has no power\r\n";
  else if (pu && pd)
    verdict = "  -> DO is connected and held high; sensor is not driving\r\n";
  else if (!pu && !pd)
    verdict = "  -> DO is connected and clamped low\r\n";
  else
    verdict = "  -> inverted, which should not happen; suspect a short\r\n";

  char line[TX_BUF_SIZE];
  int  n = snprintf(line, sizeof(line), "MISO probe: pullup=%u pulldown=%u\r\n",
                    (unsigned)pu, (unsigned)pd);
  HAL_UART_Transmit(&huart2, (uint8_t *)line, n, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t *)verdict, strlen(verdict), HAL_MAX_DELAY);

  /* Slow enough to catch on a multimeter: PB6 should swing rail to rail */
  const char msg[] = "Pulsing CS on PB6 for 10 s, measure it against GND\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);

  for (int i = 0; i < 10; i++) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
  }
}
#endif

#if IMU_SPI_SCOPE_LOOP
/**
  * @brief  Repeat the same WHO_AM_I read so the bus carries a stable, scope-
  *         triggerable waveform on the CS falling edge. Prints the byte once
  *         a second.
  * @retval Never returns
  */
static void spi_scope_loop(void)
{
  imu = lsm6dso32_spi_ctx(&imu_bus);

  const char msg[] = "Scope loop: WHO_AM_I every 5 ms, trigger on CS falling\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);

  uint32_t last_report = HAL_GetTick();
  uint8_t  whoami = 0;

  for (;;) {
    lsm6dso32_device_id_get(&imu, &whoami);

    if (HAL_GetTick() - last_report >= 1000) {
      last_report = HAL_GetTick();

      char line[TX_BUF_SIZE];
      int  n = snprintf(line, sizeof(line), "  WHO_AM_I = 0x%02X\r\n", whoami);
      HAL_UART_Transmit(&huart2, (uint8_t *)line, n, HAL_MAX_DELAY);
    }

    HAL_Delay(5);
  }
}
#endif

/**
  * @brief  Brings the LSM6DSO32 up on SPI3: confirms the sensor is responsive,
  *         restores a known register state, locks the bus to SPI, then sets
  *         the output ranges and data rates.
  * @retval None (traps in Error_Handler if the sensor does not respond)
  */
static void imu_init(void)
{
  uint8_t whoami;
  uint8_t rst;

  /* Constructor drives the chip select line idle-high */
  imu = lsm6dso32_spi_ctx(&imu_bus);

  /* Allow sensor turn-on time before the first transaction */
  HAL_Delay(IMU_BOOT_TIME_MS);

  /* Catch absent or wrong sensor ID:
       0x00        MISO stuck low: line not connected, or sensor not driving it
       0xFF        MISO floating high: CS never reaching the sensor, or no power
       0x6C        shifted (0xD8 / 0x36): a stray clock edge, check CPOL/CPHA
       other       wrong part, or the bus is picking up a different device */
  int32_t id_status = lsm6dso32_device_id_get(&imu, &whoami);

  if (id_status != 0 || whoami != LSM6DSO32_ID) {
    char msg[TX_BUF_SIZE];
    int  n = snprintf(msg, sizeof(msg),
                      "IMU WHO_AM_I: got 0x%02X, want 0x%02X (bus status %ld)\r\n",
                      whoami, LSM6DSO32_ID, (long)id_status);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, n, HAL_MAX_DELAY);
    Error_Handler();
  }

  /* Software reset, then wait for sensor to clear the flag */
  lsm6dso32_reset_set(&imu, PROPERTY_ENABLE);

  uint32_t reset_start = HAL_GetTick();

  do {
    if (lsm6dso32_reset_get(&imu, &rst) != 0 ||
        HAL_GetTick() - reset_start > IMU_RESET_TIMEOUT_MS) {
      const char err[] = "IMU software reset did not complete\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)err, sizeof(err) - 1, HAL_MAX_DELAY);
      Error_Handler();
    }
  } while (rst);

  /* Disable I2C and set the 4-wire mode for SPI */
  lsm6dso32_i2c_interface_set(&imu, LSM6DSO32_I2C_DISABLE);
  lsm6dso32_spi_mode_set(&imu, LSM6DSO32_SPI_4_WIRE);

  /* Output registers are not updated until both MSB and LSB have been read */
  lsm6dso32_block_data_update_set(&imu, PROPERTY_ENABLE);

  /* Measurement ranges */
  lsm6dso32_xl_full_scale_set(&imu, LSM6DSO32_16g);
  lsm6dso32_gy_full_scale_set(&imu, LSM6DSO32_2000dps);

  /* Output data rates */
  lsm6dso32_xl_data_rate_set(&imu, LSM6DSO32_XL_ODR_104Hz_HIGH_PERF);
  lsm6dso32_gy_data_rate_set(&imu, LSM6DSO32_GY_ODR_104Hz_HIGH_PERF);
}

/**
  * @brief  Polls the data-ready flags and sends one CSV sample per new
  *         reading. Returns immediately when nothing fresh is waiting, so
  *         the caller sets the pace, not this function.
  *
  *         Line format, one per sample:
  *           t_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps
  * @retval None
  */
static void imu_stream(void)
{
  uint8_t xl_ready = 0;
  uint8_t gy_ready = 0;
  int16_t raw_xl[3];
  int16_t raw_gy[3];
  char line[TX_BUF_SIZE];

  lsm6dso32_xl_flag_data_ready_get(&imu, &xl_ready);
  lsm6dso32_gy_flag_data_ready_get(&imu, &gy_ready);

  if (!xl_ready || !gy_ready)
    return;

  lsm6dso32_acceleration_raw_get(&imu, raw_xl);
  lsm6dso32_angular_rate_raw_get(&imu, raw_gy);

  /* Scale helpers must match the ranges set in IMU_Init. Values are kept in
     mg / mdps as integers so the line needs no float formatting */
  int len = snprintf(line, sizeof(line),
                     "%lu,%ld,%ld,%ld,%ld,%ld,%ld\n",
                     (unsigned long)HAL_GetTick(),
                     (long)lsm6dso32_from_fs16_to_mg(raw_xl[0]),
                     (long)lsm6dso32_from_fs16_to_mg(raw_xl[1]),
                     (long)lsm6dso32_from_fs16_to_mg(raw_xl[2]),
                     (long)lsm6dso32_from_fs2000_to_mdps(raw_gy[0]),
                     (long)lsm6dso32_from_fs2000_to_mdps(raw_gy[1]),
                     (long)lsm6dso32_from_fs2000_to_mdps(raw_gy[2]));

  HAL_UART_Transmit(&huart2, (uint8_t *)line, len, HAL_MAX_DELAY);

  /* Heartbeat on LD2, divided down to a visible rate.
     Blinking means samples are still going out; solid or dark means the stream stopped */
  static uint16_t led_div;

  if (++led_div >= LED_BLINK_DIV) {
    led_div = 0;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
#if IMU_SPI_SCOPE_LOOP
  spi_scope_loop();
#endif
#if IMU_SPI_PIN_PROBE
  spi_pin_probe();
#endif
#if IMU_SPI_LOOPBACK_TEST
  spi_loopback_test();
#endif
  imu_init();

  const char header[] = "t_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)header, sizeof(header) - 1, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  imu_stream();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
