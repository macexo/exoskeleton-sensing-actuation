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
#include "can.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dso32_reg.h"
#include "lsm6dso32_port.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_BOOT_TIME_MS     35  // Datasheet turn-on time
#define TX_BUF_SIZE          96  // Enough for one CSV line
#define LED_BLINK_DIV        52  // Samples per LED1 toggle (~1 Hz blink at 104 Hz ODR)
#define IMU_RESET_TIMEOUT_MS 100 // Ceiling on the software-reset flag poll
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Must outlive the ctx: lsm6dso32_spi_ctx() keeps a pointer to it. */
static lsm6dso32_spi_bus_t imu_bus = { &hspi3, IMU_CS_GPIO_Port, IMU_CS_Pin };
static stmdev_ctx_t imu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void swo_write(const char *buf, int len);
static void imu_init(void);
static void imu_stream(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Sends bytes out ITM stimulus port 0, which the debugger streams
  *         over SWO (PB3). View it in the SWV ITM Data Console with port 0
  *         enabled. With no debugger attached, or SWV not started, ITM is
  *         disabled and ITM_SendChar returns at once, so this never blocks.
  * @retval None
  */
static void swo_write(const char *buf, int len)
{
  for (int i = 0; i < len; i++)
    ITM_SendChar((uint32_t)buf[i]);
}

/**
  * @brief  Brings the LSM6DSO32 up on SPI3: confirms the sensor is responsive,
  *         restores a known register state, locks the bus to SPI, then sets
  *         the output ranges and data rates.
  * @retval None (traps in Error_Handler if the sensor does not respond)
  */
static void imu_init(void)
{
  uint8_t whoami = 0;
  uint8_t rst;
  char msg[TX_BUF_SIZE];
  int  n;

  /* Constructor drives the chip select line idle-high */
  imu = lsm6dso32_spi_ctx(&imu_bus);

  /* Allow sensor turn-on time before the first transaction */
  HAL_Delay(IMU_BOOT_TIME_MS);

  /* Report what actually came back either way -- on a new board the value
     points at the fault:
       0x00        MISO stuck low: not connected, or sensor not driving it
       0xFF        MISO floating high: CS not reaching the sensor, or no power
       0xD8 / 0x36 0x6C shifted by a bit: stray clock edge, check CPOL/CPHA
       other       wrong part, or a short on the bus */
  int32_t id_status = lsm6dso32_device_id_get(&imu, &whoami);

  n = snprintf(msg, sizeof(msg), "IMU WHO_AM_I: got 0x%02X, want 0x%02X (bus status %ld)\r\n",
               whoami, LSM6DSO32_ID, (long)id_status);
  swo_write(msg, n);

  if (id_status != 0 || whoami != LSM6DSO32_ID)
    Error_Handler();

  /* Software reset, then wait for sensor to clear the flag. Bounded so a bus
     dropout here traps instead of hanging silently */
  lsm6dso32_reset_set(&imu, PROPERTY_ENABLE);

  uint32_t reset_start = HAL_GetTick();

  do {
    if (lsm6dso32_reset_get(&imu, &rst) != 0 ||
        HAL_GetTick() - reset_start > IMU_RESET_TIMEOUT_MS) {
      const char err[] = "IMU software reset did not complete\r\n";
      swo_write(err, sizeof(err) - 1);
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
  *         reading over SWO. Returns immediately when nothing fresh is
  *         waiting, so the caller sets the pace, not this function.
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

  /* Both run at the same ODR, so wait until the pair is ready and emit one
     row per sample instead of a half-updated line */
  if (!xl_ready || !gy_ready)
    return;

  /* Reading the output registers is what clears the data-ready flags and
     releases the BDU hold for the next sample */
  lsm6dso32_acceleration_raw_get(&imu, raw_xl);
  lsm6dso32_angular_rate_raw_get(&imu, raw_gy);

  /* Scale helpers must match the ranges set in imu_init. Values are kept in
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

  swo_write(line, len);

  /* Heartbeat on LED1, divided down to ~1 Hz so it is visibly blinking.
     Blinking means samples are still going out; solid on means
     Error_Handler, off means the stream stopped */
  static uint16_t led_div;

  if (++led_div >= LED_BLINK_DIV) {
    led_div = 0;
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
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
  MX_CAN1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  imu_init();

  const char header[] = "t_ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps\n";
  swo_write(header, sizeof(header) - 1);
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
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // Keep LED on if an error occurs
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
