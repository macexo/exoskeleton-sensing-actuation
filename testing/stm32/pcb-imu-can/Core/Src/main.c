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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_ACCEL_CAN_ID     0x065U   // Give each additional board its own pair of IDs
#define IMU_GYRO_CAN_ID      0x066U
#define CAN_TX_TIMEOUT_MS    3U       // An 8-byte frame at 1 Mbit/s takes ~0.13 ms
#define IMU_BOOT_TIME_MS     35U      // Datasheet turn-on time
#define IMU_RESET_TIMEOUT_MS 100U     // Ceiling on the software-reset flag poll
#define IMU_STALE_TIMEOUT_MS 100U     // LED1 goes solid if no sample arrives for this long
#define LED_BLINK_MS         500U     // LED1 toggle period while pairs are being ACKed
#define ERROR_BLINK_LOOPS    2000000U // Busy-wait count per LED toggle in Error_Handler (roughly 0.1 s at 180 MHz)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Must outlive the ctx: lsm6dso32_spi_ctx() keeps a pointer to it. */
static lsm6dso32_spi_bus_t imu_bus = { &hspi3, IMU_CS_GPIO_Port, IMU_CS_Pin };
static stmdev_ctx_t imu;

static uint16_t imu_sequence;
static uint32_t last_sample_tick;
static uint32_t last_blink_tick;

/* Inspect these in the debugger. CAN ESR includes LEC, TEC, REC and bus-off. */
volatile uint8_t  imu_whoami;
volatile uint32_t imu_samples;
volatile uint32_t imu_read_errors;
volatile uint32_t can_pairs_sent;
volatile uint32_t can_pairs_dropped;
volatile uint32_t can_last_esr;
volatile uint32_t can_last_tsr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void imu_init(void);
static void imu_stream(void);
static void imu_pack_frame(uint8_t *out, uint16_t sequence, const int16_t *axes);
static HAL_StatusTypeDef CAN_Send_Frame(uint32_t id, const uint8_t *data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Brings the LSM6DSO32 up on SPI3: confirms the sensor is responsive,
  *         restores a known register state, locks the bus to SPI, then sets
  *         the output ranges and data rates.
  * @retval None (traps in Error_Handler if any step fails)
  */
static void imu_init(void)
{
  uint8_t whoami = 0;
  uint8_t rst = 1;

  /* Constructor drives the chip select line idle-high */
  imu = lsm6dso32_spi_ctx(&imu_bus);

  /* Allow sensor turn-on time before the first transaction */
  HAL_Delay(IMU_BOOT_TIME_MS);

  /* Read imu_whoami in the debugger -- on a new board the value points at the
     fault:
       0x00        MISO stuck low: not connected, or sensor not driving it
       0xFF        MISO floating high: CS not reaching the sensor, or no power
       0xD8 / 0x36 0x6C shifted by a bit: stray clock edge, check CPOL/CPHA
       other       wrong part, or a short on the bus */
  int32_t id_status = lsm6dso32_device_id_get(&imu, &whoami);

  imu_whoami = whoami;

  if (id_status != 0 || whoami != LSM6DSO32_ID)
    Error_Handler();

  /* Software reset, then wait for sensor to clear the flag. Bounded so a bus
     dropout here traps instead of hanging silently */
  if (lsm6dso32_reset_set(&imu, PROPERTY_ENABLE) != 0)
    Error_Handler();

  uint32_t reset_start = HAL_GetTick();

  do {
    if (lsm6dso32_reset_get(&imu, &rst) != 0 ||
        HAL_GetTick() - reset_start >= IMU_RESET_TIMEOUT_MS)
      Error_Handler();
  } while (rst);

  /* Disable I2C and set the 4-wire mode for SPI. With block data update on,
     output registers are not updated until both MSB and LSB have been read.
     Scale factors in the README must match these ranges */
  if (lsm6dso32_i2c_interface_set(&imu, LSM6DSO32_I2C_DISABLE) != 0 ||
      lsm6dso32_spi_mode_set(&imu, LSM6DSO32_SPI_4_WIRE) != 0 ||
      lsm6dso32_auto_increment_set(&imu, PROPERTY_ENABLE) != 0 ||
      lsm6dso32_block_data_update_set(&imu, PROPERTY_ENABLE) != 0 ||
      lsm6dso32_xl_full_scale_set(&imu, LSM6DSO32_16g) != 0 ||
      lsm6dso32_gy_full_scale_set(&imu, LSM6DSO32_2000dps) != 0 ||
      lsm6dso32_xl_data_rate_set(&imu, LSM6DSO32_XL_ODR_104Hz_HIGH_PERF) != 0 ||
      lsm6dso32_gy_data_rate_set(&imu, LSM6DSO32_GY_ODR_104Hz_HIGH_PERF) != 0)
    Error_Handler();
}

/**
  * @brief  Polls the data-ready flags and sends one accel frame and one gyro
  *         frame per new reading. Returns immediately when nothing fresh is
  *         waiting, so the caller sets the pace, not this function.
  *
  *         LED1 toggles every LED_BLINK_MS while pairs are being ACKed. It is
  *         solid on after a failed send or read, or if samples stop arriving.
  * @retval None
  */
static void imu_stream(void)
{
  uint8_t xl_ready = 0;
  uint8_t gy_ready = 0;
  int16_t raw_xl[3];
  int16_t raw_gy[3];
  uint8_t xl_frame[8];
  uint8_t gy_frame[8];

  if (lsm6dso32_xl_flag_data_ready_get(&imu, &xl_ready) != 0 ||
      lsm6dso32_gy_flag_data_ready_get(&imu, &gy_ready) != 0) {
    imu_read_errors++;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    return;
  }

  /* Both run at the same ODR, so wait until the pair is ready and send one
     pair per sample */
  if (!xl_ready || !gy_ready) {
    if (HAL_GetTick() - last_sample_tick >= IMU_STALE_TIMEOUT_MS)
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    return;
  }

  /* Reading the output registers is what clears the data-ready flags and
     releases the BDU hold for the next sample. Failed reads are never sent */
  if (lsm6dso32_acceleration_raw_get(&imu, raw_xl) != 0 ||
      lsm6dso32_angular_rate_raw_get(&imu, raw_gy) != 0) {
    imu_read_errors++;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    return;
  }

  imu_samples++;
  last_sample_tick = HAL_GetTick();

  imu_pack_frame(xl_frame, imu_sequence, raw_xl);
  imu_pack_frame(gy_frame, imu_sequence, raw_gy);
  imu_sequence++; // Advance even on loss so the receiver can count missing pairs

  if (CAN_Send_Frame(IMU_ACCEL_CAN_ID, xl_frame) == HAL_OK &&
      CAN_Send_Frame(IMU_GYRO_CAN_ID, gy_frame) == HAL_OK) {
    can_pairs_sent++;

    if (HAL_GetTick() - last_blink_tick >= LED_BLINK_MS) {
      last_blink_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
  } else {
    can_pairs_dropped++;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  Packs one reading into a CAN payload, all fields little endian:
  *           bytes 0..1  uint16 sequence
  *           bytes 2..7  int16 X, Y, Z in raw sensor counts
  * @retval None
  */
static void imu_pack_frame(uint8_t *out, uint16_t sequence, const int16_t *axes)
{
  out[0] = (uint8_t)sequence;
  out[1] = (uint8_t)(sequence >> 8);

  for (int i = 0; i < 3; i++) {
    uint16_t axis = (uint16_t)axes[i];

    out[2 + (2 * i)] = (uint8_t)axis;
    out[3 + (2 * i)] = (uint8_t)(axis >> 8);
  }
}

/**
  * @brief  Sends one standard-ID, 8-byte frame and waits for the attempt to
  *         finish. CAN TX interrupts must stay off alongside this polling.
  * @retval HAL_OK only if another node ACKed the frame
  */
static HAL_StatusTypeDef CAN_Send_Frame(uint32_t id, const uint8_t *data)
{
  CAN_TxHeaderTypeDef TxHeader = {0};
  uint32_t TxMailbox = 0;

  TxHeader.StdId = id;
  TxHeader.IDE   = CAN_ID_STD;
  TxHeader.RTR   = CAN_RTR_DATA;
  TxHeader.DLC   = 8;

  // Queue CAN frame if there's a free mailbox
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 ||
      HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK)
    return HAL_ERROR;

  // Wait for the transmission attempt to finish
  uint32_t start = HAL_GetTick();

  while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
    if (HAL_GetTick() - start >= CAN_TX_TIMEOUT_MS) {
      HAL_CAN_AbortTxRequest(&hcan1, TxMailbox);
      can_last_esr = hcan1.Instance->ESR;
      can_last_tsr = hcan1.Instance->TSR;
      return HAL_TIMEOUT;
    }
  }

  // TXOK is only set if the frame went out and was ACKed by another node
  uint32_t txok_bit = (TxMailbox == CAN_TX_MAILBOX0) ? CAN_TSR_TXOK0 :
                      (TxMailbox == CAN_TX_MAILBOX1) ? CAN_TSR_TXOK1 : CAN_TSR_TXOK2;
  uint32_t rqcp_bit = (TxMailbox == CAN_TX_MAILBOX0) ? CAN_TSR_RQCP0 :
                      (TxMailbox == CAN_TX_MAILBOX1) ? CAN_TSR_RQCP1 : CAN_TSR_RQCP2;

  can_last_esr = hcan1.Instance->ESR;
  can_last_tsr = hcan1.Instance->TSR;

  // Writing RQCP clears this mailbox's flags, so can_last_tsr only ever shows the latest attempt
  hcan1.Instance->TSR = rqcp_bit;

  return (can_last_tsr & txok_bit) ? HAL_OK : HAL_ERROR;
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

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
    Error_Handler();

  /* Start the stale-sample clock here: the gyro's turn-on time alone could
     otherwise read as a stalled stream on the first pass */
  last_sample_tick = HAL_GetTick();
  last_blink_tick = last_sample_tick;

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
    // Fast blink so startup errors are distinguishable from a solid "not sending" LED.
    // Busy-wait because interrupts are off, so HAL_Delay() would hang.
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    for (volatile uint32_t i = 0; i < ERROR_BLINK_LOOPS; i++);
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
