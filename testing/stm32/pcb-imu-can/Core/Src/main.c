/* SPI IMU telemetry over classic CAN; based on pcb-test-can and pcb-test-imu. */
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "spi.h"
#include "lsm6dso32_port.h"

/* Two standard-ID frames per sample. Change both IDs for each additional board. */
#define IMU_ACCEL_CAN_ID 0x065U
#define IMU_GYRO_CAN_ID  0x066U
#define CAN_TX_TIMEOUT_MS 3U
#define IMU_RESET_TIMEOUT_MS 100U
#define IMU_STALE_TIMEOUT_MS 100U

static lsm6dso32_spi_bus_t imu_bus = { &hspi3, IMU_CS_GPIO_Port, IMU_CS_Pin };
static stmdev_ctx_t imu;

/* Inspect these in the debugger. CAN ESR includes LEC, TEC, REC and bus-off. */
volatile uint32_t imu_samples;
volatile uint32_t imu_read_errors;
volatile uint32_t can_pairs_sent;
volatile uint32_t can_pairs_dropped;
volatile uint32_t can_last_esr;
volatile uint32_t can_last_tsr;
volatile uint8_t imu_whoami;

void SystemClock_Config(void);

static void imu_init(void)
{
  uint8_t id = 0;
  uint8_t reset = 1;
  imu = lsm6dso32_spi_ctx(&imu_bus);
  HAL_Delay(35);
  if (lsm6dso32_device_id_get(&imu, &id) != 0) Error_Handler();
  imu_whoami = id;
  if (id != LSM6DSO32_ID) Error_Handler();
  if (lsm6dso32_reset_set(&imu, PROPERTY_ENABLE) != 0) Error_Handler();
  uint32_t start = HAL_GetTick();
  do {
    if (lsm6dso32_reset_get(&imu, &reset) != 0 ||
        HAL_GetTick() - start >= IMU_RESET_TIMEOUT_MS) Error_Handler();
  } while (reset);
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

static void pack_u16(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)value;
  out[1] = (uint8_t)(value >> 8);
}

static void pack_sample(uint8_t out[8], uint16_t sequence, const int16_t axes[3])
{
  pack_u16(out, sequence);
  for (unsigned i = 0; i < 3; ++i)
    pack_u16(out + 2 + i * 2, (uint16_t)axes[i]);
}

/* Poll completion as in pcb-test-can. Clear completion flags explicitly so
 * TXOK from an earlier attempt cannot be mistaken for this transmission.
 * No CAN TX interrupts may be enabled alongside this polling implementation. */
static int can_send(uint32_t id, const uint8_t data[8])
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;
  header.StdId = id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 ||
      HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) != HAL_OK)
    return 0;

  uint32_t start = HAL_GetTick();
  while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox)) {
    if (HAL_GetTick() - start >= CAN_TX_TIMEOUT_MS) {
      HAL_CAN_AbortTxRequest(&hcan1, mailbox);
      can_last_esr = hcan1.Instance->ESR;
      can_last_tsr = hcan1.Instance->TSR;
      return 0;
    }
  }
  uint32_t txok = (mailbox == CAN_TX_MAILBOX0) ? CAN_TSR_TXOK0 :
                 (mailbox == CAN_TX_MAILBOX1) ? CAN_TSR_TXOK1 : CAN_TSR_TXOK2;
  uint32_t complete = (mailbox == CAN_TX_MAILBOX0) ? CAN_TSR_RQCP0 :
                     (mailbox == CAN_TX_MAILBOX1) ? CAN_TSR_RQCP1 : CAN_TSR_RQCP2;
  can_last_tsr = hcan1.Instance->TSR;
  can_last_esr = hcan1.Instance->ESR;
  hcan1.Instance->TSR = complete;
  return (can_last_tsr & txok) != 0;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_SPI3_Init();
  imu_init();
  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();

  uint16_t sequence = 0;
  uint32_t last_sample = HAL_GetTick();
  uint32_t last_blink = last_sample;
  while (1) {
    uint8_t accel_ready = 0, gyro_ready = 0;
    if (lsm6dso32_xl_flag_data_ready_get(&imu, &accel_ready) != 0 ||
        lsm6dso32_gy_flag_data_ready_get(&imu, &gyro_ready) != 0) {
      ++imu_read_errors;
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      HAL_Delay(1);
      continue;
    }
    if (!accel_ready || !gyro_ready) {
      if (HAL_GetTick() - last_sample >= IMU_STALE_TIMEOUT_MS)
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      continue;
    }

    int16_t accel[3], gyro[3];
    if (lsm6dso32_acceleration_raw_get(&imu, accel) != 0 ||
        lsm6dso32_angular_rate_raw_get(&imu, gyro) != 0) {
      ++imu_read_errors;
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      continue;
    }
    ++imu_samples;
    last_sample = HAL_GetTick();
    uint8_t accel_payload[8], gyro_payload[8];
    pack_sample(accel_payload, sequence, accel);
    pack_sample(gyro_payload, sequence, gyro);
    ++sequence; /* Advance even on loss; receiver detects missing sample pairs. */

    if (can_send(IMU_ACCEL_CAN_ID, accel_payload) &&
        can_send(IMU_GYRO_CAN_ID, gyro_payload)) {
      ++can_pairs_sent;
      if (HAL_GetTick() - last_blink >= 500U) {
        last_blink = HAL_GetTick();
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      }
    } else {
      ++can_pairs_dropped;
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
  }
}

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



void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    for (volatile uint32_t i = 0; i < 2000000U; ++i) { }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  Error_Handler();
}
#endif
