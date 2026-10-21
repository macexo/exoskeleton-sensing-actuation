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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_TX_INTERVAL_MS 250
#define CAN_TX_TIMEOUT_MS  5     // An 8-byte frame at 500 kbit/s takes ~0.25 ms
#define ERROR_BLINK_LOOPS  2000000 // Busy-wait count per LED toggle in Error_Handler (roughly 0.1 s at 180 MHz)
#define CAN_LOOPBACK_TEST  0     // 1 = controller ACKs its own frames (no bus needed), 0 = normal mode
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CAN_Send_Frame(const uint8_t *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  uint32_t last_can_tx_tick = HAL_GetTick();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0};

#if CAN_LOOPBACK_TEST
  // Re-init in loopback mode here instead of in can.c so CubeMX regeneration doesn't undo it
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  if(HAL_CAN_Init(&hcan1) != HAL_OK) {
	  Error_Handler();
  }
#endif

  if(HAL_CAN_Start(&hcan1) != HAL_OK) {
	  Error_Handler();
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if((HAL_GetTick() - last_can_tx_tick) >= CAN_TX_INTERVAL_MS)
	{
	  last_can_tx_tick = HAL_GetTick();
	  CAN_Send_Frame(data);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
static void CAN_Send_Frame(const uint8_t *data)
{
	CAN_TxHeaderTypeDef TxHeader;
	memset(&TxHeader, 0, sizeof(TxHeader));

    TxHeader.StdId = 101;
    TxHeader.IDE   = CAN_ID_STD;
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = 8;

	uint32_t TxMailbox = 0;

	// Queue CAN frame if there's a free mailbox
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 &&
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) == HAL_OK) {

    	// Wait for the transmission attempt to finish
    	uint32_t start = HAL_GetTick();
    	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {
    		if ((HAL_GetTick() - start) >= CAN_TX_TIMEOUT_MS) {
    			HAL_CAN_AbortTxRequest(&hcan1, TxMailbox);
    			break;
    		}
    	}

    	// TXOK is only set if the frame went out and was ACKed by another node
    	uint32_t txok_bit = (TxMailbox == CAN_TX_MAILBOX0) ? CAN_TSR_TXOK0 :
    	                    (TxMailbox == CAN_TX_MAILBOX1) ? CAN_TSR_TXOK1 : CAN_TSR_TXOK2;
    	if (hcan1.Instance->TSR & txok_bit) {
    		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); // Blink if frame was ACKed
    		return;
    	}
    }
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // Keep LED on if frame was not sent/ACKed
}
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
	  // Fast blink so init errors are distinguishable from a solid "not sending" LED.
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
