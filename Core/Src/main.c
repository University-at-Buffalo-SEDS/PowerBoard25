/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Drivers/ltc2990.h"
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PRINT_BUFFER_SIZE 100

#define LTC2990_Voltage_I2C_Address 0x98 >> 1
#define LTC2990_Current_I2C_Address 0x9C >> 1

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 700 * 4
};
/* Definitions for readVoltageTask */
osThreadId_t readVoltageTaskHandle;
const osThreadAttr_t readVoltageTask_attributes = {
  .name = "readVoltageTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 700 * 4
};
/* Definitions for blinkLEDTask */
osThreadId_t blinkLEDTaskHandle;
const osThreadAttr_t blinkLEDTask_attributes = {
  .name = "blinkLEDTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 700 * 4
};
/* Definitions for printVoltage */
osThreadId_t printVoltageHandle;
const osThreadAttr_t printVoltage_attributes = {
  .name = "printVoltage",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 700 * 4
};
/* Definitions for readCurrent */
osThreadId_t readCurrentHandle;
const osThreadAttr_t readCurrent_attributes = {
  .name = "readCurrent",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 700 * 4
};
/* Definitions for printCurrent */
osThreadId_t printCurrentHandle;
const osThreadAttr_t printCurrent_attributes = {
  .name = "printCurrent",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 700 * 4
};
/* Definitions for VoltageLTC2990Mutex */
osMutexId_t VoltageLTC2990MutexHandle;
const osMutexAttr_t VoltageLTC2990Mutex_attributes = {
  .name = "VoltageLTC2990Mutex"
};
/* Definitions for CurrentLTC2990Mutex */
osMutexId_t CurrentLTC2990MutexHandle;
const osMutexAttr_t CurrentLTC2990Mutex_attributes = {
  .name = "CurrentLTC2990Mutex"
};
/* USER CODE BEGIN PV */

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[64];

LTC2990_Handle_t LTC2990_Voltage_Handle;
LTC2990_Handle_t LTC2990_Current_Handle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
void StartDefaultTask(void *argument);
void startReadVoltageTask(void *argument);
void startBlinkLEDTask(void *argument);
void startPrintVoltage(void *argument);
void startReadCurrent(void *argument);
void startPrintCurrent(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CDC_Transmit_Print(const char * format, ...) {
	char buf[PRINT_BUFFER_SIZE];
	va_list args;
	va_start(args, format);
	int n = vsprintf(buf, format, args);
	va_end(args);
	CDC_Transmit_FS(buf, n);
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
  MX_FDCAN2_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  	  FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x321;
	sFilterConfig.FilterID2 = 0x7FF;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
	  CDC_Transmit_Print("Error while configuring filter for FDCAN2");
	}

	/* Configure global filter:
	   Filter all remote frames with STD and EXT ID
	   Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		CDC_Transmit_Print("Error while configuring global filter\n");
	}

  HAL_StatusTypeDef err = HAL_FDCAN_Start(&hfdcan2);
    if (err != HAL_OK) {
  	  char buf[60];// to send
  	  int n = sprintf(buf, "init err: = 0x%02x\n", err);
  	  CDC_Transmit_FS(buf, n);
    }

    /* TEMPORARY TILL I FIGURE OUR LED DRIVER */
    HAL_Delay(1000);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);




    /* END OF TEMP FOR LED DRIVER */




  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of VoltageLTC2990Mutex */
  VoltageLTC2990MutexHandle = osMutexNew(&VoltageLTC2990Mutex_attributes);

  /* creation of CurrentLTC2990Mutex */
  CurrentLTC2990MutexHandle = osMutexNew(&CurrentLTC2990Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of readVoltageTask */
  readVoltageTaskHandle = osThreadNew(startReadVoltageTask, NULL, &readVoltageTask_attributes);

  /* creation of blinkLEDTask */
  blinkLEDTaskHandle = osThreadNew(startBlinkLEDTask, NULL, &blinkLEDTask_attributes);

  /* creation of printVoltage */
  printVoltageHandle = osThreadNew(startPrintVoltage, NULL, &printVoltage_attributes);

  /* creation of readCurrent */
  readCurrentHandle = osThreadNew(startReadCurrent, NULL, &readCurrent_attributes);

  /* creation of printCurrent */
  printCurrentHandle = osThreadNew(startPrintCurrent, NULL, &printCurrent_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
<<<<<<< Updated upstream

	//QUICK TEST FOR LEDS

    LTC2990_Init(&LTC2990_Handle, &hi2c2);

=======
>>>>>>> Stashed changes
  while (1)
  {

//	  if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0) {
//	  		  CDC_Transmit_Print("There are some messages in the buffer!\n"); //Data to send
//	  		  //Recieve data
//	  		  HAL_StatusTypeDef err = HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData);
//	  		  if (err != HAL_OK)
//	  		  {
//	  			 // n = sprintf(printBuffer, );
//	  			  CDC_Transmit_Print("Error recieving message: 0x%02x\n", err);
//	  		  } else {
//	  			  //n = sprintf(printBuffer, "Recieved message: %s", RxData);
//	  			  //CDC_Transmit_FS(printBuffer, n);
//	  			  CDC_Transmit_Print("Recieved message: %s\n", RxData);
//	  		  }
//	  	  } else {
//	  		  CDC_Transmit_Print("NO MESSAGES IN FIFO0\n"); //Data to send
//	  		  CDC_Transmit_Print("Current FDCAN state: 0x%02x\n", hfdcan2.State);
//	  	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 16;
  hfdcan2.Init.NominalTimeSeg1 = 63;
  hfdcan2.Init.NominalTimeSeg2 = 16;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 4;
  hfdcan2.Init.DataTimeSeg1 = 13;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FRONT_LED_Pin|BACKLIGHT_LEDS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FRONT_LED_Pin BACKLIGHT_LEDS_Pin */
  GPIO_InitStruct.Pin = FRONT_LED_Pin|BACKLIGHT_LEDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB9);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startReadVoltageTask */
/**
* @brief Function implementing the readVoltageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startReadVoltageTask */
void startReadVoltageTask(void *argument)
{
  /* USER CODE BEGIN startReadVoltageTask */
<<<<<<< Updated upstream
  /* Infinite loop */
  for(;;)
  {
	  LTC2990_Step(&LTC2990_Handle);
	  osDelay(1);
=======
	LTC2990_Init(&LTC2990_Voltage_Handle, &hi2c2, LTC2990_Voltage_I2C_Address);
  /* Infinite loop */
  for(;;)
  {
	  if(osMutexAcquire(VoltageLTC2990MutexHandle, osWaitForever) == osOK) {
		  LTC2990_Step(&LTC2990_Voltage_Handle);
		  osMutexRelease(VoltageLTC2990MutexHandle);
	  }
>>>>>>> Stashed changes
  }
  /* USER CODE END startReadVoltageTask */
}

/* USER CODE BEGIN Header_startBlinkLEDTask */
/**
* @brief Function implementing the blinkLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBlinkLEDTask */
void startBlinkLEDTask(void *argument)
{
  /* USER CODE BEGIN startBlinkLEDTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_WritePin(GPIOB, FRONT_LED_Pin, GPIO_PIN_SET);
	osDelay(500);
	HAL_GPIO_WritePin(GPIOB, FRONT_LED_Pin, GPIO_PIN_RESET);
    osDelay(500);
  }
  /* USER CODE END startBlinkLEDTask */
}

/* USER CODE BEGIN Header_startPrintVoltage */
/**
* @brief Function implementing the printVoltage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPrintVoltage */
void startPrintVoltage(void *argument)
{
  /* USER CODE BEGIN startPrintVoltage */
  /* Infinite loop */
  for(;;)
  {
	  float voltages[4];
	  LTC2990_Get_Voltage(&LTC2990_Voltage_Handle, voltages);

	  for (int i = 0; i < 4; i++) {
		  CDC_Transmit_Print("Voltage %d: %d.%03d V\n", i ,  (int)voltages[i], (int)((voltages[i] - (int)voltages[i]) * 1000));;
	  }
	  osDelay(1);
  }
  /* USER CODE END startPrintVoltage */
}

/* USER CODE BEGIN Header_startReadCurrent */
/**
* @brief Function implementing the readCurrent thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startReadCurrent */
void startReadCurrent(void *argument)
{
  /* USER CODE BEGIN startReadCurrent */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END startReadCurrent */
}

/* USER CODE BEGIN Header_startPrintCurrent */
/**
* @brief Function implementing the printCurrent thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPrintCurrent */
void startPrintCurrent(void *argument)
{
  /* USER CODE BEGIN startPrintCurrent */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END startPrintCurrent */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
